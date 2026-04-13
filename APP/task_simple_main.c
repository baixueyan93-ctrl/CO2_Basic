#include "task_simple_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_inverter.h"
#include "bsp_exv.h"
#include "bsp_relay.h"
#include "bsp_rs485.h"
#include "task_panel.h"
#include <stdio.h>
#include <stdbool.h>

/* ===========================================================================
 * task_simple_main — 简化测试版主状态机
 *
 * 执行节拍: 1 秒一次 (用 vTaskDelayUntil 保证精准)
 * 主节拍:   每 10 秒执行一次温差换挡 + EXV PID 调整
 *
 * 详细状态说明:
 *
 *  SM_POWER_OFF
 *     仅显示柜温, 压缩机/风扇/EXV 全停.
 *     按开机键 → 若冷却剩余为 0 → 进 SELFTEST
 *                若冷却剩余 > 0 → 回退开机键 (忽略按键)
 *
 *  SM_SELFTEST (10s)
 *     风扇已开, ADC 持续采集.
 *     10 秒到, 发 'R' 启动压缩机 + 发 '0' 进 80Hz +
 *     EXV 开到初始开度 250 步 → 进 SM_RUN_LOW
 *     期间按关机键 → 中止, 回 POWER_OFF (带冷却)
 *
 *  SM_RUN_LOW (80Hz '0')
 *  SM_RUN_HIGH (160Hz '1')
 *     每 10 秒节拍: 温差换挡 + EXV PID.
 *     温差换挡带 ±0.5℃ 滞环:
 *       RUN_LOW  → ΔT ≥ 5.5℃ → RUN_HIGH (发 '1')
 *       RUN_HIGH → ΔT ≤ 4.5℃ → RUN_LOW  (发 '0')
 *     自动除霜计时: 从 SELFTEST 结束发 R 那一刻起, 累计 3h → 触发.
 *     按除霜键 → 手动进入除霜.
 *     按关机键 → 停机回 POWER_OFF (带冷却).
 *
 *  SM_DEFROST_RUN (10min)
 *     发 '2' 240Hz + EXV 全开 500 步 + 关 2 个风扇.
 *     10 分钟到 → 进 DEFROST_DRIP.
 *     按除霜键 → 取消除霜回 RUN_LOW.
 *     按关机键 → 立即中止, 回 POWER_OFF (带冷却).
 *
 *  SM_DEFROST_DRIP (5min)
 *     发 'S' 完全停机 (风扇继续关), 等待化霜水滴完.
 *     5 分钟到 → 重启压缩机 (发 R + 0 + EXV 250 + 开风扇) → RUN_LOW.
 *     按除霜键 → 取消回 RUN_LOW.
 *     按关机键 → 立即中止, 回 POWER_OFF (带冷却).
 * =========================================================================== */

/* ===========================================================================
 * 状态机枚举
 * =========================================================================== */
typedef enum {
    SM_POWER_OFF     = 0,
    SM_SELFTEST      = 1,
    SM_RUN_LOW       = 2,
    SM_RUN_HIGH      = 3,
    SM_DEFROST_RUN   = 4,
    SM_DEFROST_DRIP  = 5,
} SimpleState_t;

/* ===========================================================================
 * 内部状态变量
 * =========================================================================== */
static SimpleState_t s_state            = SM_POWER_OFF;
static uint32_t      s_state_sec        = 0;   /* 当前状态进入后的秒数        */
static uint32_t      s_main_tick_sec    = 0;   /* 主节拍计数 (10s 重置)       */
static uint32_t      s_defrost_tmr_sec  = 0;   /* 自动除霜累计计时 (3h 触发)  */
static uint32_t      s_cooldown_sec     = 0;   /* 关机后冷却剩余秒数          */
static uint8_t       s_prev_system_on   = 0;   /* 上次读到的 g_system_on      */

/* 对外: 除霜请求标志 (panel 按键置 1, simple main 消费后清零) */
volatile uint8_t g_defrost_req = 0;

/* ===========================================================================
 * 辅助函数
 * =========================================================================== */

/* 移动 EXV 到目标位置, 运动结束后断电 (线圈省电+防发热) */
static void SM_EXV_MoveTo(uint16_t target_steps)
{
    BSP_EXV_SetPosition(target_steps, EXV_STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
    BSP_EXV_DeEnergize();

    SysState_Lock();
    SysState_GetRawPtr()->VAR_EXV_OPENING = (float)target_steps;
    SysState_Unlock();
}

/* 开 2 个风扇 */
static void SM_FansOn(void)
{
    BSP_Relay_Set(RELAY_EVAP_FAN, 1);
    BSP_Relay_Set(RELAY_COND_FAN, 1);
    xEventGroupSetBits(SysEventGroup, ST_EVAP_FAN_ON | ST_COND_FAN1_ON);
}

/* 关 2 个风扇 */
static void SM_FansOff(void)
{
    BSP_Relay_Set(RELAY_EVAP_FAN, 0);
    BSP_Relay_Set(RELAY_COND_FAN, 0);
    xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON | ST_COND_FAN1_ON);
}

/* 发送挡位指令并同步状态 */
static void SM_SetGear(uint8_t gear, const char *reason)
{
    static const uint16_t hz[4] = {80, 160, 240, 320};

    if (gear > 3) gear = 3;

    BSP_Inverter_SendGear(gear);

    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = (float)hz[gear];
    SysState_Unlock();

    {
        char msg[80];
        snprintf(msg, sizeof(msg), "[SM] Gear -> '%d' (%uHz) %s\r\n",
                 (int)gear, (unsigned)hz[gear], reason ? reason : "");
        BSP_RS485_SendString(msg);
    }
}

/* 进入 POWER_OFF 状态
 *   with_cooldown = 1 → 启动 3 分钟冷却
 *   with_cooldown = 0 → 立刻可重开 (用于首次上电时从未开过机的情况)
 */
static void SM_GotoPowerOff(uint8_t with_cooldown)
{
    /* 1. 停压缩机 */
    BSP_Inverter_SendStop();

    /* 2. 关风扇 */
    SM_FansOff();

    /* 3. 清状态位 */
    xEventGroupClearBits(SysEventGroup,
        ST_COMP_RUNNING | ST_DEFROST_ACTIVE | ST_DEF_HEATING | ST_DEF_DRIPPING |
        ST_WARMUP_DONE  | ST_SYSTEM_ON);

    /* 4. 状态跳转 */
    s_state         = SM_POWER_OFF;
    s_state_sec     = 0;
    s_main_tick_sec = 0;
    s_cooldown_sec  = with_cooldown ? SIMPLE_COOLDOWN_SEC : 0;

    BSP_RS485_SendString(with_cooldown ?
        "[SM] -> POWER_OFF (cooldown 180s)\r\n" :
        "[SM] -> POWER_OFF\r\n");
}

/* 从 POWER_OFF 进入 SELFTEST
 * (调用前已确认 cooldown_sec == 0 且 g_system_on == true)
 */
static void SM_GotoSelftest(void)
{
    /* 先开 2 个风扇, 让 ADC 有稳定工况 */
    SM_FansOn();

    xEventGroupSetBits(SysEventGroup, ST_SYSTEM_ON);
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);

    s_state     = SM_SELFTEST;
    s_state_sec = 0;

    BSP_RS485_SendString("[SM] -> SELFTEST (10s warming up inverter)\r\n");
}

/* SELFTEST 结束, 启动压缩机进 RUN_LOW */
static void SM_StartCompressor(void)
{
    /* 1. 发 R 启动 */
    BSP_Inverter_SendRun();
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2. 发 '0' 进 80Hz 低档 */
    SM_SetGear(SIMPLE_GEAR_RUN_LOW, "first start");

    /* 3. EXV 到初始开度 250 步 */
    SM_EXV_MoveTo((uint16_t)SET_EXV_INIT_OPENING);

    /* 4. 状态跳转 */
    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
    s_state           = SM_RUN_LOW;
    s_state_sec       = 0;
    s_main_tick_sec   = 0;
    s_defrost_tmr_sec = 0;   /* 3h 自动除霜计时从这里开始 */

    BSP_RS485_SendString("[SM] -> RUN_LOW (80Hz, EXV=250)\r\n");
}

/* 进入除霜 RUN (从 RUN_LOW 或 RUN_HIGH) */
static void SM_EnterDefrost(void)
{
    /* 1. 发 '2' 240Hz */
    SM_SetGear(SIMPLE_GEAR_DEFROST, "defrost start");

    /* 2. EXV 全开 500 步 */
    SM_EXV_MoveTo((uint16_t)EXV_TOTAL_STEPS);

    /* 3. 关 2 个风扇 */
    SM_FansOff();

    xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE | ST_DEF_HEATING);
    xEventGroupClearBits(SysEventGroup, ST_DEF_DRIPPING);

    s_state     = SM_DEFROST_RUN;
    s_state_sec = 0;

    BSP_RS485_SendString("[SM] -> DEFROST_RUN (240Hz, EXV full open, fans off)\r\n");
}

/* 除霜结束, 进入滴水 */
static void SM_EnterDrip(void)
{
    BSP_Inverter_SendStop();

    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING | ST_DEF_HEATING);
    xEventGroupSetBits(SysEventGroup, ST_DEF_DRIPPING);

    s_state     = SM_DEFROST_DRIP;
    s_state_sec = 0;

    BSP_RS485_SendString("[SM] -> DEFROST_DRIP (5min, comp stop, fans off)\r\n");
}

/* 滴水结束, 重启回 RUN_LOW */
static void SM_ResumeFromDrip(void)
{
    xEventGroupClearBits(SysEventGroup, ST_DEFROST_ACTIVE | ST_DEF_DRIPPING);

    /* 重新开风扇 */
    SM_FansOn();

    /* 发 R + 0 */
    BSP_Inverter_SendRun();
    vTaskDelay(pdMS_TO_TICKS(100));
    SM_SetGear(SIMPLE_GEAR_RUN_LOW, "resume from drip");

    /* EXV 回到 250 步 */
    SM_EXV_MoveTo((uint16_t)SET_EXV_INIT_OPENING);

    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
    s_state           = SM_RUN_LOW;
    s_state_sec       = 0;
    s_main_tick_sec   = 0;
    s_defrost_tmr_sec = 0;    /* 除霜结束, 3h 重新计时 */

    BSP_RS485_SendString("[SM] -> RUN_LOW (resumed after drip)\r\n");
}

/* 除霜/滴水 中途取消, 直接回 RUN_LOW */
static void SM_CancelDefrost(const char *reason)
{
    xEventGroupClearBits(SysEventGroup,
        ST_DEFROST_ACTIVE | ST_DEF_HEATING | ST_DEF_DRIPPING);

    SM_FansOn();

    /* 如果之前是 DEFROST_DRIP (压缩机已停), 需要重新发 R */
    if (s_state == SM_DEFROST_DRIP) {
        BSP_Inverter_SendRun();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    SM_SetGear(SIMPLE_GEAR_RUN_LOW, reason);
    SM_EXV_MoveTo((uint16_t)SET_EXV_INIT_OPENING);

    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
    s_state           = SM_RUN_LOW;
    s_state_sec       = 0;
    s_main_tick_sec   = 0;
    s_defrost_tmr_sec = 0;

    BSP_RS485_SendString("[SM] Defrost canceled -> RUN_LOW\r\n");
}

/* 主节拍 (10s 一次): 温差换挡 + EXV PID 调整 */
static void SM_MainTick(void)
{
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    float delta_t = sensor.VAR_CABINET_TEMP - g_set_temp;
    float super_h = sensor.VAR_SUPERHEAT;

    SysState_Lock();
    SysState_GetRawPtr()->VAR_DELTA_T = delta_t;
    SysState_Unlock();

    /* --- 温差换挡 (带 ±0.5℃ 滞环) --- */
    if (s_state == SM_RUN_LOW) {
        if (delta_t >= SIMPLE_DT_UP_TH) {
            SM_SetGear(SIMPLE_GEAR_RUN_HIGH, "dT>=5.5");
            s_state = SM_RUN_HIGH;
            /* 注意: 不重置 s_state_sec, 继续走 10s 节拍 */
        }
    } else if (s_state == SM_RUN_HIGH) {
        if (delta_t <= SIMPLE_DT_DOWN_TH) {
            SM_SetGear(SIMPLE_GEAR_RUN_LOW, "dT<=4.5");
            s_state = SM_RUN_LOW;
        }
    }

    /* --- EXV PID 调整 (纯 P, 带 ±0.5℃ 死区) --- */
    float sh_err = super_h - SIMPLE_SH_TARGET;
    float cur_kp = sensor.VAR_EXV_OPENING;

    char msg[112];
    if (sh_err > SIMPLE_SH_DEADBAND || sh_err < -SIMPLE_SH_DEADBAND) {
        /* 偏离死区, 需要调整
         *   sh_err > 0 → 过热度过高 → 阀门开大 → Kp += |err|*Kp系数
         *   sh_err < 0 → 过热度过低 → 阀门关小 → Kp -= |err|*Kp系数
         *   公式: new_kp = cur_kp + SIMPLE_EXV_KP * sh_err
         */
        float new_kp = cur_kp + SIMPLE_EXV_KP * sh_err;
        if (new_kp < 0.0f)                   new_kp = 0.0f;
        if (new_kp > (float)EXV_TOTAL_STEPS) new_kp = (float)EXV_TOTAL_STEPS;

        SM_EXV_MoveTo((uint16_t)(new_kp + 0.5f));

        snprintf(msg, sizeof(msg),
                 "[SM] Tick dT=%.1f SH=%.1f(err=%+.1f) EXV:%.0f->%.0f\r\n",
                 delta_t, super_h, sh_err, cur_kp, new_kp);
    } else {
        snprintf(msg, sizeof(msg),
                 "[SM] Tick dT=%.1f SH=%.1f(err=%+.1f) EXV=%.0f (deadband)\r\n",
                 delta_t, super_h, sh_err, cur_kp);
    }
    BSP_RS485_SendString(msg);
}

/* ===========================================================================
 * 任务主循环 (1 秒精准节拍)
 * =========================================================================== */
void Task_SimpleMain_Process(void const *argument)
{
    (void)argument;

    /* 等 ADC/Panel/Relay 等任务启动就绪 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 560 步无记忆冷启动关零 (必须在调度器启动后执行, vTaskDelay 才生效) */
    BSP_RS485_SendString("[SM] EXV cold reset 560 steps...\r\n");
    BSP_EXV_ResetToZero();
    BSP_RS485_SendString("[SM] EXV reset done, de-energized\r\n");

    /* 初始状态 */
    s_state          = SM_POWER_OFF;
    s_state_sec      = 0;
    s_main_tick_sec  = 0;
    s_defrost_tmr_sec = 0;
    s_cooldown_sec   = 0;   /* 首次上电无冷却, 可立即开机 */
    s_prev_system_on = g_system_on ? 1 : 0;

    BSP_RS485_SendString(
        "[SM] BOOT -> POWER_OFF (ready, press POWER to start)\r\n");

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* 1 秒基本节拍 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));

        /* --- 按键边沿检测 --- */
        uint8_t sys_on   = g_system_on ? 1 : 0;
        uint8_t edge_on  = ( sys_on && !s_prev_system_on);
        uint8_t edge_off = (!sys_on &&  s_prev_system_on);
        s_prev_system_on = sys_on;

        switch (s_state) {

        /* =========================================================
         * SM_POWER_OFF: 仅显示柜温, 等待开机键
         * ========================================================= */
        case SM_POWER_OFF:
            if (s_cooldown_sec > 0) {
                s_cooldown_sec--;
            }

            if (edge_on) {
                if (s_cooldown_sec == 0) {
                    /* 可以开机 */
                    SM_GotoSelftest();
                } else {
                    /* 冷却未结束, 忽略本次开机请求 */
                    g_system_on = false;
                    xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
                    s_prev_system_on = 0;
                    {
                        char msg[80];
                        snprintf(msg, sizeof(msg),
                                 "[SM] Cooldown %lus remain, POWER ignored\r\n",
                                 (unsigned long)s_cooldown_sec);
                        BSP_RS485_SendString(msg);
                    }
                }
            }

            /* POWER_OFF 状态下清除任何遗留的除霜请求 */
            g_defrost_req = 0;
            break;

        /* =========================================================
         * SM_SELFTEST (10s): 变频板自检, 风扇已开
         * ========================================================= */
        case SM_SELFTEST:
            s_state_sec++;

            if (edge_off) {
                /* 自检中关机 → 中止, 回 POWER_OFF (带冷却) */
                SM_GotoPowerOff(1);
                break;
            }

            if (s_state_sec >= SIMPLE_SELFTEST_SEC) {
                SM_StartCompressor();
            }
            break;

        /* =========================================================
         * SM_RUN_LOW / SM_RUN_HIGH: 正常运行
         * ========================================================= */
        case SM_RUN_LOW:
        case SM_RUN_HIGH:
            s_state_sec++;
            s_main_tick_sec++;
            s_defrost_tmr_sec++;

            if (edge_off) {
                SM_GotoPowerOff(1);
                break;
            }

            /* 手动或自动除霜 */
            if (g_defrost_req || s_defrost_tmr_sec >= SIMPLE_DEF_AUTO_SEC) {
                g_defrost_req = 0;
                SM_EnterDefrost();
                break;
            }

            /* 10s 主节拍 */
            if (s_main_tick_sec >= SIMPLE_MAIN_TICK_SEC) {
                s_main_tick_sec = 0;
                SM_MainTick();
            }
            break;

        /* =========================================================
         * SM_DEFROST_RUN (10min)
         * ========================================================= */
        case SM_DEFROST_RUN:
            s_state_sec++;

            if (edge_off) {
                /* 除霜中关机 → 立即中止, 3h 计时清零 */
                s_defrost_tmr_sec = 0;
                SM_GotoPowerOff(1);
                break;
            }

            /* 除霜中按除霜键 → 取消 */
            if (g_defrost_req) {
                g_defrost_req = 0;
                SM_CancelDefrost("cancel by key");
                break;
            }

            if (s_state_sec >= SIMPLE_DEF_RUN_SEC) {
                SM_EnterDrip();
            }
            break;

        /* =========================================================
         * SM_DEFROST_DRIP (5min)
         * ========================================================= */
        case SM_DEFROST_DRIP:
            s_state_sec++;

            if (edge_off) {
                s_defrost_tmr_sec = 0;
                SM_GotoPowerOff(1);
                break;
            }

            if (g_defrost_req) {
                g_defrost_req = 0;
                SM_CancelDefrost("cancel by key");
                break;
            }

            if (s_state_sec >= SIMPLE_DEF_DRIP_SEC) {
                SM_ResumeFromDrip();
            }
            break;

        default:
            /* 未知状态兜底 */
            s_state = SM_POWER_OFF;
            break;
        }
    }
}
