#include "task_defrost.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_relay.h"
#include "bsp_exv.h"
#include "bsp_inverter.h"
#include <stdbool.h>

/* ===========================================================================
 * 除霜流程 (逻辑图2) — 实现文件
 *
 * 子逻辑:
 *   1. Defrost_MainProcess()       除霜主程序
 *   2. Defrost_HeatSubroutine()    加热子程序
 *
 * 除霜原理:
 *   蒸发器长时间制冷后表面会结霜, 霜层阻碍换热导致制冷效率下降.
 *   除霜过程: 停止制冷 → 加热蒸发器融霜 → 滴水排出融水 → 恢复制冷
 *
 * 三阶段状态机:
 *   [待机阶段] 正常制冷, 等待除霜间隔到时
 *       ↓ 间隔到时(3小时)
 *   [加热阶段] 停机→等待→阀门→等待→启动压缩机→升频到320Hz→加热
 *       ↓ 压缩机达320Hz后加热5分钟 或 蒸发器温度≥10℃
 *   [滴水阶段] 关闭加热器, 等待融水滴干
 *       ↓ 滴水超时(5分钟)
 *   [待机阶段] 除霜完成, 复位间隔计时, 恢复制冷
 * =========================================================================== */


/* ===================================================================
 *  内部辅助函数 / 硬件操作桩
 * =================================================================== */

/* --- 除霜加热器控制 (通过 K7 继电器, PC7) --- */
static void DefrostHeater_On(void)
{
    BSP_Relay_On(RELAY_DEF_HEATER);
}

static void DefrostHeater_Off(void)
{
    BSP_Relay_Off(RELAY_DEF_HEATER);
}

/* --- 压缩机停止 (除霜时需要停止制冷) ---
 * 注意: 与温度控制任务共用压缩机, 通过状态标志协调
 */
static void Defrost_StopCompressor(void)
{
    /* BSP_Inverter_Send(0x00, 0); — 暂停, 变频板由ASCII手动控制 */
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
}

/* --- 除霜时启动压缩机 (热气除霜) ---
 * 与正常制冷不同: 热气除霜用压缩机排出的高温气体加热蒸发器
 */
static void Defrost_StartCompressor(float freq_hz)
{
    if (freq_hz > SET_FREQ_MAX) freq_hz = SET_FREQ_MAX;
    if (freq_hz < SET_FREQ_MIN) freq_hz = SET_FREQ_MIN;

    /* BSP_Inverter_Send(0x01, (uint16_t)freq_hz); — 暂停, 变频板由ASCII手动控制 */
    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = freq_hz;
    SysState_Unlock();
}

/* --- 膨胀阀步进到除霜位置 --- */
#define DEF_EXV_POSITION   500   /* 除霜时全开 (让高温制冷剂通过蒸发器) */

static void Defrost_ValveStep(void)
{
    BSP_EXV_SetPosition(DEF_EXV_POSITION, EXV_STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));  /* 结束励磁保持 */
    BSP_EXV_DeEnergize();

    /* 同步EXV开度到系统状态, 避免除霜结束后PID读到旧值 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_EXV_OPENING = (float)DEF_EXV_POSITION;
    SysState_Unlock();
}

/* ===================================================================
 *  加热子程序状态机定义
 *
 *  流程图中加热子程序包含延时和多步操作,
 *  不能阻塞FreeRTOS任务, 所以用状态机实现,
 *  每秒被主循环调用一次, 逐步推进
 * =================================================================== */
typedef enum {
    HEAT_IDLE,           /* 空闲, 等待启动                           */
    HEAT_WAIT_DELAY1,    /* 等待第一个延时X (SET_DEF_STEP_DLY = 30s) */
    HEAT_VALVE_STEP,     /* 执行膨胀阀步进动作                       */
    HEAT_WAIT_DELAY2,    /* 等待第二个延时X (30s)                    */
    HEAT_COMP_DELAY,     /* 等待15s后开压缩机 (SET_DEF_COMP_DLY)    */
    HEAT_COMP_RAMP,      /* 压缩机由慢到快步进升频                   */
    HEAT_RUNNING         /* 加热运行中, 测温度                       */
} DefrostHeatState_t;

static DefrostHeatState_t s_heat_state = HEAT_IDLE;
static uint32_t s_heat_delay_cnt = 0;     /* 加热子程序内部延时计数器 */
static float    s_comp_ramp_freq = 0.0f;  /* 压缩机升频当前值       */
static bool     s_was_defrosting = false; /* 上一周期是否在除霜 (检测手动取消) */

/* --- 加热子程序状态机复位 --- */
static void Defrost_HeatReset(void)
{
    s_heat_state = HEAT_IDLE;
    s_heat_delay_cnt = 0;
    s_comp_ramp_freq = 0.0f;
}

/* --- 加热→滴水 阶段转换 ---
 *
 *  流程图中加热结束后的公共路径:
 *    反标记正在加热(仅做反标记)
 *    → 标记正在滴水
 *    → 开启滴水计时
 *    → 初始化加热时长定时器
 *
 *  "仅做反标记"的含义:
 *    只清除"正在加热"标志, 不清除"正在除霜"总标志,
 *    因为还要进入滴水阶段, 除霜尚未完全结束
 */
static void Defrost_TransitionToDrip(void)
{
    /* 反标记正在加热(仅做反标记) — 关加热器, 清加热标志 */
    DefrostHeater_Off();
    xEventGroupClearBits(SysEventGroup, ST_DEF_HEATING);

    /* 停止除霜用的压缩机 (滴水阶段不需要压缩机运行) */
    Defrost_StopCompressor();

    /* 复位加热子程序状态机 */
    Defrost_HeatReset();

    /* 标记正在滴水 */
    xEventGroupSetBits(SysEventGroup, ST_DEF_DRIPPING);

    /* 开启滴水计时 (从0开始, 计时5分钟) */
    g_TimerData.TMR_DRIP_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DRIP_DONE);

    /* 初始化加热时长定时器 (复位, 供下次除霜使用) */
    g_TimerData.TMR_DEF_DUR_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);
}


/* ===================================================================
 *  子逻辑1: 除霜主程序
 *
 *  流程图 (2.除霜流程.pdf 左侧):
 *
 *    开始
 *    → 读取设置值、测量值和定时标记
 *
 *    → 正在除霜?
 *
 *      ┌─ Y(正在除霜) ──────────────────────────────────┐
 *      │                                                 │
 *      │  → 正在滴水(加热不停)?                           │
 *      │                                                 │
 *      │    Y(滴水阶段):                                  │
 *      │      → 滴水时间超时?                             │
 *      │          Y → 反标记正在滴水                       │
 *      │              反标记正在除霜                       │
 *      │              初始化滴水计时、除霜间隔计时          │
 *      │              → 结束 (除霜完全结束, 恢复制冷)      │
 *      │          N → 结束 (继续滴水等待)                  │
 *      │                                                 │
 *      │    N(加热阶段):                                  │
 *      │      → 加热时间超时?                             │
 *      │          Y → [转入滴水]                          │
 *      │          N → 加热温度超值?                        │
 *      │                Y → [转入滴水]                    │
 *      │                N → 结束 (继续加热)               │
 *      │                                                 │
 *      │  [转入滴水]:                                     │
 *      │    反标记正在加热(仅做反标记)                     │
 *      │    → 标记正在滴水                                │
 *      │    → 开启滴水计时                                │
 *      │    → 初始化加热时长定时器                         │
 *      │    → 结束                                        │
 *      │                                                 │
 *      ├─ N(未在除霜) ──────────────────────────────────┐│
 *      │                                                ││
 *      │  → 除霜间隔满时?                                ││
 *      │      Y → 标记正在除霜                           ││
 *      │          标记正在加热                            ││
 *      │          开启加热计时                            ││
 *      │          调用加热子程序                          ││
 *      │          → 结束                                 ││
 *      │      N → (温差条件 + 最短间隔2-3小时, 待扩展)    ││
 *      │          → 结束                                 ││
 *      └─────────────────────────────────────────────────┘│
 *                                                         │
 *    结束                                                  │
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void Defrost_MainProcess(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    /* ---- 读取设置值、测量值和定时标记 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    /* ================================================================
     *  手动取消检测: 上周期在除霜, 本周期 ST_DEFROST_ACTIVE 被外部清除
     *  面板按键再按一次除霜键时清除所有除霜标志, 这里负责安全收尾
     * ================================================================ */
    if (s_was_defrosting && !(sys_bits & ST_DEFROST_ACTIVE)) {
        /* 关闭加热器 + 停压缩机 + 复位状态机 */
        DefrostHeater_Off();
        Defrost_StopCompressor();
        Defrost_HeatReset();

        /* 恢复风机 (除霜开始时被关闭了) */
        BSP_Relay_On(RELAY_EVAP_FAN);
        xEventGroupSetBits(SysEventGroup, ST_EVAP_FAN_ON);
        BSP_Relay_On(RELAY_COND_FAN);
        xEventGroupSetBits(SysEventGroup, ST_COND_FAN1_ON);

        /* 复位计时器, 重新开始除霜间隔计时 */
        g_TimerData.TMR_DEF_INTV_CNT = 0;
        xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);
        g_TimerData.TMR_DEF_DUR_CNT = 0;
        g_TimerData.TMR_DRIP_CNT = 0;

        s_was_defrosting = false;
        return;
    }
    s_was_defrosting = !!(sys_bits & ST_DEFROST_ACTIVE);

    /* ================================================================
     *  正在除霜?
     * ================================================================ */
    if (sys_bits & ST_DEFROST_ACTIVE) {

        /* ============================================================
         *  Y → 正在除霜中, 判断当前处于哪个阶段
         * ============================================================ */

        /* ---- 手动触发检测 ----
         * 如果 ST_DEFROST_ACTIVE 已置位, 但既不在加热也不在滴水,
         * 说明是由面板手动除霜键触发 (只置位了总标志).
         * 此时需要执行完整的除霜启动序列.
         */
        if (!(sys_bits & ST_DEF_HEATING) && !(sys_bits & ST_DEF_DRIPPING)) {
            /* 手动触发 → 执行完整除霜启动序列 */
            Defrost_StopCompressor();

            xEventGroupSetBits(SysEventGroup, ST_DEF_HEATING);

            /* 加热计时先清零, 等压缩机升到320Hz后再正式开始计时 */
            g_TimerData.TMR_DEF_DUR_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);

            /* 关闭蒸发风机和冷凝风机 */
            BSP_Relay_Off(RELAY_EVAP_FAN);
            xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON);
            BSP_Relay_Off(RELAY_COND_FAN);
            xEventGroupClearBits(SysEventGroup, ST_COND_FAN1_ON);

            DefrostHeater_On();
            Defrost_HeatReset();
            Defrost_HeatSubroutine();
            return;
        }

        /* ============================================================
         *  正在滴水(加热不停)?
         *    流程图中"加热不停"的含义:
         *    加热阶段结束后, 加热器已关闭, 但除霜尚未完成,
         *    进入滴水阶段等待融水滴干
         * ============================================================ */

        if (sys_bits & ST_DEF_DRIPPING) {
            /* ========================================================
             *  Y → 当前处于[滴水阶段]
             *
             *  滴水时间超时?
             *    TMR_DRIP_CNT 由定时中断每秒递增,
             *    到达 SET_DRIP_TIME(5分钟) 时置位 ST_TMR_DEF_DRIP_DONE
             * ======================================================== */

            if (tmr_bits & ST_TMR_DEF_DRIP_DONE) {
                /* Y → 滴水完成, 除霜全部结束
                 *
                 * 执行顺序 (严格按流程图):
                 *   1. 反标记正在滴水
                 *   2. 反标记正在除霜
                 *   3. 初始化滴水计时、除霜间隔计时
                 */

                /* 1. 反标记正在滴水 */
                xEventGroupClearBits(SysEventGroup, ST_DEF_DRIPPING);

                /* 2. 反标记正在除霜 — 除霜全部完成 */
                xEventGroupClearBits(SysEventGroup, ST_DEFROST_ACTIVE);

                /* 3. 初始化滴水计时 */
                g_TimerData.TMR_DRIP_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DRIP_DONE);

                /* 3. 初始化除霜间隔计时 (从0开始, 等待下一次除霜) */
                g_TimerData.TMR_DEF_INTV_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);

                /* 复位加热子程序状态机 (安全保障) */
                Defrost_HeatReset();

                /* 除霜结束, 恢复蒸发风机和冷凝风机到除霜前状态
                 * 蒸发风机: 启动F3延时计时器, 延时30秒后由风机任务自动开启
                 * 冷凝风机: 清除除霜标志后, 冷凝风机任务检测到压缩机运行会自动开启
                 * 压缩机: 温控主逻辑检测到 ST_DEFROST_ACTIVE 已清除后自动恢复
                 */
                g_TimerData.TMR_EVAP_FAN_DLY_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_EVAP_FAN_DLY);
            }
            /* N → 结束: 滴水时间未到, 继续等待 */

        } else {
            /* ========================================================
             *  N → 当前处于[加热阶段]
             *
             *  两个退出条件 (任一满足即转入滴水):
             *    1. 加热时间超时 (SET_DEF_HEAT_MAX = 5分钟)
             *    2. 加热温度超值 (蒸发器温度 ≥ SET_DEF_HEAT_TLIMIT = 10℃)
             * ======================================================== */

            /* 每周期驱动加热子程序状态机 (膨胀阀步进 + 压缩机升频) */
            Defrost_HeatSubroutine();

            /* ---- 加热时间超时? (从压缩机达到320Hz后开始计时) ---- */
            if (tmr_bits & ST_TMR_DEF_DUR_DONE) {
                /* Y → 加热时间到(5分钟), 无论温度是否达标都转入滴水
                 *     (安全保护: 防止加热器长时间工作)
                 */
                Defrost_TransitionToDrip();
                return;
            }

            /* ---- N → 加热温度超值? ---- */
            if (sensor.VAR_EVAP_TEMP >= SET_DEF_HEAT_TLIMIT) {
                /* Y → 蒸发器温度已达标(≥10℃), 霜已融化, 提前转入滴水
                 *     (正常情况: 温度到了就不需要继续加热)
                 */
                Defrost_TransitionToDrip();
                return;
            }

            /* N → 结束: 继续加热 (时间未到, 温度未达标) */
        }

    } else {
        /* ============================================================
         *  N → 未在除霜, 当前处于[待机阶段]
         *
         *  除霜间隔满时?
         *    TMR_DEF_INTV_CNT 由定时中断每秒递增,
         *    到达 SET_DEF_INTERVAL(3小时) 时置位 ST_TMR_DEF_INTV_DONE
         * ============================================================ */

        if (tmr_bits & ST_TMR_DEF_INTV_DONE) {
            /* Y → 除霜间隔到时, 开始除霜 */

            /* 1. 标记正在除霜 */
            xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);

            /* 停压缩机 (加热子程序会按流程重新启动并升频) */
            Defrost_StopCompressor();

            /* 2. 标记正在加热 */
            xEventGroupSetBits(SysEventGroup, ST_DEF_HEATING);

            /* 加热计时先清零, 等压缩机升到320Hz后再正式开始计时 */
            g_TimerData.TMR_DEF_DUR_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);

            /* 关闭蒸发风机和冷凝风机 */
            BSP_Relay_Off(RELAY_EVAP_FAN);
            xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON);
            BSP_Relay_Off(RELAY_COND_FAN);
            xEventGroupClearBits(SysEventGroup, ST_COND_FAN1_ON);

            /* 开启加热器 */
            DefrostHeater_On();

            /* 调用加热子程序 (处理膨胀阀步进 + 延时后开压缩机 + 升频) */
            Defrost_HeatSubroutine();
        }

        /* ============================================================
         *  温差条件提前触发除霜 (流程图2右下角: 温差 + 最短间隔2-3小时)
         *
         *  即使间隔未到3小时, 如果:
         *    1. 柜温 - 蒸发温度 > SET_DEF_TEMPDIFF_THR (15℃)
         *    2. 且已过最短间隔 SET_DEF_MIN_INTV (2小时)
         *  → 提前触发除霜 (蒸发器可能已严重结霜)
         * ============================================================ */
        else if (g_TimerData.TMR_DEF_INTV_CNT >= SET_DEF_MIN_INTV) {
            float temp_diff = sensor.VAR_CABINET_TEMP - sensor.VAR_EVAP_TEMP;

            if (temp_diff > SET_DEF_TEMPDIFF_THR) {
                /* 温差过大, 提前触发除霜 */
                xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);
                Defrost_StopCompressor();
                xEventGroupSetBits(SysEventGroup, ST_DEF_HEATING);

                g_TimerData.TMR_DEF_DUR_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);

                BSP_Relay_Off(RELAY_EVAP_FAN);
                xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON);
                BSP_Relay_Off(RELAY_COND_FAN);
                xEventGroupClearBits(SysEventGroup, ST_COND_FAN1_ON);

                DefrostHeater_On();
                Defrost_HeatSubroutine();
            }
        }
    }
}


/* ===================================================================
 *  子逻辑2: 加热子程序
 *
 *  流程图 (2.除霜流程.pdf 右侧):
 *
 *    加热子程序
 *    → 延时X时间够?             ← 第一个延时(30s): 等待系统稳定
 *        N → 结束 (等待下次调用)
 *        Y → 延时步进             ← 膨胀阀步进到除霜位置
 *    → 延时X时间够?             ← 第二个延时(30s): 等待阀门到位
 *        N → 结束
 *        Y → 15s后开压缩机       ← 等15s后启动压缩机
 *            由慢到快步进         ← 压缩机频率逐步升高
 *    → 测温度                    ← 确认加热已开始工作
 *    → 结束
 *
 *  实现方式: 状态机 (不阻塞, 每秒被主循环调用一次)
 *
 *  状态转换:
 *    IDLE → WAIT_DELAY1(30s) → VALVE_STEP → WAIT_DELAY2(30s)
 *         → COMP_DELAY(15s) → COMP_RAMP(升频) → RUNNING(测温)
 *
 *  延时X = SET_DEF_STEP_DLY (30秒)
 *  压缩机延迟 = SET_DEF_COMP_DLY (15秒)
 * =================================================================== */
void Defrost_HeatSubroutine(void)
{
    switch (s_heat_state) {

    /* ================================================================
     *  HEAT_IDLE: 空闲 → 启动第一个延时X
     *
     *  刚进入加热阶段, 初始化延时计数器,
     *  进入第一个等待周期
     * ================================================================ */
    case HEAT_IDLE:
        s_heat_delay_cnt = 0;
        s_heat_state = HEAT_WAIT_DELAY1;
        break;

    /* ================================================================
     *  HEAT_WAIT_DELAY1: 延时X时间够? (第一个30s)
     *
     *  流程图: "延时X时间够? → N → 结束"
     *  等待30秒让系统稳定 (停机后管路压力平衡)
     * ================================================================ */
    case HEAT_WAIT_DELAY1:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_STEP_DLY) {
            /* Y → 延时到, 进入膨胀阀步进 */
            s_heat_state = HEAT_VALVE_STEP;
        }
        /* N → 结束(等待下次调用) */
        break;

    /* ================================================================
     *  HEAT_VALVE_STEP: 延时步进 (膨胀阀动作)
     *
     *  流程图: "延时步进"
     *  将膨胀阀步进到除霜位置, 让高温制冷剂能通过蒸发器
     *  动作完成后立即进入第二个延时等待
     * ================================================================ */
    case HEAT_VALVE_STEP:
        Defrost_ValveStep();           /* 膨胀阀步进到除霜位置 */
        s_heat_delay_cnt = 0;          /* 复位计数, 准备第二个延时 */
        s_heat_state = HEAT_WAIT_DELAY2;
        break;

    /* ================================================================
     *  HEAT_WAIT_DELAY2: 延时X时间够? (第二个30s)
     *
     *  流程图: 第二个"延时X时间够? → N → 结束"
     *  等待30秒让膨胀阀完全到位
     * ================================================================ */
    case HEAT_WAIT_DELAY2:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_STEP_DLY) {
            /* Y → 延时到, 准备开压缩机 */
            s_heat_delay_cnt = 0;
            s_heat_state = HEAT_COMP_DELAY;
        }
        /* N → 结束(等待下次调用) */
        break;

    /* ================================================================
     *  HEAT_COMP_DELAY: 15s后开压缩机
     *
     *  流程图: "15s后开压缩机"
     *  再等15秒 (SET_DEF_COMP_DLY), 然后启动压缩机
     * ================================================================ */
    case HEAT_COMP_DELAY:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_COMP_DLY) {
            /* 15s到 → 启动压缩机, 初始低频 */
            s_comp_ramp_freq = SET_FREQ_MIN;   /* 从最低频率开始 (20Hz) */
            Defrost_StartCompressor(s_comp_ramp_freq);
            s_heat_delay_cnt = 0;
            s_heat_state = HEAT_COMP_RAMP;
        }
        /* N → 结束(等待15s) */
        break;

    /* ================================================================
     *  HEAT_COMP_RAMP: 由慢到快步进
     *
     *  流程图: "由慢到快步进"
     *  压缩机频率从最低(20Hz)逐步升到目标频率(125Hz)
     *  每秒增加一定步长, 避免瞬间高负荷
     * ================================================================ */
    case HEAT_COMP_RAMP: {
        /* 每秒升频, 直到达到目标频率 */
        const float RAMP_STEP = 10.0f;     /* 每秒升10Hz */
        const float RAMP_TARGET = SET_FREQ_MAX;   /* 目标: 320Hz */

        s_comp_ramp_freq += RAMP_STEP;
        if (s_comp_ramp_freq >= RAMP_TARGET) {
            s_comp_ramp_freq = RAMP_TARGET;
            s_heat_state = HEAT_RUNNING;

            /* 压缩机达到320Hz, 从此刻开始加热10分钟计时 */
            g_TimerData.TMR_DEF_DUR_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);
        }

        /* 更新压缩机频率并发送给变频板 */
        SysState_Lock();
        SysState_GetRawPtr()->VAR_COMP_FREQ = s_comp_ramp_freq;
        SysState_Unlock();

        /* BSP_Inverter_Send(0x02, (uint16_t)s_comp_ramp_freq); — 暂停 */
        break;
    }

    /* ================================================================
     *  HEAT_RUNNING: 测温度 → 加热运行中
     *
     *  流程图: "测温度 → 结束"
     *  压缩机已达到目标频率, 加热正常运行,
     *  温度监控和退出判断由主程序 Defrost_MainProcess() 负责:
     *    - 加热时间超时(5min, 从此刻开始计时) → 转入滴水
     *    - 蒸发器温度≥10℃ → 转入滴水
     *
     *  此状态下子程序无需额外操作, 仅保持运行
     * ================================================================ */
    case HEAT_RUNNING:
        /* 加热运行中, 无需操作 (主程序监控退出条件) */
        break;

    default:
        /* 异常状态, 复位 */
        Defrost_HeatReset();
        break;
    }
}


/* ===================================================================
 *  除霜任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒执行一轮:
 *    1. 除霜主程序 (状态机: 待机→加热→滴水→待机)
 *
 *  循环周期: 1秒
 * =================================================================== */
void Task_Defrost_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 启动除霜间隔计时
     * 上电后开始计时, 到达3小时后执行第一次除霜
     */
    g_TimerData.TMR_DEF_INTV_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);
    g_TimerData.TMR_DEF_DUR_CNT = 0;
    g_TimerData.TMR_DRIP_CNT = 0;

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        /* ============================================
         * 除霜主程序 (每秒执行)
         *   状态机驱动: 根据当前阶段执行对应逻辑
         * ============================================ */
        Defrost_MainProcess();

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
