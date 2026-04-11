#include "task_temp_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_relay.h"
#include "bsp_inverter.h"
#include "bsp_exv.h"
#include "bsp_rs485.h"
#include "task_panel.h"
#include <stdbool.h>
#include <stdio.h>

/* ===========================================================================
 * 温度控制流程 (逻辑图1) — 实现文件
 *
 * 子逻辑:
 *   1. TempCtrl_ShutdownAlarm()    停机异常逻辑告警
 *   2. TempCtrl_CompressorStart()  压缩机开机逻辑
 *   3. TempCtrl_OilHeatControl()   油壳加热逻辑
 *   4. TempCtrl_AlarmProcess()     告警(温度压力)处理逻辑
 *   5. TempCtrl_MainLogic()        温度逻辑(主逻辑)
 * =========================================================================== */

/* ===================================================================
 *  内部辅助函数 / 硬件操作桩 (待硬件资料确认后实现)
 * =================================================================== */

/* --- 油壳加热器控制 (K2 继电器, PC6) --- */
static void OilHeater_On(void)
{
    BSP_Relay_On(RELAY_OIL_HEATER);
    xEventGroupSetBits(SysEventGroup, ST_OIL_HEAT_ON);
}

static void OilHeater_Off(void)
{
    BSP_Relay_Off(RELAY_OIL_HEATER);
    xEventGroupClearBits(SysEventGroup, ST_OIL_HEAT_ON);
}

/* --- 等待变频器与压缩机自检校准完成 ---
 * 变频板与压缩机上电后有8-10秒自检校准过程,
 * 必须等自检完成后才能发频率指令, 否则压缩机会出问题.
 *
 * 老师变频板暂无状态回读, 采用固定等待10秒策略.
 */
#define INV_SELFTEST_SEC    10   /* 变频器自检等待时间 (秒) */

static bool WaitInverterReady(void)
{
    BSP_RS485_SendString("[INV] Waiting for self-test (10s)...\r\n");
    vTaskDelay(pdMS_TO_TICKS(INV_SELFTEST_SEC * 1000));
    BSP_RS485_SendString("[INV] Self-test done, ready\r\n");
    return true;
}

/* --- 压缩机启动 (通过变频器, 初始频率120Hz) --- */
static void Compressor_Start(void)
{
    /* WaitInverterReady() 暂停 — 变频板由ASCII手动控制, 无需等自检
     * if (!WaitInverterReady()) { return; }
     */
    /* 16字节调频指令暂停, 变频板由XCOM手动发R/S/0~3控制
     * BSP_Inverter_Send(0x01, (uint16_t)SET_FREQ_INIT);
     */
    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);

    /* 设置EXV初始开度(半开), 避免全关导致制冷剂不流通 */
    BSP_EXV_SetPosition((uint16_t)SET_EXV_INIT_OPENING, EXV_STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));  /* 结束励磁保持 */
    BSP_EXV_DeEnergize();
    SysState_Lock();
    SysState_GetRawPtr()->VAR_EXV_OPENING = SET_EXV_INIT_OPENING;
    SysState_Unlock();

    BSP_RS485_SendString("[COMP] START (manual INV ctrl)\r\n");
}

/* --- 压缩机停止 --- */
static void Compressor_Stop(void)
{
    /* 16字节停机指令暂停, 变频板由XCOM手动发S控制
     * BSP_Inverter_Send(0x00, 0);
     */
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
    BSP_RS485_SendString("[COMP] STOP (manual INV ctrl)\r\n");
}

/* --- 读取VAC相序状态 ---
 * 老师变频板暂无故障码回读, 默认返回正常
 * 后续如果变频板增加上行状态帧, 在此解析
 */
static bool VAC_PhaseOK(void)
{
    return true;  /* 暂无故障检测, 默认正常 */
}

/* --- 读取变频器过流状态 ---
 * 老师变频板暂无故障码回读, 默认返回正常
 */
static bool Inverter_IsOvercurrent(void)
{
    return false;  /* 暂无故障检测 */
}

/* --- 读取变频器过热状态 ---
 * 老师变频板暂无故障码回读, 默认返回正常
 */
static bool Inverter_IsOverheat(void)
{
    return false;  /* 暂无故障检测 */
}

/* --- 通知用户 (通过调试串口打印具体报警信息) --- */
static void NotifyUser(uint32_t alarm_code)
{
    char msg[128];

    if (alarm_code & ERR_SENSOR_CABINET)
        BSP_RS485_SendString("[ALARM] E1: Cabinet sensor fault!\r\n");
    if (alarm_code & ERR_VDC_LOW)
        BSP_RS485_SendString("[ALARM] EDC: DC voltage too low!\r\n");
    if (alarm_code & ERR_VAC_PHASE)
        BSP_RS485_SendString("[ALARM] EAC: Phase loss/error!\r\n");
    if (alarm_code & ERR_INV_OVERCURR)
        BSP_RS485_SendString("[ALARM] EFI: Inverter overcurrent!\r\n");
    if (alarm_code & ERR_INV_OVERHEAT)
        BSP_RS485_SendString("[ALARM] EFT: Inverter overheat!\r\n");
    if (alarm_code & ERR_TEMP_LOW_STOP)
        BSP_RS485_SendString("[ALARM] ETM: Freq min & temp abnormal, STOP!\r\n");

    if (alarm_code & WARN_EXHAUST_HIGH)
        BSP_RS485_SendString("[WARN] WTM: Exhaust temp too high!\r\n");
    if (alarm_code & WARN_SUCTION_LOW)
        BSP_RS485_SendString("[WARN] WTL: Suction temp too low!\r\n");
    if (alarm_code & WARN_PRES_HIGH)
        BSP_RS485_SendString("[WARN] WPH: Discharge pressure high!\r\n");
    if (alarm_code & WARN_PRES_LOW)
        BSP_RS485_SendString("[WARN] WPL: Suction pressure low!\r\n");
    if (alarm_code & WARN_LONGRUN)
        BSP_RS485_SendString("[WARN] WLC: Long run time!\r\n");
    if (alarm_code & WARN_SUPERHEAT_LOW)
        BSP_RS485_SendString("[WARN] EDT: Superheat too low!\r\n");

    /* 打印报警码数值 */
    sprintf(msg, "[ALARM] code=0x%08lX\r\n", (unsigned long)alarm_code);
    BSP_RS485_SendString(msg);
}

/* --- 蒸发风扇关闭 (K1继电器, 1控2) --- */
static void EvapFan_AllOff(void)
{
    BSP_Relay_Off(RELAY_EVAP_FAN);
    xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON);
}

/* --- 冷凝风扇关闭 (K5继电器, 1控3) --- */
static void CondFan_AllOff(void)
{
    BSP_Relay_Off(RELAY_COND_FAN);
    xEventGroupClearBits(SysEventGroup, ST_COND_FAN1_ON);
}

/* --- 柜温传感器有效性判断 ---
 * 返回: true = 正常, false = 故障(开路/短路/超量程)
 */
static bool CabinetSensor_IsValid(float tc)
{
    /* TODO: 根据实际传感器特性确认合理范围
     * 当前: -50℃ ~ 100℃ 视为有效
     * 超出范围视为传感器开路或短路
     */
    return (tc >= -50.0f && tc <= 100.0f);
}


/* ===================================================================
 *  子逻辑1: 停机异常逻辑告警
 *
 *  流程图 (1.1):
 *    开始
 *    → 采集电源电压VDC (PB1/VSININ)
 *    → 读取电源电压下限VDCmin
 *    → VDC ≥ VDCmin?
 *        Y → 清除 -EDC
 *        N → 置位 EDC, 关机状态
 *    → VAC错/断相?
 *        Y → 清除 -EAC
 *        N → 置位 EAC, 关机状态
 *    → 变频器过流?
 *        Y → 清除 -EFI
 *        N → 置位 EFI, 关机状态
 *    → 变频器过热?
 *        Y → 清除 -EFT
 *        N → 置位 EFT, 关机状态
 *    → 通知用户
 *    → 结束
 * =================================================================== */
void TempCtrl_ShutdownAlarm(void)
{
    SysVarData_t sensor;
    bool need_shutdown = false;

    /* ---- 第1步: 采集电源电压 VDC (来自 PB1/VSININ, ADC采集) ---- */
    SysState_GetSensor(&sensor);
    float vdc = sensor.VAR_VDC_VOLTAGE;

    /* ---- 第2步: 判断 VDC ≥ VDCmin ---- */
    /* 注意: VAR_VDC_VOLTAGE 暂未被 task_adc 采集, 跳过此检测
     * TODO: task_adc 增加 PB1/VSININ 电压采集后启用
     */
    if (vdc > 0.1f) {
        /* 有有效电压读数时才判断 */
        if (vdc >= SET_VDC_MIN) {
            g_AlarmFlags &= ~ERR_VDC_LOW;
        } else {
            g_AlarmFlags |= ERR_VDC_LOW;
            need_shutdown = true;
        }
    } else {
        /* 无采集数据(0V), 不触发告警 */
        g_AlarmFlags &= ~ERR_VDC_LOW;
    }

    /* ---- 第3步: 判断 VAC 错/断相 ---- */
    if (VAC_PhaseOK()) {
        /* 相序正常 → 清除EAC告警 */
        g_AlarmFlags &= ~ERR_VAC_PHASE;
    } else {
        /* 错相/断相 → 置位EAC, 需要关机 */
        g_AlarmFlags |= ERR_VAC_PHASE;
        need_shutdown = true;
    }

    /* ---- 第4步: 判断 变频器过流 ---- */
    if (!Inverter_IsOvercurrent()) {
        /* 正常 → 清除EFI告警 */
        g_AlarmFlags &= ~ERR_INV_OVERCURR;
    } else {
        /* 过流 → 置位EFI, 需要关机 */
        g_AlarmFlags |= ERR_INV_OVERCURR;
        need_shutdown = true;
    }

    /* ---- 第5步: 判断 变频器过热 ---- */
    if (!Inverter_IsOverheat()) {
        /* 正常 → 清除EFT告警 */
        g_AlarmFlags &= ~ERR_INV_OVERHEAT;
    } else {
        /* 过热 → 置位EFT, 需要关机 */
        g_AlarmFlags |= ERR_INV_OVERHEAT;
        need_shutdown = true;
    }

    /* ---- 第6步: 如有异常 → 停机 + 通知用户 ---- */
    if (need_shutdown) {
        Compressor_Stop();
        xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
        NotifyUser(g_AlarmFlags & ERR_MASK_ALL);
    }
}


/* ===================================================================
 *  子逻辑2: 压缩机开机逻辑
 *
 *  流程图 (1.1):
 *    开启压缩机
 *    → F = 120 (初始频率120Hz)
 *    → 热车时长C20到时?
 *        N → 继续等待 (保持F=120运行)
 *        Y → 进入PID调整模块
 *    → PID计算, 幅度限制及输出
 *    → 结束
 *
 *  说明: 此函数在确认无异常后被调用, 启动压缩机并等待热车完成
 * =================================================================== */
void TempCtrl_CompressorStart(void)
{
    EventBits_t sys_bits  = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits  = xEventGroupGetBits(SysTimerEventGroup);

    /* 如果压缩机已经在运行, 不重复启动 */
    if (sys_bits & ST_COMP_RUNNING) {
        return;
    }

    /* ---- 第1步: 开启压缩机, 设定初始频率120Hz ---- */
    Compressor_Start();
    /* 注意: Compressor_Start() 内部已经发了120Hz, 不需要再调 Compressor_SetFreq */

    /* ---- 第2步: 启动热车计时器 C20 ----
     * 热车计时由定时中断服务(逻辑图5)中的1s定时器驱动:
     *   g_TimerData.TMR_WARMUP_CNT 每秒递增
     *   到达 SET_WARMUP_C20 时置位 ST_TMR_WARMUP_DONE
     *
     * 这里复位计数器, 开始计时
     */
    g_TimerData.TMR_WARMUP_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_WARMUP_DONE);
    xEventGroupClearBits(SysEventGroup, ST_WARMUP_DONE);

    /* ---- 第3步: 等待热车完成 (在主循环中轮询) ----
     * 注意: 不在此处阻塞等待, 而是由主循环周期性检查
     *       热车期间保持 F=120Hz 运转
     *       热车完成后主循环将调用PID调整模块
     */
}

/* --- PID调整模块 (已迁移到逻辑图4) ---
 * 完整PID和膨胀阀调整逻辑已在 task_freq_exv.c 中实现 (逻辑图4).
 * Task_FreqExv 任务每30秒独立执行 PID调整 + 膨胀阀调整.
 *
 * 本函数保留作为温控任务内的快速频率修正桩:
 *   温控主逻辑中的PID调用点仍会触发此函数,
 *   但实际的PID计算和频率输出已由 FreqExv_PidAdjust() 负责.
 *   此处仅做简单的安全保护检查.
 */
/* PID核心逻辑已迁移到 Task_FreqExv (逻辑图4)
 * 该任务独立以30秒为周期执行:
 *   FreqExv_PidAdjust()  — 压缩机频率调节
 *   FreqExv_ExvAdjust()  — 膨胀阀开度调节
 */


/* ===================================================================
 *  子逻辑4: 告警（温度压力）处理逻辑
 *
 *  流程图 (1.温度控制流程.pdf 从左数第二个):
 *
 *    开始
 *    → 1s到时?  N→等待  Y→继续
 *    → 采集: T1(冷箱内温度), TH(排气温度), TL(吸气温度),
 *            F(压缩机频率), PL(吸气压力), PH(排气压力), Kp(膨胀阀开度)
 *    → 读取: TS(设定温度), Tmax(排温上限), Tmin(吸温下限), VDCmin
 *
 *    ① 排温 TH ≥ Tmax [110℃]?
 *        Y → WTM (排温过高告警)
 *        N → -(WTM)
 *
 *    ② 连续开机时长 ≥ 1-2小时?
 *        Y → WLC (长时间运行告警)
 *        N → -(WLC)
 *
 *    ③ 设定值TLS - 吸温TL ≤ Tmin [5-10℃, 温差15]?
 *        Y → WTL (吸温过低告警)
 *        N → -(WTL)
 *
 *    ④ 排压 PH ≥ Pmax [110/70]?
 *        Y → WPH1 → 高压超时? → Y:WPH2  N:-(WPH2)
 *        N → -(WPH1), -(WPH2), 复位高压计时
 *
 *    ⑤ 吸压 PL ≤ Pmin [20]?
 *        Y → WPL1 → 低压超时? → Y:WPL2  N:-(WPL2)
 *        N → -(WPL1), -(WPL2), 复位低压计时
 *
 *    ⑥ 汇总: WPL|WPH|WTM|WTL|WLC 任一存在?
 *        Y → WEN (综合告警, 通知用户)
 *        N → -(WEN)
 *    → 结束
 *
 *  调用时机: 每1秒由主循环调用一次 (与流程图"1s到时?"对应)
 *  注意: 本逻辑只产生WARN级别告警, 不直接停机;
 *        停机由子逻辑1(停机异常逻辑告警)的ERR级别负责
 * =================================================================== */
void TempCtrl_AlarmProcess(void)
{
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    /* ================================================================
     * ① 排温 TH ≥ Tmax [110℃]?
     *    排气温度过高说明压缩比过大或制冷剂不足
     * ================================================================ */
    if (sensor.VAR_EXHAUST_TEMP >= SET_EXHAUST_TMAX) {
        /* Y → WTM 置位 */
        g_AlarmFlags |= WARN_EXHAUST_HIGH;
    } else {
        /* N → -(WTM) 清除 */
        g_AlarmFlags &= ~WARN_EXHAUST_HIGH;
    }

    /* ================================================================
     * ② 连续开机时长 ≥ 1-2小时?
     *    TMR_LONGRUN_CNT 由定时中断服务(逻辑图5)在压缩机运行期间每秒递增
     *    压缩机停止时由其他逻辑复位
     *
     *    流程图标注"1-2小时":
     *      SET_WARN_LONGRUN_L = 3600s (1小时, 初级告警)
     *      SET_WARN_LONGRUN_H = 7200s (2小时, 严重告警)
     *    此处用低阈值(1小时)触发告警, 符合流程图"≥1-2小时"
     * ================================================================ */
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    if (sys_bits & ST_COMP_RUNNING) {
        /* 压缩机运行中, 检查累计运行时长 */
        if (g_TimerData.TMR_LONGRUN_CNT >= SET_WARN_LONGRUN_L) {
            /* Y → WLC 置位 */
            g_AlarmFlags |= WARN_LONGRUN;
        } else {
            /* N → -(WLC) 清除 */
            g_AlarmFlags &= ~WARN_LONGRUN;
        }
    } else {
        /* 压缩机未运行, 清除长运行告警并复位计时 */
        g_AlarmFlags &= ~WARN_LONGRUN;
        g_TimerData.TMR_LONGRUN_CNT = 0;
    }

    /* ================================================================
     * ③ 设定值TLS - 吸温TL ≤ Tmin [5-10℃, 温差15]?
     *    含义: 吸气温度过低 → 吸气过热度不足, 有液击风险
     *    计算: (设定蒸发温度TLS) - (实际吸气温度TL)
     *    如果差值 ≤ Tmin(15℃), 说明吸温偏低
     *
     *    SET_SUCTION_TMIN = 15.0f (温差阈值)
     *    TLS 近似取 SET_TEMP_TS (设定温度)
     * ================================================================ */
    float suction_diff = g_set_temp - sensor.VAR_SUCTION_TEMP;

    if (suction_diff <= SET_SUCTION_TMIN) {
        /* Y → WTL 置位: 吸温与设定值差距过小, 吸气过热度不足 */
        g_AlarmFlags |= WARN_SUCTION_LOW;
    } else {
        /* N → -(WTL) 清除 */
        g_AlarmFlags &= ~WARN_SUCTION_LOW;
    }

    /* ================================================================
     * ④ 排压 PH ≥ Pmax [110/70]?
     *    流程图标注两个值: 高档110bar, 低档70bar
     *    此处用 SET_DISCHARGE_PMAX_L (70bar) 作为告警触发阈值
     *
     *    分两级:
     *      WPH1 = 排压超限 (即时告警)
     *      WPH2 = 排压超限且持续超时 (严重告警)
     * ================================================================ */
    if (sensor.VAR_DISCHARGE_PRES >= SET_DISCHARGE_PMAX_L) {
        /* Y → WPH1 置位: 排压超限 */
        g_AlarmFlags |= WARN_PRES_HIGH;

        /* 压力超限期间每秒递增 (本函数每1秒调用一次) */
        g_TimerData.TMR_PRES_HIGH_CNT++;

        /* 高压超时? */
        if (g_TimerData.TMR_PRES_HIGH_CNT >= SET_WARN_PRES_TMO) {
            /* Y → WPH2 置位: 高压持续超时 */
            g_AlarmFlags |= WARN_PRES_HIGH_TMO;
        } else {
            /* N → -(WPH2) */
            g_AlarmFlags &= ~WARN_PRES_HIGH_TMO;
        }
    } else {
        /* N → -(WPH1), -(WPH2), 复位高压计时 */
        g_AlarmFlags &= ~WARN_PRES_HIGH;
        g_AlarmFlags &= ~WARN_PRES_HIGH_TMO;
        g_TimerData.TMR_PRES_HIGH_CNT = 0;
    }

    /* ================================================================
     * ⑤ 吸压 PL ≤ Pmin [20]?
     *    吸气压力过低说明制冷剂不足或膨胀阀开度不够
     *
     *    分两级:
     *      WPL1 = 吸压过低 (即时告警)
     *      WPL2 = 吸压过低且持续超时 (严重告警)
     * ================================================================ */
    if (sensor.VAR_SUCTION_PRES <= SET_SUCTION_PMIN) {
        /* Y → WPL1 置位: 吸压过低 */
        g_AlarmFlags |= WARN_PRES_LOW;

        /* 压力超限期间每秒递增 (本函数每1秒调用一次) */
        g_TimerData.TMR_PRES_LOW_CNT++;

        /* 低压超时? */
        if (g_TimerData.TMR_PRES_LOW_CNT >= SET_WARN_PRES_TMO) {
            /* Y → WPL2 置位: 低压持续超时 */
            g_AlarmFlags |= WARN_PRES_LOW_TMO;
        } else {
            /* N → -(WPL2) */
            g_AlarmFlags &= ~WARN_PRES_LOW_TMO;
        }
    } else {
        /* N → -(WPL1), -(WPL2), 复位低压计时 */
        g_AlarmFlags &= ~WARN_PRES_LOW;
        g_AlarmFlags &= ~WARN_PRES_LOW_TMO;
        g_TimerData.TMR_PRES_LOW_CNT = 0;
    }

    /* ================================================================
     * ⑥ 汇总: WPL|WPH|WTM|WTL|WLC 任一存在 → WEN通知用户
     *    流程图最底部: 所有告警汇聚到一个判断
     * ================================================================ */
    if (g_AlarmFlags & WARN_MASK_ALL) {
        /* 存在任意WARN → WEN 置位, 通知用户 */
        g_AlarmFlags |= WARN_NOTIFY_USER;
        NotifyUser(g_AlarmFlags & WARN_MASK_ALL);
    } else {
        /* 全部正常 → -(WEN) 清除 */
        g_AlarmFlags &= ~WARN_NOTIFY_USER;
    }
}


/* ===================================================================
 *  子逻辑5: 温度逻辑（主逻辑）
 *
 *  流程图 (1.温度控制流程.pdf 最左边):
 *
 *    开始
 *    → 系统初始化 (由FreeRTOS启动时完成)
 *
 *    → 开机状态?
 *        N → 关压缩机, 关冷凝风扇, 关蒸发风扇 → 结束
 *        Y → 继续
 *
 *    → 读取设置: Ts(设定温度), C1(温度回差), C2/C3(停机保护),
 *                C7(待机保护), C8(最短运行时间), A1-A3(报警阈值)
 *       (均为 sys_config.h 编译期常量)
 *
 *    → 首次运行?
 *        Y → 通电延迟时间C3到时?
 *              N → 结束 (等待通电延迟)
 *              Y → 继续
 *        N → 继续
 *
 *    → 柜温传感器正常?
 *        故障 → 故障安全, 显示故障码E1 → 结束
 *        正常 → 继续
 *
 *    → 采样值: 柜温Tc
 *
 *    → 压缩机工作?
 *
 *        否(停机中):
 *          → 停机时长 ≥ C2?
 *              否 → 结束 (停机保护时间未到)
 *              是 → 柜温 Tc ≥ Ts+C1?
 *                    N → 结束 (温度够低, 无需开机)
 *                    Y → 开启压缩机 → 结束
 *
 *        是(运行中):
 *          → 低速运行时长 > C8?
 *              否 → 结束 (最短运行时间未到, 继续运行)
 *              是 → 柜温 Tc ≥ Ts+C1?
 *                    Y → PID计算 → PID幅度限制及输出 → 结束
 *                    N → 停止运行 → 结束
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void TempCtrl_MainLogic(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    /* ================================================================
     * 第1步: 开机状态?
     *   流程图顶部: 系统初始化之后的第一个判断
     *   N → 关压缩机, 关冷凝风扇, 关蒸发风扇
     * ================================================================ */
    if (!(sys_bits & ST_SYSTEM_ON)) {
        Compressor_Stop();
        CondFan_AllOff();
        EvapFan_AllOff();
        return;
    }

    /* ================================================================
     * 第2步: 读取设置
     *   Ts  = SET_TEMP_TS         (-20℃, 设定温度)
     *   C1  = SET_TEMP_HYST_C1    (2℃, 温度回差)
     *   C2  = SET_STOP_TIME_C2    (180s, 停机保护时间)
     *   C3  = SET_POWERON_DLY_C3  (180s, 通电延迟)
     *   C7  = SET_STBY_TIME_C7    (300s, 待机保护)
     *   C8  = SET_RUN_MIN_C8      (300s, 最短运行时间)
     *   A1-A3 = 报警阈值
     *
     *   均为编译期常量 (#define), 无需运行时读取
     * ================================================================ */

    /* ================================================================
     * 第3步: 首次运行?
     *   Y → 通电延迟时间C3到时?
     *         N → 结束 (等待上电稳定)
     *         Y → 清除首次运行标志, 继续
     *   N → 直接继续
     *
     *   说明: 系统首次上电时 ST_FIRST_RUN 被置位,
     *         通电延迟C3保护压缩机不被立即启动,
     *         防止频繁断电重启损坏压缩机
     * ================================================================ */
    if (sys_bits & ST_FIRST_RUN) {
        if (!(tmr_bits & ST_TMR_C3_DONE)) {
            /* 通电延迟未到 → 结束, 等待下次循环 */
            static uint8_t s_c3_print_cnt = 0;
            if (++s_c3_print_cnt >= 10) {   /* 每10秒打印一次 */
                s_c3_print_cnt = 0;
                char msg[64];
                sprintf(msg, "[SYS] C3 waiting... %ds/%ds\r\n",
                        (int)g_TimerData.TMR_C3_CNT, SET_POWERON_DLY_C3);
                BSP_RS485_SendString(msg);
            }
            return;
        }
        /* C3到时 → 清除首次运行标志, 后续循环不再进入此分支 */
        xEventGroupClearBits(SysEventGroup, ST_FIRST_RUN);
        BSP_RS485_SendString("[SYS] C3 done, system ready\r\n");
    }

    /* ================================================================
     * 第4步: 柜温传感器正常?
     *   故障 → 置位 E1 (ERR_SENSOR_CABINET)
     *          启动"故障安全"模式: 停机, 显示故障码
     *   正常 → 继续
     *
     *   故障安全: 传感器坏了无法知道真实温度,
     *            必须停机防止温度失控
     * ================================================================ */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);
    float tc = sensor.VAR_CABINET_TEMP;

    if (!CabinetSensor_IsValid(tc)) {
        /* 传感器故障 → E1, 故障安全 */
        g_AlarmFlags |= ERR_SENSOR_CABINET;
        Compressor_Stop();
        NotifyUser(ERR_SENSOR_CABINET);
        return;
    }
    /* 传感器正常 → 清除E1 */
    g_AlarmFlags &= ~ERR_SENSOR_CABINET;

    /* ================================================================
     * 第5步: 采样值: 柜温Tc (已在上面通过 SysState_GetSensor 获取)
     *        读设置: 停机时长C2, 运行时长 (由定时器计数)
     * ================================================================ */

    /* 除霜期间, 压缩机由除霜任务控制, 温控不干预 */
    if (sys_bits & ST_DEFROST_ACTIVE) {
        return;
    }

    /* ================================================================
     * 第6步: 压缩机工作?
     *   核心温控决策 — 基于回差(C1)的启停判断:
     *
     *   ┌──────────────────────────────────────────────┐
     *   │  停机中:  Tc ≥ Ts+C1  → 开机 (温度偏高)     │
     *   │  运行中:  Tc < Ts+C1  → 停机 (温度已降到位)  │
     *   │          Tc ≥ Ts+C1  → PID继续调节           │
     *   └──────────────────────────────────────────────┘
     *
     *   回差C1的作用: 防止压缩机在设定温度附近频繁启停
     *   例: Ts=-20℃, C1=2℃ → 升到-18℃开机, 降到<-18℃停机
     * ================================================================ */

    if (!(sys_bits & ST_COMP_RUNNING)) {
        /* ============================================================
         * 压缩机当前: 否(停机中)
         *
         * → 停机时长 ≥ C2?
         *   C2 = 停机保护时间(180s), 防止压缩机频繁启停
         *   TMR_C2_CNT 由定时中断服务每秒递增,
         *   到达C2时置位 ST_TMR_C2_DONE
         * ============================================================ */
        if (!(tmr_bits & ST_TMR_C2_DONE)) {
            /* 否 → 结束: 停机保护时间未到, 不允许启动 */
            return;
        }

        /* 是: 停机保护已过 → 柜温 Tc ≥ Ts+C1? */
        if (tc >= g_set_temp + SET_TEMP_HYST_C1) {
            /* Y → 开启压缩机 */
            {
                char msg[80];
                sprintf(msg, "[SYS] Tc=%.1f >= Ts+C1=%.1f, starting compressor\r\n",
                        tc, g_set_temp + SET_TEMP_HYST_C1);
                BSP_RS485_SendString(msg);
            }
            TempCtrl_CompressorStart();

            /* 复位最短运行时间计时器C8 (从0开始计时) */
            g_TimerData.TMR_C8_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_C8_DONE);

            /* 复位长时间运行计时器 (从0开始计时) */
            g_TimerData.TMR_LONGRUN_CNT = 0;
        }
        /* N → 结束: 温度够低(Tc < Ts+C1), 无需开机 */

    } else {
        /* ============================================================
         * 压缩机当前: 是(运行中)
         *
         * → 低速运行时长 > C8?
         *   C8 = 最短运行时间(300s), 防止压缩机启动后很快停止
         *   TMR_C8_CNT 由定时中断服务每秒递增,
         *   到达C8时置位 ST_TMR_C8_DONE
         * ============================================================ */
        if (!(tmr_bits & ST_TMR_C8_DONE)) {
            /* 否 → 结束: 最短运行时间未到, 必须继续运行
             * (即使温度已达标也不能停, 保护压缩机)
             */
            return;
        }

        /* 是: 最短运行时间已过 → 柜温 Tc ≥ Ts+C1? */
        if (tc >= g_set_temp + SET_TEMP_HYST_C1) {
            /* Y → 温度还高, 继续运行, PID调整频率
             *
             *   PID调整需要满足两个前置条件:
             *   a) 热车时长C20已完成 (CompressorWarmup_IsDone)
             *   b) PID周期30s到时 (ST_TMR_PID_DONE)
             *
             *   热车期间保持F=120Hz, 不做PID调整
             */
            /* PID频率调节和膨胀阀调节已迁移到 task_freq_exv 独立任务
             * 此处无需操作, task_freq_exv 自行管理PID周期和热车状态 */

        } else {
            /* N → 温度已降到位(Tc < Ts+C1), 停止运行 */
            {
                char msg[80];
                sprintf(msg, "[SYS] Tc=%.1f < Ts+C1=%.1f, stopping compressor\r\n",
                        tc, g_set_temp + SET_TEMP_HYST_C1);
                BSP_RS485_SendString(msg);
            }
            Compressor_Stop();

            /* 复位停机保护计时器C2 */
            g_TimerData.TMR_C2_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_C2_DONE);

            /* 清除热车状态, 下次启动需重新热车 */
            xEventGroupClearBits(SysEventGroup, ST_WARMUP_DONE);
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_WARMUP_DONE);
            g_TimerData.TMR_WARMUP_CNT = 0;
        }
    }
}


/* ===================================================================
 *  子逻辑3: 油壳加热逻辑
 *
 *  流程图 (1.1):
 *    开始
 *    → 压缩机开机?
 *        Y → 油壳加热关 → 结束
 *        N → 环境温度 ≤ 10°C?
 *            Y → 油壳加热开 → 结束
 *            N → 油壳加热关 → 结束
 *
 *  硬件: K2 继电器 → 滑油热丝 (PSD0D)
 * =================================================================== */
void TempCtrl_OilHeatControl(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* ---- 判断: 压缩机是否在运行 ---- */
    if (sys_bits & ST_COMP_RUNNING) {
        /* 压缩机运行中 → 油壳加热关闭 (压缩机自身产热, 无需额外加热) */
        OilHeater_Off();
        return;
    }

    /* ---- 压缩机未运行 → 判断环境温度 (SHT30) ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    if (sensor.VAR_AMBIENT_TEMP <= SET_OIL_HEAT_TEMP) {
        /* 环境温度 ≤ 10℃ → 开启油壳加热 */
        OilHeater_On();
    } else {
        /* 环境温度 > 10℃ → 关闭油壳加热 */
        OilHeater_Off();
    }
}


/* ===================================================================
 *  温度控制任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒执行一轮 (对应流程图"1s到时?"):
 *
 *    ┌─ 第1步: 停机异常逻辑告警  (ERR级别, 最高优先级)
 *    │  第2步: 告警(温度压力)处理 (WARN级别, 通知用户)
 *    │  第3步: 传感器故障恢复检查 (ERR_SENSOR_CABINET自恢复)
 *    │    ↓ 有ERR → 跳过主逻辑, 仅管理油壳加热
 *    │  第4步: 温度逻辑(主逻辑)   (启停决策 + PID)
 *    │  第5步: 油壳加热逻辑       (任何状态都执行)
 *    └─ 延时1秒, 回到第1步
 *
 *  5个子逻辑在同一个FreeRTOS任务中按序执行,
 *  后续6大逻辑块完成后, 每块将独立为一个FreeRTOS任务
 * =================================================================== */
void Task_TempCtrl_Process(void const *argument)
{
    (void)argument;

    /* 系统初始化: 上电后不自动开机, 等待用户按电源键启动
     * C3计时器在按键开机后才开始有意义
     */
    /* xEventGroupSetBits(SysEventGroup, ST_SYSTEM_ON); */ /* 去掉自动开机, 必须按键 */
    xEventGroupSetBits(SysEventGroup, ST_FIRST_RUN);
    g_TimerData.TMR_C3_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_C3_DONE);

    /* 初始状态: 停机保护计时器C2直接置为到时
     * (首次上电时不需要等待停机保护, C3延迟已经提供了保护)
     */
    xEventGroupSetBits(SysTimerEventGroup, ST_TMR_C2_DONE);

    /* 等待传感器数据稳定 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        /* ============================================
         * 第1步: 停机异常逻辑告警
         *   最高优先级, 每次必检
         *   检查: VDC电压 / VAC相序 / 变频器过流 / 变频器过热
         *   有异常 → ERR级别, 直接停机
         * ============================================ */
        // TempCtrl_ShutdownAlarm();  /* 调试阶段暂时屏蔽停机报警 */

        /* ============================================
         * 第2步: 告警(温度压力)处理逻辑
         *   WARN级别: 不直接停机, 通知用户
         *   检查: 排温/连续运行/吸温/排压/吸压
         * ============================================ */
        TempCtrl_AlarmProcess();

        /* ============================================
         * 第3步: 传感器故障恢复检查
         *   传感器故障(E1)是由主逻辑设置的ERR,
         *   在ERR状态下主逻辑被跳过, 无法自行清除,
         *   所以在此独立检查传感器是否恢复正常
         * ============================================ */
        if (g_AlarmFlags & ERR_SENSOR_CABINET) {
            SysVarData_t s;
            SysState_GetSensor(&s);
            if (CabinetSensor_IsValid(s.VAR_CABINET_TEMP)) {
                g_AlarmFlags &= ~ERR_SENSOR_CABINET;
            }
        }

        /* ============================================
         * ERR安全门: 有严重故障时跳过主逻辑
         *   ERR级别(VDC/VAC/变频器/传感器)存在时,
         *   禁止运行温度逻辑(防止错误启动压缩机),
         *   但油壳加热仍需正常管理
         * ============================================ */
        /* 调试阶段暂时屏蔽ERR安全门, 报警不阻止主逻辑运行 */
        // if (g_AlarmFlags & ERR_MASK_ALL) {
        //     TempCtrl_OilHeatControl();
        //     vTaskDelay(pdMS_TO_TICKS(1000));
        //     continue;
        // }

        /* ============================================
         * 第4步: 温度逻辑(主逻辑)
         *   核心温控决策:
         *   开机/关机判断 → 传感器检查 → 回差启停 → PID调节
         * ============================================ */
        TempCtrl_MainLogic();

        /* ============================================
         * 第5步: 油壳加热逻辑
         *   无论系统处于何种状态都执行
         * ============================================ */
        TempCtrl_OilHeatControl();

        /* ============================================
         * 循环延时: 1秒
         *   对应流程图中"1s到时?"的周期
         * ============================================ */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
