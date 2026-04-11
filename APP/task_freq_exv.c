#include "task_freq_exv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_exv.h"
#include "bsp_inverter.h"
#include "bsp_rs485.h"
#include "task_panel.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* ===========================================================================
 * 变频控制和膨胀阀流程 (逻辑图4) — 实现文件
 *
 * 子逻辑:
 *   1. FreqExv_FreqAdjust()    频率调节 — 每1秒执行, 根据温差调整压缩机频率
 *   2. FreqExv_ExvAdjust()     膨胀阀调整 — 每30秒执行, 根据过热度调整阀门开度
 *
 * 频率调节策略 (老师要求):
 *   上电 → 3分钟C3延时 → 按键启动 → 8-10秒变频器校准(C20)
 *   → 从120Hz开始, 每秒调整频率:
 *     温差 ΔT ≥ 5℃:  +3Hz/s (大温差, 快速升频)
 *     温差 2℃ ≤ ΔT < 5℃: +2Hz/s (接近目标, 减速)
 *     温差 0 < ΔT < 2℃:  +1Hz/s (精细调节)
 *     温差 ΔT ≤ 0:        降频 (柜温已达标)
 *
 * 膨胀阀原理:
 *   电子膨胀阀(EXV)控制制冷剂流量, 影响蒸发器换热效率.
 *   通过传热温差(△TCZ)和过热度(△TP)两个指标来调节阀门开度(Kp).
 * =========================================================================== */


/* ===================================================================
 *  内部状态变量
 * =================================================================== */

/* --- 当前频率跟踪 --- */
static float s_last_freq = 0.0f;


/* ===================================================================
 *  内部辅助函数
 * =================================================================== */

/* --- 设置压缩机频率 (含上下限保护) --- */
static void PID_SetFreq(float freq_hz)
{
    if (freq_hz > SET_FREQ_MAX) freq_hz = SET_FREQ_MAX;
    if (freq_hz < SET_FREQ_MIN) freq_hz = SET_FREQ_MIN;

    s_last_freq = freq_hz;

    /* 写入全局状态 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = freq_hz;
    SysState_Unlock();

    /* 发送调频指令给变频板 — 暂停(16字节协议与ASCII手动控制冲突)
     * 当前变频板由XCOM手动发R/S/0~3控制, 恢复后取消注释
     * BSP_Inverter_Send(0x02, (uint16_t)freq_hz);
     */

    /* 调试串口打印: 当前PID计算频率 */
    {
        char freq_msg[80];
        sprintf(freq_msg, "[FREQ] %dHz (calc only)\r\n", (int)freq_hz);
        BSP_RS485_SendString(freq_msg);
    }
}

/* --- 设置膨胀阀开度 (含上下限保护) --- */
static void EXV_SetOpening(float kp)
{
    /* 下限保护: 开度不能为负 */
    if (kp < 0.0f) {
        kp = 0.0f;
    }
    /* 上限保护: 三花 R0415D07 全行程 500 步 */
    if (kp > (float)EXV_TOTAL_STEPS) {
        kp = (float)EXV_TOTAL_STEPS;
    }

    /* 写入全局传感器数据 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_EXV_OPENING = kp;
    SysState_Unlock();

    /* 驱动步进电机到目标位置, 完成后断电省功耗 */
    BSP_EXV_SetPosition((uint16_t)(kp + 0.5f), EXV_STEP_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));  /* 结束励磁保持 */
    BSP_EXV_DeEnergize();
}

/* ===================================================================
 *  子逻辑1: 频率调节 — 每1秒执行
 *
 *  根据温差(ΔT = 柜温 - 设定温度)调整压缩机频率:
 *    ΔT ≥ 5℃:      +3Hz/s  (柜温远高于设定, 快速升频)
 *    2℃ ≤ ΔT < 5℃: +2Hz/s  (接近目标, 减速)
 *    0 < ΔT < 2℃:  +1Hz/s  (精细调节)
 *    -2℃ < ΔT ≤ 0: -1Hz/s  (柜温达标, 缓慢降频)
 *    -5℃ < ΔT ≤ -2℃: -2Hz/s
 *    ΔT ≤ -5℃:     -3Hz/s  (过冷, 快速降频)
 *
 *  变频板内部自带升降频速率控制, 主控板每秒写一次新频率, 变频板平滑过渡
 * =================================================================== */
void FreqExv_FreqAdjust(void)
{
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* 压缩机未运行, 不调频 */
    if (!(sys_bits & ST_COMP_RUNNING)) {
        return;
    }

    /* 除霜期间, 频率由除霜任务控制, 这里不干预 */
    if (sys_bits & ST_DEFROST_ACTIVE) {
        return;
    }

    /* 等待热车完成 (C20定时器)
     * 老师变频板暂无转速回读, 用定时器等待压缩机稳定到120Hz
     * C20时间由 task_timer 倒计时, 到时设置 ST_TMR_WARMUP_DONE,
     * 再由主逻辑设置 ST_WARMUP_DONE */
    if (!(sys_bits & ST_WARMUP_DONE)) {
        EventBits_t tmr = xEventGroupGetBits(SysTimerEventGroup);
        if (tmr & ST_TMR_WARMUP_DONE) {
            xEventGroupSetBits(SysEventGroup, ST_WARMUP_DONE);
            BSP_RS485_SendString("[FREQ] Warmup done (C20), start adjusting\r\n");
        }
        return;
    }

    /* 计算温差 ΔT = 柜温 - 设定温度 */
    float delta_t = sensor.VAR_CABINET_TEMP - g_set_temp;
    float current_freq = s_last_freq;
    if (current_freq < SET_FREQ_MIN) {
        current_freq = SET_FREQ_INIT;  /* 首次调用, 从初始频率开始 */
    }
    float new_freq = current_freq;

    /* 同步 VAR_DELTA_T */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_DELTA_T = delta_t;
    SysState_Unlock();

    /* ---- 根据温差决定升降频步长 (每秒调一次) ---- */
    if (delta_t >= 5.0f) {
        new_freq += 3.0f;          /* 大温差: +3Hz/s */
    } else if (delta_t >= 2.0f) {
        new_freq += 2.0f;          /* 中温差: +2Hz/s */
    } else if (delta_t > 0.0f) {
        new_freq += 1.0f;          /* 小温差: +1Hz/s */
    } else if (delta_t > -2.0f) {
        new_freq -= 1.0f;          /* 稍过冷: -1Hz/s */
    } else if (delta_t > -5.0f) {
        new_freq -= 2.0f;          /* 中过冷: -2Hz/s */
    } else {
        new_freq -= 3.0f;          /* 大过冷: -3Hz/s */
    }

    PID_SetFreq(new_freq);

    /* 打印温差和调频方向 */
    {
        char dt_msg[80];
        int step = (int)(new_freq - current_freq);
        sprintf(dt_msg, "[FREQ] Tc=%.1f Ts=%.1f dT=%.1f %s%dHz/s\r\n",
                sensor.VAR_CABINET_TEMP, g_set_temp, delta_t,
                step >= 0 ? "+" : "", step);
        BSP_RS485_SendString(dt_msg);
    }

}

/* --- 兼容旧接口名, 供外部调用 --- */
void FreqExv_PidAdjust(void)
{
    FreqExv_FreqAdjust();
}

/* ---- 以下为原PID中的ETM停机保护, 调试阶段暂不启用 ----
 * 当频率降到最低且温差仍未缩小时, 说明制冷系统异常:
 *   g_AlarmFlags |= ERR_TEMP_LOW_STOP;
 *   PID_StopCompressor();
 * 后续恢复安全保护时取消注释
 * -------------------------------------------------------- */


/* ===================================================================
 *  子逻辑2: 膨胀阀调整模块 (右侧)
 *
 *  流程图 (4.变频控制和膨胀阀流程.pdf 右侧):
 *
 *    开始
 *    → 传热温差 △TCZ = 柜温 - 蒸发温度
 *       △TCZ ≥ 6.5±1.5 (即 △TCZ 在 5.0~8.0 范围内)?
 *
 *      ┌─ N(不在范围) ─────────────────────────────┐
 *      │                                            │
 *      │  → Kp = Kp - α1·△TCZ                     │
 *      │    (传热温差偏离目标, 按α1系数调整阀门开度) │
 *      │  → 输出Kp → 结束                          │
 *      │                                            │
 *      ├─ Y(在范围内) ─────────────────────────────┐│
 *      │                                           ││
 *      │  → 过热度 △TP < △TPmin (6.5~8)?          ││
 *      │                                           ││
 *      │    ┌─ 是 (过热度过低) ──────────┐         ││
 *      │    │  → EDT 报警                │         ││
 *      │    │    (过热度低, 液态制冷剂    │         ││
 *      │    │     可能进入压缩机, 危险)   │         ││
 *      │    │  → Kp = Kp - α2·△TP       │         ││
 *      │    │    (减小开度, 减少制冷剂    │         ││
 *      │    │     流量, 提高过热度)       │         ││
 *      │    │  → 输出Kp → 结束           │         ││
 *      │    │                             │         ││
 *      │    ├─ 否 (过热度正常) ──────────┐│         ││
 *      │    │  → -EDT (清除过热度报警)   ││         ││
 *      │    │  → 输出Kp → 结束           ││         ││
 *      │    │    (随过热度变化比例调整)   ││         ││
 *      │    └─────────────────────────────┘│         ││
 *      └───────────────────────────────────┘         ││
 *
 *  参数说明:
 *    △TCZ目标值 = SET_HT_DIFF_TARGET = 6.5℃
 *    △TCZ容差   = SET_HT_DIFF_TOL = 1.5℃ → 范围 5.0 ~ 8.0℃
 *    △TPmin下限 = SET_SH_MIN_LOW = 6.5℃
 *    △TPmin上限 = SET_SH_MIN_HIGH = 8.0℃
 *    α1 = SET_PID_ALPHA1 = 0.5 (传热温差调节系数)
 *    α2 = SET_PID_ALPHA2 = 0.8 (过热度调节系数)
 *
 *  调用时机: 每30秒由主循环调用一次
 * =================================================================== */
void FreqExv_ExvAdjust(void)
{
    /* ---- 读取传感器数据 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* 只在压缩机运行时才调整膨胀阀 */
    if (!(sys_bits & ST_COMP_RUNNING)) {
        return;
    }

    /* 除霜期间, 膨胀阀由除霜任务控制(全开500步), 这里不干预 */
    if (sys_bits & ST_DEFROST_ACTIVE) {
        return;
    }

    /* 热车未完成时不调整膨胀阀,
     * 等压缩机稳定运转后再根据过热度调节 */
    if (!(sys_bits & ST_WARMUP_DONE)) {
        return;
    }

    float kp = sensor.VAR_EXV_OPENING;  /* 当前膨胀阀开度 */

    /* ================================================================
     *  第1步: 计算传热温差 △TCZ = 柜温 - 蒸发温度
     *
     *  △TCZ 反映蒸发器与柜内空气之间的热交换效率:
     *    △TCZ大 → 蒸发器过冷, 换热效率低 (结霜等)
     *    △TCZ小 → 蒸发器不够冷, 制冷剂不足
     *    △TCZ在目标范围 → 换热效率正常
     * ================================================================ */
    float delta_tcz = sensor.VAR_CABINET_TEMP - sensor.VAR_EVAP_TEMP;

    /* 同步更新 VAR_HT_DIFF 供其他模块使用 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_HT_DIFF = delta_tcz;
    SysState_Unlock();

    /* ================================================================
     *  第2步: △TCZ 在 6.5±1.5 范围内 (5.0~8.0℃)?
     *
     *  目标: SET_HT_DIFF_TARGET = 6.5℃
     *  容差: SET_HT_DIFF_TOL = 1.5℃
     *  合格范围: [6.5 - 1.5, 6.5 + 1.5] = [5.0, 8.0]
     * ================================================================ */
    float tcz_low  = SET_HT_DIFF_TARGET - SET_HT_DIFF_TOL;   /* 5.0℃ */
    float tcz_high = SET_HT_DIFF_TARGET + SET_HT_DIFF_TOL;   /* 8.0℃ */

    if (delta_tcz < tcz_low || delta_tcz > tcz_high) {
        /* ============================================================
         *  N → 传热温差不在目标范围
         *
         *  Kp = Kp - α1·(△TCZ - 目标值)
         *
         *  调整逻辑:
         *    使用偏差值 (△TCZ - 目标值) 而非绝对值, 使调整有方向性:
         *    △TCZ > 目标 → 偏差为正 → Kp减小 → 阀门关小 → 减少流量
         *    △TCZ < 目标 → 偏差为负 → Kp增大 → 阀门开大 → 增加流量
         * ============================================================ */
        float tcz_err = delta_tcz - SET_HT_DIFF_TARGET;
        float old_kp = kp;
        kp = kp - SET_PID_ALPHA1 * tcz_err;

        {
            char msg[96];
            sprintf(msg, "[EXV] TCZ=%.1f OUT[%.1f~%.1f] Kp:%.0f->%.0f\r\n",
                    delta_tcz, tcz_low, tcz_high, old_kp, kp);
            BSP_RS485_SendString(msg);
        }

        /* 输出Kp */
        EXV_SetOpening(kp);
        return;
    }

    /* ================================================================
     *  Y → 传热温差在目标范围内, 换热效率正常
     *  继续检查过热度安全
     *
     *  第3步: 过热度 △TP < △TPmin?
     *
     *  △TP = VAR_SUPERHEAT (过热度, 由传感器计算)
     *  △TPmin 范围: SET_SH_MIN_LOW(6.5℃) ~ SET_SH_MIN_HIGH(8.0℃)
     *  这里取低值作为报警阈值:
     *    △TP < 6.5℃ → 过热度过低, 有液击风险
     *
     *  过热度含义:
     *    过热度 = 吸气温度 - 蒸发温度
     *    过热度低 → 制冷剂未完全蒸发, 液态进入压缩机 → 损坏压缩机
     *    过热度高 → 制冷剂完全蒸发, 安全但效率偏低
     * ================================================================ */
    float superheat = sensor.VAR_SUPERHEAT;

    if (superheat < SET_SH_MIN_LOW) {
        /* ============================================================
         *  是 → 过热度过低 (< 6.5℃)
         *
         *  EDT报警
         *  Kp = Kp - α2·△TP
         *
         *  α2 = SET_PID_ALPHA2 = 0.8
         *  △TP值小(如3℃) → Kp减小较多 → 阀门关小 → 制冷剂流量减少
         *  → 制冷剂在蒸发器中停留更久 → 充分蒸发 → 过热度升高
         * ============================================================ */
        g_AlarmFlags |= WARN_SUPERHEAT_LOW;
        float old_kp = kp;
        kp = kp - SET_PID_ALPHA2 * superheat;

        {
            char msg[96];
            sprintf(msg, "[EXV] EDT! SH=%.1f<%.1f Kp:%.0f->%.0f\r\n",
                    superheat, SET_SH_MIN_LOW, old_kp, kp);
            BSP_RS485_SendString(msg);
        }

    } else {
        /* ============================================================
         *  否 → 过热度正常 (≥ 6.5℃)
         *
         *  -EDT: 清除过热度过低报警
         *  传热温差OK + 过热度OK → 系统状态良好, 保持当前开度
         * ============================================================ */
        g_AlarmFlags &= ~WARN_SUPERHEAT_LOW;
        {
            char msg[80];
            sprintf(msg, "[EXV] OK TCZ=%.1f SH=%.1f Kp=%.0f\r\n",
                    delta_tcz, superheat, kp);
            BSP_RS485_SendString(msg);
        }
        return;  /* 无需调整, 不驱动步进电机 */
    }

    /* ================================================================
     *  第4步: 输出Kp
     *
     *  将计算后的Kp值输出到膨胀阀驱动
     *  (只有传热温差偏离 或 过热度过低 时才会走到这里)
     * ================================================================ */
    EXV_SetOpening(kp);
}


/* ===================================================================
 *  变频控制和膨胀阀任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒检查一次PID周期:
 *    PID周期(30s)到时 → 依次执行:
 *      1. PID调整模块 (调节压缩机频率)
 *      2. 膨胀阀调整模块 (调节阀门开度)
 *
 *  循环周期: 1秒
 *    频率调节: 每1秒执行 (根据温差 +3/+2/+1 Hz)
 *    膨胀阀:   每30秒执行 (根据过热度调节开度)
 *    变频器通信状态: 每5秒打印一次 (echo结果)
 * =================================================================== */
void Task_FreqExv_Process(void const *argument)
{
    (void)argument;

    /* 初始化 */
    s_last_freq = 0.0f;

    /* 初始化EXV硬件 */
    BSP_EXV_Init();
    BSP_EXV_ResetToZero();

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    static uint8_t s_inv_poll_cnt = 0;

    for (;;) {
        /* ============================================
         * 每1秒: 频率调节 (根据温差 +3/+2/+1 Hz)
         * ============================================ */
        FreqExv_FreqAdjust();

        /* ============================================
         * 每30秒: 膨胀阀调节 (根据过热度调整开度)
         * ============================================ */
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (tmr_bits & ST_TMR_PID_DONE) {
            FreqExv_ExvAdjust();
            g_TimerData.TMR_PID_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_PID_DONE);
        }

        /* 变频器通信状态轮询 — 暂停(改用ASCII手动控制, 无状态回读) */

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
