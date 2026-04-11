#include "task_evap_fan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_relay.h"
#include <stdbool.h>

/* ===========================================================================
 * 蒸发风机(1控6)流程 (逻辑图3) — 实现文件
 *
 * 子逻辑:
 *   1. EvapFan_ModeF1()    F1=1 跟随压缩机模式
 *   2. EvapFan_ModeF2()    F1=2 常开模式
 *   3. EvapFan_ModeF4()    F1=4 蒸发温度合格模式
 *
 * 蒸发风机原理:
 *   蒸发器表面安装6台风机, 通过一个继电器统一控制(1控6).
 *   风机将柜内空气吹过蒸发器进行换热, 实现制冷.
 *   根据不同工况(除霜/压缩机状态/温度), 选择不同的启停策略.
 *
 * 三种模式 (由 SET_EVAP_FAN_MODE = F1 决定):
 *
 *   F1=1  跟随压缩机:
 *     除霜时 → 关风机
 *     压缩机运行 → 开风机 (有F3延时)
 *     压缩机停止 → 关风机
 *
 *   F1=2  常开:
 *     除霜时 → 关风机
 *     非除霜 → 开风机 (有F3延时)
 *
 *   F1=4  温度合格:
 *     蒸发温度合格 → 开风机
 *     蒸发温度不合格 → 关风机
 *
 * F3延时说明:
 *   SET_EVAP_FAN_DLY (F3=30秒)
 *   压缩机启动后需要等待F3秒蒸发器变冷后再开风机,
 *   避免刚启动时吹出热风影响柜温.
 *   由定时中断服务递增 TMR_EVAP_FAN_DLY_CNT,
 *   到达F3时置位 ST_TMR_EVAP_FAN_DLY.
 * =========================================================================== */


/* ===================================================================
 *  内部辅助函数 / 硬件操作桩
 * =================================================================== */

/* --- 蒸发风机开启 (K1 继电器, PC9, 1控2) --- */
static void EvapFan_On(void)
{
    BSP_Relay_On(RELAY_EVAP_FAN);
}

/* --- 蒸发风机关闭 --- */
static void EvapFan_Off(void)
{
    BSP_Relay_Off(RELAY_EVAP_FAN);
}

/* --- 标记风机正在运行 --- */
static void EvapFan_MarkRunning(void)
{
    xEventGroupSetBits(SysEventGroup, ST_EVAP_FAN_ON);
}

/* --- 标记风机已关闭/已停止 --- */
static void EvapFan_MarkStopped(void)
{
    xEventGroupClearBits(SysEventGroup, ST_EVAP_FAN_ON);
}

/* --- 启动F3延时计时器 ---
 * 压缩机启动后, 开始计时F3秒, 到时后才开风机
 */
static void EvapFan_StartF3Timer(void)
{
    g_TimerData.TMR_EVAP_FAN_DLY_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_EVAP_FAN_DLY);
}


/* ===================================================================
 *  子逻辑1: F1=1 模式 — 跟随压缩机运行
 *
 *  流程图 (3.风机流程.pdf 左侧, F1=1分支):
 *
 *    F1=1?
 *    → Y
 *      → 正在除霜?
 *        ┌─ Y(正在除霜) ──────────────────┐
 *        │  → 风机正在运行?                │
 *        │    Y → 关闭风机                 │
 *        │        标记风机已关闭            │
 *        │        → 结束                   │
 *        │    N → 结束                     │
 *        │                                 │
 *        ├─ N(未在除霜) ──────────────────┐│
 *        │  → 压缩机运行?                 ││
 *        │    ┌─ Y(压缩机运行) ──────┐    ││
 *        │    │  → 风机正在运行?      │    ││
 *        │    │    Y → 结束           │    ││
 *        │    │    N → 风机F3延时到时? │    ││
 *        │    │        Y → 开启风机    │    ││
 *        │    │            标记运行    │    ││
 *        │    │            → 结束     │    ││
 *        │    │        N → 结束       │    ││
 *        │    │                       │    ││
 *        │    ├─ N(压缩机停) ────────┐│    ││
 *        │    │  → 风机正在运行?     ││    ││
 *        │    │    Y → 关闭风机      ││    ││
 *        │    │        标记已关闭    ││    ││
 *        │    │        → 结束       ││    ││
 *        │    │    N → 结束         ││    ││
 *        │    └──────────────────────┘│    ││
 *        └────────────────────────────┘    ││
 *                                          ││
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void EvapFan_ModeF1(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    bool fan_running = (sys_bits & ST_EVAP_FAN_ON) != 0;
    bool defrosting  = (sys_bits & ST_DEFROST_ACTIVE) != 0;
    bool comp_on     = (sys_bits & ST_COMP_RUNNING) != 0;

    /* 检测压缩机 停→运行 的跳变, 重启F3延时计时器
     * 流程图3: 压缩机每次启动后需重新等待F3秒再开风机 */
    static bool s_prev_comp_on = false;
    if (comp_on && !s_prev_comp_on) {
        EvapFan_StartF3Timer();
    }
    s_prev_comp_on = comp_on;

    /* ================================================================
     *  第1步: 正在除霜?
     * ================================================================ */
    if (defrosting) {
        /* ---- Y → 正在除霜, 风机必须关闭 ---- */
        if (fan_running) {
            /* 风机正在运行 → 关闭风机, 标记风机已关闭 */
            EvapFan_Off();
            EvapFan_MarkStopped();
        }
        /* N → 风机已经是关闭状态, 无需操作 → 结束 */
        return;
    }

    /* ================================================================
     *  第2步: 未在除霜 → 压缩机运行?
     * ================================================================ */
    if (comp_on) {
        /* ---- Y → 压缩机正在运行 ---- */
        if (fan_running) {
            /* 风机已在运行, 保持 → 结束 */
            return;
        }

        /* ---- N → 风机未运行, 检查F3延时 ---- */
        /* 风机F3延时到时?
         *   压缩机启动后需等待F3秒(30s), 蒸发器变冷后再开风机,
         *   避免吹出热风.
         *   TMR_EVAP_FAN_DLY_CNT 由定时中断递增,
         *   到达 SET_EVAP_FAN_DLY 时置位 ST_TMR_EVAP_FAN_DLY
         */
        if (tmr_bits & ST_TMR_EVAP_FAN_DLY) {
            /* Y → F3延时到, 开启风机 */
            EvapFan_On();
            EvapFan_MarkRunning();
        }
        /* N → F3延时未到, 继续等待 → 结束 */

    } else {
        /* ---- N → 压缩机未运行 ---- */
        if (fan_running) {
            /* 风机正在运行 → 关闭风机, 标记已关闭 */
            EvapFan_Off();
            EvapFan_MarkStopped();
        }
        /* N → 风机已关闭, 无需操作 → 结束 */
    }
}


/* ===================================================================
 *  子逻辑2: F1=2 模式 — 常开模式
 *
 *  流程图 (3.风机流程.pdf 右侧上方, F1=2分支):
 *
 *    F1=2?
 *    → Y
 *      → 正在除霜?
 *        ┌─ Y(正在除霜) ──────────────────┐
 *        │  → 风机运行?                    │
 *        │    Y → 关闭风机                 │
 *        │        标记风机已关闭            │
 *        │        → 结束                   │
 *        │    N → 结束                     │
 *        │                                 │
 *        ├─ N(未在除霜) ──────────────────┐│
 *        │  → 风机运行?                   ││
 *        │    Y → 结束 (保持运行)          ││
 *        │    N → 风机F3延时到时?          ││
 *        │        Y → 开启风机             ││
 *        │            标记风机正在运行      ││
 *        │            → 结束               ││
 *        │        N → 结束                 ││
 *        └────────────────────────────────┘││
 *
 *  说明:
 *    F1=2模式下风机基本常开, 只有除霜时才关闭.
 *    非除霜状态下, 如果风机未运行(如刚从除霜恢复),
 *    需要等待F3延时后再开启.
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void EvapFan_ModeF2(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    bool fan_running = (sys_bits & ST_EVAP_FAN_ON) != 0;
    bool defrosting  = (sys_bits & ST_DEFROST_ACTIVE) != 0;

    /* 检测除霜 结束 的跳变, 重启F3延时计时器
     * 流程图3: 除霜结束后需重新等待F3秒再开风机 */
    static bool s_prev_defrosting = false;
    if (!defrosting && s_prev_defrosting) {
        EvapFan_StartF3Timer();
    }
    s_prev_defrosting = defrosting;

    /* ================================================================
     *  第1步: 正在除霜?
     * ================================================================ */
    if (defrosting) {
        /* ---- Y → 正在除霜, 风机必须关闭 ---- */
        if (fan_running) {
            /* 风机运行 → 关闭风机, 标记风机已关闭 */
            EvapFan_Off();
            EvapFan_MarkStopped();
        }
        /* N → 风机已关闭 → 结束 */
        return;
    }

    /* ================================================================
     *  第2步: 未在除霜 → 风机应该运行
     * ================================================================ */
    if (fan_running) {
        /* Y → 风机已在运行, 保持 → 结束 */
        return;
    }

    /* ---- N → 风机未运行, 检查F3延时 ----
     * 风机F3延时到时?
     *   除霜结束后需等待F3秒(30s), 让蒸发器恢复冷态后再开风机
     */
    if (tmr_bits & ST_TMR_EVAP_FAN_DLY) {
        /* Y → F3延时到, 开启风机 */
        EvapFan_On();
        EvapFan_MarkRunning();
    }
    /* N → F3延时未到, 继续等待 → 结束 */
}


/* ===================================================================
 *  子逻辑3: F1=4 模式 — 蒸发温度合格模式
 *
 *  流程图 (3.风机流程.pdf 右侧下方, F1=4分支):
 *
 *    F1=4?
 *    → Y
 *      → 蒸发温度合格?
 *        ┌─ Y(温度合格) ──────────────────┐
 *        │  → 风机正在运行?                │
 *        │    Y → 结束 (保持运行)          │
 *        │    N → 开启风机                 │
 *        │        标记风机正在运行          │
 *        │        → 结束                   │
 *        │                                 │
 *        ├─ N(温度不合格) ────────────────┐│
 *        │  → 风机正在运行?               ││
 *        │    Y → 关闭风机                ││
 *        │        标记风机已停止           ││
 *        │        → 结束                  ││
 *        │    N → 结束                    ││
 *        └────────────────────────────────┘│
 *
 *  说明:
 *    F1=4模式下, 风机仅在蒸发温度达到合格值时才运行.
 *    蒸发温度合格判定:
 *      VAR_EVAP_TEMP <= SET_EVAP_TEMP_OK (-15℃)
 *      即蒸发器已经足够冷, 可以开始送风制冷.
 *    对应状态标志: ST_EVAP_TEMP_QUAL
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void EvapFan_ModeF4(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    bool fan_running = (sys_bits & ST_EVAP_FAN_ON) != 0;

    /* ================================================================
     *  第1步: 读取柜温(SHT30), 与面板设定温度比较
     *
     *  回差控制:
     *    柜温 ≤ 设定温度 - 2℃  → 关风机 (温度够低了)
     *    柜温 ≥ 设定温度        → 开风机 (温度回升了)
     *    中间区域               → 保持当前状态 (防抖)
     * ================================================================ */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    extern float g_set_temp;
    float cabinet_temp = sensor.VAR_CABINET_TEMP;  /* INUI2 NTC 柜温 */
    float temp_off = g_set_temp - 2.0f;           /* 关风机阈值 */
    float temp_on  = g_set_temp;                   /* 开风机阈值 */

    /* ================================================================
     *  第2步: 回差判断
     * ================================================================ */
    if (cabinet_temp <= temp_off) {
        /* 柜温 ≤ 设定-2℃ → 温度够低, 关风机 */
        if (fan_running) {
            EvapFan_Off();
            EvapFan_MarkStopped();
        }
        xEventGroupClearBits(SysEventGroup, ST_EVAP_TEMP_QUAL);
    }
    else if (cabinet_temp >= temp_on) {
        /* 柜温 ≥ 设定温度 → 温度回升, 开风机 */
        if (!fan_running) {
            EvapFan_On();
            EvapFan_MarkRunning();
        }
        xEventGroupSetBits(SysEventGroup, ST_EVAP_TEMP_QUAL);
    }
    /* 中间区域: 保持当前状态 (回差防抖) */
}


/* ===================================================================
 *  蒸发风机主逻辑入口
 *
 *  流程图 (3.风机流程.pdf 完整流程):
 *
 *    开始
 *    → 读取设置值、测量值和定时标记
 *    → F1=1?
 *        Y → EvapFan_ModeF1()  → 结束
 *        N → F1=2?
 *            Y → EvapFan_ModeF2()  → 结束
 *            N → F1=4?
 *                Y → EvapFan_ModeF4()  → 结束
 *                N → 结束 (无效模式, 不操作)
 *
 *  黄色标记"1"连接:
 *    左侧 F1=1 判断N出口 → 右侧 F1=2 判断入口
 *    即 F1!=1 时进入右侧继续判断 F1=2 和 F1=4
 *
 *  调用时机: 每1秒由任务主循环调用一次
 * =================================================================== */
static void EvapFan_MainProcess(void)
{
    /* ---- 读取设置值、测量值和定时标记 ----
     * 设置值: SET_EVAP_FAN_MODE (F1), SET_EVAP_FAN_DLY (F3),
     *         SET_EVAP_TEMP_OK (蒸发温度合格值)
     * 测量值: VAR_EVAP_TEMP (蒸发器温度)
     * 定时标记: ST_TMR_EVAP_FAN_DLY (F3延时到时)
     * 状态标记: ST_COMP_RUNNING, ST_DEFROST_ACTIVE, ST_EVAP_FAN_ON
     */

    uint8_t fan_mode = SET_EVAP_FAN_MODE;  /* F1: 风机运行模式 (1/2/4) */

    /* ================================================================
     *  模式分发: 根据F1选择对应子逻辑
     *
     *  流程图从上到下依次判断:
     *    F1=1? → Y → 左侧逻辑 (跟随压缩机)
     *    F1=2? → Y → 右侧上方逻辑 (常开)
     *    F1=4? → Y → 右侧下方逻辑 (温度合格)
     * ================================================================ */
    switch (fan_mode) {

    case 1:
        /* F1=1: 跟随压缩机运行模式 */
        EvapFan_ModeF1();
        break;

    case 2:
        /* F1=2: 常开模式 (仅除霜关) */
        EvapFan_ModeF2();
        break;

    case 4:
        /* F1=4: 蒸发温度合格模式 */
        EvapFan_ModeF4();
        break;

    default:
        /* 无效模式值, 不操作 → 结束 */
        break;
    }
}


/* ===================================================================
 *  蒸发风机任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒执行一轮:
 *    1. 蒸发风机主逻辑 (根据F1模式选择启停策略)
 *
 *  循环周期: 1秒
 * =================================================================== */
void Task_EvapFan_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 风机默认关闭, 等待条件满足后开启 */
    EvapFan_Off();
    EvapFan_MarkStopped();

    /* 初始化F3延时计时器 */
    EvapFan_StartF3Timer();

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        /* ============================================
         * 蒸发风机主逻辑 (每秒执行)
         *   根据F1模式分发到对应子逻辑
         * ============================================ */
        EvapFan_MainProcess();

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
