#include "task_timer_svc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 定时中断服务流程 (逻辑图5) — 实现文件
 *
 * 本模块是整个系统的"心跳", 每1秒执行一次,
 * 为其他所有任务提供统一的定时服务.
 *
 * 工作原理:
 *   每1秒 → 递增所有活跃的计数器
 *         → 检查每个计数器是否到达目标值
 *         → 到达时: 清空计数器, 置位"到时"标志
 *         → 消费任务检测到标志后, 执行对应逻辑并清除标志
 *
 * 计数器由"消费任务"控制生命周期:
 *   - 消费任务启动定时: 将计数器清零, 清除到时标志
 *   - 本模块每秒递增计数器
 *   - 到达目标 → 本模块清零计数器, 置位到时标志
 *   - 消费任务检测到标志 → 执行逻辑, 清除标志
 *   - 如需重新计时 → 消费任务再次清零计数器
 *
 * 流程图 (5.定时中断服务流程.pdf):
 *
 *    开始
 *    │
 *    ├── 化霜间隔到? ── 是 → 清空计数器 → 标记化霜间隔到
 *    │       否
 *    ├── 化霜时长到? ── 是 → 清空计数器 → 标记化霜到时
 *    │       否
 *    ├── C2到时?     ── 是 → 清空计数器 → 标记C2到时
 *    │       否
 *    ├── C3到时?     ── 是 → 清空计数器 → 标记C3到时
 *    │       否
 *    ├── C7到时?     ── 是 → 清空计数器 → 标记C7到时
 *    │       否
 *    ├── C8到时?     ── 是 → 清空计数器 → 标记C8到时
 *    │       否
 *    ├── PID到时?    ── 是 → 清空计数器 → 标记PID到时
 *    │       否
 *    ├── (扩展: 滴水/热车/风机延时等, 流程图"标记......到时")
 *    │
 *    └── 结束
 *
 * 注意:
 *   流程图中每个定时器判断是"瀑布式"从上到下顺序检查,
 *   不是 if-else 互斥关系, 每个都会被检查到.
 *   一次tick中可能有多个定时器同时到时.
 * =========================================================================== */


/* ===================================================================
 *  定时服务主逻辑: 每1秒调用一次
 *
 *  严格按流程图从上到下顺序检查每个定时器.
 *  每个定时器的处理模式完全一致:
 *    1. 递增计数器
 *    2. 计数器 ≥ 目标值?
 *       是 → 清空计数器 + 置位到时标志
 *       否 → 继续下一个
 * =================================================================== */
void TimerSvc_TickProcess(void)
{
    /* ================================================================
     *  流程图第1行: 化霜间隔到?
     *
     *  计数器: TMR_DEF_INTV_CNT
     *  目标值: SET_DEF_INTERVAL = 3小时 (10800秒)
     *  标志位: ST_TMR_DEF_INTV_DONE
     *
     *  用途: 除霜任务(图2)等待此标志, 触发下一次除霜周期
     *  注意: 仅在标志未置位时递增 (到时后停止计数, 等消费任务处理)
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_DEF_INTV_DONE)) {
            g_TimerData.TMR_DEF_INTV_CNT++;
            if (g_TimerData.TMR_DEF_INTV_CNT >= SET_DEF_INTERVAL) {
                g_TimerData.TMR_DEF_INTV_CNT = 0;      /* 清空计数器 */
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_DEF_INTV_DONE); /* 标记化霜间隔到 */
            }
        }
    }

    /* ================================================================
     *  流程图第2行: 化霜时长到?
     *
     *  计数器: TMR_DEF_DUR_CNT
     *  目标值: SET_DEF_HEAT_MAX = 45分钟 (2700秒)
     *  标志位: ST_TMR_DEF_DUR_DONE
     *
     *  用途: 除霜任务(图2)加热阶段超时保护
     *        加热超过45分钟强制转入滴水阶段
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_DEF_DUR_DONE)) {
            g_TimerData.TMR_DEF_DUR_CNT++;
            if (g_TimerData.TMR_DEF_DUR_CNT >= SET_DEF_HEAT_MAX) {
                g_TimerData.TMR_DEF_DUR_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_DEF_DUR_DONE);
            }
        }
    }

    /* ================================================================
     *  流程图第3行: C2到时?
     *
     *  计数器: TMR_C2_CNT
     *  目标值: SET_STOP_TIME_C2 = 180秒 (3分钟)
     *  标志位: ST_TMR_C2_DONE
     *
     *  用途: 温度控制任务(图1)停机保护延时
     *        压缩机停机后需等待C2时间才允许再次启动,
     *        保护压缩机避免频繁启停
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_C2_DONE)) {
            g_TimerData.TMR_C2_CNT++;
            if (g_TimerData.TMR_C2_CNT >= SET_STOP_TIME_C2) {
                g_TimerData.TMR_C2_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_C2_DONE);
            }
        }
    }

    /* ================================================================
     *  流程图第4行: C3到时?
     *
     *  计数器: TMR_C3_CNT
     *  目标值: SET_POWERON_DLY_C3 = 180秒 (3分钟)
     *  标志位: ST_TMR_C3_DONE
     *
     *  用途: 温度控制任务(图1)通电延迟
     *        系统首次上电后等待C3时间, 让各传感器稳定后再决策
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_C3_DONE)) {
            g_TimerData.TMR_C3_CNT++;
            if (g_TimerData.TMR_C3_CNT >= SET_POWERON_DLY_C3) {
                g_TimerData.TMR_C3_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_C3_DONE);
            }
        }
    }

    /* ================================================================
     *  流程图第5行: C7到时?
     *
     *  计数器: TMR_C7_CNT
     *  目标值: SET_STBY_TIME_C7 = 300秒 (5分钟)
     *  标志位: ST_TMR_C7_DONE
     *
     *  用途: 温度控制任务(图1)待机保护时间
     *        柜温达标停机后, 至少待机C7时间才允许重新开机,
     *        避免频繁启停
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_C7_DONE)) {
            g_TimerData.TMR_C7_CNT++;
            if (g_TimerData.TMR_C7_CNT >= SET_STBY_TIME_C7) {
                g_TimerData.TMR_C7_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_C7_DONE);
            }
        }
    }

    /* ================================================================
     *  流程图第6行: C8到时?
     *
     *  计数器: TMR_C8_CNT
     *  目标值: SET_RUN_MIN_C8 = 300秒 (5分钟)
     *  标志位: ST_TMR_C8_DONE
     *
     *  用途: 温度控制任务(图1)最短运行保护
     *        压缩机启动后至少运行C8时间才允许停机,
     *        避免压缩机短周期运行损坏
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_C8_DONE)) {
            g_TimerData.TMR_C8_CNT++;
            if (g_TimerData.TMR_C8_CNT >= SET_RUN_MIN_C8) {
                g_TimerData.TMR_C8_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_C8_DONE);
            }
        }
    }

    /* ================================================================
     *  流程图第7行: PID到时?
     *
     *  计数器: TMR_PID_CNT
     *  目标值: SET_PID_PERIOD = 30秒
     *  标志位: ST_TMR_PID_DONE
     *
     *  用途: 变频控制任务(图4)PID周期触发
     *        每30秒执行一次PID调整和膨胀阀调整
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_PID_DONE)) {
            g_TimerData.TMR_PID_CNT++;
            if (g_TimerData.TMR_PID_CNT >= SET_PID_PERIOD) {
                g_TimerData.TMR_PID_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_PID_DONE);
            }
        }
    }

    /* ================================================================
     *  以下为流程图 "标记......到时" 扩展部分
     *  流程图底部PID后标注 "标记......到时", 表示还有更多定时器,
     *  以下是 sys_state.h 中定义的其余定时器
     * ================================================================ */

    /* ================================================================
     *  扩展第1项: 滴水时间到?
     *
     *  计数器: TMR_DRIP_CNT
     *  目标值: SET_DRIP_TIME = 300秒 (5分钟)
     *  标志位: ST_TMR_DEF_DRIP_DONE
     *
     *  用途: 除霜任务(图2)滴水阶段超时
     *        滴水5分钟后除霜完全结束, 恢复制冷
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_DEF_DRIP_DONE)) {
            g_TimerData.TMR_DRIP_CNT++;
            if (g_TimerData.TMR_DRIP_CNT >= SET_DRIP_TIME) {
                g_TimerData.TMR_DRIP_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_DEF_DRIP_DONE);
            }
        }
    }

    /* ================================================================
     *  扩展第2项: 热车C20到时?
     *
     *  计数器: TMR_WARMUP_CNT
     *  目标值: SET_WARMUP_C20 = 10秒
     *  标志位: ST_TMR_WARMUP_DONE
     *
     *  用途: 温度控制任务(图1)热车等待
     *        压缩机启动后运行2分钟暖机, 之后才开始PID调整
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_WARMUP_DONE)) {
            g_TimerData.TMR_WARMUP_CNT++;
            if (g_TimerData.TMR_WARMUP_CNT >= SET_WARMUP_C20) {
                g_TimerData.TMR_WARMUP_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_WARMUP_DONE);
            }
        }
    }

    /* ================================================================
     *  扩展第3项: 蒸发风机F3延时到?
     *
     *  计数器: TMR_EVAP_FAN_DLY_CNT
     *  目标值: SET_EVAP_FAN_DLY = 30秒
     *  标志位: ST_TMR_EVAP_FAN_DLY
     *
     *  用途: 蒸发风机任务(图3) F1=1/F1=2 模式的启动延时
     *        压缩机或除霜结束后等30秒蒸发器变冷再开风机
     * ================================================================ */
    {
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
        if (!(tmr_bits & ST_TMR_EVAP_FAN_DLY)) {
            g_TimerData.TMR_EVAP_FAN_DLY_CNT++;
            if (g_TimerData.TMR_EVAP_FAN_DLY_CNT >= SET_EVAP_FAN_DLY) {
                g_TimerData.TMR_EVAP_FAN_DLY_CNT = 0;
                xEventGroupSetBits(SysTimerEventGroup,
                                   ST_TMR_EVAP_FAN_DLY);
            }
        }
    }

    /* ================================================================
     *  辅助计数器: 连续运行时间 (不置位标志, 供告警任务直接读取)
     *
     *  计数器: TMR_LONGRUN_CNT
     *  用途: 温度控制任务(图1)告警处理
     *        压缩机运行时递增, 停机时由告警逻辑清零
     *        到达1小时/2小时时触发长时间运行告警
     *
     *  注意: 此计数器不走"清零+置位"模式,
     *        而是由告警逻辑直接比较阈值和清零
     * ================================================================ */
    {
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
        if (sys_bits & ST_COMP_RUNNING) {
            g_TimerData.TMR_LONGRUN_CNT++;
        }
        /* 压缩机未运行时, 由告警逻辑负责清零 */
    }

    /* ================================================================
     *  辅助计数器: 高压/低压超时 (不置位标志, 供告警任务直接读取)
     *
     *  计数器: TMR_PRES_HIGH_CNT / TMR_PRES_LOW_CNT
     *  用途: 温度控制任务(图1)告警处理
     *        压力异常持续时递增, 由告警逻辑直接比较和清零
     *
     *  注意: 这两个计数器的递增条件由告警逻辑控制,
     *        本模块仅在压缩机运行时提供基础递增
     *        (具体告警判断在温控任务中)
     * ================================================================ */
    /* TMR_PRES_HIGH_CNT 和 TMR_PRES_LOW_CNT 的递增
     * 由温控任务的告警逻辑 TempCtrl_AlarmProcess() 直接管理,
     * 因为需要结合压力传感器值做条件判断,
     * 不适合在此无条件递增
     */

    /* ================================================================
     *  1秒基准心跳 (可选)
     *
     *  计数器: TMR_TICK_1S_CNT
     *  标志位: ST_TMR_TICK_1S
     *
     *  用途: 供需要1秒同步的任务使用
     *        每秒置位一次, 消费任务检测后自行清除
     * ================================================================ */
    g_TimerData.TMR_TICK_1S_CNT++;
    xEventGroupSetBits(SysTimerEventGroup, ST_TMR_TICK_1S);
}


/* ===================================================================
 *  定时中断服务任务主循环
 *
 *  作为FreeRTOS任务运行, 精确每1秒执行一轮:
 *    递增所有活跃计数器, 检查到时条件, 置位到时标志
 *
 *  优先级说明:
 *    使用 osPriorityAboveNormal (高于普通),
 *    确保定时服务优先于业务逻辑任务执行,
 *    保证计时精度
 *
 *  循环周期: 1秒 (使用 vTaskDelayUntil 保证精确周期)
 * =================================================================== */
void Task_TimerSvc_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 所有计数器清零 (由SysState_Init已完成, 此处做安全保障) */
    g_TimerData.TMR_TICK_1S_CNT      = 0;
    g_TimerData.TMR_PID_CNT          = 0;
    g_TimerData.TMR_C2_CNT           = 0;
    g_TimerData.TMR_C3_CNT           = 0;
    g_TimerData.TMR_C7_CNT           = 0;
    g_TimerData.TMR_C8_CNT           = 0;
    g_TimerData.TMR_DEF_INTV_CNT     = 0;
    g_TimerData.TMR_DEF_DUR_CNT      = 0;
    g_TimerData.TMR_DRIP_CNT         = 0;
    g_TimerData.TMR_WARMUP_CNT       = 0;
    g_TimerData.TMR_EVAP_FAN_DLY_CNT = 0;
    g_TimerData.TMR_LONGRUN_CNT      = 0;
    g_TimerData.TMR_PRES_HIGH_CNT    = 0;
    g_TimerData.TMR_PRES_LOW_CNT     = 0;

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 使用 vTaskDelayUntil 保证精确1秒周期
     * 避免任务执行时间累积造成周期漂移
     */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /* ============================================
         * 定时中断服务 (每秒执行)
         *   递增所有计数器, 检查到时, 置位标志
         * ============================================ */
        TimerSvc_TickProcess();

        /* 精确1秒周期延时 */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}
