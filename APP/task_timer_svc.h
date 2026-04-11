#ifndef TASK_TIMER_SVC_H
#define TASK_TIMER_SVC_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 定时中断服务流程 (逻辑图5) — FreeRTOS 任务
 *
 * 功能:
 *   每1秒执行一次, 递增所有定时计数器,
 *   计数器到达目标值时清零并置位对应的"到时"标志.
 *   其他任务通过检查"到时"标志来获知定时事件.
 *
 * 服务的定时器 (按流程图顺序):
 *   1. 化霜间隔    TMR_DEF_INTV_CNT    → ST_TMR_DEF_INTV_DONE
 *   2. 化霜时长    TMR_DEF_DUR_CNT     → ST_TMR_DEF_DUR_DONE
 *   3. C2 停机保护  TMR_C2_CNT          → ST_TMR_C2_DONE
 *   4. C3 通电延迟  TMR_C3_CNT          → ST_TMR_C3_DONE
 *   5. C7 待机时间  TMR_C7_CNT          → ST_TMR_C7_DONE
 *   6. C8 最短运行  TMR_C8_CNT          → ST_TMR_C8_DONE
 *   7. PID 周期    TMR_PID_CNT          → ST_TMR_PID_DONE
 *   (流程图 "标记......到时" 表示还有更多, 以下为扩展):
 *   8. 滴水时间    TMR_DRIP_CNT         → ST_TMR_DEF_DRIP_DONE
 *   9. 热车C20     TMR_WARMUP_CNT       → ST_TMR_WARMUP_DONE
 *  10. 风机F3延时   TMR_EVAP_FAN_DLY_CNT → ST_TMR_EVAP_FAN_DLY
 *
 * 后续扩展: 作为6大逻辑块之一, 独立FreeRTOS任务
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_TimerSvc_Process(void const *argument);

/* --- 定时服务主逻辑 (内部调用, 也可单独测试) --- */
void TimerSvc_TickProcess(void);

#endif /* TASK_TIMER_SVC_H */
