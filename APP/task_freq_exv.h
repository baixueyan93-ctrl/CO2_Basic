#ifndef TASK_FREQ_EXV_H
#define TASK_FREQ_EXV_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 变频控制(PID)和膨胀阀流程 (逻辑图4) — FreeRTOS 任务
 *
 * 包含子逻辑:
 *   1. FreqExv_PidAdjust()     PID调整模块 (左侧): 压缩机频率调节
 *   2. FreqExv_ExvAdjust()     膨胀阀调整模块 (右侧): 膨胀阀开度调节
 *
 * PID调整模块原理 (左侧):
 *   计算 △T = T1(柜温) - Ts(设定温度)
 *   △T ≥ △Tmax       → 最快速升频 (柜温远高于设定, 急需制冷)
 *   △Tmin ≤ △T < △Tmax → 比例升频 (柜温偏高, 按比例加大制冷)
 *   |△T| ≤ △Tmin(C1)   → 维持 (已在回差带内, 不调)
 *   其余(△T为负)       → 比例降频 (柜温偏低, 减小制冷)
 *   降频后若△T不缩小且F≤Fmin → ETM报警停机
 *
 * 膨胀阀调整模块原理 (右侧):
 *   传热温差 △TCZ = 柜温 - 蒸发温度
 *   △TCZ 不在 6.5±1.5 范围内 → Kp按α1·△TCZ调整
 *   △TCZ 在范围内 → 检查过热度△TP
 *     △TP < △TPmin(6.5~8) → EDT报警, Kp按α2·△TP调整
 *     △TP ≥ △TPmin         → 清除EDT, 正常
 *
 * 调用周期: PID每30秒执行一次 (由 ST_TMR_PID_DONE 触发)
 *
 * 后续扩展: 作为6大逻辑块之一, 独立FreeRTOS任务
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_FreqExv_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void FreqExv_PidAdjust(void);     /* PID调整模块 (左侧)   */
void FreqExv_ExvAdjust(void);     /* 膨胀阀调整模块 (右侧) */

#endif /* TASK_FREQ_EXV_H */
