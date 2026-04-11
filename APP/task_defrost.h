#ifndef TASK_DEFROST_H
#define TASK_DEFROST_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 除霜流程 (逻辑图2) — FreeRTOS 任务
 *
 * 包含子逻辑:
 *   1. 除霜主程序      Defrost_MainProcess()
 *   2. 加热子程序      Defrost_HeatSubroutine()
 *
 * 除霜三阶段状态机:
 *   [待机] → 间隔到时 → [加热] → 超时/温度达标 → [滴水] → 超时 → [待机]
 *
 * 后续扩展: 作为6大逻辑块之一, 独立FreeRTOS任务
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_Defrost_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void Defrost_MainProcess(void);        /* 除霜主程序   */
void Defrost_HeatSubroutine(void);     /* 加热子程序   */

/* ===========================================================================
 * 硬件接口说明
 *
 * 除霜加热器: 通过 BSP_Relay_On/Off(RELAY_DEF_HEATER) 控制 (K7, PC7)
 * 膨胀阀:     通过 BSP_EXV_SetPosition() 步进到除霜全开位置
 * =========================================================================== */

#endif /* TASK_DEFROST_H */
