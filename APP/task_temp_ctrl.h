#ifndef TASK_TEMP_CTRL_H
#define TASK_TEMP_CTRL_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 温度控制流程 (逻辑图1) — FreeRTOS 任务
 *
 * 包含子逻辑 (来自逻辑图1的5个子流程图):
 *   1. 停机异常逻辑告警        TempCtrl_ShutdownAlarm()
 *   2. 压缩机开机逻辑          TempCtrl_CompressorStart()
 *   3. 油壳加热逻辑            TempCtrl_OilHeatControl()
 *   4. 告警(温度压力)处理逻辑  TempCtrl_AlarmProcess()
 *   5. 温度逻辑(主逻辑)        TempCtrl_MainLogic()
 *
 * 后续扩展: 6大逻辑块各为一个FreeRTOS任务
 *   任务1: 温度控制流程  (本文件)
 *   任务2: 除霜流程
 *   任务3: 风机流程
 *   任务4: 变频控制和膨胀阀流程
 *   任务5: 定时中断服务流程
 *   任务6: 冷凝风机流程
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_TempCtrl_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void TempCtrl_ShutdownAlarm(void);     /* 停机异常逻辑告警           */
void TempCtrl_CompressorStart(void);   /* 压缩机开机逻辑             */
void TempCtrl_OilHeatControl(void);    /* 油壳加热逻辑               */
void TempCtrl_AlarmProcess(void);      /* 告警(温度压力)处理逻辑     */
void TempCtrl_MainLogic(void);         /* 温度逻辑(主逻辑)           */

/* ===========================================================================
 * 硬件接口说明
 *
 * 油壳加热器: BSP_Relay_On/Off(RELAY_OIL_HEATER) — K2继电器 PC6
 * 蒸发风扇:   BSP_Relay_On/Off(RELAY_EVAP_FAN)   — K1继电器 PC9
 * 冷凝风扇:   BSP_Relay_On/Off(RELAY_COND_FAN)   — K5继电器 PC8
 * VDC电压:    通过 task_adc 采集, 本模块从 VAR_VDC_VOLTAGE 读取
 * 压缩机:     变频器通信接口 (待实现)
 * =========================================================================== */

#endif /* TASK_TEMP_CTRL_H */
