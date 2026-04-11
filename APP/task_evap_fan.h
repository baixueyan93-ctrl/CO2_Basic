#ifndef TASK_EVAP_FAN_H
#define TASK_EVAP_FAN_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 蒸发风机(1控6)流程 (逻辑图3) — FreeRTOS 任务
 *
 * 包含子逻辑:
 *   1. EvapFan_ModeF1()        F1=1 模式: 跟随压缩机运行, 除霜时关闭
 *   2. EvapFan_ModeF2()        F1=2 模式: 常开, 仅除霜时关闭
 *   3. EvapFan_ModeF4()        F1=4 模式: 根据蒸发温度合格标记开关
 *
 * 风机模式 (SET_EVAP_FAN_MODE = F1):
 *   F1=1  压缩机运行时风机开, 压缩机停时风机关, 除霜时风机关
 *   F1=2  风机常开(有F3延时), 仅除霜时关闭
 *   F1=4  蒸发温度合格时风机开, 不合格时风机关
 *
 * 后续扩展: 作为6大逻辑块之一, 独立FreeRTOS任务
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_EvapFan_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void EvapFan_ModeF1(void);     /* F1=1: 跟随压缩机   */
void EvapFan_ModeF2(void);     /* F1=2: 常开模式      */
void EvapFan_ModeF4(void);     /* F1=4: 温度合格模式  */

/* ===========================================================================
 * 硬件接口说明
 *
 * 蒸发风机: 通过 BSP_Relay_On/Off(RELAY_EVAP_FAN) 控制 (K1, PC9, 1控2)
 * =========================================================================== */

#endif /* TASK_EVAP_FAN_H */
