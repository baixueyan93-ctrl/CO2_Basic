#ifndef __TASK_PANEL_H
#define __TASK_PANEL_H

#include <stdbool.h>

/* ===========================================================================
 * 面板总任务 — 一个任务同时驱动 PANEL0 + PANEL1
 *
 * PANEL0: 纯显示面板 (PB6/PB7, 无按键)
 *   - SHT-30 温度显示
 *   - 除霜/满水/风机/报警图标
 *
 * PANEL1: 操作面板 (PB4/PB5, 8按键 + 显示)
 *   - 正常模式: 显示 SHT-30 环境温度
 *   - 设置模式: 显示/调节目标温度
 *   - 8按键: Reset/Set/Up/Down/Defrost/Light/Inspect/Power
 * =========================================================================== */

extern float         g_set_temp;    /* 面板可调的设定温度 (运行时Ts)     */
extern volatile bool g_system_on;   /* 系统电源状态 (POWER 键切换)       */

void Task_Panel_Process(void const *argument);

#endif
