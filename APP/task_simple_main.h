#ifndef TASK_SIMPLE_MAIN_H
#define TASK_SIMPLE_MAIN_H

#include <stdint.h>

/* ===========================================================================
 * 简化测试版主状态机任务 (task_simple_main)
 *
 * 替代以下任务的主逻辑:
 *   - task_temp_ctrl (逻辑图1)
 *   - task_defrost   (逻辑图2)
 *   - task_evap_fan  (逻辑图3)
 *   - task_freq_exv  (逻辑图4)
 *   - task_cond_fan  (逻辑图6)
 *
 * 仅保留: 显示/按键/ADC/SHT30/LED/RS485 调试串口 等外设任务
 *
 * 状态机:
 *   POWER_OFF → SELFTEST → RUN_LOW ↔ RUN_HIGH
 *                              ↓
 *                         DEFROST_RUN → DEFROST_DRIP → RUN_LOW
 *   任意状态按关机键 → POWER_OFF (3min cooldown)
 *
 * 对外全局变量:
 *   g_defrost_req : 除霜请求 (panel 按键按一次置 1,
 *                   simple main 消费后清零)
 * =========================================================================== */

extern volatile uint8_t g_defrost_req;

/* 任务入口 (由 freertos.c StartTask_SimpleMain 调用) */
void Task_SimpleMain_Process(void const *argument);

#endif /* TASK_SIMPLE_MAIN_H */
