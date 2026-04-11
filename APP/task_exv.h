#ifndef __TASK_EXV_H
#define __TASK_EXV_H

/* ===========================================================================
 * task_exv — 电子膨胀阀 (EXV) 发热调试任务
 *
 * 目的: 在 sys_config.h 中把 EXV_DEBUG_ONLY 置为 1 后, freertos.c 只创建
 *       本任务 (以及 LED/RS485/Panel/ADC/SHT30 等外设任务), 不创建主状态机.
 *       本任务反复对膨胀阀做 "开 — 保持 — 关 — 保持" 的循环,
 *       每次动作完成后立即调用 BSP_EXV_DeEnergize() 断电线圈,
 *       并在 RS485 调试串口 (USART1 9600) 打印 GPIO 电平 + 倒计时,
 *       方便工程师在 "HOLD" 阶段用手触摸阀体, 判断线圈是否还发热.
 * =========================================================================== */

void Task_EXV_Process(void const *argument);

#endif
