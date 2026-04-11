#include "task_led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_led.h" // 引入BSP层底层武器 (用于调用 BSP_LED1_Toggle 等)

void Task_LED_Process(void const *argument) {
    uint32_t tick_count = 0; // 时间片计数器
    
    // 进入 RTOS 独立线程死循环
    for(;;) {
        tick_count++;
        
        // 【逻辑1】LED1 (PC13 系统心跳)：每 500ms 翻转一次 (50 * 10ms = 500ms)
        if(tick_count % 100 == 0) {
            BSP_LED1_Toggle(); 
        }
        
        // 【逻辑2】LED0 (PE6 报警快闪)：每 100ms 翻转一次 (10 * 10ms = 100ms)
        if(tick_count % 50 == 0) {
            BSP_LED0_Toggle(); 
        }
        
        // 【核心基准】：任务的基础心跳设置为 10ms，绝不会互相阻塞
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


