#include "task_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_buzzer.h" // µ×²ãÇý¶¯

void Task_Buzzer_Process(void const *argument) {
    for(;;) {
        BSP_Buzzer_Set(1);             // Ïì
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms
        
        BSP_Buzzer_Set(0);             // Í£
        vTaskDelay(pdMS_TO_TICKS(2000));// 2000ms
    }
}


