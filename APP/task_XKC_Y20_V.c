#include "task_XKC_Y20_V.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys_state.h"
#include "task_rs485_log.h"
#include "main.h"
#include <stdio.h>
#include "bsp_rs485.h"

// 对应 Arduino 的 int liquidLevel = 0;
int liquidLevel = 0; 

void Task_XKC_Y20_V_Process(void const *argument) {
    vTaskDelay(pdMS_TO_TICKS(500)); 
    
    char msg[64];

    // =========================================================
    // 这里强行接管 PD7 硬件配置。
    // 因为 CubeMX 没配它，不确定是否被 LCD 占用，所以强行独立夺回控制权
    // =========================================================
    __HAL_RCC_GPIOD_CLK_ENABLE(); // 1. 强行开启 GPIOD 时钟，如果没有这一句，读出来可能是 0
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;       // 指定 PD7
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 强制设为输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // 强制设为无上下拉（不干涉）
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // 配置生效
    // =========================================================

    for(;;) {
        // 1. 读取液位
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET) {
            liquidLevel = 1; 
        } else {
            liquidLevel = 0; 
        }

        sprintf(msg, "liquidLevel= %d\r\n", liquidLevel);
        BSP_RS485_SendString(msg);

        // 3. 顺手更新到系统数据黑板（为了兼容其他 GET 指令）
        SysVarData_t sensor_data = {0};
        SysState_GetSensor(&sensor_data);
        sensor_data.VAR_LIQUID_LEVEL = liquidLevel;
        SysState_UpdateSensor(&sensor_data);

        // 4. 对应 delay(500);
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}



