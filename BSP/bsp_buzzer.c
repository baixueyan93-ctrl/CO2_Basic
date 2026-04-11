#include "bsp_buzzer.h"

// 蜂鸣器控制：PE5低电平导通。输入1响，输入0灭
void BSP_Buzzer_Set(uint8_t state) {
    if(state == 1) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // 拉低响
    } else {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   // 拉高灭
    }
}



