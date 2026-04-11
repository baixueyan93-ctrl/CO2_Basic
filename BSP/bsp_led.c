#include "bsp_led.h"

// 依据 V13 原理图定义的硬件管脚
#define LED0_PORT GPIOE
#define LED0_PIN  GPIO_PIN_6

#define LED1_PORT GPIOC
#define LED1_PIN  GPIO_PIN_13

void BSP_LED_Init(void) {
    // 强制初始状态为熄灭 (输出低电平)
    HAL_GPIO_WritePin(LED0_PORT, LED0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
}

void BSP_LED0_Toggle(void) {
    HAL_GPIO_TogglePin(LED0_PORT, LED0_PIN); 
}

void BSP_LED1_Toggle(void) {
    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN); 
}



