// bsp_rs485.c
// RS485 调试串口 (USART1, PA9=TX, PA10=RX, PA11=DIR)
// 使用 ISO1500DBQR 隔离 RS485 收发器 (U9, IOT-485)
// PA11 = RE4851: HIGH=发送, LOW=接收
#include "bsp_rs485.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/* RS485_1 方向控制引脚: PA11 (RE4851) */
#define RS485_1_DIR_PORT  GPIOA
#define RS485_1_DIR_PIN   GPIO_PIN_11

UART_HandleTypeDef huart1;

/* 互斥锁: 防止多任务同时发送调试信息 */
static SemaphoreHandle_t s_rs485_mutex = NULL;

static void RS485_1_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_SET);
}

static void RS485_1_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_RESET);
}

void BSP_RS485_Init(void) {
    /* 1. 初始化 PA11 为 GPIO 输出 (RS485_1 方向控制) */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef dir_gpio = {0};
    dir_gpio.Pin   = RS485_1_DIR_PIN;
    dir_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    dir_gpio.Pull  = GPIO_NOPULL;
    dir_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS485_1_DIR_PORT, &dir_gpio);

    /* 默认接收模式 */
    RS485_1_SetRx();

    /* 2. 初始化 USART1: PA9=TX, PA10=RX, 9600 8N1 */
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 创建互斥锁 */
    if (s_rs485_mutex == NULL) {
        s_rs485_mutex = xSemaphoreCreateMutex();
    }
}

void BSP_RS485_SendString(char *str) {
    /* 加锁: 防止多任务同时操作USART1 */
    if (s_rs485_mutex != NULL) {
        xSemaphoreTake(s_rs485_mutex, portMAX_DELAY);
    }

    RS485_1_SetTx();
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);

    /* 等待发送彻底完成再切RX, 防止最后一个字节被截断 */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}

    RS485_1_SetRx();

    if (s_rs485_mutex != NULL) {
        xSemaphoreGive(s_rs485_mutex);
    }
}
