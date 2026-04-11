// bsp_rs485.h
// RS485 调试串口 (USART1, PA9=TX, PA10=RX)
#ifndef __BSP_RS485_H
#define __BSP_RS485_H
#include "main.h"

extern UART_HandleTypeDef huart1;

void BSP_RS485_Init(void);
void BSP_RS485_SendString(char *str);

#endif
