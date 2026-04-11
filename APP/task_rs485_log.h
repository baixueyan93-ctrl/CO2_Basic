// task_rs485_log.h
#ifndef __TASK_RS485_LOG_H
#define __TASK_RS485_LOG_H
#include <stdint.h>
void Task_RS485Log_Process(void const *argument);
uint8_t System_Record_Fault(uint8_t fault_code);
#endif


