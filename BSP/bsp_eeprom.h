#ifndef __BSP_EEPROM_H
#define __BSP_EEPROM_H
#include "main.h"
#include "sys_config.h"

/* 24C16 参数 */
#define EEPROM_BASE_ADDRESS  0xA0       /* I2C 基地址                     */
#define EEPROM_PAGE_WRITE    16         /* 24C16 页写最大 16 字节          */
#define EEPROM_TOTAL_SIZE    2048       /* 24C16 总容量 2KB               */

/* 底层读写接口 (自动处理 24C16 分页寻址 + 页写边界) */
void BSP_EEPROM_Write(uint16_t memAddress, uint8_t *pData, uint16_t size);
void BSP_EEPROM_Read(uint16_t memAddress, uint8_t *pData, uint16_t size);

/* 日志存储接口 */
void    BSP_Log_Init(void);
void    BSP_Log_Add(SysLog_t *new_log);
void    BSP_Log_Read_By_Index(uint8_t index, SysLog_t *out_log);
uint8_t BSP_Log_Get_Current_Index(void);

#endif




