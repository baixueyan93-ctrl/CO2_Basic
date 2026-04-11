#ifndef __BSP_SHT30_H
#define __BSP_SHT30_H

#include "main.h"
#include <stdbool.h>

/* =====================================================
 * SHT30 温湿度传感器驱动 (I2C1, 与 EEPROM 共用总线)
 * ===================================================== */

/* SHT30 I2C 地址 */
#define SHT30_ADDRESS       (0x44 << 1)  /* 0x88 */

/* SHT30 双字节命令 */
#define SHT30_CMD_MEAS_H_MSB   0x2C  /* 高精度单次测量 (Clock Stretching) */
#define SHT30_CMD_MEAS_H_LSB   0x06
#define SHT30_CMD_RESET_MSB     0x30  /* 软复位 */
#define SHT30_CMD_RESET_LSB     0xA2

/* 测量结果结构体 */
typedef struct {
    float temperature;  /* 温度 (°C) */
    float humidity;     /* 相对湿度 (% RH) */
} SHT30_Result_t;

/* 接口函数 */
bool    BSP_SHT30_Init(void);
bool    BSP_SHT30_Read(SHT30_Result_t *result);
bool    BSP_SHT30_SoftReset(void);
uint8_t BSP_SHT30_CRC(uint8_t *data, uint8_t len);

#endif /* __BSP_SHT30_H */



