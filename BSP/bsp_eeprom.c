#include "bsp_eeprom.h"
#include "bsp_i2c_mutex.h"
#include "FreeRTOS.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

/* =====================================================================
 * 24C16 寻址说明:
 *   总容量 2048 字节, 11 位地址 (0x000 ~ 0x7FF)
 *   高 3 位编入 I2C 设备地址: DevAddr = 0xA0 | ((memAddr >> 7) & 0x0E)
 *   低 8 位作为片内地址字节
 *   页写最大 16 字节, 跨 16 字节边界会回卷到页首!
 * ===================================================================== */

/**
 * @brief 计算 24C16 的 I2C 设备地址 (包含高 3 位页地址)
 */
static uint8_t EEPROM_DevAddr(uint16_t memAddr) {
    return EEPROM_BASE_ADDRESS | (uint8_t)((memAddr >> 7) & 0x0E);
}

/**
 * @brief 写入任意长度数据 (自动处理页写边界 + 分页寻址)
 */
void BSP_EEPROM_Write(uint16_t memAddress, uint8_t *pData, uint16_t size) {
    while (size > 0) {
        uint8_t  devAddr  = EEPROM_DevAddr(memAddress);
        uint8_t  wordAddr = (uint8_t)(memAddress & 0xFF);

        /* 本次最多写到当前 16 字节页的末尾 */
        uint16_t pageRemain = EEPROM_PAGE_WRITE - (memAddress % EEPROM_PAGE_WRITE);
        uint16_t writeLen   = (size < pageRemain) ? size : pageRemain;

        BSP_I2C1_Lock();
        HAL_I2C_Mem_Write(&hi2c1, devAddr, wordAddr,
                          I2C_MEMADD_SIZE_8BIT, pData, writeLen, 100);
        BSP_I2C1_Unlock();

        /* 24C16 写入周期最大 5ms */
        vTaskDelay(pdMS_TO_TICKS(5));

        memAddress += writeLen;
        pData      += writeLen;
        size       -= writeLen;
    }
}

/**
 * @brief 读取任意长度数据 (自动处理分页寻址)
 *        读操作无页边界限制, 但跨 256 字节需换设备地址
 */
void BSP_EEPROM_Read(uint16_t memAddress, uint8_t *pData, uint16_t size) {
    while (size > 0) {
        uint8_t  devAddr  = EEPROM_DevAddr(memAddress);
        uint8_t  wordAddr = (uint8_t)(memAddress & 0xFF);

        /* 本次最多读到当前 256 字节块的末尾 */
        uint16_t blockRemain = 256 - (memAddress & 0xFF);
        uint16_t readLen     = (size < blockRemain) ? size : blockRemain;

        BSP_I2C1_Lock();
        HAL_I2C_Mem_Read(&hi2c1, devAddr, wordAddr,
                         I2C_MEMADD_SIZE_8BIT, pData, readLen, 100);
        BSP_I2C1_Unlock();

        memAddress += readLen;
        pData      += readLen;
        size       -= readLen;
    }
}

/* ==========================================
 * 日志环形存储
 * ========================================== */

static uint8_t Current_Log_Index = 0;

void BSP_Log_Init(void) {
    BSP_EEPROM_Read(EEPROM_INDEX_ADDR, &Current_Log_Index, 1);

    if (Current_Log_Index >= LOG_MAX_COUNT) {
        Current_Log_Index = 0;
        BSP_EEPROM_Write(EEPROM_INDEX_ADDR, &Current_Log_Index, 1);
    }
}

void BSP_Log_Add(SysLog_t *new_log) {
    uint16_t physical_addr = EEPROM_DATA_START + (Current_Log_Index * LOG_SIZE);

    BSP_EEPROM_Write(physical_addr, (uint8_t *)new_log, LOG_SIZE);

    Current_Log_Index++;
    if (Current_Log_Index >= LOG_MAX_COUNT) {
        Current_Log_Index = 0;
    }

    BSP_EEPROM_Write(EEPROM_INDEX_ADDR, &Current_Log_Index, 1);
}

void BSP_Log_Read_By_Index(uint8_t index, SysLog_t *out_log) {
    if (index >= LOG_MAX_COUNT) return;

    uint16_t physical_addr = EEPROM_DATA_START + (index * LOG_SIZE);
    BSP_EEPROM_Read(physical_addr, (uint8_t *)out_log, LOG_SIZE);
}

uint8_t BSP_Log_Get_Current_Index(void) {
    return Current_Log_Index;
}



