#include "sys_state.h"
#include <string.h>

/* 全局静态传感器数据（系统数据仓库） */
static SysVarData_t g_SensorData = {0};

/* 全局变量定义 */
EventGroupHandle_t SysEventGroup      = NULL;
EventGroupHandle_t SysTimerEventGroup  = NULL;
SemaphoreHandle_t  SensorDataMutex     = NULL;

volatile uint32_t  g_AlarmFlags = 0;         /* 报警标志字, 初始无报警 */
SysTimerData_t     g_TimerData  = {0};       /* 定时计数器, 全部归零   */

/* 系统状态初始化函数 */
void SysState_Init(void) {
    if (SensorDataMutex == NULL) {
        SensorDataMutex = xSemaphoreCreateMutex();
    }
    if (SysEventGroup == NULL) {
        SysEventGroup = xEventGroupCreate();
    }
    if (SysTimerEventGroup == NULL) {
        SysTimerEventGroup = xEventGroupCreate();
    }
    g_AlarmFlags = 0;
    memset(&g_TimerData, 0, sizeof(SysTimerData_t));
}

/* 安全写入传感器数据 */
void SysState_UpdateSensor(SysVarData_t* newData) {
    if (SensorDataMutex != NULL && newData != NULL) {
        if (xSemaphoreTake(SensorDataMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&g_SensorData, newData, sizeof(SysVarData_t));
            xSemaphoreGive(SensorDataMutex);
        }
    }
}

/* 安全读取传感器数据 */
void SysState_GetSensor(SysVarData_t* outData) {
    if (SensorDataMutex != NULL && outData != NULL) {
        if (xSemaphoreTake(SensorDataMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(outData, &g_SensorData, sizeof(SysVarData_t));
            xSemaphoreGive(SensorDataMutex);
        }
    }
}
/* =========================================
 * 原子级安全操作接口实现
 * ========================================= */
void SysState_Lock(void) {
    if (SensorDataMutex != NULL) {
        xSemaphoreTake(SensorDataMutex, portMAX_DELAY);
    }
}

void SysState_Unlock(void) {
    if (SensorDataMutex != NULL) {
        xSemaphoreGive(SensorDataMutex);
    }
}

SysVarData_t* SysState_GetRawPtr(void) {
    return &g_SensorData;
}



