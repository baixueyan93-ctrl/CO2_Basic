#include "bsp_i2c_mutex.h"

static SemaphoreHandle_t I2C1_Mutex = NULL;

void BSP_I2C1_MutexInit(void) {
    if (I2C1_Mutex == NULL) {
        I2C1_Mutex = xSemaphoreCreateMutex();
    }
}

void BSP_I2C1_Lock(void) {
    if (I2C1_Mutex != NULL) {
        xSemaphoreTake(I2C1_Mutex, portMAX_DELAY);
    }
}

void BSP_I2C1_Unlock(void) {
    if (I2C1_Mutex != NULL) {
        xSemaphoreGive(I2C1_Mutex);
    }
}


