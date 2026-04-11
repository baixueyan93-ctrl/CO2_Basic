#include "task_sht30.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys_state.h"

/**
 * @brief SHT30 ������ʪ�Ȳɼ�����
 *        ����: 1��ɼ�һ��, д��ϵͳȫ�����ݽṹ
 */
void Task_SHT30_Process(void const *argument) {
    vTaskDelay(pdMS_TO_TICKS(500));

    while (!BSP_SHT30_Init()) {
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    SHT30_Result_t sht30_data;

    for (;;) {
        if (BSP_SHT30_Read(&sht30_data)) {
            // ���´��룺ʹ��ԭ����ֱ���޸ľ����ֶΣ����԰�ȫ��
            SysState_Lock();
            SysState_GetRawPtr()->VAR_SHT30_TEMP = sht30_data.temperature;
            SysState_GetRawPtr()->VAR_SHT30_HUMI = sht30_data.humidity;
            SysState_GetRawPtr()->VAR_AMBIENT_TEMP = sht30_data.temperature; /* 环境温度 = SHT30 */
            SysState_Unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



