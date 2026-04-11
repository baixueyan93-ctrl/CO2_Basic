#include "task_exv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_exv.h"

/* ===================================================================
 *  电子膨胀阀 (三花 DPF(R04)1.5D-07 DC12V) 测试任务
 *
 *  测试流程:
 *    1. 初始化 GPIO
 *    2. 复位 (关阀560步, 确保机械归位)
 *    3. 开阀到 250 步 (50% 开度)
 *    4. 断电等待 3 秒
 *    5. 开阀到 500 步 (100% 开度)
 *    6. 断电等待 3 秒
 *    7. 关阀到 100 步 (20% 开度)
 *    8. 断电等待 3 秒
 *    9. 复位 (全关)
 *   10. 循环重复
 *
 *  注意: 此任务在正式代码中未被创建 (Task_FreqExv 负责 EXV 控制)
 * =================================================================== */

void Task_EXV_Process(void const *argument)
{
    (void)argument;

    /* 1. 初始化 EXV GPIO */
    BSP_EXV_Init();

    /* 2. 首次复位, 确保阀门处于全关位置 */
    BSP_EXV_ResetToZero();
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        /* --- 测试1: 开阀到 50% (250步) --- */
        BSP_EXV_SetPosition(250, EXV_STEP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
        BSP_EXV_DeEnergize();
        vTaskDelay(pdMS_TO_TICKS(3000));

        /* --- 测试2: 继续开到 100% (500步) --- */
        BSP_EXV_SetPosition(500, EXV_STEP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
        BSP_EXV_DeEnergize();
        vTaskDelay(pdMS_TO_TICKS(3000));

        /* --- 测试3: 关阀到 20% (100步) --- */
        BSP_EXV_SetPosition(100, EXV_STEP_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
        BSP_EXV_DeEnergize();
        vTaskDelay(pdMS_TO_TICKS(3000));

        /* --- 测试4: 全关复位 --- */
        BSP_EXV_ResetToZero();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
