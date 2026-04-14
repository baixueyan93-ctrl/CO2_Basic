#include "bsp_exv.h"
#include "FreeRTOS.h"
#include "task.h"

/* ===================================================================
 *  三花 DPF(R04)1.5D-07 电子膨胀阀驱动
 *
 *  驱动方式: 单极4相8拍 (1-2相励磁, 半步进)
 *
 *  四相 = A, B, A-, B- (四个半绕组)
 *  八拍 = 单相与双相交替, 8拍为一个完整周期:
 *
 *    拍号  通电绕组       类型
 *    ───────────────────────────
 *     1    A              单相    ← 第1相
 *     2    A + B          双相    ← 过渡
 *     3    B              单相    ← 第2相
 *     4    B + A-         双相    ← 过渡
 *     5    A-             单相    ← 第3相
 *     6    A- + B-        双相    ← 过渡
 *     7    B-             单相    ← 第4相
 *     8    B- + A         双相    ← 过渡 → 回到第1拍
 *
 *  引脚映射 (V13原理图):
 *    PM0A(PD11)=B-,  PM0B(PD10)=A-,  PM0C(PD9)=B,  PM0D(PD8)=A
 *
 *  硬件极性: GPIO LOW = 线圈通电, GPIO HIGH = 线圈断电
 *  (MCU GPIO → TLP291-4 光耦 → ULN2803A, 与继电器电路极性相同)
 * =================================================================== */

/* 当前位置 (0 = 全关, EXV_TOTAL_STEPS = 全开) */
static volatile uint16_t s_exv_position = 0;

/* 当前相序索引 (0~7) */
static volatile uint8_t s_phase_index = 0;

/* 四相八拍相序表 (1-2相励磁, 半步进)
 *
 * 每个元素: 1=通电, 0=断电
 * 排列: {PM0A(B-), PM0B(A-), PM0C(B), PM0D(A)}
 *
 * 标准四相八拍: A → A+B → B → B+A- → A- → A-+B- → B- → B-+A → 循环
 */
static const uint8_t PHASE_TABLE[8][4] = {
    /* B-    A-    B     A      拍号  通电绕组       */
    {  0,    0,    0,    1  },  /* 1:  A       (单相) */
    {  0,    0,    1,    1  },  /* 2:  A + B   (双相) */
    {  0,    0,    1,    0  },  /* 3:  B       (单相) */
    {  0,    1,    1,    0  },  /* 4:  B + A-  (双相) */
    {  0,    1,    0,    0  },  /* 5:  A-      (单相) */
    {  1,    1,    0,    0  },  /* 6:  A- + B- (双相) */
    {  1,    0,    0,    0  },  /* 7:  B-      (单相) */
    {  1,    0,    0,    1  },  /* 8:  B- + A  (双相) */
};

/* ------------------------------------------------------------------
 * exv_set_phase - 输出指定相序到 GPIO
 * 硬件极性: 表中1(通电) → GPIO HIGH,  表中0(断电) → GPIO LOW
 * ------------------------------------------------------------------ */
static void exv_set_phase(uint8_t phase_idx)
{
    const uint8_t *ph = PHASE_TABLE[phase_idx & 0x07];

    HAL_GPIO_WritePin(EXV0_PM0A_PORT, EXV0_PM0A_PIN,
                      ph[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0B_PORT, EXV0_PM0B_PIN,
                      ph[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0C_PORT, EXV0_PM0C_PIN,
                      ph[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0D_PORT, EXV0_PM0D_PIN,
                      ph[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===================================================================
 *  BSP_EXV_Init - 初始化 GPIO 引脚 (推挽输出, 默认断电)
 * =================================================================== */
void BSP_EXV_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 所有引脚先拉低 (断电: GPIO LOW = 线圈断电) */
    HAL_GPIO_WritePin(GPIOD,
        EXV0_PM0A_PIN | EXV0_PM0B_PIN | EXV0_PM0C_PIN | EXV0_PM0D_PIN,
        GPIO_PIN_RESET);

    /* 配置 PD8, PD9, PD10, PD11 为推挽输出 */
    GPIO_InitStruct.Pin   = EXV0_PM0A_PIN | EXV0_PM0B_PIN |
                            EXV0_PM0C_PIN | EXV0_PM0D_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    s_exv_position = 0;
    s_phase_index  = 0;
}

/* ===================================================================
 *  BSP_EXV_DeEnergize - 断电所有线圈 (省电, 减少发热)
 *
 *  调用前应保持结束励磁至少 EXV_END_EXCITE_MS (0.2s),
 *  等待阀芯自保持机构锁定后再断电
 * =================================================================== */
void BSP_EXV_DeEnergize(void)
{
    /* GPIO LOW = 线圈断电 (光耦导通拉低 ULN2803A 输入, 通道截止) */
    HAL_GPIO_WritePin(GPIOD,
        EXV0_PM0A_PIN | EXV0_PM0B_PIN | EXV0_PM0C_PIN | EXV0_PM0D_PIN,
        GPIO_PIN_RESET);
}

/* ===================================================================
 *  BSP_EXV_Step - 执行指定步数 (半步进)
 *  @param dir      : EXV_DIR_OPEN / EXV_DIR_CLOSE
 *  @param steps    : 步数 (半步)
 *  @param delay_ms : 每步延时 (ms)
 * =================================================================== */
void BSP_EXV_Step(EXV_Direction_t dir, uint16_t steps, uint16_t delay_ms)
{
    for (uint16_t i = 0; i < steps; i++) {
        if (dir == EXV_DIR_OPEN) {
            s_phase_index = (s_phase_index + 1) & 0x07;
            if (s_exv_position < EXV_TOTAL_STEPS)
                s_exv_position++;
        } else {
            s_phase_index = (s_phase_index + 7) & 0x07; /* -1 mod 8 */
            if (s_exv_position > 0)
                s_exv_position--;
        }

        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/* ===================================================================
 *  BSP_EXV_GetPosition - 获取当前位置 (步数)
 * =================================================================== */
uint16_t BSP_EXV_GetPosition(void)
{
    return s_exv_position;
}

/* ===================================================================
 *  BSP_EXV_SetPosition - 移动到目标位置
 * =================================================================== */
void BSP_EXV_SetPosition(uint16_t target_steps, uint16_t delay_ms)
{
    if (target_steps > EXV_TOTAL_STEPS)
        target_steps = EXV_TOTAL_STEPS;

    if (target_steps > s_exv_position) {
        BSP_EXV_Step(EXV_DIR_OPEN,
                     target_steps - s_exv_position, delay_ms);
    } else if (target_steps < s_exv_position) {
        BSP_EXV_Step(EXV_DIR_CLOSE,
                     s_exv_position - target_steps, delay_ms);
    }
}

/* ===================================================================
 *  BSP_EXV_ResetToZero - 无记忆冷启动复位 (560步)
 *
 *  场景: 开机无记忆 或 首次上电
 *  三花规格: 步数 = 全开步数 * 112% = 560步
 *  注意: 应避免频繁调用, 防止阀针与阀口磨损
 * =================================================================== */
void BSP_EXV_ResetToZero(void)
{
    for (uint16_t i = 0; i < EXV_COLD_RESET_STEPS; i++) {
        s_phase_index = (s_phase_index + 7) & 0x07;
        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(EXV_STEP_DELAY_MS));
    }
    s_exv_position = 0;

    /* 结束励磁保持, 等待自保持机构锁定后断电 */
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
    BSP_EXV_DeEnergize();
}

/* ===================================================================
 *  BSP_EXV_ResetNormal - 正常停机复位 (当前开度 + 8步)
 *
 *  场景: 机组正常运行停机, 记忆当前开度
 *  三花规格: 关阀步数 = 当前开度 + 8步
 * =================================================================== */
void BSP_EXV_ResetNormal(void)
{
    uint16_t close_steps = s_exv_position + EXV_NORMAL_EXTRA_STEPS;

    for (uint16_t i = 0; i < close_steps; i++) {
        s_phase_index = (s_phase_index + 7) & 0x07;
        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(EXV_STEP_DELAY_MS));
    }
    s_exv_position = 0;

    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
    BSP_EXV_DeEnergize();
}

/* ===================================================================
 *  BSP_EXV_ResetFromMemory - 有记忆重启复位 (记忆开度 + 60步)
 *
 *  场景: 开机有上次保存的开度值 (从 EEPROM 读取)
 *  三花规格: 关阀步数 = 记忆开度 + 60步
 *  @param saved_pos : 上次保存的开度 (步数)
 * =================================================================== */
void BSP_EXV_ResetFromMemory(uint16_t saved_pos)
{
    uint16_t close_steps = saved_pos + EXV_MEMORY_EXTRA_STEPS;

    for (uint16_t i = 0; i < close_steps; i++) {
        s_phase_index = (s_phase_index + 7) & 0x07;
        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(EXV_STEP_DELAY_MS));
    }
    s_exv_position = 0;

    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));
    BSP_EXV_DeEnergize();
}
