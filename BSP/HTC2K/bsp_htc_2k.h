#ifndef __BSP_HTC_2K_H
#define __BSP_HTC_2K_H

#include "main.h"

/* ===========================================================================
 * TM1637 双面板驱动 (PANEL0 + PANEL1)
 *
 * PANEL0: PB6(CLK) / PB7(DIO) — 纯显示面板 (温度+图标)
 * PANEL1: PB4(PCLK1) / PB5(PDIO1) — 操作面板 (温度+图标+8按键)
 *
 * 注意: PANEL1 引脚定义需与实际原理图一致, 如有差异请修改下方宏
 * =========================================================================== */

/* ================= 1. PANEL0 硬件引脚 (原有, PB6/PB7) ================= */
#define HTC_CLK_PORT        GPIOB
#define HTC_CLK_PIN         GPIO_PIN_6
#define HTC_DIO_PORT        GPIOB
#define HTC_DIO_PIN         GPIO_PIN_7
#define HTC_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

/* ================= 2. PANEL1 硬件引脚 (PB4/PB5) ================= */
#define HTC1_CLK_PORT       GPIOB
#define HTC1_CLK_PIN        GPIO_PIN_4
#define HTC1_DIO_PORT       GPIOB
#define HTC1_DIO_PIN        GPIO_PIN_5
#define HTC1_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

/* ================= 3. PANEL0 GPIO 操作宏 ================= */
#define HTC_CLK(x)          HAL_GPIO_WritePin(HTC_CLK_PORT, HTC_CLK_PIN, (x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define HTC_DIO(x)          HAL_GPIO_WritePin(HTC_DIO_PORT, HTC_DIO_PIN, (x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define HTC_READ_DIO()      HAL_GPIO_ReadPin(HTC_DIO_PORT, HTC_DIO_PIN)

/* ================= 4. PANEL1 GPIO 操作宏 ================= */
#define HTC1_CLK(x)         HAL_GPIO_WritePin(HTC1_CLK_PORT, HTC1_CLK_PIN, (x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define HTC1_DIO(x)         HAL_GPIO_WritePin(HTC1_DIO_PORT, HTC1_DIO_PIN, (x)?GPIO_PIN_SET:GPIO_PIN_RESET)
#define HTC1_READ_DIO()     HAL_GPIO_ReadPin(HTC1_DIO_PORT, HTC1_DIO_PIN)

/* ================= 5. 图标控制结构体 ================= */
typedef struct
{
    uint8_t Clock   : 1; // Bit 0: 时钟
    uint8_t Light   : 1; // Bit 1: 灯光
    uint8_t Set     : 1; // Bit 2: 设置
    uint8_t Heat    : 1; // Bit 3: 加热
    uint8_t Fan     : 1; // Bit 4: 风扇
    uint8_t Def     : 1; // Bit 5: 除霜
    uint8_t Humi    : 1; // Bit 6: 湿度/满水
    uint8_t Ref     : 1; // Bit 7: 制冷
} icon_bits_t;

typedef union
{
    uint8_t      byte;
    icon_bits_t  bits;
} icon_type_t;

/* ================= 6. 字符地址常量 ================= */
#define ZM_FH         23  // 负号 (-)
#define ZM_NULL       24  // 空

/* ================= 7. 全局图标变量 ================= */
extern icon_type_t g_IconSet;     /* PANEL0 图标集 */
extern icon_type_t g_IconSet1;    /* PANEL1 图标集 */

/* ================= 8. PANEL1 按键码定义 (8键) =================
 *
 * TM1637 按键扫描: K1×SG1~SG4 + K2×SG1~SG4 = 8键
 * 以下编码基于典型 TM1637 按键矩阵接线,
 * 实际值需根据硬件按键矩阵走线确认, 如有偏差请修改.
 */
/* K1 线 (原有4键) */
#define KEY_CODE_RST        0xF7    /* 1. Reset 一键复位 */
#define KEY_CODE_DOWN       0xF6    /* 2. 调温下键 */
#define KEY_CODE_UP         0xF5    /* 3. 调温上键 */
#define KEY_CODE_SET        0xF4    /* 4. Set 设置目标温度 */
/* K2 线 (新增4键) */
#define KEY_CODE_DEFROST    0xEF    /* 5. 一键除霜 */
#define KEY_CODE_LIGHT      0xEE    /* 6. 照明键 */
#define KEY_CODE_INSPECT    0xED    /* 7. 点检键 (保留) */
#define KEY_CODE_POWER      0xEC    /* 8. 电源开关键 */

/* ================= 9. 函数声明 ================= */

/* --- PANEL0 (PB6/PB7) --- */
void    HTC2K_Init(void);
void    HTC2K_ShowTemp(float temp);
uint8_t HTC2K_ReadKeys(void);

/* --- PANEL1 (PB4/PB5) --- */
void    HTC2K_Init1(void);
void    HTC2K_ShowTemp1(float temp);
uint8_t HTC2K_ReadKeys1(void);

#endif
