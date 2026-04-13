#include "bsp_htc_2k.h"
#include "FreeRTOS.h"
#include "task.h"

/* ================= 数码管段码表 ================= */
const uint8_t SmgTab[] =
{
    0xFC,/*0*/ 0x60,/*1*/ 0xDA,/*2*/ 0xF2,/*3*/ 0x66,/*4*/
    0xB6,/*5*/ 0xBE,/*6*/ 0xE0,/*7*/ 0xFE,/*8*/ 0xF6,/*9*/
    0xEE,/*A*/ 0x3E,/*b*/ 0x9C,/*C*/ 0x7A,/*d*/ 0x9E,/*E*/ 0x8E,/*F*/
    0x6E,/*H*/ 0x1C,/*L*/ 0x3A,/*c*/ 0xCE,/*P*/ 0x0A,/*r*/ 0x1E,/*t*/ 0x7C,/*U*/
    0x02,/*-*/ 0x00,/*空*/
};

/* ================= 全局图标变量 ================= */
icon_type_t g_IconSet  = {0};   /* PANEL0 */
icon_type_t g_IconSet1 = {0};   /* PANEL1 */

/* ================= 通用延时 ================= */
static void TM1637_DelayUs(volatile uint32_t us) {
    volatile uint32_t delay = (SystemCoreClock / 1000000 / 4) * us;
    while(delay--) { __NOP(); }
}

/* ===========================================================================
 * PANEL0 TM1637 底层协议 (PB6/PB7)
 * =========================================================================== */
static void TM0_Start(void) {
    HTC_CLK(1); HTC_DIO(1); TM1637_DelayUs(2);
    HTC_DIO(0); TM1637_DelayUs(2);
    HTC_CLK(0);
}

static void TM0_Stop(void) {
    HTC_CLK(0); TM1637_DelayUs(2);
    HTC_DIO(0); TM1637_DelayUs(2);
    HTC_CLK(1); TM1637_DelayUs(2);
    HTC_DIO(1);
}

static void TM0_Ask(void) {
    HTC_CLK(0); TM1637_DelayUs(5);
    HTC_CLK(1); TM1637_DelayUs(2);
    HTC_CLK(0);
}

static void TM0_WriteByte(uint8_t dat) {
    for (uint8_t i = 0; i < 8; i++) {
        HTC_CLK(0);
        if (dat & 0x01) HTC_DIO(1);
        else HTC_DIO(0);
        TM1637_DelayUs(3);
        dat >>= 1;
        HTC_CLK(1); TM1637_DelayUs(3);
    }
}

/* ===========================================================================
 * PANEL1 TM1637 底层协议 (PD0/PD1)
 * =========================================================================== */
static void TM1_Start(void) {
    HTC1_CLK(1); HTC1_DIO(1); TM1637_DelayUs(2);
    HTC1_DIO(0); TM1637_DelayUs(2);
    HTC1_CLK(0);
}

static void TM1_Stop(void) {
    HTC1_CLK(0); TM1637_DelayUs(2);
    HTC1_DIO(0); TM1637_DelayUs(2);
    HTC1_CLK(1); TM1637_DelayUs(2);
    HTC1_DIO(1);
}

static void TM1_Ask(void) {
    HTC1_CLK(0); TM1637_DelayUs(5);
    HTC1_CLK(1); TM1637_DelayUs(2);
    HTC1_CLK(0);
}

static void TM1_WriteByte(uint8_t dat) {
    for (uint8_t i = 0; i < 8; i++) {
        HTC1_CLK(0);
        if (dat & 0x01) HTC1_DIO(1);
        else HTC1_DIO(0);
        TM1637_DelayUs(3);
        dat >>= 1;
        HTC1_CLK(1); TM1637_DelayUs(3);
    }
}

/* ===========================================================================
 * 通用: 温度值 → 4字节显示数据
 * =========================================================================== */
static void TempToSegments(float temp, uint8_t icon_byte,
                           uint8_t *b0, uint8_t *b1, uint8_t *b2, uint8_t *b3)
{
    if (temp < -99.9f) temp = -99.9f;
    if (temp >  99.9f) temp =  99.9f;

    int val = (int)(temp * 10);
    if (val < 0) val = -val;

    uint8_t d1 = val / 100;
    uint8_t d2 = (val / 10) % 10;
    uint8_t d3 = val % 10;

    /* Byte 0 (0xC0): 图标 */
    *b0 = icon_byte;

    /* Byte 1 (0xC1): 百位 */
    *b1 = SmgTab[d1];
    if (*b1 == 0xFC && val < 100) *b1 = 0x00;

    /* Byte 2 (0xC2): 十位 + 小数点 (Bit0) */
    *b2 = SmgTab[d2] | 0x01;

    /* Byte 3 (0xC3): 个位 + 负号 (Bit0) */
    *b3 = SmgTab[d3];
    if (temp < 0) {
        *b3 |= 0x01;
    }
}

/* ===========================================================================
 * PANEL0 公共接口
 * =========================================================================== */

void HTC2K_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HTC_CLK_ENABLE();
    GPIO_InitStruct.Pin   = HTC_CLK_PIN | HTC_DIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HTC_CLK_PORT, &GPIO_InitStruct);
    HTC_CLK(1); HTC_DIO(1);
}

uint8_t HTC2K_ReadKeys(void) {
    uint8_t rekey = 0;
    TM0_Start();
    TM0_WriteByte(0x42);
    TM0_Ask();
    HTC_DIO(1);

    for (uint8_t i = 0; i < 8; i++) {
        HTC_CLK(0);
        rekey >>= 1;
        TM1637_DelayUs(10);
        HTC_CLK(1);
        if (HTC_READ_DIO()) rekey |= 0x80;
        TM1637_DelayUs(20);
    }
    TM0_Ask();
    TM0_Stop();
    return rekey;
}

void HTC2K_ShowTemp(float temp) {
    uint8_t b0, b1, b2, b3;
    TempToSegments(temp, g_IconSet.byte, &b0, &b1, &b2, &b3);

    TM0_Start();
    TM0_WriteByte(0x40);
    TM0_Ask();
    TM0_Stop();

    TM0_Start();
    TM0_WriteByte(0xC0);
    TM0_Ask();
    TM0_WriteByte(b0); TM0_Ask();
    TM0_WriteByte(b1); TM0_Ask();
    TM0_WriteByte(b2); TM0_Ask();
    TM0_WriteByte(b3); TM0_Ask();
    TM0_Stop();

    TM0_Start();
    TM0_WriteByte(0x8C);
    TM0_Ask();
    TM0_Stop();
}

/* ===========================================================================
 * PANEL1 公共接口
 * =========================================================================== */

void HTC2K_Init1(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HTC1_CLK_ENABLE();
    GPIO_InitStruct.Pin   = HTC1_CLK_PIN | HTC1_DIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HTC1_CLK_PORT, &GPIO_InitStruct);
    HTC1_CLK(1); HTC1_DIO(1);
}

uint8_t HTC2K_ReadKeys1(void) {
    uint8_t rekey = 0;
    TM1_Start();
    TM1_WriteByte(0x42);
    TM1_Ask();
    HTC1_DIO(1);

    for (uint8_t i = 0; i < 8; i++) {
        HTC1_CLK(0);
        rekey >>= 1;
        TM1637_DelayUs(10);
        HTC1_CLK(1);
        if (HTC1_READ_DIO()) rekey |= 0x80;
        TM1637_DelayUs(20);
    }
    TM1_Ask();
    TM1_Stop();
    return rekey;
}

void HTC2K_ShowTemp1(float temp) {
    uint8_t b0, b1, b2, b3;
    TempToSegments(temp, g_IconSet1.byte, &b0, &b1, &b2, &b3);

    TM1_Start();
    TM1_WriteByte(0x40);
    TM1_Ask();
    TM1_Stop();

    TM1_Start();
    TM1_WriteByte(0xC0);
    TM1_Ask();
    TM1_WriteByte(b0); TM1_Ask();
    TM1_WriteByte(b1); TM1_Ask();
    TM1_WriteByte(b2); TM1_Ask();
    TM1_WriteByte(b3); TM1_Ask();
    TM1_Stop();

    TM1_Start();
    TM1_WriteByte(0x8C);
    TM1_Ask();
    TM1_Stop();
}
