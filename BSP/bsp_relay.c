#include "bsp_relay.h"

/* ============================================================
 * 继电器 GPIO 映射表
 *
 * 极性: LOW = ON (吸合),  HIGH = OFF (断开)
 * ============================================================ */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} Relay_Map_t;

static const Relay_Map_t relay_map[RELAY_COUNT] = {
    [RELAY_EVAP_FAN]   = { GPIOC, GPIO_PIN_9,  "EvapFan"  },   /* K1  PC9 蒸发风扇 */
    [RELAY_COND_FAN]   = { GPIOC, GPIO_PIN_8,  "CondFan"  },   /* K5  PC8 冷凝风扇 */
    [RELAY_DEF_HEATER] = { GPIOC, GPIO_PIN_7,  "DefHtr"   },   /* K7  PC7 化霜热丝 */
    [RELAY_OIL_HEATER] = { GPIOC, GPIO_PIN_6,  "OilHtr"   },   /* K2  PC6 滑油热丝 */
    [RELAY_DEW_HEATER] = { GPIOD, GPIO_PIN_15, "DewHtr"   },   /* K6  PD15 凝露热丝 */
    [RELAY_LIGHT]      = { GPIOD, GPIO_PIN_14, "Light"    },   /* K8  PD14 照明开关 */
};

/* ============================================================
 * BSP_Relay_Init — 初始化6路继电器GPIO, 上电默认全部OFF(高电平)
 * ============================================================ */
void BSP_Relay_Init(void)
{
    /* 确保时钟已使能 (gpio.c MX_GPIO_Init 中已 enable GPIOC/GPIOD) */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* 先把所有引脚拉高 (OFF), 再配置为输出 — 防止上电瞬间误动作 */
    for (int i = 0; i < RELAY_COUNT; i++) {
        HAL_GPIO_WritePin(relay_map[i].port, relay_map[i].pin, GPIO_PIN_SET);
    }

    /* PC6, PC7, PC8, PC9 */
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PD14, PD15 */
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
}

/* ============================================================
 * 基本控制
 * ============================================================ */
void BSP_Relay_On(Relay_ID id)
{
    if (id >= RELAY_COUNT) return;
    HAL_GPIO_WritePin(relay_map[id].port, relay_map[id].pin, GPIO_PIN_RESET); /* LOW = ON */
}

void BSP_Relay_Off(Relay_ID id)
{
    if (id >= RELAY_COUNT) return;
    HAL_GPIO_WritePin(relay_map[id].port, relay_map[id].pin, GPIO_PIN_SET);   /* HIGH = OFF */
}

void BSP_Relay_Set(Relay_ID id, uint8_t on)
{
    if (on) BSP_Relay_On(id);
    else    BSP_Relay_Off(id);
}

uint8_t BSP_Relay_GetState(Relay_ID id)
{
    if (id >= RELAY_COUNT) return 0;
    /* LOW = ON → 返回1;  HIGH = OFF → 返回0 */
    return (HAL_GPIO_ReadPin(relay_map[id].port, relay_map[id].pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void BSP_Relay_AllOff(void)
{
    for (int i = 0; i < RELAY_COUNT; i++) {
        BSP_Relay_Off((Relay_ID)i);
    }
}

const char* BSP_Relay_Name(Relay_ID id)
{
    if (id >= RELAY_COUNT) return "???";
    return relay_map[id].name;
}
