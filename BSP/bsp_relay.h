#ifndef BSP_RELAY_H
#define BSP_RELAY_H

#include "main.h"

/* ============================================================
 * 6路继电器输出 — 引脚定义 (V13原理图)
 *
 * 驱动链路: MCU pin → TLP291-4光耦 → ULN2803A → 继电器线圈
 * 极性: GPIO HIGH = 继电器吸合(ON), GPIO LOW = 继电器断开(OFF)
 *
 *   功能          继电器   MCU引脚   GPIO
 *   ─────────────────────────────────────
 *   蒸发风扇       K1      PC9      GPIOC,PIN9
 *   冷凝风扇       K5      PC8      GPIOC,PIN8
 *   化霜热丝       K7      PC7      GPIOC,PIN7
 *   滑油热丝       K2      PC6      GPIOC,PIN6
 *   凝露热丝       K6      PD15     GPIOD,PIN15
 *   照明开关       K8      PD14     GPIOD,PIN14
 * ============================================================ */

/* --- 继电器编号枚举 --- */
typedef enum {
    RELAY_EVAP_FAN   = 0,   /* 蒸发风扇  K1  PC9  */
    RELAY_COND_FAN   = 1,   /* 冷凝风扇  K5  PC8  */
    RELAY_DEF_HEATER = 2,   /* 化霜热丝  K7  PC7  */
    RELAY_OIL_HEATER = 3,   /* 滑油热丝  K2  PC6  */
    RELAY_DEW_HEATER = 4,   /* 凝露热丝  K6  PD15 */
    RELAY_LIGHT      = 5,   /* 照明开关  K8  PD14 */
    RELAY_COUNT      = 6
} Relay_ID;

/* --- 公开 API --- */
void BSP_Relay_Init(void);                          /* GPIO初始化, 全部OFF */
void BSP_Relay_On(Relay_ID id);                     /* 吸合(导通) */
void BSP_Relay_Off(Relay_ID id);                    /* 断开 */
void BSP_Relay_Set(Relay_ID id, uint8_t on);        /* on=1吸合, on=0断开 */
uint8_t BSP_Relay_GetState(Relay_ID id);            /* 返回1=ON, 0=OFF */
void BSP_Relay_AllOff(void);                        /* 全部断开 */
const char* BSP_Relay_Name(Relay_ID id);            /* 返回继电器名称字符串 */

#endif /* BSP_RELAY_H */
