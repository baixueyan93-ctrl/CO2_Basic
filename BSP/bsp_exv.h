#ifndef BSP_EXV_H
#define BSP_EXV_H

#include "main.h"
#include <stdint.h>

/* ===================================================================
 *  三花 DPF(R04)1.5D-07 电子膨胀阀驱动 (DC12V, 单极4相8拍)
 *
 *  硬件接线 (V13 原理图 XH-5AW EXV0):
 *    PM0A (B-) = PD11   半绕组B-
 *    PM0B (A-) = PD10   半绕组A-
 *    PM0C (B)  = PD9    半绕组B
 *    PM0D (A)  = PD8    半绕组A
 *    COM       = +12V   (硬件直连, 软件不控制)
 *
 *  单极电机线圈结构:
 *    COM ─┬─ 半绕组A  (PMXD/PD8)
 *         ├─ 半绕组A- (PMXB/PD10)
 *         ├─ 半绕组B  (PMXC/PD9)
 *         └─ 半绕组B- (PMXA/PD11)
 *
 *  信号链: MCU GPIO -> TLP291-4 光耦 (U10) -> ULN2803A (U14) -> EXV0
 *  逻辑: GPIO HIGH = 线圈通电 (光耦截止, 上拉使 ULN2803A 导通)
 *         GPIO LOW  = 线圈断电 (光耦导通, 拉低 ULN2803A 输入, 截止)
 * =================================================================== */

/* --- 引脚定义 (与V13原理图一致) --- */
#define EXV0_PM0A_PORT   GPIOD          /* B-  PD11  半绕组B- */
#define EXV0_PM0A_PIN    GPIO_PIN_11
#define EXV0_PM0B_PORT   GPIOD          /* A-  PD10  半绕组A- */
#define EXV0_PM0B_PIN    GPIO_PIN_10
#define EXV0_PM0C_PORT   GPIOD          /* B   PD9   半绕组B  */
#define EXV0_PM0C_PIN    GPIO_PIN_9
#define EXV0_PM0D_PORT   GPIOD          /* A   PD8   半绕组A  */
#define EXV0_PM0D_PIN    GPIO_PIN_8

/* --- 步进参数 --- */
#define EXV_TOTAL_STEPS      500    /* 全行程步数 (半步)             */
#define EXV_STEP_DELAY_MS    20     /* 每步延时 (ms), 约50 PPS      */
#define EXV_END_EXCITE_MS    200    /* 结束励磁保持时间 (ms), 规格0.1~1.0s */

/* --- 复位参数 (三花规格) ---
 *  1) 正常停机 (有记忆): 当前开度 + 8步
 *  2) 有记忆重启:        记忆开度 + 60步
 *  3) 无记忆冷启动:      560步 (全开步数*112%)
 *  注意: 应避免频繁的全关复位(2,3), 防止阀针与阀口磨损
 */
#define EXV_COLD_RESET_STEPS    560   /* 无记忆冷启动复位步数          */
#define EXV_NORMAL_EXTRA_STEPS  8     /* 正常停机: 当前开度 + 8步      */
#define EXV_MEMORY_EXTRA_STEPS  60    /* 有记忆重启: 记忆开度 + 60步   */

/* --- 方向 --- */
typedef enum {
    EXV_DIR_CLOSE = 0,   /* 关阀 (CW)  */
    EXV_DIR_OPEN  = 1    /* 开阀 (CCW) */
} EXV_Direction_t;

/* --- 接口函数 --- */
void     BSP_EXV_Init(void);
void     BSP_EXV_DeEnergize(void);
void     BSP_EXV_Step(EXV_Direction_t dir, uint16_t steps, uint16_t delay_ms);
uint16_t BSP_EXV_GetPosition(void);
void     BSP_EXV_SetPosition(uint16_t target_steps, uint16_t delay_ms);

/* --- 三种复位模式 (三花规格) --- */
void     BSP_EXV_ResetToZero(void);                    /* 无记忆冷启动 (560步) */
void     BSP_EXV_ResetNormal(void);                    /* 正常停机 (当前+8步)  */
void     BSP_EXV_ResetFromMemory(uint16_t saved_pos);  /* 有记忆重启 (记忆+60步) */

#endif /* BSP_EXV_H */
