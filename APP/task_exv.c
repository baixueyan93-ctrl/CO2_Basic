#include "task_exv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_exv.h"
#include "bsp_rs485.h"
#include "sys_config.h"
#include <stdio.h>

/* ===========================================================================
 *  电子膨胀阀 (三花 DPF(R04)1.5D-07 DC12V) — 发热调试任务
 *
 *  目的:
 *    只对电子膨胀阀做 "开 — 保持 — 关 — 保持" 的循环动作, 每次动作结束
 *    后立即断电线圈, 让工程师用手摸阀体线圈壳, 判断断电逻辑
 *    (BSP_EXV_DeEnergize) 是否真的把线圈关掉了. 避免其它任务/状态机干扰.
 *
 *  判断准则:
 *    - 动作期间 (阀门移动的几秒内): 某些相线圈导通, 线圈轻微发热是正常的.
 *    - 保持期间 (每次移动后 HOLD N 秒): 4 个 GPIO (PD8/9/10/11) 必须全部
 *      处于 HIGH 电平, 此时 ULN2803A 通道全部截止, 12V → 线圈 → GND 回路
 *      断开, 线圈电流 = 0, 线圈应该能迅速冷却. 如果 HOLD 期间摸上去仍然烫,
 *      说明:
 *        a) 代码层面: GPIO 没拉高 (BSP_EXV_DeEnergize 没被调用,
 *           或者被其它代码覆盖回 LOW).
 *        b) 硬件层面: ULN2803A / TLP291-4 光耦 漏电, 或 PCB 短路,
 *           或线圈 COM 与 GND 之间还有旁路.
 *
 *  测试流程 (无限循环):
 *    上电 → 等 2s 让调试串口就绪
 *        → 打印横幅
 *        → 560 步冷启动关零 (~11s)
 *        → HOLD N 秒 (用户触摸线圈: 应为冷)
 *        → 开到 250 步 (50%) → HOLD N 秒
 *        → 开到 500 步 (100%)→ HOLD N 秒
 *        → 关到 250 步 (50%) → HOLD N 秒
 *        → 关到   0 步 ( 0%) → HOLD N 秒
 *        → 从头循环
 *
 *  每次 HOLD 阶段每 10 秒打印一次 GPIO 电平, 确认断电状态没有被意外恢复.
 *
 *  配套常量 (sys_config.h):
 *    EXV_DEBUG_HOLD_SEC   — 每次动作后保持时间 (秒), 默认 30
 *    EXV_DEBUG_ONLY       — freertos.c 的编译开关
 *
 *  相关宏 (bsp_exv.h):
 *    EXV_TOTAL_STEPS      — 500 (全开步数)
 *    EXV_STEP_DELAY_MS    — 20  (每步延时)
 *    EXV_END_EXCITE_MS    — 200 (结束励磁保持时间, 让自保持机构锁定)
 *    EXV_COLD_RESET_STEPS — 560 (冷启动关零步数 = 112%)
 * =========================================================================== */

/* --- 内部: 输出一段文字到 RS485 调试口 --- */
static void exv_dbg_print(const char *msg)
{
    BSP_RS485_SendString((char *)msg);
}

/* --- 内部: 打印 4 个相线圈 GPIO 当前电平 + 状态解读 ---
 * 映射 (与 bsp_exv.h 一致):
 *   PD8  = PM0D (A)
 *   PD9  = PM0C (B)
 *   PD10 = PM0B (A-)
 *   PD11 = PM0A (B-)
 * 全部 HIGH = 全部断电 (线圈应冷却)
 */
static void exv_dbg_print_gpio(const char *label)
{
    char buf[112];
    uint8_t pd8  = (HAL_GPIO_ReadPin(GPIOD, EXV0_PM0D_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pd9  = (HAL_GPIO_ReadPin(GPIOD, EXV0_PM0C_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pd10 = (HAL_GPIO_ReadPin(GPIOD, EXV0_PM0B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pd11 = (HAL_GPIO_ReadPin(GPIOD, EXV0_PM0A_PIN) == GPIO_PIN_SET) ? 1 : 0;

    snprintf(buf, sizeof(buf),
             "[EXV-DBG] %s PD8=%u PD9=%u PD10=%u PD11=%u %s\r\n",
             label,
             (unsigned)pd8, (unsigned)pd9,
             (unsigned)pd10, (unsigned)pd11,
             (pd8 && pd9 && pd10 && pd11) ? "(all HIGH -> de-energized OK)"
                                          : "(some LOW -> STILL ENERGIZED!)");
    BSP_RS485_SendString(buf);
}

/* --- 内部: 保持阶段循环 (每 10s 打印一次 GPIO 状态) --- */
static void exv_dbg_hold(uint32_t total_sec)
{
    char buf[80];

    snprintf(buf, sizeof(buf),
             "[EXV-DBG] HOLD %lus  ** TOUCH COIL NOW — SHOULD BE COLD **\r\n",
             (unsigned long)total_sec);
    BSP_RS485_SendString(buf);

    uint32_t elapsed = 0;
    while (elapsed < total_sec) {
        uint32_t step = (total_sec - elapsed) >= 10U ? 10U : (total_sec - elapsed);
        vTaskDelay(pdMS_TO_TICKS(step * 1000U));
        elapsed += step;

        snprintf(buf, sizeof(buf),
                 "[EXV-DBG]   hold %2lu/%lus",
                 (unsigned long)elapsed, (unsigned long)total_sec);
        exv_dbg_print_gpio(buf);
    }
}

/* --- 内部: 移动到目标位置 → 结束励磁 → 断电 → 打印 → 保持 --- */
static void exv_dbg_move_and_hold(const char *action, uint16_t target)
{
    char buf[96];
    uint16_t before = BSP_EXV_GetPosition();

    /* 1. 打印动作意图 */
    snprintf(buf, sizeof(buf),
             "[EXV-DBG] >>>> %s : pos %u -> %u\r\n",
             action, (unsigned)before, (unsigned)target);
    BSP_RS485_SendString(buf);

    /* 2. 动作: 步进到目标位置 (期间线圈按相序轮流通电, 短暂发热属正常) */
    BSP_EXV_SetPosition(target, EXV_STEP_DELAY_MS);

    /* 3. 结束励磁保持 0.2s, 让自保持机构锁定 (三花规格 0.1~1.0s) */
    vTaskDelay(pdMS_TO_TICKS(EXV_END_EXCITE_MS));

    /* 4. 断电线圈 (核心动作, 所有 GPIO 拉高) */
    BSP_EXV_DeEnergize();

    /* 5. 验证: 断电后 GPIO 应全部 HIGH */
    exv_dbg_print_gpio("after DeEnergize :");

    /* 6. HOLD N 秒, 让工程师有时间触摸测温 */
    exv_dbg_hold(EXV_DEBUG_HOLD_SEC);
}

/* ===========================================================================
 *  任务主函数
 * =========================================================================== */
void Task_EXV_Process(void const *argument)
{
    (void)argument;

    /* 1. 等 2 秒, 保证 RS485 任务/串口初始化完成, 日志能正常输出 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* 2. 横幅 */
    exv_dbg_print(
        "\r\n"
        "============================================================\r\n"
        "[EXV-DBG] EXV HEATING DEBUG MODE\r\n"
        "[EXV-DBG]   Main state machine   : DISABLED\r\n"
        "[EXV-DBG]   Compressor inverter  : NO CMD (idle)\r\n"
        "[EXV-DBG]   Relays / fans        : NO CMD (idle)\r\n"
        "[EXV-DBG]   Only the EXV coil is being driven.\r\n"
        "[EXV-DBG]   Valve: sanhua DPF(R04)1.5D-07, 500 steps full,\r\n"
        "[EXV-DBG]          half-step 1-2 phase, DC12V.\r\n"
        "============================================================\r\n");

    /* 3. 560 步冷启动关零.
     *    BSP_EXV_Init() 已在 freertos.c 的 MX_FREERTOS_Init 里调用过,
     *    这里只做机械归零; 调度器已经在运行, vTaskDelay 正常工作. */
    exv_dbg_print("[EXV-DBG] Cold reset 560 steps (~11s, CLOSE direction)\r\n");
    BSP_EXV_ResetToZero();   /* 内部: 步进 560 + 200ms 结束励磁 + DeEnergize */
    exv_dbg_print_gpio("after reset      :");

    /* 4. 初始保持: 让用户先感受线圈复位后的温度 */
    exv_dbg_hold(EXV_DEBUG_HOLD_SEC);

    /* 5. 无限循环: 四段开合位置, 每段完整测一遍 "动作 + HOLD" */
    for (;;) {
        exv_dbg_move_and_hold("OPEN  to 250 (50% )", 250);
        exv_dbg_move_and_hold("OPEN  to 500 (100%)", EXV_TOTAL_STEPS);
        exv_dbg_move_and_hold("CLOSE to 250 (50% )", 250);
        exv_dbg_move_and_hold("CLOSE to   0 (0%  )", 0);
    }
}
