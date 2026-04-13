#include "task_panel.h"
#include "task_simple_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "bsp_htc_2k.h"
#include "bsp_relay.h"
#include "bsp_inverter.h"
#include "bsp_rs485.h"
#include "bsp_eeprom.h"
#include <stdio.h>
#include "sys_state.h"
#include <string.h>
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 面板操作全局变量
 * =========================================================================== */
float         g_set_temp   = SET_TEMP_TS;  /* 用户设定温度, 默认跟随Ts */
uint8_t       g_panel_mode = 0;     /* 0: 正常监控, 1: 温度设置模式 */
bool          g_light_on   = false; /* 照明灯状态 */
volatile bool g_system_on  = false; /* 系统电源状态, 上电默认关 (需按键开机) */

static uint32_t s_set_mode_tick = 0;
#define SET_MODE_TIMEOUT_MS  5000   /* 5秒无操作自动退出设置模式 */

/* EEPROM 存储设定温度 */
#define EEPROM_ADDR_SET_TEMP   0x0008   /* 存放地址 (0x0000被日志索引占用) */
#define EEPROM_TEMP_MAGIC      0xA5     /* 校验字节, 防止首次上电读到垃圾值 */

/* 从EEPROM读取上次设定温度 */
static void Panel_LoadSetTemp(void)
{
    uint8_t buf[5];  /* 1字节校验 + 4字节float */
    BSP_EEPROM_Read(EEPROM_ADDR_SET_TEMP, buf, 5);

    if (buf[0] == EEPROM_TEMP_MAGIC) {
        float temp;
        memcpy(&temp, &buf[1], 4);
        if (temp >= -50.0f && temp <= 50.0f) {
            g_set_temp = temp;
        }
    }
    /* 校验不通过则保持默认值 SET_TEMP_TS */
}

/* 保存设定温度到EEPROM */
static void Panel_SaveSetTemp(void)
{
    uint8_t buf[5];
    buf[0] = EEPROM_TEMP_MAGIC;
    memcpy(&buf[1], &g_set_temp, 4);
    BSP_EEPROM_Write(EEPROM_ADDR_SET_TEMP, buf, 5);
}

/* ===========================================================================
 * 双面板任务
 *
 * 两块屏硬件一模一样, 按键都在 TM1637 K1 扫描线上 (码值 0xF7/F6/F5/F4)
 *
 * PANEL0 (PB6/PB7): 显示温度+图标 + 4按键 (Reset/↓减温/↑加温/Set)
 * PANEL1 (PB4/PB5): 显示温度+图标 + 4按键 (电源/除霜/照明/预留)
 *
 * 两块屏显示内容相同: 正常模式显示 NTC 柜温, 设置模式显示设定温度
 * 图标也相同: 除霜/风扇/制冷/加热/灯光等
 * =========================================================================== */
void Task_Panel_Process(void const *argument)
{
    (void)argument;
    uint8_t blink_cnt = 0;

    /* 初始化两个面板 */
    HTC2K_Init();   /* PANEL0 (PB6/PB7) */
    HTC2K_Init1();  /* PANEL1 (PB4/PB5) */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 从EEPROM读取上次设定温度, 读不到则保持默认 SET_TEMP_TS */
    Panel_LoadSetTemp();

    for (;;) {
        /* 读取系统状态 */
        SysVarData_t sensor;
        SysState_GetSensor(&sensor);
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
        uint32_t alarms = g_AlarmFlags;

        /* ============================================
         * 构建统一图标 (两块屏共用)
         * ============================================ */
        icon_type_t icon_common;
        icon_common.byte = 0;

        if (sys_bits & ST_DEFROST_ACTIVE) icon_common.bits.Def   = 1;
        if (sys_bits & ST_DEF_HEATING)    icon_common.bits.Heat  = 1;
        if (sys_bits & ST_EVAP_FAN_ON)    icon_common.bits.Fan   = 1;
        if (sys_bits & ST_COMP_RUNNING)   icon_common.bits.Ref   = 1;
        if (g_light_on)                   icon_common.bits.Light = 1;
        if (g_panel_mode == 1)            icon_common.bits.Set   = 1;
        if (sensor.VAR_LIQUID_LEVEL == 1) icon_common.bits.Humi  = 1;

        /* 报警时制冷图标闪烁 */
        if (alarms & (ERR_MASK_ALL | WARN_MASK_ALL)) {
            blink_cnt++;
            if (blink_cnt & 0x04) icon_common.bits.Ref = 1;
            else                  icon_common.bits.Ref = 0;
        }

        /* 两块屏图标相同 */
        g_IconSet.byte  = icon_common.byte;
        g_IconSet1.byte = icon_common.byte;

        /* ============================================
         * 显示温度 (两块屏内容相同)
         * ============================================ */
        if (g_panel_mode == 0) {
            /* 正常模式: 显示 NTC 柜温 */
            HTC2K_ShowTemp(sensor.VAR_CABINET_TEMP);
            HTC2K_ShowTemp1(sensor.VAR_CABINET_TEMP);
        } else {
            /* 设置模式: 显示设定温度 */
            HTC2K_ShowTemp(g_set_temp);
            HTC2K_ShowTemp1(g_set_temp);
        }

        /* ============================================
         * PANEL0 按键: Reset / Set / 上 / 下
         * ============================================ */
        uint8_t key0 = HTC2K_ReadKeys();

        if (key0 != 0x00 && key0 != 0xFF) {
            if (key0 == KEY_CODE_RST) {
                g_panel_mode = 0;
                g_set_temp = SET_TEMP_TS;
                Panel_SaveSetTemp();
            }
            else if (key0 == KEY_CODE_SET) {
                g_panel_mode = !g_panel_mode;
                s_set_mode_tick = xTaskGetTickCount();
            }
            else if (g_panel_mode == 1) {
                if (key0 == KEY_CODE_UP)   { g_set_temp += 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (key0 == KEY_CODE_DOWN) { g_set_temp -= 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (g_set_temp > 50.0f)  g_set_temp = 50.0f;
                if (g_set_temp < -50.0f) g_set_temp = -50.0f;
                Panel_SaveSetTemp();
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        /* ============================================
         * PANEL1 按键: 电源 / 除霜 / 照明 / 点检
         * 两块屏硬件一样, 键码都是 K1 线 (0xF7/F6/F5/F4)
         * ============================================ */
        uint8_t key1 = HTC2K_ReadKeys1();

        if (key1 != 0x00 && key1 != 0xFF) {
            if (key1 == KEY_CODE_RST) {
                /* 按键1: 电源 ON/OFF */
                g_system_on = !g_system_on;
                if (g_system_on) {
                    BSP_RS485_SendString("[KEY] Power ON req\r\n");
                } else {
                    BSP_RS485_SendString("[KEY] Power OFF req\r\n");
                }
            }
            else if (key1 == KEY_CODE_DOWN) {
                /* 按键2: 除霜 */
                g_defrost_req = 1;
                BSP_RS485_SendString("[KEY] DEFROST req\r\n");
            }
            else if (key1 == KEY_CODE_UP) {
                /* 按键3: 照明 */
                g_light_on = !g_light_on;
                BSP_Relay_Set(RELAY_LIGHT, g_light_on ? 1 : 0);
            }
            else if (key1 == KEY_CODE_SET) {
                /* 按键4: 预留 */
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        /* 5秒无操作自动退出设置模式 */
        if (g_panel_mode == 1) {
            if ((xTaskGetTickCount() - s_set_mode_tick) > pdMS_TO_TICKS(SET_MODE_TIMEOUT_MS)) {
                g_panel_mode = 0;
                Panel_SaveSetTemp();  /* 超时退出时也保存 */
            }
        }

        /* 任务休眠 50ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
