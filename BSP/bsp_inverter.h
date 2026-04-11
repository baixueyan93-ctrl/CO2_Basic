#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *  PC12 = RS485 方向控制引脚
 *
 *  简化版: ASCII 单字符协议 (9600bps, 8N1)
 *
 *  主控 → 变频板 (每次 1 字节):
 *    'R' / 'r'  启动压缩机
 *    'S' / 's'  停止压缩机
 *    '0'        80Hz  (1200 rpm)
 *    '1'        160Hz (2400 rpm)
 *    '2'        240Hz (3600 rpm)
 *    '3'        320Hz (4800 rpm)
 *
 *  说明: 本简化版不做 echo 校验, 发出即认为成功.
 * =================================================================== */

/* --- 兼容旧宏 (其他源文件可能引用) --- */
#define INV_CMD_STOP         0x00
#define INV_CMD_START        0x01
#define INV_CMD_SET_FREQ     0x02

#define INV_FREQ_MIN         120
#define INV_FREQ_MAX         320
#define INV_FREQ_ABS_MAX     360

/* --- 挡位编号 (对应 '0'/'1'/'2'/'3') --- */
#define INV_GEAR_LOW         0   /* '0' 80Hz  */
#define INV_GEAR_MID_LOW     1   /* '1' 160Hz */
#define INV_GEAR_MID_HIGH    2   /* '2' 240Hz */
#define INV_GEAR_HIGH        3   /* '3' 320Hz */

/* ===================================================================
 *  变频器状态结构 (兼容原有字段, 简化版仅记录最后一次发送)
 * =================================================================== */
typedef struct {
    uint16_t last_freq_hz;      /* 最后一次发送的目标频率 (Hz) */
    uint8_t  last_cmd;          /* 最后一次发送的命令类型      */
    bool     echo_ok;           /* 简化版固定为 true           */
    bool     comm_ok;           /* 简化版固定为 true           */
    uint8_t  fail_reason;       /* 简化版固定为 0              */
    uint16_t motor_speed_hz;    /* 兼容字段, 无实际回读        */
    uint16_t fault_stop;        /* 兼容字段                    */
    uint16_t fault_warn;        /* 兼容字段                    */
    uint16_t out_current_x10;   /* 兼容字段                    */
    uint16_t bus_voltage;       /* 兼容字段                    */
    char     last_char;         /* 简化版新增: 最后一次发送的 ASCII 字符 */
} InvStatus_t;

/* ===================================================================
 *  外部变量
 * =================================================================== */
extern UART_HandleTypeDef huart4;
extern volatile uint8_t InvAckOK;
extern InvStatus_t g_InvStatus;

/* ===================================================================
 *  公共接口
 * =================================================================== */

/* 初始化: 配置 RS485 方向引脚 + UART 收发模式 */
void BSP_Inverter_Init(void);

/* --- 简化版 ASCII 单字符发送接口 --- */
void BSP_Inverter_SendChar(char c);        /* 发一个 ASCII 字符 */
void BSP_Inverter_SendRun(void);           /* 发 'R' 启动       */
void BSP_Inverter_SendStop(void);          /* 发 'S' 停机       */
void BSP_Inverter_SendGear(uint8_t gear);  /* gear=0/1/2/3 → '0'/'1'/'2'/'3' */

/* --- 兼容旧接口 (cmd + freq_hz 组合) --- */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz);

/* 状态读取 (兼容, 简化版总是返回 echo_ok=true) */
bool BSP_Inverter_ReadStatus(InvStatus_t *out);

#endif /* BSP_INVERTER_H */
