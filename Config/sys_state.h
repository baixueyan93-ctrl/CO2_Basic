#ifndef SYS_STATE_H
#define SYS_STATE_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  CO2 制冷系统 — 传感器 / 状态标记 / 报警 (来源: 逻辑图 1~6)
 *
 *  命名约定:
 *   VAR_   = 过程变量值 (传感器采集 / 中间层计算)
 *   ST_    = 状态标记   (bool 型, 描述当前系统状态)
 *   TMR_   = 定时器数据 (由定时中断递减, 到时置位对应 ST_ 标记)
 *   ERR_   = 故障       (严重, 停机并通知用户)
 *   WARN_  = 警告       (暂时可忽略, 超过设定时间长则通知用户)
 *
 *  传感器物理位置 (CO2跨临界循环):
 *
 *   压缩机出口 ──→ 气体冷却器 ──→ 膨胀阀 ──→ 蒸发器 ──→ 压缩机进口
 *     │                 │                     │        │        │
 *   INUI5(50K)      INUI6(50K)            INUI0(10K) INUI1(10K) INUI4(10K)
 *   +高压传感器                                                  +低压传感器
 *
 *   箱体/柜温: SHT-30 (I2C)
 * =================================================================== */

/* ===================================================================
 *  第一部分: 过程变量值 (VAR_) — 传感器采集 + 中间层计算
 * =================================================================== */
typedef struct {
    /* ===== 温度传感器 (NTC) — 按CO2循环顺序排列 ===== */

    /* 压缩机出口 / 气体冷却器进口 — INUI5 50K NTC (高温侧)
     * 用途: 排气温度监控, 压缩机保护, 过热度参考 */
    float VAR_COMP_OUT_TEMP;    /* T_comp_out  压缩机出口温度 (°C)         */

    /* 气体冷却器出口 — INUI6 50K NTC (高温侧)
     * 用途: 气冷器效率判断, 膨胀阀控制参考 */
    float VAR_GC_OUT_TEMP;      /* T_gc_out    气体冷却器出口温度 (°C)     */

    /* 蒸发器进口 — INUI0 10K NTC (低温侧)
     * 用途: 蒸发温度监控, 除霜判断 */
    float VAR_EVAP_IN_TEMP;     /* T_evap_in   蒸发器进口温度 (°C)         */

    /* 蒸发器出口 — INUI1 10K NTC (低温侧)
     * 用途: 过热度计算(实测吸气温度), 蒸发器效率 */
    float VAR_EVAP_OUT_TEMP;    /* T_evap_out  蒸发器出口温度 (°C)         */

    /* 压缩机进口 — INUI4 10K NTC (低温侧)
     * 用途: 吸气温度监控, 液击保护 */
    float VAR_COMP_IN_TEMP;     /* T_comp_in   压缩机进口温度 (°C)         */

    /* ===== 压力传感器 (SANHUA) ===== */

    /* 低压 — AN5VIN0 PA7, YCQB09L02 0~9MPa, 与INUI4同侧(压缩机进口)
     * 用途: 蒸发压力, 低压饱和温度计算, 过热度计算, 低压保护 */
    float VAR_SUCTION_PRES;     /* PL  吸气压力/低压 (bar)                 */

    /* 高压 — AN5VIN1 PC4, YCQB15L01 0~15MPa, 与INUI5同侧(压缩机出口)
     * 用途: 排气压力, 高压饱和温度计算, 高压保护 */
    float VAR_DISCHARGE_PRES;   /* PH  排气压力/高压 (bar)                 */

    /* ===== CO2饱和温度 (由压力经多项式拟合计算) ===== */

    float VAR_SAT_TEMP_LOW;     /* 低压饱和温度 (°C), 由 PL 计算           */
    float VAR_SAT_TEMP_HIGH;    /* 高压饱和温度 (°C), 由 PH 计算           */

    /* ===== 计算量 ===== */

    float VAR_SUPERHEAT;        /* 过热度 = EVAP_OUT_TEMP - SAT_TEMP_LOW (°C) */
    float VAR_DELTA_T;          /* 温差 = 柜温 - 设定温度 (°C)             */
    float VAR_COMP_FREQ;        /* 压缩机当前运行频率 (Hz)                 */
    float VAR_EXV_OPENING;      /* 膨胀阀开度 (步/%)                       */
    float VAR_VDC_VOLTAGE;      /* 电源电压 (V)                            */

    /* ===== SHT30 温湿度传感器 (I2C, 箱体内部) ===== */

    float VAR_CABINET_TEMP;     /* 柜温 (°C), INUI2 PA1 10K NTC 采集       */
    float VAR_SHT30_TEMP;       /* SHT30 原始温度 (°C)                     */
    float VAR_SHT30_HUMI;       /* SHT30 相对湿度 (% RH)                   */

    /* ===== 其他传感器 ===== */

    uint8_t VAR_LIQUID_LEVEL;   /* 液位状态 (1:满水, 0:正常)               */

    /* ===== 兼容旧变量名 (逐步淘汰) ===== */
    float VAR_EXHAUST_TEMP;     /* = VAR_COMP_OUT_TEMP, 旧名: 排气温度     */
    float VAR_SUCTION_TEMP;     /* = VAR_SAT_TEMP_LOW,  旧名: 吸气温度     */
    float VAR_COND_TEMP;        /* = VAR_SAT_TEMP_HIGH, 旧名: 冷凝温度     */
    float VAR_EVAP_TEMP;        /* = VAR_EVAP_IN_TEMP,  旧名: 蒸发温度     */
    float VAR_AMBIENT_TEMP;     /* 环境温度 (°C), SHT30采集, 油壳加热用    */
    float VAR_HT_DIFF;          /* 高低温差 (°C), 预留                     */
} SysVarData_t;


/* ===================================================================
 *  第二部分: 状态标记 (ST_) — 用 FreeRTOS EventGroup 位
 * =================================================================== */

/* --- 图1: 温度控制 / 压缩机状态 --- */
#define ST_COMP_RUNNING         (1 <<  0)  /* 压缩机正在工作                */
#define ST_SYSTEM_ON            (1 <<  1)  /* 系统开机状态                  */
#define ST_FIRST_RUN            (1 <<  2)  /* 首次上电(通电第一个周期)      */
#define ST_OIL_HEAT_ON          (1 <<  3)  /* 油壳加热已开启                */
#define ST_WARMUP_DONE          (1 <<  4)  /* 热机时间C20已到时             */

/* --- 图2: 除霜状态 --- */
#define ST_DEFROST_ACTIVE       (1 <<  5)  /* 正在除霜(总标记)              */
#define ST_DEF_HEATING          (1 <<  6)  /* 正在加热(除霜加热阶段)        */
#define ST_DEF_DRIPPING         (1 <<  7)  /* 正在滴水(除霜滴水阶段)        */

/* --- 图3: 蒸发风机状态 --- */
#define ST_EVAP_FAN_ON          (1 <<  8)  /* 蒸发风机正在运行              */
#define ST_EVAP_TEMP_QUAL       (1 <<  9)  /* 蒸发温度合格                  */

/* --- 图4: PID/变频状态 --- */
#define ST_DT_SHRINKING         (1 << 10)  /* 温差正在缩小(降频趋势)        */

/* --- 图6: 冷凝风机状态 --- */
#define ST_COND_FAN1_ON         (1 << 11)  /* 冷凝风机1正在运行             */
#define ST_COND_FAN2_ON         (1 << 12)  /* 冷凝风机2正在运行             */
#define ST_COND_FAN3_ON         (1 << 13)  /* 冷凝风机3正在运行             */

/* ===================================================================
 *  第三部分: 定时标记 (ST_TMR_) — 由图5定时中断置位
 *  使用第二个 EventGroup, 因为位数可能不够
 * =================================================================== */
#define ST_TMR_TICK_1S          (1 <<  0)  /* 1秒定时器                     */
#define ST_TMR_PID_DONE         (1 <<  1)  /* PID周期(30s)到时              */
#define ST_TMR_C2_DONE          (1 <<  2)  /* 停机保护时间C2到时            */
#define ST_TMR_C3_DONE          (1 <<  3)  /* 通电延迟C3到时                */
#define ST_TMR_C7_DONE          (1 <<  4)  /* 风机启动C7到时                */
#define ST_TMR_C8_DONE          (1 <<  5)  /* 风机间隔C8到时                */
#define ST_TMR_DEF_INTV_DONE    (1 <<  6)  /* 除霜间隔到时                  */
#define ST_TMR_DEF_DUR_DONE     (1 <<  7)  /* 除霜最大时长到时              */
#define ST_TMR_DEF_DRIP_DONE    (1 <<  8)  /* 滴水时间到时                  */
#define ST_TMR_WARMUP_DONE      (1 <<  9)  /* 热机时间C20到时               */
#define ST_TMR_EVAP_FAN_DLY     (1 << 10)  /* 蒸发风机F3延时到时            */

/* ===================================================================
 *  第四部分: 定时器数据 (TMR_) — 在定时中断中递减
 * =================================================================== */
typedef struct {
    uint32_t TMR_TICK_1S_CNT;       /* 1秒基准计数                   */
    uint32_t TMR_PID_CNT;           /* PID周期计数                   */
    uint32_t TMR_C2_CNT;            /* 停机时保护计数                */
    uint32_t TMR_C3_CNT;            /* 通电延迟计数                  */
    uint32_t TMR_C7_CNT;            /* 风机启动计数                  */
    uint32_t TMR_C8_CNT;            /* 风机间隔计数                  */
    uint32_t TMR_DEF_INTV_CNT;      /* 除霜间隔计数                  */
    uint32_t TMR_DEF_DUR_CNT;       /* 除霜最大时长计数              */
    uint32_t TMR_DRIP_CNT;          /* 滴水时间计数                  */
    uint32_t TMR_WARMUP_CNT;        /* 热机时间计数                  */
    uint32_t TMR_EVAP_FAN_DLY_CNT;  /* 蒸发风机延时计数              */
    uint32_t TMR_LONGRUN_CNT;       /* 长期运行时间计数(超时告警)    */
    uint32_t TMR_PRES_HIGH_CNT;     /* 高压超时计数                  */
    uint32_t TMR_PRES_LOW_CNT;      /* 低压超时计数                  */
} SysTimerData_t;

/* ===================================================================
 *  第五部分: 报警标志 (ERR_ / WARN_) — 按位操作, 共用一个 uint32_t
 *
 *  ERR_ = 故障 (严重, 必须停机, 必须通知用户)
 *  WARN_ = 警告 (暂时可忽略, 超过时间长则通知用户)
 *
 *  用法:
 *    置位: g_AlarmFlags |= ERR_xxx;
 *    清除: g_AlarmFlags &= ~ERR_xxx;
 *    检测: if (g_AlarmFlags & ERR_xxx) { ... }
 * =================================================================== */

/* --- 故障 (Error): 必须停机 + 通知用户 --- */
#define ERR_SENSOR_CABINET      (1U <<  0) /* E1   柜温传感器故障           */
#define ERR_VDC_LOW             (1U <<  1) /* EDC  电源电压过低             */
#define ERR_VAC_PHASE           (1U <<  2) /* EAC  VAC缺相/错相            */
#define ERR_INV_OVERCURR        (1U <<  3) /* EFI  变频器过流               */
#define ERR_INV_OVERHEAT        (1U <<  4) /* EFT  变频器过热               */
#define ERR_TEMP_LOW_STOP       (1U <<  5) /* ETM  频率归零温度异常停机     */

/* --- 警告 (Warning): 暂时可忽略, 持续则通知用户 --- */
#define WARN_EXHAUST_HIGH       (1U <<  8) /* WTM  排气温度过高             */
#define WARN_SUCTION_LOW        (1U <<  9) /* WTL  吸气温度过低(持续1-2h)   */
#define WARN_PRES_HIGH          (1U << 10) /* WPH1 排气压力过高             */
#define WARN_PRES_HIGH_TMO      (1U << 11) /* WPH2 高压超时→报警通知       */
#define WARN_PRES_LOW           (1U << 12) /* WPL1 吸气压力过低             */
#define WARN_PRES_LOW_TMO       (1U << 13) /* WPL2 低压超时→报警通知       */
#define WARN_LONGRUN            (1U << 14) /* WLC  长期运行时间超限         */
#define WARN_SUPERHEAT_LOW      (1U << 15) /* EDT  过热度过低               */
#define WARN_NOTIFY_USER        (1U << 16) /* WEN  综合告警→需通知用户      */

/* --- 故障/警告掩码 (用于批量判断) --- */
#define ERR_MASK_ALL            (0x0000003FU)  /* 全部ERR位                 */
#define WARN_MASK_ALL           (0x0001FF00U)  /* 全部WARN位                */

/* ===================================================================
 *  外部全局变量声明
 * =================================================================== */
extern EventGroupHandle_t SysEventGroup;      /* 系统状态事件组             */
extern EventGroupHandle_t SysTimerEventGroup;  /* 定时器事件组              */
extern SemaphoreHandle_t  SensorDataMutex;     /* 传感器数据互斥量          */

extern volatile uint32_t  g_AlarmFlags;        /* 报警标志字                */
extern SysTimerData_t     g_TimerData;         /* 定时器数据                */

/* ===================================================================
 *  公共接口函数
 * =================================================================== */
void SysState_Init(void);
void SysState_UpdateSensor(SysVarData_t* newData);
void SysState_GetSensor(SysVarData_t* outData);
/* 高级：原子级安全读写接口 */
void SysState_Lock(void);
void SysState_Unlock(void);
SysVarData_t* SysState_GetRawPtr(void);
#endif /* SYS_STATE_H */
