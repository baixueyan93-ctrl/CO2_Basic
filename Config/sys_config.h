#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

#include <stdint.h>

/* ===========================================================================
 * CO2 ����ϵͳ �� ȫ�ֳ����趨ֵ (��Դ: �߼�ͼ 1~6)
 *
 * ��������:  SET_<ģ��>_<����>
 *   ǰ׺ SET_ ��ʾ"�û��ɵ��趨ֵ / ϵͳ����"
 * =========================================================================== */

/* ===================================================================
 *  ͼ1  �¶ȿ������� �� �����趨ֵ
 * =================================================================== */

/* --- 1.1 �¶ȿ����뱣�� --- */
#define SET_TEMP_TS             (-10.0f)   /* 设定目标柜温 Ts (℃), 范围 -50 ~ 50          */
#define SET_TEMP_HYST_C1        2.0f       /* �¶Ȼز� C1 (��C), ��Χ 1 ~ 2                 */
#define SET_STOP_TIME_C2        180        /* ͣ������ʱ�� C2 (��)                          */
#define SET_POWERON_DLY_C3      180        /* ͨ���ӳ�ʱ�� C3 (��), �״��ϵ����            */
#define SET_STBY_TIME_C7        300        /* ��������ʱ�� C7 (��)                          */
#define SET_RUN_MIN_C8          300        /* �������ʱ�� C8 (��)                          */
#define SET_WARMUP_C20          10/* �ȳ�ʱ�� C20 (��), ���������ů��             */

/* --- 1.2 ������ֵ (A ϵ��) --- */
#define SET_ALARM_A1            (-10.0f)   /* �����趨ֵ A1 �� ���µ��±��� (��C)            */
#define SET_ALARM_A2            30.0f      /* �����趨ֵ A2 �� ���¸��±��� (��C)            */
#define SET_ALARM_A3            80.0f      /* �����趨ֵ A3 �� ���±��� (��C)                */

/* --- 1.3 ѹ����������ֵ --- */
#define SET_EXHAUST_TMAX        110.0f     /* �����¶����� Tmax (��C)                       */
#define SET_SUCTION_TMIN        15.0f      /* �����¶������²� Tmin (��C), �趨ֵTLS-����   */
#define SET_DISCHARGE_PMAX_H    110.0f     /* ����ѹ������-�ߵ� Pmax (bar)                  */
#define SET_DISCHARGE_PMAX_L    70.0f      /* ����ѹ������-�͵� Pmax (bar)                  */
#define SET_SUCTION_PMIN        20.0f      /* ����ѹ������ Pmin (bar)                       */
#define SET_VDC_MIN             300.0f     /* ��Դ��ѹ���� VDCmin (V), ������Ӳ��ȷ��       */
#define SET_FREQ_INIT           120.0f     /* 压缩机初始频率 (Hz), 启动时发120Hz给变频板   */

/* --- 1.4 �ͿǼ��� --- */
#define SET_OIL_HEAT_TEMP       10.0f      /* �ͿǼ��ȿ��������¶���ֵ (��10��C ��)       */

/* --- 1.5 �澯��ʱ --- */
#define SET_WARN_LONGRUN_H      7200       /* ���������澯ʱ��-�� (��), 2Сʱ               */
#define SET_WARN_LONGRUN_L      3600       /* ���������澯ʱ��-�� (��), 1Сʱ               */
#define SET_WARN_PRES_TMO       60         /* ��ѹ/��ѹ��ʱ�ж�ʱ�� (��), ��ȷ��            */

/* ===================================================================
 *  ͼ2  ��˪���� �� �����趨ֵ
 * =================================================================== */
#define SET_DEF_INTERVAL        (3 * 3600) /* ��˪���ʱ�� (��), 2~3Сʱ                   */
#define SET_DEF_MIN_INTV        (2 * 3600) /* ��˪��̼��ʱ�� (��), 2Сʱ                 */
#define SET_DEF_HEAT_MAX        (5 * 60)   /* 加热最大时间 (秒), 5分钟                     */
#define SET_DEF_HEAT_TLIMIT     10.0f      /* �����¶�����/��˪�˳��¶� (��C)               */
#define SET_DRIP_TIME           (5 * 60)   /* ��ˮʱ�� (��), 5����                         */
#define SET_DEF_COMP_DLY        15         /* ��˪��ѹ���������ӳ� (��)                    */
#define SET_DEF_STEP_DLY        30         /* �����ӳ�����ʱ����X (��)                     */
#define SET_DEF_TEMPDIFF_THR    15.0f      /* 温差提前除霜阈值 (℃), 柜温-蒸发温度>此值触发 */

/* ===================================================================
 *  ͼ3  �������(1��6)���� �� �����趨ֵ
 * =================================================================== */
#define SET_EVAP_FAN_MODE       1          /* �����������ģʽ F1 (1/2/4)                  */
#define SET_EVAP_FAN_DLY        30         /* ���������ʱ���� F3 (��)                     */
#define SET_EVAP_TEMP_OK        (-15.0f)   /* �����¶Ⱥϸ���ֵ (��C)                       */

/* ===================================================================
 *  ͼ4  ��Ƶ����(PID)�����ͷ����� �� �����趨ֵ
 * =================================================================== */
#define SET_DT_MAX              5.0f       /* ��Tmax: �����Ƶ�²���ֵ (��C)                */
#define SET_DT_MIN              1.0f       /* ��Tmin: ���������²����� (��C)                */
#define SET_FREQ_MIN            120.0f     /* Fmin: 压缩机最低频率 (Hz)                     */
#define SET_FREQ_MAX            320.0f     /* Fmax: 压缩机最高频率 (Hz)                     */
#define SET_SH_MIN_LOW          5.0f       /* ���ȶ�����-��ֵ ��TPmin (��C)                 */
#define SET_SH_MIN_HIGH         9.0f       /* ���ȶ�����-��ֵ ��TPmin (��C)                 */
#define SET_HT_DIFF_TARGET      6.5f       /* �����²� ��TCZ Ŀ��ֵ (��C)                   */
#define SET_HT_DIFF_TOL         1.5f       /* �����²� ��TCZ �ݲ� (��1.5��C)                */
#define SET_PID_ALPHA1          0.5f       /* ��1: �����²����ϵ��                         */
#define SET_PID_ALPHA2          0.8f       /* ��2: ���ȶȵ���ϵ��                            */
#define SET_PID_PERIOD          30         /* PID�������� (��)                              */
#define SET_EXV_INIT_OPENING    250.0f     /* EXV初始开度 (步), 压缩机启动时设置, 500步全开  */

/* ===================================================================
 *  ͼ5  ��ʱ�жϷ��� �� �޶��ⳣ�� (�������� C2/C3/C7/C8/��˪�����)
 * =================================================================== */

/* ===================================================================
 *  ͼ6  �������(3̨)���� �� �����趨ֵ
 * =================================================================== */
#define SET_COND_FAN_ON_T       35.0f      /* ������������¶� (��C)                        */
#define SET_COND_FAN_OFF_T      25.0f      /* ��������ر��¶� (��C)                        */

/* ===================================================================
 *  �洢��־�ṹ�� & EEPROM ӳ�� (����ԭ��)
 * =================================================================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t Year;
    uint8_t Month;
    uint8_t Date;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;

    float   EvapTemp;
    float   SetTemp;
    float   CondTemp;

    uint8_t EventType;
    uint8_t CheckSum;
} SysLog_t;
#pragma pack(pop)

#define LOG_MAX_COUNT       100
#define LOG_SIZE            20
#define EEPROM_INDEX_ADDR   0x0000
#define EEPROM_DATA_START   0x0010

/* ===================================================================
 *  简化测试模式 (SIMPLE_MODE) 常量
 *  — 用于 task_simple_main.c 的最简运行流程
 *
 *  流程:
 *    上电 → 显示柜温 (EXV 已 560 步关零) → 等待开机键
 *    → 按开机键 → 10s 变频板自检 + 开风扇 + 启ADC
 *    → 发 'R' 启动 → 发 '0' 进 80Hz
 *    → 每 10s 主节拍: 温差换挡 + EXV PID 调整
 *    → 3h 自动除霜 (或手动除霜键)
 *    → 除霜: 发 '2' 240Hz + EXV 全开 + 关风扇, 持续 10min
 *    → 停机 + 滴水 5min → 重启回 RUN_LOW
 *    → 任意 RUN 状态按关机键: 停机 + 风扇关 + 3min 冷却保护
 * =================================================================== */

/* --- 状态定时 --- */
#define SIMPLE_SELFTEST_SEC     10         /* 开机后变频板自检时间 (秒)     */
#define SIMPLE_COOLDOWN_SEC     180        /* 关机后再开机保护 (秒, 3分钟)   */
#define SIMPLE_MAIN_TICK_SEC    10         /* 主节拍周期 (秒)                */

/* --- 温差换挡阈值 (带 ±0.5℃ 滞环, 防边界抖动) --- */
#define SIMPLE_DT_UP_TH         5.5f       /* ΔT ≥ 此值 → 升到 160Hz 高档   */
#define SIMPLE_DT_DOWN_TH       4.5f       /* ΔT ≤ 此值 → 降回 80Hz  低档   */

/* --- EXV 过热度 PID (简化版, 纯 P 控制) --- */
#define SIMPLE_SH_TARGET        7.0f       /* 目标过热度 (℃)                */
#define SIMPLE_SH_DEADBAND      0.5f       /* 死区 ±0.5℃ (6.5~7.5 不动)    */
#define SIMPLE_EXV_KP           0.5f       /* 比例系数                      */

/* --- 除霜计时 --- */
#define SIMPLE_DEF_AUTO_SEC     (3 * 3600) /* 自动除霜间隔 (秒, 3 小时)      */
#define SIMPLE_DEF_RUN_SEC      (10 * 60)  /* 除霜持续时间 (秒, 10 分钟)     */
#define SIMPLE_DEF_DRIP_SEC     (5 * 60)   /* 滴水时间 (秒, 5 分钟)          */

/* --- 变频板挡位编号 (与 bsp_inverter.h INV_GEAR_xxx 对应) --- */
#define SIMPLE_GEAR_RUN_LOW     0          /* '0' 80Hz  (1200 rpm)          */
#define SIMPLE_GEAR_RUN_HIGH    1          /* '1' 160Hz (2400 rpm)          */
#define SIMPLE_GEAR_DEFROST     2          /* '2' 240Hz (3600 rpm)          */

/* ===================================================================
 *  电子膨胀阀 (EXV) 发热调试模式
 *
 *  EXV_DEBUG_ONLY = 1
 *    — 只运行 Task_EXV (见 APP/task_exv.c) + LED + RS485 调试串口.
 *    — 主状态机 Task_SimpleMain 不创建, 不发压缩机指令, 不动继电器.
 *    — freertos.c 上电阶段的 560 步关零也会被跳过, 改由 Task_EXV
 *      在调度器启动后自己执行 (避免在没有调度器的阶段调用 vTaskDelay).
 *    — 其它任务 (Panel/ADC/SHT30) 仍然会被创建, 它们不触碰 EXV, 不影响
 *      本次发热测量, 屏幕还能显示柜温.
 *  EXV_DEBUG_ONLY = 0
 *    — 正常运行模式.
 *
 *  使用方法: 把下面一行改成 1 → 重新编译烧录 → 接 RS485 调试口看日志,
 *  每一段 "HOLD" 阶段用手摸膨胀阀线圈壳, 正常应为冷态.
 * =================================================================== */
#define EXV_DEBUG_ONLY          1          /* 0=正常, 1=只调试膨胀阀       */

#define EXV_DEBUG_HOLD_SEC      30         /* 每次动作后保持时间 (秒)      */

#endif /* SYS_CONFIG_H */



