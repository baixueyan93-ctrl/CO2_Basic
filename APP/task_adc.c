#include "task_adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "sys_state.h"

extern ADC_HandleTypeDef hadc1;

/* DMA 缓冲区, 按 Rank 顺序:
 *   [0] = INUI4    PC3  CH13  10K NTC  压缩机进口
 *   [1] = INUI5    PC2  CH12  50K NTC  压缩机出口/气冷器进口
 *   [2] = INUI0    PA3  CH3   10K NTC  蒸发器进口
 *   [3] = INUI1    PA2  CH2   10K NTC  蒸发器出口
 *   [4] = INUI6    PC1  CH11  50K NTC  气冷器出口
 *   [5] = AN5VIN0  PA7  CH7   Low pressure  (SANHUA YCQB09L02, 0~9MPa)
 *   [6] = AN5VIN1  PC4  CH14  High pressure (SANHUA YCQB15L01, 0~15MPa)
 *   [7] = INUI2    PA1  CH1   10K NTC  柜温 (新增)
 */
volatile uint16_t adc_buffer[8] = {0};

float g_temp_inui4_10k = 0.0f;
float g_temp_inui5_50k = 0.0f;
float g_temp_inui0_10k = 0.0f;
float g_temp_inui1_10k = 0.0f;
float g_temp_inui6_50k = 0.0f;
float g_temp_inui2_10k = 0.0f;
float g_pres_low  = 0.0f;
float g_pres_high = 0.0f;

/* ==========================================================
 * 10K NTC 转换函数
 * ========================================================== */
int16_t adc_to_temperature_10k(uint16_t adc_value) {
    const float V_REF = 3.3f;
    const uint16_t ADC_MAX = 4095;
    const float R_REF = 10000.0f;
    const float R_NTC_NOMINAL = 10000.0f;
    const float T_NOMINAL = 298.15f;
    const float B_VALUE = 3950.0f;

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    float voltage = (adc_value * V_REF) / ADC_MAX;
    if (voltage < 0.001f) return 1000;

    float r_ntc = ( voltage / (V_REF - voltage)) * R_REF;
    float inv_t = (1.0f / T_NOMINAL) + (log(r_ntc / R_NTC_NOMINAL) / B_VALUE);
    float temp_celsius = (1.0f / inv_t) - 273.15f;

    int32_t temp_scaled = (int32_t)(temp_celsius * 10.0f + 0.5f);
    if (temp_scaled < -1000) return -1000;
    else if (temp_scaled > 2000) return 2000;
    else return (int16_t)temp_scaled;
}

/* ==========================================================
 * 50K NTC 转换函数
 * ========================================================== */
int16_t adc_to_temperature_50k(uint16_t adc_value) {
    const float V_REF = 3.3f;
    const uint16_t ADC_MAX = 4095;
    const float R_REF = 50000.0f;
    const float R_NTC_NOMINAL = 50000.0f;
    const float T_NOMINAL = 298.15f;
    const float B_VALUE = 3950.0f;

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    float voltage = (adc_value * V_REF) / ADC_MAX;
    if (voltage < 0.001f) return 1000;

    float r_ntc = ( voltage / (V_REF - voltage)) * R_REF;
    float inv_t = (1.0f / T_NOMINAL) + (log(r_ntc / R_NTC_NOMINAL) / B_VALUE);
    float temp_celsius = (1.0f / inv_t) - 273.15f;

    int32_t temp_scaled = (int32_t)(temp_celsius * 10.0f + 0.5f);
    if (temp_scaled < -1000) return -1000;
    else if (temp_scaled > 2000) return 2000;
    else return (int16_t)temp_scaled;
}

/* ==========================================================
 * CO2 饱和压力 → 饱和温度 (多项式拟合)
 *
 * 数据来源: CO2物性参数表 (新建 Microsoft Excel 工作表(1).xlsx)
 * 有效范围: 0.58 ~ 7.27 MPa  (-50℃ ~ +31℃)
 *
 * 拟合公式 (5阶多项式, 输入MPa, 输出℃):
 *   T = -72.00065 + 45.68597·P - 13.8827·P² + 2.86921·P³
 *       - 0.31175·P⁴ + 0.01346·P⁵
 *
 * 最大误差 ≤ 0.5℃, 满足制冷控制精度要求.
 * 采用 Horner 法计算, 减少乘法次数, 提高数值稳定性.
 * ========================================================== */
static float co2_pressure_to_sat_temp(float pressure_bar)
{
    /* bar → MPa */
    float p = pressure_bar * 0.1f;

    /* 范围保护 */
    if (p < 0.58f) p = 0.58f;
    if (p > 7.28f) p = 7.28f;

    /* Horner 法: T = ((((0.01346·P - 0.31175)·P + 2.86921)·P - 13.8827)·P + 45.68597)·P - 72.00065 */
    float t = 0.01346f;
    t = t * p + (-0.31175f);
    t = t * p + 2.86921f;
    t = t * p + (-13.8827f);
    t = t * p + 45.68597f;
    t = t * p + (-72.00065f);

    return t;
}


/* ==========================================================
 * 压力传感器 ADC → 压力值 (bar)
 *
 * 硬件电路 (原理图 V13 Page2 "ImportPre"):
 *   传感器输出 (0.5~4.5V 或 0.5~3.5V, 5V供电)
 *     → R_upper (5.1kΩ) → MCU ADC引脚
 *                              │
 *                          R_lower (3.3kΩ) → VSSA
 *
 *   分压比 K = R_lower / (R_upper + R_lower) = 3.3/8.4 = 0.3929
 *   还原传感器电压: V_sensor = V_adc / K
 *
 * 传感器规格 (三花SANHUA):
 *   高压 YCQB15L01: Vout 0.5~4.5V, P 0~15MPa (0~150bar)
 *   低压 YCQB09L02: Vout 0.5~3.5V, P 0~9MPa  (0~90bar)
 *
 * 线性关系: P = (V_sensor - V_ZERO) / (V_FULL - V_ZERO) × P_MAX
 * ========================================================== */

/* 分压电阻参数 (实测校准: 2.2kΩ / 3.3kΩ) */
#define PRES_R_UPPER        2200.0f     /* 上臂电阻 2.2kΩ */
#define PRES_R_LOWER        3300.0f     /* 下臂电阻 3.3kΩ */
#define PRES_DIV_RATIO      (PRES_R_LOWER / (PRES_R_UPPER + PRES_R_LOWER))  /* 0.6 */

/* 传感器通用参数 */
#define PRES_V_ZERO         0.5f        /* 零压力输出电压 (V) */

/* 高压传感器参数 */
#define PRES_HIGH_V_FULL    4.5f        /* 满量程输出电压 (V) */
#define PRES_HIGH_MAX_BAR   150.0f      /* 满量程压力 15MPa = 150bar */

/* 低压传感器参数 */
#define PRES_LOW_V_FULL     3.5f        /* 满量程输出电压 (V) */
#define PRES_LOW_MAX_BAR    90.0f       /* 满量程压力 9MPa = 90bar */

static float adc_to_pressure(uint16_t adc_value, float v_full, float p_max_bar)
{
    const float V_REF = 3.3f;
    const uint16_t ADC_MAX = 4095;

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    /* ADC值 → MCU引脚电压 */
    float v_adc = (adc_value * V_REF) / ADC_MAX;

    /* 还原传感器实际输出电压 (除以分压比) */
    float v_sensor = v_adc / PRES_DIV_RATIO;

    /* 传感器电压 → 压力 (bar) */
    float pressure = (v_sensor - PRES_V_ZERO) / (v_full - PRES_V_ZERO) * p_max_bar;

    /* 下限保护: 传感器断线或电压低于0.5V时钳位到0 */
    if (pressure < 0.0f) pressure = 0.0f;
    /* 上限保护: 不超过量程 */
    if (pressure > p_max_bar) pressure = p_max_bar;

    return pressure;
}

/* ==========================================================
 * ADC采集线程, 持续刷新温度和压力
 *
 * 传感器物理位置 (CO2跨临界循环):
 *   INUI4(10K) → 压缩机进口温度      + 低压传感器
 *   INUI5(50K) → 压缩机出口/气冷器进口温度 + 高压传感器
 *   INUI6(50K) → 气体冷却器出口温度
 *   INUI0(10K) → 蒸发器进口温度
 *   INUI1(10K) → 蒸发器出口温度 (过热度计算用)
 *   SHT30      → 箱体/柜温 (由task_sht30写入)
 * ========================================================== */
void Task_ADC_Process(void const *argument) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8);
    vTaskDelay(pdMS_TO_TICKS(500));

    for(;;) {
        /* 1. 转换全部6路NTC温度 (4x10K + 2x50K) */
        int16_t t_inui4 = adc_to_temperature_10k(adc_buffer[0]);  /* INUI4 PC3 10K 压缩机进口 */
        int16_t t_inui5 = adc_to_temperature_50k(adc_buffer[1]);  /* INUI5 PC2 50K 压缩机出口/气冷器进口 */
        int16_t t_inui0 = adc_to_temperature_10k(adc_buffer[2]);  /* INUI0 PA3 10K 蒸发器进口 */
        int16_t t_inui1 = adc_to_temperature_10k(adc_buffer[3]);  /* INUI1 PA2 10K 蒸发器出口 */
        int16_t t_inui6 = adc_to_temperature_50k(adc_buffer[4]);  /* INUI6 PC1 50K 气冷器出口 */
        int16_t t_inui2 = adc_to_temperature_10k(adc_buffer[7]);  /* INUI2 PA1 10K 柜温 */

        /* 2. 转换2路压力传感器 */
        float pres_l = adc_to_pressure(adc_buffer[5],
                                       PRES_LOW_V_FULL, PRES_LOW_MAX_BAR);   /* 低压 PA7 (压缩机进口侧) */
        float pres_h = adc_to_pressure(adc_buffer[6],
                                       PRES_HIGH_V_FULL, PRES_HIGH_MAX_BAR); /* 高压 PC4 (压缩机出口侧) */

        /* 3. CO2 饱和温度计算 (压力→温度, 多项式拟合) */
        float sat_temp_low  = co2_pressure_to_sat_temp(pres_l);  /* 低压饱和温度 */
        float sat_temp_high = co2_pressure_to_sat_temp(pres_h);  /* 高压饱和温度 */

        /* 4. 过热度 = 蒸发器出口实测温度(INUI1) - 低压饱和温度 */
        float evap_out_actual = t_inui1 / 10.0f;  /* INUI1 10K = 蒸发器出口 */
        float superheat = evap_out_actual - sat_temp_low;
        if (superheat < 0.0f) superheat = 0.0f;

        /* 5. 更新全局变量 (供RS485调试输出) */
        g_temp_inui4_10k = t_inui4 / 10.0f;  /* 压缩机进口 */
        g_temp_inui5_50k = t_inui5 / 10.0f;  /* 压缩机出口/气冷器进口 */
        g_temp_inui0_10k = t_inui0 / 10.0f;  /* 蒸发器进口 */
        g_temp_inui1_10k = t_inui1 / 10.0f;  /* 蒸发器出口 */
        g_temp_inui6_50k = t_inui6 / 10.0f;  /* 气冷器出口 */
        g_temp_inui2_10k = t_inui2 / 10.0f;  /* 柜温 (NTC) */
        g_pres_low  = pres_l;
        g_pres_high = pres_h;

        /* 6. 写入系统状态 (互斥保护) */
        SysState_Lock();
        SysVarData_t *p = SysState_GetRawPtr();

        /* --- 6路NTC → 新字段 (按循环位置) --- */
        p->VAR_COMP_OUT_TEMP  = g_temp_inui5_50k;  /* INUI5 压缩机出口/气冷器进口 */
        p->VAR_GC_OUT_TEMP    = g_temp_inui6_50k;  /* INUI6 气体冷却器出口 */
        p->VAR_EVAP_IN_TEMP   = g_temp_inui0_10k;  /* INUI0 蒸发器进口 */
        p->VAR_EVAP_OUT_TEMP  = g_temp_inui1_10k;  /* INUI1 蒸发器出口 */
        p->VAR_COMP_IN_TEMP   = g_temp_inui4_10k;  /* INUI4 压缩机进口 */
        p->VAR_CABINET_TEMP   = g_temp_inui2_10k;  /* INUI2 柜温 (NTC) */

        /* --- 压力 --- */
        p->VAR_SUCTION_PRES   = pres_l;             /* 低压 (压缩机进口侧) */
        p->VAR_DISCHARGE_PRES = pres_h;             /* 高压 (压缩机出口侧) */

        /* --- CO2 饱和温度 --- */
        p->VAR_SAT_TEMP_LOW   = sat_temp_low;       /* 低压饱和温度 */
        p->VAR_SAT_TEMP_HIGH  = sat_temp_high;      /* 高压饱和温度 */

        /* --- 计算量 --- */
        p->VAR_SUPERHEAT      = superheat;           /* 过热度 */

        /* --- 兼容旧字段 --- */
        p->VAR_EXHAUST_TEMP   = g_temp_inui5_50k;   /* 排气温度 */
        p->VAR_SUCTION_TEMP   = sat_temp_low;        /* 吸气温度(饱和温度) */
        p->VAR_COND_TEMP      = sat_temp_high;       /* 冷凝温度(饱和温度) */
        p->VAR_EVAP_TEMP      = g_temp_inui0_10k;   /* 蒸发温度 */

        SysState_Unlock();

        /* 7. 每 100ms 刷新一次 */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ==========================================================
 * 公共接口: CO2 饱和压力→饱和温度 (供外部模块调用)
 * 输入: pressure_bar (单位 bar)
 * 输出: 饱和温度 (单位 ℃)
 * ========================================================== */
float CO2_PressureToSatTemp(float pressure_bar)
{
    return co2_pressure_to_sat_temp(pressure_bar);
}
