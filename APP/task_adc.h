#ifndef __TASK_ADC_H
#define __TASK_ADC_H
#include <stdint.h>

/* ===================================================================
 * ADC 通道温度值 (单位 °C)
 *
 * 物理位置 (CO2跨临界循环):
 *   INUI4(10K) = 压缩机进口      + 低压传感器
 *   INUI5(50K) = 压缩机出口/气冷器进口 + 高压传感器
 *   INUI6(50K) = 气体冷却器出口
 *   INUI0(10K) = 蒸发器进口
 *   INUI1(10K) = 蒸发器出口 (过热度计算)
 * =================================================================== */
extern float g_temp_inui4_10k;   /* INUI4  PC3  10K  压缩机进口温度        */
extern float g_temp_inui5_50k;   /* INUI5  PC2  50K  压缩机出口/气冷器进口 */
extern float g_temp_inui0_10k;   /* INUI0  PA3  10K  蒸发器进口温度        */
extern float g_temp_inui1_10k;   /* INUI1  PA2  10K  蒸发器出口温度        */
extern float g_temp_inui6_50k;   /* INUI6  PC1  50K  气体冷却器出口温度    */
extern float g_temp_inui2_10k;   /* INUI2  PA1  10K  柜温                  */

/* 压力传感器 (单位 bar) */
extern float g_pres_low;         /* AN5VIN0 PA7  低压 0~9MPa  (压缩机进口侧) */
extern float g_pres_high;        /* AN5VIN1 PC4  高压 0~15MPa (压缩机出口侧) */

/* ADC DMA 原始值 (调试用) */
extern volatile uint16_t adc_buffer[8];

/* 兼容旧变量名 */
#define g_temp_10k  g_temp_inui4_10k
#define g_temp_50k  g_temp_inui5_50k

/* CO2 饱和压力→饱和温度 转换 (供外部模块调用) */
float CO2_PressureToSatTemp(float pressure_bar);

void Task_ADC_Process(void const *argument);

#endif
