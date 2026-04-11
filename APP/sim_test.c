/*
 * sim_test.c - 模拟器交互测试
 *
 * 用法:
 *   1. main.c 里打开 #define SIM_MODE
 *   2. 编译, Ctrl+F5 进Debug
 *   3. 在 SIM_RunTest() 的 while(1) 里设断点
 *   4. Watch窗口修改 sim_in_xxx 输入值
 *   5. 按F5继续, 看 sim_out_xxx 输出值怎么变
 */
#include "sim_test.h"
#include "sys_config.h"

/* ============================================
 * >>> 你改这些值 (在Watch窗口双击Value修改) <<<
 * ============================================ */
float sim_in_cabinet_temp  = 10.0f;   /* 柜温 (度) */
float sim_in_set_temp      = -10.0f;  /* 设定温度 (度) */
float sim_in_evap_temp     = -2.0f;   /* 蒸发温度 (度) */
float sim_in_superheat     = 8.0f;    /* 过热度 (度) */
float sim_in_exv_opening   = 250.0f;  /* 膨胀阀当前开度 (0~500步) */
float sim_in_comp_freq     = 120.0f;  /* 压缩机当前频率 (Hz) */

/* ============================================
 * >>> 看这些结果 <<<
 * ============================================ */
float sim_out_delta_t;       /* 温差 = 柜温 - 设定温度 */
int   sim_out_freq_step;     /* 频率步长: +3/+2/+1/0/-1/-2/-3 Hz/s */
float sim_out_new_freq;      /* 调整后的频率 */

float sim_out_delta_tcz;     /* 传热温差 = 柜温 - 蒸发温度 */
float sim_out_tcz_err;       /* 传热温差偏差 = 传热温差 - 6.5 */
float sim_out_new_exv;       /* 调整后的膨胀阀开度 */
int   sim_out_exv_action;    /* 0=不动 1=温差调整 2=过热度报警调整 */
int   sim_out_alarm_edt;     /* 1=过热度过低报警 */

/* 循环计数, 方便你知道跑了几圈 */
int   sim_loop_count = 0;

void SIM_RunTest(void)
{
    while(1)  /* <<< 在这行下面设断点 >>> */
    {
        sim_loop_count++;

        /* ============ 频率调节 ============ */
        sim_out_delta_t = sim_in_cabinet_temp - sim_in_set_temp;

        sim_out_freq_step = 0;
        sim_out_new_freq = sim_in_comp_freq;

        if (sim_out_delta_t >= 5.0f) {
            sim_out_freq_step = 3;           /* 温差>=5度: +3Hz/s */
        } else if (sim_out_delta_t >= 2.0f) {
            sim_out_freq_step = 2;           /* 温差2~5度: +2Hz/s */
        } else if (sim_out_delta_t > 0.0f) {
            sim_out_freq_step = 1;           /* 温差0~2度: +1Hz/s */
        } else if (sim_out_delta_t <= -5.0f) {
            sim_out_freq_step = -3;          /* 温差<=-5度: -3Hz/s */
        } else if (sim_out_delta_t <= -2.0f) {
            sim_out_freq_step = -2;          /* 温差-5~-2度: -2Hz/s */
        } else if (sim_out_delta_t < 0.0f) {
            sim_out_freq_step = -1;          /* 温差-2~0度: -1Hz/s */
        }

        sim_out_new_freq = sim_in_comp_freq + sim_out_freq_step;
        if (sim_out_new_freq > 320.0f) sim_out_new_freq = 320.0f;
        if (sim_out_new_freq < 120.0f) sim_out_new_freq = 120.0f;

        /* ============ 膨胀阀调节 ============ */
        sim_out_delta_tcz = sim_in_cabinet_temp - sim_in_evap_temp;
        sim_out_tcz_err = 0;
        sim_out_new_exv = sim_in_exv_opening;
        sim_out_exv_action = 0;
        sim_out_alarm_edt = 0;

        if (sim_out_delta_tcz < 5.0f || sim_out_delta_tcz > 8.0f) {
            /* 传热温差不在[5.0, 8.0]范围 -> 调整阀门 */
            sim_out_tcz_err = sim_out_delta_tcz - 6.5f;
            sim_out_new_exv = sim_in_exv_opening - 0.5f * sim_out_tcz_err;
            sim_out_exv_action = 1;
        } else {
            /* 传热温差OK, 看过热度 */
            if (sim_in_superheat < 5.0f) {
                /* 过热度过低 -> EDT报警, 关小阀门 */
                sim_out_new_exv = sim_in_exv_opening - 0.8f * sim_in_superheat;
                sim_out_exv_action = 2;
                sim_out_alarm_edt = 1;
            } else {
                /* 全部正常, 不调整 */
                sim_out_exv_action = 0;
            }
        }

        if (sim_out_new_exv > 500.0f) sim_out_new_exv = 500.0f;
        if (sim_out_new_exv < 0.0f)   sim_out_new_exv = 0.0f;

        /* >>>>>> 断点设在这里 <<<<<<
         * 每次停下来:
         *   1. 看 sim_out_xxx 的结果
         *   2. 在Watch窗口改 sim_in_xxx 的值
         *   3. 按F5继续, 用新值重新算一遍
         */
    }
}
