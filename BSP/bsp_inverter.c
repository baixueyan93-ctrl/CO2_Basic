#include "bsp_inverter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *
 *  简化版: ASCII 单字符协议 (9600bps, 8N1)
 *
 *    主控 → 变频板: 每次发送 1 字节 ASCII
 *      'R' 启动, 'S' 停机, '0'~'3' 挡位切换
 *
 *    本版本不做 echo 校验, 发出即认为成功.
 * =================================================================== */

/* ===================================================================
 *  全局变量
 * =================================================================== */
volatile uint8_t InvAckOK = 1;        /* 简化版总是认为成功 */
InvStatus_t g_InvStatus = {0};

/* ===================================================================
 *  内部变量
 * =================================================================== */

/* 互斥锁: 防止多任务同时调用 UART4 (simple_main / panel 按键等) */
static SemaphoreHandle_t s_inv_mutex = NULL;

/* ===================================================================
 *  RS485 方向控制 (PC12)
 *    高 = 发送模式, 低 = 接收模式
 * =================================================================== */
static void RS485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
}

static void RS485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

/* ===================================================================
 *  初始化
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* RS485 默认接收模式 */
    RS485_SetRx();

    /* 使用阻塞式收发, 禁用 UART4 中断避免 Overrun 错误 */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_ERR);

    /* 清空状态 */
    InvAckOK = 1;
    memset(&g_InvStatus, 0, sizeof(g_InvStatus));
    g_InvStatus.echo_ok   = true;
    g_InvStatus.comm_ok   = true;
    g_InvStatus.last_char = 0;

    /* 创建互斥锁 */
    if (s_inv_mutex == NULL) {
        s_inv_mutex = xSemaphoreCreateMutex();
    }
}

/* ===================================================================
 *  BSP_Inverter_SendChar - 发送一个 ASCII 字符给变频板
 *
 *  步骤:
 *    1. 互斥锁加锁 (防止多任务并发)
 *    2. RS485 切到发送模式
 *    3. HAL_UART_Transmit 发 1 字节
 *    4. 等待 TC 标志 (最后一个字节发送完成)
 *    5. RS485 切回接收模式
 *    6. 解锁
 * =================================================================== */
void BSP_Inverter_SendChar(char c)
{
    uint8_t b = (uint8_t)c;

    if (s_inv_mutex != NULL) {
        xSemaphoreTake(s_inv_mutex, portMAX_DELAY);
    }

    RS485_SetTx();
    HAL_UART_Transmit(&huart4, &b, 1, 100);

    /* 等发送寄存器空, 确保最后一位已经发送到线上 */
    while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET) { }

    RS485_SetRx();

    /* 更新状态 */
    g_InvStatus.last_char = c;
    g_InvStatus.echo_ok   = true;
    g_InvStatus.comm_ok   = true;
    g_InvStatus.fail_reason = 0;

    if (s_inv_mutex != NULL) {
        xSemaphoreGive(s_inv_mutex);
    }
}

/* ===================================================================
 *  简化版接口: 启动 / 停机 / 挡位
 * =================================================================== */
void BSP_Inverter_SendRun(void)
{
    BSP_Inverter_SendChar('R');
    g_InvStatus.last_cmd = INV_CMD_START;
}

void BSP_Inverter_SendStop(void)
{
    BSP_Inverter_SendChar('S');
    g_InvStatus.last_cmd     = INV_CMD_STOP;
    g_InvStatus.last_freq_hz = 0;
}

void BSP_Inverter_SendGear(uint8_t gear)
{
    static const char     map[4] = {'0', '1', '2', '3'};
    static const uint16_t hz[4]  = {80, 160, 240, 320};

    if (gear > 3) gear = 3;

    BSP_Inverter_SendChar(map[gear]);
    g_InvStatus.last_cmd     = INV_CMD_SET_FREQ;
    g_InvStatus.last_freq_hz = hz[gear];
}

/* ===================================================================
 *  兼容旧接口: cmd + freq_hz → ASCII 字符
 *
 *  把 cmd/freq 转成最接近的挡位字符:
 *    STOP          → 'S'
 *    START         → 'R' + 稍等 + '0'
 *    SET_FREQ 120~ → 根据 freq_hz 找最近挡位
 * =================================================================== */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz)
{
    if (cmd == INV_CMD_STOP) {
        BSP_Inverter_SendStop();
        return;
    }

    if (cmd == INV_CMD_START) {
        BSP_Inverter_SendRun();
        vTaskDelay(pdMS_TO_TICKS(50));
        BSP_Inverter_SendGear(INV_GEAR_LOW);
        return;
    }

    if (cmd == INV_CMD_SET_FREQ) {
        uint8_t gear;
        if      (freq_hz >= 280) gear = INV_GEAR_HIGH;      /* 320Hz '3' */
        else if (freq_hz >= 200) gear = INV_GEAR_MID_HIGH;  /* 240Hz '2' */
        else if (freq_hz >= 120) gear = INV_GEAR_MID_LOW;   /* 160Hz '1' */
        else                     gear = INV_GEAR_LOW;       /*  80Hz '0' */
        BSP_Inverter_SendGear(gear);
        return;
    }
}

/* ===================================================================
 *  状态读取 (兼容, 简化版无实际回读)
 * =================================================================== */
bool BSP_Inverter_ReadStatus(InvStatus_t *out)
{
    if (out) {
        *out = g_InvStatus;
    }
    return g_InvStatus.comm_ok;
}

/* ===================================================================
 *  UART 接收完成回调
 *
 *  UART4  = 变频器 (简化版阻塞式收发, 此回调不处理)
 *  USART1 = RS485 调试串口 (逐字节中断接收)
 * =================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        /* 简化版变频器通信使用阻塞式, 此回调不处理 */
    }
    else if (huart->Instance == USART1)
    {
        /* RS485 调试串口: 逐字节接收 */
        extern uint8_t rx_buffer[128];
        extern uint16_t rx_index;
        extern volatile uint8_t rx_complete;
        extern uint8_t rx_byte;

        rx_buffer[rx_index++] = rx_byte;

        if (rx_byte == '\n' || rx_byte == '\r' || rx_index >= 127) {
            rx_buffer[rx_index] = '\0';
            rx_complete = 1;
        } else {
            extern UART_HandleTypeDef huart1;
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
}
