#include "paramsusart.h"


// **********************************
// *   配置项 (可根据需要修改)
// **********************************
// 定义使用的UART句柄
#define UART_HANDLE huart1
// 接收缓冲区大小
#define RX_BUFFER_SIZE 128
// **********************************


// UART 句柄的外部声明
extern UART_HandleTypeDef UART_HANDLE;

// 接收缓冲区
static uint8_t g_rx_buffer[RX_BUFFER_SIZE];
// 单字节接收
static uint8_t g_rx_byte;
// 接收缓冲区写入位置
static volatile uint16_t g_rx_write_pos = 0;
// 命令接收完成标志
static volatile uint8_t g_cmd_received_flag = 0;

MotorData G_Usart_Motor_Data[4];
float G_Usart_ExpectSpeed_Data[4];

// 内部函数声明
static void uart_printf(const char *format, ...);
static void print_help(void);
static void handle_set_command(const char *param, float value);
static void handle_get_command(const char *param);
static void handle_move_command(const char *type, float value1, float value2);
void Uart_SendMotorSpeedWave(void);


void Uart_Init(void)
{
    // 启动第一次UART接收中断，之后在回调中循环启动
    HAL_UART_Receive_IT(&UART_HANDLE, &g_rx_byte, 1);
    uart_printf("Serial parameter tuning ready.\r\nType 'help' for commands.\r\n");
}

void Uart_ProcessCommand(void)
{
    if (g_cmd_received_flag)
    {
        char cmd_buffer[RX_BUFFER_SIZE];
        strncpy(cmd_buffer, (char *)g_rx_buffer, RX_BUFFER_SIZE);

        // 清理缓冲区和标志位，准备下一次接收
        g_rx_write_pos = 0;
        memset(g_rx_buffer, 0, RX_BUFFER_SIZE);
        g_cmd_received_flag = 0;

        char command[10] = {0};
        char param[10] = {0};
        float value1 = 0;
        float value2 = 0;

        // 新增move命令解析
        if (sscanf(cmd_buffer, "%s %s %f %f", command, param, &value1, &value2) == 4)
        {
            if (strcmp(command, "move") == 0)
            {
                handle_move_command(param, value1, value2);
            }
            else
            {
                uart_printf("Error: Invalid command format.\r\n");
            }
        }
        else if (sscanf(cmd_buffer, "%s %s %f", command, param, &value1) == 3)
        {
            if (strcmp(command, "move") == 0)
            {
                handle_move_command(param, value1, 0);
            }
            else if (strcmp(command, "set") == 0)
            {
                handle_set_command(param, value1);
            }
            else
            {
                uart_printf("Error: Invalid command format.\r\n");
            }
        }
        else if (sscanf(cmd_buffer, "%s %s", command, param) == 2)
        {
            if (strcmp(command, "move") == 0)
            {
                handle_move_command(param, 0, 0);
            }
            else if (strcmp(command, "get") == 0)
            {
                handle_get_command(param);
            }
            else
            {
                uart_printf("Error: Invalid command format.\r\n");
            }
        }
        else if (sscanf(cmd_buffer, "%s", command) == 1)
        {
            if (strcmp(command, "help") == 0)
            {
                print_help();
            }
            else
            {
                uart_printf("Error: Unknown command '%s'.\r\n", command);
            }
        }
    }
}

/**
 * @brief UART接收完成中断回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (g_cmd_received_flag == 0 && g_rx_write_pos < RX_BUFFER_SIZE)
        {
            // 如果收到了回车或换行符，则认为一条命令结束
            if (g_rx_byte == '\r' || g_rx_byte == '\n')
            {
                if (g_rx_write_pos > 0) // 忽略空命令
                {
                    g_rx_buffer[g_rx_write_pos] = '\0'; // 添加字符串结束符
                    g_cmd_received_flag = 1;
                }
                else
                {
                    g_rx_write_pos = 0; // 清空以便接收下一条
                }
            }
            else
            {
                g_rx_buffer[g_rx_write_pos++] = g_rx_byte;
                // 超长保护：如果缓冲区满，强制命令结束
                if (g_rx_write_pos >= RX_BUFFER_SIZE - 1) {
                    g_rx_buffer[g_rx_write_pos] = '\0';
                    g_cmd_received_flag = 1;
                }
            }
        }
        // 再次启动接收中断，准备接收下一个字节
        HAL_UART_Receive_IT(&UART_HANDLE, &g_rx_byte, 1);
    }
}


static void handle_set_command(const char *param, float value)
{
    int updated = 1;
    if (strcmp(param, "sp_p") == 0) G_Speed_PID_Params.Kp = value;
    else if (strcmp(param, "sp_i") == 0) G_Speed_PID_Params.Ki = value;
    else if (strcmp(param, "sp_d") == 0) G_Speed_PID_Params.Kd = value;
    else if (strcmp(param, "sp_il") == 0) G_Speed_PID_Params.IntegralLimit = value;
    else if (strcmp(param, "sp_ol") == 0) G_Speed_PID_Params.OutputLimit = value;

    else if (strcmp(param, "dp_p") == 0) G_DifSpeed_PID_Params.Kp = value;
    else if (strcmp(param, "dp_i") == 0) G_DifSpeed_PID_Params.Ki = value;
    else if (strcmp(param, "dp_d") == 0) G_DifSpeed_PID_Params.Kd = value;
    else if (strcmp(param, "dp_il") == 0) G_DifSpeed_PID_Params.IntegralLimit = value;
    else if (strcmp(param, "dp_ol") == 0) G_DifSpeed_PID_Params.OutputLimit = value;

    else if (strcmp(param, "ap_p") == 0) G_Angle_PID_Params.Kp = value;
    else if (strcmp(param, "ap_i") == 0) G_Angle_PID_Params.Ki = value;
    else if (strcmp(param, "ap_d") == 0) G_Angle_PID_Params.Kd = value;
    else if (strcmp(param, "ap_il") == 0) G_Angle_PID_Params.IntegralLimit = value;
    else if (strcmp(param, "ap_ol") == 0) G_Angle_PID_Params.OutputLimit = value;
    
    else {
        updated = 0;
        uart_printf("Error: Unknown parameter '%s'.\r\n", param);
    }

    if (updated) {
        uart_printf("OK. %s set to %f.\r\n", param, value);
    }
}

static void handle_get_command(const char *param)
{
    float value = 0;
    int found = 1;
    if (strcmp(param, "sp_p") == 0) value = G_Speed_PID_Params.Kp;
    else if (strcmp(param, "sp_i") == 0) value = G_Speed_PID_Params.Ki;
    else if (strcmp(param, "sp_d") == 0) value = G_Speed_PID_Params.Kd;
    else if (strcmp(param, "sp_il") == 0) value = G_Speed_PID_Params.IntegralLimit;
    else if (strcmp(param, "sp_ol") == 0) value = G_Speed_PID_Params.OutputLimit;

    else if (strcmp(param, "dp_p") == 0) value = G_DifSpeed_PID_Params.Kp;
    else if (strcmp(param, "dp_i") == 0) value = G_DifSpeed_PID_Params.Ki;
    else if (strcmp(param, "dp_d") == 0) value = G_DifSpeed_PID_Params.Kd;
    else if (strcmp(param, "dp_il") == 0) value = G_DifSpeed_PID_Params.IntegralLimit;
    else if (strcmp(param, "dp_ol") == 0) value = G_DifSpeed_PID_Params.OutputLimit;

    else if (strcmp(param, "ap_p") == 0) value = G_Angle_PID_Params.Kp;
    else if (strcmp(param, "ap_i") == 0) value = G_Angle_PID_Params.Ki;
    else if (strcmp(param, "ap_d") == 0) value = G_Angle_PID_Params.Kd;
    else if (strcmp(param, "ap_il") == 0) value = G_Angle_PID_Params.IntegralLimit;
    else if (strcmp(param, "ap_ol") == 0) value = G_Angle_PID_Params.OutputLimit;
    
    else {
        found = 0;
        uart_printf("Error: Unknown parameter '%s'.\r\n", param);
    }
    
    if (found) {
        uart_printf("%s = %f\r\n", param, value);
    }
}

static void print_help(void)
{
    uart_printf("--- Parameter Tuning Help ---\r\n");
    uart_printf("Usage:\r\n");
    uart_printf("  set <param> <value>   e.g., set sp_p 1.5\r\n");
    uart_printf("  get <param>           e.g., get sp_p\r\n");
    uart_printf("\r\nParameters:\r\n");
    uart_printf("  sp_*: Speed PID (p, i, d, il, ol)\r\n");
    uart_printf("  dp_*: Difference PID (p, i, d, il, ol)\r\n");
    uart_printf("  ap_*: Angle PID (p, i, d, il, ol)\r\n");
    uart_printf("---------------------------\r\n");
}

/**
 * @brief 实现一个简易的printf，通过UART发送
 */
static void uart_printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&UART_HANDLE, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

static void handle_move_command(const char *type, float value1, float value2)
{
    if (strcmp(type, "linear") == 0)
    {
        wheelChassis_Linear(value1, value2);
        uart_printf("OK. Linear: speed=%f, angle=%f\r\n", value1, value2);
    }
    else if (strcmp(type, "turn1") == 0)
    {
        wheelChassis_Turn1(value1);
        uart_printf("OK. Turn1: speed=%f\r\n", value1);
    }
    else if (strcmp(type, "turn2") == 0)
    {
        wheelChassis_Turn2(value1);
        uart_printf("OK. Turn2: angle=%f\r\n", value1);
    }
    else if (strcmp(type, "stop") == 0)
    {
        wheelChassis_Stop();
        uart_printf("OK. Stop.\r\n");
    }
    else
    {
        uart_printf("Error: Unknown move type '%s'.\r\n", type);
    }
}

void Uart_SendMotorSpeedWave(void)
{
    uart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                G_Usart_Motor_Data[0].speed,
                G_Usart_Motor_Data[1].speed,
                G_Usart_Motor_Data[2].speed,
                G_Usart_Motor_Data[3].speed,
                G_Usart_ExpectSpeed_Data[0],
                G_Usart_ExpectSpeed_Data[1],
                G_Usart_ExpectSpeed_Data[2],
                G_Usart_ExpectSpeed_Data[3]);
}


