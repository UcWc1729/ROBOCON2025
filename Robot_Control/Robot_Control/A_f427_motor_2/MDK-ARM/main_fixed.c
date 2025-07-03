/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (修正版)
  * @brief          : 解决全向轮底盘轮子间干扰问题的主程序
  * @author         : AI Assistant
  * @date           : 2025-01-XX
  * @note           : 禁用电机同步控制，使用正确的全向轮运动学解算
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dvc_serialplot.h"
#include "dvc_motor.h"
#include "drv_math.h"
// 注意：禁用chassis_contral.h以避免同步控制干扰
// #include "chassis_contral.h"  // 已禁用
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 全向轮运动学参数
#define CHASSIS_RADIUS      0.2f        // 底盘半径 (m)
#define FRONT_SCALE         1.05f       // 前轮补偿系数
#define BACK_SCALE          1.0f        // 后轮补偿系数
#define GEAR_RATIO          19.0f       // 电机减速比
#define MAX_WHEEL_SPEED     500.0f      // 最大轮速 (rad/s)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
Class_Serialplot serialplot;
Class_Motor_C620 motor1;  // 左前电机 (0x201)
Class_Motor_C620 motor2;  // 左后电机 (0x202)
Class_Motor_C620 motor3;  // 右后电机 (0x203)
Class_Motor_C620 motor4;  // 右前电机 (0x204)

float Target_Angle, Now_Angle, Target_Omega, Now_Omega;
float torque, fx=0, target_torque;

// 速度平滑滤波变量
float vx_f = 0.0f, vy_f = 0.0f, w_f = 0.0f;

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "pa", "ia", "da",   // 角度PID参数
    "po", "io", "do",   // 速度PID参数
    "torque", "fx",     // 扭矩和方向控制
};

void (*last_fxfunction)(float speed); // 上次运行的状态
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 全向轮运动控制函数声明
void omni_move(float vx, float vy, float w);
void move_front(float speed);
void move_back(float speed);
void move_left(float speed);
void move_right(float speed);
void turn_left(float speed);
void turn_right(float speed);
void stop(float speed);
void Direction_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 全向轮底盘运动分配 (修正版)
 * @param vx 前进速度（+为前进，-为后退）
 * @param vy 侧向速度（+为左移，-为右移）
 * @param w 旋转角速度（+为逆时针，-为顺时针）
 * @note 使用正确的全向轮运动学解算，避免轮子间干扰
 */
void omni_move(float vx, float vy, float w)
{
    // 平滑滤波系数
    float alpha = 0.2f;
    vx_f = alpha * vx + (1 - alpha) * vx_f;
    vy_f = alpha * vy + (1 - alpha) * vy_f;
    w_f  = alpha * w  + (1 - alpha) * w_f;

    // 根据全向轮运动学计算每个轮子的目标速度
    // 左前轮 (motor1): 前进 + 左移 + 逆时针旋转
    float wheel1 = vx_f + vy_f + w_f * CHASSIS_RADIUS;
    
    // 左后轮 (motor2): 前进 - 左移 + 逆时针旋转
    float wheel2 = vx_f - vy_f + w_f * CHASSIS_RADIUS;
    
    // 右后轮 (motor3): -前进 - 左移 + 逆时针旋转
    float wheel3 = -vx_f - vy_f + w_f * CHASSIS_RADIUS;
    
    // 右前轮 (motor4): -前进 + 左移 + 逆时针旋转
    float wheel4 = -vx_f + vy_f + w_f * CHASSIS_RADIUS;

    // 速度归一化（保持方向，限制幅值）
    float max_val = fabsf(wheel1);
    if(fabsf(wheel2) > max_val) max_val = fabsf(wheel2);
    if(fabsf(wheel3) > max_val) max_val = fabsf(wheel3);
    if(fabsf(wheel4) > max_val) max_val = fabsf(wheel4);
    
    if(max_val > 1.0f) {
        wheel1 /= max_val;
        wheel2 /= max_val;
        wheel3 /= max_val;
        wheel4 /= max_val;
    }

    // 应用补偿系数
    wheel1 *= FRONT_SCALE;  // 左前轮
    wheel4 *= FRONT_SCALE;  // 右前轮
    wheel2 *= BACK_SCALE;   // 左后轮
    wheel3 *= BACK_SCALE;   // 右后轮

    // 直接设置目标速度，不经过同步控制系统
    motor1.Set_Target_Omega(wheel1 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor2.Set_Target_Omega(wheel2 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor3.Set_Target_Omega(wheel3 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor4.Set_Target_Omega(wheel4 * MAX_WHEEL_SPEED / GEAR_RATIO);
}

// 运动控制函数（修正版）
void move_front(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;  // 清除滤波器状态
    omni_move(speed, 0, 0);
}

void move_back(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(-speed, 0, 0);
}

void move_left(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(0, speed, 0);
}

void move_right(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(0, -speed, 0);
}

void turn_left(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(0, 0, speed);
}

void turn_right(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(0, 0, -speed);
}

void stop(float speed)
{
    vx_f = 0; vy_f = 0; w_f = 0;
    omni_move(0, 0, 0);
}

void Direction_Init(void)
{
    last_fxfunction = stop;
}

/**
 * @brief CAN报文回调函数 (修正版)
 * @param Rx_Buffer CAN接收的信息结构体
 * @note 处理不同ID的电机CAN数据，确保正确的case语句
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case (0x202):
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case (0x203):
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case (0x204):
            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        default:
            break;
    }
}

/**
 * @brief 串口数据回调函数 (修正版)
 * @param Buffer 接收缓冲区
 * @param Length 数据长度
 * @note 处理串口调参和运动指令，不使用同步控制
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        // 角度PID参数调整
        case(0): // pa
            motor1.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
            motor2.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
            motor3.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
            motor4.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        case(1): // ia
            motor1.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor4.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        case(2): // da
            motor1.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            motor4.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        // 速度PID参数调整
        case(3): // po
            motor1.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            motor4.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        case(4): // io
            motor1.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            motor4.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        case(5): // do
            motor1.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            motor4.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            last_fxfunction(torque);
            break;
        case(6): // torque
            torque = serialplot.Get_Variable_Value();
            last_fxfunction(torque);
            break;
        case(7): // fx - 方向控制
            fx = serialplot.Get_Variable_Value();
            if(fx == 0.0f) {
                stop(torque);
            } else if(fx == 1.0f) {
                move_front(torque);
                last_fxfunction = move_front;
            } else if(fx == 2.0f) {
                move_back(torque);
                last_fxfunction = move_back;
            } else if(fx == 3.0f) {
                move_left(torque);
                last_fxfunction = move_left;
            } else if(fx == 4.0f) {
                move_right(torque);
                last_fxfunction = move_right;
            } else if(fx == 5.0f) {
                turn_left(torque);
                last_fxfunction = turn_left;
            } else if(fx == 6.0f) {
                turn_right(torque);
                last_fxfunction = turn_right;
            } else {
                last_fxfunction(torque);
            }
            break;
        default:
            break;
    }
}

/* USER CODE END 0 */

/**
 * @brief 主程序入口 (修正版)
 * @return int 程序返回值
 * @note 不使用电机同步控制系统，避免轮子间干扰
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU配置 */
    HAL_Init();
    SystemClock_Config();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    // 通信初始化
    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    serialplot.Init(&huart2, 8, (char **)Variable_Assignment_List);
    
    // 方向控制初始化
    Direction_Init();
    
    // 电机PID参数初始化 (使用较为保守的参数避免振荡)
    motor1.PID_Omega.Init(80.0f, 0.0f, 5.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor2.PID_Omega.Init(80.0f, 0.0f, 5.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor3.PID_Omega.Init(80.0f, 0.0f, 5.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor4.PID_Omega.Init(80.0f, 0.0f, 5.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    
    // 电机初始化 (仅使用速度控制模式)
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 1.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA, 1.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 1.0f);
    motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 1.0f);
    /* USER CODE END 2 */

    /* 主循环 */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 执行当前运动指令
        last_fxfunction(torque);

        // 独立更新每个电机的PID控制
        // 关键修改：不调用Motor_Speed_Sync_Control函数
        motor1.TIM_PID_PeriodElapsedCallback();
        motor2.TIM_PID_PeriodElapsedCallback();
        motor3.TIM_PID_PeriodElapsedCallback();
        motor4.TIM_PID_PeriodElapsedCallback();

        // 通信回调
        TIM_CAN_PeriodElapsedCallback();

        // 延时
        HAL_Delay(1);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief 系统时钟配置
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief 错误处理函数
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */ 