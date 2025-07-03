/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_optimized.c
  * @brief          : 优化版主程序 - 解决轮子间干扰问题
  * @author         : AI Assistant  
  * @date           : 2025-01-XX
  * @note           : 重点解决：串口调参干扰、运动切换突变、频繁函数调用
  ******************************************************************************
  */
/* USER CODE END Header */

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
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 全向轮运动学参数
#define CHASSIS_RADIUS      0.2f        // 底盘半径 (m)
#define FRONT_SCALE         1.05f       // 前轮补偿系数
#define BACK_SCALE          1.0f        // 后轮补偿系数
#define GEAR_RATIO          19.0f       // 电机减速比
#define MAX_WHEEL_SPEED     500.0f      // 最大轮速 (rad/s)

// 控制周期参数
#define MOTION_UPDATE_PERIOD_MS   10    // 运动更新周期(ms)
#define PID_UPDATE_PERIOD_MS      1     // PID更新周期(ms)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
Class_Serialplot serialplot;
Class_Motor_C620 motor1, motor2, motor3, motor4;

float Target_Angle, Now_Angle, Target_Omega, Now_Omega;
float torque = 0.0f, fx = 0.0f, target_torque = 0.0f;

// 运动状态管理
typedef enum {
    MOTION_STOP = 0,
    MOTION_FRONT,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT
} Motion_State_t;

static Motion_State_t current_motion_state = MOTION_STOP;
static Motion_State_t last_motion_state = MOTION_STOP;

// 速度平滑滤波变量 (全局状态，不随运动切换重置)
static float vx_filtered = 0.0f, vy_filtered = 0.0f, w_filtered = 0.0f;
static float target_vx = 0.0f, target_vy = 0.0f, target_w = 0.0f;

// 时间管理
static uint32_t motion_update_counter = 0;
static uint32_t pid_update_counter = 0;

// 串口调参变量
static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "pa", "ia", "da",   // 角度PID参数
    "po", "io", "do",   // 速度PID参数
    "torque", "fx",     // 扭矩和方向控制
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Motion_Targets(void);
void Update_Omni_Kinematics(void);
void Apply_PID_Parameter(uint8_t param_index, float value);
void Set_Motion_State(Motion_State_t new_state, float speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 更新运动目标值（根据当前运动状态）
 * @note 只在状态切换时调用，避免频繁计算
 */
void Update_Motion_Targets(void)
{
    // 只在状态改变时更新目标值
    if (current_motion_state == last_motion_state) {
        return;
    }
    
    switch (current_motion_state) {
        case MOTION_FRONT:
            target_vx = torque; target_vy = 0.0f; target_w = 0.0f;
            break;
        case MOTION_BACK:
            target_vx = -torque; target_vy = 0.0f; target_w = 0.0f;
            break;
        case MOTION_LEFT:
            target_vx = 0.0f; target_vy = torque; target_w = 0.0f;
            break;
        case MOTION_RIGHT:
            target_vx = 0.0f; target_vy = -torque; target_w = 0.0f;
            break;
        case MOTION_TURN_LEFT:
            target_vx = 0.0f; target_vy = 0.0f; target_w = torque;
            break;
        case MOTION_TURN_RIGHT:
            target_vx = 0.0f; target_vy = 0.0f; target_w = -torque;
            break;
        case MOTION_STOP:
        default:
            target_vx = 0.0f; target_vy = 0.0f; target_w = 0.0f;
            break;
    }
    
    last_motion_state = current_motion_state;
}

/**
 * @brief 全向轮运动学解算与速度平滑
 * @note 定期调用，实现平滑的速度过渡
 */
void Update_Omni_Kinematics(void)
{
    const float alpha = 0.15f;  // 平滑滤波系数
    
    // 平滑滤波到目标速度
    vx_filtered = alpha * target_vx + (1.0f - alpha) * vx_filtered;
    vy_filtered = alpha * target_vy + (1.0f - alpha) * vy_filtered;
    w_filtered = alpha * target_w + (1.0f - alpha) * w_filtered;
    
    // 全向轮运动学解算
    float wheel1 = vx_filtered + vy_filtered + w_filtered * CHASSIS_RADIUS;  // 左前
    float wheel2 = vx_filtered - vy_filtered + w_filtered * CHASSIS_RADIUS;  // 左后
    float wheel3 = -vx_filtered - vy_filtered + w_filtered * CHASSIS_RADIUS; // 右后
    float wheel4 = -vx_filtered + vy_filtered + w_filtered * CHASSIS_RADIUS; // 右前
    
    // 速度归一化
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
    wheel1 *= FRONT_SCALE;
    wheel4 *= FRONT_SCALE;
    wheel2 *= BACK_SCALE;
    wheel3 *= BACK_SCALE;
    
    // 设置电机目标速度
    motor1.Set_Target_Omega(wheel1 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor2.Set_Target_Omega(wheel2 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor3.Set_Target_Omega(wheel3 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor4.Set_Target_Omega(wheel4 * MAX_WHEEL_SPEED / GEAR_RATIO);
}

/**
 * @brief 应用PID参数（优化版 - 不触发运动）
 * @param param_index 参数索引
 * @param value 参数值
 * @note 只调整参数，不重新执行运动
 */
void Apply_PID_Parameter(uint8_t param_index, float value)
{
    switch (param_index) {
        case 0: // pa - 角度PID比例
            motor1.PID_Angle.Set_K_P(value);
            motor2.PID_Angle.Set_K_P(value);
            motor3.PID_Angle.Set_K_P(value);
            motor4.PID_Angle.Set_K_P(value);
            break;
        case 1: // ia - 角度PID积分
            motor1.PID_Angle.Set_K_I(value);
            motor2.PID_Angle.Set_K_I(value);
            motor3.PID_Angle.Set_K_I(value);
            motor4.PID_Angle.Set_K_I(value);
            break;
        case 2: // da - 角度PID微分
            motor1.PID_Angle.Set_K_D(value);
            motor2.PID_Angle.Set_K_D(value);
            motor3.PID_Angle.Set_K_D(value);
            motor4.PID_Angle.Set_K_D(value);
            break;
        case 3: // po - 速度PID比例
            motor1.PID_Omega.Set_K_P(value);
            motor2.PID_Omega.Set_K_P(value);
            motor3.PID_Omega.Set_K_P(value);
            motor4.PID_Omega.Set_K_P(value);
            break;
        case 4: // io - 速度PID积分
            motor1.PID_Omega.Set_K_I(value);
            motor2.PID_Omega.Set_K_I(value);
            motor3.PID_Omega.Set_K_I(value);
            motor4.PID_Omega.Set_K_I(value);
            break;
        case 5: // do - 速度PID微分
            motor1.PID_Omega.Set_K_D(value);
            motor2.PID_Omega.Set_K_D(value);
            motor3.PID_Omega.Set_K_D(value);
            motor4.PID_Omega.Set_K_D(value);
            break;
        default:
            break;
    }
}

/**
 * @brief 设置运动状态
 * @param new_state 新的运动状态
 * @param speed 运动速度
 * @note 状态机管理，避免频繁切换
 */
void Set_Motion_State(Motion_State_t new_state, float speed)
{
    if (current_motion_state != new_state) {
        current_motion_state = new_state;
        torque = speed;
    }
}

/**
 * @brief CAN电机回调函数（优化版）
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId) {
        case 0x201:
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case 0x202:
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case 0x203:
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        case 0x204:
            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
            break;
        default:
            break;
    }
}

/**
 * @brief 串口调参回调函数（优化版）
 * @note 关键改进：参数调整不触发运动重新执行
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    uint8_t var_index = serialplot.Get_Variable_Index();
    float var_value = serialplot.Get_Variable_Value();
    
    if (var_index <= 5) {
        // PID参数调整：只调参数，不触发运动
        Apply_PID_Parameter(var_index, var_value);
    } else if (var_index == 6) {
        // 扭矩/速度设置
        torque = var_value;
    } else if (var_index == 7) {
        // 方向控制
        fx = var_value;
        if(fx == 0.0f) {
            Set_Motion_State(MOTION_STOP, torque);
        } else if(fx == 1.0f) {
            Set_Motion_State(MOTION_FRONT, torque);
        } else if(fx == 2.0f) {
            Set_Motion_State(MOTION_BACK, torque);
        } else if(fx == 3.0f) {
            Set_Motion_State(MOTION_LEFT, torque);
        } else if(fx == 4.0f) {
            Set_Motion_State(MOTION_RIGHT, torque);
        } else if(fx == 5.0f) {
            Set_Motion_State(MOTION_TURN_LEFT, torque);
        } else if(fx == 6.0f) {
            Set_Motion_State(MOTION_TURN_RIGHT, torque);
        }
    }
}

/* USER CODE END 0 */

/**
 * @brief 主程序入口（优化版）
 */
int main(void)
{
    /* MCU初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    // 通信初始化
    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
    serialplot.Init(&huart2, 8, (char **)Variable_Assignment_List);
    
    // 电机PID参数初始化（保守参数，减少振荡）
    motor1.PID_Omega.Init(60.0f, 0.5f, 3.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor2.PID_Omega.Init(60.0f, 0.5f, 3.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor3.PID_Omega.Init(60.0f, 0.5f, 3.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    motor4.PID_Omega.Init(60.0f, 0.5f, 3.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
    
    // 电机初始化
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 1.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA, 1.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 1.0f);
    motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 1.0f);
    
    // 初始状态
    current_motion_state = MOTION_STOP;
    /* USER CODE END 2 */

    /* 主循环（优化版） */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // 运动目标更新（仅在状态切换时）
        if (motion_update_counter >= MOTION_UPDATE_PERIOD_MS) {
            Update_Motion_Targets();
            Update_Omni_Kinematics();
            motion_update_counter = 0;
        }
        
        // PID控制更新（每1ms）
        if (pid_update_counter >= PID_UPDATE_PERIOD_MS) {
            motor1.TIM_PID_PeriodElapsedCallback();
            motor2.TIM_PID_PeriodElapsedCallback();
            motor3.TIM_PID_PeriodElapsedCallback();
            motor4.TIM_PID_PeriodElapsedCallback();
            pid_update_counter = 0;
        }
        
        // 通信回调
        TIM_CAN_PeriodElapsedCallback();
        
        // 计数器递增
        motion_update_counter++;
        pid_update_counter++;
        
        // 延时1ms
        HAL_Delay(1);
        /* USER CODE END WHILE */
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
}
#endif 