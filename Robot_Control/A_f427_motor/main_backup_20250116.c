/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
// #include "chassis_contral.h"  // 禁用同步控制系统

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 全向轮运动学参数
#define CHASSIS_RADIUS      0.2f        // 底盘半径 (m)
#define FRONT_SCALE         1.05f       // 前轮补偿系数
#define BACK_SCALE          1.0f        // 后轮补偿系数
#define GEAR_RATIO          19.0f       // 电机减速比
#define MAX_WHEEL_SPEED     500.0f      // 最大轮速 (rad/s)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Class_Serialplot serialplot;
Class_Motor_C620 motor1;  // 左前电机
Class_Motor_C620 motor2;  // 左后电机
Class_Motor_C620 motor3;  // 右后电机
Class_Motor_C620 motor4;  // 右前电机

float Target_Angle, Now_Angle, Target_Omega, Now_Omega;
float torque, fx=0, target_torque;

// 速度平滑滤波变量
float vx_f = 0.0f, vy_f = 0.0f, w_f = 0.0f;

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    //电机调PID
	//角度
    "pa",
    "ia",
    "da",
	//速度
    "po",
    "io",
    "do",
		"torque",
		"fx",
};

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
 * @brief 全向轮底盘运动分配
 * @param vx 前进速度（+为前进，-为后退）
 * @param vy 侧向速度（+为左移，-为右移）
 * @param w 旋转角速度（+为逆时针，-为顺时针）
 */
void omni_move(float vx, float vy, float w)
{
    float alpha = 0.2f;
    vx_f = alpha * vx + (1 - alpha) * vx_f;
    vy_f = alpha * vy + (1 - alpha) * vy_f;
    w_f  = alpha * w  + (1 - alpha) * w_f;

    // 根据坐标系统计算每个轮子的目标速度
    // 左前轮：前进速度 + 左移速度 + 逆时针旋转速度
    float wheel1 = vx_f + vy_f + w_f * CHASSIS_RADIUS;
    
    // 左后轮：前进速度 - 左移速度 + 逆时针旋转速度
    float wheel2 = vx_f - vy_f + w_f * CHASSIS_RADIUS;
    
    // 右后轮：-前进速度 - 左移速度 + 逆时针旋转速度
    float wheel3 = -vx_f - vy_f + w_f * CHASSIS_RADIUS;
    
    // 右前轮：-前进速度 + 左移速度 + 逆时针旋转速度
    float wheel4 = -vx_f + vy_f + w_f * CHASSIS_RADIUS;

    float max_val = fabsf(wheel1);
    if(fabsf(wheel2) > max_val) max_val = fabsf(wheel2);
    if(fabsf(wheel3) > max_val) max_val = fabsf(wheel3);
    if(fabsf(wheel4) > max_val) max_val = fabsf(wheel4);
    if(max_val > 10.0f) {
        wheel1 /= max_val;
        wheel2 /= max_val;
        wheel3 /= max_val;
        wheel4 /= max_val;
    }

    // 应用前轮补偿系数
    wheel1 *= FRONT_SCALE;  // 左前轮
    wheel4 *= FRONT_SCALE;  // 右前轮
    
    // 应用后轮补偿系数
    wheel2 *= BACK_SCALE;   // 左后轮
    wheel3 *= BACK_SCALE;   // 右后轮

    // 考虑减速比，将目标角速度转换为电机实际转速
    motor1.Set_Target_Omega(wheel1 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor2.Set_Target_Omega(wheel2 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor3.Set_Target_Omega(wheel3 * MAX_WHEEL_SPEED / GEAR_RATIO);
    motor4.Set_Target_Omega(wheel4 * MAX_WHEEL_SPEED / GEAR_RATIO);
}

void (*last_fxfunction)(float speed);//上次运行的状态

//前进
void move_front(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(speed, 0, 0);
}

//后退
void move_back(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(-speed, 0, 0);
}

//向左
void move_left(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, speed, 0);
}

//向右
void move_right(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, -speed, 0);
}

//右转
void turn_right(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, -speed);
}

//左转
void turn_left(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, speed);
}         

//停止
void stop(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, 0);
}

//方向初始化
void Direction_Init(void)
{
	last_fxfunction=stop;
}

/**
 * @brief CAN报文回调函数
 * @param Rx_Buffer CAN接收的信息结构体
 * @return void
 * @note 处理不同ID的电机CAN数据。
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
        {
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x203):
        {
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x204):
        {
            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}

/**
 * @brief 串口数据回调函数
 * @param Buffer 接收缓冲区
 * @param Length 数据长度
 * @return void
 * @note 处理串口调参和运动指令。
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
        serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        // 电机调PID
        case(0):
        {
            motor1.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
						
        }
        break;
        case(1):
        {
            motor1.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(2):
        {
            motor1.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(3):
        {
            motor1.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
					
        }
        break;
        case(4):
        {
            motor1.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(5):
        {
            motor1.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
				case(6):
        {
            torque=serialplot.Get_Variable_Value();
						// last_fxfunction(torque);  // 禁用：避免设置扭矩时重新执行运动
        }
        break;
				case(7):
        {
            fx=serialplot.Get_Variable_Value();
            if(fx==0.0f){stop(0); last_fxfunction=stop;}
            else if(fx==1.0f){move_front(torque); last_fxfunction=move_front;}
            else if(fx==2.0f){move_back(torque); last_fxfunction=move_back;}
            else if(fx==3.0f){move_right(torque); last_fxfunction=move_right;}
            else if(fx==4.0f){move_left(torque); last_fxfunction=move_left;}
            else if(fx==5.0f){turn_left(torque); last_fxfunction=turn_left;}
            else if(fx==6.0f){turn_right(torque); last_fxfunction=turn_right;}
            else{last_fxfunction(torque);}
        }
        break;
    }

}

/* USER CODE END 0 */

/**
 * @brief 主程序入口
 * @return int 程序返回值
 * @note MCU初始化与主循环。
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);

    serialplot.Init(&huart2, 8, (char **)Variable_Assignment_List);
		
		Direction_Init();
		
    // 电机PID参数初始化 - 优化参数减少振荡和干扰
    motor1.PID_Omega.Init(80.0f, 0.0f, 0.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
		motor2.PID_Omega.Init(80.0f, 0.0f, 0.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
		motor3.PID_Omega.Init(80.0f, 0.0f, 0.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);
		motor4.PID_Omega.Init(80.0f, 0.0f, 0.0f, 300.0f, 2000.0f, 8000.0f, 0.001f, 1.0f);  
		
    // 电机初始化 - 使用速度控制模式
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_OMEGA, 1.0f);
		motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_OMEGA, 1.0f);
		motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA, 1.0f);
		motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA, 1.0f);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t motion_counter = 0;  // 运动更新计数器
  
  while (1)
  {
    // 降低运动函数调用频率，每10ms执行一次，减少干扰
    if(motion_counter >= 10) {
        last_fxfunction(torque);
        motion_counter = 0;
    }

    // 串口绘图显示内容（可选）
    /*
    Target_Angle = motor1.Get_Target_Angle();
    Now_Angle = motor1.Get_Now_Angle();
    Target_Omega = motor1.Get_Target_Omega();
    Now_Omega = motor1.Get_Now_Omega();
    serialplot.Set_Data(4, &Target_Angle, &Now_Angle, &Target_Omega, &Now_Omega);
    serialplot.TIM_Write_PeriodElapsedCallback();
    */

    // 单独更新每个电机的PID控制 - 不使用同步控制
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.TIM_PID_PeriodElapsedCallback();

    // 通信设备回调数据
    TIM_CAN_PeriodElapsedCallback();
    //TIM_UART_PeriodElapsedCallback();

    // 计数器递增
    motion_counter++;
    
    // 延时1ms
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief 系统时钟配置
 * @return void
 * @note 配置MCU系统时钟。
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief 错误处理函数
 * @return void
 * @note 进入死循环，关闭中断。
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief 断言失败处理函数
 * @param file 源文件名
 * @param line 出错行号
 * @return void
 * @note 用户可自定义错误报告。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */ 