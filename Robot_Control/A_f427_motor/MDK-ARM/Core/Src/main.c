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
 * @brief 全向轮底盘运动分配算法
 * @param vx 前进速度（+为前进，-为后退），单位：m/s
 * @param vy 侧向速度（+为左移，-为右移），单位：m/s  
 * @param w 旋转角速度（+为逆时针，-为顺时针），单位：rad/s
 * @return void
 * @note 将底盘三个自由度的运动分解到四个全向轮上，使用低通滤波器平滑速度变化。
 *       考虑电机减速比、前后轮补偿系数以及轮速限制。全向轮运动学基于底盘几何关系计算。
 */
void omni_move(float vx, float vy, float w)
{
    float alpha = 0.2f;
    vx_f = alpha * vx + (1 - alpha) * vx_f;
    vy_f = alpha * vy + (1 - alpha) * vy_f;
    w_f  = alpha * w  + (1 - alpha) * w_f;

    // 根据坐标系统计算每个轮子的目标速度 (电机转轴朝内，全部反向，前后左右都对调)
    // 左前轮：前进速度 + 左移速度 + 逆时针旋转速度 (反向，前后左右对调)
    float wheel1 = -(-vx_f + (-vy_f) + w_f * CHASSIS_RADIUS);  // 使用 -vy_f (左右反转)
    
    // 左后轮：前进速度 - 左移速度 + 逆时针旋转速度 (反向，前后左右对调)
    float wheel2 = -(-vx_f - (-vy_f) + w_f * CHASSIS_RADIUS);  // 使用 vy_f (左右反转)
    
    // 右后轮：-前进速度 - 左移速度 + 逆时针旋转速度 (反向，前后左右对调)
    float wheel3 = -(vx_f - (-vy_f) + w_f * CHASSIS_RADIUS);   // 使用 vy_f (左右反转)
    
    // 右前轮：-前进速度 + 左移速度 + 逆时针旋转速度 (反向，前后左右对调)
    float wheel4 = -(vx_f + (-vy_f) + w_f * CHASSIS_RADIUS);   // 使用 -vy_f (左右反转)

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

/**
 * @brief 底盘前进运动控制
 * @param speed 前进速度，范围：-MAX_WHEEL_SPEED ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘沿x轴正方向（前进）运动，保持其他自由度为0。
 *       速度值越大前进越快，负值为后退。
 */
void move_front(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(speed, 0, 0);
}

/**
 * @brief 底盘后退运动控制
 * @param speed 后退速度，范围：0 ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘沿x轴负方向（后退）运动，保持其他自由度为0。
 *       实际执行时会将speed取负值传给omni_move函数。
 */
void move_back(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(-speed, 0, 0);
}

/**
 * @brief 底盘左移运动控制
 * @param speed 左移速度，范围：0 ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘沿y轴正方向（左移）运动，保持其他自由度为0。
 *       使用全向轮的侧向运动能力实现平移。
 */
void move_left(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, speed, 0);
}

/**
 * @brief 底盘右移运动控制
 * @param speed 右移速度，范围：0 ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘沿y轴负方向（右移）运动，保持其他自由度为0。
 *       实际执行时会将speed取负值传给omni_move函数。
 */
void move_right(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, -speed, 0);
}

/**
 * @brief 底盘右转运动控制
 * @param speed 右转角速度，范围：0 ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘绕z轴顺时针旋转，保持其他自由度为0。
 *       实际执行时会将speed取负值传给omni_move函数实现顺时针旋转。
 */
void turn_right(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, -speed);
}

/**
 * @brief 底盘左转运动控制
 * @param speed 左转角速度，范围：0 ~ +MAX_WHEEL_SPEED
 * @return void
 * @note 控制底盘绕z轴逆时针旋转，保持其他自由度为0。
 *       根据右手坐标系，正值角速度对应逆时针旋转。
 */
void turn_left(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, speed);
}         

/**
 * @brief 底盘停止运动控制
 * @param speed 未使用的参数，保持函数指针接口一致性
 * @return void
 * @note 停止底盘所有运动，将三个自由度的目标速度都设置为0。
 *       调用omni_move(0, 0, 0)使所有电机停止。
 */
void stop(float speed)
{
    // vx_f = 0; vy_f = 0; w_f = 0;  // 禁用：避免强制重置滤波器状态
    omni_move(0, 0, 0);
}

/**
 * @brief 运动方向初始化
 * @param void
 * @return void
 * @note 初始化运动状态函数指针，将底盘初始状态设置为停止。
 *       last_fxfunction用于记录上一次执行的运动函数，便于状态保持和切换。
 */
void Direction_Init(void)
{
	last_fxfunction=stop;
}

/**
 * @brief CAN总线电机数据接收回调函数
 * @param Rx_Buffer 指向CAN接收缓冲区结构体的指针，包含CAN消息头和数据
 * @return void
 * @note 根据CAN消息的标准ID分发到对应的电机对象进行数据处理。
 *       支持ID 0x201-0x204对应motor1-motor4，处理电机反馈的角度、角速度等数据。
 *       该函数在CAN中断中被调用，需要保证执行效率。
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
 * @brief 串口参数调节和运动控制回调函数
 * @param Buffer 接收到的串口数据缓冲区指针
 * @param Length 接收数据的长度
 * @return void
 * @note 处理通过串口发送的PID参数调节和运动控制指令。
 *       支持8个变量：角度PID三参数(pa,ia,da)、角速度PID三参数(po,io,do)、
 *       扭矩设置(torque)和功能选择(fx)。fx值对应：0-停止，1-前进，2-后退，
 *       3-右移，4-左移，5-左转，6-右转。为避免干扰，PID调参时不再自动执行运动。
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
 * @brief 主程序入口函数
 * @param void
 * @return int 程序返回值（通常不会返回）
 * @note 系统初始化和主循环控制。完成HAL库初始化、系统时钟配置、外设初始化、
 *       电机和串口通信初始化。主循环以1ms周期运行，包含运动控制（10ms周期）、
 *       电机PID控制（1ms周期）、CAN通信处理等任务。使用motion_counter实现
 *       不同频率的任务调度，避免高频运动指令造成的干扰。
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
 * @brief 系统时钟配置函数
 * @param void
 * @return void
 * @note 配置STM32F427的系统时钟。使用外部高速晶振(HSE)，通过PLL倍频到180MHz。
 *       配置内容：HSE作为PLL时钟源，PLLM=6，PLLN=180，PLLP=2，PLLQ=4，
 *       启用Over-Drive模式以支持高频率运行。AHB时钟=180MHz，APB1=45MHz，APB2=90MHz。
 *       FLASH延时设置为5个等待周期以匹配高频时钟。
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
 * @brief 系统错误处理函数
 * @param void
 * @return void
 * @note 当系统发生严重错误时调用，禁用所有中断并进入死循环。
 *       用于HAL库函数返回错误状态时的异常处理。在产品开发中，
 *       可在此函数中添加错误日志记录、系统重启或安全模式等处理逻辑。
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
 * @param file 出错的源文件名指针
 * @param line 出错的行号
 * @return void
 * @note 当HAL库或用户代码中的断言条件失败时调用。仅在USE_FULL_ASSERT宏定义时编译。
 *       主要用于调试阶段检查参数有效性和程序逻辑正确性。用户可在此函数中添加
 *       错误信息输出，如通过printf打印文件名和行号，便于定位问题。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */ 