/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "drv_bsp.h"

#include "dvc_serialplot.h"
#include "dvc_motor.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Class_Serialplot serialplot;
Class_Motor_C610 motor1;
Class_Motor_C610 motor2;
Class_Motor_C620 motor3;


float Target_Angle, Now_Angle, Target_Omega, Now_Omega;

uint32_t Counter = 0;

// //气缸延时
// const int push=1000;
// const int pull=1000;
// const int gap=1000;

// 非阻塞式气缸控制变量
typedef enum {
    BALL_STATE_IDLE,                     // 空闲状态
    BALL_STATE_PUSHING,                  // 气缸向下推开始（置高电平）
    BALL_STATE_WAITING_FOR_GRIPPER_OPEN, // 等待20ms后夹爪张开90度
    BALL_STATE_PUSHING_HOLD,             // 继续保持高电平直到80ms
    BALL_STATE_PULLING,                  // 气缸往回拉（置低电平）
    BALL_STATE_WAITING_FOR_GRIPPER_CLOSE // 等待450ms后夹爪关闭
} Ball_State_t;

Ball_State_t ball_state = BALL_STATE_IDLE;
uint32_t ball_action_start_time = 0;  // 动作开始时间戳
const uint32_t GRIPPER_OPEN_DELAY_MS = 20;   // 置高电平后20ms张开夹爪
const uint32_t PUSH_TOTAL_DURATION_MS = 100;  // 高电平保持总时间100ms
const uint32_t GRIPPER_CLOSE_DELAY_MS = 450; // 拉回后450ms关闭夹爪

// 电机角度控制变量
float Motor2006_Control_Command = 0.0f;  // 0=转到0度，1=转到90度 
float Motor3508_Control_Command = 0.0f;  // 0=转到0度，1=转到180度
float Ball_Control_Command = 0.0f;  // 0=气缸向上拉夹爪平放，1=气缸向下推夹爪张开

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    //电机调PID
    "pa",
    "ia",
    "da",
    "po",
    "io",
    "do",
    // 2006电机角度控制
    "cmd1",  // 控制命令0=转到0度，1=转到90度
    // 3508电机角度控制
    "cmd2", // 控制命令0=转到0度，1=转到180度
    // 电机及气缸同步控制
    "cmd3", // 控制命令0=夹爪平放，1=夹爪张开气缸向下推后向上拉
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief 控制2006电机相对当前位置旋转90°（夹爪张开）
 * @param 
 */
void Motor2006_Rotate_90_Degree() {

  // 设置目标角度
  motor1.Set_Target_Angle(17.0f * PI);
  motor2.Set_Target_Angle(17.0f * PI);
}

/**
 * @brief 控制2006电机相对当前位置旋转0°（夹爪平放）
 * @param 
 */
void Motor2006_Rotate_0_Degree() {

  // 设置目标角度
  motor1.Set_Target_Angle(0.0f);
  motor2.Set_Target_Angle(0.0f);
}

/**
 * @brief 控制3508电机相对当前位置旋转180°（旋转至运球点）
 * @param 
 */
void Motor3508_Rotate_180_Degree() { 

  // 设置目标角度
  motor3.Set_Target_Angle(47.0f * PI);
}

/**
 * @brief 控制3508电机相对当前位置旋转0°（旋转至初始位置）    
 * @param 
 */
void Motor3508_Rotate_0_Degree() {

  // 设置目标角度
  motor3.Set_Target_Angle(0.0f);
}

/**
 * @brief 控制气缸向上拉
 * @param 
 */
void ballpull()
{
	 HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	//  HAL_Delay(push);
}

/**
 * @brief 控制气缸向下推  
 * @param 
 */
void ballpush()
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	// HAL_Delay(pull);
}

// /**
//  * @brief 控制气缸暂停  
//  * @param 
//  */
// void ballgap()
// {
// 	HAL_Delay(gap);
// }

/**
 * @brief 非阻塞式气缸控制处理函数
 * @param 需要在主循环中持续调用
 */
void Ball_Control_Process()
{
    switch(ball_state)
    {
        case BALL_STATE_IDLE:
            // 空闲状态，等待触发
            break;
            
        case BALL_STATE_PUSHING:
            // 气缸向下推
            ballpush();
            ball_action_start_time = HAL_GetTick();  // 记录开始时间
            ball_state = BALL_STATE_WAITING_FOR_GRIPPER_OPEN;  // 进入等待夹爪张开状态
            break;
            
        case BALL_STATE_WAITING_FOR_GRIPPER_OPEN:
            // 等待20ms后夹爪张开90度
            ballpush();
            if(HAL_GetTick() - ball_action_start_time >= GRIPPER_OPEN_DELAY_MS)
            {
                Motor2006_Rotate_90_Degree(); // 夹爪张开90度
                ball_state = BALL_STATE_PUSHING_HOLD;  // 进入保持推动状态
            }
            break;
            
        case BALL_STATE_PUSHING_HOLD:
            // 继续保持高电平直到总共100ms
            ballpush();
            if(HAL_GetTick() - ball_action_start_time >= PUSH_TOTAL_DURATION_MS)
            {
                ball_state = BALL_STATE_PULLING;  // 100ms后进入拉回状态
            }
            break;
            
        case BALL_STATE_PULLING:
            // 气缸往回拉，置低电平
            ballpull();     // 设置低电平拉回
            ball_action_start_time = HAL_GetTick();  // 重新记录时间
            ball_state = BALL_STATE_WAITING_FOR_GRIPPER_CLOSE;  // 进入等待夹爪关闭状态
            break;
            
        case BALL_STATE_WAITING_FOR_GRIPPER_CLOSE:
            // 等待450ms后夹爪关闭到0度
            if(HAL_GetTick() - ball_action_start_time >= GRIPPER_CLOSE_DELAY_MS)
            {
                Motor2006_Rotate_0_Degree();  // 夹爪转回0度平放
                Motor2006_Control_Command = 0.0f;  // 更新控制命令
                ball_state = BALL_STATE_IDLE; // 动作完成，回到空闲状态
            }
            break;
    }
}

/**
 * @brief 启动非阻塞式推拉球操作
 * @param 替代原来的阻塞式ballpush+延时+ballpull操作
 */
void Start_Ball_Push_Pull()
{
    if(ball_state == BALL_STATE_IDLE)
    {
        ball_state = BALL_STATE_PUSHING;  // 启动推拉球序列
    }
}

/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        // 3508电机
        case (0x205):
        {
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        // 2006电机
        case (0x206):
        {
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x207):
        {
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
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
        }
        break;
        case(1):
        {
            motor1.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());  
        }
        break;
        case(2):
        {
            motor1.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
            motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        case(3):
        {
            motor1.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(4):
        {
            motor1.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(5):
        {
            motor1.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
            motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
        // 电机角度控制命令
        case(6):
        {
            Motor2006_Control_Command = serialplot.Get_Variable_Value();
            // 根据命令值控制电机角度
          if (Motor2006_Control_Command < 0.5f) {
              Motor2006_Rotate_0_Degree();  // 转到0度
          } else {
              Motor2006_Rotate_90_Degree(); // 转到90度
            }
        }
        break;
        case(7):
        {
            Motor3508_Control_Command = serialplot.Get_Variable_Value();
            // 根据命令值控制电机角度
          if (Motor3508_Control_Command < 0.5f) {
              Motor3508_Control_Command = 0.0f;   //防止命令冲突
              Motor3508_Rotate_0_Degree();  // 转到0度
          } else {
              Motor3508_Control_Command = 1.0f;   //防止命令冲突
              Motor3508_Rotate_180_Degree(); // 转到180度
          }
        }
        break;
        case(8):
        {
            Ball_Control_Command = serialplot.Get_Variable_Value();
            if (Ball_Control_Command < 0.5f) {
              Motor2006_Control_Command = 0.0f;   //防止命令冲突
              Motor2006_Rotate_0_Degree();  // 转到0度
            } else {
              Motor2006_Control_Command = 1.0f;
              // 使用非阻塞式气缸和夹爪控制序列
              // 状态机会自动处理：气缸推下->20ms->夹爪90度->气缸拉回->350ms->夹爪0度
              Start_Ball_Push_Pull();     // 启动推拉球序列
            }
        }
        break; 
      }
}

// /**
//  * @brief 蓝牙控制回调函数
//  * @param Buffer 接收到的数据缓冲区
//  * @param Length 数据长度
//  */
// void UART_Bluetooth_Call_Back(uint8_t *Buffer, uint16_t Length)
// {
//     if(Length > 0)
//     {
//         switch(Buffer[0])
//         {
//             case '0':
//                 // 回到初始位置
//                 Motor2006_Rotate_0_Degree();
//                 Motor2006_Control_Command = 0;
//                 break;
                
//             case '1':
//                 // 执行抓球动作
//                 Motor2006_Rotate_90_Degree();
//                 Motor2006_Control_Command = 1;
//                 Start_Ball_Push_Pull();  // 启动非阻塞式推拉球序列
//                 break;
                
//             default:
//                 // 未知命令，不执行任何操作
//                 break;
//         }
//     }
// }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
    BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
    CAN_Init(&hcan1, CAN_Motor_Call_Back);
    UART_Init(&huart2, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);

    serialplot.Init(&huart2, 9, (char **)Variable_Assignment_List);

    motor1.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 200.0f * PI, 200.0f * PI);
    motor1.PID_Omega.Init(200.0f, 100.0f, 0.0f, 0.0f, 9000.0f, 9000.0f);
    motor1.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_ANGLE, 1.0f);

    motor2.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 200.0f * PI, 200.0f * PI);
    motor2.PID_Omega.Init(200.0f, 100.0f, 0.0f, 0.0f, 9000.0f, 9000.0f);
    motor2.Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_ANGLE, 1.0f);

    motor3.PID_Angle.Init(15.0f, 0.0f, 0.0f, 0.0f, 30.0f * PI, 30.0f * PI);
    motor3.PID_Omega.Init(200.0f, 100.0f, 0.0f, 0.0f, 15000.0f, 15000.0f);
    motor3.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_ANGLE, 1.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      // //测试电机是否能正常转动及定位置控制
      // Counter++;
      //   if(Counter >= 4000)
      //   {
      //       Counter = 0;
      //       if(motor1.Get_Target_Angle() == 18.0f * PI)		//电机转9圈，夹爪完全张合
      //       {
      //           // motor.Set_Target_Angle(0.0f);
      //           Motor2006_Rotate_0_Degree();
      //       }
      //       else if(motor1.Get_Target_Angle() == 0.0f)
      //       {
      //           // motor.Set_Target_Angle(18.0f * PI);
      //           Motor2006_Rotate_90_Degree();
      //       }
      //   }

      //测试气缸能否正常工作
      // ballpush();
      // ballpull();
      // ballgap();		

		//串口绘图显示内容
        Target_Angle = motor1.Get_Target_Angle();
        Now_Angle = motor1.Get_Now_Angle();
        Target_Omega = motor1.Get_Target_Omega();
        Now_Omega = motor1.Get_Now_Omega();
        serialplot.Set_Data(7, &Target_Angle, &Now_Angle, &Target_Omega, &Now_Omega, &Motor2006_Control_Command, &Motor3508_Control_Command, &Ball_Control_Command);
        serialplot.TIM_Write_PeriodElapsedCallback();

        //CAN发送数据给电机
        motor1.TIM_PID_PeriodElapsedCallback();
        motor2.TIM_PID_PeriodElapsedCallback();
        motor3.TIM_PID_PeriodElapsedCallback();

        //通信设备回调数据
        TIM_CAN_PeriodElapsedCallback();
        
        //非阻塞式气缸控制处理
        Ball_Control_Process();
		
		//UART发送数据给上位机
        TIM_UART_PeriodElapsedCallback();

        //延时1ms（避免过度频繁发送）
        HAL_Delay(1);

 

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
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
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

