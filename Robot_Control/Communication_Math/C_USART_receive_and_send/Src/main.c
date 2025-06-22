/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for UART communication and LED control
  * @module         : UART Communication Module
  * @author         : Huang shiwen
  * @date           : 2025-06-02
  * @description    : This file implements UART communication with interrupt handling,
  *                   LED control, and data processing for STM32F4xx
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/* Global variables ----------------------------------------------------------*/
char rxBuffer[256];               // Buffer for storing received UART data
volatile uint16_t rxIndex = 0;    // Index for rxBuffer
uint16_t flag = 0;                // Flag indicating new data received

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SendFloatAsString(double value);
void ProcessReceivedData(void);

/**
  * @brief  Converts a float value to string and transmits via UART
  * @param  value: The float value to be sent
  * @retval None
  */
void SendFloatAsString(double value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.2f", value); // Convert float to string with 2 decimal places
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

/**
  * @brief  Processes received UART data
  * @retval None
  * @note   Echoes received data back through USART6 and clears buffer
  */
void ProcessReceivedData(void) {
    if(!flag) return;
    
    // Echo received data to USART6
    HAL_UART_Transmit(&huart6, (uint8_t*)rxBuffer, rxIndex, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    
    // Clear buffer
    rxIndex = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    flag = 0;
}

/**
  * @brief  USART6 interrupt handler
  * @retval None
  * @note   Handles both RXNE (Receive Not Empty) and IDLE interrupts
  */
void USART6_IRQHandler(void) {
    // Handle RXNE flag (new data received)
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
        uint8_t receive = (uint8_t)(huart6.Instance->DR & 0xFF);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // Toggle LED on data reception
        
        if(rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = receive;
            
            // Check for newline or buffer full condition
            if(receive == '\n' || receive == '\r' || rxIndex >= sizeof(rxBuffer)-1) {
                rxBuffer[rxIndex] = '\0'; // Null-terminate string
                flag = 1;               // Set data ready flag
            }
        }
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    }
    
    // Handle IDLE flag (line idle condition)
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        if(rxIndex > 0) {
            rxBuffer[rxIndex] = '\0'; // Null-terminate string
            flag = 1;                 // Set data ready flag
        }
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
}

/**
  * @brief  The application entry point
  * @retval int
  */
int main(void) {
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    
    /* Initialize LEDs */
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    
    /* Enable UART interrupts */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // Receive interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // Idle interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  // Receive interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  // Idle interrupt
    
    /* Configure USART6 interrupt priority */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    
    /* Initialize receive buffer */
    memset(rxBuffer, 0, sizeof(rxBuffer));
    rxIndex = 0;
    flag = 0;

    /* Main loop */
    while (1) {
        if(flag) {
            ProcessReceivedData(); // Process received data when flag is set
        }
        HAL_Delay(1);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /** Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB busses clocks */
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* User can add custom error handling here */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports assert errors
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) { 
    /* User can add custom assert handling here */
}
#endif /* USE_FULL_ASSERT */