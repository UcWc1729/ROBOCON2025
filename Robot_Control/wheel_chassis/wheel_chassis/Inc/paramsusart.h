#ifndef PARAMSUSART_H
#define PARAMSUSART_H


#include "wheelchassis.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern MotorData G_Usart_Motor_Data[4];
extern float G_Usart_ExpectSpeed_Data[4];



void Uart_Init(void);
void Uart_ProcessCommand(void);
void Uart_SendMotorSpeedWave(void);

#endif //USART_H
