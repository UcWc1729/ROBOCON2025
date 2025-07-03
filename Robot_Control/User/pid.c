#include "pid.h"           // PID算法头文件
#include <math.h>           // 数学库，主要用于fabsf等函数
#include <main.h>           // 主工程头文件

PID_typedef Motor_pid[14];  // 电机PID参数数组，支持多路电机

/**
 * @brief   限幅宏，将输入input限制在±max之间
 */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
 * @brief  PID参数批量初始化
 * @note   可根据不同电机/功能初始化不同的PID参数
 */
void PID_total_init(void)
{
	// 初始化前三路电机的速度环PID，参数可根据实际调整
	for(uint8_t i=0;i<3;i++){
		PID_Init(&Motor_pid[i], PID_POSITION_SPEED, 0.08, 0.01, 0.0, 1000, 1000, 0);
	}
	// 其他特殊用途的PID（如云台、角度等）
	PID_Init(&Motor_pid[7], PID_POSITION_SPEED, 2, 0.1, 0, 8000, 16384, 0);
	PID_Init(&Motor_pid[12], PID_POSITION_ANGLE, 10, 0.1, 0, 8000, 30000, 0);
}

/**
 * @brief  初始化单个PID结构体参数
 * @param  PID:    PID结构体指针
 * @param  mode:   PID控制模式（位置式/增量式/角度）
 * @param  kp:     比例系数
 * @param  ki:     积分系数
 * @param  kd:     微分系数
 * @param  Max_iout: 积分输出最大值
 * @param  Max_out:  总输出最大值
 * @param  deadband: 死区范围
 */
void PID_Init(PID_typedef *PID, PID_mode Mode, float kp, float ki, float kd, float Max_iout, float Max_out, float deadband)
{
	if(PID == NULL) return;
	
	PID->mode = Mode;         // 控制模式
	PID->Kp = kp;             // 比例系数
	PID->Ki = ki;             // 积分系数
	PID->Kd = kd;             // 微分系数
	PID->Max_iout = Max_iout; // 积分限幅
	PID->Max_out = Max_out;   // 总输出限幅
	PID->DeadBand = deadband; // 死区
}

/**
 * @brief  PID控制主函数，计算输出
 * @param  PID:     PID结构体指针
 * @param  measure: 当前测量值
 * @param  target:  目标值
 * @retval PID输出
 */
float PID_calc(PID_typedef *PID, float measure, float target)
{
	if(PID == NULL)
		return 0;
	
	// 更新测量值和目标值
	PID->measure += measure;
	PID->target = target;
	
	// 误差历史更新，error[0]为当前误差，error[1]为上一次，error[2]为上上次
	PID->error[2] = PID->error[1];
	PID->error[1] = PID->error[0];
	PID->error[0] = target - measure;
	
	// 死区判断，误差在死区内则输出为0
	if(fabsf(PID->error[0]) > PID->DeadBand || PID->DeadBand == 0){
		// 位置式PID（速度/角度）
		if(PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE){
			// 角度环特殊处理，防止角度突变
			if(PID->mode == PID_POSITION_ANGLE){
				if(PID->error[0] > 4096) PID->error[0] = PID->error[0] - 8191;
				else if(PID->error[0] < -4096) PID->error[0] = PID->error[0] + 8191;
			}
			// 比例项
			PID->Pout = PID->Kp * PID->error[0];
			// 积分项累加
			PID->Iout += PID->Ki * PID->error[0];
			// 微分项（当前误差-上一次误差）
			PID->D_item = (PID->error[0] - PID->error[1]);
			PID->Dout = PID->Kd * PID->D_item;
			// 积分限幅
			LimitMax(PID->Iout, PID->Max_iout);
			// 总输出
			PID->OUT = PID->Pout + PID->Iout + PID->Dout;
			// 总输出限幅
			LimitMax(PID->OUT, PID->Max_out);
		}
		// 增量式PID（速度）
		else if(PID->mode == PID_DELTA_SPEED){
			// 比例项为本次与上次误差差值
			PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);
			// 积分项为本次误差
			PID->Iout = PID->Ki * PID->error[0];
			// 微分项为本次-2*上次+上上次
			PID->D_item = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]);
			PID->Dout = PID->Kd * PID->D_item;
			// 增量式输出为累加
			PID->OUT += PID->Pout + PID->Iout + PID->Dout;
			// 总输出限幅
			LimitMax(PID->OUT, PID->Max_out);
		}
	}else{
		PID->OUT = 0; // 死区内输出为0
	}
	
	return PID->OUT;
}














