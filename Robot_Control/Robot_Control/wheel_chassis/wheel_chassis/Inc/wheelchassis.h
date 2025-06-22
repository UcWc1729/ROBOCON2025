#ifndef __WHEELCHASSIS_H
#define __WHEELCHASSIS_H

#include "main.h"

#include "can.h"
#include "cordic.h"

#define PI 3.1415926
#define TIMESTEP 0.001f //时间步长，和定时中断频率有关

/***************方向定义***************/
//float ExpectSpeed[4] = {v1, -v2, -v1, v2};
                 /*0度*/
/***0度**ID=1****正前方****ID=2**0度***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****V***************************V***/
/*-180度**ID=4************ID=3**180度*/
             /*-180或180度*/

//float ExpectSpeed[4] = {-v2, v1, v2, -v1};
                 /*0度*/
/***0度**ID=2****正前方****ID=1**0度***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****V***************************V***/
/*-180度**ID=3************ID=4**180度*/
             /*-180或180度*/


typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float IntegralLimit; // 积分限幅
    float OutputLimit;   // 输出限幅
}PID_Params;

typedef struct
{
    float Chassis_R;
    float Wheel_R;
    float Acceleration;
    float Deceleration;
}Chassis_Params;

//电机的数据
typedef struct
{
    float angle;
    float speed;
    float TorqueCurrent;
    int8_t temperature;
}MotorData;

//运动状态
typedef enum
{
    Wait = 0, //待机
    Linear, //直线运动
    Turn1,
    Turn2//原地转动
} MotionState;

//运动控制结构体
typedef struct
{
    //运动状态
    MotionState state;

    //直线运动
    float RelativeAngle; //相对底盘正前方的角度,范围为-180~180
    float MovementSpeed; //移动的速度，单位：r/s
    float MovementDisplacement; //移动的距离，单位：r

    //原地转动
    float TurnSpeed; //转动的速度
    float TurnAngle; //底盘旋转的角度
    uint8_t TurnAngleChange_Flag;
} MotionControl;

extern PID_Params G_Speed_PID_Params;
extern PID_Params G_DifSpeed_PID_Params;
extern PID_Params G_Angle_PID_Params;
extern Chassis_Params G_Chassis_Params;
extern MotionControl G_WheelChassisMotion;

void wheelChassis_Init(void);
void wheelChassis_Linear(float MovementSpeed,float RelativeAngle);
void wheelChassis_Turn1(float TurnSpeed);
void wheelChassis_Turn2(float TurnAngle);
void wheelChassis_Stop(void);

#endif
