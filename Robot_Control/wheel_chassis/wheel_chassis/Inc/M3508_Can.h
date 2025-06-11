#ifndef __M3508_CAN_H
#define __M3508_CAN_H

#include "main.h"
#include "math.h"
#include "can.h"

#define PI 3.1415926
#define Acceleration 0.5 //加速度
#define Deceleration 0.5 //减速度
#define TimeStep 0.001 //时间步长，和定时中断频率有关
#define WheelChassisMode 0 //0为遥控模式，其他为固定方向位移模式

/***************方向定义***************/

                 /*0度*/
/***0度**ID=2****正前方****ID=1**0度***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****|***************************|***/
/****V***************************V***/
/*-180度**ID=3************ID=4**180度*/
             /*-180或180度*/

//电机的数据
typedef struct
{
    float angle;
    float speed;
    float TorqueCurrent;
    int8_t temperature;
} MotorData;

//运动状态
typedef enum
{
    Wait = 0, //待机
    LinearMotion, //直线运动
    Turn //原地转动
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
    float TurnDisplacement; //转动的距离
} MotionControl;

void M3508_Can_Start(void);

#endif
