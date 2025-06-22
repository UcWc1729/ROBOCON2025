#include "wheelchassis.h"
#include "paramsusart.h"

uint8_t G_FeedbackData[4][8]; // 反馈数据
MotionControl G_WheelChassisMotion; // 运动控制结构体，通过修改它的值来控制底盘的运动
PID_Params G_Speed_PID_Params; // 速度环PID
PID_Params G_DifSpeed_PID_Params; // 差速环PID
PID_Params G_Angle_PID_Params; // 角度环PID
Chassis_Params G_Chassis_Params; // 运动参数

static void M3508_CAN_SendData(uint8_t *data);
static void M3508_CAN_Config(void);
static void M3508_GetFeedbackData(MotorData *data);
static void M3508_CAN_SendCurrent(float *current);
static void wheelChassis_MotionControl(MotionControl *motion, MotorData *data);
static void wheelChassis_RealtimeControl(float v_expect, float *v);
static void wheelChassis_MotorSpeedControl(float *ExpectSpeed, float *Current_Output, MotorData *data);
static float wheelChassis_AbsoluteValue(float num);
static void wheelChassis_MotorSpeedDifferenceOffset(float *Offset_Output, MotorData *data);
static void wheelChassis_Init_PID_Params(void);
static void wheelChassis_Init_Chassis_Params(void);


/**
 * @brief 轮式底盘模块总初始化函数。
 *
 * 功能：
 * - 配置并启动CAN通信，包括滤波器和中断。
 * - 调用内部初始化函数，设置所有PID控制器参数和底盘物理参数（如加减速、半径等）。
 * - 将底盘初始状态设置为等待（Wait）。
 *
 * 调用时机：
 * - 在主程序（如main.c）的初始化阶段，所有依赖的硬件（如CAN1）初始化之后，主循环开始之前调用一次。
 *
 * 注意事项：
 * - 这是一个必须调用的函数，它确保了底盘模块所有功能的正常运行前提。
 * - 依赖于CubeMX生成的 `hcan1` 句柄。
 */
void wheelChassis_Init(void)
{
    M3508_CAN_Config();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    G_WheelChassisMotion.state = Wait;
    wheelChassis_Init_PID_Params();
    wheelChassis_Init_Chassis_Params();
}

/**
 * @brief 通过 CAN 总线发送8字节数据。
 *
 * 功能：
 * - 封装标准ID为0x200的CAN报文，发送8字节数据。
 *
 * @param data 指向8字节数据的指针，数据不足8字节需补齐。
 *
 * 注意事项：
 * - 发送前需确保CAN已启动。
 * - 发送失败无返回值，建议结合HAL_CAN_GetError()排查。
 */
void M3508_CAN_SendData(uint8_t *data)
{
    uint32_t TxMailbox;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x200;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
}

/**
 * @brief 配置 CAN 滤波器，仅接收0x201~0x204。
 *
 * 功能：
 * - 设置CAN为掩码模式，只允许0x201~0x204报文进入FIFO0。
 *
 * 注意事项：
 * - 需在CAN启动前调用。
 * - 多滤波器场景下需合理分配FilterBank。
 */
void M3508_CAN_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0; // 使用滤波器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    // 只接收0x201~0x204
    // 0x201~0x204的高16位为0，低16位分别为0x201~0x204
    // 掩码设置为0x7FF，表示只比较标准ID的11位
    sFilterConfig.FilterIdHigh = 0x201 << 5; // 标准ID左移5位
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x7FF << 5; // 只比较11位标准ID
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}


/**
 * @brief CAN FIFO0消息挂起回调。
 *
 * 功能：
 * - 接收并解析0x201~0x204反馈报文，存入全局反馈缓冲区。
 *
 * @param hcan CAN句柄指针。
 *
 * 注意事项：
 * - 仅处理0x201~0x204，其他ID自动丢弃。
 * - 需在HAL库中注册为回调。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Rxdata[8];

    CAN_RxHeaderTypeDef RxHeader;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Rxdata);

    switch (RxHeader.StdId)
    {
        case 0x201:
            for (int i = 0; i < 8; i++)
            {
                G_FeedbackData[0][i] = Rxdata[i];
            }
            break;
        case 0x202:
            for (int i = 0; i < 8; i++)
            {
                G_FeedbackData[1][i] = Rxdata[i];
            }
            break;
        case 0x203:
            for (int i = 0; i < 8; i++)
            {
                G_FeedbackData[2][i] = Rxdata[i];
            }
            break;
        case 0x204:
            for (int i = 0; i < 8; i++)
            {
                G_FeedbackData[3][i] = Rxdata[i];
            }
            break;
        default:
        {
            break;
        }
    }
}

/**
 * @brief 解析反馈数据，填充MotorData数组。
 *
 * 功能：
 * - 将全局反馈缓冲区的原始字节数据，解析为角度（度）、速度（r/s）、电流（A）、温度（℃）。
 *
 * @param data MotorData数组，长度至少为4。
 *
 * 注意事项：
 * - 需保证反馈缓冲区已被最新报文填充。
 * - 角度范围0~360，速度单位r/s。
 */
void M3508_GetFeedbackData(MotorData *data)
{
    for (int i = 0; i < 4; i++)
    {
        data[i].angle = (float)((G_FeedbackData[i][0] << 8) | G_FeedbackData[i][1]) * 360.0f / 8191.0f;
        data[i].speed = (int16_t) ((G_FeedbackData[i][2] << 8) | G_FeedbackData[i][3]) / 60.0f / (3591.0f / 187.0f);
        // 单位：r/s
        data[i].TorqueCurrent = (int16_t) ((G_FeedbackData[i][4] << 8) | G_FeedbackData[i][5]) * 20.0f / 16384.0f;
        data[i].temperature = (int8_t) G_FeedbackData[i][6];
    }
}

/**
 * @brief 发送4路电机电流指令。
 *
 * 功能：
 * - 将4路电流（A）转换为16位整数，打包为8字节，通过CAN发送。
 *
 * @param current 4路电流数组，单位A。
 *
 * 注意事项：
 * - 电流范围建议-20A~+20A，超出可能溢出。
 * - 需确保CAN已启动。
 */
void M3508_CAN_SendCurrent(float *current)
{
    uint8_t data[8];
    int16_t tmp[4];
    for (int i = 0; i < 4; i++)
    {
        tmp[i] = current[i] * 16384 / 20;
    }
    int j = 0;
    for (int i = 0; i < 8; i += 2)
    {
        data[i] = tmp[i - j] >> 8;
        data[i + 1] = tmp[i - j];
        j++;
    }
    M3508_CAN_SendData(data);
}

/**
 * @brief 初始化所有PID控制器的参数。
 *
 * 功能：
 * - 为速度环、差速补偿环、角度环的PID控制器设置初始的Kp, Ki, Kd, 积分限幅和输出限幅。
 * - 这些默认值是初步设定，建议根据实际底盘性能进行调试和优化。
 *
 * 调用时机：
 * - 在系统初始化阶段调用，例如在 `M3508_Can_Start` 中，以确保所有PID控制器在运行前都有确定的参数。
 *
 * 注意事项：
 * - 此函数为静态函数，仅在模块内部调用。
 * - 参数的设定直接影响底盘的动态响应和稳定性，修改时需谨慎。
 */
void wheelChassis_Init_PID_Params(void)
{
    // 速度环PID参数
    G_Speed_PID_Params.Kp = 1.3f;
    G_Speed_PID_Params.Ki = 0.14f;
    G_Speed_PID_Params.Kd = 0.0f;
    G_Speed_PID_Params.IntegralLimit = 200.0f;
    G_Speed_PID_Params.OutputLimit = 10.0f; // 最大输出10A电流

    // 差速补偿PID参数
    G_DifSpeed_PID_Params.Kp = 0.0f;
    G_DifSpeed_PID_Params.Ki = 0.0f;
    G_DifSpeed_PID_Params.Kd = 0.0f;
    G_DifSpeed_PID_Params.IntegralLimit = 200.0f;
    G_DifSpeed_PID_Params.OutputLimit = 5.0f; // 补偿量限幅

    // 角度环PID参数
    G_Angle_PID_Params.Kp = 0.0f;
    G_Angle_PID_Params.Ki = 0.0f;
    G_Angle_PID_Params.Kd = 0.0f;
    G_Angle_PID_Params.IntegralLimit = 20000.0f;
    G_Angle_PID_Params.OutputLimit = 20.0f; // 角度环输出最大速度 20 r/s
}

/**
 * @brief 初始化底盘的物理和运动参数。
 *
 * 功能：
 * - 设置底盘的默认加速度、减速度、底盘半径和车轮半径。
 * - 这些参数是运动学解算和运动控制（如斜坡、曲线运动）的基础。
 *
 * 调用时机：
 * - 在系统初始化阶段调用，确保所有运动相关的计算都有一个已知的初始状态。
 *
 * 注意事项：
 * - 此函数将底盘半径 `Chassis_R` 和车轮半径 `Wheel_R` 初始化为0。
 * - 在使用任何依赖这些尺寸的运动控制（如 `wheelChassis_TurnAngleControl`）之前，
 *   必须通过其他方式（如配置函数或调试器）为它们设置正确的、经过测量的实际值。
 * - 加速度和减速度影响速度变化的快慢。
 */
void wheelChassis_Init_Chassis_Params(void)
{
    G_Chassis_Params.Acceleration = 1.0f;
    G_Chassis_Params.Deceleration = 1.0f;
    G_Chassis_Params.Chassis_R = 0;
    G_Chassis_Params.Wheel_R = 0;
}

/**
 * @brief 速度环PID控制，输出4路电机控制量。
 *
 * 功能：
 * - 对每个电机进行PI(D)速度闭环，输出电流指令。
 * - 内含积分限幅，防止积分饱和。
 *
 * @param ExpectSpeed 期望速度数组（r/s）。
 * @param Current_Output 输出控制量数组（A）。
 * @param data MotorData数组。
 *
 * 注意事项：
 * - 需先更新MotorData的实际速度。
 * - PID参数需根据实际调试。
 */
void wheelChassis_MotorSpeedControl(float *ExpectSpeed, float *Current_Output, MotorData *data)
{
    static float Integral[4];
    static float Error[4], Errorlast[4];
    float Kp = G_Speed_PID_Params.Kp;
    float Ki = G_Speed_PID_Params.Ki;
    float Kd = G_Speed_PID_Params.Kd;
    float IntegralLimit = G_Speed_PID_Params.IntegralLimit;
    float OutputLimit = G_Speed_PID_Params.OutputLimit;

    for (int i = 0; i < 4; i++)
    {
        Error[i] = ExpectSpeed[i] - data[i].speed;

        // 更新积分项
        Integral[i] += Error[i];
        // 积分限幅
        if (Integral[i] > IntegralLimit)
        {
            Integral[i] = IntegralLimit;
        }
        if (Integral[i] < -IntegralLimit)
        {
            Integral[i] = -IntegralLimit;
        }

        // 计算PID输出
        Current_Output[i] = Kp * Error[i] + Ki * Integral[i] + Kd * (Error[i] - Errorlast[i]);

        // 输出限幅
        if (Current_Output[i] > OutputLimit)
        {
            Current_Output[i] = OutputLimit;
        }
        if (Current_Output[i] < -OutputLimit)
        {
            Current_Output[i] = -OutputLimit;
        }

        Errorlast[i] = Error[i];
    }
}

/**
 * @brief 对角轮差速补偿PID。
 *
 * 功能：
 * - 计算两组对角轮速度差，输出补偿量，提升直线行驶一致性。
 * - 内含积分限幅。
 *
 * @param Offset_Output 输出补偿量数组（A）。
 * @param data MotorData数组。
 *
 * 注意事项：
 * - 仅适用于四轮全向/麦克纳姆底盘。
 * - PID参数建议较小，防止过度补偿。
 */
void wheelChassis_MotorSpeedDifferenceOffset(float *Offset_Output, MotorData *data)
{
    static float Integral[2];
    static float Error[2], Errorlast[2];
    float Kp = G_DifSpeed_PID_Params.Kp;
    float Ki = G_DifSpeed_PID_Params.Ki;
    float Kd = G_DifSpeed_PID_Params.Kd;
    float IntegralLimit = G_DifSpeed_PID_Params.IntegralLimit;
    float OutputLimit = G_DifSpeed_PID_Params.OutputLimit;

    // 对角轮1 (电机1和3)
    Error[0] = data[2].speed + data[0].speed;
    Integral[0] += Error[0];
    if (Integral[0] > IntegralLimit)
    {
        Integral[0] = IntegralLimit;
    }
    if (Integral[0] < -IntegralLimit)
    {
        Integral[0] = -IntegralLimit;
    }
    Offset_Output[0] = Kp * Error[0] + Ki * Integral[0] + Kd * (Error[0] - Errorlast[0]);
    if (Offset_Output[0] > OutputLimit)
    {
        Offset_Output[0] = OutputLimit;
    }
    if (Offset_Output[0] < -OutputLimit)
    {
        Offset_Output[0] = -OutputLimit;
    }
    Errorlast[0] = Error[0];

    // 对角轮2 (电机2和4)
    Error[1] = data[1].speed + data[3].speed;
    Integral[1] += Error[1];
    if (Integral[1] > IntegralLimit)
    {
        Integral[1] = IntegralLimit;
    }
    if (Integral[1] < -IntegralLimit)
    {
        Integral[1] = -IntegralLimit;
    }
    Offset_Output[1] = Kp * Error[1] + Ki * Integral[1] + Kd * (Error[1] - Errorlast[1]);
    if (Offset_Output[1] > OutputLimit)
    {
        Offset_Output[1] = OutputLimit;
    }
    if (Offset_Output[1] < -OutputLimit)
    {
        Offset_Output[1] = -OutputLimit;
    }
    Errorlast[1] = Error[1];
}

/**
 * @brief 角度环PID控制，实现定角度自转。
 *
 * 功能：
 * - 累计当前角度，计算与目标角度的误差，输出期望自转速度。
 * - 支持正反转，自动处理跨0/360度跳变。
 * - 支持任务重置，防止跨任务误差。
 * - 内含积分限幅。
 *
 * @param expect_angle 期望转动角度（度，正逆时针，负顺时针）。
 * @param ExpectSpeed 输出期望速度指针（r/s）。
 * @param data 当前电机数据，需含angle（度）。
 * @param motion 运动控制结构体，需含TurnAngleChange_Flag。
 *
 * 注意事项：
 * - 需保证CHASSIS_R、WHEEL_R、INTEGRAL_LIMIT2等宏已正确定义。
 * - PID参数需根据实际调试。
 * - 调用本函数后，需将ExpectSpeed作为目标速度传递给速度环。
 */
void wheelChassis_TurnAngleControl(float expect_angle, float *ExpectSpeed, MotorData *data, MotionControl *motion)
{
    static float Integral;
    static float Error, Errorlast;
    float Kp = G_Angle_PID_Params.Kp;
    float Ki = G_Angle_PID_Params.Ki;
    float Kd = G_Angle_PID_Params.Kd;
    float IntegralLimit = G_Angle_PID_Params.IntegralLimit;
    float OutputLimit = G_Angle_PID_Params.OutputLimit;

    static float anglenow, anglelast, turn_angle;
    anglenow = data[0].angle;

    if (motion->TurnAngleChange_Flag == 1)
    {
        turn_angle = 0;
        Integral = 0;
        Errorlast = 0; // 重置上次误差，防止D项突变
        motion->TurnAngleChange_Flag = 0;
        anglelast = anglenow;
    }

    if (expect_angle >= 0)
    {
        turn_angle += anglenow - anglelast >= 0 ? anglenow - anglelast : anglenow - anglelast + 360;
    } else
    {
        turn_angle += anglenow - anglelast <= 0 ? anglenow - anglelast : anglenow - anglelast - 360;
    }


    Error = expect_angle - turn_angle;

    // 更新积分项并限幅
    Integral += Error;
    if (Integral > IntegralLimit)
    {
        Integral = IntegralLimit;
    }
    if (Integral < -IntegralLimit)
    {
        Integral = -IntegralLimit;
    }

    // 计算PID输出并限幅
    *ExpectSpeed = Kp * Error + Ki * Integral + Kd * (Error - Errorlast);
    if (*ExpectSpeed > OutputLimit)
    {
        *ExpectSpeed = OutputLimit;
    }
    if (*ExpectSpeed < -OutputLimit)
    {
        *ExpectSpeed = -OutputLimit;
    }

    Errorlast = Error;
    anglelast = anglenow;
}

/**
 * @brief 计算浮点数绝对值。
 *
 * 功能：
 * - 返回输入浮点数的绝对值。
 *
 * @param num 输入数。
 * @return 绝对值。
 *
 * 注意事项：
 * - 适用于所有浮点数，包括正数、负数和零。
 */
float wheelChassis_AbsoluteValue(float num)
{
    if (num < 0)
    {
        return -num;
    }
    return num;
}

/**
 * @brief 目标速度斜坡处理。
 *
 * 功能：
 * - 以设定加速度/减速度平滑逼近目标速度，防止突变。
 *
 * @param v_expect 目标速度（r/s）。
 * @param v 期望速度指针（r/s）。
 *
 * 注意事项：
 * - ACCELERATION、DECELERATION、TIMESTEP需正确定义。
 * - 适用于遥控/自动运动的速度平滑。
 */
void wheelChassis_RealtimeControl(float v_expect, float *v)
{
    if (v_expect - *v != 0)
    {
        if (v_expect - *v > 0)
        {
            *v += G_Chassis_Params.Acceleration * TIMESTEP;
        } else if (v_expect - *v < 0)
        {
            *v -= G_Chassis_Params.Deceleration * TIMESTEP;
        }
    }
}

/**
 * @brief 运动曲线生成，输出期望速度。
 *
 * 功能：
 * - 根据目标位移、目标速度、加减速，自动分配加速、匀速、减速阶段。
 * - 动态调整期望速度，实现平滑运动。
 *
 * @param motion 运动控制结构体，需含MovementDisplacement、MovementSpeed等。
 * @param v 期望速度指针（r/s）。
 *
 * 注意事项：
 * - 适用于自动运动、定长运动等场景。
 * - 支持位移不足时自动缩短加减速阶段。
 */
void wheelChassis_MotionCurveControl(MotionControl *motion, float *v)
{
    static float time = 0;

    float t1, t2, t3;
    float s1, s2, s3;
    float a1, a2;

    a1 = G_Chassis_Params.Acceleration;
    a2 = G_Chassis_Params.Deceleration;

    s1 = (motion->MovementSpeed * motion->MovementSpeed) / (2 * wheelChassis_AbsoluteValue(a1));
    s3 = (motion->MovementSpeed * motion->MovementSpeed) / (2 * wheelChassis_AbsoluteValue(a2));
    s2 = motion->MovementDisplacement - s1 - s3;


    if (s2 <= 0)
    {
        t1 = sqrtf(2 * motion->MovementDisplacement / (wheelChassis_AbsoluteValue(a1) * (a1 / a2 + 1)));
        t2 = t1;
        t3 = t1 * (a1 / a2 + 1);
    } else
    {
        t1 = motion->MovementSpeed / a1;
        t2 = s2 / motion->MovementSpeed;
        t3 = motion->MovementSpeed / a2;
    }

    if (time > 0 && time <= t1)
    {
        *v = a1 * time;
    } else if (time > t1 && time <= t2)
    {
        *v = a1 * t1;
    } else if (time > t2 && time <= t3)
    {
        *v = a1 * t1 - a2 * (time - t2);
    } else if (time > t3)
    {
        *v = 0;
        time = -TIMESTEP;
        motion->state = Wait;
    }
    time += TIMESTEP;
}

/**
 * @brief 底盘运动主控制函数。
 *
 * 功能：
 * - 根据运动状态机，分配直线、转向、定角度等多种运动模式。
 * - 调用各类控制函数，输出最终电流指令。
 *
 * @param motion 指向包含运动控制信息的 MotionControl 结构体的指针。
 *               结构体中应包含运动状态 (state)、目标位移、目标速度、相对角度等信息。
 * @param data 指向存储电机数据的 MotorData 结构体的指针。
 *              包含电机的实际速度反馈等信息，用于速度控制。
 *
 * @note
 * - 函数内部调用了多个辅助函数（如 `M3508_Can_RemoteControl` 和 `M3508_MotorSpeedControl`），
 *   这些函数需要在调用前正确实现。
 * - 直线运动时，目标速度会根据相对角度进行分解，分别计算每个电机的速度分量。
 * - 转向运动时，四个电机的目标速度相同，确保底盘整体旋转。
 * - 等待状态下，所有运动参数会被重置，以便重新开始新的运动任务。
 *
 * @see M3508_Can_RemoteControl, M3508_CAN_MotionCurve, M3508_MotorSpeedControl, M3508_CAN_SendCurrent
 */

void wheelChassis_MotionControl(MotionControl *motion, MotorData *data)
{
    if (motion->state == Linear)
    {
        static float v, v1, v2;
        v = motion->MovementSpeed;
        //float v1_expect = v * cos(45 * PI / 180.0 - motion->RelativeAngle * PI / 180.0);
        //float v2_expect = v * sin(45 * PI / 180.0 - motion->RelativeAngle * PI / 180.0);
        float v1_expect = v * my_cos(45 - motion->RelativeAngle);
        float v2_expect = v * my_sin(45 - motion->RelativeAngle);
        wheelChassis_RealtimeControl(v1_expect, &v1);
        wheelChassis_RealtimeControl(v2_expect, &v2);
        float ExpectSpeed[4] = {v1, -v2, -v1, v2};
        float Current_Output[4],Offect_Output[2];
        memcpy(G_Usart_ExpectSpeed_Data, ExpectSpeed, sizeof(float) * 4);

        wheelChassis_MotorSpeedControl(ExpectSpeed, Current_Output, data);//控制各轮达到期望速度
        wheelChassis_MotorSpeedDifferenceOffset(Offect_Output,data);//使相对轮的差速为0

        Current_Output[0] -= Offect_Output[0];
        Current_Output[2] -= Offect_Output[0];
        Current_Output[1] -= Offect_Output[1];
        Current_Output[3] -= Offect_Output[1];

        M3508_CAN_SendCurrent(Current_Output);
    }
    else if (motion->state == Turn1)
    {
        static float v, v_expect;
        v_expect = motion->TurnSpeed;
        wheelChassis_RealtimeControl(v_expect, &v);
        float ExpectSpeed[4] = {v, v, v, v};
        float Current_Output[4];
        memcpy(G_Usart_ExpectSpeed_Data, ExpectSpeed, sizeof(float) * 4);
        wheelChassis_MotorSpeedControl(ExpectSpeed, Current_Output, data);
        M3508_CAN_SendCurrent(Current_Output);
    }
    else if (motion->state == Turn2)
    {
        float expect_angle = motion->TurnAngle * G_Chassis_Params.Chassis_R / G_Chassis_Params.Wheel_R;
        float v = 0;
        wheelChassis_TurnAngleControl(expect_angle,&v,data,motion);
        float ExpectSpeed[4] = {v, v, v, v};
        float Current_Output[4];
        memcpy(G_Usart_ExpectSpeed_Data, ExpectSpeed, sizeof(float) * 4);
        wheelChassis_MotorSpeedControl(ExpectSpeed, Current_Output, data);
        M3508_CAN_SendCurrent(Current_Output);
    }
    else if (motion->state == Wait)
    {
        motion->MovementDisplacement = 0;
        motion->MovementSpeed = 0;
        motion->RelativeAngle = 0;
        motion->TurnAngle = 0;
        motion->TurnSpeed = 0;
        motion->TurnAngleChange_Flag = 0;
        float Current_Output[4] = {0, 0, 0, 0};
        M3508_CAN_SendCurrent(Current_Output);
    }
}

/**
 * @brief 设置底盘直线运动参数。
 *
 * 功能：
 * - 配置底盘进行直线运动，支持任意方向。
 * - 自动设置运动状态为Linear，并清零其他运动参数。
 *
 * @param MovementSpeed 直线运动速度（r/s，为正）。
 * @param RelativeAngle 运动方向角度（度，0度为前进，90度为右移，-90度为左移等）。
 *
 * 典型调用示例：
 * - wheelChassis_Linear(2.0, 0);    // 前进，速度2r/s
 * - wheelChassis_Linear(1.5, 90);   // 右移，速度1.5r/s
 * - wheelChassis_Linear(1.0, -90);  // 左移，速度1r/s
 *
 * 注意事项：
 * - 调用后底盘会立即开始直线运动。
 * - 运动过程中可重复调用以改变运动参数。
 */
void wheelChassis_Linear(float MovementSpeed,float RelativeAngle)
{
    G_WheelChassisMotion.state = Linear;
    G_WheelChassisMotion.MovementSpeed = MovementSpeed;
    G_WheelChassisMotion.RelativeAngle = RelativeAngle;
    G_WheelChassisMotion.TurnSpeed = 0;
    G_WheelChassisMotion.TurnAngle = 0;
    G_WheelChassisMotion.TurnAngleChange_Flag = 0;
}

/**
 * @brief 设置底盘连续转向运动参数。
 *
 * 功能：
 * - 配置底盘进行连续转向运动（速度环控制）。
 * - 四个轮子以相同速度转动，实现整体旋转。
 * - 自动设置运动状态为Turn1，并清零其他运动参数。
 *
 * @param TurnSpeed 转向速度（r/s，正为逆时针，负为顺时针）。
 *
 * 典型调用示例：
 * - wheelChassis_Turn1(1.0);   // 顺时针旋转，底盘旋转速度大小1r/s
 * - wheelChassis_Turn1(-0.5);  // 逆时针旋转，底盘旋转速度大小0.5r/s
 *
 * 注意事项：
 * - 调用后底盘会立即开始连续转向。
 * - 转向会一直持续，直到调用其他运动函数或输入为0。
 * - 适用于遥控转向、持续旋转等场景。
 */
void wheelChassis_Turn1(float TurnSpeed)
{
    G_WheelChassisMotion.state = Turn1;
    G_WheelChassisMotion.MovementSpeed = 0;
    G_WheelChassisMotion.RelativeAngle = 0;
    G_WheelChassisMotion.TurnSpeed = TurnSpeed;
    G_WheelChassisMotion.TurnAngle = 0;
    G_WheelChassisMotion.TurnAngleChange_Flag = 0;
}

/**
 * @brief 设置底盘定角度转向运动参数。
 *
 * 功能：
 * - 配置底盘进行定角度转向运动（角度环和速度环串级控制）。
 * - 底盘会精确转动到指定角度后停止。
 * - 自动设置运动状态为Turn2，并清零其他运动参数。
 * - 自动设置TurnAngleChange_Flag为1，重置角度累计。
 *
 * @param TurnAngle 目标转向角度（度，正为逆时针，负为顺时针）。
 *
 * 典型调用示例：
 * - wheelChassis_Turn2(90);   // 顺时针转动90度
 * - wheelChassis_Turn2(-45);  // 逆时针转动45度
 * - wheelChassis_Turn2(180);  // 顺时针转动180度
 *
 * 注意事项：
 * - 调用后底盘会立即开始定角度转向。
 * - 转向完成后会自动停止（状态变为Wait）。
 * - 适用于精确对准、定点转向等场景。
 * - 支持任意角度，包括超过360度的角度。
 */
void wheelChassis_Turn2(float TurnAngle)
{
    G_WheelChassisMotion.state = Turn2;
    G_WheelChassisMotion.MovementSpeed = 0;
    G_WheelChassisMotion.RelativeAngle = 0;
    G_WheelChassisMotion.TurnSpeed = 0;
    G_WheelChassisMotion.TurnAngle = TurnAngle;
    G_WheelChassisMotion.TurnAngleChange_Flag = 1;
}

/**
 * @brief 立即停止底盘运动。
 *
 * 功能：
 * - 将底盘运动状态机强制设置为等待（Wait）状态。
 * - 进入Wait状态后，所有运动参数将被清零，电机停止转动。
 *
 * 典型调用示例：
 * - wheelChassis_Stop();  // 在紧急情况或需要立即停止时调用
 *
 * 注意事项：
 * - 这是一个紧急停止或重置函数，会立即中断当前运动。
 * - 调用后，底盘将保持静止，直到下一次运动指令。
 */
void wheelChassis_Stop(void)
{
    G_WheelChassisMotion.state = Wait;
}

/**
 * @brief 定时器周期中断回调。
 *
 * 功能：
 * - 周期性获取反馈数据，调度底盘运动主控。
 *
 * @param htim 定时器句柄。
 *
 * 注意事项：
 * - 需在HAL库中注册为定时器回调。
 * - 适用于实时性要求高的底盘控制。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
    static MotorData data[4];
    M3508_GetFeedbackData(data);
    memcpy(G_Usart_Motor_Data, data, sizeof(MotorData) * 4);
    wheelChassis_MotionControl(&G_WheelChassisMotion, data);
}
