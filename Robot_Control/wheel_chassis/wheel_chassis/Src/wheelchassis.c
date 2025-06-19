#include "wheelchassis.h"

uint8_t G_FeedbackData[4][8]; // 反馈数据
MotionControl G_WheelChassisMotion; // 运动控制结构体，通过修改它的值来控制底盘的运动

static void M3508_CAN_SendData(uint8_t *data);
static void M3508_CAN_Config(void);
static void M3508_GetFeedbackData(MotorData *data);
static void M3508_CAN_SendCurrent(float *current);
static void wheelChassis_MotionControl(MotionControl *motion, MotorData *data);
static void wheelChassis_RealtimeControl(float v_expect, float *v);
static void wheelChassis_MotorSpeedControl(float *ExpectSpeed, float *Current_Output, MotorData *data);
static float wheelChassis_AbsoluteValue(float num);
static void wheelChassis_MotorSpeedDifferenceOffset(float *Offset_Output, MotorData *data);

/**
 * @brief 初始化并启动 CAN 通信。
 *
 * 该函数完成以下操作：
 * 1. 调用 M3508_CAN_Config() 配置 CAN 外设。
 * 2. 启动 CAN 外设 (hcan1)。
 * 3. 激活 CAN 接收 FIFO 0 的消息挂起中断通知。
 *
 * @note
 * - 该函数依赖于 HAL 库中的 CAN 相关函数。
 * - 确保在调用此函数之前，CAN 外设 (hcan1) 已正确初始化。
 *
 * @see M3508_CAN_Config()
 * @see HAL_CAN_Start()
 * @see HAL_CAN_ActivateNotification()
 */
void M3508_Can_Start(void)
{
    M3508_CAN_Config();
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    G_WheelChassisMotion.state = Wait;
}

/**
 * @brief 通过 CAN 总线发送数据。
 *
 * 该函数完成以下操作：
 * 1. 配置 CAN 发送消息头 (TxHeader)，包括标准标识符 (StdId)、数据长度 (DLC) 等。
 * 2. 使用 HAL 库的 HAL_CAN_AddTxMessage() 函数将数据发送到 CAN 总线。
 *
 * @param data 指向要发送的数据缓冲区的指针。缓冲区应包含最多 8 字节的数据。
 *
 * @note
 * - 该函数使用标准 CAN 标识符 (CAN_ID_STD)，标识符值固定为 0x200。
 * - 数据长度 (DLC) 固定为 8 字节。如果数据不足 8 字节，请确保缓冲区填充了有效数据。
 * - 确保在调用此函数之前，CAN 外设 (hcan1) 已正确初始化并启动。
 *
 * @see CAN_TxHeaderTypeDef
 * @see HAL_CAN_AddTxMessage()
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
 * @brief 配置 CAN 总线的滤波器。
 *
 * 该函数完成以下操作：
 * 1. 初始化 CAN 滤波器配置结构体 (sFilterConfig)。
 * 2. 设置滤波器的工作模式、标识符掩码、FIFO 分配等参数。
 * 3. 使用 HAL 库的 HAL_CAN_ConfigFilter() 函数应用滤波器配置。
 *
 * @note
 * - 滤波器组 0 被配置为使用 32 位标识符掩码模式 (CAN_FILTERMODE_IDMASK)。
 * - 滤波器分配给接收 FIFO 0 (CAN_RX_FIFO0)。
 * - 如果需要修改滤波器的行为，请根据实际需求调整 FilterIdHigh、FilterIdLow 和 FilterMaskId 的值。
 * - 确保在调用此函数之前，CAN 外设 (hcan1) 已正确初始化。
 *
 * @see CAN_FilterTypeDef
 * @see HAL_CAN_ConfigFilter()
 */
void M3508_CAN_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0; // 使用滤波器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}


/**
 * @brief 处理 CAN 接收 FIFO 0 的消息挂起回调。
 *
 * 该函数在 CAN 接收 FIFO 0 中有新消息到达时被调用，完成以下操作：
 * 1. 从 CAN 接收 FIFO 0 中读取消息头 (RxHeader) 和数据 (Rxdata)。
 * 2. 根据消息的标准标识符 (StdId)，将接收到的数据存储到对应的反馈数据缓冲区中。
 *
 * @param hcan 指向 CAN 外设句柄的指针。该参数通常由 HAL 库自动传递。
 *
 * @note
 * - 支持的标准标识符包括：0x201、0x202、0x203 和 0x204。
 * - 每个标准标识符对应一个反馈数据缓冲区 (FeedbackData[0] 至 FeedbackData[3])。
 * - 如果接收到的消息标识符不在支持范围内，则忽略该消息。
 * - 确保在调用此函数之前，CAN 外设已正确初始化并启动。
 *
 * @see CAN_RxHeaderTypeDef
 * @see HAL_CAN_GetRxMessage()
 * @see FeedbackData
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
 * @brief 从反馈数据缓冲区中解析电机的数据。
 *
 * 该函数完成以下操作：
 * 1. 遍历所有 4 个电机的反馈数据 (FeedbackData)。
 * 2. 解析每个电机的角度、速度、扭矩电流和温度，并将结果存储到对应的 MotorData 结构体中。
 *
 * @param data 指向存储解析后数据的 MotorData 数组的指针。数组大小应至少为 4。
 *
 * @note
 * - 反馈数据缓冲区 (FeedbackData) 中的每个元素是一个字节，需要通过位移操作组合成完整的数值。
 * - 角度单位为度 (°)，速度单位为转每秒 (r/s)，扭矩电流单位为安培 (A)，温度单位为摄氏度 (°C)。
 * - 确保在调用此函数之前，FeedbackData 已正确填充。
 *
 * @see FeedbackData
 * @see MotorData
 */
void M3508_GetFeedbackData(MotorData *data)
{
    for (int i = 0; i < 4; i++)
    {
        data[i].angle = ((G_FeedbackData[i][0] << 8) | G_FeedbackData[i][1]) * 360.0 / 8191.0;
        data[i].speed = (int16_t) ((G_FeedbackData[i][2] << 8) | G_FeedbackData[i][3]) / 60.0 / (3591.0 / 187.0);
        // 单位：r/s
        data[i].TorqueCurrent = (int16_t) ((G_FeedbackData[i][4] << 8) | G_FeedbackData[i][5]) / 16384.0;
        data[i].temperature = (int8_t) G_FeedbackData[i][6];
    }
}

/**
 * @brief 将电流值编码并通过 CAN 总线发送。
 *
 * 该函数完成以下操作：
 * 1. 将输入的电流值数组转换为适合 CAN 发送的 16 位整数格式。
 * 2. 将转换后的数据打包到一个字节数组中。
 * 3. 调用 M3508_CAN_SendData() 函数将打包的数据通过 CAN 总线发送。
 *
 * @param current 指向包含 4 个电机电流值的浮点数数组的指针。每个电流值单位为安培 (A)。
 *
 * @note
 * - 输入电流值会被缩放为 16 位整数，公式为：`tmp[i] = current[i] * 16384 / 20`。
 * - 转换后的数据会被拆分为高字节和低字节，并存储到字节数组中。
 * - 确保在调用此函数之前，CAN 外设已正确初始化并启动。
 *
 * @see M3508_CAN_SendData()
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
 * @brief 实现电机速度的闭环控制。
 *
 * 该函数使用 PI 控制器（比例-积分控制器）对 4 个电机的速度进行闭环控制，完成以下操作：
 * 1. 计算期望速度与实际速度之间的误差。
 * 2. 使用 PI 控制算法计算每个电机的输出控制量。
 * 3. 更新误差累加值和上一次误差值，以便下一次调用时使用。
 *
 * @param Expectspeed 指向包含 4 个电机期望速度的浮点数数组的指针。单位为转每秒 (r/s)。
 * @param Current_Output 指向存储计算后的输出控制量的浮点数数组的指针。数组大小应至少为 4。
 * @param data 指向包含 4 个电机当前状态数据的 MotorData 数组的指针。
 *
 * @note
 * - 误差累加值 (tmp) 和上一次误差值 (errorlast) 是静态变量，因此在多次调用之间保持其值。
 * - 确保在调用此函数之前，MotorData 中的实际速度已正确更新。
 *
 * @see MotorData
 */
void wheelChassis_MotorSpeedControl(float *ExpectSpeed, float *Current_Output, MotorData *data)
{
    static float Integral[4];
    static float Error[4], Errorlast[4];
    static float Kp = 1.3f;
    static float Ki = 0.14f;
    static float Kd = 0;
    for (int i = 0; i < 4; i++)
    {
        Error[i] = ExpectSpeed[i] - data[i].speed;
        Current_Output[i] = Kp * (ExpectSpeed[i] - data[i].speed) + Ki * Integral[i] + Kd * (Error[i] - Errorlast[i]);
        Integral[i] += ExpectSpeed[i] - data[i].speed;
        Errorlast[i] = Error[i];
    }
}

/**
 * @brief 调整相对电机的速度差，使其趋近于零。
 *
 * 该函数基于PID控制算法，通过计算两组相对电机的速度差，动态调整输出补偿量，
 * 以实现电机速度同步（即使得相对电机的速度差为零）。
 *
 * @param Offset_Output 输出补偿量数组，用于存储计算后的补偿值。
 *                      数组长度至少为2，分别对应两组相对电机的补偿量。
 * @param data 输入的电机数据数组，包含每个电机的速度信息。
 *             数组长度至少为4，data[0] 和 data[2] 为一组相对电机，data[1] 和 data[3] 为另一组相对电机。
 *
 * 内部逻辑：
 * - 根据PID公式计算补偿量：`Offset = Kp * Error + Ki * Integral + Kd * (Error - Errorlast)`。
 *   其中：
 *   - `Error` 是当前速度差；
 *   - `Integral` 是误差的累积值；
 *   - `Errorlast` 是上一次的速度差。
 * - 更新积分项和上一次的误差值以便后续调用使用。
 *
 * 注意：
 * - PID参数 `Kp`, `Ki`, `Kd` 当前被初始化为0，需要在实际使用中根据需求进行配置。
 * - 该函数假设输入数据的有效性，未对数组边界或空指针进行检查。
 * - 通过不断调整输出补偿量，最终使相对电机的速度差趋近于零，从而实现速度同步。
 */
void wheelChassis_MotorSpeedDifferenceOffset(float *Offset_Output, MotorData *data)
{
    static float Integral[2];
    static float Error[2], Errorlast[2];
    static float Kp = 0; //正数
    static float Ki = 0;
    static float Kd = 0;

    Error[0] = data[2].speed + data[0].speed;
    Offset_Output[0] = Kp * Error[0] + Ki * Integral[0] + Kd * (Error[0] - Errorlast[0]);
    Integral[0] += Error[0];
    Errorlast[0] = Error[0];

    Error[1] = data[1].speed + data[3].speed;
    Offset_Output[1] = Kp * Error[1] + Ki * Integral[1] + Kd * (Error[1] - Errorlast[1]);
    Integral[1] += Error[1];
    Errorlast[1] = Error[1];
}

/**
 * @brief 计算浮点数的绝对值。
 *
 * 该函数根据输入值的正负性返回其绝对值：
 * - 如果输入值为负数，则返回其相反数。
 * - 如果输入值为非负数，则直接返回原值。
 *
 * @param num 输入的浮点数。
 *
 * @return 返回输入浮点数的绝对值。
 *
 * @note
 * - 该函数适用于所有浮点数，包括正数、负数和零。
 *
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
 * @brief 根据目标速度调整 PID 控制的期望速度。
 *
 * 该函数用于遥控场景下，逐步调整 PID 控制器的期望速度 (`v`) 以接近目标速度：
 * - 如果目标速度大于当前期望速度，则按照加速度 (Acceleration) 增加期望速度。
 * - 如果目标速度小于当前期望速度，则按照减速度 (Deceleration) 减少期望速度。
 *
 * @param motion 指向包含运动控制信息的 MotionControl 结构体的指针。
 *               结构体中应包含目标速度 (MovementSpeed) 等相关信息。
 * @param v 指向存储 PID 控制器期望速度的浮点数变量的指针。单位为转每秒 (r/s)。
 *
 * @note
 * - 加速度 (Acceleration) 和减速度 (Deceleration) 是全局变量，需在调用此函数前定义并初始化。
 * - 时间步长 (TimeStep) 是宏定义，用于控制每次调用的时间增量。
 * - 期望速度会在每次调用时逐步逼近目标速度，但不会超过目标速度。
 * - 此函数适用于需要平滑调整期望速度的场景，例如遥控操作。
 *
 * @see MotionControl
 */


void wheelChassis_RealtimeControl(float v_expect, float *v)
{
    if (v_expect - *v != 0)
    {
        if (v_expect - *v > 0)
        {
            *v += Acceleration * TimeStep;
        } else if (v_expect - *v < 0)
        {
            *v -= Deceleration * TimeStep;
        }
    }
}

/**
 * @brief 计算运动曲线并更新 PID 控制的期望速度。
 *
 * 该函数根据目标位移、目标速度以及加速度和减速度，计算运动曲线的不同阶段，
 * 并根据当前时间动态调整 PID 控制器的期望速度 (`v`)。运动分为以下三个阶段：
 * 1. 加速阶段：从静止加速到目标速度。
 * 2. 匀速阶段：以目标速度匀速运动。
 * 3. 减速阶段：从目标速度减速到静止。
 *
 * @param motion 指向包含运动控制信息的 MotionControl 结构体的指针。
 *               结构体中应包含目标位移 (MovementDisplacement)、目标速度 (MovementSpeed)
 *               和状态 (state) 等相关信息。
 * @param v 指向存储 PID 控制器期望速度的浮点数变量的指针。单位为转每秒 (r/s)。
 *
 * @note
 * - 加速度 (Acceleration) 和减速度 (Deceleration) 是宏定义。
 * - 时间步长 (TimeStep) 是宏定义，用于控制每次调用的时间增量。
 * - 如果目标位移不足以完成完整的加速、匀速和减速过程，则会自动调整时间分配。
 * - 当运动完成后，`motion->state` 会被设置为 `Wait`，表示进入等待状态。
 *
 * @see MotionControl, AbsoluteValue
 */
void wheelChassis_MotionCurveControl(MotionControl *motion, float *v)
{
    static double time = 0;

    float t1, t2, t3;
    float s1, s2, s3;
    float a1, a2;

    a1 = Acceleration;
    a2 = Deceleration;

    s1 = (motion->MovementSpeed * motion->MovementSpeed) / (2 * wheelChassis_AbsoluteValue(a1));
    s3 = (motion->MovementSpeed * motion->MovementSpeed) / (2 * wheelChassis_AbsoluteValue(a2));
    s2 = motion->MovementDisplacement - s1 - s3;


    if (s2 <= 0)
    {
        t1 = sqrt(2 * motion->MovementDisplacement / (wheelChassis_AbsoluteValue(a1) * (a1 / a2 + 1)));
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
        time = -TimeStep;
        motion->state = Wait;
    }
    time += TimeStep;
}

/**
 * @brief 实现运动控制逻辑，包括直线运动、转向运动和等待状态。
 *
 * 该函数根据当前运动状态 (`motion->state`) 执行不同的运动控制逻辑：
 * - **直线运动 (LinearMotion)**：
 *   - 调用 `M3508_Can_RemoteControl` 或 `M3508_CAN_MotionCurve` 计算期望速度。
 *   - 根据相对角度计算四个电机的目标速度，并通过 PID 控制器输出实际速度。
 * - **转向运动 (Turn)**：
 *   - 调用 `M3508_Can_RemoteControl` 或 `M3508_CAN_MotionCurve` 计算期望速度。
 *   - 设置四个电机的目标速度相同，以实现整体旋转。
 * - **等待状态 (Wait)**：
 *   - 重置运动相关的参数（位移、速度、角度等），并准备进入下一次运动。
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
        float v1_expect = v * cos(45 * PI / 180.0 - motion->RelativeAngle * PI / 180.0);
        float v2_expect = v * sin(45 * PI / 180.0 - motion->RelativeAngle * PI / 180.0);
        wheelChassis_RealtimeControl(v1_expect, &v1);
        wheelChassis_RealtimeControl(v2_expect, &v2);
        float ExpectSpeed[4] = {v1, -v2, -v1, v2};
        float Current_Output[4],Offect_Output[2];

        wheelChassis_MotorSpeedControl(ExpectSpeed, Current_Output, data);//控制各轮达到期望速度
        wheelChassis_MotorSpeedDifferenceOffset(Offect_Output,data);//使相对轮的差速为0

        Current_Output[0] -= Offect_Output[0];
        Current_Output[2] -= Offect_Output[0];
        Current_Output[1] -= Offect_Output[1];
        Current_Output[3] -= Offect_Output[1];

        M3508_CAN_SendCurrent(Current_Output);
    }
    else if (motion->state == Turn)
    {
        static float v, v_expect;
        v_expect = motion->TurnSpeed;
        wheelChassis_RealtimeControl(v_expect, &v);
        float ExpectSpeed[4] = {v, v, v, v};
        float Current_Output[4];
        wheelChassis_MotorSpeedControl(ExpectSpeed, Current_Output, data);
        M3508_CAN_SendCurrent(Current_Output);
    }
    else if (motion->state == Wait)
    {
        motion->MovementDisplacement = 0;
        motion->MovementSpeed = 0;
        motion->RelativeAngle = 0;
        motion->TurnDisplacement = 0;
        motion->TurnSpeed = 0;
    }
}

/**
 * @brief 定时器周期中断回调函数，用于执行周期性任务。
 *
 * 该函数在定时器周期中断触发时被调用，主要完成以下任务：
 * 1. 获取电机的反馈数据。
 * 2. 调用运动控制函数，根据当前运动状态更新电机的控制指令。
 *
 * @param htim 指向 TIM_HandleTypeDef 结构体的指针，表示触发中断的定时器句柄。
 *             该参数通常由 HAL 库自动传递，无需手动设置。
 *
 * @note
 * - 函数内部使用了静态变量 `data` 来存储四个电机的反馈数据，确保数据在多次中断调用之间保持有效。
 * - 函数依赖于 `M3508_CAN_GetFeedbackData` 和 `M3508_CAN_MotionControl`，
 *   这些函数需要在调用前正确实现。
 * - 该函数通常由硬件抽象层 (HAL) 自动调用，不应在用户代码中直接调用。
 *
 * @see M3508_CAN_GetFeedbackData, M3508_CAN_MotionControl
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static MotorData data[4];
    M3508_GetFeedbackData(data);
    wheelChassis_MotionControl(&G_WheelChassisMotion, data);
}
