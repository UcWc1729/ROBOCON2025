#include "cordic.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 预计算的 atan(2^-i) 表，单位：度
static const float AtanTable[16] = {
    45.0f, 26.565051177f, 14.036243467f, 7.125016348f,
    3.576334374f, 1.789910608f, 0.895173710f, 0.447614170f,
    0.223810500f, 0.111905677f, 0.055952892f, 0.027976452f,
    0.013988227f, 0.006994114f, 0.003497057f, 0.001748528f
};

// CORDIC 增益的倒数 (1/Kn), 对于16次迭代
#define CORDIC_GAIN_INV 0.6072529350088813f

static void cordic_calculate(float angle, float *sin_val, float *cos_val);

/**
 * @brief 对一个浮点数进行四舍五入，保留两位小数。
 *
 * @param value 指向要四舍五入的浮点数的指针。函数将直接修改该指针指向的值。
 *              如果 value 为 NULL，函数不执行任何操作。
 *
 * @note 该函数遵循"四舍五入，五入"的原则（round half away from zero）。
 *       - 3.1415f -> 3.14f
 *       - 3.1465f -> 3.15f
 *       - -3.1415f -> -3.14f
 *       - -3.1465f -> -3.15f
 */
void round_float(float *value)
{
    if (value == NULL)
    {
        return;
    }
    *value = roundf(*value * 100.0f) / 100.0f;
}

/**
 * @brief 使用CORDIC算法计算给定角度的正弦值。
 *
 * @param angle 输入的角度，单位为度。角度范围不限。
 * @return float 返回对应的正弦值。
 */
float my_sin(float angle)
{
    float sin_val, cos_val;
    cordic_calculate(angle, &sin_val, &cos_val);
    round_float(&sin_val);
    return sin_val;
}

/**
 * @brief 使用CORDIC算法计算给定角度的余弦值。
 *
 * @param angle 输入的角度，单位为度。角度范围不限。
 * @return float 返回对应的余弦值。
 */
float my_cos(float angle)
{
    float sin_val, cos_val;
    cordic_calculate(angle, &sin_val, &cos_val);
    round_float(&cos_val);
    return cos_val;
}

/**
 * @brief CORDIC算法核心实现。
 *
 * 该函数包含象限转换和CORDIC旋转计算，用于计算一个角度的正弦和余弦值。
 *
 * @param angle 输入的角度，单位为度。
 * @param sin_val 指向用于存储计算出的正弦值的指针。
 * @param cos_val 指向用于存储计算出的余弦值的指针。
 */
static void cordic_calculate(float angle, float *sin_val, float *cos_val)
{
    float current_x = 1.0f;
    float current_y = 0.0f;
    float target_angle = angle;
    int quadrant = 0; // 0: [0, 90], 1: [90, 180], 2: [180, 270], 3: [270, 360]

    // 1. 将角度归一化到 [0, 360) 范围
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle < 0)
    {
        target_angle += 360.0f;
    }

    // 2. 确定象限并转换到第一象限进行计算
    if (target_angle > 270.0f)
    {
        quadrant = 3;
        target_angle = 360.0f - target_angle; // sin(360-a)=-sin(a), cos(360-a)=cos(a)
    }
    else if (target_angle > 180.0f)
    {
        quadrant = 2;
        target_angle = target_angle - 180.0f; // sin(180+a)=-sin(a), cos(180+a)=-cos(a)
    }
    else if (target_angle > 90.0f)
    {
        quadrant = 1;
        target_angle = 180.0f - target_angle; // sin(180-a)=sin(a), cos(180-a)=-cos(a)
    }
    
    // 3. CORDIC 旋转迭代
    float power_of_2 = 1.0f;
    for (int i = 0; i < 16; ++i)
    {
        float next_x, next_y;
        float sigma = (target_angle >= 0) ? 1.0f : -1.0f;

        // 执行旋转
        next_x = current_x - sigma * current_y * power_of_2;
        next_y = current_y + sigma * current_x * power_of_2;

        // 更新角度
        target_angle -= sigma * AtanTable[i];

        current_x = next_x;
        current_y = next_y;
        
        power_of_2 *= 0.5f; // 相当于 2^-(i+1)
    }

    // 4. 应用增益修正
    *cos_val = current_x * CORDIC_GAIN_INV;
    *sin_val = current_y * CORDIC_GAIN_INV;

    // 5. 根据原始象限调整符号
    switch (quadrant)
    {
        case 0: // [0, 90]: sin > 0, cos > 0
            // No change
            break;
        case 1: // [90, 180]: sin > 0, cos < 0
            *cos_val = -(*cos_val);
            break;
        case 2: // [180, 270]: sin < 0, cos < 0
            *sin_val = -(*sin_val);
            *cos_val = -(*cos_val);
            break;
        case 3: // [270, 360]: sin < 0, cos > 0
            *sin_val = -(*sin_val);
            break;
    }
}
