# 四轮全向底盘控制系统 (A_f427_motor)

## 项目简介

本项目是基于STM32F427微控制器的四轮全向底盘控制系统，采用大疆DJI 3508电机配合C620电调实现精确的全向移动控制。系统支持前进、后退、左移、右移、左转、右转六个基本运动方向，并通过串口实现实时PID参数调节和运动控制。

## 硬件架构

### 主控芯片（大疆A板）
- **MCU**: STM32F427IIHx (ARM Cortex-M4, 180MHz)
- **开发环境**: MDK-ARM (Keil uVision)
- **调试接口**: SWD

### 电机驱动系统（运球机器底盘）
- **电机型号**: DJI 3508 无刷电机 × 4
- **电调型号**: DJI C620 电调 × 4  
- **减速比**: 19:1
- **通信协议**: CAN总线 (500kbps)
- **电机布局**:
  ```
  motor1(0x201)  ●——————● motor4(0x204)
       |                    |
       |      底盘          |
       |                    |
  motor2(0x202)  ●——————● motor3(0x203)
  ```

### 通信接口
- **CAN1**: 电机通信总线 (500kbps)
- **USART2**: 串口调参和控制接口 (115200bps)
- **GPIO**: 状态指示和扩展接口

## 软件架构

### 目录结构
```
A_f427_motor/
├── Core/                    # STM32 HAL库核心代码
│   ├── Inc/                # 核心头文件
│   └── Src/                # 核心源文件
│       ├── main.c          # 主程序 (当前版本)
│       ├── main_optimized.c # 优化版主程序
│       ├── can.c           # CAN总线配置
│       ├── usart.c         # 串口配置
│       └── ...
├── User/                   # 用户自定义代码
│   ├── dvc_motor.h/.cpp    # 电机驱动类
│   ├── alg_pid.h/.cpp      # PID算法类
│   ├── dvc_serialplot.h/.cpp # 串口调参类
│   ├── drv_can.h/.c        # CAN驱动封装
│   ├── drv_uart.h/.c       # 串口驱动封装
│   └── drv_math.h/.cpp     # 数学工具函数
├── Drivers/                # STM32 HAL驱动库
├── MDK-ARM/                # Keil工程文件
└── README.md               # 项目说明文档
```

### 核心类与模块

#### 1. 电机控制类 (`Class_Motor_C620`)
```cpp
// 主要功能：
- PID角度环/速度环控制
- CAN通信数据处理
- 电机状态监控
- 目标值设置

// 关键方法：
motor.Init(CAN_HandleTypeDef, CAN_ID, Control_Method, Gearbox_Rate);
motor.Set_Target_Omega(float omega);      // 设置目标角速度
motor.CAN_RxCpltCallback(uint8_t *data);  // CAN数据接收回调
motor.TIM_PID_PeriodElapsedCallback();    // PID计算更新
```

#### 2. PID控制类 (`Class_PID`)
```cpp
// 特性：
- 支持积分限幅、输出限幅
- 变速积分、积分分离
- 微分先行
- 死区处理

// 关键方法：
pid.Init(Kp, Ki, Kd, Kf, I_Max, Out_Max, dt, Dead_Zone);
pid.Set_Target(float target);
pid.Set_Now(float current);
pid.TIM_Adjust_PeriodElapsedCallback();
```

#### 3. 串口调参类 (`Class_Serialplot`)
```cpp
// 功能：
- 变量实时调节
- 数据可视化输出
- 协议解析

// 支持变量：
"pa", "ia", "da"  // 角度PID参数
"po", "io", "do"  // 速度PID参数  
"torque", "fx"    // 扭矩控制和方向选择
```

### 全向轮运动学

#### 坐标系定义
- **X轴**: 向前为正方向
- **Y轴**: 向左为正方向  
- **Z轴**: 向上为正方向 (右手坐标系)
- **旋转**: 逆时针为正方向

#### 运动学方程
```cpp
// 输入：底盘三自由度速度 (vx, vy, w)
// 输出：四个轮子的角速度

wheel1 = vx + vy + w * CHASSIS_RADIUS;  // 左前轮
wheel2 = vx - vy + w * CHASSIS_RADIUS;  // 左后轮  
wheel3 = -vx - vy + w * CHASSIS_RADIUS; // 右后轮
wheel4 = -vx + vy + w * CHASSIS_RADIUS; // 右前轮
```

#### 运动模式
| fx值 | 运动模式 | 运动学参数 |
|------|----------|------------|
| 0    | 停止     | (0, 0, 0) |
| 1    | 前进     | (v, 0, 0) |
| 2    | 后退     | (-v, 0, 0) |
| 3    | 右移     | (0, -v, 0) |
| 4    | 左移     | (0, v, 0) |
| 5    | 左转     | (0, 0, w) |
| 6    | 右转     | (0, 0, -w) |

## 配置参数

### 系统参数
```cpp
#define CHASSIS_RADIUS      0.2f        // 底盘半径 (m)
#define FRONT_SCALE         1.05f       // 前轮补偿系数
#define BACK_SCALE          1.0f        // 后轮补偿系数
#define GEAR_RATIO          19.0f       // 电机减速比
#define MAX_WHEEL_SPEED     500.0f      // 最大轮速 (rad/s)
```

### PID参数 (当前优化值)
```cpp
// 速度环PID
Kp = 80.0f    // 比例系数
Ki = 0.0f     // 积分系数  
Kd = 0.0f     // 微分系数
I_Max = 300.0f    // 积分限幅
Out_Max = 2000.0f // 输出限幅
```

### 滤波参数
```cpp
float alpha = 0.2f;  // 低通滤波系数 (0-1)
vx_filtered = alpha * vx_target + (1-alpha) * vx_filtered;
```

## 使用方法

### 1. 硬件连接
1. **电源**: 24V直流电源为电调供电
2. **CAN总线**: 将四个C620电调串联到CAN1总线
3. **串口**: USART2连接到调试工具 (115200, 8N1)
4. **电机ID设置**: 通过DJI调参软件设置电机ID (0x201-0x204)

### 2. 编译下载
1. 使用Keil MDK打开 `MDK-ARM/A_f427_motor.uvprojx`
2. 编译工程 (Ctrl+F7)
3. 下载到STM32F427 (F8)

### 3. 串口控制
使用串口调试助手或SerialPlot软件连接USART2：

#### 参数调节指令格式
```
pa=80.0;     // 设置角度环P参数
po=60.0;     // 设置速度环P参数
torque=2.0;  // 设置运动速度/扭矩
fx=1.0;      // 设置运动方向 (1=前进)
```

#### 运动控制指令
```
fx=0;  // 停止
fx=1;  // 前进 (torque设置的速度)
fx=2;  // 后退
fx=3;  // 右移  
fx=4;  // 左移
fx=5;  // 左转
fx=6;  // 右转
```

### 4. 调试步骤
1. **连接检查**: 验证CAN通信正常，电机反馈数据有效
2. **PID调参**: 先调速度环，再调角度环
3. **运动测试**: 从低速开始测试各方向运动
4. **参数优化**: 根据实际表现调整补偿系数和滤波参数

## 故障排除

### 常见问题

#### 1. 轮子间相互干扰
**现象**: 控制一个电机时其他电机也会动作
**解决方案**: 
- 确保已禁用`chassis_contral.h`中的同步控制
- 检查CAN ID是否设置正确且无冲突
- 使用优化版本`main_optimized.c`

#### 2. 运动不平滑/振荡
**现象**: 电机抖动、运动轨迹不稳定
**解决方案**:
- 降低PID参数，特别是Kp和Kd
- 增加滤波系数alpha的值
- 检查机械装配是否牢固

#### 3. CAN通信异常
**现象**: 电机无反应或反馈数据异常
**解决方案**:
- 检查CAN总线接线 (CAN_H, CAN_L, GND)
- 确认终端电阻正确连接 (120Ω)
- 验证波特率设置 (500kbps)

#### 4. 串口调参无效
**现象**: 发送参数后电机行为无变化
**解决方案**:
- 检查串口连接和波特率 (115200)
- 确认指令格式正确 (变量名=数值;)
- 重新发送fx指令激活新参数

### 性能优化

#### 1. 提高响应速度
- 减小控制周期 (当前1ms PID, 10ms运动更新)
- 优化滤波参数
- 调整PID参数以获得更快响应

#### 2. 提高稳定性  
- 使用更保守的PID参数
- 增加机械阻尼
- 改善电源质量

#### 3. 功能扩展
- 添加位置反馈传感器 (编码器/IMU)
- 实现轨迹跟踪控制
- 集成上位机控制接口

## 版本历史

### v1.3 (当前版本)
- ✅ 解决轮子间干扰问题
- ✅ 优化串口调参逻辑
- ✅ 添加运动状态管理
- ✅ 改进滤波器实现
- ✅ 完善Google风格注释

### v1.2 (main_optimized.c)
- ✅ 实现状态机控制
- ✅ 分离PID调参和运动执行
- ✅ 优化控制频率管理

### v1.1 (main_fixed.c)  
- ✅ 修正全向轮运动学算法
- ✅ 禁用同步控制系统
- ✅ 改进CAN回调处理

### v1.0 (初始版本)
- ✅ 基础电机控制功能
- ✅ CAN通信实现
- ✅ 串口调参接口

## 技术支持

### 开发工具
- **IDE**: Keil MDK-ARM 5.37+
- **调试器**: ST-Link V2/V3
- **串口工具**: SerialPlot, XCOM
- **版本控制**: Git

### 参考资料
- [STM32F427 数据手册](https://www.st.com/resource/en/datasheet/stm32f427ii.pdf)
- [DJI 电机开发指南](https://www.dji.com/cn/robomaster-s1)
- [全向轮运动学原理](https://en.wikipedia.org/wiki/Mecanum_wheel)
- [中国科学技术大学 RoboWalker战队 电控开源架构](https://github.com/yssickjgd/robowalker_train)


