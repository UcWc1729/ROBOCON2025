# `wheel_chassis` 项目使用文档

## 1. **项目目录结构**

以下是项目的目录结构及其说明：

```
wheel_chassis
 ├── cmake                  # CMake 配置文件
 ├── cmake-build-debug      # 编译输出目录
 ├── Core                   # 核心代码目录
 │    ├── Inc               # 头文件目录
 │    └── Src               # 源文件目录
 ├── Drivers                # STM32 驱动库
 ├── wheel_chassis          # 应用层代码
 │    ├── Inc               # 应用层头文件
 │    │    └── wheelchassis.h  # 底盘控制接口定义
 │    └── Src               # 应用层源文件
 │         └── wheelchassis.c  # 底盘控制逻辑实现
 ├── .mxproject             # STM32CubeMX 配置文件
 ├── CMakeLists.txt         # CMake 构建脚本
 ├── CMakePresets.json      # CMake 预设配置
 ├── startup_stm32f407xx.s  # 启动文件
 ├── STM32F407XX_FLASH.ld   # 链接脚本
 ├── wheel_chassis.ioc      # STM32CubeMX 工程文件
```

---

## 3. **功能模块说明**


### 3.1 **底盘控制模块**

`wheelchassis.h` 和 `wheelchassis.c` 文件实现了底盘的运动控制逻辑。以下是主要功能点：

- **全局变量 `G_WheelChassisMotion`**：
    - 该变量用于存储底盘的运动状态。
    - 修改该变量后，定时中断回调函数调用 `wheelChassis_MotionControl()` 函数以更新底盘的运动状态。

- **运动控制函数**：
    - `wheelChassis_MotionControl()`：根据全局变量中的参数控制底盘的运动。

- **电机速度控制**：
    - `wheelChassis_MotorSpeedControl()`：控制电机的速度以达到期望值。

---

## 4. **使用方法**

### 4.1 初始化系统

在程序启动时，需要完成以下初始化步骤：

1. **初始化 CAN 模块**：
```c
M3508_Can_Start();
```
2.开启定时中断
```c
HAL_TIM_Base_Start_IT(&htim6);
```

### 4.2 控制底盘运动

要控制底盘的运动，请按照以下步骤操作：

1. **修改全局变量**：
   更新 `G_WheelChassisMotion` 的值以设置运动状态、目标速度和方向。例如：
```c
G_WheelChassisMotion.state = Linear;//直线运动
G_WheelChassisMotion.RelativeAngle = 60;//相对正前方顺时针60度
G_WheelChassisMotion.MovementSpeed = 10; //10r/s
```
