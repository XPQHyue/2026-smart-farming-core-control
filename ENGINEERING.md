# 工程设计笔记 - STM32智能小车控制系统

> 本文档约120行，记录核心设计决策，索引后即可了解全貌。

## 一、初始化顺序 (关键修复)

**问题**：TIM7中断(200Hz)在PWM/编码器初始化前启动，导致未定义行为。

**解决方案**：严格顺序初始化
```
SysTick → UART → PWM → Encoder → ADC/Key/Buzzer → PID_Init → Balance_Init → IMU → TIM7(中断)
```

**文件**：`USER/main.c` 第70-100行

---

## 二、蓝牙通信架构 (USART2: PD5/PD6)

### 2.1 非阻塞发送
- **原理**：环形缓冲区 + TXE中断驱动
- **缓冲区**：256字节
- **API**：
  ```c
  Bluetooth_SendString("Hello");  // 非阻塞
  Bluetooth_SendNB(data, len);    // 非阻塞二进制
  Bluetooth_TxHandler();          // 中断中调用
  ```

### 2.2 printf重定向
- **修改**：`SYSTEM/usart.c` fputc → USART2
- **效果**：printf直接输出到蓝牙

**文件**：`WHEELTEC_BSP/bsp_bluetooth.c`

---

## 三、PID远程调参系统

### 3.1 协议格式
```
{通道号:数值}     例: {0:2.5} {6:-1.2}
```

### 3.2 通道映射
| 通道 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| 0 | Yaw_Kp | 1.6 | 角度环P |
| 1 | Yaw_Ki | 0.005 | 角度环I |
| 2 | Yaw_Kd | 90 | 角度环D |
| 3 | Omega_Kp | 1.2 | 角速度P |
| 4 | Omega_Ki | 0.01 | 角速度I |
| 5 | Omega_Kd | 80 | 角速度D |
| 6 | Velocity_KP | 4.4 | 速度环P |
| 7 | Velocity_KI | 1.1 | 速度环I |

### 3.3 使用方法
```c
// main.c 初始化时绑定
PID_BindParam(0, &pid_KGain_Yaw.Kp);
PID_BindParam(6, &Velocity_KP);

// 蓝牙发送 {0:3.5} 即可修改参数
```

**文件**：`WHEELTEC_BSP/bsp_bluetooth.c` 第15-50行

---

## 四、控制频率配置

| 模块 | 频率 | 定时器 |
|------|------|--------|
| 主中断 | 200Hz | TIM7 |
| PID角度环 | 50Hz | TIM7/4分频 |
| PID角速度环 | 100Hz | TIM7/2分频 |
| 编码器读取 | 100Hz | TIM7/2分频 |
| 蓝牙波特率 | 9600bps | USART2 |

---

## 五、硬件引脚速查

| 功能 | 引脚 | 说明 |
|------|------|------|
| 蓝牙TX | PD5 | USART2_TX |
| 蓝牙RX | PD6 | USART2_RX |
| 调试串口TX | PA9 | USART1_TX |
| 调试串口RX | PA10 | USART1_RX |
| IMU_I2C | PB10/PB11 | 软件I2C |
| LED | PE8 | 状态指示 |

---

## 六、渐进式披露原则

本项目采用渐进式披露工程思维：

1. **最小可观测**：每个模块独立可测试
2. **分层验证**：底层→中层→应用层逐级验证
3. **问题隔离**：通过初始化顺序保证各模块独立性
4. **远程调参**：无需重编译即可优化参数

---

## 八、运动学逆解算模块 (麦克纳姆轮专用)

### 8.1 模块概述
从原 WHEELTEC 代码中提取并重构的运动学逆解算模块，**仅支持麦克纳姆轮车型**。

**原文参考**：
```c
// 原始代码（WHEELTEC_BSP/balance.c 注释部分）
// Inverse kinematics
MOTOR_A.Target = +Vy + Vx - Vz*(Axle_spacing + Wheel_spacing);
MOTOR_B.Target = -Vy + Vx - Vz*(Axle_spacing + Wheel_spacing);
MOTOR_C.Target = +Vy + Vx + Vz*(Axle_spacing + Wheel_spacing);
MOTOR_D.Target = -Vy + Vx + Vz*(Axle_spacing + Wheel_spacing);
```

### 8.2 麦克纳姆轮运动学公式
```
轮子布局（俯视图）：
       Y^+ (左)
       |
   B   |   C      A=后左, B=前左, C=前右, D=后右
       |
-------+--->X+ (前)
       |
   A   |   D
       |

运动分解：
  - Vx > 0: 前进 → 所有轮子同向前进
  - Vy > 0: 左移 → A,C前进; B,D后退 (对角轮同向)
  - Vz > 0: 逆时针旋转 → A,B后退; C,D前进

逆运动学公式：
A(Rear-Left)  : V_A = +Vy + Vx - Vz*(a+b)
B(Front-Left) : V_B = -Vy + Vx - Vz*(a+b)
C(Front-Right): V_C = +Vy + Vx + Vz*(a+b)
D(Rear-Right) : V_D = -Vy + Vx + Vz*(a+b)

其中：a=轴距/2, b=轮距/2
```

### 8.3 API使用方法
```c
// 1. 初始化（main.c 中调用）
Balance_Init();

// 2. 设置目标速度
float Vx = 0.2f;   // 前进速度，正值前进
float Vy = 0.1f;   // 横移速度，正值左移
float Vz = 0.5f;   // 角速度，正值逆时针
Drive_Motor(Vx, Vy, Vz);  // 计算四轮目标速度(RPM)

// 3. 读取编码器速度（100Hz）
Get_Velocity_From_Encoder();  // 更新 MOTOR_x.Encoder

// 4. 平滑控制（可选）
Smooth_Control(target_vx, target_vy, target_vz);
Drive_Motor(smooth_control.VX, smooth_control.VY, smooth_control.VZ);
```

### 8.4 关键参数 (balance.h)
| 参数 | 值 | 说明 |
|------|-----|------|
| MEC_WHEELSPACING | 0.093m | 轮距/2 |
| MEC_AXLESPACING | 0.085m | 轴距/2 |
| MOTOR_GEAR_RATIO | 30 | 电机减速比 |
| ENCODER_ACCURACY | 13 | 编码器线数 |
| WHEEL_DIAMETER | 0.075m | 轮子直径 |
| AMPLITUDE_LIMITING | 0.6m/s | 最大速度限制 |

### 8.5 数据结构
```c
// 电机控制结构体
typedef struct {
    float Target;    // 目标速度 (RPM)
    float Encoder;   // 当前速度 (m/s)
    int Motor_Pwm;   // PWM输出值
} Motor_Control_t;

// 机器人参数结构体
typedef struct {
    float WheelSpacing;   // 轮距/2
    float AxleSpacing;    // 轴距/2
    int GearRatio;        // 减速比
    int EncoderAccuracy;  // 编码器线数
    float WheelDiameter;  // 轮径
    float WheelPerimeter; // 轮周长
    int EncoderPrecision; // 编码器精度
} Robot_Param_t;

// 全局变量
extern Motor_Control_t MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern Robot_Param_t Robot_Param;
extern Smooth_Control_t smooth_control;
```

### 8.6 编码器极性 (麦克纳姆轮)
```c
// A/B 不变，C/D 取反
float Encoder_A_pr = OriginalEncoder.A;
float Encoder_B_pr = OriginalEncoder.B;
float Encoder_C_pr = -OriginalEncoder.C;
float Encoder_D_pr = -OriginalEncoder.D;
```

**文件**：`WHEELTEC_BSP/balance.c`, `WHEELTEC_BSP/Inc/balance.h`

---

| 文件 | 职责 | 关键行 |
|------|------|--------|
| `USER/main.c` | 主程序、初始化顺序 | 70-100 |
| `WHEELTEC_BSP/bsp_bluetooth.c` | 蓝牙通信、PID调参 | 15-150 |
| `WHEELTEC_BSP/Inc/bsp_bluetooth.h` | API声明 | 1-60 |
| `SYSTEM/usart.c` | printf重定向 | fputc函数 |
| `WHEELTEC_BSP/pid_study.c` | PID算法 | PID_Iterate |
| `WHEELTEC_BSP/balance.c` | 麦克纳姆轮运动学 | Drive_Motor |

---

*文档版本: V1.1 | 更新: 2026.3.24 | 行数: ~120*
