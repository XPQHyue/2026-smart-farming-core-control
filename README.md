# 2026 智能农业核心控制

*作者：XPQHyue*

STM32F407VET6 智能小车底盘控制工程，面向国际大学生智能农业装备竞赛马铃薯捡拾机器人赛题的固件与上位机工具集合。项目以 Keil uVision 5 工程为主体，配套 PID 参数整定脚本、发布说明和硬件资料，便于团队协作、版本交付和后续维护。

## 项目内容

- STM32 主控固件：电机控制、编码器采集、IMU 读取、舵机/传感器相关外设初始化
- PID 控制链路：姿态环、速度环及相关调参接口
- 上位机工具：串口 PID 自动整定脚本和图形化工具
- 工程资料：硬件资源说明、工程说明文档、Keil 工程文件

## 目录说明

- `USER/`：应用层代码与 Keil 工程入口
- `WHEELTEC_BSP/`：板级支持包与控制算法实现
- `SYSTEM/`：系统初始化、串口重定向等基础支持
- `FWLIB/`：STM32 标准外设库
- `CORE/`：Cortex-M4 内核相关启动与头文件
- `tools/`：PID 自动整定脚本、GUI 工具与打包脚本

## 快速开始

### 1. 打开工程

使用 Keil uVision 5 打开 `USER/WHEELTEC.uvprojx`。

### 2. 编译固件

在 Keil 中执行 `Project -> Rebuild All`，生成目标文件 `OBJ/WHEELTEC.axf`，并可按工程配置导出 `.hex`。

### 3. 使用 PID 工具

串口自动整定工具位于 `tools/`，可用于下发参数并观察整定过程。

安装依赖：

```powershell
pip install -r tools/requirements.txt
```

运行命令行工具示例：

```powershell
python tools/pid_autotune_serial.py --port COM8 --baud 9600 send --channels 0,1,2 --values 1000,1,1010
```

启动图形界面：

```powershell
pip install -r tools/requirements_gui.txt
python tools/pid_autotune_gui.py
```

## 硬件与接口

项目默认面向 STM32F407VET6 主控板，涉及的核心外设通常包括：

- USART：调试输出和蓝牙通信
- TIM7：实时控制调度
- I2C：IMU / 传感器相关通信
- 编码器：电机速度反馈

具体引脚与资源分配请参考 `STM32F407VET6(C30D-V2.0)主板资源分配说明.md`。

## 版本与发布

本仓库保留了可直接编译的固件工程，以及用于参数调试的上位机工具。后续若发布新版本，建议同步更新：

- 工程说明
- 变更摘要
- 上位机脚本版本
- 关键硬件约束和编译说明

## 许可证

仓库当前未显式声明许可证。若需要公开协作或二次分发，建议在正式发布前补充适用许可证文件。