# 项目指南

## 适用范围
- 本工作区面向 NUEDC 2026 的 STM32F407VET6 机器人控制固件。
- 本文件作为编码代理的默认常驻指导。
- 规则保持精简且可执行，详细内容优先链接现有文档，避免重复拷贝。

## 构建与验证
- 构建环境：Keil uVision 5.x + ARMCC V5.06（不是 ARM Compiler v6）。
- 打开工程：`USER/WHEELTEC.uvprojx`。
- IDE 构建命令：Project -> Rebuild All (F7)。
- 输出产物：`OBJ/WHEELTEC.axf` 与生成的 `.hex`。
- 可选清理脚本：`keilkilll.bat`。
- 代码修改后，优先验证以下已知编译问题：
  - ARMCC #136：IMU 结构体字段不匹配（`gyro` 字段访问）。
  - ARMCC #1-D：`WHEELTEC_BSP/pid_study.c` 文件结尾换行告警。

## 架构边界
- 主控制入口：`USER/main.c`。
- 核心时序模型：
  - 主循环负责非实时任务（传感器轮询、遥测发送、电池采样触发处理）。
  - `TIM7_IRQHandler` 负责 200 Hz 实时控制，并下采样执行 100 Hz / 50 Hz 子任务。
- 分层边界：
  - `WHEELTEC_BSP/`：驱动与控制模块。
  - `SYSTEM/` 与 `USER/`：板级/系统初始化与中断胶水层。
  - `FWLIB/`：厂商库，除非任务明确要求，否则视为外部基线，不做改动。

## 关键约定
- `main()` 初始化顺序属于安全关键。
  - TIM7 初始化与启动必须放在末尾，位于 PWM、编码器与 PID 初始化之后。
  - 未经论证，不要在硬件初始化与 TIM7 启动之间插入新的运行时关键逻辑。
- 下列全局量视为 ISR 持有的输出，不允许在主循环或普通任务中直接写入：
  - `MotorA`、`MotorB`、`MotorC`、`MotorD`
  - `EncoderA`、`EncoderB`、`EncoderC`、`EncoderD`
- 优先使用现有控制接口与 PID 流程，不要绕过流程直接写电机输出。
- `TIM7_IRQHandler` 应保持轻量，避免放入高开销逻辑。

## 传感器与控制陷阱
- VL53L0X 故障值 `65535` 为预期失败场景，需保留回退处理，防止 PID 失控。
- IMU 数据结构命名需在 `main.c`、`bsp_icm20948.c`、`WHEELTEC_BSP/Inc/bsp_icm20948.h` 三处保持一致。
- 若在 ISR 新增控制标志或计数器，必须确认有消费路径；无消费的调度标志应删除。

## 文件与命名规范
- BSP 驱动文件命名使用 `bsp_<module>.c/.h`。
- 控制模块命名使用 `pid_study.c`、`robot_state.c` 这类风格。
- BSP 头文件集中在 `WHEELTEC_BSP/Inc/`。
- 不要编辑 `OBJ/` 下的生成产物。

## 代理工作流程
- 进行较大改动前，先阅读：
  - `ENGINEERING.md`
  - `USER/main.c`
  - 对应的 `WHEELTEC_BSP/` 模块
- 缺陷修复优先做局部最小改动，并保持现有 API 稳定。
- 涉及控制环行为变更时，需说明频率影响与安全影响。
- 若当前会话无法完成编译/测试，必须在结论中明确说明。

## 文档链接
- 系统设计与工程决策：`ENGINEERING.md`。
- 板级引脚与资源映射：`STM32F407VET6(C30D-V2.0)主板资源分配说明.md`。