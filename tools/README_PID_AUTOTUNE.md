# 串口 PID 自动整定工具

本目录提供一个上位机脚本, 用于通过串口向 STM32 下发参数, 协议与固件保持一致:

- 单参数格式: `{通道:值}`
- 三参数一组固定发送: `{a:x}{b:y}{c:z}`

当前固件已绑定测试对象为 VL53L0X 前向 PID:
- 通道 `0` -> `pid_KGain_F.Kp`
- 通道 `1` -> `pid_KGain_F.Ki`
- 通道 `2` -> `pid_KGain_F.Kd`

参数缩放规则:
- 固件侧按 `gain = 接收值 / 1000.0`
- 例如发送 `{0:1000}{1:1}{2:1010}` 对应 `Kp=1.000, Ki=0.001, Kd=1.010`

脚本文件: `pid_autotune_serial.py`

图形界面文件: `pid_autotune_gui.py`（高性能重构：PySide6 + pyqtgraph）

## 1. 环境准备

在本目录安装依赖:

```powershell
pip install -r tools/requirements.txt
```

## 2. 手动发一组参数

示例: 一次发送 VL53 前向 PID 的 Kp/Ki/Kd = 1.000 / 0.001 / 1.010

```powershell
python tools/pid_autotune_serial.py --port COM8 --baud 9600 send --channels 0,1,2 --values 1000,1,1010
```

发送内容将是:

```text
{0:1000}{1:1}{2:1010}
```

## 3. 图形化界面（推荐）

首次使用请安装 GUI 依赖:

```powershell
pip install -r tools/requirements_gui.txt
```

启动 GUI:

```powershell
python tools/pid_autotune_gui.py
```

界面支持:

- 串口下拉选择（默认可填 COM6）
- 一键发送三参数组
- 一键启动/停止自动整定
- 实时日志显示整定过程输出
- 实时曲线面板：
	- 误差曲线（MAE）
	- 参数曲线（Kp/Ki/Kd）
- 高性能绘图与更流畅的实时更新（相较 tkinter 版本）
- 一键“冻结最优并导出”（发送最优参数并写入文本）
- 自动终止参数：
	- 耐心轮数（连续 N 轮无有效提升则停止）
	- 最小改进值（小于该阈值视为无有效提升）

## 4. 自动整定模式

示例:

```powershell
python tools/pid_autotune_serial.py --port COM8 --baud 9600 auto --channels 0,1,2 --init 1000,1,1010 --step 120,1,120 --min 0,0,0 --max 4000,200,4000 --rounds 8 --settle 0.8 --measure 2.0 --value-index 0 --target-index 3
```

参数说明:

- `--channels`: 需要整定的三参数通道号
- `--init`: 三参数初始值
- `--step`: 初始搜索步长
- `--min` / `--max`: 参数边界
- `--rounds`: 最大迭代轮数
- `--settle`: 每次发送后等待稳定时间（秒）
- `--measure`: 采样时间窗口（秒）
- `--value-index`: 串口 CSV 中被控量列
- `--target-index`: 串口 CSV 中目标值列
- `--patience`: 自动终止耐心轮数（默认 3）
- `--min-improve`: 最小有效改进阈值（默认 0.01）
- `--export-best`: 自动整定结束时导出最优参数文本路径

## 5. 评分规则（经验法）

脚本从串口回传 CSV 中提取误差 `e = target - value`, 计算:

- MAE（绝对误差均值）
- RMS（均方根误差）
- STD（测量波动）
- 过零次数（粗略振荡指标）

最终得分越小越好:

```text
score = MAE + 0.5*RMS + 0.2*STD + 0.05*ZeroCross
```

## 6. 重要说明

- 当前固件解码是整数通道值, 脚本会对发送值四舍五入为整数。
- 若你要整定 Kp/Ki/Kd, 必须保证固件把对应通道绑定到这三个参数。
- 若串口回传不是 CSV 或列索引不同, 需要调整 `--value-index` / `--target-index`。
- 自动整定属于经验法, 建议先小范围边界测试。

## 7. 打包为 Windows EXE（Win7+）

为方便团队协作，已提供一键打包脚本：

- `tools/build_pid_autotune_gui_exe.bat`

执行方式：

```powershell
tools\build_pid_autotune_gui_exe.bat
```

输出目录：

- `dist/PID_Autotune_GUI/`
- 主程序：`dist/PID_Autotune_GUI/PID_Autotune_GUI.exe`

分发建议：

- 建议把整个 `dist/PID_Autotune_GUI/` 文件夹打包后发给队友（不要只发 exe 单文件）。
- 目标机器无需安装 Python。

兼容性建议（重点）：

- 若要求“尽量覆盖 Win7”，建议在 Python 3.8 环境下重新打包一版（对老系统更稳）。
- 若仅 Win10/Win11，当前环境打包结果可直接使用。
