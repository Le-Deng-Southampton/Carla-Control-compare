# Carla-Control-compare

## English

CARLA controller-comparison project for evaluating LQR, PID, and MPC vehicle
controllers under repeatable Windows CARLA simulation runs.

This repository contains a packaged CARLA Windows environment plus an MSc
controller-comparison workspace in `PythonAPI/my_control_project`. The project
keeps the custom experiment code separate from the official CARLA examples and
runs multiple controllers on the same generated route so that tracking quality,
speed control, steering smoothness, and failure conditions can be compared on a
consistent basis.

### Main capabilities

- Compare LQR, PID, and MPC controllers in one run.
- Run fixed-speed controller-only experiments with `--speed-planner-mode off`.
- Run adaptive high-speed experiments with `--speed-planner-mode adaptive`.
- Apply dynamic target-speed planning from preview curvature, lateral error,
  heading error, and error growth rate.
- Limit speed before curve entry with configurable entry-curvature thresholds.
- Simulate perception-quality inputs with noise, delay, dropout, and smoothing.
- Generate shaped routes including `true_straight`, `straight`,
  `gentle_curve`, `s_curve`, and `curvy`.
- Save CSV metrics, trajectory plots, JSON run configuration, and a
  human-readable Chinese experiment summary for each run.

### Quick start on Windows

Start the desktop launcher from the repository root:

```powershell
.\Start-CARLA-Project.bat
```

The launcher runs `Launch-CARLA-Project.ps1`, starts CARLA if port `2000` is
not already available, waits for the simulator server, and then runs the
controller-comparison script.

Run the project script directly:

```powershell
cd D:\WindowsNoEditor\PythonAPI\my_control_project
.\scripts\run_my_control.ps1
```

Example fixed-speed controller comparison:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode off --target-speed 70 --route-shape true_straight --controllers lqr pid mpc
```

Example adaptive high-speed comparison:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode adaptive --target-speed 120 --route-shape s_curve --controllers lqr pid mpc
```

### Project layout

```text
PythonAPI/my_control_project/
+-- run_my_control.py          # Argument parsing and comparison entry point
+-- scripts/run_my_control.ps1 # Windows/Conda/PYTHONPATH launcher
+-- control/                   # LQR, PID, MPC, and longitudinal control
+-- Speed_Planing/             # Dynamic target-speed planner
+-- road_planning/             # Route generation and tracking geometry
+-- error_providers/           # Ground-truth and perception-like error inputs
+-- experiment/                # Runtime loop, metrics, logging, summaries
+-- tests/                     # Unit and regression tests
```

### Outputs

Experiment outputs are written under:

```text
PythonAPI/my_control_project/log/
```

Each run normally creates `step_log.csv`, `summary.csv`, `run_config.json`,
`trajectory_compare.png`, and `human_summary.txt`. These files are intentionally
excluded from version control because they are generated experiment artifacts.

### Tests

Run the focused project test suite from the repository root:

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -v
```

## 中文

这是一个基于 CARLA Windows 版本的车辆控制器对比实验项目，用于在可复现实验条件下比较
LQR、PID 和 MPC 控制器的路径跟踪、速度控制、转向平顺性和异常终止表现。

仓库包含 CARLA Windows 运行环境，以及位于 `PythonAPI/my_control_project` 的硕士项目实验代码。
自定义代码与 CARLA 官方示例分离，同一次运行中会让多个控制器使用同一条路线，从而保证对比结果公平。

### 核心功能

- 在同一次实验中对比 LQR、PID、MPC 控制器。
- 使用 `--speed-planner-mode off` 运行固定目标速度的纯控制器对比。
- 使用 `--speed-planner-mode adaptive` 运行带自适应速度规划的高速综合实验。
- 根据前方路线曲率、横向偏差、航向偏差和误差增长率动态调整目标速度。
- 在检测到弯道入口时提前限速，降低高速入弯风险。
- 支持感知误差模拟：噪声、延迟、丢帧和平滑。
- 支持 `true_straight`、`straight`、`gentle_curve`、`s_curve`、`curvy` 等路线形状。
- 每次实验保存 CSV 指标、轨迹图、JSON 参数配置，以及中文可读实验总结。

### Windows 快速启动

在仓库根目录运行：

```powershell
.\Start-CARLA-Project.bat
```

该脚本会调用 `Launch-CARLA-Project.ps1`。如果本机 `localhost:2000` 没有正在运行的 CARLA
服务器，它会先启动 CARLA，等待服务器可用后再运行控制器对比实验。

也可以直接运行项目脚本：

```powershell
cd D:\WindowsNoEditor\PythonAPI\my_control_project
.\scripts\run_my_control.ps1
```

固定速度对比示例：

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode off --target-speed 70 --route-shape true_straight --controllers lqr pid mpc
```

自适应高速对比示例：

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode adaptive --target-speed 120 --route-shape s_curve --controllers lqr pid mpc
```

### 项目结构

```text
PythonAPI/my_control_project/
+-- run_my_control.py          # 参数解析和实验入口
+-- scripts/run_my_control.ps1 # Windows/Conda/PYTHONPATH 启动脚本
+-- control/                   # LQR、PID、MPC 和纵向速度控制
+-- Speed_Planing/             # 动态目标速度规划
+-- road_planning/             # 路线生成和跟踪几何
+-- error_providers/           # 真实误差和感知代理误差输入
+-- experiment/                # 运行循环、指标、日志和总结
+-- tests/                     # 单元测试和回归测试
```

### 实验输出

实验结果写入：

```text
PythonAPI/my_control_project/log/
```

每次运行通常会生成 `step_log.csv`、`summary.csv`、`run_config.json`、
`trajectory_compare.png` 和 `human_summary.txt`。这些文件是运行产物，不纳入版本控制。

### 测试

在仓库根目录运行：

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -v
```
