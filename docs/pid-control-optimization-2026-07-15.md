# 当前 PID 控制分析与优化（2026-07-15）

## 结论

本轮以当前 `main` 上的 PID 为基线，没有改变 CARLA waypoint 角度误差语义、条件抗积分饱和、`0.65` 转向幅值/步进权限和关闭曲率前馈的默认策略。实际采用的优化是将横向微分增益从 `Kd=0.38` 降至 `Kd=0.25`。

四个确定性 CARLA 配对场景中，候选版本的横向 IAE 全部改善，最大横向误差全部改善，完成率、碰撞和越线均无回归。因此 `Kd=0.25` 设为新默认值。

同时新增了可配置的微分低通滤波参数 `--pid-derivative-filter-alpha`。`1.0` 完全保留 CARLA 原始微分行为；更小的值增强平滑。由于单独滤波在本轮场景中会牺牲约 3%–6% 横向 IAE，默认值保持 `1.0`，仅作为噪声工况消融和后续联合整定接口。

## 当前实现诊断

当前横向控制以 CARLA `PIDLateralController` 为核心：误差是车辆朝向与目标 waypoint 方向的有符号夹角；微分项是相邻两帧误差差分除以 `dt`；积分项使用长度 10 的误差队列。

已有实现的优点：

- 保留 CARLA waypoint 反馈语义；
- 最终执行器限幅后执行条件抗积分饱和；
- 横向转向权限固定，不随速度削弱；
- `reset()` 清理纵向、横向状态；
- 曲率前馈默认关闭，避免与 waypoint PID 重复补偿。

主要剩余问题是 `Kd=0.38` 对 waypoint 切换产生的离散误差跳变过于敏感。30 km/h 曲线基线中，转向单步变化均值为 `0.01807`、P95 为 `0.08747`、峰值为 `0.35419`，且转向率限幅命中率为 0，说明高频变化来自 PID 反馈本身，而不是执行器限幅。

## 开源项目证据

1. [CARLA navigation controller](https://github.com/carla-simulator/carla/blob/ue5-dev/PythonAPI/carla/agents/navigation/controller.py)：原始横向 PID 直接对 waypoint 角度误差做未滤波差分。本项目保持其误差定义，只调整微分强度并提供可选滤波。
2. [ROS control_toolbox PID](https://github.com/ros-controls/control_toolbox/blob/master/control_toolbox/include/control_toolbox/pid.hpp)：提供 conditional integration 和 back-calculation 两类抗饱和策略。当前实现已采用约束感知的条件回滚。
3. [openpilot common PID](https://github.com/commaai/openpilot/blob/master/openpilot/common/pid.py) 与 [lateral PID](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/controls/lib/latcontrol_pid.py)：强调输出限幅、积分冻结和可调增益；支持以实际闭环数据整定，而不是只依赖标称参数。
4. [ArduPilot AC_PID](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PID/AC_PID.cpp)：对目标、误差和微分使用低通滤波，并让积分增长与限幅方向联动，支持本轮对微分噪声和饱和状态的诊断。

## 学术文献证据

1. *Tuning the feedback controller gains is a simple way to improve autonomous driving performance*（2024）在 CARLA 16 个场景中通过反馈增益重整定提高驾驶得分：[arXiv:2402.05064](https://arxiv.org/abs/2402.05064)。这直接支持先做低维、可解释的增益整定。
2. Carrasco 与 Sequeira 在 CARLA 中以强化学习整定路径跟踪控制器参数，并以横向/转向轨迹误差为目标：[arXiv:2301.03363](https://arxiv.org/abs/2301.03363)。其价值在于证明参数应由闭环轨迹指标联合评价。
3. *Measurement noise filtering for common PID tuning rules* 说明微分作用需要滤波来限制控制变化，同时存在性能与噪声抑制权衡：[DOI 10.1016/j.conengprac.2014.07.005](https://doi.org/10.1016/j.conengprac.2014.07.005)。
4. *Simultaneous tuning of PID controllers and measurement filters* 强调控制器与滤波器应联合整定，而不是独立叠加：[DOI 10.1049/iet-cta.2016.0297](https://doi.org/10.1049/iet-cta.2016.0297)。本轮滤波消融的回归与这一结论一致。

## 确定性 CARLA 配对结果

所有配对的 `trajectory_hash` 一致。下表为 `Kd=0.25` 相对当前 `Kd=0.38` 的变化，负数表示误差或转向变化降低。

| 场景 | 横向 IAE | 最大横向误差 | 平均航向误差 | P95 转向变化 | 完成率 | 碰撞/越线 |
|---|---:|---:|---:|---:|---:|---:|
| 30 km/h curvy | -5.36% | -5.97% | -7.83% | -35.94% | 0.00 pp | 0 / 0 |
| 50 km/h curvy | -13.65% | -11.14% | -9.96% | -37.15% | 0.00 pp | 0 / 0 |
| 70 km/h S-curve | -13.33% | -15.55% | -4.60% | +2.35% | 0.00 pp | 0 / 0 |
| 90 km/h S-curve | -1.76% | -6.85% | +0.38% | +1.22% | 0.00 pp | 0 / 0 |

实验目录：

- 30 km/h 基线/候选：`run_20260715_215024_seed_26030201_curvy` / `run_20260715_215431_seed_26030201_curvy`
- 50 km/h 基线/候选：`run_20260714_012443_seed_26050301_curvy` / `run_20260715_215542_seed_26050301_curvy`
- 70 km/h 基线/候选：`run_20260714_012511_seed_26070201_s_curve` / `run_20260715_215616_seed_26070201_s_curve`
- 90 km/h 基线/候选：`run_20260714_012533_seed_26070201_s_curve` / `run_20260715_215644_seed_26070201_s_curve`

## 被拒绝的候选

30 km/h 配对消融中，单独启用微分滤波得到以下结果：

| 滤波系数 α | 横向 IAE | P95 转向变化 |
|---:|---:|---:|
| 0.75 | +3.26% | -30.74% |
| 0.50 | +4.94% | -59.23% |
| 0.40 | +5.38% | -68.16% |
| 0.25 | +5.68% | -80.25% |

`α=0.50, Kd=0.50` 的联合候选横向 IAE 恶化 `11.68%`，也被拒绝。因此默认只降低 `Kd`；滤波器保持关闭但可配置。

## 边界

这是一组四场景确定性代表性验证，不等同于全场景安全认证。下一阶段若继续优化，应运行多种子、质量/惯量/轮胎摩擦扰动和感知延迟矩阵，再考虑速度调度或自动整定。
