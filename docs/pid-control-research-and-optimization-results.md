# PID 控制研究与 c0 基线优化结果

日期：2026-07-14
基线提交：`c0a60d6d3608a68887e59934f3e4c660c2d2b048`

## 结论

本轮优化没有继续沿用工作区中已经证伪的“自定义横向误差反馈 + 随车速收紧转向步进 + 0.90 曲率前馈”方案，而是恢复 CARLA waypoint PID 反馈语义，在其上加入条件抗积分饱和，并用确定性 CARLA 配对实验选择默认参数。

最终默认配置为：

- 保留 c0 的 CARLA waypoint 横向 PID 反馈；
- 横向转向幅值上限 `0.65`；
- 每控制周期转向步进上限 `0.65`，不随车速衰减；
- 曲率前馈默认关闭（增益 `0.0`），但保留 CLI 参数用于后续消融；
- 仅当输出限幅且新误差继续把输出推向饱和方向时，回滚本周期积分输入；
- `reset()` 确定性清空纵向与横向 PID 状态。

这是一项“去回归并增强饱和恢复”的可行优化，不应解读为全场景安全认证。c0 本身在部分弯道场景仍未通过项目硬门槛。

## 开源项目与工程实现

1. [CARLA navigation controller](https://github.com/carla-simulator/carla/blob/ue5-dev/PythonAPI/carla/agents/navigation/controller.py)：以目标 waypoint 的车辆坐标夹角作为横向 PID 误差。本项目恢复了这一反馈语义。
2. [openpilot common PID](https://github.com/commaai/openpilot/blob/master/openpilot/common/pid.py)：包含输出限幅、积分冻结和饱和状态处理，说明抗饱和应与最终执行器限制联动。
3. [openpilot lateral PID](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/controls/lib/latcontrol_pid.py)：展示车辆横向 PID 的增益调度、前馈与饱和监测接口。
4. [ROS control_toolbox PID](https://github.com/ros-controls/control_toolbox/blob/master/control_toolbox/include/control_toolbox/pid.hpp)：提供 back-calculation 与 conditional integration 两类抗积分饱和策略。本轮采用更小改动的 conditional integration。

## 学术文献

1. Åström, K. J. and Hägglund, T., “The future of PID control,” *Control Engineering Practice*, 2001. [DOI: 10.1016/S0967-0661(01)00062-4](https://doi.org/10.1016/S0967-0661(01)00062-4)。核心启示是 PID 的工程价值依赖结构化整定、约束处理和可诊断性，而不只是继续叠加增益规则。
2. Ang, K. H., Chong, G. and Li, Y., “PID control system analysis, design, and technology,” *IEEE Transactions on Control Systems Technology*, 2005. [DOI: 10.1109/TCST.2005.847331](https://doi.org/10.1109/TCST.2005.847331)，[作者存档](https://eprints.gla.ac.uk/3817/)。该综述强调离散实现、微分噪声、饱和与整定方法对实际性能的重要性。
3. Peng, Y., Vrančić, D. and Hanus, R., “Anti-windup, bumpless, and conditioned transfer techniques for PID controllers,” *IEEE Control Systems Magazine*, 1996. [DOI: 10.1109/37.526915](https://doi.org/10.1109/37.526915)。本轮条件积分策略与其约束感知思想一致。
4. Ding, N. et al., “Vehicle path following based on gain-scheduled robust control,” SAE Technical Paper 2013-01-0687. [DOI: 10.4271/2013-01-0687](https://doi.org/10.4271/2013-01-0687)。它支持在车辆动力学确有变化时使用调度，但本项目的既有速度调度直接削弱执行器权限，实验上适得其反。
5. “Reinforcement Learning-Based PID Tuning for Autonomous Driving in CARLA,” 2023. [arXiv:2301.03363](https://arxiv.org/abs/2301.03363)。它说明自动整定可作为后续方向，但在当前确定性评估体系下，先修复反馈语义和饱和处理更可验证。

## 原实现诊断

历史完整矩阵中，工作区原“优化版”相对 c0 的 90 个配对 PID 圈次出现明显回归：

- normalized lateral IAE：`0.10799 → 0.18846`，恶化约 `74.5%`；
- 最大横向误差：恶化约 `76.3%`；
- hard-gate 通过率：下降 `4.44` 个百分点。

根因有三项：

1. 自定义 `atan(k e_y / v)` 反馈替换了 CARLA waypoint 角度误差，同一横向误差在高速时被显著削弱；
2. 转向步进上限从低速约 `0.071` 继续降到 90 km/h 约 `0.034`，高速纠偏权限不足；
3. 默认 `0.90` 曲率前馈与恢复后的 waypoint PID 叠加，在 30 km/h 弯道产生高频饱和振荡。

## 确定性 CARLA 配对结果

所有候选与 c0 的 `trajectory_hash` 均完全一致。下表为最终候选相对 c0 的变化；误差和转向变化为负数表示改善。

| 场景 | normalized IAE | 最大横向误差 | 越线样本变化 | 完成率变化 | 转向变化 P95 |
|---|---:|---:|---:|---:|---:|
| 30 km/h curvy | -1.91% | -5.32% | -10 | 0.00 pp | -2.44% |
| 50 km/h curvy | +0.17% | -0.04% | 0 | 0.00 pp | -21.82% |
| 70 km/h S-curve | 0.00% | 0.00% | 0 | 0.00 pp | 0.00% |
| 90 km/h S-curve | 0.00% | 0.00% | 0 | 0.00 pp | 0.00% |

消融中被拒绝的配置：

- 前馈 `0.90`、步进 `0.10`：30 km/h 横向 IAE 相对 c0 恶化约 `173%`，出现 90.7% 步进限幅命中率；
- 前馈 `0.0`、步进 `0.10`：30 km/h 横向 IAE仍恶化约 `10.5%`；
- 前馈 `0.0`、步进 `0.65`：四个代表场景不回归，因此成为默认值。

结果目录：

- `PythonAPI/my_control_project/log/run_20260714_012401_seed_26030201_curvy`
- `PythonAPI/my_control_project/log/run_20260714_012443_seed_26050301_curvy`
- `PythonAPI/my_control_project/log/run_20260714_012511_seed_26070201_s_curve`
- `PythonAPI/my_control_project/log/run_20260714_012533_seed_26070201_s_curve`

## 验证

- 完整 Python 测试集：234 tests passed；
- PID 反馈、固定权限、负限幅、条件抗饱和、reset、零前馈兼容性与 CLI 消融参数均有单元测试；
- 4 个代表性 CARLA 配对场景通过轨迹哈希一致性检查；
- 尚未重新运行全部 90 个 PID 圈次，因此完整矩阵结论仍以“代表场景验证”表述。
