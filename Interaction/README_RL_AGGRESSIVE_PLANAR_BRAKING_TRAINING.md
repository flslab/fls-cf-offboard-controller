# 任意平面方向激进制动 RL：训练设置

## 1. 目标与边界

本实验训练一个 **release 后的平面制动策略**：无人机可能沿任意 XY
方向运动，策略每个控制周期根据最新状态选择反向制动倾角，使无人机尽快在预先计算的
coast 目标附近停止，并在交给 position controller 前回到近水平状态。

这里的“任意方向”指任意世界坐标 XY 速度方向，不是分别为 `+X/-X/+Y/-Y`
训练四个互不相关的策略。所有状态和动作先投影到由 release 运动方向定义的局部坐标系，
同一个 policy 应能处理整个圆周方向。

本设置不训练起飞、悬停、接触检测、release 检测、coast 目标计算、位置接管或降落。
Crazyflie 原有的 attitude controller 继续负责跟踪 roll/pitch 目标。RL 不直接输出电机
RPM。第一次实现和训练均为离线 simulator experiment，不提供真机控制入口。

## 2. 与仓库现有 RL 实验的区别

当前 `Interaction/rl_braking_env.py` 是一个单轴 binary optimal-stopping 环境：

- 动作只有“继续固定反向 20°”和“回水平并永久锁存”两个选择；
- 初始速度只覆盖约 `0.2–1.0 m/s`；
- 没有 coast 目标位置；
- 不处理任意 XY 方向和 position-controller handoff；
- 回水平后直接积分完整尾部，不再允许 policy 做决定。

因此现有 PPO checkpoint 不能直接用于本任务。本文件是下一版环境与训练的实现规格，
不是对现有 checkpoint 的飞行批准。

## 3. 坐标系

在确认 release 的时刻锁存运动方向：

```text
e_parallel = normalize(v_release_xy)
e_perp     = [-e_parallel.y, e_parallel.x]
```

如果 `|v_release_xy|` 太小，则依次使用最近一段可靠速度方向、接触力方向或交互方向；
三者均不可用时不得启动 RL 制动，直接进入确定性的 level/fallback。

后续所有量投影到该坐标系：

```text
v_parallel = dot(v_xy, e_parallel)       # 正值表示仍在原方向前进
v_perp     = dot(v_xy, e_perp)
d_parallel = dot(p_target - p_xy, e_parallel)
d_perp     = dot(p_target - p_xy, e_perp)
```

`e_parallel` 在一次制动 episode 内不得随着带噪速度旋转。世界坐标中的 coast 目标也在
release 时锁存；为避免 position controller 回拉，handoff 目标仍必须满足现有的“只向前
推进、绝不落到飞机后方”规则。

## 4. Policy 接口

### 4.1 观测

每帧至少包含下列经过固定尺度归一化的量：

```text
d_parallel, d_perp
v_parallel, v_perp
theta_parallel, theta_perp
theta_rate_parallel, theta_rate_perp
body yaw rate
battery voltage
time since release
measurement age
previous commanded tilt_parallel, tilt_perp
time since the previous command was sent
```

附加最近 `200 ms` 的观测与已发送命令历史，并为缺失历史提供显式 validity mask。
不得向 actor 提供模拟器隐藏参数、未来真实状态、未来命令或无延迟真值。训练 critic
可以使用隐藏动力学参数，但部署 actor 不可以。

姿态投影必须使用当前 yaw 将世界坐标期望倾角与机身 roll/pitch 正确转换；不能把
世界 `+Y` 永久等同于某个 pitch 符号。

### 4.2 动作

第一版采用连续二维倾角动作：

```text
action[0] = braking_fraction       in [0, 1]
action[1] = lateral_fraction       in [-1, 1]

theta_parallel_command = -action[0] * max_brake_tilt
theta_perp_command     =  action[1] * max_lateral_tilt
```

沿运动方向的动作只能是制动或 level，不能命令正向加速。建议第一阶段设置：

```text
max_brake_tilt   = 20 deg
max_lateral_tilt = 3 deg
max_tilt_slew    = 180 deg/s
control_period   = 0.01 s
```

policy 不直接决定 position handoff。确定性 supervisor 只有在真实测量连续通过速度、
倾角、角速度和 dwell 门限后才允许交接。

## 5. 模拟器

### 5.1 名义模型

从最新且完整的 calibration artifact 读取：

- command-to-attitude delay；
- 二阶姿态响应的 `wn`、`zeta`、gain 和 bias；
- attitude-to-motion gain；
- `+Y/-Y` 的方向差异；
- 命令和测量采样频率。

模拟状态至少包含：

```text
x = [position_xy, velocity_xy,
     projected_tilt_xy, projected_tilt_rate_xy,
     delayed_command_state]
```

模拟器需要保存实际发送的 command history，并分别模拟测量延迟、transport delay 和
控制周期 jitter。不能把延迟后的状态当成当前真值初始化预测。

### 5.2 Domain randomization

随机范围最终由多次真实 calibration 的分布确定。第一版探索范围可以使用下列相对范围，
但必须在报告中标记为“合成假设”，不能声称已由实机确认：

```yaml
dynamics_randomization:
  mass_multiplier: [0.90, 1.10]
  attitude_gain_multiplier: [0.75, 1.25]
  motion_gain_multiplier: [0.75, 1.25]
  natural_frequency_multiplier: [0.65, 1.35]
  damping_ratio_multiplier: [0.65, 1.35]
  extra_command_delay_s: [0.000, 0.080]
  acceleration_bias_m_s2: [-0.20, 0.20]
  drag_multiplier: [0.50, 1.50]
  xy_coupling_fraction: [-0.10, 0.10]

sensor_randomization:
  measurement_delay_s: [0.000, 0.080]
  transport_delay_s: [0.000, 0.030]
  control_period_jitter_s: [-0.003, 0.006]
  velocity_bias_m_s: [-0.025, 0.025]
  velocity_noise_std_m_s: [0.000, 0.020]
  tilt_bias_deg: [-1.0, 1.0]
  tilt_rate_bias_deg_s: [-10.0, 10.0]
  sample_drop_probability: [0.000, 0.030]

operating_randomization:
  battery_voltage_v: [7.0, 8.4]
  release_speed_m_s: [0.05, 1.50]
  release_direction_rad: [-3.141593, 3.141593]
  release_tilt_parallel_deg: [-8.0, 8.0]
  release_tilt_perp_deg: [-4.0, 4.0]
  release_tilt_rate_deg_s: [-100.0, 100.0]
```

不同参数不能全部独立均匀采样。拿到足够实测数据后，应按真实相关性联合采样，例如低电压
同时降低可用加速度并增大响应时间。至少保留一组更宽的 stress distribution，且不能用
stress 测试结果选择 checkpoint。

## 6. Episode 设计

每个 episode 从已确认 release 的状态开始：

1. 采样初始速度、方向、姿态、角速度、电压和动力学参数；
2. 生成一个在当前运动方向前方、物理上可能或略微不可达的 coast 目标；
3. policy 以 `100 Hz` 输出反向制动倾角；
4. policy 输出接近 level 后仍继续模拟完整姿态响应尾部；
5. 满足确定性 handoff 门限或达到最大 episode 时间后结束；
6. 无论是否越过零速，都继续记录完整尾部，不能在第一次 `v≈0` 时提前宣布成功。

建议初始最大制动决策窗口为 `0.8 s`，之后至少观察 `1.5 s`。最大时间只结束模拟，
不等价于成功。

## 7. 训练目标

### 7.1 Dense cost

每个控制周期计算：

```text
position cost       = |d_parallel| + 0.5 |d_perp|
forward speed cost  = |v_parallel|
reverse speed cost  = max(0, -v_parallel)
lateral speed cost  = |v_perp|
attitude cost       = |theta_parallel| + |theta_perp|
rate cost           = |theta_rate_parallel| + |theta_rate_perp|
effort cost         = |commanded tilt|
slew cost           = |u_t - u_(t-1)|
```

反向速度和越过目标后的回退应使用明显高于前进 overshoot 的权重，但 reward 权重不能
被描述成安全保证。

### 7.2 Terminal cost

在完整尾部末端惩罚：

```text
|position error|
|terminal velocity|
|terminal tilt|
|terminal tilt rate|
maximum rollback after first zero crossing
time to satisfy handoff gates
```

建议采用归一化后的起始权重，而不是直接混合不同单位：

```yaml
reward_scales:
  position_m: 0.05
  velocity_m_s: 0.05
  tilt_deg: 3.0
  tilt_rate_deg_s: 20.0

reward_weights:
  running_position: 0.10
  running_velocity: 0.20
  reverse_velocity: 8.00
  rollback: 10.00
  lateral_motion: 1.00
  attitude: 0.05
  angular_rate: 0.03
  command_effort: 0.005
  command_slew: 0.02
  terminal_position: 3.00
  terminal_velocity: 4.00
  terminal_attitude: 2.00
  terminal_rate: 1.00
  elapsed_time: 0.02
```

这些值只作为第一轮 hyperparameter，开发集比较后可以调整；最终测试集不得用于调权重。

## 8. 硬约束与 supervisor

下列条件不依赖 reward，也不交给 policy 自行学习：

```yaml
hard_limits:
  max_total_tilt_deg: 20.0
  max_tilt_slew_deg_s: 180.0
  max_episode_s: 2.3
  stale_state_limit_s: 0.10
  minimum_battery_v: 7.0

handoff_gates:
  abs_parallel_velocity_m_s: 0.05
  abs_lateral_velocity_m_s: 0.03
  abs_planar_tilt_deg: 3.0
  abs_planar_tilt_rate_deg_s: 20.0
  stable_dwell_s: 0.10
```

另外保留以下确定性规则：

- 非有限 policy 输出、推理超时或状态越界：立即 level；
- 状态超出训练支持范围：不运行 policy；
- RL 命令经过限幅和 slew-rate limiter 后才可发送；
- RL 不得关闭 localization、电池、姿态或飞行边界保护；
- position handoff 目标不得位于当前飞机后方；
- 真实速度明显反向且仍在增大时，禁止继续反向制动。

## 9. Curriculum

训练分四阶段，每阶段从前一阶段 checkpoint 初始化：

```text
Stage A: 0.05–0.40 m/s，固定名义动力学，无 sensor delay
Stage B: 0.05–0.80 m/s，加入方向随机化和轻度动力学随机化
Stage C: 0.05–1.20 m/s，加入完整延迟、噪声、电压和 XY coupling
Stage D: 0.05–1.50 m/s，完整随机化，并混入 20% stress episodes
```

速度采样不能被低速区间占满。每个 batch 建议按速度桶均衡：

```text
[0.05, 0.25), [0.25, 0.50), [0.50, 0.80),
[0.80, 1.10), [1.10, 1.50]
```

## 10. 算法与训练预算

第一候选使用连续动作 SAC；PPO 作为独立对照。建议从小网络开始：

```yaml
algorithm: SAC
policy_network: [128, 128]
activation: relu
replay_buffer_size: 1000000
batch_size: 512
learning_rate: 0.0003
gamma: 0.995
tau: 0.005
train_frequency_steps: 1
gradient_steps: 1
warmup_steps: 20000
total_steps_per_seed: 3000000
training_seeds: [11, 29, 47, 83, 131]
```

必须同时报告全部 seed，不能只展示最好的一次。训练环境应向量化并尽可能保持纯数值计算；
本任务不需要图像渲染或完整 3D 场景。

## 11. 数据拆分

真实数据按完整 flight session 拆分，不能随机拆相邻帧：

```text
system-identification flights -> 拟合名义模型和随机范围
development flights           -> 调 reward、网络和随机化范围
held-out validation flights   -> 冻结设计后的模型选择门
final test flights            -> 只做最终比较
```

至少按方向、速度桶和电压桶报告性能。某个平均指标很好，不能掩盖高速、低电压或某个方向
的失败。

## 12. Baseline 与验收指标

在完全相同的 scenario population 上比较：

1. 当前固定制动 pulse；
2. 当前二阶模型滚动 selector；
3. constrained MPC；
4. 新 SAC policy；
5. 可选的 `MPC + residual RL`。

每个方法分别报告：

- reverse episode count 和比例；
- 最大及 p95 rollback；
- 终端速度 mean/p95/max；
- 终点误差 mean/p95/max；
- handoff 时倾角和角速度；
- 到达 handoff gates 的时间；
- 超时和 fallback 次数；
- 每次 inference 的 mean/p95/max 时间；
- 各速度、方向、电压桶的最差结果。

进入 shadow-mode 真机测试前，建议至少满足：

```yaml
offline_acceptance:
  held_out_reversal_rate: 0.0
  stress_reversal_rate_max: 0.001
  held_out_max_rollback_m: 0.02
  held_out_p95_abs_terminal_velocity_m_s: 0.05
  held_out_p95_abs_position_error_m: 0.05
  held_out_p95_handoff_tilt_deg: 3.0
  inference_p99_ms_on_rpi: 5.0
  beats_all_baselines_on_primary_metrics: true
```

这些是实验准入门，不是飞行安全证明。任何 held-out 高速 case 出现明显反向运动，都应
停止在离线阶段并检查模型范围、观测缺失或策略行为。

## 13. 部署顺序

```text
离线训练
  -> 未见过的合成 population
  -> 历史真实 release 状态 replay
  -> Pi 上只测 inference latency
  -> 真机 shadow mode（记录但不发送动作）
  -> 限制最大 3° 的低速 flight test
  -> 8° / 12° 分级测试
  -> 最后才允许 20° 和 1.5 m/s 范围
```

每一级使用新的、此前未参与调参的完整飞行验证。模型文件必须包含训练代码 hash、配置、
随机种子、名义 calibration hash、训练支持范围、评估结果和明确的
`flight_approved: false/true` 标志。默认始终为 `false`。

## 14. 预期实现文件

建议新增独立模块，不直接改变现有 binary 环境：

```text
Interaction/rl_planar_braking_env.py
Interaction/train_rl_planar_braking.py
Interaction/evaluate_rl_planar_braking.py
Interaction/planar_braking_policy_runtime.py       # 最初只允许 shadow mode
Interaction/tests/test_rl_planar_braking_env.py
Interaction/tests/test_rl_planar_braking_eval.py
Interaction/tests/test_planar_braking_runtime.py
```

实现后的训练入口应接受冻结配置和冻结 calibration，而不是隐式读取当前飞行文件：

```bash
python -m Interaction.train_rl_planar_braking \
  --calibration /path/to/frozen_wrench_calibration.json \
  --config /path/to/frozen_training_config.yaml \
  --output /path/to/new_training_run \
  --seed 11
```

输出目录必须不存在，避免覆盖以前实验。任何训练、评估或 shadow-mode 成功都不能自动
修改 active calibration、开启正常 interaction 或绕过现有安全检查。
