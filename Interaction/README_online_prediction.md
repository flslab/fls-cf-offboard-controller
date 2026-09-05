# Calibration 飞行中的在线预测模型拟合

这是一套 **online system identification（在线系统辨识）**，不是 RL。
使用现有校准飞行的姿态指令、实际姿态/角速度、速度、位置、电池电压和时间戳，
每完成一对 ±Y 试验，就在后台更新一个预测模型。

当前仅拟合 **姿态指令 → 实际倾角 → 速度/位置**。普通 interaction 中不增加
飞行动作，也不修改 contact/release/coasting 控制。`--calibrate` 默认允许冻结的
早期模型缩短后续 calibration brake pulse；第一对仍保持固定脉冲。

## 模型是什么

在试验方向上使用带延迟的二阶倾角响应：

```text
θ'' + 2 ζ ωn θ' + ωn² θ = ωn² [kθ u(t − delay) + bias_y × direction_y]
v' = kv × 9.81 × tan(θ)
p' = v
```

`u` 是沿试验方向的倾角指令，`θ` 是实际倾角投影。
零 yaw 时，沿 +Y 的 `θ ≈ −roll`，沿 −Y 的符号相反。
拟合 delay、ωn、ζ、kθ、bias_y、kv；预测从当前 `p、v、θ、θ'` 开始，
保留已经发出、但由于延迟尚未生效的指令。

这里的 delay 是 **主机时间基准下的有效延迟**，包括遥测与调度，
不能解释成单独测出的电机延迟。角速度使用当前小倾角投影近似，
不是完整刚体/欧拉角模型；X/Z 轴、侧向耦合和位置控制器尚未辨识。

## 飞行时的更新顺序

继续使用已有的六次姿态制动试验：20°，加速/反向脉冲分别为
0.16、0.24、0.32 s，各做 +Y 和 −Y。每次仍有 level 段、恢复位置段和起始稳定检查。

1. 完成 trial 0/1：拟合 v1。
2. 完成 trial 2/3：先用冻结的 v1 验证它们，再加入这两次数据拟合 v2。
3. 完成 trial 4/5：先用冻结的 v2 验证它们，再做全六次数据的 v3 重拟合。

**最后的独立验证结果属于 v2，不属于 v3。** 不按验证集分数挑选最优版本。
全数据 v3 始终标注未独立验证。验证用完整、未参与该版本拟合的 trial；
单次预测只读取起点实测状态，后续实测值只用于评分。
历史/延后验证是“给定已执行指令序列”的条件预测，不冒充当时实时发出的预测。

样本在真实 recovery POSITION 指令成功发送后才提交。拟合在独立低优先级
spawn 进程中执行，队列有界；控制循环不等待 **新增** 的优化器。
旧流程末尾的同步 XYZ/制动拟合没有在此改造。
工作进程在 calibration 采样前启动，可能已起飞；CPU/遥测时序仍需在 Pi 上实测。
本地 fake-clock 测试比较了在线开/关时完整的 Commander 调用序列，两者一致；
这不是 Pi 上硬实时性能的证明。

## 怎么运行

LightBender 的 `Interaction/SFL/translation_inertia.yaml` 已添加：

```yaml
online_prediction_calibration:
  enabled: true
  max_sample_gap_s: 0.06
  max_velocity_rmse_m_s: 0.06
  max_terminal_abs_error_m_s: 0.06
  max_endpoint_abs_error_m: 0.06
```

位置：`Interaction.config.wrench_interaction` 下。
库配置默认关闭，由 controller 的 `--calibrate` 在独立 mission 副本上临时开启。
普通 `--interaction` 和 `--braking-test` 不启动 adaptive braking。
不需要 Arduino，也不应追加 `--sense`。

同步此 controller 的新文件/修改文件及上述 YAML 后，确认 orchestrator
的 `swarm_manifest.yaml` 指向 `translation_inertia.yaml`。
在 **Pi 的 offboard-controller 目录及运行 controller 的 Python 环境** 中准备可选依赖：

```bash
python -m pip install -r requirements_prediction_model.txt
python -c "import numpy, scipy, threadpoolctl; print('prediction dependencies OK')"
```

缺依赖时后台会报告失败，旧校准仍继续，不会得到新的预测模型。
然后在 **电脑上的 LightBender 仓库** 执行：

```bash
cd orchestrator
python3 orchestrator.py --calibrate --skip-record
```

上面的普通 calibration 默认启用 model-guided braking。若要收集原始固定
160/240/320 ms 制动基线，显式使用
`--no-adaptive-braking-calibration`。旧的正向
`--adaptive-braking-calibration` 参数仍可用，但已不再是必需参数。

**这是实际飞行命令，本次开发没有执行。** 保留人工起飞确认；保持原有净空、
边界、速度、倾角、定位新鲜度和 trial readiness 保护。

日志会显示 `Online Prediction Trial Submitted`、`Online Prediction Candidate Fitted`、
`Online Prediction Candidate Validated`、`Online Prediction Calibration Finished`。
判断完成看 `status` 与 `data_complete`，判断误差看每次 held-out trial，
不要只看 `candidate_fitted`。

自适应飞行会分别判断两类结果：预测模型使用自己独立的 held-out 验证门；
旧的一阶 planar braking fit 继续使用原来的严格门限。如果后者在自适应动作下
失败，但校准文件中已有一份当前格式且合格的 planar fit，保存时会保留旧 fit，
仍允许新的独立验证预测模型写入。日志会记录
`Adaptive Planar Calibration Fit Preserved`，最终保存事件中的
`planar_braking_fit_source` 会明确标为保留旧值。首次校准若没有可保留的旧 fit，
仍需先运行一次 `--calibrate --no-adaptive-braking-calibration` 固定基线；任何门限
都不会因此被放宽。

## 保存与失败行为

默认 Pi 上独立报告位于：

```text
/home/fls/fls-cf-offboard-controller/Interaction/prediction_calibration_runs/
  <drone>_<UTC>_<unique>/report.json
  <drone>_<UTC>_<unique>/report.samples.jsonl
```

它们相对于实际 `wrench_calibration_file` 的父目录，不覆盖以前的运行。
完整报告保留每版模型、训练/验证 trial ID、误差、参数可辨识性、采样范围和原始样本。

原 `Interaction/wrench_calibration.json` 的该无人机条目增加：

- `online_prediction_attempt`：最近一次诊断状态和报告路径。
- `prediction_model`：只要完整、有限值的报告已在保存前返回，就保存最后冻结模型；
  `control_eligible` 明确决定它能否用于实验 calibration 控制。

新候选保留共享参数作为旧读取器的兼容证据，但 calibration 控制实际按运动方向选择
`directional_models.positive_y` 或 `directional_models.negative_y`。每个方向独立拟合
delay、ωn、ζ、倾角 gain、bias 和 motion gain，并分别通过可辨识性/参数边界检查。
冻结模型在它自己的下一组 held-out ±Y pair 上得到的每方向末端速度绝对误差，会写入
`terminal_velocity_error_margin_m_s`。只有通过这次独立验证的同一个冻结模型，才可能在
再下一组试验中控制；刚刚用全部已有数据重拟合的新候选仍标记为等待自己的 held-out
验证，不能借用前一版本的验证结果。硬约束实际检查保守速度区间：整段预测的
`minimum velocity lower bound >= -0.02 m/s`，终点区间同时满足
`lower >= -0.02 m/s` 与 `upper <= +0.05 m/s`，而不是只检查点预测。
每个方向单独放行：例如 −Y margin 合格而 +Y 不合格时，−Y 可以缩短脉冲，+Y 仍执行
原固定脉冲。第一候选尚无 held-out 数据时绝不参与控制。

held-out 数值门失败、不可辨识、碰到参数边界或误差过大时仍保存本次模型和
`failed_gates`，但写入 `control_eligible: false`。只有报告不完整、缺样本、含 NaN、
来源不一致或结构损坏时才保留旧 `prediction_model`，且不妨碍原校准保存。
如果某候选自己的 held-out validation 失败，下一对 calibration 自动保持原固定制动
脉冲继续采数，不允许失败模型改变动作。如果只有某方向的误差裕量等于或超过终端
速度容差，则仅该方向退回固定脉冲；不会在开始制动后才因模型不可用而提前 level。
推荐的 20 度 duration sweep 为 `[0.16, 0.24, 0.32, 0.32]`：第一组 0.32 s
提供高速 held-out 证据，重复的第二组 0.32 s 才让已经验证的高速模型实际接管。
若后台尚未完成，不延长控制阶段等待；独立目录保留 partial/collecting 证据，
可用完整飞行日志离线重放。退出时有界关闭后台进程，不无限等待。
独立报告不会自动由现有 orchestrator 下载；完整飞行日志仍可用于下面的重放。

保存的新模型始终 `runtime_enabled=false`、`deployment_approved=false` /
`flight_approved=false`；calibration 内存中的冻结候选仅能缩短当前试验的制动脉冲，
不会因此批准普通 interaction 使用模型。
候选制动动作还必须同时满足预测窗口末端
`|velocity| <= 0.05 m/s` 和 `|tilt| <= 3 deg` 两个硬约束；它们不是位置误差
评分的权重。原来的粗候选会补充单次向量化的 10 ms 制动时长网格（最多 64 个
候选），避免因为候选离散过粗而漏掉可稳定停止的动作。没有合格候选时会明确
level，并在 decision 日志中记录 hard-constraint 状态，不会把不合格候选标为成功。
滚动预测使用二阶系统的解析状态转移，不在控制循环里调用通用矩阵指数。如果一次
预测仍超过配置的计算预算，该次预测不能选择新的脉冲、也不能宣称满足末端硬约束；
只要实测速度仍向前，就继续发送原本固定制动命令到原 deadline（倾角不增大、时间
不延长）。速度已非正、已经 level latch 或到达 deadline 时立即 level。其他模型、
状态或输入错误仍按无命令 fallback 处理，由 adapter level/abort。
0.06 m / 0.06 m/s 是这版 **诊断误差门**，不是准许飞行或保证停准的标准。
要用于真实 release 后的控制，还需独立重复数据、状态/电量覆盖和控制策略验证。

## Release 后的可复用 brake-to-position 封装

`Interaction/predictive_brake_handoff.py` 提供了一个独立、无设备 I/O 的状态机，
供后续把通过验证的模型接入 interaction。它从 **release 时的当前状态** 和一个冻结的
三维目的地开始，没有 calibration 的加速段，也没有继续旧固定制动脉冲的 fallback：

```text
BRAKE（训练数据内的最大反向倾角，模型滚动选择剩余时长）
  -> LEVEL（回水平并等待真实响应尾部）
  -> POSITION（实测状态通过硬门限后锁存目标）

模型/状态/计算失败 -> ABORT_LEVEL（回水平，由调用者结束本次 episode）
```

模型第一次预测应该 level 时不会立即切 position。封装还要求实测纵向/横向速度、
倾角、角速度和加速度通过门限，已经真实发送的 level 指令经过模型响应时间，并连续
稳定一个 dwell，才返回 `action == "position"`。如果制动已经越过原目的地，最终目标
会沿运动方向钳制到实际位置，避免 position controller 把无人机往回拉；横向目标默认
锁存到交接时的实际横向位置。交接后如果惯性继续把飞机推过该目标，后续每次
`decide()` 还会将纵向目标单调向前推进到最新实际位置；目标绝不会再次落到飞机后方。

最小调用方式：

```python
from Interaction.predictive_brake_handoff import (
    predictive_brake_to_position,
    projected_tilt_history_from_world_acceleration,
    validated_prediction_model_for_interaction,
)

model = validated_prediction_model_for_interaction(
    saved_drone_calibration,
    enabled=feature_enabled,
    direction_xy=[0.0, direction_y],
    # 只应在完成后续独立验证后临时显式打开；正式部署应改模型 approval flags。
    allow_validated_experimental_model=True,
)
history = projected_tilt_history_from_world_acceleration(
    actually_sent_acceleration_history,
    [0.0, direction_y],
)
episode = predictive_brake_to_position(
    model,
    initial_state=release_state,
    destination_position=frozen_destination_xyz,
    now_s=release_time_s,
    sent_command_history=history,
    direction_xy=[0.0, direction_y],
    config={"allow_validated_experimental_model": True},
)

decision = episode.decide(now_s, current_state)
if decision["action"] in ("brake", "level", "abort_level"):
    assert actual_send_time_s <= decision["valid_until_s"]
    send_attitude(decision["roll_deg"], decision["pitch_deg"])
    # 仅在 send_attitude 真正成功之后记录；发送失败绝不能伪造历史。
    episode.record_sent(decision, actual_send_time_s)
elif decision["action"] == "position":
    assert actual_send_time_s <= decision["valid_until_s"]
    send_position(decision["position_target"])
```

每个 decision 都带有单调递增的 `decision_sequence` 和最多 30 ms 的
`valid_until_s`，position sender 也必须只发最新且未过期的 target。过期姿态 decision
不应发送；如果它实际上已经被发送，仍须调用 `record_sent` 记录真实输入，该函数会返回
`False`，随后 episode 会 `abort_level`。`abort_level` 发出后，调用者还必须退出该
episode 并进入自己的安全处理，不能把它当作可以继续滚动预测的普通 level。
调用者必须按 `decide -> send -> record_sent -> 下一次 decide` 的顺序执行；不能在
新的 decision 已产生后再发送上一个 decision。

`current_state` 使用与 `ModelBasedBrakingController` 相同的 host clock，包含
`time_s`、`position_xy`、`velocity_xy`、`orientation_rpy_rad`、
`angular_velocity_rad_s`、`state_group_skew_s` 和 `battery_voltage_V`；建议额外提供
`acceleration_xy`。若不提供，加速度门会等至少两个连续速度样本后才可能通过。
因为模型没有辨识 yaw dynamics，制动期间实测 yaw rate 默认必须不超过
0.35 rad/s；position 交接门则检查完整三轴角速度，而不只检查 roll/pitch rate。
目的地、方向和模型在 episode 创建时冻结。当前只支持 world ±Y；X/对角方向、训练
范围外状态、方向误差裕量不达标或验证证据不完整都会 fail closed。
进入 position 后仍须持续调用 `decide()` 并发送它返回的最新 position target，才能维持
动态 no-pullback 保证。定位样本还受单步 0.10 m、等效速率 3.0 m/s、相对冻结目的地
累计向前 0.50 m 的棘轮边界约束；越界不会把异常定位直接变成远距离 position command。
如果实际发送的 roll/pitch 与返回 decision 不一致，`record_sent()` 会尽量按实际姿态
重建并保留这个输入：调用时通过 `actual_attitude={"roll_deg": ..., "pitch_deg": ...,
"yaw_rate_deg_s": ..., "projection_yaw_rad": ...}` 传入 lower layer 实际采用的值。
它同时锁存协议错误；下一次调用只会得到 `abort_level`，不会在遗漏执行历史的情况下
继续预测制动。

这个模块目前**没有自动接入普通 interaction**，也不会改变现有飞行动作。保存模型仍
默认不获 runtime approval；等补测完成后，再在现有 release 分支中显式创建 episode，
并保持“成功发送后才 `record_sent`”和 `abort_level` 的外部退出处理。

## 80 ms 在线平动残差（默认关闭）

`ModelBasedBrakingController` 现在包含一个可选的因果残差观察器。它只使用最近最多
80 ms 的实测投影速度与实测倾角，至少需要 60 ms、4 个真实样本：

```text
b = (measured delta-v - integral(k * g * tan(theta) dt)) / measured delta-t
```

该修正只从测量锚点起作用 80 ms，之后恢复冻结基础模型。重复使用同一旧状态不会延长
有效期；时间倒退、冲突时间戳、超过 40 ms 的采样间断或超过 1.5 m/s^2 的异常残差会
清空修正。候选的硬约束同时加入 2-sigma 动态速度裕量和已有 held-out 静态裕量。

配置项 `motion_residual_observer_enabled` 默认是 `false`。当前离线结果只支持继续研究，
尚未验证完整候选选择与位置交接；因此不能用旧基础模型的 `control_eligible` 结果为这个
混合预测器自动授权，更不能据此打开普通 interaction。

可用下面的只读工具做相邻 ±Y pair 的 leave-pair-out 比较；输出目录必须不存在：

```bash
python -m Interaction.evaluate_motion_residual /absolute/path/to/lb11_log.json \
  --output /absolute/path/to/a-new-output-directory
```

## 定向高速制动标定

需要补充高速数据时，使用显式 opt-in 参数；默认 `--calibrate` 行为不变：

```bash
python controller.py --calibrate --targeted-braking-calibration \
  --log --smooth-controller-rate 100
```

该模式固定使用 20 度、每次加速 0.32 s，并以 0.16 / 0.20 / 0.24 s 三个制动时长、
每组两次、交替 ±Y，共 12 个 trial。它强制关闭 adaptive pulse replacement，保证收集到
确定的时长响应，同时保留原有边界、速度、定位新鲜度和姿态安全检查。

## 不飞也能拟合现有日志

在 offboard-controller 仓库、安装已有 numpy/scipy 依赖的环境中：

```bash
python -m Interaction.fit_online_prediction /absolute/path/to/lb11_log.json \
  --output /absolute/path/to/a-new-output-directory
```

输出目录必须不存在。该命令只生成诊断报告，不读串口、不连接飞机、
不更新在用校准。历史命令时钟由实际命令日志对齐，`context` 中明确记录近似性。

## 已有日志的首轮结果

数据：`translation_inertia_2026-09-03_16-07-59`，919 条姿态阶段样本，六个完整 trial。
前四次拟合的 v2 在最后两次（0.32 s 脉冲）上验证：

| 项目 | 结果 |
|---|---:|
| 有效延迟 | 26.25 ms |
| 二阶自然频率 ωn | 14.68 rad/s |
| 阻尼比 ζ | 0.992 |
| 倾角增益 kθ | 1.0349 |
| 加速度增益 kv | 1.0831 |
| 平均 trial 速度 RMSE | 0.04156 m/s |
| 最大末端速度绝对误差 | 0.01244 m/s |
| 最大末端位置绝对误差 | 5.685 cm |
| 反向运动分类 | 两次实际反向，两次预测反向 |

50 / 100 / 200 / 400 ms 条件滚动预测的位置 RMSE 分别约为
0.38 / 0.78 / 1.58 / 2.74 cm。第一版 v1 在中间两次试验有一次误报反向，
已保留该失败；第二版改善后才通过最后两次诊断门。
这只是同一场飞行内的 held-out 验证，不是跨飞行泛化或平稳停车保证。
末端指固定 level-after-brake 观测窗口结束，不一定是实际停止时刻。

报告：`Interaction/analysis/online_prediction_2026-09-04_verified/summary.md`。
本地三次模型拟合约 38 / 72 / 110 ms，验证约 14 / 15 ms；
这不是 Pi 的速度保证，报告中保留 `fit_elapsed_s` / `evaluation_elapsed_s` 供实测。
最后全数据拟合出现 BLAS 数值警告，但返回参数/代价有限，且代价经独立逐元素重算一致；
警告保存在 `numerical_diagnostics`，未静默删掉。非有限或代价不一致会拒绝该版模型。
本轮拟合未写入现有 `wrench_calibration.json`，也未 SSH、部署或 push。
