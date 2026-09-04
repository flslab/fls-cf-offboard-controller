# 离线 PPO 制动策略实验

本模块只在电脑上的模拟器里训练。**不连接无人机，不替换 interaction，不写入校准，不提供实机运行入口。**
技术上是 simulator-trained PPO：策略与模拟器交互学习，而不是只从一份固定飞行数据集进行 offline RL。

## 策略学什么

只学习单轴、沿原运动方向投影的“继续制动 / 解除制动”时机：

- 动作 0：继续发送反向 20° 姿态目标。
- 动作 1：发送水平目标并锁定，再模拟完整 1.5 s 的姿态尾部；不能再次制动。
- 每 10 ms 决策；最多制动 0.5 s，然后强制水平并观察完整尾部。
- 没有正向补偿脉冲、位置控制器或可移动目标。因此不能声称策略已经学会在 coast 终点停稳。

观测包括延迟且带固定偏差的速度、倾角、角速度、位置、测量年龄，12 帧历史、过去已发送命令及已经制动的时间。没有历史时用有效位标记，不伪造过去的数据。模型参数和真实传感器偏差不提供给策略。

训练会随机化初态和模型：这些是探索性的模拟条件，并非已经由实机确认的误差范围。初始状态独立采样，是受扰动的合成状态，不是假装重新执行了实际加速过程。传感器扰动为每个 episode 固定偏差，不是逐帧白噪声。

## 奖励和验收

奖励仅在完整尾部结束后给出，是以下损失的负值：

```text
abs(末段平均速度 - 0.025)
+ 4 * max(0, -全过程最小速度)
+ 2 * 最大回退距离
+ 0.03 * 制动时长
```

速度为 m/s、距离为 m、时间为 s。训练用 gamma=1，避免通过拖延负终止奖励而获得虚假的改进。反向或暂时速度接近零都不会提前结束尾部观察。

验证另外报告：反向次数、最大回退、末速度、低速水平状态数、近静止数、制动时间和位移。**较低损失不代表更安全；没有反向不代表停稳。** 近静止只表示最后 100 ms 内满足原模拟器的有限速度/姿态阈值，不保证长期保持位置。

## 代码

- `rl_braking_env.py`：Gymnasium 环境，复用独立 RK4 plant。
- `train_rl_braking.py`：CPU PPO 训练、开发集检查点选择、完整种子与版本记录。
- `evaluate_rl_braking.py`：独立模拟测试和三个成对基线。
- `rl_experiment_ledger.py`：只读的开发集指标查询。

基线包括固定 240 ms、原预测器（0.08 m/s 余量）、预测器（声明的 0.025 m/s 对照余量）。所有方法使用相同初态、模型、传感器偏差与时延。PPO 不得用最终测试结果选择检查点。

## 安装与训练

以下命令在 offboard-controller 仓库根目录执行。独立环境不影响飞行用 `venv`：

```bash
venv/bin/python -m venv .codex_tmp/braking-rl-venv
.codex_tmp/braking-rl-venv/bin/python -m pip install -r requirements_braking_rl.txt
```

版本固定到本次 macOS arm64 实验；其他系统可能需要兼容的 wheel，并应重新记录版本。完整依赖记录见实验目录的 `environment-freeze.txt`。

```bash
MPLCONFIGDIR="$PWD/.codex_tmp/rl-mpl-cache" \
.codex_tmp/braking-rl-venv/bin/python -m Interaction.train_rl_braking \
  --frozen-report /path/to/frozen_validation/report.json \
  --output /path/to/new_training_directory \
  --seed 11 --timesteps 65536 --n-envs 4 --threads 1 \
  --eval-count 64 --eval-seed 9001
```

输出目录必须不存在。`--resume /path/to/model_best.zip` 可增加训练预算；它是权重/优化器热启动，重新开始随机流与 episode，不是逐比特续跑。不要拿测试集 seed 当作开发集 seed。

输出包括 `model_initial.zip`、`model_best.zip`、`model_final.zip`，各自的元数据、软件与源码哈希、开发场景、实际训练场景 JSONL、开发指标曲线以及 `statistics.json`。`model_best.zip` 只按开发集平均损失选择。

## 独立评估

```bash
.codex_tmp/braking-rl-venv/bin/python -m Interaction.evaluate_rl_braking \
  --frozen-report /path/to/frozen_validation/report.json \
  --policy ppo_seed11=/path/to/model_best.zip \
  --output /path/to/new_evaluation_directory \
  --population-seed 491723 --count 256
```

默认同时评估三个基线。可重复 `--policy LABEL=PATH` 比较不同训练种子。`--observed-log` 与 `--exclude FILENAME:SEGMENT` 可添加以前的实测初态回归测试，之后轨迹仍由模拟器积分。

输出 `manifest.json`、`report.json`、`trajectories.json` 和 `README.md`。合成独立测试、已看过的实测初态回归分别统计，不能合并成“实机成功率”。脚本不会训练或修改策略。

## 本轮边界

实验记录位于 `autoresearch/loop-260903-1847`。使用 autoresearch 的有界训练与验证流程，最多三轮开发预算；在已有未提交文件的工作区中，用模型检查点哈希记录保留/淘汰，不自动 commit/revert/push。

这版实验不能验证定位掉帧、无线通信故障、多轴耦合、飞行边界、载荷改变或实际位置接管。任何模拟改进都需要后续独立实机验证，不能直接用模型替换现有飞控逻辑。
