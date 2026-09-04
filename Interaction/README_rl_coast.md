# 第二版离线训练：固定 coast 终点与位置接管

这组模块只在本机模拟器运行；**不连接无人机、不改变 interaction、不写入校准，也没有实机部署入口。** 第一版所有源码和模型保持不变。

## 这次改变的任务

第一版仅学习“何时从20度制动回到水平”。第二版学习“何时结束姿态制动，把控制交给固定终点的位置控制器”。动作仍是二元选择，不学习电机输出或接触检测。

```text
release时给定固定终点 → 继续姿态制动，或立即接管位置
                              ↓
                      不可逆的位置接管
                              ↓
                继续模拟到共同的3秒截止时间
```

接管时不清零倾角/角速度，不取消已经发出的延迟命令，不移动目标。位置反馈和模式切换都有延迟，旧命令会在等待期间继续产生运动。

## 哪些来自测量，哪些是假设

- 复用此前从实测拟合并冻结的姿态响应模型、运动增益及其来源记录。
- **位置环是未辨识的简化 PD 模型**，不是 Crazyflie 固件 PID 的复刻。历史位置接管试验只有统计结果，而且完整的一次质量评估未通过；不能把配置中的 PID 数字直接当作有效加速度增益。
- 位置模型名义值为 `kp=4/s², kd=3/s, acceleration_limit=2m/s²`，在预声明范围中随机变化，加入反馈与接管延迟。这些范围是敏感性测试假设，不是实测置信区间。
- 加速与回水平的前置过程由同一个动力学模型实际积分；覆盖 +Y/-Y，保留已经发出的命令历史。没有的测量历史使用无效标记，不编造测量。
- 目标距离是独立给定的虚拟 coast 输入（训练为4–40cm），不是本模块用真实隐藏模型挑出的“容易成功的停止位置”。不替换现有 force→coast 算法。

## 评价标准

所有方法从同一个首个决策时刻观察满3秒，不因提前停稳或反向而结束。终端0.3秒内同时满足位置误差≤3cm、速度≤0.04m/s、倾角≤3°、角速度≤5°/s，才记为有限窗口的联合稳定。

同时报告全过程超调、回退、反向速度，以及**实际开始发送位置环命令前/后**发生的反向。请求接管不等于位置环已经开始生效；两个时刻分开记录。位置环命令本身仍经过姿态延迟，因此“接管后反向”不单独证明位置环造成了反向。

损失包含终端位置误差、终端速度、最大越过目标距离、最大回退及全程误差积分。较低损失不是飞行安全保证；联合稳定也不代表此前没有越过目标或回拉。

对照组包括：直接位置接管、固定240ms制动后接管、原预测器的接管适配、基于剩余距离的名义制动阈值、第一版 PPO 的接管适配。**适配组将原来的“回水平”动作映射为“接管位置”，不是第一版原生轨迹。** 所有组使用相同场景、固定目标与假设位置模型。

## 运行

从 offboard-controller 仓库根目录运行，复用已安装的独立 RL 环境：

```bash
MPLCONFIGDIR="$PWD/.codex_tmp/rl-mpl-cache" \
.codex_tmp/braking-rl-venv/bin/python -m Interaction.train_rl_coast \
  --frozen-report /path/to/frozen_validation/report.json \
  --output /path/to/new_run \
  --seed 41 --timesteps 32768 --n-envs 4 --threads 1 \
  --eval-count 64 --eval-seed 19001
```

输出必须是新目录。`--resume .../model_best.zip` 仅接受配套元数据、版本、环境、模型及源码均兼容的第二版检查点；不能拿第一版模型续训。`--timesteps` 是增加的步数，不是续训后的累计目标。

```bash
MPLCONFIGDIR="$PWD/.codex_tmp/rl-mpl-cache" \
.codex_tmp/braking-rl-venv/bin/python -m Interaction.evaluate_rl_coast \
  --frozen-report /path/to/frozen_validation/report.json \
  --policy ppo_lineage41=/path/to/new_run/model_best.zip \
  --output /path/to/new_evaluation \
  --population-seed 804271 --count 256
```

最终测试不用于选择模型或调整奖励。`--v1-policy` 加入第一版适配对照；`--regression-manifest` 可以从第一版 manifest 中取出以前的实测初态，并分配明确标注的8/16/28cm合成目标。那些目标不是原日志的真实 coast 终点。

训练保存权重、源码/输入/检查点哈希、精确的随机场景与 RNG 状态、开发集曲线及统计。评估保存成对指标、场景清单和抽样轨迹。此次三轮有界实验记录位于 `autoresearch/loop-260904-0904`；不自动 commit、push 或部署。
