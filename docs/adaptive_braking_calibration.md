# 自适应姿态制动校准与降落修复

本次实现让模型实际决定校准中的制动结束时刻，不只是打分。普通交互不自动启用实验模型，也没有运行新飞行或自动同步 Pi。

## 启动方式

确认先同步本次 **offboard-controller 和 lightbender orchestrator 两处代码**，并单独验证新的低层/高层交接与降落，再进行主动制动试验。以下命令从 `lightbender/orchestrator` 目录、原来的 Python 环境中运行；本次没有替用户执行：

```bash
python orchestrator.py --calibrate --adaptive-braking-calibration
```

不加新开关的 `--calibrate` 保持固定脉冲；新开关不能与 `--interaction` 或 `--braking-test` 混用。不需要 `--sense`。现有 manifest、位置边界、20 度幅值及最大脉冲时间保持不变，不增加更强或更长的试验。

对应 YAML 为 `Interaction.config.wrench_interaction.adaptive_braking_calibration`，默认关闭。启动开关只复制并修改当次 mission，不写回共享 YAML：

```yaml
adaptive_braking_calibration:
  enabled: false
  target_distance_m: 0.30
  model_based_braking:
    max_compute_s: 0.008
```

## 实际控制流程

1. 首个 +Y/-Y 对按原固定加速—水平—制动—水平流程飞行，后台拟合延迟、二阶姿态响应与运动增益。
2. 每个后续对开始时冻结当时已完成的更早试验模型。模型未就绪则整对保持原协议，并记录原因；worker 启动失败则拒绝主动模式。
3. 每次制动开始，固定一个位于当前点沿试验方向前方 `target_distance_m` 的实验预测目标。默认 **30 cm**，不是之前的虚拟 coast 长度，也不是新的 POSITION 指令。它用于比较预测停止点，不能保证在该处停车。
4. 每周期用当前位置、速度、姿态、角速度、状态时间和真正已发送的指令历史，预测“继续短暂反向倾斜再水平”与“现在水平”。延迟队列中尚未生效的旧指令会保留。
5. 在已观察到至少两个实际制动采样后，可以提前结束制动。一旦水平就锁定，不重新施加反向制动、不延长原定制动截止时刻。预测失败或超出预算也提前水平并记录原因；不悄悄沿用旧强制动。
6. 水平观察与原 recovery POSITION 回 nominal 流程保持不变。下一对用新拟合参数重复，参数不在单个试验中途替换。

这不是 RL，也不是已验证可直接部署到用户 release/coast 的控制策略。这里只支持所辨识的世界坐标 ±Y；更高速度的校准状态可能超出前一对训练范围，会明确标记为实验性外推。没有放宽原飞行边界/状态同步保护。合成动力学测试证明提前水平可减少过制动，但仍可能有前向残速、目标误差与恢复阶段回拉。

## 看哪些结果

- `Adaptive Braking Pair Frozen`：本对是否实际自适应、使用哪个版本、训练来自哪些更早试验。
- `Adaptive Braking Episode Started`：固定目标与原制动截止时间。
- `Adaptive Braking Decision`：实际继续制动或水平、原因、预测反向速度/超调/末速度、计算耗时。
- `Planar Braking Calibration Phase` 与原 raw samples：实际发送后的 phase 和 `command_started_at`，不会把提前水平仍写成制动。
- 在线报告的 `calibration_control_enabled` 表示本次主动校准配置；`runtime_enabled: false` 仍表示没有批准普通交互部署。是否某一对真正用到模型，以 Pair Frozen/Decision 为准。
- 保存参数仍走已有质量检查，失败不覆盖旧校准；协议新增自适应标识，时间数组代表计划上限，不应解读为实际等时反向脉冲。

## 降落修复是独立问题

详见 [2026-09-04 事故报告](landing_watchdog_2026-09-04.md)。该次在 HL `land()` 之前已停桨，不是正常下降。现在在同步收尾拟合前先执行：

```text
最后的低层保持 → 高层 go_to → 匹配机载成功回复 → notify
               → 飞控自主保持，同时 Python 拟合/保存 → 高层降落
```

交接成功后 finally 不再补低层 level 指令。高层回复失败时走持续发送的低层下降；不调整固件 watchdog。无线丢包、真实固件版本与实际下降效果仍需硬件验证；软件测试不能证明实机安全着陆。
