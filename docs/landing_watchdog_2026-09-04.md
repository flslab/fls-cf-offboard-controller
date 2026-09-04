# 2026-09-04 校准结束停桨与降落交接事故分析

## 结论与纠正

运行 `translation_inertia_2026-09-04_10-40-57` 确实完成并保存了校准，但**没有正常安全降落**。最后一个低层位置指令之后，主线程同步拟合与保存造成了 **2.479 s 没有新低层 setpoint**；四个电机在约 2.045 s 后同时归零，无人机随后坠落。高层 `go_to` 和 `land` 都是在停桨之后才发送。

旧报告 `online_prediction_review/RESULTS_zh.md` 中“10:42:39.726 保存校准；随后正常降落”的判断错误，应撤回。模型保存成功仍然成立，但日志出现 `Landing`、`Disconnected` 或进程正常退出，**不能证明实物安全着陆**。本报告保留原始日志与旧报告，不覆盖历史证据。

该时序与固件的 2 s commander watchdog 高度一致，是强因果证据；不过此次日志没有直接记录 supervisor 状态、watchdog flag 或在飞固件完整版本，因此不能表述为“已经读到 watchdog 触发标志”。

## 证据文件与时钟

- 飞行记录：`/Users/shuqinzhu/Documents/FLS_Research/lightbender/orchestrator/logs/translation_inertia_2026-09-04_10-40-57/lb11_translation_inertia_2026-09-04_10-40-57.json`。
- 实际姿态指令发送时间：同目录 `online_prediction_review/report.samples.jsonl`。
- 模型结果：同目录 `online_prediction_review/report.json` 与 `wrench_calibration.json`。
- 日志标记的 controller revision：`fcaf9aeaa4796f3f8bcc02b16c0641703e56c1d1`。它不证明飞控固件版本，也不排除部署时的未提交代码。

日志中的交互低层指令时间是相对时间，而 controller 的高层指令与遥测时间是 Unix 时间。使用 30 个试验阶段的实际 `command_started_at` 对齐相应低层指令，得到偏移中位数 `1788543697.1608093 s`，各对应偏移的范围为 **15.26 μs**。下面时间均为 America/Los_Angeles 本地时间，间隔以最后低层位置指令为零点。

这反映主机记录/发送时刻，不是飞控收包确认。遥测采样与主机接收也存在延迟，不能用 2.045 s 与固件 2.000 s 的差值推导精确无线延迟。

## 最后控制与停桨时间线

| 时间 | 距最后 LL 位置指令 | 事件或观测 |
|---|---:|---|
| 10:42:35.398078 | −1.854 s | 新 online 后台拟合已完成 |
| 10:42:37.252258 | 0.000 s | 最后 `Commander.send_position_setpoint(0, 0, 1, 0)`；高度约 1.001 m |
| 10:42:39.297255 | +2.045 s | 首次 `motor.m1…m4` 全部为 0 |
| 10:42:39.371084 | +2.119 s | 机载 vz 已为 −0.522 m/s |
| 10:42:39.461997 | +2.210 s | 机载 z=0.895 m，az≈−0.996 g |
| 10:42:39.649 附近 | +2.397 s | 机载 z≈0.450 m，vz≈−3.259 m/s |
| 10:42:39.726544 | +2.474 s | `Wrench Model Calibration Saved` |
| 10:42:39.731387 | +2.479 s | 交互 finally 才补发 `send_zdistance_setpoint(0, 0, 0, 1)` |
| 10:42:39.731690 | +2.479 s | 交互 finally 调用 `send_notify_setpoint_stop()` |
| 10:42:39.732544 | +2.480 s | controller 再次调用 notify |
| 10:42:39.733532 | +2.481 s | HL `go_to` 返回初始 XY，传入当前高度已是 **0.238 m** |
| 10:42:40.557353 | +3.305 s | HL `land(0.1, 2.0)` |
| 10:42:43.558223 | +6.306 s | HL `stop()` |

首次全零后，剩余 **428 条 MOT_BAT 均为全零**，包括 `go_to` 和 `land` 全程。不能把这些命令的成功调用解读为飞控恢复了推力。

## 为什么判断为真实坠落

- Mocap 同样记录高度下降：最后 LL 指令附近为 1.0015 m，10:42:39.444 附近为 0.9090 m，10:42:39.654 为 0.4168 m。因此不是仅机载 z 估计发生跳变。
- 电机切断前记录电压约 7.398 V，切断后回升至 7.815 V。该段没有提供低压先触发停桨的证据；这也不能代替独立电源完整性测试。
- 新 online 后台 worker 在最后 LL 指令之前已完成。此处空档来自仍留在主线程的旧收尾拟合/保存路径，不是新 worker 当时持续占用控制循环。
- 无法从现有时间戳把这 2.479 s 精确分配到 XYZ 拟合、planar 拟合与文件保存三个函数。能够确认的是：它们处在停止发指令与 finally 重新发指令之间的同步路径上。

## 两个不同层面的错误

### 1. 本次已发生：同步后处理前没有持续控制所有者

事故路径在 `Interaction/interactions.py` 的 `interaction_onboard_wrench_admittance()` 中：控制循环退出后直接同步执行 `identify_xyz_alignment()`、`identify_planar_braking_response()` 与保存，然后外层 `_run_translation()` 的 finally 才尝试释放 LL。

LL `send_position_setpoint` 虽然是位置控制，也仍是**需要持续更新的低层指令**，不是高层自主保持命令。空中不能把“上一条位置目标还在”当作无限期 keepalive。

本地源码检查（不是在飞固件身份认证）：

- `/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware/src/modules/src/supervisor.c`：`COMMANDER_WDT_TIMEOUT_STABILIZE=M2T(500)`，`COMMANDER_WDT_TIMEOUT_SHUTDOWN=M2T(2000)`；根据 setpoint age 设置条件。
- 同目录 `supervisor_state_machine.c`：飞行时超时可以进入 `ExceptFreeFall`，随后锁定。此机制与后来收到高层命令仍维持零电机的观测一致，但没有直接记录状态来唯一确认该分支。

Bitcraze 的官方说明也明确指出：低层 setpoint 超时可导致电机关闭；高层 planner 在飞控内部持续生成 setpoint，且低层具有更高优先级。参考 [The Commander Framework](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/)。官网文字与不同版本源码的位置可能不同，本次数值取自上述本地源码。

### 2. 额外真实隐患：notify 不是建立安全的高层 hover

本地 `/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware/src/modules/src/commander.c` 中，新的 LL setpoint 会停止高层 planner；`commanderRelaxPriority()` 只同步当前 state 并降低优先级。

`crtp_commander_high_level.c` 的 `crtpCommanderHighLevelGetSetpoint()` 在 planner 处于 stopped 状态时会生成 `nullSetpoint`，平时依靠更高的 LL 优先级阻止其生效。因此在没有高层轨迹的情况下先 notify，再延迟建立 `go_to/land`，会暴露停桨 setpoint 窗口。**需要先准备有效高层轨迹，再释放低层优先级，而且之后不能再补发 LL 指令。**

本次日志里 notify 到 `go_to` 只有约 2 ms，而电机此前约 0.434 s 已经归零。因此不能把这个潜在窗口当成本次最初坠落的原因；它是必须同时修复的交接缺陷。

## 修复设计与验证要求

以下是针对此因果链的修复设计；是否通过代码测试，以主任务的实际测试输出为准。本报告不声明已经同步到 Pi、执行新飞行或完成硬件验收。

1. **在任何同步收尾拟合之前建立高层保持轨迹。** 使用已校验的 nominal hover 目标调用 HL `go_to`，等待对应机载成功回复后，才用 LL notify 释放优先级。两条指令在固件不同任务处理，所以仅调换 Python 调用顺序仍不够。由飞控高层 planner 持续控制，而不是让空中飞行等待 Python 拟合完成。
2. **交接后禁止 finally 再补发低层 level/position。** 新 LL 指令会停止刚建立的高层 planner，重新制造无控制更新空档。所有权和交接完成状态必须明确传递。
3. **`Controller.land()` 每次都完成明确交接。** 无论日志是否出错，先建立有效的 HL 返回/降落计划并收到机载回复，再 notify。回复超时（默认 150 ms）或拒绝时，不释放优先级；连接锁存失败，避免下一次误认延迟旧回复，改为 50 Hz 连续低层下降兜底。不能只在 logger error 的特殊分支释放 LL。
4. **不放宽 watchdog 或关闭停桨保护。** 修复控制命令所有权和执行时序，不掩盖无控制更新的问题。
5. **异常路径同样验证。** 拟合失败、日志失败、重复清理、延迟大于 2 s 的拟合，以及成功交接后不再发送 LL，都应有测试。fake commander 的调用顺序测试不能证明无线交付、实际固件行为或电机安全。

下一次硬件验证需要单独确认：新控制器代码已部署、实际飞控版本/命令交接行为匹配、handoff 前后目标高度连续、没有新的 LL 空档或停桨。不能仅以 `Landing/Disconnected`、模型文件保存或进程退出成功判定修复生效。

## 复用此检查的方法

对后续“结束时直接掉下来”的运行，至少重复以下步骤：

1. 对齐低层相对时间与遥测 Unix 时间；不要直接把两种时钟相减。
2. 找出最后连续 LL setpoint、后处理完成、HL 计划、notify、land、stop 的实际记录顺序。

## 同日 11-05-56 紧急降落补充

后一次运行未进入制动试验：激励阶段电池降到 6.98 V 后触发中止。它还暴露了另一条旧路径：`_prepare_for_emergency_landing()` 在不刷新 LL setpoint 的情况下等待约 0.6 s，然后在创建 HLC plan 前调用 notify。最后连续 LL 到该 notify 相隔约 0.61 s，超过本地固件的 0.5 s stabilize 门槛；随后四个电机约 0.13 s 全零，垂直速度由约 +0.05 m/s 降到约 -1.60 m/s，之后 HLC 才恢复推力。

紧急准备现已只设置请求标志，不等待、不触碰外围设备或 commander。它立即通过异常退出到统一的 `stop() -> land()`，由上述带 ACK 的 HLC 交接或连续 LL 下降兜底负责控制。若交接未确认，交互清理会先重发一条当前点 LL hold，刷新 watchdog，再关闭后台 worker；随后 landing 使用连续 LL 下降。该修改同样只经过软件测试，尚未通过实机复飞确认。
3. 找出首个四电机归零时刻，与 z、vz、加速度、电压及独立定位交叉对照。
4. 区分“停桨在 land 前”“land 已执行但轨迹异常”“定位异常”“着陆后预期停桨”。
5. 没有 supervisor 直接记录时，保留 watchdog 推断的不确定性；模型拟合成功和安全着陆分别给结论。
