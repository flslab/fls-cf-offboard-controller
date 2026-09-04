# 制动与定位链路诊断（离线模型，不接管飞行）

本组修改分为运行时记录和纯离线分析。没有修改 interaction 的制动控制律、接触检测、安全阈值或现有 calibration 文件。

## 新日志会多记录什么

主 controller 启用定位及日志后，自动注册 `mocap_timing`，无需增加启动参数。

| 字段 | 含义 |
|---|---|
| `wait_duration_s` | 本机等待 `waitForNextFrame()` 返回的耗时 |
| `local_loop_interval_s` | 连续两次等待返回的本机单调时钟间隔 |
| `processing_duration_s` | 等待返回后，跟踪处理和同步回调的总耗时 |
| `callback_duration_s` / `max_callback_duration_s` | 本轮同步回调的累计 / 最大耗时 |
| `callback_count` | 本轮跟踪成功并调用订阅者的次数；零不等于没有收到源帧 |
| `processing_excluding_callbacks_s` | 从总处理时间中扣除回调时间 |
| `between_processing_and_wait_s` | 前轮处理结束到本轮等待开始的间隔，包含诊断记录自身的开销 |
| `logger_queue` | 每秒一次的后台写盘队列计数快照，不执行 flush |

`frames[*].mocap_timing` 还记录 `wait_return_to_send_s`、`extpos_send_duration_s` 和 `extpos_send_interval_s`。这些只是本机调用阶段耗时，**不是相机到飞控的端到端延迟**，也不能证明飞控已使用该定位数据。

`frame_id` 保持原有的本地循环编号，`frame.time` 保持原有的 wait 返回后 host 时间。没有将它们改称相机序号或采集时间。实际 Pi 上的 motioncapture 1.0a4 Python 接口未暴露这两项源端信息。

`LiveLogger.write()` 仍同步调用自定义回调并序列化记录，但不写磁盘。单一后台线程写 JSON 数组；正常关闭时排空已接受记录。队列上限默认 20,000，溢出或后台写入错误明确报错并锁存，不能把不完整日志当成成功。

正常命令记录失败仍中止实验。仅降落清理路径使用独立的 best-effort 命令日志包装，避免已锁存的日志错误阻断原有降落命令；写盘关闭失败也会在资源清理、断开后重新报告。此处没有改变正常降落轨迹或放宽限制。

## 离线复现

在 offboard-controller 根目录运行，输出必须选择尚不存在的目录：

```bash
venv/bin/python -m Interaction.analyze_mocap_timing LOG.json OTHER_LOG.json \
  --output NEW_TIMING_REPORT_DIRECTORY

venv/bin/python -m Interaction.evaluate_braking_snapshots LOG.json \
  --frozen-report FROZEN_REPORT.json \
  --target-distance-m 0.16 \
  --output NEW_CANDIDATE_REPORT_DIRECTORY
```

第一个工具支持不完整 JSON 的完整前缀恢复；对同一组速度样本端点，分别按 host 和解包后的 24-bit CF 设备时间计算 ∫v−Δp。不拟合时间偏移，不外推缺失位置，不把缺失时间字段当作零延迟。

第二个工具只读取已经冻结的模型。`--target-distance-m` 必须显式指定，是从制动前实测位置向前的**假想目标**，不是本次开环试验实际发送的 coast 目标，不是从实测停止点反推。模型按当前角度、角速度、速度，以及已经发送但尚未生效的命令进行预测。

候选为现在水平，或再给 20/50/100 ms 的反向 20° 脉冲后水平；检查至少 0.8 s 尾部，默认不允许预测负速度及超过假想目标 2 cm。选中候选不等于已经停稳；`settled_stop_predicted` 单独记录。有限候选集合无解不等于物理上无解，尤其是高速时 100 ms 脉冲本来就可能不足。

不同快照来自原来已经执行的轨迹，**不能把它们串起来宣称新闭环已经通过验证**。选择器没有连接任何 commander，也不更新校准；部署这组工具不会自动启用候选控制器。

## 当前结果与下一步

2026-09-03 16:58:11 的最新 +Y 单次试验中：

- 同端点 ∫v−Δp：host 时间 +2.8582 cm，CF 时间 +2.8634 cm；换时钟并未消除误差。
- 全记录外部定位有 41 个大于 30 ms 的间隔，很多约 206–215 ms；原日志无法区分 SDK 等待、上游缓存或回调阻塞。
- 该次约 80 ms 空档之前，位置与速度的差异已经形成大半；不能把异步日志宣称为完整制动修复。
- 冻结模型在制动约 0.24 s 时预测，即便立刻水平，尾速仍约 −0.075 m/s。这提示需在速度到零前考虑残余姿态响应，但不是新控制器的实机保证。

下一份部署后日志应先核对上述分阶段耗时、零回调轮次和写盘队列，再决定是否需要修定位接收层。此时不应通过继续提高速度、延长脉冲、放宽安全限制或重拟合异常样本掩盖问题。冻结模型应保留，后续补测沿用已有的单方向单次条件作对照。
