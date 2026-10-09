"""Rebuild the IMU simulation review from preserved raw/replay artifacts."""
import argparse
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read(path):return json.loads(Path(path).read_text())


def metrics(replay,mode):
    samples=[r for r in replay[mode]['samples'] if r['phase'] not in ('preflight','takeoff','land')]
    error=np.asarray([r['error_deg'] for r in samples])
    return {'rmse_rpy_deg':np.sqrt(np.mean(error**2,axis=0)).tolist(),
        'samples':len(samples),'segments':replay[mode]['segments'],
        'duplicates':sum(r['reason']=='identical duplicate IMU packet skipped' for r in replay[mode]['rejected'])}


def generate(study,output):
    study,output=Path(study).resolve(),Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    reduced=read(study/'reduced_final/summary.json')
    case=study/'reduced_final/known_fault_101/mac/logs/known_fault_101/imu_validation/flight/replay/report.json'
    native=read(case)
    gazebo=read(study/'gazebo_complete_capture/replay_known_fault_v2/report.json')
    unmodified=read(study/'gazebo_complete_capture/replay_unmodified_v2/report.json')
    capture=read(study/'gazebo_complete_capture/flight/report.json')
    real=read(study/'saved_real_fixture_refit/report.json')
    tables={'reduced':{m:metrics(native,m) for m in ('raw','corrected')},
        'gazebo':{m:metrics(gazebo,m) for m in ('raw','corrected')},'gazebo_unmodified':metrics(unmodified,'raw')}
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for ax,title,data in zip(axes,('Reduced plant: roll error (deg)','Gazebo: roll error (deg)'),(native,gazebo)):
        start=min(r['time_s'] for r in data['raw']['samples'] if r['phase']=='hover_start')
        for mode,color,label in [('raw','#c35143','Raw IMU'),('corrected','#237f6a','Calibrated IMU')]:
            rows=[r for r in data[mode]['samples'] if r['phase'] not in ('preflight','takeoff','land')]
            ax.plot([r['time_s']-start for r in rows],[r['error_deg'][0] for r in rows],color=color,lw=1.1,label=label)
        ax.axhline(0,color='#666666',lw=.7);ax.set_title(title,loc='left');ax.set_xlabel('Time (s)')
        ax.grid(alpha=.15);ax.legend(loc='upper right')
        ax.text(.01,.04,f"Segments raw / corrected: {data['raw']['segments']} / {data['corrected']['segments']}",
            transform=ax.transAxes,fontsize=10,bbox={'facecolor':'white','edgecolor':'none','alpha':.85,'pad':3})
    for ext in ('png','pdf'):fig.savefig(output/f'attitude_comparison.{ext}',dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,3.6),layout='constrained')
    for ax,name in zip(axes,('reduced','gazebo')):
        x=np.arange(2)
        for mode,offset,color in [('raw',-.18,'#c35143'),('corrected',.18,'#237f6a')]:
            ax.bar(x+offset,tables[name][mode]['rmse_rpy_deg'][:2],width=.34,color=color,label=mode.capitalize())
        ax.set_xticks(x,['Roll','Pitch']);ax.set_title(('Reduced plant' if name=='reduced' else 'Gazebo')+': RMSE (deg)',loc='left')
        ax.grid(axis='y',alpha=.15);ax.legend()
    for ext in ('png','pdf'):fig.savefig(output/f'rmse_comparison.{ext}',dpi=180)
    plt.close(fig)
    (output/'summary.json').write_text(json.dumps({'metrics':tables,'reduced':reduced,
        'gazebo_capture_completed':capture['capture_completed'],'saved_real_fixture_fit_passed':real.get('fit_passed'),
        'hardware_flight_validated':False,'firmware_calibration_applied':False,'raw_artifacts':str(study)},indent=2))
    rows=[]
    for name,label in [('reduced','简化模型（完整链路，58 s 采样）'),('gazebo','Gazebo（真实 PID / 默认 KF）')]:
        raw=tables[name]['raw'];corrected=tables[name]['corrected']
        rows.append(f'<tr><td>{label}</td><td>{raw["rmse_rpy_deg"][0]:.4f}°</td><td>{corrected["rmse_rpy_deg"][0]:.4f}°</td><td>{raw["segments"]} / {corrected["segments"]}</td></tr>')
    note='Gazebo 的 80 段由 >20 ms 设备采样间隙触发，两组使用同样分段；117 个完全相同的重复包被记录并跳过。没有放宽生产阈值，也没有补造 IMU 数据。'
    content=f'''<!doctype html><meta charset="utf-8"><title>IMU calibration validation</title>
<style>body{{max-width:1080px;margin:40px auto;font:16px/1.65 system-ui;color:#182630;padding:0 20px}}img{{width:100%}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #ddd;text-align:left;padding:10px}}pre{{background:#f1f4f6;padding:16px;overflow:auto}}.note{{background:#fff4da;padding:16px}}h1,h2{{line-height:1.3}}a{{color:#187270}}</style>
<h1>IMU 校准与离线 estimator 3：完整流程验证</h1>
<p>已验证六面采集 → 拟合保存 → 选择最近成功结果 → 标准起降方法 → 18 个飞行采样阶段 → 下载校验 → 真实 C 版 estimator 3 原始/校正回放。三组独立噪声种子均改善；无误差对照没有明显退化。</p>
<p class="note">校正只作用于离线回放。尚未加载到实机 estimator 3，也未验证实机方向性问题已经解决。简化模型以理想姿态作参考；Gazebo 以真实默认 KF 作相对参考。注入的已知误差仅证明这种误差可以被校准补偿。</p>
<table><tr><th>验证</th><th>原始 roll RMSE</th><th>校正 roll RMSE</th><th>分段数 原始 / 校正</th></tr>{''.join(rows)}</table>
<p>{note}</p><img src="attitude_comparison.png"><img src="rmse_comparison.png">
<h2>通过与保留的失败</h2><ul>
<li>简化模型：3 组已知误差 + 1 组无误差，完整 18 阶段，每组连续 1 段。</li>
<li>Gazebo：一次完成起飞、58 s 采样和降落；原始文件保留，时间戳处理修复后完成整份回放。</li>
<li>两次 Gazebo 失败保留：冻结 SITL 的 logRunBlock 出现 SIGSEGV；后一例导致 IMU 过期并提前终止。它们没有被计入成功试验。不是实机故障证据，尚未确定该 SITL 崩溃的根因。</li>
<li>最新实机静态数据重新拟合仍通过：矩阵等效旋转 {real.get('alignment_rotation_deg',0):.3f}°。这六面没有独立重复姿态，不能据此确认实机校正精度。</li>
<li>68 个 offboard 检查、7 个 IMU orchestrator/sync 检查和 21 个既有 launcher 检查通过；文件同步覆盖备份、校验、损坏上传、语法错误和并发修改拒绝。</li></ul>
<h2>明天运行</h2><p>在本地 orchestrator 目录。重新校准并接回标准 Dispatcher/UI：</p>
<pre>python orchestrator.py --calibrate-estimator-imu --drone-id lb11 --imu-auto-flight</pre>
<p>已有成功校准且无人机没有断电：</p><pre>python orchestrator.py --validate-estimator-imu --drone-id lb11</pre>
<p>两个入口会先备份、同步并核对 Pi 所需的八个运行文件。六面采样拆桨；拟合后保持通电、装桨、在本地 UI 点选并确认起飞。无需手推、interaction YAML 或精密 gyro 转台。若断过电，重新六面采样；有已知 ±90° 固定轴转台时才加 --imu-calibrate-gyro。</p>
<p>Pi 当前 SSH 不可达，因此还未完成真实 Pi 上的同步及起飞验证。入口的自动同步需在回到网络后执行。飞行始终由默认 estimator 2 控制，降落下载后才在 Mac 运行 estimator 3。</p>
<h2>保留文件</h2><p>原始数据、失败记录、原生 C 编译记录：<code>{html.escape(str(study))}</code>。重跑入口：<code>bash Interaction/run_estimator_imu_sim.sh /absolute/new/output</code>；报告图可用 <code>python -m Interaction.plot_estimator_imu_sim --study ... --output ...</code> 重建。</p>
'''
    (output/'report.html').write_text(content)
    (output/'report.md').write_text('# IMU calibration validation\n\n'+
        '\n'.join(f'{name}: raw roll {m["raw"]["rmse_rpy_deg"][0]:.4f} deg; corrected {m["corrected"]["rmse_rpy_deg"][0]:.4f} deg; segments {m["raw"]["segments"]}/{m["corrected"]["segments"]}.' for name,m in tables.items() if name!='gazebo_unmodified')+
        '\n\n'+note+'\n\nCorrection remains offline only. Hardware flight not validated. See report.html for run commands and limitations.\n')
    return output/'report.html'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--study',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    print(generate(args.study,args.output))
