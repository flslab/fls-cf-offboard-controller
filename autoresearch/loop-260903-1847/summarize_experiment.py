"""Freeze development-only selections, then summarize a separate evaluation.

Selection never reads final test metrics. Existing selections are immutable.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def select(rounds):
    target=ROOT/'selection.json'
    if target.exists():
        raise ValueError('selection already frozen; do not replace it after seeing test results')
    selected=[]
    candidates=[]
    for lineage in (11,22,33):
        per_lineage=[]
        for stage in range(1,rounds+1):
            directory=ROOT/f'round{stage}_seed{lineage}'
            stats=json.loads((directory/'statistics.json').read_text())
            meta=json.loads((directory/'model_best.metadata.json').read_text())
            checkpoint=directory/'model_best.zip'
            digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
            if digest != meta['checkpoint_sha256']:
                raise ValueError('checkpoint changed after its validation')
            per_lineage.append(dict(lineage=lineage,stage=stage,
                label=f'ppo_lineage{lineage}',path=str(checkpoint),sha256=digest,
                source_hashes=meta['source_hashes'],
                development_mean_loss=meta['development_evaluation']['mean_loss'],
                model_timesteps=meta['checkpoint_timesteps'],
                actual_round_training_steps=stats['actual_training_steps']))
        candidates.extend(per_lineage)
        selected.append(min(per_lineage,key=lambda r:r['development_mean_loss']))
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),selection_split='development_only',
                rounds=rounds,candidates=candidates,selected=selected,
                primary=min(selected,key=lambda r:r['development_mean_loss'])['label'],
                final_test_seen=False,flight_approved=False)
    target.write_text(json.dumps(result,indent=2)+'\n')
    for row in selected:
        print(f"--policy {row['label']}={row['path']}")
    print('Primary (chosen on development only):',result['primary'])


def report(evaluation):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    selection=json.loads((ROOT/'selection.json').read_text())
    result=json.loads((evaluation/'report.json').read_text())
    manifest=json.loads((evaluation/'manifest.json').read_text())
    preregistered=json.loads((ROOT/'final_population_preregistered/manifest.json').read_text())
    heldout=[case for case in manifest['cases'] if case['population']=='held_out_synthetic']
    if heldout!=preregistered['cases'] or manifest['source_sha256']!=preregistered['source_sha256']:
        raise ValueError('held-out scenarios or evaluation source differ from preregistration')
    if manifest['frozen_report_sha256']!=preregistered['frozen_report_sha256']:
        raise ValueError('frozen model input differs from preregistration')
    actual={r['label']:r['sha256'] for r in manifest['checkpoints']}
    for row in selection['selected']:
        if actual.get(row['label'])!=row['sha256']:
            raise ValueError('test checkpoint does not match frozen development selection')
        if any(manifest['source_sha256'].get(k)!=v for k,v in row['source_hashes'].items()):
            raise ValueError('test source differs from the code used to train the selected checkpoint')
    methods={'fixed_240ms','predictive_margin_080','predictive_margin_025',*actual}
    expected=Counter((case['name'],case['population'],method)
                     for case in manifest['cases'] for method in methods)
    observed=Counter((row['case'],row['population'],row['method']) for row in result['rows'])
    if any(n!=1 for n in expected.values()) or observed!=expected:
        raise ValueError('evaluation rows are missing, duplicated, or from unexpected populations/methods')
    primary=selection['primary']
    rows=[r for r in result['aggregate'] if r['population']=='held_out_synthetic']
    by_method={r['method']:r for r in rows}
    p=by_method[primary]
    baseline=by_method['predictive_margin_080']
    total_steps=sum(c['actual_round_training_steps'] for c in selection['candidates'])
    lines=['# 离线 PPO 制动训练结果','',
        '**训练和验证仅发生在电脑模拟器中；没有接入飞控，没有验证位置接管或实机停点。**','',
        f"完成 {selection['rounds']} 轮、3 个独立训练谱系，共 {total_steps:,} 个训练决策步。奖励、随机化范围与开发集保持不变，仅增加训练预算。",'',
        f"主候选为 `{primary}`，在查看最终测试之前，按开发集平均损失选定。下面同时公布所有谱系，避免只展示最好看的测试结果。",'',
        f"## {p['count']} 个未参与调参的合成初态与模型组合",'',
        '| 方法 | 平均损失↓ | 反向次数↓ | 近静止次数↑ | 低速水平次数↑ | 平均绝对尾速 m/s↓ | 最大回退 cm↓ |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['method']} | {r['mean_loss']:.4f} | {r['reversal_count']}/{r['count']} | "
            f"{r['near_stationary_count']}/{r['count']} | {r['capture_count']}/{r['count']} | "
            f"{r['mean_absolute_tail_velocity_m_s']:.4f} | {100*r['maximum_rollback_m']:.2f} |")
    lines += ['',f"主候选对比原预测器：反向 {baseline['reversal_count']} → {p['reversal_count']}；"
        f"近静止 {baseline['near_stationary_count']} → {p['near_stationary_count']}；"
        f"平均绝对尾速 {baseline['mean_absolute_tail_velocity_m_s']:.3f} → {p['mean_absolute_tail_velocity_m_s']:.3f} m/s。",'',
        '低损失的来源必须结合上表解释。反向减少而尾速增大，表示选择了更早解除制动的保守取舍，不能称为更准确地停稳。', '',
        '## 已看过的实测初态回归（之后轨迹仍为模拟）','',
        '| 数据组 | 方法 | N | 反向 | 近静止 | 平均绝对尾速 m/s | 最大回退 cm |',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in result['aggregate']:
        if r['population']=='held_out_synthetic':
            continue
        lines.append(f"| {r['population']} | {r['method']} | {r['count']} | {r['reversal_count']} | "
            f"{r['near_stationary_count']} | {r['mean_absolute_tail_velocity_m_s']:.4f} | {100*r['maximum_rollback_m']:.2f} |")
    lines += ['', '## 边界与下一步', '',
        '- 合成测试是同一分布的新随机样本，不是新的飞行数据，也不是已确认的实机误差边界。',
        '- 实测初态回归中原来就有较丰富的过去命令；训练初态的过去命令均为水平。这种分布差异需要明确处理，不能把回归表现解释成实机保证。',
        '- 动作只有继续20度反向制动或不可逆回水平；已经来不及止住残余姿态的状态，策略没有正向纠正动作。',
        '- 每种方法回水平后观察1.5秒；近静止要求从制动起全程未反向，且最后100ms速度≤0.04m/s、倾角≤3°、角速度≤5°/s。低速水平状态的速度上限为0.20m/s。两者都不代表长期位置保持。',
        '- 本阶段没有位置目标，因此不报告虚假的目标停点误差或声称 coast 终点控制已经完成。',
        '- 最终测试一旦被用于改进下一版，就应归入开发资料；下一版另留独立测试。', '',
        '下一轮优先修正离线任务，而不是直接延长训练：覆盖实测的命令历史与双向运动；把 coast 终点、剩余距离和位置接管纳入模型与观测；同时评估停点误差、残余速度和回退。当前模拟结果不支持直接接入飞控。', '',
        '## 文件', '',
        '- `selection.json`：查看最终测试前冻结的检查点选择、哈希与开发分数。',
        '- 各 `round*_seed*/model_best.zip`：仅供离线加载的模型。',
        '- `final_evaluation/report.json`：所有配对指标；`manifest.json` 保存输入、模型和源码哈希。',
        '- `environment-freeze.txt`：本机安装版本；`PLAN.md`：预先约定的实验范围与指标。',
        '- `loop-results.tsv`、`handoff.json`：有界迭代记录。', '',
        '- `verification.md`：测试结果、来源核对和实验边界。', '',
        '![开发集曲线](development_curves.png)', '',
        '![独立合成测试](heldout_comparison.png)', '']
    (ROOT/'RESULTS_zh.md').write_text('\n'.join(lines))

    fig,ax=plt.subplots(figsize=(9,4.5))
    for lineage,color in zip((11,22,33),('#067f93','#bc6650','#7561ad')):
        offset=0
        for stage in range(1,selection['rounds']+1):
            directory=ROOT/f'round{stage}_seed{lineage}'
            history=[json.loads(line) for line in (directory/'development_history.jsonl').read_text().splitlines()]
            ax.plot([offset+r['training_steps'] for r in history],[r['mean_loss'] for r in history],
                    marker='.',color=color,label=f'PPO lineage {lineage}' if stage==1 else None)
            offset+=json.loads((directory/'statistics.json').read_text())['actual_training_steps']
    dev=json.loads((ROOT/'development_baselines/report.json').read_text())
    ref=next(r for r in dev['aggregate'] if r['method']=='predictive_margin_080')['mean_loss']
    ax.axhline(ref,color='grey',ls='--',label='Predictive 0.08 (development)')
    ax.set(xlabel='Cumulative sampled training steps per lineage',ylabel='Development mean loss (log scale; lower better)',
           title='Development only: warm starts restore the prior best checkpoint')
    ax.set_yscale('log')
    ax.grid(alpha=.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(ROOT/'development_curves.png',dpi=160)
    plt.close(fig)

    order=['fixed_240ms','predictive_margin_080','predictive_margin_025','ppo_lineage11','ppo_lineage22','ppo_lineage33']
    labels=['Fixed 240','Predict .08','Predict .025','PPO 11','PPO 22','PPO 33']
    fig,axes=plt.subplots(1,3,figsize=(13,4.5))
    for ax,field,label in zip(axes,['reversal_count','near_stationary_count','mean_absolute_tail_velocity_m_s'],
                             ['Reversals (lower better)','Near stationary (higher better)','Mean absolute tail speed (m/s)']):
        ax.bar(np.arange(len(order)),[by_method[m][field] for m in order],color=['#888888']*3+['#067f93']*3)
        ax.set_xticks(np.arange(len(order)),labels,rotation=35,ha='right')
        ax.set_title(label)
        ax.grid(axis='y',alpha=.2)
    fig.suptitle(f"SIMULATION ONLY: {p['count']} held-out synthetic scenarios; no position controller")
    fig.tight_layout(rect=[0,0,1,.91])
    fig.savefig(ROOT/'heldout_comparison.png',dpi=160)
    plt.close(fig)
    print(ROOT/'RESULTS_zh.md')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--select-rounds',type=int)
    group.add_argument('--report',type=Path)
    args=parser.parse_args()
    if args.select_rounds is not None:
        select(args.select_rounds)
    else:
        report(args.report)
