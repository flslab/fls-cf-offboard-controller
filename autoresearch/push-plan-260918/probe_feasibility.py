"""Synthetic kernel feasibility only; deliberately not an acceptance metric."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Interaction.post_release_pi_planner import NativePlanner, SNAPSHOT_BODY

planner = NativePlanner()
try:
    for speed, pitch in ((1., 10.), (2.7, 0.), (2.7, -20.), (2.7, 20.), (.5, 20.)):
        x = [0.] * 26
        x[0], x[3], x[8] = speed, pitch, 1.
        x[14:17], x[17:20], x[20:23] = [6., 7.1, 6.], [1., 1., 1.], [.05, .07, .08]
        x[24] = 22.18
        record = {'scope': 'synthetic single-snapshot kernel, not flight',
                  'speed_mps': speed, 'actual_pitch_cf_deg': pitch,
                  'reference_pitch_cf_deg': x[24], 'rate_deg_s': 0}
        try:
            plan = planner.solve(SNAPSHOT_BODY.pack(1000000, 1400000, 2000, 1000, *x))
            record.update(feasible=True, duration_s=plan['duration_s'])
        except ValueError as exc:
            record.update(feasible=False, error=str(exc))
        print(json.dumps(record), flush=True)
finally:
    planner.close()
