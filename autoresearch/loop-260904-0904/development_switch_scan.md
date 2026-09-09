# Development-only hindsight handoff-time scan

**Diagnostic only: not a deployable policy, checkpoint-selection rule, or physical guarantee.**

First 16 development cases, seed 19001; handoff grid 0, 0.02, ..., 0.50 s; 416 unchanged-simulator episodes.

Per-case minimum grid loss averages 6.3947; immediate-position loss on these same cases averages 14.2554. This hindsight advantage is not a fair learned-policy improvement claim.

At least one timing jointly settles in 11/16 cases; at least one timing jointly settles without any reversal in 2/16 cases.

Selecting the minimum-loss timing specifically yields 11/16 jointly settled cases and 1/16 jointly settled without reversal. These counts need not equal the existence counts.

| Development case | Initial speed m/s | Target gap m | Best grid time s | Best grid loss | Any joint settled | Any joint + no reversal |
|---|---:|---:|---:|---:|---|---|
| development:19001:0 | 0.340 | 0.166 | 0.04 | 2.9809 | yes | no |
| development:19001:1 | 0.771 | 0.318 | 0.10 | 2.1761 | yes | no |
| development:19001:2 | 0.272 | 0.150 | 0.10 | 1.9566 | yes | no |
| development:19001:3 | 0.315 | 0.149 | 0.02 | 6.1666 | no | no |
| development:19001:4 | 0.429 | 0.300 | 0.06 | 1.8443 | no | no |
| development:19001:5 | 0.630 | 0.091 | 0.20 | 11.6955 | yes | no |
| development:19001:6 | 0.335 | 0.345 | 0.02 | 0.8584 | yes | yes |
| development:19001:7 | 0.168 | 0.359 | 0.00 | 1.8639 | yes | no |
| development:19001:8 | 0.826 | 0.338 | 0.08 | 2.7314 | no | no |
| development:19001:9 | 0.493 | 0.137 | 0.24 | 9.7027 | yes | no |
| development:19001:10 | 0.506 | 0.046 | 0.14 | 22.5310 | no | no |
| development:19001:11 | 0.255 | 0.330 | 0.04 | 0.6257 | yes | no |
| development:19001:12 | 0.227 | 0.110 | 0.06 | 2.6872 | yes | no |
| development:19001:13 | 0.314 | 0.206 | 0.08 | 1.4654 | yes | no |
| development:19001:14 | 0.343 | 0.296 | 0.00 | 0.4783 | yes | yes |
| development:19001:15 | 0.438 | 0.164 | 0.12 | 32.5511 | no | no |

## Interpretation limits

- Hindsight search inspects complete future trajectories for each frozen hidden plant. It is not a causal deployable policy or a fair learned-policy baseline.
- Only the first 16 DEVELOPMENT cases from seed 19001 were scanned. No held-out cases or checkpoints were inspected or selected.
- No successful timing on this 20 ms grid does not establish impossibility between grid points, outside the scanned action family, or on hardware.
- Joint settled means the complete final 0.3 s satisfies all position, speed, tilt, and rate tolerances. Non-reversal is an additional whole-trajectory condition.
- The position controller is an explicitly assumed PD surrogate; this diagnostic does not calibrate it or establish a physical stopping envelope.
- Do not use this scan to choose checkpoints or change the frozen experiment objective. It is a descriptive action-timing diagnostic only.

Full per-timing metrics, exact selected scenarios, configuration, and provenance hashes are in `development_switch_scan.json`.
