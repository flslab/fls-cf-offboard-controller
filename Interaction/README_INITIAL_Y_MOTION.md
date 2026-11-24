# World +Y motion before the first interaction

`level_coast.initial_motion` optionally replaces only the first `ready` hold
with a bounded moving position reference. It begins after the ordinary startup
bias calibration, initial arming, yaw preparation, and optional hover calibration
have completed. Existing mission X, height, and yaw policies are retained.

```yaml
Interaction:
  config:
    level_coast:
      initial_motion:
        enabled: true
        speed_m_s: 0.03
        ramp_s: 1.0
        max_distance_m: 0.60
        boundary_margin_m: 0.20
```

The frame is world XY and the direction is fixed at +Y, independently of yaw.
The reference uses quintic velocity ramps at the beginning and end, with a
constant-speed middle segment. If the available distance is too short, peak
speed is reduced so both ramps fit. Maximum reference speed is 0.10 m/s;
for KPS detection it must also stay below the configured onset threshold.

The endpoint is limited to the earlier of the maximum travel and
`boundary_limits.y_max - boundary_margin_m`. Reaching it without contact ramps
the reference to rest and leaves the normal position hold there. These are
reference limits; existing measured-position boundary and freshness guards
remain active for actual flight.

The first accepted contact permanently retires this motion for that mission,
including contact during the starting ramp. Existing interaction, release,
capture, direct-hold, and re-arming policies then take over. Returning to ready
does not restart the motion. A configured detection delay holds the last
moving reference during the existing delay; no return to the original start
point is introduced. Omitted configuration defaults to disabled.

The detector thresholds and force model are unchanged. These trials have a
moving initial condition and should be recorded separately from hover trials.
Commanded motion can differ from physical tracking; this implementation has
local offline coverage and has not been flight validated.

Events `Level Coast Initial Motion Started/Finished` and per-sample
`initial_motion_*` fields disclose the world direction, reference speed,
commanded displacement, effective distance limit, and retirement reason.

Checks:

```sh
venv/bin/python -m unittest Interaction.tests.test_initial_y_motion Interaction.tests.test_level_coast
```
