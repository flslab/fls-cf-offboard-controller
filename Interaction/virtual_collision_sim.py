"""Simulation-only two-body collision emulation using the existing septic curve.

No radio, commander, mission or firmware interface is imported. The reference
is a collinear, frictionless point-mass collision spread over finite time. A
separate delayed first-order acceleration plant exposes tracking limitations;
it is not a quadrotor attitude simulation or hardware calibration. CrazySim
tracking can consume CollisionPlan.sample without changing the live stop path.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

import numpy as np

from Interaction.jerk_limited_braking import make_septic_brake_profile


@dataclass(frozen=True)
class CollisionConfig:
    mass1_kg: float = 0.17
    mass2_kg: float = 0.17
    velocity1_m_s: float = 0.6
    velocity2_m_s: float = -0.6
    restitution: float = 1.0
    acceleration_limit_m_s2: float = 3.0
    jerk_limit_m_s3: float = 20.0
    minimum_separation_m: float = 0.4
    drone_radius_m: float = 0.12
    delay1_s: float = 0.04
    delay2_s: float = 0.04
    response_tau1_s: float = 0.06
    response_tau2_s: float = 0.06
    trigger_delay_budget_s: float = 0.14
    separation_margin_m: float = 0.08
    initial_separation_m: float = 2.4
    dt_s: float = 0.002
    settle_s: float = 1.0

    def validate(self):
        if not all(math.isfinite(float(v)) for v in asdict(self).values()):
            raise ValueError("collision inputs must be finite")
        if min(self.mass1_kg, self.mass2_kg, self.acceleration_limit_m_s2,
               self.jerk_limit_m_s3, self.drone_radius_m) <= 0:
            raise ValueError("mass, limits and vehicle radius must be positive")
        if not 0 <= self.restitution <= 1:
            raise ValueError("restitution must be between zero and one")
        if self.velocity1_m_s <= self.velocity2_m_s:
            raise ValueError("vehicles must be approaching: v1 > v2")
        if self.minimum_separation_m < 2 * self.drone_radius_m:
            raise ValueError("minimum center separation must contain both vehicles")
        if min(self.delay1_s, self.delay2_s, self.response_tau1_s,
               self.response_tau2_s, self.trigger_delay_budget_s,
               self.separation_margin_m) < 0:
            raise ValueError("delays, time constants and reserve cannot be negative")
        if not 0 < self.dt_s <= 0.01 or self.settle_s < 0.5:
            raise ValueError("use dt in (0, .01] s and a settling tail >= .5 s")


@dataclass(frozen=True)
class CollisionPlan:
    config: CollisionConfig
    profile: object
    weight1: float
    weight2: float
    target_velocity1_m_s: float
    target_velocity2_m_s: float
    impulse_ns: float
    closest_reference_time_s: float
    reference_closure_m: float
    trigger_distance_m: float

    @property
    def duration_s(self):
        return self.profile.duration_s

    def sample(self, elapsed_s):
        """Reference about trigger-time positions ±trigger_distance / 2.

        Negative times extend the constant-velocity approach; times beyond the
        curve extend the outgoing cruise. No endpoint teleport or velocity snap.
        """
        t = float(elapsed_s)
        if not math.isfinite(t):
            raise ValueError("sample time must be finite")
        c = self.config
        if t < 0:
            return dict(x1_m=-self.trigger_distance_m / 2 + c.velocity1_m_s * t,
                        x2_m=self.trigger_distance_m / 2 + c.velocity2_m_s * t,
                        v1_m_s=c.velocity1_m_s, v2_m_s=c.velocity2_m_s,
                        a1_m_s2=0., a2_m_s2=0., j1_m_s3=0., j2_m_s3=0.)
        q = self.profile.sample(t)
        return dict(
            x1_m=-self.trigger_distance_m / 2 + self.target_velocity1_m_s * t
                 + self.weight1 * q.position_m,
            x2_m=self.trigger_distance_m / 2 + self.target_velocity2_m_s * t
                 - self.weight2 * q.position_m,
            v1_m_s=self.target_velocity1_m_s + self.weight1 * q.velocity_m_s,
            v2_m_s=self.target_velocity2_m_s - self.weight2 * q.velocity_m_s,
            a1_m_s2=self.weight1 * q.acceleration_m_s2,
            a2_m_s2=-self.weight2 * q.acceleration_m_s2,
            j1_m_s3=self.weight1 * q.jerk_m_s3,
            j2_m_s3=-self.weight2 * q.jerk_m_s3)


def plan_collision(config: CollisionConfig = CollisionConfig()):
    """Transform one existing stop profile into an equal/opposite impulse.

    The fictitious stop variable q = relative_velocity + e * incoming_relative
    goes from (1+e)*u to zero. Physical velocity crosses zero without resetting
    the S-curve. Mass weights make m1*a1 + m2*a2 identically zero.
    """
    config.validate()
    c = config
    total_mass = c.mass1_kg + c.mass2_kg
    w1, w2 = c.mass2_kg / total_mass, c.mass1_kg / total_mass
    incoming = c.velocity1_m_s - c.velocity2_m_s
    delta = (1 + c.restitution) * incoming
    impulse = delta / (1 / c.mass1_kg + 1 / c.mass2_kg)
    curve = make_septic_brake_profile(
        delta, 0., c.acceleration_limit_m_s2 / max(w1, w2),
        c.jerk_limit_m_s3 / max(w1, w2))
    lo, hi = 0., curve.duration_s
    for _ in range(64):
        mid = (lo + hi) / 2
        if curve.sample(mid).velocity_m_s > c.restitution * incoming:
            lo = mid
        else:
            hi = mid
    closest = (lo + hi) / 2
    closure = curve.sample(closest).position_m - c.restitution * incoming * closest
    # Conservative displacement-error bound for the nonnegative first-order
    # response kernels used below: |Delta v| * (pure delay + time constant).
    # This is deliberately larger than a pure u*delay latency-only reserve.
    # Unmodeled attitude, wind, estimation error, and 3D geometry are excluded.
    trigger = (c.minimum_separation_m + closure
               + delta * c.trigger_delay_budget_s + c.separation_margin_m)
    if c.initial_separation_m < trigger:
        raise ValueError(f"initial separation {c.initial_separation_m:.3f} m is "
                         f"below required trigger distance {trigger:.3f} m")
    return CollisionPlan(c, curve, w1, w2,
                         c.velocity1_m_s - impulse / c.mass1_kg,
                         c.velocity2_m_s + impulse / c.mass2_kg,
                         impulse, closest, closure, trigger)


def _plant_step(x, v, a, target, tau, dt):
    """Exact integration under a zero-order-held acceleration target."""
    if tau == 0:
        return x + v * dt + 0.5 * target * dt * dt, v + target * dt, target
    one_minus_decay = -math.expm1(-dt / tau)
    error = a - target
    return (x + v * dt + .5 * target * dt * dt
            + error * tau * (dt - tau * one_minus_decay),
            v + target * dt + error * tau * one_minus_decay,
            target + error * (1 - one_minus_decay))


def simulate_collision(config: CollisionConfig = CollisionConfig()):
    """Run a separate delayed acceleration plant; keep measured/reference apart."""
    c = config
    plan = plan_collision(c)
    u = c.velocity1_m_s - c.velocity2_m_s
    trigger_time = (c.initial_separation_m - plan.trigger_distance_m) / u
    tail = max(c.settle_s, max(c.delay1_s, c.delay2_s)
               + 12 * max(c.response_tau1_s, c.response_tau2_s))
    end = trigger_time + plan.duration_s + tail
    if not math.isfinite(end) or end / c.dt_s > 250_000:
        raise ValueError("simulation exceeds 250000 steps; increase dt or shorten approach")
    start = plan.sample(-trigger_time)
    states = [[start["x1_m"], c.velocity1_m_s, 0.],
              [start["x2_m"], c.velocity2_m_s, 0.]]
    # Include phase boundaries. Integrate the analytic reference acceleration
    # over each interval, then use its exact average in ZOH plant propagation.
    # Unlike midpoint sampling this preserves impulse even for a curve shorter
    # than dt. Only the known reference is integrated, never future plant states.
    times = np.unique(np.r_[np.arange(0, end, c.dt_s), trigger_time,
                            trigger_time + plan.duration_s, end])
    samples = []
    for i, t in enumerate(times):
        elapsed = float(t - trigger_time)
        if i:
            dt = float(t - times[i-1])
            for k, delay, tau in ((0, c.delay1_s, c.response_tau1_s),
                                  (1, c.delay2_s, c.response_tau2_s)):
                before = plan.sample(float(times[i-1] - trigger_time - delay))
                after = plan.sample(float(t - trigger_time - delay))
                target = (after[f"v{k+1}_m_s"] - before[f"v{k+1}_m_s"]) / dt
                states[k] = list(_plant_step(*states[k], target, tau, dt))
        ref = plan.sample(elapsed)
        samples.append(dict(time_s=float(t),
            x1_m=states[0][0], x2_m=states[1][0],
            v1_m_s=states[0][1], v2_m_s=states[1][1],
            a1_m_s2=states[0][2], a2_m_s2=states[1][2],
            ref_x1_m=ref['x1_m'], ref_x2_m=ref['x2_m'],
            ref_v1_m_s=ref['v1_m_s'], ref_v2_m_s=ref['v2_m_s'],
            phase='approach' if elapsed < 0 else
                  'transition' if elapsed < plan.duration_s else 'separation'))
    p0 = c.mass1_kg * c.velocity1_m_s + c.mass2_kg * c.velocity2_m_s
    momentum_errors = [c.mass1_kg*s['v1_m_s'] + c.mass2_kg*s['v2_m_s'] - p0
                       for s in samples]
    gaps = [s['x2_m'] - s['x1_m'] for s in samples]
    last = samples[-1]
    terminal_errors = [abs(last['v1_m_s'] - plan.target_velocity1_m_s),
                       abs(last['v2_m_s'] - plan.target_velocity2_m_s)]
    actual_a = max(abs(s[key]) for s in samples for key in ('a1_m_s2', 'a2_m_s2'))
    within_delay_budget = max(c.delay1_s+c.response_tau1_s,
                              c.delay2_s+c.response_tau2_s) <= c.trigger_delay_budget_s
    # For this bounded scalar plant |gap''| <= 2*a_limit. The maximum
    # deviation below a chord is at most |gap''|*dt^2/8. Guard the entire
    # integrated trajectory, rather than only its sampled positions.
    gap_lower_bound = float(min(gaps) - (2*c.acceleration_limit_m_s2)*max(np.diff(times))**2/8)
    separation_ok = gap_lower_bound >= c.minimum_separation_m
    terminal_tolerance = min(.02, .02*u)
    terminal_ok = max(terminal_errors) <= terminal_tolerance
    # A mismatched response can conserve endpoint momentum but not transient
    # momentum. Report and fail that check instead of masking it with symmetry.
    peak_momentum = max(map(abs, momentum_errors))
    momentum_tolerance = terminal_tolerance * min(c.mass1_kg, c.mass2_kg)
    checks = dict(separation=separation_ok, terminal_velocity=terminal_ok,
                  acceleration=actual_a <= c.acceleration_limit_m_s2 + 1e-6,
                  transient_momentum=peak_momentum <= momentum_tolerance,
                  response_within_budget=within_delay_budget)
    summary = dict(status='passed' if all(checks.values()) else 'failed', checks=checks,
        trigger_time_s=trigger_time, trigger_distance_m=plan.trigger_distance_m,
        minimum_separation_m=min(gaps),
        minimum_separation_is_sampled=True,
        continuous_separation_lower_bound_m=float(gap_lower_bound),
        minimum_surface_gap_m=min(gaps) - 2*c.drone_radius_m,
        closest_time_s=samples[int(np.argmin(gaps))]['time_s'],
        transition_duration_s=plan.duration_s,
        target_velocity1_m_s=plan.target_velocity1_m_s,
        target_velocity2_m_s=plan.target_velocity2_m_s,
        terminal_velocity1_m_s=last['v1_m_s'], terminal_velocity2_m_s=last['v2_m_s'],
        terminal_velocity_error_m_s=max(terminal_errors),
        terminal_velocity_tolerance_m_s=terminal_tolerance,
        measured_restitution=(last['v2_m_s']-last['v1_m_s'])/u,
        max_acceleration_m_s2=actual_a,
        max_momentum_error_kg_m_s=peak_momentum,
        momentum_tolerance_kg_m_s=momentum_tolerance,
        terminal_momentum_error_kg_m_s=momentum_errors[-1],
        impulse_ns=plan.impulse_ns,
        energy_before_j=.5*c.mass1_kg*c.velocity1_m_s**2+.5*c.mass2_kg*c.velocity2_m_s**2,
        energy_after_j=.5*c.mass1_kg*last['v1_m_s']**2+.5*c.mass2_kg*last['v2_m_s']**2,
        response_model='pure delay followed by first-order acceleration response',
        simulation_only=True, flight_validated=False, firmware_sitl=False)
    return dict(config=asdict(c), summary=summary, samples=samples)


def demo_scenarios():
    base = CollisionConfig()
    cases = [('等质量 · 弹性反弹', base),
             ('等质量 · 部分反弹 e=0.5', replace(base, restitution=.5)),
             ('A 质量为 B 的两倍', replace(base, mass1_kg=2*base.mass2_kg)),
             ('不同响应延迟 · 动量偏差', replace(base, delay2_s=.10, response_tau2_s=.10)),
             ('零恢复系数 · 停止靠近', replace(base, restitution=0.))]
    return [dict(name=name, **simulate_collision(config)) for name, config in cases]


def save_results(out, scenarios):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    data = dict(schema='virtual_collision_sim_v1', scenarios=scenarios)
    (out/'results.json').write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    (out/'summary.json').write_text(json.dumps([
        dict(name=s['name'], config=s['config'], **s['summary']) for s in scenarios],
        indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    # Retain full precision/full rate in results.json; inline playback is bounded.
    compact = dict(schema=data['schema'], scenarios=[])
    for s in scenarios:
        stride = max(1, math.ceil(len(s['samples']) / 450))
        sampled = s['samples'][::stride]
        if sampled[-1] != s['samples'][-1]:
            sampled.append(s['samples'][-1])
        compact['scenarios'].append({**s, 'samples':[
            {k: round(v, 6) if isinstance(v, float) else v for k, v in row.items()}
            for row in sampled]})
    (out/'replay.json').write_text(json.dumps(compact, ensure_ascii=False, separators=(',', ':')),
                                 encoding='utf-8')
    return compact


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/virtual_collision_sim'))
    parser.add_argument('--visualization', type=Path)
    args = parser.parse_args()
    scenarios = demo_scenarios()
    compact = save_results(args.output, scenarios)
    if args.visualization:
        template = Path(__file__).with_name('virtual_collision_viewer.html').read_text(encoding='utf-8')
        args.visualization.parent.mkdir(parents=True, exist_ok=True)
        args.visualization.write_text(template.replace('__SIMULATION_DATA__',
            json.dumps(compact, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')),
            encoding='utf-8')
    for s in scenarios:
        print(s['name'] + ': ' + json.dumps(s['summary'], ensure_ascii=False))


if __name__ == '__main__':
    main()
