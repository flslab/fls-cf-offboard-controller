"""Shadow-only world-frame swept-envelope certificate for braking plans.

The certificate never emits a command and never grants control authority.  It
checks the vehicle centre along the current state, the known transport and
inner-loop tails, and a candidate braking trajectory against a complete XYZ
axis-aligned workspace.  Vehicle radius, explicit reserve, position
uncertainty, and linearly accumulated velocity uncertainty erode every face.

Each consecutive state pair defines a cubic Hermite segment from its endpoint
positions and velocities.  Face margins are minimized analytically over every
segment, so an excursion between two otherwise-safe samples is not hidden by
endpoint-only checking.  Inputs and velocities are already in the world frame;
yaw is deliberately not an input to this certificate.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, Optional, Sequence


AXES = ("x", "y", "z")
FACE_NAMES = tuple(
    face for axis in AXES for face in (f"{axis}_min", f"{axis}_max")
)


@dataclass(frozen=True)
class WorldTrajectorySample:
    """One world-frame knot in a braking prediction.

    ``time_s`` is elapsed time from the measured current state.  The position
    and velocity vectors use world XYZ coordinates.  Validation intentionally
    lives in :func:`certify_braking_swept_envelope`, which can therefore fail
    closed with a diagnostic result instead of raising into a caller.
    """

    time_s: float
    position_m: Sequence[float]
    velocity_m_s: Sequence[float]


@dataclass(frozen=True)
class SweptEnvelopeCertificate:
    """Immutable, diagnostic-only result of a world-envelope check."""

    feasible: bool
    reason: str
    detail: Optional[str]
    face_min_margin_m: Mapping[str, Optional[float]]
    face_min_time_s: Mapping[str, Optional[float]]
    limiting_face: Optional[str]
    limiting_margin_m: Optional[float]
    violating_faces: tuple[str, ...]
    checked_knot_count: int
    checked_segment_count: int
    horizon_s: Optional[float]
    shadow_only: bool = True
    command_authorized: bool = False
    interpolation: str = "cubic_hermite_world_frame"


class _InvalidInput(ValueError):
    pass


def _invalid(detail: str) -> SweptEnvelopeCertificate:
    empty = MappingProxyType({face: None for face in FACE_NAMES})
    return SweptEnvelopeCertificate(
        feasible=False,
        reason="invalid_input",
        detail=detail,
        face_min_margin_m=empty,
        face_min_time_s=empty,
        limiting_face=None,
        limiting_margin_m=None,
        violating_faces=(),
        checked_knot_count=0,
        checked_segment_count=0,
        horizon_s=None,
    )


def _finite_scalar(value, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise _InvalidInput(f"{name} must be a finite real scalar")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise _InvalidInput(f"{name} must be a finite real scalar") from exc
    if not math.isfinite(number):
        raise _InvalidInput(f"{name} must be finite")
    if nonnegative and number < 0.0:
        raise _InvalidInput(f"{name} must be nonnegative")
    return number


def _finite_vector3(value, name: str, *, nonnegative: bool = False) -> tuple[float, ...]:
    if value is None or isinstance(value, (str, bytes, Mapping)):
        raise _InvalidInput(f"{name} must contain exactly three finite values")
    try:
        values = tuple(value)
    except TypeError as exc:
        raise _InvalidInput(f"{name} must contain exactly three finite values") from exc
    if len(values) != 3:
        raise _InvalidInput(f"{name} must contain exactly three finite values")
    return tuple(
        _finite_scalar(item, f"{name}[{index}]", nonnegative=nonnegative)
        for index, item in enumerate(values)
    )


def _validated_bounds(bounds_m) -> tuple[tuple[float, float], ...]:
    if not isinstance(bounds_m, Mapping):
        raise _InvalidInput("bounds_m must be a mapping with exactly x, y and z")
    if set(bounds_m) != set(AXES):
        raise _InvalidInput("bounds_m must contain exactly x, y and z")
    bounds = []
    for axis in AXES:
        raw = bounds_m[axis]
        if raw is None or isinstance(raw, (str, bytes, Mapping)):
            raise _InvalidInput(f"bounds_m[{axis}] must contain lower and upper")
        try:
            pair = tuple(raw)
        except TypeError as exc:
            raise _InvalidInput(
                f"bounds_m[{axis}] must contain lower and upper"
            ) from exc
        if len(pair) != 2:
            raise _InvalidInput(f"bounds_m[{axis}] must contain lower and upper")
        lower = _finite_scalar(pair[0], f"bounds_m[{axis}][0]")
        upper = _finite_scalar(pair[1], f"bounds_m[{axis}][1]")
        if lower >= upper:
            raise _InvalidInput(f"bounds_m[{axis}] must have lower < upper")
        bounds.append((lower, upper))
    return tuple(bounds)


def _validated_phase(samples, name: str, *, require_nonempty: bool):
    if samples is None or isinstance(samples, (str, bytes, Mapping)):
        raise _InvalidInput(f"{name} must be an explicit sample sequence")
    try:
        raw_samples = tuple(samples)
    except TypeError as exc:
        raise _InvalidInput(f"{name} must be an explicit sample sequence") from exc
    if require_nonempty and not raw_samples:
        raise _InvalidInput(f"{name} must contain at least one sample")

    result = []
    previous_time = None
    for index, sample in enumerate(raw_samples):
        if not isinstance(sample, WorldTrajectorySample):
            raise _InvalidInput(
                f"{name}[{index}] must be a WorldTrajectorySample"
            )
        time_s = _finite_scalar(sample.time_s, f"{name}[{index}].time_s")
        if time_s < 0.0:
            raise _InvalidInput(f"{name}[{index}].time_s must be nonnegative")
        if previous_time is not None and time_s < previous_time:
            raise _InvalidInput(f"{name} times must be nondecreasing")
        position = _finite_vector3(
            sample.position_m, f"{name}[{index}].position_m"
        )
        velocity = _finite_vector3(
            sample.velocity_m_s, f"{name}[{index}].velocity_m_s"
        )
        result.append((time_s, position, velocity, name))
        previous_time = time_s
    return tuple(result)


def _same_state(first, second, tolerance=1e-12) -> bool:
    return all(
        abs(left-right) <= tolerance
        for left, right in zip(first[1]+first[2], second[1]+second[2])
    )


def _join_knots(current_position, current_velocity, phases):
    knots = [(0.0, current_position, current_velocity, "current")]
    for phase in phases:
        for knot in phase:
            last = knots[-1]
            if knot[0] < last[0]:
                raise _InvalidInput(
                    f"{knot[3]} starts before the preceding trajectory phase ends"
                )
            if knot[0] == last[0]:
                if not _same_state(last, knot):
                    raise _InvalidInput(
                        f"trajectory state is discontinuous at time {knot[0]:.9g} s"
                    )
                continue
            knots.append(knot)
    return tuple(knots)


def _hermite_coefficients(p0: float, v0: float, p1: float, v1: float, dt: float):
    """Return coefficients ``a,b,c,d`` for ``p(s)``, ``s`` in [0, 1]."""

    return (
        2.0*p0-2.0*p1+dt*(v0+v1),
        -3.0*p0+3.0*p1-dt*(2.0*v0+v1),
        dt*v0,
        p0,
    )


def _stationary_points(a: float, b: float, c: float) -> tuple[float, ...]:
    """Roots in (0, 1) of the derivative of ``a*s^3+b*s^2+c*s+d``."""

    qa, qb, qc = 3.0*a, 2.0*b, c
    scale = max(abs(qa), abs(qb), abs(qc), 1.0)
    tolerance = 1e-14*scale
    roots = []
    if abs(qa) <= tolerance:
        if abs(qb) > tolerance:
            roots.append(-qc/qb)
    else:
        discriminant = qb*qb-4.0*qa*qc
        discriminant_tolerance = 1e-14*max(
            qb*qb, abs(4.0*qa*qc), 1.0
        )
        if discriminant >= -discriminant_tolerance:
            square_root = math.sqrt(max(discriminant, 0.0))
            roots.extend((
                (-qb-square_root)/(2.0*qa),
                (-qb+square_root)/(2.0*qa),
            ))
    return tuple(root for root in roots if 0.0 < root < 1.0)


def _polynomial_value(coefficients, s: float) -> float:
    a, b, c, d = coefficients
    return ((a*s+b)*s+c)*s+d


def certify_braking_swept_envelope(
        *,
        bounds_m,
        current_position_m,
        current_velocity_m_s,
        position_uncertainty_m,
        velocity_uncertainty_m_s,
        vehicle_radius_m,
        boundary_reserve_m,
        transport_tail_samples,
        inner_loop_tail_samples,
        candidate_samples) -> SweptEnvelopeCertificate:
    """Fail-closed shadow certificate for a complete world-frame brake path.

    All arguments are mandatory.  Tail sequences may be empty when the caller
    explicitly models no such tail, while ``candidate_samples`` must be
    nonempty.  Samples must be phase-ordered and use elapsed time from the
    current state.  Uncertainty erosion on axis ``i`` at time ``t`` is::

        vehicle_radius + boundary_reserve
        + position_uncertainty[i] + velocity_uncertainty[i] * t

    The returned certificate is diagnostic evidence only.  In particular,
    ``feasible`` never authorizes sending a command.
    """

    try:
        bounds = _validated_bounds(bounds_m)
        current_position = _finite_vector3(
            current_position_m, "current_position_m"
        )
        current_velocity = _finite_vector3(
            current_velocity_m_s, "current_velocity_m_s"
        )
        position_uncertainty = _finite_vector3(
            position_uncertainty_m, "position_uncertainty_m", nonnegative=True
        )
        velocity_uncertainty = _finite_vector3(
            velocity_uncertainty_m_s,
            "velocity_uncertainty_m_s",
            nonnegative=True,
        )
        vehicle_radius = _finite_scalar(
            vehicle_radius_m, "vehicle_radius_m", nonnegative=True
        )
        boundary_reserve = _finite_scalar(
            boundary_reserve_m, "boundary_reserve_m", nonnegative=True
        )
        transport = _validated_phase(
            transport_tail_samples, "transport_tail_samples",
            require_nonempty=False,
        )
        inner_loop = _validated_phase(
            inner_loop_tail_samples, "inner_loop_tail_samples",
            require_nonempty=False,
        )
        candidate = _validated_phase(
            candidate_samples, "candidate_samples", require_nonempty=True
        )
        knots = _join_knots(
            current_position, current_velocity,
            (transport, inner_loop, candidate),
        )

        face_minimum = {face: math.inf for face in FACE_NAMES}
        face_time = {face: None for face in FACE_NAMES}

        def consider(face, margin, time_s):
            if not math.isfinite(margin):
                raise _InvalidInput("trajectory margin calculation overflowed")
            if margin < face_minimum[face]:
                face_minimum[face] = margin
                face_time[face] = time_s

        if len(knots) == 1:
            segment_pairs = ((knots[0], knots[0]),)
        else:
            segment_pairs = zip(knots[:-1], knots[1:])

        for first, second in segment_pairs:
            t0, t1 = first[0], second[0]
            dt = t1-t0
            for axis_index, axis in enumerate(AXES):
                lower, upper = bounds[axis_index]
                base_erosion = (
                    vehicle_radius+boundary_reserve
                    + position_uncertainty[axis_index]
                )
                erosion_rate = velocity_uncertainty[axis_index]
                if dt == 0.0:
                    centre = first[1][axis_index]
                    erosion = base_erosion+erosion_rate*t0
                    consider(f"{axis}_min", centre-lower-erosion, t0)
                    consider(f"{axis}_max", upper-centre-erosion, t0)
                    continue

                a, b, c, d = _hermite_coefficients(
                    first[1][axis_index], first[2][axis_index],
                    second[1][axis_index], second[2][axis_index], dt,
                )
                lower_coefficients = (
                    a,
                    b,
                    c-erosion_rate*dt,
                    d-lower-base_erosion-erosion_rate*t0,
                )
                upper_coefficients = (
                    -a,
                    -b,
                    -c-erosion_rate*dt,
                    upper-d-base_erosion-erosion_rate*t0,
                )
                for face, coefficients in (
                        (f"{axis}_min", lower_coefficients),
                        (f"{axis}_max", upper_coefficients)):
                    samples = (0.0, 1.0)+_stationary_points(
                        coefficients[0], coefficients[1], coefficients[2]
                    )
                    for s in samples:
                        consider(
                            face,
                            _polynomial_value(coefficients, s),
                            t0+s*dt,
                        )

        if not all(math.isfinite(value) for value in face_minimum.values()):
            raise _InvalidInput("trajectory did not produce finite face margins")
    except (_InvalidInput, ArithmeticError) as exc:
        return _invalid(str(exc))

    limiting_face = min(FACE_NAMES, key=face_minimum.__getitem__)
    limiting_margin = face_minimum[limiting_face]
    violating = tuple(
        face for face in FACE_NAMES if face_minimum[face] < 0.0
    )
    feasible = not violating
    return SweptEnvelopeCertificate(
        feasible=feasible,
        reason="certified" if feasible else "boundary_violation",
        detail=(
            None if feasible else
            "negative swept-envelope margin on "+", ".join(violating)
        ),
        face_min_margin_m=MappingProxyType(dict(face_minimum)),
        face_min_time_s=MappingProxyType(dict(face_time)),
        limiting_face=limiting_face,
        limiting_margin_m=limiting_margin,
        violating_faces=violating,
        checked_knot_count=len(knots),
        checked_segment_count=max(len(knots)-1, 0),
        horizon_s=knots[-1][0],
    )
