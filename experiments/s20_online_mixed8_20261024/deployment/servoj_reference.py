"""Offline reference preparation for classic HR6.5.13 ServoJ; no SDK imports.

prepare_reference consumes accepted polynomial knots with their actual first
and second derivatives, not position-only collision samples. Each call covers
one phase. Join independently retimed phases only at rest, or supply a common
continuous time law upstream. Contact/release events remain caller-owned.
CPU dependencies: numpy, scipy. No Ruckig or Jetson installation is assumed.
"""
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly, PPoly


def load_profile(path=None):
    path = Path(path) if path else Path(__file__).with_name('servoj_speed_profile.json')
    return json.loads(path.read_text())


def _peaks(path, order):
    peaks = []
    for axis in range(6):
        curve = PPoly(path.c[:, :, axis], path.x, extrapolate=False).derivative(order)
        roots = curve.derivative().roots(extrapolate=False)
        roots = roots[np.isfinite(roots)]
        points = np.r_[path.x, roots[(roots >= path.x[0]) & (roots <= path.x[-1])]]
        peaks.append(float(np.max(np.abs(curve(points)))))
    return np.asarray(peaks)


def prepare_reference(times_s, joint_q, *, velocities_rad_s,
                      accelerations_rad_s2, phase, profile=None):
    """Return degree samples and limit-based times; never connect or execute.

    Arrays are N x 6 in radians and derivatives per second. Supplying original
    polynomial knots/derivatives exactly reproduces degree <=5 source pieces.
    Retiming may accelerate an old slow reference: denominator is live limits,
    not that reference's old speed. Original CAD and actual-start validation of
    the final reference still belongs to the main workflow.
    """
    config = load_profile() if profile is None else profile
    times = np.asarray(times_s, dtype=float)
    q, v, a = [np.asarray(value, dtype=float) for value in
               (joint_q, velocities_rad_s, accelerations_rad_s2)]
    if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)
            or any(value.shape != (len(times), 6) or not np.isfinite(value).all()
                   for value in (q, v, a))):
        raise ValueError('Need increasing finite times and N x 6 finite q/qd/qdd')
    baseline = config['baseline']
    if baseline['selected_usable_joint_max_velocity_deg_s'] is None or baseline['selected_usable_joint_max_acceleration_deg_s2'] is None:
        raise ValueError('Maximum-speed/acceleration baseline is unresolved')
    velocity_cap = (np.asarray(baseline['selected_usable_joint_max_velocity_deg_s'], dtype=float)
                    * config['phase_velocity_fractions'][phase])
    if 'executor_max_velocity_deg_s' in config:
        base=float(config['executor_max_velocity_deg_s'])
        shared_cap=base*min(config['phase_velocity_fractions'][phase],config['executor_limit_fraction'])
        velocity_cap=np.minimum(np.asarray(baseline['selected_usable_joint_max_velocity_deg_s'],dtype=float),shared_cap)
    acceleration_cap = np.asarray(baseline['selected_usable_joint_max_acceleration_deg_s2'], dtype=float)
    if 'reference_acceleration_cap_deg_s2' in config:
        acceleration_cap=np.minimum(acceleration_cap,np.asarray(config['reference_acceleration_cap_deg_s2'],dtype=float))
    if any(value.shape != (6,) or not np.isfinite(value).all() or np.any(value <= 0)
           for value in (velocity_cap, acceleration_cap)):
        raise ValueError('Need six positive velocity/acceleration limits')
    times = times-times[0]
    # Exact Hermite reconstruction, not a position-only smoothing spline.
    polynomial = PPoly.from_bernstein_basis(BPoly.from_derivatives(
        times, [[qi, vi, ai] for qi, vi, ai in zip(q, v, a)]))
    peak_v, peak_a = [np.degrees(_peaks(polynomial, order)) for order in (1, 2)]
    minimum_scale = max(float(np.max(peak_v/velocity_cap)),
                        float(np.sqrt(np.max(peak_a/acceleration_cap))))
    period = float(config['servo_period_s'])
    if period != .02 or config['lookahead_s'] != .2:
        raise ValueError('This deployment profile uses classic 0.02/0.2 second timing')
    steps = max(1, int(np.ceil(times[-1]*minimum_scale/period)))
    output_times = np.arange(steps+1)*period
    scale = float(output_times[-1]/times[-1])
    source_times = np.minimum(output_times/scale, times[-1])
    positions = np.degrees(polynomial(source_times))
    positions[0], positions[-1] = np.degrees(q[0]), np.degrees(q[-1])
    return dict(execution_enabled=False, phase=phase, servo_time_s=period,
        lookahead_time_s=.2, times_s=output_times.tolist(), positions_deg=positions.tolist(),
        duration_s=float(output_times[-1]), source_duration_s=float(times[-1]),
        velocities_deg_s=np.degrees(polynomial.derivative(1)(source_times)/scale).tolist(),
        accelerations_deg_s2=np.degrees(polynomial.derivative(2)(source_times)/scale**2).tolist(),
        time_scale=scale, velocity_cap_deg_s=velocity_cap.tolist(),
        acceleration_cap_deg_s2=acceleration_cap.tolist(),
        analytic_peak_velocity_deg_s=(peak_v/scale).tolist(),
        analytic_peak_acceleration_deg_s2=(peak_a/scale**2).tolist(),
        start_velocity_rad_s=(v[0]/scale).tolist(), end_velocity_rad_s=(v[-1]/scale).tolist(),
        baseline=baseline, sdk_sequence=['HRIF_StartServo', 'HRIF_PushServoJ', 'HRIF_GrpStop'],
        sdk_start_args=[0, 0, .02, .2], sdk_push_units='degrees',
        sdk_push_call='HRIF_PushServoJ(0, 0, positions_deg[i])',
        controller_override_written=False, controller_connected=False,
        source_curve_preserved=True, requires_actual_scene_revalidation=True,
        physical_speed_tested=False)
