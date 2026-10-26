"""Jetson-local periodic tracking of a certified continuous joint reference.

This adapter preserves the existing qualification, vacuum, fault and stop
checks. It does not infer controller timing from an offline visualization.
"""
import math
import time

import numpy as np

from continuous_executor import check_feedback, sample, validate_configuration
from manual_hardware import Hardware, controller_payload_audit
from native_execution_reference import single_point_plan, validated_blocks


SAMPLE_SERVO_PERIOD_S = .02
SAMPLE_SERVO_LOOKAHEAD_S = .2
SERVO_SDK_SEQUENCE = ('HRIF_StartServo', 'HRIF_PushServoJ', 'HRIF_GrpStop')


def start_servo_session(hw, period_s=SAMPLE_SERVO_PERIOD_S,
                        lookahead_s=SAMPLE_SERVO_LOOKAHEAD_S, startup_mode='official_servoj'):
    """Start the classic SDK suite; ServoEsJ is a different protocol."""
    if startup_mode != 'official_servoj':
        raise ValueError('Classic PushServoJ requires StartServo only; do not mix ServoEsJ')
    # HR6.5.13 uses cycle seconds, not the >= HR6.5.19 stiffness-gain variant.
    if (not math.isfinite(period_s) or not 0. < period_s < 1.
            or not math.isfinite(lookahead_s) or lookahead_s <= 0.):
        raise ValueError('Classic Servo timing must be seconds; a stiffness gain is not a period')
    for method in SERVO_SDK_SEQUENCE:
        if not callable(getattr(hw.client, method, None)):
            raise RuntimeError('SDK lacks '+method)
    code = hw.client.HRIF_StartServo(0, 0, period_s, lookahead_s)
    hw.event('continuous_servo_start', return_code=code, period_s=period_s,
             lookahead_s=lookahead_s, parameter_semantics='cycle_seconds',
             sdk_sequence=list(SERVO_SDK_SEQUENCE))
    if code != 0:
        raise RuntimeError('StartServo rejected: %s' % code)


def push_servo_sample(hw, q_rad):
    """Convert one six-joint position sample once at the Python SDK boundary."""
    q = np.asarray(q_rad, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('PushServoJ needs exactly six finite joint positions in radians')
    return hw.client.HRIF_PushServoJ(0, 0, np.degrees(q).tolist())


class SinglePointHardware(Hardware):
    def execute(self, plan, *, mode='point', **kwargs):
        if mode != 'point' or kwargs.get('native_blend_contract') is not None:
            raise ValueError('Single MoveJ entry cannot select another execution backend')
        dense = single_point_plan(plan)
        report = super().execute(dense, mode='point', **kwargs)
        report.update(execution_version='movej_single', source_plan_command_count=len(plan['segments']),
                      multi_point_compaction_enabled=False, native_blending_enabled=False)
        return report


class ServoHardware(Hardware):
    def __init__(self, log, client=None, *, reference=None, configuration=None, tracking_proof=None):
        super().__init__(log, client)
        self.reference, self.configuration = reference, configuration
        self.tracking_proof = tracking_proof

    def read(self, name, *args):
        began = time.monotonic()
        result = super().read(name, *args)
        stamp = (began+time.monotonic())/2.
        if name == 'HRIF_ReadActJointPos':
            self._position_read_stamp = stamp
        elif name == 'HRIF_ReadActJointVel':
            self._velocity_read_stamp = stamp
        return result

    def execute(self, plan, *, mode='point', speed_scale=3., velocity_cap_deg_s=30., **kwargs):
        if mode != 'point' or kwargs.get('waypoint') is not None:
            raise ValueError('Dedicated ServoJ entry cannot combine motion backends')
        dense, blocks = validated_blocks(plan, self.reference)
        settings = validate_configuration(self.configuration, self.reference['robot_model_sha256'])
        wall_clock = self.configuration.get('timing_policy') == 'wall_clock_no_replay_tracking_guarded'
        maximum_gap = float(self.configuration.get('maximum_update_gap_s', settings['period_s']))
        if wall_clock:
            conservative_lag = float(self.configuration['conservative_tracking_lookahead_bound_s'])
            if (not math.isfinite(maximum_gap) or maximum_gap < settings['period_s']
                    or settings['lookahead_s'] + maximum_gap > conservative_lag + settings['period_s']):
                raise ValueError('Update gap exceeds the collision-checked tracking lag envelope')
        stage_trial = self.configuration.get('qualification_scope') == 'this_stage_supervised_trial_not_full_cycle'
        local_tracking_guard = (stage_trial and self.configuration.get('tracking_error_policy')
                                == 'stage_verified_local_joint_tube')
        if stage_trial:
            from native_servo_stage import validate_trial
            validate_trial(plan, self.reference, self.configuration, self.tracking_proof)
            version = self.read('HRIF_ReadVersion', 0)
            if version != self.configuration['qualification_evidence']['startup']['controller_version']:
                raise RuntimeError('ServoJ commissioning firmware changed')
        startup_mode = self.configuration.get('startup_mode')
        if startup_mode != 'official_servoj':
            raise ValueError('Qualification must name the physically observed ServoJ startup mode')
        feedback_period = float(self.configuration.get('feedback_period_s', settings['period_s']))
        if not math.isfinite(feedback_period) or not settings['period_s'] <= feedback_period <= .1:
            raise ValueError('Qualified feedback period must be between send period and 0.1 s')
        feedback_budget = float(self.configuration.get('feedback_group_budget_s', settings['rpc_budget_s']))
        if not math.isfinite(feedback_budget) or not 0. < feedback_budget < settings['period_s']:
            raise ValueError('Feedback group budget must be shorter than the send period')
        for method in SERVO_SDK_SEQUENCE:
            if not callable(getattr(self.client, method, None)):
                raise RuntimeError('SDK lacks '+method)
        start = self.snapshot()
        self.ready(start)
        if np.max(np.abs(np.asarray(start['q'])-plan['start_q'])) > math.radians(.05):
            raise RuntimeError('ROBOT_START_CHANGED: regenerate the reference')
        loaded = bool(plan.get('loaded'))
        if start['io']['DO1'] or (loaded and (start['io']['DI0'] != 1 or start['io']['DO0'] != 1)):
            raise RuntimeError('Invalid vacuum/release state before continuous motion')
        if not loaded and (start['io']['DI0'] or start['io']['DO0']):
            raise RuntimeError('Empty continuous motion requires vacuum OFF')
        if loaded:
            self.event('controller_payload_before_loaded_stage',
                       **controller_payload_audit(self.read('HRIF_ReadPayload')))
            self.event('vacuum_before_loaded_motion', confirmed=True,
                       io=start['io'], confirmation_scope='current_pressure_and_output',
                       repeated_stable_hold_s=0., initial_pump_hold_unchanged=True)
        report = {'completed': False, 'execution_mode': 'servoj', 'continuous_execution': True,
                  'reference_digest': self.reference['reference_digest'], 'loaded': loaded,
                  'completed_segments': 0, 'completed_blocks': 0, 'feedback': [],
                  'reference_duration_s': self.reference['reference_duration_s'],
                  'motion_sent': False, 'actuator_outputs_written': False,
                  'maximum_deadline_lateness_s': 0., 'maximum_send_rpc_s': 0.,
                  'maximum_feedback_rpc_s': 0., 'maximum_tracking_error_rad': 0.,
                  'timing_policy': self.configuration.get('timing_policy', 'legacy_strict_deadlines'),
                  'single_rpc_budgets_are_diagnostics_only': wall_clock,
                  'maximum_update_gap_s': maximum_gap, 'maximum_observed_update_gap_s': 0.,
                  'skipped_send_slots': 0,
                  'period_s': settings['period_s'], 'feedback_period_s': feedback_period,
                  'feedback_group_budget_s': feedback_budget,
                  'no_per_waypoint_stop': True, 'controller_parameters_written': False,
                  'servo_startup_mode': startup_mode,
                  'sdk_sequence': list(SERVO_SDK_SEQUENCE),
                  'sdk_joint_unit': 'degrees', 'reference_joint_unit': 'radians',
                  'timing_owner': 'accepted_continuous_reference',
                  'controller_native_path_interpolation': False,
                  'minimum_goal_hold_s': max(settings['lookahead_s'], 3*settings['period_s'])}
        report.update(stage_scoped_supervised_trial=stage_trial,
                      full_cycle_physically_verified=False,
                      fresh_tracking_tube_checked=stage_trial,
                      qualification_scope=self.configuration.get('qualification_scope'))
        began = time.monotonic()
        attempted = False
        try:
            attempted = True
            start_servo_session(self, settings['period_s'], settings['lookahead_s'], startup_mode)
            for block_index, (block, path) in enumerate(blocks):
                feedback_profile = block.get('feedback_profile', block['profile'])
                from execution_speed_policy import DEFAULT_FREE_SPACE_VELOCITY_DEG_S, phase_speed_profile
                requested_cap = feedback_profile.get('explicit_free_space_velocity_cap_deg_s')
                if requested_cap is None:
                    requested_cap = DEFAULT_FREE_SPACE_VELOCITY_DEG_S
                ceiling = phase_speed_profile(block['phase'], loaded, 3., requested_cap)
                for key in ('velocity_cap_deg_s', 'acceleration_cap_deg_s2'):
                    if (not math.isfinite(feedback_profile[key])
                            or not 0. < feedback_profile[key] <= ceiling[key]):
                        raise ValueError('ServoJ measured-motion monitor exceeds the existing phase policy')
                epoch = time.monotonic()
                previous_velocity, previous_stamp = None, None
                tick, settled = 0, 0
                next_feedback = epoch
                previous_send = None
                while True:
                    deadline = epoch+tick*settings['period_s']
                    time.sleep(max(0., deadline-time.monotonic()))
                    late = time.monotonic()-deadline
                    report['maximum_deadline_lateness_s'] = max(report['maximum_deadline_lateness_s'], late)
                    if not wall_clock and late > settings['deadline_tolerance_s']:
                        raise RuntimeError('STREAM_DEADLINE_MISSED: no catch-up or replay')
                    current = time.monotonic()
                    stamp = min(current-epoch if wall_clock else tick*settings['period_s'],
                                float(path.times[-1]))
                    _, q, qd, qdd = sample(path, stamp)
                    sent = time.monotonic()
                    if previous_send is not None:
                        gap = sent-previous_send
                        report['maximum_observed_update_gap_s'] = max(report['maximum_observed_update_gap_s'], gap)
                        if wall_clock and gap > maximum_gap:
                            raise RuntimeError('STREAM_UPDATE_GAP_EXCEEDS_TRACKING_ENVELOPE')
                    previous_send = sent
                    report['motion_sent'] = True
                    code = push_servo_sample(self, q)
                    send_s = time.monotonic()-sent
                    report['maximum_send_rpc_s'] = max(report['maximum_send_rpc_s'], send_s)
                    if code != 0:
                        raise RuntimeError('PushServoJ rejected: %s' % code)
                    if not wall_clock and send_s > settings['rpc_budget_s']:
                        raise RuntimeError('STREAM_RPC_TOO_SLOW: reference is not slowed silently')
                    if time.monotonic() >= next_feedback or stamp >= path.times[-1]:
                        read_began = time.monotonic()
                        actual, velocity, state, fsm, io = check_feedback(self, settings, loaded)
                        now = time.monotonic()
                        read_s = now-read_began
                        report['maximum_feedback_rpc_s'] = max(report['maximum_feedback_rpc_s'], read_s)
                        if not wall_clock and read_s > feedback_budget:
                            raise RuntimeError('STREAM_FEEDBACK_TOO_SLOW')
                        next_feedback = now+feedback_period
                        measured_at = self._position_read_stamp
                        velocity_at = self._velocity_read_stamp
                        measured_t = min(max(0., measured_at-epoch), path.times[-1])
                        actual_piece, q_now, _, _ = sample(path, measured_t)
                        if stage_trial:
                            tube = self.tracking_proof['blocks'][block_index]
                            interval = next((row for row in tube['intervals']
                                             if row['start_s']-1.e-8 <= measured_t <= row['end_s']+1.e-8), None)
                            if interval is None:
                                raise RuntimeError('ServoJ measurement time has no fresh tracking proof')
                            lower, upper = interval['lower_rad'], interval['upper_rad']
                        else:
                            source = dense['segments'][block['source_interval_indices'][actual_piece]]
                            lower, upper = source['joint_envelope_lower'], source['joint_envelope_upper']
                        if (np.any(actual < np.asarray(lower)) or np.any(actual > np.asarray(upper))):
                            raise RuntimeError('Actual joints left the certified interval')
                        error = float(np.max(np.abs(actual-q_now)))
                        report['maximum_tracking_error_rad'] = max(report['maximum_tracking_error_rad'], error)
                        if not local_tracking_guard and error > math.radians(settings['max_tracking_error_deg']):
                            self.event('tracking_guard_rejection', phase=block['phase'],
                                q=actual.tolist(), q_reference=q_now.tolist(),
                                global_guard_deg=settings['max_tracking_error_deg'],
                                local_guard_rad=interval['monitored_error_rad'] if stage_trial else None)
                            raise RuntimeError('CONTINUOUS_TRACKING_ERROR')
                        if stage_trial and np.any(np.abs(actual-q_now) > np.asarray(interval['monitored_error_rad'])):
                            self.event('tracking_guard_rejection', phase=block['phase'],
                                q=actual.tolist(), q_reference=q_now.tolist(),
                                local_guard_rad=interval['monitored_error_rad'])
                            raise RuntimeError('CONTINUOUS_JOINT_TRACKING_TUBE_ERROR')
                        if np.max(np.abs(velocity)) > math.radians(feedback_profile['velocity_cap_deg_s']):
                            self.event('measured_velocity_violation', q=actual.tolist(), qd=velocity.tolist(),
                                phase=block['phase'], reference_profile=block['profile'],
                                feedback_profile=feedback_profile)
                            raise RuntimeError('ACTUAL_VELOCITY_LIMIT')
                        acceleration = None
                        if previous_stamp is not None and velocity_at > previous_stamp:
                            acceleration = (velocity-previous_velocity)/(velocity_at-previous_stamp)
                            if np.max(np.abs(acceleration)) > math.radians(feedback_profile['acceleration_cap_deg_s2']):
                                self.event('estimated_acceleration_diagnostic', q=actual.tolist(), qd=velocity.tolist(),
                                    qdd_actual_estimate=acceleration.tolist(), phase=block['phase'],
                                    reference_profile=block['profile'], feedback_profile=feedback_profile)
                        previous_velocity, previous_stamp = velocity, velocity_at
                        evidence = {'block': block_index, 'phase': block['phase'], 'elapsed_s': now-began,
                            'reference_t': stamp, 'q': actual.tolist(), 'qd': velocity.tolist(),
                            'q_reference': q.tolist(), 'qd_reference': qd.tolist(), 'qdd_reference': qdd.tolist(),
                            'reference_t_of_actual': measured_t, 'q_reference_at_measurement': q_now.tolist(),
                            'monitored_error_rad': interval['monitored_error_rad'] if stage_trial else None,
                            'measured_motion_profile': feedback_profile,
                            'global_fixed_tracking_guard_used': not local_tracking_guard,
                            'estimated_acceleration_is_diagnostic_only': True,
                            'qdd_actual_estimate': None if acceleration is None else acceleration.tolist(),
                            'tracking_error_rad': error, 'deadline_lateness_s': late,
                            'send_rpc_s': send_s, 'feedback_rpc_s': read_s, 'fsm': fsm, 'io': io}
                        report['feedback'].append(evidence)
                        self.event('motion_feedback', **evidence)
                        if stamp >= path.times[-1]:
                            good = (np.max(np.abs(actual-path.positions[-1])) <= math.radians(.05)
                                    and np.max(np.abs(velocity)) <= math.radians(.05))
                            settled = settled+1 if good else 0
                            if (settled >= 3 and time.monotonic()-epoch >=
                                    path.times[-1]+report['minimum_goal_hold_s']):
                                break
                    if time.monotonic()-epoch > path.times[-1]+settings['settle_timeout_s']:
                        raise RuntimeError('Continuous phase endpoint timeout')
                    if wall_clock:
                        # Advance to a future slot, never burst-send obsolete samples.
                        next_tick = max(tick+1, math.floor((time.monotonic()-epoch)/settings['period_s'])+1)
                        report['skipped_send_slots'] += next_tick-tick-1
                        tick = next_tick
                    else:
                        tick += 1
                report['completed_blocks'] += 1
                report['completed_segments'] += len(block['source_interval_indices'])
            code = self.client.HRIF_GrpStop(0, 0)
            self.event('continuous_normal_stop', return_code=code)
            if code != 0:
                raise RuntimeError('Continuous normal stop unconfirmed')
            cutoff = time.monotonic()+settings['settle_timeout_s']
            while True:
                final = self.snapshot()
                try:
                    self.ready(final)
                    if np.max(np.abs(np.asarray(final['q'])-blocks[-1][1].positions[-1])) > math.radians(.05):
                        raise RuntimeError('Continuous final pose mismatch')
                    break
                except RuntimeError:
                    if time.monotonic() >= cutoff:
                        raise
                    time.sleep(settings['period_s'])
            report.update(completed=True, final_q=final['q'])
        except BaseException as exc:
            report['error'] = str(exc)
            if attempted:
                try:
                    report['stop_code'] = self.client.HRIF_GrpStop(0, 0)
                except Exception as stop_error:
                    report['stop_unconfirmed'] = str(stop_error)
        report['elapsed_s'] = time.monotonic()-began
        self.event('stage_complete', **{k: v for k, v in report.items() if k != 'feedback'})
        return report
