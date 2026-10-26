"""Operator-driven S20 workflow with independent pick/place cycle commands."""
import argparse
from contextlib import contextmanager, ExitStack
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
from manual_session import load, save, associate, transition, require_scene, file_hash
from manual_hardware import Hardware, controller_payload_audit, validate_stage_plan
from manual_two_step import fresh_cycle, describe_scene, release_context, settled_support_geometry, fixed_floor_release_evidence, fixed_support_release_evidence
from execution_speed_policy import point_execution_profiles, validate_speed_scale, validate_velocity_cap, DEFAULT_FREE_SPACE_VELOCITY_DEG_S

def pick_view_complete(selected):
    """A stable graspable target need not lie at the image center."""
    box = selected['box']
    return bool(selected.get('ready_for_stable_pick') and box.get('complete_view') and box.get('grasp_patch_clear') and box.get('top_surface_valid') and box.get('height_valid'))

def checked_release_gap(value):
    if value is None:
        return None
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)) or (not np.isfinite(value)) or (not 0.012 <= value <= 0.1):
        raise ValueError('Explicit floor release gap must be finite and within 12--100 mm')
    return float(value)

class ObservationRequired(RuntimeError):

    def __init__(self, diagnostics, path, history=None, reason=None):
        super().__init__('OBSERVATION_REQUIRED: current selected surface is not ready for pick')
        self.details = {'diagnostics': str(path), 'inventory_image': diagnostics['inventory_image'], 'candidates': [{'provisional_index': row['provisional_index'], 'geometry_frames': row['geometry_frames'], 'complete_frames': row['complete_frames'], 'grasp_patch_clear_frames': row['grasp_patch_clear_frames'], 'blocking_reasons': row['blocking_reasons'], 'view': row['box']['view']} for row in diagnostics['candidates']], 'automatic_observation_history': history or [], 'motion_sent': any((row.get('motion_sent') for row in history or [])), 'reason': reason, 'next': 'inventory; inspect the retained selected_luggage before retrying'}

class Workflow:

    def __init__(self, root, hardware_factory=Hardware, *, velocity_cap_deg_s=DEFAULT_FREE_SPACE_VELOCITY_DEG_S, execution_variant='point'):
        self.root = Path(root).resolve()
        self.state_path = self.root / 'runs/manual/session.json'
        self.state = load(self.state_path)
        self.hardware_factory = hardware_factory
        self.events = self.root / 'runs/manual/events.jsonl'
        self.velocity_cap_deg_s = validate_velocity_cap(velocity_cap_deg_s)
        if execution_variant not in ('point', 'point3', 'point5', 'endpoint', 'continuous', 'blend5'):
            raise ValueError('Unknown execution variant')
        self.execution_variant = execution_variant

    def _blend_scene(self, scene):
        if self.execution_variant == 'blend5':
            from native_blend_executor import load_contract
            scene['native_blend_contract'] = load_contract(self.root)
        return scene

    def hardware(self):
        return self.hardware_factory(self.events)

    def persist(self):
        save(self.state_path, self.state)

    def run_dir(self, label):
        path = self.root / 'runs/manual' / ('%s_%d' % (label, time.time_ns()))
        path.mkdir(parents=True, exist_ok=False)
        return path

    def status(self, offline=False):
        result = {'phase': self.state['phase'], 'selected_id': self.state['selected_id'], 'selected_luggage': self.state.get('selected_luggage'), 'search_pose_file': self.state.get('search_pose_file'), 'inventory': [{'id': key, 'status': v['status'], 'visible': v.get('visible', False), 'size_m': v['box']['size_m']} for (key, v) in self.state['items'].items()], 'batch': self.state.get('batch'), 'cycle': self.state.get('cycle'), 'occupancy_policy': self.state.get('occupancy_policy')}
        scene = self.root / 'config/manual_cell.json'
        try:
            config = json.loads(scene.read_text())
            require_scene(config)
            result['place_scene_ready'] = True
            result['scene_source'] = config.get('configuration_authority', 'measured')
            result['scene_measured'] = config.get('measured', False)
            result['place_scene_ready_scope'] = 'Configuration accepted; does not certify a trajectory or a physical cycle'
        except (ValueError, FileNotFoundError) as exc:
            result.update(place_scene_ready=False, place_blocker=str(exc))
        if not offline:
            with self.hardware() as hw:
                result['robot'] = hw.snapshot()
        return result

    def recover_empty(self, confirmed=False):
        if not confirmed:
            raise ValueError('Operator must confirm that the tool is empty and all cargo is supported')
        if self.state['phase'] not in ('FAULT', 'IDLE', 'HOME', 'OBSERVED'):
            raise RuntimeError('Recovery cannot discard an active carrying or release stage')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if live['io']['DI0'] or live['io']['DO1']:
            raise RuntimeError('Recovery requires no vacuum seal and no active release output')
        run = self.run_dir('empty_recovery')
        save(run / 'previous_session.json', self.state)
        items = copy.deepcopy(self.state['items'])
        for item in items.values():
            item['visible'] = False
            if not item['status'].startswith('PLACED'):
                item['status'] = 'AVAILABLE'
        references = {key: self.state[key] for key in ('search_pose_file', 'urdf') if key in self.state}
        self.state = {'schema': 's20_manual_session/v1', 'phase': 'IDLE', 'revision': self.state.get('revision', 0) + 1, 'next_id': self.state['next_id'], 'items': items, 'selected_id': None, 'runs': self.state.get('runs', []), 'observation_requires_refresh': True, 'recovery': {'archive': str(run / 'previous_session.json'), 'robot': live, 'operator_confirmed_empty_tool': True, 'time_s': time.time()}}
        self.state.update(references)
        self.persist()
        result = {'phase': 'IDLE', 'previous_session': str(run / 'previous_session.json'), 'motion_sent': False, 'outputs_written': False, 'old_paths_invalidated': True, 'fresh_scan_required': True, 'empty_motion_requires_pump_off': bool(live['io']['DO0'])}
        save(run / 'recovery.json', result)
        return result

    def execution_config(self):
        path = self.root / 'config/manual_continuous.json'
        result = json.loads(path.read_text())
        from native_blend_executor import load_contract
        return {'configuration': result, 'native_blend_contract': load_contract(self.root), 'file': str(path), 'motion_sent': False, 'outputs_written': False, 'default_execution': 'point', 'default_speed_scale': 3.0, 'free_space_velocity_cap_deg_s': self.velocity_cap_deg_s, 'explicit_cap_overrides_legacy_speed_scale': True, 'point_speed_profiles': [*point_execution_profiles({'loaded': False, 'segments': [{'phase': 'observe'}]}, 3.0, self.velocity_cap_deg_s), *point_execution_profiles({'loaded': True, 'segments': [{'phase': 'lift'}, {'phase': 'place'}]}, 3.0, self.velocity_cap_deg_s)], 'point_execution_still_stops_at_each_waypoint': True}

    def servo_transport_check(self, samples=20):
        if not 3 <= samples <= 100:
            raise ValueError('samples must be 3..100')
        durations = []
        with self.hardware() as hw:
            hw.ready(hw.snapshot())
            methods = {name: hasattr(hw.client, name) for name in ('HRIF_StartServo', 'HRIF_PushServoJ', 'HRIF_GrpStop')}
            for _ in range(samples):
                started = time.monotonic()
                q = hw.read('HRIF_ReadActJointPos', 0)
                hw.read('HRIF_ReadActJointVel', 0)
                state = list(map(int, hw.read('HRIF_ReadRobotState', 0)))
                fsm = int(hw.read('HRIF_ReadCurFSM', 0)[0])
                hw.io()
                if len(q) != 6 or not np.isfinite(np.asarray(q, dtype=float)).all() or len(state) < 13 or (fsm != 33) or state[0] or any((state[i] for i in (2, 6, 7, 8))):
                    raise RuntimeError('Robot state changed during read-only transport check')
                durations.append(time.monotonic() - started)
                time.sleep(0.04)
        report = {'motion_sent': False, 'outputs_written': False, 'sdk_servo_methods_present': methods, 'samples': samples, 'feedback_rpc_s': {'median': float(np.median(durations)), 'p95': float(np.percentile(durations, 95)), 'maximum': max(durations)}, 'send_rpc_not_measured': True, 'hardware_execution_qualified': False}
        run = self.run_dir('servo_transport')
        save(run / 'report.json', report)
        return dict(report, report=str(run / 'report.json'))

    def compile_stage(self, plan, speed_scale=3.0, run=None):
        from continuous_reference import prepare_stage
        model = Path(self.state['urdf'])
        if plan.get('robot_model_sha256') and file_hash(model) != plan['robot_model_sha256']:
            raise RuntimeError('Source plan robot model changed')
        report = prepare_stage(plan, model.read_text(), speed_scale, self.velocity_cap_deg_s)
        run = run or self.run_dir('continuous_stage')
        path = run / 'continuous_reference.json'
        save(path, report)
        return (report, path)

    def execute_stage(self, plan, execution='point', speed_scale=3.0, run=None):
        validate_speed_scale(speed_scale)
        if execution in ('point', 'point3', 'point5', 'endpoint', 'blend5'):
            kwargs = {'speed_scale': speed_scale, 'velocity_cap_deg_s': self.velocity_cap_deg_s}
            if any(('native_blend_box' in row for row in plan.get('segments', []))):
                from native_blend_executor import load_contract
                kwargs['native_blend_contract'] = load_contract(self.root)
            with self.hardware() as hw:
                return hw.execute(plan, **kwargs)
        if execution != 'continuous':
            raise ValueError('Unknown execution mode')
        from continuous_executor import validate_configuration
        config = self.execution_config()['configuration']
        validate_configuration(config, file_hash(self.state['urdf']))
        (reference, path) = self.compile_stage(plan, speed_scale, run)
        with self.hardware() as hw:
            report = hw.execute(plan, mode='continuous', reference=reference, qualification=config)
        report['continuous_reference'] = str(path)
        return report

    def preview_execution(self, plan, execution, speed_scale, run):
        if execution in ('point', 'point3', 'point5', 'endpoint', 'blend5'):
            return {'execution_mode': 'point', 'execution_variant': execution, 'native_blended_command_count': sum(('native_blend_box' in row and row['native_blend_index'] < len(row['native_blend_box']['points']) - 1 for row in plan.get('segments', []))), 'speed_profiles': point_execution_profiles(plan, speed_scale, self.velocity_cap_deg_s), 'continuous_execution': False}
        (reference, path) = self.compile_stage(plan, speed_scale, run)
        return {'execution_mode': 'continuous', 'continuous_reference': str(path), 'continuous_reference_validated': reference['joint_box_geometry_certificate_reused'], 'continuous_reference_duration_s': reference['reference_duration_s'], 'hardware_execution_qualified': False, 'motion_sent': False}

    def initialize(self, plan_only=False, execution='point', speed_scale=3.0):
        if not (self.root / 'runs/manual/search_pose.json').is_file():
            raise RuntimeError('No recorded observation reference; initialization cannot invent a home pose')
        if self.state['phase'] not in ('IDLE', 'HOME', 'OBSERVED'):
            raise RuntimeError('Initialization requires an empty observation stage')
        require_scene(json.loads((self.root / 'config/manual_cell.json').read_text()))
        profile = self.search_pose()
        reference = np.asarray(profile['q_rad'], dtype=float)
        if reference.shape != (6,) or not np.isfinite(reference).all():
            raise RuntimeError('Recorded observation joints must be finite')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if any(live['io'].values()):
            raise RuntimeError('Initialization requires vacuum OFF and an empty tool')
        if np.max(np.abs(np.asarray(live['q']) - reference)) <= np.deg2rad(0.05):
            self.state.update(selected_id=None, search_index=0, observation_requires_refresh=True)
            self.state.pop('selected_luggage', None)
            self.persist()
            return {'completed': True, 'no_motion_needed': True, 'motion_sent': False, 'stage': 'INITIALIZE_OBSERVATION', 'fresh_scan_before_pick_required': True, 'redundant_initialization_scan_skipped': True}
        result = self.observe(plan_only, search_index=0, execution=execution, speed_scale=speed_scale)
        if result.get('no_motion_needed'):
            result.update(completed=True)
        if result.get('completed'):
            self.state.update(selected_id=None, search_index=0, observation_requires_refresh=True)
            self.state.pop('selected_luggage', None)
            self.persist()
        return dict(result, stage='INITIALIZE_OBSERVATION', fresh_scan_before_pick_required=True)

    def capture(self, run):
        completion = run / 'capture_finished.signal'
        environment = dict(os.environ, S20_CAPTURE_SECONDS='9', S20_SKIP_DETECT='1',
            S20_SENSING_DONE_FILE=str(completion), ROS_DOMAIN_ID='7', ROS_LOCALHOST_ONLY='0')
        began = time.monotonic()
        sensing_log = run / 'sensing.log'
        with sensing_log.open('w') as log:
            sense = subprocess.Popen(['bash', str(self.root / 'run_height_shadow.sh')], stdout=log, stderr=subprocess.STDOUT, env=environment)
            try:
                subprocess.run([sys.executable, str(self.root / 'scripts/capture_height_inputs.py'), '--output', str(run / 'capture'), '--duration', '9', '--period-s', '.5', '--max-frames', '8', '--min-span-s', '2'], env=environment, check=True)
                captured_at = time.monotonic()
                completion.touch(exist_ok=False)
                if sense.wait(timeout=35) != 0:
                    raise RuntimeError('Read-only sensing failed: ' + str(sensing_log))
            except subprocess.CalledProcessError as exc:
                report_path = run / 'capture/capture.json'
                report = json.loads(report_path.read_text()) if report_path.is_file() else {}
                detail = (report.get('tf_errors') or ['No synchronized RGB-D/YOLO frame received'])[-1]
                raise RuntimeError('OBSERVATION_CAPTURE_FAILED: ' + detail + '; inspect ' + str(sensing_log) + '; no pick motion sent') from exc
            finally:
                if sense.poll() is None:
                    sense.terminate()
                    sense.wait(timeout=10)
        first = sensing_log.read_text().splitlines()[0]
        source = Path(first).resolve()
        if source.parent != self.root / 'runs' or not source.name.startswith('height_shadow_'):
            raise RuntimeError('Unexpected sensing run path')
        obs = json.loads((source / 'result.json').read_text())
        if not obs.get('last_joints'):
            raise RuntimeError('No actual joint feedback')
        save(run / 'capture_timing.json', dict(capture_wall_s=captured_at-began,
            post_capture_finalize_wall_s=time.monotonic()-captured_at,
            total_wall_s=time.monotonic()-began,
            sensing_completion='valid_capture_finished', fixed_sensing_hold_removed=True))
        return (run / 'capture', obs, source / 'base_root_robot.urdf')

    def scan(self, capture=None, observation=None, urdf=None):
        from manual_inventory import scan_capture
        run = self.run_dir('scan')
        if capture is None:
            (capture, observation, urdf) = self.capture(run)
        result = scan_capture(capture, self.state, associate, assign_ids=False)
        save(run / 'inventory.json', result)
        self.state.update(last_capture=str(capture), last_observation=observation, urdf=str(urdf), last_inventory=str(run / 'inventory.json'))
        if self.state['phase'] in ('IDLE', 'HOME', 'OBSERVED'):
            transition(self.state, 'OBSERVED')
        self.persist()
        return (result, run)

    def inventory(self):
        from manual_inventory import capture_diagnostics
        if not self.state.get('last_capture'):
            raise RuntimeError('No captured inventory; run scan first')
        scanned = json.loads(Path(self.state['last_inventory']).read_text())
        result = capture_diagnostics(self.state['last_capture'], scanned)
        result.update(selected_luggage=self.state.get('selected_luggage'), search_pose_file=self.state.get('search_pose_file'))
        return result

    def _remember_search_pose(self, scanned, overwrite=False):
        path = self.root / 'runs/manual/search_pose.json'
        if path.is_file() and (not overwrite):
            self.state['search_pose_file'] = str(path)
            self.persist()
            return json.loads(path.read_text())
        from articulated_svsdf import ArticulatedModel
        from run_articulated_svsdf import resolve_package
        from observation_policy import search_profile
        model = ArticulatedModel(Path(self.state['urdf']).read_text(), resolve_package, collision_mode='primitive_proxy')
        joints = self.state['last_observation']['last_joints']
        q = [joints['q'][joints['names'].index(name)] for name in model.joint_names]
        folder = Path(self.state['last_capture'])
        name = json.loads((folder / 'capture.json').read_text())['frames'][-1]
        metadata = json.loads((folder / (name + '.json')).read_text())
        camera = np.eye(4)
        camera[:3, :3] = Rotation.from_quat(metadata['quaternion']).as_matrix()
        camera[:3, 3] = metadata['translation']
        scene = json.loads((self.root / 'config/manual_cell.json').read_text())
        require_scene(scene)
        scene['execution_variant'] = 'point3' if getattr(self, 'execution_variant', 'point') == 'point3' else getattr(self, 'execution_variant', 'point')
        floors = [box['center_m'][2] - box['size_m'][2] / 2 for row in scanned['frames'] for box in row['measurements']]
        floor = float(np.median(floors)) if floors else float(np.asarray(scene['base_T_ULD'])[2, 3])
        profile = search_profile(q, model.fk(q)['suction_contact_frame'], camera, floor, file_hash(self.state['urdf']))
        profile['ground_source'] = 'measured box support median' if floors else 'operator-authorized fixed-cell ground prior'
        save(path, profile)
        self.state.update(search_pose_file=str(path), search_index=0)
        self.persist()
        return profile

    def search_pose(self, record=False):
        if not record:
            path = Path(self.state.get('search_pose_file') or self.root / 'runs/manual/search_pose.json')
            if not path.is_file():
                raise RuntimeError('No search pose recorded yet; use search-pose --record')
            return json.loads(path.read_text())
        if self.state['phase'] not in ('IDLE', 'HOME', 'OBSERVED'):
            raise RuntimeError('Search pose recording requires an empty observation stage')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if any(live['io'].values()):
            raise RuntimeError('Search pose recording requires vacuum OFF')
        (scanned, _) = self.scan()
        return self._remember_search_pose(scanned, overwrite=True)

    def _lock_target(self, diagnostics, target='auto'):
        from observation_policy import select_locked_target
        from manual_inventory import render_selection
        if target not in ('auto', 'leftmost', 'rightmost'):
            raise ValueError('Visual ID matching is disabled; select auto/leftmost/rightmost from the current view')
        selected = select_locked_target(self.state, diagnostics['candidates'], target)
        if selected:
            self.state['selected_luggage']['persistent_id'] = None
            diagnostics['selected_provisional_index'] = selected['provisional_index']
            self.state['selected_luggage']['image'] = render_selection(self.state['last_capture'], self.state['selected_luggage'])
        self.persist()
        return selected

    def observe(self, plan_only=False, distance=0.12, scan_data=None, selected=None, search_index=None, execution='point', speed_scale=3.0):
        if self.state['phase'] not in ('IDLE', 'HOME', 'OBSERVED'):
            raise RuntimeError('View adjustment requires an empty observation stage')
        if not 0.01 <= distance <= 0.15:
            raise ValueError('View adjustment distance must be 0.01..0.15 m')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if any(live['io'].values()) and (not plan_only):
            raise RuntimeError('View adjustment requires vacuum OFF and no retained load')
        scene_path = self.root / 'config/manual_cell.json'
        scene = json.loads(scene_path.read_text())
        require_scene(scene)
        self._blend_scene(scene)
        placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
        cached = self.state.get('observation_candidate', {})
        report = None
        context = {'scene': file_hash(scene_path), 'distance': distance, 'placed': placed, 'scan_serial': self.state.get('scan_serial', 0), 'selection': self.state.get('selected_luggage', {}).get('id'), 'search_index': search_index, 'policy': 'centering_and_framing_v3', 'execution_variant': execution}
        if execution == 'blend5':
            context['native_blend_contract'] = scene['native_blend_contract']
        if cached.get('context') == context and time.time() - cached.get('created_s', 0) <= 90 and Path(cached.get('path', '')).is_file() and (file_hash(cached['path']) == cached.get('sha256')) and self.state.get('urdf') and (file_hash(self.state['urdf']) == cached.get('model_sha256')):
            candidate = json.loads(Path(cached['path']).read_text())
            if candidate.get('validated') and (not candidate.get('preview_only')) and (np.max(np.abs(np.asarray(candidate['start_q']) - live['q'])) <= np.deg2rad(0.05)):
                report = candidate
        reused = report is not None
        run = self.run_dir('observe')
        if report is None:
            (scanned, scan_run) = scan_data if scan_data is not None else self.scan()
            from manual_inventory import capture_diagnostics
            diagnostics = capture_diagnostics(self.state['last_capture'], scanned)
            if search_index is None:
                selected = selected or self._lock_target(diagnostics)
                if selected is None:
                    raise RuntimeError('No stable target; use recorded clockwise search candidates')
                diagnostics['selected_provisional_index'] = selected['provisional_index']
            context['selection'] = self.state.get('selected_luggage', {}).get('id')
            details = run / 'diagnostics.json'
            save(details, diagnostics)
            source = run / 'observation.json'
            save(source, self.state['last_observation'])
            scene['placed'] = placed
            fixed = run / 'fixed_scene.json'
            save(fixed, scene)
            output = run / 'plan.json'
            command = [sys.executable, str(self.root / 'scripts/plan_observation_trial.py'), '--capture', self.state['last_capture'], '--observation', str(source), '--urdf', self.state['urdf'], '--fixed-scene', str(fixed), '--diagnostics', str(details), '--output', str(output), '--distance', str(distance), '--execution-variant', execution]
            if search_index is not None:
                if search_index not in (0, 1, 2, 3):
                    raise ValueError('Search index must be 0..3')
                if search_index != 0 and self.state.get('selected_luggage'):
                    raise RuntimeError('Do not search away from a locked suitcase')
                profile = self._remember_search_pose(scanned)
                goal = dict(profile['candidates'][search_index], ground_z_m=profile['ground_z_m'], model_sha256=profile['model_sha256'])
                if search_index == 0:
                    goal.update(initialize_reference=True, q_rad=profile['q_rad'])
                destination = run / 'search_goal.json'
                save(destination, goal)
                command += ['--search-goal', str(destination)]
            result = subprocess.run(command, stdout=subprocess.DEVNULL)
            if not output.is_file():
                raise RuntimeError('View adjustment planner did not produce a report; exit=' + str(result.returncode))
            report = json.loads(output.read_text())
            if report.get('no_motion_needed'):
                return {'motion_sent': False, 'no_motion_needed': True, 'completed': True, 'next': 'step1 --target auto', 'plan': str(output)}
            if not report.get('validated'):
                return {'motion_sent': False, 'validated': False, 'error': report.get('error'), 'plan': str(output), 'inventory_image': diagnostics['inventory_image']}
            context['scan_serial'] = self.state.get('scan_serial', 0)
        output = run / 'validated_plan.json'
        save(output, report)
        self.state['observation_candidate'] = {'context': context, 'path': str(output), 'sha256': file_hash(output), 'created_s': cached['created_s'] if reused else time.time(), 'model_sha256': file_hash(self.state['urdf'])}
        self.persist()
        if plan_only:
            return {'motion_sent': False, 'validated': True, 'plan': str(output), 'reused_plan': reused, 'camera_translation_base_m': report['camera_translation_base_m'], 'reference_duration_s': report['reference_duration_s'], 'selected_luggage': self.state.get('selected_luggage'), 'search_index': search_index, 'next': 'observe; then step1 --target auto after the empty-tool move', **self.preview_execution(report, execution, speed_scale, run)}
        executed = self.execute_stage(report, execution, speed_scale, run)
        save(run / 'execution.json', executed)
        if not executed['completed']:
            self.state.update(phase='FAULT', last_failure=executed.get('error'))
            self.persist()
            raise RuntimeError(executed.get('error', 'View adjustment stopped'))
        self.state.update(home_q=executed['final_q'], observation_requires_refresh=True)
        if search_index is not None:
            self.state['search_index'] = search_index
        self.state.pop('observation_candidate', None)
        self.persist()
        return {'phase': self.state['phase'], 'completed': True, 'motion_sent': True, 'reused_plan': reused, 'execution': str(run / 'execution.json'), 'next': 'step1 --target auto; fresh observation required'}

    def bootstrap(self, contact_run, sensing_run):
        if self.state['items']:
            raise RuntimeError('Session already exists; do not overwrite its identities')
        contact_run = Path(contact_run)
        sensing_run = Path(sensing_run)
        execution = json.loads((contact_run / 'execution.json').read_text())
        plan = json.loads((contact_run / 'plan.json').read_text())
        observation = json.loads((sensing_run / 'result.json').read_text())
        if not execution.get('nominal_surface_pose_reached'):
            raise ValueError('Historical contact did not complete')
        box = copy.deepcopy(next((f['box'] for f in reversed(observation['frames']) if f['height_valid'])))
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if np.max(np.abs(np.asarray(live['q']) - execution['final_q'])) > np.deg2rad(0.05):
            raise RuntimeError('Current pose differs from recorded contact; no adoption performed')
        run = self.run_dir('adopt')
        rebuilt = run / 'contact_scene_replay.json'
        subprocess.run([sys.executable, str(self.root / 'scripts/plan_contact_trial.py'), '--observation', str(sensing_run / 'result.json'), '--capture', str(contact_run / 'capture'), '--urdf', str(sensing_run / 'base_root_robot.urdf'), '--output', str(rebuilt)], check=True, stdout=subprocess.DEVNULL)
        from manual_inventory import measurements
        capture = json.loads((contact_run / 'capture/capture.json').read_text())
        name = capture['frames'][-1]
        meta = json.loads((contact_run / 'capture' / (name + '.json')).read_text())
        data = np.load(contact_run / 'capture' / (name + '.npz'))
        measured = measurements(data['depth'], meta)
        nearby = [b for b in measured if np.linalg.norm(np.asarray(b['center_m']) - box['center_m']) < 0.12]
        if len(nearby) == 1:
            for field in ('collision_size_m', 'collision_pose', 'collision_source'):
                if field in nearby[0]:
                    box[field] = nearby[0][field]
        key = 'case-0001'
        box['id'] = key
        self.state.update(items={key: {'status': 'SELECTED', 'visible': True, 'box': box, 'identity_source': 'operator-confirmed previous contact; not a fresh visual Re-ID'}}, next_id=2, selected_id=key, home_q=execution['start_q'], contact_q=live['q'], contact_plan=str(rebuilt), contact_capture=str(contact_run / 'capture'), urdf=str(sensing_run / 'base_root_robot.urdf'))
        self.state['contact_scan_serial'] = self.state.get('scan_serial', 0)
        self.state['contact_model_hash'] = file_hash(self.state['urdf'])
        transition(self.state, 'AT_CONTACT')
        self.persist()
        return {'phase': 'AT_CONTACT', 'id': key, 'motion_sent': False, 'pump_state': live['io']}

    def step1(self, target='auto', plan_only=False, auto_observe=True, max_view_moves=3, execution='point', speed_scale=3.0, ik_backend='local'):
        if self.state['phase'] == 'AT_CONTACT':
            if target not in ('auto', self.state['selected_id']):
                raise RuntimeError('Finish or explicitly cancel the selected case before changing targets')
            with self.hardware() as hw:
                live = hw.snapshot()
            if np.max(np.abs(np.asarray(live['q']) - self.state['contact_q'])) > np.deg2rad(0.05):
                raise RuntimeError('Recorded contact pose has changed')
            return {'already_at_contact': True, 'id': self.state['selected_id'], 'next': 'pump on', 'motion_sent': False}
        if self.state['phase'] not in ('IDLE', 'HOME', 'OBSERVED'):
            raise RuntimeError('Step 1 requires an empty/home stage')
        if not 0 <= max_view_moves <= 6:
            raise ValueError('max-view-moves must be 0..6')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if any(live['io'].values()) and (not plan_only):
            raise RuntimeError('Step 1 requires vacuum OFF and an empty tool')
        history = []
        self.state['observation_history'] = history
        tried_search = set()
        view_moves = 0
        (result, run) = self.scan()
        self._remember_search_pose(result)
        for attempt in range(max_view_moves + 5):
            from manual_inventory import capture_diagnostics, selected_geometry_frames
            diagnostics = capture_diagnostics(self.state['last_capture'], result)
            path = run / 'observation_diagnostics.json'
            save(path, diagnostics)
            try:
                selected = self._lock_target(diagnostics, target)
            except ValueError as exc:
                raise ObservationRequired(diagnostics, path, history, str(exc)) from exc
            if selected:
                lock = self.state['selected_luggage']
                selected_frames = selected_geometry_frames(result, selected)
                if pick_view_complete(selected) and len(selected_frames) >= 5:
                    target = 'selection-%d' % time.time_ns()
                    box = copy.deepcopy(selected['box'])
                    box['id'] = target
                    self.state['items'][target] = {'status': 'AVAILABLE', 'visible': True, 'box': box, 'identity_source': 'current selected surface; no ground visual Re-ID', 'observation_capture': self.state['last_capture']}
                    lock.update(persistent_id=None, operation_handle=target, status='VISION_READY')
                    self.state['selected_id'] = target
                    self.persist()
                    break
            if not auto_observe or (selected is not None and view_moves >= max_view_moves):
                raise ObservationRequired(diagnostics, path, history, 'Automatic observation budget exhausted or disabled')
            search_index = None
            if selected is None:
                search_index = self.state.get('search_index', 0) + 1
                while search_index in tried_search:
                    search_index += 1
                if search_index > 3:
                    raise ObservationRequired(diagnostics, path, history, 'Recorded 60-degree search sector exhausted')
                tried_search.add(search_index)
            else:
                view_moves += 1
            adjusted = self.observe(plan_only, distance=0.12, scan_data=(result, run), selected=selected, search_index=search_index, execution=execution, speed_scale=speed_scale)
            history.append(copy.deepcopy(adjusted))
            self.state['observation_history'] = history
            self.persist()
            if adjusted.get('validated') is False:
                if search_index is not None and (not plan_only):
                    continue
                raise ObservationRequired(diagnostics, path, history, 'Observation path did not validate: ' + str(adjusted.get('error')))
            if plan_only:
                return dict(adjusted, stage='AUTO_OBSERVE_BEFORE_PICK', automatic_observation=True, next='step1 --target auto; it will adjust, rescan, then approach the locked suitcase')
            (result, run) = self.scan()
        else:
            raise ObservationRequired(diagnostics, path, history, 'Observation did not converge within bounded attempts')
        observation = copy.deepcopy(self.state['last_observation'])
        observation['frames'] = selected_frames
        selected = run / 'selected_observation.json'
        save(selected, observation)
        plan_path = run / 'pick_plan.json'
        cell = json.loads((self.root / 'config/manual_cell.json').read_text())
        require_scene(cell)
        self._blend_scene(cell)
        cell['placed'] = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
        fixed_scene = run / 'fixed_scene.json'
        save(fixed_scene, cell)
        planned = subprocess.run([sys.executable, str(self.root / 'scripts/plan_contact_trial.py'), '--observation', str(selected), '--capture', self.state['last_capture'], '--urdf', self.state['urdf'], '--output', str(plan_path), '--fixed-scene', str(fixed_scene), '--ik-backend', ik_backend, '--execution-variant', execution], stdout=subprocess.DEVNULL)
        if not plan_path.is_file():
            raise RuntimeError('Pick planner did not produce a report; return code %s' % planned.returncode)
        plan = json.loads(plan_path.read_text())
        if planned.returncode or not plan.get('qualified_for_supervised_trial'):
            raise RuntimeError('Pick plan was not validated: ' + str(plan.get('error', plan.get('reason', 'unknown'))))
        stage = {'validated': True, 'loaded': False, 'start_q': plan['start_q'], 'segments': plan['segments']}
        if plan_only:
            return {'plan': str(plan_path), 'motion_sent': False, 'target': target, 'validated': True, 'stage': 'PICK', 'selected_luggage': self.state.get('selected_luggage'), **self.preview_execution(stage, execution, speed_scale, run)}
        early = self._start_early_place_prefetch(plan_path, target, execution)
        try:
            executed = self.execute_stage(stage, execution, speed_scale, run)
        except BaseException:
            self._cancel_place_prefetch()
            self.persist()
            raise
        save(run / 'execution.json', executed)
        if not executed['completed']:
            self._cancel_place_prefetch()
            self.state.update(phase='FAULT', last_failure=executed.get('error'))
            self.persist()
            raise RuntimeError(executed.get('error', 'Pick stopped'))
        self.state['items'][target]['status'] = 'SELECTED'
        self.state.setdefault('home_q', plan['start_q'])
        transition(self.state, 'AT_CONTACT', selected_id=target, contact_q=executed['final_q'], contact_plan=str(plan_path))
        self.state['contact_capture'] = self.state['last_capture']
        self.state['contact_scan_serial'] = self.state.get('scan_serial', 0)
        self.state['contact_model_hash'] = file_hash(self.state['urdf'])
        self.state['contact_plan_sha256'] = file_hash(plan_path)
        self.state['contact_scene_file_sha256'] = file_hash(self.root / 'config/manual_cell.json')
        self.persist()
        self.state['selected_luggage']['status'] = 'AT_CONTACT'
        self.persist()
        return {'phase': 'AT_CONTACT', 'id': target, 'next': 'pump on', 'execution': str(run / 'execution.json'), 'motion_sent': True, 'automatic_observation_moves': history, 'early_place_prefetch': early, 'selected_luggage': self.state['selected_luggage']}

    def scene_info(self):
        path = self.root / 'config/manual_cell.json'
        result = describe_scene(json.loads(path.read_text()))
        result.update(scene_file=str(path), scene_file_sha256=file_hash(path), occupancy_policy=self.state.get('occupancy_policy'), retained_placed_ids=[key for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')])
        return result

    def pick_sequence(self):
        profile = self.search_pose()
        return {'motion_sent': False, 'outputs_written': False, 'scene': self.scene_info(), 'search_reference': profile, 'sequence': ['empty-tool initialize to recorded observation', 'fresh RGB-D scan; no old ground-box poses', 'if absent: absolute clockwise 20, 40, 60 degrees from reference', 'lock one current target; bounded adjustments only until full view', 'fit top plane/rectangle; aligned center tool goal; native SDK IK seed', 'whole-body joint-box sweep and joint-limit validation', 'contact; plan and validate attached-box lift', 'vacuum ON and pressure confirmation; lift at least 0.5 m', 'try validated standard staging; otherwise retain verified lift endpoint'], 'default_ULD_assumption': 'empty; only explicit batch continuation retains placements', 'view_centering_is_not_required': True, 'history_may_supply_motion_seeds_not_ground_box_coordinates': True, 'loaded_initialization_to_empty_home_is_forbidden': True}

    def _checkpoint(self, name, result):
        row = {'stage': name, 'phase': self.state['phase'], 'recorded_s': time.time(), 'result': copy.deepcopy(result)}
        self.state.setdefault('cycle', {}).setdefault('stages', []).append(row)
        self.persist()
        print(json.dumps({'cycle_stage': name, 'phase': self.state['phase'], 'motion_sent': result.get('motion_sent', False), 'id': self.state.get('selected_id'), 'plan': result.get('plan'), 'execution': result.get('execution')}, ensure_ascii=False), flush=True)

    def _pick_payload_check(self, plan_only=False):
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
            payload = controller_payload_audit(hw.read('HRIF_ReadPayload'))
        return (live, dict(payload, motion_sent=False, outputs_written=False))

    def begin_cycle(self, batch_size=1, continue_batch=False):
        run = self.run_dir('new_pick_cycle')
        new_state = fresh_cycle(self.state, batch_size=batch_size, continue_batch=continue_batch)
        new_state.pop('planned_release_gap_m', None)
        self._cancel_place_prefetch()
        save(run / 'previous_session.json', self.state)
        new_state['cycle']['archive'] = str(run / 'previous_session.json')
        profile = self.search_pose()
        new_state['cycle_home_q'] = profile['q_rad']
        new_state['search_pose_file'] = str(self.root / 'runs/manual/search_pose.json')
        self.state = new_state
        self.persist()
        return {'archive': str(run / 'previous_session.json'), 'occupancy_policy': new_state['occupancy_policy'], 'batch': new_state['batch'], 'fresh_ground_scan_required': True, 'old_ground_targets_discarded': True, 'placed_obstacle_count': len(new_state['items']), 'motion_sent': False, 'outputs_written': False}

    @contextmanager
    def _attachment_planner(self, case, source, contact_q, current_q, ik_backend):
        from manual_motion import Planner
        scene = json.loads((self.root / 'config/manual_cell.json').read_text())
        require_scene(scene)
        scene['execution_variant'] = getattr(self, 'execution_variant', 'point')
        self._blend_scene(scene)
        xml = Path(self.state['urdf']).read_text()
        with ExitStack() as resources:
            planner = Planner(xml, scene, source['scene'], case, contact_q, seed_library=self.root / 'config/fixed_cell_motion_seeds.json')
            resources.callback(planner.close)
            release_gap = checked_release_gap(self.state.get('planned_release_gap_m'))
            if release_gap is not None:
                planner.release_gap_m = release_gap
                planner.allow_floor_drop = True
            audit = []
            metadata = self.state.get('early_place_prefetch')
            if metadata is not None:
                try:
                    from manual_place_prefetch import checked_early_hints
                    from phase_seed_library import PhaseSeedLibrary, combine_phase_seed_libraries
                    placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
                    context = self._early_context(case, placed, self.state['contact_plan'], getattr(self, 'execution_variant', 'point'))
                    hints = checked_early_hints(metadata, context)
                    proposals = PhaseSeedLibrary.from_proposal_rows(hints['phase_rows'], xml, planner.cell, planner.names)
                    planner.motion_seeds = combine_phase_seed_libraries(proposals, planner.motion_seeds)
                    planner.endpoint_proposals = hints['endpoint_slots']
                    planner.seed_audit.append({'backend': 'early_place_proposals', 'used_as_hints': True, 'worker_wall_s': hints['worker_wall_s'], 'phase_count': len(proposals.rows), 'merge_audit': getattr(planner.motion_seeds, 'merge_audit', None), 'current_grasp_and_full_sweep_required': True, 'not_a_safety_certificate': True})
                except (ValueError, KeyError, OSError) as exc:
                    planner.seed_audit.append({'backend': 'early_place_proposals', 'used_as_hints': False, 'reason': str(exc), 'foreground_planning_retained': True})
            if ik_backend == 'native-sdk':
                from manual_native_ik import NativeContactIK
                hw = resources.enter_context(self.hardware())
                live = hw.snapshot()
                hw.ready(live)
                if np.max(np.abs(np.asarray(live['q']) - current_q)) > np.deg2rad(0.05):
                    raise RuntimeError('Robot moved before attachment planning')
                planner.ik_solver = NativeContactIK(hw, planner.free, current_q, audit)
            elif ik_backend != 'local':
                raise ValueError('Unknown IK backend')
            yield (planner, audit)

    def _attachment_inputs(self):
        if file_hash(self.state['urdf']) != self.state.get('contact_model_hash'):
            raise RuntimeError('Attachment robot model changed since contact')
        if file_hash(self.state['contact_plan']) != self.state.get('contact_plan_sha256'):
            raise RuntimeError('Attachment contact plan changed')
        if file_hash(self.root / 'config/manual_cell.json') != self.state.get('contact_scene_file_sha256'):
            raise RuntimeError('Fixed scene changed since contact')
        source = json.loads(Path(self.state['contact_plan']).read_text())
        if source.get('qualified_for_supervised_trial') is not True:
            raise RuntimeError('An independently accepted contact plan is required')
        case = self.state['items'][self.state['selected_id']]['box']
        if (case.get('owned_scene_components') or case.get('ignored_nonrigid_components')) and case.get('ownership_scene_hash') != file_hash(self.state['contact_plan']):
            raise RuntimeError('Object annotation belongs to another observation')
        placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
        return (source, case, placed)

    def _place_context(self, case, placed, start_q, lift_path, ik_backend):
        from manual_place_prefetch import implementation_hashes
        library = self.root / 'config/fixed_cell_motion_seeds.json'
        context = {'scene': file_hash(self.root / 'config/manual_cell.json'), 'model': file_hash(self.state['urdf']), 'contact': file_hash(self.state['contact_plan']), 'lift': file_hash(lift_path), 'case': copy.deepcopy(case), 'placed': copy.deepcopy(placed), 'home_q': self.state['cycle_home_q'], 'selected_id': self.state['selected_id'], 'cycle_id': self.state.get('cycle', {}).get('id'), 'start_q': list(start_q), 'ik_backend': ik_backend, 'execution_variant': getattr(self, 'execution_variant', 'point'), 'motion_seed_library_sha256': file_hash(library) if library.is_file() else None, 'implementation_sha256': implementation_hashes(self.root), 'cost_profile': os.environ.get('S20_ORIGINAL_COST_PROFILE'), 'cost_weights': json.loads(os.environ.get('S20_ORIGINAL_LAYOUT_WEIGHTS', '{}'))}
        return self._with_release_goal(context)

    def _with_release_goal(self, context):
        if self.execution_variant == 'blend5':
            from native_blend_executor import load_contract
            context['native_blend_contract'] = load_contract(self.root)
        gap = checked_release_gap(self.state.get('planned_release_gap_m'))
        if gap is not None:
            context['operator_authorized_floor_release_gap_m'] = gap
        return context

    def _early_context(self, case, placed, contact_path, execution):
        from manual_place_prefetch import implementation_hashes
        library = self.root / 'config/fixed_cell_motion_seeds.json'
        context = {'scene': file_hash(self.root / 'config/manual_cell.json'), 'model': file_hash(self.state['urdf']), 'contact': file_hash(contact_path), 'case': copy.deepcopy(case), 'placed': copy.deepcopy(placed), 'home_q': self.state['cycle_home_q'], 'selected_id': self.state['selected_id'], 'cycle_id': self.state.get('cycle', {}).get('id'), 'execution_variant': execution, 'motion_seed_library_sha256': file_hash(library) if library.is_file() else None, 'implementation_sha256': implementation_hashes(self.root), 'cost_profile': os.environ.get('S20_ORIGINAL_COST_PROFILE'), 'cost_weights': json.loads(os.environ.get('S20_ORIGINAL_LAYOUT_WEIGHTS', '{}'))}
        return self._with_release_goal(context)

    def _start_early_place_prefetch(self, contact_path, target, execution):
        from manual_place_prefetch import launch_early
        try:
            self._cancel_place_prefetch()
            case = self.state['items'][target]['box']
            placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
            context = self._early_context(case, placed, contact_path, execution)
            if context['selected_id'] != target:
                raise ValueError('Selected target changed before early planning')
            request = {'context': context, 'urdf': self.state['urdf'], 'contact_plan': str(contact_path), 'budget_s': 30.0}
            (metadata, self._early_prefetch_process) = launch_early(self.root, self.run_dir('place_prefetch'), request)
            self.state['early_place_prefetch'] = metadata
            self.persist()
            return metadata
        except (ValueError, KeyError, OSError) as exc:
            return {'started': False, 'error': str(exc), 'motion_sent': False, 'outputs_written': False}

    def _cancel_place_prefetch(self, include_early=True):
        from manual_place_prefetch import cancel
        metadata = self.state.pop('place_prefetch', None)
        if metadata is not None:
            try:
                cancel(metadata)
            except OSError:
                pass
        process = getattr(self, '_prefetch_process', None)
        if process is not None:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            self._prefetch_process = None
        if include_early:
            metadata = self.state.pop('early_place_prefetch', None)
            if metadata is not None:
                try:
                    cancel(metadata)
                except OSError:
                    pass
            process = getattr(self, '_early_prefetch_process', None)
            if process is not None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    pass
                self._early_prefetch_process = None

    def _start_place_prefetch(self, lift_path, lift_plan, case, placed, budget=30.0):
        from manual_place_prefetch import launch
        try:
            self._cancel_place_prefetch(include_early=False)
            context = self._place_context(case, placed, lift_plan['final_q'], lift_path, 'local')
            request = {'context': context, 'urdf': self.state['urdf'], 'contact_plan': self.state['contact_plan'], 'contact_q': self.state['contact_q'], 'lift_plan': str(lift_path), 'budget_s': budget}
            (metadata, self._prefetch_process) = launch(self.root, self.run_dir('place_prefetch'), request)
            self.state['place_prefetch'] = metadata
            self.persist()
            return metadata
        except (ValueError, KeyError, OSError) as exc:
            return {'started': False, 'error': str(exc), 'motion_sent': False, 'outputs_written': False}

    def lift(self, plan_only=False, budget=15.0, execution='point', speed_scale=3.0, ik_backend='native-sdk'):
        validate_speed_scale(speed_scale)
        if self.state['phase'] not in ('AT_CONTACT', 'SEALED'):
            raise RuntimeError('Lift requires this cycle contact or confirmed seal')
        if not 1 <= budget <= 120:
            raise ValueError('Planning budget must be 1..120 seconds')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if np.max(np.abs(np.asarray(live['q']) - self.state['contact_q'])) > np.deg2rad(0.05):
            raise RuntimeError('Contact pose changed; no stale attachment offset is reused')
        (source, case, placed) = self._attachment_inputs()
        started = time.monotonic()
        with self._attachment_planner(case, source, self.state['contact_q'], live['q'], ik_backend) as (planner, audit):
            report = planner.plan_lift(live['q'], self.state['cycle_home_q'], placed, budget_s=budget)
        report.update(native_ik_audit=audit, ik_backend=ik_backend, planning_wall_s=time.monotonic() - started, scene_file_sha256=file_hash(self.root / 'config/manual_cell.json'), robot_model_sha256=file_hash(self.state['urdf']), contact_plan_sha256=file_hash(self.state['contact_plan']))
        run = self.run_dir('lift')
        path = run / 'plan.json'
        save(path, report)
        result = {'validated': report['validated'], 'plan': str(path), 'motion_sent': False, 'standard_staging_reached': report.get('standard_staging_reached', False), 'planning_wall_s': report['planning_wall_s'], 'error': report.get('error')}
        if plan_only or not report['validated']:
            return result
        validate_stage_plan(report)
        if self.state['phase'] == 'AT_CONTACT':
            sealed = self.pump('on')
            self._checkpoint('VACUUM_ON', sealed)
            if not sealed['confirmed']:
                raise RuntimeError('PRESSURE_NOT_CONFIRMED: no lift sent; pump output retained')
        prefetch = self._start_place_prefetch(path, report, case, placed)
        try:
            executed = self.execute_stage(report, execution, speed_scale, run)
        except BaseException:
            self._cancel_place_prefetch()
            self.persist()
            raise
        save(run / 'execution.json', executed)
        if not executed['completed']:
            self._cancel_place_prefetch()
            transition(self.state, 'FAULT', last_failure=executed.get('error'))
            self.persist()
            raise RuntimeError(executed.get('error', 'Lift failed; vacuum retained'))
        transition(self.state, 'LIFTED', lift_q=executed['final_q'], lift_plan=str(path), lift_plan_sha256=file_hash(path), binding_panel_T_box=report['binding_panel_T_box'])
        self.state['items'][self.state['selected_id']]['status'] = 'CARRIED'
        self.persist()
        return dict(result, phase='LIFTED', motion_sent=True, execution=str(run / 'execution.json'), id=self.state['selected_id'], next='place', place_prefetch=prefetch, speed_profiles=executed.get('speed_profiles'))

    def pick(self, plan_only=False, batch_size=1, continue_batch=False, resume=False, max_view_moves=3, budget=15.0, execution='point', speed_scale=3.0, ik_backend='native-sdk', target='auto', release_gap_m=None):
        validate_speed_scale(speed_scale)
        release_gap_m = checked_release_gap(release_gap_m)
        if target not in ('auto', 'leftmost', 'rightmost'):
            raise ValueError('A fresh pick accepts auto/leftmost/rightmost from the new image, not historical case IDs')
        if not 0 <= max_view_moves <= 6:
            raise ValueError('max-view-moves must be 0..6')
        if not 1 <= budget <= 120:
            raise ValueError('Planning budget must be 1..120 seconds')
        if resume:
            if release_gap_m is not None and release_gap_m != self.state.get('planned_release_gap_m'):
                raise ValueError('Resumed pick cannot change its anticipated release policy')
            if self.state['phase'] == 'LIFTED':
                return {'phase': 'LIFTED', 'motion_sent': False, 'already_lifted': True, 'next': 'place', 'id': self.state['selected_id']}
            if self.state['phase'] not in ('OBSERVED', 'AT_CONTACT', 'SEALED') or not self.state.get('cycle'):
                raise RuntimeError('No resumable observation/contact checkpoint; inspect status')
            (live, payload) = self._pick_payload_check(plan_only)
            if self.state['phase'] == 'OBSERVED':
                if self.state['cycle'].get('completed') or any(live['io'].values()):
                    raise RuntimeError('Observation resume requires an incomplete cycle and an empty tool with pump OFF')
                self.state.pop('selected_luggage', None)
                self.state['selected_id'] = None
                self.persist()
                contact = self.step1(target, plan_only, True, max_view_moves, execution, speed_scale, ik_backend)
                self._checkpoint('ALIGN_CONTACT', contact)
                if plan_only:
                    return dict(contact, pick_complete=False, payload=payload, preview_scope='fresh selected geometry or next view only; no seal or lift execution')
        else:
            if self.state['phase'] not in ('IDLE', 'HOME', 'OBSERVED'):
                raise RuntimeError('Active contact/load/fault is not a fresh pick; inspect status, then use --resume only for this cycle')
            (live, payload) = self._pick_payload_check(plan_only)
            if any(live['io'].values()):
                raise RuntimeError('A fresh pick requires an empty tool, pump OFF and release output OFF')
            began = self.begin_cycle(batch_size, continue_batch)
            if release_gap_m is not None:
                self.state['planned_release_gap_m'] = release_gap_m
                self.persist()
            self._checkpoint('NEW_CYCLE', began)
            initialized = self.initialize(plan_only, execution, speed_scale)
            self._checkpoint('INITIALIZE_OBSERVATION', initialized)
            if not initialized.get('completed'):
                return dict(initialized, phase=self.state['phase'], pick_complete=False, motion_sent=False, payload=payload, preview_scope='next initialization move only; no future observations assumed', next='pick after reviewing initialization preview')
            contact = self.step1(target, plan_only, True, max_view_moves, execution, speed_scale, ik_backend)
            self._checkpoint('ALIGN_CONTACT', contact)
            if plan_only:
                return dict(contact, pick_complete=False, payload=payload, preview_scope='next view adjustment or contact only; lift and seal are not yet verified', next='pick; actual contact and attachment are required before lift')
        result = self.lift(plan_only, budget, execution, speed_scale, ik_backend)
        self._checkpoint('LIFT_AND_STAGE', result)
        if not result['validated'] and (not plan_only):
            raise RuntimeError('No validated lift; staying at contact with no implicit retry: ' + str(result['plan']))
        return dict(result, pick_complete=result.get('phase') == 'LIFTED', payload=payload, occupancy_policy=self.state.get('occupancy_policy'), batch=self.state.get('batch'))

    def _plan_place(self, budget, ik_backend, release_gap_m=None):
        if not 1 <= budget <= 120:
            raise ValueError('Planning budget must be 1..120 seconds')
        release_gap_m = checked_release_gap(release_gap_m if release_gap_m is not None else self.state.get('planned_release_gap_m'))
        if release_gap_m is not None:
            self.state['planned_release_gap_m'] = release_gap_m
            self.persist()
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if not (live['io']['DI0'] == 1 and live['io']['DO0'] == 1 and (live['io']['DO1'] == 0)):
            raise RuntimeError('VACUUM_NOT_CONFIRMED: place retains the attached box')
        if np.max(np.abs(np.asarray(live['q']) - self.state['lift_q'])) > np.deg2rad(0.05):
            raise RuntimeError('Lift endpoint changed; revalidation is required')
        if file_hash(self.state['lift_plan']) != self.state.get('lift_plan_sha256'):
            raise RuntimeError('Lift attachment certificate changed')
        (source, case, placed) = self._attachment_inputs()
        context = self._place_context(case, placed, self.state['lift_q'], self.state['lift_plan'], ik_backend)
        if release_gap_m is not None:
            context['operator_authorized_floor_release_gap_m'] = float(release_gap_m)
        cached = self.state.get('place_candidate', {})
        report = None
        if cached.get('context') == context and Path(cached.get('path', '')).is_file():
            if file_hash(cached['path']) == cached.get('sha256'):
                candidate = json.loads(Path(cached['path']).read_text())
                if candidate.get('validated') and (not candidate.get('preview_only')):
                    report = candidate
        prefetch_audit = None
        metadata = self.state.get('place_prefetch')
        if report is None and metadata is not None:
            from manual_place_prefetch import checked_result
            try:
                report = checked_result(metadata, context, live['q'], self.state['binding_panel_T_box'])
                prefetch_audit = {'used': True, 'background_plan_ready': True}
            except (ValueError, KeyError, OSError) as exc:
                prefetch_audit = {'used': False, 'reason': str(exc), 'fresh_planning_required': True}
        reused = report is not None
        self._cancel_place_prefetch(include_early=False)
        run = self.run_dir('place')
        path = run / 'plan.json'
        if report is None:
            started = time.monotonic()
            with self._attachment_planner(case, source, self.state['contact_q'], live['q'], ik_backend) as (planner, audit):
                if release_gap_m is not None:
                    planner.release_gap_m = float(release_gap_m)
                    planner.allow_floor_drop = True
                report = planner.plan(live['q'], self.state['cycle_home_q'], placed, budget_s=budget, include_lift=False)
            report.update(native_ik_audit=audit, planning_wall_s=time.monotonic() - started, scene_file_sha256=context['scene'], robot_model_sha256=context['model'], contact_plan_sha256=context['contact'], ik_backend=ik_backend)
        if not np.allclose(report['binding_panel_T_box'], self.state['binding_panel_T_box'], atol=1e-09, rtol=0.0):
            raise RuntimeError('Place changed the measured grasp offset')
        if prefetch_audit is not None:
            report['prefetch_adoption'] = prefetch_audit
        save(path, report)
        if report.get('validated') and (not report.get('preview_only')):
            self._cancel_place_prefetch()
            self.state['place_candidate'] = {'context': context, 'path': str(path), 'sha256': file_hash(path)}
        self.persist()
        return (report, path, run, reused)

    def place(self, plan_only=False, budget=10.0, resume=False, execution='point', speed_scale=3.0, ik_backend='native-sdk', confirm_release=None, release_policy='manual', release_gap_m=None):
        validate_speed_scale(speed_scale)
        if release_policy not in ('manual', 'fixed-floor', 'fixed-support'):
            raise ValueError('Release policy must be manual, fixed-floor or fixed-support')
        release_gap_m = checked_release_gap(release_gap_m if release_gap_m is not None else self.state.get('planned_release_gap_m'))
        phase = self.state['phase']
        if phase not in ('LIFTED', 'AT_PLACE', 'RELEASED'):
            raise RuntimeError('Place requires this cycle LIFTED state; run pick first')
        if not self.state.get('cycle'):
            raise RuntimeError('Two-step place cannot adopt an unrelated historical carrying state')
        if phase in ('AT_PLACE', 'RELEASED') and (not resume) and (not plan_only):
            raise RuntimeError('Place already reached a release/retreat checkpoint; use place --resume')
        if phase == 'LIFTED':
            (report, path, run, reused) = self._plan_place(budget, ik_backend, release_gap_m)
            result = {'motion_sent': False, 'validated': report.get('validated', False), 'plan': str(path), 'reused_plan': reused, 'error': report.get('error'), 'planning_wall_s': report.get('planning_wall_s'), 'prefetch': report.get('prefetch'), 'prefetch_adoption': report.get('prefetch_adoption')}
            self._checkpoint('PLACE_AND_RETREAT_PLAN', result)
            if plan_only:
                return dict(result, release_confirmation_required=True)
            if not report.get('validated') or report.get('preview_only'):
                raise RuntimeError('No independently validated place AND empty return; vacuum retained: ' + str(path))
            executed = self.execute_stage(report, execution, speed_scale, run)
            save(run / 'execution.json', executed)
            if not executed['completed']:
                transition(self.state, 'FAULT', last_failure=executed.get('error'))
                self.persist()
                raise RuntimeError(executed.get('error', 'Place stopped; vacuum retained'))
            transition(self.state, 'AT_PLACE', place_q=executed['final_q'], transfer_plan=str(path), transfer_plan_sha256=file_hash(path))
            self.state['items'][self.state['selected_id']]['status'] = 'AT_PLACE'
            self.persist()
            self._checkpoint('AT_PLACE', {'motion_sent': True, 'execution': str(run / 'execution.json')})
        if file_hash(self.state['transfer_plan']) != self.state.get('transfer_plan_sha256'):
            raise RuntimeError('Place/retreat certificate changed; no release or return sent')
        self._attachment_inputs()
        report = json.loads(Path(self.state['transfer_plan']).read_text())
        if report.get('validated') is not True or report.get('preview_only'):
            raise RuntimeError('Accepted place/retreat certificate is required')
        if self.state['phase'] == 'AT_PLACE':
            context = release_context(self.state, report)
            if plan_only:
                return dict(context, motion_sent=False, outputs_written=False)
            with self.hardware() as hw:
                live = hw.snapshot()
                hw.ready(live)
            if np.max(np.abs(np.asarray(live['q']) - self.state['place_q'])) > np.deg2rad(0.05) or live['io'] != {'DI0': 1, 'DO0': 1, 'DO1': 0}:
                raise RuntimeError('Place release checkpoint changed; no pump command sent')
            if release_policy in ('fixed-floor', 'fixed-support'):
                scene = json.loads((self.root / 'config/manual_cell.json').read_text())
                placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
                evidence = fixed_floor_release_evidence(scene, report) if release_policy == 'fixed-floor' else fixed_support_release_evidence(scene, report, placed)
                confirmation = {'source': evidence['source'], 'time_s': time.time(), 'context': context, 'evidence': evidence}
            else:
                if confirm_release is None or not confirm_release(context):
                    return dict(context, place_complete=False, motion_sent=False, vacuum_retained=True, next='place --resume; release confirmation is still required')
                confirmation = {'source': 'operator_at_place', 'time_s': time.time(), 'context': context}
            self.state['release_confirmation'] = confirmation
            self.persist()
            released = self.pump('off')
            self._checkpoint('VACUUM_OFF', released)
            if not released['confirmed']:
                raise RuntimeError('Pressure release not confirmed; retreat not sent')
        returned = self.step3(plan_only, False, execution, speed_scale)
        self._checkpoint('RETURN_OBSERVATION', returned)
        if not plan_only and self.state['phase'] == 'HOME':
            self.state['cycle']['completed'] = True
            self.state['cycle']['completed_s'] = time.time()
            self.state['batch']['completed'] += 1
            if self.state['batch']['completed'] >= self.state['batch']['size']:
                self.state['batch']['active'] = False
            self.persist()
            return dict(returned, place_complete=True, batch=self.state['batch'], next='pick --continue-batch' if self.state['batch']['active'] else 'pick (new empty-ULD test)')
        return dict(returned, place_complete=False)

    def pump(self, action, seconds=3.0):
        if action == 'status':
            with self.hardware() as hw:
                return hw.snapshot()
        if action == 'sample':
            if not 0 < seconds <= 60:
                raise ValueError('Sample duration must be 0..60 seconds')
            rows = []
            with self.hardware() as hw:
                start = time.monotonic()
                while time.monotonic() - start < seconds:
                    io = hw.io()
                    row = {'time_s': time.time(), **io, 'force_raw': hw.force()}
                    rows.append(row)
                    hw.event('manual_pump_sample', **{k: v for (k, v) in row.items() if k != 'time_s'})
                    print(json.dumps(row), flush=True)
                    time.sleep(0.1)
            return {'samples': len(rows), 'log': str(self.events), 'writes_sent': False}
        phase = self.state['phase']
        if action == 'on' and phase not in ('AT_CONTACT', 'SEALED'):
            raise RuntimeError('Pump ON requires the selected box contact stage')
        if action == 'off' and phase not in ('AT_CONTACT', 'SEALED', 'AT_PLACE', 'RELEASED', 'HOME', 'IDLE', 'OBSERVED'):
            raise RuntimeError('Pump release is not allowed during an unknown/failed loaded stage')
        support_update = {}
        if action == 'off' and phase == 'AT_PLACE':
            placed_plan = json.loads(Path(self.state['transfer_plan']).read_text())
            support_update = settled_support_geometry(self.state['items'][self.state['selected_id']]['box'], placed_plan)
        with self.hardware() as hw:
            current = hw.snapshot()
            hw.ready(current)
            if action == 'off' and phase not in ('AT_CONTACT', 'SEALED', 'AT_PLACE', 'RELEASED') and current['io']['DI0']:
                raise RuntimeError('Cannot release an unidentified retained load without a known support/contact stage')
            expected = self.state.get('place_q') if phase == 'AT_PLACE' else self.state.get('contact_q')
            if phase in ('AT_CONTACT', 'SEALED', 'AT_PLACE') and (expected is None or np.max(np.abs(np.asarray(current['q']) - expected)) > np.deg2rad(0.05)):
                raise RuntimeError('Robot is not at the recorded support/contact pose')
            result = hw.pump(action == 'on')
        if result['confirmed']:
            if action == 'on' and phase == 'AT_CONTACT':
                transition(self.state, 'SEALED')
            elif action == 'off' and phase == 'SEALED':
                transition(self.state, 'AT_CONTACT')
            elif action == 'off' and phase == 'AT_PLACE':
                transition(self.state, 'RELEASED')
                plan = json.loads(Path(self.state['transfer_plan']).read_text())
                item = self.state['items'][self.state['selected_id']]
                item.update(status='PLACED_UNVERIFIED', placement_measured=False)
                item['box']['pose'] = plan['settled_box_pose']
                item['box']['center_m'] = plan['settled_box_pose']['xyz']
                item['box']['size_m'] = plan.get('placed_collision_size_m', item['box']['size_m'])
                item['box']['collision_pose'] = plan['settled_box_pose']
                item['box']['collision_size_m'] = item['box']['size_m']
                item['box'].update(support_update)
        elif action == 'on' and phase == 'SEALED':
            transition(self.state, 'AT_CONTACT')
        self.state['vacuum_verification'] = result
        self.persist()
        return {**result, 'phase': self.state['phase'], 'log': str(self.events)}

    def step2(self, plan_only=False, preview_scene=False, budget=30.0, execution='point', speed_scale=3.0):
        from manual_motion import Planner
        if self.state['phase'] not in ('AT_CONTACT', 'SEALED'):
            raise RuntimeError('Step 2 requires step 1 contact')
        if not 1 <= budget <= 120:
            raise ValueError('Planning budget must be 1..120 seconds')
        scene = json.loads((self.root / 'config/manual_cell.json').read_text())
        if not (plan_only and preview_scene):
            require_scene(scene)
        if not plan_only and self.state['phase'] != 'SEALED':
            raise RuntimeError('Run pump on and obtain pressure confirmation first')
        with self.hardware() as hw:
            live = hw.snapshot()
            hw.ready(live)
        if not plan_only and (live['io']['DI0'] != 1 or live['io']['DO0'] != 1):
            raise RuntimeError('VACUUM_NOT_CONFIRMED: no lift command issued')
        if np.max(np.abs(np.asarray(live['q']) - self.state['contact_q'])) > np.deg2rad(0.05):
            raise RuntimeError('Contact pose changed; attachment offset must be re-established')
        case = self.state['items'][self.state['selected_id']]['box']
        source = json.loads(Path(self.state['contact_plan']).read_text())
        if (case.get('owned_scene_components') or case.get('ignored_nonrigid_components')) and case.get('ownership_scene_hash') != file_hash(self.state['contact_plan']):
            raise RuntimeError('Object-part annotation belongs to a different observation')
        xml = Path(self.state['urdf']).read_text()
        placed = [dict(item['box'], id=key) for (key, item) in self.state['items'].items() if item['status'].startswith('PLACED')]
        context = {'scene': file_hash(self.root / 'config/manual_cell.json'), 'model': file_hash(self.state['urdf']), 'contact_plan': file_hash(self.state['contact_plan']), 'case': case, 'placed': placed, 'home_q': self.state['home_q'], 'selected_id': self.state['selected_id']}
        cached = self.state.get('transfer_candidate', {})
        report = None
        if cached.get('context') == context and Path(cached.get('path', '')).is_file():
            if file_hash(cached['path']) == cached.get('sha256'):
                candidate = json.loads(Path(cached['path']).read_text())
                if candidate.get('validated') and (not candidate.get('preview_only')) and (np.max(np.abs(np.asarray(candidate['start_q']) - live['q'])) <= np.deg2rad(0.05)):
                    report = candidate
        reused = report is not None
        if report is None:
            planner = Planner(xml, scene, source['scene'], case, live['q'], allow_preview=plan_only and preview_scene)
            try:
                report = planner.plan(live['q'], self.state['home_q'], placed, budget_s=budget)
            finally:
                planner.close()
        report['scene_file_sha256'] = file_hash(self.root / 'config/manual_cell.json')
        report['robot_model_sha256'] = file_hash(self.state['urdf'])
        run = self.run_dir('transfer')
        plan_path = run / 'plan.json'
        save(plan_path, report)
        if report.get('validated') and (not report.get('preview_only')):
            self.state['transfer_candidate'] = {'context': context, 'path': str(plan_path), 'sha256': file_hash(plan_path)}
            self.persist()
        if plan_only:
            return {'motion_sent': False, 'validated': report['validated'], 'preview_only': report['preview_only'], 'plan': str(plan_path), 'error': report.get('error'), 'reused_plan': reused, **(self.preview_execution(report, execution, speed_scale, run) if report['validated'] and (not report['preview_only']) else {})}
        if not report['validated'] or report['preview_only']:
            raise RuntimeError('No validated real-cell full cycle: ' + str(plan_path))
        executed = self.execute_stage(report, execution, speed_scale, run)
        save(run / 'execution.json', executed)
        if not executed['completed']:
            transition(self.state, 'FAULT', last_failure=executed.get('error'))
            self.persist()
            raise RuntimeError(executed.get('error', 'Loaded stage failed; vacuum retained'))
        transition(self.state, 'AT_PLACE', place_q=executed['final_q'], transfer_plan=str(plan_path))
        self.state['items'][self.state['selected_id']]['status'] = 'AT_PLACE'
        self.persist()
        return {'phase': 'AT_PLACE', 'next': 'Visually check support, then pump off', 'plan': str(plan_path)}

    def step3(self, plan_only=False, from_contact=False, execution='point', speed_scale=3.0):
        if from_contact:
            if self.state['phase'] != 'AT_CONTACT':
                raise RuntimeError('Pick-only return requires pump off at the contact stage')
            if self.state.get('scan_serial', 0) != self.state.get('contact_scan_serial', 0):
                raise RuntimeError('Scene observations changed; revalidate the return before movement')
            if file_hash(self.state['urdf']) != self.state.get('contact_model_hash'):
                raise RuntimeError('Robot model changed since contact planning')
            source = json.loads(Path(self.state['contact_plan']).read_text())
            reverse = []
            from native_endpoint_commands import original_intervals
            for row in reversed(original_intervals(source['segments'])):
                phase = 'release_retreat' if row.get('phase', 'normal_contact') == 'normal_contact' else 'return_observe'
                reverse.append(dict(row, start_q=row['goal_q'], goal_q=row['start_q'], phase=phase))
            plan = {'validated': source.get('qualified_for_supervised_trial') is True, 'loaded': False, 'start_q': reverse[0]['start_q'], 'segments': reverse}
            run = self.run_dir('pick_only_return')
            if plan_only:
                return {'motion_sent': False, 'retreat': plan, **self.preview_execution(plan, execution, speed_scale, run)}
            executed = self.execute_stage(plan, execution, speed_scale, run)
            save(run / 'execution.json', executed)
            if not executed['completed']:
                self.state.update(phase='FAULT', last_failure=executed.get('error'))
                self.persist()
                raise RuntimeError(executed.get('error', 'Return stopped'))
            self.state['items'][self.state['selected_id']]['status'] = 'AVAILABLE'
            self.state.update(phase='HOME', selected_id=None, revision=self.state['revision'] + 1)
            self.state.pop('selected_luggage', None)
            self.persist()
            return {'phase': 'HOME', 'pick_only': True, 'next': 'scan or step1 --target auto'}
        if self.state['phase'] != 'RELEASED':
            raise RuntimeError('Step 3 requires pump OFF and pressure release confirmation')
        plan = json.loads(Path(self.state['transfer_plan']).read_text())
        if file_hash(self.root / 'config/manual_cell.json') != plan['scene_file_sha256']:
            raise RuntimeError('Site scene changed since transfer planning')
        if file_hash(self.state['urdf']) != plan['robot_model_sha256']:
            raise RuntimeError('Robot model changed since transfer planning')
        run = self.run_dir('retreat')
        if plan_only:
            return {'motion_sent': False, 'retreat': plan['retreat'], **self.preview_execution(plan['retreat'], execution, speed_scale, run)}
        executed = self.execute_stage(plan['retreat'], execution, speed_scale, run)
        save(run / 'execution.json', executed)
        if not executed['completed']:
            transition(self.state, 'FAULT', last_failure=executed.get('error'))
            self.persist()
            raise RuntimeError(executed.get('error', 'Retreat failed'))
        transition(self.state, 'HOME', selected_id=None)
        self.state.pop('selected_luggage', None)
        self.persist()
        return {'phase': 'HOME', 'next': 'step1 --target auto', 'execution': str(run / 'execution.json')}

    def prepare_continuous(self, speed_scale=5.0):
        from continuous_reference import prepare
        cached = self.state.get('transfer_candidate', {})
        source = Path(cached.get('path', ''))
        if not source.is_file() or file_hash(source) != cached.get('sha256'):
            raise RuntimeError('A matching saved validated full-cycle plan is required')
        plan = json.loads(source.read_text())
        model = Path(self.state['urdf'])
        if file_hash(model) != plan.get('robot_model_sha256'):
            raise RuntimeError('Robot model differs from the source certificate')
        report = prepare(plan, model.read_text(), speed_scale, self.velocity_cap_deg_s)
        report.update(source_plan=str(source), source_sha256=file_hash(source), robot_model_sha256=file_hash(model), current_session_phase=self.state['phase'], current_state_revalidated=False, does_not_authorize_reset_or_motion=True)
        run = self.run_dir('continuous_reference')
        output = run / 'reference.json'
        save(output, report)
        return {'motion_sent': False, 'outputs_written': False, 'reference': str(output), 'hardware_execution_qualified': False, 'current_session_phase': self.state['phase'], 'joint_box_geometry_certificate_reused': report['joint_box_geometry_certificate_reused'], 'reference_duration_s': report['reference_duration_s'], 'source_native_interval_count': report['source_native_interval_count'], 'intended_full_stop_boundaries': report['intended_full_stop_boundaries'], 'blockers': report['blockers']}

    def parts(self, include=None, ignore_nonrigid=None, box_model=None):
        lifted = self.state['phase'] == 'LIFTED'
        if lifted and (include or ignore_nonrigid or (not box_model)):
            raise RuntimeError('After lift only an explicit box-model review is allowed; ownership cannot change')
        if self.state['phase'] not in ('AT_CONTACT', 'SEALED', 'LIFTED'):
            raise RuntimeError('Object-part review requires a selected contact case')
        source = json.loads(Path(self.state['contact_plan']).read_text())
        rows = [r for r in source['scene'] if r['name'].startswith('rgbd_component_')]
        image = None
        if self.state.get('contact_capture'):
            from manual_inventory import render_parts
            image = self.root / 'runs/manual/object_parts.png'
            render_parts(self.state['contact_capture'], source['scene'], image)
        if include:
            from manual_session import include_owned_parts
            item = self.state['items'][self.state['selected_id']]
            include_owned_parts(item['box'], rows, include)
            item['box'].update(owned_scene_components=list(include), ownership_scene_hash=file_hash(self.state['contact_plan']))
            self.persist()
        if ignore_nonrigid or box_model:
            item = self.state['items'][self.state['selected_id']]
            case = item['box']
            if ignore_nonrigid:
                if not set(ignore_nonrigid).issubset({r['name'] for r in rows}):
                    raise ValueError('Only observed components can be classified as nonrigid')
                case['ignored_nonrigid_components'] = list(ignore_nonrigid)
                case['owned_scene_components'] = [name for name in case.get('owned_scene_components', []) if name not in ignore_nonrigid]
                case['ownership_scene_hash'] = file_hash(self.state['contact_plan'])
                self.state.pop('transfer_candidate', None)
            if box_model:
                if not case.get('height_valid') or not case.get('top_surface_valid'):
                    raise ValueError('Surface box requires a valid top-plane and ground height estimate')
                dimensions = np.asarray(case['size_m'])
                if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
                    raise ValueError('Invalid fitted box dimensions')
                if lifted and box_model != case.get('geometry_model', 'visible_foreground_obb'):
                    from manual_motion import _planning_case, matrix
                    from manual_session import include_owned_parts
                    self._attachment_inputs()
                    previous = copy.deepcopy(case)
                    revised = dict(previous, geometry_model=box_model)
                    owned = previous.get('owned_scene_components', [])
                    old_geometry = _planning_case(include_owned_parts(previous, rows, owned) if owned else previous)
                    new_geometry = _planning_case(include_owned_parts(revised, rows, owned) if owned else revised)
                    binding = np.asarray(self.state['binding_panel_T_box'], dtype=float)
                    if binding.shape != (4, 4) or not np.isfinite(binding).all():
                        raise ValueError('A finite measured grasp binding is required')
                    updated = binding @ np.linalg.inv(matrix(old_geometry['pose'])) @ matrix(new_geometry['pose'])
                    review = self.run_dir('geometry_review')
                    save(review / 'previous_session.json', self.state)
                    save(review / 'review.json', {'selected_id': self.state['selected_id'], 'previous_box': previous, 'revised_box': revised, 'previous_binding_panel_T_box': binding.tolist(), 'binding_panel_T_box': updated.tolist(), 'motion_sent': False, 'actuator_outputs_written': False, 'physical_grasp_unchanged': True, 'authority': 'operator_confirmed_surface_matches_body_dimensions', 'requires_new_place_and_return_validation': True})
                    self._cancel_place_prefetch()
                    self.state['binding_panel_T_box'] = updated.tolist()
                    self.state['geometry_review'] = str(review / 'review.json')
                    self.state.pop('place_candidate', None)
                case['geometry_model'] = box_model
                case['geometry_model_authority'] = 'operator_confirmed_surface_matches_body_dimensions'
                self.state.pop('transfer_candidate', None)
                if self.state.get('selected_luggage'):
                    self.state['selected_luggage']['box'] = copy.deepcopy(case)
            self.persist()
        return {'selected_id': self.state['selected_id'], 'components': [{'id': r['name'], 'kind': r.get('kind', 'box')} for r in rows], 'owned': self.state['items'][self.state['selected_id']]['box'].get('owned_scene_components', []), 'ignored_nonrigid': self.state['items'][self.state['selected_id']]['box'].get('ignored_nonrigid_components', []), 'box_model': self.state['items'][self.state['selected_id']]['box'].get('geometry_model', 'visible_foreground_obb'), 'image': str(image) if image else None, 'source_plan': self.state['contact_plan'], 'motion_sent': False, 'note': 'Include only components visually confirmed to belong to this suitcase'}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    sub = parser.add_subparsers(dest='command', required=True)
    status = sub.add_parser('status')
    status.add_argument('--offline', action='store_true')
    sub.add_parser('scan')
    sub.add_parser('inventory')
    sub.add_parser('scene-info')
    sub.add_parser('execution-config')
    transport = sub.add_parser('servo-transport-check')
    transport.add_argument('--samples', type=int, default=20)
    preparation = sub.add_parser('prepare-stage')
    preparation.add_argument('--plan', type=Path, required=True)
    preparation.add_argument('--speed-scale', type=float, default=3.0)
    recovery = sub.add_parser('recover-empty')
    recovery.add_argument('--confirm-tool-empty', action='store_true')
    search = sub.add_parser('search-pose')
    search.add_argument('--record', action='store_true')
    observe = sub.add_parser('observe')
    observe.add_argument('--plan-only', action='store_true')
    initialization = sub.add_parser('initialize')
    initialization.add_argument('--plan-only', action='store_true')
    observe.add_argument('--distance', type=float, default=0.12)
    observe.add_argument('--search-index', type=int, choices=(1, 2, 3))
    first = sub.add_parser('step1')
    first.add_argument('--target', default='auto')
    first.add_argument('--plan-only', action='store_true')
    first.add_argument('--resume-contact', action='store_true')
    first.add_argument('--ik-backend', choices=('local', 'native-sdk'), default='local')
    first.add_argument('--no-auto-observe', action='store_true')
    first.add_argument('--max-view-moves', type=int, default=3)
    second = sub.add_parser('step2')
    second.add_argument('--plan-only', action='store_true')
    second.add_argument('--preview-scene', action='store_true')
    second.add_argument('--budget', type=float, default=30.0)
    third = sub.add_parser('step3')
    third.add_argument('--plan-only', action='store_true')
    third.add_argument('--from-contact', action='store_true')
    pick = sub.add_parser('pick')
    pick.add_argument('--plan-only', action='store_true')
    pick.add_argument('--describe', action='store_true')
    pick.add_argument('--target', choices=('auto', 'leftmost', 'rightmost'), default='auto')
    pick.add_argument('--batch-size', type=int, choices=(1, 3, 4), default=1)
    pick.add_argument('--continue-batch', action='store_true')
    pick.add_argument('--resume', action='store_true')
    pick.add_argument('--max-view-moves', type=int, default=3)
    pick.add_argument('--budget', type=float, default=15.0)
    pick.add_argument('--release-gap-m', type=float, help='Anticipated Place clearance for background planning; inherited by Place, not a Pick move.')
    place = sub.add_parser('place')
    place.add_argument('--plan-only', action='store_true')
    place.add_argument('--resume', action='store_true')
    place.add_argument('--hold-at-place', action='store_true')
    place.add_argument('--release-policy', choices=('manual', 'fixed-floor', 'fixed-support'), default='manual')
    place.add_argument('--release-gap-m', type=float, help='Explicit floor settling gap (0.012..0.10 m); >0.05 disallows stack release')
    place.add_argument('--budget', type=float, default=10.0)
    for command in (pick, place):
        command.add_argument('--ik-backend', choices=('native-sdk', 'local'), default='native-sdk')
    for command in (initialization, observe, first, second, third, pick, place):
        command.add_argument('--execution', choices=('point', 'point3', 'point5', 'endpoint', 'continuous', 'blend5'), default='point')
        command.add_argument('--speed-scale', type=float, default=3.0)
        command.add_argument('--free-space-velocity-deg-s', type=float, default=DEFAULT_FREE_SPACE_VELOCITY_DEG_S, help='Explicit free-space command cap (1..15 deg/s); overrides legacy speed-scale, not contact')
    pump = sub.add_parser('pump')
    pump.add_argument('action', choices=('on', 'off', 'status', 'sample'))
    pump.add_argument('--seconds', type=float, default=3.0)
    boot = sub.add_parser('adopt-contact')
    boot.add_argument('--contact-run', type=Path, required=True)
    boot.add_argument('--sensing-run', type=Path, required=True)
    scene = sub.add_parser('scene')
    scene.add_argument('--file', type=Path)
    scene.add_argument('--right-floor-corner', type=float, nargs=3, help='Measured XYZ in elfin_base_link of CAD usable-floor reference [-0.745,-1.04,0.53], not an arbitrary door-frame corner')
    scene.add_argument('--inward-yaw-deg', type=float)
    parts = sub.add_parser('parts')
    parts.add_argument('--include', nargs='+')
    parts.add_argument('--ignore-nonrigid', nargs='+')
    parts.add_argument('--box-model', choices=('surface_box', 'visible_foreground_obb'))
    continuous = sub.add_parser('prepare-continuous')
    continuous.add_argument('--speed-scale', type=float, default=5.0)
    args = parser.parse_args()
    if args.command == 'place' and args.hold_at_place and (args.release_policy != 'manual'):
        parser.error('--hold-at-place cannot be combined with automatic release')
    workflow = Workflow(args.root, velocity_cap_deg_s=getattr(args, 'free_space_velocity_deg_s', DEFAULT_FREE_SPACE_VELOCITY_DEG_S), execution_variant=getattr(args, 'execution', 'point'))
    lock = args.root / 'runs/manual/command.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    try:
        with os.fdopen(os.dup(int(os.environ['S20_BATCH_LOCK_FD'])), 'a') if 'S20_BATCH_LOCK_FD' in os.environ else lock.open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if args.command == 'status':
                result = workflow.status(args.offline)
            elif args.command == 'inventory':
                result = workflow.inventory()
            elif args.command == 'scene-info':
                result = workflow.scene_info()
            elif args.command == 'execution-config':
                result = workflow.execution_config()
            elif args.command == 'servo-transport-check':
                result = workflow.servo_transport_check(args.samples)
            elif args.command == 'initialize':
                result = workflow.initialize(args.plan_only, args.execution, args.speed_scale)
            elif args.command == 'prepare-stage':
                plan = json.loads(args.plan.read_text())
                (report, path) = workflow.compile_stage(plan, args.speed_scale)
                result = {'motion_sent': False, 'outputs_written': False, 'reference': str(path), 'continuous_reference_validated': report['joint_box_geometry_certificate_reused'], 'duration_s': report['reference_duration_s'], 'hardware_execution_qualified': False}
            elif args.command == 'recover-empty':
                result = workflow.recover_empty(args.confirm_tool_empty)
            elif args.command == 'search-pose':
                result = workflow.search_pose(args.record)
            elif args.command == 'observe':
                result = workflow.observe(args.plan_only, args.distance, search_index=args.search_index, execution=args.execution, speed_scale=args.speed_scale)
            elif args.command == 'scan':
                (scanned, run) = workflow.scan()
                from manual_inventory import capture_diagnostics
                diagnostics = capture_diagnostics(workflow.state['last_capture'], scanned)
                save(run / 'observation_diagnostics.json', diagnostics)
                result = {'items': [{'id': key, 'status': item['status'], 'visible': item.get('visible'), 'surface_size_m': item['box']['size_m'], 'outer_bound_m': item['box'].get('collision_size_m')} for (key, item) in workflow.state['items'].items()], 'image': scanned['inventory_image'], 'details': str(run / 'inventory.json'), 'visual_identity_association': scanned['visual_identity_association'], 'current_geometry_candidates': [{'provisional_index': row['provisional_index'], 'geometry_frames': row['geometry_frames'], 'complete_clear_frames': row['complete_clear_frames'], 'ready_for_stable_pick': row['ready_for_stable_pick'], 'surface_size_m': row['box']['size_m'], 'center_m': row['box']['center_m']} for row in diagnostics['candidates']], 'motion_sent': False, 'actuator_outputs_written': False}
            elif args.command == 'step1':
                if args.resume_contact and workflow.state['phase'] == 'IDLE':
                    if args.target not in ('auto', 'case-0001'):
                        raise ValueError('Recorded contact belongs to case-0001')
                    bootstrap = json.loads((args.root / 'config/manual_bootstrap.json').read_text())
                    result = workflow.bootstrap(args.root / bootstrap['contact_run'], args.root / bootstrap['sensing_run'])
                else:
                    result = workflow.step1(args.target, args.plan_only, not args.no_auto_observe, args.max_view_moves, execution=args.execution, speed_scale=args.speed_scale, ik_backend=args.ik_backend)
            elif args.command == 'step2':
                result = workflow.step2(args.plan_only, args.preview_scene, args.budget, execution=args.execution, speed_scale=args.speed_scale)
            elif args.command == 'step3':
                result = workflow.step3(args.plan_only, args.from_contact, execution=args.execution, speed_scale=args.speed_scale)
            elif args.command == 'pick':
                if args.resume and args.continue_batch:
                    raise ValueError('--resume and --continue-batch select different stages')
                if args.describe:
                    result = workflow.pick_sequence()
                else:
                    result = workflow.pick(args.plan_only, args.batch_size, args.continue_batch, args.resume, args.max_view_moves, args.budget, args.execution, args.speed_scale, args.ik_backend, args.target, args.release_gap_m)
            elif args.command == 'place':

                def confirmation(context):
                    print(json.dumps({'release_checkpoint': context}, indent=2), flush=True)
                    if not sys.stdin.isatty():
                        print('No operator terminal; vacuum retained. Use place --resume in a terminal.', flush=True)
                        return False
                    try:
                        return input('Confirm box support/safe short settling position, then type RELEASE to switch pump OFF and return: ').strip() == 'RELEASE'
                    except (EOFError, KeyboardInterrupt):
                        return False
                result = workflow.place(args.plan_only, args.budget, args.resume, args.execution, args.speed_scale, args.ik_backend, None if args.hold_at_place else confirmation, args.release_policy, args.release_gap_m)
            elif args.command == 'pump':
                result = workflow.pump(args.action, args.seconds)
            elif args.command == 'adopt-contact':
                result = workflow.bootstrap(args.contact_run, args.sensing_run)
            elif args.command == 'parts':
                result = workflow.parts(args.include, args.ignore_nonrigid, args.box_model)
            elif args.command == 'prepare-continuous':
                result = workflow.prepare_continuous(args.speed_scale)
            else:
                if args.file:
                    result = json.loads(args.file.read_text())
                    require_scene(result)
                else:
                    if args.right_floor_corner is None or args.inward_yaw_deg is None:
                        raise ValueError('Supply measured CAD usable-floor reference XYZ and inward yaw, or a confirmed scene JSON')
                    transform = np.eye(4)
                    transform[:3, :3] = Rotation.from_euler('z', args.inward_yaw_deg, degrees=True).as_matrix()
                    transform[:3, 3] = np.asarray(args.right_floor_corner) - transform[:3, :3] @ [-0.745, -1.04, 0.53]
                    result = {'schema': 's20_manual_cell/v1', 'measured': True, 'base_T_ULD': transform.tolist(), 'parent_frame': 'elfin_base_link', 'child_frame': 'container_link', 'units': 'm', 'source': 'operator-supplied measured CAD usable-floor reference [-0.745,-1.04,0.53] and inward direction'}
                    require_scene(result)
                save(args.root / 'config/manual_cell.json', result)
            print(json.dumps(result, indent=2, ensure_ascii=False))
            if args.command == 'pump' and result.get('confirmed') is False:
                return 3
            if args.command == 'step2' and result.get('validated') is False:
                return 3
            if args.command == 'observe' and result.get('validated') is False:
                return 3
            if args.command == 'initialize' and result.get('validated') is False:
                return 3
            if args.command in ('pick', 'place') and result.get('validated') is False:
                return 3
            if args.command == 'place' and (not args.plan_only) and (result.get('place_complete') is False):
                return 3
            if result.get('continuous_reference_validated') is False:
                return 3
    except ObservationRequired as exc:
        print(json.dumps({'ok': False, 'error': str(exc), 'motion_sent': False, 'motion_not_implicitly_retried': True, **exc.details}, ensure_ascii=False), file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.SubprocessError) as exc:
        print(json.dumps({'ok': False, 'error': str(exc), 'motion_not_implicitly_retried': True}, ensure_ascii=False), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('Interrupted. Read status before restarting a stage.', file=sys.stderr)
        return 130
    return 0
from original_async_metrics import install_async_metrics
install_async_metrics(globals())
if __name__ == '__main__':
    raise SystemExit(main())
