"""Package portable geometry sources only; verify the saved JSONL requests on CPU."""
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
EXP = Path('experiments/s20_online_mixed8_20261024')
FILES = [
    EXP/'deployment/planning_candidates.py',
    EXP/'deployment/scene_tf.yaml',
    EXP/'deployment/contact_v2_offline_requests.json',
    EXP/'compare.py',
    EXP/'compact_prior.py',
    EXP/'push_compare/support_geometry_v2.py',
    EXP/'straight_place/contact_layout.py',
    EXP/'straight_place/sparse_prior.py',
    EXP/'straight_place/optimized_cycle_place.py',
    EXP/'four20_sparse_cycle.py',
    EXP/'four20_contact_delivery_attempt1/result.json',
    EXP/'four20_contact_delivery_attempt2/result.json',
    Path('reproductions/ludan_greedy_20261023/policy/online_packing_real.py'),
    Path('reproductions/ludan_greedy_20261023/policy/online_packing_demo.py'),
]


def import_paths(root, environment):
    probe = """import json, runpy, sys
from pathlib import Path
root=Path(sys.argv[1]).resolve()
runpy.run_path(str(root/'experiments/s20_online_mixed8_20261024/deployment/planning_candidates.py'),run_name='bundle_probe')
names=['compare','compact_prior','straight_place.contact_layout','straight_place.sparse_prior','push_compare.support_geometry_v2','online_packing_real','online_packing_demo']
print(json.dumps({name:str(Path(sys.modules[name].__file__).resolve().relative_to(root)) if name in sys.modules else None for name in names}))
"""
    return json.loads(subprocess.check_output([sys.executable, '-c', probe, str(root)],
        text=True, cwd=root, env=environment))


def package_sources(archive, manifest):
    with tarfile.open(archive, 'w:gz') as package:
        for path in FILES:
            package.add(ROOT/path, arcname=str(path), recursive=False)
        data = json.dumps(manifest, indent=2).encode()
        info = tarfile.TarInfo('bundle_manifest.json')
        info.size = len(data)
        package.addfile(info, io.BytesIO(data))


def main():
    output = HERE/'candidate_bundle'
    output.mkdir(exist_ok=True)
    manifest = dict(
        scope='ARM-neutral Python source; geometric candidate service only',
        runtime_dependencies=['numpy', 'scipy', 'PyYAML'],
        optional_dependency={'opencv-python': 'lazy cv2 in compare.top_surface_model only; not used by candidate service'},
        python='3.10+; target ARM packages must be installed on target, no x86 wheels bundled',
        command='python3 experiments/s20_online_mixed8_20261024/deployment/planning_candidates.py',
        input='JSONL stdin: request_id, scene_version, observed_dimensions_m, committed_occupancy[{id,dims,pos}]',
        coordinates='occupancy pos is lower corner in packing-local ULD coordinates, not world center; output box_uld uses same convention',
        outputs='JSONL stdout; proposals only, no occupancy mutation or trajectory qualification',
        excluded=['SDK', 'Newton binaries', 'Warp binaries', 'torch', 'hardware runner', 'controller operations', 'binary dependencies'],
        sources=[str(path) for path in FILES],
        missing_glue=['observed fit and immutable measured-contact attachment',
                      'ContactV2 target injection into loaded/empty trajectory planner',
                      'actual entry q and scene-version adoption check',
                      'accepted stage path/reference and tracking proof for ServoHardware',
                      'pressure-confirmed release and observed occupancy reconciliation'],
        full_hardware_cycle_ready=False,
        simulation_reference_sources=['straight_place/optimized_cycle_place.py', 'four20_sparse_cycle.py'],
        simulation_reference_warning='These two source snapshots require the original simulation dependencies; not part of the portable candidate service dependency closure',
        endpoint_sphere_policy='Geometry service disables sphere screening. Arm Contact prior explicitly enables original 1.7m sphere; sphere inclusion is not IK or path validation.',
        contact_arm_results=[dict(run='four20_contact_delivery_attempt1', completed=False,
            completed_cycles=0, reason='First box: 16 endpoint IK rejections; default sphere screening disabled'),
            dict(run='four20_contact_delivery_attempt2', completed=True, completed_cycles=4,
                 wall_s=38.43583560900879, scope='Simulation only; explicit original endpoint sphere enabled')],
        current_loaded_planner='Runtime-dependent and uniform20 hardcoded; not included or deployable as mixed-observation planner',
        reusable_runner='tools/three_box_original/native_servoj_executor.py:ServoHardware.execute',
        reusable_compile='tools/three_box_original/native_servo_stage.py:prepare_trial',
        reusable_orchestration='tools/three_box_original/manual_pick_place.py:Workflow.lift/_plan_place/place/pump',
        scene_policy='Bundled exact deployment/scene_tf.yaml; do not substitute old manual_cell.json or compare.main scene defaults',
        verification_platform='local CPU only; not ARM execution or hardware acceptance')
    manifest['packaging_correction'] = dict(
        cause='Missing experiment-local compare.py and compact_prior.py silently selected old sibling implementations',
        semantic_difference='Old compact prior allowed 5cm height and edge expansion; current prior uses exact same-height >=80% footprint support plus supported center',
        float_normalization_used=False, algorithm_changed=False,
        bad_archive='contact_v2_source_bad_attempt1.tar.gz',
        previous_exact_matches=22,
        previous_rank1_mismatches=['seed74-item8', 'seed75-item7'])
    result = json.loads((ROOT/EXP/'four20_contact_delivery_attempt2/result.json').read_text())
    manifest['contact_arm_results'][-1].update(
        completed=result['completed'], completed_cycles=result['completed_cycles'],
        wall_s=result['wall_s'], timing_basis='unpaced simulation compute wall time, not physical execution duration',
        async_planning=[{key: row[key] for key in (
            'cycle', 'worker_compute_s', 'entry_wait_s', 'consumed_background_plan',
            'actual_entry_replan_and_execution_wall_s') if key in row}
            for row in result['async_planning']])
    archive = output/'contact_v2_source.tar.gz'
    package_sources(archive, manifest)
    with tempfile.TemporaryDirectory(dir=output, prefix='cpu_verify_') as temporary:
        with tarfile.open(archive) as package:
            package.extractall(temporary, filter='data')
        root = Path(temporary).resolve()
        evidence = json.loads((root/EXP/'deployment/contact_v2_offline_requests.json').read_text())
        queries = evidence['queries']
        environment = dict(os.environ, OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
        environment.pop('S20_PACKING_UPSTREAM', None)
        environment.pop('PYTHONPATH', None)
        manifest['import_paths'] = dict(native=import_paths(ROOT, environment),
                                      isolated=import_paths(root, environment))
        bad = output/'contact_v2_source_bad_attempt1.tar.gz'
        if bad.exists():
            with tempfile.TemporaryDirectory(dir=output, prefix='bad_import_probe_') as old:
                old = Path(old).resolve()
                with tarfile.open(bad) as package:
                    package.extractall(old, filter='data')
                manifest['import_paths']['bad_attempt1'] = import_paths(old, environment)
        run = subprocess.run([sys.executable, str(root/EXP/'deployment/planning_candidates.py')],
            input=''.join(json.dumps(row['request'])+'\n' for row in queries),
            text=True, capture_output=True, env=environment, cwd=root, timeout=90)
        responses = [json.loads(line) for line in run.stdout.splitlines()]
        matched = sum(bool(response.get('candidates')) and
            response['candidates'][0]['box_uld'] == row['response']['candidates'][0]['box_uld']
            for row, response in zip(queries, responses))
        manifest['verification'] = dict(returncode=run.returncode, requests=len(queries),
            responses=len(responses), exact_first_candidate_matches=matched, stderr=run.stderr,
            exact_full_ranked_candidate_matches=sum(
                [candidate['box_uld'] for candidate in response.get('candidates', [])] ==
                [candidate['box_uld'] for candidate in row['response']['candidates']]
                for row, response in zip(queries, responses)),
            mismatches=[dict(request_id=row['request']['request_id'],
                expected=row['response']['candidates'][0]['box_uld'],
                actual=response.get('candidates', [{}])[0].get('box_uld'))
                for row, response in zip(queries, responses)
                if response.get('candidates', [{}])[0].get('box_uld') != row['response']['candidates'][0]['box_uld']],
            passed=run.returncode == 0 and len(responses) == matched == len(queries))
    (output/'bundle_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    package_sources(archive, manifest)
    print(json.dumps(dict(archive=str(archive), **manifest['verification'])))
    return 0 if manifest['verification']['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
