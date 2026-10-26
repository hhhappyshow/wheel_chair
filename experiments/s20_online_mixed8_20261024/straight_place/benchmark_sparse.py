"""Fixed recorded-prefix comparisons of isolated sparse-placement trials.

No sensing, hardware or new arrival generation. Simulator truth reconstructs
the saved simulation boundary only; it is not a hardware/perception result.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import importlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

EXP=Path(__file__).resolve().parent.parent
ROOT=EXP.parent.parent


def load_cases(folder):
    report=json.loads((folder/'result.json').read_text())
    trace=np.load(folder/'trajectory.npz')
    labels=trace['body_labels'].tolist();stages=trace['stage'].astype(str)
    tcp=labels.index('S20/suction_contact_frame')
    vacuums=np.flatnonzero(stages=='IDEAL_VACUUM_ON')
    placements=report['placements'];cases=[]
    for number,vacuum in enumerate(vacuums):
        label=f'B{number+1}';actor=labels.index(label)
        end=int(vacuums[number+1]) if number+1<len(vacuums) else len(stages)
        release=next(i for i in range(int(vacuum),end) if stages[i]==label+'_RELEASE_ACTUAL_FROZEN')
        entries=np.flatnonzero((stages=='OPERATOR_TAUGHT_CONTAINER_ENTRY') &
                               (np.arange(len(stages))>vacuum) & (np.arange(len(stages))<release))
        if not len(entries):raise ValueError(f'{label}: no recorded held-entry boundary')
        entry=int(entries[-1]);poses=trace['body_q'][entry].copy()
        view=[v for v in report['views'] if v.get('cycle')==number+1 and v['trace_index']<=vacuum][-1]
        rotation=R.from_quat(poses[tcp,3:])
        prefix=[deepcopy(p['occupied']) for p in placements[:number]]
        if {p['id'] for p in prefix}!={f'B{i+1}' for i in range(number)}:
            raise ValueError(f'{label}: recorded occupancy prefix IDs differ')
        cases.append(dict(cycle=number+1,label=label,entry_index=entry,vacuum_index=int(vacuum),
            release_index=release,q=trace['joint_q'][entry].copy(),poses=poses,
            contact_tcp=trace['body_q'][vacuum,tcp].copy(),fitted=deepcopy(view['fitted']),
            occupied=prefix,offset=rotation.inv().apply(poses[actor,:3]-poses[tcp,:3]),
            rotation=(rotation.inv()*R.from_quat(poses[actor,3:])).as_quat(),
            observed_view_index=view.get('view_index',report['views'].index(view))))
    return report,trace,cases


def summary(case):
    return {k:case[k] for k in ('cycle','label','entry_index','vacuum_index','release_index','observed_view_index')} | dict(
        committed_prefix=len(case['occupied']),q_rad=case['q'].tolist(),
        held_offset_tcp_m=case['offset'].tolist(),held_rotation_tcp_xyzw=case['rotation'].tolist())


def restore(r,case):
    held=r.model.body_label.index(case['label'])
    r.history.clear();r.qs.clear();r.stages.clear();r.simtime.clear();r.events.clear();r.t=0.
    r.fixed={b:case['poses'][b].copy() for b in r.boxes}
    r.w.held=held;r.w.grip_offset=case['offset'].copy();r.w.grip_rot=R.from_quat(case['rotation'])
    r.collision.arrived=set(range(case['cycle']));r.collision.current_source=case['cycle']-1
    r.frame(case['q'],'BENCHMARK_RESTORED_ACTUAL_HELD_ENTRY')
    r.state.body_q.assign(case['poses']);r.history[-1]=case['poses'].copy()
    cache=getattr(r.ik,'cache',None)
    if hasattr(cache,'clear'):cache.clear()
    return held


def compare_results(paths):
    rows=[];datasets=[];baseline={}
    for dataset_index,path in enumerate(paths):
        path=path/'benchmark.json' if path.is_dir() else path
        data=json.loads(path.read_text());coverage=[]
        for trial_index,t in enumerate(data['trials']):
            key=(t['cycle'],t['order'],t.get('repeat',0))
            options=t.get('options',{});placement=t.get('placement') or {}
            backend=options.get('endpoint_backend','scalar')
            row=dict(dataset=str(path.resolve()),source=data.get('source'),cycle=key[0],order=key[1],
                repeat=key[2],backend=backend,success=t['success'],error=t.get('error'),
                total_wall_s=t['total_wall_s'],planning_to_commit_s=t.get('planning_to_commit_s'),
                first_trial_in_process=t.get('first_trial_in_process',trial_index==0),
                first_trial_label_source='recorded' if 'first_trial_in_process' in t else 'inferred from trial order',
                optimizer=t.get('optimizer'),endpoint_batch_stats=t.get('endpoint_batch_stats'),
                nominal=placement.get('nominal'),release_world_pose=t.get('selected_release_world_pose'),
                candidate_stats=placement.get('candidate_stats'),failure_count=len(placement.get('failures',[])))
            if dataset_index==0:baseline[key]=(data,t)
            elif key in baseline:
                old_data,old=baseline[key]
                same=(data.get('source')==old_data.get('source') and t['entry_index']==old['entry_index']
                      and t['committed_prefix']==old['committed_prefix'])
                row.update(same_recorded_boundary=same,
                    baseline_success=old['success'],baseline_total_wall_s=old['total_wall_s'],
                    baseline_backend=old.get('options',{}).get('endpoint_backend','scalar'),
                    nominal_changed=placement.get('nominal')!=(old.get('placement') or {}).get('nominal'),
                    baseline_over_current_wall_ratio=old['total_wall_s']/t['total_wall_s'] if same and old['success'] and t['success'] and t['total_wall_s']>0 else None)
                row['backend_changed']=row['baseline_backend']!=backend
            coverage.append([key[0],key[1],backend]);rows.append(row)
        datasets.append(dict(path=str(path.resolve()),completed=data.get('completed'),
            successes=sum(t['success'] for t in data['trials']),trials=len(data['trials']),coverage=coverage))
    return dict(scope='CPU-only fixed-prefix evidence comparison; not a continuous rollout or hardware result',
        caveats=['First dataset is the timing baseline; differing endpoint backends confound prior-only attribution',
                 'One run per setting, with lazy first-use cost retained; ratios are descriptive, not statistical speedup claims',
                 'Missing item/order/backend measurements are not fabricated or extrapolated'],datasets=datasets,rows=rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-folder',type=Path,default=EXP/'four20_async_attempt1')
    p.add_argument('--output',type=Path)
    p.add_argument('--orders',nargs='+',choices=('greedy','reach','diverse'),default=['greedy','reach','diverse'])
    p.add_argument('--items',nargs='+',type=int,default=[1,2,3,4])
    p.add_argument('--max-candidates',type=int,default=10)
    p.add_argument('--knot-step-m',type=float,default=.08)
    p.add_argument('--endpoint-backend',choices=('scalar','batch'),default='scalar')
    p.add_argument('--repeats',type=int,default=1)
    p.add_argument('--inspect',action='store_true',help='CPU-only reconstruction summary; no Newton import')
    p.add_argument('--compare',nargs='+',type=Path,help='CPU-only comparison of benchmark folders/JSON; first is baseline')
    a=p.parse_args()
    if a.compare:
        text=json.dumps(compare_results(a.compare),indent=2)
        if a.output:
            with a.output.open('x') as stream:stream.write(text+'\n')
            print(a.output)
        else:print(text)
        return
    a.run_folder=a.run_folder.resolve()
    source,trace,cases=load_cases(a.run_folder)
    cases=[c for c in cases if c['cycle'] in a.items]
    if not cases:p.error('No requested recorded cases')
    if a.inspect:
        print(json.dumps(dict(source=str(a.run_folder),cases=[summary(c) for c in cases],gpu_started=False),indent=2))
        return
    if a.output is None:p.error('--output required unless --inspect')
    if a.repeats<1 or a.max_candidates<1 or not np.isfinite(a.knot_step_m) or a.knot_step_m<=0:
        p.error('Positive repeats/candidate budget/knot spacing required')
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    for name in ('optimized_cycle_place.py','sparse_prior.py','batch_pose_ik.py'):
        path=Path(__file__).with_name(name)
        if path.is_file():shutil.copy2(path,a.output/name)
    sys.dont_write_bytecode=True
    for path in (ROOT/'experiments/s20_greedy_async_20261023',EXP,EXP/'perception'):
        sys.path.insert(0,str(path))
    os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
    os.environ.setdefault('WARP_CACHE_PATH',str(EXP/'live_view/cache/warp'))
    import newton
    import observed_cycle
    from compare import Box
    from perception.suitcase_capture import make_runtime
    helper=importlib.import_module('straight_place.optimized_cycle_place').place_and_return
    started=time.perf_counter();original_scene=observed_cycle.SCENE
    observed_cycle.SCENE=a.run_folder/'scene_tf.yaml'
    try:r=make_runtime(a.output/'runtime',source['source_catalog_m'])
    finally:observed_cycle.SCENE=original_scene
    if list(r.model.body_label)!=trace['body_labels'].tolist():raise ValueError('Model body order differs from saved trace')
    waypoints=json.loads((a.run_folder/'pick_waypoints.json').read_text())
    def taught_pose(name):
        q=np.asarray(waypoints[name]['q_rad'])
        r.ik.model.joint_q.assign(q)
        newton.eval_fk(r.ik.model,r.ik.model.joint_q,r.ik.model.joint_qd,r.ik.model)
        return r.ik.model.body_q.numpy()[r.ik.contact_body].copy()
    def no_move(*args,**kwargs):raise RuntimeError('Benchmark helper requested unexpected callback replanning')
    result=dict(source=str(a.run_folder),scope=__doc__,simulation_only=True,hardware_connected=False,
        actual_prefix_from_saved_simulation=True,alternative_placements_never_feed_later_trials=True,
        catalog=source['source_catalog_m'],runtime_initialization_s=time.perf_counter()-started,
        interpreter=sys.executable,pythonpath=os.environ.get('PYTHONPATH'),warp_cache=os.environ['WARP_CACHE_PATH'],
        helper_module='straight_place.optimized_cycle_place',
        timing_scope='one persistent runtime; each trial clears native IK cache; first trial may include lazy GPU compilation; helper segment cache is per invocation',
        cases=[summary(c) for c in cases],trials=[])
    def save():
        (a.output/'benchmark.json').write_text(json.dumps(result,indent=2,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v.item() if isinstance(v,np.generic) else str(v)))
    save()
    for case in cases:
        for repeat in range(a.repeats):
            # Rotate order across items to expose, rather than fix, first-use timing bias.
            orders=a.orders[(case['cycle']-1)%len(a.orders):]+a.orders[:(case['cycle']-1)%len(a.orders)]
            for order in orders:
                held=restore(r,case)
                r.planner_options=dict(candidate_order=order,max_candidates=a.max_candidates,knot_step_m=a.knot_step_m,
                                       endpoint_backend=a.endpoint_backend)
                report=dict(placements=[],events=[]);counts=dict(native_ik_calls=0,validate_calls=0)
                original_ik=r.ik._solve_once;original_validate=r.validate
                def counted_ik(*args,**kwargs):
                    counts['native_ik_calls']+=1
                    return original_ik(*args,**kwargs)
                def counted_validate(*args,**kwargs):
                    counts['validate_calls']+=1
                    return original_validate(*args,**kwargs)
                r.ik._solve_once=counted_ik;r.validate=counted_validate
                began=time.perf_counter();commit_at=None
                def publish():
                    nonlocal commit_at
                    if commit_at is None and report['placements'] and report['placements'][-1].get('phase')=='committing':
                        commit_at=time.perf_counter()
                trial=dict(cycle=case['cycle'],order=order,repeat=repeat,entry_index=case['entry_index'],
                           committed_prefix=len(case['occupied']),options=deepcopy(r.planner_options),success=False,
                           first_trial_in_process=not result['trials'])
                print(f"START cycle={case['cycle']} order={order} repeat={repeat}",flush=True)
                try:
                    occupied=[Box(v['id'],tuple(v['dims']),tuple(v['pos'])) for v in case['occupied']]
                    returned=helper(r,deepcopy(case['fitted']),case['contact_tcp'].copy(),occupied,
                                    no_move,taught_pose,publish,report,realtime=False)
                    trial.update(success=True,returned_occupancy=asdict(returned),
                                 selected_release_world_pose=r.fixed[held].tolist())
                except Exception as exc:trial['error']=f'{type(exc).__name__}: {exc}'
                finally:
                    r.ik._solve_once=original_ik;r.validate=original_validate
                trial.update(total_wall_s=time.perf_counter()-began,
                    planning_to_commit_s=commit_at-began if commit_at is not None else None,
                    measured_calls=counts,ik_cache_size_after=len(getattr(r.ik,'cache',{})),
                    placement=deepcopy(report['placements'][-1]) if report['placements'] else None)
                trial['optimizer']=deepcopy((trial['placement'] or {}).get('optimizer'))
                trial['endpoint_batch_stats']=deepcopy((trial['placement'] or {}).get('endpoint_batch_stats'))
                result['trials'].append(trial);save()
                print(json.dumps({k:trial[k] for k in ('cycle','order','success','total_wall_s','optimizer')}),flush=True)
    result.update(completed=True,total_benchmark_wall_s=time.perf_counter()-started,
                  successes=sum(t['success'] for t in result['trials']))
    save();print(a.output/'benchmark.json')


if __name__=='__main__':main()
