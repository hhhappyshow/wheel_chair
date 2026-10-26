"""Host-side online observation/planning and explicit Jetson motion dispatch.

Without --execute, prepare one current-box cycle only. Explicit execution runs
fresh per-arrival observations, overlaps place planning with pick preparation
and execution, and carries planned-pose occupancy into subsequent rounds.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import time
from current_profile import load_profile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
REMOTE='/home/hku_reconova/s20_contact_mixed_20261025_r2'
REL='experiments/s20_online_mixed8_20261024/deployment'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rounds',type=int,default=4)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--execution',choices=('servoj','native'),default='servoj')
    p.add_argument('--perception-target',choices=('luggage','any'),default='luggage',
        help='any accepts any YOLO class; use only with a single intended object in the pickup workspace')
    p.add_argument('--contact-offset-z-m',type=float,default=0.,
        help='Final contact world-Z offset; negative is downward, not force-controlled compression')
    p.add_argument('--vacuum-feedback',choices=('digital','operator'),default='digital',
        help='Operator mode is supervised and interactive; it does not infer pressure from DI0')
    p.add_argument('--native-speed-scale',type=float,default=1.,
        help='Native motion multiplier; does not shorten vacuum confirmation or planning')
    p.add_argument('--observation',type=Path,help='Offline replay of a captured input, never executable')
    p.add_argument('--preview-rounds',type=int,default=1,
        help='Offline only: repeat the recorded input to exercise accumulated occupancy')
    p.add_argument('--scene-state',type=Path)
    p.add_argument('--host',default='hku_reconova@192.168.3.12')
    p.add_argument('--key',type=Path,default=Path.home()/'.ssh/s20_rental_ed25519')
    a=p.parse_args()
    cycle_command='run-native-cycle' if a.execution=='native' else 'run-cycle'
    if a.vacuum_feedback=='operator' and a.execution!='native':
        raise ValueError('Operator confirmation is implemented only for the native execution option')
    native_args=['--speed-scale',str(a.native_speed_scale),'--vacuum-feedback',a.vacuum_feedback] if a.execution=='native' else []
    if a.rounds<1:raise ValueError('rounds must be positive')
    if a.execute and a.observation:raise ValueError('Real execution requires a fresh observation')
    if a.preview_rounds<1 or a.execute and a.preview_rounds!=1:
        raise ValueError('preview-rounds is positive and available only without execution')
    if a.preview_rounds>1 and not a.observation:
        raise ValueError('Repeated preview needs an explicitly recorded observation')
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(map(str,[ROOT/'experiments/s20_newton_update_20261020/runtime13',
        ROOT/'experiments/s20_greedy_async_20261023']))
    env.update(OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',WARP_CACHE_PATH=str(Path.home()/'.cache/warp'))
    ssh=['ssh','-i',str(a.key),'-o','BatchMode=yes','-o','ConnectTimeout=5',
        '-o','StrictHostKeyChecking=yes',a.host]
    scp=['scp','-q','-i',str(a.key),'-o','BatchMode=yes','-o','ConnectTimeout=5',
        '-o','StrictHostKeyChecking=yes']
    session_log=(a.output/'session.log').open('x')
    def run(command,**kwargs):
        kwargs.setdefault('stdout',session_log)
        kwargs.setdefault('stderr',subprocess.STDOUT)
        return subprocess.run(command,**kwargs)
    def say(message):print(message,flush=True)
    def remote(*args):
        visible=a.vacuum_feedback=='operator' and 'run-native-cycle' in args
        run(ssh+[shlex.join(map(str,args))],check=True,
            **(dict(stdout=None,stderr=None) if visible else {}))
    def local(script,*args):
        run([sys.executable,str(HERE/script),*map(str,args)],env=env,check=True,cwd=ROOT)
    scene=a.scene_state.resolve() if a.scene_state else None
    report=dict(scope=__doc__,requested_rounds=a.rounds,execute=a.execute,rounds=[],
        deployment_profile=load_profile(),
        execution_backend=a.execution,
        perception_target=a.perception_target,
        contact_offset_z_m=a.contact_offset_z_m,
        vacuum_confirmation_source=a.vacuum_feedback,
        native_speed_scale=a.native_speed_scale if a.execution=='native' else None,
        preview_rounds=a.preview_rounds,repeated_input_preview=a.preview_rounds>1,
        completed_hardware_cycles=0,occupancy_source='planned_release_pose_assumption')
    workers=[]
    archives=[]
    observer=None
    camera_session=f'{REMOTE}/runs/{a.output.name}/camera_session'
    camera_started=False
    session_began=time.monotonic()
    try:
        if not a.observation:
            remote(f'{REMOTE}/{REL}/s20-device','camera-session','start','--folder',camera_session)
            camera_started=True
            observer=subprocess.Popen(ssh+[shlex.join([f'{REMOTE}/{REL}/s20-device','observe','--serve'])],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=session_log,text=True,bufsize=1)
        for index in range(a.rounds if a.execute else a.preview_rounds):
            out=a.output/f'round_{index+1}';out.mkdir()
            remote_out=f'{REMOTE}/runs/{a.output.name}/round_{index+1}'
            began=time.monotonic()
            if a.execute:
                say(f'[{index+1}/{a.rounds}] Returning to observation pose')
                remote('mkdir','-p',remote_out)
                remote(f'{REMOTE}/{REL}/s20-device','snapshot','--output',remote_out+'/before_observation.json')
                run(scp+[a.host+':'+remote_out+'/before_observation.json',str(out)],check=True)
                initial_state=json.loads((out/'before_observation.json').read_text())
                if a.vacuum_feedback=='digital' and initial_state['vacuum']['DI0']:
                    raise RuntimeError('Holding feedback is active (DI0=1); safely support and release '
                        'the previous box before an empty-arm restart. No return motion sent.')
                back=out/'observation_return'
                args=['--snapshot',out/'before_observation.json','--output',back,
                    '--vacuum-feedback',a.vacuum_feedback]
                if scene:args+=['--scene-state',scene]
                local('prepare_observation_return.py',*args)
                if json.loads((back/'cycle.json').read_text())['operations']:
                    remote_back=remote_out+'/observation_return'
                    run(scp+['-r',str(back),a.host+':'+remote_back],check=True)
                    return_error=None
                    try:
                        remote(f'{REMOTE}/{REL}/s20-device',cycle_command,'--cycle',remote_back+'/cycle.json',
                            '--action','/elfin_arm_controller/follow_joint_trajectory','--execute',
                            '--output',remote_out+'/observation_return_execution',*native_args)
                    except subprocess.CalledProcessError as exc:
                        return_error=exc
                    finally:
                        run(scp+['-r',a.host+':'+remote_out+'/observation_return_execution',str(out)],check=False)
                    feedback=out/'observation_return_execution/result.json'
                    if feedback.exists():
                        returned=json.loads(feedback.read_text())
                        if not returned['completed']:
                            raise RuntimeError('Observation return: '+returned.get('error','execution incomplete'))
                    elif return_error:
                        raise RuntimeError('Observation return failed; see session.log') from return_error
                row_return_s=time.monotonic()-began
            else:
                row_return_s=0.
            observation_began=time.monotonic()
            say(f'[{index+1}/{a.rounds}] Acquiring current RGB-D and fitting box')
            if a.observation:
                observation=a.observation.resolve()
            else:
                observe_args=[f'{REMOTE}/{REL}/s20-device','observe','--output',remote_out+'/observation',
                    '--camera-session',camera_session,'--perception-target',a.perception_target,
                    '--contact-offset-z-m',str(a.contact_offset_z_m)]
                if scene:
                    remote('mkdir','-p',remote_out)
                    run(scp+[str(scene),a.host+':'+remote_out+'/current_scene.json'],check=True)
                    observe_args+=['--scene-state',remote_out+'/current_scene.json']
                observe_error=None
                try:
                    observer.stdin.write(json.dumps(dict(argv=list(map(str,observe_args[2:]))))+'\n')
                    observer.stdin.flush()
                    response=json.loads(observer.stdout.readline())
                    if response['returncode']:raise RuntimeError(response.get('error','Observation failed'))
                except (RuntimeError,ValueError,OSError) as exc:
                    observe_error=exc
                if observe_error:
                    run(scp+['-r',a.host+':'+remote_out+'/observation',str(out)],check=False)
                else:
                    needed=['controller_snapshot.json','grasp.json','candidates.json','result.json',
                        'resolved_robot.urdf','observation/observation.json','observation/observed_box.json']
                    bundle=out/'observation_planning.tar'
                    with bundle.open('wb') as stream:
                        run(ssh+[shlex.join(['tar','-C',remote_out+'/observation','-cf','-',*needed])],
                            stdout=stream,check=True)
                    with tarfile.open(bundle) as saved:saved.extractall(out/'observation')
                    # Keep SCP from rewriting JSON while planning reads the small bundle.
                    archive=out/'observation_archive';archive.mkdir()
                    archives.append((subprocess.Popen(scp+['-r',a.host+':'+remote_out+'/observation',str(archive)],
                        stdout=session_log,stderr=subprocess.STDOUT),out))
                if observe_error:
                    details=out/'observation/observation/observation.json'
                    status=json.loads(details.read_text()).get('status') if details.exists() else None
                    raise RuntimeError('Perception: '+str(status or observe_error)+
                        '; recorded image and details in '+str(out/'observation')) from observe_error
                observation=out/'observation'
            row=dict(round=index+1,observation_s=time.monotonic()-observation_began,
                return_before_observation_s=row_return_s,hardware_cycle_completed=False)
            perception_result=observation/'result.json'
            if perception_result.exists():
                measured=json.loads(perception_result.read_text())
                row.update(observation_device_s=measured.get('wall_s'),
                    observation_device_timing_s=measured.get('timing_s'))
            report['rounds'].append(row)
            common=['--observation',str(observation/'observation/observation.json'),
                '--grasp',str(observation/'grasp.json'),'--site-urdf',str(HERE/'site_robot.urdf')]
            place=out/'place';pick=out/'pick';plan_began=time.monotonic()
            say(f'[{index+1}/{a.rounds}] Preparing pick and asynchronous place paths')
            with (out/'place.log').open('x') as log:
                command=[sys.executable,str(HERE/'prepare_observed_place.py'),*common,'--output',str(place)]
                if scene:command+=['--scene-state',str(scene)]
                planner=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
            workers.append(planner)
            with (out/'pick.log').open('x') as log:
                run([sys.executable,str(HERE/'prepare_observed_pick.py'),*common,
                    '--snapshot',str(observation/'controller_snapshot.json'),'--output',str(pick)],
                    env=env,stdout=log,stderr=subprocess.STDOUT,check=True,cwd=ROOT)
            pick_result=json.loads((pick/'result.json').read_text())
            if not pick_result['success']:
                raise RuntimeError('Pick planning: '+pick_result.get('error','unknown failure'))
            pick_recipe=out/'pick_recipe'
            local('compose_cycle.py','--pick',pick,'--output',pick_recipe)
            if planner.poll() is not None and (place/'result.json').exists():
                available=json.loads((place/'result.json').read_text())
                if not available['success']:
                    raise RuntimeError('Place planning: '+str(available.get('error','see place/result.json')))
            def dispatch(recipe,name,place_only=False):
                target=remote_out+'/'+recipe.name
                remote('mkdir','-p',remote_out)
                run(scp+['-r',str(recipe),a.host+':'+target],check=True)
                args=[f'{REMOTE}/{REL}/s20-device',cycle_command,'--cycle',target+'/cycle.json',
                    '--action','/elfin_arm_controller/follow_joint_trajectory','--execute',
                    '--output',remote_out+'/'+name]
                if place_only:args+=['--place-only','--initial-holding']
                args+=native_args
                remote_error=None
                try:
                    remote(*args)
                except subprocess.CalledProcessError as exc:
                    remote_error=exc
                finally:
                    run(scp+['-r',a.host+':'+remote_out+'/'+name,str(out)],check=False)
                if not (out/name/'result.json').exists():
                    raise RuntimeError(f'{name}: remote execution failed; see session.log') from remote_error
                result=json.loads((out/name/'result.json').read_text())
                if not result['completed']:raise RuntimeError(result.get('error','Hardware sequence did not finish'))
            if a.execute:
                say(f'[{index+1}/{a.rounds}] Executing contact, vacuum, lift and entry')
                dispatch(pick_recipe,'pick_execution')
            arrived=time.monotonic();code=planner.wait()
            row['entry_wait_s']=time.monotonic()-arrived if a.execute else None
            if code:raise RuntimeError('Place subprocess failed; see place.log')
            result=json.loads((place/'result.json').read_text())
            row.update(place_compute_s=result['compute_s'],place_initialization_s=result['initialization_s'],
                planning_started_before_pick=True,actual_async_overlap_measured=a.execute)
            if not result['success']:raise RuntimeError('Place planning: '+str(result.get('error','see place/result.json')))
            recipe=out/'cycle_recipe';local('compose_cycle.py','--pick',pick,'--place',place,'--output',recipe)
            if a.execute:
                say(f'[{index+1}/{a.rounds}] Executing place, release and empty return')
                dispatch(recipe,'place_execution',place_only=True)
                scene=out/'place_execution/scene_state.json'
                row['hardware_cycle_completed']=True;report['completed_hardware_cycles']+=1
            else:
                scene=recipe/'next_scene_preview.json'
            row.update(wall_s=time.monotonic()-began,
                planning_and_pick_preparation_window_s=time.monotonic()-plan_began)
            (a.output/'result.json').write_text(json.dumps(report,indent=2))
            say(f'[{index+1}/{a.rounds}] '+('Hardware cycle completed' if a.execute else 'Offline preparation completed; no motion sent'))
    except BaseException as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        say('STOPPED: '+report['error'])
    finally:
        for worker in workers:
            if worker.poll() is None:worker.terminate();worker.wait()
        if observer is not None:
            observer.stdin.close()
            if observer.wait():say('Perception worker exited with an error; see session.log')
        report['perception_archive_returncodes']=[]
        for archive,out in archives:
            code=archive.wait();report['perception_archive_returncodes'].append(code)
            if code==0:
                shutil.copytree(out/'observation_archive/observation',out/'observation',dirs_exist_ok=True)
                shutil.rmtree(out/'observation_archive')
        if any(report['perception_archive_returncodes']):
            say('Some perception media were not downloaded; remote originals retained. See session.log')
        if camera_started:
            try:remote(f'{REMOTE}/{REL}/s20-device','camera-session','stop','--folder',camera_session)
            except subprocess.CalledProcessError:
                say('Camera cleanup failed; see session.log (robot state unchanged)')
        report['session_wall_s']=time.monotonic()-session_began
        report['camera_session']=camera_session if camera_started else None
        (a.output/'result.json').write_text(json.dumps(report,indent=2))
        session_log.close()
    say(f'Completed hardware cycles: {report["completed_hardware_cycles"]}/{a.rounds}. Logs: {a.output}')
    return 1 if 'error' in report else 0


if __name__=='__main__':raise SystemExit(main())
