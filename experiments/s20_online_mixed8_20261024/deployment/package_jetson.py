"""Package current sources/assets only; preserve the workspace-relative layout.

No virtualenv, compiled libraries, histories, media or credentials are shipped.
The portable candidate/device interfaces run on ARM. Newton/cuRobo simulation
sources are retained for provenance, not declared installed on the Jetson.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[3]
CURRENT = ROOT/'experiments/s20_online_mixed8_20261024'
GREEDY = ROOT/'experiments/s20_greedy_async_20261023'
AUTHOR = ROOT/'reproductions/ludan_greedy_20261023'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    files = set(CURRENT.glob('*.py')) | set(GREEDY.glob('*.py'))
    for folder in ('straight_place','perception','push_compare','deployment'):
        files.update((CURRENT/folder).glob('*.py'))
    files.add(CURRENT/'s20-contact-sim')
    files.add(CURRENT/'pick_waypoints.json')
    files.add(CURRENT/'pose_readback/measured_pose.py')
    for pattern in ('*.json','*.yaml','*.urdf','s20-interfaces','s20-device'):
        files.update((CURRENT/'deployment').glob(pattern))
    site_urdf=CURRENT/'deployment/site_robot.urdf'
    if site_urdf.is_file():
        spec=importlib.util.spec_from_file_location('site_model',CURRENT/'pose_readback/measured_pose.py')
        site=importlib.util.module_from_spec(spec);spec.loader.exec_module(site)
        xml=site.site_empty_urdf(site_urdf,AUTHOR)
        files.update(Path(mesh.get('filename')) for mesh in xml.iter('mesh'))
        files.update((AUTHOR/'src/luggage_description/config').glob('*origin.xacro'))
    files.update((AUTHOR/'deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor').glob('*.py'))
    for folder in (CURRENT/'perception/weights',GREEDY/'v2_s20_model_h100',
                   AUTHOR/'policy/vendor/roboticarm',AUTHOR/'src/luggage_gazebo/models/suitcase_vintage'):
        files.update(p for p in folder.rglob('*') if p.is_file() and
                     not any(part in ('.git','__pycache__') for part in p.parts))
    files.update((AUTHOR/'policy').glob('*.py'))
    files.update((AUTHOR/'deployment_ws/config').glob('scene_tf*.yaml'))
    for name in ('LICENSE','LICENSE.md','LICENSE.txt'):
        if (AUTHOR/name).is_file():files.add(AUTHOR/name)
    for name in ('native_servoj_executor.py','native_servo_stage.py','manual_pick_place.py'):
        files.add(ROOT/'tools/three_box_original'/name)
    for name in ('result.json','mixed_timing_and_provenance.json'):
        files.add(CURRENT/'mixed3plus1_contact_attempt1'/name)
    files = sorted(p for p in files if p.is_file())
    manifest = dict(scope=__doc__,files=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size) for p in files],
        total_source_bytes=sum(p.stat().st_size for p in files),
        portable_entry='experiments/s20_online_mixed8_20261024/deployment/s20-device',
        arm_runtime_dependencies=['numpy','scipy','PyYAML','opencv','installed ROS Humble and vendor CPS'],
        not_installed_by_archive=['Newton','Warp','cuRobo CUDA environment','x86 virtualenv'],
        full_hardware_cycle_ready=False,automatic_motion=False,old_deployment_overwritten=False)
    (args.output/'source_manifest.json').write_text(json.dumps(manifest,indent=2))
    archive=args.output/'s20_contact_mixed_source.tar.gz'
    with tarfile.open(archive,'w:gz',dereference=True) as tar:
        for path in files:tar.add(path,arcname=str(path.relative_to(ROOT)),recursive=False)
    print(json.dumps(dict(files=len(files),source_bytes=manifest['total_source_bytes'],
                          archive=str(archive),archive_bytes=archive.stat().st_size)))


if __name__=='__main__':main()
