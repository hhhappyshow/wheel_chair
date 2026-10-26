"""Install the isolated ARM simulation dependencies from local offline files.

No sudo, apt installation, controller connection, camera or robot IO.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess


def main():
    root=Path(__file__).resolve().parents[3]
    data=root/'experiments/s20_online_mixed8_20261024/jetson_sim6_20261026'
    wheels=data/'wheels';prefix=data/'native';python=root/'sim_venv/bin/python'
    for package in sorted(wheels.glob('*.deb')):
        subprocess.run(['dpkg-deb','--extract',str(package),str(prefix)],check=True)
    env=dict(os.environ)
    env['SOURCE_DATE_EPOCH']='946684800'  # Wheel ZIP timestamps, without changing the device clock.
    env['CPATH']=str(prefix/'usr/include')
    env['LD_LIBRARY_PATH']=str(prefix/'usr/lib/aarch64-linux-gnu')+':'+env.get('LD_LIBRARY_PATH','')
    command=[str(python),'-m','pip','install','--no-index','--no-deps','--find-links',str(wheels)]
    subprocess.run(command+['numpy==1.26.4','scipy==1.13.1','trimesh','pyglet','PyYAML',
        'Cython','setuptools==70.3.0','wheel>=0.45','warp-lang==1.18.0+cu12'],env=env,check=True)
    build=data/'python-fcl-build70'
    if not build.exists():
        shutil.copytree(data/'python-fcl-source',build,
            ignore=shutil.ignore_patterns('build','dist','*.egg-info','__pycache__'))
    subprocess.run(command+['--no-build-isolation',str(build)],env=env,check=True)
    result=subprocess.run([str(python),'-c',
        'import warp as w,numpy,scipy,fcl;w.init();print(w.get_devices());print(numpy.__version__,scipy.__version__)'],
        env=env,text=True,capture_output=True)
    (data/'bootstrap_result.json').write_text(json.dumps(dict(returncode=result.returncode,
        stdout=result.stdout,stderr=result.stderr,system_environment_modified=False),indent=2))
    print(result.stdout);print(result.stderr)
    result.check_returncode()


if __name__=='__main__':main()
