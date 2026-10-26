"""Own one official camera driver for an online session; no robot commands."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def owned_driver_pid(folder):
    manifest=json.loads((folder/'driver.json').read_text())
    pid=manifest['pid']
    stat=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    if stat[19]!=manifest['process_start_ticks'] or os.getpgid(pid)!=pid:
        raise RuntimeError('Camera session PID no longer belongs to this driver')
    return pid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('start','stop'))
    p.add_argument('--folder',type=Path,required=True)
    a=p.parse_args()
    if a.action=='start':
        from camera_receive import driver_configuration
        a.folder.mkdir(parents=True,exist_ok=False)
        params,command=driver_configuration()
        with (a.folder/'driver.log').open('x') as log:
            driver=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        stat=(Path('/proc')/str(driver.pid)/'stat').read_text().rsplit(')',1)[1].split()
        (a.folder/'driver.json').write_text(json.dumps(dict(pid=driver.pid,
            process_start_ticks=stat[19],command=command,parameters=params),indent=2))
        print(json.dumps(dict(camera_driver_pid=driver.pid,motion_sent=False)))
    else:
        try:
            pid=owned_driver_pid(a.folder)
            os.killpg(pid,signal.SIGINT)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                stat=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
                if stat[0]=='Z':break
                time.sleep(.1)
            else:
                if owned_driver_pid(a.folder)==pid:os.killpg(pid,signal.SIGTERM)
        except FileNotFoundError:
            pass


if __name__=='__main__':main()
