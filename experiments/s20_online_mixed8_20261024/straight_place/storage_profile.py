"""Bounded CPU-only storage measurements on one saved trajectory; no deletion."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import statistics
import subprocess
import time

import numpy as np

EXP=Path(__file__).resolve().parent.parent
ROOT=EXP.parent.parent


def disk_usage():
    command=['du','-B1','--max-depth=1',str(EXP)]
    output=subprocess.check_output(command,text=True)
    return dict(command=command,allocated_bytes_by_path={line.split('\t',1)[1]:int(line.split('\t',1)[0])
                                                       for line in output.splitlines()})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-folder',type=Path,default=EXP/'four20_sparse_attempt4')
    p.add_argument('--output',type=Path,default=EXP/'storage_profile_run')
    p.add_argument('--writes',type=int,choices=(1,2,3),default=3)
    a=p.parse_args();a.run_folder=a.run_folder.resolve();a.output=a.output.resolve()
    a.output.mkdir(parents=True,exist_ok=False)
    before=disk_usage();load_started=time.perf_counter()
    with np.load(a.run_folder/'trajectory.npz',allow_pickle=False) as archive:
        arrays={name:archive[name] for name in archive.files}
    load_s=time.perf_counter()-load_started
    count=len(arrays['body_q'])
    frame_keys=[k for k,v in arrays.items() if k!='body_labels' and v.ndim>0 and len(v)==count]
    static={k:v for k,v in arrays.items() if k not in frame_keys}
    result=dict(captured_at_utc=datetime.now(timezone.utc).isoformat(),source=str(a.run_folder),
        gpu_started=False,deleted_files=[],source_modified=False,
        input_archive_bytes=(a.run_folder/'trajectory.npz').stat().st_size,
        source_frames=count,array_bytes=sum(v.nbytes for v in arrays.values()),load_and_decompress_s=load_s,
        filesystem_before=before,methods={},
        timing_basis='CPU perf_counter through file close; page cache enabled, no fsync/cache eviction; not durable disk bandwidth',
        same_data='All methods archive the same arrays/frames; none drops raw fields',
        publisher_observation=dict(source='four20_sparse_cycle.py:106-132',
            behavior='Each publish materializes and compresses the full growing history, then atomically replaces trajectory.npz',
            limitation='This measures three equal full-size writes, not the actual growing-prefix publish schedule; does not count historical publish calls'),
        recommendations=[
            'Keep all existing raw trajectories, timestamps, observations, failure evidence and source snapshots; delete nothing in this audit',
            'Keep per-frame history in memory; publish compact latest-state/result snapshots, then write one final compressed archive',
            'If interruption recovery is needed, use bounded raw append/chunks plus a final archive rather than recompressing the whole history at every segment',
            'Uncompressed NPZ can reduce hot-path CPU work at a disk-space cost; preserve original dtypes and fields',
            'For new pure-layout work, record arrivals, candidate decisions and occupied boxes directly; do not create unused arm trajectories'
        ])
    for method in ('np_savez_compressed','np_savez','memory_append_final_compressed'):
        folder=a.output/method;folder.mkdir();rows=[]
        for repeat in range(a.writes):
            began=time.perf_counter();payload=arrays;append_s=materialize_s=0.
            if method=='memory_append_final_compressed':
                buffers={k:[] for k in frame_keys}
                for index in range(count):
                    for key in frame_keys:buffers[key].append(arrays[key][index].copy())
                append_s=time.perf_counter()-began;tick=time.perf_counter()
                payload=dict(static,**{k:np.asarray(v,dtype=arrays[k].dtype) for k,v in buffers.items()})
                materialize_s=time.perf_counter()-tick
            path=folder/f'archive_{repeat+1}.npz';tick=time.perf_counter()
            writer=np.savez if method=='np_savez' else np.savez_compressed
            writer(path,**payload)
            archive_s=time.perf_counter()-tick
            rows.append(dict(repeat=repeat+1,append_s=append_s,materialize_s=materialize_s,
                archive_s=archive_s,total_s=time.perf_counter()-began,file_bytes=path.stat().st_size))
            print(json.dumps(dict(method=method,**rows[-1])),flush=True)
        result['methods'][method]=dict(writes=rows,
            median_total_s=statistics.median(r['total_s'] for r in rows),
            median_archive_s=statistics.median(r['archive_s'] for r in rows),
            file_bytes=[r['file_bytes'] for r in rows])
    result['filesystem_after']=disk_usage()
    result['generated_archive_bytes']=sum(r['file_bytes'] for m in result['methods'].values() for r in m['writes'])
    (a.output/'storage_profile.json').write_text(json.dumps(result,indent=2))
    print(a.output/'storage_profile.json')


if __name__=='__main__':main()
