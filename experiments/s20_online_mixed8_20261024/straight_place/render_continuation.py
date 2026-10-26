"""Prepare native renderer metadata and render only this saved continuation."""
import json
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
PARENT=HERE.parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run-folder',type=Path,default=HERE/'attempt1')
args=parser.parse_args();out=args.run_folder.resolve()
if not out.is_relative_to(HERE):parser.error('Run folder must remain inside straight_place')
report=json.loads((out/'result.json').read_text())
audit=json.loads((out/'audit.json').read_text())
report.update(source_catalog_m=report['actor_source_catalog_m'],asset_slots=len(report['actor_source_catalog_m']),
    actual_actor_inside_nominal_hull=audit['final_actor_aabb_inside_nominal_hull'],
    saved_frame_original_CAD_passed=not audit['independent_actual_saved_frame_CAD_rejections'],
    source_catalog_usage='Renderer/emulated actor geometry only; planner uses planning_dimensions_m from observed fit',
    completion_scope='Saved FK continuation only; frozen release/completion and actual actor containment measured in audit.json',
    timing_interpretation='Wall times and Cartesian reference times are separate in times.csv. First attempt1 lowering wall35.023989s included cold kernel compilation; its reference motion was0.166667s. No warm rerun of attempt1.')
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
shutil.copy2(Path(report['source'])/'scene_tf.yaml',out/'scene_tf.yaml')
media=out/'native_video'
media.mkdir(exist_ok=True)
env=os.environ.copy()
env.update(WARP_CACHE_PATH=str(HERE/'warp_cache'),XDG_CACHE_HOME=str(HERE/'cache'),
           PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
command=[sys.executable,'-B',str(PARENT/'render_reference.py'),'--run-folder',str(out),
         '--output',str(media),'--single-view','--cell-aligned','--cutaway','--suitcase-assets',
         '--stride','1','--fps','50']
subprocess.run(command,env=env,check=True)
metadata_path=media/'online_newton_render.json'
metadata=json.loads(metadata_path.read_text())
metadata.update(source_pickup_entry_run=report['source'],
    source_pickup_entry_replayed=False,uninterrupted_pick_to_place=False,
    begins_at_exact_source_final_body_q_and_joint_q=True,
    source_run_not_replaced_by_later_chain_attempt=True,
    actual_containment_validated=True,actual_actor_inside_nominal_hull=audit['final_actor_aabb_inside_nominal_hull'],containment_audit='../audit.json',
    scope=f"Native Newton perspective of saved single-box continuation. Actual first state exactly equals {Path(report['source']).name} final state; no pickup replay, no dynamics or hardware. Containment outcome is audit.json, not inferred from video.")
metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(dict(video=str(media/'online_newton_perspective.mp4'),metadata=str(metadata_path),
                     frames=metadata['rendered_frames'],duration_s=metadata['rendered_frames']/metadata['fps']),indent=2))
