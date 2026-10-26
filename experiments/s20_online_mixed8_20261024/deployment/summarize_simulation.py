"""Export measured stage timings and provenance from one recorded simulation."""
import argparse
import csv
import json
from pathlib import Path
import re


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();report=json.loads((a.run/'result.json').read_text())
    rows=[];items=[]
    for view in report.get('views',[]):
        rows.append(dict(cycle=view['cycle'],stage='OBSERVATION_'+str(view['view_index']),
            planning_wall_s=view.get('observation_wall_s'),execution_wall_s=None,reference_s=None,
            camera_render_save_s=view.get('camera_render_and_save_wall_s'),
            perception_fit_s=view.get('perception_after_render_wall_s'),
            source=view['status']))
    for event in report.get('events',[]):
        stage=event.get('stage','unknown');match=re.match(r'B(\d+)_',stage)
        cycle=event.get('cycle',int(match[1]) if match else None)
        rows.append(dict(cycle=cycle,stage=stage,planning_wall_s=event.get('planning_wall_s'),
            execution_wall_s=event.get('execution_wall_s'),reference_s=event.get('reference_s'),
            camera_render_save_s=None,perception_fit_s=None,source=event.get('planning_source','main simulation')))
    for cycle in report.get('cycles',[]):
        for stage in ('IDEAL_VACUUM_ON','IDEAL_RELEASE'):
            rows.append(dict(cycle=cycle['cycle'],stage=stage,planning_wall_s=0.,
                execution_wall_s=0.,reference_s=0.,camera_render_save_s=None,perception_fit_s=None,
                source='Instant simulation event, not measured pump or seal latency'))
    for cycle in range(1,report['requested_cycles']+1):
        async_row=next((r for r in report.get('async_planning',[]) if r['cycle']==cycle),{})
        metadata=async_row.get('worker_metadata',{})
        placement=next((r for r in report.get('placements',[]) if r.get('id')==f'B{cycle}'),{})
        candidate=(placement.get('candidate_trace') or
            (metadata.get('placement') or {}).get('candidate_trace') or {})
        views=[v for v in report.get('views',[]) if v['cycle']==cycle]
        model=next((v for v in report.get('current_item_models',[]) if v['cycle']==cycle),{})
        measured=next((v for v in report.get('cycles',[]) if v['cycle']==cycle),{})
        motion=[v for v in rows if v['cycle']==cycle and v['reference_s'] is not None]
        items.append(dict(item=cycle,size_class=report['generation_only_size_classes'][cycle-1],
            generated_dimensions_m=report['source_catalog_m'][cycle-1],
            planning_dimensions_m=model.get('planning_dimensions_m'),
            dimension_factors=report.get('generation_only_size_factors',[None]*report['requested_cycles'])[cycle-1],
            observation_total_s=sum(v.get('observation_wall_s',0) for v in views),
            perception_methods=[v['status'] for v in views],
            spatial_planning_s=candidate.get('stats',{}).get('total_s'),
            trajectory_and_other_worker_s=async_row.get('worker_compute_s'),
            worker_initialization_s=metadata.get('worker_initialization_s'),
            worker_compute_includes_spatial=True,
            candidate_count=len(candidate['candidates']) if candidate else None,
            shortlisted=len(candidate['shortlist']) if candidate else None,
            entry_wait_s=async_row.get('entry_wait_s'),
            hidden_compute_s=async_row.get('compute_overlap_with_pick_transfer_s'),
            adopted_background=async_row.get('consumed_background_plan'),
            actual_entry_replan_and_execution_s=async_row.get('actual_entry_replan_and_execution_wall_s'),
            reference_motion_s=sum(v['reference_s'] for v in motion),
            cycle_wall_s=measured.get('wall_s'),completed=measured.get('completed',False)))
    summary=dict(scope='One-run ideal-attachment FK simulation, not controller tracking or physical success',
        source_run=str(a.run.resolve()),completed_cycles=report.get('completed_cycles'),
        requested_cycles=report['requested_cycles'],error=report.get('error'),
        startup_s=report.get('initialization_wall_s'),run_wall_s=report.get('wall_s'),
        paced=report.get('wall_clock_paced'),items=items,
        timing_note='Worker/spatial computations overlap pickup. Do not add them to cycle wall time; cold startup is separate.',
        hardware_correspondence=dict(scene='same precise site YAML',robot='expanded current site URDF',
            camera=report.get('camera_model'),cargo=report.get('visible_cargo_geometry'),
            waypoints=report.get('waypoints_source'),
            differences=['FK reference following, not SDK ServoJ dynamics/braking/feedback',
                'Ideal attachment and frozen release; no real vacuum DI transitions',
                'Depth component fallback is task-prior perception, not successful YOLO detection',
                'Native pinhole does not reproduce real occlusion, lighting or depth error',
                'Existing sampled CAD checks, not a continuous GSIP certificate']))
    (a.run/'timing_summary.json').write_text(json.dumps(summary,indent=2))
    with (a.run/'stage_timings.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['cycle','stage','planning_wall_s','execution_wall_s','reference_s',
            'camera_render_save_s','perception_fit_s','source']);writer.writeheader();writer.writerows(rows)
    with (a.run/'item_timings.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(items[0]) if items else ['item']);writer.writeheader();writer.writerows(items)
    print(json.dumps(dict(completed=summary['completed_cycles'],requested=summary['requested_cycles'],
        error=summary['error'],summary=str(a.run/'timing_summary.json'))))


if __name__=='__main__':main()
