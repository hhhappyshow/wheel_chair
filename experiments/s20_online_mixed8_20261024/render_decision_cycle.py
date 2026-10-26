"""Synchronized same-run FK, saved observations and actual candidate decisions.

Reads the native wall-clock video and matching recorded timestamps. Candidate
points are disclosed only after their real generation time; future choices and
future timing results are not displayed early. This is evidence replay, not a
live camera, a dynamics validation or hardware execution.
"""
import argparse
from io import BytesIO
import json
from pathlib import Path
import subprocess
import textwrap

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from straight_place.render_candidate_trace import panel


FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def text(canvas, xy, value, size=20, fill='#e7edf1', width=48):
    draw = ImageDraw.Draw(canvas);font = ImageFont.truetype(FONT, size)
    lines = []
    for line in str(value).split('\n'):
        lines.extend(textwrap.wrap(line, width) or [''])
    draw.multiline_text(xy, '\n'.join(lines), font=font, fill=fill, spacing=8)


def plot(trace, cycle, selected=None):
    fig, ax = plt.subplots(figsize=(6.4, 6), dpi=100, layout='constrained')
    step = dict(trace, arrival=cycle)
    panel(ax, step, show_choice=True)
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, ['geometry #1' if label=='chosen' else label for label in labels],
              loc='upper left', fontsize=8)
    ax.set_title(f'Actual query {cycle}: geometry shortlist', fontsize=12)
    if selected is not None:
        b = selected
        point = np.asarray(b['pos'])+np.asarray(b['dims'])/2
        ax.scatter(*point[:2], marker='*', s=200, c='#ef941c', edgecolors='black', zorder=10)
    buffer = BytesIO();fig.savefig(buffer, format='png');plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert('RGB').copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-folder', type=Path, required=True)
    parser.add_argument('--arm-video', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.run_folder/'result.json').read_text())
    render = json.loads(args.arm_video.with_name('online_newton_render.json').read_text())
    if not render['wall_clock_playback']:
        raise ValueError('Use the same-run --wall-clock native arm video')
    data = np.load(args.run_folder/'trajectory.npz')
    times = data['wall_times_s'];times = times-times[0]
    stages = data['stage'];fps = render['fps'];frames = render['rendered_frames']
    origin = report['run_started_monotonic_s']
    reveals = np.flatnonzero(stages == 'CURRENT_ARRIVAL_REVEALED')
    async_rows = report['async_planning']
    views = report['views'];placements = report['placements']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    decode = subprocess.Popen(['ffmpeg','-v','error','-i',str(args.arm_video),
        '-f','rawvideo','-pix_fmt','rgb24','-'], stdout=subprocess.PIPE)
    encode = subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1920x1080','-r',str(fps),'-i','-','-c:v','libx264','-preset','fast',
        '-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)],stdin=subprocess.PIPE)
    charts = {};cameras = {};audit = []
    for i in range(frames):
        raw = decode.stdout.read(1920*480*3)
        if len(raw) != 1920*480*3:
            raise RuntimeError(f'Unexpected native video EOF at {i}/{frames}')
        wall = i/fps;frame = max(0, min(len(times)-1, int(np.searchsorted(times,wall,side='right')-1)))
        cycle = max(1,int(np.searchsorted(reveals,frame,side='right')))
        row = next((r for r in async_rows if r['cycle']==cycle), None)
        candidates = None;choice = None;source = 'Awaiting current observation';trace_key = None;decision = None
        if row and wall >= row['dispatch_wall_s']:
            metadata = row.get('worker_metadata', {})
            proposed = metadata.get('placement') or metadata.get('placement_meta') or {}
            actual = next((p for p in placements if p.get('id')==f'B{cycle}'), {})
            if row.get('consumed_background_plan'):
                options = [('background proposal',proposed)]
            else:
                options = [('background proposal',proposed),('actual-entry replan',actual)]
            for name, item in options:
                trace = item.get('candidate_trace')
                if trace and trace['generated_monotonic_s']-origin <= wall:
                    candidates = trace;source = name;trace_key = (cycle,name);decision = item
                    if item.get('selected_monotonic_s',float('inf'))-origin <= wall:
                        choice = item.get('nominal')
            source = source if candidates else 'Worker running; candidates not generated yet'
        canvas = Image.new('RGB',(1920,1080),'#202b34')
        canvas.paste(Image.frombytes('RGB',(1920,480),raw),(0,0))
        text(canvas,(12,8),f'FK replay | t={wall:.1f}s | item {cycle}/{report.get("requested_cycles",4)}',18)
        text(canvas,(655,8),'Top view',18);text(canvas,(1295,8),'Side view',18)
        if candidates:
            key = (trace_key, choice is not None)
            if key not in charts:charts[key] = plot(candidates,cycle,choice)
            canvas.paste(charts[key],(0,480))
        else:
            text(canvas,(25,540),'Candidate generation\n\n'+source,22)
        visible = [v for v in views if v['cycle']==cycle and v['observation_completed_wall_s']<=wall]
        if visible:
            view = visible[-1];index = view['view_index']
            if index not in cameras:
                folder=args.run_folder/f'view{index}'
                path=folder/'yolo_overlay.png'
                if not path.exists():path=folder/'camera_rgb.png'
                cameras[index]=ImageOps.pad(Image.open(path).convert('RGB'),(640,360))
            canvas.paste(cameras[index],(640,515))
            dims=view['fitted']['dimensions_m']
            model=next((m for m in report.get('current_item_models',[]) if m['cycle']==cycle),None)
            nominal=model['planning_dimensions_m'] if model else [.55,.4,.25]
            model_label=('Current RGB-D fit (continuous dimensions)' if report.get('dimension_noise_fraction') or report.get('arrivals_mode')=='four20-two24' else
                         'Current fit -> known20/24 nominal class' if report.get('arrivals_mode')=='three20-one24'
                         else 'Known uniform20 task prior')
            text(canvas,(660,895),'Saved observation (not live)\n'
                 f'Captured after {view["observation_completed_wall_s"]:.2f}s\n'
                 +'Fit cm: '+' x '.join(f'{v*100:.1f}' for v in dims)+'\n'
                 +model_label+'\nPlan cm: '+' x '.join(f'{v*100:.1f}' for v in nominal),18,width=49)
        else:text(canvas,(665,550),'Waiting for current-box observation',22)
        lines=[f'ITEM {cycle} | {source}',str(stages[frame]),'',
               'Same-run monotonic wall-clock replay',
               'Ideal suction / frozen release / no dynamics','']
        if row and wall>=row['dispatch_wall_s']:
            m=row.get('worker_metadata',{});done=m.get('worker_finished_at',float('inf'))-origin
            elapsed=max(0,wall-row['dispatch_wall_s'])
            lines += [f'Request: {row["request_id"]}',f'Dispatched at: {row["dispatch_wall_s"]:.2f}s']
            if wall>=done:
                lines += [f'Worker compute: {row["worker_compute_s"]:.3f}s',
                          'Worker result: '+('planned' if m.get('ok') else 'failed; fallback required')]
            else:lines += [f'Planning in parallel ({elapsed:.2f}s since dispatch)']
            arrival=row.get('arrived_entry_wall_s',float('inf'))
            received=arrival+row.get('entry_wait_s',0)
            if wall>=received:
                lines += [f'Entry wait: {row["entry_wait_s"]:.3f}s']
                adopted_at=received+row.get('arrival_validation',{}).get('wall_s',0.)
                if wall>=adopted_at:
                    lines += ['Adopted background: '+str(row.get('consumed_background_plan'))]
            if candidates:
                lines += [f'Proposals / shortlist: {len(candidates["candidates"])} / {len(candidates["shortlist"])}',
                          f'Spatial selection: {candidates["stats"]["total_s"]:.4f}s']
            if choice:
                lines += ['Orange star: path-accepted goal',
                          f'Route search after candidates: {decision["selected_monotonic_s"]-candidates["generated_monotonic_s"]:.3f}s',
                          'Lower XYZ: '+', '.join(f'{v:.2f}' for v in choice['pos'])]
        text(canvas,(1300,505),'\n'.join(lines),19,width=51)
        encode.stdin.write(canvas.tobytes())
        if i in (0,frames//2,frames-1):canvas.save(args.output.with_name(f'combined_review_{i}.png'))
    decode.stdout.close();encode.stdin.close()
    if decode.wait() or encode.wait():raise RuntimeError('Video decode/encode failed')
    for row in async_rows:
        audit.append({k:row.get(k) for k in ('cycle','request_id','worker_compute_s','entry_wait_s','consumed_background_plan','actual_entry_replan_and_execution_wall_s')})
    args.output.with_suffix('.json').write_text(json.dumps(dict(source_run=str(args.run_folder.resolve()),
        source_native_video=str(args.arm_video.resolve()),fps=fps,frames=frames,wall_clock=True,
        timing_basis='saved wall_times_s; no future candidates/timing disclosed before recorded timestamps',
        camera='saved per-observation images, not continuous or live feed',
        scope=__doc__,per_item=audit),indent=2))
    print(args.output)


if __name__=='__main__':main()
