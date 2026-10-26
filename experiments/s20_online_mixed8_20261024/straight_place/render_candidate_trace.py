"""Render recorded candidate decisions; no new candidate generation or motion."""
import argparse
import json
from pathlib import Path
import sys
import subprocess

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(HERE))

COLORS=dict(chosen='#1570b6',shortlisted='#238b67',eligible='#83929b',rejected='#c75a4a')


def panel(ax,step,statuses=None,show_choice=True):
    for b in step['occupied']:
        ax.add_patch(Rectangle(b['pos'][:2],*b['dims'][:2],facecolor='#c6d0d4',edgecolor='#596b74',alpha=.7))
        ax.text(b['pos'][0]+.02,b['pos'][1]+.02,b['id'],ha='left',va='bottom',fontsize=8)
    for status in ('rejected','eligible','shortlisted','chosen'):
        if statuses is not None and status not in statuses:continue
        rows=[v for v in step['candidates'] if v['status']==status]
        if rows:
            points=np.asarray([c['center'] for c in rows])
            ax.scatter(points[:,0],points[:,1],s=75 if status=='chosen' else 22,color=COLORS[status],
                       marker='*' if status=='chosen' else 'x' if status=='rejected' else 'o',label=status,alpha=.9)
    groups={}
    for c in step['candidates']:
        if show_choice and c['shortlist_rank'] is not None:
            key=tuple(np.round(np.asarray(c['center'][:2])/.03).astype(int))
            groups.setdefault(key,[]).append(c)
    for group in groups.values():
        point=np.mean([c['center'][:2] for c in group],axis=0)
        label='/'.join(str(c['shortlist_rank']) for c in sorted(group,key=lambda c:c['shortlist_rank']))
        right=point[0]>1.25
        ax.annotate(label,point,xytext=(-5 if right else 4,7),textcoords='offset points',fontsize=7,ha='right' if right else 'left')
    chosen=step.get('selected_box')
    if chosen and show_choice:ax.add_patch(Rectangle(chosen['pos'][:2],*chosen['dims'][:2],facecolor='none',edgecolor=COLORS['chosen'],lw=2))
    ax.set(xlim=(-.04,1.53),ylim=(-.04,2.01),xlabel='X: entrance -> rear (m)',ylabel='Y: right -> left (m)')
    ax.set_aspect('equal');ax.grid(alpha=.15)
    ax.set_title(f"Arrival {step['arrival']} | proposed {len(step['candidates'])}, shortlist {len(step['shortlist'])}",fontsize=10)


def video(folder,data,steps):
    path=folder/'candidate_decisions.mp4'
    encoder=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1280x720','-r','10','-i','-','-c:v','libx264','-crf','22','-pix_fmt','yuv420p',
        '-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    for step in steps:
        for phase in ('generated','filtered','selected'):
            fig,(ax,detail)=plt.subplots(1,2,figsize=(12.8,7.2),dpi=100,
                                        gridspec_kw={'width_ratios':[1,1.15]},layout='constrained')
            statuses=None if phase=='generated' else ('chosen','shortlisted','eligible') if phase=='filtered' else ('chosen','shortlisted')
            panel(ax,step,statuses,show_choice=phase=='selected')
            ax.legend(loc='upper left',fontsize=8);detail.axis('off')
            ranked=sorted((c for c in step['candidates'] if c['shortlist_rank']),key=lambda c:c['shortlist_rank'])
            lines=[f"Current box: {step.get('size_class_inches','')} inch proxy",
                   'Observed dimensions: '+ ' x '.join(f'{v*100:.1f}' for v in step['current_dimensions_m'])+' cm',
                   f"Committed prefix: {len(step['occupied'])} boxes",'',
                   f"Recorded proposals: {len(step['candidates'])}",
                   f"Rejected/deferred: {step['counts']['rejected']}",
                   f"Submitted shortlist: {len(step['shortlist'])}",'',
                   'Rank   lower XYZ (m)           support']
            if phase=='selected':
                lines += [f"{c['shortlist_rank']:>2}     "+','.join(f'{v:.2f}' for v in c['box']['pos'])+f"       {c['support']['coverage']:.0%}" for c in ranked]
            detail.text(0,.96,'\n'.join(lines),va='top',fontsize=12,fontfamily='monospace')
            detail.text(0,.05,'Recorded geometric decisions, not execution.\nVideo pacing is illustrative, not inference latency.',fontsize=10)
            fig.suptitle(f"Contact V2 | arrival {step['arrival']} | {phase}",fontsize=16)
            fig.canvas.draw();frame=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].tobytes()
            for _ in range(15):encoder.stdin.write(frame)
            plt.close(fig)
    encoder.stdin.close()
    if encoder.wait():raise RuntimeError('Candidate video encoder failed')
    print(path)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--arrival',type=int,default=2);p.add_argument('--native',action='store_true')
    p.add_argument('--video',action='store_true')
    a=p.parse_args();d=json.loads((a.folder/'trace.json').read_text());steps=d['methods'][0]['steps'];step=steps[a.arrival-1]
    if a.video:
        video(a.folder,d,steps);return
    if a.native:
        from straight_place.layout_preview import render
        accepted=[dict(accepted=True,box=s['selected_box']) for s in steps[:a.arrival] if s['accepted']]
        shortlisted=sorted((c for c in step['candidates'] if c['shortlist_rank']),key=lambda c:c['shortlist_rank'])
        overlays=[dict(box=c['box'],color=(.08,.45,.8) if c['status']=='chosen' else (.15,.7,.4)) for c in shortlisted]
        paths=render(dict(seed=d['seed'],method=f'candidate_step{a.arrival}',arrivals=accepted,candidate_overlays=overlays),a.folder)
        print(paths);return
    plt.rcParams.update({'font.size':9,'figure.facecolor':'white','axes.spines.top':False,'axes.spines.right':False})
    fig,(ax,table_ax)=plt.subplots(1,2,figsize=(13,7),gridspec_kw={'width_ratios':[1,1.6]},constrained_layout=True)
    panel(ax,step);ax.legend(loc='upper left',fontsize=8)
    rows=sorted((c for c in step['candidates'] if c['shortlist_rank']),key=lambda c:c['shortlist_rank'])
    text=[]
    for c in rows:
        b=c['box'];z,y,depth=c['sort_key'][:3]
        text.append([c['shortlist_rank'],c['id'],','.join(f'{v:.2f}' for v in b['pos']),
                     f'{c["support"]["coverage"]:.0%}',f'{c["neighbor_contact_m2"]:.3f}',
                     ','.join(f'{float(v):.2f}' for v in (z,y,depth))])
    table_ax.axis('off');table=table_ax.table(cellText=text,colLabels=['Rank','ID','Lower XYZ m','Support','Contact m2','First sort keys'],loc='upper center',cellLoc='center',bbox=[0,.37,1,.6])
    table.auto_set_font_size(False);table.set_fontsize(8)
    for col in range(6):table[(1,col)].set_facecolor('#d7eafb')
    reasons={}
    for c in step['candidates']:
        for reason in c['reasons']:reasons[reason]=reasons.get(reason,0)+1
    note='Rejections / deferrals (one proposal can have multiple reasons):\n'+'\n'.join(f'{k}: {v}' for k,v in sorted(reasons.items()))
    note+= ('\n\nActual first keys: height, rear-depth band, right-side band.\nThen face contact and envelope growth; no IK was run.' if d['active_method']=='contact' else '\n\nActual first keys: lower height, then right side, then depth.\nThis exposes the outward-before-left limitation; no IK was run.')
    note+='\nNumbers on plot = submitted shortlist ranks; projections can overlap in Z.'
    table_ax.text(0,.32,note,ha='left',va='top',fontsize=9)
    fig.suptitle(f"Recorded candidate decisions | {d['active_method']} | not a learned policy",fontsize=13)
    fig.savefig(a.folder/'overview.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,4,figsize=(15,9),constrained_layout=True)
    for ax,s in zip(axes.ravel(),steps):panel(ax,s)
    fig.suptitle('Eight online arrivals: committed prefix + actual candidate centers (not future boxes)',fontsize=13)
    fig.savefig(a.folder/'timeline.png',dpi=145);plt.close(fig)
    print(a.folder/'overview.png')


if __name__=='__main__':main()
