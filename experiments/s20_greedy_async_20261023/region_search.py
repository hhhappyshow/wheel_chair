"""Depth-first online regions with 0/90 yaw and local alternatives."""
import time
import numpy as np
from compare import Box,pool,p,contact_area


def greedy_key(b,placed,hull,profile="depth"):
    # x grows from the door to the rear; y=0 is the straight/right wall
    # when viewed through the entrance. Motion hints never outrank depth.
    depth=round(b.pos[0]+b.dims[0],2)
    layer=round(b.pos[2]/b.dims[2],4)
    side=round(b.pos[1]/.16)
    contact=-contact_area(b,placed,hull)
    tail=(contact,-b.dims[0],round(b.pos[1],4))
    if profile=="flat":return (-depth,layer,side,*tail)
    if profile=="stack":return (-depth,-layer,side,*tail)
    if profile=="pair":return (-depth,abs(layer-1),layer,side,*tail)
    return (-depth,side,layer,*tail)


def region_candidates(dims,placed,hull,domain=(),top_regions=4,profile="depth"):
    begin=time.perf_counter();rows=pool(dims,placed,hull)
    # Real interior samples, not just a partition of historical endpoints.
    for size in sorted({tuple(dims),(dims[1],dims[0],dims[2])}):
        xs=sorted(set(np.arange(0,hull.length-size[0],.16))|{hull.length-size[0]})
        ys=sorted(set(np.arange(0,hull.width-size[1],.16))|{hull.width-size[1]})
        for z in sorted({0.}|{b.pos[2]+b.dims[2] for b in placed}):
            for x in xs:
                for y in ys:
                    b=Box("current",tuple(size),(round(float(x),4),round(float(y),4),z))
                    if valid(b,placed,hull):rows.append(b)
    # Historical endpoints seed regions; occupied volumes and support are
    # freshly evaluated. Unseeded geometric regions remain fallback choices.
    for node in domain:
        yaw=node["box_yaw_rad"];size=dims if abs(yaw)<.1 else (dims[1],dims[0],dims[2])
        x,y,_=node["contact_local_m"]
        for z in sorted({0.}|{b.pos[2]+b.dims[2] for b in placed}):
            b=Box("current",tuple(size),(x-size[0]/2,y-size[1]/2,z))
            if valid(b,placed,hull):rows.append(b)
    rows=list({(b.pos,b.dims):b for b in rows}.values());groups={}
    for b in rows:
        key=(round(b.pos[2],3),b.dims,round(b.pos[0]/.16),round(b.pos[1]/.16))
        groups.setdefault(key,[]).append(b)
    def key(b):return greedy_key(b,placed,hull,profile)
    regions=sorted((sorted(g,key=key) for g in groups.values()),key=lambda g:key(g[0]))
    selected=[region[0] for region in regions[:top_regions]];refined=0
    for region in regions[:top_regions]:
        anchor=region[0];local=list(region)
        for step in (.08,.04,.02):
            for dx,dy in ((0,0),(step,0),(-step,0),(0,step),(0,-step)):
                b=Box("current",anchor.dims,(round(anchor.pos[0]+dx,4),round(anchor.pos[1]+dy,4),anchor.pos[2]))
                if valid(b,placed,hull):local.append(b);refined+=1
        selected.extend(sorted(local,key=key))
    selected.extend(b for region in regions[top_regions:] for b in region)
    selected=list({(b.pos,b.dims):b for b in selected}.values())
    # Refinement can move a witness closer to the rear wall. Re-sort the
    # combined pool rather than letting representative order defeat greed.
    selected.sort(key=key)
    return selected,dict(total_s=time.perf_counter()-begin,base_candidates=len(rows),regions=len(regions),refined_valid=refined,returned=len(selected),profile=profile,order="rear face depth first; right wall y=0; profile selects stack versus spread; no historical endpoint priority",support_rule="unchanged source floor/full-support predicate; no added gate")


def valid(b,placed,hull):
    return hull.fits(b) and not any(p.collides(b,t) for t in placed) and (hull.floor_supports(b) or p.fully_supported(b,placed))
