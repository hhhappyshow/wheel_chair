"""Current-occupancy greedy contact regions, then local motion search.

The endpoint sphere and 5cm support allowance match the reviewed old rule.
Neither is a dynamics/stability or arm-path feasibility certificate.
"""
import itertools
import time
import numpy as np
from scipy.spatial.transform import Rotation
from compare import Box,p
from online_packing_demo import coordinates


class CompactPrior:
    def __init__(self,cfg,hull,*,use_endpoint_sphere=True,max_box_top_m=None,depth_step_m=None):
        self.hull=hull
        tf=next(t for t in cfg["static_transforms"] if t["child"]=="container_link")
        self.rotation=Rotation.from_euler("xyz",tf["rotation_rpy"])
        self.translation=np.array(tf["translation"])
        self.center=np.array(cfg["pedestal"]["translation"])+[0,0,cfg["pedestal"]["size"][2]]
        self.radius=1.7
        self.use_endpoint_sphere=use_endpoint_sphere
        self.max_box_top_m=max_box_top_m
        self.depth_step_m=depth_step_m
        self.horizontal_first=False
        self.row_first=False
        self.lateral_clearance_m=0.
        self.lateral_bounds=None
        self.contact_first=False

    def top(self,b):
        h=self.hull
        return self.translation+self.rotation.apply(np.array(b.pos)+np.array(b.dims)*[.5,.5,1.]+[h.door_x,h.inner_y0,h.floor_z])

    def interval(self,size,y,z):
        h=self.hull
        if not self.use_endpoint_sphere:return (0.,h.length-size[0])
        point=self.translation+self.rotation.apply([h.door_x+size[0]/2,h.inner_y0+y+size[1]/2,h.floor_z+z+size[2]])
        relative=point-self.center;axis=self.rotation.apply([1,0,0]);projection=relative@axis
        disc=projection**2-relative@relative+self.radius**2
        if disc<0:return None
        lo=max(0.,-projection-np.sqrt(disc));hi=min(h.length-size[0],-projection+np.sqrt(disc))
        return (float(lo),float(hi)) if lo<=hi else None

    def supported(self,b,placed):
        if self.hull.floor_supports(b):return True
        x,y,z=b.pos;dx,dy,_=b.dims
        supports=[v for v in placed if -p.EPS<=z-v.pos[2]-v.dims[2]<=.05+p.EPS]
        expanded=[Box(v.id,(v.dims[0]+.10,v.dims[1]+.10,v.dims[2]),(v.pos[0]-.05,v.pos[1]-.05,z-v.dims[2])) for v in supports]
        center_supported=any(v.pos[0]-p.EPS<=x+dx/2<=v.pos[0]+v.dims[0]+p.EPS and v.pos[1]-p.EPS<=y+dy/2<=v.pos[1]+v.dims[1]+p.EPS for v in supports)
        return center_supported and p.fully_supported(b,expanded)

    def valid(self,b,placed):
        return (self.hull.fits(b) and b.pos[1]>=self.lateral_clearance_m-1e-7 and not any(p.collides(b,v) for v in placed)
                and (self.lateral_bounds is None or b.pos[1]>=self.lateral_bounds[0]-1e-7 and b.pos[1]+b.dims[1]<=self.lateral_bounds[1]+1e-7)
                and (self.max_box_top_m is None or b.pos[2]+b.dims[2]<=self.max_box_top_m+p.EPS)
                and self.supported(b,placed) and (not self.use_endpoint_sphere or np.linalg.norm(self.top(b)-self.center)<=self.radius+1e-7))

    def contacts(self,b,placed):
        lower=np.array(b.pos);upper=lower+b.dims;neighbors=0.;walls=0.
        for axis in range(3):
            others=[k for k in range(3) if k!=axis];face_area=float(np.prod(np.array(b.dims)[others]))
            for side in (0,1):
                area=0.;plane=(lower,upper)[side][axis]
                tolerance=.05 if axis==2 and side==0 else .02
                for v in placed:
                    vl=np.array(v.pos);vu=vl+v.dims
                    gap=abs(plane-(vu,vl)[side][axis])
                    overlap=np.maximum(0,np.minimum(upper[others],vu[others])-np.maximum(lower[others],vl[others]))
                    area+=float(np.prod(overlap))*max(0.,1-gap/tolerance)
                # Cap each face to avoid rewarding duplicate supporting faces.
                neighbors+=min(face_area,area)
                wall=None
                if axis==0 and side==1:wall=self.hull.length
                elif axis==1 and side==0:wall=0.
                elif axis==2 and side==0:wall=0.
                if wall is not None:walls+=face_area*max(0.,1-abs(plane-wall)/.02)
        return neighbors,walls

    def key(self,b,placed,profile="depth"):
        neighbor,wall=self.contacts(b,placed);x,y,z=b.pos;dx,dy,dz=b.dims
        orientation=(int(dx>dy),) if self.horizontal_first else ()
        if not placed:
            return orientation+(round(y/.02),-round((x+dx)/.02),z,-wall,-dy)
        if self.contact_first:
            # Contact-generated edges are kept exact. Rank connected, low
            # placements by the occupied cluster's envelope growth before
            # asking for another independent farthest-depth point.
            lower=np.array([v.pos for v in placed]);upper=lower+np.array([v.dims for v in placed])
            oldlo=lower.min(0);oldhi=upper.max(0)
            growth=float(np.prod(np.maximum(oldhi,np.array(b.pos)+b.dims)-np.minimum(oldlo,b.pos))-np.prod(oldhi-oldlo))
            outward=int(x+dx<=max(v.pos[0] for v in placed)+.02)
            return (int(neighbor<=1e-9),outward,round(z/.05),round(growth,6),
                    -round(neighbor,5),round(y/.02),-round((x+dx)/.02),*orientation)
        # Neighbor contact is the spatial prior. A distant deeper point must
        # not beat continuation of the committed compact cluster.
        height=-z if profile=="stack" else abs(z/dz-1) if profile=="pair" else z
        rear=max(v.pos[0]+v.dims[0] for v in placed)
        outward=0 if rear-x-dx<=.05 else 1
        if self.row_first:
            # Orientation is a tie-breaker within the rear/height/position
            # family, not a veto against filling a narrow leftover strip.
            # A rotated box has a different depth. Its farther rear face
            # must not redefine the entire row and evict the two short rows.
            row_front=max(v.pos[0] for v in placed)
            row_outward=int(x+dx<=row_front+.02)
            return (row_outward,round(z/.05),round(y/.02),*orientation,
                    -round((neighbor+wall)/.01),-round((x+dx)/.02),z)
        return orientation+(0 if neighbor>1e-9 else 1,outward,-round((neighbor+wall)/.01),height,
                -round((x+dx)/.02),round(y/.02))

    def candidates(self,dims,placed,profile="depth"):
        start=time.perf_counter();h=self.hull;rows=[]
        for size in sorted({tuple(dims),(dims[1],dims[0],dims[2])}):
            ys=set(coordinates(h.width,size[1],placed,1))|{h.flat_y-size[1],h.door_y0,h.door_y1-size[1]}
            ys|={v.pos[1]+(v.dims[1]-size[1])/2 for v in placed}
            if self.lateral_bounds:ys|={self.lateral_bounds[0],self.lateral_bounds[1]-size[1]}
            if self.lateral_clearance_m:
                ys|={self.lateral_clearance_m,h.flat_y-size[1]-self.lateral_clearance_m}
                ys|={v.pos[1]+v.dims[1]+self.lateral_clearance_m for v in placed}
                ys|={v.pos[1]-size[1]-self.lateral_clearance_m for v in placed}
            for y,z in itertools.product(sorted(ys),sorted({0.}|{v.pos[2]+v.dims[2] for v in placed})):
                if y<0 or y+size[1]>h.width:continue
                interval=self.interval(size,y,z)
                if interval is None:continue
                lo,hi=interval
                xs=set(coordinates(h.length,size[0],placed,0))|{lo,hi}
                if self.depth_step_m:xs.update(np.arange(lo,hi,self.depth_step_m))
                xs|={v.pos[0]+(v.dims[0]-size[0])/2 for v in placed}
                for delta in (.02,.04,.08,.16):xs.add(max(lo,hi-delta))
                for x in sorted(xs):
                    if lo-1e-8<=x<=hi+1e-8:
                        b=Box("current",size,(float(x),float(y),float(z)))
                        if self.valid(b,placed):rows.append(b)
        rows=list({(b.pos,b.dims):b for b in rows}.values());rows.sort(key=lambda b:self.key(b,placed,profile))
        adjacent=int(sum(self.contacts(b,placed)[0]>1e-9 for b in rows))
        return rows,dict(total_s=time.perf_counter()-start,returned=len(rows),adjacent_candidates=adjacent,
                         prior="empty: right/rear corner; occupied: neighboring face-contact cluster before remote points",
                         sphere="base_link radius1.7m; final top-center TCP only, not whole box/path" if self.use_endpoint_sphere else "disabled: geometry-only preview",
                         support="old5cm support-height/edge allowance, supported center, no new area-fraction gate",
                         scene_prefix=len(placed),future_arrivals_used=0,historical_xyz_used=False)
