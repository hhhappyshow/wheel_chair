"""Batch CPU FK for offline pick references; no controller or physics stepping."""
import importlib.util
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


class PickReferenceRecorder:
    def __init__(self,runtime,site_urdf):
        self.r=runtime
        experiment=Path(__file__).resolve().parents[1]
        root=experiment.parents[1]
        spec=importlib.util.spec_from_file_location('pick_site_fk',experiment/'pose_readback/measured_pose.py')
        fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
        xml=fk.site_empty_urdf(site_urdf,root/'reproductions/ludan_greedy_20261023')
        self.initial=runtime.state.body_q.numpy().copy()
        self.labels=list(runtime.model.body_label)
        base=self.initial[self.labels.index('S20/elfin_base')]
        self.base=np.eye(4);self.base[:3,:3]=Rotation.from_quat(base[3:]).as_matrix();self.base[:3,3]=base[:3]
        pending=list(xml.findall('joint'));known={'elfin_base'};self.chain=[]
        while pending:
            ready=[j for j in pending if j.find('parent').get('link') in known]
            if not ready:raise ValueError('Disconnected pick URDF chain')
            for joint in ready:
                parent=joint.find('parent').get('link');child=joint.find('child').get('link')
                index=None;axis=None
                if joint.get('type') in ('revolute','continuous'):
                    index=int(joint.get('name').removeprefix('elfin_joint'))-1
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
                self.chain.append((parent,child,fk.origin(joint),index,axis))
                known.add(child);pending.remove(joint)
        # The legacy payload placeholder is noncolliding; retain its TCP binding.
        self.offsets={}
        initial_frames=self.transforms(np.array([runtime.q]))
        for i,label in enumerate(self.labels[:runtime.w.arm_count]):
            link=label.removeprefix('S20/')
            parent=link if link in known else 'suction_contact_frame' if link=='payload_box' else None
            if parent is None:raise ValueError('Missing CPU FK link: '+label)
            pose=np.eye(4);pose[:3,:3]=Rotation.from_quat(self.initial[i,3:]).as_matrix();pose[:3,3]=self.initial[i,:3]
            self.offsets[i]=(parent,np.linalg.inv(initial_frames[parent][0])@pose)
        self.timing=dict(batch_fk_s=0.,cad_s=0.,record_s=0.)
        self.validated_points=0
        self.reference_points=0
        self.sampling=[]
        self.anchor_knots=None
        self.body_radius=np.zeros(len(self.labels))
        collision=runtime.collision
        for i in collision.objs:
            body=int(collision.bodies[i])
            if body<0:continue
            mesh=runtime.model.shape_source[i]
            radius=(np.max(np.linalg.norm(np.asarray(mesh._vertices)*collision.scales[i],axis=1))
                if mesh is not None and hasattr(mesh,'_vertices') else np.linalg.norm(collision.scales[i]))
            self.body_radius[body]=max(self.body_radius[body],np.linalg.norm(collision.local[i,:3])+radius)

    def transforms(self,grid):
        frames={'elfin_base':np.broadcast_to(self.base,(len(grid),4,4))}
        for parent,child,origin,index,axis in self.chain:
            transform=frames[parent]@origin
            if index is not None:
                motion=np.broadcast_to(np.eye(4),(len(grid),4,4)).copy()
                motion[:,:3,:3]=Rotation.from_rotvec(grid[:,index,None]*axis).as_matrix()
                transform=transform@motion
            frames[child]=transform
        return frames

    def poses(self,grid):
        frames=self.transforms(grid)
        poses=np.broadcast_to(self.initial,(len(grid),*self.initial.shape)).copy()
        for i,(parent,offset) in self.offsets.items():
            transform=frames[parent]@offset
            poses[:,i,:3]=transform[:,:3,3]
            poses[:,i,3:]=Rotation.from_matrix(transform[:,:3,:3]).as_quat()
        for body,pose in self.r.fixed.items():poses[:,body]=pose
        if self.r.w.held is not None:
            tcp=poses[:,self.r.tcp];rotation=Rotation.from_quat(tcp[:,3:])
            poses[:,self.r.w.held,:3]=tcp[:,:3]+rotation.apply(self.r.w.grip_offset)
            poses[:,self.r.w.held,3:]=(rotation*self.r.w.grip_rot).as_quat()
        return poses

    def append(self,grid,poses,stage):
        r=self.r;count=len(grid)
        r.qs.extend(np.array(grid,copy=True));r.history.extend(poses)
        r.stages.extend([stage]*count);r.simtime.extend(r.t+np.arange(count)*.02);r.t+=count*.02
        r.q=np.array(grid[-1],copy=True);r.w.q[:6]=r.q
        # Only stage boundaries are mirrored to the legacy runtime state.
        r.state.joint_q.assign(r.w.q);r.state.body_q.assign(poses[-1])

    def frame(self,q,stage):
        grid=np.asarray(q)[None,:]
        self.append(grid,self.poses(grid),stage)

    def execute(self,grid,stage):
        began=time.perf_counter();grid=np.asarray(grid)
        poses=self.poses(grid);self.timing['batch_fk_s']+=time.perf_counter()-began
        # Use the existing Cartesian planning step, not the retimed 20 ms clock.
        # Include curve knots and joint reversals, then bound body-point travel
        # between CAD samples with each body's collision-geometry radius.
        rotations=Rotation.from_quat(poses[:,:,3:].reshape(-1,4)).as_matrix().reshape(len(grid),-1,3,3)
        relative=np.swapaxes(rotations[:-1],-1,-2)@rotations[1:]
        angles=Rotation.from_matrix(relative.reshape(-1,3,3)).magnitude().reshape(len(grid)-1,-1)
        travel=np.max(np.linalg.norm(np.diff(poses[:,:,:3],axis=0),axis=2)+angles*self.body_radius,axis=1)
        indices=[0];distance=0.
        for i,step in enumerate(travel,1):
            if distance+step>.03 and i-1>indices[-1]:
                indices.append(i-1);distance=0.
            distance+=step
        indices.append(len(grid)-1)
        if self.anchor_knots is not None:
            indices.extend(int(np.argmin(np.linalg.norm(grid-q,axis=1))) for q in self.anchor_knots)
        difference=np.diff(grid,axis=0)
        indices.extend((np.flatnonzero(np.any(difference[:-1]*difference[1:]<0.,axis=1))+1).tolist())
        indices=np.unique(indices)
        arc=np.r_[0.,np.cumsum(travel)]
        self.sampling.append(dict(stage=stage,reference_points=len(grid),cad_points=len(indices),
            body_travel_step_m=.03,max_sampled_body_travel_bound_m=float(np.max(np.diff(arc[indices]))),
            validation_scope='Discrete original CAD; not a continuous native SDK sweep certificate'))
        start=time.perf_counter()
        for q,pose in zip(grid[indices],poses[indices]):
            if not self.r.collision.validate(q,poses=pose):
                raise RuntimeError(f'{stage}: CAD {self.r.collision.last_hits}')
            self.validated_points+=1
        self.reference_points+=len(grid)
        self.timing['cad_s']+=time.perf_counter()-start
        start=time.perf_counter();self.append(grid,poses,stage)
        self.timing['record_s']+=time.perf_counter()-start
        return time.perf_counter()-began
