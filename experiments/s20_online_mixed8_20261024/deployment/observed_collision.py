"""Same-CAD FCL candidate collision checks without advancing Newton state."""
import numpy as np
import fcl,newton
from scipy.spatial.transform import Rotation as R
class Collision:
 def __init__(self,world,physical_robot_convex_hulls=False,native_sdf_body_allowlist=()):
  self.world=world;self.model=m=world.model;self.arrived=set();self.current_source=None;self.last_hits=[]
  self.native_sdf_body_allowlist=frozenset(native_sdf_body_allowlist)
  self.physical_robot_convex_hulls=bool(physical_robot_convex_hulls);self.physical_hull_objs={};self.last_hit_scope=None
  self.bodies=m.shape_body.numpy();self.local=m.shape_transform.numpy();self.objs={};self.flags=m.shape_flags.numpy();sc=m.shape_scale.numpy();typ=m.shape_type.numpy();self.scales=sc
  for i in range(m.shape_count):
   if not self.flags[i]&2:continue
   mesh=m.shape_source[i]
   if mesh is not None and hasattr(mesh,'_vertices'):
    verts=np.asarray(mesh._vertices,dtype=np.float64)*sc[i];faces=np.asarray(mesh._indices,dtype=np.int32).reshape(-1,3);g=fcl.BVHModel();g.beginModel(len(verts),len(faces));g.addSubModel(verts,faces);g.endModel()
   elif typ[i]==int(newton.GeoType.BOX):g=fcl.Box(*(2*sc[i]))
   elif typ[i]==int(newton.GeoType.PLANE):g=fcl.Plane(np.array([0.,0.,1.]),0.)
   elif typ[i]==int(newton.GeoType.SPHERE):g=fcl.Sphere(float(sc[i,0]))
   else:raise RuntimeError(f'Unsupported collider {m.shape_label[i]} type{typ[i]}')
   self.objs[i]=fcl.CollisionObject(g)
   if self.physical_robot_convex_hulls and 0<=self.bodies[i]<world.arm_count and m.body_label[self.bodies[i]] not in self.native_sdf_body_allowlist and mesh is not None and hasattr(mesh,'_vertices'):
    from scipy.spatial import ConvexHull
    hull=ConvexHull(verts);hg=fcl.BVHModel();hg.beginModel(len(verts),len(hull.simplices));hg.addSubModel(verts,np.asarray(hull.simplices,dtype=np.int32));hg.endModel();self.physical_hull_objs[i]=fcl.CollisionObject(hg)
  self.rigid=list(range(m.body_count))
  def root(b):
   while self.rigid[b]!=b:b=self.rigid[b]
   return b
  for jt,jp,jc in zip(m.joint_type.numpy(),m.joint_parent.numpy(),m.joint_child.numpy()):
   if int(jt)==3 and int(jp)>=0:self.rigid[root(int(jc))]=root(int(jp))
  self.rigid=[root(b) for b in range(m.body_count)]
  self.filters={tuple(sorted(map(int,pair))) for pair in m.shape_collision_filter_pairs}
  # Importer adjacency exclusions are original model semantics, not new filters.
  self.arm={i for i in self.objs if 0<=self.bodies[i]<world.arm_count}
  self.base={i for i in self.arm if m.body_label[self.bodies[i]].endswith(('/elfin_base','/elfin_base_link'))}
  self.suction={i for i in self.arm if any(s in m.body_label[self.bodies[i]] for s in ('suction','contact_frame'))}
 def _body_id(self,v):
  if v is None:return None
  # arrived stores box indices by contract.
  return self.world.boxes[int(v)]
 def validate(self,q6,actual=False,poses=None):
  w=self.world;m=self.model
  if poses is None:
   q=w.q.copy();q[w.qidx]=q6;w.fk.joint_q.assign(q)
   newton.eval_fk(m,w.fk.joint_q,w.fk.joint_qd,w.fk)
   poses=w.state.body_q.numpy().copy()
   if not actual:poses[:w.arm_count]=w.fk.body_q.numpy()[:w.arm_count]
  else:poses=np.array(poses,copy=True)
  source_actual=None if self.current_source is None else poses[self._body_id(self.current_source)].copy()
  if w.held is not None and not actual:
   tcp=poses[w.tcp];rot=R.from_quat(tcp[3:]);poses[w.held,:3]=tcp[:3]+rot.apply(w.grip_offset);poses[w.held,3:]=(rot*w.grip_rot).as_quat()
  active={self._body_id(i) for i in self.arrived}
  if self.current_source is not None:active.add(self._body_id(self.current_source))
  if w.held is not None:active.add(w.held)
  ids=[]
  for i,obj in self.objs.items():
   b=int(self.bodies[i])
   if b in w.boxes and b not in active:continue
   p0=self.local[i,:3];r0=R.from_quat(self.local[i,3:]);rot=r0 if b<0 else R.from_quat(poses[b,3:])*r0;pos=p0 if b<0 else R.from_quat(poses[b,3:]).apply(p0)+poses[b,:3]
   transform=fcl.Transform(rot.as_matrix(),pos);obj.setTransform(transform)
   if i in self.physical_hull_objs:self.physical_hull_objs[i].setTransform(transform)
   ids.append(i)
  self.last_hits=[];self.last_hit_scope=None;source=self._body_id(self.current_source)
  for k,i in enumerate(ids):
   bi=int(self.bodies[i])
   for j in ids[k+1:]:
    bj=int(self.bodies[j])
    if bi==bj or (bi>=0 and bj>=0 and self.rigid[bi]==self.rigid[bj]) or tuple(sorted((i,j))) in self.filters:continue
    moving_i=i in self.arm or bi==w.held;moving_j=j in self.arm or bj==w.held
    if not(moving_i or moving_j):continue
    if (i in self.base and m.shape_label[j]=='robot_pedestal') or (j in self.base and m.shape_label[i]=='robot_pedestal'):continue
    if (i in self.suction and bj in (source,w.held)) or (j in self.suction and bi in (source,w.held)):continue
    result=fcl.CollisionResult();fcl.collide(self.objs[i],self.objs[j],fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),result)
    scope='original_cad'
    if not result.is_collision and (i in self.physical_hull_objs or j in self.physical_hull_objs):
     result=fcl.CollisionResult();fcl.collide(self.physical_hull_objs.get(i,self.objs[i]),self.physical_hull_objs.get(j,self.objs[j]),fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),result);scope='physx_robot_convex_hull'
    if result.is_collision:
     self.last_hit_scope=scope
     # XPBD resting support has sub-mm penetration; permit only this actual
     # ground/source support contact, not obstacles or held-box transit.
     support=(m.shape_label[i]=='world_ground' and bj==source) or (m.shape_label[j]=='world_ground' and bi==source)
     if support and result.contacts:
      ground=i if m.shape_label[i]=='world_ground' else j
      luggage=j if ground==i else i
      normal=R.from_quat(self.local[ground,3:]).as_matrix()[:,2]
      half_height=float(np.abs(R.from_quat(source_actual[3:]).as_matrix().T@normal)@self.scales[luggage])
      ground_top=self.local[ground,:3]+normal*self.scales[ground,2]
      resting_depth=max(0.,float(normal@(ground_top-source_actual[:3]))+half_height)
      # Admit the measured soft resting contact; the planned lift must not
      # deepen it. This does not excuse ground penetration during transport.
      measured_depth=getattr(self,'source_ground_reference_depth_m',None)
      allowed_depth=resting_depth if measured_depth is None else measured_depth
      admissible_start=resting_depth<=.005 if measured_depth is None else True
      if admissible_start and max(c.penetration_depth for c in result.contacts)<=max(.0005,allowed_depth+.0001):
       continue
     self.last_hits.append([m.shape_label[i],m.shape_label[j]])
     return False
  return True

