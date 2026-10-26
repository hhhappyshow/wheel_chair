"""Arrival-side generator. get(i) may reveal only the next or an already seen item."""
from copy import deepcopy
import numpy as np

NOMINAL={20:[.55,.40,.25],24:[.65,.45,.30]}
ENTRY_Q=[.41335404,-1.50384414,1.97354269,-2.06276989,-1.54618955,.4037475]
OBSERVE_Q=[-.1489124894,-.9454958439,1.6285656691,-2.2548723221,-1.5769966841,1.4118305445]


class ArrivalStream:
    def __init__(self,seed=73,state=None):
        self.seed=seed;self.rng=np.random.default_rng(seed);self.pose_rng=np.random.default_rng(seed+10000)
        self.observed=[]
        if state:
            self.observed=deepcopy(state["observed"])
            self.rng.bit_generator.state=state["size_rng_state"]
            self.pose_rng.bit_generator.state=state["pose_rng_state"]

    def get(self,index):
        if index<0 or index>len(self.observed):raise IndexError("Future arrivals are not available")
        if index==len(self.observed):
            label=int(self.rng.choice([20,24]));dims=np.round(np.array(NOMINAL[label])*self.rng.uniform(.9,1.1,3),3)
            xy=self.pose_rng.uniform([-.10,1.52],[.10,1.68]);yaw=float(self.pose_rng.uniform(-np.pi,np.pi))
            self.observed.append(dict(index=index,size_inches=label,dimensions_m=dims.tolist(),
                ground_center_world_m=[*xy,dims[2]/2],ground_yaw_rad=yaw))
        return deepcopy(self.observed[index])

    def state(self):
        return dict(seed=self.seed,observed=deepcopy(self.observed),size_rng_state=self.rng.bit_generator.state,
                    pose_rng_state=self.pose_rng.bit_generator.state,future_items_exposed=False)


def resize_arrived_slot(runtime,index,dims):
    """Resize only the newly revealed primitive box; this is an FK-only model."""
    import fcl
    body=runtime.boxes[index];scales=runtime.model.shape_scale.numpy();bodies=runtime.model.shape_body.numpy()
    for shape in np.flatnonzero(bodies==body):
        scales[shape]=np.array(dims)/2
        if shape in runtime.collision.objs:
            runtime.collision.objs[shape]=fcl.CollisionObject(fcl.Box(*dims))
    runtime.model.shape_scale.assign(scales);runtime.collision.scales=scales
