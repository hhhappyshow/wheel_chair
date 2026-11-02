"""2026-10-30 measured, left/right-symmetric rear-axle velocity mapping.

Continuous input/output model; only the existing HID transport is quantized.
Calibration and grouped validation: artifacts/mouse_calibration_20261030/.
No temporal smoothing, delay compensation, or velocity feedback is implied.
"""
import math
import numpy as np
REFERENCE_FRAME='base_link'
INPUT_RADIUS=8.0
FORWARD_GAIN=0.06707777745024013
REVERSE_GAIN=0.050458867829928096
TURN_GAIN=0.10136067817594809
TARGETS=('right','right-forward','forward','left-forward','left')


def predict_input(x,y):
    """Normalized right/forward input -> continuous axle (v,w), radial limit 1."""
    x=float(x);y=float(y)
    if not math.isfinite(x) or not math.isfinite(y):raise ValueError('Nonfinite input')
    scale=max(1.,math.hypot(x,y));x/=scale;y/=scale
    return np.array([INPUT_RADIUS*y*(FORWARD_GAIN if y>=0 else REVERSE_GAIN),-INPUT_RADIUS*x*TURN_GAIN])


def velocity_for_hid(point,points):
    delta=np.asarray(point,dtype=float)-np.asarray(points['steering wheel center'],dtype=float)
    return predict_input(delta[0]/INPUT_RADIUS,-delta[1]/INPUT_RADIUS)


def map_input(x,y,points,hid_substeps=1):
    """Uniform Cartesian position mapping, then predict the actual integer HID point."""
    x=float(x);y=float(y)
    if not math.isfinite(x) or not math.isfinite(y):raise ValueError('Nonfinite input')
    scale=max(1.,math.hypot(x,y));x/=scale;y/=scale
    center=np.asarray(points['steering wheel center'],dtype=float)
    if hid_substeps not in (1,25):raise ValueError('Unsupported HID resolution')
    quantized=np.rint((center+INPUT_RADIUS*np.array([x,-y]))*hid_substeps)/hid_substeps
    point=tuple(map(int,quantized)) if hid_substeps==1 else tuple(map(float,quantized))
    return point,velocity_for_hid(point,points)


def inverse_input(v,w):
    """Continuous inverse; out-of-envelope requests saturate along their input ray."""
    v=float(v);w=float(w)
    if not math.isfinite(v) or not math.isfinite(w):raise ValueError('Nonfinite velocity')
    x=-w/(INPUT_RADIUS*TURN_GAIN)
    y=v/(INPUT_RADIUS*(FORWARD_GAIN if v>=0 else REVERSE_GAIN))
    scale=max(1.,math.hypot(x,y))
    return np.array([x/scale,y/scale])


def map_velocity(v,w,points,hid_substeps=1):
    """cmd_vel -> integer HID point AND its achievable model velocity.

    Reverse is represented here; callers retain their existing authority limits.
    """
    return map_input(*inverse_input(v,w),points,hid_substeps=hid_substeps)


# Compatibility display anchors, generated from the continuous model, not a lookup control law.
VELOCITIES=np.array([predict_input(math.cos(a),math.sin(a)) for a in np.linspace(0,math.pi,5)])
