#!/usr/bin/env python3
"""Plot the ideal angular profile and measured simulation angle integration."""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
run=here/'simulation_angular_accel_08_0320'
d=json.loads((run/'telemetry.json').read_text())
r=next(x for x in d['results'] if x['case']=='u_turn')
a=.8; limit=.8108854254075847; target=math.pi
ta=min(limit/a,math.sqrt(target/a));peak=a*ta;tc=max(0.,target/peak-ta);total=2*ta+tc
t=np.linspace(0,total,1200)
w=np.where(t<ta,a*t,np.where(t<ta+tc,peak,np.maximum(0,peak-a*(t-ta-tc))))
odom=[x for x in d['series']['odom'] if r['start']['t']<=x['t']<=r['end']['t']+.5]
stamp=np.array([x['stamp'] for x in odom]); elapsed=stamp-stamp[0]
yaw=np.unwrap(np.array([x['yaw'] for x in odom]));direction=1 if yaw[-1]>=yaw[0] else -1
angle=(yaw-yaw[0])*direction
actual=np.array([x['w'] for x in odom])*direction
integral=np.r_[0,np.cumsum((actual[:-1]+actual[1:])*.5*np.diff(stamp))]
fig,ax=plt.subplots(2,1,figsize=(8,6),constrained_layout=True)
ax[0].plot(t,w,color='#4d4d4d',label='Ideal: a = 0.8 rad/s²')
ax[0].fill_between(t,w,color='#87b6d9',alpha=.35)
ax[0].set(xlabel='Time (s)',ylabel='Angular velocity (rad/s)',title='Ideal 180° turn: area = π rad')
ax[0].axvline(ta,color='#aaaaaa',lw=.7,ls=':');ax[0].axvline(ta+tc,color='#aaaaaa',lw=.7,ls=':')
ax[0].legend(loc='upper right');ax[0].grid(alpha=.2)
ax[1].plot(elapsed,np.degrees(angle),label='Measured pose rotation',color='#126b9d')
ax[1].plot(elapsed,np.degrees(integral),'--',label='Integral of measured angular velocity',color='#d97722')
ax[1].axhline(180,color='#777777',ls=':',label='180° target')
ax[1].set(xlabel='Simulation timestamp elapsed (s)',ylabel='Angle (deg)',title='Simulator feedback: angle and velocity integral')
ax[1].grid(alpha=.2);ax[1].legend(loc='lower right')
fig.savefig(here/'angular_profile_08.png',dpi=150)
report={'angular_acceleration':a,'angular_speed_limit':limit,'ideal_accel_s':ta,'ideal_cruise_s':tc,
        'ideal_total_s':total,'triangle_threshold_deg':math.degrees(limit*limit/a),
        'measured_pose_angle_deg':float(np.degrees(angle[-1])),
        'integrated_measured_angular_velocity_deg':float(np.degrees(integral[-1])),
        'integration_pose_difference_deg':float(np.degrees(integral[-1]-angle[-1])),
        'goal_result_yaw_error_deg':math.degrees(r['yaw_error_rad']),
        'goal_result_duration_sim_s':r['duration_s'],
        'scope':'ideal formula and one isolated simulator result; not real-chair timing validation'}
(here/'angular_profile_08.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
