"""Pure joint damping commands: no stiffness, motion reference, or feedforward."""
import numpy as np
from .protocol import ARM_NAMES
from .body_feedback import GROUPS

NAMES = dict(GROUPS, arm=ARM_NAMES, head=('head_yaw_joint','head_pitch_joint'))
KD = dict(arm=np.array([1.5,1.5,1,1,.5,.5,.5]*2),
          waist=np.full(3,2.),leg=np.array([2,2,1.5,2,1,1]*2),head=np.full(2,.5))


def damping_plan(group, q=None):
    size=len(NAMES[group])
    q=np.zeros(size) if q is None else np.asarray(q,float)
    if q.shape!=(size,) or not np.isfinite(q).all():q=np.zeros(size)
    return dict(q=q.copy(),velocity=np.zeros(size),kp=np.zeros(size),kd=KD[group].copy(),extra=np.zeros(size))


def group_command(Array,Joint,group,plan,stamp,sequence):
    command=Array()
    command.header.stamp.sec,command.header.stamp.nanosec=divmod(int(stamp),10**9)
    if hasattr(command.header,'sequence'):command.header.sequence=sequence
    if hasattr(command.header,'frame_id'):command.header.frame_id='x2_damping_stop'
    for i,name in enumerate(NAMES[group]):
        j=Joint();j.name=name
        for field,key in [('position','q'),('velocity','velocity'),('stiffness','kp'),('damping','kd'),('effort','extra')]:
            setattr(j,field,float(plan[key][i]))
        command.joints.append(j)
    return command
