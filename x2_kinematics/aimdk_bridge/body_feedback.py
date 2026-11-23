"""Joint groups and SDK feedback decoding for the web panel."""

import numpy as np


LEG_NAMES=tuple(side+'_'+part+'_joint' for side in ('left','right') for part in
    ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll'))
WAIST_NAMES=('waist_yaw_joint','waist_pitch_joint','waist_roll_joint')
GROUPS=dict(leg=LEG_NAMES,waist=WAIST_NAMES)


def decode(message,names):
    if message.state.value!=0:raise ValueError('Joint group fault')
    lookup={j.name if j.name.endswith('_joint') else j.name+'_joint':j for j in message.joints}
    if set(lookup)!=set(names) or len(message.joints)!=len(names):raise ValueError('Unexpected body joint names')
    joints=[lookup[n] for n in names]
    if any(j.error_code!=0 for j in joints):raise ValueError('Joint motor fault')
    values=np.array([[j.position,j.velocity,j.effort] for j in joints],float)
    if not np.isfinite(values).all():raise ValueError('Nonfinite body feedback')
    return values
