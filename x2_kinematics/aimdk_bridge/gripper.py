"""OmniPicker HAL adapter. Opening and drive settings are normalized 0..1."""
import math
from .protocol import stamp_ns

SIDES=('left','right')


def normalized(value):
    if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('夹爪目标必须在 0%～100% 之间')
    return float(value)


class GripperFeedback:
    def __init__(self,timeout_s=.5):
        self.timeout_s=timeout_s;self.received_ns=self.source_ns=None;self.sides={}

    def update(self,msg,now_ns,ros_now_ns):
        try:
            source=stamp_ns(msg)
            if not -.01 <= (ros_now_ns-source)*1e-9 <= self.timeout_s:raise ValueError('夹爪反馈时间戳过期')
            if self.source_ns is not None and source <= self.source_ns:raise ValueError('夹爪反馈时间戳不递增')
            sides={}
            for side in SIDES:
                joints=getattr(msg,side+'_hands');kind=int(getattr(msg,side+'_hand_type').value)
                value=dict(position=None,state=None,faultcode=None,type=kind,valid=False,reason='当前末端不是 OmniPicker')
                if kind==2 and len(joints)==1:
                    j=joints[0];p=normalized(float(j.position))
                    value.update(position=p,state=int(j.state),faultcode=int(j.faultcode),name=str(j.name),
                                 valid=int(j.faultcode)==0,reason=None if int(j.faultcode)==0 else '夹爪故障码 '+str(j.faultcode))
                sides[side]=value
            self.received_ns,self.source_ns,self.sides=int(now_ns),source,sides
        except (ValueError,AttributeError) as error:
            self.sides={side:dict(position=None,state=None,faultcode=None,type=None,valid=False,reason=str(error)) for side in SIDES}

    def status(self,now_ns,ros_now_ns):
        age=None if self.received_ns is None else (now_ns-self.received_ns)*1e-6
        source_age=None if self.source_ns is None else (ros_now_ns-self.source_ns)*1e-6
        fresh=bool(age is not None and 0<=age<=self.timeout_s*1000 and source_age is not None and -10<=source_age<=self.timeout_s*1000)
        sides={}
        for side in SIDES:
            value=dict(self.sides.get(side,dict(position=None,valid=False,reason='等待夹爪反馈',state=None,faultcode=None)))
            value['fresh']=fresh;value['ready']=bool(fresh and value['valid'])
            if value['valid'] and not fresh:value['reason']='夹爪反馈已过期'
            sides[side]=value
        return dict(age_ms=age,source_age_ms=source_age,timeout_ms=self.timeout_s*1000,sides=sides)

    def require(self,side,opening,now_ns,ros_now_ns):
        if side not in SIDES:raise ValueError('未知夹爪')
        opening=normalized(opening)
        current=self.status(now_ns,ros_now_ns)['sides'][side]
        if not current['ready']:raise ValueError(current['reason'] or '夹爪尚不可用')
        return opening


def build_gripper_command(array_type,joint_type,hand_type,side,opening,source_ns,*,sequence=0,drive=None,name=None):
    if side not in SIDES:raise ValueError('未知夹爪')
    opening=normalized(opening)
    settings=dict(velocity=.2,acceleration=.2,deceleration=.2,effort=.2)
    settings.update(drive or {})
    msg=array_type();msg.header.stamp.sec=int(source_ns//1_000_000_000);msg.header.stamp.nanosec=int(source_ns%1_000_000_000)
    if hasattr(msg.header,'sequence'):msg.header.sequence=int(sequence)
    joint=joint_type();joint.name=name or side+'_claw_joint';joint.position=opening
    for key,value in settings.items():
        if key not in ('velocity','acceleration','deceleration','effort'):raise ValueError('未知夹爪参数')
        setattr(joint,key,normalized(value))
    # NONE + empty array leaves the other hand untouched (vendor HAL API).
    for other in SIDES:
        kind=hand_type();kind.value=2 if other==side else 0
        setattr(msg,other+'_hand_type',kind)
        setattr(msg,other+'_hands',[joint] if other==side else [])
    return msg
