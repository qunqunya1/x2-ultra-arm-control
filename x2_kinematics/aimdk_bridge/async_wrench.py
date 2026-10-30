"""Bounded, nonblocking wrench diagnostics outside the ROS control process.

This worker has no ROS imports, publisher, or access to hardware commands.
Its Pinocchio model/data are private to the spawned process.
"""
import multiprocessing as mp
from queue import Empty, Full

from .dynamics import LiveWaistArmDynamics
from ..dual_wrench import JointFeedback
from ..flange_sensor import VirtualFlangeSensorEstimator


def put_latest(queue, value):
    """Discard backlog rather than making a control callback wait."""
    try:
        queue.put_nowait(value)
        return True
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
            return True
        except Full:
            return False


def _work(config, flange, incoming, outgoing, stop, ready):
    model=LiveWaistArmDynamics(base_gravity=config['base_gravity_m_s2'])
    estimator=VirtualFlangeSensorEstimator(model,
        wrist_from_sensor=flange['wrist_from_sensor'],
        sensor_convention=flange['sensor_convention'],
        calibration_confirmed=flange['calibration_confirmed'],
        window_s=config['panel_wrench_window_s'],max_sample_gap_s=config['panel_wrench_max_gap_s'])
    epoch=None
    ready.set()
    while not stop.is_set():
        try:
            packet=incoming.get(timeout=.05)
        except Empty:
            continue
        current,source,q,dq,tau,waist,gravity=packet
        if current != epoch:
            estimator.reset();epoch=current
        try:
            model.set_waist(waist);model.set_base_gravity(gravity)
            result=estimator.update(JointFeedback(source*1e-9,q,dq,tau))
            if result is not None:
                put_latest(outgoing,dict(epoch=epoch,result=result.as_dict(),error=None))
        except ValueError as error:
            estimator.reset()
            put_latest(outgoing,dict(epoch=epoch,result=None,error=str(error)))


class WrenchWorker:
    def __init__(self, config, flange):
        ctx=mp.get_context('spawn')
        self.incoming=ctx.Queue(maxsize=4)
        self.outgoing=ctx.Queue(maxsize=1)
        self.stop_event=ctx.Event()
        self.ready=ctx.Event()
        self.dropped=0
        self.process=ctx.Process(target=_work,args=(dict(config),flange,self.incoming,self.outgoing,self.stop_event,self.ready),
                                name='x2-wrench-diagnostics',daemon=True)
        self.process.start()

    def submit(self, sample, waist, gravity, epoch):
        packet=(epoch,sample.source_ns,sample.q.copy(),sample.dq.copy(),sample.torque.copy(),waist.copy(),gravity.copy())
        if not put_latest(self.incoming,packet):self.dropped+=1

    def poll(self, epoch):
        latest=None
        while True:
            try:
                value=self.outgoing.get_nowait()
            except Empty:
                return latest
            if value['epoch']==epoch:latest=value

    def close(self):
        self.stop_event.set();self.process.join(timeout=1.)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=1.)
        for queue in (self.incoming,self.outgoing):
            queue.cancel_join_thread();queue.close()
