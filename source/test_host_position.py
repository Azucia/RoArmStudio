import math,threading,time,unittest
from backend import Shared,KEYS
from host_control import forward
from host_position import HostPosition

class SimulatedArm:
    def __init__(self,shared):
        self.shared=shared;self.command_lock=threading.RLock();self.stop=threading.Event()
        self.connected=True;self.teaching=False;self.torque_pending=False;self.torque_fault=False
        self.epoch=0;self.playing=False;self.host_busy=False;self.command_sequence=0;self.sent=[]
        self.actual=[0,.1,1.5,-.05,.01,3.15]
        self.bias=[0,-.02,-.05,.003,0,0]
        self.last_joint_target=[a+b for a,b in zip(self.actual,self.bias)]
        self.publish()
    def publish(self):
        with self.shared.lock:
            pose=dict(T=1051,**dict(zip(KEYS,self.actual)),v=1144,tE=100,tT=0,tS=50)
            pose.update({'torswitch'+k:1 for k in ('B','S','E','T','R','G')})
            self.shared.pose=pose;self.shared.pose_time=time.monotonic();self.shared.pose_host_ns=time.time_ns()
    def cancel_replay(self):self.epoch+=1;self.playing=False;self.host_busy=False
    def begin_replay(self):self.cancel_replay();self.playing=True;return self.epoch
    def submit(self,c,token):
        assert token==self.epoch and self.playing
        self.sent.append(c)
        if c['T']==102:self.last_joint_target=[c[k] for k in ('base','shoulder','elbow','wrist','roll','hand')]
        elif c['T']==101:self.last_joint_target[c['joint']-1]=c['rad']
        self.actual=[a-b for a,b in zip(self.last_joint_target,self.bias)]
        self.command_sequence+=1;self.publish()

class Tests(unittest.TestCase):
    def values(self,arm):
        p=forward(arm.actual);p[2]+=5
        return p[:3]+[math.degrees(v) for v in p[3:]]
    def test_missing_reference_never_reserves_or_moves(self):
        shared=Shared();arm=SimulatedArm(shared);arm.last_joint_target=None
        with self.assertRaisesRegex(ValueError,'基准'):HostPosition(shared,arm,self.values(arm))
        self.assertEqual(arm.sent,[]);self.assertFalse(arm.playing)
    def test_cancelled_owner_never_moves(self):
        shared=Shared();arm=SimulatedArm(shared)
        worker=HostPosition(shared,arm,self.values(arm));arm.cancel_replay();worker.run()
        self.assertEqual(arm.sent,[]);self.assertFalse(arm.host_busy)
    def test_loaded_feedback_uses_only_joint_commands_and_reaches_target(self):
        shared=Shared();arm=SimulatedArm(shared);values=self.values(arm)
        worker=HostPosition(shared,arm,values);finished=threading.Event()
        def publish():
            while not finished.wait(.08):arm.publish()
        thread=threading.Thread(target=publish);thread.start()
        try:worker.run()
        finally:finished.set();thread.join()
        self.assertIsNone(worker.error)
        self.assertTrue(arm.sent);self.assertTrue(all(c['T'] in (101,102) for c in arm.sent))
        self.assertLess(math.dist(forward(arm.actual)[:3],values[:3]),1e-6)
        self.assertAlmostEqual(arm.last_joint_target[5],3.15)
        self.assertFalse(arm.host_busy);self.assertFalse(arm.playing)

if __name__=='__main__':unittest.main()
