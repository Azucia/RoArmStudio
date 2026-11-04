import math, unittest
from host_control import forward,inverse
from host_trajectory import plan,needs_pause
from host_position import HostPosition
from backend import Shared
from test_host_position import SimulatedArm

class FastWorker(HostPosition):
    def settled(self):
        self.ensure();self.arm.publish();return self.arm.actual.copy()
    def paced_feedback(self,interval,stamp):
        self.arm.publish();return self.snapshot()

class Tests(unittest.TestCase):
    def test_all_axes_and_reversals_reference_rates_endpoints_and_fixed_other_axes(self):
        start=forward([0,.1,1.5,-.05,.01,3.15])
        for axis in range(3):
            for sign in (-1,1):
                goal=start.copy();goal[axis]+=sign*10
                samples=plan(start,goal,[0,-.02,-.05,.003],start[4:])
                self.assertEqual(samples[-1][1],goal)
                previous=inverse(start);velocity=[0]*4
                displacements=[]
                for dt,pose,q,motor in samples:
                    for j in range(3):
                        if j!=axis:self.assertEqual(pose[j],start[j])
                    v=[(q[i]-previous[i])/dt for i in range(4)]
                    a=[(v[i]-velocity[i])/dt for i in range(4)]
                    self.assertLessEqual(max(map(abs,v)),math.radians(1)+1e-10)
                    self.assertLessEqual(max(map(abs,a)),math.radians(2)+1e-10)
                    self.assertEqual(motor[4:],start[4:])
                    displacements.append(abs(pose[axis]-forward(previous)[axis]))
                    previous=q;velocity=v
                self.assertLess(displacements[0],max(displacements)/2)
                self.assertLess(displacements[-1],max(displacements)/2)

    def test_full_path_preflight_rejects_unreachable_goal_and_motor_bias_limit(self):
        start=forward([0,.1,1.5,-.05,.01,3.15]);goal=start.copy();goal[0]=1000
        with self.assertRaises(ValueError):plan(start,goal,[0]*4,start[4:])
        with self.assertRaises(ValueError):plan(start,start,[0,.1,0,0],start[4:])

    def test_joint_lag_gate_detects_cancellation_hidden_by_pitch(self):
        q=[0,.1,1.5,-.05,.01,3.15];actual=q.copy()
        actual[1]+=math.radians(.5);actual[2]-=math.radians(.5)
        self.assertTrue(needs_pause(q,actual))

    def test_y_1mm_under_loaded_simulation_does_not_change_goal_z(self):
        shared=Shared();arm=SimulatedArm(shared)
        start=forward(arm.actual);goal=start.copy();goal[1]+=1
        worker=FastWorker(shared,arm,goal[:3]+[math.degrees(v) for v in goal[3:]])
        worker.execute()
        self.assertLess(math.dist(forward(arm.actual)[:3],goal[:3]),1e-8)
        self.assertEqual(worker.goal[2],start[2])
        self.assertGreater(len(arm.sent),2)
        self.assertTrue(all(c['T']==102 for c in arm.sent))

    def test_stuck_elbow_pauses_reference_then_aborts_without_xyz_fallback(self):
        shared=Shared();arm=SimulatedArm(shared)
        original_submit=arm.submit;stuck=arm.actual[2]
        def submit(c,token):
            original_submit(c,token);arm.actual[2]=stuck;arm.publish()
        arm.submit=submit
        goal=forward(arm.actual);goal[2]+=10
        worker=FastWorker(shared,arm,goal[:3]+[math.degrees(v) for v in goal[3:]])
        events=[];worker.record=lambda kind,**data:events.append((kind,data))
        with self.assertRaisesRegex(RuntimeError,'4°|预算'):worker.execute()
        self.assertTrue(any(k=='tracking_pause' for k,d in events))
        first=next(i for i,c in enumerate(arm.sent) if c['T']==101)
        self.assertTrue(all(c['T']==101 for c in arm.sent[first:]))
        self.assertTrue(all(c['T'] in (101,102) for c in arm.sent))
        self.assertEqual(worker.goal,goal)

    def test_initial_bias_not_silently_clipped_into_a_first_step(self):
        shared=Shared();arm=SimulatedArm(shared)
        arm.last_joint_target[2]=arm.actual[2]-.071
        goal=forward(arm.actual);goal[1]+=1
        worker=FastWorker(shared,arm,goal[:3]+[math.degrees(v) for v in goal[3:]])
        with self.assertRaisesRegex(RuntimeError,'4°'):worker.execute()
        self.assertEqual(arm.sent,[])

    def test_pause_resume_preserves_motor_target_without_adding_residual_sag(self):
        shared=Shared();arm=SimulatedArm(shared);original_submit=arm.submit
        changed=False
        def submit(c,token):
            nonlocal changed
            if not changed:
                arm.bias[1]+=.006;changed=True
            original_submit(c,token)
        arm.submit=submit
        goal=forward(arm.actual);goal[1]+=5
        worker=FastWorker(shared,arm,goal[:3]+[math.degrees(v) for v in goal[3:]])
        events=[];worker.record=lambda kind,**data:events.append((kind,data))
        worker.execute()
        plans=[i for i,(k,d) in enumerate(events) if k=='plan']
        self.assertGreater(len(plans),1)
        for index in plans[1:]:
            before=next(d['command'] for k,d in reversed(events[:index]) if k=='submit')
            after=next(d['command'] for k,d in events[index+1:] if k=='submit')
            # At each restarted reference, the preceding command belongs to the
            # final corrected joint, and the first new coordinated command must
            # not silently add the remaining sag to that motor's offset.
            if before['T']==101:
                field=('base','shoulder','elbow','wrist')[before['joint']-1]
                self.assertLess(abs(after[field]-before['rad']),math.radians(.04))
        self.assertEqual(worker.goal,goal)

if __name__=='__main__':unittest.main()
