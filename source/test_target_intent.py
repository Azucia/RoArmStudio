import unittest,time
from unittest.mock import patch
from PySide6.QtWidgets import QApplication
from app import Window
from host_control import TargetIntent,forward
from backend import all_command
import math
from test_jogpad import DummyArm

APP=QApplication.instance() or QApplication([])

class Tests(unittest.TestCase):
    def test_measurement_error_does_not_ratchet_user_height(self):
        intent=TargetIntent();key=(1,0);actual=[350,0,200,0,0,180]
        for i in range(10):
            target=intent.values(actual,key);target[1]+=1;intent.commit(target,key)
            actual[2]-=6
        self.assertEqual(intent.goal[:3],[350,10,200])
    def test_changed_reference_and_explicit_resync_use_measurement(self):
        intent=TargetIntent();intent.commit([350,0,200,0,0,180],(1,0))
        actual=[340,2,190,0,0,180]
        self.assertEqual(intent.values(actual,(1,1)),actual)
        intent.clear();self.assertEqual(intent.values(actual,(1,0)),actual)
    def test_ui_direct_route_uses_same_all_joint_interface_and_retains_goal(self):
        w=Window();w.arm=DummyArm();errors=[];w.error=lambda e:errors.append(str(e))
        q=[0,.1,1.5,-.05,0,3.14];xyz=forward(q)
        w.arm.last_joint_target=q.copy()
        w.shared.pose=dict(T=1051,**dict(zip(('b','s','e','t','r','g'),q)),
                          x=xyz[0],y=xyz[1],z=xyz[2],tit=xyz[3])
        w.shared.pose_time=time.monotonic();w.cartesian_pad.computer_mode.setChecked(True)
        try:
            with patch.object(w,'start_worker',side_effect=AssertionError('No trajectory worker')):
                w.cartesian_pad.move(1,1)
                w.shared.pose.update(z=xyz[2]-6,y=.3)
                w.cartesian_pad.move(1,1)
            self.assertEqual(len(w.arm.sent),2)
            sent=w.arm.sent[-1].copy();self.assertTrue(sent.pop('_host_intent'))
            actual_q=[sent[k] for k in ('base','shoulder','elbow','wrist','roll','hand')]
            self.assertEqual(sent,all_command(actual_q,w.speed.value(),w.acc.value()))
            target=forward(actual_q)
            self.assertAlmostEqual(target[1],xyz[1]+2)
            self.assertAlmostEqual(target[2],xyz[2])
            self.assertEqual(w.cartesian_pad.target_intent.goal[:3],[xyz[0],xyz[1]+2,xyz[2]])
            self.assertEqual(errors,[])
        finally:w.close()

    def test_degraded_badge_survives_refresh_and_reverse_jog_uses_accepted_target(self):
        w = Window()
        self.addCleanup(w.close)
        w.arm = DummyArm()
        errors = []
        w.error = lambda error: errors.append(str(error))
        home = [0, 0, math.pi/2, 0, 0, math.pi]
        x, y, z, pitch, _, _ = forward(home)
        w.arm.last_joint_target = home.copy()
        w.shared.pose = dict(zip(('b','s','e','t','r','g'), home), T=1051, x=x, y=y, z=z, tit=pitch)
        w.shared.pose_time = time.monotonic()
        pad = w.cartesian_pad
        pad.send_xyz([490, 0, 450, 0, 0, 180], [0, 0, 180])
        w.tick()
        self.assertEqual(pad.execution_mode.objectName(), 'degradedMode')
        accepted = pad.target_intent.goal.copy()
        self.assertLess(accepted[0], 490)
        pad.move(0, -1)
        self.assertEqual(len(w.arm.sent), 2)
        self.assertAlmostEqual(pad.target_intent.goal[0], accepted[0] - 1)
        pad.send_xyz([x, y, z, 0, 0, 180], [0, 0, 180])
        w.tick()
        self.assertEqual(pad.execution_mode.objectName(), 'strictMode')
        self.assertEqual(errors, [])

if __name__=='__main__':unittest.main()
