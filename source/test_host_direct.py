import math,json,time,unittest
from unittest.mock import patch
from host_direct import prepare, holding_offset
from host_control import forward
from backend import all_command, check_radians, Arm, Shared, KEYS
from test_backend import FakeSerial

class FullFeedbackSerial(FakeSerial):
    def write(self,raw):
        cmd=json.loads(raw);self.writes.append(cmd)
        if cmd['T']==105:
            p=dict(T=1051,**dict(zip(KEYS,[0,0,1.57,0,0,3.14])))
            p.update({'torswitch'+k:1 for k in ('B','S','E','T','R','G')})
            self.buffer.extend((json.dumps(p)+'\n').encode())
        return len(raw)

class Tests(unittest.TestCase):
    def test_xyz_every_axis_is_one_identical_native_angle_command(self):
        measured=[-.075,.65,.95,1.18,.007,3.12]
        start=forward(measured)
        for axis in range(3):
            for delta in (-10,1,10):
                goal=start.copy();goal[axis]+=delta
                values=goal[:3]+[math.degrees(v) for v in goal[3:]]
                cmd,q=prepare(values,measured,150,5)
                self.assertEqual(cmd,all_command(q,150,5))
                self.assertEqual(cmd['T'],102)
                self.assertLess(math.dist(forward(q)[:3],goal[:3]),1e-8)
                self.assertEqual(q[4:],measured[4:])
        cmd,_=prepare(start[:3]+[math.degrees(v) for v in start[3:]],measured,73,9)
        self.assertEqual((cmd['spd'],cmd['acc']),(73,9))

    def test_reachable_target_beyond_30mm(self):
        q=[0,.1,1.5,-.05,0,3.14]
        goal=forward(q)
        goal[0]+=45
        _,sent=prepare(goal[:3]+list(map(math.degrees,goal[3:])),q,150,5)
        self.assertLess(math.dist(forward(sent)[:3],goal[:3]),1e-6)

    def test_degraded_xyz_prefers_position_then_approaches_along_requested_direction(self):
        home = [0, 0, math.pi/2, 0, 0, math.pi]
        target = [500, 0, 223.13, 0, 0, 180]
        with self.assertRaises(ValueError):
            prepare(target, home, 150, 5)
        _, joints = prepare(target, home, 150, 5, allow_degraded=True, reference=home)
        reached = forward(joints)
        self.assertLess(math.dist(reached[:3], target[:3]), 1e-6)
        self.assertNotAlmostEqual(reached[3], 0)
        self.assertEqual(joints[4:], home[4:])

        target = [490, 0, 450, 0, 0, 180]
        _, joints = prepare(target, home, 150, 5, allow_degraded=True, reference=home)
        start, reached = forward(home), forward(check_radians(joints))
        fraction = (reached[0] - start[0]) / (target[0] - start[0])
        self.assertTrue(0 < fraction < 1)
        self.assertAlmostEqual(reached[2], start[2] + (target[2] - start[2]) * fraction)

    def test_degraded_xyz_respects_sector_and_preserved_gripper_limits(self):
        home = [0, 0, math.pi/2, 0, 0, math.pi]
        for target, reason in (([-400, 0, 223, 0, 0, 180], '105'),
                               ([490, 0, 450, 0, 0, 181], '夹爪')):
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, reason):
                prepare(target, home, 150, 5, allow_degraded=True, reference=home)

    def test_first_z_jog_preserves_loaded_home_and_following_xy_holds_z(self):
        held=[0,0,math.pi/2,0,0,math.pi]
        actual=[.02,.05,1.60,.02,0,3.13]
        observed=forward(actual)
        desired=observed[:3]+list(map(math.degrees,observed[3:]))
        offset=holding_offset(desired,held)
        desired[2]+=10
        for dx in (0,10):
            desired[0]+=dx
            _,sent=prepare([v+d for v,d in zip(desired,offset)],actual,150,5,max(0,8-offset[2]))
            expected=forward(held)
            expected[0]+=dx
            expected[2]+=10
            self.assertLess(math.dist(forward(sent),expected),1e-6)

    def test_full_50mm_feedforward(self):
        q=[0,.05,1.60,.02,0,3.13]
        pose=forward(q)
        values=pose[:3]+list(map(math.degrees,pose[3:]))
        _,sent=prepare(values,q,150,5,50)
        self.assertAlmostEqual(forward(sent)[2],pose[2]+50)
        with self.assertRaises(ValueError):
            prepare(values,q,150,5,51)

    def test_internal_intent_flag_not_sent_and_manual_angle_invalidates_reference(self):
        shared=Shared()
        with patch('backend.serial.Serial',FullFeedbackSerial):
            arm=Arm(shared,'TEST');arm.start()
            try:
                deadline=time.monotonic()+2
                while shared.pose is None and time.monotonic()<deadline:time.sleep(.01)
                self.assertIsNotNone(shared.pose)
                reference=arm.reference_generation
                cmd=all_command([0,0,1.57,0,0,3.14],150,5)
                arm.submit(dict(cmd,_host_intent=True))
                deadline=time.monotonic()+2
                while arm.command_sequence<1 and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(arm.command_sequence,1)
                self.assertEqual(arm.reference_generation,reference)
                self.assertIn(cmd,FakeSerial.instances[-1].writes)
                self.assertTrue(all('_host_intent' not in c for c in FakeSerial.instances[-1].writes))
                arm.submit(cmd)
                deadline=time.monotonic()+2
                while arm.command_sequence<2 and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(arm.reference_generation,reference+1)
            finally:arm.stop.set();arm.join(2)

if __name__=='__main__':unittest.main()
