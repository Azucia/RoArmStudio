import json
import time
import unittest
from unittest.mock import patch

from backend import Arm, Shared, TorqueChange, joint_command
from test_backend import FakeSerial


class SafeTorqueSerial(FakeSerial):
    bad_release = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    @in_waiting.setter
    def in_waiting(self, value):
        pass

    def __init__(self):
        super().__init__()
        self.states = [1]*7

    def write(self, raw):
        cmd = json.loads(raw)
        if cmd['T'] == 105:
            self.writes.append(cmd)
            pose=dict(T=1051,b=0,s=0,e=1.57,t=0,r=0,g=3.14)
            for key,i in zip(('torswitchB','torswitchS','torswitchE','torswitchT','torswitchR','torswitchG'),(0,1,3,4,5,6)):
                pose[key]=self.states[i]
            self.buffer.extend((json.dumps(pose)+'\n').encode())
            return len(raw)
        if cmd['T'] != 211:
            return super().write(raw)
        self.writes.append(cmd)
        if cmd['cmd'] == 0:
            self.states = [0]*7
            if self.bad_release:
                self.states[2] = 1  # The second shoulder servo must also be off.
        elif cmd['cmd'] == 1:
            self.states = [1]*7
        reply = dict(T=2111, protocol='studio-teach-v1', request=cmd['request'],
                     ok=True, states=self.states.copy())
        self.buffer.extend((json.dumps(reply)+'\n').encode())
        return len(raw)


class TorqueTests(unittest.TestCase):
    def connect(self, serial_type):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.start()
        end = time.monotonic()+2
        while shared.pose is None and time.monotonic()<end:
            time.sleep(.01)
        self.assertIsNotNone(shared.pose)
        self.addCleanup(lambda: (arm.stop.set(), arm.join(2)))
        return shared, arm, FakeSerial.instances[-1]

    def test_unpatched_firmware_never_receives_legacy_release_or_hold(self):
        with patch('backend.serial.Serial', FakeSerial):
            shared, arm, serial = self.connect(FakeSerial)
            with self.assertRaises(ValueError):
                arm.submit(dict(T=210, cmd=0))
            change = TorqueChange(shared, arm, True)
            with self.assertRaises(ValueError):
                arm.submit(joint_command(0, 1))
            change.start()
            change.join(4)
            self.assertFalse(change.is_alive())
            self.assertIsNotNone(change.error)
            self.assertFalse(arm.teaching)
            self.assertTrue(all(c['T'] in (105,211) for c in serial.writes))
            self.assertEqual([c['cmd'] for c in serial.writes if c['T']==211],[2])

    def test_release_and_restore_require_seven_servo_readbacks(self):
        with patch('backend.serial.Serial', SafeTorqueSerial):
            shared, arm, serial = self.connect(SafeTorqueSerial)
            change=TorqueChange(shared,arm,True)
            change.start(); change.join(4)
            self.assertIsNone(change.error)
            self.assertTrue(arm.teaching)
            self.assertFalse(arm.torque_pending)
            self.assertFalse(arm.torque_fault)
            with self.assertRaises(ValueError):
                arm.submit(joint_command(0,1))
            change=TorqueChange(shared,arm,False)
            change.start(); change.join(4)
            self.assertIsNone(change.error)
            self.assertFalse(arm.teaching)
            self.assertEqual([c['cmd'] for c in serial.writes if c['T']==211],[2,0,2,1])
            self.assertFalse(any(c['T'] in (101,102,1041,210) for c in serial.writes))

    def test_partial_release_latches_motion_inhibit(self):
        class Partial(SafeTorqueSerial):
            bad_release=True
        with patch('backend.serial.Serial', Partial):
            shared, arm, serial = self.connect(Partial)
            change=TorqueChange(shared,arm,True)
            change.start(); change.join(4)
            self.assertIsNotNone(change.error)
            self.assertFalse(arm.teaching)
            self.assertTrue(arm.torque_fault)
            with self.assertRaises(ValueError):
                arm.submit(joint_command(0,1))
            self.assertFalse(any(c['T']==210 or (c['T']==211 and c['cmd']==1) for c in serial.writes))

    def test_unexpected_torque_enable_during_teaching_blocks_motion(self):
        with patch('backend.serial.Serial', SafeTorqueSerial):
            shared, arm, serial = self.connect(SafeTorqueSerial)
            change=TorqueChange(shared,arm,True)
            change.start(); change.join(4)
            self.assertIsNone(change.error)
            serial.states[0]=1
            end=time.monotonic()+2
            while not arm.torque_fault and time.monotonic()<end:
                time.sleep(.02)
            self.assertTrue(arm.torque_fault)
            with self.assertRaises(ValueError): arm.submit(joint_command(0,1))
            self.assertFalse(any(c['T']==211 and c['cmd']==1 for c in serial.writes))


if __name__ == '__main__':
    unittest.main()
