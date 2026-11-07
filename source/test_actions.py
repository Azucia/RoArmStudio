import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from actions import load_trajectory, TeachRecorder, Playback
from backend import Shared, Arm, KEYS, pose_command, public_command, all_command
from test_backend import FakeSerial


class MotionSerial(FakeSerial):
    def __init__(self):
        super().__init__()
        self.values = [0, 0, math.pi/2, 0, 0, math.pi]

    def write(self, raw):
        cmd = json.loads(raw)
        if cmd['T'] == 102:
            self.values = [cmd[k] for k in ('base', 'shoulder', 'elbow', 'wrist', 'roll', 'hand')]
        self.writes.append(cmd)
        if cmd['T'] == 105:
            pose = dict(T=1051, **dict(zip(KEYS, self.values)))
            pose.update({'torswitch'+k: 1 for k in ('B', 'S', 'E', 'T', 'R', 'G')})
            self.buffer.extend((json.dumps(pose)+'\n').encode())
        return len(raw)


class ActionTests(unittest.TestCase):
    def test_pose_units_and_secret_redaction(self):
        cmd = pose_command([235, 0, 234, 45, -90, 180])
        self.assertEqual(cmd['T'], 1041)
        self.assertAlmostEqual(cmd['t'], math.pi/4)
        self.assertAlmostEqual(cmd['g'], math.pi)
        self.assertEqual(public_command(dict(T=404, ap_password='secret'))['ap_password'], '***')
        with self.assertRaises(ValueError):
            pose_command([900, 0, 0, 0, 0, 180])

    def test_sdk_import_and_invalid_times(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'sdk.json'
            rows = [dict(timestamped=100+i, radians=[0, 0, 1.57, 0, 0, .2]) for i in range(2)]
            path.write_text(json.dumps(rows))
            result = load_trajectory(path, sdk=True)
            self.assertAlmostEqual(result[0]['radians'][-1], math.pi-.2)
            self.assertEqual(result[-1]['time'], 1)
            with self.assertRaises(ValueError):
                load_trajectory(path)
            rows[1]['timestamped'] = 100
            path.write_text(json.dumps(rows))
            with self.assertRaises(ValueError):
                load_trajectory(path, sdk=True)

    def test_teach_record_and_load(self):
        shared = Shared()
        with tempfile.TemporaryDirectory() as temp:
            recorder = TeachRecorder(shared, Path(temp)/'动作.json')
            recorder.start()
            for i in range(4):
                with shared.lock:
                    shared.pose = dict(zip(KEYS, [i*.01, 0, 1.57, 0, 0, 3.14]))
                    shared.pose_time = time.monotonic()
                time.sleep(.12)
            recorder.stop.set()
            recorder.join(2)
            self.assertIsNone(recorder.error)
            self.assertGreaterEqual(len(load_trajectory(recorder.path)), 2)

    def test_loaded_shoulder_must_settle_and_large_error_still_stops(self):
        from threading import Thread
        shared=Shared()
        arm=Arm(shared,'TEST')
        arm.connected=True
        player=Playback(shared,arm,[])
        target=[0,0,1.57,0,0,3.14]
        def simulate(degrees):
            for _ in range(9):
                with shared.lock:
                    shared.pose=dict(zip(KEYS,target))
                    shared.pose['s']=math.radians(degrees)
                    shared.pose_time=time.monotonic()
                time.sleep(.075)
        worker=Thread(target=simulate,args=(5,));worker.start()
        player.wait_target(target,1.5)
        worker.join()
        worker=Thread(target=simulate,args=(12,));worker.start()
        with self.assertRaisesRegex(RuntimeError,'肩.*12.0'):
            player.wait_target(target,.5)
        worker.join()

    def test_replay_and_cancel_token(self):
        shared = Shared()
        with patch('backend.serial.Serial', MotionSerial):
            arm = Arm(shared, 'TEST')
            arm.start()
            end = time.monotonic()+2
            while not shared.pose and time.monotonic() < end:
                time.sleep(.01)
            values = [shared.pose[k] for k in KEYS]
            target = list(values)
            target[0] = .02
            player = Playback(shared, arm, [dict(time=0, radians=values), dict(time=.1, radians=target)])
            player.start()
            player.join(3)
            self.assertFalse(player.is_alive())
            self.assertIsNone(player.error)
            self.assertEqual(player.progress, 1.)
            self.assertEqual(arm.last_joint_target, target)
            self.assertTrue(any(row.get('T') == 102 and row['base'] == target[0]
                                for row in FakeSerial.instances[-1].writes))
            token = arm.begin_replay()
            arm.cancel_replay()
            with self.assertRaises(ValueError):
                arm.submit(all_command(values), token=token)
            arm.stop.set()
            arm.join(2)
            self.assertFalse(any('_epoch' in row for row in FakeSerial.instances[-1].writes))


if __name__ == '__main__':
    unittest.main()
