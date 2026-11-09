import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from actions import load_trajectory, simplify_trajectory, TeachRecorder, Playback
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
            try:
                for i in range(4):
                    with shared.lock:
                        shared.pose = dict(zip(KEYS, [i*.0005, 0, 1.57, 0, 0, 3.14]))
                        shared.pose_time = time.monotonic()
                    deadline = time.monotonic() + 1
                    while recorder.count <= i and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertEqual(recorder.count, i + 1)
            finally:
                recorder.stop.set()
                recorder.join(2)
            self.assertIsNone(recorder.error)
            saved = json.loads(recorder.path.read_text(encoding='utf-8'))
            self.assertEqual(saved['sample_count'], recorder.count)
            self.assertGreaterEqual(len(load_trajectory(recorder.path)), 2)
            self.assertLess(len(saved['records']), saved['sample_count'])

    def test_teaching_simplification_uses_original_timestamps(self):
        records = [dict(time=t, radians=[t*.2, 0., math.pi/2, 0., 0., math.pi])
                   for t in (0., .08, .23, .32, .5, .7, 1.)]
        self.assertEqual(simplify_trajectory(records), [records[0], records[-1]])

    def test_teaching_simplification_preserves_dwell_and_reversal(self):
        records = [dict(time=i/10, radians=[min(i, 10, 30-i)*.01, 0., math.pi/2, 0., 0., math.pi])
                   for i in range(31)]
        self.assertEqual(simplify_trajectory(records), [records[i] for i in (0, 10, 20, 30)])

    def test_teaching_simplification_keeps_tcp_and_gripper_details(self):
        home = [0., 0., math.pi/2, 0., 0., math.pi]
        # This base angle is within the joint error tolerance, but moves the tool >1 mm.
        for index, delta in ((0, .0035), (5, -.003)):
            with self.subTest(joint=index):
                middle = home.copy()
                middle[index] += delta
                records = [dict(time=0., radians=home), dict(time=.1, radians=middle),
                           dict(time=.2, radians=home)]
                self.assertEqual(simplify_trajectory(records), records)

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
        self.assertTrue(player.loaded_tolerance)
        self.assertTrue(player.degraded)
        worker=Thread(target=simulate,args=(12,));worker.start()
        with self.assertRaisesRegex(RuntimeError,'肩.*12.0'):
            player.wait_target(target,.5)
        worker.join()

    def test_loaded_tolerance_with_small_tcp_error_stays_strict(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        player = Playback(shared, arm, [])
        target = [0., math.radians(-70), math.radians(160), math.radians(30), 0., math.pi]
        observed = [0., math.radians(-65), *target[2:]]
        shared.pose = dict(zip(KEYS, observed))
        now = 1.
        shared.pose_time = now

        def advance(seconds):
            nonlocal now
            now += seconds
            shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), patch.object(player.stop, 'wait', advance):
            player.wait_target(target, 1.)
        self.assertTrue(player.loaded_tolerance)
        self.assertAlmostEqual(player.xyz_error_mm, 6.5364, places=3)
        self.assertFalse(player.degraded)

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

    def test_dense_replay_at_double_speed_keeps_waypoints_and_only_settles_at_ends(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        start = [0., 0., math.pi/2, 0., 0., math.pi]
        shared.pose = dict(zip(KEYS, start))
        records = [dict(time=i/10, radians=[i*.003, *start[1:]]) for i in range(11)]
        player = Playback(shared, arm, records, factor=2)
        now = 0.
        writes = []

        def advance(deadline):
            nonlocal now
            now = max(now, deadline)

        def send(target):
            nonlocal now
            writes.append((now, target.copy()))
            if len(writes) > 1:
                now += .025
            shared.pose = dict(zip(KEYS, target))
            shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), \
             patch.object(player, 'wait_until', side_effect=advance), \
             patch.object(player, 'send', side_effect=send), \
             patch.object(player, 'wait_target') as settle:
            player.run()
        self.assertIsNone(player.error)
        self.assertEqual(settle.call_count, 2)
        self.assertEqual([angles for _, angles in writes], [r['radians'] for r in records])
        self.assertAlmostEqual(writes[-1][0], .5)

    def test_replay_sends_first_frame_as_one_target(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        start = [0., .1, 1.3, 0., 0., math.pi]
        target = [0., .32, 1.42, -.08, 0., math.pi]
        shared.pose = dict(zip(KEYS, start))
        shared.pose_time = time.monotonic()
        player = Playback(shared, arm, [dict(time=0., radians=target)])
        with patch.object(player, 'send') as send, patch.object(player, 'wait_target') as settle:
            player.run()
        self.assertIsNone(player.error)
        send.assert_called_once_with(target)
        settle.assert_called_once()

    def test_replay_pauses_for_lagging_wrist_even_in_degraded_mode(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        start = [0., 0., math.pi/2, 0., 0., math.pi]
        shared.pose = dict(zip(KEYS, start))
        shared.pose_time = time.monotonic()
        records = [dict(time=i, radians=[*start[:3], i*.12, *start[4:]]) for i in range(3)]
        player = Playback(shared, arm, records)
        player.loaded_tolerance = True
        with patch.object(player, 'wait_until'), patch.object(player, 'send') as send, \
             patch.object(player, 'wait_target', side_effect=[None, RuntimeError('手腕未到位')]) as settle:
            player.run()
        self.assertEqual(player.error, '手腕未到位')
        last_target = send.call_args.args[0]
        self.assertGreater(last_target[3], .07)
        self.assertLess(last_target[3], records[-1]['radians'][3])
        self.assertEqual(settle.call_args.args[0], last_target)
        self.assertFalse(arm.playing)


if __name__ == '__main__':
    unittest.main()
