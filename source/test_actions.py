import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from actions import load_trajectory, simplify_trajectory, TeachRecorder, Playback, ReplayLoadBias
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
    def test_replay_preloads_grip_only_on_close_and_keeps_native_limit(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        pose = [0., .1, 1.5, 0., 0., math.radians(125)]
        shared.pose = dict(zip(KEYS, pose))
        shared.pose_time = time.monotonic()
        player = Playback(shared, arm, [])
        receipt = SimpleNamespace(done=SimpleNamespace(wait=lambda _: True), error=None)
        with patch.object(arm, 'submit', return_value=receipt) as submit:
            for nominal, sent in ((125, 125), (145, 155), (145, 155), (175, 180), (160, 160)):
                player.send([*pose[:-1], math.radians(nominal)])
                command = submit.call_args.args[0]
                self.assertAlmostEqual(math.degrees(command['hand']), sent)
                self.assertEqual([command[k] for k in ('base', 'shoulder', 'elbow', 'wrist', 'roll')], pose[:-1])

    def test_held_object_does_not_slow_replay_or_fail_endpoint(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        target = [0., .1, 1.5, 0., 0., math.pi]
        shared.pose = dict(zip(KEYS, [*target[:-1], math.radians(135)]))
        now = 1.
        shared.pose_time = now
        player = Playback(shared, arm, [])

        def feedback(seconds):
            nonlocal now
            now += seconds
            shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), \
             patch.object(player.stop, 'wait', feedback):
            player.check_tracking(target)
            feedback(.3)
            player.check_tracking(target)
            self.assertEqual(player.tracking_rate, 1.)
            player.wait_target(target, 1., final=True)
        self.assertAlmostEqual(player.xyz_error_mm, 0.)

    def test_replay_bias_only_learns_at_rest_and_fades_after_grip_change(self):
        bias = ReplayLoadBias()
        nominal = (0., .1, 1.5, 0., 0., 3.14)
        sag = math.radians(2)
        for frame in range(12):
            moving = (0., .1 + math.radians(frame), *nominal[2:])
            observed = (0., moving[1] - sag, moving[2] + sag, *moving[3:])
            self.assertEqual(bias.target(moving, observed, frame * .1), moving)

        observed = (0., nominal[1] - sag, nominal[2] + sag, *nominal[3:])
        for frame in range(9):
            corrected = bias.target(nominal, observed, 2 + frame * .1)
        self.assertAlmostEqual(math.degrees(corrected[1] - nominal[1]), 1.)
        self.assertAlmostEqual(math.degrees(corrected[2] - nominal[2]), -1.)

        nearby = (0., .1 + math.radians(3), *nominal[2:])
        corrected = bias.target(nearby, observed, 3.)
        self.assertAlmostEqual(math.degrees(corrected[1] - nearby[1]), .8)
        self.assertEqual(corrected[3:], nearby[3:])

        opening = (*nearby[:5], 3.0)
        for frame in range(4):
            corrected = bias.target(opening, observed, 3.1 + frame * .05)
        self.assertEqual(corrected, opening)

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

    def test_teach_wrist_clip_is_disclosed_and_larger_excursion_preserves_saved_slot(self):
        shared = Shared()
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'teach-01.json'
            recorder = TeachRecorder(shared, path)
            recorder.start()
            try:
                for count, wrist in enumerate((-89., -92.4609375, -88.), start=1):
                    with shared.lock:
                        shared.pose = dict(zip(KEYS, [0., .2, 1.3, math.radians(wrist), 0., math.pi]))
                        shared.pose_time = time.monotonic()
                    deadline = time.monotonic() + 1
                    while recorder.count < count and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertEqual(recorder.count, count)
            finally:
                recorder.stop.set()
                recorder.join(2)
            self.assertIsNone(recorder.error)
            records, metadata = load_trajectory(path, include_metadata=True)
            self.assertEqual(min(row['radians'][3] for row in records), -math.pi/2)
            self.assertAlmostEqual(metadata['wrist_clip_deg'], 2.4609375)

            original = path.read_bytes()
            with shared.lock:
                shared.pose['t'] = math.radians(-94)
                shared.pose_time = time.monotonic()
            rejected = TeachRecorder(shared, path)
            rejected.start()
            try:
                rejected.join(1)
                self.assertIn('-94.00°', rejected.error)
                self.assertEqual(path.read_bytes(), original)
            finally:
                rejected.stop.set()
                rejected.join(2)

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
            player.wait_target(target, 1., final=True)
        self.assertTrue(player.loaded_tolerance)
        self.assertAlmostEqual(player.xyz_error_mm, 6.5364, places=3)
        self.assertFalse(player.degraded)

    def test_endpoint_rejects_large_tcp_miss_even_inside_joint_tolerances(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        target = [0., 0., math.pi/2, 0., 0., math.pi]
        observed = target.copy()
        observed[1] += math.radians(3.52)
        observed[2] += math.radians(2.11)
        shared.pose = dict(zip(KEYS, observed))
        player = Playback(shared, arm, [])
        now = 1.
        shared.pose_time = now

        def advance(seconds):
            nonlocal now
            now += seconds
            shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), patch.object(player.stop, 'wait', advance):
            with self.assertRaisesRegex(RuntimeError, '示教终点未到位.*mm'):
                player.wait_target(target, 1., final=True)
        self.assertGreater(player.xyz_error_mm, 10)

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
        settle.assert_called_with(records[-1]['radians'], 5, final=True)
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

    def test_replay_slows_then_aborts_for_sustained_wrist_lag(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        start = [0., 0., math.pi/2, 0., 0., math.pi]
        shared.pose = dict(zip(KEYS, start))
        now = 0.
        shared.pose_time = now
        records = [dict(time=i, radians=[*start[:3], i*.14, *start[4:]]) for i in range(4)]
        player = Playback(shared, arm, records)
        player.loaded_tolerance = True
        def advance(deadline):
            nonlocal now
            now = deadline
            shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), \
             patch.object(player, 'wait_until', side_effect=advance), \
             patch.object(player.stop, 'wait', side_effect=lambda seconds: advance(now + seconds)), \
             patch.object(player, 'send') as send, patch.object(player, 'wait_target') as settle:
            player.run()
        self.assertIn('示教途中跟随超差：腕部俯仰', player.error)
        self.assertEqual(player.tracking_rate, .5)
        last_target = send.call_args.args[0]
        self.assertGreater(last_target[3], .14)
        self.assertLess(last_target[3], records[-1]['radians'][3])
        settle.assert_called_once()
        self.assertFalse(arm.playing)

    def test_tracking_recovers_only_after_two_fresh_frames_without_writing(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        target = [0., 0., math.pi/2, 0., 0., math.pi]
        player = Playback(shared, arm, [])
        now = 1.
        shared.pose_time = now
        shared.pose = dict(zip(KEYS, target))
        shared.pose['t'] = math.radians(8.1)
        feedback = iter((6.5, 4.2, 2.2, .9))

        def advance(seconds):
            nonlocal now
            now += seconds
            if round((now-1.)*20) % 2 == 0:
                shared.pose['t'] = math.radians(next(feedback))
                shared.pose_time = now

        with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), \
             patch.object(player.stop, 'wait', advance), patch.object(arm, 'submit') as submit:
            player.recover_tracking(target, shared.pose_time)
        self.assertAlmostEqual(now, 1.4)
        self.assertEqual(player.tracking_rate, .5)
        submit.assert_not_called()

    def test_recorded_hold_is_checked_once_before_the_next_motion(self):
        shared = Shared()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        first = [0., 0., math.pi/2, 0., 0., math.pi]
        hold = [.02, *first[1:]]
        last = [.04, *first[1:]]
        shared.pose = dict(zip(KEYS, first))
        shared.pose_time = time.monotonic()
        records = [dict(time=t, radians=q) for t, q in ((0, first), (.1, hold), (1.1, hold), (1.2, last))]
        player = Playback(shared, arm, records, factor=2)
        with patch.object(player, 'wait_until'), patch.object(player, 'send'), \
             patch.object(player, 'wait_target') as settle:
            player.run()
        self.assertIsNone(player.error)
        self.assertEqual(settle.call_count, 3)
        self.assertEqual(settle.call_args_list[1].args, (hold, 5))
        self.assertEqual(settle.call_args_list[1].kwargs, dict(final=True, label='示教停顿点', correct_dwell=True))

    def test_dwell_corrects_logged_shoulder_elbow_error_and_keeps_grip_target(self):
        target = list(map(math.radians, (36.83, 7.73, 60.82, 88.51, .09, 154.42)))
        measured = list(map(math.radians, (36.91, 9.67, 62.58, 88.86, .18, 154.95)))
        for follows in (True, False):
            with self.subTest(follows=follows):
                shared = Shared()
                arm = Arm(shared, 'TEST')
                arm.connected = True
                player = Playback(shared, arm, [], speed=600)
                initial_motor = [*target[:-1], math.radians(164.42)]
                player.last_motor_target = tuple(initial_motor)
                current = measured.copy()
                now = 1.
                shared.pose = dict(zip(KEYS, current))
                shared.pose_time = now

                def update(seconds):
                    nonlocal now
                    now += seconds
                    shared.pose = dict(zip(KEYS, current))
                    shared.pose_time = now

                def write(nominal, motor, speed, acc):
                    self.assertEqual(nominal, target)
                    self.assertEqual((motor[-1], speed, acc), (initial_motor[-1], 100, 5))
                    self.assertAlmostEqual(math.degrees(motor[1] - target[1]), -1.94)
                    self.assertAlmostEqual(math.degrees(motor[2] - target[2]), -1.76)
                    player.last_motor_target = tuple(motor)
                    if follows:
                        current[1:3] = target[1:3]

                with patch('actions.time', SimpleNamespace(monotonic=lambda: now)), \
                     patch.object(player.stop, 'wait', update), \
                     patch.object(player, 'write_motor_target', side_effect=write) as send:
                    if follows:
                        player.wait_target(target, 5, final=True, label='示教停顿点', correct_dwell=True)
                        self.assertLess(player.xyz_error_mm, 10)
                    else:
                        with self.assertRaisesRegex(RuntimeError, '微调未改善'):
                            player.wait_target(target, 5, final=True, label='示教停顿点', correct_dwell=True)
                send.assert_called_once()


if __name__ == '__main__':
    unittest.main()
