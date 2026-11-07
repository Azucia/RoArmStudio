"""Regression scenarios for motion limits, stopping and playback; no hardware."""
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import threading
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication
from actions import Playback, load_trajectory
from app import Window
from backend import Arm, Shared, KEYS, all_command, joint_command, pose_command
from test_backend import FakeSerial

APP = QApplication.instance() or QApplication([])


class MotionSafetyTests(unittest.TestCase):
    def arm(self):
        shared = Shared()
        shared.pose = dict(T=1051, **dict(zip(KEYS, [0, .06, 1.62, .02, 0, math.pi])))
        shared.pose.update({'torswitch'+k: 1 for k in ('B', 'S', 'E', 'T', 'R', 'G')})
        shared.pose_time = time.monotonic()
        arm = Arm(shared, 'TEST')
        arm.connected = True
        arm.last_joint_target = [0, 0, math.pi/2, 0, 0, math.pi]
        return shared, arm

    def test_front_sector_edges_apply_to_angles_and_xyz(self):
        for sign in (1, -1):
            with self.subTest(side=sign):
                _, arm = self.arm()
                boundary = math.radians(sign * 105)
                values = [boundary, 0, math.pi/2, 0, 0, math.pi]
                arm.submit(all_command(values))
                arm.submit(pose_command([250*math.cos(boundary), 250*math.sin(boundary), 180, 0, 0, 180]))
                with self.assertRaisesRegex(ValueError, '105'):
                    joint_command(0, sign * 105.01)
                outside = math.radians(sign * 105.01)
                with self.assertRaisesRegex(ValueError, '105'):
                    pose_command([250*math.cos(outside), 250*math.sin(outside), 180, 0, 0, 180])

    def test_raw_motion_cannot_bypass_sector_or_fold_behind_base(self):
        _, arm = self.arm()
        for command in (dict(T=101, joint=1, rad=math.radians(120)),
                        dict(T=1041, x=-200, y=0),
                        dict(T=102, base=0, shoulder=-math.pi/2, elbow=0, wrist=0, roll=0, hand=math.pi)):
            with self.subTest(command=command), self.assertRaisesRegex(ValueError, '105'):
                arm.submit(command)
        self.assertTrue(arm.commands.empty())

    def test_boundary_readback_allows_nonbase_motion_and_blocks_outside_before_write(self):
        shared, arm = self.arm()
        rounded = round(105 * 4096 / 360) * math.tau / 4096
        shared.pose['b'] = rounded
        serial = FakeSerial()
        arm.submit(joint_command(5, 180))
        arm._write_pending(serial)
        self.assertEqual(len(serial.writes), 1)
        receipt = arm.submit(joint_command(5, 180))
        shared.pose['b'] = math.radians(106)
        arm._write_pending(serial)
        self.assertEqual(len(serial.writes), 1)
        self.assertIn('105', receipt.error)

    def test_old_trajectory_with_rear_waypoint_is_rejected_on_load(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'record.json'
            path.write_text(json.dumps({'format': 'roarm-studio-native-v1', 'records': [
                {'time': i, 'radians': [math.radians(angle), 0, math.pi/2, 0, 0, math.pi]}
                for i, angle in enumerate((0, 120))
            ]}), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, '105'):
                load_trajectory(path)

    def test_stop_never_retargets_loaded_encoders_even_without_fresh_feedback(self):
        for age in (0, 5):
            with self.subTest(feedback_age=age):
                shared, arm = self.arm()
                shared.pose_time -= age
                w = Window()
                w.arm, w.shared = arm, shared
                notes, errors = [], []
                w.note = notes.append
                w.error = lambda e: errors.append(str(e))
                held = arm.last_joint_target.copy()
                try:
                    w.cartesian_pad.target_intent.commit([350, 0, 200, 0, 0, 180], (1, 0))
                    arm.submit(joint_command(0, 5))
                    w.hold()
                    self.assertTrue(arm.commands.empty())
                    self.assertEqual(arm.last_joint_target, held)
                    self.assertIsNone(w.cartesian_pad.target_intent.goal)
                    self.assertEqual(errors, [])
                    self.assertTrue(notes)
                finally:
                    w.close()

    def test_playback_failure_cancels_without_encoder_hold_command(self):
        shared, arm = self.arm()
        target = arm.last_joint_target.copy()
        player = Playback(shared, arm, [dict(time=0, radians=target)])
        held = target.copy()
        submit_command = arm.submit
        def write_command(command, token=None):
            receipt = submit_command(command, token)
            arm._write_pending(FakeSerial())
            return receipt
        with patch.object(player, 'wait_target', side_effect=RuntimeError('tracking failure')), \
             patch.object(arm, 'submit', side_effect=write_command) as submit:
            player.run()
        self.assertEqual(player.error, 'tracking failure')
        self.assertEqual(submit.call_count, 1)  # Initial intended target only.
        self.assertTrue(arm.commands.empty())
        self.assertEqual(arm.last_joint_target, held)

    def test_playback_aborts_at_stale_boundary_and_during_shutdown(self):
        for state in ('stale', 'shutdown', 'torque_pending', 'torque_fault'):
            with self.subTest(state=state):
                shared, arm = self.arm()
                player = Playback(shared, arm, [])
                if state == 'stale':
                    shared.pose_time -= .7
                elif state == 'shutdown':
                    arm.stop.set()
                else:
                    setattr(arm, state, True)
                with self.assertRaises((RuntimeError, InterruptedError)):
                    player.ensure()

    def test_disabled_torque_rejects_motion_before_queueing(self):
        shared, arm = self.arm()
        shared.pose['torswitchS'] = 0
        with self.assertRaises(ValueError):
            arm.submit(joint_command(0, 5))
        self.assertTrue(arm.commands.empty())

    def test_stale_or_disabled_feedback_cancels_at_write_boundary_and_stays_cancelled(self):
        for state in ('stale', 'disabled', 'shutdown'):
            with self.subTest(state=state):
                shared, arm = self.arm()
                player = Playback(shared, arm, [])
                arm.submit(all_command(arm.last_joint_target), token=player.token)
                if state == 'stale':
                    shared.pose_time -= .7
                elif state == 'disabled':
                    shared.pose['torswitchS'] = 0
                else:
                    arm.stop.set()
                serial = FakeSerial()
                arm._write_pending(serial)
                self.assertEqual(serial.writes, [])
                self.assertTrue(arm.commands.empty())
                shared.pose_time = time.monotonic()
                shared.pose['torswitchS'] = 1
                with self.assertRaises(InterruptedError):
                    player.ensure()

    def test_playback_waits_for_its_own_serial_write_before_arrival_checks(self):
        shared, arm = self.arm()
        player = Playback(shared, arm, [])
        finished = threading.Event()
        errors = []
        def send():
            try:
                player.send(arm.last_joint_target.copy())
            except Exception as e:
                errors.append(str(e))
            finally:
                finished.set()
        thread = threading.Thread(target=send)
        thread.start()
        try:
            end = time.monotonic()+1
            while arm.commands.empty() and not finished.is_set() and time.monotonic()<end:
                time.sleep(.001)
            self.assertFalse(finished.is_set(), 'An unsent target must not reach arrival checks')
            arm._write_pending(FakeSerial())
            self.assertTrue(finished.wait(.5))
            self.assertEqual(errors, [])
        finally:
            arm.cancel_replay()
            thread.join(2)

    def test_recovered_feedback_cannot_hide_a_gap_or_revive_playback(self):
        shared, arm = self.arm()
        player = Playback(shared, arm, [])
        receipt = arm.submit(all_command(arm.last_joint_target), token=player.token)
        shared.pose_time -= .7
        arm._accept_pose(shared.pose.copy())
        self.assertTrue(receipt.done.is_set())
        self.assertIsNotNone(receipt.error)
        self.assertTrue(arm.commands.empty())
        self.assertIsNone(arm.last_joint_target)
        self.assertLess(time.monotonic()-shared.pose_time, .1)
        with self.assertRaises(InterruptedError):
            player.ensure()

    def test_cancelled_or_replaced_receipt_never_claims_serial_delivery(self):
        shared, arm = self.arm()
        receipt = arm.submit(joint_command(0, 1))
        replacement = arm.submit(joint_command(0, 2))
        self.assertTrue(receipt.done.is_set())
        self.assertIsNotNone(receipt.error)
        arm.cancel_replay()
        self.assertTrue(replacement.done.is_set())
        self.assertIsNotNone(replacement.error)

    def test_partial_write_invalidates_reference_and_does_not_confirm_target(self):
        shared, arm = self.arm()
        receipt = arm.submit(joint_command(0, 5))
        serial = FakeSerial()
        serial.write = lambda packet: len(packet)-1
        with self.assertRaisesRegex(OSError, '未完整写出'):
            arm._write_pending(serial)
        self.assertTrue(receipt.done.is_set())
        self.assertIsNotNone(receipt.error)
        self.assertIsNone(arm.last_joint_target)
        self.assertEqual(arm.command_sequence, 0)


if __name__ == '__main__':
    unittest.main()
