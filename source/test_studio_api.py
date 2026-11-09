from contextlib import closing
import math
from tempfile import TemporaryDirectory
from threading import Event, Thread
import time
import unittest
import uuid

import cv2
import numpy as np

from backend import Arm, KEYS, Shared, all_command
from actions import TeachLibrary, save_trajectory
from host_control import forward
from roarmstudio_sdk import SDKError, Studio
from studio_api import DeviceAPI
from test_backend import FakeSerial


class DeviceAPITests(unittest.TestCase):
    def setUp(self):
        self.shared = Shared()
        self.arm = Arm(self.shared, 'SIMULATION')
        self.arm.connected = True
        self.home = [0., 0., math.pi/2, 0., 0., math.pi]
        self.feedback(self.home)
        self.arm.last_joint_target = self.home.copy()
        self.api = DeviceAPI(self.shared, lambda: self.arm)
        self.url = self.api.start(0)
        self.addCleanup(self.api.stop)
        self.client = Studio(self.url, self.api.secret)

    def teach_fixture(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        library = TeachLibrary(temp.name)
        for slot in (1, 12):
            end = self.home.copy()
            end[0] = slot / 1000
            save_trajectory(library.path(slot), [dict(time=0, radians=self.home), dict(time=.2, radians=end)])
        self.api.teach_library = library

    def feedback(self, joints):
        x, y, z, pitch, _, _ = forward(joints)
        pose = dict(zip(KEYS, joints), x=x, y=y, z=z, tit=pitch)
        pose.update({'torswitch'+key: 1 for key in ('B','S','E','T','R','G')})
        self.arm._accept_pose(pose)

    def test_authenticated_two_feeds_and_depth(self):
        with self.assertRaisesRegex(SDKError, '密钥'):
            Studio(self.url, 'wrong').status()
        color = np.full((12, 16, 3), 120, dtype=np.uint8)
        raw = np.full((12, 16), 1000, dtype=np.uint16)
        self.shared.publish('uvc', {'rgb': color})
        self.shared.publish('gemini', {'rgb': color, 'depth_raw': raw, 'depth_mm': raw.astype(np.float32)*.5},
                            depth_scale=.5, aligned=True)
        for name in ('hand', 'workspace'):
            data, stamp = self.client.frame(name)
            self.assertEqual(cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape, color.shape)
            self.assertGreater(stamp, 0)
            with closing(self.client.stream(name)) as stream:
                self.assertEqual(next(stream), (data, stamp))
        data, stamp, scale = self.client.depth()
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
        self.assertTrue(np.array_equal(decoded, raw))
        self.assertEqual(scale, .5)
        self.shared.frames['uvc']['monotonic'] -= 2
        with self.assertRaises(SDKError) as caught:
            self.client.frame('hand')
        self.assertEqual(caught.exception.code, 'camera_offline')

    def test_sdk_motion_idempotency_control_arbitration_and_new_feedback(self):
        self.client.acquire()
        with self.assertRaises(SDKError) as caught:
            Studio(self.url, self.api.secret).acquire()
        self.assertEqual(caught.exception.code, 'control_busy')
        with self.assertRaises(ValueError):
            self.arm.submit(all_command(self.home))
        with self.assertRaises(ValueError):
            self.arm.begin_replay()
        key = uuid.uuid4()
        result = self.client.jog_xyz(0, 10, 0, request_id=key)
        self.assertEqual(result['state'], 'queued')
        self.assertEqual(self.client.jog_xyz(0, 10, 0, request_id=key)['id'], result['id'])
        self.assertEqual(self.arm.commands.qsize(), 1)
        self.arm._write_pending(FakeSerial())
        self.assertEqual(self.client.action(key)['state'], 'sent')
        time.sleep(.03)
        self.feedback(self.arm.last_joint_target)
        self.assertEqual(self.client.wait(key)['state'], 'reached')
        self.client.release()
        self.assertFalse(self.arm.playing)

    def test_expiry_prevents_serial_write(self):
        self.client.acquire()
        job = self.client.home('down')
        self.arm.external_deadline = time.monotonic() - 1
        serial = FakeSerial()
        self.arm._write_pending(serial)
        self.assertEqual(serial.writes, [])
        self.assertEqual(self.client.action(job['id'])['state'], 'cancelled')

    def test_strict_rejection_then_explicit_degraded_action(self):
        self.client.acquire()
        with self.assertRaises(SDKError) as caught:
            self.client.move_xyz(500, 0, 223.13)
        self.assertEqual(caught.exception.code, 'motion_rejected')
        self.assertTrue(self.arm.commands.empty())
        job = self.client.move_xyz(500, 0, 223.13, allow_degraded=True)
        self.assertTrue(job['degraded'])
        self.assertNotEqual(job['planned_pitch_deg'], 0)
        self.arm._write_pending(FakeSerial())
        time.sleep(.03)
        self.feedback(self.arm.last_joint_target)
        self.assertEqual(self.client.wait(job['id'])['state'], 'degraded_reached')

    def test_local_cancel_revokes_lease_without_extra_motion(self):
        self.client.acquire()
        job = self.client.grip(False)
        self.arm.cancel_replay()
        self.assertEqual(self.client.action(job['id'])['state'], 'cancelled')
        self.assertTrue(self.arm.commands.empty())
        with self.assertRaises(SDKError):
            self.client.grip(False)

    def test_named_teach_slots_play_sequentially_under_one_lease(self):
        self.teach_fixture()
        slots = self.client.teach_slots()
        self.assertEqual(len(slots), 12)
        self.assertEqual([slot['name'] for slot in slots if slot['occupied']], ['示教1', '示教12'])
        self.client.acquire()
        with self.assertRaises(SDKError) as missing:
            self.client.play_teach('示教2')
        self.assertEqual(missing.exception.code, 'teach_empty')
        with self.assertRaises(SDKError) as invalid:
            self.client.play_teach('示教13')
        self.assertEqual(invalid.exception.code, 'invalid_slot')
        serial = FakeSerial()
        stop = Event()

        def drive():
            while not stop.wait(.035):
                self.arm._write_pending(serial)
                self.feedback(self.arm.last_joint_target or self.home)

        driver = Thread(target=drive)
        driver.start()
        try:
            for slot in ('示教1', '示教12'):
                key = uuid.uuid4()
                action = self.client.play_teach(slot, request_id=key)
                self.assertEqual(self.client.play_teach(slot, request_id=key)['id'], action['id'])
                with self.assertRaises(SDKError) as busy:
                    self.client.home()
                self.assertEqual(busy.exception.code, 'action_busy')
                result = self.client.wait(action['id'], timeout=5)
                self.assertEqual(result['state'], 'reached')
                self.assertEqual(result['progress'], 1.)
                self.assertTrue(result['serial_written'])
                self.assertTrue(self.arm.playing)
            self.assertAlmostEqual(self.arm.last_joint_target[0], .012)
            self.client.release()
        finally:
            stop.set()
            driver.join(2)

    def test_teach_cancel_clears_pending_motion(self):
        self.teach_fixture()
        self.client.acquire()
        action = self.client.play_teach('示教1')
        deadline = time.monotonic() + .5
        while self.arm.commands.empty() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertFalse(self.arm.commands.empty())
        self.client.cancel_pending()
        serial = FakeSerial()
        self.arm._write_pending(serial)
        self.assertEqual(serial.writes, [])
        self.assertEqual(self.client.action(action['id'])['state'], 'cancelled')
