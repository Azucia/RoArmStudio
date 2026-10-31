import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
from backend import Arm, Shared, KEYS, joint_command, hold_command, valid_pose, depth_views, Recorder


class FakeSerial:
    instances = []

    def __init__(self):
        self.writes = []
        self.buffer = bytearray()
        self.in_waiting = 0
        FakeSerial.instances.append(self)

    def open(self):
        pass

    def write(self, raw):
        cmd = json.loads(raw)
        self.writes.append(cmd)
        if cmd['T'] == 105:
            self.buffer.extend((json.dumps(dict(T=1051, **dict(zip(KEYS, [0, 0, 1.57, 0, 0, 3.14]))))+'\r\n').encode())

    def read(self, size):
        time.sleep(.001)
        data = self.buffer[:size]
        del self.buffer[:size]
        return data

    def close(self):
        pass


class Tests(unittest.TestCase):
    def test_protocol_and_limits(self):
        cmd = joint_command(5, 180)
        self.assertEqual(cmd['T'], 101)
        self.assertAlmostEqual(cmd['rad'], math.pi)  # Native gripper: never invert twice.
        with self.assertRaises(ValueError):
            joint_command(1, 95)
        self.assertFalse(valid_pose(dict(T=1051)))
        self.assertFalse(valid_pose(dict(T=1051, **dict.fromkeys(KEYS, float('nan')))))
        pose = dict(zip(KEYS, range(6)))
        self.assertEqual(hold_command(pose)['hand'], 5)

    def test_connection_never_moves_and_readback(self):
        shared = Shared()
        with patch('backend.serial.Serial', FakeSerial):
            arm = Arm(shared, 'TEST')
            arm.start()
            end = time.monotonic()+2
            while shared.pose is None and time.monotonic() < end:
                time.sleep(.01)
            self.assertIsNotNone(shared.pose)
            serial = FakeSerial.instances[-1]
            self.assertTrue(all(c['T'] == 105 for c in serial.writes))
            arm.submit(joint_command(0, 5))
            time.sleep(.12)
            self.assertTrue(any(c['T'] == 101 for c in serial.writes))
            self.assertFalse(any(c['T'] == 210 for c in serial.writes))
            arm.stop.set()
            arm.join(2)
            self.assertFalse(arm.is_alive())

    def test_stale_motion_is_discarded(self):
        shared = Shared()
        with patch('backend.serial.Serial', FakeSerial):
            arm = Arm(shared, 'TEST')
            arm.commands.put(joint_command(0, 5))
            arm.start()
            time.sleep(.1)
            arm.stop.set()
            arm.join(2)
            self.assertFalse(any(c['T'] == 101 for c in FakeSerial.instances[-1].writes))

    def test_depth_and_capture_roundtrip(self):
        rgb = np.full((8, 12, 3), 100, np.uint8)
        raw = np.full((8, 12), 1234, np.uint16)
        raw[0, 0] = 0
        depth, overlay = depth_views(rgb, raw.astype(np.float32))
        np.testing.assert_array_equal(overlay[0, 0], rgb[0, 0])
        self.assertEqual(int(depth[0, 0].sum()), 0)
        with tempfile.TemporaryDirectory() as temp:
            shared = Shared()
            shared.publish('gemini', dict(rgb=rgb, depth_raw=raw, depth_mm=raw.astype(np.float32)), depth_scale=1.)
            rec = Recorder(shared, Path(temp)/'中文采集')
            rec.start()
            time.sleep(.25)
            rec.stop.set()
            rec.join(2)
            self.assertIsNone(rec.error)
            entries = [json.loads(x) for x in (rec.folder/'timeline.jsonl').read_text().splitlines()]
            self.assertEqual(len(entries), 1)  # No duplicated cached frame.
            saved = cv2.imdecode(np.frombuffer((rec.folder/entries[0]['files']['depth_raw']).read_bytes(), np.uint8), cv2.IMREAD_UNCHANGED)
            np.testing.assert_array_equal(saved, raw)


if __name__ == '__main__':
    unittest.main()
