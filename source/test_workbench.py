import math
import time
import unittest
from threading import Event
from types import SimpleNamespace

import numpy as np
from PySide6.QtWidgets import QApplication

from app import Window
from backend import Arm, KEYS, all_command

APP = QApplication.instance() or QApplication([])


class WorkbenchTests(unittest.TestCase):
    def setUp(self):
        self.window = Window()
        self.addCleanup(self.window.close)
        self.window.arm = Arm(self.window.shared, 'COM7')
        self.window.arm.connected = True
        self.publish_pose((0., .3, 1.1, -.2, 0., math.pi))

    def publish_pose(self, values):
        self.window.shared.pose = dict(zip(KEYS, values), T=1051, x=242.6, y=53.2, z=184.)
        self.window.shared.pose.update({'torswitch'+k: 1 for k in ('B', 'S', 'E', 'T', 'R', 'G')})
        self.window.shared.pose_time = time.monotonic()

    def test_feedback_updates_model_readouts_and_clears_stale_values(self):
        w = self.window
        w.tick()
        self.assertIn('COM7', w.status_strip.labels[0].text())
        self.assertIn('已使能', w.status_strip.labels[-1].text())
        self.assertIsNotNone(w.arm_view.geometry)
        self.assertGreater(len(w.arm_view.triangles), 100)
        self.assertIn('242.6', w.tcp.text())
        self.assertEqual(w.diagnostics.angles.item(1, 2).text(), '+17.2')
        first = w.arm_view.triangles.copy()
        self.publish_pose((.5, .3, 1.1, -.2, 0., math.pi))
        w.tick()
        self.assertFalse(np.allclose(first, w.arm_view.triangles))
        w.shared.pose_time -= 1
        w.tick()
        self.assertIsNone(w.arm_view.triangles)
        self.assertEqual(w.diagnostics.angles.item(1, 2).text(), '—')

    def test_two_camera_previews_and_disconnect(self):
        w = self.window
        camera = lambda: SimpleNamespace(is_alive=lambda: True, stop=Event(), identity=None)
        w.uvc, w.gemini = camera(), camera()
        frame = np.zeros((40, 80, 3), dtype=np.uint8)
        w.shared.publish('uvc', {'rgb': frame})
        w.shared.publish('gemini', {'rgb': frame, 'depth_mm': np.full((40, 80), 500, np.float32)})
        w.show()
        APP.processEvents()
        w.render_frames()
        w.tick()
        self.assertEqual(len(w.views), 2)
        self.assertTrue(all(not view.image.pixmap().isNull() for view in w.views))
        self.assertIn('UVC', w.status_strip.labels[1].text())
        self.assertIn('USB', w.status_strip.labels[2].text())
        w.uvc.stop.set()
        w.render_frames()
        w.tick()
        self.assertTrue(w.views[0].image.pixmap().isNull())
        self.assertNotIn('UVC', w.status_strip.labels[1].text())

    def test_replay_state_and_stop_use_existing_paths(self):
        w = self.window
        self.assertIn('严格执行', w.cartesian_pad.execution_mode.text())
        w.controls.playback = SimpleNamespace(arm=w.arm, degraded=True)
        w.tick()
        self.assertIn('降标执行', w.cartesian_pad.execution_mode.text())
        cmd = all_command([0., 0., math.pi/2, 0., 0., math.pi])
        receipt = w.arm.submit(cmd)
        w.status_strip.stop.click()
        self.assertTrue(receipt.done.is_set())
        self.assertIsNotNone(receipt.error)
        self.assertTrue(w.arm.commands.empty())
        w.controls.playback = None
