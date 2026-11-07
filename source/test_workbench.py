import math
import time
import unittest
from threading import Event
from types import SimpleNamespace

import numpy as np
from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from app import Window
from appearance import TitleBar
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
        self.window.shared.pose = dict(zip(KEYS, values), T=1051, x=242.6, y=53.2, z=184., tit=0.)
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
        self.assertEqual([value.text() for value in w.cartesian_pad.coordinate_values], ['242.6', '53.2', '184.0'])
        self.assertEqual(w.diagnostics.angles.item(1, 2).text(), '+17.2')
        first = w.arm_view.triangles.copy()
        self.publish_pose((.5, .3, 1.1, -.2, 0., math.pi))
        w.tick()
        self.assertFalse(np.allclose(first, w.arm_view.triangles))
        w.shared.pose_time -= 1
        w.tick()
        self.assertIsNone(w.arm_view.triangles)
        self.assertEqual(w.diagnostics.angles.item(1, 2).text(), '—')
        self.assertTrue(all(value.text() == '—' for value in w.cartesian_pad.coordinate_values))

    def test_top_tabs_replace_only_the_left_control_panel(self):
        w = self.window
        w.resize(1093, 614)
        w.show()
        APP.processEvents()
        w.tick()
        APP.processEvents()
        shared_views = (w.diagnostics, w.arm_view, *w.views)
        original_sizes = [view.size() for view in shared_views]
        for index in (1, 2, 3, 0):
            w.control_tabs.setCurrentIndex(index)
            APP.processEvents()
            self.assertEqual(w.control_pages.currentIndex(), index)
            self.assertEqual(w.cartesian_pad.isVisible(), index == 0)
            self.assertTrue(all(view.isVisible() for view in shared_views))
            self.assertEqual([view.size() for view in shared_views], original_sizes)

    def test_integrated_caption_toggles_window_state(self):
        w = self.window
        w.show()
        APP.processEvents()
        title = w.findChild(TitleBar)
        self.assertFalse(bool(w.windowFlags() & Qt.WindowTitleHint))
        title.maximize.click()
        self.assertTrue(w.isMaximized())
        self.assertEqual(title.maximize.accessibleName(), '还原窗口')
        QTest.mouseDClick(title, Qt.LeftButton, pos=title.rect().center())
        self.assertFalse(w.isMaximized())
        self.assertEqual(title.maximize.accessibleName(), '最大化')

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

    def test_mouse_navigation_changes_only_the_preview(self):
        w = self.window
        w.show()
        APP.processEvents()
        w.tick()
        view = w.arm_view
        start = view.rect().center()
        delta = QPoint(36, 20)
        original_view = view.view.copy()
        original_mesh = view.triangles.copy()

        QTest.mousePress(view, Qt.LeftButton, pos=start)
        QTest.mouseMove(view, start + delta)
        QTest.mouseRelease(view, Qt.LeftButton, pos=start + delta)
        self.assertFalse(np.allclose(view.view, original_view))
        self.assertTrue(np.array_equal(view.triangles, original_mesh))

        QTest.mousePress(view, Qt.RightButton, pos=start)
        QTest.mouseMove(view, start + delta)
        QTest.mouseRelease(view, Qt.RightButton, pos=start + delta)
        self.assertEqual(view.pan, QPointF(delta))

        wheel = QWheelEvent(QPointF(start), QPointF(view.mapToGlobal(start)), QPoint(),
                            QPoint(0, 120), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False)
        APP.sendEvent(view, wheel)
        self.assertGreater(view.zoom, 1)
        self.assertTrue(wheel.isAccepted())
        self.assertTrue(w.arm.commands.empty())

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
