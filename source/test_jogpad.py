import math
import time
import unittest
from PySide6.QtCore import Qt, QPointF, QEvent
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication
from app import Window
from jogpad import XYDisc
from backend import pose_command

APP = QApplication.instance() or QApplication([])


class DummyArm:
    connected = True
    teaching = False
    playing = False

    def __init__(self):
        self.sent = []

    def submit(self, command):
        self.sent.append(command)

    def is_alive(self):
        return False


class JogTests(unittest.TestCase):
    def test_disc_hit_regions(self):
        disc = XYDisc()
        disc.resize(220, 220)
        x, y, r = disc.geometry_values()
        self.assertEqual(disc.hit(QPointF(x, y)), 'home')
        self.assertEqual(disc.hit(QPointF(x+r*.4, y)), (0, 1))
        self.assertEqual(disc.hit(QPointF(x-r*.8, y)), (0, -10))
        self.assertEqual(disc.hit(QPointF(x, y-r*.8)), (1, 10))
        self.assertEqual(disc.hit(QPointF(x, y+r*.4)), (1, -1))
        self.assertIsNone(disc.hit(QPointF(x+r*1.2, y)))
        disc.deleteLater()

    def test_keyboard_and_tabs_and_input_guard(self):
        w = Window()
        w.arm = DummyArm()
        w.shared.pose = dict(T=1051, x=235, y=0, z=234, tit=0, b=0, s=0, e=1.57, t=0, r=0, g=1.08)
        w.shared.pose_time = time.monotonic()
        w.show()
        w.isActiveWindow = lambda: True
        APP.processEvents()
        pad = w.cartesian_pad
        pad.refresh()
        pad.disc.setFocus()
        APP.processEvents()
        def key(k, modifier=Qt.NoModifier, repeat=False):
            return QKeyEvent(QEvent.KeyPress, k, modifier, '', repeat, 1)
        self.assertTrue(pad.eventFilter(w, key(Qt.Key_W)))
        self.assertEqual(w.arm.sent[-1]['y'], 1)
        pad.eventFilter(w, key(Qt.Key_D, Qt.ShiftModifier))
        self.assertEqual(w.arm.sent[-1]['x'], 245)
        count = len(w.arm.sent)
        pad.eventFilter(w, key(Qt.Key_D, repeat=True))
        self.assertEqual(len(w.arm.sent), count)
        w.control_tabs.setCurrentIndex(1)
        APP.processEvents()
        self.assertFalse(pad.eventFilter(w, key(Qt.Key_W)))
        w.control_tabs.setCurrentIndex(0)
        w.cam_index.setFocus()
        APP.processEvents()
        self.assertFalse(pad.eventFilter(w, key(Qt.Key_W)))
        self.assertEqual(len(w.arm.sent), count)
        w.close()

    def test_open_gripper_boundary_keeps_cartesian_control(self):
        command = pose_command([235, 0, 234, 0, 0, math.degrees(1.08)])
        self.assertAlmostEqual(command['g'], 1.08)

    def test_precise_xyz_keeps_orientation(self):
        w = Window()
        w.arm = DummyArm()
        w.shared.pose = dict(T=1051, x=235, y=1, z=234, tit=.2, b=0, s=0, e=1.57, t=0, r=.3, g=1.08)
        w.shared.pose_time = time.monotonic()
        pad = w.cartesian_pad
        pad.switch_mode()
        self.assertEqual(pad.xyz_targets[0].value(), 235)
        pad.xyz_targets[0].setValue(240)
        pad.move_xyz()
        command = w.arm.sent[-1]
        self.assertEqual(command['x'], 240)
        self.assertAlmostEqual(command['t'], .2)
        self.assertAlmostEqual(command['r'], .3)
        self.assertAlmostEqual(command['g'], 1.08)
        self.assertEqual(w.control_tabs.count(), 4)
        w.close()


if __name__ == '__main__':
    unittest.main()
