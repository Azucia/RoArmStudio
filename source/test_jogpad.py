import math
import time
import unittest
from PySide6.QtCore import Qt, QPointF, QEvent
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from app import Window
from jogpad import XYDisc
from backend import ALL_JOINT_FIELDS, pose_command
from host_control import forward

APP = QApplication.instance() or QApplication([])


class DummyArm:
    port = 'COM-TEST'
    last_joint_target = None
    connected = True
    teaching = False
    playing = False
    torque_pending = False
    torque_fault = False
    torque_state = '未确认'

    def __init__(self):
        self.sent = []

    def submit(self, command):
        self.sent.append(command)

    def is_alive(self):
        return False


class JogTests(unittest.TestCase):
    def test_legacy_pose_command_preserves_measured_gripper(self):
        cmd = pose_command([353.1467839, -2.160772722, 212.3804383, math.degrees(.027611654),
                            0, math.degrees(3.149262558)], [0, 0, math.degrees(3.149262558)])
        self.assertAlmostEqual(cmd['x'], 353.1467839)
        self.assertAlmostEqual(cmd['g'], 3.149262558)
        self.assertAlmostEqual(cmd['t'], .027611654)
        with self.assertRaises(ValueError):
            pose_command([352, 0, 212, 0, 0, 181], [0, 0, 180.44])
        with self.assertRaises(ValueError):
            pose_command([901, 0, 212, 0, 0, 180.44], [0, 0, 180.44])

    def test_gripper_buttons_only_command_joint_six_and_obey_state(self):
        w = Window()
        errors=[]
        w.error=lambda e: errors.append(str(e))
        w.arm = DummyArm()
        w.shared.pose = dict(T=1051, x=352, y=0, z=212, tit=0, b=0, s=0, e=1.57, t=0, r=0, g=3.14)
        w.shared.pose_time = time.monotonic()
        w.cartesian_pad.grip_close.click()
        self.assertEqual(w.arm.sent[-1], dict(T=101, joint=6, rad=math.pi, spd=100, acc=5))
        w.cartesian_pad.grip_open.click()
        self.assertAlmostEqual(w.arm.sent[-1]['rad'],1.08)
        for state in ('teaching', 'playing'):
            before=len(w.arm.sent)
            setattr(w.arm,state,True)
            w.grip(True)
            self.assertEqual(len(w.arm.sent),before)
            setattr(w.arm,state,False)
        before=len(w.arm.sent)
        w.shared.pose_time = 0
        w.grip(True)
        self.assertEqual(len(w.arm.sent),before)
        self.assertEqual(len(errors),3)
        w.close()

    def test_press_and_release_feedback_without_motion(self):
        disc = XYDisc()
        disc.resize(240, 240)
        disc.show()
        APP.processEvents()
        cx, cy, radius = disc.geometry_values()
        position = QPointF(cx+radius*.8, cy).toPoint()
        commands = []
        disc.jog.connect(lambda axis, distance: commands.append((axis, distance)))
        QTest.mousePress(disc, Qt.LeftButton, pos=position)
        self.assertEqual(disc.pressed, (0, 10))
        self.assertEqual(commands, [])
        QTest.mouseRelease(disc, Qt.LeftButton, pos=position)
        QTest.qWait(180)
        self.assertIsNone(disc.pressed)
        disc.close()

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
        joints = [0, .1, 1.5, -.05, 0, 1.08]
        x, y, z, pitch, _, _ = forward(joints)
        w.arm.last_joint_target = joints
        w.shared.pose = dict(T=1051, x=x, y=y, z=z, tit=pitch, **dict(zip('bsetrg', joints)))
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
        self.assertAlmostEqual(forward([w.arm.sent[-1][k] for k in ALL_JOINT_FIELDS])[1], y + 1)
        pad.eventFilter(w, key(Qt.Key_D, Qt.ShiftModifier))
        self.assertAlmostEqual(forward([w.arm.sent[-1][k] for k in ALL_JOINT_FIELDS])[0], x + 10)
        self.assertEqual(pad.disc.pressed, (0, 10))
        count = len(w.arm.sent)
        pad.eventFilter(w, key(Qt.Key_D, repeat=True))
        self.assertEqual(len(w.arm.sent), count)
        w.control_tabs.setCurrentIndex(1)
        APP.processEvents()
        self.assertFalse(pad.eventFilter(w, key(Qt.Key_W)))
        w.control_tabs.setCurrentIndex(3)
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
        original_speed = w.speed.value()
        self.addCleanup(w.speed.setValue, original_speed)
        w.speed.setValue(200)
        pad.switch_mode()
        self.assertEqual(pad.xyz_targets[0].value(), 235)
        pad.xyz_targets[0].setValue(240)
        pad.move_xyz()
        command = w.arm.sent[-1]
        self.assertEqual((command['T'], command['spd']), (102, 200))
        actual = forward([command[k] for k in ALL_JOINT_FIELDS])
        self.assertAlmostEqual(actual[0], 240)
        self.assertAlmostEqual(actual[3], .2)
        self.assertAlmostEqual(actual[4], .3)
        self.assertAlmostEqual(actual[5], 1.08)
        self.assertEqual(w.control_tabs.tabText(3), "设备/连接")
        w.close()


if __name__ == '__main__':
    unittest.main()
