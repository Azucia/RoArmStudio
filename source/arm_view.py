"""Read-only projection of the official RoArm-M3 visual links and joint origins."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


GRID_STEP = 75
GRID_BACK_X = -225  # mm, rear edge parallel to the Y axis
DEFAULT_AZIMUTH = math.atan2(-1, .4) + math.pi / 4
DEFAULT_ELEVATION = math.atan2(.48, math.hypot(.4, 1))


def rotate_z(angle):
    s, c = math.sin(angle), math.cos(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def origin_matrix(element):
    x, y, z = map(float, element.get('xyz', '0 0 0').split())
    roll, pitch, yaw = map(float, element.get('rpy', '0 0 0').split())
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rotate_z(yaw) @ ry @ rx, np.array([x, y, z]) * 1000


class ArmGeometry:
    def __init__(self, directory):
        description = ET.parse(directory / 'roarm_m3.xacro').getroot()
        self.joints = [
            (joint.find('parent').get('link'), joint.find('child').get('link'),
             joint.get('type') == 'revolute', *origin_matrix(joint.find('origin')))
            for joint in description.findall('joint')
        ]
        with np.load(directory / 'geometry.npz', allow_pickle=False) as archive:
            self.meshes = {link: archive[link] for link in archive.files}

    def posed_meshes(self, values):
        # ROS joints 1–5 share the firmware convention. The SDK inverts the
        # direct gripper between ROS and the firmware. Apply it only in this view.
        angles = iter((*values[:5], math.pi - values[5]))
        frames = {'world': (np.eye(3), np.zeros(3))}
        for parent, child, revolute, orientation, offset in self.joints:
            basis, position = frames[parent]
            rotation = basis @ orientation
            if revolute:
                rotation = rotation @ rotate_z(next(angles))
            frames[child] = rotation, position + basis @ offset
        return np.concatenate([mesh @ frames[name][0].T + frames[name][1]
                               for name, mesh in self.meshes.items()])


def view_matrix(azimuth, elevation):
    right = np.array([-math.sin(azimuth), math.cos(azimuth), 0])
    toward = np.array([math.cos(elevation) * math.cos(azimuth),
                       math.cos(elevation) * math.sin(azimuth), math.sin(elevation)])
    return np.column_stack((right, np.cross(toward, right), toward))


class ArmView(QWidget):
    def __init__(self, model_dir):
        super().__init__()
        self.setMinimumHeight(150)
        self.setAccessibleName('RoArm-M3 关节反馈姿态')
        self.setToolTip('滚轮缩放 · 左键拖动旋转 · 右键拖动平移')
        self.azimuth, self.elevation = DEFAULT_AZIMUTH, DEFAULT_ELEVATION
        self.default_view = view_matrix(self.azimuth, self.elevation)
        self.view = self.default_view
        self.view_to_floor = np.linalg.inv(self.view[:2, :2])
        self.zoom = 1.0
        self.pan = QPointF()
        self.drag_button = Qt.NoButton
        self.drag_position = QPointF()
        # Keep the same viewing scale while extending the grid beyond this reference area.
        floor = np.array([(x, y, 0) for x in (-75, 675) for y in (-450, 450)]) @ self.view
        self.frame_min = floor[:, :2].min(axis=0)
        self.frame_max = floor[:, :2].max(axis=0)
        # Reserve height for the arm so ordinary motion doesn't change the framing.
        self.frame_max[1] = max(self.frame_max[1], 300)
        self.bounds = self.frame_min, self.frame_max
        self.angles = None
        self.triangles = None
        self.message = '等待机械臂反馈'
        self.shades = [QColor(58+6*i, 68+6*i, 79+6*i) for i in range(24)]
        try:
            self.geometry = ArmGeometry(model_dir)
        except (OSError, ET.ParseError, KeyError, ValueError):
            self.geometry = None
            self.message = '机械臂模型文件不可用'

    def set_pose(self, angles, connected):
        if self.geometry is None:
            return
        message = '' if angles is not None else '等待机械臂反馈' if connected else '机械臂未连接'
        if self.angles == angles and self.message == message:
            return
        self.angles = angles
        self.message = message
        self.triangles = None if angles is None else self.geometry.posed_meshes(angles)
        if self.triangles is None:
            self.bounds = self.frame_min, self.frame_max
        else:
            vertices = (self.triangles @ self.default_view)[:, :, :2]
            self.bounds = (np.minimum(self.frame_min, vertices.min(axis=(0, 1))),
                           np.maximum(self.frame_max, vertices.max(axis=(0, 1))))
        self.update()

    def wheelEvent(self, event):
        steps = event.angleDelta().y() / 120 or event.pixelDelta().y() / 60
        if not steps:
            event.ignore()
            return
        zoom = min(4.0, max(.4, self.zoom * 1.15**steps))
        origin = QPointF(self.width() * .29, self.height() * .46) + self.pan
        self.pan += (event.position() - origin) * (1 - zoom / self.zoom)
        self.zoom = zoom
        self.update()
        event.accept()

    def mousePressEvent(self, event):
        if event.button() not in (Qt.LeftButton, Qt.RightButton):
            super().mousePressEvent(event)
            return
        self.drag_button = event.button()
        self.drag_position = event.position()
        self.setCursor(Qt.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self, event):
        if not event.buttons() & self.drag_button:
            super().mouseMoveEvent(event)
            return
        delta = event.position() - self.drag_position
        self.drag_position = event.position()
        if self.drag_button == Qt.LeftButton:
            self.azimuth = (self.azimuth + delta.x() * .008) % math.tau
            self.elevation = min(math.radians(85), max(math.radians(12), self.elevation - delta.y() * .008))
            self.view = view_matrix(self.azimuth, self.elevation)
            self.view_to_floor = np.linalg.inv(self.view[:2, :2])
        else:
            self.pan += delta
        self.update()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() != self.drag_button:
            super().mouseReleaseEvent(event)
            return
        self.drag_button = Qt.NoButton
        self.unsetCursor()
        event.accept()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#080b0f'))
        (left, bottom), (right, top) = self.bounds
        width, height = self.width(), self.height()
        cx, cy = width * .29, height * .46
        margin = 12
        scale = min((cx - margin) / -left, (width - cx - margin) / right,
                    (cy - margin) / top, (height - cy - margin) / -bottom)
        if scale <= 0:
            return
        scale *= self.zoom
        cx += self.pan.x()
        cy += self.pan.y()

        def point(world):
            x, y, _ = np.asarray(world) @ self.view
            return QPointF(cx + x * scale, cy - y * scale)

        # Project the viewport corners onto the ground, then draw whole grid cells
        # past the edges. Qt clips the lines to the preview.
        corners = np.array([(x - cx, cy - y) for x in (0, width) for y in (0, height)])
        floor = (corners / scale) @ self.view_to_floor
        x_min, y_min = np.floor(floor.min(axis=0) / GRID_STEP).astype(int) * GRID_STEP
        x_max, y_max = np.ceil(floor.max(axis=0) / GRID_STEP).astype(int) * GRID_STEP
        x_min = max(x_min, GRID_BACK_X)
        if x_min <= x_max:
            painter.setPen(QPen(QColor('#23313f'), 1))
            for y in range(y_min, y_max + 1, GRID_STEP):
                painter.drawLine(point((x_min, y, 0)), point((x_max, y, 0)))
            for x in range(x_min, x_max + 1, GRID_STEP):
                painter.drawLine(point((x, y_min, 0)), point((x, y_max, 0)))
        for color, tip in (('#c66b64', (100, 0, 0)), ('#5eab82', (0, 100, 0)), ('#639bd7', (0, 0, 100))):
            painter.setPen(QPen(QColor(color), 1.5))
            painter.drawLine(point((0, 0, 0)), point(tip))
        if self.triangles is None:
            painter.setPen(QColor('#b4bfca'))
            painter.drawText(self.rect().adjusted(12, 12, -12, -12), Qt.AlignLeft | Qt.AlignTop, self.message)
            return

        triangles = self.triangles @ self.view
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        visible = normals[:, 2] > .001
        triangles, normals = triangles[visible], normals[visible]
        unit = normals / np.linalg.norm(normals, axis=1)[:, None]
        shades = np.clip(((unit @ np.array([-.27, .58, .76]))*.5+.5)*23, 0, 23).astype(int)
        order = np.argsort(triangles[:, :, 2].mean(axis=1))
        pixels = triangles[:, :, :2] * np.array([scale, -scale]) + np.array([cx, cy])
        for index in order:
            color = self.shades[shades[index]]
            painter.setPen(QPen(color, 0))
            painter.setBrush(color)
            painter.drawPolygon(QPolygonF([QPointF(x, y) for x, y in pixels[index]]))
