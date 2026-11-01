"""Discrete Cartesian jog pad; every click uses fresh measured position."""
import math
from PySide6.QtCore import Qt, Signal, QRectF, QPointF, QEvent, QTimer
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPainterPath
from PySide6.QtWidgets import QWidget, QGroupBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QApplication, QLineEdit, QAbstractSpinBox, QPlainTextEdit, QStackedWidget, QDoubleSpinBox, QFormLayout
from backend import pose_command


class XYDisc(QWidget):
    jog = Signal(int, float)
    halt = Signal()
    home = Signal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(224, 224)
        self.setMaximumSize(240, 240)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(Qt.PointingHandCursor)
        self.setAccessibleName('XY 末端方向盘：WASD 移动，中心回中')
        self.setToolTip('内圈 1 mm · 外圈 10 mm · 中心回中\nWASD 控制 XY，Q/E 控制 Z，Shift 切换 10 mm')
        self.motion_enabled = False
        self.hovered = None
        self.pressed = None
        self.press_generation = 0
        self.setMouseTracking(True)

    def geometry_values(self):
        return self.width()/2, self.height()/2, min(self.width(), self.height())/2-5

    def hit(self, point):
        cx, cy, radius = self.geometry_values()
        dx, dy = point.x()-cx, point.y()-cy
        distance = math.hypot(dx, dy)/radius
        if distance > 1:
            return None
        if distance < .24:
            return 'home'
        step = 1 if distance < .64 else 10
        axis, sign = (0, 1 if dx >= 0 else -1) if abs(dx) >= abs(dy) else (1, 1 if dy < 0 else -1)
        return axis, sign*step

    def mouseMoveEvent(self, event):
        self.hovered = self.hit(event.position())
        self.update()

    def leaveEvent(self, event):
        self.hovered = None
        self.update()

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        self.setFocus()
        action = self.hit(event.position())
        if action is not None:
            self.pressed = action
            self.press_generation += 1
            self.update()
        if action == 'home' and self.motion_enabled:
            self.home.emit()
        elif action is not None and self.motion_enabled:
            self.jog.emit(*action)

    def mouseReleaseEvent(self, event):
        self.release_feedback()

    def release_feedback(self):
        generation = self.press_generation
        def clear():
            if generation == self.press_generation:
                self.pressed = None
                self.update()
        QTimer.singleShot(130, clear)

    def flash(self, axis, distance):
        self.pressed = (axis, distance)
        self.press_generation += 1
        self.update()
        self.release_feedback()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        keys = {Qt.Key_Right: (0, 1), Qt.Key_Left: (0, -1), Qt.Key_Up: (1, 1), Qt.Key_Down: (1, -1)}
        if event.key() in (Qt.Key_Space, Qt.Key_Escape):
            self.halt.emit()
        elif event.key() in keys and self.motion_enabled:
            axis, sign = keys[event.key()]
            self.jog.emit(axis, sign*(10 if event.modifiers() & Qt.ShiftModifier else 1))
        else:
            super().keyPressEvent(event)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy, radius = self.geometry_values()
        for ring, (inner, outer, step) in enumerate(((.24, .64, 1), (.64, 1., 10))):
            for direction, (axis, sign) in enumerate(((0, 1), (1, 1), (0, -1), (1, -1))):
                start = direction*90-43
                path = QPainterPath()
                rect = QRectF(cx-radius*outer, cy-radius*outer, 2*radius*outer, 2*radius*outer)
                small = QRectF(cx-radius*inner, cy-radius*inner, 2*radius*inner, 2*radius*inner)
                path.arcMoveTo(rect, start)
                path.arcTo(rect, start, 86)
                angle = math.radians(start+86)
                path.lineTo(cx+radius*inner*math.cos(angle), cy-radius*inner*math.sin(angle))
                path.arcTo(small, start+86, -86)
                path.closeSubpath()
                hover = self.hovered == (axis, sign*step) and self.motion_enabled
                pressed = self.pressed == (axis, sign*step)
                p.fillPath(path, QColor('#aab9e4' if pressed else '#bfcbed' if hover else '#d4daf0' if ring == 0 else '#e0e4f3'))
                p.setPen(QPen(QColor('#8195ca' if pressed else '#bac5e1'), 2 if pressed else 1))
                p.drawPath(path)
                angle = math.radians(direction*90)
                distance = radius*(inner+outer)/2
                tx, ty = cx+distance*math.cos(angle), cy-distance*math.sin(angle)
                p.setPen(QColor('#4f5e91' if self.motion_enabled else '#8490ad'))
                p.setFont(QFont('Microsoft YaHei UI', 11))
                text = ('X' if axis == 0 else 'Y')+('+' if sign > 0 else '−')+f'{step}'
                p.drawText(QRectF(tx-29, ty-13, 58, 26), Qt.AlignCenter, text)
        p.setPen(QPen(QColor('#879bcf'), 1.5))
        p.setBrush(QColor('#99addf' if self.pressed == 'home' else '#a7b4e4' if self.hovered == 'home' else '#bec8ec'))
        p.drawEllipse(QPointF(cx, cy), radius*.22, radius*.22)
        p.setPen(QColor('#4e5c8e'))
        p.setFont(QFont('Microsoft YaHei UI', 11))
        p.drawText(QRectF(cx-radius*.22, cy-12, radius*.44, 24), Qt.AlignCenter, '回中')
        if self.hasFocus():
            p.setPen(QPen(QColor('#9daedf'), 1))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), radius+2, radius+2)


class CartesianPad(QGroupBox):
    def __init__(self, owner):
        super().__init__('')
        self.owner = owner
        layout = QVBoxLayout(self)
        self.position = QLabel('X  —     Y  —     Z  —')
        self.position.setObjectName('coordinates')
        layout.addWidget(self.position)
        self.state = QLabel('等待实际末端坐标')
        self.state.setObjectName('quiet')
        layout.addWidget(self.state)
        self.mode_stack = QStackedWidget()
        layout.addWidget(self.mode_stack)
        disc_page = QWidget()
        row = QHBoxLayout(disc_page)
        row.setContentsMargins(0, 0, 0, 0)
        row.addStretch()
        self.mode_stack.addWidget(disc_page)
        self.disc = XYDisc()
        self.disc.jog.connect(self.move)
        self.disc.halt.connect(owner.hold)
        self.disc.home.connect(owner.home)
        row.addWidget(self.disc)
        zcol = QVBoxLayout()
        zcol.addWidget(QLabel('Z · 高度'))
        self.buttons = []
        for label, delta in (('↑ 10 mm', 10), ('↑ 1 mm', 1), ('↓ 1 mm', -1), ('↓ 10 mm', -10)):
            button = QPushButton(label)
            button.setMinimumWidth(100)
            button.setMinimumHeight(35)
            button.clicked.connect(lambda checked=False, d=delta: self.move(2, d))
            self.buttons.append(button)
            zcol.addWidget(button)
        row.addLayout(zcol)
        row.addStretch()
        precise_page = QWidget()
        precise_layout = QVBoxLayout(precise_page)
        form = QFormLayout()
        self.xyz_targets = []
        for axis, limit in zip('XYZ', (500, 500, 600)):
            spin = QDoubleSpinBox()
            spin.setRange(-limit, limit)
            spin.setDecimals(1)
            spin.setSuffix(' mm')
            self.xyz_targets.append(spin)
            form.addRow(axis+' 目标', spin)
        precise_layout.addLayout(form)
        self.precise_read = QPushButton('读取实际 XYZ')
        self.precise_read.clicked.connect(self.copy_xyz)
        precise_layout.addWidget(self.precise_read)
        self.precise_go = QPushButton('移动到 XYZ 目标')
        self.precise_go.setObjectName('primary')
        self.precise_go.clicked.connect(self.move_xyz)
        precise_layout.addWidget(self.precise_go)
        precise_layout.addWidget(QLabel('保留当前俯仰、旋转和夹爪角度。'))
        precise_layout.addStretch()
        self.mode_stack.addWidget(precise_page)
        info = QVBoxLayout()
        hint = QLabel('内圈 1 mm · 外圈 10 mm\nWASD 移动 XY，Q / E 升降\nShift 10 mm · 空格停止')
        hint.setToolTip('每次点击或按键移动一步，按住不连发；编辑输入框时不触发快捷键。中心回中会请求恢复初始姿态。')
        hint.setObjectName('quiet')
        hint.setWordWrap(True)
        info.addWidget(hint)
        info.addStretch()
        self.more = QPushButton('切换到精确 XYZ 输入')
        self.more.clicked.connect(self.switch_mode)
        info.addWidget(self.more)
        stop = QPushButton('停止运动 / 回放')
        stop.setObjectName('stop')
        stop.clicked.connect(owner.hold)
        info.addWidget(stop)
        layout.addLayout(info)
        QApplication.instance().installEventFilter(self)

    def switch_mode(self):
        index = 1-self.mode_stack.currentIndex()
        self.mode_stack.setCurrentIndex(index)
        self.more.setText('切换到方向盘 / WASD' if index else '切换到精确 XYZ 输入')
        if index and self.owner.fresh():
            self.copy_xyz()

    def copy_xyz(self):
        try:
            for target, value in zip(self.xyz_targets, self.actual()):
                target.setValue(value)
        except ValueError as e:
            self.owner.error(e)

    def move_xyz(self):
        try:
            if self.owner.arm and (self.owner.arm.teaching or self.owner.arm.playing):
                raise ValueError('请先结束示教或回放')
            values = self.actual()
            values[:3] = [target.value() for target in self.xyz_targets]
            self.owner.arm.submit(pose_command(values))
        except Exception as e:
            self.owner.error(e)

    def eventFilter(self, watched, event):
        if event.type() != QEvent.KeyPress or not self.owner.isActiveWindow() or not self.isVisible() or self.mode_stack.currentIndex() != 0:
            return False
        focus = QApplication.focusWidget()
        current = focus
        while current:
            if isinstance(current, (QLineEdit, QAbstractSpinBox, QPlainTextEdit)):
                return False
            current = current.parentWidget()
        keys = {Qt.Key_W: (1, 1), Qt.Key_S: (1, -1), Qt.Key_A: (0, -1), Qt.Key_D: (0, 1),
                Qt.Key_E: (2, 1), Qt.Key_Q: (2, -1)}
        if event.key() == Qt.Key_Space:
            if not event.isAutoRepeat():
                self.owner.hold()
            return True
        if event.key() in keys and not event.modifiers() & (Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier):
            if not event.isAutoRepeat() and self.disc.motion_enabled:
                axis, sign = keys[event.key()]
                self.move(axis, sign*(10 if event.modifiers() & Qt.ShiftModifier else 1))
            return True
        return False

    def actual(self):
        pose = self.owner.shared.pose
        fields = ('x', 'y', 'z', 'tit', 'r', 'g')
        if not self.owner.fresh() or not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k]) for k in fields):
            raise ValueError('缺少新鲜的末端坐标反馈')
        return [pose[k] for k in fields[:3]]+[math.degrees(pose[k]) for k in fields[3:]]

    def move(self, axis, distance):
        try:
            if self.owner.arm and (self.owner.arm.teaching or self.owner.arm.playing):
                raise ValueError('请先结束手动示教或停止回放')
            values = self.actual()
            values[axis] += distance
            self.owner.arm.submit(pose_command(values))
            if axis in (0, 1):
                self.disc.flash(axis, distance)
        except Exception as e:
            self.owner.error(e)

    def refresh(self):
        enabled = False
        try:
            values = self.actual()
            self.position.setText('   '.join(f'{axis}  {value:.1f}' for axis, value in zip('XYZ', values)))
            enabled = not self.owner.arm.teaching and not self.owner.arm.playing
            self.state.setText('单位 mm · 实际反馈' if enabled else '示教 / 回放中 · 方向控制已暂停')
        except ValueError:
            self.position.setText('X  —     Y  —     Z  —')
            self.state.setText('离线 / 坐标过期 · 方向控制已暂停')
        self.disc.motion_enabled = enabled
        self.disc.update()
        for button in self.buttons:
            button.setEnabled(enabled)
        self.precise_read.setEnabled(enabled)
        self.precise_go.setEnabled(enabled)
