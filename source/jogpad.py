"""Discrete Cartesian jog pad; every click uses fresh measured position."""
import math
from PySide6.QtCore import Qt, Signal, QRectF, QPointF, QEvent, QTimer, QSize
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPainterPath
from PySide6.QtWidgets import QWidget, QGroupBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QApplication, QLineEdit, QAbstractSpinBox, QPlainTextEdit, QStackedWidget, QDoubleSpinBox, QFormLayout
from backend import BASE_FEEDBACK_ERROR
from PySide6.QtWidgets import QScrollArea, QSizePolicy
from host_direct import plan_xyz, holding_offset
from host_control import TargetIntent, XYZ_DEGRADED_MM, base_in_sector


class XYDisc(QWidget):
    jog = Signal(int, float)
    halt = Signal()
    home = Signal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(148, 148)
        self.setMaximumSize(350, 350)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(Qt.PointingHandCursor)
        self.setAccessibleName('XY 末端方向盘：WASD 移动，中心回中')
        self.setToolTip('内圈 1 mm · 外圈 10 mm · 中心回中\nWASD 控制 XY，Q/E 控制 Z，Shift 切换 10 mm')
        self.motion_enabled = False
        self.hovered = None
        self.pressed = None
        self.press_generation = 0
        self.drag_origin = None
        self.dragged = False
        self.setMouseTracking(True)

    def sizeHint(self):
        return QSize(210, 210)

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
        if self.drag_origin is not None:
            delta=event.globalPosition()-self.drag_origin
            if delta.manhattanLength()>=QApplication.startDragDistance():
                self.dragged=True
            if self.dragged:
                self.pressed=None
                if self.scroll_parent:
                    self.scroll_parent.verticalScrollBar().setValue(self.scroll_origin-round(delta.y()))
                self.update()
                return
        self.hovered = self.hit(event.position())
        self.update()

    def leaveEvent(self, event):
        self.hovered = None
        self.update()

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        self.setFocus()
        self.drag_origin=event.globalPosition()
        self.dragged=False
        self.scroll_parent=self.parentWidget()
        while self.scroll_parent and not isinstance(self.scroll_parent,QScrollArea):
            self.scroll_parent=self.scroll_parent.parentWidget()
        if self.scroll_parent:
            self.scroll_origin=self.scroll_parent.verticalScrollBar().value()
        action = self.hit(event.position())
        if action is not None:
            self.pressed = action
            self.press_generation += 1
            self.update()
    def mouseReleaseEvent(self, event):
        if event.button()!=Qt.LeftButton or self.drag_origin is None:
            return
        moved=(event.globalPosition()-self.drag_origin).manhattanLength()
        self.drag_origin=None
        action=self.hit(event.position())
        if self.motion_enabled and not self.dragged and moved<QApplication.startDragDistance() and action==self.pressed:
            if action=='home':
                self.home.emit()
            elif action is not None:
                self.jog.emit(*action)
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
        p.setFont(QFont('Microsoft YaHei UI', min(14, max(8, round(radius / 11)))))
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
                colors = ('#315a80', '#283d50', '#242a31', '#202327')
                p.fillPath(path,QColor(colors[0 if pressed else 1 if hover else 2 if ring==0 else 3]))
                p.setPen(QPen(QColor('#73bfff' if pressed else '#424a53'), 2 if pressed else 1))
                p.drawPath(path)
                angle = math.radians(direction*90)
                distance = radius*(inner+outer)/2
                tx, ty = cx+distance*math.cos(angle), cy-distance*math.sin(angle)
                p.setPen(QColor('#f1f4f7' if self.motion_enabled else '#a7b2bd'))
                text = ('X' if axis == 0 else 'Y')+('+' if sign > 0 else '−')+f'{step}'
                p.drawText(QRectF(tx-29, ty-13, 58, 26), Qt.AlignCenter, text)
        p.setPen(QPen(QColor('#50606d'), 1))
        p.setBrush(QColor('#315a80' if self.pressed == 'home' else '#283d50' if self.hovered == 'home' else '#15171a'))
        p.drawEllipse(QPointF(cx, cy), radius*.22, radius*.22)
        p.setPen(QColor('#e8eef5'))
        p.drawText(QRectF(cx-radius*.22, cy-12, radius*.44, 24), Qt.AlignCenter, '回中')
        if self.hasFocus():
            p.setPen(QPen(QColor('#73bfff'), 1))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), radius+2, radius+2)


class CartesianPad(QGroupBox):
    def __init__(self, owner):
        super().__init__('')
        self.setObjectName('jogPad')
        self.owner = owner
        self.target_intent = TargetIntent()
        self.jog_offset = [0.0]*6
        self.xyz_degradation = ''
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 1, 5, 1)
        layout.setSpacing(4)
        self.position = QWidget()
        self.position.setObjectName('coordinates')
        coords = QHBoxLayout(self.position)
        coords.setContentsMargins(5, 6, 5, 6)
        coords.setSpacing(3)
        self.coordinate_values = []
        for axis in 'XYZ':
            cell = QHBoxLayout()
            cell.setSpacing(3)
            name = QLabel(axis)
            name.setObjectName('coordinateAxis')
            value = QLabel('—')
            value.setObjectName('coordinateValue')
            value.setAccessibleName(f'{axis} 实际坐标（毫米）')
            cell.addStretch()
            cell.addWidget(name)
            cell.addWidget(value)
            cell.addStretch()
            coords.addLayout(cell, 1)
            self.coordinate_values.append(value)
        self.state = QLabel('等待实际末端坐标')
        self.state.setObjectName('quiet')
        self.mode_stack = QStackedWidget()
        self.mode_stack.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Ignored)
        layout.addWidget(self.mode_stack)
        disc_page = QWidget()
        row = QHBoxLayout(disc_page)
        row.setContentsMargins(0, 0, 0, 0)
        self.mode_stack.addWidget(disc_page)
        self.disc = XYDisc()
        self.disc.jog.connect(self.move)
        self.disc.halt.connect(owner.hold)
        self.disc.home.connect(owner.home)
        row.addWidget(self.disc, 1)
        zcol = QVBoxLayout()
        zcol.addStretch()
        self.buttons = []
        for label, delta in (('Z+10', 10), ('Z+1', 1), ('Z−1', -1), ('Z−10', -10)):
            button = QPushButton(label)
            button.setObjectName('jogZ')
            button.setMinimumWidth(58)
            button.clicked.connect(lambda checked=False, d=delta: self.move(2, d))
            self.buttons.append(button)
            zcol.addWidget(button)
        zcol.addStretch()
        row.addLayout(zcol)
        precise_page = QWidget()
        precise_layout = QVBoxLayout(precise_page)
        form = QFormLayout()
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.xyz_targets = []
        for axis, limit in zip('XYZ', (500, 500, 600)):
            spin = QDoubleSpinBox()
            spin.setObjectName('xyzTarget')
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
        precise_layout.addWidget(QLabel('优先保留姿态，目标不可达时降标。'))
        precise_layout.addStretch()
        self.mode_stack.addWidget(precise_page)
        self.mode_stack.setMinimumHeight(disc_page.minimumSizeHint().height())
        self.jog_status = QWidget()
        info = QHBoxLayout(self.jog_status)
        info.setContentsMargins(7, 3, 7, 4)
        self.execution_mode = QLabel()
        self.execution_mode.setAlignment(Qt.AlignCenter)
        self.set_execution_mode(False)
        info.addWidget(self.execution_mode, 1)
        hint = QLabel('内圈 1 mm · 外圈 10 mm\nWASD / Q E')
        hint.setToolTip('每次点击或按键移动一步，按住不连发；编辑输入框时不触发快捷键。中心回中会请求恢复初始姿态。')
        hint.setObjectName('quiet')
        info.addWidget(hint, 1)
        self.control_settings = QWidget()
        control_layout = QVBoxLayout(self.control_settings)
        bias_row = QFormLayout()
        self.z_feedforward = QDoubleSpinBox()
        self.z_feedforward.setRange(0, 50)
        self.z_feedforward.setSingleStep(1)
        self.z_feedforward.setSuffix(' mm')
        self.z_feedforward.setToolTip('仅电脑逆解使用。相对点动接管时保留原保持目标已有的抬升量，前馈只补足差额。0 为关闭，修改后在下次移动时生效。')
        bias_row.addRow('带载 Z 前馈', self.z_feedforward)
        control_layout.addLayout(bias_row)
        grip_row = QHBoxLayout()
        self.grip_close = QPushButton('收紧夹爪')
        self.grip_open = QPushButton('松开夹爪')
        self.grip_close.setToolTip('仅控制夹爪到闭合位置；手臂关节保持原样。')
        self.grip_open.setToolTip('仅打开夹爪，不释放机械臂关节扭矩。')
        self.grip_close.clicked.connect(lambda: owner.grip(True))
        self.grip_open.clicked.connect(lambda: owner.grip(False))
        grip_row.addWidget(self.grip_close)
        grip_row.addWidget(self.grip_open)
        control_layout.addLayout(grip_row)
        self.more = QPushButton('精确 XYZ 输入')
        self.more.clicked.connect(self.switch_mode)
        stop = QPushButton('取消后续动作 / 回放')
        stop.setObjectName('stop')
        stop.setToolTip('取消待发动作；已发送的目标可能继续执行，不改写保持目标、不释放扭矩。')
        stop.clicked.connect(owner.hold)
        control_layout.addWidget(stop)
        QApplication.instance().installEventFilter(self)

    def set_execution_mode(self, degraded, detail=''):
        name, text = ('degradedMode', '● 降标执行') if degraded else ('strictMode', '● 严格执行')
        tip = detail or '末端位置偏差超过 10 mm 时显示降标。'
        if self.execution_mode.toolTip() != tip:
            self.execution_mode.setToolTip(tip)
        if self.execution_mode.objectName() != name:
            self.execution_mode.setObjectName(name)
            self.execution_mode.setText(text)
            self.execution_mode.style().unpolish(self.execution_mode)
            self.execution_mode.style().polish(self.execution_mode)

    def update_execution_mode(self, replay):
        xyz_current = self.target_intent.key == self.intent_key()
        if replay and replay.arm is self.owner.arm and (self.owner.arm.playing or not xyz_current):
            detail = f'回放到位检查：末端偏差 {replay.xyz_error_mm:.1f} mm' if replay.degraded else ''
            self.set_execution_mode(replay.degraded, detail)
        else:
            detail = self.xyz_degradation if xyz_current else ''
            self.set_execution_mode(bool(detail), detail)

    def switch_mode(self):
        index = 1-self.mode_stack.currentIndex()
        self.mode_stack.setCurrentIndex(index)
        self.mode_stack.setMinimumHeight(self.mode_stack.currentWidget().minimumSizeHint().height())
        self.more.setText('返回轮盘 / WASD' if index else '精确 XYZ 输入')
        if index and self.owner.fresh():
            self.copy_xyz()

    def copy_xyz(self):
        try:
            self.target_intent.clear()
            for target, value in zip(self.xyz_targets, self.actual()):
                target.setValue(value)
        except ValueError as e:
            self.owner.error(e)

    def move_xyz(self):
        try:
            if self.owner.arm and (self.owner.arm.teaching or self.owner.arm.playing):
                raise ValueError('请先结束示教或回放')
            values = self.intent_values()
            values[:3] = [target.value() for target in self.xyz_targets]
            self.send_xyz(values)
        except Exception as e:
            self.owner.error(e)

    def eventFilter(self, watched, event):
        if event.type() != QEvent.KeyPress or not self.owner.isActiveWindow() or not self.isVisible() or self.mode_stack.currentIndex() != 0:
            return False
        focus = QApplication.focusWidget()
        current = focus
        while current:
            if isinstance(current, (QLineEdit, QAbstractSpinBox, QPlainTextEdit)) or current.property('captionControl'):
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

    def send_xyz(self, values, offset=None):
        if not self.owner.fresh():raise ValueError('缺少新鲜的关节反馈')
        key = self.intent_key()
        with self.owner.shared.lock:
            measured=[self.owner.shared.pose[k] for k in ('b','s','e','t','r','g')]
        reference = self.owner.arm.last_joint_target
        reference = reference.copy() if reference is not None else None
        bias=self.z_feedforward.value()
        offset = [0.0]*6 if offset is None else offset
        cmd, joints, accepted, detail = plan_xyz(values, measured, self.owner.speed.value(),
            self.owner.acc.value(), bias, offset, reference, allow_degraded=True)
        if key != self.intent_key():
            raise ValueError('运动基准已更新，请重试')
        # Internal routing flag is consumed by Arm before serializing JSON.
        # Direct PC jogs retain intent; manual angle/home commands invalidate it.
        self.owner.arm.submit(dict(cmd,_host_intent=True))
        miss = math.dist(values[:3], accepted[:3])
        self.xyz_degradation = f'预计末端偏差 {miss:.1f} mm。{detail}' if miss > XYZ_DEGRADED_MM else ''
        self.set_execution_mode(bool(self.xyz_degradation), self.xyz_degradation)
        if detail:
            self.owner.shared.event(f'XYZ {self.xyz_degradation or detail}')
        for widget,angle in zip(self.owner.targets,joints):widget.setValue(math.degrees(angle))
        self.owner.shared.event(f'电脑逆解 Z目标 {values[2]:.2f} mm，接受 {accepted[2]:.2f} mm，前馈 +{bias:.1f} mm，保持偏移 {offset[2]:+.2f} mm，关节目标 {cmd}')
        self.jog_offset = offset
        self.target_intent.commit(accepted, key)

    def intent_key(self):
        return id(self.owner.arm),getattr(self.owner.arm,'reference_generation',0)

    def intent_values(self):
        actual=self.actual()
        return self.target_intent.values(actual,self.intent_key())

    def move(self, axis, distance):
        try:
            if self.owner.arm and (self.owner.arm.teaching or self.owner.arm.playing):
                raise ValueError('请先结束手动示教或停止回放')
            values = self.intent_values()
            offset = self.jog_offset
            if self.target_intent.key != self.intent_key():
                offset = holding_offset(values,self.owner.arm.last_joint_target)
            values[axis] += distance
            self.send_xyz(values, offset)
            if axis in (0, 1):
                self.disc.flash(axis, distance)
        except Exception as e:
            self.owner.error(e)

    def refresh(self):
        enabled = False
        coordinates = ('—',) * 3
        try:
            values = self.actual()
            coordinates = tuple(f'{value:.1f}' for value in values[:3])
            enabled = (not self.owner.arm.teaching and not self.owner.arm.playing
                       and not self.owner.arm.torque_pending and not self.owner.arm.torque_fault)
            self.state.setText('单位 mm · 实际反馈' if enabled else '示教 / 回放中 · 方向控制已暂停')
            if not base_in_sector(self.owner.shared.pose.get('b'), feedback=True):
                enabled = False
                self.state.setText(BASE_FEEDBACK_ERROR)
            elif getattr(self.owner.arm,'host_busy',False):
                self.state.setText('电脑轨迹定位中 · 关节跟随检查')
        except ValueError:
            self.target_intent.clear()
            self.state.setText('离线 / 坐标过期 · 方向控制已暂停')
        for label, text in zip(self.coordinate_values, coordinates):
            if label.text() != text:
                label.setText(text)
        self.disc.motion_enabled = enabled
        self.disc.update()
        for button in self.buttons:
            button.setEnabled(enabled)
        self.precise_read.setEnabled(enabled)
        self.precise_go.setEnabled(enabled)
        self.grip_close.setEnabled(enabled)
        self.grip_open.setEnabled(enabled)
