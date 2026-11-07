import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
from PySide6.QtCore import Qt, QTimer, QSettings
from PySide6.QtGui import QIcon, QImage, QPixmap
from PySide6.QtNetwork import QLocalServer, QLocalSocket
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QHBoxLayout, QVBoxLayout, QGridLayout, QComboBox, QSpinBox, QDoubleSpinBox,
    QGroupBox, QScrollArea, QFileDialog, QMessageBox, QSlider, QTabBar, QStackedWidget, QFrame, QCheckBox, QScroller, QSizePolicy)
from serial.tools import list_ports
from backend import Shared, Arm, UVC, Gemini, Recorder, KEYS, NAMES, LIMITS, joint_command, depth_views
from backend import all_command, TorqueChange, BASE_FEEDBACK_ERROR
from host_control import base_in_sector
from version import VERSION
from controls import Controls
from jogpad import CartesianPad
from appearance import STYLE, CameraScreen
from arm_view import ArmView
from workbench import Diagnostics, StatusStrip, panel

BASE = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent.parent


class Foldout(QWidget):
    def __init__(self, content):
        super().__init__()
        self.title = content.title()
        self.content = content
        content.setTitle('')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        self.toggle = QPushButton('▸  '+self.title)
        self.toggle.setCheckable(True)
        self.toggle.setObjectName('foldout')
        self.toggle.toggled.connect(self.expand)
        layout.addWidget(self.toggle)
        layout.addWidget(content)
        content.setVisible(False)

    def expand(self, checked):
        self.content.setVisible(checked)
        self.toggle.setText(('▾  ' if checked else '▸  ')+self.title)


class View(QFrame):
    def __init__(self, title, tag):
        super().__init__()
        self.setObjectName('pane')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        bar = QWidget()
        bar.setObjectName('paneHeader')
        header = QHBoxLayout(bar)
        header.setContentsMargins(8, 4, 8, 4)
        name = QLabel(title)
        name.setObjectName('paneTitle')
        header.addWidget(name)
        header.addStretch()
        kind = QLabel(tag)
        kind.setObjectName('quiet')
        header.addWidget(kind)
        layout.addWidget(bar)
        self.image = CameraScreen()
        self.meta = QLabel('等待连接')
        self.meta.setObjectName('quiet')
        self.meta.setContentsMargins(8, 3, 8, 3)
        self.meta_time = 0
        layout.addWidget(self.image, 1)
        layout.addWidget(self.meta)

    def show_image(self, bgr, meta):
        # Resize before color conversion/copy: preview is much smaller than 1080p.
        h, w = bgr.shape[:2]
        factor = min(self.image.width()/w, self.image.height()/h)
        if factor > 0 and factor < 1:
            bgr = cv2.resize(bgr, (max(1, round(w*factor)), max(1, round(h*factor))), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        self.image.setPixmap(QPixmap.fromImage(image))
        if time.perf_counter()-self.meta_time >= .25:
            self.meta.setText(meta)
            self.meta_time = time.perf_counter()


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('RoArmStudio')
        self.prefs = QSettings('RoArmStudio','PC-control')
        self.auto_paused = False
        self.pending_device = None
        self.camera_pause=set()
        self.camera_next={} 
        self.resize(1366, 780)
        self.setStyleSheet(STYLE)
        self.shared = Shared()
        self.arm = self.uvc = self.gemini = self.recorder = None
        self.workers = []
        self.controls = None
        self.folder = BASE/'captures'
        root = QWidget()
        root.setObjectName('workbench')
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        titlebar = QWidget()
        titlebar.setObjectName('titlebar')
        top = QHBoxLayout(titlebar)
        top.setContentsMargins(10, 0, 10, 0)
        title = QLabel('RoArmStudio')
        title.setObjectName('appTitle')
        top.addWidget(title)
        self.control_tabs = QTabBar()
        self.control_tabs.setExpanding(False)
        self.control_tabs.setDrawBase(False)
        for name in ('视觉工作台', '关节控制', '示教', '控制/连接'):
            self.control_tabs.addTab(name)
        top.addWidget(self.control_tabs)
        top.addStretch()
        top.addWidget(QLabel(f'v{VERSION}'))
        outer.addWidget(titlebar)
        self.status_strip = StatusStrip(self.hold)
        outer.addWidget(self.status_strip)
        self.pages = QStackedWidget()
        outer.addWidget(self.pages, 1)
        self.control_tabs.currentChanged.connect(self.pages.setCurrentIndex)
        self.settings_sections = []
        box = QGroupBox('机械臂连接')
        l = QVBoxLayout(box)
        row = QHBoxLayout()
        self.ports = QComboBox()
        self.ports.setMinimumWidth(180)
        self.ports.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        row.addWidget(self.ports, 1)
        self.button(row, '刷新', self.refresh_ports)
        l.addLayout(row)
        self.arm_btn = self.button(l, '连接 M3-Pro · USB', self.toggle_arm)
        self.arm_state = QLabel('115200 baud · 等待设备')
        l.addWidget(self.arm_state)
        self.auto_connect = QCheckBox('自动连接到上次的设备（机械臂和相机）')
        self.auto_connect.setChecked(self.prefs.value('autoConnect',False,type=bool))
        self.auto_connect.toggled.connect(self.auto_changed)
        l.addWidget(self.auto_connect)
        self.connection_box = box
        box = QGroupBox('')
        self.joint_panel = box
        l = QVBoxLayout(box)
        l.addWidget(QLabel('原生角度 · 夹爪角度减小时张开'))
        grid = QGridLayout()
        self.targets, self.actual, self.motion = [], [], []
        for i, name in enumerate(NAMES):
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            heading = QHBoxLayout()
            heading.addWidget(QLabel(name))
            actual = QLabel('—')
            heading.addWidget(actual)
            cell_layout.addLayout(heading)
            self.actual.append(actual)
            spin = QDoubleSpinBox()
            spin.setRange(*LIMITS[i])
            spin.setDecimals(1)
            spin.setSuffix('°')
            spin.setSingleStep(1)
            controls = QHBoxLayout()
            controls.addWidget(spin)
            self.targets.append(spin)
            btn = QPushButton('移动')
            btn.clicked.connect(lambda checked=False, index=i: self.move(index))
            controls.addWidget(btn)
            cell_layout.addLayout(controls)
            grid.addWidget(cell, i//2, i%2)
            self.motion.append(btn)
        l.addLayout(grid)
        row = QHBoxLayout()
        self.copy_btn = self.button(row, '读取实际角度', self.copy_pose)
        self.motion.append(self.button(row, '全关节到目标', self.move_all))
        self.motion.append(self.button(row, '回初始位', self.home))
        l.addLayout(row)
        self.motion.append(self.copy_btn)
        row = QHBoxLayout()
        row.addWidget(QLabel('速度'))
        self.speed = QSpinBox()
        self.speed.setRange(20, 600)
        self.speed.setValue(150)
        self.speed.setToolTip('关节控制与回放：舵机步/秒；XYZ 指令速度由固件处理')
        row.addWidget(self.speed)
        row.addWidget(QLabel('加速度'))
        self.acc = QSpinBox()
        self.acc.setRange(1, 254)
        self.acc.setValue(5)
        self.acc.setToolTip('单位：100 步/秒²；只用于关节控制与回放')
        row.addWidget(self.acc)
        l.addLayout(row)
        l.addStretch()
        row = QHBoxLayout()
        self.hold_btn = self.button(row, '取消后续动作 / 回放', self.hold)
        self.hold_btn.setToolTip('取消待发动作；已发送的目标可能继续执行，不改写保持目标、不释放扭矩。')
        self.hold_btn.setObjectName('stop')
        self.teach_btn = self.button(row, '释放扭矩 · 手动示教', self.teach)
        l.addLayout(row)
        box = QGroupBox('相机连接')
        l = QVBoxLayout(box)
        row = QHBoxLayout()
        row.addWidget(QLabel('AR0234 / UVC 索引'))
        self.cam_index = QSpinBox()
        self.cam_index.setRange(-1, 15)
        self.cam_index.setSpecialValueText('自动选择 DECXIN / AR0234')
        self.cam_index.setValue(self.prefs.value("cameraIndex",-1,type=int))
        row.addWidget(self.cam_index)
        l.addLayout(row)
        fps_row = QHBoxLayout()
        fps_row.addWidget(QLabel('相机目标帧率'))
        self.camera_fps = QComboBox()
        self.camera_fps.addItem('60 fps · 流畅（Gemini BS 模式）', 60)
        self.camera_fps.addItem('30 fps · 深度 Dense 模式', 30)
        self.camera_fps.setToolTip('在打开相机前选择；60 帧深度使用 BS 模式，精细度降低。')
        self.camera_fps.setCurrentIndex(max(0,self.camera_fps.findData(self.prefs.value("cameraFps",60,type=int))))
        fps_row.addWidget(self.camera_fps)
        l.addLayout(fps_row)
        self.uvc_btn = self.button(l, '打开 AR0234', self.toggle_uvc)
        self.gem_btn = self.button(l, '打开 Gemini 2 · RGB-D', self.toggle_gemini)
        l.addWidget(QLabel('默认按设备名称选择 AR0234；手动索引使用 DirectShow。'))
        section = Foldout(box)
        self.settings_sections.append(section)
        box = QGroupBox('采集与保存')
        l = QVBoxLayout(box)
        self.path_label = QLabel(str(self.folder))
        self.path_label.setWordWrap(True)
        self.path_label.setMinimumWidth(0)
        self.path_label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        l.addWidget(self.path_label)
        self.button(l, '选择保存目录', self.select_folder)
        row = QHBoxLayout()
        self.button(row, '保存快照', self.snapshot)
        self.record_btn = self.button(row, '开始采集', self.record)
        l.addLayout(row)
        l.addWidget(QLabel('图像＋原始深度＋关节反馈，按主机时间记录。'))
        section = Foldout(box)
        self.settings_sections.append(section)
        self.cartesian_pad = CartesianPad(self)
        self.diagnostics = Diagnostics()
        self.log = self.diagnostics.log
        self.arm_view = ArmView(BASE/'assets'/'roarm_m3')
        self.views = [View('手部相机', 'CAM1'), View('彩色深度融合', 'CAM2')]
        self.build_workbench()
        self.pages.addWidget(self.scroll_page(self.joint_panel))

        box = QGroupBox('融合显示')
        row = QHBoxLayout(box)
        row.addWidget(QLabel('深度显示上限'))
        self.far = QSpinBox()
        self.far.setRange(300, 10000)
        self.far.setValue(2000)
        self.far.setSuffix(' mm')
        row.addWidget(self.far)
        row.addWidget(QLabel('叠加比例'))
        self.alpha = QSlider(Qt.Horizontal)
        self.alpha.setRange(0, 100)
        self.alpha.setValue(45)
        row.addWidget(self.alpha)
        self.settings_sections.append(Foldout(box))
        credit = QLabel('By Azucia')
        credit.setObjectName('quiet')
        credit.setAccessibleName(f'RoarmStudio {VERSION} · By Azucia')
        credit.setToolTip(f'RoarmStudio {VERSION}')
        footer = QWidget()
        footer.setObjectName('footer')
        row = QHBoxLayout(footer)
        row.setContentsMargins(8, 2, 8, 2)
        row.addWidget(self.cartesian_pad.state, 1)
        row.addWidget(credit)
        outer.addWidget(footer)
        self.controls = Controls(self)
        playback=self.controls.tabs.widget(0)
        self.controls.tabs.removeTab(0)
        self.pages.addWidget(self.scroll_page(playback))
        device=self.controls.device_page
        device_box=QGroupBox('设备')
        QVBoxLayout(device_box).addWidget(device)
        link_page=QWidget()
        link_layout=QVBoxLayout(link_page)
        link_layout.addWidget(self.cartesian_pad.computer_mode)
        details=QWidget()
        detail_layout=QVBoxLayout(details)
        detail_layout.setContentsMargins(0,0,0,0)
        detail_layout.addWidget(self.cartesian_pad.control_settings)
        detail_layout.addWidget(self.connection_box)
        detail_layout.addWidget(Foldout(device_box))
        for section in self.settings_sections:
            detail_layout.addWidget(section)
        detail_layout.addStretch()
        link_layout.addWidget(self.scroll_page(details),1)
        self.pages.addWidget(link_page)
        self.refresh_ports()
        self.auto_timer=QTimer(self)
        self.auto_timer.timeout.connect(self.try_previous)
        self.auto_timer.start(5000)
        QTimer.singleShot(0,self.try_previous)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.rendered_frames = {}
        self.preview_times = {}
        self.video_timer = QTimer(self)
        self.video_timer.setTimerType(Qt.PreciseTimer)
        self.video_timer.timeout.connect(self.render_frames)
        self.video_timer.start(16)
        self.note('工作台已打开')

    def build_workbench(self):
        page = QWidget()
        page.setObjectName('workbench')
        body = QHBoxLayout(page)
        body.setContentsMargins(7, 7, 7, 7)
        body.setSpacing(6)
        left = QVBoxLayout()
        left.setSpacing(6)
        jog, jog_layout = panel('手动点动', 'BASE · mm', self.cartesian_pad.more)
        jog_layout.addWidget(self.cartesian_pad.position)
        jog_layout.addWidget(self.scroll_page(self.cartesian_pad), 1)
        jog_layout.addWidget(self.cartesian_pad.jog_status)
        left.addWidget(jog, 1)
        left.addWidget(self.diagnostics, 1)
        sidebar = QWidget()
        sidebar.setMinimumWidth(312)
        sidebar.setMaximumWidth(425)
        sidebar.setLayout(left)
        body.addWidget(sidebar, 3)
        right = QVBoxLayout()
        right.setSpacing(6)
        pose, pose_layout = panel('机械臂状态')
        pose_layout.addWidget(self.arm_view, 1)
        self.tcp = QLabel('TCP · 等待反馈')
        self.tcp.setObjectName('quiet')
        self.tcp.setContentsMargins(8, 3, 8, 3)
        pose_layout.addWidget(self.tcp)
        right.addWidget(pose, 3)
        cameras = QHBoxLayout()
        cameras.setSpacing(6)
        for view in self.views:
            cameras.addWidget(view, 1)
        right.addLayout(cameras, 2)
        body.addLayout(right, 7)
        self.pages.addWidget(page)

    def button(self, layout, text, fn):
        btn = QPushButton(text)
        btn.clicked.connect(fn)
        layout.addWidget(btn)
        return btn

    def note(self, text):
        self.log.appendPlainText(datetime.now().strftime('%H:%M:%S')+'  '+text)

    def error(self, e):
        self.note(str(e))
        QMessageBox.warning(self, '操作未完成', str(e))

    @staticmethod
    def scroll_page(content):
        scroll=QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)
        QScroller.grabGesture(scroll.viewport(),QScroller.TouchGesture)
        return scroll

    def refresh_ports(self):
        selected=self.ports.currentData()
        self.ports.clear()
        for p in list_ports.comports():
            self.ports.addItem(f'{p.device} · {p.description}',p.device)
        index=self.ports.findData(selected)
        if index>=0:
            self.ports.setCurrentIndex(index)

    @staticmethod
    def device_key(p):
        if p.serial_number and p.vid is not None and p.pid is not None:
            return f'{p.vid:04x}:{p.pid:04x}:{p.serial_number}'
        return None

    def auto_changed(self,checked):
        self.prefs.setValue('autoConnect',checked)
        self.auto_paused=False
        if checked:
            self.try_previous()

    def try_previous(self):
        if not self.auto_connect.isChecked():
            return
        for kind in ('uvc','gemini'):
            worker=getattr(self,kind)
            saved=self.prefs.value(kind+'Device','')
            if (saved and self.prefs.value(kind+'RememberOpen',False,type=bool)
                    and kind not in self.camera_pause and not (worker and worker.is_alive())
                    and time.monotonic()>=self.camera_next.get(kind,0)):
                self.open_camera(kind,saved)
        if self.auto_paused or (self.arm and self.arm.is_alive()):
            return
        saved=self.prefs.value('previousArm','')
        if saved:
            matches=[p for p in list_ports.comports() if self.device_key(p)==saved]
            if len(matches)==1:
                self.connect_arm(matches[0])

    def start_worker(self, worker):
        self.workers.append(worker)
        worker.start()
        return worker

    def toggle_arm(self):
        if self.arm and self.arm.is_alive():
            self.auto_paused=True
            self.arm.cancel_replay()
            self.arm.stop.set()
            return
        port = self.ports.currentData()
        if not port:
            self.error('未发现串口，请连接机械臂后刷新。')
            return
        device=next((p for p in list_ports.comports() if p.device==port),None)
        if device is None:
            self.error('所选串口已断开，请刷新。')
            return
        self.auto_paused=False
        self.connect_arm(device)

    def connect_arm(self,device):
        port=device.device
        self.pending_device=(self.device_key(device),port)
        idx=self.ports.findData(port)
        if idx<0:
            self.refresh_ports()
            idx=self.ports.findData(port)
        self.ports.setCurrentIndex(idx)
        self.shared.pose = None
        self.shared.pose_time = 0
        self.arm = self.start_worker(Arm(self.shared, port))

    def open_camera(self,kind,identity=None):
        fps=self.camera_fps.currentData()
        worker=(UVC(self.shared,self.cam_index.value(),fps,identity) if kind=='uvc'
                else Gemini(self.shared,fps,identity))
        self.camera_next[kind]=time.monotonic()+30
        with self.shared.lock:
            self.shared.frames.pop(kind, None)
        self.rendered_frames.pop(kind, None)
        self.preview_times.pop(kind, None)
        setattr(self,kind,self.start_worker(worker))

    def toggle_camera(self,kind):
        worker=getattr(self,kind)
        if worker and worker.is_alive():
            self.camera_pause.add(kind)
            self.prefs.setValue(kind+'RememberOpen',False)
            worker.stop.set()
        else:
            self.camera_pause.discard(kind)
            self.open_camera(kind)

    def toggle_uvc(self):
        self.toggle_camera('uvc')

    def toggle_gemini(self):
        self.toggle_camera('gemini')

    def grip(self, close):
        try:
            if not self.fresh() or self.arm.teaching or self.arm.playing:
                raise ValueError('夹爪操作需要正常反馈，并先结束示教或回放')
            # Dedicated single-joint command: never move the arm for a gripper click.
            command = joint_command(5, 180 if close else math.degrees(1.08), 100)
            command['acc'] = 5
            self.arm.submit(command)
        except Exception as e:
            self.error(e)

    def fresh(self):
        return bool(self.arm and self.arm.connected and self.shared.pose and
                    time.monotonic()-self.shared.pose_time < .6)

    def move(self, index):
        try:
            if not self.fresh():
                raise ValueError('没有新鲜的实际关节反馈，无法移动')
            command = joint_command(index, self.targets[index].value(), self.speed.value())
            command['acc'] = self.acc.value()
            self.arm.submit(command)
        except Exception as e:
            self.error(e)

    def copy_pose(self):
        if not self.fresh():
            return
        for i, key in enumerate(KEYS):
            self.targets[i].setValue(math.degrees(self.shared.pose[key]))
        self.note('已将实际角度复制到目标框；未发送运动指令。')

    def hold(self):
        # Replacing a loaded holding target with encoder angles can cause sag.
        # The native joint protocol has no verified instantaneous stop command.
        if self.arm:
            self.arm.cancel_replay()
        self.cartesian_pad.target_intent.clear()
        self.note('已取消待发动作和回放；已发送的目标可能继续执行。未改写保持目标，未释放扭矩。')

    def move_all(self):
        try:
            if not self.fresh():
                raise ValueError('没有新鲜的实际关节反馈')
            self.arm.submit(all_command([math.radians(s.value()) for s in self.targets],
                                        self.speed.value(), self.acc.value()))
        except Exception as e:
            self.error(e)

    def home(self):
        if not self.fresh() or self.arm.teaching or self.arm.playing:
            self.error('回中需要正常反馈，并先结束示教或回放。')
            return
        if not base_in_sector(self.shared.pose.get('b'), feedback=True):
            self.error(BASE_FEEDBACK_ERROR)
            return
        if QMessageBox.question(self, '回中 · 官方初始姿态',
            '会移动所有关节到初始姿态（不是将 XYZ 清零，也不改写中位）。确认周围没有障碍？') == QMessageBox.Yes:
            try:
                self.arm.submit(all_command([0, 0, math.pi/2, 0, 0, math.pi], 100, 5))
            except ValueError as e:
                self.error(e)

    def open_controls(self):
        self.control_tabs.setCurrentIndex(2)

    def teach(self):
        if not self.fresh():
            return
        off=all(self.shared.pose.get('torswitch'+k)==0 for k in ('B','S','E','T','R','G'))
        self.set_teaching(not (self.arm.teaching or off))

    def set_teaching(self, release):
        if not self.fresh() or self.arm.torque_pending:
            return
        text = ('请先托住机械臂及末端负载。确认安全固件后将直接释放扭矩，不预先移动关节。\n'
                '只有七个舵机都回读为关闭，才会显示“已释放”。继续？' if release else
                '请继续托住机械臂。将读取七个舵机当前位置，再在当前位置恢复扭矩。\n'
                '未确认成功前请不要松手。继续？')
        if QMessageBox.question(self, '手动示教扭矩确认', text) != QMessageBox.Yes:
            return
        try:
            self.start_worker(TorqueChange(self.shared, self.arm, release))
        except Exception as e:
            self.error(e)

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, '采集保存位置', str(self.folder.parent))
        if folder:
            self.folder = Path(folder)
            self.path_label.setText(folder)

    def destination(self, prefix):
        return self.folder/(prefix+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))

    def snapshot(self):
        if not self.shared.frames and not self.fresh():
            self.error('没有可保存的设备数据。')
            return
        recorder = Recorder(self.shared, self.destination('snapshot'))
        self.start_worker(recorder)
        QTimer.singleShot(350, recorder.stop.set)

    def record(self):
        if self.recorder and self.recorder.is_alive():
            self.recorder.stop.set()
        else:
            active = self.fresh() or any(time.monotonic()-p['monotonic'] < 1 for p in self.shared.frames.values())
            if not active:
                self.error('请先连接至少一个设备，并等待有效反馈或画面。')
                return
            self.recorder = self.start_worker(Recorder(self.shared, self.destination('session')))
            self.note('开始采集：主机接收时间对齐，不是硬件同步。')

    def tick(self):
        while not self.shared.events.empty():
            self.note(self.shared.events.get_nowait())
        for kind in ('uvc','gemini'):
            worker=getattr(self,kind)
            if worker and worker.identity and not worker.remembered and not worker.stop.is_set():
                self.prefs.setValue(kind+'Device',worker.identity)
                self.prefs.setValue(kind+'RememberOpen',True)
                self.prefs.setValue('cameraFps',worker.fps)
                if kind=='uvc':
                    self.prefs.setValue('cameraIndex',worker.index)
                worker.remembered=True
        fresh = self.fresh()
        if fresh and self.pending_device and self.arm.port==self.pending_device[1]:
            key,port=self.pending_device
            self.prefs.setValue('previousArm',key or '')
            self.pending_device=None
        teaching = bool(self.arm and self.arm.teaching)
        playing = bool(self.arm and self.arm.playing)
        torque_busy = bool(self.arm and (self.arm.torque_pending or self.arm.torque_fault))
        self.cartesian_pad.refresh()
        inside = fresh and base_in_sector(self.shared.pose.get('b'), feedback=True)
        for btn in self.motion:
            btn.setEnabled(inside and not teaching and not playing and not torque_busy)
        self.hold_btn.setEnabled(fresh and not teaching and not torque_busy)
        self.teach_btn.setEnabled(fresh and not playing and not (self.arm and self.arm.torque_pending))
        self.teach_btn.setText('恢复扭矩 · 锁定当前位置' if teaching or (fresh and all(self.shared.pose.get('torswitch'+k)==0 for k in ('B','S','E','T','R','G'))) else '释放扭矩 · 手动示教')
        self.arm_state.setText('反馈正常 · 手动示教' if fresh and teaching else '反馈正常 · USB 串口' if fresh else '等待反馈 / 离线')
        if fresh and (teaching or torque_busy or self.arm.torque_state != '未确认'):
            self.arm_state.setText(self.arm.torque_state)
        connected = bool(self.arm and self.arm.connected)
        now = time.monotonic()
        age = now-self.shared.pose_time if connected and self.shared.pose_time else None
        if connected and not fresh:
            self.arm_state.setText('等待下一帧反馈' if age is None else f'上次反馈 {age:.1f} 秒前')
        pose = self.shared.pose if fresh else None
        if connected and self.arm.torque_pending:
            torque = '扭矩切换中'
        elif connected and self.arm.torque_fault:
            torque = '扭矩未确认'
        else:
            torque = self.torque_label(pose)
        cameras = tuple(self.camera_online(name, now) for name in ('uvc', 'gemini'))
        self.status_strip.refresh(self.arm.port if connected else '—', connected, age, cameras, torque)
        targets = tuple(self.arm.last_joint_target or ()) if fresh else ()
        self.diagnostics.set_angles(pose, targets)
        angles = tuple(pose[key] for key in KEYS) if pose else None
        self.arm_view.set_pose(angles, connected)
        coords = tuple(pose.get(key) for key in ('x', 'y', 'z')) if pose else ()
        valid_tcp = coords and all(isinstance(x, (float, int)) and math.isfinite(x) for x in coords)
        self.tcp.setText('TCP · ' + ' / '.join(f'{x:.1f}' for x in coords) + ' mm' if valid_tcp
                         else 'TCP · 等待反馈')
        replay = self.controls.playback
        self.cartesian_pad.set_execution_mode(bool(replay and replay.arm is self.arm and replay.degraded))
        if self.shared.pose:
            for label, key in zip(self.actual, KEYS):
                label.setText(f'{math.degrees(self.shared.pose[key]):.1f}°'+('' if fresh else ' 旧'))
        self.arm_btn.setText('断开机械臂' if self.arm and self.arm.is_alive() else '连接 M3-Pro · USB')
        self.uvc_btn.setText('关闭 AR0234' if self.uvc and self.uvc.is_alive() else '打开 AR0234')
        self.gem_btn.setText('关闭 Gemini 2' if self.gemini and self.gemini.is_alive() else '打开 Gemini 2 · RGB-D')
        self.record_btn.setText(f'停止采集 · {self.recorder.count}' if self.recorder and self.recorder.is_alive() else '开始采集')
        self.camera_fps.setEnabled(not any(w and w.is_alive() for w in (self.uvc, self.gemini)))

    @staticmethod
    def torque_label(pose):
        if not pose:
            return '扭矩未知'
        states = [pose.get('torswitch'+key) for key in ('B', 'S', 'E', 'T', 'R', 'G')]
        if all(value == 1 for value in states):
            return '已使能'
        if all(value == 0 for value in states):
            return '已释放'
        return '部分使能' if set(states) == {0, 1} else '扭矩未知'

    def camera_online(self, name, now):
        worker = getattr(self, name)
        packet = self.shared.frames.get(name)
        return bool(worker and worker.is_alive() and not worker.stop.is_set()
                    and packet and now-packet['monotonic'] < 1)

    def render_frames(self):
        with self.shared.lock:
            packets = dict(self.shared.frames)
        for source, view in zip(('uvc', 'gemini'), self.views):
            worker = getattr(self, source)
            packet = packets.get(source)
            if not (worker and worker.is_alive() and not worker.stop.is_set() and packet):
                if view.meta.text() != '等待连接':
                    view.image.clear()
                    view.meta.setText('等待连接')
                continue
            if self.rendered_frames.get(source) == packet['host_ns']:
                age = time.monotonic()-packet['monotonic']
                if age > 1:
                    view.meta.setText(f'最后帧 {age:.1f}s 前')
                continue
            self.rendered_frames[source] = packet['host_ns']
            now = time.perf_counter()
            times = self.preview_times.setdefault(source, [])
            times.append(now)
            times[:] = [t for t in times if now-t < 2]
            display_fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0
            age = time.monotonic()-packet['monotonic']
            images = packet['images']
            suffix = (f' · 最后帧 {age:.1f}s 前' if age > 1 else
                      f' · 采集 {packet.get("fps", 0):.1f} / 预览 {display_fps:.1f} fps')
            if source == 'uvc':
                im = images['rgb']
                view.show_image(im, f'{im.shape[1]} × {im.shape[0]}'+suffix)
            else:
                rgb, mm = images['rgb'], images['depth_mm']
                _, overlay = depth_views(rgb, mm, self.far.value(), self.alpha.value()/100)
                view.show_image(overlay, 'D2C 已对齐'+suffix)

    def closeEvent(self, event):
        self.auto_paused=True
        self.auto_timer.stop()
        alive = [w for w in self.workers if w.is_alive()]
        if alive:
            for worker in alive:
                worker.stop.set()
            self.note('正在关闭设备、完成文件写入…')
            event.ignore()
            QTimer.singleShot(300, self.close)
            return
        event.accept()




def main():
    if '--diagnose' in sys.argv:
        import pyorbbecsdk as ob
        report = dict(orbbec_import=True, devices=ob.Context().query_devices().get_count(),
                      ports=[p.device for p in list_ports.comports()],
                      align_filter=hasattr(ob, 'AlignFilter'), opencv=cv2.__version__)
        (BASE/'diagnostics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        return
    if sys.platform == 'win32':
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID('RoArmStudio.PCControl')
    app = QApplication(sys.argv)
    app.setApplicationName('RoarmStudio')
    app.setApplicationVersion(VERSION)
    app.setWindowIcon(QIcon(str(BASE/'assets'/'arm-drawing-rounded.ico')))
    # A second shortcut launch activates the existing window, rather than
    # silently exiting behind a hidden process or opening devices twice.
    smoke = '--smoke' in sys.argv
    instance_name = 'RoArmStudioPCPositionTrialWindow' + ('-smoke' if smoke else '')
    activation = QLocalSocket()
    activation.connectToServer(instance_name)
    if activation.waitForConnected(300):
        activation.write(b'activate')
        activation.waitForBytesWritten(300)
        return
    instance_server = QLocalServer()
    if not instance_server.listen(instance_name):
        QMessageBox.warning(None, 'RoarmStudio', '程序正在启动，请稍后再次打开。')
        return
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    window = Window()
    if smoke:
        window.auto_connect.blockSignals(True)
        window.auto_connect.setChecked(False)
        window.auto_connect.blockSignals(False)
    def activate_existing():
        while instance_server.hasPendingConnections():
            connection = instance_server.nextPendingConnection()
            connection.close()
            connection.deleteLater()
        if window.isMinimized() or not window.isVisible():
            window.showNormal()
        window.raise_()
        window.activateWindow()
    instance_server.newConnection.connect(activate_existing)
    window.showMaximized()
    if smoke:
        QTimer.singleShot(1600, lambda: window.grab().save(str(BASE/'界面预览.png')))
        if '--angles' in sys.argv:
            QTimer.singleShot(1800, lambda: window.control_tabs.setCurrentIndex(1))
            QTimer.singleShot(2300, lambda: window.grab().save(str(BASE/'角度控制预览.png')))
        if '--device' in sys.argv:
            QTimer.singleShot(1800, lambda: window.control_tabs.setCurrentIndex(3))
            QTimer.singleShot(2300, lambda: window.grab().save(str(BASE/'设备控制预览.png')))
        if '--precise' in sys.argv:
            QTimer.singleShot(1800, window.cartesian_pad.switch_mode)
            QTimer.singleShot(2300, lambda: window.grab().save(str(BASE/'精确坐标预览.png')))
        if '--controls' in sys.argv:
            QTimer.singleShot(1800, window.open_controls)
            QTimer.singleShot(2300, lambda: window.grab().save(str(BASE/'示教控制预览.png')))
        QTimer.singleShot(3000, window.close)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
