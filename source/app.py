import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QHBoxLayout, QVBoxLayout, QGridLayout, QComboBox, QSpinBox, QDoubleSpinBox,
    QGroupBox, QPlainTextEdit, QScrollArea, QFileDialog, QMessageBox, QSlider, QTabWidget, QFrame)
from serial.tools import list_ports
from backend import Shared, Arm, UVC, Gemini, Recorder, KEYS, NAMES, LIMITS, joint_command, hold_command, depth_views
from backend import all_command
from controls import Controls
from jogpad import CartesianPad
from appearance import STYLE, GlassCanvas, CameraScreen, enable_native_backdrop, soft_shadow

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
        self.setObjectName('cameraCard')
        soft_shadow(self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 12)
        layout.setSpacing(10)
        header = QHBoxLayout()
        name = QLabel(title)
        name.setObjectName('cameraTitle')
        header.addWidget(name)
        header.addStretch()
        kind = QLabel(tag)
        kind.setObjectName('cameraTag')
        header.addWidget(kind)
        layout.addLayout(header)
        self.image = CameraScreen()
        self.meta = QLabel('等待连接')
        self.meta.setObjectName('cameraMeta')
        layout.addWidget(self.image, 1)
        layout.addWidget(self.meta)

    def show_image(self, bgr, meta):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        self.image.setPixmap(QPixmap.fromImage(image).scaled(self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.meta.setText(meta)


class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('RoArm Studio · M3-Pro / 双相机工作台')
        self.resize(1440, 940)
        if sys.platform == 'win32' and sys.getwindowsversion().build >= 22000:
            self.setAttribute(Qt.WA_TranslucentBackground)
        self.shared = Shared()
        self.arm = self.uvc = self.gemini = self.recorder = None
        self.workers = []
        self.last_frames = {}
        self.controls = None
        self.folder = BASE/'captures'
        root = GlassCanvas()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 12, 20, 18)
        outer.setSpacing(16)
        top = QHBoxLayout()
        title = QLabel('RoArm Studio')
        title.setObjectName('appTitle')
        top.addWidget(title)
        subtitle = QLabel('M3-Pro  ·  双相机')
        subtitle.setObjectName('subtitle')
        top.addWidget(subtitle)
        top.addStretch()
        self.status = QLabel('离线 · 连接不会自动运动')
        self.status.setObjectName('connectionStatus')
        top.addWidget(self.status)
        outer.addLayout(top)
        body = QHBoxLayout()
        body.setSpacing(16)
        outer.addLayout(body, 1)
        sidebar = QWidget()
        sidebar.setObjectName('sidebar')
        soft_shadow(sidebar, radius=30, opacity=33, offset=8)
        left = QVBoxLayout(sidebar)
        left.setContentsMargins(14, 14, 14, 14)
        left.setSpacing(12)
        sidebar.setFixedWidth(420)
        body.addWidget(sidebar)
        settings = QWidget()
        side = QVBoxLayout(settings)
        side.setContentsMargins(0, 0, 0, 0)
        self.settings_sections = []
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(settings)
        scroll.setMinimumHeight(140)
        box = QGroupBox('机械臂连接')
        l = QVBoxLayout(box)
        row = QHBoxLayout()
        self.ports = QComboBox()
        self.ports.setMinimumWidth(180)
        row.addWidget(self.ports, 1)
        self.button(row, '刷新', self.refresh_ports)
        l.addLayout(row)
        self.arm_btn = self.button(l, '连接 M3-Pro · USB', self.toggle_arm)
        self.arm_state = QLabel('115200 baud · 等待设备')
        l.addWidget(self.arm_state)
        section = Foldout(box)
        self.settings_sections.append(section)
        side.addWidget(section)
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
        self.button(row, '回初始位', self.home)
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
        self.hold_btn = self.button(row, '停止运动 / 回放', self.hold)
        self.hold_btn.setObjectName('stop')
        self.teach_btn = self.button(row, '释放扭矩 · 手动示教', self.teach)
        l.addLayout(row)
        box = QGroupBox('相机连接')
        l = QVBoxLayout(box)
        row = QHBoxLayout()
        row.addWidget(QLabel('AR0234 / UVC 索引'))
        self.cam_index = QSpinBox()
        self.cam_index.setRange(0, 15)
        row.addWidget(self.cam_index)
        l.addLayout(row)
        self.uvc_btn = self.button(l, '打开 AR0234', self.toggle_uvc)
        self.gem_btn = self.button(l, '打开 Gemini 2 · RGB-D', self.toggle_gemini)
        l.addWidget(QLabel('AR0234 需为 USB UVC 模组；索引选错可重选。'))
        section = Foldout(box)
        self.settings_sections.append(section)
        side.addWidget(section)
        box = QGroupBox('采集与保存')
        l = QVBoxLayout(box)
        self.path_label = QLabel(str(self.folder))
        self.path_label.setWordWrap(True)
        l.addWidget(self.path_label)
        self.button(l, '选择保存目录', self.select_folder)
        row = QHBoxLayout()
        self.button(row, '保存快照', self.snapshot)
        self.record_btn = self.button(row, '开始采集', self.record)
        l.addLayout(row)
        l.addWidget(QLabel('图像＋原始深度＋关节反馈，按主机时间记录。'))
        section = Foldout(box)
        self.settings_sections.append(section)
        side.addWidget(section)
        side.addStretch()
        right = QVBoxLayout()
        body.addLayout(right, 1)
        grid = QGridLayout()
        grid.setSpacing(14)
        self.views = [View('手上视角', 'AR0234'), View('彩色画面', 'Gemini 2 · RGB'),
                      View('深度画面', 'Gemini 2 · Depth'), View('彩色与深度', 'RGB-D')]
        for i, view in enumerate(self.views):
            grid.addWidget(view, i//2, i%2)
        right.addLayout(grid, 1)
        self.cartesian_pad = CartesianPad(self)
        self.control_tabs = QTabWidget()
        self.control_tabs.addTab(self.cartesian_pad, 'XYZ')
        self.control_tabs.addTab(self.joint_panel, '关节角度')
        self.control_tabs.setFixedHeight(580)
        left.addWidget(self.control_tabs)
        left.addWidget(scroll, 1)
        row = QHBoxLayout()
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
        right.addLayout(row)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(250)
        self.log.setMaximumHeight(66)
        right.addWidget(self.log)
        self.controls = Controls(self)
        while self.controls.tabs.count():
            page = self.controls.tabs.widget(0)
            title = '示教回放' if self.control_tabs.count() == 2 else '设备'
            self.controls.tabs.removeTab(0)
            self.control_tabs.addTab(page, title)
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.note('就绪。未连接设备；所有画面与读数均来自实际设备。')
        QTimer.singleShot(100, self.apply_backdrop)

    def apply_backdrop(self):
        self.centralWidget().native_backdrop = enable_native_backdrop(self)
        self.centralWidget().update()

    def showEvent(self, event):
        super().showEvent(event)

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

    def refresh_ports(self):
        self.ports.clear()
        for p in list_ports.comports():
            self.ports.addItem(f'{p.device} · {p.description}', p.device)

    def start_worker(self, worker):
        self.workers.append(worker)
        worker.start()
        return worker

    def toggle_arm(self):
        if self.arm and self.arm.is_alive():
            self.arm.cancel_replay()
            self.arm.stop.set()
            return
        port = self.ports.currentData()
        if not port:
            self.error('未发现串口，请连接机械臂后刷新。')
            return
        self.shared.pose = None
        self.shared.pose_time = 0
        self.arm = self.start_worker(Arm(self.shared, port))

    def toggle_uvc(self):
        if self.uvc and self.uvc.is_alive():
            self.uvc.stop.set()
        else:
            self.uvc = self.start_worker(UVC(self.shared, self.cam_index.value()))

    def toggle_gemini(self):
        if self.gemini and self.gemini.is_alive():
            self.gemini.stop.set()
        else:
            self.gemini = self.start_worker(Gemini(self.shared))

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
        try:
            if self.arm:
                self.arm.cancel_replay()
            if not self.fresh():
                raise ValueError('反馈不新鲜，无法可靠发送保持当前位置指令。')
            self.arm.submit(hold_command(self.shared.pose))
        except Exception as e:
            self.error(e)

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
        if QMessageBox.question(self, '回中 · 官方初始姿态',
            '会移动所有关节到初始姿态（不是将 XYZ 清零，也不改写中位）。确认周围没有障碍？') == QMessageBox.Yes:
            self.arm.submit(all_command([0, 0, math.pi/2, 0, 0, math.pi], 100, 5))

    def open_controls(self):
        self.control_tabs.setCurrentIndex(2)

    def teach(self):
        if not self.fresh():
            return
        if not self.arm.teaching:
            if QMessageBox.question(self, '手动示教',
                '即将释放所有关节扭矩，机械臂可能下落。请先用手托住机械臂。\n开始采集后可以同时记录拖动过程。继续？') != QMessageBox.Yes:
                return
            self.arm.submit(dict(T=210, cmd=0))
            self.arm.teaching = True
            self.note('已发送释放扭矩；请观察实机确认。结束示教时会锁定新鲜反馈位置。')
        else:
            self.arm.teaching = False
            try:
                self.arm.submit(hold_command(self.shared.pose))
            except Exception as e:
                self.arm.teaching = True
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
        fresh = self.fresh()
        teaching = bool(self.arm and self.arm.teaching)
        playing = bool(self.arm and self.arm.playing)
        self.cartesian_pad.refresh()
        for btn in self.motion:
            btn.setEnabled(fresh and not teaching and not playing)
        self.hold_btn.setEnabled(fresh and not teaching)
        self.teach_btn.setEnabled(fresh and not playing)
        self.teach_btn.setText('结束示教 · 锁定当前位置' if teaching else '释放扭矩 · 手动示教')
        self.arm_state.setText('反馈正常 · 手动示教' if fresh and teaching else '反馈正常 · USB 串口' if fresh else '等待反馈 / 离线')
        self.status.setText('●  机械臂已连接' if fresh else '○  机械臂未连接')
        if self.shared.pose:
            for label, key in zip(self.actual, KEYS):
                label.setText(f'{math.degrees(self.shared.pose[key]):.1f}°'+('' if fresh else ' 旧'))
        self.arm_btn.setText('断开机械臂' if self.arm and self.arm.is_alive() else '连接 M3-Pro · USB')
        self.uvc_btn.setText('关闭 AR0234' if self.uvc and self.uvc.is_alive() else '打开 AR0234')
        self.gem_btn.setText('关闭 Gemini 2' if self.gemini and self.gemini.is_alive() else '打开 Gemini 2 · RGB-D')
        self.record_btn.setText(f'停止采集 · {self.recorder.count}' if self.recorder and self.recorder.is_alive() else '开始采集')
        with self.shared.lock:
            packets = dict(self.shared.frames)
        for source, packet in packets.items():
            age = time.monotonic()-packet['monotonic']
            images = packet['images']
            suffix = f' · 最后帧 {age:.1f}s 前' if age > 1 else ' · 实时'
            if source == 'uvc':
                im = images['rgb']
                self.views[0].show_image(im, f'{im.shape[1]} × {im.shape[0]}'+suffix)
            else:
                rgb, mm = images['rgb'], images['depth_mm']
                depth, overlay = depth_views(rgb, mm, self.far.value(), self.alpha.value()/100)
                self.views[1].show_image(rgb, f'{rgb.shape[1]} × {rgb.shape[0]}'+suffix)
                center = mm[mm.shape[0]//2, mm.shape[1]//2]
                self.views[2].show_image(depth, (f'中心 {center:.0f} mm' if center else '中心深度无效')+suffix)
                self.views[3].show_image(overlay, 'D2C 已对齐 · 叠加显示'+suffix)

    def closeEvent(self, event):
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
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    window = Window()
    window.show()
    if '--smoke' in sys.argv:
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
