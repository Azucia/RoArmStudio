"""User-facing controls for M3 firmware capabilities."""
import json
import math
import time
from pathlib import Path
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QDialog, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QComboBox,
    QLineEdit, QPlainTextEdit, QFileDialog, QMessageBox, QInputDialog, QApplication)
from backend import KEYS, NAMES, all_command, pose_command
from actions import TeachRecorder, Playback, load_trajectory


class Controls(QDialog):
    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.setWindowTitle('M3-Pro · 末端控制 / 示教 / 设备设置')
        self.resize(760, 750)
        self.trajectory = None
        self.teach_record = None
        self.playback = None
        self.record_loaded = True
        outer = QVBoxLayout(self)
        self.state = QLabel('等待机械臂实际反馈')
        outer.addWidget(self.state)
        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)
        self.motion_widgets = []
        self.make_actions()
        self.device_page=QWidget()
        self.make_device_simple()
        row = QHBoxLayout()
        stop = owner.button(row, '取消后续动作 / 回放', self.stop_playback)
        stop.setObjectName('stop')
        stop.setToolTip('取消待发动作；已发送的目标可能继续执行，不改写保持目标、不释放扭矩。')
        owner.button(row, '关闭面板', self.close)
        outer.addLayout(row)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

    def page(self, name):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.tabs.addTab(page, name)
        return layout

    def button(self, layout, text, fn, motion=False):
        btn = self.owner.button(layout, text, fn)
        if motion:
            self.motion_widgets.append(btn)
        return btn

    def spin(self, low, high, value=0, suffix=''):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(1)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def send(self, command, motion=False):
        try:
            if not self.owner.arm or not self.owner.arm.connected:
                raise ValueError('请先连接机械臂串口')
            if motion and not self.owner.fresh():
                raise ValueError('缺少新鲜实际反馈，无法运动')
            self.owner.arm.submit(command)
        except Exception as e:
            self.owner.error(e)






    def make_actions(self):
        layout = self.page('示教与回放')
        hint = QLabel('录制关节角度，或载入已保存的轨迹。')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        row = QHBoxLayout()
        self.record_btn = self.button(row, '开始录制', self.record_teach)
        self.button(row, '释放 / 恢复扭矩', self.owner.teach)
        layout.addLayout(row)
        self.trajectory_label = QLabel('未载入轨迹')
        self.trajectory_label.setWordWrap(True)
        layout.addWidget(self.trajectory_label)
        row = QHBoxLayout()
        self.import_format = QComboBox()
        self.import_format.addItems(['Studio 原生角度轨迹', '官方 SDK 轨迹（夹爪已反转）'])
        self.import_format.setMaximumWidth(230)
        row.addWidget(self.import_format)
        self.button(row, '载入轨迹', self.load)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel('回放倍率'))
        self.factor = self.spin(.1, 999., .5, ' ×')
        self.factor.setSingleStep(.1)
        row.addWidget(self.factor)
        self.button(row, '开始回放', self.play, True)
        layout.addLayout(row)
        self.play_state = QLabel('未开始')
        layout.addWidget(self.play_state)
        text = QLabel('回放从首帧开始，速度受关节速度上限约束。\n请先结束手动示教并托稳机械臂。')
        text.setWordWrap(True)
        layout.addWidget(text)
        self.button(layout, '取消后续动作 / 回放', self.owner.hold).setObjectName('stop')
        layout.addStretch()

    def make_device_simple(self):
        layout = QVBoxLayout(self.device_page)
        layout.addWidget(QLabel('扭矩保持'))
        row = QHBoxLayout()
        self.button(row, '释放 / 恢复扭矩', self.owner.teach)
        self.button(row, '开启保持', self.torque_on)
        layout.addLayout(row)
        layout.addWidget(QLabel('释放前请托住机械臂；设置发送后请观察实机。'))
        row = QHBoxLayout()
        row.addWidget(QLabel('补光灯'))
        self.led = QSpinBox()
        self.led.setRange(0, 255)
        row.addWidget(self.led)
        self.button(row, '应用', lambda: self.send(dict(T=114, led=self.led.value())))
        self.button(row, '关闭', lambda: self.send(dict(T=114, led=0)))
        layout.addLayout(row)
        layout.addWidget(QLabel('实际反馈'))
        self.device_feedback = QLabel('未连接')
        self.device_feedback.setWordWrap(True)
        self.device_feedback.setTextInteractionFlags(self.device_feedback.textInteractionFlags())
        layout.addWidget(self.device_feedback)
        layout.addStretch()
        self.button(layout, '取消后续动作 / 回放', self.owner.hold).setObjectName('stop')

    def record_teach(self):
        if self.teach_record and self.teach_record.is_alive():
            self.teach_record.stop.set()
            return
        if not self.owner.fresh() or self.owner.arm.playing:
            self.owner.error('需要正常关节反馈，且没有正在执行的回放。')
            return
        path = self.owner.destination('teach').with_suffix('.json')
        self.teach_record = self.owner.start_worker(TeachRecorder(self.owner.shared, path))
        self.record_loaded = False
        self.trajectory_label.setText(f'录制位置：{path}')

    def load(self):
        path, _ = QFileDialog.getOpenFileName(self.owner, '载入关节轨迹', str(self.owner.folder), 'JSON (*.json)')
        if not path:
            return
        try:
            self.trajectory = load_trajectory(path, self.import_format.currentIndex() == 1)
            self.trajectory_label.setText(f'{Path(path).name} · {len(self.trajectory)} 点 · {self.trajectory[-1]["time"]:.1f} 秒')
        except Exception as e:
            self.trajectory = None
            self.owner.error(e)

    def play(self):
        if not self.trajectory:
            self.owner.error('请先载入有效轨迹')
            return
        if self.teach_record and self.teach_record.is_alive():
            self.owner.error('请先停止轨迹录制')
            return
        if not self.owner.fresh() or self.owner.arm.teaching or self.owner.arm.playing:
            self.owner.error('需要新鲜反馈、保持扭矩，并且没有其他回放')
            return
        if QMessageBox.question(self.owner, '开始回放', '机械臂将先移动到轨迹首帧，然后执行整段动作。确认工作空间已清空？') != QMessageBox.Yes:
            return
        self.playback = self.owner.start_worker(Playback(self.owner.shared, self.owner.arm,
                                                       self.trajectory, self.factor.value(),
                                                       self.owner.speed.value(), self.owner.acc.value()))

    def stop_playback(self):
        self.owner.hold()



    def torque_on(self):
        self.owner.set_teaching(False)






    def tick(self):
        fresh = self.owner.fresh()
        arm = self.owner.arm
        playing = bool(arm and arm.playing)
        teaching = bool(arm and arm.teaching)
        self.state.setText(('反馈正常' if fresh else '反馈过期 / 未连接')+
                           (' · 正在回放' if playing else ' · 手动示教' if teaching else ''))
        for button in self.motion_widgets:
            button.setEnabled(fresh and not playing and not teaching)
        pose = self.owner.shared.pose
        if fresh:
            lines = ['反馈正常'+(' · 手动示教' if teaching else ' · 回放中' if playing else '')]
            for name, key in zip(NAMES, KEYS):
                lines.append(f'{name}  {math.degrees(pose[key]):.1f}°')
            lines.append('扭矩状态：'+arm.torque_state)
            for name, key in zip(NAMES, ('torswitchB','torswitchS','torswitchE','torswitchT','torswitchR','torswitchG')):
                lines.append(f'{name}扭矩：'+{0:'关闭',1:'开启'}.get(pose.get(key),'未知'))
            for key in ('x', 'y', 'z'):
                if isinstance(pose.get(key), (int, float)):
                    lines.append(f'{key.upper()}  {pose[key]:.1f} mm')
            self.device_feedback.setText('\n'.join(lines))
        else:
            self.device_feedback.setText('未连接 / 反馈已过期')
        self.record_btn.setText(f'停止并保存 · {self.teach_record.count} 点' if self.teach_record and self.teach_record.is_alive() else '开始录制')
        if self.teach_record and not self.teach_record.is_alive() and not self.record_loaded:
            self.record_loaded = True
            if self.teach_record.error is None and self.teach_record.path.exists():
                try:
                    self.trajectory = load_trajectory(self.teach_record.path)
                    self.trajectory_label.setText(f'已保存并载入：{self.teach_record.path.name} · {len(self.trajectory)} 点；结束手动示教后可直接回放')
                except Exception as e:
                    self.owner.note(f'录制文件载入失败：{e}')
        if self.playback:
            self.play_state.setText(f'回放进度 {self.playback.progress:.0%}'+(' · 执行中' if self.playback.is_alive() else ' · '+(self.playback.error or '已结束')))

