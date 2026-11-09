"""User-facing controls for M3 firmware capabilities."""
import json
import math
import time
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QDialog, QTabWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QComboBox,
    QLineEdit, QPlainTextEdit, QFileDialog, QMessageBox, QInputDialog, QApplication, QButtonGroup)
from backend import KEYS, NAMES, all_command, pose_command
from actions import TEACH_SLOTS, TeachRecorder, Playback, load_trajectory, save_trajectory
from host_control import base_in_sector


class Controls(QDialog):
    def __init__(self, owner):
        super().__init__(owner)
        self.owner = owner
        self.setWindowTitle('M3-Pro · 末端控制 / 示教 / 设备设置')
        self.resize(760, 750)
        self.teach_library = owner.teach_library
        self.teach_slot = 1
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
        grid = QGridLayout()
        grid.setSpacing(4)
        self.slot_group = QButtonGroup(self)
        self.slot_buttons = {}
        for slot in TEACH_SLOTS:
            button = QPushButton(f'示教{slot}')
            button.setObjectName('teachSlot')
            button.setCheckable(True)
            self.slot_group.addButton(button, slot)
            self.slot_buttons[slot] = button
            grid.addWidget(button, (slot - 1) // 4, (slot - 1) % 4)
        self.slot_buttons[self.teach_slot].setChecked(True)
        self.slot_group.idClicked.connect(self.select_teach_slot)
        layout.addLayout(grid)
        self.trajectory_label = QLabel()
        self.trajectory_label.setWordWrap(True)
        layout.addWidget(self.trajectory_label)
        row = QHBoxLayout()
        self.record_btn = self.button(row, '开始录制', self.record_teach)
        self.record_btn.setToolTip('停止录制后，自动将关键点保存到选中的槽位。')
        self.button(row, '释放 / 恢复扭矩', self.owner.teach)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel('回放倍率'))
        self.factor = self.spin(.1, 999., .5, ' ×')
        self.factor.setSingleStep(.1)
        row.addWidget(self.factor)
        self.play_btn = self.button(row, '开始回放', self.play)
        layout.addLayout(row)
        self.play_state = QLabel('未开始')
        layout.addWidget(self.play_state)
        row = QHBoxLayout()
        self.import_format = QComboBox()
        self.import_format.addItems(['Studio 原生角度轨迹', '官方 SDK 轨迹（夹爪已反转）'])
        self.import_format.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.import_format.setMinimumContentsLength(12)
        self.import_format.setMaximumWidth(230)
        row.addWidget(self.import_format, 1)
        self.import_btn = self.button(row, '导入本槽', self.load)
        layout.addLayout(row)
        text = QLabel('倍率调整轨迹时间，实际速度受关节上限和跟随误差约束。\n请先结束手动示教并托稳机械臂。')
        text.setWordWrap(True)
        layout.addWidget(text)
        row = QHBoxLayout()
        self.button(row, '取消后续动作 / 回放', self.owner.hold).setObjectName('stop')
        self.clear_btn = self.button(row, '清空本槽', self.clear_teach_slot)
        layout.addLayout(row)
        layout.addStretch()
        self.refresh_teach_slots()

    def make_device_simple(self):
        layout = QVBoxLayout(self.device_page)
        layout.addWidget(QLabel('扭矩保持'))
        row = QHBoxLayout()
        self.button(row, '释放 / 恢复扭矩', self.owner.teach)
        self.button(row, '开启保持', self.torque_on)
        layout.addLayout(row)
        torque_hint = QLabel('释放前请托住机械臂；设置发送后请观察实机。')
        torque_hint.setWordWrap(True)
        layout.addWidget(torque_hint)
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
        if not self.confirm_replace_slot():
            return
        arm = self.owner.arm
        if arm is None:
            self.owner.error('机械臂已断开。')
            return
        with arm.command_lock:
            if self.owner.fresh() and self.owner.arm is arm and not arm.playing:
                path = self.teach_library.path(self.teach_slot)
                self.teach_record = self.owner.start_worker(TeachRecorder(self.owner.shared, path))
                self.record_loaded = False
                self.trajectory_label.setText(f'示教{self.teach_slot} · 正在录制')
                return
        self.owner.error('设备状态已变化，请重试。')

    def select_teach_slot(self, slot):
        if not self.record_loaded:
            self.slot_buttons[self.teach_slot].setChecked(True)
            return
        self.teach_slot = slot
        self.refresh_teach_slots()

    def refresh_teach_slots(self, sample_count=None):
        for item in self.teach_library.slots():
            button = self.slot_buttons[item['slot']]
            button.setText(item['name'] + (' •' if item['occupied'] else ''))
            button.setToolTip('已有轨迹' if item['occupied'] else '空槽位')
        name = f'示教{self.teach_slot}'
        try:
            records = self.teach_library.load(self.teach_slot)
            count = f'{sample_count} 个采样 → ' if sample_count is not None else ''
            self.trajectory_label.setText(f'{name} · {count}{len(records)} 个关键点 · {records[-1]["time"]:.1f} 秒')
        except FileNotFoundError:
            self.trajectory_label.setText(f'{name} · 空槽位')
        except (OSError, ValueError, TypeError, KeyError, OverflowError):
            self.trajectory_label.setText(f'{name} · 轨迹文件无法读取')

    def confirm_replace_slot(self):
        return (not self.teach_library.path(self.teach_slot).is_file()
                or QMessageBox.question(self.owner, '替换示教',
                    f'保存成功后将替换示教{self.teach_slot}。继续？') == QMessageBox.Yes)

    def slot_busy(self):
        return not self.record_loaded or bool(self.owner.arm and self.owner.arm.playing)

    def load(self):
        if self.slot_busy():
            return
        path, _ = QFileDialog.getOpenFileName(self.owner, '载入关节轨迹', str(self.owner.folder), 'JSON (*.json)')
        if not path or not self.confirm_replace_slot() or self.slot_busy():
            return
        try:
            records = load_trajectory(path, self.import_format.currentIndex() == 1)
            save_trajectory(self.teach_library.path(self.teach_slot), records)
            self.refresh_teach_slots()
        except Exception as e:
            self.owner.error(e)

    def clear_teach_slot(self):
        if self.slot_busy() or QMessageBox.question(self.owner, '清空示教',
                f'清空示教{self.teach_slot}的轨迹？') != QMessageBox.Yes or self.slot_busy():
            return
        try:
            self.teach_library.path(self.teach_slot).unlink(missing_ok=True)
            self.refresh_teach_slots()
        except OSError as error:
            self.owner.error(error)

    def play(self):
        if self.teach_record and self.teach_record.is_alive():
            self.owner.error('请先停止轨迹录制')
            return
        if (not self.owner.fresh() or not base_in_sector(self.owner.shared.pose.get('b'), feedback=True)
                or self.owner.arm.teaching or self.owner.arm.playing):
            self.owner.error('需要新鲜反馈、保持扭矩，并且没有其他回放')
            return
        if QMessageBox.question(self.owner, '开始回放', '机械臂将先移动到轨迹首帧，然后执行整段动作。确认工作空间已清空？') != QMessageBox.Yes:
            return
        try:
            records = self.teach_library.load(self.teach_slot)
            self.playback = self.owner.start_worker(Playback(self.owner.shared, self.owner.arm,
                records, self.factor.value(), self.owner.speed.value(), self.owner.acc.value()))
        except (OSError, ValueError, TypeError, KeyError, OverflowError) as error:
            self.owner.error(error)

    def stop_playback(self):
        self.owner.hold()



    def torque_on(self):
        self.owner.set_teaching(False)






    def tick(self):
        fresh = self.owner.fresh()
        arm = self.owner.arm
        playing = bool(arm and arm.playing)
        teaching = bool(arm and arm.teaching)
        pose = self.owner.shared.pose
        inside = fresh and base_in_sector(pose.get('b'), feedback=True)
        self.state.setText(('反馈正常' if fresh else '反馈过期 / 未连接')+
                           (' · 正在回放' if playing else ' · 手动示教' if teaching else ''))
        for button in self.motion_widgets:
            button.setEnabled(inside and not playing and not teaching)
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
        recording = self.teach_record and self.teach_record.is_alive()
        saving = recording and self.teach_record.stop.is_set()
        occupied = self.teach_library.path(self.teach_slot).is_file()
        self.play_btn.setEnabled(inside and not playing and not teaching and occupied and not recording)
        self.record_btn.setEnabled(not saving and not playing)
        self.import_btn.setEnabled(not recording and not playing)
        self.clear_btn.setEnabled(occupied and not recording and not playing)
        for button in self.slot_buttons.values():
            button.setEnabled(not recording)
        self.record_btn.setText('正在保存关键点…' if saving else
                                f'停止并保存 · {self.teach_record.count} 个采样' if recording else '开始录制')
        if self.teach_record and not self.teach_record.is_alive() and not self.record_loaded:
            self.record_loaded = True
            count = self.teach_record.count if self.teach_record.error is None else None
            self.refresh_teach_slots(count)
        external = self.owner.external_api.teach_status()
        if external is not None and self.owner.external_api.lease:
            status = {'queued': '待执行', 'sent': '执行中', 'reached': '已完成',
                      'degraded_reached': '降标完成', 'cancelled': '已取消', 'failed': '失败'}
            self.play_state.setText(f'外部{external["name"]} · {external["progress"]:.0%} · {status.get(external["state"], external["state"])}')
        elif self.playback:
            self.play_state.setText(f'回放进度 {self.playback.progress:.0%}'+(' · 执行中' if self.playback.is_alive() else ' · '+(self.playback.error or '已结束')))
        else:
            self.play_state.setText('未开始')

