"""Readouts and diagnostics for the desktop workbench."""
from html import escape
import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView, QFrame, QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit,
    QPushButton, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from backend import KEYS, NAMES


def panel(title, suffix='', action=None):
    frame = QFrame()
    frame.setObjectName('pane')
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(0)
    header = QWidget()
    header.setObjectName('paneHeader')
    row = QHBoxLayout(header)
    row.setContentsMargins(8, 4, 8, 4)
    label = QLabel(title)
    label.setObjectName('paneTitle')
    row.addWidget(label)
    row.addStretch()
    if suffix:
        hint = QLabel(suffix)
        hint.setObjectName('quiet')
        row.addWidget(hint)
    if action is not None:
        row.addWidget(action)
    layout.addWidget(header)
    return frame, layout


class StatusStrip(QWidget):
    def __init__(self, stop):
        super().__init__()
        self.setObjectName('statusStrip')
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 3, 6, 3)
        row.setSpacing(0)
        self.labels = []
        for _ in range(5):
            label = QLabel()
            label.setObjectName('connectionStatus')
            label.setTextFormat(Qt.RichText)
            row.addWidget(label)
            self.labels.append(label)
        row.addStretch()
        self.stop = QPushButton('急停')
        self.stop.setObjectName('emergency')
        self.stop.setToolTip('软件取消待发动作与回放。已发送的目标可能继续执行。')
        self.stop.clicked.connect(stop)
        row.addWidget(self.stop)
        self.refresh('—', False, None, (False, False), '扭矩未知')

    @staticmethod
    def device(name, value, connected):
        color = '#70d7a4' if connected else '#82878d'
        state = '已连接' if connected else '未连接'
        return (f'<span style="color:{color}">●</span> {name} <b>{escape(value)}</b>', state)

    def refresh(self, port, connected, age, cameras, torque):
        age_text = '—' if age is None else f'{age:.2f} s 前'
        feedback_color = '#f2bd6e' if age is not None and age >= .6 else '#f1f3f5'
        torque_color = '#70d7a4' if torque == '已使能' else '#82878d'
        entries = (
            self.device('机械臂', port, connected),
            self.device('CAM1', 'UVC' if cameras[0] else '—', cameras[0]),
            self.device('CAM2', 'USB' if cameras[1] else '—', cameras[1]),
            (f'反馈 <b style="color:{feedback_color}">{age_text}</b>', '距上次机械臂反馈'),
            (f'<span style="color:{torque_color}">●</span> <b>{torque}</b>', '机械臂扭矩'),
        )
        for label, (text, tip) in zip(self.labels, entries):
            if label.text() != text:
                label.setText(text)
                label.setToolTip(tip)


def table(columns, labels):
    grid = QTableWidget(len(labels), len(columns))
    grid.setHorizontalHeaderLabels(columns)
    grid.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    grid.verticalHeader().hide()
    grid.verticalHeader().setDefaultSectionSize(24)
    grid.setAlternatingRowColors(True)
    grid.setEditTriggers(QAbstractItemView.NoEditTriggers)
    for row, values in enumerate(labels):
        for column in range(len(columns)):
            cell = QTableWidgetItem(values[column] if column < len(values) else '—')
            if column >= 2:
                cell.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.setItem(row, column, cell)
    return grid


class Diagnostics(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName('pane')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(250)
        self.tabs.addTab(self.log, '日志')
        self.angles = table(('轴', '关节', '实际 °', '目标 °'),
                            [(f'J{i}', name) for i, name in enumerate(NAMES, 1)])
        self.tabs.addTab(self.angles, '角度')
        temperature = QWidget()
        column = QVBoxLayout(temperature)
        column.setContentsMargins(0, 0, 0, 0)
        labels = ('底座', '肩部 A', '肩部 B', '肘部', '腕部俯仰', '腕部旋转', '夹爪')
        self.temperature = table(('ID', '舵机', '温度 °C'),
                                 [(str(i), name) for i, name in enumerate(labels, 11)])
        column.addWidget(self.temperature, 1)
        info = QLabel('当前固件未提供温度反馈')
        info.setObjectName('quiet')
        column.addWidget(info)
        self.tabs.addTab(temperature, '温度')
        layout.addWidget(self.tabs)

    def set_angles(self, pose, targets):
        for i, key in enumerate(KEYS):
            actual = f'{math.degrees(pose[key]):+.1f}' if pose else '—'
            target = f'{math.degrees(targets[i]):+.1f}' if len(targets) == len(KEYS) else '—'
            for column, value in ((2, actual), (3, target)):
                cell = self.angles.item(i, column)
                if cell.text() != value:
                    cell.setText(value)
