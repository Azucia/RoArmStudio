"""Flat, dark desktop workbench surfaces."""
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QLabel, QSizePolicy


class CameraScreen(QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(180, 105)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#080b0f'))
        pixmap = self.pixmap()
        if not pixmap.isNull():
            painter.drawPixmap((self.width()-pixmap.width())//2,
                               (self.height()-pixmap.height())//2, pixmap)
        else:
            painter.setPen(QColor('#aab4bf'))
            painter.drawText(self.rect(), Qt.AlignCenter, '等待相机画面')


STYLE = '''
QWidget { background:#171819; color:#f1f3f5; font-family:"Segoe UI","Microsoft YaHei UI"; font-size:13px; }
QLabel { background:transparent; }
QWidget#workbench { background:#000; }
QWidget#titlebar, QWidget#statusStrip { background:#101112; border-bottom:1px solid #32363b; }
QLabel#appTitle { font-size:16px; font-weight:700; }
QWidget#footer { background:#202224; border-top:1px solid #32363b; }
QFrame#pane { border:1px solid #32363b; }
QWidget#paneHeader { background:#202224; border-bottom:1px solid #32363b; }
QLabel#paneTitle { font-weight:650; }
QLabel#quiet { color:#acb3bb; font-size:12px; }
QLabel#coordinates { background:#090a0b; border:1px solid #32363b; padding:4px; font-family:Consolas,"Microsoft YaHei UI"; font-size:15px; font-weight:650; }
QLabel#strictMode { border:1px solid #32363b; background:#101112; color:#79dfa5; padding:4px 6px; font-weight:650; }
QLabel#degradedMode { border:1px solid #32363b; background:#101112; color:#ffd164; padding:4px 6px; font-weight:650; }
QLabel#connectionStatus { padding:2px 9px; border-right:1px solid #32363b; }
QPushButton { background:#24272a; border:1px solid #42474d; border-radius:2px; padding:4px 9px; }
QPushButton:hover, QComboBox:hover { background:#283749; }
QPushButton:pressed { background:#34526b; }
QPushButton#stop, QPushButton#emergency { background:#a41f2a; border-color:#d93844; color:white; font-weight:700; }
QPushButton#emergency { padding:3px 17px; font-size:18px; }
QPushButton#stop:hover, QPushButton#emergency:hover { background:#c92a36; }
QPushButton#primary { background:#214361; border-color:#4b83b1; }
QPushButton:disabled { color:#79818a; background:#1c1e20; border-color:#303338; }
QPushButton:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border:1px solid #73bfff; }
QTabWidget::pane { border:0; background:#171819; }
QTabBar::tab { background:#202224; color:#bac2ca; padding:6px 12px; border-bottom:2px solid transparent; }
QTabBar::tab:selected { background:#171819; color:#87c9ff; border-bottom-color:#73bfff; }
QTabBar::tab:hover { color:#fff; }
QTabBar::tab:focus { color:#fff; }
QLineEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox, QComboBox { background:#101112; border:1px solid #32363b; padding:3px; }
QPlainTextEdit { padding:4px; selection-background-color:#284667; }
QScrollArea { border:0; }
QGroupBox { border:1px solid #32363b; margin-top:9px; padding:7px 5px 4px; }
QGroupBox::title { subcontrol-origin:margin; left:6px; padding:0 4px; }
QGroupBox#jogPad { border:0; margin:0; padding:0; }
QTableWidget { background:#171819; alternate-background-color:#101112; gridline-color:#32363b; border:0; }
QHeaderView::section { color:#c1c8cf; background:#202224; border:0; border-bottom:1px solid #32363b; padding:4px; }
QToolTip { color:#f1f3f5; background:#202224; border:1px solid #4c5863; }
'''
