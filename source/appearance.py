"""Flat, dark desktop workbench surfaces."""
from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QApplication, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QWidget


class TitleBar(QWidget):
    def __init__(self, window, tabs, version):
        super().__init__(window)
        self.setObjectName('titlebar')
        self.setAttribute(Qt.WA_StyledBackground)
        self.drag_origin = None
        row = QHBoxLayout(self)
        row.setContentsMargins(10, 0, 0, 0)
        title = QLabel(window.windowTitle())
        title.setObjectName('appTitle')
        title.setAttribute(Qt.WA_TransparentForMouseEvents)
        row.addWidget(title)
        row.addWidget(tabs)
        row.addStretch()
        version_label = QLabel(f'v{version}')
        version_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        row.addWidget(version_label)
        buttons = QHBoxLayout()
        buttons.setSpacing(0)
        buttons.addWidget(self.caption_button('−', '最小化', window.showMinimized))
        self.maximize = self.caption_button('□', '最大化', self.toggle_maximized)
        buttons.addWidget(self.maximize)
        close = self.caption_button('×', '关闭', window.close)
        close.setObjectName('captionClose')
        buttons.addWidget(close)
        row.addLayout(buttons)
        window.installEventFilter(self)

    def caption_button(self, symbol, name, callback):
        button = QPushButton(symbol, self)
        button.setFixedSize(42, 31)
        button.setToolTip(name)
        button.setAccessibleName(name)
        button.setProperty('captionControl', True)
        button.clicked.connect(callback)
        return button

    def toggle_maximized(self):
        window = self.window()
        if window.isMaximized() or window.isFullScreen():
            window.showNormal()
        else:
            window.showMaximized()

    def eventFilter(self, watched, event):
        if event.type() == QEvent.WindowStateChange:
            maximized = watched.isMaximized() or watched.isFullScreen()
            name = '还原窗口' if maximized else '最大化'
            self.maximize.setText('❐' if maximized else '□')
            self.maximize.setToolTip(name)
            self.maximize.setAccessibleName(name)
        return False

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_origin = event.globalPosition()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (event.buttons() & Qt.LeftButton and self.drag_origin is not None
                and (event.globalPosition() - self.drag_origin).manhattanLength() >= QApplication.startDragDistance()):
            self.drag_origin = None
            self.window().windowHandle().startSystemMove()
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_origin = None
            self.toggle_maximized()
            event.accept()
        else:
            super().mouseDoubleClickEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_origin = None
        super().mouseReleaseEvent(event)


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
QWidget#titlebar QPushButton { background:transparent; border:0; border-radius:0; padding:0; font-family:"Segoe UI Symbol"; font-size:16px; }
QWidget#titlebar QPushButton:hover { background:#30353b; }
QWidget#titlebar QPushButton#captionClose:hover { background:#a41f2a; }
QWidget#titlebar QPushButton:focus { border:1px solid #73bfff; }
QWidget#footer { background:#202224; border-top:1px solid #32363b; }
QFrame#pane { border:1px solid #32363b; }
QWidget#paneHeader { background:#202224; border-bottom:1px solid #32363b; }
QLabel#paneTitle { font-weight:650; }
QLabel#quiet { color:#acb3bb; font-size:12px; }
QWidget#coordinates { background:#090a0b; border-bottom:1px solid #32363b; }
QLabel#coordinateAxis { color:#9bb1c5; font-size:12px; font-weight:700; }
QLabel#coordinateValue { color:#f4f7fa; font-family:Consolas,"Microsoft YaHei UI"; font-size:24px; font-weight:700; }
QDoubleSpinBox#xyzTarget { font-family:Consolas,"Microsoft YaHei UI"; font-size:20px; padding:4px; }
QPushButton#jogZ { font-family:Consolas,"Microsoft YaHei UI"; font-size:15px; }
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
