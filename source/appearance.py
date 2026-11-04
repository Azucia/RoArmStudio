"""Material-inspired tonal surfaces with a native Windows acrylic backdrop."""
import ctypes
import sys
from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QWidget, QLabel, QGraphicsDropShadowEffect, QSizePolicy


class GlassCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.native_backdrop = False
        self.setObjectName('canvas')

    def paintEvent(self, event):
        painter = QPainter(self)
        theme=getattr(self,"theme","color")
        if theme!="color":
            painter.fillRect(self.rect(),QColor("#182028" if theme=="dark" else "#f3f5f7"))
            return
        tint = QLinearGradient(0, 0, self.width(), self.height()*.35)
        tint.setColorAt(0, QColor('#fbf5ee'))
        tint.setColorAt(.3, QColor('#f6edf1'))
        tint.setColorAt(.62, QColor('#dce8f7'))
        tint.setColorAt(1, QColor('#a3d0ef'))
        painter.fillRect(self.rect(), tint)
        lower_tint = QLinearGradient(0, 0, self.width()*.45, self.height())
        lower_tint.setColorAt(0, QColor(234, 214, 233, 0))
        lower_tint.setColorAt(.55, QColor(217, 203, 233, 20))
        lower_tint.setColorAt(1, QColor(119, 128, 204, 105))
        painter.fillRect(self.rect(), lower_tint)


def soft_shadow(widget, radius=26, opacity=30, offset=7):
    effect = QGraphicsDropShadowEffect(widget)
    effect.setBlurRadius(radius)
    effect.setOffset(0, offset)
    effect.setColor(QColor(39, 42, 78, opacity))
    widget.setGraphicsEffect(effect)


def enable_native_backdrop(window):
    """Use compositor blur on Win11; fall back to a readable tonal surface."""
    if sys.platform != 'win32' or sys.getwindowsversion().build < 22000:
        return False
    try:
        dwm = ctypes.WinDLL('dwmapi')
        hwnd = ctypes.c_void_p(int(window.winId()))
        dark = ctypes.c_int(0)
        dwm.DwmSetWindowAttribute(hwnd, 20, ctypes.byref(dark), ctypes.sizeof(dark))
        caption = ctypes.c_uint(0x00f5f2f7)
        text = ctypes.c_uint(0x00452b29)
        dwm.DwmSetWindowAttribute(hwnd, 35, ctypes.byref(caption), ctypes.sizeof(caption))
        dwm.DwmSetWindowAttribute(hwnd, 36, ctypes.byref(text), ctypes.sizeof(text))
        material = ctypes.c_int(3)  # DWMSBT_TRANSIENTWINDOW / desktop acrylic.
        result = dwm.DwmSetWindowAttribute(hwnd, 38, ctypes.byref(material), ctypes.sizeof(material))
        class Margins(ctypes.Structure):
            _fields_ = [('left', ctypes.c_int), ('right', ctypes.c_int),
                        ('top', ctypes.c_int), ('bottom', ctypes.c_int)]
        margins = Margins(-1, -1, -1, -1)
        extended = dwm.DwmExtendFrameIntoClientArea(hwnd, ctypes.byref(margins))
        return result == 0 and extended == 0
    except (OSError, AttributeError):
        return False


class CameraScreen(QLabel):
    def __init__(self):
        super().__init__()
        self.setObjectName('cameraScreen')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(250, 150)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.pixmap() and not self.pixmap().isNull():
            painter.fillRect(self.rect(), QColor('#11151b'))
            pixmap = self.pixmap()
            painter.drawPixmap((self.width()-pixmap.width())//2, (self.height()-pixmap.height())//2, pixmap)
            return
        dark=getattr(self.window().centralWidget(),'theme','light')=='dark'
        painter.fillRect(self.rect(), QColor('#27333c' if dark else '#e6ebf5'))
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 37))
        painter.drawRoundedRect(QRectF(self.rect()), 18, 18)
        x, y = self.width()/2, self.height()/2-12
        painter.setPen(QPen(QColor('#8590ad'), 1.8))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(QRectF(x-18, y-12, 28, 24), 5, 5)
        painter.drawLine(int(x+10), int(y-5), int(x+21), int(y-10))
        painter.drawLine(int(x+21), int(y-10), int(x+21), int(y+10))
        painter.drawLine(int(x+21), int(y+10), int(x+10), int(y+5))
        painter.setPen(QColor('#727c99'))
        painter.drawText(QRectF(0, y+25, self.width(), 24), Qt.AlignCenter, '相机未连接')


STYLE = '''
QWidget {background:transparent;color:#292b45;font-family:"Segoe UI Variable Text","Microsoft YaHei UI";font-size:14px;}
QMainWindow {background:transparent;}
QWidget#sidebar {background:rgba(255,255,255,95);border:1px solid rgba(255,255,255,150);border-radius:26px;}
QLabel#appTitle {font-size:24px;font-weight:600;color:#24263f;}
QLabel#subtitle {color:#6e7389;font-size:13px;}
QLabel#connectionStatus {background:rgba(255,255,255,110);border:1px solid rgba(255,255,255,140);border-radius:16px;padding:8px 14px;color:#626b82;}
QFrame#cameraCard {background:rgba(255,255,255,77);border:1px solid rgba(255,255,255,128);border-radius:26px;}
QLabel#cameraTitle {color:#292b45;font-size:15px;font-weight:600;}
QLabel#cameraTag {background:rgba(255,255,255,100);border:1px solid rgba(255,255,255,150);color:#68728f;padding:4px 9px;border-radius:12px;font-size:12px;}
QLabel#cameraMeta {color:#717c97;font-size:12px;}
QLabel#coordinates {font-size:21px;font-weight:600;color:#4f5b91;}
QLabel#quiet {color:#64708e;font-size:12px;}
QGroupBox {border:none;margin-top:0;padding:8px 2px;font-weight:400;}
QGroupBox::title {color:#737d97;subcontrol-origin:margin;left:8px;}
QPushButton {background:rgba(255,255,255,105);border:1px solid rgba(105,125,169,125);border-radius:16px;padding:8px 12px;color:#434967;min-height:19px;}
QPushButton:hover {background:rgba(255,255,255,185);border-color:#8e9fcb;}
QPushButton:pressed {background:rgba(170,186,228,190);border-color:#7189c2;}
QPushButton:focus {border:1px solid #8c9cd0;}
QPushButton:disabled {background:rgba(255,255,255,60);color:#8b97b2;border-color:rgba(115,135,179,85);}
QPushButton#primary {background:#b5c0ef;color:#303c6d;border:1px solid #8fa3d6;font-weight:600;}
QPushButton#primary:hover {background:#c4cdf5;}
QPushButton#primary:disabled {background:rgba(181,192,239,70);color:#8995b9;}
QPushButton#stop {background:rgba(222,165,178,125);color:#764354;border:1px solid #c294a4;font-weight:500;}
QPushButton#stop:hover {background:rgba(231,175,186,180);}
QPushButton#stop:disabled {background:rgba(222,175,186,65);color:#ac8a99;}
QPushButton#foldout {background:rgba(255,255,255,40);text-align:left;border:1px solid rgba(110,130,175,90);border-radius:14px;padding:10px;color:#535c79;}
QPushButton#foldout:hover,QPushButton#foldout:checked {background:rgba(255,255,255,95);}
QComboBox,QSpinBox,QDoubleSpinBox,QLineEdit {background:rgba(255,255,255,90);border:1px solid rgba(115,135,179,100);border-radius:12px;padding:7px 8px;color:#3a4263;selection-background-color:#c5cff1;selection-color:#283156;}
QComboBox:focus,QSpinBox:focus,QDoubleSpinBox:focus,QLineEdit:focus {border-color:#a1aedb;}
QComboBox::drop-down {border:none;width:22px;}
QSpinBox::up-button,QDoubleSpinBox::up-button {border:none;background:rgba(255,255,255,60);width:18px;border-top-right-radius:10px;}
QSpinBox::down-button,QDoubleSpinBox::down-button {border:none;background:rgba(255,255,255,60);width:18px;border-bottom-right-radius:10px;}
QComboBox QAbstractItemView {background:#edf0fb;color:#3a4263;selection-background-color:#d7dff7;border:1px solid #d3daf0;outline:none;}
QTabWidget::pane {background:transparent;border:none;}
QTabBar {background:transparent;}
QTabBar::tab {background:rgba(255,255,255,30);color:#626b88;border:1px solid rgba(115,135,179,85);border-radius:17px;padding:9px 12px;margin:0 4px 10px 0;}
QTabBar::tab:selected {background:rgba(160,174,227,135);color:#3f4b7a;border:1px solid #9daed6;border-radius:17px;font-weight:600;}
QTabBar::tab:hover:!selected {background:rgba(255,255,255,105);border-radius:17px;}
QPlainTextEdit {background:rgba(255,255,255,67);border:1px solid rgba(255,255,255,90);border-radius:16px;padding:8px;color:#717c96;font-size:12px;}
QScrollArea,QScrollArea>QWidget>QWidget,QStackedWidget {background:transparent;border:none;}
QScrollBar:vertical {background:transparent;width:5px;margin:4px 0;}
QScrollBar::handle:vertical {background:rgba(123,137,180,70);border-radius:2px;min-height:24px;}
QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical {height:0;}
QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical {background:transparent;}
QSlider::groove:horizontal {height:6px;background:rgba(111,131,183,55);border-radius:3px;}
QSlider::sub-page:horizontal {background:#a7b4e2;border-radius:3px;}
QSlider::handle:horizontal {width:16px;margin:-5px 0;background:#8598d6;border-radius:8px;}
QToolTip {background:#f4f5fc;color:#343e64;border:1px solid #d7ddf1;padding:6px;}
QMessageBox,QFileDialog,QInputDialog {background:#edf0fa;}
'''



def theme_style(key):
    compact = """
    QTabBar::tab {padding:6px 7px;margin:0 2px 5px 0;border-radius:6px;}
    QPushButton:checked {border:2px solid #43849f;}
    QPushButton#themeIcon {padding:0;min-height:0;border-radius:7px;font-size:19px;}
    QCheckBox#inverseMode {border:2px solid #55899e;border-radius:8px;padding:10px;font-weight:600;}
    QMenu {background:#f5f7fa;color:#24323c;border:1px solid #8595a1;padding:3px;}
    QMenu::item {padding:5px 16px;}
    QMenu::item:selected {background:#d6e7ef;color:#24323c;}
    """
    if key=='color':
        return STYLE+compact
    dark=key=='dark'
    surface='#232d36' if dark else '#ffffff'
    text='#e5e9ee' if dark else '#24323c'
    muted='#b3bfca' if dark else '#56646e'
    edge='#465561' if dark else '#cdd6dc'
    field='#303d49' if dark else '#f4f6f8'
    accent='#3a5667' if dark else '#d6e7ef'
    return STYLE+compact+f"""
    QMenu {{background:{surface};color:{text};}}
    QCheckBox#inverseMode {{background:{accent};color:{text};}}
    QWidget,QLabel#appTitle,QLabel#cameraTitle,QLabel#coordinates {{color:{text};}}
    QWidget#sidebar,QFrame#cameraCard,QMessageBox,QFileDialog {{background:{surface};border-radius:10px;border-color:{edge};}}
    QLabel#quiet,QLabel#subtitle,QLabel#cameraMeta,QLabel#cameraTag,QLabel#connectionStatus {{color:{muted};background:transparent;}}
    QPushButton,QPushButton#foldout,QComboBox,QSpinBox,QDoubleSpinBox,QPlainTextEdit,QComboBox QAbstractItemView {{color:{text};background:{field};border-color:{edge};border-radius:6px;}}
    QPushButton:hover,QPushButton:checked,QPushButton#foldout:checked,QTabBar::tab:selected,QPushButton#primary {{background:{accent};color:{text};}}
    QTabBar::tab {{color:{muted};background:{surface};}}
    QPushButton:disabled {{color:{muted};background:{surface};}}
    QPushButton#stop {{color:{'#ffdae0' if dark else '#712739'};background:{'#653c46' if dark else '#f9e3e7'};}}
    """
