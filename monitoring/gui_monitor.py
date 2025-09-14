# 단일 파일: 실시간 시각화 앱 (pyqtgraph + PyQt5)
# 필요한 패키지:
#   pip install pyserial pyqtgraph PyOpenGL PyQt5 numpy
# 실행 예:
#   python analysis\gui_monitor.py --port COM6 --baud 921600 --ui-fps 60 --history 10 --max-points 2000

import sys
import time
import math
import signal
import threading
import argparse
from dataclasses import dataclass, field
from typing import Optional, List, Deque, Dict, Tuple
from collections import deque

# Third-party
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("pyserial이 필요합니다. pip install pyserial")
    raise

# PyQt / PyQtGraph
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from pyqtgraph.opengl import (
    GLViewWidget, GLGridItem, GLLinePlotItem, GLMeshItem
)

# log_reader
from log_reader import LogReader, LogData


# ===========================================
# 실시간 파이프라인: ReaderThread + DataHub
# ===========================================

@dataclass
class AppConfig:
    port: str = "COM6"
    baud: int = 921600
    read_timeout: float = 0.02
    read_chunk: int = 512
    ui_fps: int = 60
    history_sec: float = 3.0
    max_points: int = 1000
    max_queue: int = 8192
    theme: str = "dark"  # dark / light
    line_width: int = 2
    antialias: bool = True
    enable_3d: bool = True
    enable_plots: Dict[str, bool] = field(default_factory=lambda: {
        "raw_accel": True,
        "raw_gyro": True,
        "accel": True,
        "gyro": True,
        "raw_rc": True,
        "rc": True,
        "pid": True,
        "esc": True,
    })

class SerialReaderThread(threading.Thread):
    """
    LogReader를 내부에 보유하고 프레임을 큐에 푸시.
    주기적으로 LogReader.stats()를 snapshot으로 보관해 UI에서 표시 가능.
    """
    def __init__(self, cfg: AppConfig, out_queue: Deque[Tuple[float, LogData]]):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.out_queue = out_queue
        self._stop_evt = threading.Event()
        self._lr: Optional[LogReader] = None
        self.last_error: Optional[str] = None
        self._stats_lock = threading.Lock()
        self._stats: Dict[str, float] = {
            "ok": 0, "crc_fail": 0, "desync": 0, "len_err": 0, "error_rate": 0.0, "fps": 0.0
        }

    def run(self):
        try:
            self._lr = LogReader(self.cfg.port, self.cfg.baud, self.cfg.read_timeout, self.cfg.read_chunk)
        except Exception as e:
            self.last_error = f"Open failed: {e}"
            return
        last_stats_t = 0.0
        try:
            while not self._stop_evt.is_set():
                any_frame = False
                for frame in self._lr.read_frames():
                    any_frame = True
                    t = time.time()
                    if len(self.out_queue) >= self.cfg.max_queue:
                        # overflow: drop oldest burst
                        drop = max(1, self.cfg.max_queue // 4)
                        for _ in range(drop):
                            if self.out_queue:
                                self.out_queue.popleft()
                            else:
                                break
                    self.out_queue.append((t, frame))
                now = time.time()
                # 5~10Hz로 통계 스냅샷
                if now - last_stats_t > 0.1:
                    try:
                        s = self._lr.stats()
                        with self._stats_lock:
                            self._stats = s
                    except Exception:
                        pass
                    last_stats_t = now
                # 데이터가 없을 때 busy loop 방지
                if not any_frame:
                    time.sleep(0.001)
        except Exception as e:
            self.last_error = f"Reader error: {e}"
        finally:
            try:
                if self._lr:
                    self._lr.close()
            except Exception:
                pass

    def stop(self):
        self._stop_evt.set()

    def get_stats(self) -> Dict[str, float]:
        with self._stats_lock:
            return dict(self._stats)

class TimeSeriesBuffer:
    def __init__(self, history_sec: float):
        self.history_sec = history_sec
        self.t: Deque[float] = deque()
        self.values: Dict[str, Deque[float]] = {}

    def ensure_channels(self, names: List[str]):
        for n in names:
            if n not in self.values:
                self.values[n] = deque()

    def append(self, t: float, data: Dict[str, float]):
        self.t.append(t)
        for k, v in data.items():
            self.values[k].append(float(v))
        self._trim(t)

    def _trim(self, now: float):
        cutoff = now - self.history_sec
        # Trim time
        while self.t and self.t[0] < cutoff:
            self.t.popleft()
            # Trim each channel synchronized
            for dq in self.values.values():
                if dq:
                    dq.popleft()

    def get_arrays(self, channels: List[str], max_points: int) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        if not self.t:
            return np.array([]), {ch: np.array([]) for ch in channels}
        x = np.fromiter(self.t, dtype=float)
        arrays = {}
        for ch in channels:
            if ch in self.values:
                arrays[ch] = np.fromiter(self.values[ch], dtype=float)
            else:
                arrays[ch] = np.zeros_like(x)
        if len(x) > max_points:
            step = max(1, len(x) // max_points)
            sl = slice(None, None, step)
            x = x[sl]
            for ch in channels:
                arrays[ch] = arrays[ch][sl]
        # Time zero to relative seconds for plot readability
        x = x - x[-1]
        return x, arrays

class DataHub:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.queue: Deque[Tuple[float, LogData]] = deque(maxlen=cfg.max_queue)
        self.reader: Optional[SerialReaderThread] = None
        self.latest: Optional[LogData] = None
        self.last_t: float = 0.0

        # Time series per group
        self.ts: Dict[str, TimeSeriesBuffer] = {
            "raw_accel": TimeSeriesBuffer(cfg.history_sec),
            "raw_gyro": TimeSeriesBuffer(cfg.history_sec),
            "accel": TimeSeriesBuffer(cfg.history_sec),
            "gyro": TimeSeriesBuffer(cfg.history_sec),
            "raw_rc": TimeSeriesBuffer(cfg.history_sec),
            "rc": TimeSeriesBuffer(cfg.history_sec),
            "pid": TimeSeriesBuffer(cfg.history_sec),
            "esc": TimeSeriesBuffer(cfg.history_sec),
        }

        # Stats (UI-side)
        self.frames_ok_ui = 0

    def connect(self) -> Optional[str]:
        # 기존 연결 정리 후 새 연결
        if self.reader and self.reader.is_alive():
            self.disconnect()
        self.queue.clear()
        self.latest = None
        self.reader = SerialReaderThread(self.cfg, self.queue)
        self.reader.start()
        time.sleep(0.02)
        return self.reader.last_error

    def disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.queue.clear()

    def is_connected(self) -> bool:
        return self.reader is not None and self.reader.is_alive()

    def drain(self):
        # Drain all queued frames; keep last for numeric panel and 3D
        if not self.queue:
            return
        while self.queue:
            t, frame = self.queue.popleft()
            self.latest = frame
            self.last_t = t
            # Update time-series by group
            self._update_ts(t, frame)
            self.frames_ok_ui += 1

    def _update_ts(self, t: float, f: LogData):
        if self.cfg.enable_plots.get("raw_accel", True):
            self.ts["raw_accel"].ensure_channels(["x","y","z"])
            self.ts["raw_accel"].append(t, {"x": f.raw_accel_x, "y": f.raw_accel_y, "z": f.raw_accel_z})
        if self.cfg.enable_plots.get("raw_gyro", True):
            self.ts["raw_gyro"].ensure_channels(["x","y","z"])
            self.ts["raw_gyro"].append(t, {"x": f.raw_gyro_x, "y": f.raw_gyro_y, "z": f.raw_gyro_z})
        if self.cfg.enable_plots.get("accel", True):
            self.ts["accel"].ensure_channels(["x","y","z"])
            self.ts["accel"].append(t, {"x": f.accel_x, "y": f.accel_y, "z": f.accel_z})
        if self.cfg.enable_plots.get("gyro", True):
            self.ts["gyro"].ensure_channels(["x","y","z"])
            self.ts["gyro"].append(t, {"x": f.gyro_x, "y": f.gyro_y, "z": f.gyro_z})
        if self.cfg.enable_plots.get("raw_rc", True):
            self.ts["raw_rc"].ensure_channels(["throttle","roll","pitch","yaw"])
            self.ts["raw_rc"].append(t, {
                "throttle": f.raw_rc_throttle, "roll": f.raw_rc_roll, "pitch": f.raw_rc_pitch, "yaw": f.raw_rc_yaw
            })
        if self.cfg.enable_plots.get("rc", True):
            self.ts["rc"].ensure_channels(["throttle","roll","pitch","yaw"])
            self.ts["rc"].append(t, {
                "throttle": f.rc_throttle, "roll": f.rc_roll, "pitch": f.rc_pitch, "yaw": f.rc_yaw
            })
        if self.cfg.enable_plots.get("pid", True):
            self.ts["pid"].ensure_channels(["x","y","z"])
            self.ts["pid"].append(t, {"x": f.pid_x, "y": f.pid_y, "z": f.pid_z})
        if self.cfg.enable_plots.get("esc", True):
            self.ts["esc"].ensure_channels(["fl","fr","bl","br"])
            self.ts["esc"].append(t, {"fl": f.esc_fl, "fr": f.esc_fr, "bl": f.esc_bl, "br": f.esc_br})

    def reader_stats(self) -> Dict[str, float]:
        if self.reader:
            return self.reader.get_stats()
        return {"ok": 0, "crc_fail": 0, "desync": 0, "len_err": 0, "error_rate": 0.0, "fps": 0.0}


# ===========================================
# 유틸: 색/퀘터니언 회전/3D
# ===========================================

COLORS = {
    "x": (255, 70, 70),
    "y": (70, 200, 70),
    "z": (70, 120, 255),
    "throttle": (255, 170, 0),
    "roll": (0, 220, 180),
    "pitch": (180, 0, 220),
    "yaw": (200, 200, 0),
    "fl": (255, 80, 80),
    "fr": (80, 255, 80),
    "bl": (80, 80, 255),
    "br": (255, 160, 0),
}

def quat_to_rotmat_unity(w, x, y, z) -> np.ndarray:
    """
    Unity(Y-up, Left-handed)의 쿼터니언을 OpenGL(Right-handed, Z-up) 좌표계 회전행렬로 변환.
    변환: [x, y, z]_unity -> [x, z, y]_gl  (A 행렬)
    R_gl = A * R_unity * A^{-1}  (A는 직교행렬이므로 A^{-1}=A^T)
    """
    # normalize
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        R_u = np.eye(3)
    else:
        w, x, y, z = w/n, x/n, y/n, z/n
        # 표준 회전행렬 (수식 자체는 handed에 무관; 좌표계가 LH일 뿐)
        R_u = np.array([
            [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
        ], dtype=float)
    A = np.array([[1,0,0],[0,0,1],[0,1,0]], dtype=float)  # Corrected
    R_gl = A @ R_u @ A.T
    return R_gl

def apply_rotation(points: np.ndarray, R: np.ndarray) -> np.ndarray:
    # points: (N,3)
    return (R @ points.T).T

def make_box_geometry(sx=1.0, sy=1.0, sz=1.0):
    """
    중심 기준 직사각형 박스 geometry (8 vertices, 12 faces)
    """
    hx, hy, hz = sx/2, sy/2, sz/2
    # 8 vertices
    verts = np.array([
        [-hx, -hy, -hz],
        [ hx, -hy, -hz],
        [ hx,  hy, -hz],
        [-hx,  hy, -hz],
        [-hx, -hy,  hz],
        [ hx, -hy,  hz],
        [ hx,  hy,  hz],
        [-hx,  hy,  hz],
    ], dtype=float)
    # faces (two triangles per face)
    faces = np.array([
        [0,1,2], [0,2,3],  # bottom (z-)
        [4,5,6], [4,6,7],  # top    (z+)
        [0,1,5], [0,5,4],  # y- (front/back depends on coords)
        [2,3,7], [2,7,6],  # y+
        [1,2,6], [1,6,5],  # x+
        [0,3,7], [0,7,4],  # x-
    ], dtype=int)
    # per-face colors: 6색(두 삼각형씩 동일색)
    c = np.array([
        [1,0,0,0.8], [1,0,0,0.8],       # bottom - red
        [0,1,0,0.8], [0,1,0,0.8],       # top - green
        [0,0,1,0.8], [0,0,1,0.8],       # y- - blue
        [1,1,0,0.8], [1,1,0,0.8],       # y+ - yellow
        [1,0,1,0.8], [1,0,1,0.8],       # x+ - magenta
        [0,1,1,0.8], [0,1,1,0.8],       # x- - cyan
    ], dtype=float)
    return verts, faces, c

class Quaternion3DView(GLViewWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.setCameraPosition(distance=6.5, elevation=22, azimuth=35)
        g = GLGridItem()
        g.setSize(8, 8, 1)
        g.setSpacing(1, 1, 1)
        self.addItem(g)

        # World axes (reference)
        self.world_axes = []
        for i, seg in enumerate(self._make_axis_lines(1.5)):
            color = [(1,0.5,0.5,0.8),(0.5,1,0.5,0.8),(0.5,0.5,1,0.8)][i]
            item = GLLinePlotItem(pos=seg, color=color, width=2, antialias=True, mode='lines')
            self.addItem(item)
            self.world_axes.append(item)

        # Body mesh (box with 6 faces colored)
        self.base_verts, self.faces, self.face_colors = make_box_geometry(1.5, 2.2, 0.8)
        self.mesh = GLMeshItem(
            vertexes=self.base_verts, faces=self.faces, faceColors=self.face_colors,
            smooth=False, drawEdges=True, edgeColor=(0.2,0.2,0.2,1.0)
        )
        self.addItem(self.mesh)

    def _make_axis_lines(self, scale=1.0) -> List[np.ndarray]:
        o = np.array([0,0,0], dtype=float)
        X = np.array([scale,0,0], dtype=float)
        Y = np.array([0,scale,0], dtype=float)
        Z = np.array([0,0,scale], dtype=float)
        return [np.vstack([o, X]),
                np.vstack([o, Y]),
                np.vstack([o, Z])]

    def update_quat_unity(self, w: float, x: float, y: float, z: float):
        # Unity(Y-up,LH) -> OpenGL(RH,Z-up) 변환 회전행렬
        R = quat_to_rotmat_unity(w, x, y, z)
        # Qt matrix (column-major)
        M = QtGui.QMatrix4x4(
            R[0,0], R[0,1], R[0,2], 0.0,
            R[1,0], R[1,1], R[1,2], 0.0,
            R[2,0], R[2,1], R[2,2], 0.0,
            0.0,    0.0,    0.0,    1.0
        )
        self.mesh.setTransform(M)


# ===========================================
# UI 구성요소
# ===========================================

class ValuePanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        form = QtWidgets.QFormLayout(self)
        self.labels: Dict[str, QtWidgets.QLabel] = {}
        defs = [
            ("usage", "0.000"),
            ("usage_max", "0.000"),
            ("dt", "0.000"),
            ("dt_max", "0.000"),
            ("temp", "0.00"),
            ("rc_arm", "0"),
            ("rc_ctrl_mode", "0"),
            ("drone_arm", "0"),
        ]
        for name, init in defs:
            lab = QtWidgets.QLabel(init)
            lab.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.labels[name] = lab
            form.addRow(QtWidgets.QLabel(name), lab)

    def update_values(self, f: LogData):
        self.labels["usage"].setText(f"{f.usage:.3f}")
        self.labels["usage_max"].setText(f"{f.usage_max:.3f}")
        self.labels["dt"].setText(f"{f.dt*1000:.3f} ms")
        self.labels["dt_max"].setText(f"{f.dt_max*1000:.3f} ms")
        self.labels["temp"].setText(f"{f.temp:.2f} C")
        self.labels["rc_arm"].setText(str(int(f.rc_arm)))
        self.labels["rc_ctrl_mode"].setText(str(int(f.rc_ctrl_mode)))
        self.labels["drone_arm"].setText(str(int(f.drone_arm)))

class PlotGroupWidget(QtWidgets.QWidget):
    def __init__(self, title: str, channels: List[str], colors: Dict[str, Tuple[int,int,int]],
                 cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.title = title
        self.channels = channels
        self.colors = colors
        self.cfg = cfg

        v = QtWidgets.QVBoxLayout(self)
        self.plot = pg.PlotWidget(title=title)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend(offset=(10,10))
        self.plot.setLabel('bottom', 't', units='s', unitPrefix='')
        v.addWidget(self.plot)

        # Create curves
        self.curves: Dict[str, pg.PlotDataItem] = {}
        for ch in channels:
            c = self.colors.get(ch, (200,200,200))
            curve = self.plot.plot([], [], pen=pg.mkPen(c, width=self.cfg.line_width), name=ch)
            self.curves[ch] = curve

    def update_data(self, x: np.ndarray, arrays: Dict[str, np.ndarray]):
        if x.size == 0:
            return
        for ch, curve in self.curves.items():
            y = arrays.get(ch, None)
            if y is None or y.size == 0:
                continue
            curve.setData(x, y)

class Toolbar(QtWidgets.QWidget):
    connect_clicked = QtCore.pyqtSignal()
    disconnect_clicked = QtCore.pyqtSignal()
    refresh_ports_clicked = QtCore.pyqtSignal()
    plot_toggled = QtCore.pyqtSignal(str, bool)

    def __init__(self, cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        h = QtWidgets.QHBoxLayout(self)
        h.setContentsMargins(0,0,0,0)

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        if cfg.port:
            idx = self.port_combo.findText(cfg.port)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        self.port_combo.currentTextChanged.connect(self._on_port_changed)

        self.baud_combo = QtWidgets.QComboBox()
        for b in [115200, 230400, 460800, 921600, 1500000, 2000000]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText(str(cfg.baud))
        self.baud_combo.currentTextChanged.connect(self._on_baud_changed)

        self.ui_fps_spin = QtWidgets.QSpinBox()
        self.ui_fps_spin.setRange(1, 240)
        self.ui_fps_spin.setValue(cfg.ui_fps)
        self.ui_fps_spin.setSuffix(" fps")

        self.history_spin = QtWidgets.QDoubleSpinBox()
        self.history_spin.setRange(1.0, 120.0)
        self.history_spin.setSingleStep(1.0)
        self.history_spin.setValue(cfg.history_sec)
        self.history_spin.setSuffix(" s")

        self.max_points_spin = QtWidgets.QSpinBox()
        self.max_points_spin.setRange(200, 20000)
        self.max_points_spin.setValue(cfg.max_points)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.refresh_btn = QtWidgets.QPushButton("↻")

        self.connect_btn.clicked.connect(self.connect_clicked.emit)
        self.disconnect_btn.clicked.connect(self.disconnect_clicked.emit)
        self.refresh_btn.clicked.connect(self._on_refresh)

        # Toggles
        self.toggle_checks: Dict[str, QtWidgets.QCheckBox] = {}
        for key in ["raw_accel","raw_gyro","accel","gyro","raw_rc","rc","pid","esc"]:
            cb = QtWidgets.QCheckBox(key)
            cb.setChecked(cfg.enable_plots.get(key, True))
            cb.stateChanged.connect(lambda state, k=key: self._on_toggle(k, state))
            self.toggle_checks[key] = cb

        h.addWidget(QtWidgets.QLabel("Port:"))
        h.addWidget(self.port_combo)
        h.addWidget(self.refresh_btn)
        h.addWidget(QtWidgets.QLabel("Baud:"))
        h.addWidget(self.baud_combo)
        h.addSpacing(10)
        h.addWidget(QtWidgets.QLabel("UI:"))
        h.addWidget(self.ui_fps_spin)
        h.addWidget(QtWidgets.QLabel("History:"))
        h.addWidget(self.history_spin)
        h.addWidget(QtWidgets.QLabel("MaxPts:"))
        h.addWidget(self.max_points_spin)
        h.addSpacing(10)
        h.addWidget(self.connect_btn)
        h.addWidget(self.disconnect_btn)
        h.addSpacing(10)
        for cb in self.toggle_checks.values():
            h.addWidget(cb)
        h.addStretch(1)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.addItems(ports)

    def _on_refresh(self):
        self.refresh_ports_clicked.emit()
        self.refresh_ports()

    def _on_port_changed(self, text: str):
        self.cfg.port = text

    def _on_baud_changed(self, text: str):
        try:
            self.cfg.baud = int(text)
        except:
            pass

    def _on_toggle(self, key: str, state: int):
        enabled = (state == QtCore.Qt.Checked)
        self.cfg.enable_plots[key] = enabled
        self.plot_toggled.emit(key, enabled)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, cfg: AppConfig):
        super().__init__()
        self.setWindowTitle("ESP32 Quadcopter Realtime Visualizer")
        self.cfg = cfg
        self.hub = DataHub(cfg)

        # Theme
        if cfg.theme == "dark":
            pg.setConfigOption('background', (20,20,24))
            pg.setConfigOption('foreground', 'w')
        else:
            pg.setConfigOption('background', 'w')
            pg.setConfigOption('foreground', 'k')
        pg.setConfigOptions(antialias=cfg.antialias)

        # Central Layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central)
        v.setContentsMargins(6,6,6,6)
        v.setSpacing(6)

        # Top Toolbar
        self.toolbar = Toolbar(cfg)
        v.addWidget(self.toolbar)

        # Connect toolbar signals
        self.toolbar.connect_clicked.connect(self.on_connect)
        self.toolbar.disconnect_clicked.connect(self.on_disconnect)
        self.toolbar.ui_fps_spin.valueChanged.connect(self.on_ui_fps_changed)
        self.toolbar.history_spin.valueChanged.connect(self.on_history_changed)
        self.toolbar.max_points_spin.valueChanged.connect(self.on_max_points_changed)
        self.toolbar.plot_toggled.connect(self.on_plot_toggled)

        # Middle Splitter: Left values + plots + right 3D
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        v.addWidget(splitter, 1)

        # Left panel: Values + Stats
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        self.value_panel = ValuePanel()
        left_layout.addWidget(self.value_panel)

        self.stats_label = QtWidgets.QLabel("Stats: -")
        self.stats_label.setStyleSheet("QLabel { font-family: Consolas, monospace; }")
        self.stats_label.setMinimumWidth(360)
        left_layout.addWidget(self.stats_label)
        left_layout.addStretch(1)
        splitter.addWidget(left_widget)
        splitter.setStretchFactor(0, 0)

        # Center: Tabs for plots
        self.tabs = QtWidgets.QTabWidget()
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(1, 1)

        # Plot groups
        self.plot_groups: Dict[str, PlotGroupWidget] = {}
        def add_plot_tab(key, title, channels, color_keys):
            colors = {ch: COLORS.get(ck if ck else ch, (200,200,200)) for ch, ck in zip(channels, color_keys)}
            w = PlotGroupWidget(title, channels, colors, cfg)
            self.plot_groups[key] = w
            self.tabs.addTab(w, title)

        add_plot_tab("raw_accel", "Raw Accel (x,y,z)", ["x","y","z"], ["x","y","z"])
        add_plot_tab("raw_gyro",  "Raw Gyro (x,y,z)",  ["x","y","z"], ["x","y","z"])
        add_plot_tab("accel",     "Accel (x,y,z)",     ["x","y","z"], ["x","y","z"])
        add_plot_tab("gyro",      "Gyro (x,y,z)",      ["x","y","z"], ["x","y","z"])
        add_plot_tab("raw_rc",    "Raw RC (t,r,p,y)",  ["throttle","roll","pitch","yaw"], ["throttle","roll","pitch","yaw"])
        add_plot_tab("rc",        "RC (t,r,p,y)",      ["throttle","roll","pitch","yaw"], ["throttle","roll","pitch","yaw"])
        add_plot_tab("pid",       "PID (x,y,z)",       ["x","y","z"], ["x","y","z"])
        add_plot_tab("esc",       "ESC (fl,fr,bl,br)", ["fl","fr","bl","br"], ["fl","fr","bl","br"])

        # Right: 3D views
        right_container = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_container)
        self.view_rot = Quaternion3DView("rot_*")
        self.view_head = Quaternion3DView("target_tilt_quat_*")
        if self.cfg.enable_3d:
            right_layout.addWidget(QtWidgets.QLabel("Rotation (rot_wxyz)"))
            right_layout.addWidget(self.view_rot, 1)
            right_layout.addWidget(QtWidgets.QLabel("Heading (target_tilt_quat_wxyz)"))
            right_layout.addWidget(self.view_head, 1)
        splitter.addWidget(right_container)
        splitter.setStretchFactor(2, 0)

        # Timer for UI updates
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.set_ui_fps(cfg.ui_fps)

        # Status bar
        self.status = self.statusBar()
        self.status.showMessage("Ready")

        # 초기 토글 상태를 탭에 반영
        for key, enabled in self.cfg.enable_plots.items():
            self._set_plot_visibility(key, enabled)

    # ------- Toolbar handlers -------
    def on_connect(self):
        err = self.hub.connect()
        if err:
            QtWidgets.QMessageBox.critical(self, "Connect Failed", err)
        else:
            self.status.showMessage(f"Connected: {self.cfg.port} @ {self.cfg.baud}")
            if not self.timer.isActive():
                self.timer.start()

    def on_disconnect(self):
        self.hub.disconnect()
        self.status.showMessage("Disconnected")

    def on_ui_fps_changed(self, val: int):
        self.set_ui_fps(val)

    def on_history_changed(self, val: float):
        self.cfg.history_sec = float(val)
        # update buffers' history
        for ts in self.hub.ts.values():
            ts.history_sec = self.cfg.history_sec

    def on_max_points_changed(self, val: int):
        self.cfg.max_points = int(val)

    def on_plot_toggled(self, key: str, enabled: bool):
        self._set_plot_visibility(key, enabled)

    def _set_plot_visibility(self, key: str, enabled: bool):
        w = self.plot_groups.get(key)
        if not w:
            return
        # 시각적/연산적 모두 비활성화
        w.setEnabled(enabled)
        w.plot.setVisible(enabled)

    def set_ui_fps(self, fps: int):
        self.cfg.ui_fps = max(1, int(fps))
        interval = int(1000 / self.cfg.ui_fps)
        self.timer.setInterval(interval)

    # ------- Main tick -------
    def on_tick(self):
        # Drain new frames
        self.hub.drain()
        f = self.hub.latest
        if f:
            # Numeric panel
            self.value_panel.update_values(f)
            # 3D views (Unity quaternion -> GL)
            if self.cfg.enable_3d:
                self.view_rot.update_quat_unity(f.rot_w, f.rot_x, f.rot_y, f.rot_z)
                self.view_head.update_quat_unity(
                    f.target_tilt_quat_w, f.target_tilt_quat_x, f.target_tilt_quat_y, f.target_tilt_quat_z
                )

        # Update plots
        for key, group in self.plot_groups.items():
            if not self.cfg.enable_plots.get(key, True):
                continue
            ts = self.hub.ts[key]
            x, arrays = ts.get_arrays(group.channels, self.cfg.max_points)
            group.update_data(x, arrays)

        # Update stats line (LogReader stats + UI)
        s = self.hub.reader_stats()
        qlen = len(self.hub.queue)
        self.stats_label.setText(
            "SrcFPS:{fps:5.1f} | OK:{ok:6d} CRC:{crc:5d} DS:{ds:5d} LEN:{le:5d} ERR:{er:6.2f}%"
            " | Queue:{ql:4d} UI:{uifps:3d}fps".format(
                fps=s.get("fps",0.0),
                ok=int(s.get("ok",0)),
                crc=int(s.get("crc_fail",0)),
                ds=int(s.get("desync",0)),
                le=int(s.get("len_err",0)),
                er=float(s.get("error_rate",0.0)*100.0),
                ql=qlen,
                uifps=self.cfg.ui_fps
            )
        )

    def closeEvent(self, e: QtGui.QCloseEvent):
        try:
            self.hub.disconnect()
        except Exception:
            pass
        return super().closeEvent(e)


# ===========================================
# Entrypoint
# ===========================================

def _install_sigint_handler():
    try:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    except Exception:
        pass

def main():
    _install_sigint_handler()
    parser = argparse.ArgumentParser(description="ESP32 Quadcopter Realtime Visualizer (pyqtgraph)")
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--ui-fps", type=int, default=60)
    parser.add_argument("--history", type=float, default=3.0)
    parser.add_argument("--max-points", type=int, default=1000)
    parser.add_argument("--max-queue", type=int, default=8192)
    parser.add_argument("--light", action="store_true", help="light theme")
    parser.add_argument("--no-3d", action="store_true", help="disable 3D views")
    args = parser.parse_args()

    cfg = AppConfig(
        port=args.port,
        baud=args.baud,
        ui_fps=args.ui_fps,
        history_sec=args.history,
        max_points=args.max_points,
        max_queue=args.max_queue,
        theme="light" if args.light else "dark",
        enable_3d=(not args.no_3d),
    )

    # High-DPI friendly (QApplication 생성 전)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)

    win = MainWindow(cfg)
    win.resize(1700, 950)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
