import serial
import struct
import time
import sys
import signal
from dataclasses import dataclass
from typing import Optional, Iterator, Iterable, Callable, List
from collections import deque

"""
ESP32 Quadcopter Binary Log Reader

프레임 형식 (펌웨어 기준):
[SOF 0xAA 0x55][Length(LE 2B = 163)][Payload 163B][CRC16-CCITT-FALSE(LE 2B)]
Payload 레이아웃 = '<33f3B7f' (총 163B)

개선 사항:
- 부분 수신 데이터 내부 버퍼(self._buf)에 누적 → 다음 호출에서 이어서 파싱
- 버퍼 과도 성장 방지: MAX_BUFFER 초과 시 최근 MAX_FRAME_WINDOW 만큼만 유지
- 재동기화: SOF 탐색, 길이 불일치/CRC 실패 시 1바이트 drop 반복
- FPS 지수평활
- 통계 출력 함수, raw/CSV 저장 지원
- 구조/길이 런타임 검증
"""

SOF = b'\xAA\x55'
PAYLOAD_SIZE = 163
STRUCT_FMT = '<33f3B7f'
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)
assert STRUCT_SIZE == PAYLOAD_SIZE, f"struct size mismatch {STRUCT_SIZE} != {PAYLOAD_SIZE}"

FRAME_SIZE = 2 + 2 + PAYLOAD_SIZE + 2  # 169
MAX_BUFFER_MULTIPLIER = 8
MAX_BUFFER = FRAME_SIZE * MAX_BUFFER_MULTIPLIER
# 버퍼 축소 시 유지할 최소 최근 바이트 길이
BUFFER_SHRINK_KEEP = FRAME_SIZE * 2

FIELD_ORDER = [
    # 33 floats
    'usage','usage_max','dt','dt_max',
    'raw_accel_x','raw_accel_y','raw_accel_z',
    'raw_gyro_x','raw_gyro_y','raw_gyro_z',
    'accel_x','accel_y','accel_z',
    'gyro_x','gyro_y','gyro_z',
    'temp',
    'rot_w','rot_x','rot_y','rot_z',
    'target_tilt_quat_w','target_tilt_quat_x','target_tilt_quat_y','target_tilt_quat_z',
    'raw_rc_throttle','raw_rc_roll','raw_rc_pitch','raw_rc_yaw',
    'rc_throttle','rc_roll','rc_pitch','rc_yaw',
    # 3 bytes
    'rc_arm','rc_ctrl_mode','drone_arm',
    # 7 floats
    'pid_x','pid_y','pid_z',
    'esc_fl','esc_fr','esc_bl','esc_br'
]

@dataclass
class LogData:
    usage: float
    usage_max: float
    dt: float
    dt_max: float
    raw_accel_x: float
    raw_accel_y: float
    raw_accel_z: float
    raw_gyro_x: float
    raw_gyro_y: float
    raw_gyro_z: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    temp: float
    rot_w: float
    rot_x: float
    rot_y: float
    rot_z: float
    target_tilt_quat_w: float
    target_tilt_quat_x: float
    target_tilt_quat_y: float
    target_tilt_quat_z: float
    raw_rc_throttle: float
    raw_rc_roll: float
    raw_rc_pitch: float
    raw_rc_yaw: float
    rc_throttle: float
    rc_roll: float
    rc_pitch: float
    rc_yaw: float
    rc_arm: int
    rc_ctrl_mode: int
    drone_arm: int
    pid_x: float
    pid_y: float
    pid_z: float
    esc_fl: float
    esc_fr: float
    esc_bl: float
    esc_br: float

    @property
    def pid_val(self):
        return (self.pid_x, self.pid_y, self.pid_z)

    @property
    def esc_throttle(self):
        return (self.esc_fl, self.esc_fr, self.esc_bl, self.esc_br)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class LogReader:
    def __init__(self,
                 port: str,
                 baud: int = 115200,
                 timeout: float = 0.05,
                 read_chunk: int = 512,
                 auto_open: bool = True):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.read_chunk = read_chunk
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 통계
        self.frames_ok = 0
        self.frames_crc_fail = 0
        self.frames_desync = 0
        self.frames_length_err = 0
        self.last_frame_time: Optional[float] = None
        self.fps = 0.0
        self._running = False

        # FPS 계산: 최근 window 내 타임스탬프 유지
        self._fps_window_sec = 1.0
        self._fps_times = deque()

        if auto_open:
            self.open()

    # --------------- Serial 관리 ---------------
    def open(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except serial.SerialException as e:
            raise RuntimeError(f"Serial open failed: {e}")

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def __enter__(self):
        if not self.ser:
            self.open()
        self._running = True
        return self

    def __exit__(self, exc_type, exc, tb):
        self._running = False
        self.close()

    # --------------- 내부 유틸 ---------------
    def _update_fps(self):
        now = time.time()
        self.last_frame_time = now

        # 최근 프레임 타임스탬프 누적
        self._fps_times.append(now)

        # 윈도우 밖(과거) 타임스탬프 제거
        cutoff = now - self._fps_window_sec
        while self._fps_times and self._fps_times[0] < cutoff:
            self._fps_times.popleft()

        # 윈도우 내 평균 FPS = (프레임수-1) / (마지막-처음) 시간
        if len(self._fps_times) >= 2:
            span = self._fps_times[-1] - self._fps_times[0]
            if span > 0:
                self.fps = (len(self._fps_times) - 1) / span
            else:
                self.fps = 0.0
        else:
            self.fps = 0.0

    def _shrink_buffer_if_needed(self):
        if len(self._buf) > MAX_BUFFER:
            # 최근 데이터만 남긴다 (SOF 미포함 중간 데이터 일부 손실 가능)
            self._buf[:] = self._buf[-BUFFER_SHRINK_KEEP:]

    # --------------- 파싱 로직 ---------------
    def _try_extract_frames(self) -> Iterable[LogData]:
        out: List[LogData] = []
        while True:
            sof_index = self._buf.find(SOF)
            if sof_index < 0:
                # SOF 없음 → 축소 검사 후 종료
                self._shrink_buffer_if_needed()
                break
            if sof_index > 0:
                # 불필요 데이터 제거
                self.frames_desync += 1
                del self._buf[:sof_index]
            # 최소 헤더(4B) 검사
            if len(self._buf) < 4:
                break
            length = self._buf[2] | (self._buf[3] << 8)
            if length != PAYLOAD_SIZE:
                # 길이 오류 → 1바이트 drop
                self.frames_length_err += 1
                del self._buf[0:1]
                continue
            need = 2 + 2 + length + 2
            if len(self._buf) < need:
                # 아직 전체 도착 안 함
                break
            payload = self._buf[4:4+length]
            recv_crc = self._buf[4+length] | (self._buf[5+length] << 8)
            calc_crc = crc16_ccitt_false(payload)
            if recv_crc != calc_crc:
                self.frames_crc_fail += 1
                del self._buf[0:1]
                continue
            # 정상 프레임
            del self._buf[:need]
            values = struct.unpack(STRUCT_FMT, payload)
            data_dict = {FIELD_ORDER[i]: values[i] for i in range(len(FIELD_ORDER))}
            ld = LogData(**data_dict)
            self.frames_ok += 1
            self._update_fps()
            out.append(ld)
        return out

    # --------------- 외부 API ---------------
    def read_frames(self) -> Iterator[LogData]:
        if not self.ser:
            raise RuntimeError("Serial not opened")
        chunk = self.ser.read(self.read_chunk)
        if chunk:
            self._buf.extend(chunk)
        for frame in self._try_extract_frames():
            yield frame

    def iter(self) -> Iterator[LogData]:
        while True:
            yield from self.read_frames()

    def read_blocking(self, timeout_sec: Optional[float] = None) -> Optional[LogData]:
        start = time.time()
        while True:
            for f in self.read_frames():
                return f
            if timeout_sec is not None and (time.time() - start) > timeout_sec:
                return None

    def stats(self) -> dict:
        total_err = self.frames_crc_fail + self.frames_desync + self.frames_length_err
        total = self.frames_ok + total_err
        return {
            "ok": self.frames_ok,
            "crc_fail": self.frames_crc_fail,
            "desync": self.frames_desync,
            "len_err": self.frames_length_err,
            "error_rate": (total_err / total) if total else 0.0,
            "fps": self.fps
        }

    def print_stats(self):
        s = self.stats()
        print(f"[STATS] ok={s['ok']} crc={s['crc_fail']} desync={s['desync']} len={s['len_err']} "
              f"err_rate={s['error_rate']*100:.2f}% fps={s['fps']:.1f}")

    def print_stats_periodic(self, interval_sec: float = 5.0):
        now = time.time()
        if not hasattr(self, "_last_stats_t"):
            self._last_stats_t = now
        if now - self._last_stats_t >= interval_sec:
            self.print_stats()
            self._last_stats_t = now

    def print_loop(self, limit: Optional[int] = None, stats_interval: float = 5.0):
        i = 0
        try:
            for frame in self.iter():
                print(f"[{i}] usage={frame.usage:.3f} dt={frame.dt*1000:.3f}ms "
                      f"acc=({frame.accel_x:.2f},{frame.accel_y:.2f},{frame.accel_z:.2f}) "
                      f"gyro=({frame.gyro_x:.1f},{frame.gyro_y:.1f},{frame.gyro_z:.1f}) "
                      f"arm={frame.rc_arm} mode={frame.rc_ctrl_mode} "
                      f"pid=({frame.pid_x:.4f},{frame.pid_y:.4f},{frame.pid_z:.4f}) "
                      f"esc=({frame.esc_fl:.3f},{frame.esc_fr:.3f},{frame.esc_bl:.3f},{frame.esc_br:.3f}) "
                      f"fps={self.fps:.1f}")
                i += 1
                self.print_stats_periodic(stats_interval)
                if limit and i >= limit:
                    break
        except KeyboardInterrupt:
            print("\n[Interrupted]")
            self.print_stats()

    def save_csv(self, path: str, seconds: float = 5.0, flush_every: int = 50):
        end_t = time.time() + seconds
        count = 0
        with open(path, "w", encoding="utf-8") as f:
            f.write(",".join(FIELD_ORDER) + "\n")
            while time.time() < end_t:
                for frame in self.read_frames():
                    row = [getattr(frame, k) for k in FIELD_ORDER]
                    f.write(",".join(str(v) for v in row) + "\n")
                    count += 1
                    if count % flush_every == 0:
                        f.flush()
        self.print_stats()

    def save_raw(self, path: str, seconds: float = 5.0):
        """
        프레임 전체(헤더+페이로드+CRC)를 그대로 raw 저장.
        재분석/재파싱용.
        """
        end_t = time.time() + seconds
        stored = 0
        with open(path, "wb") as f:
            while time.time() < end_t:
                chunk = self.ser.read(self.read_chunk)
                if chunk:
                    f.write(chunk)
                    stored += len(chunk)
        print(f"[RAW] stored {stored} bytes")

