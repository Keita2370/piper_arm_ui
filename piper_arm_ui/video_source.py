#!/usr/bin/env python3
"""WebRTC 用の映像ソース.

- RosImageTrack : sensor_msgs/Image を購読して WebRTC の VideoStreamTrack にする
- DummyVideoTrack : カメラ未定の間、テストパターンを生成する

いずれも aiortc の VideoStreamTrack として同じように扱える。
"""

import asyncio
import fractions
import time

import numpy as np

try:
    from av import VideoFrame
    from aiortc import VideoStreamTrack
    AIORTC_AVAILABLE = True
except ImportError:  # aiortc 未インストールでも import 自体は通るようにする
    AIORTC_AVAILABLE = False
    VideoFrame = None

    class VideoStreamTrack:  # type: ignore[no-redef]
        kind = 'video'

        def __init__(self):
            raise RuntimeError(
                'aiortc / av がインストールされていません。'
                'pip install aiortc av を実行してください。')

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

VIDEO_CLOCK_RATE = 90000
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)


def _ros_image_to_bgr(msg):
    """sensor_msgs/Image を BGR の numpy 配列に変換する（cv_bridge 不要）."""
    height, width = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()

    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(height, msg.step // 3, 3)[:, :width, :]
        return img[:, :, ::-1].copy() if enc == 'rgb8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(height, msg.step // 4, 4)[:, :width, :3]
        return img[:, :, ::-1].copy() if enc == 'rgba8' else img.copy()
    if enc == 'mono8':
        gray = buf.reshape(height, msg.step)[:, :width]
        return np.repeat(gray[:, :, None], 3, axis=2)
    if enc == 'mono16':
        gray = buf.view(np.uint16).reshape(height, msg.step // 2)[:, :width]
        gray8 = (gray >> 8).astype(np.uint8)
        return np.repeat(gray8[:, :, None], 3, axis=2)
    raise ValueError('未対応のエンコーディング: %s' % msg.encoding)


def _compressed_to_bgr(msg):
    """sensor_msgs/CompressedImage を BGR の numpy 配列に変換する.

    別PCから LAN 越しに映像を受けるときに使う。生の Image は
    640x480 の rgb8 で 30fps だと 220Mbps 近くになり Wi-Fi では流せないが、
    JPEG 圧縮なら 1/10 以下で済む。
    """
    if not CV2_AVAILABLE:
        raise RuntimeError(
            '圧縮画像のデコードには OpenCV が必要です '
            '(sudo apt install python3-opencv)')
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError('圧縮画像をデコードできませんでした (format=%s)'
                         % getattr(msg, 'format', '?'))
    return img


class _BaseTrack(VideoStreamTrack):
    """共通のタイムスタンプ処理."""

    def __init__(self, fps):
        super().__init__()
        self.fps = max(1, int(fps))
        self._start = None
        self._count = 0
        self._dropped = 0      # 遅れて基準を取り直した回数（診断用）

    # これ以上遅れたら追いつくのを諦めて基準を取り直す [s]。
    # 取り戻そうとして連続で送ると、受け側にはまとめて届いてカクついて見える。
    MAX_LAG = 0.5

    async def _next_timestamp(self):
        if self._start is None:
            self._start = time.time()
            self._count = 0
        else:
            self._count += 1
            target = self._start + self._count / self.fps
            delay = target - time.time()
            if delay > 0:
                await asyncio.sleep(delay)
            elif -delay > self.MAX_LAG:
                # エンコードが間に合わず大きく遅れた。ここで基準を今に合わせ直す。
                # そうしないと以後ずっと「遅れを取り戻す」モードのままになり、
                # フレームが束で届いて映像がガクガクし続ける。
                self._dropped += 1
                self._start = time.time() - self._count / self.fps
        pts = int(self._count * VIDEO_CLOCK_RATE / self.fps)
        return pts, VIDEO_TIME_BASE


class DummyVideoTrack(_BaseTrack):
    """カメラ未接続時のテストパターン.

    detections_provider() が検出結果リストを返す場合は、その位置に
    レモンを描画するので、UI 側のオーバーレイと位置が一致する。
    """

    def __init__(self, width=640, height=480, fps=15, label='DUMMY',
                 detections_provider=None):
        super().__init__(fps)
        self.width = width
        self.height = height
        self.label = label
        self.detections_provider = detections_provider

    def _render(self):
        h, w = self.height, self.width
        # 背景: 果樹園を思わせる緑のグラデーション + 走査線
        yy = np.linspace(0, 1, h, dtype=np.float32)[:, None]
        xx = np.linspace(0, 1, w, dtype=np.float32)[None, :]
        img = np.zeros((h, w, 3), dtype=np.float32)
        img[:, :, 0] = 40 + 25 * xx                      # B
        img[:, :, 1] = 70 + 60 * (1 - yy) + 20 * xx      # G
        img[:, :, 2] = 35 + 20 * yy                      # R
        phase = (time.time() * 40) % h
        band = np.exp(-((np.arange(h) - phase) ** 2) / (2 * 18.0 ** 2))[:, None, None]
        img += band * 18.0
        frame = np.clip(img, 0, 255).astype(np.uint8)

        targets = []
        if self.detections_provider is not None:
            try:
                targets = self.detections_provider() or []
            except Exception:  # noqa: BLE001
                targets = []

        if CV2_AVAILABLE:
            for t in targets:
                cx = int(float(t.get('u', 0.5)) * w)
                cy = int(float(t.get('v', 0.5)) * h)
                r = max(8, int(float(t.get('w', 0.12)) * w * 0.5))
                cv2.circle(frame, (cx, cy), r, (60, 210, 245), -1)
                cv2.circle(frame, (cx, cy), r, (30, 140, 180), 2)
                cv2.line(frame, (cx, cy - r), (cx, cy - r - 10), (40, 120, 60), 3)
            cv2.putText(frame, '%s  %s' % (self.label, time.strftime('%H:%M:%S')),
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, 'no camera connected - test pattern',
                        (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 230, 210), 1)
        else:
            for t in targets:
                cx = int(float(t.get('u', 0.5)) * w)
                cy = int(float(t.get('v', 0.5)) * h)
                r = max(8, int(float(t.get('w', 0.12)) * w * 0.5))
                y0, y1 = max(0, cy - r), min(h, cy + r)
                x0, x1 = max(0, cx - r), min(w, cx + r)
                frame[y0:y1, x0:x1] = (60, 210, 245)
        return frame

    async def recv(self):
        pts, time_base = await self._next_timestamp()
        frame = VideoFrame.from_ndarray(self._render(), format='bgr24')
        frame.pts = pts
        frame.time_base = time_base
        return frame


class RosImageTrack(_BaseTrack):
    """sensor_msgs/Image の最新フレームを WebRTC で流す.

    ノード側が set_frame() で最新画像を書き込み、こちらは fps 間隔で読み出す。
    画像が来ていない間は「待機中」を示す暗いフレームを返すので、
    カメラが落ちても WebRTC 接続自体は維持される。
    """

    def __init__(self, width=640, height=480, fps=15, label='CAMERA'):
        super().__init__(fps)
        self.width = width
        self.height = height
        self.label = label
        self._frame = None
        self._frame_stamp = 0.0

    def set_frame(self, bgr):
        self._frame = bgr
        self._frame_stamp = time.time()

    @property
    def has_recent_frame(self):
        return self._frame is not None and (time.time() - self._frame_stamp) < 2.0

    def _placeholder(self):
        img = np.full((self.height, self.width, 3), 25, dtype=np.uint8)
        if CV2_AVAILABLE:
            cv2.putText(img, 'waiting for %s ...' % self.label,
                        (20, self.height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (160, 160, 160), 2)
        return img

    async def recv(self):
        pts, time_base = await self._next_timestamp()
        img = self._frame if self.has_recent_frame else self._placeholder()
        if img.shape[0] != self.height or img.shape[1] != self.width:
            if CV2_AVAILABLE:
                img = cv2.resize(img, (self.width, self.height))
            else:
                img = self._placeholder()
        frame = VideoFrame.from_ndarray(np.ascontiguousarray(img), format='bgr24')
        frame.pts = pts
        frame.time_base = time_base
        return frame
