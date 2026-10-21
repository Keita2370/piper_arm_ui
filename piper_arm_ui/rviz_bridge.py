#!/usr/bin/env python3
"""RViz2 をブラウザUIに埋め込むためのブリッジ.

やっていること:
  1. 仮想ディスプレイ (Xvfb) を立てて、その上で本物の rviz2 を起動する
  2. その画面を定期的にキャプチャして sensor_msgs/Image で publish する
     （ui_server がそれを WebRTC でブラウザへ流す）
  3. ブラウザからのマウス操作を /rviz_ui/input で受け取り、
     X の XTEST 拡張で仮想ディスプレイに注入する（視点回転・ズームができる）

「RViz風の自作ビュー」ではなく RViz2 そのものなので、RobotModel / TF /
PointCloud2 / MarkerArray など、いつも RViz で見ているものがそのまま出る。

すでにモニタが繋がっていて :0 に rviz2 を出しているなら、
  use_xvfb:=false display:=:0 launch_rviz:=false
とすればその画面をキャプチャするだけになる。
"""

import json
import os
import shutil
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from piper_arm_ui.common import TOPIC_RVIZ_INPUT

# --- 画面キャプチャのバックエンド ---
try:
    import mss
    MSS_AVAILABLE = True
except ImportError:
    MSS_AVAILABLE = False

# --- 入力注入のバックエンド ---
try:
    from Xlib import X, XK, display as xdisplay
    from Xlib.ext import xtest
    XLIB_AVAILABLE = True
except ImportError:
    XLIB_AVAILABLE = False

BUTTON_MAP = {0: 1, 1: 2, 2: 3}          # ブラウザのボタン番号 -> X のボタン番号
WHEEL_UP, WHEEL_DOWN = 4, 5


class ScreenCapture:
    """X の画面を BGR の numpy 配列で取り出す."""

    def __init__(self, display, width, height, logger):
        self.display = display
        self.width = width
        self.height = height
        self.logger = logger
        self._sct = None
        self._ffmpeg = None
        self._frame_bytes = width * height * 4

        if MSS_AVAILABLE:
            try:
                self._sct = mss.mss(display=display)
                self.logger.info('画面キャプチャ: mss')
                return
            except Exception as exc:  # noqa: BLE001
                self.logger.warn('mss を使えないため ffmpeg に切り替えます (%s)' % exc)
        self._start_ffmpeg()

    def _start_ffmpeg(self):
        if not shutil.which('ffmpeg'):
            raise RuntimeError(
                '画面キャプチャの手段がありません。'
                'pip install mss か apt install ffmpeg を実行してください。')
        cmd = [
            'ffmpeg', '-loglevel', 'error', '-f', 'x11grab',
            '-video_size', '%dx%d' % (self.width, self.height),
            '-i', '%s.0' % self.display,
            '-f', 'rawvideo', '-pix_fmt', 'bgra', '-',
        ]
        self._ffmpeg = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        self.logger.info('画面キャプチャ: ffmpeg x11grab')

    def grab(self):
        if self._sct is not None:
            region = {'left': 0, 'top': 0, 'width': self.width, 'height': self.height}
            shot = self._sct.grab(region)
            arr = np.frombuffer(shot.raw, dtype=np.uint8)
            arr = arr.reshape(shot.height, shot.width, 4)
            return np.ascontiguousarray(arr[:, :, :3])
        raw = self._ffmpeg.stdout.read(self._frame_bytes)
        if len(raw) < self._frame_bytes:
            return None
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width, 4)
        return np.ascontiguousarray(arr[:, :, :3])

    def close(self):
        if self._ffmpeg is not None:
            # 先に stdout を閉じると ffmpeg 側が Broken pipe を出すので、
            # terminate してから閉じる
            self._ffmpeg.terminate()
            try:
                self._ffmpeg.wait(timeout=2)
            except Exception:  # noqa: BLE001
                self._ffmpeg.kill()
            if self._ffmpeg.stdout:
                self._ffmpeg.stdout.close()
        if self._sct is not None:
            try:
                self._sct.close()
            except Exception:  # noqa: BLE001
                pass


class InputInjector:
    """ブラウザのマウス操作を X に注入する（XTEST、無ければ xdotool）."""

    def __init__(self, display, logger):
        self.display = display
        self.logger = logger
        self._disp = None
        self._xdotool = shutil.which('xdotool')

        if XLIB_AVAILABLE:
            try:
                self._disp = xdisplay.Display(display)
                self.logger.info('入力注入: XTEST')
                return
            except Exception as exc:  # noqa: BLE001
                self.logger.warn('XTEST を使えません (%s)' % exc)
        if self._xdotool:
            self.logger.info('入力注入: xdotool')
        else:
            self.logger.warn(
                '入力注入の手段がありません（表示のみになります）。'
                'pip install python-xlib か apt install xdotool を検討してください。')

    @property
    def available(self):
        return self._disp is not None or self._xdotool is not None

    def move(self, x, y):
        if self._disp is not None:
            xtest.fake_input(self._disp, X.MotionNotify, x=int(x), y=int(y))
            self._disp.sync()
        elif self._xdotool:
            self._run(['mousemove', '--sync', str(int(x)), str(int(y))])

    def button(self, index, pressed):
        btn = BUTTON_MAP.get(index, 1)
        if self._disp is not None:
            event = X.ButtonPress if pressed else X.ButtonRelease
            xtest.fake_input(self._disp, event, btn)
            self._disp.sync()
        elif self._xdotool:
            self._run(['mousedown' if pressed else 'mouseup', str(btn)])

    def key(self, name):
        """キーを1回叩く（rviz2 に F11 を送って全画面にするのに使う）.

        WM が無い Xvfb では入力フォーカスが PointerRoot なので、
        呼ぶ前に move() でポインタを rviz のウィンドウ上へ置いておくこと。
        """
        if self._disp is not None:
            keysym = XK.string_to_keysym(name)
            keycode = self._disp.keysym_to_keycode(keysym)
            if not keycode:
                self.logger.warn('キー %s のキーコードを引けません' % name)
                return False
            xtest.fake_input(self._disp, X.KeyPress, keycode)
            xtest.fake_input(self._disp, X.KeyRelease, keycode)
            self._disp.sync()
            return True
        if self._xdotool:
            self._run(['key', name])
            return True
        return False

    def wheel(self, delta):
        btn = WHEEL_UP if delta < 0 else WHEEL_DOWN
        if self._disp is not None:
            xtest.fake_input(self._disp, X.ButtonPress, btn)
            xtest.fake_input(self._disp, X.ButtonRelease, btn)
            self._disp.sync()
        elif self._xdotool:
            self._run(['click', str(btn)])

    def _run(self, args):
        env = dict(os.environ, DISPLAY=self.display)
        subprocess.run(['xdotool'] + args, env=env, check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class RvizBridge(Node):

    def __init__(self):
        super().__init__('rviz_bridge')

        self.declare_parameter('display', ':99')
        self.declare_parameter('use_xvfb', True)
        self.declare_parameter('launch_rviz', True)
        self.declare_parameter('rviz_config', '')
        # 解像度と fps。UI の RViz パネルは実際には 600〜700px 幅で表示されるので
        # 1280x720 は過剰。実測では VP8 エンコードが 1 枚あたり
        #   1280x720 = 29.7ms / 960x540 = 14.8ms / 848x480 = 13.1ms
        # かかり、1280x720 のままだと 30fps 前後で頭打ちになる。
        # なめらかさは解像度より fps が効くので、解像度を落として fps を上げる。
        self.declare_parameter('width', 960)
        self.declare_parameter('height', 540)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('image_topic', '/rviz_ui/image')
        self.declare_parameter('allow_input', True)
        # 'auto' … Xvfb のときだけソフトウェア GL を強制する。
        # 実ディスプレイ(:0)に出すときに強制すると GPU 描画を捨ててしまい、
        # かえって遅くなる。true/false で明示指定もできる。
        self.declare_parameter('software_rendering', 'auto')
        # UI パネルは横 600px 程度しかないので、左右のドック（Displays / Views）を
        # 畳んで 3D ビューを目一杯使う。元の設定ファイルは書き換えず、
        # 一時ファイルにコピーしてから畳む。
        self.declare_parameter('hide_docks', True)
        # メニューバー・ツールバー・ステータスバー（＝画面上の文字）を消して
        # 3Dビューだけにする。rviz2 の全画面表示(F11)を1回叩いて実現する。
        self.declare_parameter('hide_chrome', True)
        self.declare_parameter('chrome_delay', 3.0)

        # --- 初期視点（rviz_default_plugins/Orbit）---
        # カメラ位置は注視点から見て
        #   x = d cos(yaw) cos(pitch), y = d sin(yaw) cos(pitch), z = d sin(pitch)
        # PiPER は base_link の +X 方向へ手を伸ばすので
        #   yaw = 3.14159 : 後ろから（アームが伸びる向きを見る）  ← 既定
        #   yaw = 0.0     : 正面から（アームがこちらへ向かってくる）
        #   yaw = 1.5708  : 左から   /  yaw = -1.5708 : 右から
        # pitch は 0 が真横、+ で見下ろし（1.5708 が真上）。
        self.declare_parameter('apply_view', True)
        self.declare_parameter('view_yaw', 3.14159)
        self.declare_parameter('view_pitch', 0.25)
        self.declare_parameter('view_distance', 1.2)
        self.declare_parameter('view_focal_point', [0.1, 0.0, 0.2])

        p = self.get_parameter
        self.display = p('display').value
        self.width = int(p('width').value)
        self.height = int(p('height').value)
        self.fps = max(1.0, float(p('fps').value))
        self.allow_input = bool(p('allow_input').value)

        self.procs = []
        self._tmp_config = None
        self._rviz_started = False
        if bool(p('use_xvfb').value):
            self._start_xvfb()
        if bool(p('launch_rviz').value):
            config = p('rviz_config').value
            if bool(p('hide_docks').value) or bool(p('apply_view').value):
                config = self._make_embedded_config(
                    config,
                    hide_docks=bool(p('hide_docks').value),
                    apply_view=bool(p('apply_view').value))
            self._start_rviz(config, self._want_software_gl(
                p('software_rendering').value, bool(p('use_xvfb').value)))

        # rviz2 のウィンドウが開くまで少し待つ
        time.sleep(2.0)

        self.capture = ScreenCapture(
            self.display, self.width, self.height, self.get_logger())
        # 注入器は F11 を送るのにも使うので allow_input に関係なく作る。
        # ブラウザからの操作を通すかどうかだけを allow_input で分ける。
        self._injector = InputInjector(self.display, self.get_logger())
        self.injector = self._injector if self.allow_input else None

        if self._rviz_started and bool(p('hide_chrome').value):
            self._hide_chrome(float(p('chrome_delay').value))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.image_pub = self.create_publisher(Image, p('image_topic').value, qos)
        self.create_subscription(
            String, TOPIC_RVIZ_INPUT, self._on_input,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        self._stop = threading.Event()
        self._frames = 0
        self._rate_t0 = time.monotonic()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name='rviz_capture', daemon=True)
        self._capture_thread.start()
        self.create_timer(10.0, self._report_rate)
        self._image_topic = p('image_topic').value
        self._dup_timer = self.create_timer(5.0, self._check_duplicates)
        self.get_logger().info(
            'rviz_bridge started: display=%s %dx%d @%.0ffps'
            % (self.display, self.width, self.height, self.fps))

    def _check_duplicates(self):
        """同じ画像トピックに publish している別のブリッジがいないか確かめる.

        二重起動していると 2 つの rviz2 の画面が交互に届いて映像がチカチカする。
        """
        self._dup_timer.cancel()
        try:
            count = self.count_publishers(self._image_topic)
        except Exception:  # noqa: BLE001
            return
        if count > 1:
            self.get_logger().error(
                '%s に publisher が %d 個あります。rviz_bridge が二重起動しています。\n'
                '    RViz の映像がチカチカする原因になります:\n'
                '    pkill -f rviz_bridge  （そのあと launch し直す）'
                % (self._image_topic, count))

    # ------------------------------------------------------------------ #
    @staticmethod
    def _display_in_use(display):
        """そのディスプレイ番号が既に使われているか（X のロックファイルで判定）."""
        try:
            num = int(display.lstrip(':').split('.')[0])
        except ValueError:
            return False
        return os.path.exists('/tmp/.X%d-lock' % num)

    def _pick_free_display(self):
        """指定のディスプレイが使用中なら、空いている番号を探す.

        前回の起動が残っていると Xvfb が
        "Server is already active for display 99" で落ち、
        そのまま前回の画面をキャプチャしてしまう（rviz2 が2つになる）。
        それを避けるため、空き番号へ自動でずらす。
        """
        if not self._display_in_use(self.display):
            return self.display
        try:
            base = int(self.display.lstrip(':').split('.')[0])
        except ValueError:
            base = 99
        for num in range(base + 1, base + 21):
            if not self._display_in_use(':%d' % num):
                picked = ':%d' % num
                self.get_logger().warn(
                    'ディスプレイ %s は使用中のため %s を使います'
                    '（前回の起動が残っている可能性があります: '
                    'pkill -f "Xvfb %s" && rm -f /tmp/.X%s-lock）'
                    % (self.display, picked, self.display,
                       self.display.lstrip(':')))
                return picked
        self.get_logger().error(
            '空いているディスプレイが見つかりません（%s 付近）' % self.display)
        return self.display

    def _start_xvfb(self):
        if not shutil.which('Xvfb'):
            self.get_logger().error(
                'Xvfb がありません: sudo apt install xvfb を実行してください')
            return
        self.display = self._pick_free_display()
        cmd = ['Xvfb', self.display, '-screen', '0',
               '%dx%dx24' % (self.width, self.height), '-nolisten', 'tcp']
        self.procs.append(subprocess.Popen(cmd))
        self.get_logger().info('Xvfb を起動: %s' % ' '.join(cmd))
        time.sleep(1.0)

    def _make_embedded_config(self, config, hide_docks=True, apply_view=True):
        """設定ファイルを一時コピーし、埋め込み向けに窓の形と初期視点を直す.

        Displays / Views のドックは UI パネルでは字が潰れて読めないので畳む。
        初期視点は Orbit カメラの Yaw / Pitch / Distance / Focal Point を
        パラメータの値で上書きする（既定は「アームの後ろから」）。
        表示内容（Displays の中身＝何を描くか）には一切手を触れない。
        元のファイルは書き換えず、一時ファイルにコピーしてから直す。
        """
        if not config or not os.path.isfile(config):
            return config
        try:
            import tempfile

            import yaml
            with open(config) as f:
                data = yaml.safe_load(f)

            if hide_docks:
                wg = data.setdefault('Window Geometry', {})
                wg['Hide Left Dock'] = True
                wg['Hide Right Dock'] = True
                wg['Width'] = self.width
                wg['Height'] = self.height
                wg['X'] = 0
                wg['Y'] = 0
                # 保存されたドックの配置(QMainWindow State)が残っていると
                # Hide 指定より優先されることがあるので落とす
                wg.pop('QMainWindow State', None)

            if apply_view:
                self._apply_view(data)

            tmp = tempfile.NamedTemporaryFile(
                mode='w', suffix='.rviz', prefix='piper_arm_ui_', delete=False)
            yaml.safe_dump(data, tmp, default_flow_style=False,
                           allow_unicode=True, sort_keys=True)
            tmp.close()
            self._tmp_config = tmp.name
            self.get_logger().info(
                '%s を埋め込み用に調整して使用: %s' % (os.path.basename(config), tmp.name))
            return tmp.name
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                'rviz 設定の調整に失敗したので元のまま使います (%s)' % exc)
            return config

    def _apply_view(self, data):
        """設定の Orbit カメラを、パラメータで指定した初期視点にする."""
        p = self.get_parameter
        try:
            current = data['Visualization Manager']['Views']['Current']
        except (KeyError, TypeError):
            self.get_logger().warn('rviz 設定に Views/Current が無いので視点は変更しません')
            return
        cls = str(current.get('Class', ''))
        if 'Orbit' not in cls:
            self.get_logger().warn(
                '視点の変更は Orbit カメラのみ対応です（現在: %s）。'
                'apply_view:=false にするか、rviz 設定の View を Orbit にしてください' % cls)
            return

        fp = list(p('view_focal_point').value)
        if len(fp) != 3:
            self.get_logger().warn('view_focal_point は [x, y, z] の3要素です。既定を使います')
            fp = [0.1, 0.0, 0.2]

        yaw = float(p('view_yaw').value)
        pitch = float(p('view_pitch').value)
        current['Yaw'] = yaw
        current['Pitch'] = pitch
        current['Distance'] = float(p('view_distance').value)
        current['Focal Point'] = {'X': float(fp[0]), 'Y': float(fp[1]), 'Z': float(fp[2])}
        # 起動直後にこの視点になるよう、保存済みの視点は消しておく
        data['Visualization Manager']['Views']['Saved'] = None

        self.get_logger().info(
            '初期視点: %s (yaw=%.3f pitch=%.3f dist=%.2f 注視点=%.2f,%.2f,%.2f)'
            % (self._view_name(yaw), yaw, pitch, float(p('view_distance').value), *fp))

    @staticmethod
    def _view_name(yaw):
        """yaw を人が読める向きの名前にする（ログ用）."""
        import math
        deg = math.degrees(yaw) % 360.0
        for center, name in ((0, '正面から'), (90, '左から'), (180, '後ろから'), (270, '右から')):
            if min(abs(deg - center), 360 - abs(deg - center)) <= 22.5:
                return name
        return '斜めから'

    def _hide_chrome(self, delay):
        """rviz2 を全画面にして、メニューバー・ツールバー・ステータスバーを消す.

        rviz_common/VisualizationFrame::setFullScreen() が
        menuBar / toolbar / statusBar と左右ドックの開閉ボタンをまとめて
        隠してくれるので、そこへ F11 を1回送るのがいちばん確実。
        WM の無い Xvfb では入力フォーカスが PointerRoot なので、
        先にポインタを画面中央（＝rviz のウィンドウ上）へ置く。
        """
        if self._injector is None or not self._injector.available:
            self.get_logger().warn(
                '入力注入の手段が無いため hide_chrome を適用できません'
                '（pip install python-xlib か apt install xdotool）。'
                'RViz のメニューバーが映り込みます')
            return
        # rviz2 のウィンドウが実際に出るまで待つ（起動直後は空の画面）
        time.sleep(max(0.0, delay))
        self._injector.move(self.width // 2, self.height // 2)
        if self._injector.key('F11'):
            self.get_logger().info(
                'rviz2 を全画面にしました（メニューバー等を非表示）。'
                '戻すには hide_chrome:=false')
        else:
            self.get_logger().warn('F11 を送れませんでした。RViz の文字が映り込みます')

    def _want_software_gl(self, setting, use_xvfb):
        """ソフトウェア OpenGL を強制するかを決める.

        'auto'（既定）なら Xvfb を使うときだけ強制する。Xvfb には GPU が無いので
        llvmpipe が要るが、実ディスプレイ(:0)では GPU が使えるので強制しない。
        """
        text = str(setting).strip().lower()
        if text in ('true', '1', 'yes'):
            return True
        if text in ('false', '0', 'no'):
            return False
        if text != 'auto':
            self.get_logger().warn(
                "software_rendering は auto/true/false のいずれかです（指定: %s）。"
                'auto として扱います' % setting)
        if not use_xvfb:
            self.get_logger().info(
                '実ディスプレイに描くので GPU 描画を使います'
                '（強制したいときは software_rendering:=true）')
        return bool(use_xvfb)

    def _start_rviz(self, config, software_rendering):
        env = dict(os.environ, DISPLAY=self.display)
        if software_rendering:
            # Xvfb 上では GPU が使えないのでソフトウェア OpenGL に固定する
            env['LIBGL_ALWAYS_SOFTWARE'] = '1'
            env['GALLIUM_DRIVER'] = 'llvmpipe'
        cmd = ['rviz2']
        if config:
            cmd += ['-d', config]
        try:
            self.procs.append(subprocess.Popen(cmd, env=env))
            self._rviz_started = True
            self.get_logger().info('rviz2 を起動: %s' % ' '.join(cmd))
        except FileNotFoundError:
            self.get_logger().error('rviz2 が見つかりません（source install/setup.bash 済みか確認）')

    # ------------------------------------------------------------------ #
    def _capture_loop(self):
        """専用スレッドで一定間隔にキャプチャして publish する.

        ROS のタイマーで回すと、キャプチャに時間がかかったぶんだけ
        ノード全体（視点操作の受け取りなど）が止まり、映像も間隔が乱れて
        カクつく。キャプチャは独立したスレッドに出しておく。
        """
        period = 1.0 / self.fps
        next_at = time.monotonic()
        while not self._stop.is_set():
            next_at += period
            try:
                frame = self.capture.grab()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('画面キャプチャに失敗: %s' % exc,
                                       throttle_duration_sec=5.0)
                frame = None

            if frame is not None:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'rviz'
                msg.height, msg.width = frame.shape[0], frame.shape[1]
                msg.encoding = 'bgr8'
                msg.is_bigendian = 0
                msg.step = msg.width * 3
                msg.data = frame.tobytes()
                self.image_pub.publish(msg)
                self._frames += 1

            # 間に合わなかったぶんは取り戻そうとせず、次の周期へ送る。
            # （取り戻そうとすると連続キャプチャになって余計に重くなる）
            sleep = next_at - time.monotonic()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_at = time.monotonic()

    def _report_rate(self):
        """実際に出せている fps を報告する（設定値どおり出ているかの確認用）."""
        now = time.monotonic()
        elapsed = now - self._rate_t0
        if elapsed <= 0:
            return
        actual = self._frames / elapsed
        self._frames = 0
        self._rate_t0 = now
        if actual < self.fps * 0.6:
            self.get_logger().warn(
                'RViz の配信が %.1f fps しか出ていません（設定 %.0f fps）。'
                'カクつく場合は解像度を下げてください: '
                'rviz_width:=848 rviz_height:=480' % (actual, self.fps),
                throttle_duration_sec=30.0)
        else:
            self.get_logger().info('RViz 配信 %.1f fps' % actual,
                                   throttle_duration_sec=60.0)

    def _on_input(self, msg: String):
        if self.injector is None or not self.injector.available:
            return
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        kind = ev.get('type')
        try:
            if kind == 'move':
                self.injector.move(ev['x'] * self.width, ev['y'] * self.height)
            elif kind in ('down', 'up'):
                # 押す前に座標を合わせてから押下する
                if 'x' in ev:
                    self.injector.move(ev['x'] * self.width, ev['y'] * self.height)
                self.injector.button(int(ev.get('button', 0)), kind == 'down')
            elif kind == 'wheel':
                if 'x' in ev:
                    self.injector.move(ev['x'] * self.width, ev['y'] * self.height)
                self.injector.wheel(float(ev.get('delta', 0.0)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('入力注入に失敗: %s' % exc, throttle_duration_sec=5.0)

    def destroy_node(self):
        # キャプチャスレッドを先に止める（止める前に capture.close() すると
        # 閉じた側を掴んだまま grab して例外が出る）
        stop = getattr(self, '_stop', None)
        if stop is not None:
            stop.set()
        thread = getattr(self, '_capture_thread', None)
        if thread is not None:
            thread.join(timeout=2.0)
        if self._tmp_config and os.path.isfile(self._tmp_config):
            try:
                os.unlink(self._tmp_config)
            except OSError:
                pass
        try:
            self.capture.close()
        except Exception:  # noqa: BLE001
            pass
        for proc in reversed(self.procs):
            try:
                proc.send_signal(signal.SIGTERM)
                proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RvizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
