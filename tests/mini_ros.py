"""ROS 抜きで piper_arm_ui のノードを動かすための最小スタブ.

本物の rclpy の代わりに import されて、ノードのコードを無改造で
instantiate できるようにする。以前の e2e ハーネス（mini_ros.py）は
コンテナが作り直されて失われたので、座標指定の検証に必要な範囲だけを
作り直したもの。

できること: パラメータ、publisher/subscriber の配線、タイマーの手動駆動。
できないこと: DDS、QoS の実際の挙動、別プロセス間通信。
"""
import sys
import threading
import time
import types


class _Logger:
    def __init__(self, name):
        self.name = name
        self.lines = []

    def _put(self, level, text):
        self.lines.append((level, str(text)))
        if level in ('error', 'fatal'):
            print('[%s] [%s] %s' % (level, self.name, text))

    def info(self, t, **_kw):    # throttle_duration_sec などは無視
        self._put('info', t)

    def warn(self, t, **_kw):    # throttle_duration_sec などは無視
        self._put('warn', t)

    def error(self, t, **_kw):    # throttle_duration_sec などは無視
        self._put('error', t)

    def fatal(self, t, **_kw):    # throttle_duration_sec などは無視
        self._put('fatal', t)

    def debug(self, t, **_kw):    # throttle_duration_sec などは無視
        self._put('debug', t)

    def text(self, level=None):
        return [t for lv, t in self.lines if level is None or lv == level]


class Bus:
    """トピックの配線。publish したら購読者をその場で呼ぶ."""

    def __init__(self):
        self.subs = {}
        self.sent = {}

    def publish(self, topic, msg):
        self.sent.setdefault(topic, []).append(msg)
        for cb in self.subs.get(topic, []):
            cb(msg)

    def subscribe(self, topic, cb):
        self.subs.setdefault(topic, []).append(cb)

    def last(self, topic):
        items = self.sent.get(topic, [])
        return items[-1] if items else None

    def clear(self):
        self.sent.clear()


BUS = Bus()


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.topic_name = topic

    def publish(self, msg):
        BUS.publish(self.topic, msg)


class _Timer:
    def __init__(self, period, cb):
        self.period = period
        self.cb = cb
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


# --- サービスの模擬 ----------------------------------------------------- #
# テストから {サービス名: handler(request) -> response} を仕込む。handler は
# 別スレッドで呼ばれる（本物と同じく、応答は呼び出し側のスレッドとは別に届く）。
# 仕込んでいない名前は「相手が居ない」扱い（wait_for_service が False）。
SERVICES = {}
SERVICE_CALLS = []        # (サービス名, request) の記録


class _Future:
    def __init__(self):
        self._done = threading.Event()
        self._result = None
        self._cbs = []
        self._lock = threading.Lock()

    def set_result(self, res):
        with self._lock:
            self._result = res
            self._done.set()
            cbs = list(self._cbs)
        for cb in cbs:
            cb(self)

    def add_done_callback(self, cb):
        with self._lock:
            if not self._done.is_set():
                self._cbs.append(cb)
                return
        cb(self)

    def done(self):
        return self._done.is_set()

    def result(self):
        return self._result


class _Client:
    def __init__(self, srv_type, name):
        self.srv_type = srv_type
        self.srv_name = name

    def wait_for_service(self, timeout_sec=None):
        if self.srv_name in SERVICES:
            return True
        if timeout_sec:
            time.sleep(min(timeout_sec, 0.05))   # 本物ほど待たない（テストを速く）
        return False

    def call_async(self, request):
        SERVICE_CALLS.append((self.srv_name, request))
        fut = _Future()
        handler = SERVICES[self.srv_name]
        threading.Thread(target=lambda: fut.set_result(handler(request)),
                         daemon=True).start()
        return fut


class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = _Logger(name)
        self.timers = []

    # --- パラメータ ---
    def declare_parameter(self, name, default):
        self._params.setdefault(name, PARAM_OVERRIDES.get(self._name, {}).get(
            name, default))

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params.get(name))

    def set_parameters(self, *_a, **_k):      # pragma: no cover
        pass

    # --- 通信 ---
    def create_publisher(self, _type, topic, _qos=None, **_kw):
        return _Publisher(topic)

    def create_subscription(self, _type, topic, cb, _qos=None, **_kw):
        BUS.subscribe(topic, cb)
        return types.SimpleNamespace(topic=topic)

    def create_client(self, srv_type, name, **_kw):
        return _Client(srv_type, name)

    def create_timer(self, period, cb, **_kw):
        t = _Timer(period, cb)
        self.timers.append(t)
        return t

    # テストから上書きできるようにしておく {topic: 個数}
    publisher_counts = {}

    def count_publishers(self, topic):
        return self.publisher_counts.get(topic, 1)

    def get_logger(self):
        return self._logger

    def get_name(self):
        return self._name

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass

    # --- テスト用 ---
    def tick(self, which=0, times=1):
        """指定のタイマーを手で回す（0=制御ループ, 1=状態配信）."""
        for _ in range(times):
            if not self.timers[which].cancelled:
                self.timers[which].cb()


PARAM_OVERRIDES = {}


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _Trigger:
    """std_srvs/Trigger もどき."""
    class Request(_Msg):
        pass

    class Response(_Msg):
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message


class Bool(_Msg):
    def __init__(self, data=False):
        self.data = data


class String(_Msg):
    def __init__(self, data=''):
        self.data = data


class JointState(_Msg):
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class PointStamped(_Msg):
    def __init__(self, frame_id='', x=0.0, y=0.0, z=0.0):
        self.header = types.SimpleNamespace(stamp=None, frame_id=frame_id)
        self.point = types.SimpleNamespace(x=x, y=y, z=z)


def install():
    """rclpy とメッセージ型をスタブに差し替える."""
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.spin = lambda *a, **k: None

    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = Node

    qos_mod = types.ModuleType('rclpy.qos')

    class QoSProfile:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    for cls in ('ReliabilityPolicy', 'HistoryPolicy', 'DurabilityPolicy'):
        setattr(qos_mod, cls, types.SimpleNamespace(
            RELIABLE=1, BEST_EFFORT=2, KEEP_LAST=1, VOLATILE=1, TRANSIENT_LOCAL=2))
    qos_mod.QoSProfile = QoSProfile

    rclpy.node = node_mod
    rclpy.qos = qos_mod

    cbg_mod = types.ModuleType('rclpy.callback_groups')
    cbg_mod.ReentrantCallbackGroup = lambda: types.SimpleNamespace(kind='reentrant')
    cbg_mod.MutuallyExclusiveCallbackGroup = lambda: types.SimpleNamespace(kind='exclusive')
    rclpy.callback_groups = cbg_mod

    ex_mod = types.ModuleType('rclpy.executors')

    class MultiThreadedExecutor:            # main() でしか使わない
        def __init__(self, *a, **k):
            pass
    ex_mod.MultiThreadedExecutor = MultiThreadedExecutor
    rclpy.executors = ex_mod

    std_srvs = types.ModuleType('std_srvs')
    std_srvs_srv = types.ModuleType('std_srvs.srv')
    std_srvs_srv.Trigger = _Trigger
    std_srvs.srv = std_srvs_srv

    sensor = types.ModuleType('sensor_msgs')
    sensor_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msg.JointState = JointState
    sensor_msg.Image = _Msg
    sensor_msg.CompressedImage = _Msg
    sensor_msg.CameraInfo = _Msg
    sensor.msg = sensor_msg

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = String
    std_msg.Bool = Bool
    std.msg = std_msg

    geometry = types.ModuleType('geometry_msgs')
    geometry_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msg.Pose = _Msg
    geometry_msg.PointStamped = PointStamped
    geometry.msg = geometry_msg

    for name, mod in (
            ('rclpy', rclpy), ('rclpy.node', node_mod), ('rclpy.qos', qos_mod),
            ('rclpy.callback_groups', cbg_mod), ('rclpy.executors', ex_mod),
            ('std_srvs', std_srvs), ('std_srvs.srv', std_srvs_srv),
            ('sensor_msgs', sensor), ('sensor_msgs.msg', sensor_msg),
            ('std_msgs', std), ('std_msgs.msg', std_msg),
            ('geometry_msgs', geometry), ('geometry_msgs.msg', geometry_msg)):
        sys.modules[name] = mod
