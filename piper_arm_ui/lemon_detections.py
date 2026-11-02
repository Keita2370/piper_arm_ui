#!/usr/bin/env python3
"""検出アダプタ（段階2）.

チームの lemon_ellipse_node が出す /lemon_ellipse/lemons（lemon_msgs/LemonEllipseArray）を、
UI が使う /arm_ui/detections（JSON）に直す。dummy_perception と同じ形なので、
ブラウザも harvest_sequencer も無改造で検出枠を描ける。

さらに lemon_target が固定した対象（/lemon_target/target）に一番近い検出に
"locked": true を付けて、画面で「相手が狙っているのはどれか」が分かるようにする。

【読むだけ】サービスは呼ばない。相手のトピックにも publish しない。

【座標】
- center_u / center_v / major_radius / minor_radius は画像の px。UI は 0〜1 に正規化した
  値を使うので camera_info の幅・高さで割る。
- 楕円は minor_angle だけ回っているので、枠は回った楕円の外接矩形にする
  （半径のまま縦横に使うと、斜めのレモンで枠が小さすぎる／大きすぎる）。
- distance は center.z（カメラの光学座標系の奥行き）。検出は「見えている表面」の深度なので、
  レモンの中心より 1 半径ほど手前になる。

【TF は対象が固定されている間だけ】
Python の TF リスナーは /tf を全部受け取る（実機で約100Hz）。ミニPCは CPU に余裕が無く、
UI のぶんの CPU が増えると YOLO が遅くなって lemon_target が対象を選べなくなる。
印を付けるのに TF が要るのは対象が固定されている間だけなので、そのときだけ購読する。

lemon_msgs は package.xml の依存にしていない。開発用PC（~/lemon_ws が無い）でも
piper_arm_ui をビルドできるようにするため。無ければ起動時に理由を出して終わる。
"""

import json
import math
import time

from piper_arm_ui.common import TOPIC_DETECTIONS, TOPIC_TEAM_TARGET


def ellipse_box(major, minor, minor_angle):
    """回った楕円の外接矩形の半幅・半高 [px].

    major / minor は半径（半軸）。minor_angle は短辺が画像の横軸となす角
    （lemon_ellipse_node の定義）。長辺はそれと直交する。
    """
    s = math.sin(minor_angle)
    c = math.cos(minor_angle)
    # 長辺の向き = 短辺 + 90°。cos(m+90°) = -sin m, sin(m+90°) = cos m
    half_w = math.sqrt((major * s) ** 2 + (minor * c) ** 2)
    half_h = math.sqrt((major * c) ** 2 + (minor * s) ** 2)
    return half_w, half_h


def convert(lemons, width, height):
    """LemonEllipse の並び → UI の targets（dict の list）.

    id は YOLO の tracking ID。実機は use_tracking:=False で空なので、そのときは
    その回の並び順で 1 から振る（フレームをまたいで同じ番号になる保証は無い）。
    """
    out = []
    for i, lemon in enumerate(lemons):
        half_w, half_h = ellipse_box(
            float(lemon.major_radius), float(lemon.minor_radius), float(lemon.minor_angle))
        raw_id = str(getattr(lemon, 'id', '') or '').strip()
        out.append({
            'id': raw_id if raw_id else i + 1,
            'u': round(float(lemon.center_u) / width, 4),
            'v': round(float(lemon.center_v) / height, 4),
            'w': round(2.0 * half_w / width, 4),
            'h': round(2.0 * half_h / height, 4),
            'distance': round(float(lemon.center.z), 3),
            'confidence': round(float(lemon.score), 3),
            'label': str(getattr(lemon, 'class_name', '') or 'lemon'),
            'locked': False,
        })
    return out


def mark_locked(targets, lemons, target_in_camera, match_radius):
    """カメラ座標系に直した対象の位置に一番近い検出に locked を付ける.

    match_radius [m] より遠ければ何も付けない（見えている別のレモンを
    対象と取り違えるほうが、印が無いより悪い）。付けた index を返す。
    """
    if target_in_camera is None:
        return None
    tx, ty, tz = target_in_camera
    best, best_d = None, None
    for i, lemon in enumerate(lemons):
        c = lemon.center
        d = math.sqrt((c.x - tx) ** 2 + (c.y - ty) ** 2 + (c.z - tz) ** 2)
        if best_d is None or d < best_d:
            best, best_d = i, d
    if best is None or best_d > match_radius:
        return None
    targets[best]['locked'] = True
    return best


def _make_node():
    """rclpy を使う部分。テストでは convert / mark_locked だけを使う."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String

    try:
        from lemon_msgs.msg import LemonEllipseArray
    except ImportError as exc:
        raise RuntimeError(
            'lemon_msgs が見つかりません（~/lemon_ws を source してください）: %s' % exc)

    class LemonDetections(Node):

        def __init__(self):
            super().__init__('lemon_detections')
            self.declare_parameter('lemons_topic', '/lemon_ellipse/lemons')
            self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
            self.declare_parameter('target_topic', TOPIC_TEAM_TARGET)
            # 対象と検出を同じものとみなす距離 [m]。lemon_target の track_radius（0.04）より
            # 少し広め。検出は表面の深度なので、ぴったりは一致しない
            self.declare_parameter('match_radius', 0.05)
            p = self.get_parameter
            self.match_radius = float(p('match_radius').value)

            self.width = None
            self.height = None
            self.target = None              # PointStamped（frame_id が空なら対象なし）
            self.tf_buffer = None
            self.tf_listener = None

            self.pub = self.create_publisher(String, TOPIC_DETECTIONS, 10)
            self.create_subscription(
                CameraInfo, p('camera_info_topic').value, self._on_info, 10)
            self.create_subscription(
                LemonEllipseArray, p('lemons_topic').value, self._on_lemons, 10)
            latched = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(
                PointStamped, p('target_topic').value, self._on_target, latched)
            self.get_logger().info(
                '%s -> %s（対象の印: %s）'
                % (p('lemons_topic').value, TOPIC_DETECTIONS, p('target_topic').value))

        def _on_info(self, msg):
            if msg.width and msg.height:
                self.width, self.height = int(msg.width), int(msg.height)

        def _on_target(self, msg):
            self.target = msg if msg.header.frame_id else None
            self._set_tf(self.target is not None)

        def _set_tf(self, on):
            """TF の購読を、対象が固定されている間だけにする（CPU を食うため）."""
            if on and self.tf_listener is None:
                from tf2_ros import Buffer, TransformListener
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            elif not on and self.tf_listener is not None:
                try:
                    self.tf_listener.unregister()
                except Exception:  # noqa: BLE001
                    pass
                self.tf_listener = None
                self.tf_buffer = None

        def _target_in(self, frame_id):
            """対象の位置を検出の座標系（カメラの光学座標系）に直す。直せなければ None."""
            if self.target is None or self.tf_buffer is None:
                return None
            try:
                from rclpy.time import Time
                import tf2_geometry_msgs  # noqa: F401  PointStamped の変換を登録する
                pt = PointStamped()
                pt.header.frame_id = self.target.header.frame_id
                pt.header.stamp = Time().to_msg()       # 最新の TF を使う
                pt.point = self.target.point
                out = self.tf_buffer.transform(pt, frame_id)
                return (out.point.x, out.point.y, out.point.z)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('対象の座標を変換できません: %s' % exc,
                                       throttle_duration_sec=5.0)
                return None

        def _on_lemons(self, msg):
            if not self.width:
                self.get_logger().warn(
                    'camera_info がまだ来ていないので検出を送れません',
                    throttle_duration_sec=5.0)
                return
            targets = convert(msg.lemons, self.width, self.height)
            mark_locked(targets, msg.lemons,
                        self._target_in(msg.header.frame_id), self.match_radius)
            out = String()
            out.data = json.dumps(
                {'targets': targets, 'source': 'lemon_ellipse', 'stamp': time.time()},
                ensure_ascii=False)
            self.pub.publish(out)

    return rclpy, LemonDetections


def main(args=None):
    rclpy, LemonDetections = _make_node()
    rclpy.init(args=args)
    node = LemonDetections()
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
