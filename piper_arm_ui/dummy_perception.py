#!/usr/bin/env python3
"""レモン検出結果のダミー配信ノード.

カメラが未定のため、UI のレイアウト・操作フローを確認できるように
検出結果だけを模擬的に流す。実装が決まったら、このノードを
実際の認識ノード（YOLO 等）に置き換え、同じ JSON を
/arm_ui/detections に publish すれば UI 側の変更は不要。

JSON 形式:
  {"targets": [
      {"id": 1, "u": 0.31, "v": 0.44,          # 画像内の正規化中心座標
       "w": 0.12, "h": 0.16,                    # 正規化バウンディングボックス
       "distance": 0.38, "confidence": 0.92}    # 距離[m], 信頼度
  ]}
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from piper_arm_ui.common import TOPIC_DETECTIONS


class DummyPerception(Node):

    def __init__(self):
        super().__init__('dummy_perception')
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('num_targets', 3)

        self.num_targets = int(self.get_parameter('num_targets').value)
        rate = max(0.2, float(self.get_parameter('rate').value))

        self.pub = self.create_publisher(String, TOPIC_DETECTIONS, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.t0 = time.monotonic()
        self._dup_timer = self.create_timer(5.0, self._check_duplicates)
        self.get_logger().info('dummy_perception started (%d targets)' % self.num_targets)

    def _check_duplicates(self):
        """検出トピックに publish しているノードが複数いないか確かめる.

        二重起動していると 2 組の検出枠が交互に届いて、枠が飛び回る。
        実際の認識ノードに置き換えたあと、ダミーを止め忘れた場合もここで気づける。
        """
        self._dup_timer.cancel()
        try:
            count = self.count_publishers(TOPIC_DETECTIONS)
        except Exception:  # noqa: BLE001
            return
        if count > 1:
            self.get_logger().error(
                '%s に publisher が %d 個あります。検出ノードが二重に動いています。\n'
                '    検出枠が飛び回る原因になります。実際の認識ノードを使うなら\n'
                '    use_dummy_perception:=false でダミーを止めてください'
                % (TOPIC_DETECTIONS, count))

    def _tick(self):
        t = time.monotonic() - self.t0
        targets = []
        for i in range(self.num_targets):
            phase = t * 0.35 + i * 2.1
            targets.append({
                'id': i + 1,
                'u': round(0.25 + 0.22 * i + 0.02 * math.sin(phase), 4),
                'v': round(0.45 + 0.10 * math.cos(phase * 0.8), 4),
                'w': 0.13,
                'h': 0.17,
                'distance': round(0.32 + 0.06 * i + 0.01 * math.sin(phase), 3),
                'confidence': round(0.82 + 0.10 * abs(math.sin(phase * 0.5)), 3),
                'label': 'lemon',
            })
        msg = String()
        msg.data = json.dumps({'targets': targets}, ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPerception()
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
