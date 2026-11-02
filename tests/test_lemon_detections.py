"""検出アダプタ（段階2）の検証.

守りたいこと:

    /lemon_ellipse/lemons（px の楕円）を、UI の /arm_ui/detections（0〜1 の枠）に
    取り違えずに直す。lemon_target が固定した対象に一番近い検出に印を付ける。
    TF は対象が固定されている間だけ購読する（ミニPCの CPU に余裕が無い）。

特に:
  - 楕円の半径（半軸）を、回転を考えた外接矩形の全幅・全高に直す
  - px を camera_info の幅・高さで割る
  - id が空（実機は use_tracking:=False）なら並び順で振る
  - 対象から match_radius より遠い検出には印を付けない（取り違えるより無いほうがよい）

ROS も lemon_msgs も無しで走る（lemon_msgs と tf2_ros はここで模擬する）。
"""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

from piper_arm_ui.common import TOPIC_DETECTIONS, TOPIC_TEAM_TARGET  # noqa: E402
from piper_arm_ui.lemon_detections import convert, ellipse_box, mark_locked  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def lemon(u, v, major, minor, angle=0.0, xyz=(0.0, 0.0, 0.4), score=0.9, id_=''):
    """LemonEllipse もどき（使う欄だけ）."""
    return types.SimpleNamespace(
        class_name='lemon', score=score, id=id_,
        center=types.SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
        center_u=u, center_v=v, major_radius=major, minor_radius=minor, minor_angle=angle)


# --------------------------------------------------------------------- #
# 1. 楕円 → 外接矩形
# --------------------------------------------------------------------- #
print('[楕円の外接矩形]')
hw, hh = ellipse_box(40, 20, 0.0)
check('短辺が横向き（角度0）なら、半幅=短辺・半高=長辺', (hw, hh) == (20, 40), (hw, hh))
hw, hh = ellipse_box(40, 20, math.pi / 2)
check('短辺が縦向き（90°）なら、半幅=長辺・半高=短辺',
      abs(hw - 40) < 1e-9 and abs(hh - 20) < 1e-9, (hw, hh))
hw, hh = ellipse_box(40, 20, math.pi / 4)
expect = math.sqrt((40 ** 2 + 20 ** 2) / 2)
check('45°なら半幅も半高も sqrt((a²+b²)/2)',
      abs(hw - expect) < 1e-9 and abs(hh - expect) < 1e-9, (hw, hh, expect))
# 数値で外接矩形を確かめる（楕円上の点を回して最大値を取る）
a, b, m = 50.0, 18.0, 0.6
xs, ys = [], []
for k in range(3600):
    t = 2 * math.pi * k / 3600
    # 短辺の向き (cos m, sin m)、長辺の向き (-sin m, cos m)
    x = b * math.cos(t) * math.cos(m) + a * math.sin(t) * (-math.sin(m))
    y = b * math.cos(t) * math.sin(m) + a * math.sin(t) * math.cos(m)
    xs.append(x)
    ys.append(y)
hw, hh = ellipse_box(a, b, m)
check('任意の角度で、楕円上の点の最大値と一致する',
      abs(hw - max(xs)) < 0.01 and abs(hh - max(ys)) < 0.01,
      (round(hw, 3), round(max(xs), 3), round(hh, 3), round(max(ys), 3)))

# --------------------------------------------------------------------- #
# 2. px → 0〜1
# --------------------------------------------------------------------- #
print('[正規化]')
t = convert([lemon(320, 240, 48, 32, 0.0, xyz=(0.01, 0.02, 0.36), score=0.96)], 640, 480)[0]
check('中心は幅・高さで割る', (t['u'], t['v']) == (0.5, 0.5), (t['u'], t['v']))
check('枠の全幅 = 2×半幅 / 幅', t['w'] == round(2 * 32 / 640, 4), t['w'])
check('枠の全高 = 2×半高 / 高さ', t['h'] == round(2 * 48 / 480, 4), t['h'])
check('distance は center.z（光学座標系の奥行き）', t['distance'] == 0.36, t['distance'])
check('confidence は score', t['confidence'] == 0.96, t['confidence'])
check('印は既定で付けない', t['locked'] is False)
t = convert([lemon(320, 240, 48, 32, 0.0, xyz=(0.01, 0.02, 0.36))], 640, 480,
            'camera_color_optical_frame')[0]
check('3D 位置を載せる（select に渡すため）', t['xyz'] == [0.01, 0.02, 0.36], t['xyz'])
check('3D 位置の座標系を載せる', t['frame_id'] == 'camera_color_optical_frame', t['frame_id'])

ts = convert([lemon(100, 100, 10, 10), lemon(200, 200, 10, 10)], 640, 480)
check('id が空なら並び順で 1 から振る', [x['id'] for x in ts] == [1, 2], [x['id'] for x in ts])
ts = convert([lemon(100, 100, 10, 10, id_='7')], 640, 480)
check('tracking ID があればそれを使う', ts[0]['id'] == '7', ts[0]['id'])
check('検出が無ければ空の並び', convert([], 640, 480) == [])

# --------------------------------------------------------------------- #
# 3. 対象の印
# --------------------------------------------------------------------- #
print('[対象の印]')
ls = [lemon(100, 100, 10, 10, xyz=(0.10, 0.0, 0.40)),
      lemon(300, 100, 10, 10, xyz=(-0.05, 0.0, 0.36)),
      lemon(500, 100, 10, 10, xyz=(-0.20, 0.0, 0.50))]
ts = convert(ls, 640, 480)
i = mark_locked(ts, ls, (-0.04, 0.01, 0.38), 0.05)
check('対象に一番近い検出に印を付ける', i == 1 and ts[1]['locked'], i)
check('印は1つだけ', sum(x['locked'] for x in ts) == 1)
ts = convert(ls, 640, 480)
check('match_radius より遠ければ印を付けない',
      mark_locked(ts, ls, (0.0, 0.3, 0.4), 0.05) is None and not any(x['locked'] for x in ts))
ts = convert(ls, 640, 480)
check('対象が無ければ何もしない', mark_locked(ts, ls, None, 0.05) is None)

# --------------------------------------------------------------------- #
# 4. ノード（lemon_msgs と TF を模擬）
# --------------------------------------------------------------------- #
print('[ノード]')
lm = types.ModuleType('lemon_msgs')
lm_msg = types.ModuleType('lemon_msgs.msg')
lm_msg.LemonEllipseArray = mini_ros._Msg
lm.msg = lm_msg
sys.modules['lemon_msgs'] = lm
sys.modules['lemon_msgs.msg'] = lm_msg

tf_log = {'created': 0, 'unregistered': 0}


class FakeListener:
    def __init__(self, _buffer, _node):
        tf_log['created'] += 1

    def unregister(self):
        tf_log['unregistered'] += 1


tf2 = types.ModuleType('tf2_ros')
tf2.Buffer = lambda: object()
tf2.TransformListener = FakeListener
sys.modules['tf2_ros'] = tf2

mini_ros.BUS.clear()
mini_ros.BUS.subs.clear()
from piper_arm_ui.lemon_detections import _make_node  # noqa: E402

_, LemonDetections = _make_node()
node = LemonDetections()


def lemons_msg(items):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='camera_color_optical_frame'), lemons=items)


def last_out():
    m = mini_ros.BUS.last(TOPIC_DETECTIONS)
    return json.loads(m.data) if m else None


mini_ros.BUS.publish('/lemon_ellipse/lemons', lemons_msg(ls))
check('camera_info が来るまでは送らない（幅が分からない）', last_out() is None, last_out())

mini_ros.BUS.publish('/camera/camera/color/camera_info',
                     types.SimpleNamespace(width=640, height=480))
mini_ros.BUS.publish('/lemon_ellipse/lemons', lemons_msg(ls))
out = last_out()
check('/arm_ui/detections に targets を出す', out and len(out['targets']) == 3, out)
check('出どころを書く', out and out.get('source') == 'lemon_ellipse')
check('対象が無い間は TF を購読しない', tf_log['created'] == 0, tf_log)
check('対象が無い間は印が無い', out and not any(x['locked'] for x in out['targets']))

# 対象が固定された（変換は模擬: カメラ座標系で 2 番目のレモンの位置）
node._target_in = lambda _frame: (-0.05, 0.0, 0.37)
mini_ros.BUS.publish(TOPIC_TEAM_TARGET, mini_ros.PointStamped('base_link', 0.39, 0.02, 0.06))
check('対象が固定されたら TF を購読し始める', tf_log['created'] == 1, tf_log)
mini_ros.BUS.publish('/lemon_ellipse/lemons', lemons_msg(ls))
out = last_out()
check('固定された対象に印が付く',
      out and [x['locked'] for x in out['targets']] == [False, True, False],
      out and [x['locked'] for x in out['targets']])

mini_ros.BUS.publish(TOPIC_TEAM_TARGET, mini_ros.PointStamped('base_link', 0.39, 0.02, 0.06))
check('固定が続いても TF のリスナーを作り直さない', tf_log['created'] == 1, tf_log)

# 解除（frame_id が空）
mini_ros.BUS.publish(TOPIC_TEAM_TARGET, mini_ros.PointStamped('', 0, 0, 0))
check('対象が解除されたら TF の購読をやめる', tf_log['unregistered'] == 1, tf_log)

pubs = [v.topic for v in vars(node).values() if isinstance(v, mini_ros._Publisher)]
check('publish するのは /arm_ui/detections だけ', pubs == [TOPIC_DETECTIONS], pubs)

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
