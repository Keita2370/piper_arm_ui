"""backend:=lemon（段階3: サービス呼び出しの土台）の検証.

守りたいこと:

    UI のボタンがチームのサービスを正しく呼び、その間も harvest_sequencer が
    固まらない（状態配信と 3D ビューが止まらない）。アームが動くものは呼ばない。

特に:
  - サービスの結果を待っている間も、コマンドのコールバックはすぐ返り、状態配信が動く
    （計画書 §3。ここを間違えると「ボタンを押すと画面が固まる」）
  - 相手が居ない・応答しないときに、固まらずに理由を出す
  - 同じサービスを二重に呼ばない
  - 「UI操作を有効化」が OFF の間は呼ばない
  - lemon では /joint_states の publisher を作らず、アームが動くコマンドは受け付けない

サービスは mini_ros の SERVICES で模擬する（応答は別スレッドで届く）。
"""
import json
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

# lemon_msgs/srv/SelectTarget もどき（~/lemon_ws が無くても走るように）
lm = types.ModuleType('lemon_msgs')
lm_srv = types.ModuleType('lemon_msgs.srv')


class SelectTarget:
    class Request:
        def __init__(self):
            self.point = mini_ros.PointStamped()

    class Response(mini_ros._Msg):
        pass


lm_srv.SelectTarget = SelectTarget
lm.srv = lm_srv
sys.modules['lemon_msgs'] = lm
sys.modules['lemon_msgs.srv'] = lm_srv

# action_msgs/srv/CancelGoal もどき（空の要求＝全部のゴールを取り消す）
am = types.ModuleType('action_msgs')
am_srv = types.ModuleType('action_msgs.srv')


class CancelGoal:
    class Request(mini_ros._Msg):
        pass

    class Response(mini_ros._Msg):
        def __init__(self, return_code=0, goals_canceling=()):
            self.return_code = return_code
            self.goals_canceling = list(goals_canceling)


am_srv.CancelGoal = CancelGoal
am.srv = am_srv
sys.modules['action_msgs'] = am
sys.modules['action_msgs.srv'] = am_srv

from piper_arm_ui.common import TOPIC_DETECTIONS, TOPIC_UI_COMMAND, TOPIC_UI_LOG, TOPIC_UI_STATE  # noqa: E402,E501

FAIL = []
Resp = mini_ros._Trigger.Response


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def make(**params):
    mini_ros.BUS.clear()
    mini_ros.BUS.subs.clear()
    mini_ros.SERVICES.clear()
    del mini_ros.SERVICE_CALLS[:]
    mini_ros.Node.publisher_counts = {'/joint_states': 2}
    mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = dict(
        {'backend': 'lemon', 'stub_step_duration': 0.0}, **params)
    from piper_arm_ui.harvest_sequencer import HarvestSequencer
    node = HarvestSequencer()
    node.tick(2)         # 二重起動・横取りの確認
    return node


def send(obj):
    t0 = time.monotonic()
    mini_ros.BUS.publish(TOPIC_UI_COMMAND, mini_ros.String(json.dumps(obj)))
    return time.monotonic() - t0


def logs():
    return [json.loads(m.data)['text'] for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])]


def wait_log(fragment, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        hit = [t for t in logs() if fragment in t]
        if hit:
            return hit[-1]
        time.sleep(0.01)
    return None


def state(node):
    node.tick(1)
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)


def detections(items):
    mini_ros.BUS.publish(TOPIC_DETECTIONS, mini_ros.String(json.dumps({'targets': items})))


# --------------------------------------------------------------------- #
print('[lemon では /joint_states に触らない]')
node = make(enable_arm_command=True)
check('/joint_states の publisher を作らない', node.joint_pub is None)
check('enable_arm_command を付けても関節指令は無効', node.enable_arm_command is False)
node.tick(0, times=5)
check('制御ループを回しても /joint_states に出さない',
      '/joint_states' not in mini_ros.BUS.sent, list(mini_ros.BUS.sent))
st = state(node)
check('状態に backend が載る', st.get('backend') == 'lemon', st.get('backend'))
check('「UI操作」は既定で OFF', st['enable_arm_command'] is False)
check('横取りの警告を出さない（publisher が無いので比べるものが無い）',
      st['command_topic_taken'] is False)

print('[元栓が OFF なら、アームが動くコマンドはどれも呼ばない]')
MOVING = ('plan_grasp', 'grasp', 'home', 'release', 'goto_basket', 'jog',
          'gripper', 'move_to_xyz', 'jog_xyz')
for name in MOVING:
    node._last_jog_at = None       # 続けざまの入力の歯止めとは別に、元栓だけを見る
    send({'cmd': name, 'index': 1, 'joint': 0, 'delta': 0.1, 'x': 0.2, 'y': 0, 'z': 0.3})
check('どれも内部の状態機械を動かさない', node.state == 'IDLE' and node.trajectory is None,
      node.state)
check('どれもサービスを呼ばない', not mini_ros.SERVICE_CALLS, mini_ros.SERVICE_CALLS)
check('呼ばない理由を出す（記録の無い収穫位置は「記録されていません」）',
      sum('OFF なので' in t or 'まだ記録されていません' in t for t in logs()) == len(MOVING),
      sum('OFF なので' in t or 'まだ記録されていません' in t for t in logs()))
send({'cmd': 'some_unknown_command'})
check('知らないコマンドは理由を出して何もしない',
      any('lemon 構成では「some_unknown_command」' in t for t in logs()))

# --------------------------------------------------------------------- #
print('[UI操作の元栓]')
mini_ros.SERVICES['/lemon_target/clear_exclusions'] = lambda _r: Resp(True, 'Cleared 3 exclusions')
send({'cmd': 'clear_exclusions'})
time.sleep(0.2)
check('OFF の間は呼ばない', not mini_ros.SERVICE_CALLS, mini_ros.SERVICE_CALLS)
check('OFF なので呼ばないと出す', any('OFF なので' in t for t in logs()))
send({'cmd': 'set_enable', 'value': True})
check('ON にできる（lemon では横取りの安全弁に止められない）',
      state(node)['enable_arm_command'] is True)

print('[除外をクリア]')
send({'cmd': 'clear_exclusions'})
got = wait_log('Cleared 3 exclusions')
check('/lemon_target/clear_exclusions を呼び、結果を出す', got is not None, got)
check('呼んだのは clear_exclusions だけ',
      [n for n, _ in mini_ros.SERVICE_CALLS] == ['/lemon_target/clear_exclusions'],
      [n for n, _ in mini_ros.SERVICE_CALLS])

# --------------------------------------------------------------------- #
print('[対象の選択]')
seen = {}


def on_select(req):
    seen['frame'] = req.point.header.frame_id
    seen['xyz'] = (req.point.point.x, req.point.point.y, req.point.point.z)
    return Resp(True, 'Locked (0.360, 0.055, 0.035)')


mini_ros.SERVICES['/lemon_target/select'] = on_select
detections([
    {'id': 1, 'u': 0.3, 'v': 0.5, 'w': 0.1, 'h': 0.1, 'distance': 0.34,
     'xyz': [-0.027, 0.035, 0.341], 'frame_id': 'camera_color_optical_frame'},
    {'id': 2, 'u': 0.6, 'v': 0.5, 'w': 0.1, 'h': 0.1, 'distance': 0.4},
])
send({'cmd': 'select_target', 'id': 1})
got = wait_log('Locked')
check('/lemon_target/select を呼び、結果を出す', got is not None, got)
check('検出の座標系をそのまま渡す', seen.get('frame') == 'camera_color_optical_frame', seen)
check('検出の 3D 位置をそのまま渡す', seen.get('xyz') == (-0.027, 0.035, 0.341), seen)
n_calls = len(mini_ros.SERVICE_CALLS)
send({'cmd': 'select_target', 'id': 2})
check('位置が載っていない検出は呼ばずに理由を出す',
      len(mini_ros.SERVICE_CALLS) == n_calls and any('位置が載っていません' in t for t in logs()))
send({'cmd': 'select_target', 'id': 9})
check('無い ID は呼ばずに理由を出す',
      len(mini_ros.SERVICE_CALLS) == n_calls and any('ID 9 は' in t for t in logs()))

mini_ros.SERVICES['/lemon_target/select'] = lambda _r: Resp(False, 'no stable lemon within 0.05 m')
send({'cmd': 'select_target', 'id': 1})
got = wait_log('no stable lemon within')
check('断られたら相手の理由をそのまま出す', got is not None, got)
check('断られたログは warn',
      any(json.loads(m.data)['level'] == 'warn' and 'no stable' in json.loads(m.data)['text']
          for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])))

# --------------------------------------------------------------------- #
print('[固まらない（計画書 §3 の本題）]')


def slow(_r):
    time.sleep(1.5)
    return Resp(True, 'Cleared 0 exclusions (slow)')


mini_ros.SERVICES['/lemon_target/clear_exclusions'] = slow
dt = send({'cmd': 'clear_exclusions'})
check('コマンドのコールバックはすぐ返る（結果を待たない）', dt < 0.1, '%.3f 秒' % dt)
t0 = time.monotonic()
st = state(node)
node.tick(0, times=10)
dt = time.monotonic() - t0
check('待っている間も状態配信と制御ループが回る', dt < 0.2, '%.3f 秒' % dt)
check('呼び出し中は busy と出る（ボタンを押せなくする）',
      st['calls']['clear_exclusions']['busy'] is True, st['calls'])
send({'cmd': 'clear_exclusions'})
check('呼び出し中に押しても二重に呼ばない',
      wait_log('前の呼び出しがまだ終わっていません', 1.0) is not None)
got = wait_log('(slow)', 5.0)
check('遅くても結果は届く', got is not None, got)
time.sleep(0.05)
check('終わったら busy が消える', state(node)['calls']['clear_exclusions']['busy'] is False)

print('[相手が居ない・応答しない]')
del mini_ros.SERVICES['/lemon_target/clear_exclusions']
t0 = time.monotonic()
send({'cmd': 'clear_exclusions'})
got = wait_log('見つかりません', 5.0)
check('相手が居なければ固まらずに理由を出す', got is not None, got)
check('すぐに分かる（数秒以内）', time.monotonic() - t0 < 3.0, '%.2f 秒' % (time.monotonic() - t0))

mini_ros.SERVICES['/lemon_target/clear_exclusions'] = lambda _r: (time.sleep(3.0), Resp(True, 'late'))[1]
node.callers['clear_exclusions'].timeout = 0.3
send({'cmd': 'clear_exclusions'})
got = wait_log('応答しません', 3.0)
check('応答が無ければタイムアウトして理由を出す', got is not None, got)
check('タイムアウトのあとは busy が消える（次を押せる）',
      state(node)['calls']['clear_exclusions']['busy'] is False)

# --------------------------------------------------------------------- #
# 段階4: home / 開放 / 対象を解除 / 停止
# --------------------------------------------------------------------- #
def fresh_lemon():
    n = make()
    n.tick(3)        # 相手が居るかの確認（manager は居ない）
    return n


def names():
    return [n for n, _ in mini_ros.SERVICE_CALLS]


CANCEL_EXEC = '/execute_trajectory/_action/cancel_goal'
CANCEL_ARM = '/arm_controller/follow_joint_trajectory/_action/cancel_goal'

print('[ホームへ戻す]')
node = fresh_lemon()
mini_ros.Node.publisher_counts['/lemon_harvest_manager/status'] = 0
node.tick(3)
mini_ros.SERVICES['/arm_home/go_home'] = lambda _r: Resp(True, 'reached')
send({'cmd': 'home'})
time.sleep(0.2)
check('元栓が OFF なら home を呼ばない', '/arm_home/go_home' not in names(), names())
send({'cmd': 'set_enable', 'value': True})
send({'cmd': 'home'})
got = wait_log('ホームへ戻す: reached')
check('/arm_home/go_home を呼び、結果を出す', got is not None, got)

print('[動いている間はほかの動作を呼ばない]')


def slow_home(_r):
    time.sleep(1.0)
    return Resp(True, 'reached (slow)')


mini_ros.SERVICES['/arm_home/go_home'] = slow_home
mini_ros.SERVICES['/lemon_grasp/open'] = lambda _r: Resp(True, 'opened')
mini_ros.SERVICES['/lemon_target/release'] = lambda _r: Resp(True, 'Released (0.1, 0.2, 0.3), 1 excluded')
mini_ros.SERVICES['/lemon_grasp/stop'] = lambda _r: Resp(True, 'not grasping')
mini_ros.SERVICES[CANCEL_EXEC] = lambda _r: CancelGoal.Response(0, ['g1'])
mini_ros.SERVICES[CANCEL_ARM] = lambda _r: CancelGoal.Response(1, [])
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'home'})
st = state(node)
check('home の間は motion_busy に出る', st.get('motion_busy') == 'go_home', st.get('motion_busy'))
send({'cmd': 'release'})
send({'cmd': 'start_selection'})
send({'cmd': 'home'})
check('home の間に開放を押しても呼ばない', '/lemon_grasp/open' not in names(), names())
check('home の間に対象の解除を押しても呼ばない', '/lemon_target/release' not in names(), names())
check('home を二重に呼ばない', names().count('/arm_home/go_home') == 1, names())
check('断った理由を出す', any('いま「go_home」が動いている' in t for t in logs()))

print('[停止は何が動いていても呼ぶ]')
send({'cmd': 'abort'})
time.sleep(0.3)
check('動いている最中でも /lemon_grasp/stop を呼ぶ', '/lemon_grasp/stop' in names(), names())
check('MoveIt の実行（home / approach）を取り消す', CANCEL_EXEC in names(), names())
check('アームの軌道を取り消す', CANCEL_ARM in names(), names())
check('ソフト停止であって非常停止ではないと出す',
      any('非常停止ではありません' in t for t in logs()))
check('取り消した件数を出す', wait_log('1 件を取り消し') is not None)
check('取り消すものが無いのは失敗にしない',
      any('取り消すものなし' in json.loads(m.data)['text'] and json.loads(m.data)['level'] == 'info'
          for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])))
wait_log('reached (slow)', 3.0)

send({'cmd': 'set_enable', 'value': False})
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'abort'})
time.sleep(0.3)
check('元栓が OFF でも停止は呼ぶ', '/lemon_grasp/stop' in names(), names())
send({'cmd': 'set_enable', 'value': True})

print('[開放 = グリッパを開いてから対象を解除]')
time.sleep(0.2)
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'release'})
got = wait_log('対象を解除: Released')
check('open のあとに release を呼ぶ', got is not None and
      names()[:2] == ['/lemon_grasp/open', '/lemon_target/release'], names())
mini_ros.SERVICES['/lemon_grasp/open'] = lambda _r: Resp(False, 'failed to open gripper')
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'release'})
wait_log('failed to open gripper')
time.sleep(0.2)
check('開けなかったら対象は解除しない（まだ掴んでいるかもしれない）',
      names() == ['/lemon_grasp/open'], names())

print('[対象を解除]')
mini_ros.BUS.publish('/lemon_target/target', mini_ros.PointStamped('base_link', 0.36, 0.05, 0.03))
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'start_selection'})
check('「対象を解除」は /lemon_target/release', wait_log('対象を解除: Released') is not None
      and names() == ['/lemon_target/release'], names())

print('[auto_run:=true のマネージャが居たら動かさない]')
mini_ros.Node.publisher_counts['/lemon_harvest_manager/status'] = 1
node.tick(3)
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'home'})
send({'cmd': 'release'})
time.sleep(0.2)
check('マネージャが居る間は home も開放も呼ばない', names() == [], names())
check('理由を出す', any('auto_run:=true' in t for t in logs()))
send({'cmd': 'abort'})
time.sleep(0.2)
check('マネージャが居ても停止は呼ぶ', '/lemon_grasp/stop' in names(), names())
mini_ros.Node.publisher_counts['/lemon_harvest_manager/status'] = 0

# --------------------------------------------------------------------- #
# 段階5: 把持姿勢へ移動（approach）と把持（grasp）
# --------------------------------------------------------------------- #
print('[把持姿勢へ移動]')
node = fresh_lemon()
mini_ros.Node.publisher_counts['/lemon_harvest_manager/status'] = 0
node.tick(3)
send({'cmd': 'set_enable', 'value': True})
mini_ros.SERVICES['/lemon_approach/approach'] = lambda _r: Resp(True, 'reached')
mini_ros.SERVICES['/lemon_grasp/grasp'] = lambda _r: Resp(True, 'grasped')
mini_ros.SERVICES['/arm_home/go_home'] = lambda _r: Resp(True, 'reached')
mini_ros.SERVICES['/lemon_grasp/stop'] = lambda _r: Resp(True, 'stopping')
mini_ros.SERVICES[CANCEL_EXEC] = lambda _r: CancelGoal.Response(0, ['g1'])
mini_ros.SERVICES[CANCEL_ARM] = lambda _r: CancelGoal.Response(0, [])
mini_ros.SERVICES['/lemon_target/release'] = lambda _r: Resp(True, 'Released')
del mini_ros.SERVICE_CALLS[:]

send({'cmd': 'plan_grasp'})
time.sleep(0.2)
check('対象が無ければ approach を呼ばない', names() == [], names())
check('対象を選ぶよう促す', any('対象を選んでください' in t for t in logs()))


def lock(x=0.36, y=0.05, z=0.03):
    mini_ros.BUS.publish('/lemon_target/target', mini_ros.PointStamped('base_link', x, y, z))


lock()
send({'cmd': 'grasp'})
time.sleep(0.2)
check('approach の前は grasp を呼ばない（いまの姿勢から差し込むので危ない）',
      '/lemon_grasp/grasp' not in names(), names())
check('先に把持姿勢へ移動するよう促す', any('先に「把持姿勢へ移動」' in t for t in logs()))
check('状態の approached は false', state(node)['approached'] is False)

send({'cmd': 'plan_grasp'})
check('/lemon_approach/approach を呼ぶ', wait_log('把持姿勢へ移動: reached') is not None, names())
time.sleep(0.05)
check('成功したら approached が true（把持を押せる）', state(node)['approached'] is True)

print('[把持]')
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'grasp'})
got = wait_log('把持: 把持成功')
check('approach の直後なら /lemon_grasp/grasp を呼ぶ', '/lemon_grasp/grasp' in names(), names())
check('成功は「把持成功」と出す', got is not None, got)
check('1 回やったら approached は false（やり直し）', state(node)['approached'] is False)
del mini_ros.SERVICE_CALLS[:]
send({'cmd': 'grasp'})
time.sleep(0.2)
check('続けて把持を押しても呼ばない', names() == [], names())

mini_ros.SERVICES['/lemon_grasp/grasp'] = lambda _r: Resp(False, 'missed (gripper closed to 0.000105)')
send({'cmd': 'plan_grasp'})
wait_log('把持姿勢へ移動: reached')
time.sleep(0.05)
send({'cmd': 'grasp'})
got = wait_log('掴めませんでした')
check('missed は「掴めませんでした」と出す（失敗扱いにしない）', got is not None, got)
check('missed のログは warn（error にしない）',
      any(json.loads(m.data)['level'] == 'warn' and '掴めませんでした' in json.loads(m.data)['text']
          for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])))

print('[approach のあとに起きたら、やり直し]')
send({'cmd': 'plan_grasp'})
wait_log('把持姿勢へ移動: reached')
time.sleep(0.05)
send({'cmd': 'home'})
wait_log('ホームへ戻す: reached')
check('home へ戻ったら把持は押せない', state(node)['approached'] is False)

send({'cmd': 'plan_grasp'})
time.sleep(0.3)
lock(0.30, 0.10, 0.04)                 # 別の対象に変わった
check('対象が変わったら把持は押せない', state(node)['approached'] is False)

send({'cmd': 'plan_grasp'})
time.sleep(0.3)
send({'cmd': 'abort'})
time.sleep(0.2)
check('停止を押したら把持は押せない', state(node)['approached'] is False)

print('[停止で止めたことを、失敗と区別して出す]')


def slow_fail(_r):
    time.sleep(0.6)
    return Resp(False, 'execution failed')


mini_ros.SERVICES['/arm_home/go_home'] = slow_fail
send({'cmd': 'home'})
time.sleep(0.2)
send({'cmd': 'abort'})
got = wait_log('ホームへ戻す: 停止で止めました', 3.0)
check('停止のあとに返った失敗は「停止で止めました」', got is not None, got)
time.sleep(0.1)
mini_ros.SERVICES['/arm_home/go_home'] = lambda _r: Resp(False, 'unreachable after 3 attempts')
send({'cmd': 'home'})
got = wait_log('ホームへ戻す: unreachable', 3.0)
check('停止していない失敗はそのまま失敗と出す',
      got is not None and '停止で止めました' not in got, got)

# --------------------------------------------------------------------- #
# 手動ジョグ・座標指定・グリッパ（lemon では MoveIt で動かす）
# --------------------------------------------------------------------- #
print('[手動ジョグ]')
from piper_arm_ui.lemon_backend import ServiceCaller  # noqa: E402
node = make(robot_description_file=os.path.join(HERE, 'test_arm.urdf'))
mini_ros.Node.publisher_counts['/lemon_harvest_manager/status'] = 0
node.tick(3)
send({'cmd': 'set_enable', 'value': True})
moves, grips = [], []
node.callers['joint_move'] = ServiceCaller(node, None, '/fake/joint_move', None, 5.0)
node.callers['gripper'] = ServiceCaller(node, None, '/fake/gripper', None, 5.0)
mini_ros.SERVICES['/fake/joint_move'] = lambda q: (moves.append(list(q)), Resp(True, 'reached'))[1]
mini_ros.SERVICES['/fake/gripper'] = lambda pos: (grips.append(pos), Resp(True, 'closed'))[1]
mini_ros.SERVICES['/lemon_grasp/open'] = lambda _r: Resp(True, 'opened')


def feedback(q):
    js = mini_ros.JointState()
    js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
    js.position = list(q) + [0.03]
    mini_ros.BUS.publish('/joint_states_single', js)


node._last_jog_at = None
send({'cmd': 'jog', 'joint': 1, 'delta': 0.04})
check('実機の関節角が届いていなければ動かさない',
      moves == [] and any('実機の関節角が届いていません' in t for t in logs()), moves)
Q = [0.1, 0.8, -0.9, 0.0, 0.4, -0.2]
feedback(Q)
node._last_jog_at = None
send({'cmd': 'jog', 'joint': 1, 'delta': 0.04})
wait_log('関節ジョグ J2: reached')
check('実機の関節角 ＋ delta を目標にする（指令値ではない）',
      moves and abs(moves[-1][1] - 0.84) < 1e-9 and moves[-1][0] == 0.1, moves[-1:])
st = state(node)
check('関節の表には実機の関節角を出す', st['command_joints'] == [round(v, 4) for v in Q],
      st['command_joints'])
feedback([0.1, 3.14, -0.9, 0.0, 0.4, -0.2])
n = len(moves)
node._last_jog_at = None
send({'cmd': 'jog', 'joint': 1, 'delta': 0.04})
time.sleep(0.2)
check('可動域の端からは動かさない', len(moves) == n and any('可動域の端' in t for t in logs()))

print('[座標指定]')
feedback(Q)
here = node.chain.fk_position(Q)
check('現在の手先は実機の関節角から出す',
      state(node)['tcp'] == [round(float(v), 4) for v in here], state(node)['tcp'])
node._last_jog_at = None
send({'cmd': 'jog_xyz', 'axis': 2, 'delta': 0.01})
wait_log('座標ジョグ (')
q = moves[-1]
reached = node.chain.fk_position(q)
check('座標ジョグ: 手先が +Z に 1cm の関節角を目標にする',
      abs(reached[2] - (here[2] + 0.01)) < 0.001 and abs(reached[0] - here[0]) < 0.001,
      [round(float(v), 4) for v in reached])
send({'cmd': 'move_to_xyz', 'x': float(here[0]) + 0.02, 'y': float(here[1]), 'z': float(here[2])})
wait_log('座標指定 (')
reached = node.chain.fk_position(moves[-1])
check('座標指定: 指定した位置の関節角を目標にする', abs(reached[0] - (here[0] + 0.02)) < 0.001,
      [round(float(v), 4) for v in reached])
n = len(moves)
send({'cmd': 'move_to_xyz', 'x': float(here[0]) + 0.5, 'y': float(here[1]), 'z': float(here[2])})
send({'cmd': 'move_to_xyz', 'x': 'nan', 'y': 0, 'z': 0})
time.sleep(0.2)
check('1回の上限より遠い位置・数値でない位置へは動かさない', len(moves) == n, moves[n:])

print('[グリッパ]')
send({'cmd': 'gripper', 'close': True})
wait_log('グリッパを閉じる: closed')
check('閉: コントローラへ 0（閉じる）を送る', grips == [0.0], grips)
send({'cmd': 'gripper', 'close': False})
check('開: /lemon_grasp/open を呼ぶ', wait_log('グリッパを開く: opened') is not None)

print('[ジョグ・座標指定にも元栓と排他が効く]')
send({'cmd': 'set_enable', 'value': False})
n = len(moves)
node._last_jog_at = None
send({'cmd': 'jog', 'joint': 0, 'delta': 0.04})
node._last_jog_at = None
send({'cmd': 'jog_xyz', 'axis': 0, 'delta': 0.01})
time.sleep(0.2)
check('元栓が OFF ならジョグも座標指定も動かさない', len(moves) == n)
send({'cmd': 'set_enable', 'value': True})
mini_ros.SERVICES['/fake/joint_move'] = lambda q: (time.sleep(0.8), moves.append(list(q)),
                                                   Resp(True, 'reached'))[2]
node._last_jog_at = None
send({'cmd': 'jog', 'joint': 0, 'delta': 0.04})
time.sleep(0.1)
send({'cmd': 'home'})
check('ジョグで動いている間はほかの動作を呼ばない',
      wait_log('いま「joint_move」が動いている', 1.0) is not None)
wait_log('関節ジョグ J1: reached', 3.0)

print('[続けざまのジョグは無視する（長押しが止まらなくなった件の歯止め）]')
mini_ros.SERVICES['/fake/joint_move'] = lambda q: (moves.append(list(q)), Resp(True, 'reached'))[1]
time.sleep(0.6)
n = len(moves)
for _ in range(10):                    # 画面が 0.12 秒ごとに送り続けた状況
    feedback(Q)
    send({'cmd': 'jog', 'joint': 0, 'delta': 0.02})
    time.sleep(0.12)
time.sleep(0.2)
check('0.12 秒ごとに 10 回来ても、動くのは最初の 1 回だけ', len(moves) - n == 1, len(moves) - n)
check('無視したことを出す', any('続けざまの入力を無視しました' in t for t in logs()))
time.sleep(0.6)
feedback(Q)
send({'cmd': 'jog', 'joint': 0, 'delta': 0.02})
time.sleep(0.2)
time.sleep(0.1)
check('間を空ければ次の 1 回は動く', len(moves) - n == 2, len(moves) - n)
n = len(moves)
time.sleep(0.6)
feedback(Q)
send({'cmd': 'jog_xyz', 'axis': 2, 'delta': 0.005})
send({'cmd': 'jog_xyz', 'axis': 2, 'delta': 0.005})
time.sleep(0.3)
check('座標ジョグも続けざまは 1 回だけ', len(moves) - n == 1, len(moves) - n)

# --------------------------------------------------------------------- #
print('[internal は従来どおり]')
mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {'backend': 'internal', 'stub_step_duration': 0.0}
mini_ros.BUS.clear()
mini_ros.BUS.subs.clear()
mini_ros.Node.publisher_counts = {}
from piper_arm_ui.harvest_sequencer import HarvestSequencer  # noqa: E402
node = HarvestSequencer()
check('internal では /joint_states の publisher がある', node.joint_pub is not None)
check('internal ではサービスのクライアントを作らない', node.callers == {})
send({'cmd': 'start_selection'})
check('internal では従来のコマンドが効く', node.state == 'SELECTING', node.state)

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
