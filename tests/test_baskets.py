"""収穫位置①②③（backend:=lemon）の検証.

守りたいこと:

    人が実機で置いて記録した姿勢へだけ動かす。記録していない位置へは動かさない。
    記録はファイルに残り、起動し直しても使える。壊れたファイルでは動かさない。

  - 記録は実機の関節角（/joint_states_single）から取る。届いていなければ記録しない
  - 動いている最中は記録しない（どの姿勢を記録したか分からなくなる）
  - 収穫位置への移動は、ほかの動く呼び出しと同時にしない／元栓が OFF なら呼ばない
  - 停止で止めたら「停止で止めました」

MoveIt（/plan_kinematic_path・/execute_trajectory）は使わず、収穫位置へ動かす部分は
mini_ros の偽のサービスに差し替える（BasketMover の中身は実機で確かめる）。
"""
import json
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

from piper_arm_ui.basket_store import BasketStore     # noqa: E402
from piper_arm_ui.common import TOPIC_UI_COMMAND, TOPIC_UI_LOG, TOPIC_UI_STATE  # noqa: E402
from piper_arm_ui.lemon_backend import ServiceCaller  # noqa: E402

FAIL = []
Resp = mini_ros._Trigger.Response
TMP = tempfile.mkdtemp(prefix='basket_test_')


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


# --------------------------------------------------------------------- #
print('[記録のファイル]')
path = os.path.join(TMP, 'sub', 'basket_poses.json')
st = BasketStore(path)
check('ファイルが無ければ記録なし', st.get(1) is None and st.error == '')
check('記録なしは recorded=false', st.snapshot()['1'] == {'recorded': False})
st.set(2, [0.1, 0.2, -0.3, 0.0, 0.4, -0.5])
check('記録できる（フォルダが無くても作る）', os.path.exists(path))
st2 = BasketStore(path)
check('起動し直しても残る', st2.get(2) == [0.1, 0.2, -0.3, 0.0, 0.4, -0.5], st2.get(2))
check('記録した時刻も残る', st2.snapshot()['2']['recorded_at'] != '')
check('ほかの位置は記録なしのまま', st2.get(1) is None and st2.get(3) is None)
try:
    st2.set(4, [0] * 6)
    check('1〜3 以外は記録しない', False)
except ValueError:
    check('1〜3 以外は記録しない', True)
try:
    st2.set(1, [0] * 5)
    check('関節角が 6 個でなければ記録しない', False)
except ValueError:
    check('関節角が 6 個でなければ記録しない', True)
with open(path, 'w') as f:
    f.write('{ 壊れた')
st3 = BasketStore(path)
check('壊れたファイルなら記録なし扱い（動かさない）', st3.get(2) is None)
check('壊れたことを理由として出す', '読めません' in st3.error, st3.error)
leftovers = [n for n in os.listdir(os.path.dirname(path)) if n.endswith('.tmp')]
check('書き込みの一時ファイルを残さない', not leftovers, leftovers)

# --------------------------------------------------------------------- #
print('[ノード: 記録]')
mini_ros.BUS.clear()
mini_ros.BUS.subs.clear()
mini_ros.SERVICES.clear()
del mini_ros.SERVICE_CALLS[:]
mini_ros.Node.publisher_counts = {'/joint_states': 2, '/lemon_harvest_manager/status': 0}
node_file = os.path.join(TMP, 'node', 'basket_poses.json')
mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {
    'backend': 'lemon', 'stub_step_duration': 0.0, 'basket_file': node_file}
from piper_arm_ui.harvest_sequencer import HarvestSequencer  # noqa: E402
node = HarvestSequencer()
node.tick(3)
# 収穫位置へ動かす部分を偽のサービスに差し替える（request は関節角の並び）
node.callers['basket'] = ServiceCaller(node, None, '/fake/basket', None, 5.0)
moved = []


def fake_basket(joints):
    moved.append(list(joints))
    return Resp(True, 'reached')


mini_ros.SERVICES['/fake/basket'] = fake_basket


def send(obj):
    mini_ros.BUS.publish(TOPIC_UI_COMMAND, mini_ros.String(json.dumps(obj)))


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


def state():
    node.tick(1)
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)


def feedback(q):
    js = mini_ros.JointState()
    js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
    js.position = list(q) + [0.03]
    mini_ros.BUS.publish('/joint_states_single', js)


check('状態に収穫位置の記録が載る', state()['baskets']['1'] == {'recorded': False})
send({'cmd': 'set_basket', 'index': 1})
check('元栓が OFF なら記録しない', node.baskets.get(1) is None and any('OFF なので' in t for t in logs()))
send({'cmd': 'set_enable', 'value': True})
send({'cmd': 'set_basket', 'index': 1})
check('実機の関節角が届いていなければ記録しない',
      node.baskets.get(1) is None and any('届いていません' in t for t in logs()))
Q1 = [1.2, 0.8, -0.9, 0.0, 0.5, -0.2]
feedback(Q1)
send({'cmd': 'set_basket', 'index': 1})
check('実機の関節角を記録する', node.baskets.get(1) == Q1, node.baskets.get(1))
check('記録したことをログに出す', wait_log('収穫位置①を記録しました') is not None)
check('記録はファイルに残る', BasketStore(node_file).get(1) == Q1)
check('状態にも recorded=true で載る', state()['baskets']['1']['recorded'] is True)

# --------------------------------------------------------------------- #
print('[ノード: 移動]')
send({'cmd': 'goto_basket', 'index': 2})
check('記録していない位置へは動かさない',
      moved == [] and any('まだ記録されていません' in t for t in logs()), moved)
send({'cmd': 'goto_basket', 'index': 1})
got = wait_log('収穫位置①へ移動: reached')
check('記録した関節角へ動かす', got is not None and moved == [Q1], moved)
time.sleep(0.05)
check('着いたら basket_index が 1', state()['basket_index'] == 1)

send({'cmd': 'set_enable', 'value': False})
send({'cmd': 'goto_basket', 'index': 1})
time.sleep(0.2)
check('元栓が OFF なら動かさない', len(moved) == 1, moved)
send({'cmd': 'set_enable', 'value': True})


def slow_basket(joints):
    time.sleep(0.8)
    return Resp(False, 'execution failed (error_code=-7)')


mini_ros.SERVICES['/fake/basket'] = slow_basket
mini_ros.SERVICES['/arm_home/go_home'] = lambda _r: Resp(True, 'reached')
send({'cmd': 'goto_basket', 'index': 1})
time.sleep(0.1)
check('移動中は motion_busy に basket と出る', state()['motion_busy'] == 'basket')
send({'cmd': 'home'})
send({'cmd': 'set_basket', 'index': 2})
time.sleep(0.2)
check('移動中は home を呼ばない', '/arm_home/go_home' not in [n for n, _ in mini_ros.SERVICE_CALLS])
check('移動中は記録しない', node.baskets.get(2) is None)
mini_ros.SERVICES['/lemon_grasp/stop'] = lambda _r: Resp(True, 'not grasping')
send({'cmd': 'abort'})
got = wait_log('収穫位置①へ移動: 停止で止めました', 3.0)
check('停止で止めたら「停止で止めました」', got is not None, got)
time.sleep(0.05)
check('止めたら basket_index は付かない', state()['basket_index'] is None)

shutil.rmtree(TMP, ignore_errors=True)
print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
