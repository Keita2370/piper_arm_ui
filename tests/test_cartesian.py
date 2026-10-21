"""座標指定（手先位置）でアームを動かす経路の検証.

UI のボタンから出るコマンドを、本物の HarvestSequencer に流して確かめる。
見たいのはここ:

  - 座標を指定したら、その座標に手先が来る関節角が指令されるか
  - 座標ジョグ（±）が本当に 1 ステップぶん動かすか
  - 届かない座標・単位間違い・壊れた入力で、黙って無視せず警告を出すか
  - 可動域を超える指令を出さないか
  - URDF が無い環境でも起動して、理由を UI に伝えるか
  - 関節角指定（既存のジョグ）を壊していないか
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mini_ros                                       # noqa: E402

mini_ros.install()

import numpy as np                                    # noqa: E402

URDF = os.path.join(HERE, 'test_arm.urdf')
mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {
    'robot_description_file': URDF,
    'enable_arm_command': True,
    # 軌道は実時間で進む（Trajectory が time.monotonic を見る）。
    # テストでは最短の 0.05 秒に落として、実際に時間を進めながら回す。
    'stub_step_duration': 0.0,
}

from piper_arm_ui.common import (                     # noqa: E402
    JOINT_LIMITS, TOPIC_UI_COMMAND, TOPIC_UI_LOG, TOPIC_UI_STATE,
)
from piper_arm_ui.harvest_sequencer import HarvestSequencer   # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def send(node, obj):
    """UI からのコマンドを 1 つ流す."""
    BUS = mini_ros.BUS
    BUS.publish(TOPIC_UI_COMMAND, mini_ros.String(json.dumps(obj)))


def state_of(node):
    node.tick(1)                       # 状態配信タイマー
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)


def logs(node, level=None):
    return node.get_logger().text(level)


def ui_logs():
    return [json.loads(m.data)['text']
            for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])]


def run_trajectory(node, steps=400):
    """軌道が終わるまで制御ループを回す.

    Trajectory は実時間（time.monotonic）で進むので、ただ tick を
    連打しても終わらない。少しずつ本当に待つ必要がある。
    """
    for _ in range(steps):
        node.tick(0)
        if node.trajectory is None:
            return True
        time.sleep(0.005)
    return False


node = HarvestSequencer()

# --------------------------------------------------------------------- #
# 1. 起動して座標指定が有効になる
# --------------------------------------------------------------------- #
check('URDF を読んで座標指定が有効になる', node.chain is not None, node.chain_error)
st = state_of(node)
check('状態に tcp_available が載る', st['tcp_available'] is True, st.get('tcp_reason'))
check('状態に現在の手先位置が載る',
      isinstance(st['tcp'], list) and len(st['tcp']) == 3, st.get('tcp'))
check('状態に手先リンク名が載る', bool(st['tcp_link']), st.get('tcp_link'))
check('状態に可動範囲の目安が載る',
      st['tcp_reach'] and st['tcp_reach']['max_radius'] > 0.3, st.get('tcp_reach'))

fk = node.chain.fk_position(node.cmd_pose)
check('tcp が指令姿勢の順運動学と一致',
      max(abs(a - b) for a, b in zip(st['tcp'], fk)) < 1e-3,
      (st['tcp'], [round(v, 4) for v in fk]))

# --------------------------------------------------------------------- #
# 2. 座標を指定して移動する
# --------------------------------------------------------------------- #
# 現在位置から 10cm ほど離れた、確実に到達できる点を作る
here = np.array(node.chain.fk_position(node.cmd_pose))
reachable = node.chain.fk_position([0.3, 0.9, -0.8, 0.0, 0.2, 0.0])
step_ok = float(np.linalg.norm(np.array(reachable) - here)) <= node.max_cartesian_step
check('テスト用の目標が 1 回の上限内にある', step_ok,
      '%.0f mm' % (float(np.linalg.norm(np.array(reachable) - here)) * 1000))

send(node, {'cmd': 'move_to_xyz',
            'x': float(reachable[0]), 'y': float(reachable[1]), 'z': float(reachable[2])})
check('座標指定で軌道が始まる', node.trajectory is not None)
run_trajectory(node)
reached = node.chain.fk_position(node.cmd_pose)
err = float(np.linalg.norm(np.array(reached) - np.array(reachable)))
check('指定した座標に手先が来る（1mm 未満）', err < 1e-3, '%.3f mm' % (err * 1000))
check('移動後は待機に戻る', node.state == 'IDLE', node.state)

st = state_of(node)
check('移動後の tcp が指定座標と一致',
      max(abs(a - b) for a, b in zip(st['tcp'], reachable)) < 1e-3,
      (st['tcp'], [round(float(v), 4) for v in reachable]))

# --------------------------------------------------------------------- #
# 3. 座標ジョグ（± ボタン）
# --------------------------------------------------------------------- #
for axis, name in enumerate('XYZ'):
    before = np.array(node.chain.fk_position(node.cmd_pose))
    send(node, {'cmd': 'jog_xyz', 'axis': axis, 'delta': 0.01})
    after = np.array(node.chain.fk_position(node.cmd_pose))
    moved = after - before
    expected = np.zeros(3)
    expected[axis] = 0.01
    check('%s+ のジョグがその軸だけ 10mm 動かす' % name,
          float(np.abs(moved - expected).max()) < 2e-4,
          '移動 %s mm' % [round(v * 1000, 2) for v in moved])

before = np.array(node.chain.fk_position(node.cmd_pose))
send(node, {'cmd': 'jog_xyz', 'axis': 0, 'delta': -0.01})
after = np.array(node.chain.fk_position(node.cmd_pose))
check('X− のジョグが逆向きに動かす',
      abs((after - before)[0] + 0.01) < 2e-4,
      '%.2f mm' % ((after - before)[0] * 1000))

check('座標ジョグは軌道を使わず即座に反映する', node.trajectory is None)

# 連続ジョグ（長押し）が累積すること
start = np.array(node.chain.fk_position(node.cmd_pose))
for _ in range(5):
    send(node, {'cmd': 'jog_xyz', 'axis': 2, 'delta': 0.005})
end = np.array(node.chain.fk_position(node.cmd_pose))
check('連続ジョグが累積する（5回 x 5mm = 25mm）',
      abs((end - start)[2] - 0.025) < 1e-3,
      '%.1f mm' % ((end - start)[2] * 1000))

# --------------------------------------------------------------------- #
# 4. 可動域を超えない
# --------------------------------------------------------------------- #
lower = [JOINT_LIMITS['joint%d' % (i + 1)][0] for i in range(6)]
upper = [JOINT_LIMITS['joint%d' % (i + 1)][1] for i in range(6)]
violations = []
rng = np.random.default_rng(5)
for _ in range(40):
    # 可動範囲のふちを狙う
    target = list(node.chain.fk_position(rng.uniform(node.chain.lower, node.chain.upper)))
    send(node, {'cmd': 'move_to_xyz', 'x': target[0], 'y': target[1], 'z': target[2]})
    run_trajectory(node)
    for i, v in enumerate(node.cmd_pose):
        if v < lower[i] - 1e-9 or v > upper[i] + 1e-9:
            violations.append((i + 1, v))
check('どんな座標指定でも関節可動域を出ない', not violations, violations[:4])

# --------------------------------------------------------------------- #
# 5. おかしな入力は黙って無視しない
# --------------------------------------------------------------------- #
mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
before = list(node.cmd_pose)
send(node, {'cmd': 'move_to_xyz', 'x': 0.0, 'y': 0.0, 'z': 3.0})
check('届かない座標は動かさない', node.cmd_pose == before)
check('届かない座標は「可動範囲の外」と言う（離れすぎ、ではなく）',
      any('可動範囲の外' in t for t in ui_logs()), ui_logs())

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
before = list(node.cmd_pose)
# mm で入れてしまったケース（250 = 250m 先）
send(node, {'cmd': 'move_to_xyz', 'x': 250.0, 'y': 0.0, 'z': 0.0})
check('単位を間違えた入力は動かさない', node.cmd_pose == before)
check('単位の間違い（mm で入力）を名指しで警告する',
      any('単位は m' in t for t in ui_logs()), ui_logs())

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
before = list(node.cmd_pose)
send(node, {'cmd': 'move_to_xyz', 'x': 'abc', 'y': 0.0, 'z': 0.2})
check('数値でない入力は動かさない', node.cmd_pose == before)
check('数値でない入力を警告する',
      any('数値' in t for t in ui_logs()), ui_logs())

before = list(node.cmd_pose)
send(node, {'cmd': 'move_to_xyz', 'x': 0.2})           # y, z が無い
check('キーが足りない入力でも落ちない', node.cmd_pose == before)
check('例外で ERROR 状態に落ちていない', node.state != 'ERROR', node.state)

before = list(node.cmd_pose)
send(node, {'cmd': 'jog_xyz', 'axis': 9, 'delta': 0.01})
check('軸番号が範囲外なら何もしない', node.cmd_pose == before)
send(node, {'cmd': 'jog_xyz', 'axis': 0, 'delta': 0.0})
check('delta=0 なら何もしない', node.cmd_pose == before)

# 1 回の上限を超える座標ジョグ（ステップを大きくしすぎた場合）
mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
before = list(node.cmd_pose)
send(node, {'cmd': 'jog_xyz', 'axis': 0, 'delta': 0.5})
check('上限を超える座標ジョグは止める', node.cmd_pose == before)

# --------------------------------------------------------------------- #
# 6. 関節角指定（既存機能）を壊していない
# --------------------------------------------------------------------- #
before = list(node.cmd_pose)
send(node, {'cmd': 'jog', 'joint': 0, 'delta': 0.04})
check('関節ジョグがこれまでどおり効く',
      abs(node.cmd_pose[0] - before[0] - 0.04) < 1e-9,
      (before[0], node.cmd_pose[0]))

send(node, {'cmd': 'home'})
run_trajectory(node)
check('ホームへ戻せる', node.state == 'IDLE' and
      max(abs(v) for v in node.cmd_pose) < 1e-6, node.cmd_pose)

send(node, {'cmd': 'goto_basket', 'index': 2})
run_trajectory(node)
check('収穫位置への移動が効く', node.state == 'AT_BASKET', node.state)

send(node, {'cmd': 'abort'})
check('停止が効く', node.state == 'IDLE' and node.trajectory is None)

# 関節指令の形が変わっていないこと（piper 側の仕様）
jc = mini_ros.BUS.last('/joint_states')
check('piper へ出す JointState の形が変わっていない',
      jc is not None and len(jc.name) == 8 and len(jc.position) == 8,
      None if jc is None else (len(jc.name), len(jc.position)))

# --------------------------------------------------------------------- #
# 7. URDF が無い環境
# --------------------------------------------------------------------- #
mini_ros.BUS.clear()
mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {
    'robot_description_file': '/nonexistent/arm.urdf',
    'enable_arm_command': True,
}
node2 = HarvestSequencer()
check('URDF が無くても起動する', node2.state == 'IDLE')
check('座標指定だけが無効になる', node2.chain is None)
st2 = state_of(node2)
check('使えない理由が状態に載る',
      st2['tcp_available'] is False and bool(st2['tcp_reason']),
      st2.get('tcp_reason'))
check('tcp は None になる', st2['tcp'] is None, st2.get('tcp'))

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
before = list(node2.cmd_pose)
send(node2, {'cmd': 'move_to_xyz', 'x': 0.2, 'y': 0.0, 'z': 0.3})
check('座標指定を押しても落ちない', node2.cmd_pose == before and node2.state != 'ERROR')
check('使えないことを UI に伝える',
      any('座標指定は使えません' in t for t in
          [json.loads(m.data)['text']
           for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])]))

before = list(node2.cmd_pose)
send(node2, {'cmd': 'jog', 'joint': 1, 'delta': 0.04})
check('URDF 無しでも関節ジョグは使える',
      abs(node2.cmd_pose[1] - before[1] - 0.04) < 1e-9)

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
