"""指令トピックの横取り検出（ros2_control 構成での安全弁）の検証.

守りたいこと:

    /joint_states を他のノード（joint_state_broadcaster）が publish している
    構成では、UI から関節指令を出さない。

本家 piper_ros は joint_ctrl_single → /joint_states のリマップがあるので
/joint_states に publish すればアームが動いた。ros2_control を使う構成では
向きが逆で、publish すると MoveIt / Servo が読む現在姿勢が壊れる。
画面を見ても分からない種類の事故なので、自動で止まることを確かめる。
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

from piper_arm_ui.common import TOPIC_UI_COMMAND, TOPIC_UI_LOG, TOPIC_UI_STATE  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def make(publisher_counts, **params):
    """publisher の数を仕込んでシーケンサを作り、検出タイマーを回す."""
    mini_ros.BUS.clear()
    mini_ros.BUS.subs.clear()
    mini_ros.Node.publisher_counts = dict(publisher_counts)
    mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = dict(
        {'enable_arm_command': True, 'stub_step_duration': 0.0}, **params)

    # import はスタブを入れてから
    from piper_arm_ui.harvest_sequencer import HarvestSequencer
    node = HarvestSequencer()
    node.tick(2)          # 二重起動＋横取りの確認タイマー
    return node


def ui_logs():
    return [json.loads(m.data)['text']
            for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])]


def send(obj):
    mini_ros.BUS.publish(TOPIC_UI_COMMAND, mini_ros.String(json.dumps(obj)))


def state_of(node):
    node.tick(1)
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)


# --------------------------------------------------------------------- #
# 1. 本家の構成（UI だけが /joint_states に publish）
# --------------------------------------------------------------------- #
node = make({})
check('他に publisher がいなければ指令は有効のまま', node.enable_arm_command is True)
check('横取りフラグは立たない', node._command_topic_taken is False)
st = state_of(node)
check('状態にも載る', st['command_topic_taken'] is False, st.get('command_topic_taken'))
check('指令トピック名が状態に載る',
      st['joint_command_topic'] == '/joint_states', st.get('joint_command_topic'))

before = len(mini_ros.BUS.sent.get('/joint_states', []))
node.tick(0)
check('制御ループが関節指令を publish する',
      len(mini_ros.BUS.sent.get('/joint_states', [])) > before)

# --------------------------------------------------------------------- #
# 2. ros2_control の構成（broadcaster も publish している）
# --------------------------------------------------------------------- #
node = make({'/joint_states': 2})
check('横取りを検出する', node._command_topic_taken is True)
check('関節指令を自動で止める', node.enable_arm_command is False)
check('理由を UI のログに出す',
      any('他のノードも publish' in t for t in ui_logs()), ui_logs())
check('無効にしたことも伝える',
      any('安全のため' in t for t in ui_logs()), ui_logs())

st = state_of(node)
check('状態に横取りを載せる', st['command_topic_taken'] is True)
check('状態の enable_arm_command も false', st['enable_arm_command'] is False)

before = len(mini_ros.BUS.sent.get('/joint_states', []))
for _ in range(5):
    node.tick(0)
check('制御ループは関節指令を publish しない',
      len(mini_ros.BUS.sent.get('/joint_states', [])) == before,
      len(mini_ros.BUS.sent.get('/joint_states', [])) - before)

# 画面のトグルで戻せないこと（戻せたら安全弁の意味がない）
mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
send({'cmd': 'set_enable', 'value': True})
check('画面のトグルでも再有効化できない', node.enable_arm_command is False)
check('再有効化できない理由を出す',
      any('有効にできません' in t for t in ui_logs()), ui_logs())

before = len(mini_ros.BUS.sent.get('/joint_states', []))
node.tick(0)
check('トグル後も publish しない',
      len(mini_ros.BUS.sent.get('/joint_states', [])) == before)

# --------------------------------------------------------------------- #
# 3. 監視としては生きていること（止めるのは指令だけ）
# --------------------------------------------------------------------- #
send({'cmd': 'start_selection'})
check('UI の操作自体は受け付ける（状態は進む）', node.state == 'SELECTING', node.state)

before = list(node.cmd_pose)
send({'cmd': 'jog', 'joint': 0, 'delta': 0.04})
check('ジョグで内部の指令姿勢は動く（publish しないだけ）',
      abs(node.cmd_pose[0] - before[0] - 0.04) < 1e-9)

st = state_of(node)
check('3D ビュー用の関節角は出続ける',
      isinstance(st['command_joints'], list) and len(st['command_joints']) == 6)

# --------------------------------------------------------------------- #
# 4. 指令トピックを変えれば使える（将来 /joint_ctrl_single に向ける場合）
# --------------------------------------------------------------------- #
node = make({'/joint_states': 2}, joint_command_topic='/joint_ctrl_single')
check('別のトピックに向ければ横取り扱いにならない',
      node._command_topic_taken is False)
check('その場合は指令が有効のまま', node.enable_arm_command is True)
st = state_of(node)
check('状態に新しい指令トピック名が載る',
      st['joint_command_topic'] == '/joint_ctrl_single', st.get('joint_command_topic'))

before = len(mini_ros.BUS.sent.get('/joint_ctrl_single', []))
node.tick(0)
check('新しいトピックへ publish する',
      len(mini_ros.BUS.sent.get('/joint_ctrl_single', [])) > before)

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
