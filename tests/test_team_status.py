"""チームの収穫システムの状態表示（段階1）の検証.

守りたいこと:

    相手のノード（lemon_harvest_manager / lemon_approach / lemon_grasp /
    lemon_target）が流す status と対象を、/arm_ui/state の 'team' に
    取り違えずに載せる。サービスは呼ばない（読むだけ）。

特に:
  - grasp の 'missed (...)' を失敗（fail）にしない。掴めなかっただけ
  - ただし 'missed ..., retreat failed: ...' は fail（アームがレモンの近くに残る）
  - grasp の 'opening' は把持の準備であって開放ではない
  - manager が居ない（auto_run:=false）ことを「居ない」と出す
  - 対象の frame_id が空なら「対象なし」
  - 既存の状態機械・関節指令に一切触らない

status の文字列は 2026-10-30 に ~/lemon_ws のソースから拾ったもの。
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

from piper_arm_ui.common import (                     # noqa: E402
    TOPIC_TEAM_APPROACH_STATUS, TOPIC_TEAM_GRASP_STATUS, TOPIC_TEAM_MANAGER_STATUS,
    TOPIC_TEAM_TARGET, TOPIC_UI_LOG, TOPIC_UI_STATE)
from piper_arm_ui.team_status import classify         # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def make(publisher_counts=None):
    mini_ros.BUS.clear()
    mini_ros.BUS.subs.clear()
    # 実機と同じ: /joint_states は joint_state_broadcaster も出している
    counts = {'/joint_states': 2}
    counts.update(publisher_counts or {})
    mini_ros.Node.publisher_counts = counts
    mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {'stub_step_duration': 0.0}
    from piper_arm_ui.harvest_sequencer import HarvestSequencer
    return HarvestSequencer()


def status(topic, text):
    mini_ros.BUS.publish(topic, mini_ros.String(text))


def target(frame_id, x=0.0, y=0.0, z=0.0):
    mini_ros.BUS.publish(TOPIC_TEAM_TARGET, mini_ros.PointStamped(frame_id, x, y, z))


def team_of(node):
    node.tick(1)
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)['team']


def ui_logs():
    return [json.loads(m.data) for m in mini_ros.BUS.sent.get(TOPIC_UI_LOG, [])]


# --------------------------------------------------------------------- #
# 1. 文字列の分類（ROS 無しの純粋関数）
# --------------------------------------------------------------------- #
print('[分類]')
check('manager の waiting_target は idle', classify('manager', 'waiting_target')[0] == 'idle')
check('manager の error は fail', classify('manager', 'error')[0] == 'fail')
check('approach の reached は ok', classify('approach', 'reached')[0] == 'ok')
check('approach の知らない失敗は fail（素通し）',
      classify('approach', 'execution failed') == ('fail', 'execution failed'),
      classify('approach', 'execution failed'))
check('grasp の grasped は ok', classify('grasp', 'grasped')[0] == 'ok')

k, label = classify('grasp', 'missed (gripper closed to 0.002000)')
check('grasp の missed は fail ではなく missed', k == 'missed', k)
check('missed の表示は「掴めませんでした」で元の文字列も残す',
      '掴めませんでした' in label and '0.002000' in label, label)

k, _ = classify('grasp', 'missed (gripper closed to 0.0), retreat failed: timeout')
check('missed でも戻りに失敗していたら fail', k == 'fail', k)
check('insertion failed は fail', classify('grasp', 'insertion failed: timeout')[0] == 'fail')
k, label = classify('grasp', 'stopped (while inserting)')
check('stop で止めたのは fail ではなく stopped', k == 'stopped', k)
from piper_arm_ui.team_status import explain  # noqa: E402
k_ins, label_ins = classify('approach', 'cannot insert straight into the lemon')
check('差し込めない: 次にすること（位置を変える・別のレモン）を出し、元の文字列も残す',
      k_ins == 'fail' and '別のレモン' in label_ins and 'cannot insert straight' in label_ins, label_ins)
check('経路が見つからない（unreachable after 3 attempts）も言い換える。把持に限らない言い方',
      '経路が見つかりませんでした' in explain('unreachable after 3 attempts')
      and '把持' not in explain('unreachable after 3 attempts'))
check('知らない失敗はそのまま', explain('arm goal rejected') == 'arm goal rejected')
check('止めたらアームがその場に残ると出す', 'その場に残っています' in label, label)

k, label = classify('grasp', 'opening')
check('grasp の opening は把持の準備（busy）と出す',
      k == 'busy' and '把持の準備' in label, (k, label))
check('retreating は止められない区間だと出す',
      'ソフト停止では止まりません' in classify('grasp', 'retreating')[1])
check('未知の文字列も落とさず素通し', classify('grasp', '')[0] == 'fail')

# --------------------------------------------------------------------- #
# 2. ノードに流す（購読 → /arm_ui/state）
# --------------------------------------------------------------------- #
print('[ノード]')
node = make({TOPIC_TEAM_MANAGER_STATUS: 0})
node.tick(3)                      # 相手が居るかの確認（1 Hz のタイマー）
cmd_before = list(node.cmd_pose)
state_before = node.state
enable_before = node.enable_arm_command

team = team_of(node)
check('state に team が載る', set(team) >= {'manager', 'approach', 'grasp', 'target'}, team)
check('manager が居ないことを present=false で出す（auto_run:=false）',
      team['manager']['present'] is False, team['manager'])
check('approach は居る', team['approach']['present'] is True, team['approach'])
check('まだ何も来ていなければ text は None', team['grasp']['text'] is None)
check('対象は未受信', team['target']['seen'] is False and not team['target']['locked'])

# 把持の一連（lemon_grasp_node.cpp の grasp() の順）
for s in ('opening', 'inserting', 'closing', 'retreating'):
    status(TOPIC_TEAM_GRASP_STATUS, s)
team = team_of(node)
check('grasp の最新が retreating', team['grasp']['text'] == 'retreating', team['grasp'])
check('busy として載る', team['grasp']['kind'] == 'busy')
check('経過秒が載る', isinstance(team['grasp']['age'], float), team['grasp']['age'])

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
status(TOPIC_TEAM_GRASP_STATUS, 'missed (gripper closed to 0.001000)')
team = team_of(node)
check('missed は missed として載る（ERROR にしない）', team['grasp']['kind'] == 'missed')
logs = ui_logs()
check('missed はログに warn で出る（error ではない）',
      logs and logs[-1]['level'] == 'warn' and '掴めませんでした' in logs[-1]['text'], logs)

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
status(TOPIC_TEAM_GRASP_STATUS, 'missed (gripper closed to 0.001000)')
check('同じ文字列が続いてもログは重ねない', not ui_logs(), ui_logs())

status(TOPIC_TEAM_APPROACH_STATUS, 'approaching')
status(TOPIC_TEAM_APPROACH_STATUS, 'cannot insert straight into the lemon')
mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
status(TOPIC_TEAM_APPROACH_STATUS, 'unreachable after 3 attempts')
team = team_of(node)
check('approach の失敗は fail', team['approach']['kind'] == 'fail', team['approach'])
check('失敗の表示に元の message も残す',
      'unreachable after 3 attempts' in team['approach']['label'], team['approach'])
check('失敗はログに error で出る', ui_logs() and ui_logs()[-1]['level'] == 'error', ui_logs())

# manager が後から現れた（auto_run:=true で起動し直した）
mini_ros.Node.publisher_counts[TOPIC_TEAM_MANAGER_STATUS] = 1
node.tick(3)
status(TOPIC_TEAM_MANAGER_STATUS, 'going_home')
team = team_of(node)
check('manager が居れば present=true', team['manager']['present'] is True)
check('manager の status が載る',
      team['manager']['text'] == 'going_home' and team['manager']['label'] == 'ホームへ移動中',
      team['manager'])

# --------------------------------------------------------------------- #
# 3. 対象（latched、frame_id が空なら対象なし）
# --------------------------------------------------------------------- #
print('[対象]')
mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
target('base_link', 0.31234, -0.05, 0.2)
team = team_of(node)
check('frame_id があれば固定', team['target']['locked'] is True, team['target'])
check('位置が mm 単位まで載る', team['target']['point'] == [0.312, -0.05, 0.2],
      team['target']['point'])
check('frame_id が載る', team['target']['frame_id'] == 'base_link')
check('固定をログに出す', any('固定' in l['text'] for l in ui_logs()), ui_logs())

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
target('')
team = team_of(node)
check('frame_id が空なら対象なし', team['target']['locked'] is False, team['target'])
check('対象なしのときは位置を出さない', team['target']['point'] is None)
check('一度は受け取った（解除済み）と区別できる', team['target']['seen'] is True)
check('解除をログに出す', any('解除' in l['text'] for l in ui_logs()), ui_logs())

mini_ros.BUS.sent.pop(TOPIC_UI_LOG, None)
target('')
check('解除が続いてもログは重ねない', not ui_logs(), ui_logs())

# --------------------------------------------------------------------- #
# 4. 読むだけであること
# --------------------------------------------------------------------- #
print('[読むだけ]')
check('UI の状態機械は動かない', node.state == state_before, node.state)
check('関節指令は変わらない', node.cmd_pose == cmd_before)
check('UI操作の有効／無効は変わらない', node.enable_arm_command == enable_before)
team_topics = (TOPIC_TEAM_MANAGER_STATUS, TOPIC_TEAM_APPROACH_STATUS,
               TOPIC_TEAM_GRASP_STATUS, TOPIC_TEAM_TARGET)
# BUS.sent にはテストが流したものも入るので、ノードの publisher を数える
pubs = [t for t in team_topics if t in {p.topic for p in vars(node).values()
                                         if isinstance(p, mini_ros._Publisher)}]
check('相手のトピックに publisher を作らない', not pubs, pubs)

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
