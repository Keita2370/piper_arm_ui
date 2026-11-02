"""アームの状態（/arm_status）の読み取りの検証.

守りたいこと:

    piper_msgs/PiperStatusMsg の真偽値を、piper_sdk の定義どおりに読む。

piper_sdk（interface/piper_interface_v2.py）の定義:
  - joint_N_angle_limit (bool):          True为超限     （True が可動限界）
  - communication_status_joint_N (bool): True为通信异常 （True が通信異常）

communication_status を「正常なら True」と読んでいたため、実機（全関節 False
＝正常）で画面に「関節通信：異常 J1〜J6」と出ていた。名前から意味を推測すると
逆になりやすいので、定義に合わせて数値で押さえる。
"""
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

from piper_arm_ui.common import TOPIC_UI_STATE        # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def status_msg(comm=(), limit=(), err=0):
    """comm / limit に入れた関節番号だけを True にした PiperStatusMsg もどき."""
    fields = {'ctrl_mode': 1, 'arm_status': 0, 'mode_feedback': 1,
              'motion_status': 0, 'teach_status': 0, 'err_code': err}
    for j in range(1, 7):
        fields['communication_status_joint_%d' % j] = j in comm
        fields['joint_%d_angle_limit' % j] = j in limit
    return types.SimpleNamespace(**fields)


mini_ros.BUS.clear()
mini_ros.BUS.subs.clear()
mini_ros.Node.publisher_counts = {'/joint_states': 2}
mini_ros.PARAM_OVERRIDES['harvest_sequencer'] = {'stub_step_duration': 0.0}
from piper_arm_ui.harvest_sequencer import HarvestSequencer  # noqa: E402

node = HarvestSequencer()


def arm_status_after(msg):
    node._on_arm_status(msg)
    node.tick(1)
    return json.loads(mini_ros.BUS.last(TOPIC_UI_STATE).data)['arm_status']


st = arm_status_after(status_msg())
check('全関節 False（実機の正常時）なら通信異常は無し',
      st['comm_error_joints'] == [], st['comm_error_joints'])
check('全関節 False なら可動限界も無し', st['angle_limit_joints'] == [], st['angle_limit_joints'])

st = arm_status_after(status_msg(comm=(2, 5)))
check('True の関節だけを通信異常として出す',
      st['comm_error_joints'] == [2, 5], st['comm_error_joints'])

st = arm_status_after(status_msg(limit=(3,)))
check('True の関節だけを可動限界として出す',
      st['angle_limit_joints'] == [3], st['angle_limit_joints'])
check('可動限界は通信異常に混ざらない', st['comm_error_joints'] == [], st['comm_error_joints'])

st = arm_status_after(status_msg(err=7))
check('エラーコードはそのまま載る', st['err_code'] == 7, st['err_code'])

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
