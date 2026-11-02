#!/usr/bin/env python3
"""収穫シーケンサノード.

スライド4「アーム専用UI」の操作列
    把持対象の選択 → 把持（把持姿勢の計算）→ 把持 → 収穫位置(①②③) → 開放（収穫完了）
を状態機械として実装する。

【この骨組みの考え方】
- UI との通信は rosbridge 経由。JSON を std_msgs/String で運ぶので、
  UI 側もこのノードもトピック定義の追加ビルドなしに拡張できる。
- 「把持姿勢の計算」だけは実装方式（MoveIt / 独自IK / 学習ベース）が未定なので、
  _compute_grasp_pose() を差し替えポイントとして切り出してある。
  現状はカメラ座標の簡易マッピングによるスタブ。
- 実際のアーム駆動は piper_ros の /joint_states に関節角を流す方式。
  既存の joy_to_piper_jointstates と同時に publish すると競合するため、
  enable_arm_command が true のときだけ publish する。
"""

import json
import math
import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from piper_arm_ui.common import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    JOINT_LIMITS,
    JOINT_NAMES,
    STATE_AT_BASKET,
    STATE_DONE,
    STATE_ERROR,
    STATE_GRASPED,
    STATE_GRASPING,
    STATE_IDLE,
    STATE_LABELS,
    STATE_MOVING_TO_BASKET,
    STATE_PLANNED,
    STATE_PLANNING,
    STATE_RELEASING,
    STATE_SELECTED,
    STATE_SELECTING,
    TOPIC_DETECTIONS,
    TOPIC_TEAM_APPROACH_STATUS,
    TOPIC_TEAM_GRASP_STATUS,
    TOPIC_TEAM_MANAGER_STATUS,
    TOPIC_TEAM_TARGET,
    TOPIC_UI_COMMAND,
    TOPIC_UI_LOG,
    TOPIC_UI_STATE,
    clamp_pose,
)
from piper_arm_ui.kinematics import ArmChain, ChainError, find_description_file
from piper_arm_ui.team_status import TeamStatus

# backend:=lemon で受け付けるコマンド（段階5まで）。それ以外（収穫位置・ジョグ・
# 座標指定）は理由を出して何もしない。内部の状態機械を間違って動かさないため
#   start_selection → 対象を解除 / plan_grasp → 把持姿勢へ移動 / grasp → 把持 /
#   home → home へ / release → 開放 / abort → 停止 /
#   goto_basket → 記録した収穫位置へ / set_basket → いまの姿勢を収穫位置として記録 /
#   jog・jog_xyz・move_to_xyz → 実機の関節角を基準に MoveIt で動かす / gripper → 開閉
LEMON_COMMANDS = ('set_enable', 'select_target', 'clear_exclusions',
                  'start_selection', 'plan_grasp', 'grasp', 'home', 'release', 'abort',
                  'goto_basket', 'set_basket', 'jog', 'jog_xyz', 'move_to_xyz', 'gripper')

# approach が済んだとみなす対象と、いまの対象が同じかの判定 [m]
SAME_TARGET = 0.005

# lemon のジョグ（関節・座標）は、前のジョグの指令からこの秒数より早く来たら無視する。
# 画面の長押しの繰り返しが止まらなくなり、動き終わるたびに次を受け付けて
# 動き続けたことがある（実機）。人が押す間隔はこれより長い
JOG_MIN_INTERVAL = 0.5


class Trajectory:
    """関節空間の線形補間（S字加減速つき）で目標姿勢まで動かす簡易軌道."""

    def __init__(self, start, goal, duration):
        self.start = list(start)
        self.goal = list(goal)
        self.duration = max(0.05, float(duration))
        self.t0 = time.monotonic()

    @property
    def progress(self):
        return min(1.0, (time.monotonic() - self.t0) / self.duration)

    @property
    def finished(self):
        return self.progress >= 1.0

    def sample(self):
        s = self.progress
        # smoothstep で急加速を避ける
        s = s * s * (3.0 - 2.0 * s)
        return [a + (b - a) * s for a, b in zip(self.start, self.goal)]


class HarvestSequencer(Node):

    def __init__(self):
        super().__init__('harvest_sequencer')

        # internal: 自前の状態機械＋軌道＋/joint_states（実機なしの開発用、従来どおり）
        # lemon:    チームの収穫システムのサービスを呼ぶ（ミニPC。/joint_states は使わない）
        self.declare_parameter('backend', 'internal')
        # lemon の収穫位置①②③の記録（実機で置いて記録した関節角）。basket_store.py
        self.declare_parameter('basket_file', '~/.ros/piper_arm_ui/basket_poses.json')
        self.declare_parameter('enable_arm_command', False)
        self.declare_parameter('joint_command_topic', '/joint_states')
        self.declare_parameter('joint_feedback_topic', '/joint_states_single')
        self.declare_parameter('end_pose_topic', '/end_pose')
        self.declare_parameter('arm_status_topic', '/arm_status')
        self.declare_parameter('state_publish_rate', 10.0)
        self.declare_parameter('stub_step_duration', 1.5)
        # piper 側の全体動作速度 [%]。0 なら指定せず piper の既定(100%)に任せる。
        # デモで穏やかに動かしたいときは 20〜40 くらいにする。
        self.declare_parameter('motion_speed', 0)
        # グリッパ把持力。piper 側で 0.5〜3 にクリップされ ×1000 される。
        # 0 のままだと 0.5 に丸められて弱いので、piper の既定値と同じ 1.0 にしておく。
        self.declare_parameter('gripper_effort', 1.0)
        # 収穫かごの位置。joint1（旋回）が置き場の方位をそのまま決める。
        #   ① +2.094 rad (+120°) = 左後方
        #   ② -2.094 rad (-120°) = 右後方
        #   ③ +1.047 rad ( +60°) = 左斜め前
        # joint2 以降は腕の伸ばし方で、①② は距離 0.265m、③ は 0.190m。
        # 置き場を動かすときは joint1 だけ変えればよい（方位＝joint1 になる）。
        self.declare_parameter('basket_pose_1', [2.094, 1.20, -1.00, 0.0, 0.40, 0.0])
        self.declare_parameter('basket_pose_2', [-2.094, 1.20, -1.00, 0.0, 0.40, 0.0])
        self.declare_parameter('basket_pose_3', [1.047, 0.90, -0.70, 0.0, 0.30, 0.0])
        self.declare_parameter('home_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # --- 座標指定（手先位置）で動かすための設定 ---
        # 空なら piper_description の URDF を自動で探す。
        # URDF が見つからなければ座標指定だけが無効になり、
        # 関節角指定（ジョグ）と収穫シーケンスはそのまま使える。
        self.declare_parameter('robot_description_file', '')
        # 手先として扱うリンク。空なら joint6 の子リンク。
        self.declare_parameter('tcp_link', '')
        # 手先リンクからのオフセット [m]。URDF には「グリッパの指の間」を
        # 表すリンクが無いので、実測して入れる。たとえばグリッパ先端が
        # link6 の +Z に 0.15m なら [0.0, 0.0, 0.15]。
        # 合わせかたは README の「座標指定を実機に合わせる」を参照。
        self.declare_parameter('tcp_offset', [0.0, 0.0, 0.0])
        # 座標指定で 1 回に動かせる距離の上限 [m]。
        # 入力ミス（単位を mm で入れた等）で大きく飛ぶのを防ぐ。
        self.declare_parameter('max_cartesian_step', 0.25)

        # --- チームの収穫システム（段階1：購読して表示するだけ）---
        # ここで購読するだけでは相手のノードに何も起きない。サービスは呼ばない
        self.declare_parameter('team_manager_status_topic', TOPIC_TEAM_MANAGER_STATUS)
        self.declare_parameter('team_approach_status_topic', TOPIC_TEAM_APPROACH_STATUS)
        self.declare_parameter('team_grasp_status_topic', TOPIC_TEAM_GRASP_STATUS)
        self.declare_parameter('team_target_topic', TOPIC_TEAM_TARGET)

        p = self.get_parameter
        self.backend = str(p('backend').value or 'internal').lower()
        if self.backend not in ('internal', 'lemon'):
            raise ValueError('backend は internal か lemon: %s' % self.backend)
        # lemon では /joint_states に何も出さないので、関節指令は常に無効
        self.enable_arm_command = (bool(p('enable_arm_command').value)
                                   if self.backend == 'internal' else False)
        # lemon の「UI操作を有効化」= サービスを呼んでよいか（計画書 §5）。既定は OFF
        self.service_enabled = False
        # approach が成功したときの対象の位置（base_link）。grasp はこれが今の対象と
        # 同じときだけ呼ぶ。grasp は「いまの姿勢から」差し込むので、pregrasp に居ないまま
        # 呼ぶと、ありえない向きから差し込みに行く
        self.approached = None
        # 最後に「停止」を押した時刻。そのあとに返ってきた失敗は「停止で止めた」と出す
        self._stop_at = None
        self._last_jog_at = None        # 最後にジョグの指令を受け取った時刻
        self._jog_ignored = 0
        self.step_duration = float(p('stub_step_duration').value)
        self.motion_speed = max(0, min(100, int(p('motion_speed').value)))
        self.gripper_effort = float(p('gripper_effort').value)
        self.basket_poses = [
            clamp_pose(p('basket_pose_1').value),
            clamp_pose(p('basket_pose_2').value),
            clamp_pose(p('basket_pose_3').value),
        ]
        self.home_pose = clamp_pose(p('home_pose').value)
        self.max_cartesian_step = abs(float(p('max_cartesian_step').value))

        # --- 運動学（座標指定用）---
        self.chain = None
        self.chain_error = ''
        self._setup_kinematics(
            str(p('robot_description_file').value or ''),
            str(p('tcp_link').value or ''),
            [float(v) for v in (p('tcp_offset').value or [0, 0, 0])])

        # --- 内部状態 ---
        self.state = STATE_IDLE
        self.prev_state = STATE_IDLE
        self.status_message = 'システム待機中'
        self.cmd_pose = list(self.home_pose)      # 指令している joint1..6
        self.gripper = GRIPPER_OPEN               # グリッパ開度
        self.fb_pose = [0.0] * 6                  # 実機フィードバック
        self.fb_gripper = 0.0
        self.fb_stamp = 0.0
        self.arm_status = {}
        self.end_pose = {}
        self.trajectory = None
        self.after_trajectory = None               # 軌道完了後に遷移する状態
        self.detections = []                       # 認識されたレモン
        self.selected_id = None
        self.basket_index = None
        self.harvest_count = 0
        self.grasp_plan = None
        # 指令トピックを他のノードが握っている（= ros2_control 構成）。
        # このときは UI から関節指令を出さない
        self._command_topic_taken = False
        self.team = TeamStatus()

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # --- publisher ---
        # lemon では /joint_states の publisher そのものを作らない。この構成では
        # /joint_states は joint_state_broadcaster の出力で、混ぜると現在姿勢が壊れる
        # （ワークスペースの CLAUDE.md §4）。作らなければ間違えて出すことも無い
        self.joint_pub = (self.create_publisher(
            JointState, p('joint_command_topic').value, 10)
            if self.backend == 'internal' else None)
        self.state_pub = self.create_publisher(String, TOPIC_UI_STATE, qos)
        self.log_pub = self.create_publisher(String, TOPIC_UI_LOG, qos)

        # --- subscriber ---
        self.create_subscription(String, TOPIC_UI_COMMAND, self._on_command, qos)
        self.create_subscription(String, TOPIC_DETECTIONS, self._on_detections, qos)
        self.create_subscription(
            JointState, p('joint_feedback_topic').value, self._on_joint_feedback, 10)
        self._subscribe_optional_piper_topics(
            p('arm_status_topic').value, p('end_pose_topic').value)
        self._subscribe_team_topics(qos)

        # --- timer ---
        self.create_timer(0.02, self._control_loop)          # 50 Hz: 軌道生成 + 指令
        rate = max(1.0, float(p('state_publish_rate').value))
        self.create_timer(1.0 / rate, self._publish_state)   # UI への状態配信

        # 二重起動の検出用。PID を混ぜて他のインスタンスと必ず違う値にする
        self._instance_id = '%s#%d' % (self.get_name(), os.getpid())
        # 起動直後は相手を見つけられないので、少し待ってから1回だけ確かめる
        self._dup_timer = self.create_timer(5.0, self._check_duplicates)

        # --- チームのサービス（backend:=lemon）---
        # クライアントだけ Reentrant のグループにし、結果は別スレッドで待つ
        # （lemon_backend.py）。ほかのコールバックは既定のグループのまま順に処理する
        self.callers = {}
        self.baskets = None
        if self.backend == 'lemon':
            from piper_arm_ui.basket_store import BasketStore
            self.baskets = BasketStore(str(p('basket_file').value))
            if self.baskets.error:
                self.get_logger().warn(self.baskets.error)
            from piper_arm_ui.lemon_backend import make_callers
            self._service_group = ReentrantCallbackGroup()
            self.callers = make_callers(self, self._service_group)
        # 相手のノードが居るかを数える。manager は auto_run:=false では居ないのが
        # 正しいので、「何も来ない」と「居ない」を画面で区別できるようにする
        self.create_timer(1.0, self._check_team_presence)

        self.get_logger().info(
            'harvest_sequencer started (enable_arm_command=%s)' % self.enable_arm_command)
        self._log('シーケンサ起動', 'info')

    def _setup_kinematics(self, path, tcp_link, tcp_offset):
        """URDF から運動学チェーンを組む（座標指定に使う）.

        失敗しても起動は続ける。座標指定だけが使えなくなり、理由は
        /arm_ui/state に載せて UI 側に出す（黙って効かないのが一番困る）。
        """
        found = path or find_description_file()
        if not found:
            self.chain_error = (
                'piper_description の URDF が見つかりません。'
                'robot_description_file で明示するか piper_description をビルドしてください')
            self.get_logger().warn('座標指定は使えません: %s' % self.chain_error)
            return
        try:
            # 可動域は UI と同じ common.JOINT_LIMITS を使う。URDF の limit と
            # 微妙に違う（joint6 が 2.094 と 2.0944）ので、揃えておかないと
            # 「UI では入る角度が IK では出ない」というずれが起きる。
            self.chain = ArmChain.from_urdf(
                found, tcp_link=tcp_link, tcp_offset=tcp_offset,
                limits=JOINT_LIMITS)
        except ChainError as exc:
            self.chain_error = str(exc)
            self.get_logger().warn('座標指定は使えません: %s' % exc)
            return
        except Exception as exc:  # noqa: BLE001
            self.chain_error = 'URDF の解析に失敗しました: %s' % exc
            self.get_logger().warn('座標指定は使えません: %s' % exc)
            return

        reach = self.chain._reach
        self.get_logger().info(
            '座標指定を有効にしました（手先リンク %s / オフセット %s / '
            '可動半径 約%.2fm、URDF %s）'
            % (self.chain.tcp_link, tcp_offset, reach['max_radius'],
               os.path.basename(found)))
        if not any(tcp_offset):
            self.get_logger().info(
                'tcp_offset が 0 のため、手先の基準は %s の原点です。'
                'グリッパの指の間を基準にしたいときは tcp_offset を実測して'
                '設定してください' % self.chain.tcp_link)

    def tcp_position(self, pose=None):
        """いまの指令姿勢での手先位置 [m]。チェーンが無ければ None."""
        if self.chain is None:
            return None
        try:
            return [round(float(v), 4) for v in
                    self.chain.fk_position(pose if pose is not None else self.cmd_pose)]
        except Exception:  # noqa: BLE001
            return None

    def _check_duplicates(self):
        """同じトピックに publish している別のシーケンサがいないか確かめる.

        前回の launch のノードが残っていると、UI は2つ分の状態を交互に受け取り、
        「計画あり／なし」やボタンの有効・無効が高速に入れ替わってチカチカする。
        原因が分かりにくいので、起動時に気づけるようにしておく。
        """
        self._dup_timer.cancel()
        try:
            count = self.count_publishers(TOPIC_UI_STATE)
        except Exception:  # noqa: BLE001
            return
        if count > 1:
            msg = ('%s に publisher が %d 個あります。'
                   'harvest_sequencer が二重起動しています' % (TOPIC_UI_STATE, count))
            self.get_logger().error(
                msg + '\n'
                '    画面がチカチカする原因になります。前回の起動が残っていないか確認してください:\n'
                '    ros2 node list | grep harvest_sequencer\n'
                '    pkill -f harvest_sequencer   （そのあと launch し直す）')
            self._log(msg, 'error')

        self._check_command_topic_owner()

    def _check_command_topic_owner(self):
        """指令トピックを他の誰かが publish していないか確かめる.

        【なぜこの確認が要るか】
        本家 piper_ros では、start_single_piper.launch.py が
        joint_ctrl_single → /joint_states にリマップしているので、
        /joint_states に publish すればアームが動いた。

        ros2_control を使う構成（topic_based_ros2_control + arm_controller）では
        **向きが逆**で、/joint_states は joint_state_broadcaster の「出力」になる。
        そこへこのノードが publish すると:

          - アームは動かない（指令はもう別の経路）
          - robot_state_publisher / MoveIt / Servo が読む「現在の関節角」に
            指令値が混ざる。TF が実機と指令の間で揺れ、**Servo が誤った
            現在姿勢を元に動く**ので危険

        この取り違えは画面を見ても分からないので、他に publisher がいたら
        指令を止めて理由を出す。判断に迷うときは止める側に倒す。
        """
        if self.joint_pub is None:      # lemon: そもそも publish しない
            return
        topic = self.joint_pub.topic_name
        try:
            count = self.count_publishers(topic)
        except Exception:  # noqa: BLE001
            return
        # 自分を除いて誰かいるか
        if count <= 1:
            return

        self._command_topic_taken = True
        was_enabled = self.enable_arm_command
        self.enable_arm_command = False

        msg = ('%s を他のノードも publish しています（publisher %d 個）。'
               'UI からの関節指令を停止しました' % (topic, count))
        self.get_logger().error(
            msg + '\n'
            '    ros2_control を使う構成では /joint_states は「出力」です。\n'
            '    UI が publish すると、アームは動かないうえに\n'
            '    MoveIt / Servo が読む現在姿勢が壊れます。\n'
            '    確認: ros2 topic info %s --verbose\n'
            '    指令の送り先を joint_command_topic で変えるか、\n'
            '    UI を監視のみで使ってください。' % topic)
        self._log(msg, 'error')
        if was_enabled:
            self._log('安全のため「UI操作」を無効にしました', 'warn')

    # ------------------------------------------------------------------ #
    # piper_msgs は環境によっては未ビルドなので、失敗しても落とさない
    # ------------------------------------------------------------------ #
    def _subscribe_optional_piper_topics(self, status_topic, end_pose_topic):
        try:
            from piper_msgs.msg import PiperStatusMsg
            self.create_subscription(
                PiperStatusMsg, status_topic, self._on_arm_status, 10)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                'piper_msgs が見つからないため %s の購読をスキップ (%s)' % (status_topic, exc))
        try:
            from geometry_msgs.msg import Pose
            self.create_subscription(Pose, end_pose_topic, self._on_end_pose, 10)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('end_pose の購読をスキップ (%s)' % exc)

    def _subscribe_team_topics(self, qos):
        """チームの収穫システムの status と対象を購読する."""
        p = self.get_parameter
        self._team_topics = {
            'manager': str(p('team_manager_status_topic').value),
            'approach': str(p('team_approach_status_topic').value),
            'grasp': str(p('team_grasp_status_topic').value),
        }
        self._team_target_topic = str(p('team_target_topic').value)
        for source, topic in self._team_topics.items():
            self.create_subscription(
                String, topic,
                lambda msg, s=source: self._on_team_status(s, msg), qos)
        # 相手は TRANSIENT_LOCAL で出している。VOLATILE で購読すると
        # 起動前に固定された対象が届かない
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            PointStamped, self._team_target_topic, self._on_team_target, latched)

    def _check_team_presence(self):
        for source, topic in self._team_topics.items():
            try:
                self.team.present[source] = self.count_publishers(topic) > 0
            except Exception:  # noqa: BLE001
                self.team.present[source] = None
        try:
            self.team.present_target = self.count_publishers(self._team_target_topic) > 0
        except Exception:  # noqa: BLE001
            self.team.present_target = None

    # ------------------------------------------------------------------ #
    # コールバック
    # ------------------------------------------------------------------ #
    def _on_joint_feedback(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES[:6]):
            if name in msg.name:
                self.fb_pose[i] = msg.position[msg.name.index(name)]
        # piper_ctrl_single_node は /joint_states_single を
        # ['joint1'..'joint6', 'gripper'] という名前で出す（joint7 ではない）
        for gripper_name in ('gripper', 'joint7'):
            if gripper_name in msg.name:
                self.fb_gripper = msg.position[msg.name.index(gripper_name)]
                break
        self.fb_stamp = time.monotonic()

    def _on_arm_status(self, msg):
        """piper_msgs/PiperStatusMsg を UI 向けに整形する."""
        limits = [i + 1 for i in range(6)
                  if getattr(msg, 'joint_%d_angle_limit' % (i + 1), False)]
        # communication_status_joint_N は「通信が異常か」で、True が異常
        # （piper_sdk の定義「True为通信异常」）。名前から「正常なら True」と読むと
        # 逆になり、正常なのに全関節を異常と表示していた（実機で確認）
        comm_ng = [i + 1 for i in range(6)
                   if getattr(msg, 'communication_status_joint_%d' % (i + 1), False)]
        self.arm_status = {
            'ctrl_mode': int(getattr(msg, 'ctrl_mode', 0)),
            'arm_status': int(getattr(msg, 'arm_status', 0)),
            'mode_feedback': int(getattr(msg, 'mode_feedback', 0)),
            'motion_status': int(getattr(msg, 'motion_status', 0)),
            'teach_status': int(getattr(msg, 'teach_status', 0)),
            'err_code': int(getattr(msg, 'err_code', 0)),
            'angle_limit_joints': limits,
            'comm_error_joints': comm_ng,
        }

    def _on_end_pose(self, msg):
        self.end_pose = {
            'x': round(msg.position.x, 4),
            'y': round(msg.position.y, 4),
            'z': round(msg.position.z, 4),
        }

    def _on_team_status(self, source, msg):
        changed = self.team.on_status(source, msg.data, time.monotonic())
        if changed:
            kind, label = changed
            names = {'manager': 'マネージャ', 'approach': '接近', 'grasp': '把持'}
            # missed は異常ではないので warn（error にしない）
            level = {'fail': 'error', 'missed': 'warn', 'stopped': 'warn'}.get(kind, 'info')
            self._log('[%s] %s' % (names.get(source, source), label), level)

    def _on_team_target(self, msg):
        h = msg.header
        pt = msg.point
        if self.team.on_target(h.frame_id, pt.x, pt.y, pt.z, time.monotonic()):
            if h.frame_id:
                self._log('[対象] 固定 (%.3f, %.3f, %.3f) %s'
                          % (pt.x, pt.y, pt.z, h.frame_id), 'info')
            else:
                self._log('[対象] 解除', 'info')

    def _on_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.detections = data.get('targets', [])
        # 選択中の対象が消えたら選択解除
        if self.selected_id is not None:
            if not any(d.get('id') == self.selected_id for d in self.detections):
                if self.state in (STATE_SELECTING, STATE_SELECTED):
                    self.selected_id = None
                    self._set_state(STATE_SELECTING, '対象を見失いました')

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('不正なコマンド: %s' % msg.data)
            return
        name = cmd.get('cmd', '')
        if self.backend == 'lemon':
            if name not in LEMON_COMMANDS:
                self._log('lemon 構成では「%s」はまだ使えません（段階4以降。'
                          'アームは動かしません）' % name, 'warn')
                return
            handler = getattr(self, '_lemon_' + name, None)
        else:
            handler = getattr(self, '_cmd_' + name, None)
        if handler is None:
            self.get_logger().warn('未知のコマンド: %s' % name)
            return
        try:
            handler(cmd)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('コマンド %s の処理で例外: %s' % (name, exc))
            self._set_state(STATE_ERROR, '内部エラー: %s' % exc)

    # ------------------------------------------------------------------ #
    # コマンド実装（UI のボタン 1 つ = 1 コマンド）
    # ------------------------------------------------------------------ #
    def _cmd_set_enable(self, cmd):
        """UI からのアーム指令 publish を有効／無効にする."""
        want = bool(cmd.get('value', False))
        if want and self._command_topic_taken:
            # 画面のトグルで戻せてしまうと安全弁の意味がない
            self._log('%s は他のノードが publish しているため、'
                      'UI操作は有効にできません' % self.joint_pub.topic_name, 'warn')
            self.enable_arm_command = False
            return
        self.enable_arm_command = want
        self._log('UI操作を%s' % ('有効化' if self.enable_arm_command else '無効化'), 'info')

    # ------------------------------------------------------------------ #
    # backend:=lemon のコマンド（チームのサービスを呼ぶ）
    # ------------------------------------------------------------------ #
    def _lemon_set_enable(self, cmd):
        """lemon の「UI操作を有効化」: サービスを呼んでよいかの元栓."""
        self.service_enabled = bool(cmd.get('value', False))
        self._log('UI操作を%s（チームのサービスを%s）'
                  % (('有効化', '呼びます') if self.service_enabled
                     else ('無効化', '呼びません')), 'info')

    def _service_gate(self, label):
        if not self.service_enabled:
            self._log('「UI操作を有効化」が OFF なので%sはしません' % label, 'warn')
            return False
        return True

    def _call(self, key, request, label, on_ok=None, on_done=None):
        """チームのサービスを呼ぶ。結果は画面のログに出す（すぐ返る）.

        on_ok は成功したとき、on_done は成功・失敗にかかわらず結果が返ったときに呼ぶ。
        """
        from piper_arm_ui.lemon_backend import MOTION
        caller = self.callers.get(key)
        if caller is None:
            self._log('%s: サービスが使えません（lemon_msgs が無い？）' % label, 'error')
            return
        def done(success, message):
            text = message or ('成功' if success else '失敗')
            level = 'info' if success else 'warn'
            stopped = (not success and key in MOTION and self._stop_at is not None
                       and caller.started is not None and self._stop_at >= caller.started)
            if stopped:
                # 人が止めた。失敗ではない（段階4の実機確認で「execution failed」と出て紛らわしかった）
                text = '停止で止めました（%s）' % text
            elif key == 'grasp':
                from piper_arm_ui.team_status import classify
                kind, desc = classify('grasp', message)
                text = desc
                level = {'ok': 'info', 'missed': 'warn'}.get(kind, 'warn')
            self._log('%s: %s' % (label, text), level)
            if on_done:
                on_done(success, message)
            if success and on_ok:
                on_ok(message)
        if not caller.call(request, done):
            self._log('%s: 前の呼び出しがまだ終わっていません' % label, 'warn')

    def _motion_gate(self, label):
        """アームやグリッパが動く呼び出しの前に確かめる.

        - 元栓（UI操作を有効化）が ON
        - ほかの動く呼び出しが動いていない（相手のノードは自分の busy しか見ないので、
          home と open を同時に呼べてしまう）
        - auto_run:=true のマネージャが居ない（2 つが同時に指令すると危ない）
        """
        if not self._service_gate(label):
            return False
        busy = self.motion_busy()
        if busy:
            self._log('%s: いま「%s」が動いているので呼びません' % (label, busy), 'warn')
            return False
        if self.team.present.get('manager'):
            self._log('%s: lemon_harvest_manager（auto_run:=true）が動いているので、'
                      'UI からはアームを動かしません' % label, 'warn')
            return False
        return True

    def motion_busy(self):
        """動いている呼び出しの名前（無ければ None）."""
        from piper_arm_ui.lemon_backend import MOTION
        for key in MOTION:
            c = self.callers.get(key)
            if c is not None and c.busy:
                return key
        return None

    def _current_target(self):
        """lemon_target が固定している対象の位置（無ければ None）."""
        tg = self.team.target
        if tg and tg.get('frame_id'):
            return (tg['x'], tg['y'], tg['z'])
        return None

    def _approached_current(self):
        """いまの対象に対して approach が済んでいるか."""
        cur = self._current_target()
        if self.approached is None or cur is None:
            return False
        return max(abs(a - b) for a, b in zip(self.approached, cur)) < SAME_TARGET

    def _lemon_plan_grasp(self, _cmd):
        """「把持姿勢へ移動」: /lemon_approach/approach（★アームが大きく動く）."""
        if not self._motion_gate('把持姿勢へ移動'):
            return
        target = self._current_target()
        if target is None:
            self._log('把持姿勢へ移動: 先にレモンの枠をクリックして対象を選んでください', 'warn')
            return
        from std_srvs.srv import Trigger
        self.approached = None

        def ok(_message):
            # 返ってくるまでに対象が変わっていたら、済んだことにしない
            if self._current_target() == target:
                self.approached = target
        self._log('把持姿勢へ移動します（対象 %.3f, %.3f, %.3f）' % target, 'info')
        self._call('approach', Trigger.Request(), '把持姿勢へ移動', on_ok=ok)

    def _lemon_grasp(self, _cmd):
        """「把持」: /lemon_grasp/grasp（★アームが大きく動く。実機で最大 40 s）.

        同じ対象への approach が成功した直後だけ呼ぶ（理由は self.approached の注記）。
        掴めても掴めなくても、1 回やったら approach からやり直す。
        """
        if not self._motion_gate('把持'):
            return
        if not self._approached_current():
            self._log('把持: 先に「把持姿勢へ移動」をしてください（同じ対象への接近が'
                      '済んだ直後だけ押せます）', 'warn')
            return
        from std_srvs.srv import Trigger
        self.approached = None
        self._log('把持を始めます（差し込み中は「停止」で止められます）', 'info')
        self._call('grasp', Trigger.Request(), '把持')

    def _lemon_home(self, _cmd):
        """「ホームへ戻す」: /arm_home/go_home（★アームが動く）."""
        if not self._motion_gate('ホームへ戻す'):
            return
        self.approached = None          # pregrasp から離れる
        from std_srvs.srv import Trigger
        self._log('ホームへ移動します', 'info')
        self._call('go_home', Trigger.Request(), 'ホームへ戻す')

    def _lemon_release(self, _cmd):
        """「開放」: グリッパを開いてから対象を解除する（★グリッパが動く）.

        相手の manager と同じ順（opening → releasing）。開けなかったら対象は残す
        （まだ掴んでいるかもしれないので、何が起きたか人が見て決める）。
        """
        if not self._motion_gate('開放'):
            return
        from std_srvs.srv import Trigger
        self.approached = None

        def then_release(_message):
            self._call('release', Trigger.Request(), '対象を解除')
        self._call('open', Trigger.Request(), 'グリッパを開く', on_ok=then_release)

    def _lemon_start_selection(self, _cmd):
        """「対象を解除」（internal の「把持対象の選択」）: /lemon_target/release.

        アームは動かない。ただし動作中に解除すると、差し込み中の把持が中断される
        （相手の PoseTracking が止まる）ので、動いている間は断る。止めるなら「停止」。
        """
        if not self._service_gate('対象の解除'):
            return
        busy = self.motion_busy()
        if busy:
            self._log('対象の解除: いま「%s」が動いています。止めるなら「停止」を'
                      % busy, 'warn')
            return
        self.approached = None
        from std_srvs.srv import Trigger
        self._call('release', Trigger.Request(), '対象を解除')

    def _lemon_abort(self, _cmd):
        """「停止」（赤）: ソフト停止。元栓が OFF でも、何が動いていても必ず呼ぶ.

        同時に呼ぶ:
          - /lemon_grasp/stop（差し込み・閉じる・戻り）
          - /execute_trajectory の全ゴールの取り消し（home / approach）
          - /arm_controller/follow_joint_trajectory の全ゴールの取り消し
        **非常停止ではない**（ソフトが止まっていれば効かない）。画面にもそう出している
        """
        self._stop_at = time.monotonic()
        self.approached = None          # 止めた場所はどこか分からないので、やり直す
        self._log('停止を要求しました（ソフト停止。非常停止ではありません）', 'warn')
        from std_srvs.srv import Trigger
        self._call('stop', Trigger.Request(), '把持の停止')
        try:
            from action_msgs.srv import CancelGoal
        except ImportError:
            return
        self._call('cancel_execute', CancelGoal.Request(), 'MoveIt の実行の取り消し')
        self._call('cancel_arm', CancelGoal.Request(), 'アームの軌道の取り消し')

    def _lemon_goto_basket(self, cmd):
        """「収穫位置 ①②③」: 記録した姿勢へ MoveIt で動かす（★アームが大きく動く）.

        記録していない位置へは動かさない（basket_store.py の注記）。
        着いたら「開放」で離す。
        """
        index = int(cmd.get('index', 0))
        label = '収穫位置%sへ移動' % '①②③'[index - 1] if 1 <= index <= 3 else '収穫位置へ移動'
        if not 1 <= index <= 3:
            self._log('収穫位置は 1〜3 で指定してください', 'warn')
            return
        joints = self.baskets.get(index) if self.baskets else None
        if joints is None:
            self._log('%s: まだ記録されていません。アームをかごの上に置いて'
                      '「いまの姿勢を記録」してください' % label, 'warn')
            return
        if not self._motion_gate(label):
            return
        self.approached = None

        def ok(_message):
            self.basket_index = index
        self.basket_index = None
        self._log('%sします' % label, 'info')
        self._call('basket', joints, label, on_ok=ok)

    def _lemon_set_basket(self, cmd):
        """「いまの姿勢を①にする」: 実機の関節角を収穫位置として記録する（動かない）."""
        index = int(cmd.get('index', 0))
        if not 1 <= index <= 3:
            self._log('収穫位置は 1〜3 で指定してください', 'warn')
            return
        mark = '①②③'[index - 1]
        if not self._service_gate('収穫位置%sの記録' % mark):
            return
        busy = self.motion_busy()
        if busy:
            self._log('収穫位置%sの記録: いま「%s」が動いています。止まってから記録してください'
                      % (mark, busy), 'warn')
            return
        fb_age = time.monotonic() - self.fb_stamp if self.fb_stamp else None
        if fb_age is None or fb_age > 0.5:
            self._log('収穫位置%sの記録: 実機の関節角が届いていません' % mark, 'error')
            return
        try:
            item = self.baskets.set(index, self.fb_pose)
        except (OSError, ValueError) as exc:
            self._log('収穫位置%sを記録できません: %s' % (mark, exc), 'error')
            return
        self._log('収穫位置%sを記録しました（%s）' % (
            mark, ', '.join('%.3f' % v for v in item['joints'])), 'info')

    # ---- ジョグ・座標指定・グリッパ（lemon）------------------------------
    # /joint_states には出さず、実機の関節角を基準にした目標を MoveIt で計画して動かす
    # （lemon_backend.BasketMover）。1 回ごとに計画するので、押してから動くまで少しかかる。
    def _jog_too_fast(self, label):
        """ジョグの指令が続けざまに来ていないか（来ていたら無視する）."""
        now = time.monotonic()
        last, self._last_jog_at = self._last_jog_at, now
        if last is not None and now - last < JOG_MIN_INTERVAL:
            self._jog_ignored += 1
            if self._jog_ignored in (1, 10, 100):
                self._log('%s: 続けざまの入力を無視しました（%d 回）。1 回押すと 1 ステップ'
                          '動きます' % (label, self._jog_ignored), 'warn')
            return True
        self._jog_ignored = 0
        return False

    def _fresh_joints(self, label):
        """実機の関節角（0.5 s 以内に届いたもの）。無ければ理由を出して None."""
        fb_age = time.monotonic() - self.fb_stamp if self.fb_stamp else None
        if fb_age is None or fb_age > 0.5:
            self._log('%s: 実機の関節角が届いていません' % label, 'error')
            return None
        return list(self.fb_pose)

    def _move_joints(self, pose, label):
        self.approached = None          # pregrasp から離れる
        self.basket_index = None
        self._call('joint_move', [float(v) for v in pose], label)

    def _lemon_jog(self, cmd):
        """関節ジョグ: 実機の関節角 ＋ delta へ MoveIt で動かす（★アームが動く）."""
        idx = int(cmd.get('joint', 0))
        delta = float(cmd.get('delta', 0.0))
        if not 0 <= idx < 6 or delta == 0.0 or not math.isfinite(delta):
            return
        label = '関節ジョグ J%d' % (idx + 1)
        if self._jog_too_fast(label):
            return
        if not self._motion_gate(label):
            return
        here = self._fresh_joints(label)
        if here is None:
            return
        pose = list(here)
        pose[idx] += delta
        pose = clamp_pose(pose)
        if abs(pose[idx] - here[idx]) < 1e-4:
            self._log('%s: 可動域の端です' % label, 'warn')
            return
        self._move_joints(pose, label)

    def _lemon_move_to_xyz(self, cmd):
        """座標指定: 手先を x, y, z [m]（base_link）へ（★アームが動く）."""
        try:
            target = [float(cmd['x']), float(cmd['y']), float(cmd['z'])]
        except (KeyError, TypeError, ValueError):
            self._log('座標指定: x, y, z を数値で指定してください', 'warn')
            return
        if not all(math.isfinite(v) for v in target):
            self._log('座標指定: x, y, z を数値で指定してください', 'warn')
            return
        self._lemon_move_tcp(target, '座標指定')

    def _lemon_jog_xyz(self, cmd):
        """座標ジョグ: 手先を x/y/z のどれかの向きに少しずらす（★アームが動く）."""
        axis = int(cmd.get('axis', 0))
        delta = float(cmd.get('delta', 0.0))
        if not 0 <= axis < 3 or delta == 0.0 or not math.isfinite(delta):
            return
        if self._jog_too_fast('座標ジョグ'):
            return
        # 元栓・排他を先に見る（ほかの動くコマンドと同じ順。理由の出かたをそろえる）
        if not self._motion_gate('座標ジョグ'):
            return
        if self.chain is None:
            self._log('座標指定は使えません: %s' % self.chain_error, 'warn')
            return
        here = self._fresh_joints('座標ジョグ')
        if here is None:
            return
        target = list(self.chain.fk_position(here))
        target[axis] += delta
        self._lemon_move_tcp(target, '座標ジョグ', here, gated=True)

    def _lemon_move_tcp(self, target, label, here=None, gated=False):
        if not gated and not self._motion_gate(label):
            return
        if here is None:
            here = self._fresh_joints(label)
            if here is None:
                return
        pose = self._solve_to(target, label, base=here)
        if pose is None:
            return
        self._move_joints(pose, '%s (%.3f, %.3f, %.3f)' % ((label,) + tuple(target)))

    def _lemon_gripper(self, cmd):
        """グリッパの開閉（★グリッパが動く）.

        開: /lemon_grasp/open（掴んだレモンを planning scene から外すのも相手がする）
        閉: グリッパのコントローラへ直接（lemon_grasp に閉じるサービスが無い）
        """
        close = bool(cmd.get('close'))
        label = 'グリッパを%s' % ('閉じる' if close else '開く')
        if not self._motion_gate(label):
            return
        from std_srvs.srv import Trigger
        if close:
            self._call('gripper', GRIPPER_CLOSED, label)
        else:
            self._call('open', Trigger.Request(), label)

    def _lemon_clear_exclusions(self, _cmd):
        """「除外をクリア」ボタン: /lemon_target/clear_exclusions（アームは動かない）."""
        if not self._service_gate('除外のクリア'):
            return
        from std_srvs.srv import Trigger
        self._call('clear_exclusions', Trigger.Request(), '除外をクリア')

    def _lemon_select_target(self, cmd):
        """検出枠のクリック: /lemon_target/select（アームは動かない）.

        lemon_detections が各検出に載せているカメラ座標系の位置をそのまま渡す。
        base_link への変換は lemon_target がする（TF を持っているのは相手）。
        """
        if not self._service_gate('対象の選択'):
            return
        target_id = cmd.get('id')
        det = next((d for d in self.detections if d.get('id') == target_id), None)
        if det is None:
            self._log('ID %s は現在の検出結果にありません' % target_id, 'warn')
            return
        xyz, frame = det.get('xyz'), det.get('frame_id')
        if not xyz or not frame:
            self._log('検出 #%s に位置が載っていません（lemon_detections が古い？）'
                      % target_id, 'warn')
            return
        from lemon_msgs.srv import SelectTarget
        req = SelectTarget.Request()
        req.point.header.frame_id = str(frame)
        req.point.point.x, req.point.point.y, req.point.point.z = (float(v) for v in xyz)
        self._call('select', req, '対象 #%s を選択' % target_id)

    def _cmd_start_selection(self, _cmd):
        """「把持対象の選択」ボタン: 選択モードに入る."""
        self.selected_id = None
        self.grasp_plan = None
        self._set_state(STATE_SELECTING, 'カメラ画像上のレモンを選択してください')

    def _cmd_select_target(self, cmd):
        """カメラ画像上のレモンをクリックしたとき."""
        target_id = cmd.get('id')
        if not any(d.get('id') == target_id for d in self.detections):
            self._log('ID %s は現在の検出結果にありません' % target_id, 'warn')
            return
        self.selected_id = target_id
        self.grasp_plan = None
        self._set_state(STATE_SELECTED, '対象 #%s を選択しました' % target_id)

    def _cmd_plan_grasp(self, _cmd):
        """「把持（把持姿勢の計算）」ボタン."""
        if self.selected_id is None:
            self._log('先に把持対象を選択してください', 'warn')
            return
        target = self._selected_target()
        self._set_state(STATE_PLANNING, '把持姿勢を計算中...')
        self.grasp_plan = self._compute_grasp_pose(target)
        if self.grasp_plan is None:
            self._set_state(STATE_ERROR, '把持姿勢を計算できませんでした')
            return
        # 計算に要する時間を模擬しつつ、計算結果の姿勢へ接近する
        self._start_trajectory(self.grasp_plan, self.step_duration, STATE_PLANNED,
                               '把持姿勢へ接近しました')

    def _cmd_grasp(self, _cmd):
        """「把持」ボタン: グリッパを閉じる."""
        if self.state not in (STATE_PLANNED, STATE_SELECTED):
            self._log('把持姿勢の計算後に実行してください', 'warn')
            return
        self._set_state(STATE_GRASPING, 'グリッパを閉じています')
        self.gripper = GRIPPER_CLOSED
        self._start_trajectory(self.cmd_pose, 0.8, STATE_GRASPED, '把持しました')

    def _cmd_goto_basket(self, cmd):
        """「収穫位置（①②③）」ボタン."""
        index = int(cmd.get('index', 1))
        if not 1 <= index <= 3:
            self._log('収穫位置は 1〜3 で指定してください', 'warn')
            return
        self.basket_index = index
        self._set_state(STATE_MOVING_TO_BASKET, '収穫位置 %d へ移動中' % index)
        self._start_trajectory(self.basket_poses[index - 1], self.step_duration * 1.6,
                               STATE_AT_BASKET, '収穫位置 %d に到達' % index)

    def _cmd_release(self, _cmd):
        """「開放（収穫完了）」ボタン."""
        self._set_state(STATE_RELEASING, 'グリッパを開いています')
        self.gripper = GRIPPER_OPEN
        self._start_trajectory(self.cmd_pose, 0.8, STATE_DONE, '収穫完了')

    def _cmd_home(self, _cmd):
        """ホーム姿勢へ戻す."""
        self.gripper = GRIPPER_OPEN
        self._set_state(STATE_MOVING_TO_BASKET, 'ホーム姿勢へ移動中')
        self._start_trajectory(self.home_pose, self.step_duration * 1.6,
                               STATE_IDLE, 'ホーム姿勢に戻りました')

    def _cmd_abort(self, _cmd):
        """非常停止: 軌道生成を止め、現在角度で保持する."""
        self.trajectory = None
        self.after_trajectory = None
        self._set_state(STATE_IDLE, '停止しました')
        self._log('ユーザ操作により停止', 'warn')

    def _cmd_jog(self, cmd):
        """手動ジョグ（UI の関節微調整）."""
        idx = int(cmd.get('joint', 0))
        delta = float(cmd.get('delta', 0.0))
        if not 0 <= idx < 6:
            return
        self.trajectory = None
        pose = list(self.cmd_pose)
        pose[idx] += delta
        self.cmd_pose = clamp_pose(pose)

    def _cmd_gripper(self, cmd):
        """グリッパ開閉のみ."""
        self.gripper = GRIPPER_CLOSED if cmd.get('close') else GRIPPER_OPEN

    # ------------------------------------------------------------------ #
    # 座標指定（手先位置）
    # ------------------------------------------------------------------ #
    def _solve_to(self, target, label, base=None):
        """手先を target [m] へ持っていく関節角を求める.

        返り値は関節角のリスト。無理なときは None を返し、
        理由を UI のログに出す（画面に出ないまま効かないのを避ける）。
        """
        if self.chain is None:
            self._log('座標指定は使えません: %s' % self.chain_error, 'warn')
            return None

        # 可動範囲の外かを先に見る。そちらのほうが具体的な診断なので、
        # 「離れすぎです」より先に出したい（遠い目標は両方に引っかかる）。
        if self.chain.obviously_out_of_reach(target):
            # アームの何倍も先の値なら、たいてい mm で入力している。
            # 「可動範囲の外」だけ出すより、単位を名指ししたほうが早く直せる。
            hint = ('。単位は m です（mm で入れていませんか）'
                    if max(abs(v) for v in target) > 2.0 else '')
            self._log('%s: (%.3f, %.3f, %.3f) は可動範囲の外です%s'
                      % (label, target[0], target[1], target[2], hint), 'warn')
            return None

        # 基準の関節角。internal は指令値、lemon は実機の関節角（指令値は使っていない）
        base = list(base) if base is not None else self.cmd_pose
        here = self.chain.fk_position(base)
        move = float(((target[0] - here[0]) ** 2 + (target[1] - here[1]) ** 2
                      + (target[2] - here[2]) ** 2) ** 0.5)
        if move > self.max_cartesian_step:
            # 単位の入れ違い（mm で入力した等）で大きく飛ぶのを止める。
            self._log(
                '%s: 現在位置から %.0fmm 離れています（1回の上限 %.0fmm）。'
                '単位は m です。近い位置から少しずつ動かしてください'
                % (label, move * 1000, self.max_cartesian_step * 1000), 'warn')
            return None

        pose, err, ok = self.chain.solve_ik(target, q0=base)
        if not ok:
            self._log(
                '%s: (%.3f, %.3f, %.3f) に届きませんでした'
                '（最も近い姿勢でも %.0fmm のずれ）'
                % (label, target[0], target[1], target[2], err * 1000), 'warn')
            return None
        return clamp_pose(pose)

    def _cmd_move_to_xyz(self, cmd):
        """「この位置へ移動」ボタン: 手先を指定座標へ動かす.

        base_link 基準の x, y, z [m]。関節空間の補間で動くので、
        途中の手先の軌跡は直線にはならない（枝の回避もしない）。
        デモの位置決め用で、経路が要るなら MoveIt2 に置き換える。
        """
        try:
            target = [float(cmd['x']), float(cmd['y']), float(cmd['z'])]
        except (KeyError, TypeError, ValueError):
            self._log('座標指定: x, y, z を数値で指定してください', 'warn')
            return
        pose = self._solve_to(target, '座標指定')
        if pose is None:
            return
        self._set_state(STATE_MOVING_TO_BASKET,
                        '座標 (%.3f, %.3f, %.3f) へ移動中' % tuple(target))
        self._start_trajectory(
            pose, self.step_duration, STATE_IDLE,
            '座標 (%.3f, %.3f, %.3f) に到達' % tuple(target))

    def _cmd_jog_xyz(self, cmd):
        """座標ジョグ: 手先を x/y/z のどれかの向きに少しずらす.

        関節ジョグ（_cmd_jog）と同じ感覚で使えるように、軌道は挟まず
        指令姿勢を直接置き換える。1 回 1cm 程度なら IK は数 ms で解ける。
        """
        axis = int(cmd.get('axis', 0))
        delta = float(cmd.get('delta', 0.0))
        if not 0 <= axis < 3 or delta == 0.0:
            return
        if self.chain is None:
            self._log('座標指定は使えません: %s' % self.chain_error, 'warn')
            return
        target = list(self.chain.fk_position(self.cmd_pose))
        target[axis] += delta
        pose = self._solve_to(target, '座標ジョグ')
        if pose is None:
            return
        self.trajectory = None
        self.after_trajectory = None
        self.cmd_pose = pose

    # ------------------------------------------------------------------ #
    # 把持姿勢の計算（★差し替えポイント★）
    # ------------------------------------------------------------------ #
    def _compute_grasp_pose(self, target):
        """検出結果から joint1..6 の把持姿勢を求める.

        現状はカメラ画像上の正規化座標と推定距離から関節角へ線形写像するスタブ。
        本実装では以下のいずれかに置き換える想定:
          - MoveIt2 (MoveGroupInterface / moveit_py) で IK + 経路計画
          - piper_ros の /pos_cmd (PosCmd) にエンドエフェクタ姿勢を投げる
          - 学習ベースの把持姿勢推定 (GraspNet 等)
        target は dummy_perception / 実認識ノードが出す辞書:
          {"id": 1, "u": 0.42, "v": 0.55, "distance": 0.35, "confidence": 0.9}
        """
        if target is None:
            return None
        u = float(target.get('u', 0.5))          # 画像内 x [0,1]
        v = float(target.get('v', 0.5))          # 画像内 y [0,1]
        dist = float(target.get('distance', 0.35))  # 距離 [m]

        # 画像中心からのずれ → 旋回角 / 俯仰角へ、距離 → 伸ばし具合へ
        yaw = (0.5 - u) * 1.2                     # joint1: 左右
        pitch = (v - 0.5) * 0.8                   # 上下方向の狙い
        reach = max(0.0, min(1.0, (dist - 0.15) / 0.45))

        pose = [
            yaw,                                  # joint1 旋回
            0.55 + 0.85 * reach + pitch * 0.4,    # joint2 肩
            -0.35 - 0.95 * reach,                 # joint3 肘
            0.0,                                  # joint4 手首回転
            0.25 + pitch,                         # joint5 手首俯仰
            yaw * 0.3,                            # joint6 手先回転
        ]
        return clamp_pose(pose)

    # ------------------------------------------------------------------ #
    # 状態機械 / 制御ループ
    # ------------------------------------------------------------------ #
    def _start_trajectory(self, goal, duration, next_state, message):
        self.trajectory = Trajectory(self.cmd_pose, clamp_pose(goal), duration)
        self.after_trajectory = (next_state, message)

    def _control_loop(self):
        if self.trajectory is not None:
            self.cmd_pose = self.trajectory.sample()
            if self.trajectory.finished:
                self.trajectory = None
                if self.after_trajectory:
                    next_state, message = self.after_trajectory
                    self.after_trajectory = None
                    if next_state == STATE_DONE:
                        self.harvest_count += 1
                    self._set_state(next_state, message)

        if self.enable_arm_command and self.joint_pub is not None:
            self._publish_joint_command()

    def _publish_joint_command(self):
        """piper_ctrl_single_node が期待する形で関節指令を publish する.

        piper 側の joint_callback の仕様に合わせてある:
          - joint1..joint6 は「名前」で引かれる
          - グリッパ開度は position[6]
          - velocity の長さがちょうど 7 のときだけ velocity[6] が
            全体の動作速度(1..100%)として使われる。それ以外は 100% 扱い
          - effort[6] がグリッパの把持力（0.5〜3 にクリップされて ×1000）
        name / position は 8 個のまま出す。/joint_states は
        robot_state_publisher も見ており、joint8 が無いと RViz の
        グリッパ片側が描画されないため。
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.cmd_pose) + [self.gripper, -self.gripper]
        if self.motion_speed > 0:
            msg.velocity = [0.0] * 6 + [float(self.motion_speed)]
        else:
            msg.velocity = [0.0] * len(JOINT_NAMES)   # 全ゼロ = piper 側で 100%
        msg.effort = [0.0] * 6 + [float(self.gripper_effort)] + [0.0]
        self.joint_pub.publish(msg)

    def _set_state(self, state, message=''):
        self.prev_state = self.state
        self.state = state
        if message:
            self.status_message = message
            level = 'error' if state == STATE_ERROR else 'info'
            self._log(message, level)

    def _selected_target(self):
        for d in self.detections:
            if d.get('id') == self.selected_id:
                return d
        return None

    def _log(self, text, level='info'):
        msg = String()
        msg.data = json.dumps(
            {'t': time.time(), 'level': level, 'text': text}, ensure_ascii=False)
        self.log_pub.publish(msg)

    def _publish_state(self):
        fb_age = time.monotonic() - self.fb_stamp if self.fb_stamp else None
        payload = {
            # このノードの識別子。同じトピックに2つ以上のシーケンサが
            # publish していると UI 側が状態の食い違いに気づけるようにする
            # （＝画面が高速に切り替わってチカチカする状態の自動検出）。
            'src': self._instance_id,
            'state': self.state,
            'state_label': STATE_LABELS.get(self.state, self.state),
            'message': self.status_message,
            # lemon では「UI操作を有効化」のトグルはサービスの元栓（service_enabled）
            'enable_arm_command': (self.service_enabled if self.backend == 'lemon'
                                   else self.enable_arm_command),
            'backend': self.backend,
            'progress': round(self.trajectory.progress, 3) if self.trajectory else None,
            'selected_id': self.selected_id,
            'basket_index': self.basket_index,
            'harvest_count': self.harvest_count,
            'gripper': round(self.gripper, 4),
            'gripper_closed': self.gripper <= GRIPPER_CLOSED + 1e-6,
            # lemon では指令値を持たないので、実機の関節角を出す（関節の表・3Dビュー用）
            'command_joints': [round(v, 4) for v in
                               (self.fb_pose if self.backend == 'lemon' else self.cmd_pose)],
            'feedback_joints': [round(v, 4) for v in self.fb_pose],
            'feedback_gripper': round(self.fb_gripper, 4),
            'feedback_age': round(fb_age, 2) if fb_age is not None else None,
            'arm_connected': fb_age is not None and fb_age < 1.0,
            'arm_status': self.arm_status,
            'motion_speed': self.motion_speed,
            # 収穫位置とホームの関節角。UI 側はこれを順運動学にかけて
            # 「ロボットの状態」パネルの ①②③ を実際の位置に描く
            # （固定座標で描くと、姿勢を調整したときに絵とずれる）。
            'basket_poses': [[round(v, 4) for v in pose] for pose in self.basket_poses],
            'home_pose': [round(v, 4) for v in self.home_pose],
            'end_pose': self.end_pose,
            # 座標指定用。tcp は「いま指令している関節角を順運動学にかけた
            # 手先位置」で、UI の入力欄と同じ基準になる。piper が出す
            # end_pose とは基準点が違うことがあるので、別の項目にしてある。
            'tcp': self.tcp_position(self.fb_pose if self.backend == 'lemon' else None),
            'tcp_available': self.chain is not None,
            'tcp_reason': self.chain_error,
            'tcp_link': self.chain.tcp_link if self.chain else '',
            'tcp_reach': self.chain._reach if self.chain else None,
            'max_cartesian_step': self.max_cartesian_step,
            # 指令トピックを他のノードが握っているか（ros2_control 構成の検出）。
            # true なら UI からアームは動かせない
            'command_topic_taken': self._command_topic_taken,
            'joint_command_topic': self.joint_pub.topic_name if self.joint_pub else '',
            'grasp_plan': [round(v, 4) for v in self.grasp_plan] if self.grasp_plan else None,
            'detections': self.detections,
            # チームの収穫システムが言っていること（段階1：表示だけ）
            'team': self.team.snapshot(time.monotonic()),
            # チームのサービスの呼び出し（busy / 最後の結果）。lemon のときだけ中身がある
            'calls': {k: c.snapshot(time.monotonic()) for k, c in self.callers.items()},
            # アームやグリッパが動いている呼び出し（UI はその間、動くボタンを押せなくする）
            'motion_busy': self.motion_busy() if self.callers else None,
            # いまの対象への approach が済んでいる（＝「把持」を押してよい）
            'approached': self._approached_current() if self.callers else False,
            # 収穫位置①②③の記録（lemon）。記録していない位置へは動かさない
            'baskets': self.baskets.snapshot() if self.baskets else None,
            'stamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HarvestSequencer()
    # 複数スレッドで回す。既存のコールバック（タイマー・コマンド・購読）は既定の
    # グループ（同時に1つ）のままなので、順番と排他は単一スレッドのときと変わらない。
    # サービスの応答だけが Reentrant のグループで並行に受け取られる（lemon_backend.py）
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
