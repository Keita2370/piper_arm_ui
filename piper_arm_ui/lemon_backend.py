"""チームの収穫システムのサービスを呼ぶ（段階3: 土台）.

harvest_sequencer の backend:=lemon で使う。UI のボタンは /arm_ui/command で
harvest_sequencer に届き、ここからチームのサービスを呼ぶ。

【なぜ別スレッドで待つのか】
rclpy では、コールバックの中で同期的にサービスの結果を待つと executor が止まり、
結果が永久に返らない（計画書 §3）。しかも harvest_sequencer の 50Hz の制御ループと
10Hz の状態配信が止まり、画面（3Dビュー・状態表示）が固まる。grasp は実機で
最大 40 秒かかる。

そこで:
  - クライアントは Reentrant のコールバックグループに入れる（応答の受け取りを、
    ほかのコールバックと並べて処理できるように）
  - 結果待ちは呼び出しごとの専用スレッドで行う。コマンドのコールバックはすぐ返る
  - 同じサービスの二重呼び出しはここで断る（相手も busy を返すが、こちらでも止める）

結果は on_done(success, message) で返す。呼び出し側（harvest_sequencer）は
画面のログと状態に載せる。
"""

import threading
import time


class ServiceCaller:
    """1 つのサービスを、固まらずに呼ぶ."""

    def __init__(self, node, srv_type, name, callback_group, timeout, parse=None):
        self.name = name
        self.timeout = float(timeout)
        # 応答 → (success, message)。std_srvs/Trigger 以外（CancelGoal）はここで直す
        self.parse = parse or (lambda res: (bool(res.success), str(res.message)))
        self.client = node.create_client(srv_type, name, callback_group=callback_group)
        self.srv_type = srv_type
        self.busy = False
        self.started = None         # 最後に呼び始めた時刻（停止との前後を比べる）
        self.last = None            # {'success', 'message', 't'}
        self._lock = threading.Lock()

    def call(self, request, on_done):
        """呼ぶ。すでに呼んでいる最中なら False を返して何もしない."""
        with self._lock:
            if self.busy:
                return False
            self.busy = True
            self.started = time.monotonic()
        threading.Thread(target=self._run, args=(request, on_done),
                         name='call ' + self.name, daemon=True).start()
        return True

    def _run(self, request, on_done):
        success, message = False, ''
        try:
            # 相手が居ないときに固まらないよう、居るかを先に短く確かめる
            if not self.client.wait_for_service(timeout_sec=2.0):
                success, message = False, '%s が見つかりません（ノードが起動していない）' % self.name
            else:
                future = self.client.call_async(request)
                done = threading.Event()
                future.add_done_callback(lambda _f: done.set())
                if not done.wait(self.timeout):
                    success, message = False, '%s が %.1f 秒で応答しません' % (self.name, self.timeout)
                else:
                    success, message = self.parse(future.result())
        except Exception as exc:  # noqa: BLE001
            success, message = False, '%s の呼び出しで例外: %s' % (self.name, exc)
        finally:
            self.last = {'success': success, 'message': message, 't': time.monotonic()}
            with self._lock:
                self.busy = False
        try:
            on_done(success, message)
        except Exception:  # noqa: BLE001
            pass

    def snapshot(self, now):
        last = self.last
        return {
            'busy': self.busy,
            'success': last['success'] if last else None,
            'message': last['message'] if last else None,
            'age': round(now - last['t'], 1) if last else None,
        }


class BasketMover(ServiceCaller):
    """収穫位置（記録した関節角）へ MoveIt で動かす.

    チームの moveit_helper と同じ形: /plan_kinematic_path で計画（最大 max_attempts 回）→
    /execute_trajectory で実行。実行を /execute_trajectory にしてあるので、
    「停止」（/execute_trajectory の全ゴールの取り消し）がそのまま効く。
    call(joints, on_done) の joints は joint1..joint6 [rad]。
    """

    JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def __init__(self, node, callback_group, timeout, group='arm', planning_time=5.0,
                 attempts=10, max_attempts=3, velocity=0.2, acceleration=0.3,
                 tolerance=0.01, name='収穫位置への移動'):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient
        self.name = name
        self.timeout = float(timeout)
        self.busy = False
        self.started = None
        self.last = None
        self._lock = threading.Lock()
        self.client = node.create_client(GetMotionPlan, '/plan_kinematic_path',
                                         callback_group=callback_group)
        self.execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory',
                                    callback_group=callback_group)
        self.GetMotionPlan = GetMotionPlan
        self.ExecuteTrajectory = ExecuteTrajectory
        # 速さはチームの実機の値（lemon_harvest.launch.py の REAL_MOVEIT_PARAMS）に合わせる
        self.group, self.planning_time = group, planning_time
        self.attempts, self.max_attempts = attempts, max_attempts
        self.velocity, self.acceleration, self.tolerance = velocity, acceleration, tolerance

    def _wait(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return future.result() if done.wait(timeout) else None

    def _run(self, joints, on_done):
        success, message = False, ''
        try:
            success, message = self._move(joints)
        except Exception as exc:  # noqa: BLE001
            success, message = False, '収穫位置への移動で例外: %s' % exc
        finally:
            self.last = {'success': success, 'message': message, 't': time.monotonic()}
            with self._lock:
                self.busy = False
        try:
            on_done(success, message)
        except Exception:  # noqa: BLE001
            pass

    def _move(self, joints):
        from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
        if not self.client.wait_for_service(timeout_sec=2.0):
            return False, '/plan_kinematic_path が見つかりません（move_group が起動していない）'

        req = self.GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = self.group
        r.start_state.is_diff = True
        c = Constraints()
        for name, pos in zip(self.JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.tolerance
            jc.tolerance_below = self.tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        r.num_planning_attempts = self.attempts
        r.allowed_planning_time = self.planning_time
        r.max_velocity_scaling_factor = self.velocity
        r.max_acceleration_scaling_factor = self.acceleration

        trajectory, code = None, None
        for _ in range(self.max_attempts):
            res = self._wait(self.client.call_async(req), self.planning_time + 5.0)
            if res is None:
                code = 'timeout'
                continue
            code = res.motion_plan_response.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                trajectory = res.motion_plan_response.trajectory
                break
        if trajectory is None:
            return False, 'unreachable after %d attempts (error_code=%s)' % (self.max_attempts, code)

        if not self.execute.wait_for_server(timeout_sec=2.0):
            return False, '/execute_trajectory が見つかりません'
        goal = self.ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self._wait(self.execute.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False, 'execution goal rejected'
        result = self._wait(handle.get_result_async(), self.timeout)
        if result is None:
            return False, 'execution timed out'
        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return False, 'execution failed (error_code=%s)' % code
        return True, 'reached'


class GripperMover(BasketMover):
    """グリッパを閉じる（lemon_grasp に閉じるサービスが無いので、コントローラへ直接）.

    /gripper_controller/follow_joint_trajectory に joint7 の目標を送る。
    開くほうは /lemon_grasp/open を使う（掴んだレモンを planning scene から外すため）。
    call(position, on_done) の position は joint7 [m]（0 で閉じる）。
    """

    def __init__(self, node, callback_group, timeout, joint='joint7', duration=1.0):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self.name = 'グリッパ'
        self.timeout = float(timeout)
        self.busy = False
        self.started = None
        self.last = None
        self._lock = threading.Lock()
        self.FollowJointTrajectory = FollowJointTrajectory
        self.action = ActionClient(node, FollowJointTrajectory,
                                   '/gripper_controller/follow_joint_trajectory',
                                   callback_group=callback_group)
        self.joint, self.duration = joint, duration

    def _move(self, position):
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint
        if not self.action.wait_for_server(timeout_sec=2.0):
            return False, '/gripper_controller/follow_joint_trajectory が見つかりません'
        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.joint]
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start = Duration(sec=int(self.duration),
                                      nanosec=int((self.duration % 1) * 1e9))
        goal.trajectory.points.append(pt)
        handle = self._wait(self.action.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False, 'gripper goal rejected'
        result = self._wait(handle.get_result_async(), self.timeout)
        if result is None:
            return False, 'gripper timed out'
        # レモンを挟むと閉じきらずに abort されることがある（lemon_grasp と同じ）。
        # 動いたのは確かなので、結果の種類も含めてそのまま出す
        code = result.result.error_code
        return True, 'closed' if code == 0 else 'closed (error_code=%s)' % code


def _parse_cancel(res):
    """action_msgs/CancelGoal の応答。空の要求＝全部のゴールを取り消す."""
    names = {0: 'ok', 1: 'rejected', 2: 'unknown goal', 3: 'goal terminated'}
    code = int(getattr(res, 'return_code', 0))
    n = len(getattr(res, 'goals_canceling', []) or [])
    if code == 0:
        return True, '%d 件を取り消し' % n
    # 取り消すゴールが無いときも rejected が返ることがある。止まっているので失敗ではない
    return code in (1, 3), '取り消すものなし（%s）' % names.get(code, code)


# サービスごとの待ち時間 [s]。動くものは実機の所要時間＋余裕
TIMEOUTS = {
    'clear_exclusions': 5.0, 'select': 5.0, 'release': 5.0,
    'go_home': 90.0,        # MoveIt の計画（最大3回）＋実行
    'open': 20.0,           # グリッパの軌道＋待ち
    'approach': 120.0,      # 差し込める向きの探索＋MoveIt の計画（最大3回）＋実行
    'basket': 90.0,         # 収穫位置への MoveIt の実行
    'joint_move': 30.0,     # 関節ジョグ・座標指定（MoveIt の計画＋実行）
    'gripper': 10.0,
    'cancel_gripper': 5.0,
    'grasp': 120.0,         # 開く＋差し込み（servo_timeout 実機 40 s）＋閉じる＋戻り
    'stop': 5.0, 'cancel_execute': 5.0, 'cancel_arm': 5.0,
}

# アームやグリッパが動く呼び出し。同時に1つしか動かさない（harvest_sequencer が見る）
MOTION = ('go_home', 'open', 'approach', 'grasp', 'basket', 'joint_move', 'gripper')


def make_callers(node, callback_group, timeouts=None):
    """チームのサービス（段階4まで）.

    lemon_msgs は ~/lemon_ws にある。開発用PCに無いときは select だけ使えない。
    """
    from std_srvs.srv import Trigger
    t = dict(TIMEOUTS, **(timeouts or {}))

    def trig(key, name):
        return ServiceCaller(node, Trigger, name, callback_group, t[key])

    callers = {
        'clear_exclusions': trig('clear_exclusions', '/lemon_target/clear_exclusions'),
        'release': trig('release', '/lemon_target/release'),
        'go_home': trig('go_home', '/arm_home/go_home'),
        'open': trig('open', '/lemon_grasp/open'),
        'stop': trig('stop', '/lemon_grasp/stop'),
        'approach': trig('approach', '/lemon_approach/approach'),
        'grasp': trig('grasp', '/lemon_grasp/grasp'),
    }
    # 停止で、MoveIt の実行（home / approach）とコントローラに渡った軌道を取り消す。
    # 空の要求は、送り手が誰でも全部のゴールを取り消す（ROS 2 のアクションの決まり）
    try:
        from action_msgs.srv import CancelGoal
        for key, name in (('cancel_execute', '/execute_trajectory/_action/cancel_goal'),
                          ('cancel_arm',
                           '/arm_controller/follow_joint_trajectory/_action/cancel_goal'),
                          ('cancel_gripper',
                           '/gripper_controller/follow_joint_trajectory/_action/cancel_goal')):
            callers[key] = ServiceCaller(node, CancelGoal, name, callback_group, t[key],
                                         parse=_parse_cancel)
    except ImportError as exc:
        node.get_logger().warn('action_msgs が無いため、停止で軌道を取り消せません: %s' % exc)
    try:
        callers['basket'] = BasketMover(node, callback_group, t['basket'])
        callers['joint_move'] = BasketMover(node, callback_group, t['joint_move'],
                                            name='関節の移動')
    except ImportError as exc:
        node.get_logger().warn('moveit_msgs が無いため、収穫位置・ジョグ・座標指定は使えません: %s'
                               % exc)
    try:
        callers['gripper'] = GripperMover(node, callback_group, t['gripper'])
    except ImportError as exc:
        node.get_logger().warn('control_msgs が無いため、グリッパを閉じられません: %s' % exc)
    try:
        from lemon_msgs.srv import SelectTarget
        callers['select'] = ServiceCaller(
            node, SelectTarget, '/lemon_target/select', callback_group,
            t['select'])
    except ImportError as exc:
        node.get_logger().warn(
            'lemon_msgs/srv/SelectTarget が見つからないため、対象の選択は使えません'
            '（~/lemon_ws を source してください）: %s' % exc)
    return callers
