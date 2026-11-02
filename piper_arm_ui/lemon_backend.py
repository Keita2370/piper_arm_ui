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
    'grasp': 120.0,         # 開く＋差し込み（servo_timeout 実機 40 s）＋閉じる＋戻り
    'stop': 5.0, 'cancel_execute': 5.0, 'cancel_arm': 5.0,
}

# アームやグリッパが動く呼び出し。同時に1つしか動かさない（harvest_sequencer が見る）
MOTION = ('go_home', 'open', 'approach', 'grasp')


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
                           '/arm_controller/follow_joint_trajectory/_action/cancel_goal')):
            callers[key] = ServiceCaller(node, CancelGoal, name, callback_group, t[key],
                                         parse=_parse_cancel)
    except ImportError as exc:
        node.get_logger().warn('action_msgs が無いため、停止で軌道を取り消せません: %s' % exc)
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
