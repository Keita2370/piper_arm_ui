#!/usr/bin/env python3
"""URDF から組む順運動学と逆運動学.

座標指定でアームを動かすために要るのはこの 2 つ:

    fk_position(q)        joint1..6 の角度 → 手先の位置 [m]
    solve_ik(target, q0)  手先の位置 → joint1..6 の角度

piper_ros には /pos_cmd（手先姿勢を直接投げる）もあるが、ここでは
UI から見えている関節角と同じ経路（/joint_states）に落としたいので
自前で解いている。理由は 3 つ:

  - 実機が無い開発用PCでも座標指定が試せる（URDF があれば動く）
  - 既存の軌道生成（Trajectory）と可動域クランプがそのまま効く
  - UI が描いている 3D ビューと必ず同じ数値になる

【向きは解かない】
位置 3 自由度だけを合わせ、手先の向きは解に任せている。レモンを掴む
向きまで決めたいなら、ここに姿勢の残差を足して 6 次元にする
（Jacobian を 6xN にして角速度ぶんの行を足す）。

【座標系の注意】
URDF の rpy は固定軸 X→Y→Z の順。つまり R = Rz(yaw)Ry(pitch)Rx(roll)。
ブラウザ側（robotview.js）が three.js の 'ZYX' 指定を使っているのと
同じ規約で、ここを取り違えるとリンクの向きが静かにずれる。
"""

import os
import time
import xml.etree.ElementTree as ET

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:  # pragma: no cover - 実行環境依存
    NUMPY_AVAILABLE = False
    np = None

# 既定で動かす関節。グリッパ（joint7/8）は位置に効かないので入れない。
DEFAULT_ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')


def find_description_file(extra_paths=()):
    """piper_description の URDF を探す（ui_server と同じ探しかた）."""
    candidates = list(extra_paths)
    try:
        from ament_index_python.packages import get_package_share_directory
        urdf_dir = os.path.join(
            get_package_share_directory('piper_description'), 'urdf')
        candidates += [os.path.join(urdf_dir, 'piper_description.urdf'),
                       os.path.join(urdf_dir, 'piper_description_old.urdf')]
    except Exception:  # noqa: BLE001
        pass
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return ''


def _floats(text, default):
    if not text:
        return list(default)
    try:
        return [float(v) for v in text.replace(',', ' ').split()]
    except ValueError:
        return list(default)


def rpy_to_matrix(rpy):
    """固定軸 X→Y→Z の rpy を回転行列にする（URDF の規約）."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_rotation(axis, angle):
    """任意軸まわりの回転（Rodrigues）."""
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    k = np.asarray(axis, dtype=float) / norm
    kx, ky, kz = k
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


class ChainError(Exception):
    """チェーンを組めなかった（URDF が想定と違う）."""


class ArmChain:
    """URDF の base から手先までの 1 本のチェーン.

    URDF の <joint> は並び順がチェーン順とは限らないので、
    parent→child を辿って順番を決めている。
    """

    def __init__(self, joints, arm_joints, tcp_offset=(0.0, 0.0, 0.0),
                 tcp_link=''):
        self.joints = joints                 # チェーン順の dict リスト
        self.arm_joints = list(arm_joints)   # 動かす関節名（順番つき）
        self.tcp_offset = np.asarray(tcp_offset, dtype=float)
        self.tcp_link = tcp_link
        self.lower = np.array([j['lower'] for j in joints if j['movable']])
        self.upper = np.array([j['upper'] for j in joints if j['movable']])

        # 関節の原点（xyz/rpy）は姿勢によらず一定なので、4x4 にして持っておく。
        # IK は 1 回の呼び出しで数千回 FK を回すため、ここを毎回作り直すと
        # それだけで支配的なコストになる。
        for entry in joints:
            local = np.eye(4)
            local[:3, :3] = rpy_to_matrix(entry['rpy'])
            local[:3, 3] = entry['xyz']
            entry['_local'] = local
            axis = np.asarray(entry['axis'], dtype=float)
            norm = np.linalg.norm(axis)
            entry['_axis'] = axis / norm if norm > 1e-12 else np.array([0., 0., 1.])
        self._tool = np.eye(4)
        self._tool[:3, 3] = self.tcp_offset
        self._reach = None          # reach_estimate の遅延キャッシュ

    # ------------------------------------------------------------------ #
    @classmethod
    def from_urdf(cls, path, arm_joints=DEFAULT_ARM_JOINTS,
                  tcp_link='', tcp_offset=(0.0, 0.0, 0.0), limits=None):
        """URDF を読んでチェーンを組む.

        tcp_link が空なら、動かす最後の関節（既定では joint6）の子リンクを
        手先とみなす。グリッパの指の間を手先にしたいときは tcp_offset で
        ずらす（URDF には「指の間」を表すリンクが無いため）。

        limits に {関節名: (下限, 上限)} を渡すと、URDF の limit より
        そちらを優先する。UI 側の可動域（common.JOINT_LIMITS）と
        食い違ったままにしないため。
        """
        if not NUMPY_AVAILABLE:
            raise ChainError('numpy が無いため座標指定は使えません')
        if not path or not os.path.isfile(path):
            raise ChainError('URDF が見つかりません: %s' % (path or '(未指定)'))

        root = ET.parse(path).getroot()
        by_parent = {}
        all_joints = {}
        children = set()
        parents = set()
        for j in root.findall('joint'):
            name = j.get('name')
            parent_el, child_el = j.find('parent'), j.find('child')
            if parent_el is None or child_el is None:
                continue
            origin = j.find('origin')
            axis = j.find('axis')
            limit = j.find('limit')
            jtype = j.get('type', 'revolute')
            entry = {
                'name': name,
                'type': jtype,
                'parent': parent_el.get('link'),
                'child': child_el.get('link'),
                'xyz': _floats(origin.get('xyz') if origin is not None else None,
                               (0, 0, 0)),
                'rpy': _floats(origin.get('rpy') if origin is not None else None,
                               (0, 0, 0)),
                'axis': _floats(axis.get('xyz') if axis is not None else None,
                                (0, 0, 1)),
                'lower': float(limit.get('lower', -3.14)) if limit is not None else -3.14,
                'upper': float(limit.get('upper', 3.14)) if limit is not None else 3.14,
            }
            all_joints[name] = entry
            by_parent.setdefault(entry['parent'], []).append(entry)
            children.add(entry['child'])
            parents.add(entry['parent'])

        missing = [n for n in arm_joints if n not in all_joints]
        if missing:
            raise ChainError(
                'URDF に %s がありません（想定と違う URDF です）' % ', '.join(missing))

        # base リンク = 誰の子でもないリンク
        roots = [n for n in parents if n not in children]
        if not roots:
            raise ChainError('URDF の根リンクを決められません')
        base = roots[0]

        # 最後に動かす関節の子リンクを目的地にする
        goal_link = tcp_link or all_joints[arm_joints[-1]]['child']

        chain = cls._walk(by_parent, base, goal_link)
        if chain is None:
            raise ChainError('%s から %s までの経路がありません' % (base, goal_link))

        movable = set(arm_joints)
        for entry in chain:
            entry['movable'] = entry['name'] in movable
            if limits and entry['name'] in limits:
                lo, hi = limits[entry['name']]
                entry['lower'], entry['upper'] = float(lo), float(hi)

        found = [e['name'] for e in chain if e['movable']]
        if found != list(arm_joints):
            raise ChainError(
                '経路上の可動関節が想定と違います: %s（想定 %s）'
                % (found, list(arm_joints)))

        built = cls(chain, arm_joints, tcp_offset, goal_link)
        # 可動範囲の見積もりは起動時にやっておく。コマンド処理の最中に
        # 初回計算が走ると、そのぶん 50Hz の制御ループが止まる。
        built._reach = built.reach_estimate()
        return built

    @staticmethod
    def _walk(by_parent, link, goal):
        """link から goal までの関節列を深さ優先で探す."""
        if link == goal:
            return []
        for entry in by_parent.get(link, []):
            rest = ArmChain._walk(by_parent, entry['child'], goal)
            if rest is not None:
                return [dict(entry)] + rest
        return None

    # ------------------------------------------------------------------ #
    # 順運動学
    # ------------------------------------------------------------------ #
    def _walk_chain(self, q, collect_axes):
        """チェーンを 1 回たどる。FK とヤコビアンで共通の処理."""
        T = np.eye(4)
        axes, origins, kinds = [], [], []
        k = 0
        for entry in self.joints:
            T = T @ entry['_local']
            if not entry['movable']:
                continue
            angle = q[k] if k < len(q) else 0.0
            k += 1
            if collect_axes:
                axes.append(T[:3, :3] @ entry['_axis'])
                origins.append(T[:3, 3].copy())
                kinds.append(entry['type'])
            if entry['type'] == 'prismatic':
                slide = np.eye(4)
                slide[:3, 3] = entry['_axis'] * angle
                T = T @ slide
            elif entry['type'] != 'fixed':
                rot = np.eye(4)
                rot[:3, :3] = axis_rotation(entry['_axis'], angle)
                T = T @ rot
        return T @ self._tool, axes, origins, kinds

    def fk(self, q):
        """手先の 4x4 同次変換を返す."""
        return self._walk_chain(np.asarray(q, dtype=float), False)[0]

    def fk_position(self, q):
        """手先の位置 [m] を返す（base_link 基準）."""
        return self.fk(q)[:3, 3]

    # ------------------------------------------------------------------ #
    # ヤコビアン（位置 3 行ぶんだけ）
    # ------------------------------------------------------------------ #
    def fk_and_jacobian(self, q):
        """手先位置と 3xN ヤコビアンを 1 回のチェーン走査で出す.

        回転関節 i については、世界座標での回転軸 z_i と関節位置 p_i から
            J_i = z_i x (p_tcp - p_i)
        直動関節なら J_i = z_i。教科書どおりの式。

        IK のループでは位置とヤコビアンの両方が毎回要るので、
        別々に呼ぶとチェーンを 2 度たどることになる。ここで一度にまとめる。
        """
        T, axes, origins, kinds = self._walk_chain(
            np.asarray(q, dtype=float), True)
        tip = T[:3, 3]
        J = np.empty((3, len(axes)))
        for i, (z, p, kind) in enumerate(zip(axes, origins, kinds)):
            J[:, i] = z if kind == 'prismatic' else np.cross(z, tip - p)
        return tip, J

    def jacobian_position(self, q):
        """d(手先位置)/d(関節角) の 3xN 行列."""
        return self.fk_and_jacobian(q)[1]

    # ------------------------------------------------------------------ #
    # 逆運動学
    # ------------------------------------------------------------------ #
    def obviously_out_of_reach(self, target):
        """明らかに届かない目標か（安い判定）.

        IK は届かない目標に対して全回数ぶん粘るので、1 回 0.5 秒ほどかかる。
        シーケンサは単一スレッドで 50Hz の制御ループも回しているため、
        そこで止まると指令の publish が数十ms 飛ぶ。先に安く弾いておく。

        判定は可動範囲の見積もり（reach_estimate）を使う。内側に入って
        いても届かない姿勢はあるので、これは「確実に届かない」側だけを
        言う片側の判定。
        """
        if self._reach is None:
            self._reach = self.reach_estimate()
        target = np.asarray(target, dtype=float)
        if float(np.linalg.norm(target)) > self._reach['max_radius'] * 1.05:
            return True
        lo = np.asarray(self._reach['min'], dtype=float) - 0.05
        hi = np.asarray(self._reach['max'], dtype=float) + 0.05
        return bool(np.any(target < lo) or np.any(target > hi))

    def solve_ik(self, target, q0=None, tol=1e-4, max_iters=120,
                 max_step=0.4, restarts=12, seed=0, time_budget=0.6):
        """手先を target [m] に持っていく関節角を探す.

        減衰最小二乗を Levenberg-Marquardt のやりかたで解く:

            Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e

        λ を固定にすると、小さすぎれば特異姿勢で Δq が爆発し、
        大きすぎれば最後の詰めが進まない。そこで **1 歩ごとに λ を調整**する:

            誤差が減った  → 受け入れて λ を下げる（Gauss-Newton 寄り＝速い）
            誤差が増えた  → 棄却して λ を上げる（最急降下寄り＝確実）

        これを入れる前は到達可能な目標の 2.5% を取りこぼしていた。
        「入れた座標が到達可能なのに届かないと言われる」のは
        UI としては使えないので、ここは素直に受け入れ判定を入れている。

        それでも IK は局所解に落ちるため、収束しなければ初期姿勢を
        振り直して再挑戦する。1 回目は必ず現在姿勢（q0）から始める:
        座標ジョグのように「いまの姿勢のすぐ近く」を狙う使い方では、
        近い解が選ばれてアームが突然大きく跳ばないようにしたい。

        返り値: (関節角のリスト, 残差 [m], 収束したか)
                収束しなかった場合も「一番近づけた姿勢」を返すので、
                呼び出し側は残差を見て採否を決める。
        """
        target = np.asarray(target, dtype=float)
        n = len(self.arm_joints)
        start = np.zeros(n) if q0 is None else np.clip(
            np.asarray(q0, dtype=float)[:n], self.lower, self.upper)

        rng = np.random.default_rng(seed)
        best_q, best_err = start.copy(), np.inf
        eye3 = np.eye(3)
        # time_budget は「最悪でもここで打ち切る」ための保険であって、
        # 速くするための設定ではない。短くしすぎると、再挑戦が要る難しい
        # 目標（到達可能なのに局所解に落ちたもの）を取りこぼす。
        # 実測では 0.12 秒にすると到達可能な目標の 4% を落とし、
        # そのわりに合計時間はほとんど変わらなかった（1回 35ms 程度のため）。
        # 届かない目標は obviously_out_of_reach で先に弾いているので、
        # ここは余裕のある値にしてある。
        deadline = (time.monotonic() + time_budget) if time_budget else None

        for attempt in range(max(1, restarts)):
            if deadline is not None and time.monotonic() > deadline:
                break      # 時間切れ。いちばん近づけた姿勢を返す
            q = start.copy() if attempt == 0 else rng.uniform(self.lower, self.upper)
            tip, J = self.fk_and_jacobian(q)
            dist = float(np.linalg.norm(target - tip))
            lam = 0.05

            for _ in range(max_iters):
                if dist < best_err:
                    best_err, best_q = dist, q.copy()
                if dist < tol:
                    return [float(v) for v in q], dist, True

                err = target - tip
                try:
                    dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * eye3, err)
                except np.linalg.LinAlgError:       # pragma: no cover
                    break
                norm = float(np.linalg.norm(dq))
                if norm < 1e-12:                    # 完全な特異点。動けない
                    break
                if norm > max_step:
                    dq *= max_step / norm

                q_try = np.clip(q + dq, self.lower, self.upper)
                tip_try, J_try = self.fk_and_jacobian(q_try)
                dist_try = float(np.linalg.norm(target - tip_try))

                if dist_try < dist:
                    q, tip, J, dist = q_try, tip_try, J_try, dist_try
                    lam = max(1e-4, lam * 0.7)
                else:
                    lam *= 2.5
                    if lam > 1e3:                   # どう動かしても減らない
                        break

            if best_err < tol:
                return [float(v) for v in best_q], best_err, True

        return [float(v) for v in best_q], best_err, best_err < tol

    # ------------------------------------------------------------------ #
    def reach_estimate(self, samples=1500, seed=0):
        """だいたいの可動範囲を返す（UI の入力欄のヒント用）.

        解析的に出すのは面倒なので、可動域内の姿勢をばら撒いて
        手先位置の最小・最大を見る。起動時に 1 回だけ呼ぶ。
        """
        rng = np.random.default_rng(seed)
        qs = rng.uniform(self.lower, self.upper, size=(samples, len(self.lower)))
        pts = np.array([self.fk_position(q) for q in qs])
        return {
            'min': [round(float(v), 3) for v in pts.min(axis=0)],
            'max': [round(float(v), 3) for v in pts.max(axis=0)],
            'max_radius': round(float(np.linalg.norm(pts, axis=1).max()), 3),
        }
