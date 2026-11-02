"""運動学（kinematics.py）の検証.

ここが間違っていると、座標を入れてもアームが別の場所へ行く。
目で見て気づけない種類のバグなので、数値で押さえる。

確かめること:
  1. 回転の規約が URDF と合っているか（固定軸 X→Y→Z）
     → scipy の extrinsic 'xyz' と一致するか、という独立な照合
  2. 順運動学が独立実装と一致するか（scipy で組み直して比較）
  3. ヤコビアンが数値微分と一致するか
  4. IK の往復: ランダムな姿勢 → その手先位置 → IK → 同じ位置に戻るか
  5. 可動域を絶対に出ないか
  6. 届かない目標をちゃんと「届かない」と言うか
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                    # noqa: E402
from scipy.spatial.transform import Rotation          # noqa: E402

from piper_arm_ui import kinematics as K              # noqa: E402

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_arm.urdf')
FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


# --------------------------------------------------------------------- #
# 1. 回転の規約
# --------------------------------------------------------------------- #
rng = np.random.default_rng(7)
worst = 0.0
for _ in range(500):
    rpy = rng.uniform(-np.pi, np.pi, 3)
    mine = K.rpy_to_matrix(rpy)
    # URDF の rpy は固定軸（extrinsic）XYZ。scipy では小文字 'xyz'。
    theirs = Rotation.from_euler('xyz', rpy).as_matrix()
    worst = max(worst, float(np.abs(mine - theirs).max()))
check('rpy が固定軸XYZ（scipy の extrinsic xyz と一致）', worst < 1e-12,
      'max 差 %.2e' % worst)

# 取り違えたときは本当に違う値になる、という対照実験。
# （intrinsic ZYX と一致してしまうなら上のテストは何も守っていない）
rpy = np.array([0.3, -0.7, 1.1])
intrinsic = Rotation.from_euler('ZYX', rpy).as_matrix()
check('規約を取り違えれば差が出る（テストが効いている）',
      float(np.abs(K.rpy_to_matrix(rpy) - intrinsic).max()) > 0.1)

worst_axis = 0.0
for _ in range(300):
    axis = rng.normal(size=3)
    angle = rng.uniform(-np.pi, np.pi)
    mine = K.axis_rotation(axis, angle)
    theirs = Rotation.from_rotvec(angle * axis / np.linalg.norm(axis)).as_matrix()
    worst_axis = max(worst_axis, float(np.abs(mine - theirs).max()))
check('任意軸回転が scipy と一致', worst_axis < 1e-12, 'max 差 %.2e' % worst_axis)

# --------------------------------------------------------------------- #
# 2. チェーンの組み立てと順運動学
# --------------------------------------------------------------------- #
chain = K.ArmChain.from_urdf(URDF)
check('URDF からチェーンを組める', chain is not None)
check('可動関節は joint1..6', chain.arm_joints == list(K.DEFAULT_ARM_JOINTS),
      chain.arm_joints)
check('手先リンクを自動で決める（joint6 の子）', chain.tcp_link == 'link6',
      chain.tcp_link)
check('固定関節もチェーンに入る', len(chain.joints) == 6,
      [j['name'] for j in chain.joints])
check('可動域を URDF から読む',
      abs(chain.lower[0] + 2.618) < 1e-9 and abs(chain.upper[1] - 3.14) < 1e-9,
      (list(chain.lower), list(chain.upper)))


def fk_reference(urdf_joints, q):
    """scipy で組み直した順運動学（独立実装）."""
    T = np.eye(4)
    k = 0
    for entry in urdf_joints:
        local = np.eye(4)
        local[:3, :3] = Rotation.from_euler('xyz', entry['rpy']).as_matrix()
        local[:3, 3] = entry['xyz']
        T = T @ local
        if entry['movable']:
            axis = np.asarray(entry['axis'], float)
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(
                q[k] * axis / np.linalg.norm(axis)).as_matrix()
            T = T @ rot
            k += 1
    return T


worst_fk = 0.0
for _ in range(400):
    q = rng.uniform(chain.lower, chain.upper)
    worst_fk = max(worst_fk, float(np.abs(
        chain.fk(q) - fk_reference(chain.joints, q)).max()))
check('順運動学が独立実装と一致', worst_fk < 1e-12, 'max 差 %.2e m' % worst_fk)

check('ゼロ姿勢の手先が妥当な位置にある',
      np.linalg.norm(chain.fk_position(np.zeros(6))) > 0.1,
      [round(v, 4) for v in chain.fk_position(np.zeros(6))])

# 手先オフセットがそのぶんだけ動かすこと（手先リンクの座標系で）
offset_chain = K.ArmChain.from_urdf(URDF, tcp_offset=(0.0, 0.0, 0.05))
q = rng.uniform(chain.lower, chain.upper)
delta = offset_chain.fk_position(q) - chain.fk_position(q)
expected = chain.fk(q)[:3, :3] @ np.array([0.0, 0.0, 0.05])
check('tcp_offset が手先座標系でそのぶん動かす',
      float(np.abs(delta - expected).max()) < 1e-12,
      'max 差 %.2e' % float(np.abs(delta - expected).max()))

# 末端の固定関節まで含めた指定もできること
tip = K.ArmChain.from_urdf(URDF, tcp_link='gripper_base')
check('tcp_link で末端の固定リンクまで伸ばせる',
      float(np.linalg.norm(tip.fk_position(np.zeros(6))
                           - chain.fk_position(np.zeros(6))) - 0.02) < 1e-9)

# --------------------------------------------------------------------- #
# 3. ヤコビアン vs 数値微分
# --------------------------------------------------------------------- #
worst_j = 0.0
for _ in range(100):
    q = rng.uniform(chain.lower * 0.8, chain.upper * 0.8)
    J = chain.jacobian_position(q)
    num = np.zeros((3, 6))
    h = 1e-7
    for i in range(6):
        qp, qm = q.copy(), q.copy()
        qp[i] += h
        qm[i] -= h
        num[:, i] = (chain.fk_position(qp) - chain.fk_position(qm)) / (2 * h)
    worst_j = max(worst_j, float(np.abs(J - num).max()))
check('ヤコビアンが数値微分と一致', worst_j < 1e-6, 'max 差 %.2e' % worst_j)

# --------------------------------------------------------------------- #
# 4. IK の往復
# --------------------------------------------------------------------- #
solved_err, missed = [], []
trials = 200
for _ in range(trials):
    q_true = rng.uniform(chain.lower, chain.upper)
    target = chain.fk_position(q_true)
    # わざと違う姿勢から解き始める（答えを教えない）
    q0 = rng.uniform(chain.lower, chain.upper)
    q_ik, err, ok = chain.solve_ik(target, q0=q0)
    reached = float(np.linalg.norm(chain.fk_position(q_ik) - target))
    (solved_err if ok else missed).append(reached)

solved = len(solved_err)
check('IK が到達可能な目標を必ず解く（%d/%d）' % (solved, trials),
      solved == trials,
      '成功率 %.1f%%%s' % (100.0 * solved / trials,
                           '' if not missed
                           else ' / 取りこぼしの誤差 %s mm'
                           % [round(e * 1000, 1) for e in missed[:5]]))
# 「解けた」と言ったときの誤差は、報告どおり小さいこと。
# ここを全件の最大で見ると、取りこぼし 1 件で意味が変わってしまう。
check('解けたと報告したときの誤差は 0.1mm 未満',
      bool(solved_err) and max(solved_err) < 1e-4,
      '最大 %.4f mm / 中央 %.6f mm'
      % (max(solved_err) * 1000, float(np.median(solved_err)) * 1000))

# 現在姿勢のすぐ近くを狙う使い方（座標ジョグ）が確実に効くこと
near_err = []
for _ in range(200):
    q_now = rng.uniform(chain.lower * 0.7, chain.upper * 0.7)
    here = chain.fk_position(q_now)
    goal = here + rng.normal(scale=0.01, size=3)      # 1cm ずらす
    q_ik, err, ok = chain.solve_ik(goal, q0=q_now)
    near_err.append(float(np.linalg.norm(chain.fk_position(q_ik) - goal)))
near_err = np.array(near_err)
check('1cm の座標ジョグは必ず解ける', near_err.max() < 1e-4,
      '最大 %.4f mm' % (near_err.max() * 1000))

# 時間制限は「最悪の打ち切り」であって速度調整ではない。
# 短くすると、再挑戦が要る難しい目標を取りこぼす。既定値がそこを
# 削っていないことを押さえておく（0.12 秒にすると 4% ほど落ちた）。
budget_miss = 0
for _ in range(60):
    q_true = rng.uniform(chain.lower, chain.upper)
    target = chain.fk_position(q_true)
    q_ik, err, ok = chain.solve_ik(target, q0=rng.uniform(chain.lower, chain.upper))
    if not ok:
        budget_miss += 1
check('既定の time_budget が到達可能な目標を削っていない', budget_miss == 0,
      '%d 件取りこぼし' % budget_miss)

# 明らかに届かない目標は、IK を回す前に安く弾けること
# （回すと 0.5 秒かかり、50Hz の制御ループが止まる）
check('明らかに届かない目標は安く弾く',
      chain.obviously_out_of_reach([0.0, 0.0, 3.0])
      and not chain.obviously_out_of_reach(chain.fk_position(np.zeros(6))))

# --------------------------------------------------------------------- #
# 5. 可動域を出ない
# --------------------------------------------------------------------- #
violations = 0
# 届かない目標は IK が全回数ぶん粘るので重い。件数は控えめにする。
for _ in range(60):
    target = rng.uniform(-1.5, 1.5, 3)        # 届かない所も混ぜる
    q_ik, err, ok = chain.solve_ik(target, q0=np.zeros(6))
    if np.any(np.asarray(q_ik) < chain.lower - 1e-9) or \
       np.any(np.asarray(q_ik) > chain.upper + 1e-9):
        violations += 1
check('どんな目標でも可動域を出ない', violations == 0, '%d 件' % violations)

# --------------------------------------------------------------------- #
# 6. 届かない目標
# --------------------------------------------------------------------- #
reach = chain.reach_estimate(samples=3000)
far = np.array([0.0, 0.0, 3.0])               # 明らかに届かない
q_ik, err, ok = chain.solve_ik(far, q0=np.zeros(6))
check('届かない目標は収束しないと報告する', not ok, 'err=%.3f m' % err)
check('届かないときも最も近い姿勢を返す', err < 3.0 and len(q_ik) == 6,
      'err=%.3f m' % err)

check('可動範囲の見積もりが出る',
      reach['max_radius'] > 0.3 and len(reach['min']) == 3, reach)

# --------------------------------------------------------------------- #
# 7. 想定外の URDF は素直に失敗する
# --------------------------------------------------------------------- #
try:
    K.ArmChain.from_urdf('/nonexistent.urdf')
    check('URDF が無ければ ChainError', False)
except K.ChainError as exc:
    check('URDF が無ければ ChainError', True, str(exc)[:40])

try:
    K.ArmChain.from_urdf(URDF, arm_joints=('joint1', 'nope'))
    check('関節名が合わなければ ChainError', False)
except K.ChainError as exc:
    check('関節名が合わなければ ChainError', True, str(exc)[:40])

# common.JOINT_LIMITS を優先できること（UI と可動域を揃えるため）
from piper_arm_ui.common import JOINT_LIMITS           # noqa: E402
tight = K.ArmChain.from_urdf(URDF, limits=JOINT_LIMITS)
check('limits 引数で可動域を上書きできる',
      abs(tight.upper[5] - JOINT_LIMITS['joint6'][1]) < 1e-9,
      (tight.upper[5], JOINT_LIMITS['joint6'][1]))

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
