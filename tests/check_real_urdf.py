#!/usr/bin/env python3
"""実機の URDF（piper_description）で運動学が成立するか確かめる.

【なぜこれが要るか】
tests/test_kinematics.py は piper_description が無い環境でも走るように、
同梱の test_arm.urdf（PiPER に寄せた作りもの）で検証している。
**本物の piper_description.urdf で動かしたことは一度も無い。**
関節名やリンク構成が想定と違えばチェーンを組めず、座標指定だけが
静かに無効になる（画面に理由は出るが、気づかないと「効かない」で終わる）。

piper_description がある環境で最初に 1 回これを走らせて、
  - チェーンが組めるか
  - 手先リンクの自動判定が妥当か
  - ゼロ姿勢の手先位置と可動半径が実機の感覚と合うか
  - IK が往復するか
を確かめる。

    python3 tests/check_real_urdf.py
    python3 tests/check_real_urdf.py /path/to/piper_description.urdf
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                        # noqa: E402

from piper_arm_ui.common import JOINT_LIMITS              # noqa: E402
from piper_arm_ui import kinematics as K                  # noqa: E402


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else K.find_description_file()
    if not path:
        print('piper_description の URDF が見つかりません。')
        print('  ros2 pkg prefix piper_description   で入っているか確認し、')
        print('  source install/setup.bash           を忘れていないか見てください。')
        print('  パスを直接渡すこともできます: python3 tests/check_real_urdf.py <urdf>')
        return 1

    print('URDF: %s\n' % path)
    try:
        chain = K.ArmChain.from_urdf(path, limits=JOINT_LIMITS)
    except K.ChainError as exc:
        print('✗ チェーンを組めません: %s' % exc)
        print()
        print('座標指定はこの環境では無効になります。URDF の関節名・リンク構成が')
        print('想定（joint1..joint6 が base から1本で繋がる）と違う可能性があります。')
        print('URDF の <joint> を見て、必要なら harvest_sequencer の tcp_link や')
        print('kinematics.DEFAULT_ARM_JOINTS を実機に合わせてください。')
        return 1

    print('✓ チェーンを組めました')
    print('  可動関節 : %s' % ', '.join(chain.arm_joints))
    print('  経路     : %s' % ' -> '.join(
        [chain.joints[0]['parent']] + [j['child'] for j in chain.joints]))
    print('  手先リンク: %s  （tcp_link 未指定なので joint6 の子を採用）'
          % chain.tcp_link)
    print()

    zero = chain.fk_position(np.zeros(6))
    reach = chain._reach
    print('ゼロ姿勢の手先位置 : (%.4f, %.4f, %.4f) m' % tuple(zero))
    print('可動半径の目安     : %.3f m' % reach['max_radius'])
    print('可動範囲の箱       : x %.3f〜%.3f / y %.3f〜%.3f / z %.3f〜%.3f m'
          % (reach['min'][0], reach['max'][0], reach['min'][1],
             reach['max'][1], reach['min'][2], reach['max'][2]))
    print()
    print('  → PiPER の公称リーチは約 0.7m。上の値がそこから大きく外れていたら、')
    print('    読んでいる URDF か関節の並びを疑ってください。')
    print()

    # IK の往復。到達可能な点を FK で作り、別姿勢から解き直して戻るか見る。
    rng = np.random.default_rng(0)
    errs, missed = [], 0
    for _ in range(100):
        q_true = rng.uniform(chain.lower, chain.upper)
        target = chain.fk_position(q_true)
        q_ik, err, ok = chain.solve_ik(target, q0=rng.uniform(chain.lower, chain.upper))
        if ok:
            errs.append(float(np.linalg.norm(chain.fk_position(q_ik) - target)))
        else:
            missed += 1

    print('IK の往復（100 回）')
    print('  解けた       : %d / 100' % len(errs))
    if errs:
        print('  誤差の最大   : %.4f mm' % (max(errs) * 1000))
        print('  誤差の中央値 : %.5f mm' % (float(np.median(errs)) * 1000))

    ok = missed == 0 and errs and max(errs) < 1e-4
    print()
    if ok:
        print('✓ 実機の URDF でも運動学は成立しています。座標指定を使って大丈夫です。')
        print()
        print('次にやること: UI を開いて「ロボットの状態」の')
        print('  手先位置(指令)  … いま計算したこの運動学の値')
        print('  手先位置(実機)  … piper が出す /end_pose')
        print('を見比べ、ずれていたら差を config/arm_ui.yaml の tcp_offset に入れる。')
        print('（実機が無い開発用PCでは「手先位置(実機)」は出ません）')
    else:
        print('✗ IK が安定しません（取りこぼし %d 件）。' % missed)
        print('  URDF の可動域と common.JOINT_LIMITS が食い違っていないか、')
        print('  関節の並びが想定どおりかを確認してください。')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
