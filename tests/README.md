# テスト

ROS も実機も無しで走る。`piper_arm_ui/` の中から:

```bash
python3 tests/test_kinematics.py   # 順運動学・逆運動学の数値検証（26項目）
python3 tests/test_cartesian.py    # 座標指定コマンドをシーケンサに流す（41項目）
python3 tests/test_xyz_ui.py       # 座標指定トグルを Chromium で押す（22項目）
python3 tests/check_real_urdf.py   # 実機URDFで運動学が成立するか（初回に1回）
```

どこから実行してもよい（パスは自分で解決する）。
ワークスペース直下からなら `python3 src/piper_arm_ui/tests/test_kinematics.py`。

必要なもの: `numpy` `scipy`（照合用）`aiohttp` `playwright`。
`mini_ros.py` が rclpy を差し替えるので ROS 2 は要らない。
`test_arm.urdf` は piper_description が無い環境でも運動学を検証するための
テスト用アーム（PiPER に寄せた寸法、rpy に 0 以外を入れて規約を踏む形）。

`check_real_urdf.py` だけは piper_description が要る。**この運動学は本物の
piper_description.urdf で動かした実績が無い**ので、実機の URDF がある環境では
最初に 1 回走らせて、チェーンが組めることと可動半径が実機の感覚と合うことを
確かめること。

**IK を触ったら必ず走らせること。** 逆運動学の良し悪しは画面を見ても
分からない。`test_kinematics.py` は scipy との照合、ヤコビアン vs 数値微分、
IK の往復（ランダム姿勢 → 手先位置 → IK → 同じ位置に戻るか）で押さえている。
