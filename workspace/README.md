# lemon_ui_ws

レモン自動収穫デモの **操作UI**（`piper_arm_ui`）を、チームが作った収穫システムに
合わせるためのワークスペース。

Claude Code で作業する場合は `CLAUDE.md` を読む（起動すると自動で読まれる）。

---

## これは何か

既存の `~/lemon_ws`（チームの収穫システム）と `~/piper_ros`（本家＋改造）を
**壊さずに** UI 側だけを直すために、別のワークスペースとして切ってある。

```
~/piper_ros      piper, piper_description, piper_msgs,
                 piper_with_gripper_moveit, piper_gazebo      読むだけ
      ↓ 重ねる
~/lemon_ws       lemon_reach, lemon_grasp, lemon_msgs,
                 piper_real, piper_isaacsim, yolo_ros         読むだけ
      ↓ 重ねる
~/lemon_ui_ws    piper_arm_ui                                 ← ここだけ編集する
```

**他のワークスペースの中身はコピーしていない。** 重ねて使う。
コピーすると `lemon_msgs` などカスタム msg の定義が二重になり、
片方だけ更新されたときに通信が静かに壊れるため。

---

## 置きかた

```bash
cd ~
unzip ~/Downloads/lemon_ui_ws.zip     # → ~/lemon_ui_ws
```

## 建てかた

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/lemon_ws/install/setup.bash

cd ~/lemon_ui_ws
colcon build --symlink-install
source install/setup.bash
```

毎回この順で source する（重ねる順序が大事）。
`.bashrc` に入れておくと楽。

テストに要るもの（ROS とは別）:

```bash
pip3 install numpy scipy aiohttp playwright
playwright install chromium
```

## 動かしかた

```bash
# 1) CAN
cd ~/piper_ros && bash can_activate.sh can0 1000000

# 2) アーム + MoveIt + 手首カメラ（cam_* は実測値）
ros2 launch piper_real piper_real_bringup.launch.py cam_x:=... cam_y:=... ...

# 3) YOLO  ※ yolo_ros を ~/lemon_ws/src に clone してから
ros2 launch lemon_reach lemon_yolo.launch.py

# 4) 収穫シーケンス（UI から1段ずつ進めるなら auto_run:=false）
ros2 launch lemon_reach lemon_harvest.launch.py auto_run:=false

# 5) UI
ros2 launch piper_arm_ui arm_ui.launch.py \
    start_robot_state_publisher:=false \
    start_realsense:=false
```

ブラウザで `http://<このPCのIP>:8080/`。

`start_robot_state_publisher:=false` と `start_realsense:=false` は必須。
2) が既に両方を起動しているため。

---

## まだ終わっていないこと

UI は**まだチームのシステムに合わせていない**。いまの状態で起動すると:

- 3D ビュー・カメラ映像・状態表示は動く
- **収穫シーケンスのボタンは効かない**（UI 独自の状態機械のままで、
  チームのサービスを呼んでいない）
- UI からアームは動かせない（安全弁が止める。理由は `CLAUDE.md` §4）

やることの順序は `CLAUDE.md` §7。
**人が決めないといけないことが4つある**（§8）ので、そこから。

---

## yolo_ros

サイズが大きいので同梱していない。

```bash
cd ~/lemon_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
pip3 install ultralytics
cd ~/lemon_ws && colcon build --symlink-install
```

無いと検出が来ず、`lemon_target` は対象を選べない
（シーケンスは `waiting_target` のまま止まる）。

---

## テスト

```bash
cd ~/lemon_ui_ws/src/piper_arm_ui
python3 tests/test_kinematics.py            # 順/逆運動学（25項目）
python3 tests/test_cartesian.py             # 座標指定コマンド（41項目）
python3 tests/test_command_topic_guard.py   # /joint_states の安全弁（22項目）
python3 tests/test_xyz_ui.py                # 座標指定UI を Chromium で（22項目）
python3 tests/check_real_urdf.py            # 実機URDFでの運動学
```

ROS を起動せずに走る。

---

## 詳しい資料

| | |
|---|---|
| `CLAUDE.md` | **作業の前提・安全の約束・やることの順序** |
| `src/piper_arm_ui/CLAUDE.md` | UI パッケージ自体の設計・落とし穴 |
| `src/piper_arm_ui/README.md` | 起動オプション、トラブル対応 |
| `src/piper_arm_ui/docs/HANDOVER.md` | 設計の経緯 |
| `~/lemon_ws/src/lemon_reach/docs/lemon_harvest_system.md` | チームの収穫システムの設計資料 |
