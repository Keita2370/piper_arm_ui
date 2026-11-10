# lemon_ui_ws

レモン自動収穫デモの **アーム操作UI**（`piper_arm_ui`）のワークスペース。
チームが作った収穫システム（`~/lemon_ws`）を、ブラウザの画面から操作できるようにしたもの。

**使い方（起動・画面・操作）は [src/piper_arm_ui/README.md](src/piper_arm_ui/README.md) を見てください。**
このファイルは、ワークスペースの組み立てと全体の状況だけを書いています。

> 実機が動きます。物理的な非常停止を手元に置いてから使ってください。

---

## 1. 3つのワークスペースを重ねて使う

```
~/piper_ros      本家 piper_ros（＋改造）
                 piper, piper_description, piper_msgs, piper_with_gripper_moveit …
      ↓ 重ねる
~/lemon_ws       チームの収穫システム
                 lemon_reach, lemon_grasp, lemon_msgs, piper_real, yolo_ros …
      ↓ 重ねる
~/lemon_ui_ws    この UI（piper_arm_ui）
```

**中身はコピーせず、重ねて使う。** コピーすると `lemon_msgs` などのカスタム msg の定義が二重になり、
片方だけ更新されたときに通信が静かに壊れるため。

| ワークスペース | GitHub（private） | 備考 |
|---|---|---|
| `~/lemon_ui_ws/src/piper_arm_ui` | [Keita2370/piper_arm_ui](https://github.com/Keita2370/piper_arm_ui) | この UI。このファイルと `CLAUDE.md` は `workspace/` に同梱 |
| `~/lemon_ws` | [Keita2370/lemon_ws](https://github.com/Keita2370/lemon_ws) | ミニPCの `~/lemon_ws` のスナップショット。元のリポジトリと、UI のために足した変更は `SOURCES.md` に記録 |
| `~/piper_ros` | [Keita2370/piper_ros](https://github.com/Keita2370/piper_ros) | 本家 agilexrobotics/piper_ros（humble）＋ミニPCでの改造 |

---

## 2. 取得とビルド

```bash
# まだ無ければ（ミニPCには既にある）
git clone -b humble https://github.com/Keita2370/piper_ros.git ~/piper_ros
git clone https://github.com/Keita2370/lemon_ws.git ~/lemon_ws
mkdir -p ~/lemon_ui_ws/src
git clone https://github.com/Keita2370/piper_arm_ui.git ~/lemon_ui_ws/src/piper_arm_ui
```

毎回この順で source する（重ねる順序が大事）。`.bashrc` に入れておくと楽。

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/lemon_ws/install/setup.bash
cd ~/lemon_ui_ws && colcon build --symlink-install && source install/setup.bash
```

`--symlink-install` なので、`web/` や Python を直しただけなら再ビルドは要らない
（起動し直す／ブラウザを再読込するだけ）。

### yolo_ros（検出）

`Keita2370/lemon_ws` には `src/yolo_ros` が入っているが、Python の仮想環境（`.venv`、約12GB）は入っていない。
新しい PC では作り直す。

```bash
pip3 install ultralytics          # または yolo_ros の手順で .venv を作る
cd ~/lemon_ws && colcon build --symlink-install
```

YOLO が動かないと検出が来ず、`lemon_target` は対象を選べない。

---

## 3. 起動（要点）

ミニPCで5つの端末を使う。**画面は別の機器のブラウザで** `http://<ミニPCのIP>:8080/` を開く
（ミニPCで開くと CPU が足りず、検出が遅くなって対象が選べなくなる）。

```bash
# 1) CAN
cd ~/piper_ros && bash can_activate.sh can0 1000000
# 2) アーム + MoveIt + 手首カメラ。cam_* はカメラの取り付けの実測値（2026-11-11）。
#    省略するとレモンの約10cm上を掴みにいく。値の意味は src/piper_arm_ui/README.md
ros2 launch piper_real piper_real_bringup.launch.py use_rviz:=false \
    cam_x:=-0.06 cam_y:=-0.04 cam_z:=0.04 cam_roll:=0.0 cam_pitch:=-1.2217 cam_yaw:=0.0
# 3) YOLO
ros2 launch lemon_reach lemon_yolo.launch.py
# 4) 収穫システム（UI が順に呼ぶので auto_run:=false。対象は人が選ぶ manual）
ros2 launch lemon_reach lemon_harvest.launch.py auto_run:=false selection_mode:=manual
# 5) UI
ros2 launch piper_arm_ui arm_ui.launch.py \
    backend:=lemon use_lemon_detections:=true \
    start_robot_state_publisher:=false start_realsense:=false use_dummy_perception:=false
```

- `start_robot_state_publisher:=false` と `start_realsense:=false` は必須（2) が両方を起動している）。
- 止めるときは 5 → 4 → 3 → 2 の順に Ctrl+C。
- 画面の操作、収穫位置の記録、停止、困ったときは [src/piper_arm_ui/README.md](src/piper_arm_ui/README.md)。

---

## 4. いまの状況

**画面のボタンだけで、レモンを選ぶ → 掴む → かごへ運ぶ → 離す → 戻る、まで通る**（実機で確認済み）。

| | 状態 |
|---|---|
| 状態表示（チームの収穫システムの status を画面に） | 完了 |
| 検出枠と「対象」の印（`lemon_detections`） | 完了 |
| サービス呼び出しの土台（`backend:=lemon`、固まらない） | 完了 |
| 対象の選択／把持姿勢へ移動／把持／ホームへ戻す／開放 | 完了 |
| **停止**（把持・MoveIt の動作・コントローラの軌道を止める。ソフト停止） | 完了 |
| 収穫位置①②③（実機で置いて記録した姿勢へ動かす） | 完了 |
| 手動ジョグ・座標指定（MoveIt で動かす） | 完了 |

### チーム側（`~/lemon_ws`）に足した口

担当メンバーの了承を得て、UI から操作するために足した（呼ばない限り・指定しない限り従来の動きは変わらない）。

| | |
|---|---|
| `lemon_grasp` | `/lemon_grasp/stop`（把持を途中で止める） |
| `lemon_msgs` | `srv/SelectTarget.srv` |
| `lemon_target` | `/lemon_target/select`、`selection_mode`（`auto` 既定 / `manual`） |
| `lemon_approach` | 早く打ち切る失敗も `~/status` に出す |

### 残っていること

- **掴む位置が少し高い**ことがある（検出の位置が表面か中心か、`tcp_offset` の意味の食い違い、を測って切り分ける）
- 差し込み中に追従が途切れることがある（`Lemon is not tracked`）
- **カメラの映像がときどき来なくなる**（ROS の通信相手の発見がうまくいっていない様子。全部を起動し直すと戻る。原因は未特定）
- ui_server が映像のエンコード中にまれに落ちる（自動で起動し直す。原因は未特定）
- 逆運動学の取りこぼし（`test_kinematics` の1件、`check_real_urdf`）
- ワークスペースの `CLAUDE.md` の一部が古い（下の表）

---

## 5. テスト

ROS を起動せずに走る（rclpy を差し替えるスタブと、本物の Chromium を使う）。

```bash
pip3 install numpy scipy aiohttp playwright && playwright install chromium
cd ~/lemon_ui_ws/src/piper_arm_ui
for t in tests/test_*.py; do python3 $t > /dev/null && echo "OK  $t" || echo "NG  $t"; done
```

一覧と項目数は [src/piper_arm_ui/CLAUDE.md](src/piper_arm_ui/CLAUDE.md) の「4. テスト」。
`test_kinematics.py` の1件と `check_real_urdf.py` は既知の取りこぼしで落ちる。

---

## 6. 資料

| | |
|---|---|
| [src/piper_arm_ui/README.md](src/piper_arm_ui/README.md) | **使い方**（起動、画面、1サイクルの操作、収穫位置、停止、困ったとき） |
| [src/piper_arm_ui/CLAUDE.md](src/piper_arm_ui/CLAUDE.md) | UI の設計の前提、踏んだ落とし穴、テストの一覧 |
| [src/piper_arm_ui/docs/REFERENCE.md](src/piper_arm_ui/docs/REFERENCE.md) | 旧 README（起動オプションの全部、RViz、分散構成、トピック一覧） |
| [src/piper_arm_ui/docs/PLAN_harvest_buttons.md](src/piper_arm_ui/docs/PLAN_harvest_buttons.md) | チームのシステムに繋ぐ作業計画（段階0〜6。いまは全段階が済んでいる） |
| [CLAUDE.md](CLAUDE.md) | Claude Code 向けの安全の約束。**一部が古い**: §1「他のワークスペースは読むだけ」（lemon_ws は了承を得て修正済み）、§3 のサービスの表（stop / select が無い）、§5「確実な停止手段が無い」（stop を足した）、§7・§8（済んだ段階と、決めた事項） |
| `~/lemon_ws/src/lemon_reach/docs/lemon_harvest_system.md` | チームの収穫システムの設計資料（足した口も追記済み） |
