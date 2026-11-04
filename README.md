# piper_arm_ui — レモン収穫デモのアーム操作UI

犬型ロボットに載せたアーム（AgileX **PiPER**）でレモンを収穫するデモの、**ブラウザで操作する画面**です。
ROS 2 Humble / Ubuntu 22.04。

チームの収穫システム（`~/lemon_ws` の `lemon_reach` / `lemon_grasp`）のサービスを、画面のボタンから呼びます。
**画面のボタンだけで、レモンを選ぶ → 掴む → かごへ運ぶ → 離す → 戻る、までを一通りできます。**

> **実機が動きます。物理的な非常停止を、必ず手の届くところに置いてください。**
> 画面の「停止」はソフト停止で、非常停止ではありません（[停止について](#6-停止について)）。

---

## 目次

1. [画面の見かた](#1-画面の見かた)
2. [実機で動かす](#2-実機で動かす)（ミニPC。いつもの使い方）
3. [1サイクルの操作](#3-1サイクルの操作)
4. [収穫位置①②③の記録](#4-収穫位置の記録)
5. [手動ジョグ・座標指定](#5-手動ジョグ座標指定)
6. [停止について](#6-停止について)
7. [困ったとき](#7-困ったとき)
8. [実機なしで動かす（開発用PC）](#8-実機なしで動かす開発用pc)
9. [しくみ](#9-しくみ)
10. [テスト](#10-テスト)
11. [もっと詳しく](#11-もっと詳しく)

---

## 1. 画面の見かた

```
┌──────────────────────────────────────────────────────────┐
│ レモン自動収穫デモ  [移動用UI][アーム用UI]  ●●● [停止] ソフト停止 │
├────────────────────────────┬─────────────────────────────┤
│ カメラ                     │ 3D ビュー                   │
│  レモンの枠（クリックで選ぶ）│  実機の関節角で動くモデル    │
│  橙の太枠 =「対象」         │  ドラッグで視点を回せる      │
├────────────────────────────┼─────────────────────────────┤
│ 収穫シーケンス             │ ロボットの状態               │
│  □ UI操作を有効化（元栓）   │  上から見た図と ①②③ の位置  │
│  収穫システムの状態（4+1行）│  関節通信・エラーコードなど   │
│  ボタン（下の表）           │                             │
│  収穫位置の記録／ジョグ／座標│                             │
└────────────────────────────┴─────────────────────────────┘
  ログ（いちばん下。何を呼んで、どう返ってきたか）
```

| 場所 | 見るもの |
|---|---|
| **カメラ** | 手首カメラの映像と、レモンの検出枠。**橙の太枠に「対象」と付いたもの**が、いま狙っているレモン |
| **収穫システムの状態** | 対象／マネージャ／接近／把持／**UI の操作**（いま何を呼んでいるか・最後の結果）。色: 黄＝動作中、緑＝成功、赤＝失敗、黄の「掴めませんでした」は失敗ではない |
| **UI操作を有効化** | **元栓**。OFF の間は、どのボタンを押してもアームは動かない（停止だけは常に効く） |
| **ロボットの状態**（右下） | 上から見た図。①②③は**記録した位置だけ**が出る |
| **ログ**（下） | 押したボタンの結果。断られたときは理由が出る |

---

## 2. 実機で動かす

ミニPC（PiPER が CAN で繋がっている PC）での、いつもの起動です。**5つの端末**を使います。

### 起動の前に（初回だけ）

ワークスペースは3つを重ねて使います。毎回この順で source します。

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash     # 本家 piper_ros（＋改造）
source ~/lemon_ws/install/setup.bash      # チームの収穫システム
source ~/lemon_ui_ws/install/setup.bash   # この UI
```

ビルド（UI を直したら）:

```bash
cd ~/lemon_ui_ws && colcon build --packages-select piper_arm_ui --symlink-install
```

`--symlink-install` なので、`web/` や Python を直しただけなら再ビルドは要りません（起動し直す／ブラウザを再読込するだけ）。

### 起動（端末1〜5）

| 端末 | コマンド | 何が起きるか |
|---|---|---|
| 1 | `cd ~/piper_ros && bash can_activate.sh can0 1000000` | CAN を有効にする（再起動のたび） |
| 2 | `ros2 launch piper_real piper_real_bringup.launch.py` | アーム＋MoveIt＋手首カメラ。**起動から5秒後の姿勢が保持目標になる** |
| 3 | `ros2 launch lemon_reach lemon_yolo.launch.py` | レモンの検出（YOLO） |
| 4 | `ros2 launch lemon_reach lemon_harvest.launch.py auto_run:=false selection_mode:=manual` | 収穫システム。**この時点の姿勢が home になる**。マネージャは起動しない（UI が順に呼ぶ） |
| 5 | 下のコマンド | この UI |

```bash
# 端末5
ros2 launch piper_arm_ui arm_ui.launch.py \
    backend:=lemon use_lemon_detections:=true \
    start_robot_state_publisher:=false start_realsense:=false use_dummy_perception:=false
```

- 端末2の bringup にはカメラの取り付け値 `cam_x:=… cam_y:=… cam_z:=… cam_roll:=… cam_pitch:=… cam_yaw:=…` を渡せます。
  渡さないと既定値（link6 の原点、pitch −90°）です。**ずれていると、アームはずれた場所へ差し込みに行きます。**
  画面の「対象」の座標と、物差しで測った実物を比べて確かめてください。
- 止めるときは **5 → 4 → 3 → 2** の順に Ctrl+C（アームが止まっているときに）。

### 画面を開く

**別の機器（ノートPC・タブレット）のブラウザで** `http://<ミニPCのIP>:8080/` を開きます（IP は `hostname -I`）。

> **ミニPCの上でブラウザを開かないでください。** ミニPCは YOLO で CPU に余裕が無く、
> ブラウザのぶん YOLO が遅くなると、`lemon_target` が対象を選べなくなります
> （検出が毎秒 3.3 枚を割ると選ばれない。実測: ミニPCで開くと 1.1 枚、別の機器なら 5.2 枚）。

UI を直したあとは、ブラウザを**強制的に再読込**してください（Ctrl+Shift+R）。古いページのままだと、新しいボタンが出ません。

### `auto_run:=true` は使わない

`auto_run:=true` にすると、チームのマネージャが自動で 選ぶ→掴む→home→離す を繰り返します。
**離したレモンが転がると、それを次の対象として追いかけます。** UI から操作するときは `auto_run:=false` にしてください。
マネージャが動いている間は、UI はアームを動かすボタンを受け付けません（2つが同時に指令すると危ないため）。

---

## 3. 1サイクルの操作

**「UI操作を有効化」を ON にしてから**、上から順に押します。押せないボタンは灰色になっていて、押せない理由はログに出ます。

| 順 | 操作 | 呼ぶもの | アーム | 次に押せるようになるもの |
|---|---|---|---|---|
| 1 | カメラのレモンの**枠をクリック** | `/lemon_target/select` | 動かない | 枠が橙の「対象」になる |
| 2 | **把持姿勢へ移動** | `/lemon_approach/approach` | **大きく動く** | 「把持」 |
| 3 | **把持** | `/lemon_grasp/grasp` | **大きく動く（最大40秒）** | 差し込み→閉じる→戻る |
| 4 | **収穫位置 ①②③** | MoveIt で記録した姿勢へ | **大きく動く** | 「開放」 |
| 5 | **開放** | `/lemon_grasp/open` → `/lemon_target/release` | グリッパ | レモンが落ちる |
| 6 | **ホームへ戻す** | `/arm_home/go_home` | 動く | 1 からまた |

そのほかのボタン:

| ボタン | 呼ぶもの | |
|---|---|---|
| **対象を解除** | `/lemon_target/release` | 選んだ対象をやめる（アームは動かない）。解除した位置は除外リストに入るが、`selection_mode:=manual` では人が選ぶので無視される |
| **除外をクリア** | `/lemon_target/clear_exclusions` | 除外リストを空にする |
| **停止**（赤） | 下の「停止について」 | |

決まりごと（安全のため）:

- **「把持」は、同じ対象への「把持姿勢へ移動」が成功した直後だけ**押せます。`grasp` はいまの姿勢から差し込むので、pregrasp に居ないまま押すと危ないためです。home・解除・対象の切り替え・停止・把持を1回やったら、もう一度「把持姿勢へ移動」から。
- **アームが動く呼び出しは、同時に1つだけ。** 動いている間は、ほかの動くボタンを押せません。
- 結果の出かた: 掴めなかった → 黄の「**掴めませんでした**」（失敗ではない）／停止のあとに返った失敗 → 「**停止で止めました**」。

---

## 4. 収穫位置の記録

①②③は、**実機で置いて記録した姿勢へだけ**動きます。記録していない番号は押せません。
（`config/arm_ui.yaml` の `basket_pose_1〜3` は開発用の仮の値で、実機の胴体やかごの位置と合っていないので使いません。）

1. アームを**かごの上（レモンを離したい場所）**に動かす（[手動ジョグ・座標指定](#5-手動ジョグ座標指定)や RViz の MotionPlanning で）
2. 「収穫シーケンス」の **「収穫位置の記録」** を開き、**①②③のどれかを押す**
   - いまの実機の関節角がその番号として記録されます（アームは動かない）
   - すでに記録がある番号は「上書きしますか？」と確認が出ます
3. ログに「収穫位置①を記録しました（…）」と出て、右下の図に①が現れる

記録は `~/.ros/piper_arm_ui/basket_poses.json` に残り、起動し直しても使えます。

> MoveIt は**犬型の胴体やかごを知りません**（URDF に無い）。記録したあと最初に①へ動かすときは、
> 通り道をよく見て、ぶつかりそうなら停止してください。

---

## 5. 手動ジョグ・座標指定

「収穫シーケンス」の下の折りたたみにあります。

| | 動き |
|---|---|
| 関節ジョグ（J1〜J6 の ±） | **いまの実機の関節角 ＋ 1ステップ**を目標に、MoveIt で計画して動かす |
| 座標ジョグ（X/Y/Z の ±）・「この位置へ移動」 | 手先の目標位置（base_link、m）を逆運動学で関節角に直し、MoveIt で動かす。1回の上限は 250mm |
| グリッパの開／閉 | 開＝`/lemon_grasp/open`、閉＝グリッパのコントローラへ直接 |

- **1回押すと1ステップ。** 長押ししても繰り返しません（1回ごとに MoveIt が計画するので、押してから動き出すまで 0.5〜1 秒かかります）。続けざまの入力（0.5 秒以内）は無視されます。
- 座標の基準は **link6 の原点**です（`tcp_link` / `tcp_offset` が空のまま）。グリッパの指先ではありません。
- 逆運動学は、まれに届く位置を「届きません」と断ることがあります（既知の取りこぼし）。

---

## 6. 停止について

画面右上の **「停止」は、いつでも押せます**（元栓が OFF でも、何が動いていても）。押すと次の3つを同時に呼びます。

| 止めるもの | どうやって |
|---|---|
| 把持（差し込み・閉じる・戻り） | `/lemon_grasp/stop`。差し込み中は 0.04 秒で返り、目で見てもすぐ止まった |
| 把持姿勢への移動・home・収穫位置・ジョグ | MoveIt の `/execute_trajectory` の全ゴールを取り消す |
| コントローラに渡った軌道 | `/arm_controller` と `/gripper_controller` の全ゴールを取り消す |

**これは非常停止ではありません。** ROS やミニPCが固まっていれば効きません。確実に止めるのは物理的な非常停止だけです。
止めたあと、アームは**その場に残ります**（レモンや枝の近くのこともある）。「ホームへ戻す」で戻してください。

---

## 7. 困ったとき

| 症状 | 見るところ・対処 |
|---|---|
| **カメラの映像が来ない** | 別の端末で `ros2 topic hz /camera/camera/color/camera_info` と `.../color/image_raw` を比べる。**camera_info は来るのに画像が0枚**なら、ROS の通信相手の発見がうまくいっていない（`ros2 node list --no-daemon` でノードが足りない）。**端末5→4→3→2 を止めて全部起動し直す**と戻った（`ros2 daemon stop` も挟むとよい）。何度も起きるなら、全端末で `export ROS_LOCALHOST_ONLY=1` してから起動するのを試す（原因はまだ特定できていない） |
| **対象が選ばれない／把持姿勢へ移動を押せない** | レモンの枠が出ているか。出ていても選ばれないなら検出が遅い（毎秒 3.3 枚未満）。ミニPCでブラウザを開いていないか確かめる |
| **ボタンが灰色で押せない** | 元栓（UI操作を有効化）が OFF／ほかの動作が動いている／対象が無い／接近がまだ（把持）／記録が無い（①②③）。理由はログに出る |
| **押しても何も起きない** | ログを見る。「OFF なので」「いま〜が動いている」「auto_run:=true のマネージャが動いている」などが出ている |
| **新しいボタンが出ない・画面が古い** | ブラウザを強制的に再読込（Ctrl+Shift+R） |
| **レモンの少し上を掴む** | 既知の課題。検出の位置がレモンの中心ではなく表面である可能性が高い（下の「既知の課題」） |
| **画面がチカチカする** | UI の二重起動。`pkill -f harvest_sequencer` してから端末5を起動し直す |
| **ui_server が落ちた** | 映像のエンコード中に落ちることがある（原因は未特定）。2秒後に自動で起動し直すので、ブラウザを再読込する |
| **ポート 8080 が埋まっている** | 前の ui_server が残っている。[docs/REFERENCE.md](docs/REFERENCE.md) の「ポートが埋まっているとき」 |

### 既知の課題

- **掴む位置が高い**ことがある。差し込みの方向がほぼ真上からなので、差し込みが浅いと高さのずれになる。候補は、検出の位置が「見えている表面」であること（中心より約1半径カメラ側）と、`tcp_offset`（0.136m）が指の付け根か指先かの食い違い。測定で切り分ける必要がある
- 差し込み中に `Lemon is not tracked`（追従が途切れる）が出ることがある。近づくと検出から外れている可能性（`min_depth` 0.12m）
- 逆運動学の取りこぼし（`test_kinematics` の1件、`check_real_urdf`）
- `/joint_states` の velocity の欄は当てにならない（止まっていても 0.5 rad/s と出る）

---

## 8. 実機なしで動かす（開発用PC）

```bash
ros2 launch piper_arm_ui arm_ui.launch.py use_dummy_video:=true enable_arm_command:=true
# → http://localhost:8080/
```

`backend:=internal`（既定）では、UI 自身の状態機械が動き、指令した関節角がそのまま 3D ビューに出ます。
カメラはテストパターン、検出はダミー（`dummy_perception.py`）です。

> **`enable_arm_command:=true` は、実機（ros2_control 構成）では付けないでください。**
> この構成では `/joint_states` は出力で、そこへ出すと MoveIt と Servo が読む現在の姿勢が壊れます。
> 安全弁が自動で止めますが、そもそも付けないこと。`backend:=lemon` では `/joint_states` の publisher を作りません。

---

## 9. しくみ

```
ブラウザ（別の機器）
  │ HTTP / WebSocket / WebRTC（ポート 8080）
ui_server ── rosbridge（127.0.0.1:9090）── /arm_ui/command ──▶ harvest_sequencer（backend:=lemon）
  │ 映像                                     ◀── /arm_ui/state ──┘   │ サービス・アクション
  └ /camera/camera/color/image_raw                                  ▼
                                    チームの収穫システム（lemon_target / lemon_approach /
lemon_detections                    lemon_grasp / arm_home）と MoveIt
  /lemon_ellipse/lemons ─▶ /arm_ui/detections（検出枠と「対象」の印）
```

| ノード | 役割 |
|---|---|
| `ui_server` | 画面の配信、カメラ映像（WebRTC）、rosbridge の中継。アームへの指令は出さない |
| `harvest_sequencer` | ボタンを受けて、チームのサービスを呼ぶ（`backend:=lemon`）。結果待ちは別スレッドなので、待っている間も画面は止まらない。元栓・排他・各種の決まりはここ（`lemon_backend.py`） |
| `lemon_detections` | チームの検出（px の楕円）を、画面の検出枠（0〜1）に直す。固定された対象に印を付ける |
| `rosbridge` | ブラウザと ROS の橋渡し（外には開かない） |

チーム側（`~/lemon_ws`）には、UI から操作するための口を足してあります（担当メンバーの了承済み）:
`/lemon_grasp/stop`、`/lemon_target/select`（`lemon_msgs/srv/SelectTarget`）、`selection_mode:=manual`。
詳しくは `~/lemon_ws/src/lemon_reach/docs/lemon_harvest_system.md`。

---

## 10. テスト

ROS を起動せずに走ります（`tests/mini_ros.py` が rclpy を差し替える。画面は本物の Chromium で押す）。

```bash
pip3 install numpy scipy aiohttp playwright && playwright install chromium
cd ~/lemon_ui_ws/src/piper_arm_ui
for t in tests/test_*.py; do python3 $t > /dev/null && echo "OK  $t" || echo "NG  $t"; done
```

一覧と項目数は [CLAUDE.md](CLAUDE.md) の「4. テスト」。`test_kinematics.py` の1件と `check_real_urdf.py` は既知の取りこぼしで落ちます。

---

## 11. もっと詳しく

| | |
|---|---|
| [docs/REFERENCE.md](docs/REFERENCE.md) | **旧 README**。起動オプションの全部、RViz の映像、3D ビュー、分散構成、別ネットワーク、トピック一覧 |
| [docs/HANDOVER.md](docs/HANDOVER.md) | 設計の方針、データの流れ、運動学の中身 |
| [docs/PLAN_harvest_buttons.md](docs/PLAN_harvest_buttons.md) | チームの収穫システムに繋ぐ作業計画（段階0〜6） |
| [docs/lemon_integration_review.md](docs/lemon_integration_review.md) | チームのシステムとの突き合わせの調査 |
| [docs/SETUP_ubuntu2204_humble.md](docs/SETUP_ubuntu2204_humble.md) | まっさらな Ubuntu からミニPCを作る |
| [docs/SETUP_dev_pc.md](docs/SETUP_dev_pc.md) | 開発用PCへの導入 |
| [docs/SETUP_remote_access.md](docs/SETUP_remote_access.md) | 別ネットワークから使う |
| [CLAUDE.md](CLAUDE.md) | Claude Code 向けの記憶（設計の前提、踏んだ落とし穴） |
