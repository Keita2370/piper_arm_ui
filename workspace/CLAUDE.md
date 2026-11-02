# lemon_ui_ws — Claude Code 向けのプロジェクト記憶

レモン自動収穫デモの **操作UI** を、チームが作った収穫システムに合わせるための
ワークスペース。

@src/piper_arm_ui/CLAUDE.md

---

## 0. ★ 最初に読む：この環境には実機が繋がっている

**このPCは PiPER 実機に CAN で繋がっているミニPC。** 書いたコードは本物のアームを
動かす。次の約束を守ること。

| | |
|---|---|
| **`/joint_states` に publish しない** | この構成では `/joint_states` は `joint_state_broadcaster` の**出力**。publish するとアームは動かないうえに、MoveIt と Servo が読む「現在の関節角」が壊れる。**把持動作中なら危険**（詳細は §4） |
| **`enable_arm_command:=true` を安易に付けない** | README や過去の例に出てくるが、**この構成では意味がないどころか有害**。安全弁が自動で止めるようにしてあるが、そもそも付けない |
| **アームを動かす変更は、動かす前に人へ知らせる** | 「試しに動かしてみます」を黙ってやらない。何がどう動くかを先に述べて確認を取る |
| **止められないことを前提に考える** | この系に**確実なソフト非常停止は無い**（§5）。物理的な非常停止が唯一の手段 |
| 迷ったら止める | 判断がつかないときは実行せず、何が分からないかを述べる |

---

## 1. このワークスペースの役割

**ここで編集してよいのは `src/piper_arm_ui/` だけ。**

チームが作ったもの（`~/lemon_ws`）と本家＋改造（`~/piper_ros`）は
**別のワークスペースにあり、読むだけ**。アンダーレイとして重ねて使う。

```
~/piper_ros      … piper, piper_description, piper_msgs,
                   piper_with_gripper_moveit, piper_gazebo   ★読むだけ
      ↓ 重ねる
~/lemon_ws       … lemon_reach, lemon_grasp, lemon_msgs,
                   piper_real, piper_isaacsim, yolo_ros      ★読むだけ
      ↓ 重ねる
~/lemon_ui_ws    … piper_arm_ui                              ← ここだけ編集する
```

**なぜコピーせず重ねるか**: コピーすると定義が二重になり、片方だけ更新されたときに
`lemon_msgs` などカスタム msg の食い違いで**通信が静かに壊れる**。
重ねれば実体は1つで済む。

他のワークスペースを直す必要が出たら、**勝手に直さず、何をなぜ変えたいかを先に出す。**

---

## 2. 建てかたと起動

```bash
# 毎回この順で source する（重ねる順序が大事）
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/lemon_ws/install/setup.bash
cd ~/lemon_ui_ws && colcon build --symlink-install && source install/setup.bash
```

`--symlink-install` を付けると `web/` を直しただけでブラウザの再読込に反映される。

### 起動（実機）

```bash
# 1) CAN
cd ~/piper_ros && bash can_activate.sh can0 1000000

# 2) アーム + MoveIt + 手首カメラ（チーム版。cam_* は実測値を入れる）
ros2 launch piper_real piper_real_bringup.launch.py cam_x:=... cam_y:=... ...

# 3) YOLO
ros2 launch lemon_reach lemon_yolo.launch.py

# 4) 収穫シーケンス。UI を使うなら auto_run:=false
ros2 launch lemon_reach lemon_harvest.launch.py auto_run:=false

# 5) UI。robot_state_publisher と RealSense は 2) が既に起動しているので止める
ros2 launch piper_arm_ui arm_ui.launch.py \
    start_robot_state_publisher:=false \
    start_realsense:=false
```

**`yolo_ros` は未 clone。** `~/lemon_ws/src` に
`https://github.com/mgonzs13/yolo_ros.git` を clone してビルドするまで 3) は動かない。
YOLO が無いと検出が来ず、`lemon_target` は対象を選べない。

---

## 3. チームのシステムとの接点

UI が繋ぐ先。**これが仕様の中心。**

### サービス（すべて `std_srvs/Trigger`）

| サービス | 役割 | 失敗時の message |
|---|---|---|
| `/lemon_approach/approach` | pregrasp へ移動 | `busy` / `no target` / `cannot insert straight into the lemon` / `unreachable after N attempts` |
| `/lemon_grasp/grasp` | 差し込んで掴み、pregrasp へ戻る | `not ready` / `busy` / `no target` / `missed (gripper closed to X)` |
| `/lemon_grasp/open` | グリッパを開いてレモンを離す | `busy` / `failed to open gripper` |
| `/arm_home/go_home` | 起動時の姿勢へ戻る | `home is not recorded` / `busy` |
| `/lemon_target/release` | 対象を解除（**除外リストに入る**） | `No target is locked` |
| `/lemon_target/clear_exclusions` | 除外リストを空にする | — |
| `/arm_home/set_home` | 現在姿勢を home として記録し直す | — |

**`grasp` は掴み損ねると `success=false, message="missed (...)"` を返すが、
動作自体は正常に完了している。** UI は「失敗」ではなく「掴めなかった」と出すこと。

### 購読するトピック

| トピック | 型 | 内容 |
|---|---|---|
| `/lemon_harvest_manager/status` | `std_msgs/String` | `waiting_target` `approaching` `grasping` `going_home` `opening` `releasing` `error` |
| `/lemon_grasp/status` | `std_msgs/String` | `opening` `inserting` `closing` `retreating` ＋ 最終メッセージ |
| `/lemon_approach/status` | `std_msgs/String` | `approaching` ＋ 最終メッセージ |
| `/lemon_ellipse/lemons` | `lemon_msgs/LemonEllipseArray` | 検出。UI の検出枠の元 |
| `/lemon_target/target` | `geometry_msgs/PointStamped`（latched） | 固定された対象。**`frame_id` が空なら対象なし** |
| `/lemon_target/tracked` | 同上 | 追従位置 |
| `/joint_states` | `sensor_msgs/JointState` | 実機の関節角。3Dビューの入力（**購読のみ**） |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 手首カメラ |
| `/arm_status`, `/end_pose` | piper | エラーコード、実機の手先位置 |

### 手先の基準（確認済み・チームと一致させる）

```yaml
# config/arm_ui.yaml
tcp_link: "gripper_base"        # link6 と同一フレーム
tcp_offset: [0.0, 0.0, 0.136]   # 指の付け根（joint7/8 の原点 0.1358 の丸め）
```

相手も `ee_link: gripper_base` / `tcp_offset: 0.136` / `planning_frame: base_link` /
`planning_group: "arm"` を使っている。

---

## 4. ★ `/joint_states` の向きが本家と逆

ここを取り違えると事故になるので、仕組みを書いておく。

```
本家 piper_ros:
  start_single_piper.launch.py が joint_ctrl_single → /joint_states にリマップ
  → UI が /joint_states に publish すればアームが動いた

チーム版（ros2_control）:
  move_group → arm_controller(JTC) → topic_based_ros2_control
    → /piper_joint_bridge/joint_command → piper_joint_bridge
    → /joint_ctrl_single → piper_single_ctrl → CAN
  /joint_states は joint_state_broadcaster の出力
```

UI が `/joint_states` に publish すると:

1. アームは動かない（指令はもう別経路）
2. `robot_state_publisher` / MoveIt / Servo が読む現在姿勢に指令値が混ざる
   → **TF が実機と指令の間で揺れ、Servo が誤った姿勢を元に動く**

**安全弁を入れてある**（v0.4.0）: 起動5秒後に指令トピックの publisher を数え、
他にいれば関節指令を自動で止める。画面のトグルでも戻せない。
`tests/test_command_topic_guard.py` が守っている。**この安全弁を外さないこと。**

また、UI が持っていた piper の癖への対応
（`position[6]`＝グリッパ、`velocity` 長さ7、`effort[6]`）は
**`piper_joint_bridge` が肩代わりしている**ので、UI 側の実装は捨ててよい。

---

## 5. ★ 確実な停止手段が無い

`lemon_grasp` に cancel / abort のサービスは**無い**。把持が止まるのは:

| | 時間 |
|---|---|
| `servo_timeout` 経過（実機 40 秒） | 最大40秒 |
| 対象が解除されて目標の配信が止まる → PoseTracking が中断 | **約0.5秒** |
| 正常終了 | — |

**UI の停止ボタンが使える唯一の現実的な手段は `/lemon_target/release`。**
差し込み中なら約0.5秒で止まる。ただし:

- **戻り動作は止まらない**（目標がレモンの有無に依存しない）
- **cartesian 戻りはさらに止まらない**（JTC に投げ終わっている）
- `grasp` と `open` は `busy_` を共有。**把持中はグリッパを開けない**

`lemon_approach` と `arm_home` は `/execute_trajectory` アクション経由なので
**そちらは cancel できる**。

**停止ボタンは「ソフト停止であって非常停止ではない」と画面に出し続けること。**
できないことをできるように見せない。

---

## 6. 運用上の落とし穴

| | |
|---|---|
| **除外リストで詰まる** | `release` のたびに、掴めても掴めなくてもその位置が**永久に**除外される。視野内を一通り試すと全部除外され `waiting_target` のまま止まる。**UI に「除外をクリア」ボタンが要る**（`/lemon_target/clear_exclusions`） |
| 検出はマスク必須 | `lemon_ellipse_node` はマスクのある検出しか出さない。セグメンテーションモデルが要る |
| `id` は空かも | 実機は `use_tracking:=False` なので YOLO の tracking ID が入らない |
| 座標は px | `LemonEllipse.center_u/center_v` は**ピクセル**。UI の正規化 0–1 に直すには camera_info の幅高で割る |
| home は起動時の姿勢 | `lemon_harvest.launch.py` を起動したときの姿勢が home になる。レモン全体が見える姿勢にしてから起動する |

---

## 7. やること

**作業の手順は `src/piper_arm_ui/docs/PLAN_harvest_buttons.md` に段階0〜6でまとめてある。
着手する前に必ずそれを読む。**

段階は危険度の低い順に並んでいる。

| | | アーム |
|---|---|---|
| 0 | 現状確認（コードは変えない） | 動かない |
| 1 | 状態表示だけ | 動かない |
| 2 | 検出アダプタ | 動かない |
| 3 | サービス呼び出しの土台（executor の作り替え） | 動かない |
| 4 | `release` → `go_home` → `open` | **動く** |
| 5 | `approach` → `grasp` | **大きく動く** |
| 6 | 停止と後始末 | |

**段階0〜3 は §8 の判断が決まっていなくても進められる。**
段階4 から実機が動くので、物理的な非常停止を手元に置くこと。

**1段ずつ。終わるたびに実機で確かめてから次へ。**

---

## 8. ★ 人が決めること（Claude Code が決めない）

見つけたら**変更案を出さずに、何が問題かだけを述べて止まる。**

1. **把持対象を誰が選ぶか** — `lemon_target` は3秒観測して自動で一番近い安定した
   レモンを選ぶ。UI は人がクリックして選ぶ設計。両立には**相手のコード改造が要る**
2. **UI はマネージャか監視役か** — `auto_run:=false` で UI が1段ずつ呼ぶか、
   `auto_run:=true` のまま見るだけか
3. **座標指定・関節ジョグを残すか** — 残すなら MoveIt 経由に作り直し
   （`~/lemon_ws/src/lemon_reach/lemon_reach/moveit_helper.py` がそのまま参考になる）
4. **収穫位置 ①②③ をどうするか** — 相手に該当機能が無い

---

## 9. テスト

```bash
cd ~/lemon_ui_ws/src/piper_arm_ui
python3 tests/test_kinematics.py            # 順/逆運動学（25項目）
python3 tests/test_cartesian.py             # 座標指定コマンド（41項目）
python3 tests/test_command_topic_guard.py   # /joint_states の安全弁（22項目）
python3 tests/test_xyz_ui.py                # 座標指定UI を Chromium で（22項目）
python3 tests/check_real_urdf.py            # 実機URDFでの運動学
```

ROS を起動せずに走る（`tests/mini_ros.py` が rclpy を差し替える）。
必要: `numpy scipy aiohttp playwright` ＋ `playwright install chromium`。

**変更したら走らせてから「できた」と言う。**
新しい機能を足したら同じ粒度でテストも足す。落としてみて本当に落ちるかまで確かめる。

---

## 10. 参照先（読むだけ）

| | |
|---|---|
| `src/piper_arm_ui/docs/PLAN_harvest_buttons.md` | **作業の手順（段階0〜6）。着手前に読む** |
| `src/piper_arm_ui/docs/lemon_integration_review.md` | 突き合わせの調査結果。サービスの応答、状態の対応、落とし穴の詳細 |
| `~/lemon_ws/src/lemon_reach/docs/lemon_harvest_system.md` | **チームの設計資料** |
| `~/lemon_ws/src/lemon_reach/lemon_reach/moveit_helper.py` | MoveIt の呼びかた。UI の作り直しの参考 |
| `~/lemon_ws/src/piper_real/README.md` | 実機 bringup の構成と注意 |
| `src/piper_arm_ui/docs/HANDOVER.md` | UI 側の設計・落とし穴 |
| `src/piper_arm_ui/README.md` | 起動オプション、トラブル対応 |

---

## 11. 既知の小さな不具合（未修正）

`kinematics.solve_ik` の許容誤差 `tol=1e-4`（0.1mm）が厳しすぎて、
**根元に近い折りたたんだ姿勢で、到達可能な座標を約1%「届かない」と断る**
（実機URDFで 0.12mm まで詰めて止まる例を確認）。

対策: `tol` を `5e-4`（0.5mm）にする。レモン検出の精度は cm 単位なので十分。
`tests/test_kinematics.py` の該当する判定も合わせて直すこと。
