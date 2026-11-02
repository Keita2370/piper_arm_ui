# lemon 収穫システムと UI の突き合わせ（精読版）

対象: `lemon.zip` 全ファイル（約3,800行）＋ 別途 clone する `yolo_ros`
**修正は行っていない。調査結果のみ。**

初版（インタフェースのみの確認）から**変わった結論**は §1 にまとめた。

---

## 1. 精読して変わったこと

| | 初版の見立て | 精読後 |
|---|---|---|
| 停止ボタン | 「要検討・中」 | **クリーンな停止手段が存在しない。** 設計上の課題として扱う必要がある（§3） |
| 座標指定・関節ジョグ | 「MoveIt 経由に作り直し・大」 | **作り直しの道筋は明確。** `moveit_helper.py` がほぼそのまま使える（§6） |
| 検出のクリック選択 | 「設計判断」 | 変わらず。ただし**選択中のレモンを画面に出すのは容易**（アダプタで突き合わせ可能・§5） |
| 除外リスト | 言及なし | **UI に「除外をクリア」ボタンが要る。** 無いと運用が詰まる（§4） |
| zip の範囲 | 「これが相手のすべて」 | **違う。`~/piper_ros` 側にも改造がある**（§8） |
| 分散構成 | こちらから提案 | **相手も既に同じことをしている**（`piper_real_rviz.launch.py`）（§9） |

**piper_ros も受領して確認済み（§16）。** 未確認だった点はすべて解けた。
新たに UI 側の不具合を1件見つけた（§16-5）。

---

## 2. 全体の構成（確定）

```
piper_single_ctrl (CAN)                    ← ~/piper_ros 側に publish_rate を追加済
  ↑ /joint_ctrl_single        ↓ /joint_states_single
piper_joint_bridge              gripper(0–0.07) ⇔ joint7/joint8 変換
  ↑ ~/joint_command           ↓ ~/joint_states   max_jump 0.3rad の安全弁
topic_based_ros2_control (update_rate 100Hz)
  ├ joint_state_broadcaster → /joint_states          ★ 出力
  ├ arm_controller      JTC  joint1–6
  ├ gripper_controller  JTC  joint7
  └ gripper8_controller JTC  joint8
move_group (piper_with_gripper_moveit, planning_group = "arm")
robot_state_publisher / RViz / realsense2_camera(手首D435 640x480@15) / link6→camera_link
```

```
D435 → yolo_ros(seg, CPU, 320x256) → /yolo/detections_3d → lemon_ellipse_node
                                        → /lemon_ellipse/lemons, /markers, /dbg_image
  lemon_target   自動で1つ選んで固定   → ~/target ~/tracked ~/short_axis（全て latched）
  lemon_approach MoveIt で pregrasp へ  → ~/approach
  lemon_grasp    Servo で差し込んで掴む → ~/grasp ~/open      （C++、Servo 内蔵）
  arm_home       起動時姿勢へ戻す       → ~/go_home ~/set_home
  lemon_harvest_manager  上を順に呼ぶ  （auto_run:=false で起動しない）
```

ワークスペースは `~/lemon_ws`。`~/piper_ros` は別（`can_activate.sh` 用）。

---

## 3. ★ 停止ボタンに使える手段が無い

**これが精読で見つかった最大の問題。**

`lemon_grasp` には cancel / abort のサービスが無い。把持動作が止まるのは次の3つだけ。

| 止まる条件 | 時間 |
|---|---|
| `servo_timeout`（実機 40 秒）経過 → `publish_target()` が `tracker_->stopMotion()` | 最大40秒 |
| 対象が解除されて `lemon_` が空になり、目標の配信が止まる → PoseTracking が `target_pose_timeout`(0.5s) で中断 | **約0.5秒** |
| 正常終了 | — |

つまり **UI の停止ボタンが使える唯一の現実的な手段は `/lemon_target/release` を呼ぶこと。**
差し込み中（`Mode::TRACK`）なら約0.5秒で止まる。

ただし限界がある:

- **戻り動作（`Mode::RETREAT`）は止まらない。** 目標が `retreat_goal_` で、
  レモンの有無に依存しないため
- **cartesian 戻りはさらに止まらない。** `/arm_controller/follow_joint_trajectory` に
  まとめて投げた後なので、アクションを cancel するしかない
- `~/grasp` と `~/open` は `busy_` を共有する。**把持中はグリッパを開けない**
- Servo は `/arm_controller/joint_trajectory`（トピック）へ直接出すので、
  cancel できるゴールが存在しない

`lemon_approach` と `arm_home` は `/execute_trajectory` アクション経由なので、
**そちらは cancel できる**（`moveit_helper.execute()` が `cancel_goal_async` を持つ）。

### UI として取れる現実的な停止

1. `/lemon_target/release` を呼ぶ（差し込み中に効く・最速）
2. UI 自身がマネージャなら、次の段へ進むのをやめる
3. `/execute_trajectory` のゴールを cancel（approach / home 中に効く。要実装）

**これらを足しても「いつでも即座に止まる」にはならない。**
UI の停止ボタンは現状どおり「ソフト停止であって非常停止ではない」と
明示し続ける必要がある。物理的な非常停止が唯一の確実な手段。

---

## 4. ★ 除外リストで運用が詰まる

`lemon_target` は `release` のたびに、その位置を `exclusions` に**永久に**追加する
（`exclusion_radius` 3cm）。掴めても掴めなくても追加される。

視野内のレモンを一通り試すと**全部が除外され、新しい対象が選ばれなくなる**。
`/lemon_target/clear_exclusions` を呼ぶまで `waiting_target` のまま止まる。

**UI に「除外をクリア」ボタンが要る。** 画面が固まったように見える典型的な原因になる。
除外数は `release` の応答メッセージ（`"... N excluded"`）から拾える。

---

## 5. 検出表示 — 選択中のレモンも出せる

`LemonEllipse` に UI が必要とするものは全部ある。

| UI の項目 | 対応 | 備考 |
|---|---|---|
| `u, v` | `center_u, center_v` ÷ 画像幅高 | **px なので camera_info で割る** |
| `w, h` | `major_radius, minor_radius` ＋ `minor_angle` | 楕円。枠にするなら外接矩形 |
| `distance` | `center.z` | カメラ光学座標系なので z が奥行き |
| `confidence` | `score` | |
| `id` | `id`（YOLO の tracking ID） | **実機では `use_tracking:=False` なので空の可能性** |

**選択中のレモンの表示**: アダプタノードが `/lemon_target/target`（base_link）も
購読し、各検出の3D中心を base_link に変換して最も近いものに印を付ければよい。
TF と camera_info は `lemon_ellipse_node` と同じものが使える。

**注意**: `lemon_ellipse_node` は**マスクのある検出しか出さない**
（`if not d.mask.data: continue`）。セグメンテーションモデル必須。
また `d.bbox3d.frame_id` が画像の frame と一致しない検出は捨てる。

簡易案として `/lemon_ellipse/dbg_image`（楕円が描き込み済み）をそのまま流す手もある。

---

## 6. 座標指定・関節ジョグの作り直し — 道筋は明確

初版では「大」としたが、**参考実装が手元にある**。

### 使えるもの

`lemon_reach/moveit_helper.py` が、UI が必要とする操作をすべて同期ラッパで持っている。

| メソッド | 用途 |
|---|---|
| `compute_ik(frame, link, pos, quat, seed)` | `/compute_ik`。UI の座標指定の IK をこれに置換できる |
| `plan_and_execute(constraints, label)` | `/plan_kinematic_path` + `/execute_trajectory` |
| `get_joint_positions()` | `/get_planning_scene` から現在の関節角 |
| `execute()` 内の `cancel_goal_async` | **停止に使える** |

UI の自前 IK（`kinematics.py`）は捨てて `compute_ik` に寄せるのが筋。
相手の `lemon_approach` は特異点（条件数 > 100）と関節限界余裕（0.15 rad）まで
見ており、**UI の自前 IK より安全**。

### 関節ジョグの選択肢

| 方法 | 可否 |
|---|---|
| 相手の Servo を使う | **不可**。`lemon_grasp` のプロセス内にあり、把持中以外は pause されていて外から再開できない |
| 別途 `servo_node` を立てて `delta_twist_cmds` / `delta_joint_cmds` を送る | 可能。**ただし `/arm_controller/joint_trajectory` を2つのServoが共有するので、同時に動かさない配慮が要る** |
| `/arm_controller/follow_joint_trajectory` に短い軌道を投げる | 最も単純。ジョグ1回＝1アクション。cancel もできる |

**3番目が一番素直**で、停止もできる。

### 参考値（相手と揃えるべき）

```
planning_group  : "arm"
planning_frame  : "base_link"
ee_link         : "gripper_base"
tcp_offset      : 0.136   （ee_link の z 方向）
kinematics_tip  : "link6"
joint_limit_margin : 0.15
max_condition   : 100     （Servo は 120 から減速）
velocity_scaling / acceleration_scaling : 0.2 / 0.3（実機）
```

UI の `tcp_link: "gripper_base"` / `tcp_offset: [0, 0, 0.136]` が対応する。
**確認済み（§16-2）**: `gripper_base` は素の `.urdf` にあり、`link6` と**同一フレーム**
（`joint6_to_gripper_base` が単位変換）。`joint7`/`joint8` の原点が `xyz = 0 0 0.1358` なので、
**0.136 は指の付け根の位置**。UI 側の設定で完全に一致させられる。

---

## 7. サービスの応答と状態文字列（UI の表示に使う）

### 状態トピック（すべて `std_msgs/String`）

| トピック | 値 |
|---|---|
| `/lemon_harvest_manager/status` | `waiting_target` `approaching` `grasping` `going_home` `opening` `releasing` `error` |
| `/lemon_grasp/status` | `opening` `inserting` `closing` `retreating` ＋ 最終メッセージ |
| `/lemon_approach/status` | `approaching` ＋ 最終メッセージ |

### サービスの応答メッセージ

| サービス | 失敗時の message |
|---|---|
| `/lemon_approach/approach` | `busy` / `no target` / `failed to compute pregrasp` / `cannot insert straight into the lemon` / `unreachable after N attempts` / `execution failed` |
| `/lemon_grasp/grasp` | `not ready` / `busy` / `no target` / `insertion failed: timeout` 等 / `missed (gripper closed to X)` / `..., retreat failed: ...` |
| `/lemon_grasp/open` | `busy` / `opened` / `failed to open gripper` |
| `/arm_home/go_home` | `home is not recorded` / `busy` / `unreachable after N attempts` / `execution failed` |
| `/lemon_target/release` | `No target is locked` / `Released (x, y, z), N excluded` |
| `/lemon_target/clear_exclusions` | `Cleared N exclusions` |

**成功しても `success=false` になる例がある**: `grasp` は掴み損ねると
`success=false, message="missed (...)"` を返すが、動作自体は正常に終わっている。
UI は「失敗」ではなく「掴めなかった」と出すべき。

### グリッパ

`gripper_joint: joint7` / `gripper_open: 0.035` / `gripper_close: 0.0` /
`min_grasp_width: 0.005`（これより開いていれば把持成功とみなす）。
UI が独自にグリッパを開閉するなら `/gripper_controller/follow_joint_trajectory`。
**ただし把持中は `busy_` で弾かれる。**

---

## 8. ★ zip が相手のすべてではない

`piper_real/README.md` にこう書かれている。

> `piper_single_ctrl` 側は `~/piper_ros` に足した `publish_rate` パラメータで渡す。

**本家 piper_ros 自体にも改造が入っている**（`publish_rate` の追加）。
zip にはそれが含まれていない。

→ 開発用PCに揃えるときは、**`~/piper_ros` 側のコピーも要る**。
これが無いと `control_rate` が効かず、200Hz のまま CPU を食う。

ほかに zip に含まれない依存:

```
yolo_ros (mgonzs13)         別途 clone
piper_description           piper_ros
piper_with_gripper_moveit   piper_ros
piper                       piper_ros（改造済み）
topic_based_ros2_control    apt
realsense2_camera           apt
ultralytics                 pip
best.pt / lemon.pt          zip に同梱（6.8MB / 6.3MB）
```

---

## 9. 分散構成 — 相手も同じことをしている

`piper_real/launch/piper_real_rviz.launch.py` のコメント:

> ロボットPCで `piper_real_bringup.launch.py use_rviz:=false` を動かし、
> こちらは手元のPCで起動する。同じ LAN・同じ ROS_DOMAIN_ID なら
> DDS で move_group / TF / カメラ画像につながる。

**前提が揃っている。** 手元のPCに必要なものも明記されている:
ROS 2 Humble + MoveIt、ビルド済みの `piper_description` / `piper_with_gripper_moveit` / `piper_real`。

UI も同じ条件で入る。ただし:

- `lemon_msgs` は**両機で同一定義**が必須（カスタム msg）
- サービス呼び出しが LAN をまたぐ。`call_timeout` 120秒、
  `grasp` は実機で最大40秒かかる。**切断時の UI の挙動を決める必要がある**
- `use_rviz:=false` にすればミニPC側の RViz は上がらない

---

## 10. 起動時の衝突（確定）

| 相手が起動するもの | UI 側の対処 |
|---|---|
| `robot_state_publisher` | **`start_robot_state_publisher:=false` が必須** |
| `rviz2` | `rviz_bridge` は既定 false。衝突しない |
| `realsense2_camera` | `start_realsense:=false`（既定）のまま |
| `move_group` | 衝突なし |
| `joint_state_broadcaster` → `/joint_states` | **UI は publish してはいけない**（§11） |

`joy_to_piper_jointstates` の後継（`go2_piper_joy_teleop`）は **COLCON_IGNORE** で
ビルドされない。かつて懸念していたゲームパッドとの `/joint_states` 競合は消えている。

---

## 11. `/joint_states` の向きが逆（初版から変わらず）

| | これまでの UI | 相手 |
|---|---|---|
| `/joint_states` | UI が publish してアームを動かす | **joint_state_broadcaster が出力** |

UI が publish すると**アームは動かないうえにロボット状態が壊れる**
（RViz・MoveIt・TF・lemon_target・Servo の `joint_topic` が全部これを読む）。

`harvest_sequencer._publish_joint_command()` が守っていた piper の癖
（`velocity` 長さ7、`effort[6]`、`gripper` という名前）は
**`piper_joint_bridge` が同じ変換をしている。** UI 側の実装は重複で捨てられる。

---

## 12. 機能ごとの見積もり（更新）

| UI の機能 | これから | 規模 |
|---|---|---|
| 3D ビュー | `/joint_states` を購読するだけ。変更なし | **なし** |
| 遠隔アクセス | 変更なし | **なし** |
| カメラ映像 | トピック名が既定と一致。変更なし | **なし** |
| 状態表示 | 3つの status トピックを購読 | **小** |
| 検出枠＋選択中の表示 | アダプタノード1つ（LemonEllipseArray + target → UI の JSON） | **小〜中** |
| 収穫シーケンスのボタン | `_cmd_*` を Trigger 呼び出しに置換 | **中** |
| **除外クリアのボタン** | 新規。`/lemon_target/clear_exclusions` | **小**（必須） |
| 停止ボタン | `release` ＋ `/execute_trajectory` の cancel。**完全には止まらないことを明示** | **中** |
| 収穫位置 ①②③ | 相手に該当機能なし。捨てるか別途作るか | 設計判断 |
| 把持対象のクリック選択 | lemon_target が自動選択。両立には相手の改造が要る | 設計判断 |
| 座標指定 | `moveit_helper.compute_ik` + `plan_and_execute` に載せ替え | **中**（初版の「大」から下方修正） |
| 関節ジョグ | `/arm_controller/follow_joint_trajectory` に短い軌道 | **中** |

---

## 13. 決めていただきたいこと（初版から変わらず）

1. **把持対象を誰が選ぶか** — 自動選択のままにするか、UI でクリックしたいか
   （後者は相手のコード改造が要る）
2. **UI はマネージャか監視役か** — `auto_run:=false` で UI が1段ずつ呼ぶか、
   `auto_run:=true` のまま見るだけか
3. **座標指定・関節ジョグを残すか** — 相手のシーケンスは自動で完結している
4. **停止の落としどころ** — §3 の制約を踏まえて、どこまでを UI の責任にするか

---

## 14. 読んだ範囲

| ファイル | 行 | |
|---|---|---|
| `docs/lemon_harvest_system.md` | — | ○ |
| `lemon_msgs/msg/*.msg` | — | ○ |
| `lemon_grasp/src/lemon_grasp_node.cpp` | 965 | ○ |
| `lemon_reach/lemon_target_node.py` | 598 | ○ |
| `lemon_reach/lemon_approach_node.py` | 756 | ○ |
| `lemon_reach/moveit_helper.py` | 272 | ○ |
| `lemon_reach/lemon_harvest_manager_node.py` | 180 | ○ |
| `lemon_reach/arm_home_node.py` | 159 | ○ |
| `lemon_reach/kinematics.py` | 119 | ○ |
| `lemon_reach/lemon_ellipse.py` | 94 | ○ |
| `lemon_reach/lemon_ellipse_node.py` | 359 | ○ |
| `lemon_reach/launch/*.py` | 290 | ○ |
| `piper_real/` 全部（bridge / 2 launch / config / README） | — | ○ |
| `piper_isaacsim/` config・launch・README | — | △（xacro は topic 設定のみ確認） |
| `go2_piper_joy_teleop/` | — | △（COLCON_IGNORE でビルドされないため要点のみ） |
| `servo.yaml` / `servo_real.yaml` | — | ○ |

**追加で受領・確認済み**: 改造済み `piper_ros`（§16）。
`piper_with_gripper_moveit` もその中にあり確認した。

**未入手のまま**: `yolo_ros`（別途 clone）、`best.pt` / `lemon.pt` の学習内容、
`piper_ros` トップの `can_activate.sh`（ミニPC側にのみ必要）。

---

## 15. 補足: 相手の `kinematics.py` と UI の `kinematics.py`

偶然ほぼ同じ作りだった（固定軸 XYZ の rpy、Rodrigues、チェーン走査、
`J_i = z_i × (p_tip − p_i)`）。相手は特異点判定のため 6×n、UI は位置のみ 3×n。

**規約が一致していることの裏付けになる。** ただし UI 側は MoveIt の
`compute_ik` に寄せるので、最終的には使わなくなる見込み。

---

## 16. piper_ros を受領して確認した結果

受領物: `piper_ros/src/` のみ（8パッケージ、119ファイル、git なし）。
**トップの `can_activate.sh` と README は含まれていない**（ミニPC側にのみ必要）。

### 16-1. `publish_rate` の改造は入っている

`piper/piper/piper_ctrl_single_node.py` に `declare_parameter('publish_rate', 200)` があり、
`create_rate(self.publish_rate)` で使われている。**本家には無い追加。**
`piper_real_bringup.launch.py` が `control_rate`（既定100）を渡している。

`piper_ctrl_single_node_new.py` という別版も同梱されているが、`publish_rate` を持たず、
トピック名も違う（`joint_ctrl_states_feedback`）。
**`setup.py` の entry_point は `piper_ctrl_single_node:main` を指しているので、こちらは未使用。**

### 16-2. 手先の基準が完全に一致した

```
joint6_to_gripper_base   fixed   link6 → gripper_base   xyz=0 0 0  rpy=0 0 0   ← 単位変換
joint7                   prismatic  gripper_base → link7  xyz=0 0 0.1358
joint8                   prismatic  gripper_base → link8  xyz=0 0 0.1358
```

- `gripper_base` と `link6` は**同じ位置・同じ姿勢**。
  相手が `ee_link: gripper_base` と `kinematics_tip: link6` を併用できるのはこのため
- 相手の `tcp_offset = 0.136` は **joint7/8 の原点 0.1358 の丸め** = 指の付け根
  （指先ではない）

UI 側の設定:

```yaml
tcp_link: "gripper_base"
tcp_offset: [0.0, 0.0, 0.136]
```

### 16-3. UI の運動学は実機 URDF で動く（実測）

| | 値 |
|---|---|
| チェーン | `base_link → link1 → … → link6` を正しく構築 |
| ゼロ姿勢の手先（link6） | (0.0561, 0.0000, 0.2132) m |
| ゼロ姿勢の手先（+0.136） | (0.1916, 0.0000, 0.2250) m |
| 可動半径（link6 / 指付け根） | 0.748 m / 0.884 m（PiPER 公称 0.7m と整合） |
| 座標指定の往復（実URDF） | 到達誤差 0.096 mm、可動域内、状態は IDLE に復帰 |

**関節可動域は URDF と `common.JOINT_LIMITS` がほぼ一致。**
唯一の差は joint6（URDF ±2.0944 / UI ±2.094）で、UI 側がわずかに狭い＝安全側。

### 16-4. ★ `/joint_states` の向き逆転が両側から裏付けられた

本家 `piper/launch/start_single_piper.launch.py`:

```python
remappings=[
    ('joint_ctrl_single', '/joint_states'),   # ← これがあるから UI が動いていた
]
```

チーム版 `piper_real_bringup.launch.py` の `piper_single_ctrl` には**この remap が無い**。
したがって:

| | 指令の受け口 | `/joint_states` の役割 |
|---|---|---|
| 本家 | `/joint_states`（remap 経由） | **入力**（UI が publish して動かす） |
| チーム版 | `/joint_ctrl_single` | **出力**（joint_state_broadcaster） |

§11 の結論が確定した。

また `joint_callback` の実装で、UI が守っていた癖も裏付けられた:
`position[6]`＝グリッパ（×`gripper_val_mutiple`）、
`velocity` の長さが**ちょうど7**のとき `velocity[6]`＝速度1–100%、
`effort[6]` は 0.5–3 にクリップ×1000（NaN なら 1000）、
フィードバックの関節名は `['joint1'…'joint6', 'gripper']`。
**これらは全て `piper_joint_bridge` が肩代わりするので、UI 側の実装は捨てられる。**

### 16-5. ★ UI 側に不具合を1件発見（実機URDFで露見）

実機 URDF で IK の往復を100回試すと **1件取りこぼす**（合成URDFでは0件だった）。

```
目標 (0.029, 0.011, 0.111)  原点から 0.115 m
正解の関節角は joint5 が上限まで 0.03 rad
到達できた誤差 0.12 mm  ← 許容値 0.1 mm をわずかに超えて「失敗」判定
```

**原因**: `solve_ik` の許容誤差 `tol=1e-4`（0.1mm）が厳しすぎる。
根元に近い折りたたんだ姿勢（関節限界に張り付く特異姿勢）で 0.12mm まで詰めて止まる。

**影響**: 到達可能な座標を「届きませんでした」と断ることがある（約1%、根元付近）。

**対策（未実施）**: `tol` を `5e-4`（0.5mm）にする。
レモン検出の精度は cm 単位なので 0.5mm は十分すぎる。
`tests/test_kinematics.py` の「解けたと報告したときの誤差は 0.1mm 未満」も
0.5mm に合わせて直す。

### 16-6. かつての `/joint_states` 競合は消えている

`joy_to_piper_jointstates` は**この piper_ros に存在しない**。
後継の `go2_piper_joy_teleop` も `COLCON_IGNORE` でビルドされない。
「ゲームパッドと UI が同じトピックを奪い合う」という長年の制約は無くなった。

### 16-7. `piper_description.xacro` について

- `$(find piper_gazebo)` を参照している → **xacro を使うなら `piper_gazebo` のビルドが必須**。
  同梱されているので問題はない
- xacro は `world → base_link` の固定関節を追加する（原点は単位＝無害）。
  UI がどちらを読んでも手先位置は同じ値になることを確認した
- 相手の `piper_isaacsim.urdf.xacro` は `piper_description.xacro` を include しているので、
  **チーム版の robot_description も `piper_gazebo` に依存する**

### 16-8. `PiperStatusMsg` は UI の表示とそのまま合う

`err_code` / `joint_N_angle_limit` / `communication_status_joint_N` が定義どおり。
UI の「エラーコード」「可動限界」「関節通信」の表示は無改造で動く。
ただし**チーム版では `/arm_status` を出すのは `piper_single_ctrl` のまま**なので、
UI のパラメータ `arm_status_topic: /arm_status` はそのままでよい。

### 16-9. 開発用PCでビルドが要るパッケージ（確定）

| パッケージ | 理由 |
|---|---|
| `piper_description` | UI の3Dビュー・座標指定、robot_state_publisher |
| `piper_msgs` | `/arm_status` の購読、`PosCmd` |
| `piper_with_gripper_moveit` | MoveIt 設定（UI が MoveIt 経由で動かすなら必須） |
| `piper_gazebo` | `piper_description.xacro` が `$(find piper_gazebo)` を参照 |
| `lemon_msgs` | 検出の購読。**両機で同一定義が必須** |
| `piper_real` | `piper_real_rviz.launch.py` を使うなら |
| `piper`（CAN ドライバ） | **不要**（実機に繋がらないので） |
