# 開発引き継ぎ（piper_arm_ui v0.3.0 / 2026-10-19 時点）

これから **アームの開発を続ける人** に向けた資料。
何をどう作ってあるか、どこを差し替えれば伸ばせるか、実際に踏んだ落とし穴をまとめる。

- 操作方法 → 別資料「PiPER アーム操作マニュアル」
- 環境構築 → [SETUP_ubuntu2204_humble.md](SETUP_ubuntu2204_humble.md) /
  [SETUP_dev_pc.md](SETUP_dev_pc.md)
- 起動オプションやトラブル対応 → [../README.md](../README.md)
- 別ネットワークからの利用 → [SETUP_remote_access.md](SETUP_remote_access.md)

---

## 1. 設計の方針

このUIは **デモを成立させるための骨組み**。認識も把持計画も本実装が未定だったので、
**「決まっていない部分を後から差し替えられる形」** を最優先にした。

### 1.1 UI とノードの間は JSON で運ぶ

UI ↔ シーケンサのやり取りは、すべて `std_msgs/String` に載せた JSON。
カスタムメッセージを定義するとビルドが必要になり、項目を 1 つ増やすたびに両側を作り直す
ことになる。仕様が固まるまでの試作にはこの方式が向く。
固まったら独自 msg/srv に置き換えればよい（`common.py` のトピック定数が起点）。

### 1.2 形と動きを分ける

3D 表示は **形（URDF とメッシュ）は開いたときに一度だけ、動き（関節角）はトピックで
流し続ける**。当初は RViz の画面を動画で送っていたが、遅延・解像度・CPU のすべてで
不利だったので作り直した（→ §9）。通信量は初回 0.8MB と毎秒数百バイトで済む。

### 1.3 実機を壊さない既定値にする

`enable_arm_command` は既定 `false`。UI を開いただけでは指令が飛ばない。
既存のゲームパッドノード（`joy_to_piper_jointstates`）と同じ `/joint_states` を使うため、
**両方が同時に publish するとアームが暴れる**。この競合は設計上の制約として残っている。

---

## 2. データの流れ

```
ブラウザ ──HTTP/WebRTC:8080──► ui_server ──► 映像・静的ファイル・URDF 配信
   │                             │
   └── WebSocket:8080/rosbridge ─┘（権限判定つきの中継）
                                 ▼
                    127.0.0.1:9090 rosbridge ──► /arm_ui/command ──► harvest_sequencer
                            ▲                                    │
                            └── /arm_ui/state ───────────────────┘
                                                                 │ /joint_states
                                                                 ▼
                                        piper_ros（本家・無改造）── CAN ──► PiPER 実機
                                          → /joint_states_single, /arm_status, /end_pose
```

ノードは 4 つ（`ui_server` / `harvest_sequencer` / `dummy_perception` / `rviz_bridge`）。
加えて launch が `rosbridge_websocket` と `robot_state_publisher` を起動する。

**ブラウザが繋ぐ先は 8080 ひとつだけ**にしてある（v0.2.0 から）。
rosbridge へも `ui_server` の `/rosbridge` を経由する。理由は 3 つ:

1. トンネル 1 本・VPN 1 本で丸ごと外から使える（→ `docs/SETUP_remote_access.md`）
2. https で開いたとき `wss://` になり、混在コンテンツで弾かれない
3. **権限判定が 1 か所に集まる。** この UI には認証が無いので、
   遠隔（トンネル経由）からの `publish` はここで落としている

手元か遠隔かの判定は `public_port` に届いたかどうかが第一。
`X-Forwarded-For` を見る判定も入れてあるが、**それだけに頼ってはいけない**。
ヘッダを付けない中継もあるし、WebSocket の upgrade だけヘッダが落ちる実装も
実在する（Chromium で確認した）。そこが権限判定の入口なので、落ちると穴になる。

`ui_server` を通るのは「映像」「静的ファイル」「URDF 配信」とこの中継だけ。
中継は素通しが基本なので、**トピックを増やす分にはここを触る必要はない**
（遠隔にも常時許したいトピックがあるときだけ `remote_access.py` に足す）。

---

## 3. ファイルの地図

```
piper_arm_ui/
├─ piper_arm_ui/            # ROS ノード（Python）
│  ├─ common.py             # 関節名・可動域・トピック名・状態定義（78行）
│  ├─ harvest_sequencer.py  # 収穫の状態機械と軌道生成（701行）★中核
│  ├─ kinematics.py         # URDF から順運動学と逆運動学（412行）
│  ├─ ui_server.py          # HTTP + WebRTC + URDF 配信 + 中継（579行）
│  ├─ remote_access.py      # 別ネットワークからの利用と権限（324行）
│  ├─ urdf_scene.py         # URDF/STL → ブラウザ用に変換（241行）
│  ├─ video_source.py       # WebRTC の映像トラック（226行）
│  ├─ rviz_bridge.py        # RViz 画面配信（644行・任意機能）
│  └─ dummy_perception.py   # 検出のダミー（96行）★差し替え対象
├─ web/                     # ブラウザ側（ビルド不要・素の JS）
│  ├─ index.html            # 画面の構造（260行）
│  ├─ css/style.css         # 配色トークンとレイアウト（496行）
│  └─ js/
│     ├─ app.js             # 画面の制御全般（947行）★中核
│     ├─ robotview.js       # URDF の 3D 描画と順運動学（456行）
│     ├─ armview.js         # 簡易ビューと俯瞰マップ（332行）
│     ├─ rosbridge.js       # rosbridge クライアント自作（125行）
│     ├─ webrtc.js          # WebRTC 受信（124行）
│     └─ vendor/three.min.js  # 同梱 three.js r149（MIT）
├─ launch/arm_ui.launch.py  # 起動一式・引数 34 個（398行）
├─ config/arm_ui.yaml       # 全ノードのパラメータ（159行）
└─ setup.py                 # web/ を丸ごと share へ入れる仕掛け
```

まず読むべき 3 ファイル:

| ファイル | 何が書いてあるか |
|---|---|
| `common.py` | 関節名・可動域・トピック名・状態の定義。**全体の語彙**。まずここに定数を置く |
| `harvest_sequencer.py` | 収穫の手順そのもの。ボタン1つ＝`_cmd_*` メソッド1つ |
| `web/js/app.js` | 画面のすべての制御。上から「ROS 接続 → 映像 → 3D → 状態表示 → 操作」 |

**ブラウザ側にビルドは無い。** npm も webpack も使っていない。
`--symlink-install` でビルドしてあれば `web/` を直せばブラウザの再読込だけで反映される。

---

## 4. 起動の仕組み

`ros2 launch piper_arm_ui arm_ui.launch.py` の 1 行で以下が立ち上がる。

| ノード | 備考 |
|---|---|
| `ui_server` | :8080 — 画面・映像・URDF 配信・rosbridge への中継。`public_port` で公開用の 2 口目も開く |
| `harvest_sequencer` | 状態機械。既定は監視のみ |
| `rosbridge_websocket` | 127.0.0.1:9090 — `start_rosbridge` で切れる。**外には開かない** |
| `robot_state_publisher` | `/robot_description` と TF |
| `dummy_perception` | 仮。`use_dummy_perception:=false` で切る |

`rviz_bridge` と `realsense2_camera` は既定では起動しない。

パラメータは **YAML が土台、launch 引数が上書き**の二段構え。
引数 34 個は「接続 / カメラ / 3D と RViz / 安全・動作 / 遠隔アクセス」の 5 群に分かれる。

```bash
# 実機なしで UI を一通り動かす（開発の基本形）
ros2 launch piper_arm_ui arm_ui.launch.py use_dummy_video:=true enable_arm_command:=true

# 実機に繋ぐ。まずは監視のみ
ros2 launch piper_arm_ui arm_ui.launch.py

# 別ネットワークの人に閲覧だけさせる
ros2 launch piper_arm_ui arm_ui.launch.py \
    allow_remote_control:=false ice_servers:=stun:stun.l.google.com:19302
```

---

## 5. 3D の生成経路

```
piper_description/urdf/piper_description.urdf
  │  ① XML を解析 → リンク階層・関節の原点/軸/可動域
  ├─────────────────────────────► /api/robot   (JSON 3.8KB)
  │
  └─ meshes/*.STL（10個・計 9.2MB・18.4万三角形）
       ② 頂点クラスタリングで簡略化（格子 1mm）
       ③ 位置 float32 + 索引 uint32 の独自バイナリに詰める
     ─────────────────────────► /api/mesh/<id> (計 1.9MB / gzip 0.8MB)
```

| 格子 | 三角形 | 転送量 | 形状誤差 |
|---|---|---|---|
| 元の STL | 183,746 | 9.19 MB | — |
| 0.5 mm | 139,018 | 2.48 MB | 0.02 mm |
| **1.0 mm（既定）** | 107,267 | 1.91 MB | 0.06 mm |
| 2.0 mm | 67,842 | 1.19 MB | 0.51 mm |
| 3.0 mm | 47,379 | 0.81 MB | 1.04 mm |

簡略化は numpy だけで書いてある（外部のメッシュ処理ライブラリ不要）。
変換にかかるのは起動時の 0.5 秒だけ。細かさは `mesh_simplify_cell` で変える。

**座標系と回転の規約（重要）**

```js
// URDF の rpy は固定軸 X→Y→Z。three.js では 'ZYX' 指定が対応する。
// ここを間違えると静かにリンクの向きがずれる。
pivot.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
```

ROS は Z 上、three.js は Y 上。`robotview.js` は `world` グループを
`rotation.x = -π/2` して吸収している。したがって `getWorldPosition()` は three の座標を
返すので、ROS 座標で比べたいときは `world.worldToLocal()` を通すこと。

見た目では検証にならないため、**ブラウザ内の順運動学が URDF と一致するかを数値照合する
テスト**を入れてある（`test_robot_view.py`、誤差 0.000000 mm）。
座標系や Euler 順序を触ったら必ず走らせること。

---

## 5.5 座標指定の運動学

左下の「座標指定（手先位置）」は `kinematics.py` が支えている。

```
URDF ──解析──> ArmChain（base から joint6 までの1本の鎖）
                 ├─ fk(q)              関節角 → 手先の 4x4
                 ├─ fk_and_jacobian(q) 位置と 3xN ヤコビアンを1回の走査で
                 └─ solve_ik(target)   目標位置 → 関節角
```

**解いているのは位置 3 自由度だけ。** 手先の向きは解に任せている。
掴む向きまで決めたいなら、ヤコビアンに姿勢の 3 行を足して 6 次元にする
（`fk_and_jacobian` と `solve_ik` の残差の作りかたを変える）。

IK は**減衰最小二乗を Levenberg-Marquardt のやりかた**で解いている。

```
Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
  誤差が減った → 受け入れて λ を下げる（Gauss-Newton 寄り＝速い）
  誤差が増えた → 棄却して λ を上げる（最急降下寄り＝確実）
```

**λ を固定にしていたときは、到達可能な目標の 2.5% を取りこぼしていた。**
「入れた座標が到達可能なのに届かないと言われる」のは UI として使えないので、
1 歩ごとの受け入れ判定を入れてある。いまは 200 回試して取りこぼし 0。

収束しなければ初期姿勢を振り直して再挑戦する（既定 12 回）。
**1 回目は必ず現在姿勢から始める** — 座標ジョグのように近くを狙うときに、
遠い別解が選ばれてアームが大きく跳ぶのを避けるため。

### 触るときに気をつけること

| | |
|---|---|
| **`time_budget` を短くしない** | 速度調整の設定ではなく最悪時の打ち切り。0.12 秒にすると到達可能な目標の 4% を落とし、合計時間はほとんど変わらなかった（1 回 35ms 程度のため）。既定 0.6 秒 |
| **届かない目標は先に弾く** | `obviously_out_of_reach()`。IK を回すと全回数ぶん粘って 0.5 秒かかり、その間 50Hz の制御ループが止まる |
| **可動域は `common.JOINT_LIMITS` を渡す** | URDF の limit とは微妙に違う（joint6 が 2.094 と 2.0944）。揃えないと「UI では入る角度が IK では出ない」ずれが起きる |
| **rpy は固定軸 X→Y→Z** | `robotview.js` の three.js `'ZYX'` と同じ規約。取り違えると静かにずれる。scipy の extrinsic `'xyz'` と照合するテストで守っている |

### 実機に合わせる（必ずやる）

UI の **手先位置(指令)** はこちらの順運動学、**手先位置(実機)** は piper の
`/end_pose`。URDF に「グリッパの指の間」のリンクが無いので、既定では
`joint6` の子リンクの原点が基準になり、**両者はずれうる**。
2 つを並べて出しているので、差をそのまま `tcp_offset` に入れて揃える
（→ README「座標指定を実機に合わせる」）。

---

## 6. データ契約

### トピック

| topic | type | 向き |
|---|---|---|
| `/arm_ui/command` | String (JSON) | UI → シーケンサ |
| `/arm_ui/state` | String (JSON) | シーケンサ → UI（10Hz） |
| `/arm_ui/log` | String (JSON) | シーケンサ → UI |
| `/arm_ui/detections` | String (JSON) | 認識 → UI / シーケンサ |
| `/joint_states` | JointState | シーケンサ → piper（有効時のみ） |
| `/joint_states_single` | JointState | piper → シーケンサ / UI |
| `/arm_status` | PiperStatusMsg | piper → シーケンサ（任意） |
| `/end_pose` | geometry_msgs/Pose | piper → シーケンサ（任意） |
| `/dog_ui/mode` | String | UI → 走行系（骨組みのみ） |
| `/rviz_ui/image`, `/rviz_ui/input` | Image, String | RViz 併用時のみ |

### 検出結果 — 認識を作る人はここだけ見ればよい

この形で `/arm_ui/detections` に流せば UI 側は無改造で動く。

```json
{"targets": [
  {"id": 1,
   "u": 0.31, "v": 0.44,
   "w": 0.13, "h": 0.17,
   "distance": 0.38,
   "confidence": 0.92,
   "label": "lemon"}
]}
```

`u, v` は画像内の正規化座標で、そのまま検出枠の描画位置になる。
`distance` は把持姿勢の計算に使われる。

### コマンド

`start_selection` / `select_target(id)` / `plan_grasp` / `grasp` /
`goto_basket(index 1–3)` / `release` / `home` / `abort` /
`set_enable(value)` / `jog(joint, delta)` / `gripper(close)` /
`move_to_xyz(x, y, z)` / `jog_xyz(axis 0–2, delta)`

追加は `harvest_sequencer` に `_cmd_<名前>(self, cmd)` を生やすだけ。
ディスパッチは `getattr(self, '_cmd_' + name)` なので登録作業は要らない。

### piper への関節指令 — 仕様が独特

| field | piper 側の解釈 |
|---|---|
| `name` / `position` | joint1〜6 は名前で参照、`position[6]` がグリッパ開度。joint8 まで出すのは RViz 用 |
| `velocity` | **長さがちょうど 7 のときだけ** `velocity[6]` が全体動作速度 1–100%。それ以外は 100% |
| `effort` | `effort[6]` が把持力。0.5〜3 にクリップされ ×1000。**全ゼロだと 0.5 に丸められて弱い** |

フィードバックの `/joint_states_single` は、グリッパを `joint7` ではなく
**`gripper`** という名前で出す。UI 側は両方を受けられるようにしてある。

### HTTP API

| endpoint | 返すもの |
|---|---|
| `GET /` | 画面本体 |
| `GET /api/info` | 接続先・カメラ構成・3D の可否・ICE サーバ・手元か遠隔か |
| `GET /api/robot` | URDF から作ったシーン定義（JSON） |
| `GET /api/mesh/{mesh_id}` | リンク形状のバイナリ（1日キャッシュ） |
| `POST /offer` | WebRTC のシグナリング |
| `GET /static/*` | CSS・JS・同梱 three.js |
| `WS /rosbridge` | rosbridge への中継。**権限判定はここ** |
| `GET /api/control` | いまの操作権（許可中か・残り秒数・遠隔からの申請） |
| `POST /api/control/grant` | 遠隔からの操作を許可（**手元からのみ**。他は 403） |
| `POST /api/control/revoke` | 許可の取り消し（**手元からのみ**） |
| `POST /api/control/request` | 遠隔から「操作したい」と手元に知らせる |

---

## 7. 状態機械と収穫かご

12 状態。`_cmd_*` が状態を進め、`_control_loop()` が 50Hz で軌道を刻む。

```
IDLE ─①─► SELECTING ─枠クリック─► SELECTED ─②─► PLANNING ─自動─► PLANNED
                                                                     │③
    DONE ◄─自動─ RELEASING ◄─⑤─ AT_BASKET ◄─自動─ MOVING_TO_BASKET ◄─④─ GRASPED
```

軌道は**関節空間の線形補間に smoothstep の加減速**をかけただけ。障害物回避はしていない。

### 収穫かごの位置

`joint1`（旋回）がそのまま置き場の方位になる構造。動かすときは先頭の値だけ変えればよく、
右下の俯瞰図もこの値から順運動学で描いているので**絵とずれない**。

| | joint1 | 方位 | 距離 |
|---|---|---|---|
| ① | +2.094 | +120° 左後方 | 0.265 m |
| ② | −2.094 | −120° 右後方 | 0.265 m |
| ③ | +1.047 | +60° 左斜め前 | 0.190 m |

> **後方へ振るときの注意**: ±120° は可動域（±150°）内だが、アームは本体の脇を通って
> 後ろへ回る。ホームから ① へは joint1 が 0°→120° と連続で振れるので、
> `motion_speed` を 20〜40 に落として干渉を確かめること。

**可動域の注意**: joint6 だけカタログ（±100°）より広い ±120° になっている
（URDF がそう）。実機で端まで回すとファーム側の可動限界に当たる可能性がある。

---

## 8. 差し替えポイント

**ここがこの引き継ぎの本題。** 4 箇所が仮実装で、境界をトピックかメソッドで切ってある。

### 優先 高 — レモンの検出

いまは `dummy_perception` が 3 個の枠を 2Hz で巡回させているだけ。
YOLO などの認識ノードに置き換える。**§6 の JSON を同じトピックに流せば UI 側は無改造**。

- 置き換え先: `piper_arm_ui/dummy_perception.py`
- 起動時に `use_dummy_perception:=false`

### 優先 高 — 把持姿勢の計算

いまは画像座標と距離から関節角への**線形写像のスタブ**。
候補は MoveIt2 での IK＋経路計画 / piper の `/pos_cmd` にエンドエフェクタ姿勢を投げる /
学習ベースの把持推定（GraspNet 等）。

- 置き換え先: `harvest_sequencer.py` の `_compute_grasp_pose(target)` 1 メソッド
- 入力は検出の辞書、出力は joint1..6 の配列（`None` で失敗）

### 優先 中 — 軌道生成

関節空間の線形補間のみで障害物回避なし。枝を避ける必要が出たら MoveIt2 の経路計画へ。

- 置き換え先: `harvest_sequencer.py` の `Trajectory` クラスと `_start_trajectory()`

### 優先 低 — 走行モード（犬型の制御）

移動用UI のボタンは `/dog_ui/mode` に文字列（`move / harvest / return / stop`）を
publish するだけの骨組み。走行系のインタフェースが決まったら合わせる。

- 置き換え先: `web/js/app.js` の走行モード部分

> UI ↔ シーケンサが JSON トピックで切れているので、上の 4 つを全部入れ替えても
> **ブラウザ側のコードは 1 行も変わらない。**

---

## 9. 検証のしかた

テストは**実物を動かして確かめる**方式。本物のノード・本物の rosbridge プロトコル・
本物の WebRTC・本物の Chromium を通す。この形にしたのは、
実際に**これでしか見つからないバグを 4 件見つけた**ため。

| ファイル | 何を守っているか |
|---|---|
| `test_e2e.py` | 収穫シーケンス一巡・検出枠クリック・映像受信・複数ブラウザ（23項目） |
| `test_robot_view.py` | 3D の組み立てと**順運動学の URDF 一致**（29項目） |
| `test_basket_map.py` | 俯瞰図の ①②③ が実際の姿勢と一致（21項目） |
| `test_joint_cmd.py` | piper へ出す JointState の形 |
| `test_launch_description.py` | launch が組み立てる内容・URDF の選択 |
| `test_static_assets.py` | symlink-install でも CSS/JS が配信されるか |
| `test_duplicate_nodes.py` | ノード二重起動の検出 |
| `test_port_config.py` | ポート衝突の検出とずらし |
| `test_rviz_view.py` / `test_rviz_smoothness.py` | RViz 併用時の視点と配信レート |
| `test_compressed.py` | 分散構成の圧縮画像 |
| `test_remote_access.py` | **遠隔からの操作が本当に ROS に届かないか**（29項目） |
| `test_remote_browser.py` | 公開用ポートを本物の Chromium で開き、閲覧のみ→許可→取り消しを通す |
| `test_kinematics.py` | **FK/IK の数値検証**（25項目）。scipy と照合、ヤコビアン vs 数値微分、IK の往復 |
| `test_cartesian.py` | 座標指定コマンドをシーケンサに流して確かめる（41項目） |
| `test_xyz_ui.py` | 座標指定トグルを Chromium で押す（22項目） |

```bash
python3 e2e/test_robot_view.py                    # 個別に
for t in e2e/test_*.py; do python3 "$t" >/dev/null && echo "OK  $t" || echo "NG  $t"; done
```

`run_stack.py` が ROS 抜きでノード群を動かす仕掛け（`mini_ros.py` が rclpy を差し替える）
なので、**ROS が入っていない環境でも走る**。ただし piper 実機との結合は当然カバー外。

**触ったら走らせるもの**: 運動学や IK → `test_kinematics.py` と `test_cartesian.py`
（**IK の良し悪しは画面を見ても分からない。必ず数値で確かめる**）/
座標系や Euler 順序 → `test_robot_view.py` /
収穫位置や姿勢 → `test_basket_map.py` / piper への指令の形 → `test_joint_cmd.py` /
launch や設定 → `test_launch_description.py` /
`remote_access.py` や `/rosbridge` の周り → `test_remote_access.py` と
`test_remote_browser.py`（**権限判定は目視では確認できない。必ず両方走らせる**）

---

## 10. 踏んだ落とし穴

実際に時間を溶かしたもの。いずれも対策済みだが、周辺を触るときは思い出すこと。

| 症状 | 原因と対策 |
|---|---|
| **装飾の無い素の HTML が出る** | `--symlink-install` で `share/.../web/` の中身がシンボリックリンクになり、aiohttp の静的配信が既定で 404 にする。`follow_symlinks=True` で対策済み |
| **画面全体がチカチカする** | ノードの二重起動。UI が 2 つ分の状態を交互に受け取る。いまは検出して画面に警告を出す |
| **3つの映像が全部同じ絵になる** | aiortc は複数トラックを 1 つの MediaStream にまとめる。`event.streams[0]` ではなく `new MediaStream([event.track])` |
| **グリッパのフィードバックが常に 0** | piper は `joint7` ではなく `gripper` という名前を使う |
| **把持力が弱い / 速度指定が効かない** | `velocity` の長さがちょうど 7 のときだけ速度指定が効く。`effort` 全ゼロは 0.5 に丸められる |
| **`piper_gazebo not found` で起動失敗** | `piper_description.xacro` が gazebo の設定を参照している。素の `.urdf` を使う（既定はそちら） |
| **8080 / 9090 / :99 が埋まって起動失敗** | 前回の残骸か別プロジェクト。起動前に検出して対処法つきで落ちる |
| **リンクの向きが静かにずれる** | URDF の rpy は固定軸 XYZ、three.js では `'ZYX'` 指定。順運動学の数値照合テストで守っている |
| **ヘッダによる遠隔判定がすり抜ける** | Chromium は `extra_http_headers` を **WebSocket の upgrade に付けない**。ヘッダだけで権限を決めると、そこがちょうど入口なので静かに穴が開く。`public_port` に届いたかを第一の判定にして回避した |
| **空欄の座標が 0 として送られる** | `Number('')` は `NaN` ではなく **`0`**。`Number.isFinite` だけで弾くと、入れ忘れた軸が 0 扱いになって意図しない所へアームが動く。文字列として空かを先に見る |
| **IK が「到達可能なのに届かない」と言う** | 減衰（λ）を固定にすると局所解で止まり、2.5% を取りこぼす。1 歩ごとに受け入れ判定をして λ を調整する（§5.5） |
| **IK の時間制限が解を削る** | 打ち切りを 0.12 秒にすると到達可能な目標の 4% を落とす。速度調整のつもりで短くしないこと（§5.5） |

### RViz 映像方式をやめた経緯

当初は仮想ディスプレイ上の rviz2 を画面キャプチャして WebRTC で送っていた。
遅延とカクつきの原因を実測したところ、**VP8 エンコードが 1 枚 30ms**（1280x720）かかり
これが上限を作っていた。解像度を落として fps を上げる調整もしたが、根本的には
「動画で送る」構造の限界だったので URDF をブラウザで直接描く方式に作り替えた。
RViz 方式のコードは `rviz_bridge.py` に残してあり、TF や点群を見たいときは
`start_rviz_bridge:=true` で併用できる。

---

## 11. 次にやること

| | やること | 目安 |
|---|---|---|
| 1 | 実機で一巡させて、収穫位置 ①②③ が実際のかご位置と合うか確かめる。合わなければ `basket_pose_*` の `joint1` を調整 | 半日 |
| 2 | 認識ノードを載せる。まず静止画で `/arm_ui/detections` を流して UI 側の枠が合うか見る | 数日 |
| 3 | 把持姿勢の計算を本実装にする。**ここがデモの成否を分ける** | 1〜2週 |
| 4 | 枝の回避が要るなら軌道生成を MoveIt2 に | 2週〜 |
| 5 | 走行系と繋ぐ（`/dog_ui/mode` の購読側） | 別担当と調整 |

### 設計上の宿題

- **ゲームパッドと UI の競合**: 両方が `/joint_states` に出す構造はそのまま。
  本番運用するなら調停ノードを 1 つ挟むか、どちらかを `/pos_cmd` 側に寄せるのが筋。
- **認証が無い**: ログインもパスワードも無い。URL を知っている人 = 操作できる人。
  別ネットワークからの利用に対しては「手元か遠隔か」で権限を分けて凌いでいる
  （`remote_access.py`）が、これは**認証の代わりにはならない**。
  同じ LAN・同じ VPN の中は全員が手元＝フル操作である。
  外部に長期間出すなら、トンネル側で認証をかける（Cloudflare Access 等）か、
  UI 自体にログインを入れるかのどちらかが要る。
- **JSON 契約の型が緩い**: 試作としては正解だが、仕様が固まったら独自 msg/srv にしたほうが
  事故が減る。
- **3D ビューは視覚化のみ**: 干渉チェックはしていない。RViz の点群と組み合わせるか、
  MoveIt2 の planning scene を持ち込む必要がある。

---

仮実装には全部コメントで理由と差し替え先を書いてある。迷ったら次で辿れる。

```bash
grep -rn "差し替えポイント\|仮\|ダミー" piper_arm_ui/
```
