# piper_arm_ui — Claude Code 向けのプロジェクト記憶

犬型ロボット搭載アーム（AgileX **PiPER**）でレモンを自動収穫するデモの
**アーム操作UI**。ROS 2 Humble / Ubuntu 22.04。

このファイルは Claude Code が自動で読む。**作業前にここを読み、
迷ったら README.md と docs/HANDOVER.md を見る。**

---

## 0. まずこれを確かめる（初回だけ）

```bash
colcon build --packages-select piper_arm_ui --symlink-install
source install/setup.bash

python3 tests/check_real_urdf.py      # パッケージの中から
```

**この UI の運動学は、本物の `piper_description.urdf` で動かした実績が無い。**
同梱テストは作りもののURDF（`tests/test_arm.urdf`）で検証している。
関節名やリンク構成が想定と違えば座標指定だけが静かに無効になるので、
**実機のURDFがある環境で最初に 1 回これを走らせて確認すること。**

落ちたら、そこが最初の仕事。`kinematics.ArmChain.from_urdf` の
`ChainError` が理由を名指しする。

---

## 1. 環境について

**このファイルは UI パッケージ自体の説明。
どのPCで動かしているか・何に繋ぐかは、ワークスペース直下の `CLAUDE.md` を見る。**

実機が繋がっている環境では、そちらに安全上の約束が書いてある。**先にそれを読む。**

### 実機なしで動かす（開発用PCの場合）

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_dummy_video:=true enable_arm_command:=true
# → ブラウザで http://localhost:8080/
```

カメラはテストパターン、検出はダミー（`dummy_perception.py`）、
アームは指令した関節角がそのまま 3D ビューに反映される。

> **`enable_arm_command:=true` は、実機が ros2_control 構成で動いている環境では
> 付けてはいけない。** `/joint_states` の向きが逆になり、現在姿勢が壊れる。
> 安全弁が自動で止めるが、そもそも付けないこと（ワークスペースの CLAUDE.md §4）。

**実機が要る作業（tcp_offset の校正、かご位置の実測合わせ）は、
実機が無い環境ではできない。** やろうとしたら、そう言って止まること。

---

## 2. 設計の前提（崩さないこと）

| | |
|---|---|
| **UI ↔ ノードは JSON** | `std_msgs/String` に JSON を載せる。カスタム msg を足すとビルドが要るので、仕様が固まるまではこの形。トピック定数は `common.py` |
| **`common.py` が全体の語彙** | 関節名・可動域・トピック名・状態の定義。定数を足すならまずここ |
| **ブラウザ側にビルドは無い** | npm も webpack も使わない。素の JS を `<script>` で読むだけ。`--symlink-install` なら `web/` を直せば再読込で反映される |
| **CDN は使わない** | 現場はオフライン。three.js は `web/js/vendor/` に同梱（r149、UMD） |
| **ゲームパッドと共存する** | `joy_to_piper_jointstates` と同じ `/joint_states` に出す。両方が publish すると**アームが暴れる**ので `enable_arm_command` は既定 false |
| **rosbridge は外に開かない** | ブラウザは `ui_server` の `/rosbridge` 経由。rosbridge 自体は `127.0.0.1`。ここを開けると権限判定を素通りされる（`remote_access.py`） |
| **コメントは日本語** | 既存コードに合わせる。「何をしているか」より**「なぜそうしたか」**を書く |
| **他のワークスペースは読むだけ** | `~/piper_ros` と `~/lemon_ws` はミニPCで動いている実体。読む・ビルドするのはよいが**編集しない**。直してもミニPCには反映されず、両機で定義がずれて通信が壊れる |

---

## 3. ファイルの地図

```
piper_arm_ui/
├─ piper_arm_ui/            # ROS ノード
│  ├─ common.py             # 関節名・可動域・トピック名・状態定義 ★全体の語彙
│  ├─ harvest_sequencer.py  # 収穫の状態機械と軌道生成 ★中核
│  ├─ kinematics.py         # URDF から順運動学と逆運動学（座標指定用）
│  ├─ ui_server.py          # HTTP + WebRTC + URDF配信 + rosbridge中継
│  ├─ remote_access.py      # 別ネットワークからの利用と権限
│  ├─ urdf_scene.py         # URDF/STL → ブラウザ用に変換
│  ├─ video_source.py       # WebRTC の映像トラック
│  ├─ rviz_bridge.py        # RViz 画面配信（任意機能・既定 off）
│  └─ dummy_perception.py   # 検出のダミー ★差し替え対象
├─ web/                     # ブラウザ側
│  ├─ index.html / css/style.css
│  └─ js/app.js             # 画面の制御全般 ★中核
│     robotview.js          # URDF の 3D 描画と順運動学
│     armview.js            # 簡易ビューと俯瞰マップ
│     rosbridge.js          # rosbridge クライアント（自作・依存なし）
│     webrtc.js             # WebRTC 受信
├─ launch/arm_ui.launch.py  # 起動一式
├─ config/arm_ui.yaml       # 全ノードのパラメータ
└─ tests/                   # ROS 無しで走る（下記）
   ├─ mini_ros.py           # rclpy を差し替えるスタブ
   ├─ test_arm.urdf         # piper_description が無い環境でも運動学を試す用
   └─ check_real_urdf.py    # 実機URDFでの確認
```

まず読む 3 つ: `common.py` → `harvest_sequencer.py` → `web/js/app.js`

---

## 4. テスト — 変更したら必ず走らせる

```bash
python3 tests/test_kinematics.py   # 順運動学・逆運動学の数値検証（25項目）
python3 tests/test_cartesian.py    # 座標指定コマンドをシーケンサに流す（41項目）
python3 tests/test_xyz_ui.py       # 座標指定トグルを Chromium で押す（22項目）
python3 tests/test_command_topic_guard.py  # /joint_states の安全弁（22項目）
python3 tests/test_team_status.py  # チームの収穫システムの状態表示（46項目）
python3 tests/test_team_ui.py      # 状態表示・検出枠の印・lemon のボタン・長押し・右下の図・認識姿勢を Chromium で（107項目）
python3 tests/test_video_relay.py  # 映像を溜めない・同じ絵を送らない・送る映像の選び方（17項目）
python3 tests/test_video_ui.py     # ブラウザの映像要求・再接続・関節角の間引き（15項目）
python3 tests/test_arm_status.py   # /arm_status の真偽値の読み方（6項目）
python3 tests/test_lemon_detections.py  # 検出アダプタ: 楕円→枠、正規化、対象の印、TF は固定中だけ（29項目）
python3 tests/test_lemon_backend.py     # backend:=lemon: サービス呼び出し・固まらない・停止・把持の決まり・ジョグ・認識姿勢と収穫可否・指令と実機のずれの警告（117項目）
python3 tests/test_baskets.py      # 収穫位置①②③: 記録のファイル・記録した位置へだけ動く（27項目）
python3 tests/check_real_urdf.py   # 実機URDFでの運動学の成立確認
```

`tests/mini_ros.py` が rclpy を差し替えるので **ROS 2 を起動せずに走る**。
必要なもの: `numpy` `scipy` `aiohttp` `playwright`。

```bash
pip3 install numpy scipy aiohttp playwright && playwright install chromium
```

### テストの考えかた

**実物を動かして確かめる方式にしてある。** モックで固めた単体テストではなく、
本物のノード・本物の rosbridge プロトコル・本物の Chromium を通す。
この形にしたのは、**実際にこれでしか見つからないバグを何件も見つけた**ため。

- **IK の良し悪しは画面を見ても分からない。** 必ず数値で確かめる。
  `test_kinematics.py` は scipy との回転規約の照合、ヤコビアン vs 数値微分、
  IK の往復（ランダム姿勢 → 手先位置 → IK → 同じ位置に戻るか）で押さえている。
- **UI は実際にブラウザで押す。** 宣言順の取り違え（`const` の TDZ）や
  セレクタの打ち間違いは、ページを開かないと気づけない。
- 新しい機能を足したら、**同じ粒度でテストも足す**。
  「通った」ではなく「落としてみて本当に落ちるか」まで確かめると良い。

---

## 5. 踏んだ落とし穴（繰り返さないこと）

| 症状 | 原因と対策 |
|---|---|
| **装飾の無い素の HTML が出る** | `--symlink-install` で `share/.../web/` の中身がシンボリックリンクになり、aiohttp の静的配信が既定で 404 にする。`follow_symlinks=True` で対策済み |
| **画面全体がチカチカする** | ノードの二重起動。UI が 2 つ分の状態を交互に受け取る。検出して画面に警告を出す。`pkill -f harvest_sequencer` |
| **3つの映像が全部同じ絵になる** | aiortc は複数トラックを 1 つの MediaStream にまとめる。`event.streams[0]` ではなく `new MediaStream([event.track])` |
| **グリッパのフィードバックが常に 0** | piper は `joint7` ではなく **`gripper`** という名前で出す |
| **把持力が弱い / 速度指定が効かない** | `velocity` の長さが**ちょうど 7** のときだけ速度指定が効く。`effort` 全ゼロは 0.5 に丸められる |
| **リンクの向きが静かにずれる** | URDF の rpy は**固定軸 X→Y→Z**。three.js では `'ZYX'` 指定が対応する。数値照合テストで守っている |
| **空欄の座標が 0 として送られる** | `Number('')` は `NaN` ではなく **`0`**。文字列として空かを先に見る |
| **IK が「到達可能なのに届かない」と言う** | 減衰（λ）を固定にすると局所解で止まり 2.5% 取りこぼす。1歩ごとに受け入れ判定して λ を調整する |
| **IK の時間制限が解を削る** | 打ち切りを 0.12 秒にすると到達可能な目標の 4% を落とす。**`time_budget` は速度調整ではなく最悪時の保険**。短くしない |
| **`/joint_states` の向きが構成で変わる** | 本家 piper_ros は「入力」、ros2_control 構成では「出力」。後者で publish すると MoveIt / Servo の現在姿勢が壊れる。v0.4.0 で安全弁を入れた（`_check_command_topic_owner`）。**外さないこと** |
| **長押しのジョグが止まらず、アームが動き続ける** | 押している間にボタンを disabled にすると、ブラウザはそのボタンに mouseup を届けず、繰り返しのタイマーが止まらない（lemon で実機が動き続けた）。長押しは `holdRepeat()` を使う（どこで離しても止まる・押せなくなったら止まる・lemon は 1 回押す = 1 ステップ）。サーバ側も 0.5 s 以内の続けざまのジョグを無視する（`JOG_MIN_INTERVAL`） |
| **ヘッダによる遠隔判定がすり抜ける** | Chromium は `extra_http_headers` を **WebSocket の upgrade に付けない**。`public_port` に届いたかを第一の判定にしている |

---

## 6. 差し替えポイント（優先順）

| 優先 | やること | どこ |
|---|---|---|
| 高 | **レモンの検出**。いまは 3 個を巡回させるだけのダミー。`/arm_ui/detections` に同じ形の JSON を流せば UI 側は無改造 | `dummy_perception.py` |
| 高 | **把持姿勢の計算**。画像座標と距離から関節角への線形写像というスタブ。**ここがデモの成否を分ける** | `harvest_sequencer._compute_grasp_pose()` |
| 中 | **軌道生成**。関節空間の線形補間（smoothstep）で障害物回避なし。枝を避けるなら MoveIt2 へ | `harvest_sequencer.Trajectory` |
| 低 | **走行モード**。`/dog_ui/mode` に文字列を publish するだけの骨組み | `web/js/app.js` |

### 検出結果の形（認識を作る人はここだけ見ればよい）

```json
{"targets": [
  {"id": 1, "u": 0.31, "v": 0.44, "w": 0.13, "h": 0.17,
   "distance": 0.38, "confidence": 0.92, "label": "lemon"}
]}
```

`u, v` は画像内の正規化座標でそのまま検出枠の描画位置になる。
`distance` は把持姿勢の計算に使う。

---

## 7. コマンドの足しかた

UI のボタン 1 つ = シーケンサの `_cmd_<名前>` メソッド 1 つ。
ディスパッチは `getattr(self, '_cmd_' + name)` なので**登録作業は要らない**。

```python
def _cmd_my_thing(self, cmd):
    """UI の「〇〇」ボタン."""
    ...
```

ブラウザ側は `sendCommand({cmd: 'my_thing', ...})`。

既存: `start_selection` / `select_target(id)` / `plan_grasp` / `grasp` /
`goto_basket(index 1–3)` / `release` / `home` / `abort` / `set_enable(value)` /
`jog(joint, delta)` / `gripper(close)` / `move_to_xyz(x,y,z)` / `jog_xyz(axis, delta)`

backend:=lemon では `_lemon_<名前>` が呼ばれ、受け付けるのは `LEMON_COMMANDS` にあるものだけ。
lemon だけのもの: `observe(side right|left)`（認識姿勢へ）/ `set_basket(index)` / `clear_exclusions`

---

## 8. 作業の進めかた（このプロジェクトでの希望）

- **変更したらテストを走らせてから「できた」と言う。** 特に運動学まわり。
- **実機が要る作業は、実機が無いことを理由に推測で埋めない。** そう言って止まる。
- **仮実装には理由と差し替え先をコメントに書く。**
- 既存の設計判断を変えるときは、**なぜ変えるかを先に述べる**（上の「前提」は
  どれも理由があって選んでいる）。
- 大きく作り直す前に、いまのテストが何を守っているかを読む。

---

## 9. もっと詳しく

| | |
|---|---|
| `README.md` | **いまの実機での使い方**（起動、1サイクルの操作、収穫位置の記録、停止、困ったとき） |
| `docs/REFERENCE.md` | 旧 README。起動オプションの全部、RViz、分散構成、トピック一覧（本家構成が前提の部分あり） |
| `docs/HANDOVER.md` | 設計の方針、データの流れ、3D の生成経路、運動学の中身 |
| `docs/SETUP_dev_pc.md` | 開発用PCへの導入 |
| `docs/SETUP_ubuntu2204_humble.md` | まっさらな Ubuntu からミニPCを作る |
| `docs/SETUP_remote_access.md` | 別ネットワークから使う（Tailscale / ngrok） |
