# piper_arm_ui — 詳しい資料（旧 README）

> **この文書は、以前の README をそのまま残したものです。**
> 本家 piper_ros の構成（UI が `/joint_states` に関節角を出してアームを動かす、
> `backend:=internal`）を前提に書かれている部分があります。
> **いまの実機（チームの収穫システム、`backend:=lemon`）での使い方は [README.md](../README.md) を見てください。**
>
> ここに残しているのは、起動オプションの全部、RViz の映像の調整、3D ビュー、
> 分散構成、別ネットワークからの利用、トピック一覧など、細かい情報です。
> 「3.5 分散構成」は本家の構成を前提にしているので、チームの構成で使うときは読み替えが要ります。

---


犬型ロボット搭載アーム（AgileX **PiPER**）でレモンを自動収穫するデモ用の **アーム操作UI**。
パワポ4枚目「アーム専用UI」のイメージ図をそのまま画面にしたもので、ブラウザから操作する。

- カメラは **RealSense**（カラー画像）、映像は **WebRTC**（低遅延・タブレット/スマホでも見られる）
- 右上は **3D ビュー**。piper_description の URDF とメッシュをブラウザへ渡し、
  関節角のトピックだけで動かす。60fps・画面いっぱいの解像度で、遅延はほぼ無い
  （RViz の画面を映像で送る従来方式にも切り替えられる）
- ROSのトピック読み書きは **rosbridge**（WebSocket）
- 移動用UI / アーム用UI は3枚目のメモどおり **ページ切替**

```
┌──────────────────────────────────────────────┐
│ レモン自動収穫デモ   [移動用UI][アーム用UI]  ●●●  [停止] │
├───────────────────────┬──────────────────────┤
│ カメラ（RealSense）   │ 3D ビュー（URDF）    │
│  └ 検出枠クリックで選択│  └ ドラッグで視点操作│
├───────────────────────┼──────────────────────┤
│ 収穫シーケンス        │ ロボットの状態       │
│  把持対象の選択       │  （犬型＋収穫位置①②③）│
│  把持（把持姿勢の計算）│                      │
│  把持                 │                      │
│  収穫位置 ① ② ③      │                      │
│  開放（収穫完了）     │                      │
└───────────────────────┴──────────────────────┘
```

---

## 1. 構成

| ノード | 役割 |
|---|---|
| `ui_server` | Web UI の配信 + WebRTC 映像配信（RealSense と RViz 画面を購読） |
| `rviz_bridge` | （任意）Xvfb 上で rviz2 を起動し画面を配信。3D ビューが既定になったので通常は不要 |
| `harvest_sequencer` | 収穫シーケンスの状態機械。UIからのコマンド処理と状態配信 |
| `dummy_perception` | レモン検出結果のダミー配信（実装が決まるまでの仮） |
| `rosbridge_websocket` | ブラウザ ↔ ROS のブリッジ（`rosbridge_server` パッケージ） |

```
 ブラウザ ──WS:8080/rosbridge──> ui_server ──> 127.0.0.1:9090 rosbridge
    │                              │                    │
    │                              │                    ├─> /arm_ui/command ──> harvest_sequencer
    │                              │                    │  <── /arm_ui/state  <──┘        │
    │                              │                    └─> /rviz_ui/input ──> rviz_bridge ↓
    │                              │                                             │ /joint_states → piper_ros
    └──WebRTC:8080───────────────> ui_server <── /rviz_ui/image ──────────────────┘
                                       ↑
                          RealSense (/camera/camera/color/image_raw)
```

ブラウザが繋ぐ先は **8080 の 1 つだけ**。rosbridge へも `ui_server` の
`/rosbridge` を経由する（rosbridge 自体は `127.0.0.1` に閉じている）。
別ネットワークからの利用と権限制御のためで、詳しくは §3.6。

UI ↔ ノード間は **JSON を `std_msgs/String` で運ぶ**方式にしてある。
カスタムメッセージのビルドなしに項目を増やせるので、仕様が固まるまでの試作に向く。
固まったら `piper_msgs` 相当の独自 msg/srv に置き換えればよい。

---

## 2. インストール

> **開発を引き継ぐ人へ**: 設計の考え方・差し替えポイント・踏んだ落とし穴は
> **[docs/HANDOVER.md](HANDOVER.md)** にまとめてある。まずそこから読むと早い。

> - まっさらな Ubuntu 22.04 から**ミニPC**を構築する場合:
>   **[docs/SETUP_ubuntu2204_humble.md](SETUP_ubuntu2204_humble.md)**
>   （ROS 2 の導入から CAN 設定・RealSense・動作確認まで通し）
> - ROS 2 が入っている**開発用PC**に UI 側だけ入れる場合:
>   **[docs/SETUP_dev_pc.md](SETUP_dev_pc.md)**
>
> ここでは既に ROS 2 Humble と piper_ros が動いている前提で、追加分だけを書く。

### 依存パッケージ

```bash
# ROS 側
sudo apt install ros-humble-rosbridge-server ros-humble-cv-bridge \
                 ros-humble-realsense2-camera ros-humble-rviz2

# WebRTC 側（ui_server が使う）
pip3 install aiohttp aiortc av

# RViz 映像を使う場合だけ（rviz_bridge。3D ビューだけなら不要）
sudo apt install xvfb
pip3 install mss python-xlib
```

> 3D ビューに追加の依存は無い。three.js は `web/js/vendor/` に同梱してあり、
> メッシュの変換は numpy だけで行う（外部のメッシュ処理ライブラリは不要）。

> - `aiortc` が入らない環境では映像だけ出ないが、UI と操作系は動く。
> - `mss` が無い場合は `ffmpeg` があればそちらで画面キャプチャする（`apt install ffmpeg`）。
> - `python-xlib` が無い場合は `xdotool` があればそちらで操作を注入する。
>   どちらも無いと RViz は「表示のみ」になる。

### ビルド

```bash
cd ~/piper_ros            # 既存のワークスペース
cp -r /path/to/piper_arm_ui src/
colcon build --packages-select piper_arm_ui --symlink-install
source install/setup.bash
```

---

## 3. 起動

```bash
# 1) PiPER 本体（既存）
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=true gripper_exist:=true

# 2) RealSense（別で起動済みならこの手順は不要）
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true

# 3) UI 一式（RViz も一緒に立ち上がる）
ros2 launch piper_arm_ui arm_ui.launch.py
```

RealSense も一緒に起動したい場合は `start_realsense:=true` を付ける。

ブラウザで **`http://<ロボットのIP>:8080/`** を開く。

### よく使う起動オプション

```bash
# RealSense も一緒に起動する
ros2 launch piper_arm_ui arm_ui.launch.py start_realsense:=true

# RealSense のトピック名が違うとき（ros2 topic list で確認）
ros2 launch piper_arm_ui arm_ui.launch.py arm_camera_topic:=/camera/color/image_raw

# カメラ無しでレイアウトだけ確認する
ros2 launch piper_arm_ui arm_ui.launch.py use_dummy_video:=true

# モニタ接続済みの実機で、すでに出している :0 の rviz2 を映す
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_xvfb:=false rviz_display:=:0 launch_rviz:=false

# RViz 映像も使いたい（TF や点群を見たいとき。既定は 3D ビューのみ）
ros2 launch piper_arm_ui arm_ui.launch.py start_rviz_bridge:=true

# 3D ビューのメッシュを粗くして軽くする（既定 0.001）
ros2 launch piper_arm_ui arm_ui.launch.py mesh_simplify_cell:=0.003

# RViz の初期視点を変える（既定は「アームの後ろから」）
ros2 launch piper_arm_ui arm_ui.launch.py view_yaw:=0.0        # 正面から
ros2 launch piper_arm_ui arm_ui.launch.py view_yaw:=1.5708     # 左から

# RViz のメニューバー等も映したい（既定は消してモデルだけ）
ros2 launch piper_arm_ui arm_ui.launch.py hide_chrome:=false

# 最初からUIでアームを動かす（既存の joy ノードと同時に使わないこと）
ros2 launch piper_arm_ui arm_ui.launch.py enable_arm_command:=true

# rosbridge を別で起動している場合
ros2 launch piper_arm_ui arm_ui.launch.py start_rosbridge:=false

# 8080 / 9090 が別のプログラムに使われているとき（下の「ポートが埋まっているとき」参照）
ros2 launch piper_arm_ui arm_ui.launch.py \
    port:=8081 rosbridge_port:=9091

# 別ネットワークから外部の人に見せる
# 公開用ポート 8081 を開け、トンネルはそちらへ向ける（8080 は手元用のまま）
# 詳しくは docs/SETUP_remote_access.md
ros2 launch piper_arm_ui arm_ui.launch.py \
    public_port:=8081 allow_remote_control:=false \
    ice_servers:=stun:stun.l.google.com:19302

# 実機なし（開発用PC）で UI を一通り動かす
# カメラはテストパターン、UI の操作が RViz のモデルに反映される
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_dummy_video:=true enable_arm_command:=true
```

### 収穫かごの位置

かごは**斜め後ろ 2 箇所と斜め前 1 箇所**に置く前提。
`joint1`（旋回）がそのまま置き場の方位になるので、動かすときはそこだけ変える。

| | 方位 | 位置（base_link） | 距離 |
|---|---|---|---|
| ① | `+2.094` rad = **+120°** 左後方 | x −0.132 / y +0.229 | 0.265 m |
| ② | `-2.094` rad = **−120°** 右後方 | x −0.132 / y −0.229 | 0.265 m |
| ③ | `+1.047` rad = **+60°** 左斜め前 | x +0.095 / y +0.164 | 0.190 m |

```yaml
# config/arm_ui.yaml
basket_pose_1: [2.094, 1.20, -1.00, 0.0, 0.40, 0.0]     # 左後方
basket_pose_2: [-2.094, 1.20, -1.00, 0.0, 0.40, 0.0]    # 右後方
basket_pose_3: [1.047, 0.90, -0.70, 0.0, 0.30, 0.0]     # 左斜め前
```

右下の俯瞰図の ①②③ は、この関節角を順運動学にかけて描いている。
値を変えれば図もそのまま追従するので、調整しても絵とずれない。

> **後方へ振るときの注意。** ±120° は joint1 の可動域（±150°）に収まっているが、
> アームは犬型本体の脇を通って後ろへ回る。ホーム（0°）から ① へ動かすと
> joint1 が 0°→120° と連続で振れるので、**本体・配線・人に当たらないか**
> 最初は `motion_speed` を 20〜40 に落として確かめること。
> ① から ② への移動は前方（0°）を通る経路になる。

### 座標指定で動かす

左下パネルの **「手動ジョグ（関節角度指定）」の下**に
**「座標指定（手先位置）」**のトグルがある。関節角ではなく
**手先をどこに持っていきたいか**で動かす。

- **X / Y / Z** は `base_link` 基準の座標 [m]。X が前後、Y が左右、Z が上下。
- **±ボタン**は「いまの手先位置から相対で」動かす。5 / 10 / 50 mm を選べる。
  現場での寄せはほぼこれで足りる。
- **「取込」**で現在の手先位置を入力欄に入れる。そこから数字をいじって
  **「この位置へ移動」**、という使いかたが一番やりやすい。
- 届かない座標、可動範囲の外、入れ忘れは**送る前に止めて理由を出す**。
  単位を mm で入れてしまったときは「単位は m です」と名指しで言う。

内部では URDF から組んだチェーンで**逆運動学を解いている**
（`piper_arm_ui/kinematics.py`）。解くのは**位置 3 自由度だけ**で、
**手先の向きは解に任せている**。掴む向きまで指定したいときは、
ヤコビアンに姿勢の行を足して 6 次元にする必要がある。

動きは既存の関節補間（`Trajectory`）に乗せているので、
**途中の手先の軌跡は直線にならない**し、枝の回避もしない。
デモの位置決め用と考え、経路が要るなら MoveIt2 に置き換える。

> `piper_description` の URDF が見つからない環境では、**座標指定だけが
> 自動的に無効**になる（画面に理由が出る）。関節角度指定と収穫シーケンスは
> そのまま使える。

#### 座標指定を実機に合わせる

UI が出す **「手先位置(指令)」** は、指令している関節角をこちらの
順運動学にかけた値。piper が出す **`/end_pose`（手先位置(実機)）** とは
**基準点が違うことがある**（URDF には「グリッパの指の間」を表すリンクが
無いため、既定では `joint6` の子リンクの原点が基準）。

2 つを並べて表示しているので、**ずれていたら `tcp_offset` で合わせる**。

```yaml
# config/arm_ui.yaml  （/**/harvest_sequencer）
tcp_link: ""                      # 空なら joint6 の子リンク
tcp_offset: [0.0, 0.0, 0.15]      # 手先リンクの原点から指の間までの実測値 [m]
```

手順:

1. UI を開き、「ロボットの状態」パネルの **手先位置(指令)** と
   **手先位置(実機)** を見比べる
2. 差が一定なら、その差をそのまま `tcp_offset` に入れる
3. 入れ直して起動し、2 つの値が揃うことを確かめる

合わせておかないと「実機の表示を見て座標を打ち込んだのに別の所へ行く」
ことになる。**座標指定を実機で使う前に必ず 1 回やる。**

### 3D ビュー（既定）

右上のパネルは **URDF をブラウザで直接描画**している。RViz の画面を動画で
送っていた頃と比べて、遅延・解像度・なめらかさのすべてが改善する。

```
起動時に一度だけ          : /api/robot（URDF の構造 3.8KB）
                            /api/mesh（形状 計 1.9MB / gzip 0.8MB）
そのあと流れ続けるもの     : /joint_states の関節角だけ（毎秒 数百バイト）
```

| | RViz 映像（旧） | 3D ビュー（新） |
|---|---|---|
| 通信量 | 常時 約 2 Mbps | 初回 0.8MB のみ + 関節角 |
| 遅延 | キャプチャ+エンコード+デコード | 関節角の到達時間だけ |
| 解像度 | 960x540 固定 | 画面の実解像度（端末の DPI に追従）|
| なめらかさ | 20fps | 60fps（受信 10〜50Hz を補間）|
| CPU（ミニPC）| Xvfb + rviz2 + VP8 エンコード | ほぼゼロ |

- 形状は `piper_description` の STL を頂点クラスタリングで軽くしている
  （9.2MB / 18.4万三角形 → 1.9MB / 10.7万三角形、形状誤差 0.06mm）。
  細かさは `mesh_simplify_cell` で変えられる。
- ブラウザ内の順運動学が URDF と一致することを検証済み（誤差 0.000000 mm）。
- 動きが無いフレームは描画しないので、止まっている間の消費はほぼゼロ。
  タブレットの電池にも効く。
- three.js は現場のオフライン運用に備えて `web/js/vendor/` に同梱している（CDN 不要）。

視点操作は RViz と同じ。左ドラッグで回転、右ドラッグ（または Shift+ドラッグ）で
平行移動、ホイールでズーム。右下のボタンで「後・前・左・右」と視点リセット。

関節角の取得元はパネル右上に出る。実機のフィードバック
（`/joint_states_single`）が来ていれば**実機**、無ければ指令
（`/joint_states`）を表示する。

### RViz 映像に切り替えたいとき

TF や点群など RViz でしか出せないものを見たいときは、パネル右上の
**RViz** ボタンで従来の映像に切り替わる。ただし `rviz_bridge` が
動いている必要がある。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py start_rviz_bridge:=true
```

3D ビューだけで足りるなら `start_rviz_bridge:=false`（既定）のままでよい。
Xvfb も rviz2 も動かないぶん、ミニPC の負荷が大きく下がる。

### RViz 映像の見えかたを変える（`start_rviz_bridge:=true` のとき）

RViz 映像に切り替えたときの画面は、既定で**アームの後ろから見た視点**で、
**モデルだけ**が映る（メニューバー・ツールバー・ステータスバーは消してある）。

| 変えたいこと | 引数 |
|---|---|
| 正面から見る | `view_yaw:=0.0` |
| 後ろから見る（既定）| `view_yaw:=3.14159` |
| 左から / 右から | `view_yaw:=1.5708` / `view_yaw:=-1.5708` |
| もっと見下ろす | `view_pitch:=0.6`（0 が真横、1.5708 が真上）|
| 引き／寄り | `view_distance:=1.8` / `0.8` |
| RViz のメニュー類も映す | `hide_chrome:=false` |
| 設定ファイルの視点をそのまま使う | `apply_view:=false`（パラメータで指定）|

視点の指定は Orbit カメラの式そのままで、注視点から見たカメラ位置が

```
x = distance * cos(yaw) * cos(pitch)
y = distance * sin(yaw) * cos(pitch)
z = distance * sin(pitch)
```

になる。PiPER は `base_link` の **+X 方向へ手を伸ばす**ので、
カメラを −X 側に置く `yaw = π` が「後ろから」になる。

「モデルだけ」は rviz2 を**全画面（F11）**にして実現している。
`VisualizationFrame::setFullScreen()` がメニューバー・ツールバー・
ステータスバーとドックの開閉ボタンをまとめて隠してくれる。
F11 は起動後に `rviz_bridge` が 1 回だけ送る（`chrome_delay` 秒待ってから）。

> 3D ビューの中に文字が出る場合は TF の **Show Names** が ON になっている。
> 同梱の `harvest.rviz` では OFF にしてある。

### RViz 映像がカクつくとき（`start_rviz_bridge:=true` のとき）

> 3D ビュー（既定）ならこの問題は起きない。以下は RViz 映像を使う場合の話。


なめらかさは**解像度より fps** が効く。既定は **960x540 @20fps**。
実測（この構成での 1 枚あたりの処理時間）:

| 解像度 | 画面キャプチャ | VP8 エンコード | 合計 | 出せる上限 |
|---|---|---|---|---|
| 1280x720 | 5.0 ms | 29.7 ms | 34.8 ms | 約 29 fps |
| 960x540（既定）| 2.8 ms | 14.8 ms | 17.7 ms | 約 57 fps |
| 848x480 | 2.2 ms | 13.1 ms | 15.4 ms | 約 65 fps |
| 640x360 | 1.4 ms | 8.3 ms | 9.7 ms | 約 103 fps |

エンコードが支配的なので、**重い機体では解像度を下げて fps を保つ**のが正解。
UI の RViz パネルは実表示 600〜700px なので 960x540 でも見た目は変わらない。

```bash
# もっとなめらかにしたい（軽い機体）
ros2 launch piper_arm_ui arm_ui.launch.py rviz_fps:=30

# CPU が苦しいとき（解像度を落として fps を保つ）
ros2 launch piper_arm_ui arm_ui.launch.py rviz_width:=848 rviz_height:=480

# さらに苦しいとき
ros2 launch piper_arm_ui arm_ui.launch.py rviz_width:=640 rviz_height:=360 rviz_fps:=25
```

`rviz_bridge` は実際に出せている fps を 10 秒ごとにログへ出し、
設定の 6 割を割ると警告する。まずこれを見て、設定どおり出ていないなら
解像度を下げる。

そのほか効くところ:

- **モニタがある機体なら GPU を使う。** `use_xvfb:=false rviz_display:=:0 launch_rviz:=false`
  にすると、Xvfb のソフトウェア描画（llvmpipe）を避けられて大きく速くなる。
  `software_rendering` は既定 `auto` で、Xvfb のときだけ llvmpipe を強制する。
- **RViz の表示物を減らす。** 点群（RealSense PointCloud）を出していると重い。
  同梱の `harvest.rviz` では既定 OFF。
- キャプチャは専用スレッドで回している。遅れたぶんを取り戻そうとはせず次の周期へ送るので、
  重くなってもフレームが束で届いて余計にガクつくことはない。

### 画面がチカチカする / 表示が高速に切り替わる

「計画あり／なし」が高速に入れ替わる、収穫位置のボタンが点滅する、
ボタンの有効・無効がばたつく — これはたいてい**ノードの二重起動**。
前回の launch のノードが残っていると、UI は 2 つ分の状態を交互に受け取る。

現在の版は自分で気づいて、画面上部に赤い警告バーと、どのノードが
重複しているかを出す。端末側にも error ログが出る。

```bash
ros2 node list | grep harvest_sequencer   # 2つ出ていないか
ros2 topic info /arm_ui/state             # Publisher count が 2 以上か
```

止めてから起動し直す。

```bash
pkill -f harvest_sequencer
pkill -f dummy_perception
pkill -f rviz_bridge
pkill -f ui_server
pkill -f rosbridge_websocket
pkill -f 'Xvfb :99' ; rm -f /tmp/.X99-lock
ros2 launch piper_arm_ui arm_ui.launch.py
```

同じ検出を `rviz_bridge`（`/rviz_ui/image`）と `dummy_perception`
（`/arm_ui/detections`）にも入れてある。RViz の映像がチカチカする、
検出枠が飛び回る、といった症状も同じ原因のことが多い。

### 装飾のない素の HTML が出るとき

見出しと箇条書きだけの白い画面になったら、CSS と JS が 404 になっている。

`colcon build --symlink-install` で入れると `share/piper_arm_ui/web/` の中身は
ソースツリーへのシンボリックリンクになる。aiohttp の静的配信は既定で
「配信ルートの外を指すシンボリックリンク」を 404 にするため、
`index.html` だけが表示されて CSS・JS が全部落ちる、という壊れ方をする
（`ui_server` は `follow_symlinks=True` で配信するようにして修正済み）。
古い版を入れている場合は更新して入れ直す。

```bash
cd ~/piper_ros
colcon build --packages-select piper_arm_ui --symlink-install
source install/setup.bash
```

### ポートが埋まっているとき

UI は 3 つの資源を掴む。前回の起動が残っていたり、別のプロジェクト
（例: 自作の `webrtc_arm_controller`）が同じ番号を使っていると起動に失敗する。

| 資源 | 既定 | 変える引数 |
|---|---|---|
| Web UI（HTTP + WebRTC シグナリング） | 8080 | `port:=8081` |
| rosbridge（ブラウザ ↔ ROS） | 9090 | `rosbridge_port:=9091` |
| RViz 用の仮想ディスプレイ | `:99` | `rviz_display:=:98` |

まず誰が掴んでいるかを見る。

```bash
ss -ltnp | grep -E ':8080|:9090'
ls /tmp/.X*-lock
```

前回の起動が残っているだけなら片付ける。

```bash
pkill -f ui_server
pkill -f rosbridge_websocket
pkill -f 'Xvfb :99' && rm -f /tmp/.X99-lock
```

他のプログラムが正当に使っているなら番号をずらす。
`rosbridge_port` を変えてもブラウザ側の設定は要らない。UI は起動時に
`/api/info` を読んで、そこに書かれたポートへ繋ぎにいく。

ディスプレイ番号については、指定した番号が埋まっていれば
`rviz_bridge` が空き番号へ自動的にずらし、その旨を warn ログに出す
（前回の Xvfb が残ったまま古い画面を映し続ける事故を防ぐため）。

---

## 3.5 分散構成（ミニPCは本家のまま、UI は別PC）

ミニPC には **piper_ros を本家のまま** 置いて、UI 側（WebRTC・rosbridge・RViz）を
開発用PCで動かす構成にできる。ノード間はすべて ROS のトピックなので、
同じ LAN・同じ `ROS_DOMAIN_ID` にすれば DDS がそのまま繋いでくれる。

```
ミニPC（本家 piper_ros のみ）                 開発用PC（piper_arm_ui）
├ piper_single_ctrl                         ├ ui_server（WebRTC）
│   /joint_states_single ─┐                 ├ rviz_bridge（RViz + Xvfb）
│   /arm_status           │                 ├ harvest_sequencer
│   ← /joint_states     ──┼── DDS(LAN) ──►  ├ robot_state_publisher
├ realsense2_camera       │                 └ rosbridge_server
    .../image_raw/compressed ┘                    ▲
                                              ブラウザ（8080）
```

ミニPC側（本家のまま。`piper_arm_ui` は入れなくてよい）:

```bash
export ROS_DOMAIN_ID=42
ros2 launch piper start_single_piper.launch.py \
    can_port:=can0 auto_enable:=true gripper_exist:=true
ros2 launch realsense2_camera rs_launch.py enable_color:=true
```

開発用PC側:

```bash
export ROS_DOMAIN_ID=42
ros2 launch piper_arm_ui arm_ui.launch.py camera_transport:=compressed
```

### 帯域に注意（ここが肝）

`sensor_msgs/Image` を生のまま LAN に流すと、640x480 の rgb8 で 30fps なら
**約 221 Mbps** になる。有線ギガビットなら通るが、Wi-Fi では現実的でない。

`camera_transport:=compressed` にすると `<トピック名>/compressed`
（`sensor_msgs/CompressedImage`）を購読するようになり、実際の果樹園の絵で
おおむね 10〜30 Mbps 程度まで落ちる。ミニPC側に配信プラグインを入れておく。

```bash
# ミニPC側
sudo apt install ros-humble-compressed-image-transport
```

realsense2_camera は image_transport 経由で publish するので、
このパッケージを入れるだけで `/compressed` が自動的に生える。
確認は次の通り。

```bash
ros2 topic list | grep compressed
ros2 topic bw /camera/camera/color/image_raw/compressed
```

### ミニPC側でやること／やらなくてよいこと

**piper_ros のソースには一切手を加えない。** ただし「何もしなくていい」わけではなく、
本家 README どおりのセットアップは必要。

やること:

| | 内容 |
|---|---|
| 依存 | `pip3 install python-can scipy piper_sdk` と ros2-control 系の apt |
| ビルド | `colcon build`（本家をそのまま） |
| CAN | `bash can_activate.sh can0 1000000`（再起動のたび。systemd 化推奨）|
| 追加で1つ | `sudo apt install ros-humble-compressed-image-transport`（圧縮配信用）|
| 環境変数 | `export ROS_DOMAIN_ID=42`（開発用PCと揃える）|

**やらなくてよいこと**: `piper_arm_ui` の配置、`xvfb` / `aiortc` などの UI 系依存、
launch ファイルやソースの編集。UI 側が使うトピックは本家のノードが最初から出している。

| UI が使うトピック | 出どころ | 本家のまま出ているか |
|---|---|---|
| `/joint_states_single` | `piper_ctrl_single_node` | 出ている |
| `/arm_status` | 同上 | 出ている |
| `/end_pose` | 同上 | 出ている |
| `/joint_states`（指令） | UI → piper が購読 | `start_single_piper.launch.py` が `joint_ctrl_single` からリマップ済み |
| カメラ画像 | `realsense2_camera` | 出ている（`/compressed` は上の apt で生える）|

### 疎通の確認

開発用PC側で実行して、ミニPCのトピックが見えていれば繋がっている。

```bash
ros2 node list                        # /piper_ctrl_single_node が見えるか
ros2 topic hz /joint_states_single    # 関節角が流れているか
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /arm_status --once
```

見えない場合は `ROS_DOMAIN_ID`、マルチキャスト、`RMW_IMPLEMENTATION`
（両側とも既定の `rmw_fastrtps_cpp` のままにする）を疑う。

### そのほかの注意

- **DDS の疎通**: 既定の DDS はマルチキャストで相手を見つける。
  マルチキャストを通さない Wi-Fi AP だと繋がらないので、
  その場合は AP の設定を見直すか、有線にする。
- **`ROS_DOMAIN_ID` を両方で揃える**こと。揃っていないと何も見えない。
- **RViz は開発用PC側で描画される**。ミニPCの GPU は使わないので、
  開発用PCにモニタがあるなら `use_xvfb:=false rviz_display:=:0` のほうが速い。
- **デモ本番はミニPCに全部載せるのが安全**。この分散構成は開発中の切り分け用と
  考えたほうがよい。Wi-Fi が切れると UI ごと落ちる。

---

## 3.6 別ネットワークから使う

同一 LAN の外から UI を開くための構成。手順は
**[docs/SETUP_remote_access.md](SETUP_remote_access.md)** にまとめてある。ここは要点だけ。

**この UI には認証がない。URL を知っている人 = 実機を動かせる人**だった。
外に出す以上それでは危ないので、**手元と遠隔で権限を分けている**。

| | 見る | 動かす | 非常停止 |
|---|---|---|---|
| 手元（LAN / VPN） | ○ | ○ | ○ |
| 遠隔（トンネル経由） | ○ | ×（手元が許可した間だけ ○） | ○ |

止めているのは**サーバ側**であって画面のボタンではない。
ボタンを隠すだけでは devtools から WebSocket に直接 publish できてしまう。
遠隔からの `publish` は `ui_server` の中継で捨てられるので、何を送っても ROS に届かない。
**非常停止だけは遠隔でも常に通す**（止める操作はアームをより安全な側にしか動かさない）。

そのために通信路を 1 本にまとめた。ブラウザは rosbridge へも
`ui_server` 経由（`/rosbridge`）で繋ぐ。

```
    ブラウザ ──┬── 8080 /              UI（HTML/CSS/JS）
               ├── 8080 /rosbridge  ──→ 127.0.0.1:9090 rosbridge
               └── 8080 /offer         WebRTC のシグナリング
```

- 外に開けるポートは **1 つだけ**（トンネル 1 本で済む）
- **https で開いても動く**（`wss://` になるので混在コンテンツで弾かれない）
- rosbridge は既定で `127.0.0.1` に閉じる（`rosbridge_address`）。
  ここを開けると中継を素通りされるので、開けないこと

### 手元と遠隔をどう見分けるか

`public_port:=8081` を指定すると、UI が **8080（手元用）と 8081（公開用）の
2 口**で待ち受ける。中身は同じだが、**8081 に届いた接続は送信元に関わらず
必ず遠隔扱い**になる。トンネルはこちらに向ける。

`X-Forwarded-For` などのヘッダを見る判定も入れてあるが、
**それだけに頼ってはいけない**。ヘッダを付けない中継もあるし、
WebSocket の upgrade だけヘッダが落ちる実装も実在する
（開発中に Chromium で確認した）。そこがちょうど権限判定の入口なので、
落ちると静かに穴が開く。ポートで分ける方式にはその弱点がない。

### どちらを使うか

- **チーム内 → Tailscale**。ロボットと端末を同じ仮想 LAN に入れるだけで、
  UI 側は**設定変更なし**にフル操作できる（`100.64.0.0/10` を手元に含めてある）。
  無料枠は 6 ユーザまで。見る人全員にアプリが要るので外部向けではない。
- **外部の人 → 公開トンネル**（cloudflared / ngrok）。8080 を 1 本通すだけ。
  `allow_remote_control:=false` で閲覧専用に固定して渡す。
  URL 自体に鍵をかけたいなら Cloudflare Access などトンネル側で。

映像（WebRTC）だけはトンネルを通らず直接繋ごうとするので、
別ネットワークからは `ice_servers:=stun:stun.l.google.com:19302` が要ることが多い。
**映像が不要なら 3D ビューだけのほうが圧倒的に軽い**（数 kbps）。

---

## 4. 既存のゲームパッド操作との関係

`joy_to_piper_jointstates` はそのまま併用する前提。
ただし **両方が `/joint_states` に publish すると指令が競合する**ため、
`harvest_sequencer` は既定で publish しない（`enable_arm_command: false` = 監視のみ）。

| 操作したいもの | 手順 |
|---|---|
| ゲームパッドで動かす | UI の「UI操作を有効化」を **OFF** のまま |
| UI で動かす | ゲームパッドから手を離し、「UI操作を有効化」を **ON** |

UI 右上の赤い「停止」ボタンで軌道生成を止め、その場の角度で保持する。

### `/joint_states` について

piper のノードが実際に購読しているのは `joint_ctrl_single` で、
`start_single_piper.launch.py` が `('joint_ctrl_single', '/joint_states')` と
リマップしているため `/joint_states` で動く。リマップなしで起動する場合は
`joint_command_topic:=/joint_ctrl_single` を指定すること。

指令メッセージは piper 側の `joint_callback` の仕様に合わせてある。

| 中身 | piper 側での扱い | このパッケージの既定 |
|---|---|---|
| `name` / `position`（8個）| joint1〜6 は名前で参照、`position[6]` がグリッパ開度 | joint1〜joint8。joint8 は RViz でグリッパ両側を描くために必要 |
| `velocity`（長さ 7 のときのみ）| `velocity[6]` が全体動作速度 1〜100% | `motion_speed: 0` = 指定せず piper 既定の 100%。穏やかに動かすなら 20〜40 |
| `effort` | `effort[6]` が把持力（0.5〜3 にクリップ ×1000）| `gripper_effort: 1.0`（全ゼロだと 0.5 に丸められて弱いため）|

フィードバックの `/joint_states_single` は
`['joint1'..'joint6', 'gripper']` という名前で来る（`joint7` ではない）。

---

## 5. UIの操作

| 操作 | 内容 | キーボード |
|---|---|---|
| 把持対象の選択 | 選択モードに入り、カメラ画像の検出枠をクリックして対象を決める | `1` |
| 把持（把持姿勢の計算） | 選択した対象に対する把持姿勢を計算し、その姿勢へ接近 | `2` |
| 把持 | グリッパを閉じる | `3` |
| 収穫位置 ①②③ | かごの位置へ移動 | `4`（①） |
| 開放（収穫完了） | グリッパを開き、収穫数をカウント | `5` |
| ホームへ戻す | ホーム姿勢へ | `h` |
| 停止 | 非常停止 | `Space` |

- 「手動ジョグ」を開くと各関節を ±で微調整できる（長押しで連続）。
- 右上のランプは左から **rosbridge / 映像 / アーム** の接続状態。

### RViz パネル

- **ドラッグで視点回転、ホイールでズーム**。RViz2 の Orbit カメラをそのまま操作している。
- 操作を止めたいときはパネル右上の「視点操作」を OFF にする
  （タブレットで誤って視点を動かしたくないとき用）。
- 「RViz / 簡易」で切り替えられる。**簡易**は自前 Canvas の側面図＋上面図で、
  rviz_bridge が動いていないときや、負荷を下げたいときのフォールバック。
  橙色のゴーストが計算された把持姿勢、水色が現在の指令姿勢。
### どの RViz 設定が使われるか

既定は **piper_ros に元から入っている `piper_description/rviz/piper_ctrl.rviz`**。
普段 `start_single_piper_rviz.launch.py` で見ているものと同じ画面になる。
`piper_description` が見つからない環境のときだけ、同梱の `config/harvest.rviz` に落ちる。

別の設定に切り替えたいとき:

```bash
# 同梱の収穫デモ向け設定（本家 piper_ctrl.rviz + TF / 点群 / マーカー）
ros2 launch piper_arm_ui arm_ui.launch.py \
    rviz_config:=$(ros2 pkg prefix piper_arm_ui)/share/piper_arm_ui/config/harvest.rviz

# MoveIt の設定（本家 piper_moveit のもの）
ros2 launch piper_arm_ui arm_ui.launch.py \
    rviz_config:=$(ros2 pkg prefix piper_with_gripper_moveit)/share/piper_with_gripper_moveit/config/moveit.rviz
```

### RViz にアームのモデルを出すには

`robot_state_publisher` が `/robot_description` と TF を出している必要がある。
**piper の `start_single_piper.launch.py` はこれを起動しない**（piper のノードだけを
起動する）ので、この launch が `start_robot_state_publisher:=true`（既定）で
代わりに起動する。すでに `display_xacro.launch.py` などを動かしている場合だけ
`start_robot_state_publisher:=false` にする。

RViz に渡す URDF は **素の `piper_description.urdf`**。`piper_description.xacro` は
gazebo 用の記述の中で `$(find piper_gazebo)` を参照しており、展開に `piper_gazebo` の
ビルドが必要になるため使っていない。古いファーム用に切り替えるには
`robot_description_file:=<...>/piper_description_old.urdf` を指定する。

RViz が表示するのは `/joint_states` の内容、つまり **指令している姿勢**。
実機のフィードバック（`/joint_states_single`）は UI 下部の関節テーブルに数値で出る。
指令と実機がずれると、テーブルの数値が橙色になる。

### 左右のドックについて

本家 `piper_ctrl.rviz` はドック（左の Displays、右の Views）が表示される設定なので、
そのまま埋め込むと横 600px 程度のパネルではドックが幅を食って 3D ビューが狭くなり、
文字も潰れて読めない。そのため `hide_docks: true`（既定）にしてあり、
**元の設定ファイルには手を触れずに一時コピーを作ってドックだけを畳んで** rviz2 に渡す。
表示内容（Displays の中身）や Fixed Frame は本家のまま。

ドックが必要になったら、埋め込んだ RViz のメニューバー **Panels** から出せる
（マウス操作はブラウザから注入されるので埋め込んだままで操作できる）。
最初からドックを出しておきたいなら `hide_docks:=false`。

同梱の `config/harvest.rviz` は本家 `piper_ctrl.rviz` をベースに、
TF / RealSense の点群（既定 OFF）/ `/arm_ui/markers` の MarkerArray を足し、
UI パネルに収まるよう 1280x720・左右ドック畳みにしただけのもの。
差分はファイル先頭のコメントに書いてある。

> **注意**: `start_single_piper_rviz.launch.py` は自前で rviz2 を起動する。
> `rviz_bridge` と併用すると rviz2 が2つ立ち上がるので、
> `start_single_piper.launch.py`（rviz なし）を使うか、
> `launch_rviz:=false` にしてどちらか一方にすること。

---

## 6. 実装を差し替えるところ

この骨組みで「仮」になっているのは次の3か所だけ。他はそのまま使える。

### ① 把持姿勢の計算 — `harvest_sequencer.py: _compute_grasp_pose()`

いまは画像内座標と距離から関節角へ線形写像するだけのスタブ。ここを

- MoveIt2（`moveit_py` / `MoveGroupInterface`）で IK + 経路計画
- piper_ros の `/pos_cmd`（`piper_msgs/PosCmd`）に手先姿勢を直接投げる
- 学習ベースの把持姿勢推定

のいずれかに置き換える。返り値は `joint1..joint6` の配列（rad）。

### ② レモン検出 — `dummy_perception.py`

`/arm_ui/detections` に次のJSONを流すノードに置き換えるだけで、UI側の変更は不要。

```json
{"targets": [
  {"id": 1, "u": 0.31, "v": 0.44, "w": 0.13, "h": 0.17,
   "distance": 0.38, "confidence": 0.92, "label": "lemon"}
]}
```

`u,v` は画像内の正規化中心座標、`w,h` は正規化バウンディングボックス、
`distance` は深度カメラから求めた距離 [m]。

### ③ フロントカメラ — `config/arm_ui.yaml`

アーム用カメラは RealSense で確定済み（`arm_camera_topic`）。
犬型のフロントカメラは未定なので `front_camera_topic: ""` にしてあり、
空のあいだはテストパターンが出る。決まったらトピック名を入れるだけ。
`rgb8 / bgr8 / rgba8 / bgra8 / mono8 / mono16` に対応済み（cv_bridge なしでも変換できる）。

そのほか、移動用UIの「移動 / 収穫 / 帰還 / ストップ」は
`/dog_ui/mode` に `std_msgs/String` でモード名を publish するだけの骨組み。
走行系の担当者が決めたインタフェースに合わせて `app.js` の `TOPIC_DOG_MODE` 周辺を直す。

---

## 7. トピック一覧

| トピック | 型 | 向き | 内容 |
|---|---|---|---|
| `/arm_ui/command` | `std_msgs/String` (JSON) | UI → シーケンサ | 操作コマンド |
| `/arm_ui/state` | `std_msgs/String` (JSON) | シーケンサ → UI | 状態一式（10Hz）。収穫位置の関節角 `basket_poses` も含む |
| `/arm_ui/log` | `std_msgs/String` (JSON) | シーケンサ → UI | イベントログ |
| `/arm_ui/detections` | `std_msgs/String` (JSON) | 認識 → UI/シーケンサ | 検出結果 |
| `/rviz_ui/image` | `sensor_msgs/Image` | rviz_bridge → ui_server | RViz 画面 |
| `/rviz_ui/input` | `std_msgs/String` (JSON) | UI → rviz_bridge | RViz へのマウス操作 |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense → ui_server | カラー画像 |
| `/dog_ui/mode` | `std_msgs/String` | UI → 走行系 | 走行モード |
| `/joint_states` | `sensor_msgs/JointState` | シーケンサ → piper | 関節指令（有効時のみ） |
| `/joint_states_single` | `sensor_msgs/JointState` | piper → シーケンサ | 関節フィードバック |
| `/arm_status` | `piper_msgs/PiperStatusMsg` | piper → シーケンサ | アーム状態（任意） |
| `/end_pose` | `geometry_msgs/Pose` | piper → シーケンサ | 手先姿勢（任意） |

コマンドの例:

```jsonc
{"cmd": "start_selection"}
{"cmd": "select_target", "id": 2}
{"cmd": "plan_grasp"}
{"cmd": "grasp"}
{"cmd": "goto_basket", "index": 1}
{"cmd": "release"}
{"cmd": "home"}
{"cmd": "abort"}
{"cmd": "set_enable", "value": true}
{"cmd": "jog", "joint": 0, "delta": 0.04}
{"cmd": "gripper", "close": true}
```

CLI からも試せる:

```bash
ros2 topic pub --once /arm_ui/command std_msgs/String '{data: "{\"cmd\":\"start_selection\"}"}'
ros2 topic echo /arm_ui/state
```

---

## 8. 現場で気をつけること

- **オフライン前提**で作ってある。roslibjs も three.js も使わず、rosbridge クライアントと
  簡易3Dビューは自前実装。ネットが無い果樹園でもそのまま動く。
- ロボットとタブレットは同じLAN（ロボットのWi-Fi APなど）に置く。
  WebRTC は STUN/TURN なしのローカル接続想定。
- **RViz は Xvfb 上でソフトウェア描画になるので GPU 実機より重い**（実測で 5〜15fps 想定）。
  ミニPCが苦しいようなら `rviz_fps` を下げるか、点群表示を切る。
  モニタを繋げるなら `use_xvfb:=false rviz_display:=:0 launch_rviz:=false` で
  GPU 描画の rviz2 をそのままキャプチャするほうが軽くて速い。
- RViz の `Fixed Frame` は `base_link`（本家設定と同じ）。RobotModel は
  `/robot_description` を購読するので、piper の robot_state_publisher
  （`display_xacro.launch.py` が起動する）が動いている必要がある。
- 関節可動範囲は `common.py` の `JOINT_LIMITS` に集約（`joy_to_piper_jointstates.cpp` と同値）。
  実機で範囲を変えたときは両方直す。
- `LINKS`（`web/js/armview.js`）は「簡易」ビュー用の PiPER のおおよそのリンク長。
  見た目が実機とずれるときはここを URDF の値に合わせる（RViz 表示には影響しない）。
- タッチ操作前提でボタンは 42px 以上。屋外の直射日光下を想定してダークテーマ＋高コントラスト。
