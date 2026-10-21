# 開発用PCのセットアップ（UI 側）

ROS 2 Humble が入っている Ubuntu 22.04 機に、**UI 側一式だけ**を入れて動かす手順。
アームもカメラも繋がっていない状態で UI を一通り触れるようにし、
そのあとミニPCの実機に繋ぐところまで。

所要 20〜30 分。

---

## 目次

0. [前提の確認](#0-前提の確認)
1. [依存パッケージの導入](#1-依存パッケージの導入)
2. [ワークスペースの用意とビルド](#2-ワークスペースの用意とビルド)
3. [モードA：実機なしで動かす](#3-モードa実機なしで動かす)
4. [モードB：ミニPCの実機に繋ぐ](#4-モードbミニpcの実機に繋ぐ)
5. [開発の回し方](#5-開発の回し方)
6. [困ったときは](#6-困ったときは)

---

## 0. 前提の確認

ROS 2 が使える状態か確認する。

```bash
printenv ROS_DISTRO       # humble と出ること
ros2 topic list           # /parameter_events, /rosout が出れば OK
```

何も出ない場合は source を忘れている。

```bash
source /opt/ros/humble/setup.bash
```

**Desktop 版が入っているかも確認する。** UI は RViz を使うので `rviz2` が必要。

```bash
ros2 pkg list | grep -E "^(rviz2|robot_state_publisher)$"
```

両方出れば `ros-humble-desktop` 相当が入っている。出ない場合（`ros-humble-ros-base`
だけを入れた場合）は §1.1 で個別に足す。

---

## 1. 依存パッケージの導入

### 1.1 ROS パッケージ

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server \
                 ros-humble-cv-bridge \
                 ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-rviz-default-plugins
```

| パッケージ | 何に使うか |
|---|---|
| `rosbridge-server` | ブラウザ ↔ ROS の WebSocket ブリッジ（9090番）|
| `cv-bridge` | 画像まわり。`python3-opencv` も一緒に入る |
| `xacro` | URDF の展開（`piper_description` が依存として要求する。既定の起動では使わない）|
| `robot-state-publisher` | `/robot_description` と TF を出す |
| `rviz2` / `rviz-default-plugins` | UI 右上に埋め込む RViz 本体 |

確認する。

```bash
ros2 pkg list | grep -E "rosbridge_server|cv_bridge|^xacro$|robot_state_publisher|^rviz2$"
```

5行出れば OK。

### 1.2 UI 用（apt）

```bash
sudo apt install xvfb ffmpeg python3-xlib python3-opencv
```

| パッケージ | 何に使うか | 無いとどうなるか |
|---|---|---|
| `xvfb` | RViz を出す仮想ディスプレイ | RViz 埋め込みが使えない |
| `python3-xlib` | RViz への視点操作の注入 | RViz が表示のみになる |
| `python3-opencv` | 圧縮画像のデコード、画像のリサイズ | 分散構成で映像が出ない |
| `ffmpeg` | 画面キャプチャの予備手段 | `mss` があれば無くてもよい |

### 1.3 UI 用（pip）

```bash
pip3 install aiohttp aiortc av mss
```

| パッケージ | 何に使うか | 無いとどうなるか |
|---|---|---|
| `aiohttp` | UI の配信と WebRTC シグナリング | UI サーバが起動しない |
| `aiortc` / `av` | WebRTC の映像配信 | 映像が出ない（操作系は動く）|
| `mss` | RViz 画面のキャプチャ | `ffmpeg` で代替される |

### 1.4 まとめて確認

```bash
python3 -c "import aiohttp, aiortc, av, mss, cv2, Xlib; print('Python 依存 OK')"
which Xvfb ffmpeg
```

`~/.local/bin` に入ったものが見えない場合は PATH を通す。

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
```

---

## 2. ワークスペースの用意とビルド

UI 側でも `piper_ros` は必要になる。アームを動かす本体（`piper` パッケージ）ではなく、
**URDF（`piper_description`）とメッセージ定義（`piper_msgs`）**を使うため。

- `piper_description` … RViz にアームのモデルを出すための URDF とメッシュ
- `piper_msgs` … `/arm_status`（`PiperStatusMsg`）を購読するための型定義

```bash
cd ~
git clone -b humble https://github.com/agilexrobotics/piper_ros.git
cd ~/piper_ros/src
unzip ~/Downloads/piper_arm_ui.zip -d .
ls piper_arm_ui        # package.xml, setup.py, web/ が見えること
```

ビルドは**必要な3つだけ**にする。CAN ドライバ本体や Gazebo / MoveIt の設定は
開発用PCでは使わないので、入れると時間がかかるうえ MoveIt の依存で警告が出る。

```bash
cd ~/piper_ros
colcon build --symlink-install \
    --packages-select piper_msgs piper_description piper_arm_ui
source install/setup.bash
```

> 全部ビルドしても動く（`colcon build --symlink-install` だけ）。
> MoveIt が入っていない環境では依存の警告が出るが、
> それらのパッケージは設定ファイルを置くだけなのでビルド自体は通る。

> **`piper_gazebo` は要らない。** `piper_description.xacro` は gazebo 用の記述を含み、
> その中で `$(find piper_gazebo)/config/ros2_controllers.yaml` を参照しているため、
> xacro を展開しようとすると `piper_gazebo` が必要になる。
> この launch は既定で**素の `piper_description.urdf`** を読むので、
> `piper_description` だけあれば RViz にモデルが出る。

`--symlink-install` を付けておくこと。§5 の開発サイクルが楽になる。

毎回 source するのが面倒なら bashrc に追記する。

```bash
echo "source ~/piper_ros/install/setup.bash" >> ~/.bashrc
```

### 確認

```bash
ros2 pkg executables piper_arm_ui
```

次の4つが出れば成功。

```
piper_arm_ui dummy_perception
piper_arm_ui harvest_sequencer
piper_arm_ui rviz_bridge
piper_arm_ui ui_server
```

---

## 3. モードA：実機なしで動かす

アームもカメラも無い状態で、UI を一通り動かすモード。まずはこれで確認する。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_dummy_video:=true enable_arm_command:=true
```

| オプション | 効果 |
|---|---|
| `use_dummy_video:=true` | カメラの代わりにテストパターンを配信 |
| `enable_arm_command:=true` | UI の指令を `/joint_states` に流す（実機が無いので安全）|

`robot_state_publisher`（RViz にモデルを出すのに必要）と `rosbridge` は
既定で一緒に起動するので、指定は不要。

ブラウザで **`http://localhost:8080/`** を開く。

### 確認すること

- [ ] 右上のランプ3つのうち **rosbridge** と **映像** が緑
      （**アーム**は実機が無いので橙のままで正常）
- [ ] カメラ枠にテストパターンとレモンの検出枠3つが出る
- [ ] RViz 枠にアームのモデルが出る
- [ ] RViz 枠をドラッグすると視点が回る
- [ ] 「把持対象の選択」→ 枠をクリック → 「把持姿勢の計算」→「把持」
      →「収穫位置②」→「開放」と進み、**RViz の中でアームが動く**

最後の項目まで通れば、UI 側は完成している。実機に触る前のシーケンス確認は
ここで済ませられる。

---

## 4. モードB：ミニPCの実機に繋ぐ

### 4.1 両方で `ROS_DOMAIN_ID` を揃える

```bash
# 開発用PCとミニPCの両方で
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc && source ~/.bashrc
```

### 4.2 ミニPC側を起動する

ミニPCでは**本家 piper_ros のまま**、次を動かしておく（詳細は
[SETUP_ubuntu2204_humble.md](SETUP_ubuntu2204_humble.md) の §04・§05）。

```bash
# ミニPC側。初回だけ圧縮配信のパッケージを入れる
sudo apt install ros-humble-compressed-image-transport

bash ~/piper_ros/can_activate.sh can0 1000000
ros2 launch piper start_single_piper.launch.py \
    can_port:=can0 auto_enable:=true gripper_exist:=true
ros2 launch realsense2_camera rs_launch.py enable_color:=true
```

### 4.3 疎通を確認する

**開発用PC側**で実行して、ミニPCのトピックが見えていれば繋がっている。

```bash
ros2 node list                        # /piper_ctrl_single_node が見えるか
ros2 topic hz /joint_states_single    # 関節角が流れているか
ros2 topic echo /arm_status --once
ros2 topic hz /camera/camera/color/image_raw/compressed
```

見えない場合は §6 を参照。

### 4.4 UI を起動する

```bash
ros2 launch piper_arm_ui arm_ui.launch.py camera_transport:=compressed
```

`enable_arm_command` は **付けない**（既定 false）。この状態では UI は監視のみで、
アームには一切指令を出さない。まず映像と状態表示が正しいことを確認する。

- 右上のランプ3つがすべて緑になる
- 関節テーブルの数値が実機の姿勢と一致する
- カメラ枠に RealSense の映像が出る

### 4.5 UI からアームを動かす

```
安全確認：周囲に人がいないこと、非常停止に手が届くこと。
ゲームパッド操作と同時には使わない（両方が /joint_states を出すと競合する）。
```

確認できたら、UI 上の「**UI操作を有効化**」を ON にする。
最初は穏やかな速度にしておくとよい。

```bash
# config/arm_ui.yaml の motion_speed を 20〜40 にしてから起動
ros2 launch piper_arm_ui arm_ui.launch.py camera_transport:=compressed
```

---

## 5. 開発の回し方

### 5.1 何を直したら何を再起動するか

| 直したもの | 必要な操作 |
|---|---|
| `web/*.html` `*.css` `*.js` | **ブラウザの再読込だけ**（`--symlink-install` のため）|
| `piper_arm_ui/*.py`（ノード）| そのノードだけ再起動 |
| `launch/*.py`, `config/*.yaml` | launch し直し |
| `package.xml`, `setup.py` | `colcon build` し直し |

`--symlink-install` で反映されないときは、素直に
`colcon build --packages-select piper_arm_ui` し直す。

### 5.2 ノードを個別に起動する

launch でまとめて起動すると、1つ直すたびに全部落とすことになる。
UI だけを触っているときは、端末を分けて個別に起動したほうが速い。

```bash
# 端末1: rosbridge（ほぼ触らない）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 端末2: UI サーバ（web/ を触るならここだけ再起動）
ros2 run piper_arm_ui ui_server --ros-args -p use_dummy_video:=true

# 端末3: シーケンサ（状態機械を触るならここだけ再起動）
ros2 run piper_arm_ui harvest_sequencer --ros-args -p enable_arm_command:=true

# 端末4: 検出のダミー
ros2 run piper_arm_ui dummy_perception

# 端末5: RViz 埋め込み（重いので不要なときは起動しない）
ros2 run piper_arm_ui rviz_bridge
```

RViz 枠が要らないときは `rviz_bridge` を起動しなければよい。
UI 側はパネル右上の「簡易」に切り替えれば自前の姿勢ビューが出る。

### 5.3 RViz を自分の画面にも出す

仮想ディスプレイの中身が見えないと RViz の設定をいじりにくい。
自分の画面（`:0`）に rviz2 を出して、それをそのままキャプチャさせることもできる。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_dummy_video:=true enable_arm_command:=true \
    use_xvfb:=false rviz_display:=:0 allow_rviz_input:=false
```

RViz の窓が自分の画面に開き、同じ画面が UI にも映る。GPU 描画になるので軽い。

> **`allow_rviz_input:=false` を必ず付けること。** これを忘れると、
> ブラウザで RViz 枠をドラッグしたときに**自分の実際のマウスカーソルが動く**。

### 5.4 ブラウザ側のデバッグ

UI は素の HTML/CSS/JS なので、ブラウザの開発者ツールがそのまま使える。

- **Console** … `app.js` のエラー、rosbridge の接続状況
- **Network → WS** … rosbridge の WebSocket。送受信している JSON がそのまま見える
- `chrome://webrtc-internals` … WebRTC の統計（解像度、fps、パケットロス）

ROS 側から状態を覗くこともできる。

```bash
ros2 topic echo /arm_ui/state      # UI に送っている状態一式
ros2 topic echo /arm_ui/command    # UI から来たコマンド
```

コマンドは CLI からも投げられるので、ボタンを押さずに試せる。

```bash
ros2 topic pub --once /arm_ui/command std_msgs/String \
  '{data: "{\"cmd\":\"start_selection\"}"}'
```

---

## 6. 困ったときは

### 6.0 「address already in use」で起動しない

開発用PCには他のプロジェクトが同居していることが多く、これが一番よく出る。
起動ログにこう出たら該当する。

```
ui_server: OSError: [Errno 98] ... address already in use
rosbridge_websocket: Unable to start server: [Errno 98] Address already in use
Xvfb: Server is already active for display 99
```

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

`ss` の出力が別プロジェクト（`webrtc_arm_controller` など）だったら、
そちらを止めるか、UI 側の番号をずらす。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    port:=8081 rosbridge_port:=9091 use_dummy_video:=true
# → http://localhost:8081/
```

ブラウザ側の設定は要らない。UI は起動時に `/api/info` を読んで
そこに書かれた rosbridge のポートへ繋ぐ。
仮想ディスプレイの番号は、指定した番号が埋まっていれば `rviz_bridge` が
空き番号へ自動でずらし、warn ログにそのことを出す。

### 6.05 装飾のない素の HTML が出る

見出しと箇条書きだけの白い画面になり、ボタンを押しても何も起きない場合は、
CSS と JS が 404 になっている（HTML だけ配信できている状態）。

`colcon build --symlink-install` で入れると `share/piper_arm_ui/web/` の中身は
ソースツリーへのシンボリックリンクになる。これを配信できていないのが原因。
**2026-09-07 より前の版のバグで、現在の版では修正済み。** 更新して入れ直す。

```bash
cd ~/piper_ros
colcon build --packages-select piper_arm_ui --symlink-install
source install/setup.bash
```

確認は Chrome の DevTools（F12）→ Network で、`style.css` と `app.js` が
200 で返っているかを見る。ui_server 側も起動時に静的ファイルの有無を確かめて、
足りなければ error ログを出す。

### 6.1 症状別

| 症状 | 確認すること |
|---|---|
| `Package 'piper_arm_ui' not found` | `source ~/piper_ros/install/setup.bash` したか |
| ビルドで `piper_msgs` が見つからない | `--packages-select` に `piper_msgs` を含めたか |
| UI は出るがランプが全部赤 | ノードが起動しているか（`ros2 node list`）|
| 「rosbridge」が赤 | 9090番が使われていないか（`ss -ltnp \| grep 9090`）→ §6.0 |
| 「映像」が赤 | `python3 -c "import aiortc"` が通るか。8080番が空いているか → §6.0 |
| `address already in use` で起動しない | §6.0 |
| 画面がチカチカする／表示が高速に切り替わる | ノードの二重起動。`ros2 topic info /arm_ui/state` の Publisher count を見る。`pkill -f harvest_sequencer` で止めて launch し直す |
| 装飾のない素の HTML が出る／ボタンが効かない | §6.05（CSS・JS が 404） |
| RViz 枠に前回の古い画面が出る | ディスプレイ番号の衝突。起動ログの `ディスプレイ :99 は使用中` を確認 |
| RViz 枠が真っ黒 | `which Xvfb`。`ros2 topic hz /rviz_ui/image` で流れているか |
| RViz にモデルが出ない | `ros2 topic echo /robot_description --once`。`piper_description` をビルドしたか |
| 起動時に `package 'piper_gazebo' not found` | 古い版の launch。`robot_description_file` が `.xacro` を指していないか確認する。既定は `.urdf` |
| 姿勢が実機とずれる（古いファーム）| `robot_description_file:=$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_description_old.urdf` |
| RViz のモデルが動かない | RViz が映すのは `/joint_states`（＝指令）。`enable_arm_command:=true` にしたか |
| RViz の視点操作が効かない | `python3 -c "import Xlib"` が通るか。UI の「視点操作」が ON か |
| RViz にメニューバーが映り込む | `python3 -c "import Xlib"` か `which xdotool`。F11 を送れないと消せない。窓が出る前に送っている場合は `chrome_delay` を伸ばす |
| RViz の動きがカクつく | 解像度を下げて fps を保つ: `rviz_width:=848 rviz_height:=480`。起動ログの「RViz 配信 N fps」で実際の値を確認 |
| RViz の初期視点を変えたい | `view_yaw:=0.0`（正面）/ `1.5708`（左）/ `-1.5708`（右）。既定は後ろから |
| 実機のトピックが見えない | 両方で `ROS_DOMAIN_ID` が同じか。`printenv ROS_DOMAIN_ID` |
| 同上（それでも見えない） | `RMW_IMPLEMENTATION` を両側とも既定（`rmw_fastrtps_cpp`）に戻す |
| 同上（Wi-Fi 経由） | AP がマルチキャストを通しているか。有線で試す |
| 映像だけ来ない（分散時） | ミニPC側に `compressed-image-transport` を入れたか。`camera_transport:=compressed` を付けたか |
| 映像がカクつく（分散時） | `video_fps` を下げる。ミニPC側の解像度を下げる |

### 切り分けの順番

1. `ros2 node list` — ノードが立っているか
2. `ros2 topic list` — トピックが出ているか
3. `ros2 topic hz <topic>` — 実際に流れているか
4. ブラウザの Console — UI 側のエラー
5. `ros2 topic echo /arm_ui/state` — UI に届いている中身
