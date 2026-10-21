# 環境構築手順（Ubuntu 22.04 + ROS 2 Humble）

まっさらな Ubuntu 22.04 から、レモン自動収穫デモのアーム側一式
（PiPER + RealSense + アーム操作UI）が動くまでの手順。

所要時間は回線速度にもよるがおよそ 1〜2 時間。

**対象**: アーム制御用ミニPC（x86_64 / Ubuntu 22.04 Desktop）
**最終形**: ブラウザで `http://<ミニPCのIP>:8080/` を開くと操作UIが出る状態

---

## 目次

0. [はじめに確認すること](#0-はじめに確認すること)
1. [Ubuntu の下準備](#1-ubuntu-の下準備)
2. [ROS 2 Humble のインストール](#2-ros-2-humble-のインストール)
3. [PiPER（piper_ros）のセットアップ](#3-piperpiper_ros-のセットアップ)
4. [CAN モジュールの設定](#4-can-モジュールの設定)
5. [RealSense のセットアップ](#5-realsense-のセットアップ)
6. [rosbridge・ゲームパッド](#6-rosbridgeゲームパッド)
7. [piper_arm_ui の依存パッケージ](#7-piper_arm_ui-の依存パッケージ)
8. [piper_arm_ui の配置とビルド](#8-piper_arm_ui-の配置とビルド)
9. [段階的な動作確認](#9-段階的な動作確認)
10. [起動チートシート](#10-起動チートシート)
11. [ネットワーク設定（タブレットから見る）](#11-ネットワーク設定タブレットから見る)
12. [困ったときは](#12-困ったときは)

---

## 0. はじめに確認すること

### 0.1 どのPCで作業するのか

この手順書は **アームとカメラが物理的に繋がっているミニPC**（犬型ロボットに載せる機体）
に対して行うもの。UI を見るタブレットや手元のノートPCには**何もインストールしない**。

```
┌─────────────────────────┐        ┌──────────────────┐
│  ミニPC（この手順の対象）   │        │  タブレット / ノートPC │
│  犬型ロボットに搭載         │  LAN   │                  │
│                         │◄──────►│  ブラウザだけ       │
│  ├ USB-CAN ─→ PiPER     │        │  インストール不要    │
│  ├ USB ─→ RealSense     │        │  OS も何でもよい    │
│  └ ROS 2 + UI サーバ一式  │        │                  │
└─────────────────────────┘        └──────────────────┘
```

ブラウザで開くだけで操作できるようにしたのは、まさにこの
「見る側に何も入れなくていい」ためなので、現場でタブレットを追加するのも
別の人に見せるのも、ブラウザで `http://<ミニPCのIP>:8080/` を開くだけで済む。

**ミニPCの前に座って作業する必要はない。** 同じLANから SSH で入って進めてよい。
ただし §2.4 の `rviz2` の起動確認と §9 の UI の目視確認だけは画面が要るので、
ミニPCにモニタを繋ぐか、SSH の X 転送（`ssh -X`）を使う。

### 0.2 手元のPCで開発だけしたい場合

UI のプログラムを書く・見た目を直すだけなら、アームもカメラも無い普通の
Ubuntu 22.04 機で作業できる。その場合は一部の節を飛ばせる。

| 節 | ミニPC（実機） | 開発用PC（実機なし） |
|---|---|---|
| 1. Ubuntu の下準備 | 必要 | 必要 |
| 2. ROS 2 Humble | 必要 | 必要 |
| 3. PiPER（piper_ros） | 必要 | 必要（URDF と RViz 表示に使う） |
| 4. CAN モジュール | 必要 | **不要** |
| 5. RealSense | 必要 | **不要**（ダミー映像で代用） |
| 6. rosbridge・ゲームパッド | 必要 | rosbridge のみ必要 |
| 7. UI の依存パッケージ | 必要 | 必要 |
| 8. piper_arm_ui のビルド | 必要 | 必要 |
| 9. 動作確認 | 全部 | ダミーの節まで |

開発用PCでの起動はこれ1行。**アームもカメラも無しで UI が一通り動く。**

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    use_dummy_video:=true enable_arm_command:=true
```

| オプション | 効果 |
|---|---|
| `use_dummy_video:=true` | カメラの代わりにテストパターンを配信する |
| `enable_arm_command:=true` | UI の指令を `/joint_states` に流す（実機が無いので安全） |

`robot_state_publisher`（RViz にモデルを出すのに要る）は既定で起動するので、
指定は不要。この状態だと **UI のボタン操作がそのまま RViz のモデルに反映される**。
「把持対象の選択 → 把持姿勢の計算 → 把持 → 収穫位置 → 開放」を一巡させると
RViz の中でアームが動くので、実機に触る前にシーケンスの確認ができる。

なお `ros2 launch piper_description display_xacro.launch.py` でも RViz にモデルは出るが、
こちらは `joint_state_publisher_gui`（スライダーの窓）と rviz2 も一緒に起動する。
**実機では単体で使わないこと** — `joint_state_publisher_gui` が `/joint_states` を
publish するので、UI やゲームパッドの指令と競合する。実機側は
`start_single_piper.launch.py` を使う。

### 0.3 ミニPCと開発用PCで分担する場合

ミニPC には **本家 piper_ros だけ**を置いて、UI 側（WebRTC・rosbridge・RViz）を
開発用PCで動かすこともできる。ノード間はすべて ROS のトピックなので、
同じ LAN・同じ `ROS_DOMAIN_ID` にすれば DDS が繋いでくれる。

| | ミニPC | 開発用PC |
|---|---|---|
| 入れるもの | 本家 piper_ros のみ | piper_ros + piper_arm_ui |
| 動かすもの | `piper_single_ctrl` / `realsense2_camera` | `ui_server` / `rviz_bridge` / `harvest_sequencer` / `rosbridge` / `robot_state_publisher` |
| 手順の §04 CAN | 必要 | 不要 |
| 手順の §05 RealSense | 必要 | 不要 |

```bash
# ミニPC側
export ROS_DOMAIN_ID=42
sudo apt install ros-humble-compressed-image-transport   # 映像の圧縮配信
ros2 launch piper start_single_piper.launch.py \
    can_port:=can0 auto_enable:=true gripper_exist:=true
ros2 launch realsense2_camera rs_launch.py enable_color:=true

# 開発用PC側
export ROS_DOMAIN_ID=42
ros2 launch piper_arm_ui arm_ui.launch.py camera_transport:=compressed
```

**帯域に注意。** 生の `sensor_msgs/Image` を LAN に流すと 640x480 の rgb8 で
30fps なら約 **221 Mbps** になり、Wi-Fi では現実的でない。
`camera_transport:=compressed` にすると `<トピック名>/compressed` を購読するようになり、
実際の果樹園の絵でおおむね 10〜30 Mbps 程度まで落ちる。

**ミニPC側は piper_ros のソースに一切手を加えない。** ただし本家 README どおりの
セットアップ（§03 の依存とビルド、§04 の CAN、§05 の RealSense）は必要で、
それに加えて上の `compressed-image-transport` と `ROS_DOMAIN_ID` だけ足す。
`piper_arm_ui` や `xvfb` / `aiortc` などの UI 系は**ミニPCには入れなくてよい**。

疎通の確認は開発用PC側で。

```bash
ros2 node list                        # /piper_ctrl_single_node が見えるか
ros2 topic hz /joint_states_single    # 関節角が流れているか
ros2 topic hz /camera/camera/color/image_raw/compressed
```

そのほかの注意点。

- 既定の DDS はマルチキャストで相手を見つける。マルチキャストを通さない Wi-Fi AP だと
  繋がらないので、AP の設定を見直すか有線にする。
- `RMW_IMPLEMENTATION` は両側とも既定（`rmw_fastrtps_cpp`）のままにする。
  片方だけ cyclonedds などに変えると繋がらない。
- 開発用PCは実機を触らないので `piper_sdk` は無くてもビルドできる（実行しないため）。
- `ROS_DOMAIN_ID` を両方で揃えること。揃っていないと何も見えない。
- **デモ本番はミニPCに全部載せるのが安全**。この構成は開発中の切り分け用と考えたほうがよい。
  Wi-Fi が切れると UI ごと落ちる。

### 0.4 前提の確認

| 項目 | 内容 |
|---|---|
| OS | Ubuntu 22.04 LTS（Desktop 版）。Server 版だと RViz に必要な X 環境が無い |
| アーキテクチャ | x86_64 想定。ARM（Jetson 等）は §7 の注記を参照 |
| 用意するもの | PiPER 本体、付属の USB-CAN モジュール、RealSense（D400 系）、USB ケーブル、ゲームパッド |
| ネットワーク | インストール中はインターネット必須。デモ本番はオフラインで可 |

USB は **必ず USB3.0 ポート**に挿す。RealSense は USB2.0 だと解像度・フレームレートが
大きく制限される。

```bash
# 前提の確認
lsb_release -a          # Ubuntu 22.04 であること
uname -m                # x86_64
```

---

## 1. Ubuntu の下準備

### 1.1 システム更新（先にやること）

ROS 2 の公式手順に明記された注意点として、**ROS 2 を入れる前に systemd と udev 関連を
更新しておく**必要がある。これを飛ばすと、ROS 2 のインストール時に重要なシステム
パッケージが削除されてしまうことがある。まず素直に全体を更新する。

```bash
sudo apt update
sudo apt upgrade -y
sudo reboot
```

### 1.2 ロケール（UTF-8）

ROS 2 は UTF-8 ロケールを前提にしている。日本語環境で使う場合も、
`en_US.UTF-8` を生成しておくと余計なトラブルがない。

```bash
locale   # UTF-8 になっているか確認

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale   # 反映を確認
```

---

## 2. ROS 2 Humble のインストール

> **注意**: ネット上の古い記事にある `apt-key` や
> `/usr/share/keyrings/ros-archive-keyring.gpg` を直接置く方式は現在は使われていない。
> 下記の `ros2-apt-source` パッケージを入れる方式が現行の公式手順。

### 2.1 リポジトリの登録

```bash
# universe リポジトリを有効化
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS 2 の apt ソースを入れる
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

`ROS_APT_SOURCE_VERSION` は GitHub の API から最新版のタグを取ってくる。
**社内プロキシや API の回数制限で空になることがある**ので、必ず確認する。

```bash
echo "$ROS_APT_SOURCE_VERSION"    # 空でなければ OK（例: 1.1.0）
```

空だった場合は URL が壊れて `dpkg -i` が分かりにくいエラーになる。
その場合は <https://github.com/ros-infrastructure/ros-apt-source/releases>
をブラウザで開いて最新のタグを確認し、手で指定する。

```bash
export ROS_APT_SOURCE_VERSION=1.1.0    # 実際の最新タグに置き換える
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
file /tmp/ros2-apt-source.deb   # "Debian binary package" と出ること
sudo dpkg -i /tmp/ros2-apt-source.deb
```

`JSON text data` などと出た場合はダウンロードに失敗しているので、
URL とネットワークを見直す。

### 2.2 本体のインストール

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools
```

`ros-humble-desktop` に RViz2、rqt、デモ類が入る。
`ros-dev-tools` に colcon、rosdep、vcstool などのビルド用ツールが入る。

### 2.3 環境設定

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update
```

### 2.4 動作確認

端末を2つ開いて、それぞれで実行する。

```bash
# 端末A
ros2 run demo_nodes_cpp talker
```
```bash
# 端末B
ros2 run demo_nodes_py listener
```

`I heard: [Hello World: 1]` のように出れば OK。`Ctrl+C` で止める。

```bash
# RViz2 も開いてみる（GUI が出れば OK）
rviz2
```

---

## 3. PiPER（piper_ros）のセットアップ

### 3.1 依存パッケージ

```bash
# Python 側
sudo apt install python3-pip -y
pip3 install python-can scipy piper_sdk

# ROS 側
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-xacro \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui
```

`pip3 install` で `~/.local/bin` に入ったものが使えない場合は PATH を通す。

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
```

### 3.2 取得とビルド

`piper_ros` は中に `src/` を持つ **それ自体がワークスペース**になっている。
ホーム直下に置いて、そこをワークスペースとして使うのが素直。

```bash
cd ~
git clone -b humble https://github.com/agilexrobotics/piper_ros.git
cd ~/piper_ros
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` を付けておくと、Python ファイルや設定ファイルを編集したときに
ビルドし直さなくても反映されるので開発中は便利。

毎回 source するのが面倒なら bashrc に追記する。

```bash
echo "source ~/piper_ros/install/setup.bash" >> ~/.bashrc
```

### 3.3 URDF のバージョンに注意

アームのファームウェアによって使う URDF が変わる。

| ファームウェア | URDF |
|---|---|
| S-V1.6-3 より前 | `piper_description_old.urdf` |
| S-V1.6-3 以降 | `piper_description.urdf`（既定） |

既定は後者。古いファームのままだと **RViz の姿勢と実機がずれる**ので、
J2・J3 が 2 度ほどおかしいと感じたらここを疑う。古い方に切り替えるには
起動時に指定する。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
  robot_description_file:=$(ros2 pkg prefix piper_description)/share/piper_description/urdf/piper_description_old.urdf
```

> なお本 launch は RViz 用に **素の `.urdf`** を読む。`piper_description.xacro` は
> gazebo 用の記述の中で `$(find piper_gazebo)` を参照しているため、
> xacro を展開すると `piper_gazebo` までビルドが必要になるのを避けている。

---

## 4. CAN モジュールの設定

PiPER は付属の USB-CAN モジュール経由で通信する。ここは**PCを再起動するたびに
必要な設定**なので、最後に自動化まで書いておく。

### 4.1 ツールの導入

```bash
sudo apt update
sudo apt install can-utils ethtool iproute2
```

### 4.2 モジュールの検出

アームの CAN モジュールを USB に挿してから実行する。

```bash
cd ~/piper_ros
bash find_all_can_port.sh
```

こう出れば認識されている。

```
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

`Both ethtool and can-utils are installed.` だけで終わる場合は**認識されていない**。
ケーブル、USB ポート、アームの電源を確認する。

### 4.3 有効化

CAN モジュールが1つだけの場合。

```bash
cd ~/piper_ros
bash can_activate.sh can0 1000000
```

`1000000`（ボーレート）は変更してはいけない。`can0` の部分は好きな名前でよい。

複数の CAN モジュールを挿している場合（走行系と併用するときなど）は、
USB ポートを指定して名前を固定する。

```bash
# 3-1.4:1.0 は find_all_can_port.sh で調べた値
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

確認する。

```bash
ip -details link show can0
# state UP と bitrate 1000000 が見えれば OK
```

### 4.4 起動時に自動で有効化する（推奨）

デモ現場で毎回コマンドを打つのは事故のもとなので、systemd サービスにしておく。

```bash
sudo tee /etc/systemd/system/piper-can.service > /dev/null <<'EOF'
[Unit]
Description=Activate PiPER USB-CAN interface
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
# USB の認識が終わるのを少し待つ
ExecStartPre=/bin/sleep 5
ExecStart=/bin/bash /home/USER/piper_ros/can_activate.sh can0 1000000

[Install]
WantedBy=multi-user.target
EOF

# USER を自分のユーザ名に置き換える
sudo sed -i "s|/home/USER|$HOME|" /etc/systemd/system/piper-can.service

sudo systemctl daemon-reload
sudo systemctl enable --now piper-can.service
sudo systemctl status piper-can.service
```

再起動して `ip link show can0` が UP になっていれば成功。
うまくいかない場合は `ExecStartPre` の待ち時間を伸ばす。

---

## 5. RealSense のセットアップ

### 5.1 まずは ROS の apt パッケージで入れる

```bash
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-*
```

### 5.2 動作確認

RealSense を USB3.0 ポートに挿してから実行する。

```bash
ros2 launch realsense2_camera rs_launch.py
```

別端末でトピックを確認する。

```bash
ros2 topic list | grep camera
```

`/camera/camera/color/image_raw` が見えれば OK。
**ここで実際に出たトピック名を控えておくこと。** バージョンや起動方法によって
`/camera/color/image_raw` になることもあり、UI 側の設定に使う。

画像が来ているかも確認しておく。

```bash
ros2 topic hz /camera/camera/color/image_raw
```

### 5.3 カメラが見つからない場合（udev ルール）

`No RealSense devices were found` のように出る場合は、
一般ユーザからデバイスにアクセスする権限（udev ルール）が入っていない可能性が高い。
Intel 公式の apt リポジトリを足して SDK を入れると udev ルールも設定される。

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
  gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev
```

カメラを挿し直してから、単体ツールで確認する。

```bash
realsense-viewer
```

ここで映れば SDK 側は問題なく、ROS 側の設定の話になる。

---

## 6. rosbridge・ゲームパッド

```bash
sudo apt install ros-humble-rosbridge-server \
                 ros-humble-cv-bridge \
                 ros-humble-joy
```

ゲームパッドの確認（既存の `joy_to_piper_jointstates` を使うなら必要）。

```bash
# 端末A
ros2 run joy joy_node
```
```bash
# 端末B
ros2 topic echo /joy
```

スティックを動かして `axes` の値が変われば OK。
デバイスが `/dev/input/js0` に出ていない場合は `ls /dev/input/` で確認する。

---

## 7. piper_arm_ui の依存パッケージ

```bash
# 画面キャプチャ・入力注入（RViz 埋め込み用）
sudo apt install xvfb ffmpeg python3-xlib

# WebRTC と画面キャプチャ
pip3 install aiohttp aiortc av mss
```

それぞれの役割は次の通り。

| パッケージ | 役割 | 無いとどうなるか |
|---|---|---|
| `aiohttp` | UI の配信と WebRTC シグナリング | UI サーバが起動しない |
| `aiortc` / `av` | WebRTC の映像配信 | 映像が出ない（操作系は動く） |
| `mss` | RViz 画面のキャプチャ | `ffmpeg` があればそちらで代替 |
| `xvfb` | RViz を出す仮想ディスプレイ | RViz 埋め込みが使えない |
| `python3-xlib` | RViz への視点操作の注入 | RViz が表示のみになる |

> **ARM 機（Jetson 等）の場合**: `aiortc` のホイールが無くソースからビルドになることが
> あるので、先に依存を入れておく。
> ```bash
> sudo apt install libopus-dev libvpx-dev libsrtp2-dev libffi-dev libssl-dev
> ```

---

## 8. piper_arm_ui の配置とビルド

配布した zip を `piper_ros` の `src/` に展開する。

```bash
cd ~/piper_ros/src
unzip ~/Downloads/piper_arm_ui.zip -d .
ls piper_arm_ui       # package.xml, setup.py, web/ などが見えること

cd ~/piper_ros
colcon build --packages-select piper_arm_ui --symlink-install
source install/setup.bash
```

ビルドが通ったか確認する。

```bash
ros2 pkg executables piper_arm_ui
# ui_server / rviz_bridge / harvest_sequencer / dummy_perception の4つが出る
```

---

## 9. 段階的な動作確認

一度に全部起動せず、下から順に確かめると原因の切り分けが楽になる。

### 9.1 アーム単体

```bash
# CAN を有効化してから
ros2 launch piper start_single_piper.launch.py \
    can_port:=can0 auto_enable:=true gripper_exist:=true
```

別端末で確認する。

```bash
ros2 topic echo /joint_states_single --once   # 現在の関節角
ros2 topic echo /arm_status --once            # アームの状態
```

> **注意**: `start_single_piper_rviz.launch.py` は自前で RViz2 を起動する。
> 後述の `rviz_bridge` と併用すると RViz が2つ立ち上がるので、
> UI を使うときは **rviz なしの `start_single_piper.launch.py`** を使う。

### 9.2 UI 一式（カメラ無しで先に見る）

```bash
ros2 launch piper_arm_ui arm_ui.launch.py use_dummy_video:=true
```

ミニPC のブラウザで `http://localhost:8080/` を開く。

- 右上のランプ3つ（rosbridge / 映像 / アーム）が緑になるか
- カメラ枠にテストパターンとレモンの検出枠が出るか
- RViz 枠にロボットモデルが出るか
- 「把持対象の選択」→ 枠をクリック →「把持姿勢の計算」と進むか

### 9.3 RealSense をつなぐ

§5.2 で控えたトピック名を指定する。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py \
    arm_camera_topic:=/camera/camera/color/image_raw
```

カメラ枠に実際の映像が出れば完成。

### 9.4 アームを UI から動かす

ここから実際にアームが動く。**周囲に人がいないこと、非常停止に手が届くことを確認**してから。

UI の「UI操作を有効化」を ON にする。既存のゲームパッド操作と同時には使わない
（両方が `/joint_states` を出すと指令が競合する）。

最初は穏やかな速度で試すとよい。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py enable_arm_command:=true
# config/arm_ui.yaml の motion_speed を 20〜40 にしておくとゆっくり動く
```

---

## 10. 起動チートシート

デモ本番の手順。端末を3つ使う場合。

```bash
# 端末A: CAN（systemd 化していれば不要）+ アーム
cd ~/piper_ros && bash can_activate.sh can0 1000000
ros2 launch piper start_single_piper.launch.py \
    can_port:=can0 auto_enable:=true gripper_exist:=true

# 端末B: RealSense
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=true

# 端末C: UI 一式（rosbridge と RViz も一緒に立ち上がる）
ros2 launch piper_arm_ui arm_ui.launch.py
```

RealSense も UI の launch にまとめる場合。

```bash
# 端末B は不要になる
ros2 launch piper_arm_ui arm_ui.launch.py start_realsense:=true
```

ブラウザで `http://<ミニPCのIP>:8080/` を開く。IP は `hostname -I` で分かる。

よく使うオプション。

| やりたいこと | オプション |
|---|---|
| カメラ無しで画面だけ確認 | `use_dummy_video:=true` |
| RealSense のトピック名が違う | `arm_camera_topic:=/camera/color/image_raw` |
| UI からアームを動かす | `enable_arm_command:=true` |
| RViz 埋め込みを使わない | `start_rviz_bridge:=false` |
| モニタ接続済みで :0 の RViz を映す | `use_xvfb:=false rviz_display:=:0 launch_rviz:=false` |
| RViz のドックも表示する | `hide_docks:=false` |
| RViz のメニューバー等も映す | `hide_chrome:=false`（既定はモデルだけ）|
| RViz をもっとなめらかにする | `rviz_fps:=30` |
| RViz が重い／カクつく | `rviz_width:=848 rviz_height:=480`（解像度を下げて fps を保つ）|
| RViz を正面から見る | `view_yaw:=0.0`（既定 3.14159 = 後ろから）|
| RViz を左から / 右から見る | `view_yaw:=1.5708` / `view_yaw:=-1.5708` |
| RViz をもっと見下ろす | `view_pitch:=0.6`（0 が真横）|
| rosbridge を別で動かしている | `start_rosbridge:=false` |
| 8080 / 9090 が他に使われている | `port:=8081 rosbridge_port:=9091`（§12 参照）|
| Xvfb の :99 が埋まっている | `rviz_display:=:98`（指定しなくても空き番号へ自動でずれる）|
| UI を別PCで動かして LAN 越しに映像を受ける | `camera_transport:=compressed` |
| RViz のモデルを別 launch で出している | `start_robot_state_publisher:=false` |
| 実機なしで RViz にアームを出す | 既定のまま（`enable_arm_command:=true` と併用）。UI の指令が `/joint_states` に出てモデルが動く |
| 古いファームで姿勢がずれる | `robot_description_file:=.../piper_description_old.urdf` |

---

## 11. ネットワーク設定（タブレットから見る）

ミニPC とタブレットを同じ LAN に置く。現場ではミニPC を Wi-Fi アクセスポイントに
するか、モバイルルータを1台持ち込むのが確実。

```bash
hostname -I     # ミニPC の IP を確認
```

タブレットのブラウザで `http://<その IP>:8080/` を開く。

ファイアウォールが有効な場合はポートを開ける。

```bash
sudo ufw status
sudo ufw allow 8080/tcp    # Web UI と WebRTC シグナリング
sudo ufw allow 9090/tcp    # rosbridge
```

複数台の ROS 2 マシンが同じ LAN にいる場合（走行系の担当者と同時に作業するときなど）は、
`ROS_DOMAIN_ID` を揃える／分けることで混線を防げる。

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

**この UI 自体に認証は無い**。同じ LAN にいる人は誰でも操作できるので、
デモ用の閉じたネットワークで使うこと。

---

## 12. 困ったときは

### 12.0 「address already in use」で起動しない

```
ui_server: OSError: [Errno 98] ... address already in use
rosbridge_websocket: Unable to start server: [Errno 98] Address already in use
Xvfb: Server is already active for display 99
```

前回の起動が残っているか、別のプログラムが同じ番号を使っている。
掴んでいる相手を確かめる。

```bash
ss -ltnp | grep -E ':8080|:9090'
ls /tmp/.X*-lock
```

残骸なら片付ける。

```bash
pkill -f ui_server
pkill -f rosbridge_websocket
pkill -f 'Xvfb :99' && rm -f /tmp/.X99-lock
```

別プログラムが正当に使っているなら番号をずらす。

```bash
ros2 launch piper_arm_ui arm_ui.launch.py port:=8081 rosbridge_port:=9091
# → http://<ロボットのIP>:8081/
```

`rosbridge_port` を変えてもブラウザ側の設定は不要。UI は `/api/info` を読んで
そこに書かれたポートへ繋ぐ。仮想ディスプレイは、指定した番号が埋まっていれば
`rviz_bridge` が空き番号へ自動でずらして warn ログを出す。

### 12.05 装飾のない素の HTML が出る

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

ブラウザの DevTools（F12）→ Network で `style.css` と `app.js` が 200 で
返っているかを見れば確実。ui_server も起動時に静的ファイルの有無を確かめ、
足りなければ error ログを出す。

### 12.1 症状別

| 症状 | 確認すること |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` したか。bashrc に入れたか |
| `address already in use` で起動しない | §12.0 |
| 画面がチカチカする／表示が高速に切り替わる | ノードの二重起動。`ros2 topic info /arm_ui/state` の Publisher count を見る。`pkill -f harvest_sequencer` で止めて launch し直す |
| 装飾のない素の HTML が出る／ボタンが効かない | §12.05（CSS・JS が 404） |
| `Package 'piper_arm_ui' not found` | `source ~/piper_ros/install/setup.bash` したか |
| CAN が UP にならない | `find_all_can_port.sh` で認識されているか。アームの電源、ボーレートが `1000000` か |
| アームが動かない | `auto_enable:=true` で起動したか。`ros2 topic echo /arm_status` の `err_code` を確認 |
| UI の「アーム」ランプが赤 | `/joint_states_single` が出ているか（`ros2 topic hz` で確認） |
| UI の「rosbridge」ランプが赤 | 9090 番が開いているか。`start_rosbridge:=false` にしていないか |
| UI の「映像」ランプが赤 | `pip3 list \| grep aiortc`。8080 番がファイアウォールで塞がれていないか |
| カメラ枠が「waiting for...」 | トピック名が合っているか。`ros2 topic hz <トピック名>` で実際に流れているか |
| RViz 枠が真っ黒 | `xvfb` が入っているか。`ros2 topic hz /rviz_ui/image` で画面が流れているか |
| RViz にロボットが出ない | `ros2 topic echo /robot_description --once` で出ているか。既定では本 launch が `robot_state_publisher` を起動する（piper の `start_single_piper.launch.py` は起動しない）。Fixed Frame が `base_link` か |
| RViz のモデルが動かない | RViz が映すのは `/joint_states`（＝指令）。UI操作もゲームパッドも止まっていると更新されない |
| RViz が重い | `rviz_fps` を下げる。点群表示を切る。モニタを繋いで `use_xvfb:=false` にすると GPU 描画で速くなる |
| RViz の視点が動かない | `python3-xlib` か `xdotool` が入っているか。UI の「視点操作」が ON か |
| 姿勢が実機とずれる | URDF のバージョン（§3.3）。ファームが S-V1.6-3 より古くないか |
| ゲームパッドと UI が喧嘩する | UI の「UI操作を有効化」を OFF にする。同時に `/joint_states` を出さない |

ログの見かた。

```bash
ros2 node list                    # 動いているノード
ros2 topic list                   # トピック一覧
ros2 topic hz <topic>             # 流れているか
ros2 param list /harvest_sequencer  # パラメータの確認
```

---

## 付録: 一気に流す場合のコマンド一覧

内容を理解したうえで、2台目以降のセットアップに使うためのもの。
CAN と RealSense の確認だけは手作業で行うこと。

```bash
# --- 1. 下準備 ---
sudo apt update && sudo apt upgrade -y && sudo reboot   # 再起動後に続き

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# --- 2. ROS 2 Humble ---
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-humble-desktop ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
sudo rosdep init && rosdep update

# --- 3〜7. 依存一式 ---
sudo apt install -y python3-pip can-utils ethtool iproute2 xvfb ffmpeg python3-xlib \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
    ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-rosbridge-server ros-humble-cv-bridge ros-humble-joy \
    ros-humble-librealsense2* ros-humble-realsense2-*
pip3 install python-can scipy piper_sdk aiohttp aiortc av mss

# --- 3.2 / 8. ワークスペース ---
cd ~ && git clone -b humble https://github.com/agilexrobotics/piper_ros.git
cd ~/piper_ros/src && unzip ~/Downloads/piper_arm_ui.zip -d .
cd ~/piper_ros && colcon build --symlink-install
echo "source ~/piper_ros/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
