"""piper_arm_ui 一式の起動.

実機（ミニPC）:

    ros2 launch piper_arm_ui arm_ui.launch.py
    ros2 launch piper_arm_ui arm_ui.launch.py start_realsense:=true

実機なし（開発用PC）— アームもカメラも無い状態で UI を一通り動かす:

    ros2 launch piper_arm_ui arm_ui.launch.py \\
        use_dummy_video:=true enable_arm_command:=true

分散構成（ミニPCは本家 piper_ros のみ、UI はこのPC）:

    ros2 launch piper_arm_ui arm_ui.launch.py \\
        camera_transport:=compressed

ブラウザで http://<ロボットのIP>:8080/ を開く。
アームを実際に動かす場合は UI 上の「UI操作を有効化」を ON にするか、
enable_arm_command:=true で起動する（既存の joy ノードとの競合に注意）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_rviz_config(pkg_share):
    """RViz の設定ファイルを決める.

    既定は piper_ros に元から入っている piper_description/rviz/piper_ctrl.rviz。
    普段 start_single_piper_rviz.launch.py で見ているものと同じ画面になる。
    piper_description が見つからないときだけ、同梱の harvest.rviz を使う。

    別の設定を使いたいときは rviz_config:=<パス> で明示的に渡す。例:
      # MoveIt の設定
      rviz_config:=$(ros2 pkg prefix piper_with_gripper_moveit)/share/\\
                   piper_with_gripper_moveit/config/moveit.rviz
      # 同梱の収穫デモ向け設定（点群・マーカー入り）
      rviz_config:=$(ros2 pkg prefix piper_arm_ui)/share/piper_arm_ui/config/harvest.rviz
    """
    try:
        stock = os.path.join(get_package_share_directory('piper_description'),
                             'rviz', 'piper_ctrl.rviz')
        if os.path.isfile(stock):
            return stock
    except Exception:  # noqa: BLE001
        pass
    return os.path.join(pkg_share, 'config', 'harvest.rviz')


def _default_description_file():
    """RViz に出すロボット記述ファイルを決める.

    既定は素の URDF（piper_description.urdf）。xacro ではない理由:
    piper_description.xacro は gazebo 用の記述を含んでいて、その中で
    `$(find piper_gazebo)/config/ros2_controllers.yaml` を参照している。
    そのため xacro を展開するには piper_gazebo までビルドしておく必要があり、
    RViz に表示したいだけの開発用PCでは無駄な依存になる。
    素の URDF は package://piper_description のメッシュしか参照しないので、
    piper_description だけビルドしてあれば足りる。

    ファームウェアが S-V1.6-3 より古い場合は
    robot_description_file:=<...>/piper_description_old.urdf を指定する。
    """
    try:
        urdf_dir = os.path.join(
            get_package_share_directory('piper_description'), 'urdf')
    except Exception:  # noqa: BLE001
        return ''
    for name in ('piper_description.urdf', 'piper_description.xacro'):
        path = os.path.join(urdf_dir, name)
        if os.path.isfile(path):
            return path
    return ''


def _robot_description(path):
    """URDF / xacro からロボット記述の文字列を作る.

    .xacro は xacro コマンドで展開する（xacro と piper_gazebo が必要）。
    それ以外はそのまま読む（追加の依存なし）。
    """
    if path.endswith('.xacro'):
        return ParameterValue(Command(['xacro ', path]), value_type=str)
    with open(path) as f:
        return f.read()


def _robot_state_publisher(context, *args, **kwargs):
    """robot_state_publisher を組み立てる（引数の値を解決してから読むため）."""
    enabled = LaunchConfiguration('start_robot_state_publisher').perform(context)
    if enabled.strip().lower() not in ('true', '1', 'yes'):
        return []

    path = LaunchConfiguration('robot_description_file').perform(context).strip()
    if not path:
        print('[piper_arm_ui] piper_description が見つからないため '
              'robot_state_publisher を起動しません（RViz にモデルは出ません）')
        return []
    if not os.path.isfile(path):
        print('[piper_arm_ui] ロボット記述が見つかりません: %s' % path)
        return []

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': _robot_description(path)}],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('piper_arm_ui')
    default_params = os.path.join(pkg_share, 'config', 'arm_ui.yaml')
    default_rviz = _find_rviz_config(pkg_share)

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='パラメータ YAML'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='Web UI のポート'),

        # --- カメラ (RealSense) ---
        DeclareLaunchArgument('start_realsense', default_value='false',
                              description='realsense2_camera を同時起動するか'),
        DeclareLaunchArgument(
            'arm_camera_topic', default_value='/camera/camera/color/image_raw',
            description='RealSense カラー画像トピック'),
        DeclareLaunchArgument('front_camera_topic', default_value='',
                              description='フロントカメラ（空ならテストパターン）'),
        DeclareLaunchArgument('use_dummy_video', default_value='false',
                              description='カメラ映像をテストパターンにするか'),
        DeclareLaunchArgument(
            'camera_transport', default_value='raw',
            description="raw か compressed。UI を別PCで動かして LAN 越しに "
                        "映像を受けるときは compressed"),

        # --- RViz 埋め込み ---
        DeclareLaunchArgument(
            'start_rviz_bridge', default_value='false',
            description='RViz 映像のブリッジを起動するか。'
                        '既定の 3D ビュー（ブラウザで URDF を直接描画）で足りるので'
                        '通常は false。TF や点群を RViz で見たいときだけ true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='rviz2 に渡す設定ファイル'
                        '（既定: piper_description/rviz/piper_ctrl.rviz）'),
        DeclareLaunchArgument('rviz_display', default_value=':99',
                              description='rviz2 を出す X ディスプレイ'),
        DeclareLaunchArgument('use_xvfb', default_value='true',
                              description='仮想ディスプレイを立てるか（実機の :0 を映すなら false）'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='ブリッジが rviz2 を起動するか（既に起動済みなら false）'),
        DeclareLaunchArgument('hide_docks', default_value='true',
                              description='RViz の左右ドックを畳んで3Dビューを広く使うか'),
        DeclareLaunchArgument(
            'hide_chrome', default_value='true',
            description='RViz のメニューバー・ツールバー・ステータスバーを消して'
                        'モデルだけを映すか（rviz2 を F11 で全画面にする）'),
        DeclareLaunchArgument(
            'view_yaw', default_value='3.14159',
            description='RViz の初期視点の方位 [rad]。'
                        '3.14159=後ろから（既定）、0=正面から、'
                        '1.5708=左から、-1.5708=右から'),
        DeclareLaunchArgument(
            'view_pitch', default_value='0.25',
            description='RViz の初期視点の仰角 [rad]。0=真横、大きいほど見下ろす'),
        DeclareLaunchArgument(
            'view_distance', default_value='1.2',
            description='RViz の初期視点のカメラ距離 [m]'),
        DeclareLaunchArgument(
            'rviz_fps', default_value='20',
            description='RViz 配信のフレームレート。'
                        'カクつくときは解像度を下げてこちらを保つ'),
        DeclareLaunchArgument(
            'rviz_width', default_value='960',
            description='RViz 配信の横幅 [px]。重い機体では 848 や 640 に下げる'),
        DeclareLaunchArgument(
            'rviz_height', default_value='540',
            description='RViz 配信の高さ [px]。480 / 360 など 16:9 で下げる'),
        DeclareLaunchArgument(
            'mesh_simplify_cell', default_value='0.001',
            description='3D ビューのメッシュ簡略化の格子 [m]。'
                        '大きいほど粗く軽い（0.003 で約 0.8MB、形状誤差 1mm）'),
        DeclareLaunchArgument(
            'allow_rviz_input', default_value='true',
            description='ブラウザからの RViz 視点操作を許可するか。'
                        'rviz_display:=:0 で自分の画面を映すときは false にする'
                        '（実マウスが動いてしまうため）'),

        # --- RViz にアームのモデルを出すため ---
        DeclareLaunchArgument(
            'robot_description_file', default_value=_default_description_file(),
            description='RViz に出すロボット記述。既定は piper_description.urdf。'
                        'ファームが S-V1.6-3 より古いなら '
                        'piper_description_old.urdf を指定する'),
        DeclareLaunchArgument(
            'start_robot_state_publisher', default_value='true',
            description='robot_state_publisher を起動して /robot_description と TF を出す。'
                        'これが無いと RViz にアームのモデルが出ない。'
                        'display_xacro.launch.py などを別途動かしている場合のみ false'),

        # --- その他 ---
        DeclareLaunchArgument('enable_arm_command', default_value='false',
                              description='UI から /joint_states を publish するか'),
        DeclareLaunchArgument('use_dummy_perception', default_value='true',
                              description='レモン検出のダミーを起動するか'),
        DeclareLaunchArgument('use_lemon_detections', default_value='false',
                              description='チームの検出（/lemon_ellipse/lemons）を画面に出すか。'
                                          'true ならダミーの検出は起動しない'),
        DeclareLaunchArgument('start_rosbridge', default_value='true',
                              description='rosbridge_server を同時起動するか'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge_server の WebSocket ポート。'
                                          '別プロセスが 9090 を使用中なら 9091 などに変更する。'
                                          'ブラウザは ui_server の /rosbridge 経由で繋ぐので'
                                          '直接この番号を叩くことはない'),
        # rosbridge は既定で 127.0.0.1（同じ PC の中）だけに開く。
        # ブラウザは ui_server の /rosbridge を通るので、これで困らない。
        # rosbridge を外に開くと、権限判定を素通りして誰でも実機を動かせてしまう。
        DeclareLaunchArgument('rosbridge_address', default_value='127.0.0.1',
                              description='rosbridge_server の待ち受けアドレス。'
                                          '既定はローカル閉じ。古い構成に戻すなら 0.0.0.0'),

        # --- 別ネットワークからの利用 ---
        DeclareLaunchArgument('public_port', default_value='0',
                              description='公開用の待ち受けポート。0 で無効。'
                                          'ここに届いた接続は送信元に関わらず'
                                          '必ず遠隔（＝既定は閲覧のみ）扱いになる。'
                                          'トンネルはこのポートへ向けるのが確実'),
        DeclareLaunchArgument('trusted_networks', default_value='',
                              description='「手元」とみなす送信元をカンマ区切りで。'
                                          '空なら既定（LAN と VPN の 100.64.0.0/10）。'
                                          'ここからの接続だけがフル操作でき、'
                                          '遠隔に操作許可を出せる'),
        DeclareLaunchArgument('allow_remote_control', default_value='true',
                              description='false にすると遠隔からは一切操作できない'
                                          '（非常停止を除く完全な閲覧専用）'),
        DeclareLaunchArgument('control_grant_seconds', default_value='600.0',
                              description='1 回の許可で遠隔が操作できる秒数。'
                                          '切り忘れてもこの時間で自動的に閉じる'),
        DeclareLaunchArgument('ice_servers', default_value='',
                              description='WebRTC の STUN/TURN をカンマ区切りで。'
                                          '別ネットワークから映像を見るときに必要。'
                                          '例: stun:stun.l.google.com:19302'),
    ]

    params_file = LaunchConfiguration('params_file')

    ui_server = Node(
        package='piper_arm_ui',
        executable='ui_server',
        name='ui_server',
        output='screen',
        # 実機（ミニPC）で映像のエンコード中に SIGSEGV で落ちたことがある（原因は
        # aiortc / PyAV の内部と思われ、未特定）。落ちると画面が止まったままになるので、
        # 自動で起動し直す。ui_server はアームへの指令を出さないので、起動し直しても
        # アームの動きには影響しない
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'rosbridge_port': ParameterValue(
                LaunchConfiguration('rosbridge_port'), value_type=int),
            'use_dummy_video': LaunchConfiguration('use_dummy_video'),
            'arm_camera_topic': LaunchConfiguration('arm_camera_topic'),
            'front_camera_topic': LaunchConfiguration('front_camera_topic'),
            'camera_transport': LaunchConfiguration('camera_transport'),
            # RViz の映像をブラウザへ送るか（送らないなら待機画面をエンコードしない）
            'rviz_enabled': LaunchConfiguration('start_rviz_bridge'),
            # rviz_bridge と必ず同じ値にする。ずれると ui_server 側で
            # リサイズが入って画質と速度をどちらも損なう。
            'rviz_fps': ParameterValue(LaunchConfiguration('rviz_fps'), value_type=int),
            'rviz_width': ParameterValue(
                LaunchConfiguration('rviz_width'), value_type=int),
            'rviz_height': ParameterValue(
                LaunchConfiguration('rviz_height'), value_type=int),
            # 3D ビュー用。空なら piper_description から自動で見つける
            'robot_description_file':
                LaunchConfiguration('robot_description_file'),
            'mesh_simplify_cell': ParameterValue(
                LaunchConfiguration('mesh_simplify_cell'), value_type=float),
            # 別ネットワークからの利用と権限。
            # リストではなくカンマ区切りの文字列にしてある。ROS の文字列配列を
            # launch 引数で渡すのは書き方が煩雑で、間違えても静かに空になるため。
            'public_port': ParameterValue(
                LaunchConfiguration('public_port'), value_type=int),
            'trusted_networks': ParameterValue(
                LaunchConfiguration('trusted_networks'), value_type=str),
            'allow_remote_control': ParameterValue(
                LaunchConfiguration('allow_remote_control'), value_type=bool),
            'control_grant_seconds': ParameterValue(
                LaunchConfiguration('control_grant_seconds'), value_type=float),
            'ice_servers': ParameterValue(
                LaunchConfiguration('ice_servers'), value_type=str),
        }],
    )

    rviz_bridge = Node(
        package='piper_arm_ui',
        executable='rviz_bridge',
        name='rviz_bridge',
        output='screen',
        parameters=[params_file, {
            'rviz_config': LaunchConfiguration('rviz_config'),
            'display': LaunchConfiguration('rviz_display'),
            'use_xvfb': LaunchConfiguration('use_xvfb'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'hide_docks': LaunchConfiguration('hide_docks'),
            'hide_chrome': LaunchConfiguration('hide_chrome'),
            'allow_input': LaunchConfiguration('allow_rviz_input'),
            'view_yaw': ParameterValue(
                LaunchConfiguration('view_yaw'), value_type=float),
            'view_pitch': ParameterValue(
                LaunchConfiguration('view_pitch'), value_type=float),
            'view_distance': ParameterValue(
                LaunchConfiguration('view_distance'), value_type=float),
            'fps': ParameterValue(LaunchConfiguration('rviz_fps'), value_type=float),
            'width': ParameterValue(LaunchConfiguration('rviz_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('rviz_height'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('start_rviz_bridge')),
    )

    sequencer = Node(
        package='piper_arm_ui',
        executable='harvest_sequencer',
        name='harvest_sequencer',
        output='screen',
        parameters=[params_file, {
            'enable_arm_command': LaunchConfiguration('enable_arm_command'),
            # 座標指定（手先位置）の IK に使う。ui_server と
            # robot_state_publisher と同じ URDF を渡して、
            # 3 者の手先位置が食い違わないようにする。
            # tcp_offset は実測して決める校正値なので launch 引数にはせず、
            # config/arm_ui.yaml に書く。
            'robot_description_file':
                LaunchConfiguration('robot_description_file'),
        }],
    )

    # 本物の検出（チームの lemon_ellipse）を使うときはダミーを止める。
    # 両方が /arm_ui/detections に出すと、検出枠が交互に入れ替わってちらつく
    perception = Node(
        package='piper_arm_ui',
        executable='dummy_perception',
        name='dummy_perception',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_dummy_perception'), "' == 'true' and '",
            LaunchConfiguration('use_lemon_detections'), "' != 'true'"])),
    )

    # 検出アダプタ: /lemon_ellipse/lemons → /arm_ui/detections（段階2。読むだけ）
    lemon_detections = Node(
        package='piper_arm_ui',
        executable='lemon_detections',
        name='lemon_detections',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_lemon_detections')),
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('rosbridge_port'),
                                   value_type=int),
            # 既定は 127.0.0.1。ブラウザは ui_server の /rosbridge を通るので
            # rosbridge 自体を外に出す必要がなく、出すと権限判定を素通りされる。
            'address': ParameterValue(LaunchConfiguration('rosbridge_address'),
                                      value_type=str),
        }],
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
    )

    entries = [ui_server, rviz_bridge, sequencer, perception, lemon_detections, rosbridge]

    # RViz にアームを描くために robot_state_publisher を起動する。
    # piper の start_single_piper.launch.py は piper のノードしか起動せず、
    # /robot_description を出さないので、実機でもこれが要る。
    # joint_state_publisher(_gui) は起動しない（あれが /joint_states を出すと
    # UI やゲームパッドの指令と競合するため）。関節角は実機のフィードバック、
    # または harvest_sequencer の指令がそのまま使われる。
    # robot_description_file は起動時にしか値が決まらないので OpaqueFunction で解決する
    entries.append(OpaqueFunction(function=_robot_state_publisher))

    # realsense2_camera は別パッケージ。入っていない環境でも launch 自体は通るように
    # ここで存在確認してから include する。
    try:
        rs_launch = os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    except Exception:  # noqa: BLE001
        rs_launch = None

    if rs_launch and os.path.isfile(rs_launch):
        entries.insert(0, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'rgb_camera.color_profile': '640,480,30',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_realsense')),
        ))

    return LaunchDescription(args + entries)
