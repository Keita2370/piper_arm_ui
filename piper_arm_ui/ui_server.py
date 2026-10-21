#!/usr/bin/env python3
"""UI サーバノード.

1 つのプロセスで
  - Web UI (HTML/CSS/JS) の配信
  - WebRTC のシグナリング (/offer) と映像配信
  - ROS のカメラトピック購読
  - rosbridge への WebSocket 中継 (/rosbridge) と、その権限判定
をまとめて行う。

ブラウザは ROS のトピックを /rosbridge 経由で読み書きする。
UI と ROS への経路が同じ 1 つのポート (既定 8080) に集まるので、
トンネル 1 本／VPN 1 本で丸ごと外から使える。
中継のところで「手元か遠隔か」を見て権限を決める（remote_access.py）。

    ros2 run piper_arm_ui ui_server
    → ブラウザで http://<ロボットのIP>:8080/ を開く
"""

import asyncio
import json
import os
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from piper_arm_ui.common import TOPIC_DETECTIONS, TOPIC_RVIZ_IMAGE
from piper_arm_ui.remote_access import (
    DEFAULT_TRUSTED_NETWORKS,
    ControlGate,
    parse_networks,
)
from piper_arm_ui.remote_access import add_routes as add_remote_routes
from piper_arm_ui.urdf_scene import UrdfScene
from piper_arm_ui.video_source import (
    AIORTC_AVAILABLE,
    DummyVideoTrack,
    RosImageTrack,
    _compressed_to_bgr,
    _ros_image_to_bgr,
)

try:
    from aiohttp import web
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False
    web = None

if AIORTC_AVAILABLE:
    from aiortc import (
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    from aiortc.contrib.media import MediaRelay
else:
    MediaRelay = None


def _web_root():
    """インストール済み share/ の web ディレクトリ、無ければソースツリーを使う."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('piper_arm_ui'), 'web')
        if os.path.isdir(share):
            return share
    except Exception:  # noqa: BLE001
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


def _split_list(value):
    """カンマ（または空白）区切りの文字列をリストにする.

    ROS の文字列配列パラメータは launch 引数から渡すのが煩雑で、
    書き間違えても静かに空になる。ここでは素直な文字列で受けている。
    リストが直接来た場合（params_file で書いた場合）もそのまま通す。
    """
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    text = str(value).strip().strip('[]')
    return [part.strip() for part in text.replace(' ', ',').split(',') if part.strip()]


def _parse_ice_servers(values):
    """ICE サーバ指定を RTCPeerConnection に渡せる形にする.

    受け付ける書き方:
        stun:stun.l.google.com:19302
        turn:turn.example.com:3478                    （認証なし）
        turn:ユーザ:パスワード@turn.example.com:3478   （よくある形）

    ブラウザの RTCPeerConnection は user:pass@ を URL に埋める書き方を
    受け付けないので、ここで username / credential に分けておく。
    """
    servers = []
    for raw in values or []:
        text = str(raw).strip()
        if not text:
            continue
        scheme, _, rest = text.partition(':')
        scheme = scheme.lower()
        if scheme not in ('stun', 'stuns', 'turn', 'turns') or not rest:
            continue
        entry = {'urls': text}
        if '@' in rest:
            creds, _, hostpart = rest.rpartition('@')
            user, _, password = creds.partition(':')
            entry = {'urls': '%s:%s' % (scheme, hostpart),
                     'username': user, 'credential': password}
        servers.append(entry)
    return servers


class UiServer(Node):

    def __init__(self):
        super().__init__('ui_server')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        # ブラウザが繋ぎにいく rosbridge のポート。UI にそのまま伝える
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('use_dummy_video', False)
        # RealSense (realsense2_camera) のカラー画像。
        # humble の既定は /camera/camera/color/image_raw、
        # 旧構成やリマップ時は /camera/color/image_raw になることがある。
        self.declare_parameter('arm_camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('front_camera_topic', '')
        # 'raw' か 'compressed'。別PCから LAN 越しに受けるときは compressed。
        # compressed のときは <トピック名>/compressed を購読する
        # （image_transport の命名規則。ros-humble-compressed-image-transport が必要）
        self.declare_parameter('camera_transport', 'raw')
        self.declare_parameter('video_width', 640)
        self.declare_parameter('video_height', 480)
        self.declare_parameter('video_fps', 15)
        # RViz 画面（rviz_bridge が publish）
        self.declare_parameter('rviz_image_topic', TOPIC_RVIZ_IMAGE)
        # rviz_bridge の既定（960x540 @20fps）に合わせる。
        # 1280x720 は VP8 エンコードが 1 枚 30ms かかり 30fps 台で頭打ちになるため、
        # 解像度を落として fps を稼ぐほうがなめらかに見える。
        self.declare_parameter('rviz_width', 960)
        self.declare_parameter('rviz_height', 540)
        self.declare_parameter('rviz_fps', 20)

        # --- 3D ビュー（ブラウザで URDF を直接描く）---
        # RViz の画面を動画で送る代わりに、URDF とメッシュをブラウザへ渡して
        # 関節角だけをトピックで流す。遅延も解像度も段違いに良くなる。
        # 空なら piper_description の URDF を自動で探す。
        self.declare_parameter('robot_description_file', '')
        # メッシュ簡略化の格子サイズ [m]。小さいほど精細で重い。
        # 0.001 で 9.2MB→1.9MB（gzip 0.8MB）、形状誤差 0.06mm。
        self.declare_parameter('mesh_simplify_cell', 0.001)

        # --- 別ネットワークからの利用 ---
        # 「手元」とみなす送信元をカンマ区切りで。ここから来た接続だけが
        # フル操作でき、操作許可を出せる。
        # 空なら既定（LAN + VPN(Tailscale) の 100.64.0.0/10）。
        # 公開用の待ち受けポート。0 なら無効。
        # ここに届いた接続は、送信元が何であろうと **必ず遠隔扱い**。
        # トンネルをこのポートに向けておけば、ヘッダの有無を当てにしなくてよい。
        # 外に出すときはこの形を使うのが確実。
        self.declare_parameter('public_port', 0)
        self.declare_parameter('trusted_networks', '')
        # false にすると、遠隔からの操作は許可を出しても一切通らない（閲覧専用）。
        self.declare_parameter('allow_remote_control', True)
        # 1 回の許可で操作できる秒数。切り忘れても自動で閉じる。
        self.declare_parameter('control_grant_seconds', 600.0)
        # WebRTC の ICE サーバをカンマ区切りで。同一 LAN なら空で繋がるが、
        # 別ネットワークからは STUN が要る。NAT が厳しい環境では TURN も。
        #   stun:stun.l.google.com:19302
        #   turn:user:pass@turn.example.com:3478
        self.declare_parameter('ice_servers', '')

        p = self.get_parameter
        self.host = p('host').value
        self.port = int(p('port').value)
        self.rosbridge_port = int(p('rosbridge_port').value)
        self.use_dummy = bool(p('use_dummy_video').value)
        self.width = int(p('video_width').value)
        self.height = int(p('video_height').value)
        self.fps = int(p('video_fps').value)
        self.camera_transport = str(p('camera_transport').value).lower()
        self.camera_topics = {
            'arm': p('arm_camera_topic').value,
            'front': p('front_camera_topic').value,
            'rviz': p('rviz_image_topic').value,
        }

        self.public_port = int(p('public_port').value or 0)
        trusted = _split_list(p('trusted_networks').value) or DEFAULT_TRUSTED_NETWORKS
        self.trusted_networks = parse_networks(trusted)
        self.allow_remote_control = bool(p('allow_remote_control').value)
        self.control_gate = ControlGate(
            logger=self.get_logger(),
            default_seconds=float(p('control_grant_seconds').value))
        self.ice_servers = _parse_ice_servers(_split_list(p('ice_servers').value))
        if self.ice_servers:
            self.get_logger().info(
                'ICE サーバ: %s' % ', '.join(s['urls'] for s in self.ice_servers))
        self.get_logger().info(
            '手元とみなす範囲: %s / 遠隔からの操作: %s'
            % (', '.join(str(n) for n in self.trusted_networks) or '(なし)',
               '許可を出せば可' if self.allow_remote_control else '不可（閲覧専用）'))
        if self.public_port:
            self.get_logger().info(
                '公開用ポート %d を開きます。ここに届いた接続は必ず遠隔扱いです。'
                'トンネルはこちらへ向けてください' % self.public_port)

        self.latest_detections = []
        self.pcs = set()
        self.relay = MediaRelay() if MediaRelay is not None else None
        # カメラごとの ROS フレーム保持用トラック（トピックが指定されているものだけ）
        self.ros_tracks = {}

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(
            String, TOPIC_DETECTIONS, self._on_detections,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        # RViz 画面はダミー設定に関係なく、トピックがあれば必ず実映像を流す
        sources = {
            'rviz': (self.camera_topics['rviz'],
                     int(p('rviz_width').value), int(p('rviz_height').value),
                     int(p('rviz_fps').value), 'RVIZ'),
        }
        if not self.use_dummy:
            sources['arm'] = (self.camera_topics['arm'],
                              self.width, self.height, self.fps, 'ARM CAM (RealSense)')
            sources['front'] = (self.camera_topics['front'],
                                self.width, self.height, self.fps, 'FRONT CAM')

        compressed = self.camera_transport == 'compressed'
        for key, (topic, w, h, fps, label) in sources.items():
            if not topic:
                self.get_logger().warn(
                    'camera[%s] はトピック未設定のためテストパターンになります' % key)
                continue
            track = RosImageTrack(w, h, fps, label=label)
            self.ros_tracks[key] = track
            # RViz 画面は同じPC上の rviz_bridge から来るので常に raw
            if compressed and key != 'rviz':
                self.create_subscription(
                    CompressedImage, topic + '/compressed',
                    (lambda t: (lambda msg: self._on_image(t, msg, True)))(track), qos)
                self.get_logger().info(
                    'camera[%s] <- %s/compressed (%dx%d)' % (key, topic, w, h))
            else:
                self.create_subscription(
                    Image, topic,
                    (lambda t: (lambda msg: self._on_image(t, msg, False)))(track), qos)
                self.get_logger().info('camera[%s] <- %s (%dx%d)' % (key, topic, w, h))

        if self.use_dummy:
            self.get_logger().warn(
                'use_dummy_video=true のためカメラ映像はテストパターンです '
                '（RealSense 接続時は use_dummy_video:=false）')

        self.web_root = _web_root()
        self.get_logger().info('web root: %s' % self.web_root)
        self._check_web_assets()
        self.robot_scene = self._load_robot_scene(
            str(p('robot_description_file').value or ''),
            float(p('mesh_simplify_cell').value))
        if not AIORTC_AVAILABLE:
            self.get_logger().error(
                'aiortc が未インストールです: pip install aiortc aiohttp av')
        if not AIOHTTP_AVAILABLE:
            self.get_logger().error('aiohttp が未インストールです: pip install aiohttp')

    # ------------------------------------------------------------------ #
    def _find_description(self):
        """piper_description の URDF を自動で探す."""
        try:
            from ament_index_python.packages import get_package_share_directory
            urdf_dir = os.path.join(
                get_package_share_directory('piper_description'), 'urdf')
        except Exception:  # noqa: BLE001
            return ''
        for name in ('piper_description.urdf', 'piper_description_old.urdf'):
            path = os.path.join(urdf_dir, name)
            if os.path.isfile(path):
                return path
        return ''

    def _load_robot_scene(self, path, cell):
        """URDF とメッシュをブラウザ用に変換して持っておく.

        失敗しても UI 全体は動かす（3D ビューだけ出せなくなる）。
        """
        path = path.strip() or self._find_description()
        if not path:
            self.get_logger().warn(
                'piper_description が見つからないため 3D ビューは使えません。'
                'RViz 映像のほうに切り替えてください')
            return None
        if not os.path.isfile(path):
            self.get_logger().warn('ロボット記述が見つかりません: %s' % path)
            return None
        # package://piper_description/... は通常 ament index で解決するが、
        # ソースツリーから直接動かす場合に備えて URDF の置き場所からも探す。
        #   <root>/piper_description/urdf/x.urdf → <root> を候補に加える
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(path)))
        extra_roots = [os.path.dirname(pkg_dir), pkg_dir]
        try:
            scene = UrdfScene(path, cell=cell, extra_roots=extra_roots,
                              logger=self.get_logger())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('URDF を読み込めませんでした (%s): %s' % (path, exc))
            return None
        if not scene.meshes:
            self.get_logger().warn(
                'メッシュを 1 つも読めませんでした。piper_description の '
                'meshes/ が入っているか確認してください')
        return scene

    # ------------------------------------------------------------------ #
    def _check_web_assets(self):
        """CSS / JS が本当に読めるか起動時に確かめる.

        入れ忘れると「素の HTML だけ表示される」という分かりにくい壊れ方をする
        ので、ブラウザを開く前に気付けるようにログへ出す。
        """
        required = ['index.html', 'css/style.css', 'js/rosbridge.js',
                    'js/webrtc.js', 'js/armview.js', 'js/robotview.js',
                    'js/app.js', 'js/vendor/three.min.js']
        missing = [rel for rel in required
                   if not os.path.isfile(os.path.join(self.web_root, *rel.split('/')))]
        if missing:
            self.get_logger().error(
                'UI の静的ファイルが足りません: %s\n'
                '    web root: %s\n'
                'このまま開くと装飾のない素の HTML になります。'
                'colcon build --packages-select piper_arm_ui --symlink-install '
                'をやり直してください。'
                % (', '.join(missing), self.web_root))

    # ------------------------------------------------------------------ #
    def _on_detections(self, msg: String):
        try:
            self.latest_detections = json.loads(msg.data).get('targets', [])
        except json.JSONDecodeError:
            pass

    def _on_image(self, track, msg, compressed=False):
        try:
            track.set_frame(
                _compressed_to_bgr(msg) if compressed else _ros_image_to_bgr(msg))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('画像変換に失敗: %s' % exc, throttle_duration_sec=5.0)

    def make_track(self, camera):
        """カメラ名から映像トラックを作る.

        ROS 由来のトラックは MediaRelay 経由で配る。こうしないと
        タブレットとPCなど複数ブラウザが繋がったときにフレームを奪い合う。
        """
        track = self.ros_tracks.get(camera)
        if track is not None:
            return self.relay.subscribe(track) if self.relay else track
        labels = {'arm': 'ARM CAM', 'front': 'FRONT CAM', 'rviz': 'RVIZ (未起動)'}
        return DummyVideoTrack(
            self.width, self.height, self.fps,
            label=labels.get(camera, camera.upper()),
            detections_provider=(lambda: self.latest_detections)
            if camera == 'arm' else None)


# ---------------------------------------------------------------------- #
# HTTP / WebRTC
# ---------------------------------------------------------------------- #
async def _index(request):
    node = request.app['node']
    path = os.path.join(node.web_root, 'index.html')
    if not os.path.isfile(path):
        return web.Response(status=500, text='index.html が見つかりません: %s' % path)
    return web.FileResponse(path)


async def _info(request):
    node = request.app['node']
    from piper_arm_ui.remote_access import peer_is_trusted
    trusted = peer_is_trusted(request)
    return web.json_response({
        'use_dummy_video': node.use_dummy,
        'camera_topics': node.camera_topics,
        'camera_transport': node.camera_transport,
        'live_sources': sorted(node.ros_tracks.keys()),
        'video': {'width': node.width, 'height': node.height, 'fps': node.fps},
        'rosbridge_port': node.rosbridge_port,
        # ブラウザはここへ繋ぐ。ui_server と同じポートなので、
        # トンネル 1 本でも https でも、そのまま通る
        'rosbridge_path': '/rosbridge',
        'webrtc_available': AIORTC_AVAILABLE,
        'robot_view_available': node.robot_scene is not None,
        'ice_servers': node.ice_servers,
        # この接続が手元か遠隔か。UI の表示を切り替えるためのもので、
        # 権限そのものはサーバ側（/rosbridge の中継）で強制している
        'trusted': trusted,
        'allow_remote_control': node.allow_remote_control,
    })


async def _robot(request):
    """URDF から作ったシーン定義（リンク階層・関節・メッシュの参照）を返す."""
    node = request.app['node']
    scene = node.robot_scene
    if scene is None:
        return web.json_response(
            {'available': False,
             'reason': 'piper_description が見つからないため 3D ビューは使えません'},
            status=200)
    payload = dict(scene.scene)
    payload['available'] = True
    payload['mesh_url'] = '/api/mesh/'
    return web.json_response(payload)


async def _mesh(request):
    """リンクの形状を、そのまま TypedArray にできるバイナリで返す."""
    node = request.app['node']
    scene = node.robot_scene
    mesh_id = request.match_info.get('mesh_id', '')
    data = scene.mesh_bytes(mesh_id) if scene is not None else None
    if data is None:
        return web.Response(status=404, text='mesh not found: %s' % mesh_id)
    # 内容は起動中変わらないので、ブラウザに強くキャッシュさせる
    return web.Response(
        body=data,
        content_type='application/octet-stream',
        headers={'Cache-Control': 'public, max-age=86400',
                 'ETag': '"%s"' % mesh_id})


def _rtc_config(node):
    """サーバ側の RTCPeerConnection 設定（ICE サーバ）を作る.

    同一 LAN なら不要だが、別ネットワークからのときは
    サーバ側も自分の公開アドレスを知る必要があるので STUN を使う。
    """
    if not node.ice_servers or not AIORTC_AVAILABLE:
        return None
    servers = []
    for s in node.ice_servers:
        servers.append(RTCIceServer(
            urls=s['urls'],
            username=s.get('username'),
            credential=s.get('credential')))
    return RTCConfiguration(iceServers=servers)


async def _offer(request):
    """ブラウザからの SDP offer を受けて、要求されたカメラの映像を返す."""
    node = request.app['node']
    if not AIORTC_AVAILABLE:
        return web.json_response({'error': 'aiortc unavailable'}, status=503)

    params = await request.json()
    cameras = params.get('cameras') or ['arm']
    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

    pc = RTCPeerConnection(_rtc_config(node))
    node.pcs.add(pc)

    @pc.on('connectionstatechange')
    async def on_state_change():
        node.get_logger().info('WebRTC connection state: %s' % pc.connectionState)
        if pc.connectionState in ('failed', 'closed'):
            await pc.close()
            node.pcs.discard(pc)

    sender_map = {}
    for cam in cameras:
        track = node.make_track(cam)
        sender = pc.addTrack(track)
        sender_map[cam] = sender

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    # どの mid がどのカメラかをブラウザへ伝える
    mids = {}
    for cam, sender in sender_map.items():
        for transceiver in pc.getTransceivers():
            if transceiver.sender is sender:
                mids[cam] = transceiver.mid
                break

    return web.json_response({
        'sdp': pc.localDescription.sdp,
        'type': pc.localDescription.type,
        'mids': mids,
    })


async def _on_shutdown(app):
    node = app['node']
    await asyncio.gather(*[pc.close() for pc in list(node.pcs)], return_exceptions=True)
    node.pcs.clear()


def build_app(node):
    app = web.Application()
    app['node'] = node
    app.router.add_get('/', _index)
    app.router.add_get('/api/info', _info)
    app.router.add_get('/api/robot', _robot)
    app.router.add_get('/api/mesh/{mesh_id}', _mesh)
    app.router.add_post('/offer', _offer)
    # /rosbridge（中継）と /api/control/*（操作許可）
    add_remote_routes(app)
    # follow_symlinks=True は必須。
    # colcon build --symlink-install で入れると share/piper_arm_ui/web/ の中身が
    # ソースツリーへのシンボリックリンクになる。aiohttp の既定 (False) は
    # 静的ルートの外を指すシンボリックリンクを 404 にするため、index.html だけ
    # 表示できて CSS と JS が全部 404 になる（＝素の HTML が出る）。
    app.router.add_static('/static/', path=node.web_root, name='static',
                          follow_symlinks=True)
    app.on_shutdown.append(_on_shutdown)
    return app


def _port_in_use(host, port):
    """待ち受けようとしているポートが既に使われていないか確かめる."""
    probe_host = '127.0.0.1' if host in ('0.0.0.0', '') else host
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.5)
        return sock.connect_ex((probe_host, port)) == 0


def main(args=None):
    rclpy.init(args=args)
    node = UiServer()

    for label, port in (('port', node.port),
                        ('public_port', node.public_port)):
        if port and _port_in_use(node.host, port):
            node.get_logger().fatal(
                'ポート %d は既に使われています。前回の起動が残っていないか確認してください:\n'
                "    ss -ltnp | grep ':%d'\n"
                '    pkill -f ui_server\n'
                'ほかのプログラムが使っているなら %s:=%d のようにずらしてください。'
                % (port, port, label, port + 1))
            node.destroy_node()
            rclpy.shutdown()
            return

    if not AIOHTTP_AVAILABLE:
        node.get_logger().fatal('aiohttp が無いため UI サーバを起動できません')
        node.destroy_node()
        rclpy.shutdown()
        return

    # ROS のスピンは別スレッドで回し、メインスレッドは asyncio に渡す
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    shown = '<robot-ip>' if node.host == '0.0.0.0' else node.host
    node.get_logger().info(
        'UI: http://%s:%d/  (ブラウザで開いてください)' % (shown, node.port))
    if node.public_port:
        node.get_logger().info(
            '公開用: http://%s:%d/  ← トンネルはこちらへ向ける（必ず閲覧のみで始まる）'
            % (shown, node.public_port))

    try:
        asyncio.run(_serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


async def _serve(node):
    """本来のポートと、（設定されていれば）公開用ポートの両方で待ち受ける.

    同じアプリを 2 つの入口で出す。公開用ポートに届いた接続は
    送信元に関わらず遠隔扱いになるので、トンネルはそちらへ向ければよい。
    """
    runner = web.AppRunner(build_app(node))
    await runner.setup()
    await web.TCPSite(runner, node.host, node.port).start()
    if node.public_port:
        await web.TCPSite(runner, node.host, node.public_port).start()
    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        await runner.cleanup()


if __name__ == '__main__':
    main()
