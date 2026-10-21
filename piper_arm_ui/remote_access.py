#!/usr/bin/env python3
"""別ネットワークからの UI 利用と、その権限制御.

【なぜ ui_server が rosbridge の前に立つか】
この UI には認証が無く、rosbridge をそのまま公開すると、
ブラウザの devtools から /arm_ui/command に publish するだけで
誰でも実機のアームを動かせてしまう。画面のボタンを隠しても意味がない
（通信路が開いている限り送れる）。

そこで rosbridge は 127.0.0.1 に閉じ、ブラウザは ui_server の
/rosbridge を経由する。ここで 1 箇所だけ権限を判定する:

    手元（LAN / VPN）から   → 素通し。従来どおりフル操作
    遠隔（トンネル経由）から → subscribe だけ通し、publish は落とす
                              手元で操作を許可している間だけ publish も通す

「落とす」のはサーバ側なので、遠隔のクライアントが何を送ろうと
既定ではコマンドが ROS に届かない。
"""

import asyncio
import ipaddress
import json
import time

try:
    import aiohttp
    from aiohttp import WSMsgType, web
    AIOHTTP_AVAILABLE = True
except ImportError:  # pragma: no cover - 実行環境依存
    AIOHTTP_AVAILABLE = False
    aiohttp = None
    web = None
    WSMsgType = None

# 「手元」とみなす送信元。VPN(Tailscale) の 100.64.0.0/10 を含める。
DEFAULT_TRUSTED_NETWORKS = [
    '127.0.0.0/8',
    '::1/128',
    '10.0.0.0/8',
    '172.16.0.0/12',
    '192.168.0.0/16',
    'fc00::/7',        # ユニークローカル
    '100.64.0.0/10',   # CGNAT。Tailscale / WireGuard の割り当てがここ
]

# 遠隔から publish を許すトピック。閲覧に必要なものだけ。
# 操作系（/arm_ui/command, /dog_ui/mode）は許可が出るまで通さない。
VIEWER_ALLOWED_PUBLISH = set()

# 遠隔でも常に許してよいトピック（RViz の視点操作。実機は動かない）
VIEWER_ALLOWED_ALWAYS = {'/rviz_ui/input'}

# 非常停止だけは閲覧者にも許す。
# 「危ないと思った人が止められない」ほうが危険で、止める操作は
# アームをより安全な側にしか動かさない。トピック名だけでは区別できないので
# 中身まで見て、この形のときだけ通す。
EMERGENCY_PAYLOADS = {
    '/arm_ui/command': ({'cmd': 'abort'},),
    '/dog_ui/mode': ('stop',),
}


def _is_emergency(topic, msg):
    """この publish が非常停止そのものか（余計なものが混ざっていないか）."""
    allowed = EMERGENCY_PAYLOADS.get(topic)
    if not allowed or not isinstance(msg, dict):
        return False
    data = msg.get('data')
    if not isinstance(data, str):
        return False
    for want in allowed:
        if isinstance(want, str):
            if data.strip() == want:
                return True
            continue
        try:
            got = json.loads(data)
        except ValueError:
            continue
        if got == want:      # キーが 1 つでも増えていたら通さない
            return True
    return False


def parse_networks(values):
    """文字列のリストを ip_network のリストにする（不正な指定は捨てる）."""
    nets = []
    for text in values or []:
        text = str(text).strip()
        if not text:
            continue
        try:
            nets.append(ipaddress.ip_network(text, strict=False))
        except ValueError:
            continue
    return nets


PROXY_HEADERS = ('X-Forwarded-For', 'X-Real-IP', 'CF-Connecting-IP', 'Forwarded')


def local_port(request):
    """この接続がロボット側のどのポートに届いたか（分からなければ None）."""
    transport = getattr(request, 'transport', None)
    if transport is None:
        return None
    sockname = transport.get_extra_info('sockname')
    if isinstance(sockname, (tuple, list)) and len(sockname) >= 2:
        try:
            return int(sockname[1])
        except (TypeError, ValueError):
            return None
    return None


def is_trusted_peer(request, networks, public_port=0):
    """この接続を「手元」とみなしてよいか.

    判定は 3 段。上から順に、確実なものを先に見る。

    1. 公開用ポート (public_port) に届いた接続 → **必ず遠隔**。
       トンネルをこのポートに向けておけば、あとは何も当てにしなくてよい。
       これが一番確実なので、外に出すときはこの形を使う。
    2. X-Forwarded-For などが付いていたら → 遠隔。
       トンネルはロボット上の localhost から繋いでくるので、
       送信元アドレスだけ見ると手元に見えてしまう。ヘッダで気づく。
       ただし **これは当てにしすぎない**。ヘッダを付けない中継や、
       WebSocket の upgrade だけヘッダが落ちる実装があり得る。
    3. 残りは送信元アドレスが信頼レンジに入っているかで判定。
    """
    if public_port and local_port(request) == int(public_port):
        return False
    for header in PROXY_HEADERS:
        if request.headers.get(header):
            return False
    peer = request.remote
    if not peer:
        return False
    try:
        addr = ipaddress.ip_address(peer)
    except ValueError:
        return False
    return any(addr in net for net in networks)


def peer_is_trusted(request):
    """ノードの設定を使って判定する（各ハンドラはこれを呼ぶ）."""
    node = request.app['node']
    return is_trusted_peer(request, node.trusted_networks,
                           getattr(node, 'public_port', 0))


class ControlGate:
    """遠隔からの操作を許すかどうかの状態を持つ.

    許可は「手元」からしか出せない。時間で自動的に切れるので、
    切り忘れてもいつまでも開きっぱなしにはならない。
    """

    def __init__(self, logger=None, default_seconds=600.0):
        self.logger = logger
        self.default_seconds = float(default_seconds)
        self._until = 0.0
        self._granted_to = ''
        self._requests = {}      # 要求元 -> 最後に要求した時刻

    # ------------------------------------------------------------------ #
    @property
    def granted(self):
        return time.monotonic() < self._until

    @property
    def remaining(self):
        return max(0.0, self._until - time.monotonic())

    def grant(self, seconds=None, who=''):
        seconds = self.default_seconds if seconds is None else float(seconds)
        seconds = max(0.0, min(3600.0, seconds))
        self._until = time.monotonic() + seconds
        self._granted_to = who
        if self.logger:
            self.logger.warn(
                '遠隔からの操作を %.0f 秒間 許可しました（%s）。'
                '実機が外部から動かせる状態です' % (seconds, who or '遠隔'))
        return seconds

    def revoke(self):
        was = self.granted
        self._until = 0.0
        self._granted_to = ''
        if was and self.logger:
            self.logger.info('遠隔からの操作許可を取り消しました')

    def note_request(self, who):
        """遠隔からの「操作したい」を記録する（手元の画面に出すため）."""
        self._requests[who or '遠隔'] = time.monotonic()

    def pending_requests(self, within=120.0):
        now = time.monotonic()
        return sorted(k for k, t in self._requests.items() if now - t < within)

    def status(self):
        return {
            'granted': self.granted,
            'remaining': round(self.remaining, 1),
            'granted_to': self._granted_to,
            'pending': self.pending_requests(),
        }


def _is_publish_allowed(op, gate):
    """遠隔クライアントからのこの op を通してよいか."""
    topic = op.get('topic') or op.get('service') or ''
    if topic in VIEWER_ALLOWED_ALWAYS or topic in VIEWER_ALLOWED_PUBLISH:
        return True
    if gate.granted:
        return True
    # advertise は publish の下ごしらえなので、非常停止のために通しておく。
    # 実際に何が流れるかは publish のときに中身で判定する。
    if op.get('op') == 'advertise':
        return topic in EMERGENCY_PAYLOADS
    return _is_emergency(topic, op.get('msg'))


async def rosbridge_proxy(request):
    """/rosbridge — ブラウザと rosbridge の間に入る WebSocket プロキシ.

    手元からの接続は素通し。遠隔からの接続は publish / advertise / call_service を
    許可が出るまで落とす（subscribe と受信は通すので閲覧はできる）。
    """
    node = request.app['node']
    gate = node.control_gate
    trusted = peer_is_trusted(request)

    ws_client = web.WebSocketResponse(heartbeat=30)
    await ws_client.prepare(request)

    who = request.headers.get('CF-Connecting-IP') or \
        request.headers.get('X-Forwarded-For', '').split(',')[0].strip() or \
        (request.remote or '?')
    if not trusted:
        gate.note_request(who)
        node.get_logger().info('遠隔クライアントが接続しました（%s・閲覧のみ）' % who)

    url = 'http://127.0.0.1:%d/' % node.rosbridge_port
    session = aiohttp.ClientSession()
    dropped = 0
    try:
        async with session.ws_connect(url, heartbeat=30) as ws_up:

            async def client_to_ros():
                nonlocal dropped
                async for msg in ws_client:
                    if msg.type != WSMsgType.TEXT:
                        continue
                    if trusted:
                        await ws_up.send_str(msg.data)
                        continue
                    # --- ここが遠隔クライアントの関門 ---
                    try:
                        op = json.loads(msg.data)
                    except ValueError:
                        continue
                    kind = op.get('op')
                    if kind in ('publish', 'advertise', 'call_service'):
                        if not _is_publish_allowed(op, gate):
                            topic = op.get('topic') or op.get('service') or ''
                            dropped += 1
                            if dropped in (1, 10, 100):
                                node.get_logger().warn(
                                    '遠隔からの操作を遮断しました（%s → %s）。'
                                    '許可するには手元の画面で「遠隔操作を許可」'
                                    % (who, topic))
                            continue
                    await ws_up.send_str(msg.data)

            async def ros_to_client():
                async for msg in ws_up:
                    if msg.type == WSMsgType.TEXT:
                        await ws_client.send_str(msg.data)
                    elif msg.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                        break

            task_up = asyncio.create_task(client_to_ros())
            task_down = asyncio.create_task(ros_to_client())
            done, pending = await asyncio.wait(
                {task_up, task_down}, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warn(
            'rosbridge へ中継できません (%s)。rosbridge_server は動いていますか' % exc)
    finally:
        await session.close()
        if not ws_client.closed:
            await ws_client.close()
    return ws_client


async def control_status(request):
    """いまの操作権の状態。手元の画面が定期的に見る."""
    node = request.app['node']
    status = node.control_gate.status()
    status['trusted'] = peer_is_trusted(request)
    status['remote_enabled'] = node.allow_remote_control
    return web.json_response(status)


async def control_grant(request):
    """遠隔からの操作を許可する。手元からしか呼べない."""
    node = request.app['node']
    if not peer_is_trusted(request):
        return web.json_response(
            {'error': '操作の許可は手元（LAN / VPN）からしか出せません'}, status=403)
    if not node.allow_remote_control:
        return web.json_response(
            {'error': '遠隔操作は allow_remote_control:=false で無効化されています'},
            status=403)
    try:
        body = await request.json()
    except Exception:  # noqa: BLE001
        body = {}
    seconds = node.control_gate.grant(body.get('seconds'), body.get('who', ''))
    return web.json_response({'granted': True, 'seconds': seconds})


async def control_revoke(request):
    """許可を取り消す。手元からしか呼べない."""
    node = request.app['node']
    if not peer_is_trusted(request):
        return web.json_response(
            {'error': '許可の取り消しは手元からしか出せません'}, status=403)
    node.control_gate.revoke()
    return web.json_response({'granted': False})


async def control_request(request):
    """遠隔から「操作したい」と手元へ知らせる（許可はされない）."""
    node = request.app['node']
    who = request.headers.get('CF-Connecting-IP') or \
        request.headers.get('X-Forwarded-For', '').split(',')[0].strip() or \
        (request.remote or '遠隔')
    node.control_gate.note_request(who)
    node.get_logger().warn('遠隔から操作の要求が来ています（%s）' % who)
    return web.json_response({'requested': True})


def add_routes(app):
    """ui_server の aiohttp アプリに遠隔アクセス用の経路を足す."""
    app.router.add_get('/rosbridge', rosbridge_proxy)
    app.router.add_get('/api/control', control_status)
    app.router.add_post('/api/control/grant', control_grant)
    app.router.add_post('/api/control/revoke', control_revoke)
    app.router.add_post('/api/control/request', control_request)
