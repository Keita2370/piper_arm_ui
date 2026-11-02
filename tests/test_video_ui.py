"""ブラウザが要求する映像を本物のブラウザで確かめる.

守りたいこと:

    ブラウザは /api/info の video_cameras にある映像だけを要求する。
    要求した映像は中身が無くてもサーバでエンコードされ続け、ミニPCでは
    その CPU が YOLO を遅くして lemon_target が対象を選べなくなったため。

/offer を偽物にして、ブラウザが送ってきた cameras を記録する
（WebRTC の接続そのものは張らない。503 を返すとブラウザは再試行する）。
"""
import asyncio
import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aiohttp import web                                # noqa: E402
from playwright.sync_api import sync_playwright        # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
PORT = 8079
FAIL = []


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


class Stack:
    def __init__(self):
        self.info_extra = {}
        self.offers = []
        self.subscribes = []       # ブラウザが rosbridge に送った subscribe
        self.ready = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        self.ready.wait(10)

    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._serve())
        loop.run_forever()

    async def _serve(self):
        app = web.Application()
        app.router.add_get('/', lambda _r: web.FileResponse(os.path.join(WEB, 'index.html')))
        app.router.add_get('/api/info', self._info)
        app.router.add_get('/api/robot', lambda _r: web.json_response(
            {'available': False, 'reason': 'テストでは 3D 無効'}))
        app.router.add_post('/offer', self._offer)
        app.router.add_get('/rosbridge', self._ws)
        app.router.add_static('/static/', path=WEB, follow_symlinks=True)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, '127.0.0.1', PORT).start()
        self.ready.set()

    async def _info(self, _req):
        info = {
            'use_dummy_video': False, 'camera_topics': {}, 'camera_transport': 'raw',
            'live_sources': ['arm', 'rviz'],
            'video': {'width': 640, 'height': 480, 'fps': 15},
            'rosbridge_port': 9090, 'rosbridge_path': '/rosbridge',
            'webrtc_available': True, 'robot_view_available': False,
            'ice_servers': [], 'trusted': True, 'allow_remote_control': True,
        }
        info.update(self.info_extra)
        return web.json_response(info)

    async def _ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                op = json.loads(msg.data)
                if op.get('op') == 'subscribe':
                    self.subscribes.append(op)
        return ws

    async def _offer(self, req):
        body = await req.json()
        self.offers.append(body)
        return web.json_response({'error': 'test'}, status=503)


def first_offer(page, stack, timeout_ms=10000):
    stack.offers.clear()
    page.goto('http://127.0.0.1:%d/' % PORT)
    waited = 0
    while not stack.offers and waited < timeout_ms:
        page.wait_for_timeout(200)
        waited += 200
    return stack.offers[0] if stack.offers else None


# 作られた RTCPeerConnection を数えられるようにする（閉じ忘れの検出用）
HOOK = '''
window.__pcs = [];
const Orig = window.RTCPeerConnection;
window.RTCPeerConnection = function (...a) { const pc = new Orig(...a); window.__pcs.push(pc); return pc; };
window.RTCPeerConnection.prototype = Orig.prototype;
'''

stack = Stack()
stack.start()

with sync_playwright() as pw:
    browser = pw.chromium.launch(args=['--no-sandbox'])
    page = browser.new_page()
    page.add_init_script(HOOK)
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))

    # ---- 実機の既定：手首カメラだけ ---------------------------------------
    stack.info_extra = {'video_cameras': ['arm']}
    offer = first_offer(page, stack)
    check('映像の要求が届く', offer is not None)
    check('video_cameras=[arm] なら手首カメラだけを要求する',
          offer and offer.get('cameras') == ['arm'], offer and offer.get('cameras'))
    sdp = (offer or {}).get('sdp', '')
    check('受信の口（m=video）も1本だけ作る', sdp.count('m=video') == 1, sdp.count('m=video'))
    check('要求しない RViz は理由を画面に出す',
          'start_rviz_bridge' in page.text_content('#video-hint-rviz'),
          page.text_content('#video-hint-rviz'))
    check('要求しない前方カメラも理由を画面に出す',
          'front_camera_topic' in page.text_content('#video-hint-front'),
          page.text_content('#video-hint-front'))

    # ---- RViz を起動したとき -------------------------------------------
    stack.info_extra = {'video_cameras': ['arm', 'rviz']}
    offer = first_offer(page, stack)
    check('video_cameras=[arm, rviz] ならその2本を要求する',
          offer and offer.get('cameras') == ['arm', 'rviz'], offer and offer.get('cameras'))
    check('受信の口は2本', (offer or {}).get('sdp', '').count('m=video') == 2)

    # ---- 古いサーバ（一覧を返さない）------------------------------------
    stack.info_extra = {}
    offer = first_offer(page, stack)
    check('一覧が無い古いサーバには従来どおり3本とも要求する',
          offer and offer.get('cameras') == ['arm', 'front', 'rviz'],
          offer and offer.get('cameras'))

    # ---- 再接続で古い接続を残さない --------------------------------------
    # /offer が失敗し続けると、ブラウザは 2 秒おきに作り直す。以前は古い接続を
    # 閉じずに次を作っていた（サーバはその全部についてエンコードを続ける）
    stack.info_extra = {'video_cameras': ['arm']}
    stack.offers.clear()
    page.goto('http://127.0.0.1:%d/' % PORT)
    page.wait_for_timeout(9000)
    made = page.evaluate('() => window.__pcs.length')
    alive = page.evaluate("() => window.__pcs.filter(pc => pc.signalingState !== 'closed').length")
    check('失敗が続いても作り直している（再接続は止まらない）', made >= 2, made)
    check('開いたままの接続は常に1本だけ', alive <= 1, '%d 本中 %d 本が開いたまま' % (made, alive))
    check('再接続が同時に何本も走らない（2秒おき程度）',
          len(stack.offers) <= 9000 / 2000 + 1, '%d 回 / 9 秒' % len(stack.offers))

    # ---- 関節角は間引いて受け取る ----------------------------------------
    subs = {op['topic']: op.get('throttle_rate') for op in stack.subscribes}
    check('/joint_states は 50ms（毎秒20回）に間引く',
          subs.get('/joint_states') == 50, subs.get('/joint_states'))
    check('/joint_states_single も 50ms に間引く',
          subs.get('/joint_states_single') == 50, subs.get('/joint_states_single'))
    check('状態（/arm_ui/state）は間引かない',
          subs.get('/arm_ui/state') == 0, subs.get('/arm_ui/state'))

    check('ページの JS 例外なし', not errors, errors[:3])
    browser.close()

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
