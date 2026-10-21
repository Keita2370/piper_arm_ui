"""座標指定トグルを本物のブラウザで確かめる.

サーバ側（test_cartesian.py）は通っていても、UI の配線が間違っていれば
ボタンは何も送らない。宣言順の取り違え（const の TDZ）やセレクタの
打ち間違いは、実際にページを開かないと気づけない種類の不具合なので、
ここは Chromium で開いて押してみる。

ROS も ui_server も使わず、
  - web/ を静的配信するだけのサーバ
  - rosbridge のふりをして状態を 1 回流し、publish を記録する WebSocket
の 2 つだけを立てて、ブラウザから見える部分を検証する。
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
PORT = 8077
FAIL = []

# シーケンサが流す状態の、座標指定に関わる部分だけを模したもの
BASE_STATE = {
    'src': 'harvest_sequencer#1',
    'state': 'IDLE', 'state_label': '待機', 'message': 'システム待機中',
    'enable_arm_command': True, 'progress': None,
    'selected_id': None, 'basket_index': None, 'harvest_count': 0,
    'gripper': 0.035, 'gripper_closed': False,
    'command_joints': [0, 0, 0, 0, 0, 0], 'feedback_joints': [0] * 6,
    'feedback_gripper': 0.0, 'feedback_age': None, 'arm_connected': False,
    'arm_status': {}, 'motion_speed': 0,
    'basket_poses': [[2.094, 1.2, -1.0, 0, 0.4, 0],
                     [-2.094, 1.2, -1.0, 0, 0.4, 0],
                     [1.047, 0.9, -0.7, 0, 0.3, 0]],
    'home_pose': [0, 0, 0, 0, 0, 0], 'end_pose': {},
    'tcp': [0.1234, -0.0056, 0.3456],
    'tcp_available': True, 'tcp_reason': '', 'tcp_link': 'link6',
    'tcp_reach': {'min': [-0.62, -0.59, -0.22], 'max': [0.62, 0.6, 0.75],
                  'max_radius': 0.749},
    'max_cartesian_step': 0.25,
    'grasp_plan': None, 'detections': [], 'stamp': 0,
}


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


class Stack:
    """静的配信 + 偽 rosbridge を別スレッドの asyncio で回す."""

    def __init__(self, state):
        self.state = state
        self.published = []        # ブラウザが publish したもの
        self.loop = None
        self.ready = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        self.ready.wait(10)

    def _run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._serve())
        self.loop.run_forever()

    async def _serve(self):
        app = web.Application()
        app.router.add_get('/', self._index)
        app.router.add_get('/api/info', self._info)
        app.router.add_get('/api/robot', self._no_robot)
        app.router.add_get('/rosbridge', self._ws)
        app.router.add_static('/static/', path=WEB, follow_symlinks=True)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, '127.0.0.1', PORT).start()
        self.ready.set()

    async def _index(self, _req):
        return web.FileResponse(os.path.join(WEB, 'index.html'))

    async def _info(self, _req):
        return web.json_response({
            'use_dummy_video': True, 'camera_topics': {}, 'camera_transport': 'raw',
            'live_sources': [], 'video': {'width': 640, 'height': 480, 'fps': 15},
            'rosbridge_port': 9090, 'rosbridge_path': '/rosbridge',
            'webrtc_available': False, 'robot_view_available': False,
            'ice_servers': [], 'trusted': True, 'allow_remote_control': True,
        })

    async def _no_robot(self, _req):
        return web.json_response({'available': False, 'reason': 'テストでは 3D 無効'})

    async def _ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            op = json.loads(msg.data)
            if op.get('op') == 'subscribe' and op.get('topic') == '/arm_ui/state':
                await ws.send_str(json.dumps({
                    'op': 'publish', 'topic': '/arm_ui/state',
                    'msg': {'data': json.dumps(self.state)}}))
            elif op.get('op') == 'publish':
                self.published.append(op)
        return ws

    def commands(self):
        """/arm_ui/command に流れた JSON を取り出す."""
        out = []
        for op in self.published:
            if op.get('topic') != '/arm_ui/command':
                continue
            try:
                out.append(json.loads(op['msg']['data']))
            except (KeyError, ValueError):
                pass
        return out


def open_page(pw, stack):
    browser = pw.chromium.launch(args=['--no-sandbox', '--use-gl=swiftshader',
                                       '--enable-unsafe-swiftshader'])
    page = browser.new_page()
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    # このテストは ui_server を立てないので /offer（WebRTC）は 404 になる。
    # それ由来のコンソールエラーは本題ではないので除く。JS の例外
    # （pageerror）は常に拾う。
    page.on('console', lambda m: errors.append('console: ' + m.text)
            if m.type == 'error' and 'offer' not in m.text
            and 'Failed to load resource' not in m.text else None)
    page.goto('http://127.0.0.1:%d/' % PORT)
    page.wait_for_function(
        "() => document.querySelector('#xyz-current')"
        "&& document.querySelector('#xyz-current').textContent !== '—'",
        timeout=15000)
    return browser, page, errors


# --------------------------------------------------------------------- #
stack = Stack(dict(BASE_STATE))
stack.start()

with sync_playwright() as pw:
    browser, page, errors = open_page(pw, stack)

    # ---- 置き場所: 関節角度指定の「下」にあること -----------------------
    order = page.eval_on_selector_all(
        '.panel-actions details.jog > summary', 'els => els.map(e => e.textContent.trim())')
    check('左下パネルにトグルが2つ並ぶ', len(order) == 2, order)
    check('1つめが関節角度指定', order and '関節角度指定' in order[0], order)
    check('2つめが座標指定（下に追加されている）',
          len(order) > 1 and '座標指定' in order[1], order)

    # ---- 現在位置の表示 ------------------------------------------------
    # details は閉じていると中身が描画されず inner_text が空になるので、
    # 中の文字を見るテストはトグルを開いてから行う。
    page.click('#xyz-box > summary')
    page.wait_for_timeout(100)

    check('現在の手先位置が出る',
          '0.123' in page.inner_text('#xyz-current'), page.inner_text('#xyz-current'))
    check('手先リンクと可動半径のヒントが出る',
          'link6' in page.inner_text('#xyz-hint')
          and '0.749' in page.inner_text('#xyz-hint'), page.inner_text('#xyz-hint'))

    # ---- X/Y/Z の入力欄 ------------------------------------------------
    names = page.eval_on_selector_all(
        '.xyz-axis .xyz-name', 'els => els.map(e => e.textContent.trim())')
    check('X / Y / Z の3軸がある',
          len(names) == 3 and names[0].startswith('X'), names)
    vals = page.eval_on_selector_all('.xyz-axis input', 'els => els.map(e => e.value)')
    check('入力欄に現在値が入っている', vals == ['0.123', '-0.006', '0.346'], vals)

    # ---- ± ボタンが座標ジョグを送る -------------------------------------
    stack.published.clear()
    page.click('.xyz-axis:nth-child(1) button[data-dir="1"]')
    page.wait_for_timeout(250)
    cmds = stack.commands()
    check('X+ が jog_xyz を送る',
          any(c.get('cmd') == 'jog_xyz' and c.get('axis') == 0
              and abs(c.get('delta', 0) - 0.01) < 1e-9 for c in cmds), cmds)

    stack.published.clear()
    page.click('.xyz-axis:nth-child(3) button[data-dir="-1"]')
    page.wait_for_timeout(250)
    cmds = stack.commands()
    check('Z− が軸2・負の delta を送る',
          any(c.get('cmd') == 'jog_xyz' and c.get('axis') == 2
              and abs(c.get('delta', 0) + 0.01) < 1e-9 for c in cmds), cmds)

    # ステップを変えたら delta も変わること
    page.select_option('#xyz-step', '0.05')
    stack.published.clear()
    page.click('.xyz-axis:nth-child(2) button[data-dir="1"]')
    page.wait_for_timeout(250)
    cmds = stack.commands()
    check('ステップ 50mm を選ぶと delta が 0.05 になる',
          any(abs(c.get('delta', 0) - 0.05) < 1e-9 for c in cmds), cmds)

    # ---- 「この位置へ移動」 --------------------------------------------
    page.fill('.xyz-axis:nth-child(1) input', '0.250')
    page.fill('.xyz-axis:nth-child(2) input', '0.000')
    page.fill('.xyz-axis:nth-child(3) input', '0.400')
    stack.published.clear()
    page.click('#btn-xyz-move')
    page.wait_for_timeout(250)
    cmds = stack.commands()
    check('移動ボタンが move_to_xyz を入力値で送る',
          any(c.get('cmd') == 'move_to_xyz' and c.get('x') == 0.25
              and c.get('z') == 0.4 for c in cmds), cmds)

    # ---- 数値でない入力はその場で止める --------------------------------
    page.fill('.xyz-axis:nth-child(1) input', '')
    stack.published.clear()
    page.click('#btn-xyz-move')
    page.wait_for_timeout(200)
    check('空欄のまま押しても送らない（Number("")===0 の罠）',
          not any(c.get('cmd') == 'move_to_xyz' for c in stack.commands()),
          stack.commands())
    check('空欄のときは画面で理由を出す',
          '3つとも' in page.inner_text('#xyz-hint'), page.inner_text('#xyz-hint'))

    # 数値でない文字は input[type=number] がそもそも受け付けない
    # （ブラウザ側で弾かれ、値は空文字になる）。よって UI からは
    # 上の「空欄」の経路に合流する。非数値そのものの扱いは
    # test_cartesian.py のサーバ側テストで見ている。

    # ---- 「取込」が現在値を戻す ----------------------------------------
    page.click('#btn-xyz-pick')
    page.wait_for_timeout(150)
    vals = page.eval_on_selector_all('.xyz-axis input', 'els => els.map(e => e.value)')
    check('取込で現在の手先位置が入る', vals == ['0.123', '-0.006', '0.346'], vals)

    # ---- 編集中の値を状態更新で上書きしない ----------------------------
    page.fill('.xyz-axis:nth-child(1) input', '0.300')
    page.wait_for_timeout(400)          # 状態が来るのを待つ
    check('打ち込んだ値が勝手に書き換わらない',
          page.input_value('.xyz-axis:nth-child(1) input') == '0.300',
          page.input_value('.xyz-axis:nth-child(1) input'))

    check('コンソールエラーなし', not errors, errors[:3])
    browser.close()

# --------------------------------------------------------------------- #
# URDF が無い環境（tcp_available=false）
# --------------------------------------------------------------------- #
off = dict(BASE_STATE)
off['tcp_available'] = False
off['tcp'] = None
off['tcp_reason'] = 'piper_description の URDF が見つかりません'
off['tcp_reach'] = None
stack.state = off

with sync_playwright() as pw:
    browser = pw.chromium.launch(args=['--no-sandbox'])
    page = browser.new_page()
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto('http://127.0.0.1:%d/' % PORT)
    page.wait_for_selector('#xyz-box.is-off', timeout=15000)
    page.click('#xyz-box > summary')
    page.wait_for_timeout(100)

    check('使えないときはトグル自体は残る',
          page.is_visible('#xyz-box > summary'))
    check('使えないときは入力欄を伏せる', not page.is_visible('#xyz-grid'))
    check('使えないときは移動ボタンも伏せる', not page.is_visible('#btn-xyz-move'))
    check('使えない理由を画面に出す',
          'URDF' in page.inner_text('#xyz-hint'), page.inner_text('#xyz-hint'))
    check('関節角度指定のほうは生きている',
          page.is_visible('.panel-actions details.jog:not(#xyz-box) > summary'))
    check('この状態でもコンソールエラーなし', not errors, errors[:3])
    browser.close()

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
