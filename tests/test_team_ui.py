"""収穫システムの状態表示（段階1）を本物のブラウザで確かめる.

test_team_status.py はシーケンサが /arm_ui/state に正しく載せるところまで。
ここでは、それが画面の「収穫システムの状態」に取り違えずに出るかを
Chromium で見る（test_xyz_ui.py と同じく、静的配信＋偽 rosbridge だけを立てる）。

偽 rosbridge は 0.1 秒ごとに「いまの状態」を流し続けるので、
テストの途中で stack.state を差し替えれば画面がそれに追従する。
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
PORT = 8078
FAIL = []

BASE_STATE = {
    'src': 'harvest_sequencer#1',
    'state': 'IDLE', 'state_label': '待機', 'message': 'システム待機中',
    'enable_arm_command': False, 'progress': None,
    'selected_id': None, 'basket_index': None, 'harvest_count': 0,
    'gripper': 0.035, 'gripper_closed': False,
    'command_joints': [0, 0, 0, 0, 0, 0], 'feedback_joints': [0] * 6,
    'feedback_gripper': 0.0, 'feedback_age': 0.02, 'arm_connected': True,
    'arm_status': {}, 'motion_speed': 0,
    'basket_poses': [[2.094, 1.2, -1.0, 0, 0.4, 0],
                     [-2.094, 1.2, -1.0, 0, 0.4, 0],
                     [1.047, 0.9, -0.7, 0, 0.3, 0]],
    'home_pose': [0, 0, 0, 0, 0, 0], 'end_pose': {},
    'tcp': None, 'tcp_available': False, 'tcp_reason': '', 'tcp_link': '',
    'tcp_reach': None, 'max_cartesian_step': 0.25,
    'command_topic_taken': True, 'joint_command_topic': '/joint_states',
    'grasp_plan': None, 'detections': [], 'stamp': 0,
}


def team(manager=None, approach=None, grasp=None, target=None):
    def src(v, present=True):
        if v is None:
            return {'present': present, 'text': None, 'kind': None, 'label': None, 'age': None}
        text, kind, label = v
        return {'present': True, 'text': text, 'kind': kind, 'label': label, 'age': 1.5}
    return {
        'manager': manager if isinstance(manager, dict) else src(manager),
        'approach': src(approach), 'grasp': src(grasp),
        'target': target or {'present': True, 'seen': False, 'locked': False,
                             'frame_id': '', 'point': None, 'age': None},
    }


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


class Stack:
    """静的配信 + 状態を流し続ける偽 rosbridge."""

    def __init__(self, state):
        self.state = state
        self.logs = []             # 次に流す /arm_ui/log
        self.published = []
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
        app.router.add_get('/rosbridge', self._ws)
        app.router.add_static('/static/', path=WEB, follow_symlinks=True)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, '127.0.0.1', PORT).start()
        self.ready.set()

    async def _info(self, _req):
        return web.json_response({
            'use_dummy_video': True, 'camera_topics': {}, 'camera_transport': 'raw',
            'live_sources': [], 'video': {'width': 640, 'height': 480, 'fps': 15},
            'rosbridge_port': 9090, 'rosbridge_path': '/rosbridge',
            'webrtc_available': False, 'robot_view_available': False,
            'ice_servers': [], 'trusted': True, 'allow_remote_control': True,
        })

    async def _ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        subscribed = set()

        async def pump():
            while not ws.closed:
                if '/arm_ui/state' in subscribed:
                    await ws.send_str(json.dumps({
                        'op': 'publish', 'topic': '/arm_ui/state',
                        'msg': {'data': json.dumps(self.state)}}))
                while self.logs and '/arm_ui/log' in subscribed:
                    await ws.send_str(json.dumps({
                        'op': 'publish', 'topic': '/arm_ui/log',
                        'msg': {'data': json.dumps(self.logs.pop(0))}}))
                await asyncio.sleep(0.1)

        task = asyncio.ensure_future(pump())
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            op = json.loads(msg.data)
            if op.get('op') == 'subscribe':
                subscribed.add(op.get('topic'))
            elif op.get('op') == 'publish':
                self.published.append(op)
        task.cancel()
        return ws


def row(page, key):
    sel = '#team-status [data-team="%s"] dd' % key
    return page.inner_text(sel), page.get_attribute(sel, 'class') or ''


def settle(page):
    page.wait_for_timeout(350)


# --------------------------------------------------------------------- #
stack = Stack(dict(BASE_STATE))
stack.start()

with sync_playwright() as pw:
    browser = pw.chromium.launch(args=['--no-sandbox'])
    page = browser.new_page()
    # 検出枠の描画を記録する（色とラベル）。canvas の画素を読むより取り違えにくい
    page.add_init_script('''
      window.__draw = [];
      const P = CanvasRenderingContext2D.prototype;
      const sr = P.strokeRect, ft = P.fillText;
      P.strokeRect = function (...a) { window.__draw.push({k: 'rect', c: this.strokeStyle, w: this.lineWidth}); return sr.apply(this, a); };
      P.fillText = function (t, ...a) { window.__draw.push({k: 'text', t: String(t)}); return ft.call(this, t, ...a); };
    ''')
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.on('console', lambda m: errors.append('console: ' + m.text)
            if m.type == 'error' and 'offer' not in m.text
            and 'Failed to load resource' not in m.text else None)
    page.goto('http://127.0.0.1:%d/' % PORT)
    page.wait_for_selector('#team-status', timeout=15000)

    # ---- 置き場所 ------------------------------------------------------
    check('収穫シーケンスのパネルの中にある',
          page.eval_on_selector('#team-status', 'e => !!e.closest(".panel-actions")'))
    check('表示のみと明記している', '表示のみ' in page.inner_text('#team-status'))

    # ---- 古いシーケンサ（team が無い）でも壊れない ------------------------
    settle(page)
    check('team が無い状態でも — を出す', row(page, 'grasp')[0] == '—', row(page, 'grasp'))

    # ---- auto_run:=false（manager 不在）--------------------------------
    stack.state = dict(BASE_STATE, team=team(
        manager={'present': False, 'text': None, 'kind': None, 'label': None, 'age': None}))
    settle(page)
    text, cls = row(page, 'manager')
    check('manager 不在を「起動していません（auto_run:=false）」と出す',
          '起動していません' in text and 'auto_run' in text, text)
    check('manager 不在は異常の色にしない', 'k-fail' not in cls, cls)
    check('何も来ていない把持は「まだ何も来ていません」',
          'まだ何も来ていません' in row(page, 'grasp')[0], row(page, 'grasp'))
    check('対象なし', row(page, 'target')[0] == '対象なし', row(page, 'target'))

    # ---- 把持中 --------------------------------------------------------
    stack.state = dict(BASE_STATE, team=team(
        grasp=('inserting', 'busy', '差し込み中'),
        target={'present': True, 'seen': True, 'locked': True, 'frame_id': 'base_link',
                'point': [0.312, -0.05, 0.2], 'age': 3.0}))
    settle(page)
    text, cls = row(page, 'grasp')
    check('把持の状態と経過秒が出る', '差し込み中' in text and '1.5 秒前' in text, text)
    check('動作中は busy の色', 'k-busy' in cls, cls)
    text, cls = row(page, 'target')
    check('固定された対象の位置と座標系が出る',
          '0.312' in text and '-0.050' in text and 'base_link' in text, text)

    # ---- 掴めなかった --------------------------------------------------
    stack.state = dict(BASE_STATE, team=team(
        grasp=('missed (gripper closed to 0.001)', 'missed',
               '掴めませんでした（missed (gripper closed to 0.001)）')))
    settle(page)
    text, cls = row(page, 'grasp')
    check('missed は「掴めませんでした」', '掴めませんでした' in text, text)
    check('missed は異常（赤）にしない', 'k-missed' in cls and 'k-fail' not in cls, cls)
    check('UI 自身の状態バッジは異常にならない',
          'is-error' not in (page.get_attribute('#state-badge', 'class') or ''))

    # ---- 失敗の message をそのまま、HTML として解釈しない ------------------
    evil = 'arm trajectory failed: <img src=x onerror="window.__pwned=1">'
    stack.state = dict(BASE_STATE, team=team(approach=(evil, 'fail', evil)))
    stack.logs.append({'t': 0, 'level': 'error', 'text': '[接近] ' + evil})
    settle(page)
    text, cls = row(page, 'approach')
    check('失敗の message が文字のまま出る', '<img' in text, text)
    check('失敗は fail の色', 'k-fail' in cls, cls)
    check('ログにも文字のまま出る', '<img' in page.inner_text('#log-list'),
          page.inner_text('#log-list')[:120])
    check('相手の文字列が HTML として実行されない',
          page.evaluate('() => window.__pwned === undefined'))

    # ---- ボタンの状態に影響しない ---------------------------------------
    check('表示を足してもボタンの有効条件は変わらない（IDLE で把持は押せない）',
          page.is_disabled('[data-cmd="grasp"]'))
    check('表示しただけでコマンドを送らない',
          not any(op.get('topic') == '/arm_ui/command' for op in stack.published),
          [op for op in stack.published if op.get('topic') == '/arm_ui/command'])

    # ---- 検出枠：チームが固定した対象（段階2の lemon_detections が印を付ける）----
    det = [{'id': 1, 'u': 0.3, 'v': 0.4, 'w': 0.1, 'h': 0.12, 'distance': 0.36,
            'confidence': 0.96, 'label': 'lemon', 'locked': False},
           {'id': 2, 'u': 0.6, 'v': 0.5, 'w': 0.1, 'h': 0.12, 'distance': 0.41,
            'confidence': 0.88, 'label': 'lemon', 'locked': True}]
    stack.state = dict(BASE_STATE, detections=det, team=team())
    settle(page)
    page.evaluate('() => { window.__draw = []; }')
    page.wait_for_timeout(300)
    draws = page.evaluate('() => window.__draw')
    rects = [d for d in draws if d['k'] == 'rect']
    texts = [d['t'] for d in draws if d['k'] == 'text']
    check('検出枠が描かれる', len(rects) >= 2, len(rects))
    check('固定された対象は橙の太枠', any(r['c'] == '#ff7a3d' and r['w'] == 4 for r in rects),
          sorted({(r['c'], r['w']) for r in rects}))
    check('固定されていない検出は普通の枠',
          any(r['c'] != '#ff7a3d' and r['w'] == 2 for r in rects))
    check('固定された対象のラベルに「対象」と出る',
          any(t.startswith('対象 #2') for t in texts), texts[:4])
    check('ほかの検出には「対象」と出ない',
          any(t.startswith('#1') for t in texts) and not any(t.startswith('対象 #1') for t in texts),
          texts[:4])
    check('検出の数のタグ', page.inner_text('#tag-detections') == '検出 2',
          page.inner_text('#tag-detections'))

    check('コンソールエラーなし', not errors, errors[:3])
    browser.close()

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
