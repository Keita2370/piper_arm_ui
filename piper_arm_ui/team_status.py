"""チームの収穫システムの状態をまとめる（段階1：表示だけ）.

lemon_harvest_manager / lemon_approach / lemon_grasp の status と
lemon_target の対象を受け取り、/arm_ui/state に載せる形に整える。

【なぜ UI の12状態に写さないのか】
auto_run:=false では lemon_harvest_manager が起動せず、
/lemon_harvest_manager/status は誰も出さない。計画書 §7 の対応表は
このトピックを前提にしているので、そのまま写すと待機中しか出ない。
どこから状態を組み立てるかは段階3以降（サービスを呼ぶ側になってから）に
決めることにして、ここでは相手が言ったことをそのまま見せる。

ROS に依存しない。時刻は呼び出し側が渡す（テストで回しやすいように）。
"""

from piper_arm_ui.common import TEAM_STATUS_LABELS

SOURCES = ('manager', 'approach', 'grasp')


# 相手が返す失敗の message のうち、画面で言い換えたほうが次の操作が分かるもの。
# 前方一致で引く（'observe_left is not set' などの後ろに続きが付いても拾える）
EXPLAIN = (
    ('harvest is disabled', '収穫できません（アームが移動姿勢です）。先に「認識姿勢へ」を押してください'),
    ('observe_right is not set', '認識姿勢（右）が設定されていません（lemon_reach の harvest_sequence.yaml）'),
    ('observe_left is not set', '認識姿勢（左）が設定されていません（lemon_reach の harvest_sequence.yaml）'),
    ('collect is not set', '回収姿勢が設定されていません（lemon_reach の harvest_sequence.yaml）'),
    ('home is not set', '移動姿勢（home）が設定されていません'),
    ('home is not recorded', '移動姿勢（home）が記録されていません'),
    # lemon_approach（把持姿勢へ移動）の失敗。どれもアームは動いていない
    ('cannot insert straight into the lemon',
     'このレモンには、まっすぐ差し込める向きがありません（高すぎる・横すぎる・遠すぎる）。'
     'レモンの位置を変えるか、別のレモンを選んでください'),
    ('failed to compute pregrasp', '把持姿勢を計算できませんでした（対象の位置が取れない）。選び直してください'),
    ('no target', '対象が選ばれていません。レモンの枠をクリックしてください'),
    # MoveIt の計画の失敗。把持姿勢・home・認識姿勢・回収姿勢のどれでも返る
    ('unreachable after', '経路が見つかりませんでした（MoveIt。届かない・可動域・干渉）'),
)


def explain(text):
    """相手の message を、画面で分かる言葉にする（知らないものはそのまま）."""
    text = (text or '').strip()
    for key, desc in EXPLAIN:
        if text.startswith(key):
            return '%s（%s）' % (desc, text)
    return text


def classify(source, text):
    """status 文字列を (種別, 画面表示) にする.

    種別は idle / busy / ok / missed / stopped / fail。
    """
    text = (text or '').strip()
    known = TEAM_STATUS_LABELS.get(source, {}).get(text)
    if known:
        return known
    if source == 'grasp':
        # 'missed (gripper closed to X)' は動作が正常に終わって掴めなかっただけ。
        # 異常にしない。ただし戻りに失敗していたら（', retreat failed: ...'）
        # アームがレモンの近くに残っているので fail にする
        if 'retreat failed' in text:
            return ('fail', '戻りに失敗: %s' % text)
        if text.startswith('missed'):
            return ('missed', '掴めませんでした（%s）' % text)
        # /lemon_grasp/stop で止めた（人が止めた）。異常ではない。
        # 止めたあとはレモンや枝の近くに残るので、それも出す
        if text.startswith('stopped'):
            return ('stopped', '停止しました（%s）。アームはその場に残っています' % text)
    # 相手は失敗の message をそのまま status に流す。列挙しきれないので素通し
    # （ただし次の操作が分かるものは言い換える）
    return ('fail', explain(text))


class TeamStatus:
    """相手の各ノードの最新状態を持つ."""

    def __init__(self):
        self.status = {s: None for s in SOURCES}     # {'text', 'kind', 'label', 't'}
        # 該当トピックに publisher がいるか。None は「まだ確かめていない」
        self.present = {s: None for s in SOURCES}
        self.present_target = None
        self.target = None                            # {'frame_id', 'x','y','z', 't'}
        self.target_seen = False                      # 一度でも受け取ったか

    def on_status(self, source, text, now):
        """status を受け取る。表示が変わったら (種別, 画面表示) を返す."""
        kind, label = classify(source, text)
        prev = self.status.get(source)
        self.status[source] = {'text': text, 'kind': kind, 'label': label, 't': now}
        if prev is not None and prev['text'] == text:
            return None
        return kind, label

    def on_target(self, frame_id, x, y, z, now):
        """対象を受け取る。固定／解除が切り替わったら True."""
        was_locked = bool(self.target and self.target['frame_id'])
        self.target_seen = True
        self.target = {'frame_id': frame_id or '', 'x': x, 'y': y, 'z': z, 't': now}
        return was_locked != bool(frame_id)

    def snapshot(self, now):
        """/arm_ui/state の 'team' に載せる dict."""
        out = {}
        for s in SOURCES:
            st = self.status[s]
            out[s] = {
                'present': self.present[s],
                'text': st['text'] if st else None,
                'kind': st['kind'] if st else None,
                'label': st['label'] if st else None,
                'age': round(now - st['t'], 1) if st else None,
            }
        tg = self.target
        locked = bool(tg and tg['frame_id'])
        out['target'] = {
            'present': self.present_target,
            'seen': self.target_seen,
            'locked': locked,
            'frame_id': tg['frame_id'] if locked else '',
            'point': [round(tg[k], 3) for k in ('x', 'y', 'z')] if locked else None,
            'age': round(now - tg['t'], 1) if tg else None,
        }
        return out
