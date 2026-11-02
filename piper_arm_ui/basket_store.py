"""収穫位置①②③の記録（backend:=lemon）.

アームをかごの上に置いた状態で、画面の「いまの姿勢を①にする」で関節角を記録する。
記録はファイルに残し、起動し直しても使えるようにする。

【なぜ設定ファイルの値を使わないのか】
config/arm_ui.yaml の basket_pose_1〜3 は実機なしの開発用の仮の値で、joint1 が ±120°
（後方）になっている。犬型の胴体やかごの位置とは合っていない。MoveIt は URDF に無い
胴体やかごを避けられないので、測っていない値で実機を動かすのは危ない。
lemon では、人が実機で置いて記録した姿勢だけを使う。

ROS に依存しない。
"""

import json
import os
import tempfile
import time

INDICES = (1, 2, 3)


class BasketStore:

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.poses = {}          # {1: {'joints': [6], 'recorded_at': '...'}}
        self.error = ''
        self.load()

    def load(self):
        self.poses = {}
        self.error = ''
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path) as f:
                data = json.load(f)
            for key, item in data.items():
                i = int(key)
                joints = [float(v) for v in item['joints']]
                if i in INDICES and len(joints) == 6:
                    self.poses[i] = {'joints': joints,
                                     'recorded_at': str(item.get('recorded_at', ''))}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # 壊れたファイルで動かすより、記録が無い扱いにして理由を出す
            self.poses = {}
            self.error = '%s を読めません（記録し直してください）: %s' % (self.path, exc)

    def set(self, index, joints):
        """記録する。書き込みは一時ファイル経由で置き換える（途中で落ちても壊さない）."""
        index = int(index)
        if index not in INDICES:
            raise ValueError('収穫位置は 1〜3: %s' % index)
        joints = [round(float(v), 4) for v in joints]
        if len(joints) != 6:
            raise ValueError('関節角は 6 個: %s' % joints)
        self.poses[index] = {'joints': joints,
                             'recorded_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path) or '.', suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            json.dump({str(k): v for k, v in sorted(self.poses.items())}, f,
                      ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)
        self.error = ''
        return self.poses[index]

    def get(self, index):
        item = self.poses.get(int(index))
        return list(item['joints']) if item else None

    def snapshot(self):
        return {str(i): ({'recorded': True, 'joints': self.poses[i]['joints'],
                          'recorded_at': self.poses[i]['recorded_at']}
                         if i in self.poses else {'recorded': False})
                for i in INDICES}
