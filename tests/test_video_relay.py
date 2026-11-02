"""映像の中継（MediaRelay）でフレームが溜まらないことの検証.

守りたいこと:

    エンコードがカメラの fps に追いつかなくても、ui_server の映像トラックは
    フレームを溜め込まない（メモリが増え続けない・映像が遅れていかない）。

実機（ミニPC、YOLO と同居で CPU が溢れる）で ui_server が毎秒 20〜28MB
メモリを食い、5分で 6GB に達した。aiortc の MediaRelay.subscribe() の既定
buffered=True が上限の無いキューを挟んでいたため。画面を見ても
「映像が少しずつ遅れる」くらいしか分からないので、数値で押さえる。

ui_server.UiServer.make_track を本物のまま通す（ノード全体は作らず、
make_track が使う属性だけを持つ入れ物に対して呼ぶ）。aiortc が要る。
"""
import asyncio
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import mini_ros                                       # noqa: E402

mini_ros.install()

import numpy as np                                    # noqa: E402
from aiortc.contrib.media import MediaRelay           # noqa: E402

from piper_arm_ui.ui_server import UiServer           # noqa: E402
from piper_arm_ui.video_source import VIDEO_CLOCK_RATE, RosImageTrack  # noqa: E402

FAIL = []

FPS = 15
CONSUMER_INTERVAL = 0.2      # エンコーダが 5fps しか出せない想定（カメラの 1/3）
DURATION = 4.0


def check(name, cond, detail=''):
    print('%s %s%s' % ('  OK  ' if cond else ' FAIL ', name,
                       ('  -> ' + str(detail)) if detail else ''))
    if not cond:
        FAIL.append(name)


def holder():
    """make_track が見る属性だけを持つ入れ物（ROS ノードは作らない）."""
    return types.SimpleNamespace(
        ros_tracks={'arm': RosImageTrack(640, 480, FPS, label='ARM')},
        relay=MediaRelay(), width=640, height=480, fps=FPS, latest_detections=[])


async def slow_consumer(proxies, source):
    """カメラは新しい絵を出し続け、受け手は遅いペースでしか取りに来ない."""
    t0 = time.time()
    got = {id(p): 0 for p in proxies}
    last = {}
    while time.time() - t0 < DURATION:
        source.set_frame(np.zeros((480, 640, 3), np.uint8))
        for p in proxies:
            last[id(p)] = await p.recv()
            got[id(p)] += 1
        await asyncio.sleep(CONSUMER_INTERVAL)
    return got, last


def queued(proxy):
    q = getattr(proxy, '_queue', None)
    return q.qsize() if q is not None else 0


def lag_of(frame, source):
    """受け取った最後の絵が、いまからどれだけ前のものか [s]."""
    return time.time() - (source._start + frame.pts / VIDEO_CLOCK_RATE)


async def main():
    # ---- 1台のブラウザ --------------------------------------------------
    h = holder()
    src = h.ros_tracks['arm']
    src.set_frame(np.zeros((480, 640, 3), np.uint8))
    proxy = UiServer.make_track(h, 'arm')
    got, last = await slow_consumer([proxy], src)
    n_q = queued(proxy)
    lag = lag_of(last[id(proxy)], src)
    check('受け手が遅くてもフレームを溜めない', n_q <= 1,
          '%d 枚（約 %.0fMB）溜まった' % (n_q, n_q * 0.92))
    check('映像の遅れが伸びない（1枚分＋受け手の間隔以内）',
          lag < CONSUMER_INTERVAL + 2.0 / FPS, '%.2f 秒' % lag)
    check('映像は流れ続ける（受け手のペースで受け取れる）',
          got[id(proxy)] >= DURATION / CONSUMER_INTERVAL * 0.7, got[id(proxy)])
    proxy.stop()
    src.stop()

    # ---- 2台のブラウザ（MediaRelay を使う理由）--------------------------
    h = holder()
    src = h.ros_tracks['arm']
    src.set_frame(np.zeros((480, 640, 3), np.uint8))
    a = UiServer.make_track(h, 'arm')
    b = UiServer.make_track(h, 'arm')
    check('ブラウザごとに別のトラックになる', a is not b)
    got, last = await slow_consumer([a, b], src)
    check('2台とも映像を受け取れる（奪い合わない）',
          min(got.values()) >= DURATION / CONSUMER_INTERVAL * 0.7, list(got.values()))
    check('2台でも溜めない', max(queued(a), queued(b)) <= 1, [queued(a), queued(b)])
    a.stop()
    b.stop()
    src.stop()

    # ---- 中継の無いカメラはテストパターンのまま ---------------------------
    h = holder()
    dummy = UiServer.make_track(h, 'front')
    check('トピック未設定のカメラはテストパターン',
          type(dummy).__name__ == 'DummyVideoTrack', type(dummy).__name__)


asyncio.run(main())


# --------------------------------------------------------------------- #
# 新しい絵が来たときだけ送る（同じ絵を何度もエンコードしない）
# --------------------------------------------------------------------- #
async def pump_camera(track, cam_fps, seconds):
    """カメラが cam_fps で絵を出し、受け手はできるだけ速く取りに行く."""
    async def camera():
        while True:
            track.set_frame(np.zeros((480, 640, 3), np.uint8))
            await asyncio.sleep(1.0 / cam_fps)
    cam = asyncio.ensure_future(camera()) if cam_fps else None
    frames = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        f = await track.recv()
        frames.append((time.time(), f.pts))
    if cam:
        cam.cancel()
    return frames


async def frames_main():
    print('[送る枚数]')
    tr = RosImageTrack(640, 480, FPS)
    tr.set_frame(np.zeros((480, 640, 3), np.uint8))
    got = await pump_camera(tr, 6, 3.0)
    rate = len(got) / 3.0
    check('カメラが 6fps なら 6fps 前後しか送らない（15fps で繰り返さない）',
          4.5 <= rate <= 8.0, '%.1f fps' % rate)

    tr = RosImageTrack(640, 480, FPS)
    tr.set_frame(np.zeros((480, 640, 3), np.uint8))
    got = await pump_camera(tr, 30, 3.0)
    rate = len(got) / 3.0
    check('カメラが速くても fps（15）を超えない', rate <= FPS * 1.15, '%.1f fps' % rate)

    tr = RosImageTrack(640, 480, FPS)
    tr.set_frame(np.zeros((480, 640, 3), np.uint8))
    got = await pump_camera(tr, 0, 3.2)
    check('絵が来なくても約1秒ごとに送って接続を保つ',
          3 <= len(got) <= 5, '%d 枚 / 3.2 秒' % len(got))

    tr = RosImageTrack(640, 480, FPS)
    tr.set_frame(np.zeros((480, 640, 3), np.uint8))
    got = await pump_camera(tr, 6, 2.0)
    t_first, pts_first = got[0]
    worst = max(abs((t - t_first) - (pts - pts_first) / VIDEO_CLOCK_RATE) for t, pts in got)
    check('pts は実時間に沿う（不規則に送っても再生速度が狂わない）',
          worst < 0.05, '最大のずれ %.3f 秒' % worst)
    check('pts は単調に増える',
          all(b[1] > a[1] for a, b in zip(got, got[1:])))


asyncio.run(frames_main())


# --------------------------------------------------------------------- #
# 送る映像の選び方（中身の無い映像をエンコードしない）
# --------------------------------------------------------------------- #
def cams(use_dummy=False, front='', rviz_enabled=False, rviz_pubs=0):
    h = types.SimpleNamespace(
        use_dummy=use_dummy, rviz_enabled=rviz_enabled,
        camera_topics={'arm': '/camera/camera/color/image_raw', 'front': front,
                       'rviz': '/rviz_ui/image'},
        count_publishers=lambda _t: rviz_pubs)
    return UiServer.video_cameras(h)


print('[送る映像]')
check('実機の既定（前方カメラ無し・RViz 無し）は手首カメラだけ',
      cams() == ['arm'], cams())
check('前方カメラのトピックがあれば送る',
      cams(front='/front/image_raw') == ['arm', 'front'], cams(front='/front/image_raw'))
check('開発用（use_dummy_video）は前方のテストパターンも送る',
      cams(use_dummy=True) == ['arm', 'front'], cams(use_dummy=True))
check('start_rviz_bridge:=true なら RViz を送る',
      cams(rviz_enabled=True) == ['arm', 'rviz'], cams(rviz_enabled=True))
check('別に起動した rviz_bridge でも、画像が来ていれば送る',
      cams(rviz_pubs=1) == ['arm', 'rviz'], cams(rviz_pubs=1))

print()
if FAIL:
    print('FAILED (%d): %s' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('すべて通りました')
