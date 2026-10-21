/*
 * ui_server の /offer と WebRTC でつなぎ、カメラ映像を <video> に流す。
 * 同一 PeerConnection で複数カメラを受けられるよう、
 * サーバから返ってくる mids（カメラ名 -> mid）でトラックを振り分ける。
 */
class WebRtcVideo {
  constructor(cameras, videoElements, onState, iceServers) {
    this.cameras = cameras;              // 例: ['arm', 'front']
    this.videoElements = videoElements;  // 例: {arm: <video>, front: <video>}
    this.onState = onState || (() => {});
    // 同一 LAN なら空でよい。別ネットワークから見るときは
    // ui_server の ice_servers パラメータで STUN / TURN を渡す。
    this.iceServers = iceServers || [];
    this.pc = null;
    this.pendingTracks = [];
    this.mids = {};
    this.retryDelay = 2000;
    this.closed = false;
  }

  async start() {
    this.closed = false;
    await this._connect();
  }

  stop() {
    this.closed = true;
    if (this.pc) {
      this.pc.close();
      this.pc = null;
    }
    this.onState('closed');
  }

  async _connect() {
    if (this.closed) return;
    this.onState('connecting');
    try {
      const pc = new RTCPeerConnection({ iceServers: this.iceServers });
      this.pc = pc;
      this.pendingTracks = [];
      this.mids = {};

      pc.addEventListener('track', (event) => this._onTrack(event));
      pc.addEventListener('connectionstatechange', () => {
        this.onState(pc.connectionState);
        if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)
            && !this.closed) {
          setTimeout(() => this._connect(), this.retryDelay);
        }
      });

      this.cameras.forEach(() => pc.addTransceiver('video', { direction: 'recvonly' }));

      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      await this._waitIceGathering(pc);

      const response = await fetch('/offer', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sdp: pc.localDescription.sdp,
          type: pc.localDescription.type,
          cameras: this.cameras,
        }),
      });
      if (!response.ok) throw new Error(`offer failed: ${response.status}`);
      const answer = await response.json();
      this.mids = answer.mids || {};
      await pc.setRemoteDescription(answer);
      this._flushPendingTracks();
    } catch (err) {
      console.error('WebRTC接続に失敗:', err);
      this.onState('failed');
      if (!this.closed) setTimeout(() => this._connect(), this.retryDelay);
    }
  }

  _waitIceGathering(pc) {
    if (pc.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise((resolve) => {
      const check = () => {
        if (pc.iceGatheringState === 'complete') {
          pc.removeEventListener('icegatheringstatechange', check);
          resolve();
        }
      };
      pc.addEventListener('icegatheringstatechange', check);
      // ローカルだけなら一瞬で終わる。STUN/TURN があると往復ぶん待つので長めに。
      setTimeout(resolve, this.iceServers.length ? 5000 : 2000);
    });
  }

  _onTrack(event) {
    this.pendingTracks.push(event);
    this._flushPendingTracks();
  }

  _flushPendingTracks() {
    if (!Object.keys(this.mids).length) return;
    const midToCamera = {};
    Object.entries(this.mids).forEach(([cam, mid]) => { midToCamera[mid] = cam; });

    const remaining = [];
    this.pendingTracks.forEach((event, index) => {
      const mid = event.transceiver && event.transceiver.mid;
      const camera = midToCamera[mid] || this.cameras[index];
      const el = this.videoElements[camera];
      if (el) {
        // aiortc は複数トラックを 1 つの MediaStream にまとめて msid を振るため、
        // event.streams[0] を使うと 3 つの <video> が同じストリーム（＝先頭の
        // カメラ）を指してしまう。必ずトラック単位のストリームを作り直す。
        el.srcObject = new MediaStream([event.track]);
        el.play().catch(() => {});
      } else {
        remaining.push(event);
      }
    });
    this.pendingTracks = remaining;
  }
}

window.WebRtcVideo = WebRtcVideo;
