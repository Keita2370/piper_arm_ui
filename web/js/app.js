/*
 * アーム用UI 本体。
 *  - ROS とのやり取りは rosbridge。ポートは /api/info から受け取る
 *    （launch の rosbridge_port:= でずらせる。既定 9090）
 *  - 映像は ui_server との WebRTC
 * どちらも切断したら自動で再接続する。
 */
(async () => {
  'use strict';

  const TOPIC_STATE = '/arm_ui/state';
  const TOPIC_COMMAND = '/arm_ui/command';
  const TOPIC_LOG = '/arm_ui/log';
  const TOPIC_DOG_MODE = '/dog_ui/mode';
  const TOPIC_RVIZ_INPUT = '/rviz_ui/input';
  const TOPIC_JOINT_CMD = '/joint_states';
  const TOPIC_JOINT_FB = '/joint_states_single';
  const JOINT_STATE_TYPE = 'sensor_msgs/msg/JointState';
  const STRING_TYPE = 'std_msgs/msg/String';

  const JOINT_LIMITS = [
    [-2.618, 2.618], [0.0, 3.14], [-2.967, 0.0],
    [-1.745, 1.745], [-1.220, 1.220], [-2.094, 2.094],
  ];

  // シーケンスのどのステップが完了済みかを判定するための順序
  const STEP_ORDER = {
    IDLE: 0, SELECTING: 1, SELECTED: 1, PLANNING: 2, PLANNED: 2,
    GRASPING: 3, GRASPED: 3, MOVING_TO_BASKET: 4, AT_BASKET: 4,
    RELEASING: 5, DONE: 5, ERROR: 0,
  };
  const STEP_STATES = ['SELECTING', 'PLANNING', 'GRASPING', 'MOVING_TO_BASKET', 'RELEASING'];

  const $ = (sel) => document.querySelector(sel);
  const $$ = (sel) => Array.from(document.querySelectorAll(sel));

  const el = {
    pillRos: $('#pill-ros'),
    pillVideo: $('#pill-video'),
    pillArm: $('#pill-arm'),
    videoArm: $('#video-arm'),
    videoArm2: $('#video-arm2'),
    videoFront: $('#video-front'),
    videoRviz: $('#video-rviz'),
    rvizWrap: $('#rviz-wrap'),
    armViewCanvas: $('#arm-view'),
    overlay: $('#overlay-arm'),
    hintArm: $('#video-hint-arm'),
    hintFront: $('#video-hint-front'),
    hintRviz: $('#video-hint-rviz'),
    chkRvizInput: $('#chk-rviz-input'),
    tagDetections: $('#tag-detections'),
    tagPlan: $('#tag-plan'),
    tagCount: $('#tag-count'),
    jointTable: $('#joint-table'),
    jogGrid: $('#jog-grid'),
    jogStep: $('#jog-step'),
    xyzBox: $('#xyz-box'),
    xyzGrid: $('#xyz-grid'),
    xyzStep: $('#xyz-step'),
    xyzCurrent: $('#xyz-current'),
    xyzHint: $('#xyz-hint'),
    btnXyzPick: $('#btn-xyz-pick'),
    btnXyzMove: $('#btn-xyz-move'),
    readout: $('#readout'),
    stateBadge: $('#state-badge'),
    stateMessage: $('#state-message'),
    progressBar: $('#progress-bar'),
    logList: $('#log-list'),
    chkEnable: $('#chk-enable'),
    robotView: $('#robot-view'),
    robotHint: $('#robot-hint'),
    viewButtons: $('#view-buttons'),
    tagSource: $('#tag-source'),
    alertDup: $('#alert-dup'),
    alertDupDetail: $('#alert-dup-detail'),
    alertRemote: $('#alert-remote'),
    remoteTitle: $('#remote-title'),
    remoteDetail: $('#remote-detail'),
    btnRequestControl: $('#btn-request-control'),
    alertGrant: $('#alert-grant'),
    grantTitle: $('#grant-title'),
    grantDetail: $('#grant-detail'),
    btnGrantControl: $('#btn-grant-control'),
    btnRevokeControl: $('#btn-revoke-control'),
    chkOverlay: $('#chk-overlay'),
  };

  let latest = null;         // 最新の /arm_ui/state
  let vizMode = 'robot';     // 右上パネル: 'robot'(3D) | 'rviz'(映像) | 'simple'(予備)
  let detections = [];
  let hoverId = null;
  const logs = [];

  // ---------------------------------------------------------------- ROS
  let serverInfo = {};
  try {
    const res = await fetch('/api/info');
    if (res.ok) serverInfo = await res.json();
  } catch (e) {
    console.warn('/api/info を取得できません', e);
  }

  // rosbridge へは ui_server の /rosbridge を経由する。UI と同じ生成元なので
  //   - ポートを 1 つ開けるだけで済む（トンネル 1 本で外からも使える）
  //   - https で開いたときも wss になり、混在コンテンツで弾かれない
  //   - 手元／遠隔の権限判定がサーバ側の 1 箇所で効く
  // 古い ui_server（/rosbridge が無い）に対しては 9090 直結に戻す。
  const wsScheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const rosUrl = serverInfo.rosbridge_path
    ? `${wsScheme}://${location.host}${serverInfo.rosbridge_path}`
    : `ws://${location.hostname}:${serverInfo.rosbridge_port || 9090}`;
  const ros = new RosBridge(rosUrl);

  ros.on('connect', () => {
    setPill(el.pillRos, 'ok', 'rosbridge');
    ros.advertise(TOPIC_COMMAND, STRING_TYPE);
    ros.advertise(TOPIC_DOG_MODE, STRING_TYPE);
    ros.advertise(TOPIC_RVIZ_INPUT, STRING_TYPE);
    pushLog('rosbridge に接続しました', 'info');
  });
  ros.on('close', () => {
    setPill(el.pillRos, 'bad', 'rosbridge');
    setPill(el.pillArm, 'bad', 'アーム');
  });

  ros.subscribe(TOPIC_STATE, STRING_TYPE, (msg) => {
    try { onState(JSON.parse(msg.data)); } catch (e) { /* 無視 */ }
  });
  ros.subscribe(TOPIC_LOG, STRING_TYPE, (msg) => {
    try {
      const entry = JSON.parse(msg.data);
      pushLog(entry.text, entry.level);
    } catch (e) { /* 無視 */ }
  });

  function sendCommand(obj) {
    const ok = ros.publish(TOPIC_COMMAND, STRING_TYPE, { data: JSON.stringify(obj) });
    if (!ok) pushLog('rosbridge 未接続のため送信できません', 'warn');
    return ok;
  }

  // ------------------------------------------------------- 遠隔からの操作権
  // 「手元」＝ LAN か VPN からの接続。フル操作でき、遠隔に許可を出せる。
  // 「遠隔」＝ トンネル経由。既定では閲覧のみ（非常停止だけは常に通る）。
  // ここでやっているのは表示の切り替えだけで、実際の遮断は ui_server 側。
  const isLocal = serverInfo.trusted !== false;
  let controlGranted = false;

  function renderControl(status) {
    controlGranted = !!(status && status.granted);
    if (isLocal) {
      // 手元の画面。遠隔から申請が来ているとき／許可中だけ帯を出す。
      const pending = (status && status.pending) || [];
      const show = controlGranted || pending.length > 0;
      el.alertGrant.hidden = !show;
      el.alertGrant.classList.toggle('is-granted', controlGranted);
      if (controlGranted) {
        el.grantTitle.textContent = '遠隔からの操作を許可中';
        el.grantDetail.textContent =
          `残り約${Math.round(status.remaining)}秒。この間、外から実機を動かせます。`;
      } else {
        el.grantTitle.textContent = '遠隔から操作の申請があります';
        el.grantDetail.textContent = `申請元: ${pending.join(', ')}`;
      }
      el.btnGrantControl.hidden = controlGranted;
      el.btnRevokeControl.hidden = !controlGranted;
      return;
    }
    // 遠隔の画面。常に帯を出して、いまどちらの状態か分かるようにする。
    el.alertRemote.hidden = false;
    el.alertRemote.classList.toggle('is-granted', controlGranted);
    document.body.classList.toggle('is-viewonly', !controlGranted);
    if (controlGranted) {
      el.remoteTitle.textContent = '操作できます';
      el.remoteDetail.textContent =
        `手元で許可されています（残り約${Math.round(status.remaining)}秒）。`;
      el.btnRequestControl.hidden = true;
    } else {
      el.remoteTitle.textContent = '閲覧のみ';
      el.remoteDetail.textContent = serverInfo.allow_remote_control === false
        ? 'この UI は遠隔からの操作を無効にして起動されています（停止だけは可能）。'
        : '別ネットワークから接続しています。操作するには手元で許可してもらってください（停止だけは常に可能）。';
      el.btnRequestControl.hidden = serverInfo.allow_remote_control === false;
    }
  }

  async function pollControl() {
    try {
      const res = await fetch('/api/control');
      if (res.ok) renderControl(await res.json());
    } catch (e) { /* 一時的な切断は無視 */ }
  }

  async function postControl(path, body) {
    try {
      const res = await fetch(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body || {}),
      });
      const data = await res.json();
      if (!res.ok) pushLog(data.error || '要求が拒否されました', 'warn');
      return res.ok;
    } catch (e) {
      pushLog('サーバへ届きませんでした', 'warn');
      return false;
    }
  }

  el.btnGrantControl.addEventListener('click', async () => {
    if (await postControl('/api/control/grant', { seconds: 600 })) {
      pushLog('遠隔からの操作を10分間 許可しました', 'warn');
    }
    pollControl();
  });
  el.btnRevokeControl.addEventListener('click', async () => {
    await postControl('/api/control/revoke');
    pushLog('遠隔からの操作許可を取り消しました', 'info');
    pollControl();
  });
  el.btnRequestControl.addEventListener('click', async () => {
    if (await postControl('/api/control/request')) {
      pushLog('手元の画面に操作の申請を出しました', 'info');
    }
  });

  renderControl(null);
  pollControl();
  setInterval(pollControl, 3000);

  // ---------------------------------------------------------------- 映像
  const video = new WebRtcVideo(
    ['arm', 'front', 'rviz'],
    { arm: el.videoArm, front: el.videoFront, rviz: el.videoRviz },
    (state) => {
      const ok = state === 'connected';
      setPill(el.pillVideo, ok ? 'ok' : (state === 'connecting' ? 'warn' : 'bad'), '映像');
      [el.hintArm, el.hintFront, el.hintRviz].forEach((h) => {
        if (h) h.classList.toggle('is-hidden', ok);
      });
    },
    serverInfo.ice_servers,
  );
  video.start();

  // 移動用UIのアームカメラは同じストリームを共有する
  el.videoArm.addEventListener('loadedmetadata', () => {
    if (el.videoArm2) el.videoArm2.srcObject = el.videoArm.srcObject;
    resizeOverlay();
  });

  // ---------------------------------------------------------------- 描画
  const armView = new ArmView(el.armViewCanvas);
  const robotMap = new RobotMap($('#robot-map'));

  // ------------------------------------------------- 座標指定（手先位置）
  // 入力欄は「いま編集中なら勝手に書き換えない」。状態は 10Hz で届くので、
  // 値を上書きし続けると数字を打ち込めなくなる。
  const AXES = [['X', 0, '前後'], ['Y', 1, '左右'], ['Z', 2, '上下']];
  const xyzInputs = [];
  let xyzTouched = false;

  AXES.forEach(([name, axis, note]) => {
    const wrap = document.createElement('div');
    wrap.className = 'xyz-axis';
    wrap.innerHTML = `<span class="xyz-name">${name} <small>${note}</small></span>`
      + `<input type="number" step="0.005" inputmode="decimal" aria-label="${name} [m]">`
      + '<div class="jog-btns"><button data-dir="-1">−</button>'
      + '<button data-dir="1">＋</button></div>';

    const input = wrap.querySelector('input');
    input.addEventListener('input', () => {
      xyzTouched = true;
      // 打ち直し始めたら、前のエラー表示は消す
      if (el.xyzHint.classList.contains('is-bad')) setXyzHint('');
    });
    xyzInputs.push(input);

    // ± は「いまの手先位置から相対で動かす」。入力欄の値ではなく
    // 実際の手先位置を基準にするので、押すたびに確実に 1 ステップ進む。
    wrap.querySelectorAll('button').forEach((b) => {
      const send = () => sendCommand({
        cmd: 'jog_xyz', axis,
        delta: Number(b.dataset.dir) * Number(el.xyzStep.value),
      });
      let timer = null;
      const start = (ev) => {
        ev.preventDefault();
        send();
        // 関節ジョグより間隔を広く取る。1回ごとに IK を解くうえ、
        // 連打すると目標が現在位置から離れていきやすい。
        timer = setInterval(send, 220);
      };
      const stop = () => { if (timer) { clearInterval(timer); timer = null; } };
      b.addEventListener('mousedown', start);
      b.addEventListener('touchstart', start, { passive: false });
      ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach((e) =>
        b.addEventListener(e, stop));
    });
    el.xyzGrid.appendChild(wrap);
  });

  function fillXyzFromState() {
    const tcp = latest && latest.tcp;
    if (!tcp) return false;
    tcp.forEach((v, i) => { xyzInputs[i].value = v.toFixed(3); });
    xyzTouched = false;
    return true;
  }

  el.btnXyzPick.addEventListener('click', () => {
    if (!fillXyzFromState()) pushLog('手先位置がまだ届いていません', 'warn');
  });

  el.btnXyzMove.addEventListener('click', () => {
    // 空欄を Number() に通すと 0 になってしまう（Number('') === 0）。
    // それを座標として送ると、入れ忘れた軸が勝手に 0 として扱われ、
    // 意図しない所へアームが動く。先に文字列として空かどうかを見る。
    const raw = xyzInputs.map((inp) => inp.value.trim());
    if (raw.some((s) => s === '')) {
      setXyzHint('X / Y / Z の3つとも入れてください（単位は m）', true);
      return;
    }
    const vals = raw.map(Number);
    if (vals.some((v) => !Number.isFinite(v))) {
      setXyzHint('X / Y / Z をメートルの数値で入れてください（例 0.25）', true);
      return;
    }
    setXyzHint('');
    sendCommand({ cmd: 'move_to_xyz', x: vals[0], y: vals[1], z: vals[2] });
  });

  // 関節角度指定と座標指定は「同じことをする別のやりかた」なので、
  // 片方を開いたらもう片方を閉じる。左下パネルは背が低く、両方開くと
  // 下のほうがスクロールしないと見えなくなるため。
  const jogToggles = $$('.panel-actions details.jog');
  jogToggles.forEach((d) => d.addEventListener('toggle', () => {
    if (!d.open) return;
    jogToggles.forEach((other) => { if (other !== d) other.open = false; });
  }));

  function setXyzHint(text, bad) {
    el.xyzHint.textContent = text || '';
    el.xyzHint.classList.toggle('is-bad', !!bad);
  }

  // 座標指定が使えるかどうかは状態に載ってくる（URDF が無ければ使えない）
  function renderXyz(state) {
    if (state.tcp_available === false) {
      el.xyzBox.classList.add('is-off');
      setXyzHint(state.tcp_reason || '座標指定は使えません', true);
      el.xyzCurrent.textContent = '—';
      return;
    }
    el.xyzBox.classList.remove('is-off');
    if (Array.isArray(state.tcp)) {
      el.xyzCurrent.textContent = state.tcp.map((v) => v.toFixed(3)).join(',  ') + '  m';
      // 初回だけ入力欄に現在値を入れておく（空欄から始まるより使いやすい）
      if (!xyzTouched && xyzInputs.every((inp) => inp.value === '')) fillXyzFromState();
    }
    if (!el.xyzHint.textContent && state.tcp_reach) {
      const r = state.tcp_reach;
      setXyzHint(`${state.tcp_link || '手先'} を基準にした base_link 座標 [m]。`
        + `目安の可動半径 約${r.max_radius}m。`
        + `1回の移動は ${Math.round((state.max_cartesian_step || 0.25) * 1000)}mm まで。`);
    }
  }

  // ---------------------------------------------------------------- 3D ビュー
  // URDF をブラウザで直接描く。形は起動時に一度取るだけで、
  // 動きは関節角のトピックだけで流すので、映像を送るより遥かに軽くて速い。
  let robotView = null;
  let robotReady = false;

  function setHint(text, isError) {
    el.robotHint.textContent = text || '';
    el.robotHint.classList.toggle('is-hidden', !text);
    el.robotHint.classList.toggle('is-error', !!isError);
  }

  (async () => {
    if (typeof THREE === 'undefined' || typeof RobotView === 'undefined') {
      setHint('3D ライブラリを読み込めません', true);
      switchViz('rviz');
      return;
    }
    try {
      robotView = new RobotView(el.robotView);
      robotView.interactive = el.chkRvizInput.checked;
      await robotView.load();
      robotReady = true;
      setHint('');
      pushLog('3D モデルを読み込みました', 'info');
    } catch (e) {
      console.warn('3D ビューを使えません', e);
      setHint('モデルを読み込めません（RViz に切り替えます）', true);
      pushLog('3D モデルを読み込めないため RViz 映像に切り替えます', 'warn');
      switchViz('rviz');
    }
  })();

  // 関節角の受信。実機のフィードバックがあればそちらを、無ければ指令を映す。
  const jointState = { cmd: null, fb: null, cmdAt: 0, fbAt: 0 };

  function jointsFromMsg(msg) {
    // piper のフィードバックはグリッパを 'gripper' という名前で出す（joint7 ではない）
    const out = {};
    const names = msg.name || [];
    const pos = msg.position || [];
    for (let i = 0; i < names.length && i < pos.length; i += 1) {
      const v = pos[i];
      if (typeof v !== 'number' || !isFinite(v)) continue;
      if (names[i] === 'gripper') {
        out.joint7 = v;
        out.joint8 = -v;
      } else {
        out[names[i]] = v;
      }
    }
    // joint8 が来ていなければ joint7 の反対側として補う（片側しか出さない実装向け）
    if (out.joint7 !== undefined && out.joint8 === undefined) out.joint8 = -out.joint7;
    return out;
  }

  ros.subscribe(TOPIC_JOINT_CMD, JOINT_STATE_TYPE, (msg) => {
    jointState.cmd = jointsFromMsg(msg);
    jointState.cmdAt = performance.now();
  });
  ros.subscribe(TOPIC_JOINT_FB, JOINT_STATE_TYPE, (msg) => {
    jointState.fb = jointsFromMsg(msg);
    jointState.fbAt = performance.now();
  });

  function pumpJoints() {
    if (robotView && robotReady) {
      const now = performance.now();
      const fbFresh = jointState.fb && (now - jointState.fbAt) < 1000;
      const cmdFresh = jointState.cmd && (now - jointState.cmdAt) < 1000;
      let src = null;
      let label = '—';
      if (fbFresh) { src = jointState.fb; label = '実機'; }
      else if (cmdFresh) { src = jointState.cmd; label = '指令'; }
      else if (latest && latest.command_joints) {
        // どちらのトピックも来ていないときは /arm_ui/state の指令値で動かす
        src = {};
        latest.command_joints.forEach((v, i) => { src['joint' + (i + 1)] = v; });
        if (typeof latest.gripper === 'number') {
          src.joint7 = latest.gripper;
          src.joint8 = -latest.gripper;
        }
        label = '指令';
      }
      if (src) robotView.setJoints(src);
      if (el.tagSource.textContent !== label) el.tagSource.textContent = label;
    }
    requestAnimationFrame(pumpJoints);
  }
  requestAnimationFrame(pumpJoints);

  // ---------------------------------------------------------------- RViz 操作
  // 映像上のマウス操作を正規化座標にして /rviz_ui/input へ流し、
  // rviz_bridge 側で XTEST を使って実際の rviz2 に注入する。
  // object-fit: contain の余白ぶんを差し引いて座標を合わせる。
  function rvizPoint(clientX, clientY) {
    const box = el.videoRviz.getBoundingClientRect();
    const vw = el.videoRviz.videoWidth || 16;
    const vh = el.videoRviz.videoHeight || 9;
    const scale = Math.min(box.width / vw, box.height / vh);
    const w = vw * scale;
    const h = vh * scale;
    const x = (clientX - box.left - (box.width - w) / 2) / w;
    const y = (clientY - box.top - (box.height - h) / 2) / h;
    if (x < 0 || x > 1 || y < 0 || y > 1) return null;
    return { x: Number(x.toFixed(4)), y: Number(y.toFixed(4)) };
  }

  let rvizDragging = false;
  let lastMoveSent = 0;

  function sendRvizInput(ev) {
    if (!el.chkRvizInput.checked) return;
    ros.publish(TOPIC_RVIZ_INPUT, STRING_TYPE, { data: JSON.stringify(ev) });
  }

  el.videoRviz.addEventListener('pointerdown', (e) => {
    if (!el.chkRvizInput.checked) return;
    const p = rvizPoint(e.clientX, e.clientY);
    if (!p) return;
    e.preventDefault();
    el.videoRviz.setPointerCapture(e.pointerId);
    rvizDragging = true;
    el.rvizWrap.classList.add('is-dragging');
    sendRvizInput({ type: 'down', button: e.button, ...p });
  });

  el.videoRviz.addEventListener('pointermove', (e) => {
    if (!rvizDragging) return;
    // XTEST への注入は 1 回あたり数ms かかるので 30Hz に間引く
    const now = performance.now();
    if (now - lastMoveSent < 33) return;
    lastMoveSent = now;
    const p = rvizPoint(e.clientX, e.clientY);
    if (p) sendRvizInput({ type: 'move', ...p });
  });

  ['pointerup', 'pointercancel'].forEach((name) => {
    el.videoRviz.addEventListener(name, (e) => {
      if (!rvizDragging) return;
      rvizDragging = false;
      el.rvizWrap.classList.remove('is-dragging');
      const p = rvizPoint(e.clientX, e.clientY) || {};
      sendRvizInput({ type: 'up', button: e.button, ...p });
    });
  });

  el.videoRviz.addEventListener('wheel', (e) => {
    if (!el.chkRvizInput.checked) return;
    const p = rvizPoint(e.clientX, e.clientY);
    if (!p) return;
    e.preventDefault();
    sendRvizInput({ type: 'wheel', delta: e.deltaY, ...p });
  }, { passive: false });

  // 右クリックメニューは RViz 操作の邪魔になるので抑制
  el.videoRviz.addEventListener('contextmenu', (e) => {
    if (el.chkRvizInput.checked) e.preventDefault();
  });

  el.chkRvizInput.addEventListener('change', () => {
    el.rvizWrap.classList.toggle('is-locked', !el.chkRvizInput.checked);
    if (robotView) robotView.interactive = el.chkRvizInput.checked;
  });

  // 3D / RViz の切替
  function switchViz(mode) {
    vizMode = mode;
    $$('#viz-mode .seg-btn').forEach((b) =>
      b.classList.toggle('is-active', b.dataset.viz === mode));
    el.robotView.hidden = mode !== 'robot';
    el.viewButtons.hidden = mode !== 'robot';
    el.robotHint.classList.toggle('is-hidden', mode !== 'robot' || robotReady);
    el.rvizWrap.hidden = mode !== 'rviz';
    el.armViewCanvas.hidden = mode !== 'simple';
    // 「視点操作」は 3D と RViz のどちらでも意味があるので出したままにする
    el.tagSource.hidden = mode !== 'robot';
    requestAnimationFrame(() => {
      if (mode === 'robot' && robotView) robotView._resize();
      if (mode === 'simple') armView._resize();
    });
  }

  $$('#viz-mode .seg-btn').forEach((btn) => {
    btn.addEventListener('click', () => switchViz(btn.dataset.viz));
  });

  // 視点プリセット
  $$('#view-buttons button').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (!robotView) return;
      if (btn.dataset.view === 'reset') robotView.resetView();
      else robotView.setView(btn.dataset.view);
    });
  });

  // ---------------------------------------------------------------- 二重起動の検出
  // シーケンサが2つ動いていると、状態が交互に届いて画面が高速に切り替わる
  // （「計画あり/なし」やボタンの有効・無効がチカチカする）。
  // 各メッセージに載っている送信元 id を数えて、2つ以上あれば名指しで知らせる。
  const senders = new Map();          // src -> 最後に受け取った時刻
  let dupReported = false;

  function trackSender(state) {
    const src = state.src;
    if (!src) return;                 // 古い版のシーケンサには src が無い
    const now = performance.now();
    senders.set(src, now);
    // 5秒以上来ていない送信元は数えない（止めた直後に警告が残らないように）
    for (const [key, seen] of senders) {
      if (now - seen > 5000) senders.delete(key);
    }

    const many = senders.size > 1;
    el.alertDup.hidden = !many;
    if (many) {
      el.alertDupDetail.textContent =
        `検出したノード: ${Array.from(senders.keys()).join(' / ')}。`;
      if (!dupReported) {
        dupReported = true;
        pushLog(`harvest_sequencer が ${senders.size} 個動いています`, 'error');
      }
    } else {
      dupReported = false;
    }
  }

  function onState(state) {
    trackSender(state);
    latest = state;
    detections = state.detections || [];

    setPill(el.pillArm, state.arm_connected ? 'ok' : 'warn', 'アーム');
    el.chkEnable.checked = !!state.enable_arm_command;

    // ステップ表示
    const step = STEP_ORDER[state.state] ?? 0;
    $$('#steps li').forEach((li) => {
      const idx = STEP_STATES.indexOf(li.dataset.state) + 1;
      li.classList.toggle('is-done', idx < step || state.state === 'DONE');
      li.classList.toggle('is-active', idx === step && state.state !== 'DONE');
    });

    // 状態バッジ
    el.stateBadge.textContent = state.state_label || state.state;
    el.stateBadge.className = 'state-badge'
      + (state.state === 'ERROR' ? ' is-error'
        : state.state === 'DONE' ? ' is-done'
          : state.progress !== null && state.progress !== undefined ? ' is-busy' : '');
    el.stateMessage.textContent = state.message || '';
    el.progressBar.style.width = `${Math.round((state.progress || 0) * 100)}%`;

    el.tagDetections.textContent = `検出 ${detections.length}`;
    el.tagPlan.textContent = state.grasp_plan ? '計画あり' : '計画なし';
    el.tagCount.textContent = `収穫 ${state.harvest_count || 0} 個`;

    renderXyz(state);

    // 収穫位置ボタンの選択表示
    $$('[data-cmd="goto_basket"]').forEach((b) => {
      b.classList.toggle('is-active', Number(b.dataset.index) === state.basket_index);
    });

    renderJoints(state);
    renderReadout(state);
    if (vizMode === 'simple') {
      armView.update(state.command_joints, state.grasp_plan, state.gripper_closed);
    }
    // 3D ビューは pumpJoints() が毎フレーム更新するのでここでは触らない
    robotMap.update(state.basket_index, (state.command_joints || [0])[0],
                    basketPositions(state));
    updateButtons(state);
    drawOverlay();
  }

  // 収穫位置 ①②③ の実座標を求める。
  // 3D ビューが使えるときは URDF の順運動学で厳密に、
  // 使えないときは joint1（左右の向き）と代表的な伸び量で近似する。
  // どちらにしても「関節角から求める」ので、姿勢を調整すれば図も追従する。
  let basketCache = { key: '', value: null };

  function basketPositions(state) {
    const poses = state.basket_poses;
    if (!Array.isArray(poses) || !poses.length) return null;
    const key = JSON.stringify(poses) + (robotReady ? '|fk' : '|approx');
    if (basketCache.key === key) return basketCache.value;

    const value = poses.map((q) => {
      if (robotView && robotReady) {
        const p = robotView.fkGripper(q);
        if (p) return { x: p.x, y: p.y };
      }
      // 近似: joint1 が左右の向き、伸び量は代表値
      const a = q[0] || 0;
      const r = 0.22;
      return { x: r * Math.cos(a), y: r * Math.sin(a) };
    });
    basketCache = { key, value };
    return value;
  }

  function renderJoints(state) {
    const cmd = state.command_joints || [];
    const fb = state.feedback_joints || [];
    if (el.jointTable.children.length !== 6) {
      el.jointTable.innerHTML = '';
      for (let i = 0; i < 6; i += 1) {
        const cell = document.createElement('div');
        cell.className = 'joint-cell';
        cell.innerHTML = `<span class="jn">J${i + 1}</span>`
          + '<span class="jv">0.00</span>'
          + '<div class="jbar"><span></span></div>';
        el.jointTable.appendChild(cell);
      }
    }
    Array.from(el.jointTable.children).forEach((cell, i) => {
      const v = cmd[i] ?? 0;
      const [lo, hi] = JOINT_LIMITS[i];
      cell.querySelector('.jv').textContent = v.toFixed(2);
      const ratio = Math.max(0, Math.min(1, (v - lo) / (hi - lo)));
      cell.querySelector('.jbar span').style.left = `calc(${ratio * 100}% - 1.5px)`;
      const err = fb.length === 6 ? Math.abs(fb[i] - v) : 0;
      cell.querySelector('.jv').style.color = err > 0.15 ? 'var(--warn)' : '';
    });
  }

  function renderReadout(state) {
    const fbAge = state.feedback_age;
    const rows = [
      ['シーケンス', state.state_label || state.state, ''],
      ['UI操作', state.enable_arm_command ? '有効' : '無効（監視のみ）',
        state.enable_arm_command ? 'v-warn' : ''],
      ['アーム接続', state.arm_connected ? '受信中' : '未受信',
        state.arm_connected ? 'v-ok' : 'v-bad'],
      ['フィードバック遅延', fbAge === null || fbAge === undefined ? '—' : `${fbAge.toFixed(2)} s`,
        fbAge > 0.5 ? 'v-warn' : ''],
      ['グリッパ', state.gripper_closed ? '閉（把持中）' : '開',
        state.gripper_closed ? 'v-warn' : ''],
      ['選択中の対象', state.selected_id === null || state.selected_id === undefined
        ? '—' : `#${state.selected_id}`, ''],
      ['収穫位置', state.basket_index ? `${state.basket_index}` : '—', ''],
      ['収穫数', `${state.harvest_count || 0} 個`, ''],
    ];
    // 座標指定の基準になる手先位置（指令値を順運動学にかけたもの）。
    // 下の「手先位置(実機)」は piper が出す end_pose で、基準点が違う
    // ことがあるので別の行にしてある（→ README「座標指定を実機に合わせる」）。
    if (Array.isArray(state.tcp)) {
      rows.push(['手先位置(指令)', state.tcp.map((v) => v.toFixed(3)).join(', '), '']);
    }
    if (state.end_pose && state.end_pose.x !== undefined) {
      rows.push(['手先位置(実機)',
        `${state.end_pose.x}, ${state.end_pose.y}, ${state.end_pose.z}`, '']);
    }
    const st = state.arm_status || {};
    if (st.err_code !== undefined) {
      rows.push(['エラーコード', String(st.err_code), st.err_code ? 'v-bad' : 'v-ok']);
    }
    if (st.angle_limit_joints) {
      rows.push(['可動限界', st.angle_limit_joints.length
        ? `J${st.angle_limit_joints.join(', J')}` : 'なし',
      st.angle_limit_joints.length ? 'v-warn' : 'v-ok']);
    }
    if (st.comm_error_joints) {
      rows.push(['関節通信', st.comm_error_joints.length
        ? `異常 J${st.comm_error_joints.join(', J')}` : '正常',
      st.comm_error_joints.length ? 'v-bad' : 'v-ok']);
    }
    el.readout.innerHTML = rows.map(([k, v, cls]) =>
      `<div><dt>${k}</dt><dd class="${cls}">${v}</dd></div>`).join('');
  }

  function updateButtons(state) {
    const s = state.state;
    const enable = (sel, cond) => $$(sel).forEach((b) => { b.disabled = !cond; });
    enable('[data-cmd="plan_grasp"]', s === 'SELECTED');
    enable('[data-cmd="grasp"]', s === 'PLANNED' || s === 'SELECTED');
    enable('[data-cmd="goto_basket"]', s === 'GRASPED' || s === 'AT_BASKET');
    enable('[data-cmd="release"]', s === 'AT_BASKET' || s === 'GRASPED');
  }

  // ---------------------------------------------------------------- オーバーレイ
  function resizeOverlay() {
    const rect = el.overlay.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    el.overlay.width = Math.max(1, Math.round(rect.width * dpr));
    el.overlay.height = Math.max(1, Math.round(rect.height * dpr));
    el.overlay.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
    drawOverlay();
  }
  window.addEventListener('resize', resizeOverlay);

  /** video の object-fit: contain を考慮した実描画領域 */
  function videoRect() {
    const box = el.overlay.getBoundingClientRect();
    const vw = el.videoArm.videoWidth || 4;
    const vh = el.videoArm.videoHeight || 3;
    const scale = Math.min(box.width / vw, box.height / vh);
    const w = vw * scale;
    const h = vh * scale;
    return { x: (box.width - w) / 2, y: (box.height - h) / 2, w, h };
  }

  function drawOverlay() {
    const ctx = el.overlay.getContext('2d');
    const box = el.overlay.getBoundingClientRect();
    ctx.clearRect(0, 0, box.width, box.height);
    if (!el.chkOverlay.checked) return;

    const r = videoRect();
    const selected = latest ? latest.selected_id : null;

    detections.forEach((d) => {
      const w = (d.w || 0.12) * r.w;
      const h = (d.h || 0.16) * r.h;
      const x = r.x + d.u * r.w - w / 2;
      const y = r.y + d.v * r.h - h / 2;
      const isSel = d.id === selected;
      const isHover = d.id === hoverId;

      ctx.lineWidth = isSel ? 4 : 2;
      ctx.strokeStyle = isSel ? '#ffd23f' : (isHover ? '#7fd1ff' : 'rgba(255,255,255,.75)');
      ctx.strokeRect(x, y, w, h);

      if (isSel) {
        ctx.strokeStyle = 'rgba(255,210,63,.85)';
        ctx.lineWidth = 1;
        const cx = x + w / 2;
        const cy = y + h / 2;
        ctx.beginPath();
        ctx.moveTo(cx - w, cy); ctx.lineTo(cx + w, cy);
        ctx.moveTo(cx, cy - h); ctx.lineTo(cx, cy + h);
        ctx.stroke();
      }

      const label = `#${d.id}  ${(d.distance ?? 0).toFixed(2)}m  ${Math.round((d.confidence ?? 0) * 100)}%`;
      ctx.font = '12px system-ui, sans-serif';
      const tw = ctx.measureText(label).width + 10;
      ctx.fillStyle = isSel ? 'rgba(255,210,63,.92)' : 'rgba(0,0,0,.62)';
      ctx.fillRect(x, y - 19, tw, 18);
      ctx.fillStyle = isSel ? '#1a1405' : '#fff';
      ctx.fillText(label, x + 5, y - 6);
    });
  }

  function hitTest(clientX, clientY) {
    const box = el.overlay.getBoundingClientRect();
    const r = videoRect();
    const px = clientX - box.left;
    const py = clientY - box.top;
    for (const d of detections) {
      const w = (d.w || 0.12) * r.w;
      const h = (d.h || 0.16) * r.h;
      const x = r.x + d.u * r.w - w / 2;
      const y = r.y + d.v * r.h - h / 2;
      if (px >= x && px <= x + w && py >= y && py <= y + h) return d;
    }
    return null;
  }

  el.overlay.addEventListener('mousemove', (e) => {
    const hit = hitTest(e.clientX, e.clientY);
    const id = hit ? hit.id : null;
    if (id !== hoverId) { hoverId = id; drawOverlay(); }
    el.overlay.style.cursor = hit ? 'pointer' : 'crosshair';
  });
  el.overlay.addEventListener('mouseleave', () => { hoverId = null; drawOverlay(); });
  el.overlay.addEventListener('click', (e) => {
    const hit = hitTest(e.clientX, e.clientY);
    if (hit) sendCommand({ cmd: 'select_target', id: hit.id });
  });
  el.chkOverlay.addEventListener('change', drawOverlay);

  // ---------------------------------------------------------------- 操作
  $$('.act[data-cmd]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const cmd = { cmd: btn.dataset.cmd };
      if (btn.dataset.index) cmd.index = Number(btn.dataset.index);
      sendCommand(cmd);
    });
  });

  $$('[data-gripper]').forEach((btn) => {
    btn.addEventListener('click', () =>
      sendCommand({ cmd: 'gripper', close: btn.dataset.gripper === 'close' }));
  });

  $('#btn-estop').addEventListener('click', () => {
    sendCommand({ cmd: 'abort' });
    ros.publish(TOPIC_DOG_MODE, STRING_TYPE, { data: 'stop' });
  });

  el.chkEnable.addEventListener('change', () =>
    sendCommand({ cmd: 'set_enable', value: el.chkEnable.checked }));

  // ジョグボタンを生成
  for (let i = 0; i < 6; i += 1) {
    const wrap = document.createElement('div');
    wrap.className = 'jog-joint';
    wrap.innerHTML = `<span class="jog-name">J${i + 1}</span>`
      + '<div class="jog-btns"><button data-dir="-1">−</button>'
      + '<button data-dir="1">＋</button></div>';
    wrap.querySelectorAll('button').forEach((b) => {
      const send = () => sendCommand({
        cmd: 'jog', joint: i,
        delta: Number(b.dataset.dir) * Number(el.jogStep.value),
      });
      // 長押しで連続ジョグ
      let timer = null;
      const start = (ev) => {
        ev.preventDefault();
        send();
        timer = setInterval(send, 120);
      };
      const stop = () => { if (timer) { clearInterval(timer); timer = null; } };
      b.addEventListener('mousedown', start);
      b.addEventListener('touchstart', start, { passive: false });
      ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach((e) =>
        b.addEventListener(e, stop));
    });
    el.jogGrid.appendChild(wrap);
  }

  // 走行モード（骨組み）
  $$('[data-mode]').forEach((btn) => {
    btn.addEventListener('click', () => {
      ros.publish(TOPIC_DOG_MODE, STRING_TYPE, { data: btn.dataset.mode });
      $$('[data-mode]').forEach((b) => b.classList.toggle('is-active-mode', b === btn));
      pushLog(`走行モード: ${btn.textContent}`, 'info');
    });
  });

  // ページ切替
  $$('.pager-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      $$('.pager-btn').forEach((b) => b.classList.toggle('is-active', b === btn));
      $$('.page').forEach((p) =>
        p.classList.toggle('is-active', p.id === `page-${btn.dataset.page}`));
      requestAnimationFrame(() => {
        resizeOverlay();
        if (vizMode === 'simple') armView._resize();
        robotMap._resize();
      });
    });
  });

  // キーボードショートカット（現場でゲームパッドが使えないとき用）
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const map = {
      1: { cmd: 'start_selection' }, 2: { cmd: 'plan_grasp' }, 3: { cmd: 'grasp' },
      4: { cmd: 'goto_basket', index: 1 }, 5: { cmd: 'release' },
      h: { cmd: 'home' }, ' ': { cmd: 'abort' },
    };
    const cmd = map[e.key];
    if (cmd) { e.preventDefault(); sendCommand(cmd); }
  });

  // ---------------------------------------------------------------- ログ
  function pushLog(text, level = 'info') {
    const time = new Date().toLocaleTimeString('ja-JP', { hour12: false });
    logs.unshift({ text, level, time });
    if (logs.length > 8) logs.pop();
    el.logList.innerHTML = logs
      .map((l) => `<li class="lv-${l.level}">${l.time} ${l.text}</li>`).join('');
  }

  function setPill(node, level, label) {
    node.classList.toggle('is-ok', level === 'ok');
    node.classList.toggle('is-warn', level === 'warn');
    node.innerHTML = `<i></i>${label}`;
  }

  // 初期化
  switchViz(vizMode);
  resizeOverlay();
  pushLog('UI を起動しました', 'info');
  setInterval(drawOverlay, 100);   // 映像サイズ変化に追従
})();
