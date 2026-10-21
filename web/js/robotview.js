/*
 * URDF をブラウザで直接描く 3D ビュー。
 *
 * RViz の画面を動画で受け取る代わりに、
 *   - 形（URDF とメッシュ）は起動時に一度だけ /api/robot, /api/mesh から取る
 *   - 動き（関節角）は rosbridge で流し続ける
 * という分け方にしている。送るのが 1 回 0.8MB ＋ 毎秒数百バイトで済むので、
 * 遅延はほぼ無く、解像度は画面いっぱい、フレームレートは 60fps 出る。
 *
 * 視点操作は RViz と同じ感覚に合わせて自前で実装している
 * （左ドラッグ=回転 / 中ドラッグ・右ドラッグ=平行移動 / ホイール=ズーム）。
 */
/* global THREE */

class RobotView {
  constructor(canvas) {
    this.canvas = canvas;
    this.joints = new Map();      // 関節名 -> { obj, axis, type, lower, upper }
    this.target = new Map();      // 関節名 -> 目標角
    this.current = new Map();     // 関節名 -> 表示中の角
    this.ready = false;
    this.ghost = null;            // 実機フィードバックの重ね表示
    this.showGhost = false;
    this._raf = null;
    this._lastTick = 0;
    // 何も動いていないときは描き直さない。止まっている間の CPU/GPU をほぼ 0 にする
    // （タブレットの電池と、GPU の無い環境での重さに効く）。
    this._dirty = true;

    this._initScene();
  }

  // ------------------------------------------------------------------ 場面
  _initScene() {
    const canvas = this.canvas;
    this.renderer = new THREE.WebGLRenderer({
      canvas, antialias: true, alpha: false,
    });
    this.renderer.setClearColor(0x0c141d, 1);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();

    // ROS は Z が上。three.js は Y が上なので、全体を Z-up として扱う。
    this.world = new THREE.Group();
    this.world.rotation.x = -Math.PI / 2;
    this.scene.add(this.world);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);

    // 視点（球座標）。既定はアームの後ろから＝RViz 側の既定と揃える。
    // アームは高さ 0.4m 程度。パネルいっぱいに写るよう寄せておく。
    this.viewTarget = new THREE.Vector3(0.02, 0.0, 0.22);
    this.yaw = Math.PI;
    this.pitch = 0.30;
    this.distance = 0.72;
    this._applyCamera();

    // 照明: 上からの主光 + 補助 + 環境光
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(1.2, 1.6, 2.2);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 8;
    const span = 1.2;
    Object.assign(key.shadow.camera, {
      left: -span, right: span, top: span, bottom: -span,
    });
    key.shadow.camera.updateProjectionMatrix();
    this.scene.add(key);

    const fill = new THREE.DirectionalLight(0x88aacc, 0.7);
    fill.position.set(-1.5, 0.8, -1.0);
    this.scene.add(fill);
    this.scene.add(new THREE.HemisphereLight(0xbcd6f0, 0x1a2430, 1.0));

    // 床（影を受けるだけの面）とグリッド
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(4, 4),
      new THREE.ShadowMaterial({ opacity: 0.35 }),
    );
    floor.receiveShadow = true;
    this.world.add(floor);

    const grid = new THREE.GridHelper(1.6, 16, 0x35506a, 0x243444);
    grid.rotation.x = Math.PI / 2;      // world が Z-up なので戻す
    grid.position.z = 0.001;
    this.world.add(grid);

    this.robot = new THREE.Group();
    this.world.add(this.robot);

    this._bindPointer();
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _applyCamera() {
    this._dirty = true;
    const cp = Math.cos(this.pitch);
    // rviz の Orbit カメラと同じ式にして、見え方を揃える
    const x = this.distance * Math.cos(this.yaw) * cp + this.viewTarget.x;
    const y = this.distance * Math.sin(this.yaw) * cp + this.viewTarget.y;
    const z = this.distance * Math.sin(this.pitch) + this.viewTarget.z;
    // world 側で Z-up にしているので、カメラには three の座標系で渡す
    this.camera.position.set(x, z, -y);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(this.viewTarget.x, this.viewTarget.z, -this.viewTarget.y);
  }

  // ------------------------------------------------------------------ 操作
  _bindPointer() {
    const el = this.canvas;
    let mode = null;
    let lastX = 0;
    let lastY = 0;

    el.addEventListener('pointerdown', (e) => {
      if (!this.interactive) return;
      el.setPointerCapture(e.pointerId);
      mode = (e.button === 0 && !e.shiftKey) ? 'orbit' : 'pan';
      lastX = e.clientX; lastY = e.clientY;
      el.classList.add('is-dragging');
      e.preventDefault();
    });

    el.addEventListener('pointermove', (e) => {
      if (!mode) return;
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
      if (mode === 'orbit') {
        this.yaw -= dx * 0.008;
        this.pitch = Math.max(-1.45, Math.min(1.45, this.pitch + dy * 0.008));
      } else {
        // 画面の上下左右を、カメラから見た平面上の移動に変換する
        const s = this.distance * 0.0016;
        const right = new THREE.Vector3(Math.sin(this.yaw), -Math.cos(this.yaw), 0);
        this.viewTarget.addScaledVector(right, dx * s);
        this.viewTarget.z += dy * s;
      }
      this._applyCamera();
    });

    ['pointerup', 'pointercancel', 'pointerleave'].forEach((name) => {
      el.addEventListener(name, () => {
        mode = null;
        el.classList.remove('is-dragging');
      });
    });

    el.addEventListener('wheel', (e) => {
      if (!this.interactive) return;
      e.preventDefault();
      const k = Math.exp(e.deltaY * 0.0012);
      this.distance = Math.max(0.25, Math.min(6, this.distance * k));
      this._applyCamera();
    }, { passive: false });

    el.addEventListener('contextmenu', (e) => {
      if (this.interactive) e.preventDefault();
    });
  }

  set interactive(v) { this._interactive = v; }

  get interactive() { return this._interactive !== false; }

  /** 視点を既定に戻す */
  resetView() {
    this.viewTarget.set(0.02, 0, 0.22);
    this.yaw = Math.PI;
    this.pitch = 0.30;
    this.distance = 0.72;
    this._applyCamera();
  }

  /** 決まった向きへ（rviz の yaw と同じ意味） */
  setView(name) {
    const yaws = { back: Math.PI, front: 0, left: Math.PI / 2, right: -Math.PI / 2 };
    if (name in yaws) {
      this.yaw = yaws[name];
      this._applyCamera();
    }
  }

  // ------------------------------------------------------------------ 読み込み
  /** /api/robot と /api/mesh からロボットを組み立てる */
  async load() {
    const res = await fetch('/api/robot');
    if (!res.ok) throw new Error('/api/robot が ' + res.status);
    const def = await res.json();
    if (!def.available) throw new Error(def.reason || '3D ビューを使えません');

    // メッシュはリンクごとに並列で取る（1つ失敗しても他は出す）
    const ids = new Set();
    Object.values(def.links).forEach((l) =>
      (l.visuals || []).forEach((v) => ids.add(v.mesh)));
    const geometries = new Map();
    await Promise.all(Array.from(ids).map(async (id) => {
      try {
        geometries.set(id, await this._fetchGeometry(def.mesh_url + id));
      } catch (e) {
        console.warn('メッシュを読めません', id, e);
      }
    }));

    // リンクを Object3D にする
    const objects = new Map();
    Object.entries(def.links).forEach(([name, link]) => {
      const group = new THREE.Group();
      group.name = name;
      (link.visuals || []).forEach((v) => {
        const geom = geometries.get(v.mesh);
        if (!geom) return;
        const c = v.color || [0.8, 0.82, 0.93, 1];
        const mat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(c[0], c[1], c[2]),
          metalness: 0.25,
          roughness: 0.55,
          flatShading: true,
        });
        const mesh = new THREE.Mesh(geom, mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.position.set(v.xyz[0], v.xyz[1], v.xyz[2]);
        mesh.rotation.set(v.rpy[0], v.rpy[1], v.rpy[2], 'ZYX');
        if (v.scale && v.scale.some((s) => s !== 1)) {
          mesh.scale.set(v.scale[0], v.scale[1], v.scale[2]);
        }
        group.add(mesh);
      });
      objects.set(name, group);
    });

    // 関節でつないで階層にする。
    // 併せて「子リンク -> その親関節」を控えておく。表示を動かさずに
    // 任意の関節角で順運動学を解きたい場面（収穫位置の俯瞰表示）で使う。
    this.linkParentJoint = new Map();
    def.joints.forEach((j) => {
      this.linkParentJoint.set(j.child, j);
      const parent = objects.get(j.parent);
      const child = objects.get(j.child);
      if (!parent || !child) return;
      // 関節の原点に置いた入れ物を作り、その中で子リンクを回す
      const pivot = new THREE.Group();
      pivot.name = 'joint:' + j.name;
      pivot.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
      // URDF の rpy は固定軸 X→Y→Z の順。three.js では 'ZYX' 指定が対応する。
      pivot.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
      pivot.add(child);
      parent.add(pivot);

      if (j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic') {
        const axis = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize();
        this.joints.set(j.name, {
          obj: child, type: j.type, axis,
          base: child.position.clone(),
          lower: j.lower, upper: j.upper,
        });
        this.target.set(j.name, 0);
        this.current.set(j.name, 0);
      }
    });

    const root = objects.get(def.root);
    if (!root) throw new Error('根元のリンクが見つかりません: ' + def.root);
    this.robot.add(root);
    this.def = def;
    this.ready = true;
    this._apply(true);
    this.start();
    return def;
  }

  async _fetchGeometry(url) {
    const res = await fetch(url);
    if (!res.ok) throw new Error(url + ' が ' + res.status);
    const buf = await res.arrayBuffer();
    const head = new DataView(buf);
    // 'PARM' + version + 頂点数 + 索引数
    const magic = String.fromCharCode(
      head.getUint8(0), head.getUint8(1), head.getUint8(2), head.getUint8(3));
    if (magic !== 'PARM') throw new Error('形式が違います: ' + magic);
    const vcount = head.getUint32(8, true);
    const icount = head.getUint32(12, true);
    const posBytes = vcount * 12;
    const positions = new Float32Array(buf, 16, vcount * 3);
    const indices = new Uint32Array(buf, 16 + posBytes, icount);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setIndex(new THREE.BufferAttribute(indices, 1));
    geom.computeVertexNormals();
    geom.computeBoundingSphere();
    return geom;
  }

  // ------------------------------------------------------------------ 順運動学
  /** 根元から linkName までの関節列 */
  _linkChain(linkName) {
    const chain = [];
    let cur = linkName;
    const guard = 64;
    for (let i = 0; i < guard && this.linkParentJoint.has(cur); i += 1) {
      const j = this.linkParentJoint.get(cur);
      chain.unshift(j);
      cur = j.parent;
    }
    return chain;
  }

  /**
   * 任意の関節角における link の位置を base_link 座標で返す。
   * 表示中の姿勢には触らないので、収穫位置の俯瞰図など
   * 「今とは別の姿勢」を計算したいときに使える。
   */
  fkLink(linkName, values) {
    if (!this.linkParentJoint) return null;
    const T = new THREE.Matrix4();
    const step = new THREE.Matrix4();
    const quat = new THREE.Quaternion();
    const euler = new THREE.Euler();
    const pos = new THREE.Vector3();
    const one = new THREE.Vector3(1, 1, 1);
    const axis = new THREE.Vector3();

    this._linkChain(linkName).forEach((j) => {
      // 関節の原点（シーンの組み立てと同じ規約: URDF の rpy は 'ZYX'）
      euler.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
      quat.setFromEuler(euler);
      pos.set(j.xyz[0], j.xyz[1], j.xyz[2]);
      step.compose(pos, quat, one);
      T.multiply(step);

      const v = values ? values[j.name] : undefined;
      if (typeof v !== 'number' || !isFinite(v)) return;
      axis.set(j.axis[0], j.axis[1], j.axis[2]).normalize();
      if (j.type === 'prismatic') {
        step.makeTranslation(axis.x * v, axis.y * v, axis.z * v);
      } else if (j.type === 'revolute' || j.type === 'continuous') {
        quat.setFromAxisAngle(axis, v);
        step.makeRotationFromQuaternion(quat);
      } else {
        return;
      }
      T.multiply(step);
    });
    return new THREE.Vector3().setFromMatrixPosition(T);
  }

  /** joint1..6 の配列から手先位置を求める（収穫位置の俯瞰表示用） */
  fkGripper(jointArray) {
    if (!Array.isArray(jointArray)) return null;
    const values = {};
    jointArray.forEach((v, i) => { values['joint' + (i + 1)] = v; });
    return this.fkLink('gripper_base', values);
  }

  // ------------------------------------------------------------------ 関節
  /** 関節名 -> 角度[rad] / 変位[m] の連想配列を渡す */
  setJoints(values) {
    if (!values) return;
    Object.entries(values).forEach(([name, v]) => {
      if (!this.joints.has(name) || typeof v !== 'number' || !isFinite(v)) return;
      const j = this.joints.get(name);
      let x = v;
      if (typeof j.lower === 'number' && typeof j.upper === 'number') {
        x = Math.max(j.lower, Math.min(j.upper, x));
      }
      if (Math.abs((this.target.get(name) || 0) - x) > 1e-6) this._dirty = true;
      this.target.set(name, x);
    });
  }

  _apply(immediate) {
    // 受信は 10〜50Hz なので、描画側で目標へ滑らかに寄せる。
    // そのままだと段階的に見えるが、補間すると実機の動きらしく見える。
    const now = performance.now();
    const dt = immediate ? 1 : Math.min(0.1, (now - this._lastTick) / 1000);
    this._lastTick = now;
    // 時間に対して指数的に近づける（フレームレートに依らない）
    const k = immediate ? 1 : 1 - Math.exp(-dt * 18);

    let moving = false;
    this.joints.forEach((j, name) => {
      const goal = this.target.get(name) || 0;
      const cur = this.current.get(name) || 0;
      const diff = goal - cur;
      // 目標にほぼ着いたら止める（微小差でずっと描き続けないように）
      if (Math.abs(diff) < 1e-5) {
        if (cur !== goal) {
          this.current.set(name, goal);
          this._setJointObject(j, goal);
          this._dirty = true;
        }
        return;
      }
      moving = true;
      const next = cur + diff * k;
      this.current.set(name, next);
      this._setJointObject(j, next);
    });
    if (moving) this._dirty = true;
  }

  _setJointObject(j, value) {
    if (j.type === 'prismatic') {
      j.obj.position.copy(j.base).addScaledVector(j.axis, value);
    } else {
      j.obj.quaternion.setFromAxisAngle(j.axis, value);
    }
  }

  // ------------------------------------------------------------------ 描画
  _resize() {
    this._dirty = true;
    const rect = this.canvas.getBoundingClientRect();
    const w = Math.max(1, Math.round(rect.width));
    const h = Math.max(1, Math.round(rect.height));
    // 端末の画素密度に合わせる（上限 2 で、4K タブレットでも重くしない）
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  start() {
    if (this._raf !== null) return;
    const loop = () => {
      this._raf = requestAnimationFrame(loop);
      if (!this.ready) return;
      const rect = this.canvas.getBoundingClientRect();
      if (rect.width < 2 || rect.height < 2) return;    // 非表示中は描かない
      if (Math.abs(this.canvas.clientWidth - rect.width) > 1) this._resize();
      this._apply(false);
      // 変化があったフレームだけ描く
      if (!this._dirty) return;
      this._dirty = false;
      this.renderer.render(this.scene, this.camera);
    };
    this._lastTick = performance.now();
    loop();
  }

  stop() {
    if (this._raf !== null) {
      cancelAnimationFrame(this._raf);
      this._raf = null;
    }
  }
}

window.RobotView = RobotView;
