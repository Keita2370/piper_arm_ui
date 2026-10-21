/*
 * 「RViz のような表示」パネル。
 * 外部ライブラリ（three.js など）に依存すると現場でオフラインのときに
 * 表示できないため、Canvas 2D で側面図＋上面図を描く簡易ビューにしている。
 * リンク長は PiPER のおおよその寸法。実機 URDF に合わせて LINKS を調整すること。
 */
const LINKS = {
  base: 0.123,   // ベース〜肩
  upper: 0.285,  // 肩〜肘
  fore: 0.250,   // 肘〜手首
  hand: 0.095,   // 手首〜手先
};

function themeColors() {
  const s = getComputedStyle(document.documentElement);
  const get = (name, fallback) => (s.getPropertyValue(name) || fallback).trim();
  return {
    grid: get('--viz-grid', '#22303f'),
    axis: get('--viz-axis', '#3d5a75'),
    link: get('--viz-link', '#7fd1ff'),
    linkDark: get('--viz-link-dark', '#2f7fa8'),
    ghost: get('--viz-ghost', '#f0b64a'),
    text: get('--viz-text', '#c8d8e4'),
    target: get('--viz-target', '#ffd23f'),
  };
}

class ArmView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.joints = [0, 0, 0, 0, 0, 0];
    this.ghost = null;
    this.gripperClosed = false;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = rect.width;
    this.h = rect.height;
    this.draw();
  }

  update(joints, ghost, gripperClosed) {
    if (joints) this.joints = joints;
    this.ghost = ghost || null;
    this.gripperClosed = !!gripperClosed;
    this.draw();
  }

  /** 側面図でのリンク端点（メートル）を返す */
  _chain(joints) {
    const [, j2, j3, , j5] = joints;
    const a2 = j2;              // 肩（鉛直から前方向へ）
    const a3 = a2 + j3;         // 肘
    const a5 = a3 + j5;         // 手首
    const pts = [{ x: 0, z: 0 }, { x: 0, z: LINKS.base }];
    const step = (prev, angle, len) => ({
      x: prev.x + len * Math.sin(angle),
      z: prev.z + len * Math.cos(angle),
    });
    pts.push(step(pts[1], a2, LINKS.upper));
    pts.push(step(pts[2], a3, LINKS.fore));
    pts.push(step(pts[3], a5, LINKS.hand));
    return pts;
  }

  draw() {
    const ctx = this.ctx;
    const c = themeColors();
    if (!this.w) return;
    ctx.clearRect(0, 0, this.w, this.h);

    const splitX = this.w * 0.62;
    this._drawSideView(ctx, c, 0, 0, splitX, this.h);
    this._drawTopView(ctx, c, splitX, 0, this.w - splitX, this.h);

    ctx.strokeStyle = c.axis;
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    ctx.moveTo(splitX, 8);
    ctx.lineTo(splitX, this.h - 8);
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  _drawSideView(ctx, c, x0, y0, w, h) {
    const scale = Math.min(w / 1.0, h / 0.95);
    const originX = x0 + w * 0.32;
    const originY = y0 + h * 0.82;
    const P = (p) => [originX + p.x * scale, originY - p.z * scale];

    // グリッド（0.1 m 刻み）
    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    for (let m = -4; m <= 8; m += 1) {
      const gx = originX + m * 0.1 * scale;
      ctx.globalAlpha = m === 0 ? 0.9 : 0.35;
      ctx.beginPath(); ctx.moveTo(gx, y0 + 6); ctx.lineTo(gx, originY); ctx.stroke();
    }
    for (let m = 0; m <= 8; m += 1) {
      const gy = originY - m * 0.1 * scale;
      ctx.globalAlpha = m === 0 ? 0.9 : 0.35;
      ctx.beginPath(); ctx.moveTo(x0 + 6, gy); ctx.lineTo(x0 + w - 6, gy); ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // 地面
    ctx.strokeStyle = c.axis;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x0 + 6, originY); ctx.lineTo(x0 + w - 6, originY); ctx.stroke();

    // 計画姿勢（ゴースト）
    if (this.ghost) {
      this._drawChain(ctx, this._chain(this.ghost).map(P), c.ghost, 5, 0.45);
    }
    // 現在姿勢
    const pts = this._chain(this.joints).map(P);
    this._drawChain(ctx, pts, c.link, 8, 1.0);

    // グリッパ
    const tip = pts[pts.length - 1];
    const prev = pts[pts.length - 2];
    const ang = Math.atan2(tip[1] - prev[1], tip[0] - prev[0]);
    const spread = this.gripperClosed ? 3 : 11;
    ctx.strokeStyle = this.gripperClosed ? c.target : c.link;
    ctx.lineWidth = 4;
    [-1, 1].forEach((s) => {
      ctx.beginPath();
      ctx.moveTo(tip[0], tip[1]);
      ctx.lineTo(tip[0] + Math.cos(ang) * 16 - Math.sin(ang) * spread * s,
                 tip[1] + Math.sin(ang) * 16 + Math.cos(ang) * spread * s);
      ctx.stroke();
    });

    ctx.fillStyle = c.text;
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText('側面図 (side)', x0 + 10, y0 + 18);
    ctx.fillText('grid = 0.1 m', x0 + 10, y0 + 32);
  }

  _drawChain(ctx, pts, color, width, alpha) {
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.stroke();
    ctx.fillStyle = color;
    pts.forEach((p, i) => {
      if (i === 0) return;
      ctx.beginPath();
      ctx.arc(p[0], p[1], width * 0.45, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.globalAlpha = 1;
  }

  _drawTopView(ctx, c, x0, y0, w, h) {
    const cx = x0 + w / 2;
    const cy = y0 + h * 0.62;
    const radius = Math.min(w, h) * 0.36;

    ctx.strokeStyle = c.grid;
    ctx.lineWidth = 1;
    [0.4, 0.7, 1.0].forEach((r) => {
      ctx.beginPath();
      ctx.arc(cx, cy, radius * r, Math.PI, Math.PI * 2);
      ctx.stroke();
    });
    ctx.strokeStyle = c.axis;
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - radius);
    ctx.stroke();

    // 可動範囲
    ctx.fillStyle = c.link;
    ctx.globalAlpha = 0.08;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, radius, -Math.PI / 2 - 2.618, -Math.PI / 2 + 2.618);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 1;

    // joint1 の向き
    const yaw = this.joints[0];
    ctx.strokeStyle = c.link;
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx - Math.sin(yaw) * radius * 0.92, cy - Math.cos(yaw) * radius * 0.92);
    ctx.stroke();
    ctx.fillStyle = c.linkDark;
    ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI * 2); ctx.fill();

    ctx.fillStyle = c.text;
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText('上面図 (top)', x0 + 10, y0 + 18);
    ctx.fillText(`J1 ${(yaw * 180 / Math.PI).toFixed(1)}°`, x0 + 10, y0 + 32);
  }
}

/* 「ロボットの状態（犬型を含む）」パネル: 上から見た犬型ロボットと収穫位置①②③ */
class RobotMap {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.activeBasket = null;
    this.yaw = 0;
    // 収穫位置の実座標（base_link 系 [m]）。update() で渡される。
    // 固定の画面座標で描くと、姿勢を調整したときに実際の位置とずれるため、
    // 関節角から求めた位置をそのまま俯瞰図に落とす。
    this.baskets = null;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * dpr));
    this.canvas.height = Math.max(1, Math.round(rect.height * dpr));
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = rect.width;
    this.h = rect.height;
    this.draw();
  }

  /**
   * @param activeBasket 選択中の収穫位置 (1..3)
   * @param yaw          joint1 の角度（搭載アームの向き）
   * @param baskets      収穫位置の実座標 [{x, y}, ...]（base_link 系 [m]）。
   *                     省略時は前回値を使う。
   */
  update(activeBasket, yaw, baskets) {
    this.activeBasket = activeBasket;
    this.yaw = yaw || 0;
    if (Array.isArray(baskets) && baskets.length) this.baskets = baskets;
    this.draw();
  }

  draw() {
    const ctx = this.ctx;
    const c = themeColors();
    if (!this.w) return;
    ctx.clearRect(0, 0, this.w, this.h);

    const cx = this.w / 2;
    const cy = this.h * 0.52;
    const bodyW = Math.min(this.w, this.h) * 0.20;
    const bodyH = bodyW * 1.9;

    // 犬型ロボット本体
    ctx.fillStyle = 'rgba(200,210,220,0.18)';
    ctx.strokeStyle = c.axis;
    ctx.lineWidth = 2;
    this._roundRect(ctx, cx - bodyW / 2, cy - bodyH / 2, bodyW, bodyH, 10);
    ctx.fill(); ctx.stroke();

    // 頭（進行方向）
    ctx.fillStyle = 'rgba(200,210,220,0.28)';
    ctx.beginPath();
    ctx.ellipse(cx, cy - bodyH / 2 - 6, bodyW * 0.34, bodyW * 0.3, 0, 0, Math.PI * 2);
    ctx.fill();

    // 搭載アーム（joint1 の向きで回る）
    ctx.save();
    ctx.translate(cx, cy - bodyH * 0.12);
    ctx.rotate(-this.yaw);
    ctx.fillStyle = c.link;
    this._roundRect(ctx, -bodyW * 0.16, -bodyW * 0.9, bodyW * 0.32, bodyW * 0.9, 5);
    ctx.fill();
    ctx.restore();

    // 収穫位置 ①②③ を、実際の関節角から求めた位置に描く。
    // 俯瞰図なので base_link の XY をそのまま画面に写す:
    //   +X（前方）= 画面の上   /   +Y（左）= 画面の左
    // アーム本体の描画（上向き・-yaw 回転）と同じ向きになる。
    const armX = cx;
    const armY = cy - bodyH * 0.12;      // アームの取り付け位置＝原点
    const marks = this.baskets || [];
    const reach = Math.max(0.05, ...marks.map((b) => Math.hypot(b.x, b.y)));
    // いちばん遠い収穫位置がパネルに収まるように倍率を決める
    const scale = (Math.min(this.w, this.h) * 0.34) / reach;

    const baskets = marks.map((b, i) => ({
      n: ['①', '②', '③'][i] || String(i + 1),
      x: armX - b.y * scale,
      y: armY - b.x * scale,
    }));
    baskets.forEach((b, i) => {
      const active = this.activeBasket === i + 1;
      ctx.beginPath();
      ctx.arc(b.x, b.y, 17, 0, Math.PI * 2);
      ctx.fillStyle = active ? c.target : 'rgba(120,140,160,0.18)';
      ctx.fill();
      ctx.strokeStyle = active ? c.target : c.axis;
      ctx.lineWidth = active ? 3 : 1.5;
      ctx.stroke();
      ctx.fillStyle = active ? '#1a2430' : c.text;
      ctx.font = '15px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(b.n, b.x, b.y + 1);
    });
    ctx.textAlign = 'start';
    ctx.textBaseline = 'alphabetic';
  }

  _roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }
}

window.ArmView = ArmView;
window.RobotMap = RobotMap;
