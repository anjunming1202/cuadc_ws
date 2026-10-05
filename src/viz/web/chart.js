// 极简滚动折线图：单序列、2px 线、hairline 网格，按 dataviz skill 的
// mark spec（references/marks-and-anatomy.md）——不做任何平滑/插值，
// 逐点原始连线，呼应设计文档里"地面端不能悄悄处理数据"的原则。

const WINDOW_SEC = 60; // 只保留最近 60s，够看趋势，不需要无限增长的数组

export class RollingChart {
  constructor(canvas, { color = '#2a78d6' } = {}) {
    this.canvas = canvas;
    this.color = color;
    this.points = []; // [{t, v}]
    const style = getComputedStyle(document.documentElement);
    this.gridColor = style.getPropertyValue('--gridline').trim() || '#e1e0d9';
    this.labelColor = style.getPropertyValue('--text-muted').trim() || '#898781';
  }

  push(value, tSec = performance.now() / 1000) {
    this.points.push({ t: tSec, v: value });
    const cutoff = tSec - WINDOW_SEC;
    while (this.points.length && this.points[0].t < cutoff) this.points.shift();
    this._draw();
  }

  _draw() {
    const canvas = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    if (this.points.length < 2) return;

    const vs = this.points.map((p) => p.v);
    let vMin = Math.min(...vs), vMax = Math.max(...vs);
    if (vMin === vMax) { vMin -= 1; vMax += 1; }
    const pad = (vMax - vMin) * 0.1;
    vMin -= pad; vMax += pad;
    const tNow = this.points[this.points.length - 1].t;
    const tOld = tNow - WINDOW_SEC;
    const valueToY = (v) => h - ((v - vMin) / (vMax - vMin)) * h;

    // y 轴刻度：4 条等分线（含首尾），每条都标数值——之前只有一条底部
    // 基线加两个角上的数字，量级能读但刻度太稀；分成 4 条本身也是网格线
    // （recessive，细、低对比度），不是额外画的东西。
    const fmt = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1));
    const TICKS = 4;
    ctx.strokeStyle = this.gridColor;
    ctx.lineWidth = 1;
    ctx.fillStyle = this.labelColor;
    ctx.font = '10px system-ui, -apple-system, sans-serif';
    ctx.textAlign = 'left';
    for (let i = 0; i <= TICKS; i++) {
      const v = vMin + (vMax - vMin) * (i / TICKS);
      const y = valueToY(v);
      ctx.beginPath();
      ctx.moveTo(0, Math.round(y) - 0.5);
      ctx.lineTo(w, Math.round(y) - 0.5);
      ctx.stroke();
      ctx.textBaseline = i === 0 ? 'bottom' : i === TICKS ? 'top' : 'middle';
      ctx.fillText(fmt(v), 3, y);
    }

    ctx.strokeStyle = this.color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    this.points.forEach((p, i) => {
      const x = ((p.t - tOld) / (tNow - tOld || 1)) * w;
      const y = h - ((p.v - vMin) / (vMax - vMin)) * h;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}
