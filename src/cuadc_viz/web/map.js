// 场地地图视图。坐标系数字来自 docs/spec/field-layout.md /
// src/cuadc_sim/field/config.py——**这两个是唯一权威来源，这里是手抄的
// 第二份，field/config.py 改了这里要跟着改**（跨 Python/JS 语言边界，
// 没有更好的办法共享同一份常量，只能靠这条注释提醒）。

const FIELD_Y_MIN = -4.0, FIELD_Y_MAX = 4.0;
const LANDING_ZONE_X_MIN = -3.0;
const DROP_ZONE_X_MIN = 30.0, DROP_ZONE_X_MAX = 35.0;
const RECON_ZONE_X_MIN = 55.0, RECON_ZONE_X_MAX = 60.0;
const FIELD_X_MIN = LANDING_ZONE_X_MIN, FIELD_X_MAX = RECON_ZONE_X_MAX;
const LANDING_PAD_RADIUS = 0.4;

// 开发/联调阶段的已知固定坐标，仅供参照——正式比赛是随机摆放，这些点
// 不代表真实位置（field/config.py 自己的注释也这么写）。
const DROP_CYLINDERS_REF = [[31.0, -1.5], [32.5, 1.5], [34.0, 0.0]];
const RECON_CYLINDERS_REF = [[55.8, -2.5], [56.6, 1.8], [57.5, -0.3], [58.3, 2.5], [59.2, -1.8]];

// telemetry/local_position、Detection.position 都是 NED(north,east)，跟场地
// 坐标 (x=推进方向, y=侧向) 不是恒等映射——mission_logic.py
// `_world_xy_to_ned` 定义的是 (north,east)=(field_y,field_x)，这里用它的
// 反变换：field_x = ned_east, field_y = ned_north。
export function nedToField(north, east) { return [east, north]; }

export class FieldMap {
  constructor(canvas) {
    this.canvas = canvas;
    this.vehicle = null; // {x, y, yawRad}
    this.trail = [];
    this.detections = []; // [{x, y, confidence}]
    this.showReference = true; // 开发环境已知筒位参照，见 setShowReference
    const style = getComputedStyle(document.documentElement);
    this.ink = style.getPropertyValue('--text-secondary').trim();
    this.muted = style.getPropertyValue('--text-muted').trim();
    this.grid = style.getPropertyValue('--gridline').trim();
    this.vehicleColor = style.getPropertyValue('--series-1').trim();
    this.detColor = style.getPropertyValue('--series-2').trim();
    // 静态场地轮廓不依赖任何飞行数据——之前只在 setVehicle/setDetections
    // 第一次调用时才画，导致仿真/任务节点起来之前地图一直是空白的，
    // 场地本身应该一直画着。
    this._draw();
  }

  setShowReference(show) { this.showReference = show; this._draw(); }

  setVehicle(fieldX, fieldY, yawRad) {
    this.vehicle = { x: fieldX, y: fieldY, yaw: yawRad };
    this.trail.push([fieldX, fieldY]);
    if (this.trail.length > 20000) this.trail.shift(); // 地面端算力不是瓶颈，留够长的历史
    this._draw();
  }

  setDetections(list) { this.detections = list; this._draw(); }

  _transform(w, h) {
    const marginPx = 20;
    const labelPx = 16; // 顶部留给分区名字标注的高度，不挤占场地本身
    const sx = (w - 2 * marginPx) / (FIELD_X_MAX - FIELD_X_MIN);
    const sy = (h - 2 * marginPx - labelPx) / (FIELD_Y_MAX - FIELD_Y_MIN);
    const s = Math.min(sx, sy);
    // 场地是一条很窄很长的带子（约 63m x 8m），画出来的宽高比跟 canvas
    // 容器的宽高比几乎不可能一致，按较紧的那个轴定 scale 之后，另一个轴
    // 必然有富余——居中而不是顶格画，不然富余的空间会全部堆在一侧看起来
    // 像是"漏了什么没画"。
    const renderedW = (FIELD_X_MAX - FIELD_X_MIN) * s;
    const renderedH = (FIELD_Y_MAX - FIELD_Y_MIN) * s;
    const offsetX = (w - renderedW) / 2;
    const offsetY = labelPx + (h - labelPx - renderedH) / 2;
    const toPx = (fx, fy) => [
      offsetX + (fx - FIELD_X_MIN) * s,
      offsetY + (FIELD_Y_MAX - fy) * s, // 场地 +y 画在屏幕上方
    ];
    toPx.scale = s;
    toPx.offsetY = offsetY;
    return toPx;
  }

  _draw() {
    const canvas = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr; canvas.height = h * dpr;
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const toPx = this._transform(w, h);

    // 静态图层：场地分区轮廓 + 名字标注（标注画在框上方留出来的那条
    // labelPx 空间里，不是凭空加高度，见 _transform）
    ctx.strokeStyle = this.grid;
    ctx.lineWidth = 1;
    ctx.fillStyle = this.muted;
    ctx.font = '10px system-ui, -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    const rect = (x0, x1, label) => {
      const [px0, py0] = toPx(x0, FIELD_Y_MAX);
      const [px1, py1] = toPx(x1, FIELD_Y_MIN);
      ctx.strokeRect(px0, py0, px1 - px0, py1 - py0);
      ctx.fillText(label, (px0 + px1) / 2, py0 - 3);
    };
    rect(LANDING_ZONE_X_MIN, DROP_ZONE_X_MIN, '起降');
    rect(DROP_ZONE_X_MIN, DROP_ZONE_X_MAX, '投放区');
    rect(DROP_ZONE_X_MAX, RECON_ZONE_X_MIN, '过渡');
    rect(RECON_ZONE_X_MIN, RECON_ZONE_X_MAX, '侦察区');

    // 起降点
    const [padX, padY] = toPx(0, 0);
    const padR = LANDING_PAD_RADIUS * (toPx(1, 0)[0] - toPx(0, 0)[0]);
    ctx.beginPath();
    ctx.arc(padX, padY, Math.abs(padR), 0, Math.PI * 2);
    ctx.stroke();

    // 参照筒位（半透明，明确标注仅供参考）——只在开发/仿真环境有意义，
    // 正式比赛筒是随机摆放的，这几个点跟真实位置无关，真机场景下必须能
    // 关掉，不然会被误当成真实信息（呼应地面端计算的诚实原则），见
    // setShowReference/app.js 里的开关。
    if (this.showReference) {
      ctx.fillStyle = this.muted;
      ctx.globalAlpha = 0.35;
      [...DROP_CYLINDERS_REF, ...RECON_CYLINDERS_REF].forEach(([fx, fy]) => {
        const [px, py] = toPx(fx, fy);
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);
        ctx.fill();
      });
      ctx.globalAlpha = 1;
    }

    // 飞机轨迹
    if (this.trail.length > 1) {
      ctx.strokeStyle = this.vehicleColor;
      ctx.globalAlpha = 0.4;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      this.trail.forEach(([fx, fy], i) => {
        const [px, py] = toPx(fx, fy);
        i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // 检测原始散点——不做跨帧聚类/平滑（诚实原则），如实反映每一帧的输出
    ctx.fillStyle = this.detColor;
    this.detections.forEach((d) => {
      const [px, py] = toPx(d.x, d.y);
      ctx.globalAlpha = Math.max(0.25, Math.min(1, d.confidence));
      ctx.beginPath();
      ctx.arc(px, py, 4, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.globalAlpha = 1;

    // 飞机位置 + 朝向
    if (this.vehicle) {
      const [px, py] = toPx(this.vehicle.x, this.vehicle.y);
      const dx = Math.sin(this.vehicle.yaw), dy = Math.cos(this.vehicle.yaw); // NED yaw -> field 方向
      ctx.strokeStyle = this.vehicleColor;
      ctx.fillStyle = this.vehicleColor;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + dx * 16, py - dy * 16);
      ctx.stroke();
    }
  }
}
