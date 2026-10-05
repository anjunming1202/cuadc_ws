/* DroneLink 赛道平面图 + 地面估算机位。尺寸同 docs/spec/field-layout.md。
 * 位姿由 /telemetry.json 的 field_* 字段提供（server 里 field_pose.py 算）。
 */
(function (global) {
  "use strict";

  const FIELD = {
    prepDepth: 5, // 准备区深度规则未定，仅示意
    xPrep: -3,
    xH: 0,
    xLine: 2,
    xDrop0: 30,
    xDrop1: 35,
    xRecon0: 55,
    xRecon1: 60,
    halfW: 4,
    hR: 0.4,
  };

  function svgY(fieldY) {
    return -fieldY; // 屏幕向上 = 赛道 +y（机头朝 +x 时的左侧）
  }

  function mount(el) {
    // 赛道 SVG 已写在 index.html 里，这样不跑 JS、没有飞控也能看见。
    return {
      setCraft: function (pose) {
        const g = el.querySelector("#fieldCraft");
        const lab = el.querySelector("#fieldCraftLabel");
        if (!g || !lab) {
          return;
        }
        const x = typeof pose.x === "number" ? pose.x : 0;
        const y = typeof pose.y === "number" ? pose.y : 0;
        const yaw = typeof pose.yawDeg === "number" ? pose.yawDeg : 0;
        // SVG 绕 +z 顺时针为正；赛道机头偏航与之一致（+x 为 0°）。
        g.setAttribute("transform", "translate(" + x + "," + svgY(y) + ") rotate(" + yaw + ")");
        lab.setAttribute("x", String(x));
        lab.setAttribute("y", String(svgY(y) - 1.35));
        lab.textContent = pose.label || "";
        g.style.opacity = "1";
      },
    };
  }

  function mountAlt(el) {
    // 最近 30s 高度；无采样也保留坐标轴，不依赖飞控是否在线。
    const W = 320, H = 96, L = 32, R = 10, T = 12, B = 22;
    const plotW = W - L - R, plotH = H - T - B;
    el.innerHTML = `<svg viewBox="0 0 ${W} ${H}" width="100%" preserveAspectRatio="none" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="最近30秒高度">
      <rect x="0" y="0" width="${W}" height="${H}" fill="#12171d"/>
      <rect x="${L}" y="${T}" width="${plotW}" height="${plotH}" fill="#1a1f26" stroke="#2a3340" stroke-width="0.6"/>
      <polyline id="altLine" fill="none" stroke="#3d9cf0" stroke-width="1.4" stroke-linejoin="round"/>
      <text id="altMaxLab" x="${L - 3}" y="${T + 4}" text-anchor="end" fill="#8b95a5" font-size="7" font-family="Consolas, monospace">—</text>
      <text id="altMinLab" x="${L - 3}" y="${T + plotH}" text-anchor="end" fill="#8b95a5" font-size="7" font-family="Consolas, monospace">0</text>
      <text id="altNowLab" x="${W - R}" y="${T - 2}" text-anchor="end" fill="#3d9cf0" font-size="8" font-family="Consolas, monospace"></text>
      <text x="${L}" y="${H - 6}" fill="#8b95a5" font-size="7" font-family="Segoe UI, Microsoft YaHei, sans-serif">−30 s</text>
      <text x="${L + plotW}" y="${H - 6}" text-anchor="end" fill="#8b95a5" font-size="7" font-family="Segoe UI, Microsoft YaHei, sans-serif">现在</text>
    </svg>`;
    const line = el.querySelector("#altLine");
    const maxLab = el.querySelector("#altMaxLab");
    const nowLab = el.querySelector("#altNowLab");
    return {
      setTrace: function (pts) {
        const rows = Array.isArray(pts) ? pts : [];
        if (!line) {
          return;
        }
        if (!rows.length) {
          line.setAttribute("points", "");
          if (maxLab) maxLab.textContent = "—";
          if (nowLab) nowLab.textContent = "等待高度";
          return;
        }
        const t1 = rows[rows.length - 1].t;
        const t0 = t1 - 30;
        let aMax = 5;
        for (let i = 0; i < rows.length; i++) {
          const a = rows[i].alt;
          if (typeof a === "number" && a > aMax) aMax = a;
        }
        aMax = Math.max(5, aMax * 1.15);
        const ptsStr = [];
        for (let i = 0; i < rows.length; i++) {
          const t = rows[i].t, a = rows[i].alt;
          if (typeof t !== "number" || typeof a !== "number") continue;
          const x = L + plotW * Math.max(0, Math.min(1, (t - t0) / 30));
          const y = T + plotH * (1 - Math.max(0, Math.min(1, a / aMax)));
          ptsStr.push(x.toFixed(1) + "," + y.toFixed(1));
        }
        line.setAttribute("points", ptsStr.join(" "));
        if (maxLab) maxLab.textContent = aMax.toFixed(0) + "m";
        const last = rows[rows.length - 1].alt;
        if (nowLab) nowLab.textContent = (typeof last === "number" ? last.toFixed(1) : "—") + " m";
      },
    };
  }

  function makePoseTracker() {
    // 与 field_pose.py 同一规则：每次 GUIDED 上升沿锁 H；退出则钉回 H。
    let airborne = false;
    let n0 = null, e0 = null, yaw0 = 0;
    let last = { x: 0, y: 0, yawDeg: 0, airborne: false, label: "钉在 H" };
    return {
      update: function (d) {
        d = d || {};
        const guided = String(d.mode || "").trim().toUpperCase() === "GUIDED";
        const nn = d.ned_n, ne = d.ned_e;
        const has = typeof nn === "number" && typeof ne === "number"
          && isFinite(nn) && isFinite(ne);
        if (airborne && !guided) {
          airborne = false;
          n0 = null;
          e0 = null;
          last = { x: 0, y: 0, yawDeg: 0, airborne: false, label: "钉在 H" };
          return last;
        }
        if (!airborne) {
          if (!guided) {
            last = { x: 0, y: 0, yawDeg: 0, airborne: false, label: "钉在 H" };
            return last;
          }
          if (!has) {
            last = { x: 0, y: 0, yawDeg: 0, airborne: false, label: "GUIDED 等 NED" };
            return last;
          }
          n0 = nn;
          e0 = ne;
          yaw0 = (typeof d.yaw_deg === "number" ? d.yaw_deg : 0) * Math.PI / 180;
          airborne = true;
        }
        if (!has) {
          last.label = "hold";
          last.airborne = true;
          return last;
        }
        const dn = nn - n0, de = ne - e0;
        const c = Math.cos(yaw0), s = Math.sin(yaw0);
        const yawNow = (typeof d.yaw_deg === "number" ? d.yaw_deg : 0) * Math.PI / 180;
        last = {
          x: dn * c + de * s,
          y: dn * s - de * c,
          yawDeg: ((yawNow - yaw0) * 180 / Math.PI % 360 + 360) % 360,
          airborne: true,
          label: "NED",
        };
        return last;
      },
    };
  }

  global.FieldMap = { FIELD: FIELD, mount: mount, mountAlt: mountAlt, makePoseTracker: makePoseTracker };
})(window);
