import { RosBridge } from './rosbridge.js';
import { RollingChart } from './chart.js';
import { FieldMap, nedToField } from './map.js';

// 端口跟 viz_bringup.launch.py 的默认值一致，需要的话用 query string 覆盖
// （比如浏览器和 rosbridge 不在同一台机器时）。
const params = new URLSearchParams(location.search);
const ROS_HOST = params.get('ros_host') || location.hostname;
const ROSBRIDGE_PORT = params.get('rosbridge_port') || '9090';
const VIDEO_PORT = params.get('video_port') || '8080';

const ros = new RosBridge(`ws://${ROS_HOST}:${ROSBRIDGE_PORT}`);

function videoSrc(topic) {
  return `http://${ROS_HOST}:${VIDEO_PORT}/stream?topic=${encodeURIComponent(topic)}`;
}

function setDot(id, level) {
  const el = document.getElementById(id);
  el.className = 'status-dot' + (level ? ` ${level}` : '');
}

function quatToYaw(x, y, z, w) {
  // 标准 ArduPilot/MAVLink 姿态四元数惯例，跟 decision_node.py 的
  // _roll_pitch_from_quaternion 是同一套公式的 yaw 分量。
  return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
function quatToRollPitchDeg(x, y, z, w) {
  const roll = Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2.0 * (w * y - z * x))));
  return [roll * 180 / Math.PI, pitch * 180 / Math.PI];
}

// ---- 状态条 ----
ros.onStatus((s) => setDot('dot-connected', s === 'connected' ? 'good' : 'critical'));

ros.subscribe('/cuadc_decision/mission_state', 'std_msgs/String',
  (m) => { document.getElementById('mission-state').textContent = m.data; });

ros.subscribe('/cuadc_interface/status/armed', 'std_msgs/Bool', (m) => {
  document.getElementById('txt-armed').textContent = m.data ? '已解锁' : '未解锁';
  setDot('dot-armed', m.data ? 'warning' : 'good'); // 解锁中是需要留意的状态，不是"好/坏"
});
ros.subscribe('/cuadc_interface/status/mode', 'std_msgs/String',
  (m) => { document.getElementById('txt-mode').textContent = m.data; });
ros.subscribe('/cuadc_interface/status/failsafe', 'cuadc_interface_msgs/FailsafeState', (m) => {
  document.getElementById('txt-failsafe').textContent = m.active ? `Failsafe: ${m.reason}` : 'Failsafe';
  setDot('dot-failsafe', m.active ? 'critical' : 'good');
});
ros.subscribe('/cuadc_interface/telemetry/battery', 'sensor_msgs/BatteryState', (m) => {
  const pct = Math.round(m.percentage * 100);
  document.getElementById('txt-battery').textContent = `${pct}%`;
  setDot('dot-battery', pct < 20 ? 'critical' : pct < 40 ? 'warning' : 'good');
  batteryChart.push(pct);
});
ros.subscribe('/cuadc_interface/telemetry/ekf_status', 'cuadc_interface_msgs/EkfStatus',
  (m) => setDot('dot-ekf', m.healthy ? 'good' : 'critical'));
ros.subscribe('/cuadc_interface/telemetry/gps_status', 'cuadc_interface_msgs/GpsStatus', (m) => {
  document.getElementById('txt-gps').textContent = `${m.satellites_visible}sat`;
  setDot('dot-gps', m.fix_type >= 3 ? 'good' : 'warning');
});

// ---- 通用：地图 + 姿态/速度/高度 ----
const map = new FieldMap(document.getElementById('map-canvas'));
const speedChart = new RollingChart(document.getElementById('chart-speed'));
const altChart = new RollingChart(document.getElementById('chart-alt'));
const rollChart = new RollingChart(document.getElementById('chart-roll'));
const pitchChart = new RollingChart(document.getElementById('chart-pitch'));
const batteryChart = new RollingChart(document.getElementById('chart-battery'));

let lastYaw = 0;

ros.subscribe('/cuadc_interface/telemetry/local_position', 'geometry_msgs/PoseStamped', (m) => {
  const north = m.pose.position.x, east = m.pose.position.y, down = m.pose.position.z;
  const [fx, fy] = nedToField(north, east);
  map.setVehicle(fx, fy, lastYaw);
  const alt = -down;
  document.getElementById('stat-alt').textContent = alt.toFixed(1);
  altChart.push(alt);
});

ros.subscribe('/cuadc_interface/telemetry/attitude', 'sensor_msgs/Imu', (m) => {
  const q = m.orientation;
  lastYaw = quatToYaw(q.x, q.y, q.z, q.w);
  const [rollDeg, pitchDeg] = quatToRollPitchDeg(q.x, q.y, q.z, q.w);
  document.getElementById('stat-roll').textContent = rollDeg.toFixed(1);
  document.getElementById('stat-pitch').textContent = pitchDeg.toFixed(1);
  rollChart.push(rollDeg);
  pitchChart.push(pitchDeg);
});

ros.subscribe('/cuadc_interface/telemetry/velocity', 'geometry_msgs/TwistStamped', (m) => {
  const v = m.twist.linear;
  const speed = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  document.getElementById('stat-speed').textContent = speed.toFixed(2);
  speedChart.push(speed);
});

// ---- 投放区：检测列表 + profiling ----
ros.subscribe('/perception/detections', 'cuadc_perception_msgs/DetectionArray', (m) => {
  const rows = m.detections.map((d) => `<tr>
    <td>${d.position.x.toFixed(2)}</td><td>${d.position.y.toFixed(2)}</td>
    <td>${d.apparent_size.toFixed(3)}</td><td>${d.confidence.toFixed(2)}</td></tr>`).join('');
  document.getElementById('detections-body').innerHTML = rows;
  map.setDetections(m.detections.map((d) => {
    const [fx, fy] = nedToField(d.position.x, d.position.y);
    return { x: fx, y: fy, confidence: d.confidence };
  }));
});

const perfChart = new RollingChart(document.getElementById('chart-perf'));
// kv.* 这几个 key 名字权威来源是 cv_pipeline.py 的 detect_cylinders()
// 里填 perf dict 的地方——改了那边的 key 名，这里要跟着改，不然对应的
// stat 会一直显示 "--"（不会报错，容易漏掉）。
ros.subscribe('/cuadc_perception_cv/perf', 'diagnostic_msgs/DiagnosticArray', (m) => {
  const status = m.status[0];
  if (!status) return;
  const kv = Object.fromEntries(status.values.map((v) => [v.key, parseFloat(v.value)]));
  document.getElementById('perf-segment').textContent = kv.segment_ms?.toFixed(1) ?? '--';
  document.getElementById('perf-morph').textContent = kv.morphology_ms?.toFixed(1) ?? '--';
  document.getElementById('perf-contours').textContent = kv.contours_ms?.toFixed(1) ?? '--';
  document.getElementById('perf-fit').textContent = kv.fit_reproject_ms?.toFixed(1) ?? '--';
  if (kv.total_ms !== undefined) perfChart.push(kv.total_ms);
});

// ---- 图像面板 ----
const imgRaw = document.getElementById('img-raw');
const imgRecon = document.getElementById('img-recon');
imgRecon.src = videoSrc('/cuadc_perception_cv/recon_image'); // 独立通道，固定订阅，没有开关

document.getElementById('chk-raw').addEventListener('change', (e) => {
  imgRaw.src = e.target.checked ? videoSrc('/camera/image_raw') : '';
});
imgRaw.src = videoSrc('/camera/image_raw');

// 调试图像：每个格子对应 CV 节点的一个 debug_<view>_enabled 布尔参数，
// 通过 rosbridge 的 rcl_interfaces/srv/SetParameters 服务改，等价于
// `ros2 param set`。真正的"最多几路同时算/发"由节点自己的
// max_concurrent_debug_views 硬性限制，网页这边的"允许多选"只是 UI
// 便利（不勾选时表现成单选），不是安全边界——见 cuadc_viz/README.md。
const debugTiles = [...document.querySelectorAll('#debug-view-grid .debug-tile')];
const multiCheckbox = document.getElementById('chk-multi-debug');

function setDebugView(tile, active) {
  const view = tile.dataset.view;
  tile.classList.toggle('active', active);
  const img = tile.querySelector('img');
  img.src = active ? videoSrc(`/cuadc_perception_cv/debug/${view}`) : '';
  ros.callService('/cuadc_perception_cv/set_parameters', 'rcl_interfaces/srv/SetParameters', {
    parameters: [{ name: `debug_${view}_enabled`, value: { type: 1, bool_value: active } }],
  }).catch((err) => console.error(`设置 debug_${view}_enabled 失败`, err));
}

debugTiles.forEach((tile) => {
  tile.addEventListener('click', () => {
    const nowActive = !tile.classList.contains('active');
    if (nowActive && !multiCheckbox.checked) {
      debugTiles.filter((t) => t !== tile && t.classList.contains('active'))
        .forEach((t) => setDebugView(t, false));
    }
    setDebugView(tile, nowActive);
  });
});

ros.connect();
