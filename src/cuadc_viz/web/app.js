import { RosBridge } from './rosbridge.js';
import { RollingChart } from './chart.js';
import { FieldMap, nedToField } from './map.js';
import { createViewPlayer } from './player.js';

// 端口跟 viz_bringup.launch.py 的默认值一致，需要的话用 query string 覆盖
// （比如浏览器和 rosbridge 不在同一台机器时）。
const params = new URLSearchParams(location.search);
const ROS_HOST = params.get('ros_host') || location.hostname;
const ROSBRIDGE_PORT = params.get('rosbridge_port') || '9090';
const VIDEO_PORT = params.get('video_port') || '8080';

const ros = new RosBridge(`ws://${ROS_HOST}:${ROSBRIDGE_PORT}`);

function videoSrc(topic) {
  // 不能用 encodeURIComponent——ROS2 话题名只会包含字母/数字/下划线/斜杠，
  // 本来就不需要转义，但 encodeURIComponent 会把 / 转成 %2F，而
  // web_video_server 不会把 %2F 解回 /，导致查询串里的话题名对不上任何
  // 真实话题，请求会一直挂着等一个不存在的话题，界面上表现为"已选中但
  // 未收到画面"，直接在地址栏手动输入（斜杠不会被转义）反而能看到画面，
  // 这是真实踩过的坑。
  return `http://${ROS_HOST}:${VIDEO_PORT}/stream?topic=${topic}`;
}

function setDot(id, level) {
  const el = document.getElementById(id);
  el.className = 'status-dot' + (level ? ` ${level}` : '');
}

function quatToYaw(x, y, z, w) {
  // 标准 ArduPilot/MAVLink 姿态四元数惯例，跟 decision_node.py 的
  // _rpy_from_quaternion 是同一套公式的 yaw 分量。
  return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
function quatToRollPitchDeg(x, y, z, w) {
  const roll = Math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2.0 * (w * y - z * x))));
  return [roll * 180 / Math.PI, pitch * 180 / Math.PI];
}

// ---- 状态条 ----
ros.onStatus((s) => setDot('dot-connected', s === 'connected' ? 'good' : 'critical'));

// cuadc_decision.mission_logic.State.SURVEY_RECON_ZONE 的字符串值——跟
// cv_perception_node.py 的 _RECON_ZONE_STATE 是同一个契约字符串，这里
// 只是又手抄了一份用来驱动侦察图传播放器的"该不该播"判断，不是替代
// 机上的真实门控（机上不发消息，这里就算显示成"播放中"也不会有画面）。
const RECON_ZONE_STATE = 'SURVEY_RECON_ZONE';

ros.subscribe('/cuadc_decision/mission_state', 'std_msgs/String', (m) => {
  document.getElementById('mission-state').textContent = m.data;
  reconPlayer.setActive(m.data === RECON_ZONE_STATE);
});

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
document.getElementById('chk-ref-cylinders').addEventListener('change',
  (e) => map.setShowReference(e.target.checked));
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
// kv.* 这几个 key 名字权威来源是 detectors/ 里检测器填 perf dict 的地方
// （segment_ms/morphology_ms/contours_ms/fit_ms）——改了那边的 key 名，这里
// 要跟着改，不然对应的
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

// ---- 图像面板：三处都是同一个播放器（player.js），区别只在"谁来决定
// 该不该播"（选择器），播放器本身不用关心这个。 ----

// 原始画面：选择器是一个本地勾选框，不涉及机上参数，不发 SetParameters
// ——相机本来就一直在发，不是可选的调试功能，见 cuadc_viz/README.md。
const rawPlayer = createViewPlayer(document.getElementById('tile-raw'), '/camera/image_raw', videoSrc);
document.getElementById('chk-raw').addEventListener('change', (e) => rawPlayer.setActive(e.target.checked));
rawPlayer.setActive(true);

// 侦察图传：选择器是 mission_state 订阅（见上面 RECON_ZONE_STATE），
// 没有用户可操作的开关。
const reconPlayer = createViewPlayer(document.getElementById('tile-recon'), '/cuadc_perception_cv/recon_image', videoSrc);

// 调试图像：选择器是"点格子 + 排队"，两个数字分工不同：
// - backendCap：机上真实上限（max_concurrent_debug_views），从
//   GetParameters 读一次，是滑块能选的范围上限，也是最终兜底——不管
//   网页这边选了几个，机上按 annotated > contours > mask 固定优先级
//   只算/只发这么多路。
// - userCap：用户自己在 [1, backendCap] 之间用滑块选的"我想同时看几个"，
//   纯本地体验设置，默认 1（不主动选更多的话行为跟以前单选一样）。
//   选中数量达到 userCap 时，新选一个会自动关掉最早选中的那个让位。
const DEBUG_SET_PARAMS_SERVICE = '/cuadc_perception_cv/set_parameters';
const DEBUG_GET_PARAMS_SERVICE = '/cuadc_perception_cv/get_parameters';
let backendCap = 1;
let userCap = 1;

const capSlider = document.getElementById('debug-cap-slider');
const capValue = document.getElementById('debug-cap-value');
const capHint = document.getElementById('debug-cap-hint');

ros.callService(DEBUG_GET_PARAMS_SERVICE, 'rcl_interfaces/srv/GetParameters', {
  names: ['max_concurrent_debug_views'],
}).then((res) => {
  backendCap = res.values[0].integer_value;
  capSlider.max = backendCap;
  capSlider.disabled = false;
  capHint.textContent = `这个滑块只能在 1~${backendCap}（机上真实上限）之间选——选够这个数量后`
    + '再选一个，会自动关闭最早选中的那个让位。这只是本地体验设置，机上的'
    + ' max_concurrent_debug_views 参数才是最终硬性上限，跟这个滑块选多少无关。';
}).catch((err) => console.error('读取 max_concurrent_debug_views 失败，滑块保持锁定在 1', err));

const debugPlayers = new Map(); // view -> player
const debugQueue = []; // 已选中的 view，按选中顺序排列，最早的在最前面

document.querySelectorAll('#debug-view-grid .view-tile').forEach((tile) => {
  const view = tile.dataset.view;
  debugPlayers.set(view, createViewPlayer(tile, `/cuadc_perception_cv/debug/${view}`, videoSrc));
});

function setDebugParam(view, enabled) {
  debugPlayers.get(view).setActive(enabled);
  ros.callService(DEBUG_SET_PARAMS_SERVICE, 'rcl_interfaces/srv/SetParameters', {
    parameters: [{ name: `debug_${view}_enabled`, value: { type: 1, bool_value: enabled } }],
  }).catch((err) => console.error(`设置 debug_${view}_enabled 失败`, err));
}

function evictToFit(limit) {
  while (debugQueue.length > limit) setDebugParam(debugQueue.shift(), false);
}

capSlider.addEventListener('input', () => {
  userCap = parseInt(capSlider.value, 10);
  capValue.textContent = userCap;
  evictToFit(userCap); // 往下拖滑块，立刻按新上限清掉多出来的那几个
});

document.querySelectorAll('#debug-view-grid .view-tile').forEach((tile) => {
  const view = tile.dataset.view;
  tile.addEventListener('click', () => {
    const idx = debugQueue.indexOf(view);
    if (idx !== -1) {
      debugQueue.splice(idx, 1);
      setDebugParam(view, false);
      return;
    }
    evictToFit(userCap - 1); // 腾出一个位置给新选的
    debugQueue.push(view);
    setDebugParam(view, true);
  });
});

ros.connect();
