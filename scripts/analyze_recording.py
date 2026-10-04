#!/usr/bin/env python3
"""机载录制 bag 离线分析：多面板总览图 + 终端文本时间线 —— 见 docs/spec/recording-analysis.md。

    python3 scripts/analyze_recording.py local/recordings/20261004_200827_dropbench_bag
    python3 scripts/analyze_recording.py local/recordings          # 目录下所有 *_bag 逐个处理
    python3 scripts/analyze_recording.py <bag> --show              # 另弹交互窗口

产物固定存到 local/analyze/<录制名>.png + .txt（录制名 = bag 目录名去掉 _bag，与 .ts 同名）。

纯 Python 读 mcap，不用 source ROS。依赖：pip3 install --user mcap mcap-ros2-support matplotlib
"""
from __future__ import annotations

import argparse
import datetime
import math
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    sys.exit('缺 mcap 读取库：pip3 install --user mcap mcap-ros2-support')

TOPIC_POS = '/cuadc_interface/telemetry/local_position'
TOPIC_VEL = '/cuadc_interface/telemetry/velocity'
TOPIC_ATT = '/cuadc_interface/telemetry/attitude'
TOPIC_BATT = '/cuadc_interface/telemetry/battery'
TOPIC_MODE = '/cuadc_interface/status/mode'
TOPIC_ARMED = '/cuadc_interface/status/armed'
TOPIC_DROP = '/cuadc_interface/status/drop'
TOPIC_CMD_POS = '/cuadc_interface/cmd/position_target_local'
TOPIC_CMD_VEL = '/cuadc_interface/cmd/velocity'
TOPIC_CMD_SPEED = '/cuadc_interface/cmd/speed'
TOPIC_STATE = '/cuadc_decision/mission_state'
TOPIC_DET = '/perception/detections'
TOPIC_PERF = '/cuadc_perception_cv/perf'
TOPIC_ROSOUT = '/rosout'
OUT_DIR = Path(__file__).resolve().parents[1] / 'local' / 'analyze'  # 产物固定落这里（local/ 不入库）
TOPICS = [TOPIC_POS, TOPIC_VEL, TOPIC_ATT, TOPIC_BATT, TOPIC_MODE, TOPIC_ARMED, TOPIC_DROP,
          TOPIC_CMD_POS, TOPIC_CMD_VEL, TOPIC_CMD_SPEED, TOPIC_STATE, TOPIC_DET, TOPIC_PERF,
          TOPIC_ROSOUT]

LOG_WARN = 30  # rcl_interfaces/Log level
LOG_NAMES = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
DET_RATE_WINDOW_S = 1.0


@dataclass
class Recording:
    name: str
    start_ns: int = 0
    duration_s: float = 0.0
    series: dict = field(default_factory=lambda: defaultdict(list))  # 键 -> [(t, v...)]
    changes: dict = field(default_factory=lambda: defaultdict(list))  # 键 -> [(t, 值)]，只记变化
    det_frames: list = field(default_factory=list)  # (t, 检测数, 状态)
    dets: list = field(default_factory=list)        # (t, x, y, conf, size, 状态)
    logs: list = field(default_factory=list)        # (t, level, 节点名, 文本)
    drops: list = field(default_factory=list)       # 投放执行时刻

    def arr(self, key):
        return np.array(self.series[key], dtype=float).reshape(-1, len(self.series[key][0]) if self.series[key] else 1)

    def note_change(self, key, t, value):
        if not self.changes[key] or self.changes[key][-1][1] != value:
            self.changes[key].append((t, value))

    def state_at(self, t):
        cur = None
        for ts, s in self.changes['state']:
            if ts > t:
                break
            cur = s
        return cur or '-'


def find_bags(paths):
    """bag 目录 / .mcap / 含多个 bag 的目录 -> [(名字, [mcap 文件])]。"""
    bags = []
    for p in map(Path, paths):
        if p.is_file() and p.suffix == '.mcap':
            bags.append((p.parent.name, [p]))
        elif p.is_dir() and sorted(p.glob('*.mcap')):
            bags.append((p.name, sorted(p.glob('*.mcap'))))
        elif p.is_dir():
            bags += [(d.name, sorted(d.glob('*.mcap'))) for d in sorted(p.iterdir())
                     if d.is_dir() and sorted(d.glob('*.mcap'))]
        else:
            sys.exit(f'找不到 bag：{p}')
    if not bags:
        sys.exit('参数里没有任何 *.mcap')
    return bags


def _iter_messages(files):
    factory = DecoderFactory()
    decoders = {}
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh)
            for schema, channel, message in reader.iter_messages(topics=TOPICS, log_time_order=True):
                if not schema.data:  # std_msgs/Empty 等无字段消息录下来 schema 为空，解码器不认
                    yield channel.topic, message.log_time, None
                    continue
                dec = decoders.get(schema.id)
                if dec is None:
                    dec = decoders[schema.id] = factory.decoder_for(channel.message_encoding, schema)
                yield channel.topic, message.log_time, dec(message.data)


def _bag_time_range(files):
    start, end = None, None
    for f in files:
        with open(f, 'rb') as fh:
            stats = make_reader(fh).get_summary().statistics
        start = stats.message_start_time if start is None else min(start, stats.message_start_time)
        end = stats.message_end_time if end is None else max(end, stats.message_end_time)
    return start, end


def _rpy_deg(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def load(name, files) -> Recording:
    start, end = _bag_time_range(files)
    rec = Recording(name, start, (end - start) / 1e9)
    state = '-'
    for topic, log_ns, m in _iter_messages(files):
        t = (log_ns - start) / 1e9
        s = rec.series
        if topic == TOPIC_POS:
            p = m.pose.position
            s['pos'].append((t, p.x, p.y, p.z))
        elif topic == TOPIC_VEL:
            v = m.twist.linear
            s['vel'].append((t, v.x, v.y, v.z))
        elif topic == TOPIC_ATT:
            s['att'].append((t, *_rpy_deg(m.orientation)))
        elif topic == TOPIC_BATT:
            s['batt'].append((t, m.voltage))
        elif topic == TOPIC_CMD_POS:
            s['cmd_pos'].append((t, m.x, m.y, m.z))
        elif topic == TOPIC_CMD_VEL:
            v = m.twist.linear
            s['cmd_vel'].append((t, v.x, v.y, v.z))
        elif topic == TOPIC_CMD_SPEED:
            s['cmd_speed'].append((t, m.data))
        elif topic == TOPIC_MODE:
            rec.note_change('mode', t, m.data)
        elif topic == TOPIC_ARMED:
            rec.note_change('armed', t, m.data)
        elif topic == TOPIC_STATE:
            state = m.data
            rec.note_change('state', t, m.data)
        elif topic == TOPIC_DROP:
            if m.actuated:
                rec.drops.append(t)
        elif topic == TOPIC_DET:
            rec.det_frames.append((t, len(m.detections), state))
            for d in m.detections:
                rec.dets.append((t, d.position.x, d.position.y, d.confidence, d.apparent_size, state))
        elif topic == TOPIC_PERF:
            for kv in (m.status[0].values if m.status else []):
                if kv.key == 'total_ms':
                    s['perf'].append((t, float(kv.value)))
        elif topic == TOPIC_ROSOUT:
            if m.name != 'rosbag2_recorder':
                rec.logs.append((t, m.level, m.name, m.msg))
    return rec


# ---------------------------------------------------------------- 文本摘要

def _intervals(changes, end):
    """[(t, 值)] 变化序列 -> [(t0, t1, 值)]。"""
    return [(t, changes[i + 1][0] if i + 1 < len(changes) else end, v) for i, (t, v) in enumerate(changes)]


def summarize(rec: Recording) -> str:
    lines = []
    emit = lines.append
    start = datetime.datetime.fromtimestamp(rec.start_ns / 1e9)
    emit(f'== {rec.name}   start {start:%Y-%m-%d %H:%M:%S}   duration {rec.duration_s:.1f}s')
    for key, title in (('state', 'mission state'), ('mode', 'FC mode'), ('armed', 'armed')):
        if rec.changes[key]:
            emit(f'-- {title}')
            for t0, t1, v in _intervals(rec.changes[key], rec.duration_s):
                emit(f'   {t0:7.2f}s  {str(v):26s} ({t1 - t0:6.1f}s)')
    for t in rec.drops:
        emit(f'-- DROP actuated at {t:.2f}s')
    warns = [lg for lg in rec.logs if lg[1] >= LOG_WARN]
    if warns:
        emit('-- rosout WARN+')
        for t, lvl, node, msg in warns:
            emit(f'   {t:7.2f}s  {LOG_NAMES.get(lvl, lvl):5s} [{node}] {msg}')
    if rec.det_frames:
        emit('-- detections by mission state')
        by_state = defaultdict(lambda: [0, 0])
        for _, n, st in rec.det_frames:
            by_state[st][0] += 1
            by_state[st][1] += n > 0
        for st, (n, nd) in by_state.items():
            ds = [d for d in rec.dets if d[5] == st]
            extra = (f'  conf med {statistics.median(d[3] for d in ds):.2f}'
                     f'  radius med {statistics.median(d[4] for d in ds):.3f}m') if ds else ''
            emit(f'   {st:26s} frames {n:5d}  with-det {nd:4d} ({100 * nd / n:5.1f}%){extra}')
    perf = rec.series['perf']
    if perf:
        ms = sorted(p[1] for p in perf)
        emit(f'-- perception total_ms  med {ms[len(ms) // 2]:.1f}  p95 {ms[int(len(ms) * 0.95)]:.1f}  max {ms[-1]:.1f}')
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------- 绘图

def _state_colors(rec):
    import matplotlib.pyplot as plt
    cmap = plt.get_cmap('tab20')
    names = list(dict.fromkeys(v for _, v in rec.changes['state']))
    return {s: cmap(i % 20) for i, s in enumerate(names)}


def _shade_states(ax, rec, colors):
    for t0, t1, s in _intervals(rec.changes['state'], rec.duration_s):
        ax.axvspan(t0, t1, color=colors[s], alpha=0.12, lw=0)
    for t in rec.drops:
        ax.axvline(t, color='magenta', ls='--', lw=1.2)


def _plot_xy(ax, rec, colors):
    pos = rec.arr('pos')
    if colors:
        for t0, t1, s in _intervals(rec.changes['state'], rec.duration_s):
            seg = pos[(pos[:, 0] >= t0) & (pos[:, 0] <= t1)]
            if len(seg):
                ax.plot(seg[:, 2], seg[:, 1], color=colors[s], lw=1.8)
    else:
        sc = ax.scatter(pos[:, 2], pos[:, 1], c=pos[:, 0], s=3, cmap='viridis')
        plt_colorbar(ax, sc, 'time (s)')
    ax.plot(pos[0, 2], pos[0, 1], 'k^', ms=8, label='start')
    ax.plot(pos[-1, 2], pos[-1, 1], 'ks', ms=7, label='end')
    if rec.series['cmd_pos']:
        cp = rec.arr('cmd_pos')
        uniq = np.unique(np.round(cp[:, 1:3], 2), axis=0)
        ax.scatter(uniq[:, 1], uniq[:, 0], marker='s', s=60, facecolors='none', edgecolors='red',
                   label='position target')
    if rec.dets:
        d = np.array([x[1:5] for x in rec.dets], dtype=float)
        sc = ax.scatter(d[:, 1], d[:, 0], c=d[:, 2], cmap='plasma', vmin=0, vmax=1, marker='x', s=18,
                        label='detection (color=conf)')
        plt_colorbar(ax, sc, 'detection conf')
    ax.set_xlabel('y (m)')
    ax.set_ylabel('x (m)')
    ax.set_title('top-down trajectory (x up, y right)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)


def plt_colorbar(ax, mappable, label):
    cb = ax.figure.colorbar(mappable, ax=ax, orientation='horizontal', fraction=0.035, pad=0.06)
    cb.set_label(label)


def _plot_alt(ax, rec):
    pos = rec.arr('pos')
    ax.plot(pos[:, 0], -pos[:, 3], 'b-', lw=1.2, label='altitude (-z)')
    if rec.series['cmd_pos']:
        cp = rec.arr('cmd_pos')
        ax.step(cp[:, 0], -cp[:, 3], 'r:', where='post', lw=1.2, label='cmd target alt')
    ax.set_ylabel('alt (m)')
    ax.legend(loc='upper right', fontsize=8)


def _plot_speed(ax, rec):
    if rec.series['vel']:
        v = rec.arr('vel')
        ax.plot(v[:, 0], np.hypot(v[:, 1], v[:, 2]), 'b-', lw=1.2, label='horiz speed')
        ax.plot(v[:, 0], -v[:, 3], 'c-', lw=0.8, alpha=0.7, label='climb rate')
    if rec.series['cmd_vel']:
        cv = rec.arr('cmd_vel')
        ax.plot(cv[:, 0], np.hypot(cv[:, 1], cv[:, 2]), 'g.', ms=2, label='|cmd vel| horiz (body)')
    if rec.series['cmd_speed']:
        cs = rec.arr('cmd_speed')
        ax.step(cs[:, 0], cs[:, 1], 'r:', where='post', lw=1.2, label='cmd speed limit')
    ax.set_ylabel('m/s')
    ax.legend(loc='upper right', fontsize=8)


def _plot_att(ax, rec):
    a = rec.arr('att')
    ax.plot(a[:, 0], a[:, 1], 'r-', lw=0.8, label='roll')
    ax.plot(a[:, 0], a[:, 2], 'g-', lw=0.8, label='pitch')
    ax.set_ylabel('roll/pitch (deg)')
    ay = ax.twinx()
    ay.plot(a[:, 0], a[:, 3], 'b.', ms=1.5, label='yaw')
    ay.set_ylabel('yaw (deg)', color='b')
    ax.legend(loc='upper left', fontsize=8)
    ay.legend(loc='upper right', fontsize=8)


def _plot_det(ax, rec):
    f = np.array([(t, n) for t, n, _ in rec.det_frames], dtype=float)
    hit = (f[:, 1] > 0).astype(float)
    rate = np.array([hit[(f[:, 0] > t - DET_RATE_WINDOW_S) & (f[:, 0] <= t)].mean() for t in f[:, 0]])
    ax.plot(f[:, 0], rate, 'k-', lw=1, label=f'det rate ({DET_RATE_WINDOW_S:.0f}s window)')
    if rec.dets:
        d = np.array([(x[0], x[3]) for x in rec.dets], dtype=float)
        ax.plot(d[:, 0], d[:, 1], 'r.', ms=3, label='conf')
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel('det')
    ax.legend(loc='upper right', fontsize=8)


def _plot_simple(ax, rec, key, ylabel, style):
    a = rec.arr(key)
    ax.plot(a[:, 0], a[:, 1], style, lw=1, ms=2)
    ax.set_ylabel(ylabel)


def _plot_timeline(ax, rec, colors):
    rows = []
    for key in ('state', 'mode', 'armed'):
        if rec.changes[key]:
            rows.append(key)
    mode_cmap = {}
    for i, key in enumerate(rows):
        y = len(rows) - 1 - i
        for t0, t1, v in _intervals(rec.changes[key], rec.duration_s):
            if key == 'state':
                c = colors[v]
            elif key == 'mode':
                c = mode_cmap.setdefault(v, f'C{len(mode_cmap) % 10}')
            else:
                c = 'tab:red' if v else 'lightgray'
            ax.broken_barh([(t0, t1 - t0)], (y - 0.4, 0.8), color=c, alpha=0.7 if key != 'state' else 0.9)
            if t1 - t0 > rec.duration_s * 0.03:
                label = v if key != 'armed' else ('ARMED' if v else 'disarmed')
                ax.text(t0 + (t1 - t0) / 2, y, str(label), ha='center', va='center', fontsize=7, clip_on=True)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(list(reversed(rows)))
    for t, lvl, node, msg in rec.logs:
        if lvl >= LOG_WARN:
            ax.plot(t, len(rows) - 0.45, 'v', color='orange' if lvl == LOG_WARN else 'red', ms=6)
    ax.set_ylim(-0.6, len(rows) - 0.3)
    ax.set_xlabel('time since recording start (s)   (▼ rosout WARN/ERROR)')


def plot(rec: Recording, out: Path, show: bool):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    colors = _state_colors(rec)
    panels = []
    if rec.series['pos']:
        panels.append(lambda ax: _plot_alt(ax, rec))
    if rec.series['vel'] or rec.series['cmd_vel'] or rec.series['cmd_speed']:
        panels.append(lambda ax: _plot_speed(ax, rec))
    if rec.series['att']:
        panels.append(lambda ax: _plot_att(ax, rec))
    if rec.det_frames:
        panels.append(lambda ax: _plot_det(ax, rec))
    if rec.series['perf']:
        panels.append(lambda ax: _plot_simple(ax, rec, 'perf', 'CV ms/frame', 'm.'))
    if rec.series['batt']:
        panels.append(lambda ax: _plot_simple(ax, rec, 'batt', 'battery (V)', 'k-'))
    has_timeline = any(rec.changes[k] for k in ('state', 'mode', 'armed'))
    n = len(panels) + has_timeline
    if not n and not rec.series['pos']:
        print('   (bag 里没有可画的话题，跳过出图)')
        return

    fig = plt.figure(figsize=(20, max(8, 1.9 * n + 1)))
    gs = GridSpec(max(n, 1), 2, figure=fig, width_ratios=[1, 1.8], hspace=0.12, wspace=0.15, top=0.95)
    if rec.series['pos']:
        _plot_xy(fig.add_subplot(gs[:, 0]), rec, colors)
    axes = []
    for i, draw in enumerate(panels):
        ax = fig.add_subplot(gs[i, 1], sharex=axes[0] if axes else None)
        draw(ax)
        _shade_states(ax, rec, colors)
        ax.grid(alpha=0.3)
        axes.append(ax)
    if has_timeline:
        ax = fig.add_subplot(gs[n - 1, 1], sharex=axes[0] if axes else None)
        _plot_timeline(ax, rec, colors)
        ax.grid(alpha=0.3, axis='x')
        axes.append(ax)
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    if axes:
        axes[0].set_xlim(0, rec.duration_s)
    start = datetime.datetime.fromtimestamp(rec.start_ns / 1e9)
    fig.suptitle(f'{rec.name}   {start:%Y-%m-%d %H:%M:%S}   {rec.duration_s:.0f}s', fontsize=13)
    fig.savefig(out, dpi=110, bbox_inches='tight')
    print(f'-- saved {out}')
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=f'机载录制 bag 离线分析（总览图 + 文本时间线，存到 {OUT_DIR}）')
    ap.add_argument('paths', nargs='+', help='bag 目录 / .mcap / 含多个 *_bag 的目录')
    ap.add_argument('--show', action='store_true', help='另弹交互窗口')
    ap.add_argument('--no-plot', action='store_true', help='只出文本摘要')
    args = ap.parse_args()
    bags = find_bags(args.paths)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, files in bags:
        stem = name.removesuffix('_bag')  # 与同次录制的 <stem>.ts 同名
        rec = load(name, files)
        text = summarize(rec)
        print('\n' + text, end='')
        (OUT_DIR / f'{stem}.txt').write_text(text, encoding='utf-8')
        print(f'-- saved {OUT_DIR / f"{stem}.txt"}')
        if not args.no_plot:
            plot(rec, OUT_DIR / f'{stem}.png', args.show)


if __name__ == '__main__':
    main()
