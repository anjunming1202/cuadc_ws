#!/usr/bin/env python3
"""仿真压力测试台的无头多轮跑分器 —— 见 docs/spec/sim-stress-testbed.md。

每轮：起 Gazebo 世界 + 任务节点(真实 CV) → 等就绪 → 触发任务开始 → 订阅
mission_state 记状态时间线 → 判定第二次投放是否完成 → 干净拆除 → 汇总 N 轮。
不用桩;扰动通过 realism parm / 感知时延参数注入(默认关 = 干净 sim)。

用 scripts/run_search_stress.sh 起(它先 source 环境再跑本文件)。直接
`python3` 跑会因为没 source ROS 环境而 import rclpy 失败。
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

REPO_ROOT = Path(__file__).resolve().parents[1]
MISSION_STATE_TOPIC = '/cuadc_decision/mission_state'
TASK_START_TOPIC = '/cuadc_decision/sim/task_start_trigger'
READY_STATE = 'WAIT_TASK_START'          # 决策到此态 = 载具就绪(SITL+EKF 起来了)
# 投放区结束(离开投放任务)的终态。到达其一 = 该判定了,但"到 NAV_RECON"本身
# 不等于成功:决策的 _check_guards 有个投放区任务超时守卫,超时会放弃投放、强制
# 转 NAVIGATE_TO_RECON_ZONE(0~1 次投放)。真成功要看实际投放次数(见下)。
TERMINAL_STATES = {'NAVIGATE_TO_RECON_ZONE', 'RTL_OR_LAND'}
DROP_STATE = 'DROP_PAYLOAD'              # 时间线里出现一次 = 完成一次投放
DROP_TARGET = 2                          # 投满 2 次才算任务成功(= mission_logic._DROP_COUNT_TARGET)

# 清场进程名。lessons.md：本机 `pkill -f` 不可靠,用 pgrep 反查 PID 再 kill -9;
# ros2 launch 会拉起一大堆卫星进程,清场必须覆盖全,否则残留占 5760/9002 端口
# 导致下一轮"起来就死"。这些模式不会匹配本编排器自身(python search_stress.py)。
RESIDUAL_PATTERNS = ['sim_vehicle', 'arducopter', 'mavproxy', 'gz sim',
                     'parameter_bridge', 'ros_gz', 'micro_ros_agent']


class MissionMonitor(Node):
    """订阅 mission_state 记时间线,并能发任务开始触发。跨轮复用,每轮 reset。"""

    def __init__(self):
        super().__init__('search_stress_monitor')
        self.timeline: list[tuple[str, float]] = []
        self.create_subscription(String, MISSION_STATE_TOPIC, self._on_state, 10)
        self._trigger_pub = self.create_publisher(Empty, TASK_START_TOPIC, 10)

    def reset(self):
        self.timeline = []

    def _on_state(self, msg: String):
        state = msg.data
        if not self.timeline or self.timeline[-1][0] != state:
            self.timeline.append((state, time.monotonic()))

    @property
    def current_state(self):
        return self.timeline[-1][0] if self.timeline else None

    @property
    def drops_done(self):
        """时间线里 DROP_PAYLOAD 出现次数 = 实际完成的投放次数。"""
        return sum(1 for s, _ in self.timeline if s == DROP_STATE)

    def trigger_task_start(self):
        self._trigger_pub.publish(Empty())


def _spin_until(node, predicate, timeout_s):
    """spin 到 predicate() 为真(返回 True)或超时(返回 False)。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def _spawn(cmd, log_path):
    """在独立 session(独立进程组)里起一个栈,输出重定向到日志文件。
    独立 session 让我们能 killpg 整棵子进程树,不误伤编排器自身。"""
    log = open(log_path, 'wb')
    proc = subprocess.Popen(
        cmd, cwd=str(REPO_ROOT), stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    return proc, log


def _kill_tree(proc):
    """先 SIGINT(ros2 launch 优雅停全部节点),等一会,再 SIGKILL 兜底。"""
    if proc is None or proc.poll() is not None:
        return
    pgid = os.getpgid(proc.pid)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=8 if sig == signal.SIGINT else 4)
            return
        except subprocess.TimeoutExpired:
            continue


def _reap_residual():
    """兜底扫掉所有残留卫星进程(见 RESIDUAL_PATTERNS 说明)。返回还活着的数。"""
    self_pid = os.getpid()
    for pat in RESIDUAL_PATTERNS:
        out = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True)
        for line in out.stdout.split():
            pid = int(line)
            if pid == self_pid:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    time.sleep(1.0)
    alive = 0
    for pat in RESIDUAL_PATTERNS:
        out = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True)
        alive += len([x for x in out.stdout.split() if int(x) != self_pid])
    return alive


def _run_trial(node, idx, args, logdir):
    node.reset()
    gz_log = logdir / f'trial{idx:02d}_gazebo.log'
    mission_log = logdir / f'trial{idx:02d}_mission.log'
    gz_proc = mission_proc = None
    gz_f = mission_f = None
    result = {'trial': idx, 'outcome': 'error', 'last_state': None,
              'drops': 0, 'ready_s': '', 'complete_s': '', 'note': ''}
    t_start = time.monotonic()
    try:
        _reap_residual()  # 起新一轮前确保干净,避免端口冲突

        gz_cmd = ['bash', 'scripts/run_gazebo_demo.sh',
                  'gui:=true' if args.gui else 'gui:=false']
        if args.realism_parm:
            gz_cmd.append(f'sim_realism_params:={args.realism_parm}')
        gz_proc, gz_f = _spawn(gz_cmd, gz_log)

        mission_cmd = ['bash', 'scripts/run_mission_sim.sh']
        if args.detection_latency_ms:
            mission_cmd.append(f'detection_latency_ms:={args.detection_latency_ms}')
        mission_proc, mission_f = _spawn(mission_cmd, mission_log)

        # 等载具就绪(决策到 WAIT_TASK_START)。Gazebo 冷启慢,给足超时。
        if not _spin_until(node, lambda: node.current_state == READY_STATE,
                           args.ready_timeout_sec):
            result.update(outcome='ready_timeout', last_state=node.current_state,
                          note='never reached WAIT_TASK_START')
            return result
        result['ready_s'] = round(time.monotonic() - t_start, 1)

        # 触发任务开始;多发几次,确保订阅已连上。
        for _ in range(5):
            node.trigger_task_start()
            rclpy.spin_once(node, timeout_sec=0.3)

        t_task = time.monotonic()
        # 跑到离开投放任务(终态)或整轮超时;成功与否单独按实际投放次数判。
        reached = _spin_until(node, lambda: node.current_state in TERMINAL_STATES,
                              args.trial_timeout_sec)
        result['last_state'] = node.current_state
        result['drops'] = node.drops_done
        if node.drops_done >= DROP_TARGET:
            result.update(outcome='success',
                          complete_s=round(time.monotonic() - t_task, 1))
        elif reached:
            # 到了 NAV_RECON/RTL 但投放不足 = 投放区任务超时守卫放弃了投放。
            result.update(outcome='gave_up',
                          complete_s=round(time.monotonic() - t_task, 1),
                          note=f'reached {node.current_state} with {node.drops_done} drop(s)')
        else:
            result.update(outcome='timeout',
                          note=f'timed out in {node.current_state} with {node.drops_done} drop(s)')
        return result
    finally:
        _kill_tree(mission_proc)
        _kill_tree(gz_proc)
        for f in (gz_f, mission_f):
            if f is not None:
                f.close()
        alive = _reap_residual()
        if alive:
            result['note'] = (result['note'] + f' | {alive} residual procs survived').strip(' |')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--gui', action='store_true',
                    help='起带窗口的 Gazebo(默认无头);看/验一轮或手动用时加上')
    ap.add_argument('--ready-timeout-sec', type=float, default=180.0)
    ap.add_argument('--trial-timeout-sec', type=float, default=300.0)
    ap.add_argument('--realism-parm', default='',
                    help='SITL 风/传感器噪声 parm 文件(叠加);空=干净 sim')
    ap.add_argument('--detection-latency-ms', type=float, default=0.0,
                    help='感知发布延迟(建模真实检测流水线时延);0=无')
    ap.add_argument('--out', default='',
                    help='结果 CSV 路径;空=只打印摘要')
    ap.add_argument('--logdir', default='/tmp/search_stress_logs')
    args = ap.parse_args()

    logdir = Path(args.logdir)
    logdir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = MissionMonitor()
    rows = []
    try:
        for i in range(1, args.trials + 1):
            print(f'=== trial {i}/{args.trials} ===', flush=True)
            row = _run_trial(node, i, args, logdir)
            rows.append(row)
            print(f'  -> {row["outcome"]}  drops={row["drops"]} '
                  f'last_state={row["last_state"]}  '
                  f'ready={row["ready_s"]}s complete={row["complete_s"]}s '
                  f'{row["note"]}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    n = len(rows)
    succ = sum(1 for r in rows if r['outcome'] == 'success')
    print(f'\n===== summary: {succ}/{n} success ({100 * succ / max(n, 1):.0f}%) =====')
    for r in rows:
        print(f'  trial {r["trial"]:2d}: {r["outcome"]:8s} drops={r["drops"]} '
              f'last={r["last_state"]} {r["note"]}')
    if args.out:
        with open(args.out, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f'\nwrote {args.out}')
    return 0 if succ == n else 1


if __name__ == '__main__':
    sys.exit(main())
