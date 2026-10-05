"""shm 帧拍照时刻旁路：配对逻辑单测 + 本机端到端（测试源代替 libcamerasrc，相机服务同款 shm 分支 +
同一个 attach_capture_stamps，感知同一个 ShmFrameSource 读）。"""

import importlib.util
import os
import time
from pathlib import Path

import pytest

from cuadc_perception import frame_sources
from cuadc_perception.frame_sources import ShmFrameSource, match_capture_ns

MS = 1_000_000


def test_match_picks_latest_stamp_sent_before_arrival():
    stamps = [(1000 * MS, 960 * MS), (1100 * MS, 1060 * MS), (1200 * MS, 1160 * MS)]
    assert match_capture_ns(stamps, 1101 * MS) == 1060 * MS
    assert match_capture_ns(stamps, 1100 * MS - 1 * MS) == 1060 * MS   # 旁路比帧晚到 1ms 也认
    assert match_capture_ns(stamps, 1180 * MS) is None                # 离最近一条 80ms：配不上
    assert match_capture_ns([], 1000 * MS) is None
    assert match_capture_ns(stamps, None) is None


def _load_camera_service():
    path = Path(__file__).resolve().parents[2] / 'cuadc_onboard' / 'scripts' / 'camera_service.py'
    spec = importlib.util.spec_from_file_location('camera_service', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_end_to_end_capture_stamps_through_shm():
    gi = pytest.importorskip('gi')
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    if Gst.ElementFactory.find('shmsink') is None:
        pytest.skip('no shmsink')
    cam_svc = _load_camera_service()
    shm = f'/tmp/cuadc-test-{os.getpid()}.shm'
    port = 56000 + os.getpid() % 1000
    delay_ms = 50  # 在拍照和进 shm 之间压一段队列，模拟相机链路延迟
    cam = Gst.parse_launch(
        'videotestsrc is-live=true ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! '
        f'queue min-threshold-time={delay_ms * MS} max-size-time=0 max-size-buffers=0 max-size-bytes=0 ! '
        'videorate drop-only=true ! video/x-raw,framerate=10/1 ! '
        f'shmsink name=rawshm socket-path={shm} shm-size={cam_svc._shm_bytes(1920, 1080)} '
        'wait-for-connection=false sync=false async=false')
    cam_svc.attach_capture_stamps(cam, port)
    cam.set_state(Gst.State.PLAYING)
    seen = []
    src = ShmFrameSource(shm, 1920, 1080, 10, 960, 540, stamp_port=port)
    try:
        assert src.start(lambda _f: seen.append((src.last_arrival_ns, src.last_capture_ns())))
        time.sleep(4.0)
    finally:
        src.stop()
        cam.set_state(Gst.State.NULL)
    frames = seen[5:]
    assert len(frames) >= 20
    matched = [(a, c) for a, c in frames if c is not None]
    assert len(matched) / len(frames) > 0.95
    lat_ms = sorted((a - c) / MS for a, c in matched)
    assert 10 < lat_ms[len(lat_ms) // 2] < 150                 # 拍照早于进管线：不是把到达时刻原样返回
    assert lat_ms[int(0.9 * len(lat_ms))] - lat_ms[int(0.1 * len(lat_ms))] < 5   # 配对稳定，不随机错配
    gaps = [(c2 - c1) / MS for (_, c1), (_, c2) in zip(matched, matched[1:])]
    assert all(90 < g < 110 or 190 < g < 210 for g in gaps), gaps   # 每帧配到自己的拍照时刻（允许偶尔丢一帧）
    assert frame_sources._STAMP_MATCH_NS == 50 * MS
