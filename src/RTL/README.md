# RTL（GUIDED 视觉返航）

不切飞控 RTL（找不到垫时除外）。任务结束后停在 GUIDED：飞回起飞原点 0.5 m 内 →
原地悬停约 2 s 收帧找垫心 → 有 waypoint 则飞控对准（连续 0.5 s 在 0.20 m 内，否则 10 s 后边降边校）GUIDED 5→3 m 再切 LAND；
没有可用圆心则切飞控 RTL。

取帧与投放区 OpenCV 同一路：**10 fps 原始 NV12 shm**（`ShmFrameSource`），
图不进 ROS，也不走 5602 先编码再解码。位姿是起飞系 `local_position`。
扫描结束释放 shm，降落再接。

## 怎么跑

| 用途 | 命令 | 状态 |
|---|---|---|
| **全链路（推荐）** | `python3 -m RTL.rtl_main` | 一次拉起回原点 + 扫描 + 降落三个节点 |
| 同上 | `python3 -m RTL.visual_rtl` | 转调 `rtl_main`，不单独实现逻辑 |
| 只回原点 | `python3 -m RTL.rtl_go_home` | 调试用 |
| 只扫描 | `python3 -m RTL.rtl_scan` | 调试用；需已在原点约 0.5 m 内或收到 `home_arrived` |
| 只降落 | `python3 -m RTL.rtl_land` | 调试用；需已有 latched `pad_waypoint` |

触发全链路（任务完成指令未定，先用 Empty）：

```
ros2 topic pub --once /cuadc_rtl/rtl_start std_msgs/msg/Empty
```

交接（上一段结束话题，下一段才开始；`home_arrived` / `pad_waypoint` / `stage` 均 latched）：

`rtl_start` → 回原点 → `home_arrived` → 悬停找垫 → `pad_waypoint` → 降落  
扫描没圆心则 `stage=fc_rtl` + 飞控 RTL，**不发** `pad_waypoint`，降落节点一直等。

`/cuadc_rtl/stage`：`go_home` → `home_arrived` → `scan` → `pad_waypoint` | `fc_rtl` → `land`

## 文件与函数

### 入口

| 文件 | 作用 | 调用 |
|---|---|---|
| [`rtl_main.py`](rtl_main.py) | `main()` 把三个阶段节点放进同一进程 | 全链路入口 |
| [`visual_rtl.py`](visual_rtl.py) | 转调 `rtl_main.main` | 同入口别名 |

### 三个飞行阶段（ROS 节点，可被主脚本一起调，也可单独 `python3 -m`）

| 文件 | 节点 / 入口 | 关键函数 | 做什么 |
|---|---|---|---|
| [`rtl_go_home.py`](rtl_go_home.py) | `RtlGoHomeNode`，`main()` | 订 `rtl_start`，发 `(0,0)` | 进原点约 0.5 m 后发 `home_arrived`，停指令 |
| [`rtl_scan.py`](rtl_scan.py) | `RtlScanNode`，`main()` | `sample_pad_xy` | **等 `home_arrived`** 才悬停 2 s；有圆心发 `pad_waypoint`，没有则飞控 RTL |
| [`rtl_land.py`](rtl_land.py) | `RtlLandNode`，`main()` | 连续 0.5 s 锁 0.20 m | GUIDED 5→3 m 途中对准；3 m 切 LAND 不查水平。10 s 未锁也降 |

### 算法库（被上面调用，自己不飞）

| 文件 | 函数 | 做什么 |
|---|---|---|
| [`detect_landing_pad.py`](detect_landing_pad.py) | `find_landing_pad_center`，`landing_pad_to_takeoff_xy`，`load_landing_pad_config` | HSV 外轮廓找垫心；射线从镜头交地面 → 起飞系 XY（已含飞控↔镜头距离；指令不再减 0.11） |
| [`scan_plan.py`](scan_plan.py) | `scan_waypoints`，`ground_footprint_m`，`load_rtl_config` | FOV 航点规划仍在；位移扫描在 `rtl_scan` 里注释掉 |
| [`cluster_hits.py`](cluster_hits.py) | `consensus_xy` | 多帧垫心：中位数剔野值再均值 → 一个点 |
| [`topics.py`](topics.py) | 话题常量 | `rtl_start` / `home_arrived` / `pad_waypoint` / `stage` |

### 配置

| 文件 | 内容 |
|---|---|
| [`landing_pad.yaml`](landing_pad.yaml) | 蓝 HSV、面积/圆度 |
| [`rtl.yaml`](rtl.yaml) | 原点 0.5 m、悬停 2 s、锁 0.20 m×0.5 s、10 s 未锁也降、3 m LAND |

`__init__.py` 只导出检测/规划/聚合纯函数，不自动起节点。
