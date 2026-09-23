# 机载运行时：传输底座与自启编排（cuadc_onboard）

> 活文档：描述伴随计算机（树莓派）上「让飞控串口与相机被多个消费者
> 共享、并把整套链路做成仓库管理的开机自启」的设计，会随实现迭代修订。
> 模块落在 `src/cuadc_onboard/`（非 colcon 包，systemd + 脚本）。

## 范围与目标

机载有两样物理资源——**飞控串口 `/dev/ttyAMA0`** 和 **相机**——本来
"一次只能一个进程用"。任务链路（`cuadc_interface`）、数据集采集、地面站
遥测/图传、Mission Planner 都想用，于是相互抢占：这就是现场"仓库外自启
的遥测占着串口，采集/接口层起不来"的病根。本模块要：

1. 用「**单一持有者 + 分发**」把两样资源变成**多消费者可共享**的标准端点。
2. 把链路做成**仓库管理的开机自启**，取代散落在 `/etc`、`/usr/local/sbin`
   的仓库外服务；启动顺序显式、可复现。
3. 让**正式任务与测试/调试共用同一套底座**，差异只在"起哪些消费者"。

## 核心架构：底座 + 消费者，自启分两层

```
① 底座层（无条件开机自启 · 持有硬件、产出端点）
   mav_fanout      独占串口 → 主动 keepalive 喂活飞控 → 原始 MAVLink 推到本地 UDP 端点（14550/14551…）
   camera-service  独占相机 → raw NV12 tee：编码→5602/5603（图传）/5605（机载录制）+ shm raw（CV 直吃）；
                     可运行时控制:监听 5610 收 JSON 改曝光/增益/对焦（gst 应用,非静态 gst-launch）
   ground-bus      （预留）被动转发器：本地 5700 有人投才转发到地面，否则空管道
   network/AP、eth 链路

② 消费者层（profile 选择性自启，或手动跑；都只连端点、不碰硬件）
   数传发送 / video-forward（5603→5600）/ cv-preview+cv-forward（5602→5604→5606）/
   onboard_record（录制 5605 全质量 + rosbag）/ dataset 采集 / CV / cuadc_interface …
```

**底座不向地面发**：它把硬件变成本地端点（并主动 keepalive 喂活飞控数据流），
**任何往地面发的都是消费者**——遥测由数传、干净图传由 **video-forward**
（5603→5600）、CV 预览由 **cv-preview + cv-forward**（5602→标注→5604→5606）。
video-forward / cv-* 都归 **ground-monitor profile**（不是底座）：投到本地
5603 的干净画面、或经 cv-preview 产出的 5604，由对应 forward 上地面，生产者
不各写"发地面"。因此 **profile=none 时底座在跑、本地端点有数据，但地面什么
都没有**；要地面看到东西，起对应 profile（如 ground-monitor）或手动起消费者。

**相机服务可运行时控制**：相机生产者是一个 **gst 应用**（非静态 gst-launch）,
独占相机 + 编码 + tee 到本地端点,并监听 `udp:127.0.0.1:5610` 收 JSON 命令,
PLAYING 态即改曝光/增益/对焦（实测该机 `libcamerasrc` 这些属性 runtime 可控,
不需 picamera2）。于是"调参 = 发控制命令"而非独占相机停 tee——调参与图传不互斥;
运行时改 `lens-position` 也就是命令式动态对焦。详见
[`camera-tuning.md`](camera-tuning.md) §6。

**底座对 ROS 一无所知**。消费者是否 ROS 化是它自己的私事：`cuadc_interface`
读同一个 MAVLink 端点后 republish 成 ROS 话题，普通脚本读同一个端点直接
用字节——两者对底座完全一样。这就是"非 ROS 底座、ROS 按需消费"。

## 接口契约（消费者看到的端点）

本模块对外的唯一契约——都是标准线协议上的端点，与实现语言、是否 ROS 无关：

| 资源 | 端点 | 协议 | 用法 |
|---|---|---|---|
| 飞控 MAVLink | `udpin:127.0.0.1:14550` / `14551` / … | MAVLink | 每个消费者绑定一个即收（扇出主动推流）；授权者可写指令（自主飞行走这条） |
| 相机本地帧 | `udp:127.0.0.1:5602` | H.264/RTP | CV/采集/调参 取帧（编码后兼容口），**不开相机** |
| 相机 raw 帧 | `shm:/tmp/cuadc-cam-raw.shm` | NV12 raw | OpenCV/cv-preview 直吃（相机 raw tee，无需解码） |
| 图传转发输入 | `udp:127.0.0.1:5603` | H.264/RTP | 相机服务 tee 的一路；video-forward 读它转地面 |
| 机载录制 tap | `udp:127.0.0.1:5605` | H.264/RTP | 相机服务 tee 的一路(同干净编码)；onboard_record 读它录全质量；无人读则内核丢弃 |
| CV 预览本地 | `udp:127.0.0.1:5604` | H.264/RTP | cv-preview 标注/处理后的画面；cv-forward 读它转地面 |
| 相机控制口 | `udpin:127.0.0.1:5610` | JSON datagram | 向相机服务发 `{ae,exposure_us,gain,af,lens,…}` 运行时改拍摄参数 |
| 图传→地面 | `192.168.50.2:5600` | H.264/RTP | 干净图传；**video-forward** 读 5603 转发 |
| CV 预览→地面 | `192.168.50.2:5606` | H.264/RTP | OpenCV/标注预览；**cv-forward** 读 5604 转发 |
| 地面数据总线 | `udp:127.0.0.1:5700` | JSON datagram（含 `topic`） | 要往地面发的数据投这里，底座按 topic 转发到地面 |
| GCS | `192.168.50.2:14560` | MAVLink | Mission Planner 双向 |
| 回程 JSON（地面→机载） | `udp:127.0.0.1:5701`（预留） | JSON datagram | 地面下发自定义数据给某消费者；管道搭好，按需接 |

三种典型用法：

- **读飞控**：`mavutil.mavlink_connection('udpin:127.0.0.1:14550')` 取代
  `('/dev/ttyAMA0', baud=57600)`，下游不变，且不再独占串口。
- **取相机帧**：从 `:5602` 的本地 RTP 取，不碰相机。
- **往地面发自定义数据**：`bus.send({"topic": "cv/cylinder", ...})`。加一路
  数据 = 新起一个 topic；减一路 = 不投。**底座代码不改**——它是通用的带
  topic 转发器，不把字段写死。

**回程（地面→机载）**：MAVLink 端点本就双向，"地面给飞控发指令"这条回程
免费已有。自定义数据回程是地面总线的镜像（`:5701` 监听）——管道搭好、
默认不接任何具体消息，等真实消费者出现再接。

## profile 与消费者

自启两层：**底座无条件自启；消费者是否开机自启由 profile 选。** profile 是
一套起名的"开机跑哪些消费者"的预设，就是接管自启，与通信怎么走无关。

清单很短（只有"需要免手动"的才配 profile）：

| profile | 开机自启的消费者 | 场景 |
|---|---|---|
| `none` | 无（只起底座） | 日常开发：消费者全手动跑 |
| `ground-monitor` | 数传 + video-forward（5603→5600）+ cv-preview + cv-forward（5602→5604→5606） | 外场一通电，地面站有干净图传 + CV 预览 + 遥测 |
| `mission` | 任务侧 bringup（**按名引用**） | 比赛：无人值守、栈内等"开始"信号 |

profile 落地为一行运行态设置 + 一个 oneshot applier：`/etc/cuadc-onboard/profile`
写一个词，`apply-profile.sh`（由 `cuadc-onboard-profile.service` 在底座起来后跑一次）
按名 `systemctl start` 对应消费者单元。消费者单元装了但不 enable，起不起全看 profile；
改 profile = 编辑该文件 + `systemctl restart cuadc-onboard-profile`（或整机重启）。

- **dataset 采集、CV 调试不是 profile**：它们是**有意识手动跑的消费者**
  （采集每次还要填 session.yaml、定间隔/张数），压在常驻底座上手动跑。
- **profile 选择**是一行运行态设置：`/etc/cuadc-onboard/profile` 写
  `none` / `ground-monitor` / `mission`。仓库给默认，Pi 上可覆盖。
- **"等开始信号"归任务层**，不在编排器里：`mission` profile 只把任务栈
  拉起，等信号发生在任务栈内部。编排器只按序接线，不含任务/决策逻辑。

## 自启与通信解耦（为何一个模块、两层子目录）

```
src/cuadc_onboard/
  scripts/   通信程序（mav_fanout / 数传发送 / ground-bus / 图传转发…）—— 不知 systemd 存在，能手动跑
  systemd/   自启薄壳：单元 ExecStart 指向 scripts/；base target + 各 profile
  deploy.sh  装到 Pi 系统路径并 enable 底座
```

- **依赖方向**：`scripts/` 对 `systemd/` 零依赖——手动 `python scripts/xxx.py`
  照跑。这正是前面"手动调试自由"的来源：同一份通信程序既能手动跑、进测试、
  也能被 systemd 拉起。
- **任务逻辑绝不进本模块**：`mission` profile 只有一行 ExecStart 引用任务侧
  bringup；起哪些节点、什么顺序、等开始信号，全在任务包里。
- **不拆成两个模块**：自启这层本质是通信的薄皮（单元全指向通信脚本），
  离开通信几乎是空的；拆开只多一条模块边界/两套 deploy/版本对齐，为假想
  的未来买单。真长成通用编排框架了再抽出去（可逆、YAGNI）。

## 开发/调试工作流

底座常驻 → 消费者手动跑。迭代 CV v1→vN 时**不碰 profile、不碰配置**：SSH
进去 `python my_cv_debug.py`，连 `:5602` 取帧、往 `:5700` 投任意 topic、看
效果、改代码、再跑。调试数据**不进任何配置**——能随意改的地方是你的脚本，
config（profile）保持小而稳。地面 viewer 做**通用兜底**：不认识的 topic
原样显示，于是加一路调试数据除了发送那行脚本哪儿都不用改。某个搭法稳定到
值得反复开机自启，才把它升格成一个消费者单元/profile。

## 部署

- 单元落 `/etc/systemd/system/`、配置落 `/etc/cuadc-onboard/`；base target
  无条件 enable，用 `After=`/`Wants=` 声明顺序（AP/eth → mav_fanout →
  camera → ground-bus）；profile 由 `/etc/cuadc-onboard/profile` 选。
- 迁移：先 **disable Pi 上仓库外的 4 个 dronelink 服务**，能力由本模块对应
  单元接管；其中 `dronelink-network`/`eth0-100m` 承载现场 SSH，迁移须保证
  不断链。现场分歧**以 Pi 上实际运行版为准**回灌仓库（telemetry 脚本 Pi
  端更新）。

## 与 ROS 的关系

非 ROS 底座、ROS 按需消费。`cuadc_interface` 读 MAVLink 端点 republish 成
ROS 话题喂感知/决策；要把 ROS 里的数据（遥测、CV 结果）送到地面监看，用
**一个通用桥**：订阅配置好的 ROS 话题 → 往地面总线投 JSON。桥只在 ROS↔非
ROS 边界上需要、一个管所有；不进 ROS 的数据（采集脚本、独立 CV）直接投总线，
不过桥。

## 与现有文档/模块的关系

- 底座的 MAVLink 端点，正是接口层 [`04-interfaces.md`](04-interfaces.md) 里
  `cuadc_interface` 所连的对象——本模块是它下面的运行时管道，不改其 ROS
  话题契约。
- `dronelink` 只作**参考实现**（借鉴其验证过的 gst 推流、串口流请求逻辑），
  **不导入**；本模块在 master 谱系上新写。
- 任务侧 bringup 已存在（master 上按场景拆分的脚本），`mission` profile 指
  过去即可，不重造。
- 底座为何用**自写 MAVLink 扇出**（`mav_fanout.py`）而非 mavlink-router——真机
  上的 DTR/RTS 串口控制线与数据流 keepalive 原因——见
  [ADR-0004](adr/0004-onboard-transport-custom-fanout.md)。

## 非目标

- 不覆盖飞控内部调参、机体/电源/投放机构（见 `00-index` 非目标）。
- 不规定感知/决策/接口层各自的内部实现；只提供它们赖以运行的端点。
- 不把图传/遥测强行塞进 ROS（低延迟 RTP 直路保留；ROS 化留给消费者按需做）。

## 开放问题

- ground-bus（本地 5700 → 地面的 topic 转发器）尚未实现：里程碑链路（图传+
  遥测+采集）都不经它——遥测由数传消费者直发地面。等真出现要走总线的自定义
  数据（如 CV 结果上传）再补，届时定 topic 命名空间与 viewer 通用兜底展示。
- 回程通道 `:5701` 仅预留监听，未接具体消费者。
