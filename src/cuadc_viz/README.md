# cuadc_viz

地面站实时监测 dashboard。纯订阅型旁路工具，不发布任何指令、不参与任务
闭环，符合 [ADR-0002](../../docs/spec/adr/0002-onboard-inference-only.md)/
[ADR-0003](../../docs/spec/adr/0003-recon-identification-visual-downlink.md)
"地面不参与决策、单向数据流"的既定边界。这份文件跟着实现持续更新，不是
写一次就不动的设计文档；决策过程的完整讨论记录不在这里重复，只记当前
状态、怎么用、踩过什么坑。

**仅覆盖实时监测**（`feature/viz-live`）。机上 `rosbag2` 黑匣子录制 +
离线回放分析是另一半，还没做，见下面"未做"一节。

## 架构：为什么是这几个包，不是自己写一套

页面本身不写任何 ROS2↔浏览器桥接代码，全部用现成工具组合：

- **rosbridge_suite**（`rosbridge_server`）——ROS2 话题/服务 ↔ 浏览器
  WebSocket 的标准 JSON 桥。前端没有用配套的 `roslibjs`，而是手写了一个
  几十行的极简客户端（`web/rosbridge.js`）——页面只需要 subscribe 和
  call_service 两种操作，换一个第三方库的收益抵不过"前端零依赖、不用
  考虑现场没有外网怎么 vendor"的代价。
- **web_video_server**——压缩图像话题 ↔ 浏览器 MJPEG，`<img>` 标签直接
  订阅即可，不用在页面里写视频解码逻辑。
- **image_transport 的 `republish`**（`image_transport_plugins`
  自带的标准可执行文件）——把感知节点发布的原始 `sensor_msgs/Image`
  转成压缩话题。之所以不在 `cv_perception_node.py` 里直接用
  `image_transport::Publisher`：ROS2 Humble 的 `image_transport`
  **没有 Python 绑定**（`rclpy` 节点拿不到这个 C++ API），所以节点侧
  仍然发布最朴素的 `Image`，压缩这一步交给 `republish` 独立进程做，这是
  标准工具链里绕开"Python 节点用不了 image_transport"这个限制的做法，
  不是我们发明的 workaround。
- **`python3 -m http.server`**——托管 `web/` 静态页面，没有理由为几个
  HTML/JS 文件单独起一个 Node.js/构建工具链。

都在 `launch/viz_bringup.launch.py` 里组合起来：

```
ros2 launch cuadc_viz viz_bringup.launch.py
```

浏览器打开 `http://<地面站或伴随计算机的地址>:8000`（默认端口，见下方
参数表；`rosbridge_port`/`web_video_port`/`web_port` 都能在命令行覆盖）。

**依赖没有装在 rosdep 默认基础安装里**，第一次用需要：

```
sudo apt-get install ros-humble-rosbridge-suite ros-humble-web-video-server
```

## 仿真/真机复用

`cuadc_viz` 只订阅既有契约话题（`docs/spec/04-interfaces.md`、
`05-perception-decision-interface.md`），不区分发布者是仿真桥接还是真实
驱动，本身不用改任何代码——跟任务节点（还要切一个 `connection` 参数）
比，这里连参数都不用切。真正随部署环境变的是传输介质（局域网 vs 真实
无线链路），已经在带宽设计里处理（见下一节），不需要第二套代码。

## 带宽设计：只有图像需要精细控制

遥测/检测/perf/任务状态这些消息几十到几百字节，常开无妨，**没有为它们
加开关**——开关本身也是维护成本，收益接近零时不值得加。图像不一样：
640×480 原始画面单帧约 920KB，10Hz 未压缩要 9.2MB/s，任何数传电台或
远距离 WiFi 都扛不住，必须压缩。三路图像话题都走上面说的
`republish`（raw→compressed）+ `web_video_server`
（`default_transport: compressed`）流水线：

| 话题（原始，节点发布） | 内容 | 开关方式 |
|---|---|---|
| `/camera/image_raw` | 相机原始画面 | 网页勾选框，独立开关 |
| `/cuadc_perception_cv/debug/{annotated,contours,mask}` | 三种调试视图，各自独立话题 | 网页上每种一个可点击格子，点击调用 `rcl_interfaces/srv/SetParameters` 改对应的 `debug_<view>_enabled` 布尔参数。前端区分两个上限：**`backendCap`**（机上真实上限，页面启动时用 `GetParameters` 读一次 `max_concurrent_debug_views`）和 **`userCap`**（用户自己拖滑块在 `[1, backendCap]` 之间选的"我想同时看几个"，纯本地体验设置，默认 1）。选中数量达到 `userCap` 时，新选一个会自动关掉最早选中的那个让位。`userCap` 只是体验设置，**真正的硬性上限还是节点自己的 `max_concurrent_debug_views` 参数**，按 `annotated > contours > mask` 固定优先级只算/只发排前面的几路，网页这边不管滑块拖到多少、两个标签页同时开、或者有人直接在终端改参数，最终还是靠这个兜底 |
| `/cuadc_perception_cv/recon_image` | 侦察区图传 | **没有开关**，`cv_perception_node.py` 内部订阅 `mission_state`，只在 `SURVEY_RECON_ZONE` 阶段转发这一帧，其余阶段这个话题没有消息、`republish`/`web_video_server` 自然不产生流量。裁判要求机组能人工判读，比赛现场没有余裕手动开关，必须自动 |

**关于"网页反向控制机上参数"这件事本身**：`SetParameters` 调用是一条
真实的地面→机上信号（不只是被动订阅），消息本身很小（不是这里的性能
瓶颈），但严格说这跟"图传纯单向监看"不是同一件事。这次的判断是：这个
功能仅用于联调/训练阶段，正式比赛不会有人操作这个网页，所以不受
[ADR-0002](../../docs/spec/adr/0002-onboard-inference-only.md)/
[ADR-0003](../../docs/spec/adr/0003-recon-identification-visual-downlink.md)
"比赛全程不允许人工操纵飞行器"这条约束影响；但这是一个人工判断，不是
拿去跟组委会确认过的结论（不像 ADR-0002/0003 那样有正式 Q&A 依据）——如果
以后真的想在比赛日也用这条反向控制路径，应该先走同样的确认流程，不能
直接照搬现在这个判断。

## 地面端计算的诚实原则（贯穿所有面板）

任何地面端算出来的派生视图（滑动平均、聚类、滤波），必须是原始数据之上
的可关闭叠加层，不能替代原始数据显示。这次首个版本没有实现任何派生
视图——地图上的检测散点、各图表都是逐帧/逐采样点原始连线，不做平滑。
以后加聚类/平滑一类的面板前，先确认这条原则不会被违反：dashboard 存在
的意义是暴露检测/控制的真实表现（含噪声和不稳定），地面端悄悄把噪声
抹平会产生"机上表现比实际更好"的误导，比没有 dashboard 更危险。

## 页面结构

全局状态条（`mission_state`/连接/解锁/模式/电量/EKF·GPS/failsafe）常驻
顶部，下方三栏按场地物理推进顺序排列（起降→投放→侦察，跟
`field-layout.md` 的 x 轴方向、`mission_logic.py` 的 `_DROP_STATES`/
`_RECON_STATES` 分组顺序一致）：

- **通用**——地图视图 + 姿态/速度/高度瞬时值+图表。
- **投放区**——原始画面 + 调试图像 + 检测列表（原始，不跨帧关联）+
  CV profiling。
- **侦察区**——独立图传通道。

### 视图播放器（`web/player.js`）

原始画面/调试图像/侦察图传三处共用同一个播放器组件，只关心"现在该不该
播"这一个布尔输入，不关心是谁触发的——决定"该不该播"是外层选择器各自
的事（勾选框 / 点格子+排队 / `mission_state` 自动判断），互不相同，但
播放器本身完全一样。四种状态：未开启 / 已选中等待首帧 / 正在接收 / 已
选中但未收到——用来区分"网页选了但机上没传"和"网页本来就没选"这两种
容易混在一起的情况，呼应上面的诚实原则（不能让人分不清是没连上还是
本来就没开）。

"正在接收"靠 `<img>` 的 `load` 事件判断，"已选中但未收到"靠一个几秒的
超时——MJPEG 是 `<img>` 直接拉 multipart 流，拿不到每一帧的时间戳，
`load` 事件在不同浏览器上对 multipart 流的触发时机也不完全一致，这里
只当它是"至少收到过一次"的信号，不是精确的持续活体检测，但够用（不需要
判断到"卡了几秒"这么精细，见分支讨论）。"网页没选却一直收到"这种情况
**不检测**——格子关闭时 `img.src` 直接清空，前端压根没有再去看那个话题，
硬要看等于又订阅一路本来想省掉的流，跟省流量的目的矛盾；真出现这种
后端泄漏的 bug，应该在开发时用 `ros2 topic hz` 之类的工具查，不做成
常驻前端功能。

### 地图视图（`web/map.js`）

场地边界/起降点坐标是从 `src/cuadc_sim/field/config.py`
**手抄的第二份常量**（JS/Python 语言边界，没有更好的办法共享同一份
数字）——**改 `field/config.py` 记得同步改 `web/map.js` 顶部那几个
常量**，这是目前唯一的跨语言重复维护点。

飞机/检测点的坐标转换：`telemetry/local_position`、`Detection.position`
都是 NED(north,east)，跟场地坐标系 `(x=推进方向, y=侧向)`不是恒等映射
——`mission_logic.py` 的 `_world_xy_to_ned` 定义 `(north,east) =
(field_y, field_x)`，`map.js` 里的 `nedToField` 是它的反变换
`(field_x, field_y) = (ned_east, ned_north)`。

## 关键参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `rosbridge_port` | 9090 | ROS2↔浏览器 WebSocket 桥端口 |
| `web_video_port` | 8080 | 压缩图像 MJPEG 服务端口 |
| `web_port` | 8000 | 前端静态页面 HTTP 端口 |
| `cv_perception_cv` 节点的 `debug_{annotated,contours,mask}_enabled` | 均为 `false` | 布尔，独立开关（不用字符串枚举是吸取上次教训——YAML 1.1 裸写 off/no/false 会被解析成布尔，字符串参数会类型冲突崩掉，见 `docs/lessons.md`；布尔参数天然没这个问题） |
| `cv_perception_cv` 节点的 `max_concurrent_debug_views` | `1` | 同时最多算/发几路调试图像，真机上的硬性保护 |

浏览器端口若跟 rosbridge/web_video_server 不在同一台机器，用查询字符串
覆盖：`http://host:8000/?ros_host=<ip>&rosbridge_port=9090&video_port=8080`。

## 未做（`feature/viz-replay`，以后开分支再做）

- 机上 `rosbag2` 黑匣子式录制（任务节点生命周期绑定自动开始/结束）。
- 网页"选一个 bag 目录点开即看"的离线回放。
- 设计方向已经讨论过（是否复用这次的前后端还待定，倾向不默认复用，
  避免实时监测和离线分析耦合），过渡期可以直接用 ROS2 自带工具：
  `ros2 bag info`、`rqt_bag`、`ros2 bag play` 配合 `ros2 topic echo`。

## 已知问题 / 后续要跟的点

- 地图/图表还没有做"聚类叠加层"（诚实原则里提到的可选项），当前就是
  纯原始数据展示，如果以后要加，必须是可关闭的叠加层，不能替代原始
  数据。
- 前端没有做任何认证/访问控制——假设 dashboard 只在可信的现场局域网内
  使用，不暴露到公网。
- 网页启动时只读一次 `max_concurrent_debug_views`，运行过程中如果有人
  在终端把这个参数改小了，网页的排队逻辑不会跟着变（还是按启动时读到
  的旧值排队），要刷新页面才会重新读——这个参数本来就不是运行中常改的
  东西，暂时不值得为这个加一条常驻订阅去实时跟踪它的变化。
