# Lessons

> 踩坑记录：环境/工具的坑 + 实验评测的发现，混在一起按时间记，不细分类型
> （分开对两人团队来说分类负担大于收益，见 2026-07-07 的讨论）。只增不改，
> 除非发现记错了；如果以后这份文档写得又长又杂、不同类型的内容互相干扰不
> 好找，再考虑拆分。

## 2026-07-06

- **PDF 中文提取乱码**：`2026中国大学生飞行器设计创新大赛竞赛规则 ★公开★.pdf`
  用 `pdftotext` 提取中文会乱码（字体编码问题）。改用 PyMuPDF
  （`import fitz; fitz.open(path)[i].get_text()`）能正确提取 Unicode。

## 2026-07-07

- **从 Windows 侧 Claude Code 会话桥接到 WSL 时的路径坑**：`wsl.exe -d
  <distro> -- bash -lc '...'` 里如果传了 `/mnt/c/...` 这种 POSIX 风格路径，
  Git Bash 的 MSYS 层会在传给 `wsl.exe` 之前把它悄悄重写成 Windows 路径
  （比如变成 `D:/Program Files/Git/mnt/c/...`），导致找不到文件。加
  `MSYS_NO_PATHCONV=1` 前缀能关掉这个转换。另外多语句的 `-lc` 字符串里，
  变量赋值可能在跨桥接时丢失（`A=hello; echo "$A"` 会打印空），更可靠的
  做法是把脚本写成文件再用 `bash /path/to/script.sh` 执行。

## 2026-07-09

- **多赛项合订 PDF 里摘错了竞赛章节**：《2026中国大学生飞行器设计创新
  大赛竞赛规则 ★公开★.pdf》一份文件里收录了 12 个不同竞赛项目，命名
  模式高度相似（"无人机定点空投""固定翼无人机侦察与打击""多旋翼无人机
  侦察与救援"……都是"XX无人机YY"或"YY无人机侦察与ZZ"的结构）。最初摘录
  `docs/reference/competition-rules-summary.md` 时只凭"侦察与打击/救援"
  字面联想就认定是同一个项目，摘了「固定翼无人机侦察与打击」章节（文档
  内页 19–24），但实际报名的是「多旋翼无人机侦察与救援」章节（文档内页
  36–41），两者任务性质、场地布局、计分规则、判罚条款几乎处处不同。这个
  错误一直没被发现，直到 `README.md`、`docs/spec/*.md`、
  `docs/reference/*.md` 整套文档都已经按错误章节写完才被人工发现。
  **教训**：从多赛项合订 PDF 摘录规则前，必须先翻到目录页，核对页码
  **和**赛项全名都跟实际报名项目完全一致，不能只凭章节标题"读起来像"就
  认定摘对了；摘录规则时如果原始 PDF 是多赛项合订本，摘要文档里应该
  明确写出页码范围和相邻赛项名称，方便后续复核发现类似的选错情况。

- **肉眼读示意图会看错比例关系**：规则原文"投放区示意图"（图2）里 B 区
  的标注文字是"以筒心为圆心，向外延伸直径为1m 的筒外圆环区域"，第一次
  核对时肉眼看图，直觉判断成"1m 是圆环的径向宽度（环宽固定，外圈直径
  随筒径变化）"，写进了文档。后来用 PyMuPDF 把图渲染成图片、按颜色做
  连通域分析、从圆心沿多个角度做像素级径向扫描，量出三个圆筒对应的
  **外圈直径完全相等（都是同一个像素值），只有内圈（筒身）大小随筒径
  比例变化**，才发现真相是反过来的——"1m"是外圈整体直径固定，环宽随
  筒径变化。**教训**：示意图里"两个同心圆+一个距离标注"这种图，肉眼判断
  "标注的是环宽还是外圈直径"很容易凭直觉看反，尤其当图不是严格按比例画
  （这张图里圆筒本身的大小明显被放大以便观察，不是真实 8m×5m 场地下
  15–25cm 筒径该有的视觉比例）。涉及具体尺寸判断且肉眼直觉可能不可靠时，
  应该直接把图渲染出来做像素测量（对比多个实例的相对大小关系），而不是
  只看一遍就下结论。

- **本机 colcon 编译出的 ROS2 包，`source install/setup.bash` 后
  `ros2 run`/`ros2 launch` 找不到包**：正常预期下 `colcon build` 会给
  每个包生成一个 `ament_prefix_path` 环境 hook（写在
  `install/<pkg>/share/<pkg>/hook/ament_prefix_path.sh` 等文件里），
  `install/setup.bash` source 时会执行这个 hook，把包所在的 prefix 加进
  `AMENT_PREFIX_PATH`，`ros2` 系工具才找得到包。用 `cuadc_sim` 包实测
  （`ament_python` build type，`colcon-ros 0.5.0` + `colcon-core 0.20.1`，
  均为系统 apt 包）发现这个 hook 文件始终没有生成——不管 `--merge-install`
  与否都一样，日志和 events.log 里也没有任何报错或异常。单独调用
  `colcon_core.shell.create_environment_hook()` 复现是正常工作的，说明不是
  这个函数本身坏了，具体是 `colcon_ros` 调用它的哪个环节没生效还没有
  继续深挖（对搭仿真环境这个目标没有增量价值，先记录现象和 workaround）。
  **Workaround**：`source install/setup.bash` 之后，手动把每个用到的包的
  安装前缀显式加进 `AMENT_PREFIX_PATH`，例如：
  `export AMENT_PREFIX_PATH="$(pwd)/install/<pkg>:$AMENT_PREFIX_PATH"`。
  **教训**：这台机器上任何新建的 ROS2 包，`colcon build` 之后如果
  `ros2 run`/`ros2 launch` 报"package not found"，先怀疑这个已知坑，
  不要假设是包本身写错了（`package.xml`/`setup.py` 配置没问题）。

- **Gazebo+SITL 反复起不来，其实是残留进程/端口冲突，不是架构问题**：
  接 Gazebo 物理（`--model JSON`）跑三方联调（Gazebo + arducopter +
  MAVProxy/QGC）时，遇到一连串"起来就死"：MAVProxy 连上 SITL 的
  TCP 5760 但一直收不到心跳、随后自己卸载模块退出，arducopter 也莫名
  其妙不在了。一度怀疑是 arducopter 的 Gazebo-JSON 握手没完成、或
  Gazebo 插件状态坏了。**真实原因**：一次 WSL2 VM 中途重启（Windows
  触发，非我们操作）把所有进程杀了但保留了磁盘状态，之后我几次快速
  重启只清理了一部分进程，残留的 arducopter/xterm/gz sim 抢占
  5760/9002 端口，新起的实例和旧的互相打架。**根因排查方法**（有效）：
  别在满是残留进程的环境里做取证，直接全部清干净、冷启动、一层一层
  叠加验证。清干净后按下面顺序单独验证，一次就通了：

  ```bash
  # 0) 彻底清场（pkill -f 在本机不可靠，用 kill -9 <显式PID>，见下）
  for p in sim_vehicle arducopter "xterm.*ArduCopter" "gz sim" mavproxy; do
    pids=$(pgrep -f "$p"); [ -n "$pids" ] && kill -9 $pids
  done
  ss -tln | grep -E '5760|9002|1455'   # 必须为空才算清干净

  # 1) 只起 Gazebo 物理服务端（headless，-s 不开 GUI，先不管渲染/GPU）
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
  export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
  gz sim -s -r -v4 iris_runway.sdf      # 日志出现 "Found IMU sensor" 即插件已加载、在等 SITL

  # 2) 只起 SITL 连 Gazebo（先不挂 MAVProxy，排除转发层干扰）
  cd ~/ardupilot
  Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --no-mavproxy --no-wsl2-network --no-rebuild --use-dir ~/ardupilot_run_gazebo_physics

  # 3) 直接 pymavlink 连 tcp:127.0.0.1:5760 验心跳+GPS
  #    心跳 0s 到、GPS 拿到 -35.3632621,149.1652374（来自 Gazebo 物理）即全链路通
  ```

  验证结果：心跳瞬间就到，GPS 位置正确，Gazebo 日志出现
  `ArduPilotPlugin.cc:1599 ArduPilot controller has reset`——这行是
  **正常**的握手确认（新 SITL 客户端连上时插件重置状态），不是报错。
  **教训**：(a) 用 Gazebo 物理时 frame 要用 `-f gazebo-iris` 且必须带
  `--model JSON`，跟内置物理的 `-f quad` 是两条不同的路子；(b) WSL2
  下永远带 `--no-wsl2-network`，否则 MAVProxy 的 `--out` 会被悄悄改到
  Windows 宿主 IP；(c) 联调起不来时，先怀疑残留进程/端口冲突，冷启动
  分层验证比在乱局里取证快得多；(d) 排查阶段刻意用 `-s`（headless）+
  `--no-mavproxy`，把 GPU 渲染和消息转发两个变量都摘掉，能一刀切定位
  问题到底在物理桥还是在外围。
