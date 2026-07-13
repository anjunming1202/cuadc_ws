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

- **Gazebo+SITL 联调"起来就死"，根因是残留进程/端口冲突（5760/9002），
  不是握手/架构问题**。冷启动、一层层叠加验证定位最快：

  ```bash
  # 0) 彻底清场（pkill -f 在本机不可靠，用 kill -9 <显式PID>）
  for p in sim_vehicle arducopter "xterm.*ArduCopter" "gz sim" mavproxy; do
    pids=$(pgrep -f "$p"); [ -n "$pids" ] && kill -9 $pids
  done
  ss -tln | grep -E '5760|9002|1455'   # 必须为空才算清干净

  # 1) 只起 Gazebo 物理服务端（headless，先不管渲染/GPU）
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
  export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
  gz sim -s -r -v4 iris_runway.sdf      # 日志出现 "Found IMU sensor" 即插件已加载

  # 2) 只起 SITL 连 Gazebo（先不挂 MAVProxy）
  cd ~/ardupilot
  Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --no-mavproxy --no-wsl2-network --no-rebuild --use-dir ~/ardupilot_run_gazebo_physics

  # 3) 直接 pymavlink 连 tcp:127.0.0.1:5760 验心跳+GPS
  ```

  `ArduPilotPlugin.cc:1599 ArduPilot controller has reset` 是新客户端
  连上时的正常握手确认，不是报错。**教训**：(a) Gazebo 物理要
  `-f gazebo-iris --model JSON`，跟内置物理的 `-f quad` 是两条路；
  (b) WSL2 下永远带 `--no-wsl2-network`，否则 MAVProxy 的 `--out` 会被
  悄悄改到 Windows 宿主 IP；(c) 排查阶段用 `-s`（headless）+
  `--no-mavproxy` 摘掉渲染和转发两个变量，能一刀切定位问题在物理桥
  还是外围。

- **`ardupilot_gz_bringup` 官方 launch（`iris_runway.launch.py` 等）在
  本机必现崩溃，是上游未解决的已知 bug**：这些 launch 靠
  `robot_state_publisher` 生成机体描述、`create` 节点读话题去 Gazebo
  spawn，`robot_state_publisher` 解析模型时崩溃
  （`sdformat_urdf`：`Unable to find uri[package://ardupilot_gazebo/...]`）。
  根因是 Gazebo 的 `model://` 路径和 ROS/RViz 的 `package://` 路径不
  兼容，是
  [ArduPilot/ardupilot_gz#49](https://github.com/ArduPilot/ardupilot_gz/issues/49)
  和 [ros/sdformat_urdf#25](https://github.com/ros/sdformat_urdf/issues/25)
  记录在案的已知问题，官方无修复。**应对**：不改第三方代码，自己写
  `gazebo_bringup.launch.py`，复用"world 文件里写死飞机"这条已验证
  路径做物理链路，单独接 `ros_gz_bridge` 做摄像头桥接。

- **两份不同分支的 `ardupilot_gazebo` 同时存在时，`SDF_PATH` 环境变量
  互相踩**：`~/ardupilot_gazebo`（`main` 分支，`model://`，能正常解析）
  和 `ardupilot_gz` vcs import 进 `~/gazebo_ros2_ws/src/ardupilot_gazebo`
  的（`ros2` 分支，`package://`，就是上一条 bug 的源头）**都有一个叫
  `iris_with_gimbal` 的同名模型**。`source ~/gazebo_ros2_ws/install/setup.bash`
  会把 `SDF_PATH`（不是 `GZ_SIM_RESOURCE_PATH`）设成指向坏的那份；
  libsdformat 解析 `model://` 优先吃 `SDF_PATH`，比 `gz-sim` 自己基于
  `GZ_SIM_RESOURCE_PATH` 的逻辑更早生效，导致静默加载错的模型。
  **教训**：`GZ_SIM_SYSTEM_PLUGIN_PATH`/`GZ_SIM_RESOURCE_PATH`/
  `SDF_PATH` 三个都要显式覆盖；不依赖继承自父 shell 的值，直接写成
  `gz sim` 那条 shell 命令自己的行内 `export`（`gazebo_bringup.launch.py`
  的 `gz_env_prefix`）。

- **`ros2 launch` 拉起的进程树，清场关键词必须覆盖全部卫星节点**：
  最早一次官方 launch 测试的 `ros2 launch` 父进程、`micro_ros_agent`、
  `mavproxy`、`ros_gz_sim create`、`rviz2`、`topic_tools relay` 这几个
  卫星进程，被只杀 `gz sim`/`arducopter`/`sim_vehicle`/`mavproxy`/
  `parameter_bridge` 的清场漏掉，活了几个小时。**教训**：`ros2 launch`
  的卫星进程比手动 xterm 拼起来的多得多，清场要么按最初 `ros2 launch`
  的 PID 连子进程一起杀，要么 `ps aux` 全量扫一遍；`ros2 node list`/
  `ros2 topic list` 有几秒到十几秒的 DDS 发现缓存滞后，节点/话题"看起来
  还在"不代表进程没死，以 `ps -p <pid>` 为准。

- **`sim_vehicle.py -f <frame>` 不会自动加载该 frame 的默认参数文件，
  `FRAME_CLASS` 一直是 0，QGC 报 `PreArm: Motors: Check frame class
  and type`**：`vehicleinfo.json` 里 `gazebo-iris` 写了
  `default_params_filename`，但这个字段**只在 `start_CAN_Periph()`
  （跟我们无关的代码路径）里被读取**，主飞控启动逻辑不查这个映射；
  `-w` 清 EEPROM、`--defaults` 都没用（`sim_vehicle.py` 源码注释明说
  "we no longer pass --defaults here"）。**正确做法**：
  `--add-param-file <path/to/gazebo-iris.parm>`（`action="append"`，
  可多次传），已加进 `gazebo_bringup.launch.py`。**教训**：配置文件里
  某字段"看起来应该生效"不代表真的生效，验证参数是否真加载了要直接
  `param_request_read` 查实际值，不能只读配置文件猜测。

- **清场确认只查 `ss -tln`（TCP 监听端口）查不出 MAVProxy 这类 UDP
  进程还活着**：MAVProxy 的 `--out` 是 UDP 发送，不占监听端口，不会
  因端口冲突报错、也不出现在 `ss -tln` 里，导致好几轮 relaunch 的
  `sim_vehicle.py`/`mavproxy.py`/`parameter_bridge` 只堆积没被杀掉，
  十几个僵尸进程链活了很久没发现。**教训**：清场验证要用
  `ps aux | grep -iE "gz sim|arducopter|sim_vehicle|mavproxy|parameter_bridge"`
  直接列进程，`ss -tln` 只能证明"新进程能绑定"，证明不了"旧进程已经
  不在了"。

- **`set -euo pipefail` 的 `-u`（nounset）跟 ROS2 官方 `setup.bash` 不
  兼容**：`scripts/run_gazebo_demo.sh`/`run_sitl_demo.sh` 一开始带了
  `-u`，一 source `/opt/ros/humble/setup.bash` 就报
  `AMENT_TRACE_SETUP_FILES: unbound variable`——ROS2 自己的 setup 脚本
  内部直接引用一些可能没设置的变量，假设的是 bash 默认"没设置就当空
  字符串"的宽松行为，不是`-u`那种"没设置就报错"。**教训**：写
  会 source ROS2 官方/colcon 生成脚本的 wrapper，别带 `-u`，只留
  `set -eo pipefail` 就够（`-e` 捕获我们自己命令的真实错误，`pipefail`
  照旧）。

## 2026-07-11

- **GUIDED 起飞指令被接受，但电机从不真正爬升，几十秒后自动上锁**：根因
  是 SITL 默认编译 `HAL_WITH_ESC_TELEM` 为假（没有 CAN/串口 RCOut），
  导致 `ArduCopter/takeoff_check.cpp` 里唯一负责解除电机 `spool-up
  block` 的函数整个函数体被编译掉，block 永远解不开——指令层面看起来
  完全正常（`COMMAND_ACK` 是 `ACCEPTED`），只有电机推力这一层是假的。
  **修复**：`sim_vehicle.py` 命令行加
  `--configure-define=HAL_WITH_ESC_TELEM=1`（两个 bringup launch 文件
  已经固定加了这个参数，见文件里对应的行内注释），已验证起飞后约 8 秒
  内爬升到目标高度并稳定悬停，不再自动上锁。首次生效需要 SITL 完整
  重新编译（waf define 变了会触发 clean rebuild），后续正常增量编译。
  **教训**：`COMMAND_ACK: ACCEPTED` 只证明指令被接受，不证明物理世界
  真的有反应——协议层测试（发指令、读 ACK）测不出这类"接受了但没执行"
  的 bug，必须另外写断言真实物理状态变化（高度、姿态随时间变化）的
  验收测试，`flight_acceptance_test` 就是为此而加的。

- **pymavlink 的 `recv_match(type=...)` 按单一类型过滤时，会静默丢弃
  等待期间收到的其它类型消息**：写 `flight_acceptance_test.py` 时，
  同一个轮询循环里分别用 `recv_match(type='GLOBAL_POSITION_INT', ...)`
  和 `recv_match(type='HEARTBEAT', ...)` 交替读，结果测试报
  `vehicle never disarmed after LAND`，但飞控自己的日志明明显示已经
  正常落地上锁——根因是 `recv_match` 内部为了找到匹配类型的消息，会
  把等待期间收到的其它类型消息读出来直接丢弃（不是缓存起来留给下一次
  调用），高频调用"只等 GLOBAL_POSITION_INT"的那次读取会把同一时间窗口
  里的 HEARTBEAT 顺手吃掉扔了，导致"另一半"轮询永远等不到它要的类型。
  **教训**：一个轮询循环里如果同时关心多种消息类型（比如同时要看位置
  和武装状态），必须把所有关心的类型放进同一次 `recv_match(type=[...])`
  调用里按 `msg.get_type()` 分支处理，不能分成多次单类型调用交替读——
  后者天然会互相"饿死"对方。修复方式见 `flight_acceptance_test.py` 的
  `_next_state()`。
