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
  (b) WSL2 下永远带 `--no-wsl2-network`，否则 MAVProxy 的 `--out` 会
  悄悄改到 Windows 宿主 IP；(c) 排查阶段用 `-s`（headless）+
  `--no-mavproxy` 摘掉渲染和转发两个变量，能一刀切定位问题在物理桥
  还是外围——但 `-s` 本身还有另一个独立的摄像头渲染 bug，见
  2026-07-14 记录，排查时别把两层混在一起看。

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

## 2026-07-14

- **诊断"`gazebo_bringup.launch.py` 卡住"时，用了一个跟真实路径不
  等价的简化写法，把自己制造出的假象当成产品 bug 查了一整天**——
  产品代码全程没有问题，这个方法论错误怎么发生、怎么发现的，比"卡住"
  这个表象本身更值得记住。

  诊断时为了方便直接抓 ArduCopter 的控制台输出，给
  `run_in_terminal_window.sh` 设了 `SITL_RITW_TERMINAL=bash`，让它
  不开真正的 `xterm` 终端窗口、直接把 ArduCopter 当 `bash` 的普通子
  进程跑（用户实际操作、`sim_vehicle.py` 默认行为，都是通过
  `xterm -e ...` 分配一个真实伪终端 pty 去跑 ArduCopter）。**这个
  替代写法本身会导致 ArduCopter 卡在收到 Gazebo 第一个 JSON 物理包、
  打印完 `validate_structures: Validating structures` 之后再也不发
  MAVLink 心跳**，跟真实路径完全不是同一回事。用这个写法测了一整天
  （换 ArduPilot 版本、换场地文件、清空/复用 EEPROM、手动调 Gazebo
  物理步进速率……），每次都稳定复现同一个卡死，一度被当成"确认成立
  的产品 bug"写进了文档和记忆。

  最后靠同一时刻、同一份代码的对照实验才发现真相：走真实路径
  （`./scripts/run_gazebo_demo.sh`，`xterm` 起 ArduCopter）连续两次
  完整跑通（心跳 0.1 秒内收到，起飞爬升到 9.53m、悬停、`LAND`、
  上锁，`flight_acceptance_test` PASSED）；只切换 `SITL_RITW_TERMINAL
  =bash` 这一个变量，立刻又复现卡死；把顺手加的 `< /dev/null` 标准
  输入重定向去掉、只留"不开真终端"这一个变量，仍然复现——确认真正的
  变量是"有没有真实 pty"，不是喂了什么当 stdin。具体是 ArduCopter
  内部哪一步依赖 `isatty()`/真实终端，没有定位到代码行（本机
  `strace`/`gdb` 被沙盒权限挡住，见下一条），但触发条件已经用对照
  实验锁死。

  **教训**：诊断时为了方便换一种"看起来等价"的进程拉起方式（不开真
  终端、直接 pipe 标准输出），不能默认它跟真实路径行为一致——尤其是
  跟终端/pty 相关的行为差异极不直观，容易被忽略。正确顺序永远是
  **先跑一次跟用户实际操作完全一致的路径当基线**，简化写法只能用来
  加速复现"已经在基线上确认过"的现象，不能反过来拿简化写法自己的
  复现结果去推断产品本身有没有 bug——本次连续测了一整天"都复现"，
  恰恰是因为方法论错误本身是确定性的，不代表现象本身是真的；越是
  "稳定复现、怎么测都一样"，越应该先怀疑测试方法本身有没有系统性
  偏差，而不是越测越确信结论。二次验证（重启电脑后）：`gazebo launch
  + flight_acceptance_test + QGroundControl` 三个终端一起跑，结论
  不变——流畅、起飞/悬停/降落正常。

- **Gazebo 渲染流畅不需要任何额外 GPU 配置**：`~/.gz/rendering/ogre2.log`
  显示用的是 WSLg 默认的 D3D12→NVIDIA GPU 直通
  （`GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`），全程
  没做过任何手动调优。之前排查"卡"时在 GPU 加速方向上折腾过、卡在某
  一步没解决，现在看那条路线不是必须的——默认直通本身就够流畅。

- **这台机器排查工具受限，遇到需要看进程内部行为的问题这条路走不通**：
  `strace`（含只 trace 自己起的子进程）报
  `ptrace(PTRACE_SEIZE...): Operation not permitted`，`tcpdump` 报
  `Operation not permitted`，都是沙盒/容器权限限制，不是装的方式
  不对；`gdb` 没装。只能靠黑盒对照实验（每次只换一个变量，看现象
  变不变）定位，定位不到具体代码行。

- **Gazebo `-s`（headless）模式下，带摄像头传感器的 world 会卡死，
  是真实存在、跟上面那条方法论错误无关的独立 bug**：任何带摄像头
  传感器的 world（包括 upstream 最简单的官方示例 `camera_sensor.sdf`）
  用 `-s` 起，都会在 `~/.gz/rendering/ogre2.log` 里报
  `Texture memory budget exceeded. Stalling GPU.` 然后卡死。跟
  `gazebo_bringup.launch.py` 无关（它用的是 GUI 模式，从不走 `-s`），
  GUI 模式（含摄像头传感器）不受影响，跟官方 world 一样流畅；但以后
  如果哪个场景需要用 headless 模式会踩到。**教训**：`-s` headless
  和 GUI 模式在传感器渲染这条路上行为不等价，拿 `-s` 的复现结果去
  推断 GUI 模式的问题，前提本身可能不成立。

## 2026-07-19

- **场地世界坐标 (X, Y) 不能直接当 NED (north, east) 用，两者是对调
  关系，不是恒等映射**：`cuadc_perception`（桩感知）、`cuadc_decision`
  （导航目标点）第一版实现都想当然地把 `cuadc_sim/field/config.py` 的
  场地坐标（x=飞行前进方向、y=侧向）直接塞进接口层
  `telemetry/local_position`/`cmd/position_target_local` 的 x/y 字段，
  依据是仿真世界文件 `cuadc_sim/worlds/cuadc_field.sdf` 的
  `<spherical_coordinates><heading_deg>0</heading_deg>`——想当然认为
  这意味着"世界系 +X 对齐正北"。代码注释里当时确实写了"这个假设需要
  重新核实"，但没有在合入前真的核实过。SITL+Gazebo 第一次实机测试，
  飞机一路往侧边飞（该往投放区飞却垂直于场地长轴飞出去），现象跟这条
  没验证的假设错了完全对得上；纯逻辑单元测试和不带 Gazebo 画面的
  SITL 遥测数字自洽性检查都测不出这个问题——遥测数字自洽（发 x=32，
  local_position.x 收敛到 32）只能证明飞控内部控制环闭环了，证明不了
  这个数字对应世界里正确的物理位置。

  真正的换算依据在**飞行器模型自己的插件配置**里，不在世界文件的
  `spherical_coordinates`（那个只管地理经纬度朝向，不管 Gazebo 世界
  坐标系本身怎么定义）：`~/ardupilot_gazebo/models/iris_with_gimbal/
  model.sdf` 的 `ArduPilotPlugin` 里
  `<gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>`
  （roll=180°, yaw=90°）算出来是 `NED north = Gazebo 世界系 Y`、
  `NED east = Gazebo 世界系 X`——X/Y 对调，Z 取负（Gazebo Z 朝上，
  NED Z 朝下）。

  **教训**：Gazebo/ArduPilot 集成里"世界坐标"和"飞控 NED 坐标"的换算
  关系，要去查飞行器模型自己的 `ArduPilotPlugin` 配置
  （`gazeboXYZToNED`/`modelXYZToAirplaneXForwardZDown` 这两个参数），
  不要从世界文件的 `heading_deg` 或者"看起来应该是这样"去猜；任何
  "把一个世界坐标发给飞控"的代码，写完不能只看遥测数字自洽就当作
  验证过了，至少要在 Gazebo 画面里肉眼确认一次飞机真的往预期方向飞
  ——最好是先发一个小步长的单方向位移、肉眼确认方向对了，再跑完整
  任务，不要一次性发一个跨越几十米的完整任务再回头看结果对不对。

## 2026-09-10

- **`iris_with_gimbal` 云台 pitch 关节的正负号跟直觉相反的方向惯例
  又踩了一次同类的坑**：把仿真相机从朝前改成固定朝下时，一开始凭
  常见云台惯例假设"pitch 越负越朝下"（0°=水平前视，-90°=垂直朝下，
  参照大多数航拍云台的惯例），直接想改代码/配置就用 -90°
  （-1.5708 rad）。跟上一条 NED 换算的坑是同一类错误——没有实测，
  只凭"看起来应该是这样"的惯例去猜方向。

  实测方法：Gazebo+SITL 起来后直接用 `gz topic pub` 给
  `/gimbal/cmd_pitch`（`iris_with_gimbal` 的 `JointPositionController`
  订阅的话题，独立于 ArduPilot 的 SERVO 通道）发目标角度，再从
  `/camera/image_raw` 抓一帧存图肉眼看。结果：-90° 拍到的是清晰对焦
  的飞机自己机身腹部（相机翻过去看向了机体内侧），+90°
  （+1.5708 rad）才是垂直朝下（贴地悬停时拍到的是模糊的近距离地面
  纹理，因为离地太近超出了摄像头景深/近裁剪面附近，不是朝上拍到天空）
  ——这个模型的 pitch 关节正负号跟常见云台惯例正好相反。

  最终方案见 `src/cuadc_sim/launch/gazebo_bringup.launch.py` 的
  `fix_camera_pose_process`：开机时通过 `gz topic pub` 一次性把
  roll/pitch/yaw 三个云台关节都钉死在固定角度（pitch 默认
  `+1.5708`），不接 ArduPilot 的 SERVO8/9/10 云台控制通道。之所以
  一次性命令就够、不用担心后续被 ArduPilot 覆盖回去：默认参数里没有
  给 SERVO8/9/10 分配云台相关的 `SERVO*_FUNCTION`，ArduPilot 不会在
  这三个话题上持续发布，实测过命令下达后角度一直保持住，没有被
  ArduPilot 或物理仿真拉回去。另外这个方案让相机变成跟机身刚性绑死的
  效果（`JointPositionController` 只是把关节角度钉在机体坐标系里的
  固定值，不是世界系姿态稳定），跟真机"无云台、相机固定安装、机身
  姿态变化时相机视角跟着晃"的实际情况一致，是有意选的、不是简化
  带来的副作用。

  **教训**：任何"角度/方向哪边是正"的假设，只要没有查到对应关节/
  传感器的原始定义（这次是 `gimbal_small_3d/model.sdf` 里
  `yaw_joint`→`roll_joint`→`pitch_joint`→相机传感器 pose 这条多级
  坐标变换链，手工推导容易出错也确实出错了两次），就不要直接写死
  在代码/配置里；花几分钟在 Gazebo 里实测拍一帧图肉眼确认，比事后
  在真实任务里飞错方向、飞控画面里排查便宜得多。

## 2026-09-11

- **`APPROACH_DROP_TARGET` 从 `close_in` 切到 `converge` 时机身有真实的
  剧烈摆动，已知问题，暂不深修**：把 `_DROP_ALTITUDE_M` 调到明显低于
  `_CRUISE_ALTITUDE_M`（真正触发下降动作）之后，SITL 实测发现每次进入
  `converge` 子步骤那一刻，roll/pitch 会甩到 ±20°量级、水平速度冲到
  ~2.7 m/s，几秒后才衰减下来。加诊断日志（`[ATTITUDE]`，见
  `decision_node.py`）确认这是真实的姿态/速度大幅波动，不是数字
  噪声；两次投放都是同一个模式，且摆动幅度看不出跟离地高度强相关，更
  像是 `close_in`（巡航高度、粗接近半径）一下子跳到 `converge`
  （投放高度、精确目标点）这个位置指令阶跃太大，飞控要用力加减速去
  追，而不是贴地飞行的地面效应。

  当前处理：加了"位置+高度+姿态摆平+水平/垂直速度趋近零，且要连续
  满足 `_STABLE_HOLD_SECONDS` 才算数"的判定（见
  `_tick_approach_drop_target`），确保不会在抖动过程中偶然经过一次
  容差就误判"对齐完成"去触发投放——这解决的是"正确性"（不会在没稳住
  时投放），不是"晃这件事本身"，实测下来现在会正确地等 5~7 秒摆动
  完全平息才投放，比较慢，也不代表真机能这么晃还稳得住。

  **决定**：先不做轨迹平滑/渐进式降高这类飞控层面的优化——跟感知模块
  还是桩节点、真实 CV 还没上线时同时调，容易分不清抖动是控制问题还是
  视觉噪声问题，且真实 CV 上线后大概率还要跟着新的噪声特性重新调一遍，
  等于要调两次。计划先把 CV 做出来，之后再回来对投放区（含这个抖动）、
  侦察区、返航这几个阶段做一轮统一的飞行质量优化，而不是现在单独抠
  投放区这一处。

- **ArduPilot 的 `LOCAL_POSITION_NED` 在这套 SITL+Gazebo 配置下固定绑定
  Gazebo 世界系原点，不跟着出生点走——凭源码推理得出的结论是错的，
  靠实测才发现**：给投放区单点测试世界（飞机出生点从 world (0,0) 挪到
  world (29,0)）设计坐标换算时，查了 `~/ardupilot` 的 EKF3/SITL 源码
  （`AP_NavEKF3_Measurements.cpp`、`SIM_Aircraft.cpp`），推出"home/EKF
  原点是上电时飞机所在的位置"，据此给 `_world_xy_to_ned` 加了一个
  出生点偏移参数、要求所有 world 坐标常量按出生点重新换算。结果 SITL
  实测发现：飞机静止在新出生点、还没起飞时，`local_position` 读数是
  `(0, ~29)`，不是预期的 `(0, 0)`——直接打脸了源码推出来的结论。用
  默认（不做偏移补偿）的换算重新测一遍，从新出生点到第一个扫描航点
  只飞了 ~2.6 秒（预期的短距离），投放区全流程正常跑完，证实
  `local_position` 实际上是相对 Gazebo 世界系原点算的，出生点在哪都
  一样，根本不需要额外的偏移补偿。

  **教训**：跨越多个外部依赖（ArduPilot SITL + JSON FDM backend +
  ArduPilotPlugin + Gazebo）拼起来的坐标链路，即使每一段源码都读过、
  推理过程看起来自洽，也可能因为某个中间环节没读全（比如 JSON FDM
  backend 具体怎么用 `origin`/`home` 这两个概念）而得出错误结论——这
  种情况下"读源码推理"跟"猜"没有本质区别，必须靠直接测量验证。已把
  这次错误设计加的 `home_world_xy` 相关代码全部回退，投放区单点测试
  世界最终不需要任何坐标补偿就能正常工作。
