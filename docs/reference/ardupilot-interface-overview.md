# ArduPilot 接口能力参考

> **本文件性质**：伴随计算机可以对 ArduPilot 使用的能力概览（不是官方 API
> Reference，只是软件组需要知道的飞控能力清单），供设计 `docs/spec/` 里的
> 接口文档（planning/decision ↔ ArduPilot 的具体契约）时参考。
>
> **这是本文件唯一的权威版本**。原始草稿曾放在 Windows 侧的
> `Companion_Computer_ArduPilot_接口概览.md`（V1.2），内容已原样迁移到这里；
> 那份 Windows 原始文件视为历史草稿，之后如果需要修改这份接口概览，只改
> 这里，不要再改 Windows 那份，避免两处不同步。

飞控：Pixhawk + ArduPilot。

---

## 第一部分：伴随计算机 → 飞控（输入接口）

### 一、飞行模式控制

飞行模式决定飞控当前采用哪种控制逻辑，是最高层的控制方式。同一个导航或
控制指令，在不同飞行模式下可能具有不同的行为，因此通常需要先切换到合适
的飞行模式，再发送后续控制指令。

- **Set Mode**：切换飞控工作模式。
  - **AUTO**：执行已上传的 Mission，由飞控自主完成整个任务流程。
  - **GUIDED**：接受伴随计算机实时发送的导航和控制指令（软件组最常用的模式）。
  - **RTL (Return To Launch)**：自动返回 Home 点并按飞控配置完成返航。
  - **LOITER**：在当前位置附近盘旋等待，保持高度和位置。
  - **MANUAL**：完全由遥控器人工控制。
  - **FBWA**：固定翼辅助飞行模式，由飞控辅助保持飞行姿态，但仍由人工控制飞行方向。
- **Arm**：解锁飞控，允许电机启动并进入可飞行状态（仅表示允许飞行，不代表飞机立即起飞）。
- **Disarm**：上锁飞控，关闭动力输出，结束飞行状态。

### 二、Mission 控制

Mission 用于管理一整条飞行任务，由多个 Waypoint 组成，通常适用于预先规划
好的自动飞行流程。

- **Upload Mission**：上传整条 Mission（包含多个 Waypoint）。
- **Start Mission**：开始执行已上传的 Mission。
- **Pause Mission**：暂停当前 Mission。
- **Resume Mission**：继续执行当前 Mission。
- **Clear Mission**：删除当前 Mission。

### 三、导航控制

导航控制用于指定飞机当前应该飞向哪里，可用于动态改变飞行目标；与 Mission
不同，这些接口通常控制的是当前目标，而不是整条任务。

- **Go To Location**：立即飞向指定 GPS 坐标（最常用的动态导航接口）。
- **Set Waypoint**：设置或修改单个导航目标点（Waypoint）。Mission 本质上也
  是由多个 Waypoint 组成，但此接口更关注单个目标点，而不是整条 Mission。
- **Set Home Position**：设置返航（RTL）使用的 Home 点。

### 四、飞行目标控制

飞行目标控制用于指定飞机"如何飞"，而不是"飞到哪里"；通常与导航控制配合使用。

- **Set Target Altitude**：指定目标飞行高度（飞控会自动控制飞机达到该高度）。
- **Set Target Heading**：指定目标航向（Heading，飞机最终朝向哪个方向飞）。
- **Set Cruise Speed**：指定巡航速度（飞控在正常巡航时尽量保持该速度）。
- **Set Velocity**：直接指定目标飞行速度（速度控制，而不是位置控制；更适合连续控制）。
- **Set Position Target**：直接指定目标位置（属于比 Go To Location 更底层的
  位置控制接口，一般开发中较少直接使用，大多数情况下 Go To Location 已经
  足够）。
- **Set Yaw**：直接指定偏航角（Yaw）（属于姿态层控制；固定翼一般更关注
  Heading，因此较少直接使用）。

### 五、执行机构控制

- **Servo Output**：控制舵机输出（例如投放机构）。
- **Relay Output**：控制继电器。
- **GPIO Output**：控制数字 IO。

### 六、飞控管理

- **Read Parameter**：读取飞控参数（主要用于调试或配置）。
- **Write Parameter**：修改飞控参数（通常只在调试或初始化阶段使用）。
- **Request Calibration**：请求进行传感器校准。
- **Reboot Autopilot**：重启飞控（开发调试阶段常用）。

---

## 第二部分：飞控 → 伴随计算机（输出接口）

### 一、位置状态

- **Global Position**：GPS 经纬度（飞机当前全球坐标）。
- **Local Position**：局部坐标（通常相对于起飞点建立的坐标系）。
- **Relative Altitude**：相对起飞点高度。
- **Absolute Altitude**：绝对海拔高度。
- **Home Position**：返航 Home 点坐标。

### 二、运动状态

- **Velocity**：当前三维速度向量（完整速度信息）。
- **Ground Speed**：相对于地面的速度大小。
- **Air Speed**：相对于空气的速度大小（安装空速计时）。
- **Vertical Speed**：当前爬升或下降速度。

### 三、姿态状态

- **Roll**：横滚角。
- **Pitch**：俯仰角。
- **Yaw / Heading**：当前航向角（Heading 是导航意义上的朝向；固定翼通常更
  关注 Heading）。
- **Attitude Quaternion**：四元数姿态（另一种姿态表示方式，通常算法内部使用）。

### 四、飞行状态

- **Current Mode**：当前飞行模式（AUTO、GUIDED、RTL 等）。
- **Armed State**：当前是否解锁。
- **Mission State**：当前 Mission 的执行状态（例如未开始、执行中、暂停、完成）。
- **Current Waypoint**：当前正在执行的 Waypoint（Mission 中的第几个目标点）。
- **Flight Time**：当前飞行时间。

### 五、导航状态

- **GPS Status**：GPS 当前定位状态及定位质量（表示 GPS 是否可用）。
- **Satellite Count**：当前可用卫星数量。
- **HDOP**：GPS 定位精度指标（数值越小通常表示定位越好）。
- **EKF Status**：飞控状态估计算法运行状态（用于判断导航是否可靠）。

### 六、电源状态

- **Battery Voltage**：当前电池电压。
- **Battery Current**：当前放电电流。
- **Battery Remaining**：飞控估算的剩余电量百分比。
- **Power Status**：飞控供电状态。

### 七、飞控健康状态

- **Heartbeat**：飞控在线心跳（表示飞控仍在正常通信）。
- **System Status**：飞控整体运行状态。
- **Failsafe State**：当前是否进入失效保护。
- **Status Text**：飞控输出的状态信息、警告和错误日志。
- **System Time**：飞控系统时间（常用于日志同步）。

### 八、通信状态

- **RC Input**：当前遥控器输入值（各通道输入）。
- **RC Link Status**：遥控器链路状态（是否正常连接）。

### 九、传感器状态

- **IMU Data**：加速度计和陀螺仪原始数据。
- **Barometer**：气压计数据。
- **Magnetometer**：磁力计数据。

### 十、执行反馈

- **Servo Output State**：当前各 PWM 输出状态。
- **Mission ACK**：Mission 上传或操作后的确认信息。
- **Command ACK**：飞控对控制命令的执行确认。
