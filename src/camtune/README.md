# camtune（相机调参 · 调试工具）

台架上给伴随机相机(**IMX519** + **AK7375** 自动对焦)扫**快门/增益/对焦**,
定出运动+摇摆下够清晰、曝光/噪声可接受的拍摄参数。**调试件、非任务件**,
故不带 `cuadc` 前缀,也不进任何自启。方法/物理/判据见
[`docs/spec/camera-tuning.md`](../../docs/spec/camera-tuning.md)。

## 前提

- 在 **Pi** 上跑(用 `rpicam-still` + `cv2`,系统 python3 都有)。
- 相机是**独占**资源:先停图传底座,跑完再起。停/起 tee 安全(相机没有飞控串口
  那种交接卡死问题)。

## 用法（两阶段:先找焦、再调快门）

两种后端,同一套扫描/找焦/CSV:

- **控制器模式(推荐,加 `--via-service`)**:不占相机。相机由常驻 `camera-service`
  持有、仍推地面(`profile=ground-monitor` 时 `video-forward` 在跑);脚本往控制口
  5610 发参数、从 5602 读帧。→ **不停 tee,边扫边在地面 viewer 实时看,同时本地出数据。**
- **独占模式(离线兜底)**:`sudo systemctl stop cuadc-onboard-camera` 后用 rpicam-still
  自己占相机;跑完 `... start`。没有相机服务时用。

下面命令加不加 `--via-service` 决定走哪种;两阶段(先找焦、再调快门)对两者一样。

**① 找焦**(**静止细节标靶、不用节拍器**;固定一档亮的快门/增益,只扫对焦):
```bash
python3 ~/camtune/tune_shutter.py --via-service \
    --lens-positions 0,0.5,1,1.5,2,3 \   # 扫对焦屈光度
    --shutters 4000 --gains 8 --frames 3 \
    --distance 1.0                        # 本次标靶实际距离(m),用于建 距离→对焦 表
```
把细节靶(分辨率卡/报纸/带纹理面,尽量填满画面)放 **~0.5 / 1 / 2 / 4 m 各跑一次**
(0.5 与 4m 必测——景深两端最难)。每个距离取 `lap_var` **mean 峰值**的 lens → 得
"距离→对焦"表。若一个固定焦(景深)就盖住 0.5–4m,取之;否则这表就是**动态命令式
对焦**(按 AGL/分区命令 lens position,非自动对焦)的查表。

**② 调快门**(锁定对焦后,**节拍器摆动**当运动标尺,扫快门/增益):
```bash
python3 ~/camtune/tune_shutter.py --via-service \
    --lens-positions 1.0 \                # 上一步定的固定焦
    --shutters 300,500,1000,2000,4000 \   # 300µs≈1/3333s … 4000µs≈1/250s
    --gains 4,8,16 --frames 6             # 多帧采不同摆动相位,取最糊那帧
```
(去掉 `--via-service` = 独占模式,须先停 `cuadc-onboard-camera`。)

每次生成 `~/camtune_out/sweep_<时间>/`:各档图像 + `index.csv`
(`distance_m,lens_pos,shutter_us,gain,frame,lap_var,brightness,file`)。

## 台架怎么摆

- 相机始终**固定**(三脚架)。
- **①找焦:静止细节标靶,不用节拍器**——只求"某距离下最清";标靶挪到 0.5/1/2/4m。
- **②调快门:让节拍器摆**(摆臂贴高对比标记),别的不动。节拍器速度尽量接近飞行像面
  速度(见 spec `v_px`),拿不准多设几档。
- 关 AF、用命令式对焦(见①)——不要连续自动对焦(会 hunting、延迟、废帧)。
- 一次扫描**光照恒定**,否则亮度变化混淆快门效果。

## 怎么读结果

- **找焦(静止)**:同一距离比各 lens 的 `lap_var` **mean**,峰值那档=该距离对焦。
- **调快门(运动)**:同一档看 `lap_var` **min**(最糊那帧=运动最狠相位)+ **肉眼看
  节拍器标记条纹**(越短越好,这是运动模糊最直接的量)。
- `lap_var` 只在**亮度相近**时可比(暗帧锐度天然低);亮度太低=进光不够:升增益(噪声↑)
  / 加光 / 放长快门(模糊↑)——权衡点就在这。

选定 `(快门, 增益, 对焦)` 后,把它回灌成运行时相机服务的配置(见 spec §6:
可能需把 tee 从 gst `libcamerasrc` 换成 picamera2 才能钉死这些)。
