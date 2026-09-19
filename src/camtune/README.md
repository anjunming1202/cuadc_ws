# camtune（相机调参 · 调试工具）

台架上给伴随机相机(**IMX519** + **AK7375** 自动对焦)扫**快门/增益/对焦**,
定出运动+摇摆下够清晰、曝光/噪声可接受的拍摄参数。**调试件、非任务件**,
故不带 `cuadc` 前缀,也不进任何自启。方法/物理/判据见
[`docs/spec/camera-tuning.md`](../../docs/spec/camera-tuning.md)。

## 前提

- 在 **Pi** 上跑(用 `rpicam-still` + `cv2`,系统 python3 都有)。
- 相机是**独占**资源:先停图传底座,跑完再起。停/起 tee 安全(相机没有飞控串口
  那种交接卡死问题)。

## 用法

```bash
sudo systemctl stop  cuadc-onboard-camera          # 让出相机
cd src/camtune       # 或 Pi 上对应路径
python3 tune_shutter.py \
    --shutters 500,1000,2000,4000,8000 \           # µs;1/2000s≈500µs
    --gains 1,4,8 \
    --lens-position 1.0 \                           # 屈光度:1m→1.0、0.5m→2.0、∞→0
    --frames 3
sudo systemctl start cuadc-onboard-camera          # 恢复图传
```

每次运行生成 `~/camtune_out/sweep_<时间>/`:各档图像 + `index.csv`
(`shutter_us,gain,frame,lap_var,brightness,file`)。

## 台架怎么摆

- 相机**固定**(三脚架),只让**节拍器**动;摆臂上贴**高对比标记**。
- **对焦先对准目标距离**(设 `--lens-position`),否则清晰度会被跑焦搅乱。
- 一次扫描**光照恒定**,否则亮度变化混淆快门效果。
- 节拍器速度尽量接近飞行的像面速度(见 spec 的 `v_px`);拿不准就多设几档。

## 怎么读结果

1. **肉眼看标记条纹**——越短越好,这是运动模糊的直接量。
2. `lap_var` 越大越锐,但**只在亮度相近时比**(暗帧锐度天然低)。
3. 亮度太低 = 该档进光不够:升增益(噪声↑)/加光/放长快门(模糊↑)——权衡点就在这。

选定 `(快门, 增益, 对焦)` 后,把它回灌成运行时相机服务的配置(见 spec §6:
可能需把 tee 从 gst `libcamerasrc` 换成 picamera2 才能钉死这些)。
