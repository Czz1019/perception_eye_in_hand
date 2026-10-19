# 真实场景：只扫描、只预览

## 1. 设备接入

使用本机已验证的工厂校准与手眼链：`tool0 → camera_color_optical_frame`，再组合相机内部深度外参。
默认配置 `config/real.yaml` 引用 `/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf` 与相机/机械臂过滤球模板。
若更换相机、安装方式或标定，必须更新这套模型与外部安装 TF。

本机已有启动脚本（根目录执行，每个终端先检查避免重复驱动）：

```bash
bash src/ur7e_real_reconstruction/hardware/start_robot_driver.sh
```

驱动就绪后，**等待操作员在示教器启动 External Control**。运动控制器保持 inactive。
在其他终端启动：

```bash
bash src/ur7e_real_reconstruction/hardware/start_camera.sh
bash src/ur7e_real_reconstruction/hardware/start_camera_mount.sh
```

相机深度和彩色均为 640×480、15 Hz；没有要求深度对齐到彩色。单独处理双相机内参/畸变和时间戳。
本功能包不会自动启动驱动、复位保护停机、激活控制器或移动机械臂。

## 2. 启动与扫描

```bash
./run_ur7e_real_reconstruction.sh
# 若改参数：./run_ur7e_real_reconstruction.sh config:=/absolute/path/real.yaml
```

另一个终端加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
ros2 service call /ur7e_real_reconstruction/start_scan std_srvs/srv/Trigger '{}'
ros2 topic echo /ur7e_real_reconstruction/status
```

- 等待状态 `SCANNING`，`frames`、`vertices` 增加，RViz 显示有真实颜色的场景表面。
- 用示教器手动选择观察姿态；每次停稳约 1 秒以上。软件要求 0.5 秒静止历史、速度低于 0.015 rad/s。
- 从多个方向观察同一物体，检查几何是否重叠、颜色是否贴合；尤其观察桌边、墙面和物体轮廓。
- 默认融合上限 5 Hz、网格显示 1 Hz、体素 1.5 cm。状态报告耗时与丢帧计数；不保证系统降频时持续达到配置频率。
- 与已保存姿态最大关节差均不少于 0.15 rad 时记录新观察姿态；视角图片仅保存被实际融合的帧。
- 本配置只过滤机械臂和相机，未建模夹爪/支架可能残留在重建中；不会因此自动修改过滤模型。
- 深度与彩色视场不同，尚无 RGB 覆盖的网格可能呈灰色；透明、反光、细小物体和遮挡区域可能缺失深度。多视角补扫后仍须对照原始图像确认。

## 3. 完成、冻结与保存

```bash
ros2 service call /ur7e_real_reconstruction/freeze_scan std_srvs/srv/Trigger '{}'
```

服务返回的是请求已接收。**等待状态变成 `FROZEN`** 才表示最终网格提取与写盘成功；`ERROR` 表示未成功，检查 error 字段。
冻结后保持 RViz 显示，新加入的 RViz 也会收到完整快照。输出到 `status.session` 所示独立目录：

- `surface.ply`：米制、base_link 坐标、真实 RGB 的网格，可用 MeshLab/CloudCompare 查看。
- `surface.npz`：网格/颜色/元信息，可重新加载预览；不是可继续融合的 TSDF 层。
- `metadata.json`、`config.json`、`status.json`：参数、模型指纹、姿态和统计。
- `observations/001/rgb.png`、`depth.npz`、`pose.json` 等：逐观察姿态的真实图像和位姿。

重新开始会创建新目录，不覆盖旧扫描。只保存独立姿态的原始输入，不是全帧 rosbag；逐姿态原图仍为相机画面，网格颜色使用校正畸变后的图像。

## 4. 离线重新预览

```bash
./run_ur7e_real_reconstruction.sh load:=/absolute/path/scan_xxx/surface.npz
```

无需活跃相机或机器人；本机标定模型文件仍须存在。不要与同名在线重建节点同时运行。
RViz `Reconstructed Surface` 为彩色网格；可关闭它、开启 `Reconstructed Vertices` 看融合顶点。所有显示固定 `base_link`。

## 5. 和 ESDF 做对照

保持同一 base_link、安装外参和物体位置，比较物体所在侧、桌面高度、尺寸和边缘。
本轮不是自动定量精度验收，也没有向规划器导入表面模型。上一轮 ESDF 若来自已改变的环境，不能直接逐点比较。
更可靠的精度判断需额外测量已知物体的位置/尺寸；共享相机与 TF 的两张地图一致不能排除共同误差。
