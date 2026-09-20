# UR7e 眼在手上彩色场景重建

Gazebo RGB + 深度 + 各自 CameraInfo + 图像时刻 TF → GPU TSDF/颜色融合 → RViz 彩色三角网格。
世界坐标系为 `base_link`，不使用 SLAM，不计算 ESDF。
复用 `realtime_nvblox` 后端；该包和 `ur7e_d435_gazebo` 的源码保持不变。

## 启动完整扫描

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_reconstruction.sh
# 终端 1：新初始姿态、原来的两个墙体和球，默认打开 Gazebo
./run_ur7e_reconstruction_scene.sh
# 终端 2：自动激活 whole_control，打开 RViz，执行多视角扫描
./run_ur7e_reconstruction.sh scan:=true
```

启动前关闭已有 Gazebo/重建实例，避免重复节点和控制器。
`gazebo_gui:=false` 可关闭 Gazebo 窗口，`run_rviz:=false` 可关闭 RViz。
普通 `./run_ur7e_reconstruction.sh` 不主动移动机械臂；只有 `scan:=true` 执行扫描。
扫描通过 `/forward_position_controller/commands` 控制已有 ros2_control 位置控制器。
这些观察姿态是为本包固定仿真场景设置的，不是通用避障规划器。

初始相机位置约为 `(0.30, 0.16, 0.83)` 米，原来约为 `(0.57, 0.15, 0.72)` 米。
观察姿态覆盖俯视、南侧、东侧、西北侧、西侧、西南侧，以及球体北侧、西侧和东侧的低位补扫。
默认共 21 个停留点（含安全过渡与返回），本机一次完整扫描约需数分钟。
跨区域移动先返回高位；同一侧的补扫通过已验证的过渡姿态移动。
扫描器移动前调用服务暂停融合，到位并保持稳定 0.8 秒后恢复融合，
每次停留至少 5 秒且融合至少 15 对 RGB-D 图像，再移动到下一视角。
运动使用仿真时间插值，最大关节速度默认 0.3 rad/s，并检查实际关节反馈。
扫描结束返回俯视姿态，自动保存 PLY 和 `scan_report.json`，RViz 保留累积地图。
关节反馈超时、到位失败或感知错误时停止扫描并保持当前反馈位置。

## 图像和坐标系

| 输入 | 话题 / 坐标 |
|---|---|
| 深度、内参 | `/camera/depth/image_rect_raw`、`/camera/depth/camera_info` |
| RGB、内参 | `/camera/color/image_raw`、`/camera/color/camera_info` |
| 深度位姿 | `base_link ← camera_depth_optical_frame` |
| Gazebo RGB 位姿 | `base_link ← camera_sim_color_optical_frame` |
| 关节反馈 | `/joint_states` |

RGB 和深度使用近似时间同步（35 ms），各自使用自身图像时间戳查询 TF；不假设深度已经对齐 RGB。
Gazebo URDF 中 RGB 传感器实际位于 `camera_link` 的 `(0, 0.015, 0)`，
与名义外壳光学原点不同。因此本包发布仿真专用 RGB 光学坐标：
相对深度光学坐标平移 `(-0.015, 0, 0)` 米、旋转为单位阵。
这只适用于当前 Gazebo 模型；真实相机应使用实际标定的 RGB TF，不能套用此仿真变换。

支持 16UC1 毫米和 32FC1 米深度；RGB 支持 rgb8/bgr8/rgba8/bgra8 和带行填充的数据。
默认降采样至 640×360，并分别缩放两套内参。TSDF 体素为 1 cm，截断距离为 2 个体素。
只融合停稳的图像；运动判断来自关节位置变化，避免 Gazebo 约束求解器的瞬时速度噪声。
没有对应时刻 TF 或图像过期时丢帧，不使用最新 TF 代替。

## RViz、导出与配置

| 输出 / 服务 | 内容 |
|---|---|
| `/ur7e_reconstruction/mesh` | 彩色三角网格，默认显示 |
| `/ur7e_reconstruction/surface_points` | 带 RGB 的融合网格顶点，默认隐藏 |
| `/ur7e_reconstruction/camera_pose` | 实际采用的深度相机位姿 |
| `/ur7e_reconstruction/stats` | 帧数、丢帧、时延、网格规模、运行环境 |
| `/ur7e_reconstruction/set_integration_enabled` | SetBool：暂停/恢复融合，保留已有地图 |
| `/ur7e_reconstruction/reset_map` | Trigger：清空地图 |
| `/ur7e_reconstruction/save_mesh` | Trigger：导出当前 PLY |

颜色来自 RGB 相机，保存在网格顶点中，不再使用高度伪彩色。
默认显示/导出范围为 `base_link` 中 `[0.15, -0.65, -0.025]` 至 `[0.8, 0.65, 0.6]` 米，
覆盖目标墙体、球及附近地面。范围裁剪只影响输出，不合成场景几何。
网格更新上限为 1 Hz，实际速度受仿真和计算负载影响。

- `config/gazebo.yaml`：输入、分辨率、范围、频率。
- `config/high_initial_positions.yaml`：新场景初始关节角。
- `config/scan_views.yaml`：扫描姿态；关节顺序写在文件中。
- `config:=/absolute/path.yaml`：启动时覆盖感知配置。
- `RECON_CONDA_ROOT`：conda 根目录，默认 `$HOME/miniconda3`。

默认导出：`~/nvblox_output/ur7e_reconstruction/base_link_surface.ply`。
网格保留被观测的表面；没有观测到的背面、接触地面的底部及墙体埋在地面以下的部分不会凭空补齐。
当前针对静态场景，没有动态物体清理或通用机器人自身过滤。

## 验证

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate whole_control
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
python -m ur7e_reconstruction.verify --output /tmp/reconstruction_check.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_GPU_TESTS=1 python -m pytest src/ur7e_reconstruction/test -q
python -m ur7e_reconstruction.evaluate --output /tmp/scene_accuracy.json --plot /tmp/scene_comparison.png
python -m ur7e_reconstruction.check_scan --output /tmp/scan_clearance.json
```

最后一项从保存的 PLY 计算几何误差、逐物体/逐墙面覆盖率、球拟合半径和实测颜色，
并生成参考几何与实测网格对比图。参考 YAML 只用于验证，不参与图像融合或补全。
验收阈值为 2.5 cm 距离内每个物体覆盖率 ≥90%、非地面顶点误差 P95 ≤2.5 cm，
以及蓝色墙面、红色球的颜色检查。壁面只评价地面以上外表面，球排除最底部 1 cm 接触带。
本机完整测试结果和证据位置见 [TEST_RESULTS.md](TEST_RESULTS.md)。

`check_scan` 使用实际 URDF 碰撞网格顶点，对固定扫描关节插值路径采样检查墙体间距；
它是针对当前场景的离线检查，不是连续碰撞检测或通用运动规划器。
