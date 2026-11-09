# 三维ESDF仿真：逐终端运行与测试指令

适用目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

本文用于执行当前版本的手动仿真测试，包括机械臂恢复之前的观察姿态、三维ESDF显示、频率统计、原生统计与结果保存。本次仅整理文档，没有再次启动仿真。已有测试数据见文末。

## 1. 测试内容与当前配置

| 项目 | 当前配置 |
| --- | --- |
| 输入 | D435仿真RGB/深度，640×480，目标30Hz |
| 相机位姿 | 机械臂运动学及手眼关系，查询图像时刻TF |
| 体素与范围 | 0.03m；最大融合距离3m |
| 深度/颜色融合门限 | 40Hz；不是把传感器改成40Hz |
| ESDF更新/三维显示门限 | 33.4Hz / 33.4Hz |
| Mesh/地图层 | 5Hz目标 |
| 三维配色 | intensity距离，固定−0.09～0.24m彩虹色带 |
| 运动方式 | 之前的观察姿态；可选末端小幅摆动，不执行环绕 |

**查看三维点云请使用 `run_esdf.sh`。** 默认 `run.sh` 使用官方组件，虽然计算3D ESDF，但不发布项目附加的 `/nvblox_node/esdf_points`。run_esdf.sh显式选择诊断导出，感知主体仍调用官方Mapper。

## 2. 开始前停止旧程序

在原来的启动终端按Ctrl+C，等待Gazebo、Mapper和RViz退出。仅关闭Gazebo窗口不一定停止gzserver。

检查本次端口：

```bash
ss -ltnp 'sport = :11457'
```

如果仍有监听，先核对输出中的PID和对应启动终端，停止那次旧场景。不要盲目批量结束所有ROS/Gazebo进程，也不要同时执行两条场景启动命令。

## 3. 终端A：构建并启动

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
```

构建应完成ur7e_d435_gazebo与nvblox_in_hand两个包。依赖来自 `/home/czz/workspaces/isaac_ros-dev/install`；不要手动source父工程旧install。

可选：先运行既有离线检查。

```bash
./test.sh
```

然后启动：

```bash
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_esdf.sh gazebo_gui:=true
```

保持终端A运行。该入口同时启动本项目场景、诊断Mapper、位姿/路径旁路和诊断RViz；不自动移动机械臂，不执行环绕。

如果已经有同环境的本项目场景，先停止旧Mapper，保留场景，再改用下面这一条；它与上面的完整启动互选：

```bash
./run_esdf.sh start_scene:=false
```

已有场景必须提供对应RGB-D、CameraInfo、机器人TF和时钟；该参数不会修改外部相机的分辨率。

## 4. 终端B：加载相同环境并检查场景

每个新的监测/操作终端都先运行：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
```

检查：

```bash
./check_sim.sh
ros2 node list
ros2 topic list -t
ros2 topic info /nvblox_node/esdf_points --verbose
ros2 param get /nvblox_node esdf_mode
ros2 param get /nvblox_node enable_esdf_view
ros2 param get /nvblox_node esdf_view_hz
ros2 param get /nvblox_node update_esdf_rate_hz
```

预期：esdf_mode=3d，enable_esdf_view=true，两个频率参数为33.4；esdf_points类型sensor_msgs/msg/PointCloud2，Publisher count为1。若出现两个同名Mapper或两个发布器，先排除重复启动。

## 5. 终端B：恢复之前的机械臂观察姿态

以下脚本会发送关节指令，**只用于当前Gazebo仿真**，不连接实机执行；不要与其他关节控制脚本同时运行。它检查domain127、仿真时钟、机器人关节和控制器。

先只检查、不运动：

```bash
/usr/bin/python3 /home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/scripts/view_sweep.py --check-only
```

移动到之前使用的观察姿态，之后保持静止：

```bash
/usr/bin/python3 /home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/scripts/view_sweep.py --seconds 0
```

等待约10秒，观察RGB窗口是否看到障碍物，以及ESDF是否积累有效信息。

若需要测试相机运动时的刷新，在另一个已加载相同环境的终端执行：

```bash
/usr/bin/python3 /home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/scripts/view_sweep.py --seconds 15 --amplitude 0.06
```

它采用末端小幅摆动并返回，不是绕障碍物环绕。指令还包括靠近观察姿态及返回过程，墙钟总耗时可能超过15秒。

## 6. RViz显示设置

默认诊断RViz应具备：

| 设置 | 应用值 |
| --- | --- |
| Fixed Frame | base_link |
| 三维显示项 | Diagnostic ESDF samples |
| Topic | /nvblox_node/esdf_points |
| Style / Size (Pixels) | Points / 3 |
| Color Transformer / Channel | Intensity / intensity |
| Autocompute Intensity Bounds | false |
| Min / Max Intensity | −0.09 / 0.24 |
| Use rainbow / Invert Rainbow | true / false |
| Decay Time | 0 |

选Move Camera，视角选择Orbit，左键拖动旋转，鼠标滚轮缩放。3cm体素是空间精度，3像素是屏幕点大小，两者不同。

先只打开ESDF和机器人模型观察，减少Mesh遮挡；再按需打开Official incremental RGB Mesh查看重建表面。三维ESDF点包括内部与外部的有效距离体素，不是物体表面点云，观察几何精度应结合Mesh。

如果Mapper已运行、RViz被单独关闭，在加载相同环境的终端重新打开：

```bash
./rviz.sh diagnostic
```

不要为重新打开RViz再次启动整个场景或Mapper。

## 7. 终端C：实际频率测试

先执行第4节的环境加载命令。以下命令逐项执行，每项等待约20～30秒，再Ctrl+C结束；需要测试运动阶段时，让另一个终端执行第5节的小幅摆动。

```bash
ros2 topic hz /camera/depth/image_rect_raw --wall-time
```

```bash
ros2 topic hz /camera/color/image_raw --wall-time
```

```bash
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

```bash
ros2 topic hz /nvblox_node/mesh --wall-time
```

额外订阅会产生真实的传输/显示负载，做对照时应保留相同订阅和RViz开关。输入/点云使用约30秒窗口时，15秒运动可能只覆盖其中一部分，应明确记录测试阶段。

### 区分三种频率

- 图像topic hz：接收到的传感器消息频率，不等于实际融合频率。
- esdf_points topic hz：三维显示点云的发布/接收频率，不等于每秒全新地图数量。
- ESDF更新调用：看现有控制台ros/update_esdf、save_rates和save_timings；可能包含没有新深度的调用。

RViz右下角FPS是界面渲染频率，不能代替上述任何一个指标。只看topic hz也无法得出端到端延迟。

## 8. 保存本次测试结果

在终端C建立一个新的结果目录，保留该终端，后续命令复用IN_HAND_TEST_OUT：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
IN_HAND_TEST_OUT="$PWD/artifacts/manual_esdf_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$IN_HAND_TEST_OUT"
echo "$IN_HAND_TEST_OUT"
ros2 param dump /nvblox_node > "$IN_HAND_TEST_OUT/parameters.yaml"
ros2 topic list -t > "$IN_HAND_TEST_OUT/topics.txt"
ros2 topic info /nvblox_node/esdf_points --verbose > "$IN_HAND_TEST_OUT/esdf_topic_info.txt"
./check_sim.sh --output "$IN_HAND_TEST_OUT/scene_check.json"
```

保存一次诊断指标：

```bash
ros2 topic echo /nvblox_node/metrics --once > "$IN_HAND_TEST_OUT/metrics.json.txt"
```

保存一段topic hz记录，手动采样约30秒后Ctrl+C；这是接收频率，不是原生ESDF计算频率：

```bash
ros2 topic hz /nvblox_node/esdf_points --wall-time | tee "$IN_HAND_TEST_OUT/esdf_topic_hz.txt"
```

### 保存官方统计快照

静止预热后、运动前执行：

```bash
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/rates_before.txt'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/timings_before.txt'}"
```

完成一段运动或固定时长观察后执行：

```bash
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/rates_after.txt'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/timings_after.txt'}"
```

服务回复应为success。save_rates为原有滚动窗口统计；要计算整个测试段实际融合/ESDF调用率，应取save_timings中ros/depth/integrate、ros/color/integrate、ros/esdf的两次累计样本数差，再除以两次快照间实际墙钟时间。服务命令本身存在耗时，手工粗测应如实标注；不要直接把累计次数当作Hz。

保存Mesh和原生地图，建议在频率采样结束后操作，避免文件写入干扰测试：

```bash
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/reconstruction.ply'}"
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath "{file_path: '$IN_HAND_TEST_OUT/map.nvblx'}"
```

### 截图

使用系统截图工具截取RViz窗口，将文件保存到上面打印出的结果目录。建议保留：

1. 静止、完成初次融合后的ESDF窗口，命名esdf_static.png。
2. 运动过程窗口，命名esdf_motion.png。
3. 停止后约5秒的窗口，命名esdf_stopped.png。
4. Mesh表面及RGB输入窗口，命名mesh_rgb.png。

比较截图时保持Orbit视角、缩放、Mesh开关和点大小一致；记录色带上下界。截图只反映显示效果，精度仍需与场景真值/几何测量比较。

## 9. 推荐的简单测试记录表

| 项目 | 本次填写 |
| --- | --- |
| 日期/结果目录 | |
| 输入分辨率/体素/范围 | |
| ESDF与显示门限 | |
| 静止或运动阶段/采样时长 | |
| 深度输入/颜色输入Hz | |
| 实际融合深度/颜色Hz（有计数差时填写） | |
| ESDF调用Hz（有计数差时填写） | |
| esdf_points发布Hz / Mesh Hz | |
| metrics的esdf_view_points | |
| unknown盲区、残影及自体进入视场情况 | |
| 截图文件名 | |

本版metrics存在esdf_view_points等现有字段，可用于查看最近导出点数；原生字段说明见[参数话题文档](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/PARAMETERS_AND_TOPICS.md)。不要将RViz目测点密度当作实际点数。

## 10. 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| esdf_points话题不存在 | 确认使用run_esdf.sh；默认run.sh官方组件没有项目三维发布器；话题不是旧ur7e_nvblox命名 |
| 只有parameter_events与rosout | 检查终端A是否启动失败，以及domain、RMW、env.sh是否一致 |
| Publisher count=1但无消息 | 检查深度、CameraInfo、图像时刻TF、/clock、ESDF开关和3d模式；等待首批有效融合 |
| topic hz有消息，RViz空白 | Topic与Fixed Frame、显示Enabled、视角/焦点及时间源；使用diagnostic配置 |
| 相机中看不到障碍物 | 先执行view_sweep.py --check-only，再用--seconds 0恢复观察姿态 |
| Gazebo端口已占用 | 回到旧启动终端停止旧gzserver，核对11457监听者，不重复起场景 |
| 机械臂状态不一致 | check_sim检查关节与TF；确保只有一个本项目场景与机器人状态发布链 |
| 颜色仍单调 | 确认使用新源配置而非旧artifact RViz；三维Intensity范围应为−0.09～0.24m |
| 出现unknown空白 | 不代表自由空间；单视角遮挡与未观测区域不会因调高Hz或换色自动补全 |
| 运动后残影 | 保存运动/停止截图，进一步检查图像时刻TF、自体遮挡、融合历史等；本轮配色未修复这些问题 |

lo不支持multicast的CycloneDDS提示本身不能直接证明通信失败；应结合发布器数量、节点和真实消息判断。

## 11. 已有测试结果，供对照

2026-11-05上一轮新配色测试，640×480、3cm体素、3m范围、末端小幅摆动时：深度融合29.75Hz、颜色融合29.94Hz、ESDF更新调用32.76Hz、三维点云发布29.44Hz、Mesh4.74Hz；停止后18184点。

这些是特定场景、姿态和订阅负载下的测量，不是当前任何运行条件下的保证。先前还发现点云关联输入时间戳重复，不能把约30Hz发布解释成每秒30份全新地图。

- [发布原因、新配色、实测与截图](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)
- [33.4Hz门限机制、重复时间戳及延迟边界](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_334HZ_AND_COLOR_TEST_20261105.md)
- [整体操作与其他功能开关](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/OPERATIONS.md)

测试结束后回到终端A按Ctrl+C，等待退出；单独打开的RViz终端也按Ctrl+C。检查11457端口是否释放，保留结果目录与本次配置。
