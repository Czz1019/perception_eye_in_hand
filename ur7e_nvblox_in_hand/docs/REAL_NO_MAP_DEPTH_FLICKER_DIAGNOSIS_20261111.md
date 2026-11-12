# 实机无重建与深度闪黑：测试结果、原因与解决方案

后续记录：[恢复后的30秒实机测试结果与分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_TEST_RESULTS_AND_ANALYSIS_20261111.md)。原故障诊断保留当时状态，恢复后结果单独归档。


日期：2026-11-11。功能包：`ur7e_nvblox_in_hand`；UR7e与D435i，ROS域135。本轮为只读诊断：没有移动机械臂、重启相机、修改源代码或设备参数，没有主动发布手眼TF改变运行状态。

## 1. 结论

**本轮已确认的无重建直接原因：手眼静态TF启动在默认域0，相机及Mapper在135域，机器人TF树与相机TF树在135域没有连通。** Mapper虽然接收到图像且CUDA已就绪，但无法取得图像时刻的相机位姿，因而没有执行深度融合、颜色融合与ESDF更新。

**深度闪黑是另一问题，不能与无TF混为一谈。** 18秒内实际收到320帧深度，未发现全零/全无效帧，有效像素比例中位数约82.18%；再用轻量可靠订阅检查10秒，收到296帧、约29.56Hz，也未发现抽样全无效帧。这不证明故障从未发生，但本轮未复现持续的全黑输入。

保存的原始深度有少量远点，首/末快照最大值约5.59/5.74m，而主体约半米。按整帧最小最大值显示时主体会很暗；RViz自动归一化与深度显示问题值得优先隔离，而不是直接升级SDK或改变融合算法。

此外，**测试期间的实际Mapper体素为0.01m**。用户附的早先启动日志为0.005m，两者不是同一次启动条件；本轮没有将体素从5mm改成1cm，不能把本轮诊断称为5mm建图性能/精度验收。

## 2. 已完成的现场测试

### 2.1 深度、颜色、时间与TF：18秒

| 检查 | 实际结果 | 解释 |
|---|---:|---|
| 深度收到帧数 | 320 | 较重诊断器的BEST_EFFORT接收量，不代表传感器实际帧率 |
| 颜色收到帧数 | 540，约30Hz | 颜色链路正常接收 |
| 全零/无效深度帧 | 0 | 在已收到的帧中统计 |
| 有效像素<10%的帧 | 0 | 未观察到大面积瞬间失效 |
| 深度有效比例P5/P50/P95 | 82.06% / 82.18% / 82.26% | 约18%像素无有效深度，但不是全帧无效 |
| 0.2–2.05m有效比例P50 | 82.16% | 与Mapper量程吻合 |
| 图像到达时间戳年龄P50/P95 | 34.74 / 35.97ms | 相对于当前系统时钟，未显示普遍超过0.5s |
| 关节收到消息 | 8,994，约500Hz | 机器人反馈存在 |
| 图像时刻Depth TF成功 | **0 / 320** | 相机与机器人两棵树未连接 |
| 已确认TF错误 | 两个frame不属于同一棵树 | 不是正常运动造成的短暂未来外推 |

最后约1秒的待检查帧尚未超时，因此错误计数294不等于320；320帧的成功TF数仍为0。没有通过latest TF替代图像时刻TF。

### 2.2 轻量RELIABLE深度对照：10秒

只订阅深度，空间抽样计算有效率，不再执行密集TF检查：

- 296帧，接收约29.56Hz；编码16UC1，即毫米。
- 抽样全无效帧0。
- 最后帧抽样有效比例约84.29%，有效距离中位数0.490m，P95为0.531m。
- 本段与前段并非完全同一时间、同一统计像素集合，不能将有效率变化直接归因于相机参数。

这说明第一段接收17.78Hz不能直接解释成“相机只有18Hz”；诊断负载和订阅接收路径会影响接收统计。轻量测试也不保证持续每帧不丢失。

![深度有效率和时间](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/depth_validity_timeline.png)

## 3. 为什么Mapper ready，却没有Mesh和ESDF

### 3.1 进程环境已核对

诊断读取了相关进程的ROS环境字段，没有读取其余环境内容：

| 进程 | ROS_DOMAIN_ID | ROS_LOCALHOST_ONLY | RMW |
|---|---|---|---|
| 静态手眼TF，PID9476 | **未设置，默认0** | 0 | 未显式设置 |
| D435i，PID10921 | 135 | 1 | rmw_cyclonedds_cpp |
| 本段Mapper，PID12055 | 135 | 1 | rmw_cyclonedds_cpp |

观察到静态TF进程存在，不等于本域能收到它。相机和Mapper虽然来自正确环境，静态TF仍是在另一个终端按旧环境启动的。修改其他终端环境变量不会更改已运行的静态TF进程。

域135实际TF中有：

```text
base_link → ... → flange → tool0

camera_link → camera_depth_frame → camera_depth_optical_frame
            → camera_color_frame → camera_color_optical_frame
```

但缺少：

```text
                  tool0 → camera_link
```

### 3.2 代码如何阻止无位姿融合

[node.py:154](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:154)中，`candidate()`核对尺寸/frame、输入年龄、融合门限后，执行：

```python
tf = buffer.lookup_transform(global_frame, image.header.frame_id,
                             Time.from_msg(image.header.stamp))
```

如果TF查询失败，它返回None，暂不提交融合；[work():220](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:220)遇到None直接跳过该路输入。只有成功融合深度才会设置dirty_depth并允许后续ESDF更新。

本轮计数变化：

| 计数 | 首个metrics | 最后metrics |
|---|---:|---:|
| depth_received | 330 | 840 |
| color_received | 348 | 858 |
| depth_overwritten | 326 | 836 |
| color_overwritten | 344 | 854 |
| depth_integrated | 未出现，按0解释 | 未出现，按0解释 |
| color_integrated | 未出现，按0解释 | 未出现，按0解释 |
| esdf_updated | 未出现，按0解释 | 未出现，按0解释 |
| esdf_points_published | 1 | 1 |

队列最多4帧，等待不到TF时不断被新图像覆盖。这种覆盖**不是本轮GPU融合太慢的证据**，而是未满足位姿条件。发布过一次的计数也不证明生成了非空距离场。

CUDA日志中的`Mapper ready`仅表示核心及桥接初始化成功；`error`为空是因为TransformException被作为可重试条件处理，而非致命异常。该实现避免错误位姿进入地图，但缺少持续TF缺失的明确计数和提示，是代码可观测性上的不足。

## 4. 深度为什么会看起来闪黑

必须分别判断数据、接收和显示：

| 情况 | 是否本轮已证实 | 对策 |
|---|---|---|
| 原始深度整帧全零 | 未复现 | 闪黑当时保留原帧与时间，不能用主观显示判断 |
| 局部无效深度 | 已观察，约18% | 结合遮挡、材质和距离检查，不能当作free |
| 图像接收有间隔 | 已观察 | 轻量可靠订阅对照，避免把测量负载当相机帧率 |
| 全图极值拉伸导致主体暗 | 保存帧可直接演示 | 使用固定显示范围，只改查看方式 |
| RViz深度显示故障 | 尚未与本机故障同步对照 | 用rqt_image_view及原始数值交叉验证 |
| 发射器逐帧开关 | 本轮参数为false | 没有证据支持它是当前根因 |
| 相机跨机硬件同步等待 | inter_cam_sync_mode=0 | 没有证据支持从机等待问题 |
| HDR交替曝光 | hdr_enabled=false | 不应盲目归因HDR |

相机当前参数：emitter_enabled=1、emitter_on_off=false、laser_power=150、深度自动曝光开启、depth_profile=640x480x30、enable_rgbd=false、enable_sync=true、global_time_enabled=true。仅这些静态参数不能证明所有硬件时刻都正常，但不支持直接关闭激光或更换SDK作为第一步。

### 4.1 原始数据与显示缩放

![相同原始深度的两种显示](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/depth_fixed_scale_comparison.png)

左列按整帧最大值约5.6m作灰度拉伸，主体半米左右会较暗；中列使用固定0.2–0.8m显示，轮廓更容易看清；右列是有效掩码。这只是同一保存数组的显示对照，不是相机参数优化，更没有填洞。固定范围外的颜色被裁到色标端点，不代表距离被修改。

RViz Humble的[官方图像显示代码](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/image/ros_image_texture.cpp)会对16UC1作归一化，自动模式使用图像极值及多帧中位窗口，因此极值变化可能改变亮度。官方[NVIDIA Isaac Sim说明](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/ros2_tutorials/bridge_configuration/tutorial_ros2_python.html)也记录了RViz2深度显示可能出现黑帧，并建议用rqt_image_view核对。该说明是显示层已知现象的参考，不证明当前真实D435i闪黑必然由同一实现缺陷造成。

## 5. 解决方案与操作顺序

### 第一步：修正手眼TF的域，先恢复建图输入条件

在原来启动static_transform_publisher的终端Ctrl+C，**只退出那条静态TF命令**，不停止机械臂驱动或相机。然后在该终端执行：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
printf 'domain=%s localhost=%s rmw=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$RMW_IMPLEMENTATION"

ros2 run tf2_ros static_transform_publisher \
  --frame-id tool0 --child-frame-id camera_link \
  --x -0.0406379061113624 \
  --y -0.05902311421350448 \
  --z -0.012384254831755863 \
  --qx 0.0025872583947125077 \
  --qy -0.002933202909281628 \
  --qz 0.9251005977793049 \
  --qw -0.3797019705019513
```

继续使用之前未变的安装标定；若硬件支架拆装过，需要重新确认标定有效性。保持该终端运行，不另发一条以Color optical为child的冲突TF。

另一个相同135环境终端分别检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

不应再出现two or more unconnected trees。

### 第二步：保留或重新启动一个Mapper，并核对实际配置

若当前Mapper仍在运行，恢复TF后理论上可自行继续，无需重启相机；以计数实际增长为准。若已退出，用[实机运行说明](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_MANUAL_SCAN_RUN_COMMANDS_20261111.md)中的命令只启动一套Mapper。

```bash
ros2 param get /ur7e_nvblox use_sim_time
ros2 param get /ur7e_nvblox voxel_size
python3 tools/watch_realtime.py
```

use_sim_time必须false；本次诊断的实际体素是0.01，而附的早先日志是0.005，需按本轮目标明确选择，不混合报告。预期depth_fuse、color_fuse、esdf_update开始增长。

恢复TF后才能评估真实工作区是否覆盖目标。本轮TF从未成功，因此没有计算真实点云在bounding_box内的比例；**尚不能宣称当前边界正确或错误**。目标在范围外时再调整对应边界，不直接改成无界5mm地图。

### 第三步：独立处理深度显示

优先使用：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择`/camera/camera/depth/image_rect_raw`，与RGB、原始深度数值对照。如果只在RViz闪黑，不改变建图原始输入。

RViz中展开深度Image显示，关闭Normalize Range，固定Min Value=200、Max Value=800，可观察本次约0.3–0.6m近场。因为输入为16UC1毫米，这里用200/800，**不是0.2/0.8**；若以后改32FC1米才使用米值。固定范围仅用于显示，不能当作Mapper过滤配置。

不要为了预览给Mapper的原始深度话题启用会将其编码变成彩色图的colorizer。可视化RGB图和用于融合的数值深度必须保留独立语义。

### 第四步：如果仍复现真实无效帧，再查相机链路

保留闪黑时的原始消息、有效像素比例和相机日志；比较单独相机、加入Mapper和加入RViz三个条件。只有确认原始输入确实失效，再依次检查USB线/端口、设备占用、曝光和固件兼容性。本轮未改变曝光、帧率、同步或SDK版本，不能声称这些操作已解决问题。

## 6. 后续代码改善建议（未实施）

1. Mapper新增持续TF缺失的分类统计及节流日志，给出目标frame、图像frame、最近一次错误；不要因非致命等待而静默无限覆盖。
2. 启动自检同时验证ROS域、图像和CameraInfo、完整base→相机TF及真实工作区覆盖；不是只打印CUDA ready。
3. 提供原始深度有效率、范围和过期计数的轻量诊断，显示器单独采用固定范围或明确的自动范围。
4. 复测时记录实际生效参数，分别评价输入健康、融合成功、地图输出及显示；没有融合时不发布“GPU计算频率低”的结论。

## 7. 证据文件与验证边界

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111`。

- [summary.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/summary.json)：18秒深度/TF统计。
- [metrics.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/metrics.json)：现有Mapper计数及实际配置。
- [process_ros_environment.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/process_ros_environment.json)：ROS环境字段。第二次读取时PID12055已退出，前面采集的metrics仍保留；未把进程退出误报为仍运行。
- [camera_params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/camera_params.yaml)：相机当前参数。
- [light_probe.log](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/light_probe.log)：10秒轻量可靠接收结果。
- [probe.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/probe.py)、[light_probe.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_depth_diagnosis_20261111/light_probe.py)：只读测试代码。

本轮没有重发正确域的安装TF，因此**没有完成恢复后的Mesh/ESDF验收**；文中修复后的预期不冒充实测结果。深度未复现全黑的结论只适用于收到的样本和本次短窗口，不能排除更早或更晚的间歇故障。
