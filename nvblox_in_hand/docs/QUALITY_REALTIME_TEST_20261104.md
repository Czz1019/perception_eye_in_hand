# nvblox_in_hand 重建质量与实时性实测报告

测试日期：2026-11-04。测试对象：当前安装的 nvblox_in_hand C++ 适配节点、官方 ROS nvblox 核心、Gazebo D435 场景与真实 RViz 窗口。

后续默认目标已改为30 Hz。本报告保留修改前5 Hz/1 Hz实测；复现时需显式传入 color_hz:=5 esdf_hz:=5 mesh_hz:=5 layer_hz:=5 esdf_view_hz:=1。新配置结果见 [30Hz测试报告](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/COLOR_ESDF_30HZ_TEST_20261104.md)。

本报告中的截图来自实际运行窗口，RGB 来自实际仿真话题，网格来自原生 save_ply 服务。没有生成示意重建图，也没有用场景真值作为建图输入。本次未修改功能代码。

## 1. 结论

当前配置下，小幅腕部转动时仍能维持约 4.74 Hz 的 ESDF 调用周期；Mesh 消息约 4.68–4.71 Hz，三维 ESDF 距离带显示约 0.95 Hz。Gazebo 实时率接近 1，RViz 在测试窗口内没有闪退。

目标墙面已重建部分的局部几何诊断较稳定：墙体邻域顶点到场景墙面模型的距离 P95，从静态约 9.05 mm 到最终约 9.42 mm，没有出现明显增长。但整体覆盖不足，原始 RGB 能直接看到机械臂入图、墙面只在画面边缘出现。因此当前主要问题不能概括成“ESDF 算不快”，还包括观察姿态、机器人自过滤、显示时效与完整性。

本次慢速、小幅运动没有充分复现明显的目标墙面长尾；不能据此宣布此前残影问题已解决，更没有测得长尾清除时间。

## 2. 测试条件和流程

| 项目 | 实际条件 |
| --- | --- |
| GPU | NVIDIA RTX 4060 Laptop GPU |
| 场景 | 新工作区 ur7e_d435_gazebo，两个墙状立方体、球体及地面 |
| 体素/融合范围 | 0.02 m / 3 m |
| 地图模式 | 3d、static_tsdf、base_link |
| 深度输入 | 实际捕获 1280×720，16UC1，camera_depth_optical_frame |
| 调度目标 | 深度 30 Hz、颜色 5 Hz、ESDF 5 Hz、Mesh/层 5 Hz |
| 三维 ESDF 显示 | 1 Hz 上限、0.12 m 距离带 |
| GUI | 打开真实 RViz，显示 RGB、机器人、Mesh；后半段再开三维 ESDF |
| 运动 | 之前的观察姿态，wrist_1 ±0.06 rad，20 秒往复；每次包含平滑接近/回位 |
| 环绕扫描 | 未执行 |

先启动场景，再让机械臂到达已有观察姿态，最后才启动 mapper，从而避免把到达观察姿态的长路径混入初始地图。随后依次采集：Mesh 静态、Mesh 运动、Mesh 停稳、Mesh+ESDF 静态、Mesh+ESDF 运动、Mesh+ESDF 停稳。

运动阶段统计窗口约 27 秒，包含 20 秒往复及脚本接近/回位；静态窗口约 21 秒，停稳窗口约 13 秒。截图操作占约一秒，已包含在相应窗口中。每组只做一轮，结果是短时诊断，不是长期性能保证。

测试观察节点订阅 RGB、深度、Mesh 与 metrics；ESDF 阶段额外订阅 Marker。订阅和截图本身也有开销，本报告不是无探针的纯算法性能测试。

## 3. 真实效果图

### 3.1 原始 RGB：当前视角存在明显局限

![实际仿真相机在腕部运动中的 RGB](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_wrist_motion_rgb.png)

画面右侧出现机械臂，蓝色墙体主要出现在上下边缘，中心大部分为地面。说明之前的观察姿态虽然能采集局部几何，但没有让完整目标处于理想视场。小幅 wrist_1 转动不会自动补齐背面、墙角和遮挡区域。

抽查的静态深度范围约 0.225–0.805 m，全部为正值且落在 3 m 范围内。因此这些抽样帧的目标缺失不能简单归因于超出 3 m。深度全为正不等于所有目标表面都被看见：射线可能首先击中地面或机械臂。

### 3.2 真实 RViz：静态 Mesh

![静态 Mesh 与机器人、相机图像](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_static.png)

可见局部平面已成形，边界有体素化锯齿和缺口。机器人模型与地图重叠会遮挡部分表面；RobotModel 是 URDF 显示，不能把它本身当成重建出的机械臂。相机原始输入已经包含机器人，所以地图中出现机器人相关表面的风险仍然存在，需要独立自过滤验证。

### 3.3 真实 RViz：小幅运动后的 Mesh

![小幅腕部运动后的 Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_stopped.png)

运动前后相同 RViz 视角下，目标局部表面没有出现非常明显的整体双层偏移；网格数量有所增加。但缺口、局部碎片和有限观察范围仍存在，不能仅凭画面平整就认定完整重建合格。

### 3.4 真实 RViz：运动中的 ESDF

![Mesh 与三维 ESDF 在腕部运动中的真实显示](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_esdf_wrist_motion.png)

### 3.5 真实 RViz：停稳后的 ESDF

![停稳后的三维 ESDF 距离带](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_esdf_stopped.png)

ESDF 显示为表面周围的彩色距离带：内部红色，外部绿到蓝。它不是实体物体的彩色外观，外部带宽也不能直接解释为障碍物被错误加厚或长尾。当前只显示 observed 且距离绝对值不超过 12 cm 的体素；画面空白可能是 unknown，也可能是距离带外的有效自由空间。

截图右下角约 31 fps 是 RViz 渲染帧率，不是深度融合或 ESDF 更新频率。真实距离带消息约 0.95 Hz，即画面中的场数据平均约每 1.05 秒更新一次。

## 4. 实时性结果

所有 Hz 使用墙钟时间。ESDF 是方法调用周期；未测 GPU 完成，不保证每次调用都有新的变化块。

| 阶段 | 深度融合 Hz | 颜色融合 Hz | ESDF 周期 Hz | Mesh 消息 Hz | ESDF 显示 Hz | 仿真实时率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Mesh 静态 | 19.37 | 4.72 | 5.06 | 4.79 | 未订阅 | 1.001 |
| Mesh 腕部运动 | 18.52 | 4.51 | 4.74 | 4.71 | 未订阅 | 1.000 |
| Mesh 停稳 | 17.71 | 4.61 | 4.70 | 4.67 | 未订阅 | 0.999 |
| Mesh+ESDF 静态 | 17.95 | 4.55 | 4.71 | 4.65 | 0.950 | 0.999 |
| Mesh+ESDF 腕部运动 | 18.05 | 4.51 | 4.74 | 4.68 | 0.950 | 0.998 |
| Mesh+ESDF 停稳 | 17.47 | 4.55 | 4.72 | 4.59 | 0.948 | 1.003 |

![实际测量的处理和发布频率](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/measured_frequencies.png)

| 阶段 | ESDF 主机调用样本均值 | 附加显示样本均值 | 附加显示样本 P95 |
| --- | ---: | ---: | ---: |
| Mesh 静态 | 0.59 ms | 未执行 | 不适用 |
| Mesh 腕部运动 | 1.81 ms | 未执行 | 不适用 |
| Mesh+ESDF 静态 | 1.53 ms | 24.07 ms | 27.26 ms |
| Mesh+ESDF 腕部运动 | 1.31 ms | 25.09 ms | 30.30 ms |
| Mesh+ESDF 停稳 | 0.92 ms | 24.58 ms | 28.39 ms |

耗时是约每秒 metrics 中最近一次操作值的样本统计，P95 不是所有调用的逐次 P95。显示耗时包含序列化、CUDA 同步等待、CPU 筛选和消息发布，不等于纯拷贝耗时；ESDF 主机调用耗时也不等于 GPU 全部工作完成耗时。

控制台 28 个统计窗口中，ESDF delay 的滚动均值范围为 45–51 ms，深度融合 delay 的滚动均值范围为 19–27 ms。这不是逐帧延迟最小值/最大值，也没有包含完整屏幕端到端链路。Marker 使用发布时刻打戳，不能仅从 Marker header 得到原始图像到显示的完整帧龄。

## 5. 重建质量的局部数值诊断

除肉眼截图外，分别导出了静态、首次运动后及最终 Mesh。核对 TF 后，本次 world 与 base_link 的变换为单位变换。使用场景配置中的两个静态墙体作为参考几何，仅作离线评价，没有反向输入 mapper。

### 5.1 方法及限制

在墙体包围盒外扩 8 cm、z 为 0.02–0.58 m 的邻域内，计算 Mesh 顶点到最近参考墙面表面的绝对距离。距离是顶点加权，受三角网格分布影响，不是均匀面积采样的精度，也没有包含整个场景的异常点。

另外以约 1 cm 间距采样墙体地上表面，统计其中距任意 Mesh 顶点不超过 3 cm 的比例。该指标衡量模型表面的几何邻近覆盖，包含背面等本来不可见区域；不是 observed 体素比例，也不能直接取补集称为 unknown 比例。它可能受到邻近非目标几何影响。

### 5.2 结果

| 指标 | 静态 | 首次运动后 | 最终 |
| --- | ---: | ---: | ---: |
| Mesh 顶点数 | 4,196 | 4,810 | 4,798 |
| 三角形数 | 6,129 | 7,091 | 7,084 |
| 墙体邻域内顶点数 | 1,098 | 1,148 | 1,134 |
| 邻域顶点表面距离均值 | 1.41 mm | 1.46 mm | 1.48 mm |
| 邻域顶点表面距离 P95 | 9.05 mm | 9.34 mm | 9.42 mm |
| 邻域内距墙面 ≤2 cm 的顶点比例 | 98.09% | 98.00% | 97.97% |
| 墙体 cube1 参考表面邻近覆盖 | 39.97% | 41.16% | 40.78% |
| 墙体 cube2 参考表面邻近覆盖 | 24.30% | 24.90% | 24.90% |

这些数字说明：已形成的局部墙面比较稳定，但整体覆盖仅有限改善。不能据此宣称系统达到 1.4 mm 精度——轴对齐仿真平面、已观测区域筛选和顶点分布都会使这个局部均值偏小。当前地图分辨率仍是 2 cm。

墙体邻域内没有发现距参考表面超过 4 cm 的顶点，但这一检查不覆盖机器人周围、ROI 外和完全未重建的区域，不能作为“全图没有残影”的证明。此前明显长尾需要用原始复现场景、速度和轨迹进一步测试。

## 6. 对已有问题的判断

### 墙角 unknown 与不完整

当前最直接的证据来自原始 RGB：观察姿态偏向地面，机械臂入图，完整墙面没有居中覆盖。小幅转动能增加少量局部观察，但无法看到所有墙角或背面。当前部分空白也受 12 cm 显示距离带限制。

优先优化视角目标、俯仰与高度，让目标和角落实际进入深度视场；增加 observed 状态诊断。无需默认环绕，可以先设计少量针对墙角的可达视角。未观测区域应继续保持 unknown。

### 运动残影与长尾

此次目标墙面慢速转动后的数值变化较小，未取得足够证据定位此前长尾根因。现有输入依然存在机器人入图；程序也仍使用 static_tsdf、没有新接入机器人自过滤。成像时间与 TF 的真正一致性尚未通过独立真值测量验证。

建议记录关节/光学 TF、成像测量时刻和深度帧龄，对机器人附近单独评价。先区分 TSDF 错表面、ESDF 距离传播和显示滞后，再选择时间修正、自过滤、局部清除或显示解耦。不得以开启强衰减替代输入一致性修复。

### 实时显示

当前 ESDF 调用周期接近 5 Hz，但用户看到的三维距离带只有约 1 Hz。即使 RViz 显示 31 fps，也可能反复绘制同一份旧数据。这一显示节流本身就会造成“更新慢”的观感。

应先把全层同步导出拆分计时，考虑 GPU 筛选与变化块增量显示，再评估提高显示频率。直接把显示改为 5 Hz 会增加负载，不能先承诺地图时效提升。

## 7. 与 11 月 3 日简单测试如何比较

此前初始姿态、无 RViz/Mesh 消费者的测试显示导出约 106 ms；此次到达观察姿态后才建图，并运行 RGB/Mesh/RViz，显示导出样本约 24–25 ms。两次地图分配、相机姿态和负载不同，且本轮没有修改算法，不能将耗时差称为优化成功或认为旧数据无效。

两次共同支持：显示与底层 ESDF 不是同一频率，导出开销需要独立统计。后续公平比较应使用同一录制输入、同样消费者、同样地图状态，交替运行并重复多轮。

## 8. 复现入口

完整记录脚本位于 [run_measurement.py](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/run_measurement.py)。该脚本会启动仿真、发送上述小幅仿真运动、打开 RViz 和保存结果；运行前必须停止旧场景。它使用固定输出目录，直接重跑会覆盖本报告数据，应先复制归档或改为新的测试目录。

人工操作仍可参考 [仿真测试手册](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/SIMULATION_TEST_GUIDE_20261103.md)。为了复现本报告“先就位再建图”的条件，按以下顺序操作：

```bash
# 终端 A：仅启动场景
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch ur7e_d435_gazebo scene.launch.py gazebo_gui:=false
```

```bash
# 终端 B：先到之前的观察姿态
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
../run_ur7e_view_sweep.sh --check-only
../run_ur7e_view_sweep.sh --seconds 0

# 到位后启动 mapper，不再重复启动场景
./run.sh start_scene:=false run_rviz:=false \
  mapper_backend:=observed_view esdf_mode:=3d \
  enable_esdf_view:=true voxel_size:=0.02 max_range_m:=3.0 \
  color_hz:=5 esdf_hz:=5 mesh_hz:=5 layer_hz:=5 esdf_view_hz:=1
```

```bash
# 终端 C：打开报告相同配置的 RViz
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
rviz2 -d artifacts/quality_realtime_20261104/mesh_view.rviz --ros-args -p use_sim_time:=true
```

```bash
# 终端 D：记录/查看 metrics；另开同环境终端执行小幅转动
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 topic echo /nvblox_node/metrics
# 其他终端执行：../run_ur7e_view_sweep.sh --seconds 20 --amplitude 0.06
```

需要三维 ESDF 时在 RViz 勾选对应 Marker，或关闭原 RViz 后用同目录 esdf_view.rviz 打开。不要同时开两份界面再与单界面结果作公平比较。

## 9. 证据和验证边界

- [结构化性能与输入结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/results.json)
- [原始采样计数、时钟和发布记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/samples.json)
- [局部质量诊断数据](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/quality_metrics.json)
- [实际运行参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/runtime_parameters.yaml)
- [mapper 日志与延迟统计](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mapping.log)
- [静态网格](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_static.ply)、[运动后网格](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_after_motion.ply)、[最终网格](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/mesh_final.ply)
- [局部质量计算脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104/analyze_quality.py)

首轮因截图辅助程序 Python Qt 库冲突而中止，其记录保存在 artifacts/quality_realtime_20261104/attempt1_qt_capture_failure，不与本轮结果混合。改用隔离系统 Qt 的截图环境后，完整流程成功。没有修改系统依赖或感知程序。

本轮结束后已关闭自行启动的 Gazebo、mapper、RViz，并核对端口 11457 已释放。没有测试实机、完整环绕、长时间大地图、动态障碍、GPU 完成延迟或严格屏幕端到端延迟；报告不对这些条件作性能承诺。
