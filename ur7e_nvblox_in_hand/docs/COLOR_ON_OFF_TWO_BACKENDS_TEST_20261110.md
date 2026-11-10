# 两套感知实现：颜色开启/关闭的静止仿真对照

日期：2026-11-10。只通过启动参数切换颜色模块；未修改现有融合源码、配置、依赖或实际机械臂状态。实验脚本和输出保存在artifacts，不是对生产感知流程的更改。

## 1. 结论

对 `ur7e_nvblox_in_hand`（Python wheel接口）及 `nvblox_in_hand`（官方ROS C++适配）各执行颜色开启、关闭两组测试。使用同一段静止仿真输入录包、相同体素/量程与主要融合参数，每轮从全新Mapper开始。

结果：关闭颜色没有改善当前场景的Mesh覆盖率、网格规模或相对输入深度的误差。两套实现均存在相同位置的缺面和边缘误差；颜色开关主要改变顶点颜色。不能把颜色模块当作本轮几何异常的主要原因。

这不是对昨日实机灯板的完整重现，也没有验证所有动态场景；不能据此宣布实机颜色投影或颜色停滞问题已解决。

## 2. 两种方案在代码层面的区别

| 项目 | 开启颜色 | 关闭颜色 |
|---|---|---|
| Python启动参数 | enable_color:=true | enable_color:=false |
| C++实际参数 | use_color=true | use_color=false |
| 深度TSDF | 保持开启 | 保持开启 |
| ESDF与Mesh几何 | 从TSDF计算 | 仍从TSDF计算 |
| 颜色层 | 接收RGB并投影到已有几何 | 不执行颜色积分，网格默认灰色 |
| 潜在性能影响 | 增加颜色转换、GPU积分及相关调度成本 | 减少该部分负载 |
| 是否理论上必然提升几何 | 否 | 否 |

Python代码在work()中分别调用backend.depth()/backend.color()；C++官方Mapper::integrateColor()读取const TSDF并更新ColorLayer。颜色积分不直接改写TSDF距离。颜色模块可能通过系统负载间接影响接收到的深度帧数，但不能推导它必然造成边缘缺面。

本轮关闭Mapper颜色处理，相机/录包仍保留RGB。这隔离的是融合模块开关，不是“关闭RGB硬件采集后”的全系统功耗或USB带宽实验。

## 3. 输入、隔离条件和方法

- 仿真：项目自带UR7e/D435/障碍物场景，640×480；虚拟机械臂停在既有观察姿态。
- 隔离：ROS_DOMAIN_ID=136，Gazebo端口11566；无真实硬件控制。
- 体素0.03m、最大融合距离3m、深度/颜色限频40Hz、ESDF更新/导出上限33.4Hz、Mesh/层调度5Hz，关闭衰减和半径清图。
- 每组使用独立进程、新地图；不叠加旧地图。
- 先录制，再停止实时仿真源，四组依次回放同一bag；没有将实时源和回放源混在一起。
- 保持ESDF和Mesh订阅以触发导出，禁用额外体素层显示；无RViz窗口参与测量。

固定录包：19.357秒、约509.6MiB，包含389张深度图、318张RGB、各自CameraInfo、TF、静态TF、时钟和关节信息。318张RGB都有精确配对CameraInfo；386张深度图有精确同时间CameraInfo。Python采用CameraInfo缓存，C++采用ExactTime配对，因此两者可能接受不同输入数量；相同录包不等于所有后端实际融合完全相同的帧集合。

录包深度平均约20.1Hz、RGB约16.4Hz，低于30Hz目标。本轮是同输入几何对照，不是满速30Hz吞吐量或实际端到端延迟基准。

参考深度与位姿在相同静止姿态采集，Mesh保存于回放结束后；原始深度参考不是每一融合帧的同步真值。相机不动使同视角比较有效，但不能当作动态真值评测。

## 4. 核心几何结果

将每轮Mesh从base_link变换到参考深度光学坐标系，使用相同K、640×480光线投射生成Mesh深度图，与原始深度比较。

| 实现 | 颜色 | 顶点 | 三角面 | 有效深度覆盖 | 深度误差中位数 | 深度误差P95 |
|---|---|---:|---:|---:|---:|---:|
| ur7e_nvblox_in_hand | 开 | 3006 | 4306 | 84.71% | 0.335mm | 17.797mm |
| ur7e_nvblox_in_hand | 关 | 3006 | 4306 | 84.71% | 0.335mm | 17.799mm |
| nvblox_in_hand | 开 | 3006 | 4306 | 84.71% | 0.335mm | 17.800mm |
| nvblox_in_hand | 关 | 3006 | 4306 | 84.71% | 0.335mm | 17.799mm |

原始有效深度307200像素，Mesh重叠260237像素，约15.29%有效深度区域没有对应Mesh表面。误差只在重叠区域计算，不能掩盖这些缺失像素。它衡量对本轮输入的一致性，不是绝对真实精度。

各后端颜色开关之间进一步对比：

| 指标 | Python ON/OFF | C++ ON/OFF |
|---|---:|---:|
| 投影轮廓交并比IoU | 1.000 | 1.000 |
| 重叠投影深度差P95 | 0mm | 0mm |
| 最大投影深度差 | 0.511mm | 0.214mm |
| 双向顶点最近距离最大值 | 0.136mm | 0.084mm |

“P95=0”不等于每个浮点数完全相等，最大差值给出少量边界/浮点差异。其规模远小于30mm体素，未改变整体轮廓或缺面覆盖。

## 5. 图片与颜色统计

![同相机视角下的四组Mesh几何与开关差值](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/color_ab_20261110/color_on_off_geometry.png)

上图颜色是距离伪彩色，不是RGB纹理；所以颜色关闭组仍显示彩色。左右开关的对照目的是判断几何是否改变。右列大面积黑色表示差值接近零，而非无数据的统一标记；有效性和IoU来自独立掩码计算。

![四组导出Mesh的实际顶点颜色](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/color_ab_20261110/color_on_off_meshes.png)

| 实现 | 开启颜色时RGB种类 | 恰为127灰值的顶点比例 | 关闭颜色时 |
|---|---:|---:|---|
| Python | 100 | 11.68% | 100%为127灰色 |
| C++ | 99 | 14.97% | 100%为127灰色 |

灰值比例仅为排查指标，真实灰色也可能等于127；不是精确的未着色标签。本次C++仿真回放颜色计数持续增长，没有复现此前实机“累计仅一次”。

## 6. 融合调用频率：只作本轮运行记录

| 实现 | 颜色 | 深度融合调用 | 颜色融合调用 | ESDF更新调用 |
|---|---|---:|---:|---:|
| Python | 开 | 19.68Hz | 16.30Hz | 19.68Hz |
| Python | 关 | 19.40Hz | 0Hz | 19.40Hz |
| C++ | 开 | 17.40Hz | 14.31Hz | 29.26Hz |
| C++ | 关 | 18.09Hz | 0Hz | 30.66Hz |

来自监测器接收的metrics累计计数差/墙钟间隔。短时单轮结果存在调度和采样波动，不足以证明关闭颜色稳定提升某个百分比。C++ ESDF调用可在无新深度时运行，不能与有效新观测频率等同。

监测程序接收大型Mesh时存在消息接收/处理不完整的迹象，保存的mesh/esdf接收Hz不得直接解释为Mapper实际发布Hz；本报告不以这些接收数给出实时性排名。若需要可靠吞吐量结论，应单独使用原始序列化订阅、明确暖机窗口及重复测量。

## 7. 本轮可以排除什么、尚不能排除什么

可以支持：该静止场景下，颜色开关不显著改变Mesh几何；两套实现都在从深度生成相近几何。因此当前缺面不应继续主要归因于颜色模块。

不能支持：所有相机模型一定正确、实机RGB投影一定正确、所有运动条件下颜色负载都无影响，或者3cm参数一定足够。

下一步最有价值的是检查同一缺面区域：

1. 对应深度像素存在时，哪些TSDF角点没有获得足够有效权重？
2. TSDF零交叉有效但Mesh没有三角面时，是邻块缺失、角点门限还是网格提取条件导致？
3. 固定该录包，对体素、截断与Mesh权重单因素比较，不能同时修改多项。

不建议为了让画面更平滑而直接给Mesh补洞掩盖缺失。

## 8. 手工复现的直接命令

两套按顺序运行，不同时启动。以下命令复用本次固定录包，不启动机械臂或相机。每次切换颜色应退出旧Mapper并启动全新进程。

### 8.1 Python版终端A

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true run_rviz:=true \
  voxel_size:=0.03 max_range_m:=3.0 enable_color:=false \
  depth_hz:=40.0 color_hz:=40.0 esdf_hz:=33.4 esdf_view_hz:=33.4 \
  mesh_hz:=5.0 layer_hz:=5.0 decay_hz:=0.0 clearing_hz:=0.0
```

### 8.2 C++版终端A（先退出Python版）

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 launch nvblox_in_hand realsense_example.launch.py \
  start_scene:=false launch_camera:=false use_sim_time:=true run_rviz:=true \
  rviz_mode:=diagnostic mapper_backend:=diagnostic \
  voxel_size:=0.03 max_range_m:=3.0 enable_color:=false \
  depth_hz:=40.0 color_hz:=40.0 esdf_hz:=33.4 esdf_view_hz:=33.4 \
  mesh_hz:=5.0 layer_hz:=5.0 decay_hz:=0.0 clearing_hz:=0.0
```

颜色开启对照：退出后将enable_color:=false改为true，重新启动。C++ RViz需要勾选Official incremental RGB Mesh；灰色Mesh同样可查看几何。比较纯表面时关闭ESDF点显示。

### 8.3 对应环境终端B

```bash
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
# 先source终端A相同工作区的env.sh
ros2 bag play /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/color_ab_20261110/input_bag
```

录包已包含/clock，不添加--clock，不同时运行实时仿真源，也不使用循环回放混入时间回退。图像和TF已经录制，不需要另起静态安装TF。

## 9. 数据索引与结束状态

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/color_ab_20261110/`

- input_bag/：四组共同输入，约510MiB。
- reference/：原始深度点云、RGB、K与图像时刻位姿。
- python_color_on/、python_color_off/、cpp_color_on/、cpp_color_off/：每轮command、最终参数、日志、Mesh、监测及投影结果。
- summary.json：全部结果与ON/OFF几何对比。
- input_pair_counts.json：录包图像/内参精确时间配对统计。
- color_on_off_geometry.png：几何深度对照。
- color_on_off_meshes.png：实际顶点色的网格对照。
- run_four.py、monitor_run.py、analyze_four.py：本轮实验脚本，不是生产启动入口。

四组Mesh均成功导出。结束阶段的SIGINT可能产生KeyboardInterrupt日志，发生于保存之后；不应误当运行中的融合失败。测试结束后已停止本轮场景、回放与Mapper，不留下真实设备控制进程。
