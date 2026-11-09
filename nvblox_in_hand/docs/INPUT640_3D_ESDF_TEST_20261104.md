# 640×480输入下的三维ESDF复测

日期：2026-11-04。只测3D ESDF，没有运行二维ESDF模式。已修改本项目默认输入、构建并完成仿真；没有操作真实机械臂或真实D435。

## 1. 原本的输入是什么

上一轮重构版仿真中，Depth与RGB都是 **1280×720，传感器目标30Hz**。Depth实际编码16UC1（毫米）、RGB编码rgb8；Mapper直接使用各自CameraInfo和光学TF，没有先缩成640×480。

本轮改成 **Depth 640×480 + RGB 640×480，目标仍30Hz**。这是传感器原生生成尺寸的修改，不是先生成1280×720再在Python里缩图。

每幅像素从921,600降至307,200，为原来的1/3。按本轮编码计，Depth+RGB单帧组有效像素数据从4.608MB降到1.536MB；按30Hz约138.24MB/s降到46.08MB/s，不包括DDS、CameraInfo、复制和其他消息开销，不能把它当作实测网络吞吐。

## 2. 修改了哪些代码

| 文件 | 修改与作用 |
| --- | --- |
| src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro | 深度和彩色图像使用camera_width/camera_height参数，默认640/480；保持30Hz和水平FOV |
| src/ur7e_d435_gazebo/launch/scene.launch.py | 声明尺寸参数、检查正整数并传给xacro |
| src/nvblox_in_hand/launch/bringup.launch.py | 顶层把尺寸转发给场景或真实D435入口 |
| src/nvblox_in_hand/config/sensors/d435.yaml | 默认深度和彩色profile改为640x480x30 |
| src/nvblox_in_hand/launch/sensors/d435.launch.py | 按传入尺寸构造两路真实相机profile；本轮只改配置，未实测设备 |

Gazebo插件原有CameraInfo生成函数根据实际图像width/height和水平FOV计算焦距、主点，无需手写旧内参或把Color内参替换成Depth内参。没有修改TSDF、ESDF数学算法、体素精度、调度限频或诊断点云筛选。

未参与当前感知的1Hz红外传感器仍保留旧声明；其中出现1280×720不表示本轮Depth/RGB仍是旧尺寸。输入外部已经运行的相机或bag时，尺寸参数也不会重采样这些外部消息。

## 3. 实际输入与内参核对

运行时两路图像与各自CameraInfo均验证为640×480，frame_id分别为camera_depth_optical_frame、camera_color_optical_frame。

| 项目 | 本轮仿真实际值 |
| --- | --- |
| Depth | 640×480，16UC1；保存的数组shape=(480,640)，uint16 |
| RGB | 640×480，rgb8；保存PNG为640×480 |
| fx / fy | 320.254913 / 320.254913 |
| cx / cy | 320 / 240 |
| 深度/彩色CameraInfo | 各自640×480，图像与标定frame匹配 |

这组数值是仿真针孔内参，不能当作真实D435标定。

**对照限制：**1280×720为16:9，640×480为4:3。水平FOV仍1.57rad时，纵向FOV由约58.68°扩大至约73.70°。因此本轮不只是减少像素数，也改变了纵向可见范围。最大融合距离仍3m，体素仍3cm；不能声称两轮采样视场完全一致。

## 4. 测试条件与方法

独立ROS_DOMAIN_ID=129、Gazebo端口11459；原UR7e+D435和高墙场景。到此前观察姿态，静态10秒，小幅wrist_1 ±0.06rad往复15秒并返回观察姿态，最后停稳8秒。含进入/返回等待的运动统计约22秒，没有环绕。

两轮分别测试：

1. **official3d**：默认官方NvbloxNode组件，原生3D ESDF与Mesh，无自定义点云开销。
2. **diagnostic3d**：同一官方核心加observed三维体素导出，诊断显示目标5Hz。

体素0.03m、融合范围3m，深度/颜色/ESDF目标30Hz，Mesh/层公共发布目标5Hz，衰减与范围清图关闭。两轮均新建地图。保存实际窗口截图、原生地图、Mesh、输入、参数与计数。

实际融合率/ESDF调用率采用官方save_timings原有累计样本数，在静态末与运动末快照间差分除墙钟间隔。端点近似包含约0.1秒统计保存间隙，不是严格GPU帧率；ESDF调用可能没有新块。消息频率直接计接收次数，没有给生产节点新增统计。

## 5. 实测结果：重点是3D ESDF

### 5.1 默认官方三维主体与上一轮对照

| 指标 | 上一轮1280×720 | 本轮640×480 |
| --- | ---: | ---: |
| 深度输入接收 | 29.36Hz | 29.83Hz |
| RGB输入接收 | 29.41Hz | 29.42Hz |
| 实际深度融合 | 18.51Hz | **18.43Hz** |
| 实际颜色融合 | 18.60Hz | **18.11Hz** |
| 3D ESDF更新调用 | 24.61Hz | **24.61Hz** |
| Mesh接收 | 4.74Hz | **4.75Hz** |
| 深度融合区间主机timer平均 | 0.465ms | 0.322ms |
| 3D ESDF区间主机timer平均 | 0.670ms | 0.554ms |
| 官方ESDF数据年龄滚动均值范围 | 46–52ms | **47–54ms** |

减少像素后，主机timer显示单次图像处理开销下降；短测中没有观察到有效融合率、ESDF频率或数据年龄明显改善。上述timer不是单独CUDA完成耗时；两轮非同一bag，视场和机器状态也不同，不作严格因果性能承诺。

官方三维组件正常加载，Mesh和地图服务运行成功。官方后端仍无项目esdf_points/metrics，3D ESDF由原生服务验证存在；没有用二维切片频率替代三维计算频率。

### 5.2 三维诊断点云

| 指标 | 本轮640×480 |
| --- | ---: |
| 诊断后端3D ESDF更新调用 | 24.78Hz |
| 原生三维点云发布 | **4.84Hz**（目标5Hz） |
| 运动阶段每帧点数 | 14,747–18,152 |
| 停稳后总点数 | **18,152** |
| outside | **10,891** |
| inside | **7,261** |
| site | 3,319（上述集合的状态子集） |

24字节字段、observed/inside/site、原生体素中心、距离符号及有限值检查通过。点云包含真实已观测的内外侧样本，不再只显示障碍物内部。规则点间距来自原生体素，不是额外固定ROI查询网格。

上一轮停稳为14,791点，本轮更多，不与“像素减少”矛盾：ESDF点数取决于被观测体素、视场、轨迹、体素尺寸和融合历史，并不等于输入像素数。本轮纵向视场扩大，已观测空间不同，不能把点数增加当作分辨率或精度提高的证明。

本轮未测30Hz诊断显示；核心ESDF目标30Hz和诊断显示目标5Hz是两件事。

### 5.3 原生三维场的有界查询

在频率窗口结束后，对与上一轮相同的base_link AABB查询一次：最小角(-1,-1,-0.2)m、尺寸(2,2,1.4)m，不额外更新或显示。官方3D返回68×68×48，共221,952值：

- known：17,349，unknown：204,603；
- known正距离：10,781，负距离：3,485，零距离：3,083。

这证明三维场中存在外侧有效距离；AABB含大量视场外/遮挡空间，known比例不是全场景重建完整率。查询AABB与全已观测点云的空间范围不同，数量无需一致。距离有±2m上限，饱和值不等于无限精确的真实距离；零距离site也可能属于inside。

本轮官方有界查询客户端往返约50.8ms，诊断约59.2ms，计时在收到响应后立即结束，不含随后数组落盘。上一轮查询计时包含转换和保存，口径不同，不作直接延迟比较。

## 6. 为什么输入变小但频率仍约24–25Hz

原生shouldProcess使用严格 `elapsed > 1/frequency`。ESDF由10ms tick检查，目标30Hz对应33.33ms，一般要到约40ms才满足更新条件，因此仍接近25Hz。减小图像不会自动改变这个调度规则。

深度融合按图像时间戳使用同频限流。本轮相邻输入时间戳间隔中位约33ms，约57.5%的间隔不超过33.33ms。对接收时间戳只模拟此门限得到约18.49Hz准入率，与实际18.43Hz接近，继续支持“同频严格限流过滤了部分输入帧”的解释。该模拟不覆盖TF、队列和传输，不能当作独立融合实测。

下一步若要提高有效3D实时性，应基于官方主体检查ESDF调度相位、图像同频限流容差与最新帧策略，再测TF/队列等待。继续减小图像或改变点样式不会自动消除这个上限。本轮未修改这些核心逻辑。

## 7. 重建质量与剩余问题

官方3D最终Mesh：2,496顶点、3,554三角形。与上一轮相同方法，使用保存的world←base_link变换，计算配置墙体邻域Mesh顶点到最近理想盒表面距离：

| 墙邻域诊断 | 1280×720 | 640×480 |
| --- | ---: | ---: |
| 邻域顶点数 | 370 | 499 |
| 中位绝对距离 | 2.08mm | 2.21mm |
| 均值 | 7.85mm | 8.55mm |
| P95 | 35.29mm | **37.60mm** |
| ≤20mm比例 | 84.6% | 82.6% |

这个局部顶点指标未表现出精度提升；它不是全场景/均匀面积误差，采样与视场不同，不能仅据此把差异全部归因于640×480。两墙参考表面3cm邻近比例约40.4%/37.0%，也受可见范围扩大和未观测背面影响，不等于unknown覆盖率。

机器人自身仍进入静态重建，表面边缘仍有离散和模糊，遮挡区域仍unknown；没有新增自过滤或解决运动残留。本轮没有测实机、长时大地图或残影消退时长。

官方ESDF数据年龄各窗口均值的中位约51ms，不是逐帧P95或屏幕端到端延迟。诊断消息在接收瞬间按最近/clock估算年龄，中位约66ms、P95约114ms，也不是屏幕延迟。

## 8. 真实3D窗口

官方三维主体的彩色Mesh（不是ESDF点云）：

![640×480官方三维主体Mesh](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/official3d_stopped.png)

同一核心生成的原生三维ESDF诊断点云：

![640×480三维ESDF真实点云窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/diagnostic3d_stopped.png)

截图来自本轮RViz窗口后台像素，没有用AI或绘图替代实际效果。视角和显示开关有交互变化，不作逐像素对照；原始RGB与深度另存。

## 9. 当前运行指令

先在旧启动终端Ctrl+C停掉旧场景与mapper。旧Gazebo如果还运行1280×720，必须重启传感器，单独重启mapper不会改变输入尺寸。

默认官方3D主体：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./run.sh gazebo_gui:=true camera_width:=640 camera_height:=480 esdf_mode:=3d
```

如果需要直接看三维ESDF点，停止上一轮mapper后改用诊断后端：

```bash
./run.sh gazebo_gui:=true camera_width:=640 camera_height:=480 \
  mapper_backend:=diagnostic esdf_mode:=3d esdf_view_mode:=observed esdf_view_hz:=5
```

只有确认同域本项目640×480场景已在运行，才加start_scene:=false，避免重复启动。两条命令不能同时启动两套mapper。

新终端核对：

```bash
source ./env.sh
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /camera/color/camera_info --once
ros2 topic hz /camera/depth/image_rect_raw --wall-time
# 仅诊断后端有三维点云话题：
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

默认官方后端的3D更新查控制台ros/update_esdf及原生save_rates；不要查询官方二维话题作为3D运行判断。需要恢复上一轮输入尺寸，重新启动场景时传camera_width:=1280 camera_height:=720。

## 10. 证据与验证范围

- [完整结果及实际双相机标定](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/results.json)、[频率/时间门限分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/analysis.json)、[质量指标](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/quality_metrics.json)。
- [实际参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/official3d_runtime_parameters.yaml)、[官方统计快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/official3d_wrist_motion_timings.txt)、[运行日志](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/measurement.log)。
- [本轮采集脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/run_measurement.py)、[Python/既有C++检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/offline_tests.log)、[构建](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/input640_3desdf_test_20261104/build.log)。

两个包构建成功，20项Python及既有C++检查通过；本轮三维GPU导出运行格式检查通过，没有因分辨率变化重复执行未改动的独立GPU单元测试。仿真结束后，本轮Gazebo、mapper、RViz均已停止。
