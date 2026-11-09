# workspaces示例主体重构后的仿真复测

日期：2026-11-04。对象：`/home/czz/perception_eye_in_hand/nvblox_in_hand` 重构版。结论：官方组件、三维地图服务、二维切片与可选三维诊断均能运行；默认inside-only输出问题已纠正。但尚未稳定达到30Hz，也没有证明重建几何质量优于旧3cm版本。

## 1. 测试方式与条件

采用独立ROS_DOMAIN_ID=129、Gazebo端口11459，原UR7e+D435与高墙障碍物场景。先到此前观察姿态，再保持静态10秒、wrist_1约±0.06rad往复15秒并回到观察姿态、最后停稳8秒。运动阶段包含进入/返回姿态等待，实际统计窗口约22秒，不是单独15秒纯运动。没有环绕，也没有操作实机。

三轮都保持：0.03m体素、原始1280×720输入、3m融合范围、深度/颜色/ESDF目标30Hz、Mesh/层公共发布目标5Hz，realtime档关闭衰减和范围清图。每轮新建地图，实际图像/噪声序列并非同一个bag，因此是短时功能和性能验证，非严格算法因果基准。

| 轮次 | 感知主体 | 输出与订阅负载 |
| --- | --- | --- |
| official3d | 官方组件nvblox::NvbloxNode，3d | Mesh、原始RGB、机器人/路径；测量端订阅Mesh，无项目附加点云 |
| official2d | 同一官方组件，2d | 上述订阅及官方二维ESDF切片；切片高度0.25m，投影高度0到0.8m |
| diagnostic3d | 项目派生诊断节点，3d | 官方核心加observed原生体素导出，显示目标5Hz |

截图是实际RViz后台窗口像素，不是生成图。窗口存在交互视角差异，不用像素外观推断误差。RViz的约31fps只是绘制频率。

## 2. 功能检查结果

- 官方轮次实际组件列表为 `/nvblox_container` 内1个 `/nvblox_node`，没有并行第二个mapper。
- 官方后端custom esdf_points和metrics发布者均为0，Mesh发布者为1，符合新的职责划分。
- 两轮官方RViz及诊断RViz均保持运行，没有启动后立即闪退。
- Joint/TF检查通过，没有重复状态源问题；光学TF与相同关节时刻FK残差约数值精度量级。它只证明内部一致性，不是手眼外部真值验收。
- 三轮Mesh、原生地图保存成功；两轮3d原生ESDF服务返回成功。
- 20项Python检查、既有C++策略检查和GPU筛选/中心/符号/字段检查通过；测试后恢复默认BUILD_TESTING=OFF并构建两包成功。

## 3. 实际频率

### 3.1 有新深度/颜色融合与ESDF调用频率

官方后端没有项目metrics，因此采用已有save_timings中 `ros/depth/integrate`、`ros/color/integrate`、`ros/esdf` 的累计样本数，在静态末到运动末两次快照之间差分，并除墙钟间隔。未往生产节点新增统计项。

端点用阶段结束时间近似服务执行时间，包含阶段之间约0.1秒的保存间隙，所以这是近似区间率；另保存save_rates的官方滚动率供核对。样本数对应成功进入融合或ESDF更新函数，不代表每次ESDF都产生新块/新信息，更不是GPU完成帧率。

| 指标 | 官方3d | 官方2d | 诊断3d |
| --- | ---: | ---: | ---: |
| 深度实际融合 | 18.51Hz | 18.02Hz | 18.23Hz |
| 颜色实际融合 | 18.60Hz | 18.39Hz | 18.27Hz |
| ESDF更新调用 | **24.61Hz** | **24.47Hz** | **24.55Hz** |
| Mesh消息接收 | 4.74Hz | 4.75Hz | 4.74Hz |
| 二维ESDF点云消息 | 不输出 | **24.36Hz** | 不输出 |
| 项目三维诊断消息 | 不输出 | 不输出 | **4.84Hz**（目标5Hz） |
| RGB输入接收 | 29.41Hz | 29.69Hz | 29.78Hz |
| 深度输入接收 | 29.36Hz | 29.46Hz | 29.05Hz |
| 仿真RTF | 0.999 | 0.999 | 0.999 |

阶段末save_rates的ESDF滚动率分别为25.0、24.5、25.2Hz，与区间计数差分接近，但采样窗口不同，不能把两者当完全相同的统计量。

本轮诊断显示按新版默认5Hz测试，**没有测试其30Hz显示设置**，不能把4.84Hz解释为30Hz目标下只达到4.84Hz。

![真实数据汇总图](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/test_summary.png)

### 3.2 为什么仍没有30Hz：本次得到的证据

原生 `shouldProcess()` 采用严格 `elapsed > 1/frequency`。ESDF由10ms tick检查，30Hz对应33.33ms，通常需等到约40ms才满足条件，因而约25Hz。实际tick滚动率约101Hz与此现象一致。这是代码与实测支持的主要原因，不是已经逐项排除了所有调度影响。

深度融合也按图像时间戳使用同一个限频判断。运动阶段实测输入时间戳间隔中位数约33ms，约55%的相邻间隔不超过33.33ms。离线只模拟这一时间门限（不模拟TF、队列、转换）得到约18.3Hz的准入率，与实际18.0–18.5Hz融合接近，支持“30Hz输入在严格同频限流下丢失部分帧”的判断。该模拟不是新实测融合计数，采样丢包和相位仍会带来差异。

因此下一步值得优先检查相位累加调度/同频图像限流容差及最新帧策略，而不只是进一步缩小点尺寸或把所有参数继续写成30。应以本地官方实现为基线修改和验证，本轮没有因此改动官方核心。

## 4. ESDF是否仍只有内部点

### 4.1 官方三维原生服务

在频率测量结束后，只做一次有界查询：base_link坐标中AABB最小角(-1,-1,-0.2)m、尺寸(2,2,1.4)m，不要求额外更新或可视化。

official3d返回68×68×48，共221,952个标量：

| 返回内容 | 数量 |
| --- | ---: |
| 有效已观测值 | 14,188 |
| unknown（本地默认-1000） | 207,764 |
| 有效正距离 | 8,811 |
| 有效负距离 | 2,879 |
| 有效零距离 | 2,498 |

这证明官方场保留了外侧正距离，并非只计算内部点。AABB包含大量视场外/遮挡区域，这个known/unknown比例只是该查询盒的结果，不能作为整个场景重建完整率。零距离site可能位于inside侧，不能仅以非负距离推断自由。

一次查询的客户端计时约53.6ms，诊断轮约59.2ms，**包含服务往返、转换和落盘**，不是纯RPC或ESDF核函数延迟；请求不计入上面的频率窗口。

### 4.2 诊断三维原生点云

停稳快照共 **14,791点**：

- outside：**8,893点**；
- inside：**5,898点**；
- site：2,676点，是上述集合中的状态子集，不能与前两项相加。

字段24字节布局、observed/inside/site、有限值、原生体素中心及距离符号检查通过。该集合由真实分配ESDF层遍历得到，不是先铺固定空间查询点再填数。规则间距来自体素中心；unknown没有输出。

距离范围达到±2m上限，饱和值不能解释为无限精确的真实距离。点云与有界服务数量不同，主要因为服务只返回指定AABB，点云选择全部已观测体素。

### 4.3 官方二维切片

运动阶段每帧985–1,161点，停稳后1,161点，约24.36Hz。它在固定切片高度展示二维距离分布，与14,791个三维体素不是同一空间维度，点数不能直接比较性能或完整性。

## 5. 延迟与计算耗时

官方 `ros/esdf_integration` 日志的滚动平均数据年龄：

| 后端 | 滚动均值范围 | 各窗口均值的中位数 |
| --- | ---: | ---: |
| 官方3d | 46–52ms | 48ms |
| 官方2d | 46–51ms | 48ms |
| 诊断3d | 46–53ms | 49ms |

这是ESDF更新时最新已融合深度的时间年龄，不是每帧分布的P95，也不含RViz绘制/屏幕扫描。

从官方累计timer时间差分得到，运动区间 `ros/esdf` 主机调用平均约0.67ms、0.95ms、0.68ms。调用可能没有新块，且未用CUDA事件独立测GPU完成时间，所以不能把它当作完整有效地图更新的GPU耗时。

诊断点云按最近/clock与header估计的年龄，中位约63ms、P95约105ms。它是异步采样诊断值，受5Hz显示节奏影响，不能与上述官方滚动平均或屏幕延迟混用。

## 6. 重建质量：没有因重构自动变好

三轮最终Mesh分别约2,060/2,061/2,058顶点，2,929/2,932/2,929三角形。采用已保存world←base_link变换，对两面配置墙体附近的Mesh顶点到最近理想盒表面距离作诊断：

| 量 | 官方3d结果 |
| --- | ---: |
| 墙邻域顶点数 | 370 |
| 绝对表面距离中位数 | 2.08mm |
| 均值 | 7.85mm |
| P95 | **35.29mm** |
| 距离≤20mm比例 | 84.6% |

其他两轮非常接近。这与旧3cm版本的P95约35.29mm相同，**没有证明本次结构重构提高了几何精度**。

这个指标只评价配置墙体附近的顶点，受网格采样密度影响，包含边缘及邻近几何影响，不是均匀表面面积误差或全场景精度。两墙参考表面在3cm内存在Mesh的比例约37.7%和24.7%，其中包含相机看不到的背面；不能当作已观测区域完整率。

实际截图仍有机器人自身进入融合、墙体边缘不平整和观察覆盖有限。当前30mm体素、仿真深度噪声声明、图像采样时刻/手眼和缺少自过滤均需后续核实；静态TSDF历史保留策略没有改变。此次没有量化残影消退时间，也没有宣称已修复运动长尾。

## 7. 真实窗口截图

官方三维主体，显示原生彩色Mesh：

![官方3D实际Mesh窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/official3d_stopped.png)

官方二维ESDF距离切片：

![官方2D实际ESDF窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/official2d_stopped.png)

可选三维诊断，包含外侧自由距离样本和内侧体素：

![三维原生ESDF诊断窗口](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/diagnostic3d_stopped.png)

左上RGB直接来自当前仿真相机，其中可见机械臂本体；这不是用截图美化消除的质量问题。查看表面应看Mesh，查看距离应看ESDF，不能要求两种点分布相同。

## 8. 用户复现指令

默认官方三维主体：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./run.sh gazebo_gui:=true
```

只保留一套场景和mapper。若同环境场景已存在，使用start_scene:=false；不要直接重复启动Gazebo。

停止上一轮mapper后，官方二维查看：

```bash
./run.sh esdf_mode:=2d
```

可选三维诊断：

```bash
./run.sh mapper_backend:=diagnostic esdf_view_mode:=observed esdf_view_hz:=5
```

新终端同环境查看官方输出：

```bash
source ./env.sh
ros2 component list
ros2 topic hz /nvblox_node/mesh --wall-time
# 只有2d模式才有官方切片消息：
ros2 topic hz /nvblox_node/static_esdf_pointcloud --wall-time
# 只有诊断后端才有此话题：
ros2 topic hz /nvblox_node/esdf_points --wall-time
```

完整操作与保存服务见 [操作说明](OPERATIONS.md)。本轮自动采集脚本包含仿真运动和进程管理，不作为普通查看命令自动执行；应先阅读其独立域、端口与运动条件。

## 9. 原始证据与验证边界

- [完整运行结果](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/results.json)、[计数差分与时间门限分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/analysis.json)、[质量指标](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/quality_metrics.json)。
- [采集脚本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/run_measurement.py)、[原始样本](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/samples.json)、[测试过程](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/measurement.log)。
- [官方3d实际参数](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/official3d_runtime_parameters.yaml)、[官方更新记录](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/official3d_wrist_motion_rates.txt)、[原生timer快照](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/official3d_wrist_motion_timings.txt)。
- [Python/既有C++检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/offline_tests.log)、[GPU检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/gpu_test.log)、[机器人TF检查](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/robot_state_audit.log)、[最终构建](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/example_refactor_test_20261104/final_build.log)。

本轮只测试realtime档，没有测reference档、实机、大地图、30Hz诊断显示、完整360°覆盖或严格屏幕端到端延迟。测试启动的Gazebo、mapper、RViz均已停止，未保留后台测试场景。没有修改生产感知逻辑来美化本轮结果。
