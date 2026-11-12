# 0.005m现场仿真扫描：Mesh、3D ESDF精度与实时性测试

日期：2026-11-11。功能包：`ur7e_nvblox_in_hand`。本次使用当前运行的Gazebo实验台、640×480相机和机械臂TF，**没有使用rosbag回放，没有修改生产算法或运行参数，也没有连接实机**。

## 1. 结论与测试边界

本次已完成静止15秒→高位左右各4cm扫描35秒→返回中心后静止15秒，并保存三组真实Mesh、TSDF/Color/ESDF地图和实际发布ESDF点云。扫描段频率统计包含接近姿态和返回中心，总墙钟窗口约42秒。

这是对用户正在运行的累积地图的续测，**没有清图，也没有重启地图**；起始地图已有观测历史。因此三段不是“单帧、60帧、空地图多视角”的对照，不能根据覆盖变化小判断扫描无效。只有一次运行，不能代表多轮稳定上限；GUI、用户原有topic hz订阅和本次监测都保留。

最终Mesh可见区域总体P95约3.21mm，物体区域P95约11.18mm；ESDF已观测真值自由空间样本P95约6.77mm，但仍有约15.31% ROI未知空间和严重的少量距离高估。实时性没有达到30Hz，扫描中更新约10.69Hz、接收约10.47Hz。

## 2. 配置和数据来源

配置加载：安装目录base.yaml→realtime.yaml→launch显式参数；本次以节点实际参数转储为准，不以源文件或历史测试配置推断。

| 配置 | 实际值 |
|---|---|
| voxel_size | 0.005m |
| truncation_vox | 4，物理截断距离0.02m |
| 输入 | RGB/Depth 640×480，Gazebo配置30Hz |
| depth_hz / color_hz | 40/40Hz准入上限，不代表相机40Hz |
| esdf_hz / esdf_view_hz | 33.4/33.4Hz上限 |
| mesh_hz / layer_hz | 5/5Hz；地图层仅有订阅时导出 |
| ESDF | 3d、observed、stride=2 |
| 显示间距 | 原生5mm中心每隔2个索引采样，约1cm；未重算成1cm地图 |
| 深度范围 | 0.05–2.05m |
| 工作区 | bounding_box，x[-0.1,1.1]、y[-0.6,0.6]、z[-0.05,1.05]m |
| TSDF | max_weight=5；inverse_square_tsdf_distance_penalty |
| ESDF | min_weight=0.1、max_site=2体素、max_distance=2m、unknown策略kIgnore |
| 衰减/清图 | 均关闭 |
| 输入队列 | 每路4帧，最大输入年龄0.5s |
| 位姿 | 机械臂TF，在每张图像自己的时间戳查询 |
| ROS域/Gazebo | 164 / http://127.0.0.1:11665 |

完整生效参数：[params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/params.yaml)。配置字典：[config.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/config.json)。仿真真值来自实验台`lab_scene.yaml`及`lab_scene.py`生成的同一组箱体、圆柱、球体与架子几何，参考几何不含机器人。

## 3. 实时性：实际得到多少Hz

| 阶段 | 深度接收 | 深度融合 | 颜色融合 | ESDF更新 | ESDF发布 | ESDF实际接收 | Mesh发布 | 点数 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 静止前 | 31.37 | 11.22 | 11.36 | 11.15 | 11.02 | 10.47 | 4.60 | 637,674 |
| 左右扫描 | 29.92 | 10.71 | 10.54 | 10.69 | 10.64 | 10.47 | 4.46 | 637,691 |
| 静止后 | 29.89 | 10.56 | 10.56 | 9.99 | 9.56 | 9.26 | 4.42 | 637,691 |

接收/融合/更新/发布来自原有metrics计数差除以墙钟时间，属于host调用完成口径，不是GPU事件测量。ESDF实际接收是监测者收到的PointCloud2频率，不是RViz FPS。静止前短窗口深度接收31.37Hz与30Hz配置略有差异，可受回调批量处理和窗口端点影响，不能据此说传感器配置成31.37Hz。

| 阶段 | CameraInfo接收→ESDF接收代理P50 | 代理P95 | ESDF头时间戳相对当前/clock的年龄P95 | 仿真秒/墙钟秒 |
|---|---:|---:|---:|---:|
| 静止前 | 259.86ms | 304.87ms | 293.20ms | 1.0012 |
| 左右扫描 | 249.88ms | 305.90ms | 287.05ms | 0.9986 |
| 静止后 | 274.48ms | 319.79ms | 307.30ms | 0.9987 |

CameraInfo与点云按输入时间戳关联。这是本机接收代理，受两个订阅者调度和消息丢失影响；不是曝光到屏幕延迟。/clock年龄也是仿真时间指标，不可与墙钟延迟混算。相机输入代理监测进程还订阅了图像和大地图，本身有测量开销。

![分阶段实时性](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/realtime_stages.png)

## 4. 分阶段定位：时间花在哪里

以下是扫描结束时原有metrics里最近200次调用的滚动耗时，不是每一帧的严格串行全链路分解，也不是本段独立GPU耗时。

| 阶段/原有指标 | Host平均 | Host P95 | 解释 |
|---|---:|---:|---|
| Depth submit / `depth_submit_ms` | 12.78ms | 65.52ms | 深度预处理、上传及融合接口返回；部分等待可在此体现 |
| Color submit / `color_submit_ms` | 4.29ms | 6.51ms | RGB上传与颜色融合接口返回 |
| ESDF update / `esdf_submit_ms` | 10.79ms | 17.62ms | ESDF更新接口返回；不能等同纯CUDA核时长 |
| ESDF export + wait / `esdf_export_with_gpu_wait_ms` | 34.90ms | 82.48ms | 全层筛选、跨流同步、GPU→CPU导出 |
| ESDF pack + publish / `esdf_points_pack_publish_ms` | 41.29ms | 53.31ms | 构建PointCloud2与publish调用，含分配、拷贝、序列化等 |
| Mesh update + read / `mesh_readback_ms` | 59.84ms | 72.15ms | 更新Mesh并读取完整顶点/三角形/颜色 |
| Mesh pack + publish / `mesh_pack_publish_ms` | 93.91ms | 110.76ms | 完整MarkerArray的构建、序列化、发布调用 |

**输入阶段：** 扫描中深度接收29.92Hz，融合10.71Hz；深度队列覆盖约18.89帧/秒，门限丢弃仅0.32帧/秒，过期丢弃0。颜色队列覆盖约18.87帧/秒。主要可见损失是有限队列覆盖，当前40Hz门限不是这一轮掉到约11Hz的主要解释；覆盖计数不能直接当作GPU算力不足的证明。

**调度阶段：** `node.py::work()`由同一工作线程依次处理输入、ESDF更新、ESDF导出和Mesh读取。Mesh即使只设5Hz，单次较长的更新/回读仍会阻塞下一批输入。发布在另一个线程，但CPU、Python运行时、内存分配与DDS通信依旧竞争资源。

**导出阶段：** `native/export.cu::run()`先`cudaDeviceSynchronize()`，每次获取全部已分配块，按512块一批筛选，分批读回计数与结果并同步；stride减少输出数量，但不等于只访问更新过的块。`backend.py::esdf()`随后还有numpy拷贝。因此底层增量ESDF计算不等于ROS输出也是增量。

**输出阶段：** 实际约637,691点×24字节≈15.30MB/条。33.4Hz对应约511MB/s的单路未计协议原始载荷；扫描发布10.64Hz也约163MB/s。这是载荷估算，不是实测网络带宽。Mesh同样每次读取完整地图并通过MarkerArray发送，未采用官方ROS分块增量Mesh消息路径。GUI、额外topic hz和多个订阅者会进一步增加开销。

**为何还达不到30Hz：** 扫描结束ESDF导出平均34.90ms、点云打包发布平均41.29ms已经分别接近或超过33.33ms预算；Mesh平均约59.84ms回读及93.91ms打包发布还会穿插执行。两线程可以重叠，不能把表中全部时间相加当作一帧延迟，但同一工作线程的长任务与发布线程吞吐均构成约束。单改ESDF定时器或体素显示大小不能解决这些成本。

资源快照为RTX4060 Laptop 8GB，具体见[gpu_snapshot.csv](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/gpu_snapshot.csv)与[process_snapshot.txt](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/process_snapshot.txt)。存在Gazebo、RViz、用户topic hz与本次监测，未停止用户任务；本轮不是无GUI性能上限测试，也没有进行干预对照来量化每个后台进程的因果贡献。

## 5. Mesh精度与效果

Mesh通过保存PLY的真实三角形做射线求交，与同一阶段实际深度相机位姿下的仿真几何比较。统计的是**相机轴向距离误差**，不是三维Hausdorff距离。ROI为桌面物体可见区域，排除无效深度及明显机器人前景；遮挡边缘仍可能残留影响。

| 阶段 | Mesh MAE | Mesh P95 | 去台面物体P95 | 可见参考覆盖率 | 顶点 / 三角面 |
|---|---:|---:|---:|---:|---:|
| 静止前 | 1.75mm | 3.21mm | 11.20mm | 99.92% | 210,413 / 337,913 |
| 左右扫描 | 1.62mm | 3.07mm | 10.25mm | 99.96% | 210,114 / 337,537 |
| 静止后 | 1.75mm | 3.21mm | 11.18mm | 99.92% | 210,566 / 338,177 |

最终原始深度轴向P95约3.23mm，Mesh约3.21mm；大面积台面占比较高，不能把总体P95当作每个瓶盖和细杆的精度。物体区域P95仍约11.18mm，Mesh最大误差约253.53mm，而原始深度最大误差约256.16mm；这些尾部可能含边缘或前景参考不完整，不能据最大值直接认定整个表面错位25cm，也不能隐藏。

5mm网格能保留比厘米网格更多轮廓，但12mm尺度的瓶盖只有约2.4个体素宽、细杆和夹缝会受离散化、TSDF截断带、取色可见性影响。截图中颜色下移、侧面混色或局部阶梯仍可能存在；本轮测的是几何精度，未做逐像素颜色真值评价，不能声称纹理已修复。

![真实保存Mesh渲染](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/mesh_live_comparison.png)

上排是本轮真实RGB；下排由各段保存PLY渲染，使用该段真实深度相机视角。RGB与深度光学中心不同，不能直接用两幅图的像素差当颜色误差。扫描列是在扫描结束返回中心后保存的累积地图，不是扫描中途某一帧。

保存PLY和地图是顺序服务，期间建图继续，因此两者并非原子同帧快照。TSDF零面与Mesh一致性检查只能作近似诊断；扫描结束附近ROI顶点TSDF残差P95约0.53mm，最后静止后约0.00043mm，不应把这一区别当成运动导致的Mesh算法错误。

## 6. 3D ESDF精度、覆盖与异常

ESDF直接读取保存BLOX中的原生EsdfVoxel，核对block_size=0.04m、每块8³体素、原生记录20字节。测试不重新生成距离场，不填unknown，也不靠固定查询点创建地图。

评估ROI为x[0.16,1.0]、y[-0.5,0.5]、z[0,0.4]m；取5mm原生体素中心的stride2子集，共336,000点，采样间距1cm。**不是ROI内所有5mm体素的穷举结果**。只在真值自由空间且原生observed的位置计算距离误差；unknown另计。参考为场景几何联合体外部的最近距离，机器人不在参考中；其附近需谨慎解释。

| 阶段 | ROI unknown | ESDF MAE | P95 | 最大误差 | >10cm误差样本 | +2m饱和样本 |
|---|---:|---:|---:|---:|---:|---:|
| 静止前 | 15.32% | 3.01mm | 7.03mm | 1907.47mm | 79 | 16 |
| 左右扫描 | 15.31% | 3.08mm | 6.69mm | 1907.47mm | 81 | 18 |
| 静止后 | 15.31% | 3.01mm | 6.77mm | 1907.47mm | 81 | 18 |

最终地图全层原生observed约5,060,380体素，发布显示637,691点；这两个数不是同一口径，显示经过stride2筛选。发布点云中observed已筛选为真，unknown没有作为“空点”发布；ESDF自由空间有距离值的体素本来就是有效距离场，并不都是障碍物表面。

最终ROI未知51,457个：27,520个没有TSDF块，23,937个有块但TSDF权重为0；本次抽样没有发现“已有达到0.1权重的TSDF却仍为ESDF未知”的情况。低门限不能补出没有看到的表面。小幅左右扫描且起始地图已有历史，只额外增加少量观测；要补墙角、试管后方及底部，需要扩大有效入射视角，单纯静止重复融合无法消除真实遮挡。

![3D ESDF实际点云](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/esdf_3d.png)

该3D图使用真实发布点云，仅显示ROI内绝对距离≤8cm的点，再每5点取1点绘图。它用于看近表面空间结构；**饱和点被带宽条件排除，不能用这张图证明没有异常**。完整未裁剪点云与原始CDR均保留。

![ESDF截面与几何误差](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/esdf_sections.png)

截面高度0.1125m；第一排原生ESDF，第二排真值外部距离，第三排自由空间误差。空白可能是unknown或不属于自由空间误差的内部区域，应结合observed字段判断。误差色标限制±30mm只为便于观察，所有大误差仍计入表格。

**需要优先处理的距离高估：** 最后抽样有18个observed自由体素输出+2m，parent_direction为[0,0,0]且不是site。已核实最大误差点(0.8725,0.4325,0.1925)m，真值距离约0.09253m，原生却为2m；通过observed体素可以用20步、0.10m路径到达site。实际发布点也携带2m，因此不是RViz色彩或显示大小造成的。其他5个检查点在限定20步搜索内未找到site，这不证明它们与site完全不连通。

该证据指向原生ESDF传播/更新状态需要核查，不能将全部异常简单归因于unknown；但还没有通过固定输入全量重算与增量更新对照定位到具体CUDA缺陷，也不能排除保存时更新状态等因素。异常虽少，距离高估会把靠近障碍物的位置误判为较远，不宜只看P95来认可用于碰撞安全。

原始异常核对：[native_outlier_checks.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/native_outlier_checks.json)。

## 7. 下一步优化优先级

1. **正确性优先：** 用本次保存BLOX复现饱和点，对照一次全量ESDF重算与原增量状态，核查site初始化、parent传播、块间传播与unknown策略。对未建立可靠距离来源的点保守处理，不能直接把饱和上限视为已证实的净空。
2. **导出/发布：** 保持5mm地图计算，按规划或查看ROI导出，使用持久GPU/CPU缓冲与合适流同步，避免每次全图分批往返及多次拷贝。显示降频与规划查询需求分开设定。stride仅改变显示采样，不改善原生地图精度。
3. **Mesh路径：** 参考官方分块增量Mesh发布和RViz缓存，避免完整Mesh反复CPU回读、MarkerArray重发；可先降低Mesh查看频率，观察输入队列覆盖与ESDF频率是否改善，再量化收益。
4. **输入/调度：** 使用阶段耗时和覆盖计数做有控制的A/B，区分接入、GPU、拷贝、序列化；保留图像时刻精确TF查询，不能为了频率改用latest TF。增加队列可能增加延迟，不能自动提高计算吞吐。
5. **重建质量：** 分离纯深度几何、颜色投影和Mesh取色验证；对薄结构按物理截断尺度/权重做单变量试验。补充更多有效视角，并区分静态场景与机械臂自身运动残影；本轮未量化动态残影清除能力。

这些是后续方案，本轮未实施算法修改，也没有为了图像好看丢弃异常数据。

## 8. 保存位置、查看及复现

每个阶段目录包含：

| 文件 | 内容 |
|---|---|
| mesh.ply | 真实彩色三角网格，可用CloudCompare打开 |
| map.blox | 原生TSDF/Color/ESDF等层 |
| esdf.cdr | 实际发布PointCloud2的完整ROS序列化消息 |
| esdf_points.npy | 实际显示点的xyz、距离、observed、inside、site |
| esdf_points.ply | xyz和distance标量，CloudCompare中切换distance标量场 |
| accuracy_fields.npz | ROI查询中心、原生距离、observed及参考距离 |
| rgb.png、depth_m.npy、capture.json | 本轮输入快照和精确图像时间戳TF |
| mesh_render.png | 从保存三角网格进行射线渲染的图像 |

最终Mesh：[static_after/mesh.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/static_after/mesh.ply)；最终ESDF点云：[static_after/esdf_points.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/static_after/esdf_points.ply)；完整地图：[static_after/map.blox](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/static_after/map.blox)。

当前终端查看（不重启正在运行的场景）：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=164 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11665
source ./env.sh
python3 tools/watch_realtime.py
```

需要再次扫描，另开相同环境终端：

```bash
python3 tools/view_sweep.py --domain-id 164 --seconds 35 --amplitude 0.04
```

CloudCompare用File→Open打开上面的PLY；Mesh选择RGB，ESDF选择distance标量场，距离单位m。也可用系统已安装的CloudCompare命令，具体可执行名以本机安装为准。

P50为绝对误差或延迟的中位数，P95为95%分位数；例如ESDF误差P95=6.77mm表示该统计范围内95%的绝对误差不超过此值，并不是最大误差。文件完整性、原生格点和所有本地链接检查见[validation.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/validation.json)。

本次自动采集脚本[collect.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/collect.py)固定使用164域，拒绝覆盖已有结果，且会真实控制该Gazebo机械臂扫描；如需复测应复制整个测试目录中的脚本到新输出目录或调整输出目录，不能直接重复覆盖。离线分析[analyze.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/analyze.py)只读保存地图；结果[result.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/result.json)及[quality_summary.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/quality_summary.json)可复核。

第一次采集在保存初始地图后，测试采集器查询图像TF时遇到8ms未来外推；已保留失败记录并仅修改采集器为等待同一图像时间戳TF到达后重试。生产Mapper已有同类等待机制，本轮未修改它，也未用latest TF替代。失败尝试未进入最终统计。

当前Gazebo、Mapper和RViz保持运行，机械臂已正常返回中心。只结束了本次采集和离线分析进程。

实际RViz窗口记录：

![扫描后RViz](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/rviz_after_scan.png)
