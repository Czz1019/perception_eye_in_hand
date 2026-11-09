# nvblox_ws与ur7e_nvblox_in_hand源码对比

核对日期：2026-11-09。实际对比路径为 `/home/czz/nvblox_ws/src/nvblox` 和 `/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`。前者位于项目外的home目录，不在perception目录内。本文只读分析两套代码，不修改建图代码、不运行建图或性能测试。

## 1. 核心结论

两者沿用同类nvBlox C++/CUDA稀疏体素建图算法，并都提供/使用nvblox_torch接口。**主要差别在版本、应用输入、位姿来源、调度、参数与显示导出路径，不是“官方用C++，我们用Python重新算ESDF”。**

nvblox_ws是官方核心仓库的一份本地源码，包含离线示例、实时RealSense示例、C++数据集fuser、完整核心及测试，不能把整个仓库定义为“只支持离线”。本次重点对比其SUN3D/3DMatch离线重建和ESDF展示入口。

ur7e_nvblox_in_hand是针对UR7e/D435的ROS2在线应用：使用官方wheel核心，增加机械臂图像时刻TF、有限队列、更新/显示调度、ROS接口及本地原生导出桥接。它没有复制完整core源码重新维护一套算法。

## 2. 版本与来源不是同一项

| 项目 | nvblox_ws本地源码 | ur7e_nvblox_in_hand实际依赖 |
| --- | --- | --- |
| 官方上游 | nvidia-isaac/nvblox，remote已确认 | 同一官方仓库的wheel |
| CMake核心版本 | 0.0.9 | vendor对应wheel版本线0.0.10 |
| 当前源码Git HEAD | 3a97d1fd77f4ece1012af42329b0763893e9af57 | 应用代码在perception工程，核心由wheel提供 |
| _version.py内部版本 | 0.0.9.dev1+cuubuntu22 | 0.0.10.dev1+cu12ubuntu22 |
| _version.py源码SHA | 3f42b210df9ad7a2099f00fcf324049d97342cb0 | e4a2b00f45ecf989682d72a33a2c29defe39fd2d |
| 本地core库位置 | src/nvblox/build/nvblox/libnvblox_lib.so；源码包lib目录通过软链接引用 | ~/.local/lib/python3.10/site-packages/nvblox_torch/lib/nvblox/libnvblox_lib.so |
| 本地Python绑定 | 源码包lib下软链接到build/nvblox_torch/cpp/libpy_nvblox.so | wheel自带libpy_nvblox.so |

nvblox_ws的Git HEAD与自动生成_version.py的SHA不同。因此当前源码、版本文件和历史编译产物不能直接视为完全同步；本轮没有重新编译验证该目录里的二进制。HEAD存在本地提交，esdf示例还含保存 `my_sun3d_room.ply` 的代码，故本文分析的是这份实际本地快照，不声称它逐文件等同某个未经修改的官方release。

当前系统解释器默认find_spec实际指向用户site-packages的0.0.10 wheel。**仅在nvblox_ws目录执行一份脚本，不保证它加载该目录的0.0.9源码/库**；取决于PYTHONPATH、安装方式和入口。做对照时必须记录模块__file__、_version与实际.so路径，否则可能是“两份脚本都调用同一已安装wheel”。

已做文件对照：两套EsdfVoxel等体素结构文件一致，WeightingFunction模板一致；TSDF更新模板和ESDF参数头文件存在差异。例如新vendor头文件增加动态TSDF不一致检查分支，但其默认阈值为负，本工程也未显式启用。不能把新版存在该分支写成当前应用已经开启动态残影清除。

## 3. 目录和应用组织对比

| 层次 | nvblox_ws/src/nvblox | ur7e_nvblox_in_hand |
| --- | --- | --- |
| 核心 | nvblox/src、include等完整C++/CUDA源码 | wheel二进制；vendor保留桥接需要的头文件 |
| Python接口 | nvblox_torch源码、C++绑定、参数封装 | 安装官方wheel，backend.py调用 |
| 应用入口 | Python examples及C++ executables | ROS launch、node.py、运行脚本 |
| 数据集 | SUN3D/3DMatch及其他数据加载器 | ROS图像/CameraInfo/TF，支持bag入口 |
| 地图显示 | Open3D/Rerun、PLY等 | RViz标准PointCloud2/MarkerArray |
| 机器人 | 通用核心无本项目UR7e场景 | 本地UR7e/D435、标定、控制器、Gazebo |
| 测试 | 核心单元、数据集、Python API和性能测试 | GPU契约、ROS消息编码、仿真/话题/服务测试 |
| 其他能力 | feature、场景原语、其他传感器与动态MultiMapper等 | 聚焦D435/机械臂静态建图，未暴露所有核心能力 |

新工程做的是应用适配与功能裁剪，官方代码的通用性更强。离线文件读取示例本身不提供ROS运行时的所有接入与排错机制，新工程相应增加了这些内容。

## 4. 输入与pose的具体差异

### 4.1 离线SUN3D路径

`Sun3dDataset`按文件名排序，把同一frame前缀对应的color.png、depth.png、pose.txt读成一组。深度除1000变成米，内参来自camera-intrinsics.txt，DataLoader为batch_size=1、shuffle=false、num_workers=0。

pose支持4×4矩阵或本地扩展的7元素位置/四元数格式；加载器还应用SUN3D到nvBlox的世界坐标旋转。此处使用的是数据集提供的相机位姿，不在重建循环里重新估计SLAM。

### 4.2 UR7e在线路径

node.py订阅深度、RGB和各自CameraInfo，分别排队；按每张图像header.stamp查询 `T_global_camera`，通过机械臂TF与手眼外参得到pose。深度支持16UC1/32FC1，颜色支持rgb8/bgr8，范围外和非有限深度置无效。

两路没有严格应用层RGB-D配对；内参使用最近缓存CameraInfo的K。队列默认3帧，输入超过0.5秒丢弃，TF缺失先等到年龄上限，不以最新TF替代图像时刻TF。

| 对重建的影响 | 离线 | 在线 |
| --- | --- | --- |
| 同一帧组织 | 文件前缀组成深度/颜色/pose组合；不等于数据本身绝对无时间误差 | 两路消息独立，必须验证实际时间同步 |
| 处理速度不足 | 循环变慢，通常仍逐帧处理到终止条件 | 覆盖旧帧、过期丢弃，以限制延迟 |
| pose误差 | 取决于数据集提供的结果 | 取决于关节、时间戳、TF插值、运动学、手眼标定 |
| 坐标约定 | 存在SUN3D世界旋转 | 使用base_link及相机光学frame，不能照抄SUN3D旋转 |
| 输入来源 | 已保存数据，便于重复试验 | 真相机或Gazebo/bag，受传输、调度和时钟影响 |

即使GPU算法相同，在线pose/图像不一致仍会导致厚面、串色和残影；离线成图更完整也可能只是用了更长轨迹、更多有效帧或更好的位姿，不能仅凭视觉下结论。

## 5. 重建循环有什么不同

### 5.1 Python重建示例reconstruction/sun3d.py

```text
读取一组depth/rgb/pose/sensor
→ add_depth_frame
→ add_color_frame
→ 可选每20帧深度特征融合
→ 开启可视化时update_color_mesh/get_color_mesh/Open3D
→ 下一帧
→ 按需保存最终Mesh
```

此入口没有持续调用ESDF更新。visualize=true时Mesh每个处理循环生成/获取，循环有多快取决于读取、GPU和可视化，不是ROS意义的30Hz发布。

该本地脚本在创建Visualizer时调用 `RadioFeatureExtractor().embedding_dim()`，而其构造函数会加载RADIO模型。故这份实现即便不打开deep_feature_mapping，也可能因打开可视化而初始化模型；不能笼统说此分支完全没有模型开销。新工程没有调用该Extractor或特征融合路径。

### 5.2 Python离线ESDF示例esdf/esdf.py

```text
get_sun3d_scene_mapper：顺序融合选定帧，更新Mesh和ESDF
→ main再次调用update_esdf
→ 获取TSDF已分配块的外接AABB
→ 构造覆盖AABB的密集体素中心网格
→ 批量query_differentiable_layer(ESDF)
→ unknown值过滤
→ 沿Z轴前后播放已计算场的切片
```

主循环播放的是完成建图后的距离场切片，没有持续接收相机帧。每片显示里的poll_events和sleep属于窗口事件处理/播放节奏，不是每次重新计算ESDF，也不能拿切片播放速度当实时地图更新Hz。

### 5.3 C++数据集fuser不能与上述Python示例混为一谈

`nvblox/executables/src/fuser.cpp`使用DataLoader和MultiMapper，按帧编号的subsampling参数分别决定深度、颜色、Mesh和ESDF是否更新。因此**这套C++离线入口可以逐帧更新ESDF**，不能把Python例子“融合结束后更新”推广成整个仓库行为。

frame_period_ms默认33是数据时间语义/动态处理的参数，不表示这个磁盘循环必然按真实30Hz接收或发布图像；其吞吐与应用端到端延迟需要单独测量。

### 5.4 本工程node.py

```text
ROS回调入队
→ 图像时刻TF与融合门限
→ 单GPU所有者深度/颜色融合
→ 地图dirty且更新到期时update_esdf
→ ESDF新版本、有订阅者时导出原生体素
→ Mesh/地图层低频导出
→ 独立线程打包发布ROS消息
```

ESDF与显示不是每张RGB都无条件重算；默认新深度或地图维护使地图变脏。输出待发队列按话题保留最新快照，消息打包不在GPU所有者中逐点执行。保存/查询等服务仍会占用GPU所有者，可能影响融合。

## 6. 最重要的区别：ESDF计算与ESDF显示查询

**该离线官方示例确实使用密集AABB查询网格；但查询网格不是建图算法，也不会把unknown自动变成有效ESDF。**

| 项目 | 本地离线Python ESDF示例 | 本工程默认3D ESDF输出 |
| --- | --- | --- |
| 场从哪里来 | 官方Mapper由TSDF更新ESDF | 官方wheel Mapper由TSDF更新ESDF |
| 取点位置 | 根据地图块AABB建立torch.meshgrid，间距随voxel_size | 遍历真实已分配块与原生ESDF体素 |
| 如何获得距离 | 每个采样坐标调用ESDF查询 | 读取EsdfVoxel.squared_distance_vox并恢复米制/符号 |
| unknown | 用官方unknown sentinel构造valid_mask | observed位筛选，排除无效距离 |
| 显示范围 | 每次显示一个Z切片 | 默认所有observed3D体素，可选择band/inside |
| 点数 | 单片有效查询数，与AABB/v/切片高度有关 | 当前选择下原生有效体素数，与地图/v/门限有关 |
| 查询计算量 | 包围盒体素数量，包含未分配块之间的空隙 | 已分配块数×512的筛选与有效点读回 |
| 输出 | Open3D VoxelGrid，plasma，0～1m归一化 | ROS点云，RViz显示颜色；打包rgb尺度0.24m |

因此，两者都含规则体素间距，而且离线示例调用的是VoxelGrid，本身也可以显示方块。不能把“点/方块”外观作为“是否官方算法”的判断依据。

离线一次查询AABB对教学和切片展示很直接，但大而稀疏的包围盒可能产生许多无效查询；高频重复执行容易扩大负担。本工程避免了默认固定ROI密集查询，但当前导出仍扫描全部已分配块并显式GPU同步，并未实现只导出变更块的最优增量方案。密集已知地图中它也不保证一定比所有查询方式快。

本工程QueryEsdf服务可显式请求AABB采样，这是按需服务，受20万点限制，不是日常ESDF话题构建/发布路径。

## 7. 默认参数差异足以影响质量与覆盖

以下“离线”列针对本地Python SUN3D/ESDF示例的显式设置和其本地0.0.9头文件默认值；不是所有C++fuser和RealSense示例的统一默认。

| 参数 | 离线Python示例/源码默认 | 本工程realtime配置 | 影响 |
| --- | --- | --- | --- |
| voxel_size | 0.05m | 0.03m | 原生离散精度和点数不可直接比较 |
| 最大融合距离 | 示例显式5m | 3m | 可见范围、地图体积与负载 |
| TSDF截断 | 4体素，即默认0.20m | 4体素，即0.12m | 物理融合尺度不同 |
| TSDF/颜色权重模式 | kInverseSquareWeight | kInverseSquareTsdfDistancePenalty | 测量权重不同，需同条件验证 |
| 最大融合权重 | 5 | 5 | 此项相同，不等于只保留最近5帧 |
| ESDF有效TSDF权重门限 | 1e-4 | 0.1 | 本工程更严格，可能排除低置信观测 |
| ESDF site距离阈值 | 1体素 | 2体素 | site选择和距离离散表现可能不同 |
| ESDF最大距离 | 2m | 2m | 此项相同，不意味着空间全已知 |
| Mesh最小权重 | 1e-4 | 0.1 | Mesh低置信区域保留程度不同 |
| Mesh顶点焊接 | true | true | 此项相同 |
| 观测帧策略 | 顺序处理至帧数/结束条件 | 有限队列、年龄和频率限制 | 在线可能少融帧，换取时延控制 |
| 颜色/ESDF/Mesh节奏 | 取决于示例循环阶段 | 融合上限40、ESDF/显示33.4、Mesh5Hz | 上限不是实际输入或实测频率 |

提高有效权重门限可能降低噪点，也可能增加缺失；site阈值差异可能改变近表面体素参与方式。本文没有实测证明哪一组对当前场景更好，不能把这些差异一概称作已经验证的优化。

## 8. 各自的优势与不足

| 方面 | nvblox_ws离线入口 | ur7e_nvblox_in_hand |
| --- | --- | --- |
| 算法基准 | 输入固定、顺序融合、易重复、核心源码完整 | 可模拟实际输入，但需录bag才能稳定重复 |
| 系统接入 | 文件接口简单，缺少本项目ROS在线链 | 已接UR7e/D435、TF、话题、服务和参数 |
| 数据完整性 | 可慢速融完指定数据 | 负载高时主动覆盖/丢过期帧，可能损失覆盖 |
| 延迟控制 | 离线吞吐不等于实时延迟 | 队列/年龄限制，统计接入开销，但不保证所有负载下达标 |
| 可视化 | Open3D等直接显示，切片直观 | RViz方便机器人查看，DDS/打包/读回有额外成本 |
| 显示扩展性 | AABB查询对稀疏大地图成本高 | 原生块筛选更适合当前需求，但仍全图扫描/全Mesh读取 |
| 功能广度 | 核心、C++MultiMapper、其他传感器/特征等较完整 | 聚焦机械臂静态D435，动态/特征等未完整接入 |
| 迁移 | 源码编译需匹配工具链；本地softlink不能直接搬走 | wheel减少core构建，但固定SHA、Torch/ABI及桥接仍有约束 |

官方本仓库还带RealSense+cuvslam的实时Python示例，使用SDK采集、红外视觉里程计和Rerun；这条路径不同于SUN3D离线，也不同于本工程机械臂TF。其存在说明底层不局限离线，并不表示当前UR7e工程需要引入cuvslam。

## 9. 应怎样做公平的后续对比

1. 将一段实际UR7e相机的RGB、Depth、各自内参及准确图像时刻pose保存成固定输入；保留原坐标定义，不套SUN3D世界旋转。
2. 第一轮固定同一wheel/core、同一有效输入帧、分辨率、v、范围、权重和ESDF/Mesh门限，对比顺序循环与ROS链，隔离接入/调度开销。
3. 第二轮在相同输入上比较0.0.9/0.0.10核心，记录Python模块及实际动态库路径；不要混换桥接头文件或绕过SHA检查。
4. 第三轮比较AABB查询与原生导出，先关闭显示，再分别开启显示；对同坐标已知体素比较距离/符号，切片与全3D点数单独报告。
5. 分别统计融合成功率、覆盖/unknown、表面误差、Mesh质量、ESDF误差、调用吞吐、ROS发布/新输入关联Hz、输出年龄、GPU/CPU/显存。不能拿离线切片动画或RViz FPS代替ESDF新数据实时性。

这种分层对照能判断主要差异究竟来自输入/pose、参数、版本，还是导出与ROS开销；本轮仅给出分析与方案，没有实际执行这些测试或产生新的Hz结论。

## 10. 源码证据入口

nvblox_ws实际文件：

- [SUN3D数据加载与pose转换](/home/czz/nvblox_ws/src/nvblox/nvblox_torch/nvblox_torch/datasets/sun3d_dataset.py)
- [Python逐帧重建示例](/home/czz/nvblox_ws/src/nvblox/nvblox_torch/nvblox_torch/examples/reconstruction/sun3d.py)
- [离线建图辅助函数](/home/czz/nvblox_ws/src/nvblox/nvblox_torch/nvblox_torch/examples/utils/scenes.py)
- [ESDF AABB查询与切片播放](/home/czz/nvblox_ws/src/nvblox/nvblox_torch/nvblox_torch/examples/esdf/esdf.py)
- [C++ Fuser循环](/home/czz/nvblox_ws/src/nvblox/nvblox/executables/src/fuser.cpp)
- [RealSense/cuvslam实时示例](/home/czz/nvblox_ws/src/nvblox/nvblox_torch/nvblox_torch/examples/realsense/run_realsense_mapper.py)

本工程文件：

- [ROS输入、TF、调度与服务](../src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py)
- [参数传递与官方Mapper调用](../src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py)
- [原生体素筛选与读回](../native/export.cu)
- [配置](../src/ur7e_nvblox_in_hand/config/base.yaml)
- [完整模块与算法说明](PACKAGES_DEPENDENCIES_CORE_ALGORITHMS_20261109.md)
- [wheel来源和workspaces关系](NVBLOX_TORCH_ORIGIN_AND_CLEAN_ROS2_DEPLOYMENT_20261109.md)
