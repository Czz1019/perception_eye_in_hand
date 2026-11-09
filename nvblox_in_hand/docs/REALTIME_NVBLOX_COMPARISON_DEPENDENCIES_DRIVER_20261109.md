# nvblox_in_hand与realtime_nvblox：实现、依赖、实时性和驱动兼容说明

日期：2026-11-09。本轮读取本地两套源码、检查实际安装与动态库，使用隔离进程验证旧Python绑定初始化，并查阅NVIDIA官方兼容说明；未修改感知源码、驱动、CUDA或Python依赖，未重新进行性能对比仿真。

## 1. 两套代码的差别

比较对象是父工程src/realtime_nvblox，并非src/realtime_nvblox_2或src/ur7e_nvblox。

| 项目 | realtime_nvblox | nvblox_in_hand |
| --- | --- | --- |
| 主接口 | Python/rclpy、PyTorch、nvblox_torch封装GPU Mapper | C++ ROS NvbloxNode，链接workspaces已安装库 |
| 底层是否GPU | 是，不能称为纯CPU或纯Python算法 | 是，官方核心与项目导出器均含CUDA |
| 库来源 | wheel自带libnvblox_lib.so | workspaces/install/nvblox_ros/lib下的nvblox库 |
| ESDF输出 | 在给定空间构建规则查询网格，调用query_layer查询距离 | 从原生稀疏ESDF层导出observed体素中心，附带inside/site标记 |
| 点数决定因素 | 查询范围与grid_resolution_m；独立于地图voxel_size_m | 已观测地图、体素大小和导出选择策略 |
| unknown | full_grid输出保留unknown并画灰色；observed输出另行筛选 | 诊断observed导出不填充unknown点；不代表未知区域已被观测 |
| 实时组织 | LatestSlot、独立线程；Mapper深度/颜色/更新/查询共用RLock | 官方tick、图像队列、CUDA流、更新块跟踪，项目tick后导出 |
| Mesh路径 | 通用runtime可更新Mesh并整体转CPU/NumPy、ROS Marker发布 | 官方分块Mesh生成与增量发布、官方RViz插件 |
| 当前Gazebo专用入口 | gazebo_ros_node.py只做深度融合与ESDF，不订阅RGB、不周期发布Mesh | 当前测试同时执行深度、颜色、3D ESDF、Mesh |

特别注意：realtime_nvblox同时存在通用D435/cuVSLAM运行链和Gazebo专用运行链，不能将通用链的功能当作Gazebo入口已经执行的内容。Gazebo入口使用机械臂图像时间TF，不需要cuVSLAM或实机相机SDK。

旧Gazebo默认配置depth_stride=2：640×480输入实际降采样到320×240后融合；esdf_hz=10；体素3cm、查询网格5cm、范围2×2×1.5m。当前规则网格实现生成41×41×31=52,111个查询点，全网格显示点数由此确定。把地图体素改成2cm，不会自动把这张5cm查询点阵变密。

新工程最近测试使用640×480完整输入、融合门限40Hz、ESDF更新/显示33.4Hz，并同时融合RGB与发布Mesh。两者默认负载和调度不同，不能拿默认topic hz作同条件速度胜负结论。更换接口也不意味着替换了全部底层TSDF/ESDF算法；两条链都使用nvblox GPU代码，但库版本和参数控制面不同。

## 2. 新工程需要哪些依赖

### 当前构建与运行直接依赖

- NVIDIA CUDA兼容GPU及驱动；当前为RTX4060 Laptop、计算能力8.9。
- CUDA Toolkit：当前nvcc12.6，构建架构89；C++17/CUDA17、CMake、colcon、ament_cmake及编译器。
- ROS2 Humble：rclcpp、rclcpp_components、rclpy、launch、TF2、sensor/std/geometry/nav/visualization消息、rosbag2_cpp等。
- nvblox_ros、nvblox_msgs、nvblox_ros_common、nvblox_rviz_plugin及其core。
- 当前nvblox_ros间接需要Isaac ROS NITROS、图像/CameraInfo类型适配、GXF、isaac_ros_common及negotiated等库；删除人物识别不等于这些ROS传输基础设施也可删除。
- glog、gflags、Eigen/底层CUDA编译头文件及实际链接的CUDA运行时、NPP库等。
- Mesh可视化：RViz2与匹配的nvblox_rviz_plugin、图形环境。
- 当前仿真额外需要Gazebo Classic、gazebo_ros、gazebo_ros2_control、控制器、robot_state_publisher、xacro、UR/D435模型与本地RealSense Gazebo插件；扫描/检查工具还使用NumPy/SciPy/FCL等。

无需为了当前仿真安装PyTorch、nvblox_torch、cuVSLAM或pyrealsense2。实机D435才需RealSense ROS驱动及其SDK等；本项目可选实机launch与当前Gazebo是不同输入链。

当前env.sh显式依赖/home/czz/workspaces/isaac_ros-dev/install/setup.bash，默认build.sh仅构建新工程两个包，再链接该环境中的库。upstream源码副本并不会自动替代已安装依赖。迁移到另一台机器需要重建或安装匹配的官方ROS依赖，不能只复制nvblox_in_hand的源码或当前install目录就认定是完整独立发行版。

## 3. 关键库函数与实时性的关系

| 功能 | 新工程关键调用 | 作用 |
| --- | --- | --- |
| 深度接入 | NvbloxNode::processDepthImage → MultiMapper/Mapper::integrateDepth | 图像时间戳限频、TF、GPU图像转换、融合入口 |
| TSDF | ProjectiveTsdfIntegrator::integrateFrame | GPU投影、分配/更新稀疏体素块、距离权重融合 |
| 颜色 | NvbloxNode::processColorImage → Mapper::integrateColor → ProjectiveColorIntegrator::integrateFrame | 利用TSDF合成深度与可见性给颜色体素融合 |
| 3D ESDF | NvbloxNode::processEsdf → Mapper::updateEsdf → EsdfIntegrator::integrateBlocks | 从TSDF更新所需块的ESDF，不必用人为全空间点阵查询来构建ESDF |
| Mesh | Mapper::updateMesh → MeshIntegrator::integrateBlocksGPU / colorMesh | TSDF零交叉提取与顶点着色；订阅层发布时计算/发布 |
| 项目3D导出 | EsdfPointCloudExporter::run / compactEsdf CUDA kernel | 遍历已分配块，只压缩输出符合条件的observed体素，生成PointCloud2 |

实时性来自GPU并行、稀疏块结构、更新块跟踪、频率/任务调度及减少不必要查询输出，不是某一个函数能保证30Hz。旧Python封装的add_depth_frame/update_esdf同样调用GPU，不能仅用“C++比Python快”解释全部差异。

旧query_esdf_grid在Mapper锁内提交固定网格查询，并用.cpu().numpy()读回；因此查询和读回期间，其他线程不能同时进入该Mapper的融合操作。通用链还可能整体读回Mesh。预分配GPU查询缓存已经减少一部分分配开销，不能忽略旧版已有优化；没有新Profiler数据，不能断言当前大多数耗时一定在查询。

新项目导出也不是零成本：扫描全部已分配ESDF块，每批512块读回计数并同步，再读回点数据并同步。地图扩大、体素变细、多个订阅者或渲染负载增加后，仍可能成为瓶颈。当前输入来自普通ROS Gazebo图像，不能因依赖NITROS就声称端到端零拷贝。

此前18Hz主要已有代码与测试证据指向时间戳限频门限：30Hz门限要求间隔超过约33.333ms，而部分输入只有33ms，会交替拒绝帧。未改驱动，只将门限留出余量后融合接近30Hz，支持调度逻辑是那次问题的重要原因。当前点云约29Hz包含重复关联输入，约19～20Hz新时间戳推进不是驱动不支持30Hz的证明。

## 4. 本机实际版本与兼容检查

| 项目 | 本次检查结果 |
| --- | --- |
| 系统 | Ubuntu22.04.5 LTS / ROS2 Humble |
| GPU | RTX4060 Laptop，compute capability8.9 |
| NVIDIA驱动 | 580.178.04 |
| nvidia-smi展示CUDA版本 | 13.0：驱动支持能力，不等于应用用了CUDA13 |
| C++编译器 | nvcc12.6.20，/usr/local/cuda→cuda-12.6 |
| 新工程CUDA架构 | 89，与本机GPU匹配 |
| 新工程libcudart | /usr/local/cuda-12.6/targets/x86_64-linux/lib/libcudart.so.12 |
| 新工程nvblox_ros/nvblox_msgs | 3.2.5，来自workspaces/install |
| isaac_ros_nitros | 3.2.5，来自/opt/ros/humble |
| PyTorch | 2.9.1+cu128，torch.version.cuda=12.8，CUDA可用 |
| nvblox_torch | 0.0.10.dev1+cu12ubuntu22 |
| 旧wheel动态库 | ~/.local/lib/python3.10/site-packages/nvblox_torch/lib/nvblox/libnvblox_lib.so |
| 旧进程实际CUDA运行时来源 | ~/.local/lib/python3.10/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12 |

独立清洁进程中，按旧后端的预加载顺序成功导入nvblox_torch并创建0.03m空Mapper；实际加载的是wheel中的nvblox与PyTorch/CUDA库，没有加载workspaces版本。检查出现requests/urllib3依赖警告，但本次空Mapper初始化成功，不能把该Python网络依赖警告当成GPU故障。这个检查不等于旧系统全部融合/查询内核和性能都已重新验收。

新工程已在上轮完成实际编译、融合、ESDF、Mesh仿真测试；本轮ldd仍解析到预期workspaces/CUDA12.6路径。当前证据没有显示“驱动过旧、CUDA不可用”是实时性差或Mesh模糊的主因，也没有用切换驱动的A/B实验排除所有驱动性能影响。

NVIDIA说明：较新的驱动可运行较旧CUDA应用；CUDA12.x的基本兼容最低驱动系列为525，580以上仍通过向后兼容支持CUDA12应用。具体功能、PTX、第三方库还需满足各自要求。当前580驱动与12.6/12.8组合并非仅因版本数字不同就冲突。[CUDA兼容文档](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)

nvidia-smi的CUDA字段表示驱动支持的CUDA能力，而非证明安装了相同Toolkit；nvcc、应用实际动态库和torch.version.cuda是不同检查项。[nvidia-smi文档](https://docs.nvidia.com/deploy/nvidia-smi/index.html)

## 5. 迁移版本问题真正可能发生在哪里

1. **同名库混用**：两处都有libnvblox_lib.so，版本/API未必相同。LD_LIBRARY_PATH、PYTHONPATH和已source环境可能使Python绑定加载ROS库，或C++节点加载wheel库。旧代码的显式预加载、新env.sh重置环境正是在防范此问题；它与“系统装了两个CUDA版本”不是同一回事。
2. **头文件与二进制不匹配**：重新编译适配包时用了A版nvblox头文件，链接B版库；模板符号、结构体布局和API可能不一致。
3. **CUDA/Thrust/CUB与编译方式不一致**：此前native_geometry_test中带THRUST命名空间的undefined reference是链接阶段问题，不能由此认定显卡驱动有错；需核对nvcc/宿主编译器、CUDA/Thrust版本、模板实例化和依赖导出。仅凭当时一条链接错误无法定位唯一根因。
4. **PyTorch扩展ABI**：旧wheel与libtorch、C++ ABI、CUDA构建不匹配时可出现undefined symbol或导入失败。新C++运行链没有PyTorch这层依赖，但仍有ROS/GXF等版本约束。
5. **ROS/RViz接口与资源版本**：本机曾需Humble材质fallback；这属于图形资源适配，不代表GPU不支持计算，也不能解释所有TSDF颜色融合伪影。
6. **GPU架构不同**：新工程当前指定89，移到其他GPU需按实际架构重编，不能照搬4060二进制。
7. **代码版本本身不同**：本机workspaces源码git describe为bc18194-dirty，表示有本地改动；包版本3.2.5不代表是官方原样文件。迁移应同时记录commit、dirty补丁、参数、编译器和实际.so哈希，不能只记录包名。

因此目前不建议为解决模糊或29Hz盲目升降驱动。优先统一库来源、固定可复现环境，随后对齐两套输入分辨率、RGB/Mesh开关、体素、工作范围和输出语义，使用Nsight/CUDA事件与ROS统计分析计算、等待、读回、发布和渲染的成本。重建质量则继续检查自体过滤、颜色投影、截断带和图像时刻TF；这些都不能由驱动升级自动修复。

## 6. 本地源码依据

- [旧后端：Torch调用、锁与网格查询](/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox/nvblox_backend.py:37)
- [旧Gazebo入口](/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox/gazebo_ros_node.py:1)
- [旧Gazebo默认参数](/home/czz/perception_eye_in_hand/src/realtime_nvblox/config/gazebo_robot.yaml:1)
- [旧全网格/unknown发布](/home/czz/perception_eye_in_hand/src/realtime_nvblox/realtime_nvblox/ros_publishers.py:112)
- [新构建依赖](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/CMakeLists.txt:1)
- [新环境与依赖路径](/home/czz/perception_eye_in_hand/nvblox_in_hand/env.sh:1)
- [新派生节点调用官方处理](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:36)
- [新ESDF原生压缩导出](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu:1)
- [官方Mapper更新块/融合实现](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mapper/mapper.cpp:268)
- [最近实测报告](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/BUILD_MESH_ESDF_SIMULATION_TEST_20261109.md)
