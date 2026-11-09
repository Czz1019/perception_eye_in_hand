# ur7e_nvblox_native 模块说明 — 官方结构对齐 / 0.4.0 / 接口 v4

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

模块保留原生块导出与 RGB 融合；主调度直接复用官方 tick，MeshView 独立封装显示，GpuTimings 使用 CUDA 事件统计。

## 目录

```text
/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/
├── CMakeLists.txt                 C++17/CUDA17、生产目标及可选几何工具
├── package.xml                    0.4.0，ament_cmake / 官方 nvblox_ros 依赖
├── README.md                      当前接口和未运行状态
├── THIRD_PARTY.md                 官方来源及定义/实际发布分支的区别
├── launch/perception.launch.py    项目统一场景、输入、感知、显示编排
├── config/
│   ├── gazebo.yaml               3 cm 三维地图、ROI、显示和路径参数
│   ├── gazebo_fast_esdf.yaml     同精度的 30 Hz ESDF 目标
│   ├── python_comparison.yaml    历史 Python 对照配置
│   └── cyclonedds_local.xml       本机多进程启动所需 DDS 索引预算
├── rviz/
│   └── native.rviz               RGB Mesh、颜色体素、ESDF 图例、绿色路径与 RGB 输入
└── src/
    ├── native_mapper.cpp         官方模块封装、调度、订阅、轨迹、Mesh桥接与统计
    ├── block_export.h            稀疏输出选择与缓存契约
    ├── block_export.cu           实际分配块遍历、有效体素筛选和 GPU 压缩
    ├── mesh_view.h / mesh_view.cpp 显示管理；快照不强制全图重建
    ├── gpu_timings.h              有界复用事件池，非阻塞测量
    ├── mesh_protocol_test.cpp     不启动 ROS/GPU 的协议检查
    ├── mesh_visualization.h      索引三角形转 Marker，坐标/删除语义
    └── native_geometry_test.cpp  更新后的回归源码，尚未执行
```

## 主要函数

| 位置 | 职责 |
| --- | --- |
| `NativeMapper::processDepthQueue` | 记录输入队列首见时间，复用官方处理 |
| `processDepthImage` | 校验 frame/时间/帧龄，调用官方融合；成功后发布图像时刻相机 pose 和路径 |
| `processColorImage` | 校验颜色 frame/尺寸/焦距/编码/时效，调用官方 RGB 融合并计数；不强行同步 CUDA |
| `publishTopicInfo` | 发布保留的 JSON 话题语义和颜色图例 |
| `processEsdf` | 新输入驱动官方三维 ESDF 更新，保存提交时间/版本及 CUDA 事件 |
| `tick` | 官方 NvbloxNode::tick 主调度，外加 Mesh 需求/事件轮询/按需导出 |
| `BlockExporter::run` | 枚举实际 ESDF 块、ROI 块裁剪，最多512块/批；不生成 ROI 点阵 |
| CUDA `gather` | 先排除未知/非法距离，再输出 occupied、band 或所有 observed 距离 |
| `publishCameraPose` | PoseStamped、有限容量 Path、运动门槛和限频；来源为机械臂 TF |
| `MeshView::updateDemand` | 有 Marker 消费者时创建官方 Mesh 内部订阅，无消费者时释放 |
| `MeshView::afterLayers` | 官方图层周期后检查消费者 GID，必要时快照 |
| `meshMarkersFromMessage` | 全局 Mesh 顶点按索引展开，块增量更新、DELETE/DELETEALL |
| `MeshView::snapshot` | 新消费者/重同步时，序列化官方已更新的 MeshLayer；不额外调用全图 updateMesh(kYes) |
| `resetMap` | 清原生地图、深度/颜色队列及限频时间、路径和 Mesh，递增 epoch，拒绝旧代 Mesh 回调 |
| `pollGpuTimings` | 非阻塞查询事件，区分提交/确认完成与实际发布 |
| `publishStats` | 接口版本、实际更新/导出、分配块/访问槽位/点数和滚动延迟 |

所有本节点的地图读写、Marker 桥接和快照均在官方 processing callback group 内串行，避免地图并发读写；并非无锁架构。Mesh 首次快照和显示会增加负载，需后续测量。

## 接口原则

- `esdf_occupied`：observed && is_inside 的不定长原生中心点云。
- `esdf_distance_band`：已观测近表面距离带；`esdf_distance`：可选的全部已知距离诊断，默认不创建。
- `mesh`：继承的官方增量 Mesh；`mesh_markers`：标准 RViz MarkerArray。
- `camera_path`：robot TF 路径，不是视觉 SLAM；删除重复 slam_path 别名。
- `get_esdf_and_gradient`：保留按需 AABB 距离查询服务，未知默认 -1000，不含附加梯度通道。
- `reset_map` / `reset_stats` / `republish_mesh`：清图、清滚动延迟、请求 Mesh 显示重同步。

旧版完整固定 ROI 点云、未知哨兵及固定数组下标假设不再适用于 esdf_3d。默认统一入口 `run_ur7e_perception.sh`，旧 Python 入口保留兼容；运行前使用构建脚本和 --check-build 验证安装状态。
