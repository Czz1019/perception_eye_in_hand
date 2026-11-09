# perception_eye_in_hand 感知模块已完成修改详细说明

> **sparse-v2 当前状态：** 已修复测试工具的 Thrust 链接错误及构建过期检查，生产目标和可选工具均已编译、链接、安装成功；**未执行工具测试，未启动新版感知或仿真**。下一步按 [构建修复与仿真操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 启动应用，再查看 Topic。算法/接口说明见 [sparse-v2 修改说明](/home/czz/perception_eye_in_hand/docs/ESDF_SPARSE_V2_MODIFICATION.md)。旧性能和截图不能作为 v2 验收结果。

> 2026-10-27 统一入口更新：推荐使用本项目 `run_ur7e_perception.sh` 联合启动场景、输入、感知与显示，详见 [项目内统一感知结构与启动说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_INTEGRATED_DESIGN_AND_STARTUP.md)。下文分组件方式保留用于调试；不要与统一入口同时启动。

核对日期：2026-10-27。修改对象是 **`/home/czz/perception_eye_in_hand`**。本文明确回答“改的是哪个仓库、改了什么、是否已经完成、怎样运行到修改后的实现”。

## 1. 是否已经完成

**已完成本仓库感知路径的实时性改造、原生 C++ 感知模块接入和同条件仿真验证，并已生成操作文档与结果证据。** 修改已写入本仓库文件，原生模块已构建安装到本仓库的 `install_native`。这不是只对 workspaces 进行参数调整，也不是只把体素改成 0.05 m。

当前有两条可运行的感知路径：

| 路径 | 本仓库实现 | 当前状态 |
| --- | --- | --- |
| 原有 Python/Torch 感知路径 | `src/ur7e_nvblox` | 已修改调度、导出、采样、统计和版本管理，保留作兼容/对照 |
| 新增原生 C++ 感知路径 | `src/ur7e_nvblox_native` | 已接入官方底层模块，完成块级导出与应用调度；本轮推荐的实时感知入口 |

**没有将所有旧入口自动替换为 C++。** `run_ur7e_esdf.sh` 仍启动 Python；`run_ur7e_native.sh` 才启动新 C++。原有 SAGE、auto 扫描、真机应用没有全部迁移为新网格接口，物理机器人实时性尚未验收。因此，可以确认“仓库内独立感知模块改造与仿真验证已完成”，不能扩大为“全项目规划控制闭环和全部实机应用都已迁移完成”。

## 2. workspaces 与本仓库各自承担什么

| 位置 | 角色 | 本次结果 |
| --- | --- | --- |
| `/home/czz/perception_eye_in_hand/src/ur7e_nvblox` | 原有应用侧感知代码 | 在这里修改 Python 实现、增加公共输入和监控工具 |
| `/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native` | 新应用侧 C++ 感知模块 | 在这里实现节点、调度、导出、配置和几何验证 |
| `/home/czz/perception_eye_in_hand/build_native`、`install_native` | 本仓库构建/安装产物 | 编译并运行本仓库的 native_mapper |
| `/home/czz/perception_eye_in_hand/docs`、`artifacts/native_cpp_20261027` | 文档和本轮实验 | 保存修改说明、操作命令、图像、频率、延迟和原始数据 |
| `/home/czz/workspaces/isaac_ros-dev` | 用户指定的官方 ROS 参考及底层依赖 | 读取其实现并链接已有 nvblox_ros/NITROS/GXF 库；不是本次应用改造交付目录 |

新模块直接继承官方 `nvblox::NvbloxNode`，相当于使用已有库实现本项目自己的感知节点。**本项目的感知实现和启动入口在 perception_eye_in_hand，底层第三方库目前仍由 workspaces 安装目录提供。** 尚未把完整 NVIDIA 依赖复制进本仓库；换机器部署需要准备匹配的官方依赖或设置 `NVBLOX_ROS_WS`。这是外部依赖关系，不代表应用修改发生在外部仓库。

统一入口更新前的该次复核中，本仓库关键源码/配置/脚本的 23 个哈希均与仿真归档一致，所链接官方库的哈希也与归档一致。该检查说明当前交付与实测对应，不把它解释为对外部整个工作区所有文件的审计。

## 3. 原有 Python 感知代码已经改了什么

以下均位于 `/home/czz/perception_eye_in_hand`，不是 workspaces。

| 本仓库文件 | 修改前的问题 | 已完成修改 | 作用与边界 |
| --- | --- | --- | --- |
| [node.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py:161) | 融合路径中紧接查询/读回，查询负担直接影响输入处理 | 分出融合、导出和发布线程，latest-only 交接 | 减少不必要的路径耦合；GPU 地图访问仍受锁保护，不宣称无锁并行 |
| [node.py 导出循环](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py:267) | 无消费者或已有相同地图时仍可能做重复工作 | 检查订阅需求、epoch、map_version、帧龄后导出 | 无点云消费者时持续建图但停止导出；避免旧版本重复处理 |
| [sampling.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/sampling.py:7) | 固定查询网格与地图 voxel_size 不关联 | 增加原生体素中心采样及更新预算/截止时间工具 | native 模式的采样间距随地图精度变化；dense 模式保留兼容 |
| [esdf.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/esdf.py:35) | 显示时将大量未知/远离表面的位置读回 CPU | GPU 过滤未知与距离带，只读回显示需要的点 | 减少读回和序列化；该 Python 模式仍使用 Torch 任意点查询 |
| [node.py 发布/统计](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py:329) | 消息 Hz 难以区分新地图与重复发布 | 按输出话题去重版本，保留源图像时间戳，分开统计融合/更新/导出/发布 | 不使用当前时间覆盖 header 来掩盖旧图延迟 |
| [depth_adapter.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/depth_adapter.py) | 两个后端可能使用不同分辨率、单位或内参缩放 | 公共深度转换、匹配内参和原始时间戳 | 为本项目两种感知后端提供相同输入契约 |
| [monitor.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/monitor.py) | 只看 topic hz 无法判断真实建图速度 | 输出融合、ESDF、导出、接收、唯一源图时间戳和版本 | C++ queued 与 Python depth_received 的计数口径明确区分 |
| [setup.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/setup.py) | 新工具没有安装入口 | 增加 esdf_rates、depth_adapter | 可以作为本项目 ROS/Python 工具运行 |

旧 Python 默认配置在早期阶段曾改为 5 cm；它**不是本轮 3 cm 同精度性能改善的证据**。本轮 Python 对照明确使用新包的 `python_comparison.yaml`，地图体素 0.03 m、公共 640×360 输入、depth_stride=1。新 C++ 的默认地图体素同样为 0.03 m。

## 4. 本仓库新增的 C++ 感知实现

### 4.1 实际文件与职责

| 本仓库文件 | 已完成的功能 |
| --- | --- |
| [native_mapper.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_mapper.cpp:21) | 本项目 NativeMapper 节点，继承官方 ROS 库；输入检查、调度、地图版本、pose、按需发布、统计与 reset |
| [block_export.cu](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.cu) | 调用 NVIDIA 原生层读取模板，在 GPU 上生成有符号距离并压缩有效显示点；替代应用侧 Torch 点查询 |
| [block_export.h](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.h) | 原生导出结果、20 字节 xyz/rgb/distance 点结构、可复用 GPU 缓存 |
| [native_geometry_test.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_geometry_test.cpp) | CUDA 平面、位姿、未知值、清空缓存及官方转换器逐元素对照 |
| [gazebo.yaml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo.yaml) | 本项目 3 cm 三维 ESDF、图像时刻 TF、工作范围、更新目标、队列和距离带配置 |
| [python_comparison.yaml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/python_comparison.yaml) | 本项目 Python 后端的同条件配置 |
| [CMakeLists.txt](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/CMakeLists.txt)、[package.xml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/package.xml) | 在本仓库构建应用节点和 CUDA 几何测试，链接外部官方依赖 |
| [native.rviz](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/rviz/native.rviz) | 本项目 observed ESDF、相机 pose、base_link 实时显示配置 |

### 4.2 感知数据流

```text
本项目 Gazebo / 物理深度相机
    │ 深度图 + 对应 CameraInfo
    ▼
本项目 depth_adapter：单位/有效值处理 + 图像/内参同步缩放
    │ /esdf_input/depth，/esdf_input/camera_info
    ▼
本项目 NativeMapper + 图像时刻机械臂 TF
    │ 调用官方 C++ TSDF / 3D ESDF 模块
    ▼
本项目 BlockExporter：原生体素中心 ROI → GPU 距离带压缩
    ├── /ur7e_nvblox/esdf_observed → RViz/观察器
    ├── /ur7e_nvblox/esdf_3d → 有需求时输出完整 ROI
    └── /ur7e_nvblox/stats、camera_pose → 真实更新和位姿核验
```

机械臂姿态来自图像时刻 TF，沿用项目现有机器人运动学与手眼外参。正常感知不发送关节命令，也不要求自动扫描。验收脚本的模拟小幅运动是单独测试步骤。

### 4.3 什么修改解决什么问题

1. **查询/导出路径**：由应用层 Torch 任意点查询改为官方原生层读取模板 + 本项目 CUDA 压缩，减少张量处理、CPU 读回与序列化。不是在 Python 上把 publish_hz 数值调大。
2. **未知灰点**：默认只显示已观测且 `abs(distance) <= 0.15 m` 的体素。完整网格仍保留未知，供明确需要它的消费者使用。不能删除建图所需的自由空间积分或把未知当成自由。
3. **精度与点数关联**：导出原生体素中心，精度由地图 voxel_size 决定；相同范围内体素更细，候选数量随之变化。候选数量还受 ROI、显示数量受观测和距离带影响。
4. **低频仿真时钟**：该场景 `/clock` 约 9.99 Hz；本项目适配器用 steady clock 调度工作，图像与 TF 继续使用真实 ROS stamp，避免被 ROS now 的周期判断限制在约 10 Hz。
5. **图像时间量化节流**：提高内部深度接收上限，避免约 33 ms 图像间隔与 30 Hz 门槛比较时误丢帧。配置中的 1000 是内部上限，不是声称相机更新 1000 Hz。
6. **重复旧图**：新输入驱动 ESDF 版本递增，每个话题仅发布尚未发布的版本；停止消费者时不再强制导出。
7. **数据积压**：短输入队列、过期/倒序帧检查及有限更新预算，优先处理当前可用图像；不靠补发旧地图制造高 Hz。
8. **缺乏真实指标**：实际积分/更新/导出计数、CUDA 完成时间、源图 stamp 和地图版本同时记录。共同观察器测量输入到点云接收延迟。

本次 exporter 仍读取导出 ROI 内的原生规则网格，**不是完全零成本，也不是只访问已占用体素的全稀疏算法**。地图读写仍在官方 processing callback group 内串行；本轮没有承诺任意 ROI/精度下恒定实时性。

## 5. 本仓库启动和验证入口

| 文件 | 用途 | 是否启动新 C++ 感知 |
| --- | --- | --- |
| [build_ur7e_native.sh](/home/czz/perception_eye_in_hand/build_ur7e_native.sh) | 本仓库隔离编译，输出 build_native/install_native | 构建入口 |
| [run_ur7e_common_depth.sh](/home/czz/perception_eye_in_hand/run_ur7e_common_depth.sh) | 公共标定深度输入 | 输入组件 |
| [run_ur7e_native.sh](/home/czz/perception_eye_in_hand/run_ur7e_native.sh) | 加载依赖和本仓库配置，运行本仓库 native_mapper | **是** |
| [run_ur7e_native_rviz.sh](/home/czz/perception_eye_in_hand/run_ur7e_native_rviz.sh) | 显示本仓库实时 ESDF | 可视化组件 |
| [run_ur7e_esdf.sh](/home/czz/perception_eye_in_hand/run_ur7e_esdf.sh) | 原有 Python 路径，已优化、保留作兼容/对照 | **否** |
| [run_gazebo_validation.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/test/run_gazebo_validation.py) | 外部观察器、静止/运动、延迟、TF、版本和生命周期验收 | 验证已启动的指定后端 |

完整多终端命令、环境、手动采集、监控、地图服务、两后端比较和实机准备见 [操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)。不要继续执行旧 Python 入口，却期待自动进入新 C++ 路径。

## 6. 已执行的同条件仿真验证

两后端使用 **3 cm 体素、640×360 公共深度输入、中心 [0,0,0.5] m / 大小 [2,2,1.5] m 导出范围、±0.15 m 显示带**。均为独立 mapper 进程、同一外部观察器，顺序运行。性能窗口不启动 RViz/完整网格消费者。

| 后端 | 阶段 | 实际点云接收 Hz | 输入到点云 P50 / P95 / P99，ms |
| --- | --- | --- | --- |
| 已优化 Python | 静止约 60 s | 17.71 | 13.55 / 17.75 / 22.09 |
| 本仓库 C++ | 静止约 60 s | 18.62 | 2.43 / 4.18 / 5.62 |
| 已优化 Python | 腕部运动约 60 s | 17.15 | 15.61 / 19.29 / 24.22 |
| 本仓库 C++ | 腕部运动约 60 s | 17.79 | 2.71 / 4.61 / 5.21 |

中位延迟降低约 5.6～5.8 倍。两组静止快照都是 9,130 个点，中心及距离逐点一致，未以降低精度或输入分辨率换取该结果。性能改善主要体现在延迟；当前上游输入约 17～19 Hz，不能宣称真实新地图已经稳定达到 30 Hz。

顺序实时流并非逐帧相同的 rosbag 回放，且两个 nvblox 核心为不同构建，因此不能把 Hz 的小幅差异或全部加速都归因于 C++ 语言本身。延迟起点是观察器收到公共深度图，终点是同 stamp 点云接收，不包含相机曝光和公共预处理之前的开销。

其他已通过的验证：

- CUDA 几何测试与官方转换器逐元素误差为 0，覆盖平面、原生中心、未知和清空。
- 已有 Python CUDA/采样/版本回归共 13 项通过。
- C++ 仿真 2,351 次图像时刻 TF 核验，最大矩阵误差约 3.74×10⁻⁸。
- C++ 无点云订阅者时 ESDF 更新 15 次，导出 0 次；重新订阅/清图后恢复，epoch=2。
- 三维距离服务、地图保存和完整 ROI 点云已实际调用验证。
- 默认 RViz 点云配置实际显示成功；本机官方 Mesh 插件缺 GLSL150 资源，未作为默认可用显示。另一次 GUI 发生无日志段错误，长期 GUI 稳定性仍需观察。

证据：[原生结果](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/native_003.json)、[Python 对照](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/python_003.json)、[源码哈希](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/source_manifest.json)、[服务验证](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/service_validation.json)。本次核对读取已归档实验与源码，不把它写成又执行了一轮仿真。

![本仓库同精度感知结果](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/esdf_comparison.png)

[真实 RViz 窗口](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/rviz_live.png)、[实际点云采样动图](/home/czz/perception_eye_in_hand/artifacts/native_cpp_20261027/native_live.gif) 均已保存。动图为 2 FPS 采样预览，不能用播放帧率推导地图 Hz。

## 7. 尚未完成的范围，避免误以为全项目均已替换

| 内容 | 当前状态 |
| --- | --- |
| 本仓库原有 Python 感知优化 | 已完成，并保留 |
| 本仓库新 C++ 感知路径、脚本、配置和构建 | 已完成 |
| 同精度仿真、效果图、更新/延迟统计和操作文档 | 已完成 |
| 所有旧 launch/根脚本默认切到 C++ | 未实施，入口明确区分 |
| SAGE/其他消费者直接适配新完整网格 | 未全部实施；尺寸、顺序、未知值契约需要逐个迁移 |
| auto/真机应用全部切换 | 未实施，不在本轮独立感知验收内 |
| 实机相机、动态障碍、自体过滤和安全闭环验收 | 尚未完成 |
| NVIDIA 依赖完全内置、脱离 workspaces 独立部署 | 尚未完成，当前链接已有匹配依赖 |
| 真实场景稳定 30 Hz 新地图 | 尚未证实，受当前输入交付率等约束 |

完整网格点云的未知值为 1e6，继承距离服务默认 -1000，旧 Python 完整网格当前为 100；服务名称含 gradient，但当前安装实现输出三维距离数组，没有附加梯度通道。下游迁移时必须处理这些差异，不能仅重映射话题名就认定规划兼容。

## 8. 对应文档

- 本文：已完成修改的范围、文件和验收，回答“改的是不是 perception_eye_in_hand”。
- [原生 C++ 接入报告](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_INTEGRATION_REPORT.md)：官方复用细节、具体参数、完整实测和接口限制。
- [完整操作说明](/home/czz/perception_eye_in_hand/docs/NATIVE_CPP_USER_GUIDE.md)：从构建到启动、手动采集、监控、测试和保存的具体命令。
- [原生包目录与模块说明](/home/czz/perception_eye_in_hand/docs/modules/ur7e_nvblox_native.md)：逐文件职责、函数流程和 ROS 接口。
