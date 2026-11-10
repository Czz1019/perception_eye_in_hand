# ur7e_nvblox_in_hand

[迁移部署：完整依赖、库版本、安装、编译和仿真指令](docs/MIGRATION_DEPLOYMENT_20261109.md)。

以`nvblox_torch`为主接口的独立UR7e/D435感知工作区，与`../nvblox_in_hand`并列，保留原工程不变。ROS输入与调度由Python实现，TSDF/Color/ESDF/Mesh由wheel中的nvblox C++/CUDA计算。小型本地Torch桥接补充原生ESDF/地图层导出、二维ESDF和半径清图；不链接原workspaces中的nvblox_ros，不启动NITROS、GXF、cuVSLAM或人物/任务模型。

**这不是纯Python重新实现距离场，也不意味着可以不安装ROS、CUDA和驱动。** 为保留原生ESDF标记，需要编译本工程自己的C++/CUDA桥接。迁移不需要复制原isaac_ros-dev工作区，但需要安装匹配的系统/Python依赖并重新构建；不要跨机器复制install目录代替构建。

- [UR7e＋D435i实机测试：历史IP/标定、驱动、相机、TF、ESDF和数据记录完整指令](docs/REAL_UR7E_D435I_TEST_GUIDE_20261109.md)
- [官方如何定义、构建与打包nvblox_torch](docs/OFFICIAL_NVBLOX_TORCH_BUILD_EXPLANATION_20261109.md)
- [与nvblox_ws离线示例的版本、输入、ESDF、显示和参数对比](docs/NVBLOX_WS_OFFLINE_COMPARISON_20261109.md)
- [nvblox_torch来源与版本、是否依赖workspaces、空白ROS2环境配置](docs/NVBLOX_TORCH_ORIGIN_AND_CLEAN_ROS2_DEPLOYMENT_20261109.md)
- [功能包、依赖、数据流、核心算法与迁移注意事项](docs/PACKAGES_DEPENDENCIES_CORE_ALGORITHMS_20261109.md)
- [完整编译、仿真、实机输入、话题与服务操作说明](docs/OPERATIONS.md)
- [代码结构、功能等价边界与计算差异](docs/ARCHITECTURE_AND_PARITY.md)
- [构建、原生GPU功能检查和实际仿真结果](docs/VALIDATION_20261109.md)
- [固定版本、资产和第三方来源](THIRD_PARTY.md)

## 快速启动

本机依赖已具备；新机器先阅读操作文档，按需运行`./install_dependencies.sh --system`。该安装脚本不会安装显卡驱动或CUDA Toolkit，也不会自动被build/run调用。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
./build.sh
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11565
./run.sh gazebo_gui:=true rviz_mode:=diagnostic
```

此命令不自动移动机械臂。推荐按操作文档“先场景→高位观察→Mapper→小幅左右扫描”的顺序测试，以免把接近过程的活动机械臂写入静态地图。

默认640×480、体素0.03m、最大融合距离3m、深度/颜色限频40Hz、ESDF/显示33.4Hz、Mesh/层5Hz；这些是门限，不是实测吞吐承诺。核心使用static TSDF，默认衰减和清图关闭。原有的自体污染、遮挡unknown和体素颜色平均问题不会仅因改用Python接口消失。

自动仿真检查创建新的artifact目录，不覆盖历史结果；只启动并关闭自己的子进程，沿已检查路径在仿真中移动机械臂：

```bash
source ./env.sh
DISPLAY=:1 /usr/bin/python3 tools/sim_test.py
```

所有监测终端都必须设置相同域并`source ./env.sh`，不要source其他工作区。测试工具固定域135/端口11565，防止误向其他机械臂或实机发指令。

## 1 cm / ESDF 显示 stride2 实测（2026-11-10）

当前默认 `voxel_size=0.01`、`esdf_view_stride=2`。stride 仅对3D显示点云作GPU空间抽样，完整ESDF和Mesh仍为1cm。编译、运行、精度与异常说明见 [测试报告](docs/LAB_1CM_ESDF_STRIDE2_TEST_20261110.md)。

[图像缺帧修复与 3D ESDF 复测](docs/INPUT_DELIVERY_FIX_AND_ESDF_RETEST_20261110.md)：配套仿真采用可靠图像传输，包含修复说明、实测频率、数据渲染与运行指令。

[相机位姿时间匹配与重建效果测试](docs/POSE_TIMESTAMP_RECONSTRUCTION_TEST_20261110.md)：同一仿真输入下的正常、100 ms 与 500 ms 人为偏差对照，包含实际 Mesh 和 ESDF 渲染。

[正常配置的仿真效果与配置参数说明](docs/NORMAL_SIMULATION_RESULTS_AND_PARAMETERS_20261110.md)：仅整理正常时间匹配的结果、实际配置文件及参数，不包含人为时间偏差配置。
