# ur7e_real_reconstruction

真实 UR7e + D435i 眼在手上彩色表面重建，用于人工比较现实场景与感知结果。

- [2026-09-28 真实重建测试报告（35 个观察姿态与最终网格）](report/2026-09-28_real_reconstruction_report.md)
- [自动扫描观察点方案（仅离线选点，尚不可执行）](scan_design/OBSERVATION_PLAN.md)

- 融合深度到 TSDF，融合校正畸变后的 RGB，输出彩色三角网格和顶点点云。
- 使用图像时间的 `base_link ← depth/color optical` TF，无 SLAM、无 ESDF 更新。
- 复用 `ur7e_reconstruction` 的 SurfaceMapper、`ur7e_real_static` 的标定模型与自身过滤；不启动其规划节点。
- 没有目标订阅、轨迹 Action 客户端、速度或关节命令发布接口。独立话题 `/ur7e_real_reconstruction/*` 不进入规划输入。
- 默认 IDLE，显式开始；移动时跳过融合，停稳后继续；冻结时提取完整最终网格并保存。
- 每个新视角保存原始 RGB、深度字节/米制数组、CameraInfo、双相机时间戳、TF 和关节角。

```bash
./build_ur7e_real_reconstruction.sh
./run_ur7e_real_reconstruction.sh
```

默认配置针对本机现有真实标定文件，应用在 `whole_control` 运行。驱动与相机须单独启动。

详见 [真实测试说明](REAL_TEST_GUIDE.md)、[实现与计划](ToDoList.md)、[测试结果](validation/RESULTS.md)。

本任务只能辅助判断 ESDF 的场景位置、尺度和几何是否明显异常。重建与 ESDF 共享相机/TF，因此共同的标定误差可能同时出现；彩色网格不能独立证明距离值、未知空间处理或规划净空正确。与上一轮对比时须确认环境未变化，并考虑 ESDF 近表面带 ±5 cm 与 TSDF 零面不同。
