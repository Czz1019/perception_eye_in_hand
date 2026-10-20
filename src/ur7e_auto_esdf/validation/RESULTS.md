# 自动 ESDF 离线验证

2026-10-13，ROS Humble＋whole_control＋真实 nvBlox/CUDA。**没有连接真实 RTDE，也没有发送真机运动指令。** 测试控制/接收构造函数被禁止，控制接口与反馈为模拟，深度来自合成 z=0.15 m 平面。未声明真实场景精度或完整避障能力通过。

| 检查 | 结果 |
|---|---|
| 构建 | 新包 colcon 构建成功；[build.log](build.log) |
| 单元/共享控制 | 28 项通过，含 lease、过期反馈、运动状态、视图时间边界、漂移、冻结语义及原 RTDE 保护；[unit.log](unit.log) |
| 协调器故障 | 12 类通过，停止、不续跑、采集漂移/断流/异常状态、停止应答失败、重新取得控制权时位置变化；[结果](faults/results.json) |
| 完整扫描 | 所有 8 点，每点 8 帧，共 64 帧；[结果](integration/results.json) |
| 地图来源 | 19,983 个观测自由体素，显式先验 0，未知单元没有正距离 |
| 原生 ESDF 数值 | 合成平面上方的指定已知区域距离误差 P95=0.025 m（当前体素 0.025 m），不是实体测量误差 |
| 规划格式兼容 | 用真实 `ur7e_real_static.field.FrozenMap.load` 读取并校验成功 |
| S08 主机停顿 | 采集首帧后持有 GIL 约 81.10 ms；控制已断开、没有新增伺服命令，仍采满 8 帧 |
| 取消 | 第二轮在进入段取消，停止且不自动续跑 |
| 安装后的 launch | 只回看模式无 RTDE 节点；正常退出；[结果](launch_results.json) |
| 晚订阅 | 冻结加载后新订阅收到规划自由点 19,868、原生表面点 13,333 |
| 私有 TF | 使用合成关节反馈驱动实际 robot_state_publisher，深度相机 TF 与校准 FK 最大矩阵差约 2.74e-8 |

完整模拟测试约 130 秒，不能作为真机耗时承诺。控制按原生产扫描速度/加速度参数运行；没有为通过测试放大 40 ms 运动周期保护。

地图摘要中的 `current_arm_observed_safe=false` 与 `planning_ready=false` 是保留的真实计算结果（在合成测试地图/现有球模型下），不是扫描失败；它说明冻结成功不能代替整臂通道覆盖与碰撞模型验收。本包只完成扫描建图。

## 复现

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_auto_esdf.sh
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
AUTO_PY=/home/wqj/miniconda3/envs/whole_control/bin/python
ROS_DOMAIN_ID=85 $AUTO_PY -B -m pytest -q src/ur7e_auto_esdf/test/test_mapper.py src/ur7e_auto_reconstruction/test/test_control.py
ROS_DOMAIN_ID=86 $AUTO_PY -B src/ur7e_auto_esdf/test/run_faults.py
ROS_DOMAIN_ID=84 $AUTO_PY -B src/ur7e_auto_esdf/test/run_integration.py
ROS_DOMAIN_ID=87 $AUTO_PY -B src/ur7e_auto_esdf/test/run_launch_smoke.py
```

后两个测试按顺序执行，launch 回看测试使用完整扫描生成的地图。ROS_DOMAIN_ID 必须隔离于真实设备图。复测会更新本包 validation 下测试记录，不覆盖真机扫描输出。
