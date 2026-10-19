# S08 静止采集超时修复（仅离线验证）

2026-09-29。**本次遵照用户要求，不进行真机测试；没有创建真实 RTDE 连接、发送运动命令、调用真实扫描服务或重启真机节点。** 原真机测试报告和失败结果原样保留。

## 问题判断

上次 S08 已停稳，旧实现却在整个 CAPTURING 阶段仍运行 500 Hz 的主机伺服线程，持续向相同目标发送 servoJ。一次主机控制循环间隔超过 40 ms 后，保护终止了只完成 1 帧的采集。该保护本身是有效的运动故障保护；问题在于静止采集仍依赖高频运动控制循环。

旧日志缺少触发时的完整耗时分解，无法确认那次停顿究竟来自 Linux 调度、Python 线程竞争、RTDE 调用，还是其他主机停顿。本修复没有将未经证实的原因写成结论，也没有把 40 ms 阈值调大。

## 修改后的执行顺序

1. 关闭采集门，核对当前实际关节状态，再连接独立 RTDEControl。
2. 以实际关节角作为本段起点，使用原五次轨迹与 500 Hz 目标频率运动；保留原超时、速度、跟踪、反馈及心跳保护。
3. 到点并连续停稳后，执行 `servoStop → stopScript → disconnect`，结束该段伺服线程；若停止失败，禁止打开采集门。
4. 再用只读反馈连续核验静止 0.7 s，随后采集至少 8 帧。此时没有 RTDEControl 连接，也没有 500 Hz 伺服线程。
5. 采集时持续检查反馈新鲜度、实际速度、与目标的偏差以及机器人运行/安全状态；零速度缩放在已停止脚本的采集阶段允许出现。发生故障或取消即终止，不进入下一段。
6. 关门并核对关闭状态后，下一段重新取得控制权，从新的实际关节反馈开始；S08 完成后直接冻结导出。

这保持了“走一步、停稳、采集”的使用方式，不使用 SAGE 或 ESDF，不改变原手动扫描功能。机器人静止采集不依赖主机每 2 ms 发送重复目标；运动期间的实时性要求仍存在。本方案增加了每点的脚本连接/退出及一次静止核验，额外时间和实际接续感受需经用户批准后真机验证。

## 可检查的代码

- [node.py](../../ur7e_auto_reconstruction/node.py)：`stationary_state`、`acquire_servo`、`release_servo` 和逐点状态机。
- [transport.py](../../ur7e_auto_reconstruction/transport.py)：停止命令返回 False 的处理；周期耗时、超时间隔和故障诊断。
- [run_integration.py](../../test/run_integration.py)：模拟控制接口 + 真实 CUDA 的全部 8 点流程，最后一点采集首帧后人为暂停主进程 GIL 80 ms。
- [run_faults.py](../../test/run_faults.py)：静止采集漂移、速度异常、反馈丢失、安全状态变化、取消、停止应答失败、重新取得控制时起点变化等故障。
- [test_control.py](../../test/test_control.py)：运动阶段 >40 ms 延迟仍停止；诊断数据与停止返回值验证。

新增状态：`ACQUIRING_CONTROL`、`STOPPING_AT_VIEW`、`VERIFYING_STATIONARY`。采集期间状态字段为 `servo_active=false`、`capture_stationary_guard=true`。

`servo_trace.npz` 保持原来的 21 列样本，新增 `segment_index`。只有同一 segment_index 的相邻样本才用于计算伺服周期和命令步长；段间间隔属于停止采集。`result.json.motion_segments` 记录各段样本范围、停止错误及周期诊断，避免把采集间隔当作漏周期。

## 离线验证结果

所有测试使用隔离 ROS 域与模拟反馈/控制接口；真实 RTDE 构造函数被替换成直接报错，测试机器人地址为 `OFFLINE_ONLY`。CUDA 测试图像来自合成平面。

| 验证 | 结果 |
|---|---|
| 控制层单元测试 | **19 项通过**；运动阶段延迟 >40 ms 仍停止，故障诊断记录实际延迟；停止返回 False 可见 |
| 协调器故障测试 | **12 类通过**；含采集漂移/异常速度/安全状态/反馈丢失、取消、停止失败、再次连接时位置变化 |
| 完整模拟 8 点 + 真实 CUDA | **DONE；8 点各记录至少 8 帧，实际共融合 67 帧**；完成后停止，第二轮取消也通过 |
| S08 采集首帧后主机停顿 | 故意持有主进程 GIL **80.44 ms**；采集期间所有模拟控制连接已关闭，命令计数不增加，最终 S08 正常完成 |
| 运动段记录 | 8 段单独索引；最大单周期命令步长 0.000240 rad；段内周期 P95 3.04 ms（不是硬实时证明） |
| 构建 | colcon 构建通过，无节点启动 |

证据：[unit.log](unit.log)、[faults/results.json](faults/results.json)、[integration/results.json](integration/results.json)、[build.log](build.log)。生产代码改动对照：[production_changes.patch](production_changes.patch)。

完整模拟运行记录：`integration/plans/run_20260929_133611_a3add7/result.json`，S01–S08 的协调器帧数均为 8；地图总计 67 帧，是因为关门前还可能完成额外融合帧。模拟结果不修改先前真实测试的 ERROR 记录。

离线复现（不启动真实反馈/控制）：

```bash
cd /home/wqj/realtime_nvblox_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
AUTO_PY=/home/wqj/miniconda3/envs/whole_control/bin/python
$AUTO_PY -B -m pytest -q src/ur7e_auto_reconstruction/test/test_control.py
ROS_DOMAIN_ID=86 AUTO_TEST_OUTPUT=/tmp/ur7e_capture_fix_faults $AUTO_PY -B src/ur7e_auto_reconstruction/test/run_faults.py
ROS_DOMAIN_ID=84 AUTO_TEST_OUTPUT=/tmp/ur7e_capture_fix_integration $AUTO_PY -B src/ur7e_auto_reconstruction/test/run_integration.py
```

## 仍需用户检查与批准的事项

- 本次不再进行真机执行；软件通过不能替代真机性能确认。
- 运动阶段仍采用普通 Linux/Python 线程，不宣称硬实时；若运动期间出现调度超时，仍会停止，不会放宽阈值或跳点追赶。
- 之后批准复测时，重新根据实际起点生成预览，检查逐点停止/重新连接是否平稳、S08 是否采满至少 8 帧、最终是否 DONE。
- 原有环境/自身碰撞验证边界不变，仍没有无碰撞保证。
