# ur7e_gazebo_esdf_sage_continuous：连续局部 SAGE 实验

[返回项目文档索引](../README.md)

## 职责和边界

源码 launch 默认 strict、online_engine=v1、online_obstacles=false；README 的 v2/bounded_unknown 示例需要显式参数。目标周期 40 ms、伺服 200 Hz 是设计目标而非本次测得性能。

## 输入、处理与输出

**数据链路：** 冻结底图 + 独立在线感知快照 → 常驻 SAGE 优化进程 → 最新图检查 → 原子短轨迹替换 → 独立伺服。

新目标建立最终末端目标并重置一次分布；之后按反馈时间平移预测时域；最多取四个 50 ms 动作重检；旧段有效时可继续执行，300 ms 无新计划停止推进；停滞时后台引导搜索不阻塞局部优化。

## 目录结构

```text
ur7e_gazebo_esdf_sage_continuous/
├── config/
│   ├── collision_spheres.yaml
│   ├── gazebo.yaml
│   ├── initial_positions.yaml
│   ├── mapping.yaml
│   ├── moving_stage2.rviz
│   ├── real_calibrated.urdf
│   ├── scan.yaml
│   ├── scan_coverage.yaml
│   ├── scan_high_central.yaml
│   ├── scan_oblique.yaml
│   ├── tabletop.yaml
│   └── view.rviz
├── launch/
│   └── bringup.launch.py
├── scripts/
│   ├── closed_loop
│   ├── control
│   └── spawn_fixture
├── test/
│   ├── audit_continuous.py
│   ├── audit_crossing_route.py
│   ├── audit_moving.py
│   ├── audit_online.py
│   ├── audit_ownership.py
│   ├── audit_partial.py
│   ├── audit_tabletop.py
│   ├── bench_online_worker.py
│   ├── benchmark_self_filter.py
│   ├── design_coverage_scan.py
│   ├── design_high_central_scan.py
│   ├── design_oblique_scan.py
│   ├── design_scan.py
│   ├── diagnose_owned_boundary.py
│   ├── diagnose_subvoxel.py
│   ├── faults_continuous.py
│   ├── find_targets.py
│   ├── moving_scenario.py
│   ├── online_scenario.py
│   ├── performance.py
│   ├── plot_moving.py
│   ├── plot_online.py
│   ├── plot_ownership.py
│   ├── plot_partial.py
│   ├── plot_run.py
│   ├── probe_online_process.py
│   ├── probe_runtime.py
│   ├── record_control.py
│   ├── replay_tabletop.py
│   ├── system_scenario.py
│   ├── test_continuous.py
│   ├── test_offline.py
│   ├── test_online.py
│   ├── test_online_v2.py
│   ├── test_ownership.py
│   ├── test_partial_map.py
│   ├── test_self_filter.py
│   ├── test_stationary.py
│   ├── test_tabletop.py
│   ├── validate_run.py
│   └── visualize_moving.py
├── ur7e_gazebo_esdf_sage_continuous/
│   ├── __init__.py
│   ├── capture.py
│   ├── config.py
│   ├── continuous.py
│   ├── field.py
│   ├── mapper.py
│   ├── markers.py
│   ├── node.py
│   ├── online.py
│   ├── online_v2.py
│   ├── optimizer.py
│   ├── ownership.py
│   ├── partial_map.py
│   ├── planning.py
│   ├── scene.py
│   ├── self_filter.py
│   ├── stationary.py
│   └── task.py
├── validation/
├── CMakeLists.txt
├── README.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/bringup.launch.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/launch/bringup.launch.py)

组装应用进程、配置路径和可选可视化；参数可改变实际启动路径。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `setup(context)` | 14 | 解析 launch 上下文中的路径/开关，准备资源并返回实际启动动作。 |
| `generate_launch_description()` | 59 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.scene`。

启动/命令行参数声明：

```python
DeclareLaunchArgument('output', default_value=str(Path.home() / 'nvblox_output/gazebo_esdf_sage' / datetime.now().strftime('%Y%m%d_%H%M%S')))
DeclareLaunchArgument('gazebo_gui', default_value='false')
DeclareLaunchArgument('run_rviz', default_value='false')
DeclareLaunchArgument('auto_start', default_value='true')
DeclareLaunchArgument('scan_config', default_value='')
DeclareLaunchArgument('planning_policy', default_value='strict')
DeclareLaunchArgument('test_load_map', default_value='')
DeclareLaunchArgument('online_engine', default_value='v1')
DeclareLaunchArgument('online_obstacles', default_value='false')
DeclareLaunchArgument('hold_after_scan', default_value='false')
```

### `scripts/closed_loop`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/scripts/closed_loop)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
SIM_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$SIM_ENV"
export PATH="$SIM_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$SIM_ENV/bin/python" -B -m ur7e_gazebo_esdf_sage_continuous.node "$@"
```

### `scripts/control`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/scripts/control)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

源码说明：Client for the isolated simulation, never a hardware driver.

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `'/sim_esdf_sage/status'` | 13 |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 20 |
| `create_client` | `Trigger` | `'/sim_esdf_sage/' + a.command` | 28 |

启动/命令行参数声明：

```python
p.add_argument('command', choices=['status', 'target', 'start', 'stop', 'invalidate', 'test_feedback_loss'])
p.add_argument('xyz', nargs='*', type=float)
```

### `scripts/spawn_fixture`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/scripts/spawn_fixture)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

源码说明：Static test fixture; dimensions are not published to the planner.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 8 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `SpawnEntity` | `'/spawn_entity'` | 10 |
| `create_publisher` | `String` | `'/sim_esdf_sage/fixture_ready'` | 21 |

### `test/audit_continuous.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_continuous.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Independent audit of every submitted short horizon and continuity metrics.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.trajectory`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_crossing_route.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_crossing_route.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Check whether obstacle sweep intersects a sensor-only static baseline route.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_gazebo_esdf_sage_continuous.planning`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_moving.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_moving.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Independent whole-arm audit against recorded moving-object ground truth.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.trajectory`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_online.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_online.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Post-run stage-one audit; fixture truth is used only here, never by control.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.trajectory`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_ownership.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_ownership.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Post-run audit of body interiors, with an independent hull/corner check.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.ownership`, `ur7e_nvblox.depth`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_partial.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_partial.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Independent policy and actual trajectory audit; no fixture geometry input.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_nvblox.depth`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/audit_tabletop.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/audit_tabletop.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only post-run audit. Fixture truth is never returned to the planner.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/bench_online_worker.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/bench_online_worker.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Exercise spawn-based geometry worker using saved sensor evidence.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 15 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.online_v2`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/benchmark_self_filter.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/benchmark_self_filter.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Reproduce Python-thread contention; diagnostic, not a timing assertion.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 16 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.self_filter`, `ur7e_perception_planning_core.self_filter`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
parser.add_argument('output')
```

### `test/design_coverage_scan.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/design_coverage_scan.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Commission extra observation poses, never generate planning free space.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 21 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('--output', default=str(ROOT / 'config/scan_coverage.yaml'))
```

### `test/design_high_central_scan.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/design_high_central_scan.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline fixture certification of low and high central scan poses; no planning-map input.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `gaps(a, b, step=0.002)` | 11 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_real_static.model`。

### `test/design_oblique_scan.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/design_oblique_scan.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline fixture certification of oblique scan poses; no planning-map input.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `gaps(a, b, step=0.002)` | 11 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_real_static.model`。

### `test/design_scan.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/design_scan.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline fixture commissioning only. Runtime planning never imports truth.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Fixture` | 9 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `query`（L10） |
| `main()` | 18 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_perception_planning_core.geometry`, `ur7e_real_static.model`。

### `test/diagnose_owned_boundary.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/diagnose_owned_boundary.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only geometric witness: interior points rejected by whole-cell ownership.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.ownership`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
parser.add_argument('output')
```

### `test/diagnose_subvoxel.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/diagnose_subvoxel.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline conservative subdivision experiment; never writes a planning map.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `classify(points, res)` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.ownership`, `ur7e_nvblox.depth`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
p.add_argument('--levels', type=int, default=4)
```

### `test/faults_continuous.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/faults_continuous.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Simulation-only local outage injection; verifies actual command hold and recovery.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `receive(msg)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `wait(pred, timeout)` | 22 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `call(service)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `'/sim_esdf_sage/status'` | 19 |
| `create_subscription` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 20 |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 21 |
| `create_client` | `Trigger` | `'/sim_esdf_sage/' + service` | 30 |

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/find_targets.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/find_targets.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline test-case selection from a frozen map; never used by the controller.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/moving_scenario.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/moving_scenario.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Independent Gazebo moving-object experiment; truth never enters the planner.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `now()` | 27 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `state(m)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `odom(m)` | 31 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `live_status(msg)` | 35 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `same_run()` | 39 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `status()` | 44 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `record(kind, **data)` | 47 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `wait(fn, timeout=120)` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `pause(t)` | 58 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `call(kind, topic, req)` | 60 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `speed(y)` | 65 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `goal(xyz)` | 67 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `visible_cells(center)` | 69 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `spawn(center)` | 76 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `remove()` | 84 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_perception_planning_core.geometry`, `ur7e_real_static.model`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `'/sim_esdf_sage/status'` | 38 |
| `create_subscription` | `JointState` | `'/joint_states'` | 40 |
| `create_subscription` | `Odometry` | `namespace + '/odom'` | 41 |
| `create_subscription` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 42 |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 43 |
| `create_publisher` | `Twist` | `namespace + '/cmd_vel'` | 43 |
| `create_client` | `kind` | `topic` | 62 |

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/online_scenario.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/online_scenario.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Gazebo-only stage-one experiment. Fixture truth stays in this test process.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `joint(m)` | 21 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `status()` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `record(kind, **kw)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `wait(predicate, seconds)` | 30 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `pause(seconds)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `call(kind, service, request)` | 40 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `region_cells()` | 47 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_perception_planning_core.geometry`, `ur7e_real_static.model`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 20 |
| `create_subscription` | `JointState` | `'/joint_states'` | 23 |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 24 |
| `create_client` | `kind` | `service` | 41 |

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/performance.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/performance.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Report measured command arrivals and planning latency without equating rates.

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/plot_moving.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/plot_moving.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Measured motion and online-update timeline, independent of planner input.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/plot_online.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/plot_online.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Plot measured online updates and independent experiment events.

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/plot_ownership.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/plot_ownership.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Map cross sections showing sensed freedom, owned interiors and unknowns.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/plot_partial.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/plot_partial.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Show environmental evidence separately from explicitly admitted uncertainty.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_nvblox.depth`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/plot_run.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/plot_run.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Export measured trajectory and independent clearance evidence.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/probe_online_process.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/probe_online_process.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only live component check of isolated mapping, no robot commands.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 15 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.online_v2`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 18 |
| `create_subscription` | `Image` | `c['depth_topic']` | 18 |

启动/命令行参数声明：

```python
p.add_argument('source')
p.add_argument('output')
```

### `test/probe_runtime.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/probe_runtime.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only checks of live calibrated TF, rendered depth and ros2_control.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `receive(name, msg)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.self_filter`, `ur7e_nvblox.depth`, `ur7e_real_static.camera_collision`, `ur7e_real_static.model`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `ListControllers` | `'/controller_manager/list_controllers'` | 31 |
| `create_subscription` | `kind` | `topic` | 30 |

启动/命令行参数声明：

```python
p.add_argument('output')
p.add_argument('--expect-hold', action='store_true')
```

### `test/record_control.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/record_control.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only command arrival timing; ROS receipt timing is not publisher timing.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `status(msg)` | 13 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `command(msg)` | 16 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `'/sim_esdf_sage/status'` | 18 |
| `create_subscription` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 19 |

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/replay_tabletop.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/replay_tabletop.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Audit new semantics using recorded depth views and saved native ESDF.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_gazebo_esdf_sage_continuous.self_filter`, `ur7e_nvblox.depth`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('session')
p.add_argument('output')
```

### `test/system_scenario.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/system_scenario.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Run isolated live Gazebo acceptance steps; never starts a real robot.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `status()` | 6 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `wait(predicate, seconds)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `control(*args)` | 17 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `target(xyz)` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `outcome(rev, seconds=190)` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `record(name, **kwargs)` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `probe(name)` | 34 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

启动/命令行参数声明：

```python
p.add_argument('output')
p.add_argument('--scan-timeout', type=float, default=1200.0)
```

### `test/test_continuous.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_continuous.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_shift_accumulates_fractional_elapsed_time()` | 4 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_only_collision_free_prefix_is_executable(monkeypatch)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_expired_short_horizon_cancels_all_queued_motion()` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_stale_map_discards_current_and_future_segments()` | 27 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_replacement_between_validation_and_execution_waits_for_recheck(monkeypatch)` | 39 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.continuous`, `ur7e_gazebo_esdf_sage_continuous.online`。

### `test/test_offline.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_offline.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_real_calibration_and_render_pose()` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_scan_corridor_and_unknown()` | 30 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `servo_fixture(elapsed=0.0, age=0.0)` | 48 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_servo_clock_jump_is_bounded()` | 64 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_tracking_failure_never_publishes_rejected_command()` | 70 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_paused_simulation_holds_segment_until_fresh_feedback()` | 76 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_fresh_joint_stamp_ahead_of_clock_is_not_expired()` | 82 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_transient_stale_feedback_holds_then_latches()` | 95 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_gazebo_esdf_sage_continuous.node`, `ur7e_gazebo_esdf_sage_continuous.scene`, `ur7e_real_static.camera_collision`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.trajectory`。

### `test/test_online.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_online.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_hidden_obstacle_is_retained_until_a_free_ray_clears_it()` | 4 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_stale_online_map_cancels_segment_and_holds_command()` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.online`。

### `test/test_online_v2.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_online_v2.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Sparse clearing must exactly preserve the original dense occupancy rule.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_sparse_clearing_equals_dense_at_occlusion_and_invalid_pixels()` | 7 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_online_recovery_holds_before_reusing_a_fresh_snapshot()` | 16 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.node`, `ur7e_gazebo_esdf_sage_continuous.online`, `ur7e_perception_planning_core.grid`。

### `test/test_ownership.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_ownership.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_entire_voxel_is_inside_rotated_solid_without_expansion()` | 8 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_ownership_does_not_clear_outside_unknown_or_measured_obstacles()` | 23 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_saved_snapshot_rejects_tampering_and_wrong_environment(tmp_path)` | 42 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.ownership`, `ur7e_real_static.field`。

### `test/test_partial_map.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_partial_map.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_partial_policy_preserves_unknown_and_obstacles_and_bounds_exploration()` | 6 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_partial_policy_refuses_empty_evidence()` | 22 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_policy_selection_uses_local_scan_config(tmp_path)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`。

### `test/test_self_filter.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_self_filter.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：The batched filter must preserve every pixel of the reference mask.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_batched_mask_matches_reference_for_camera_and_robot_views()` | 11 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.self_filter`, `ur7e_perception_planning_core.self_filter`, `ur7e_real_static.model`。

### `test/test_stationary.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_stationary.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `samples()` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `check(states, now=3.0)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_slow_static_result_needs_continuous_fresh_stationary_feedback()` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.stationary`。

### `test/test_tabletop.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/test_tabletop.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Perception-only column inference, boundaries, counterevidence and isolation.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `setup()` | 11 | 解析 launch 上下文中的路径/开关，准备资源并返回实际启动动作。 |
| `frame(intr, tx=0.0, height=0.0, hole=False)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `update(f, intr, tx=0.0, height=0.0, hole=False, occupied=None)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `index(f, xyz)` | 30 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_two_distinct_views_required_and_ray_vs_inference()` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_invalid_depth_holes_not_filled_and_obstacles_override_columns()` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_higher_top_retracts_previous_inference()` | 52 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_side_faces_cannot_support_top_columns()` | 59 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_old_prior_identity_is_rejected()` | 71 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_robot_hit_retains_only_measured_free_ray_before_return()` | 89 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.field`, `ur7e_real_static.config`, `ur7e_real_static.field`。

### `test/validate_run.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/validate_run.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Independent audit of measured motion; never supplies geometry to control.

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`, `ur7e_gazebo_esdf_sage_continuous.ownership`, `ur7e_gazebo_esdf_sage_continuous.partial_map`, `ur7e_real_static.field`, `ur7e_real_static.model`。

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `test/visualize_moving.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/test/visualize_moving.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only RViz overlay. Test-object truth is visualization-only, never planner input.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `on_status(msg)` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `on_odom(msg)` | 23 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `marker(i, kind, color)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `tick()` | 30 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_gazebo_esdf_sage_continuous.config`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `MarkerArray` | `'/stage2_visualization'` | 18 |
| `create_subscription` | `String` | `'/sim_esdf_sage/status'` | 26 |
| `create_subscription` | `Odometry` | `'/obstacle_' + out.name + '/odom'` | 27 |

启动/命令行参数声明：

```python
p.add_argument('output')
```

### `ur7e_gazebo_esdf_sage_continuous/__init__.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

### `ur7e_gazebo_esdf_sage_continuous/capture.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/capture.py)

停稳深度采集工作循环：匹配图像时刻关节与 TF，过滤本体，融合并提交距离场。

源码说明：Stationary-view nvBlox capture. No Gazebo priors and no robot commands.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `worker(node)` | 17 | 工作循环入口；处理停止通知与异常，将实际处理转交工作函数。 |
| `_worker(node)` | 27 | 实际后台处理循环；按本文件职责读取输入、校验并提交结果。 |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_perception_planning_core.runtime`, `ur7e_real_static.trajectory`。

### `ur7e_gazebo_esdf_sage_continuous/config.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/config.py)

加载本包 mapping 配置并计算包含场景模型/参数的身份标识。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `load_mapping(path)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `identity(c)` | 53 | 将模型文件与关键建图配置绑定为身份标识，供冻结/重载一致性检查。 |

直接项目/规划依赖：`ur7e_real_static.config`。

### `ur7e_gazebo_esdf_sage_continuous/continuous.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/continuous.py)

针对最终目标持续优化；按时间平移预测时域，重检至多四步前缀，并原子替换待执行队列；停滞时异步请求引导搜索。

源码说明：Final-goal SAGE, persistent shifted horizon, asynchronous bounded execution.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `background_route(config, snapshot, start, goal, anchors)` | 13 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `checked_prefix(g, snapshot, start, velocities, dt, margin, self_margin)` | 22 | A rejected later edge does not invalidate the earlier checked prefix. |
| `shift_count(now, origin, dt, horizon)` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `reach(n, task, goal, token)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.model`, `ur7e_real_static.trajectory`。

### `ur7e_gazebo_esdf_sage_continuous/field.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/field.py)

区分扫描阶段几何先验与深度规划场；根据观测到的可靠桌面列推断上方自由并保留来源。

源码说明：Scan-only geometric prior, separate from the depth-derived planning field.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `scan_samples(views, step=0.015)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `ScanPriorField` | 16 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L17） |
| 类 `FieldBuilder` | 32 | Only depth rays and declared tabletop extrapolation enter planning. |
| 方法 | — | `__init__`（L40）、`_observe_tops`（L52）、`update`（L96）、`evidence`（L133） |

直接项目/规划依赖：`ur7e_perception_planning_core.grid`, `ur7e_real_static.field`, `ur7e_real_static.model`。

### `ur7e_gazebo_esdf_sage_continuous/mapper.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/mapper.py)

EsdfMapper 管理仿真预扫描深度门与冻结地图；与真机采集共享底层工具，但添加本包策略。

源码说明：Scan-only ESDF mapper: no target subscription, Action client or robot control.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `EsdfMapper` | 37 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `on_state`（L39）、`__init__`（L58）、`service`（L98）、`on_info`（L107）、`on_depth`（L110）、`on_lease`（L119）、`capture_allowed`（L121）、`start_scan`（L126）、`capture`（L138）、`record_capture`（L155）、`publish_cloud`（L166）、`freeze_scan`（L170）、`load`（L200）、`warmup`（L214）、`_warmup`（L225）、`report`（L238）、`close`（L253） |
| `main()` | 259 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.stats`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.node`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.node`, `ur7e_real_static.visualization`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PointCloud2` | `'~/esdf'` | 78 |
| `create_publisher` | `PointCloud2` | `'~/surface_esdf'` | 79 |
| `create_publisher` | `String` | `'~/capture_status'` | 80 |
| `create_subscription` | `JointState` | `c['joint_topic']` | 82 |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 83 |
| `create_subscription` | `Image` | `c['depth_topic']` | 87 |
| `create_subscription` | `Empty` | `'/sim_esdf_sage/capture_lease'` | 88 |

启动/命令行参数声明：

```python
p.add_argument('--config', required=True)
p.add_argument('--load', default='')
```

### `ur7e_gazebo_esdf_sage_continuous/markers.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/markers.py)

发布当前碰撞球包络、末端及轨迹等 RViz 标记。

源码说明：Measured collision cover and TCP trace in RViz.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `publish(node, q)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_gazebo_esdf_sage_continuous/node.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/node.py)

ClosedLoop 负责仿真反馈、预扫描、目标、在线地图和短段执行保护；reach 决定所用规划策略。

源码说明：Simulation-only prescan and mode B: every SAGE step starts at measured q,dq.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `ClosedLoop` | 35 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L36）、`on_fixture`（L68）、`now`（L70）、`state`（L72）、`feedback_ready`（L78）、`event`（L82）、`start`（L87）、`fail`（L92）、`stop`（L99）、`invalidate`（L103）、`inject_feedback_loss`（L107）、`inject_map_stale`（L111）、`read_online`（L116）、`on_goal`（L120）、`wait`（L135）、`set_segment`（L149）、`segment_done`（L157）、`tick`（L160）、`settled`（L227）、`scan_scene`（L231）、`online_snapshot`（L280）、`reach`（L295）、`run`（L299）、`status`（L366）、`close`（L377） |
| `main()` | 384 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_perception_planning_core.demo`, `ur7e_perception_planning_core.geometry`, `ur7e_real_static.field`, `ur7e_real_static.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 46 |
| `create_publisher` | `String` | `'~/status'` | 47 |
| `create_publisher` | `MarkerArray` | `'~/markers'` | 48 |
| `create_subscription` | `PoseStamped` | `'/target_pose'` | 49 |
| `create_subscription` | `String` | `'/sim_esdf_sage/fixture_ready'` | 50 |
| `create_service` | `Trigger` | `'~/start'` | 52 |
| `create_service` | `Trigger` | `'~/stop'` | 53 |
| `create_service` | `Trigger` | `'~/invalidate'` | 54 |
| `create_service` | `Trigger` | `'~/test_feedback_loss'` | 55 |
| `create_service` | `Trigger` | `'~/test_map_stale'` | 56 |

启动/命令行参数声明：

```python
parser.add_argument('--config', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--auto-start', default='true')
parser.add_argument('--test-load-map', default='')
```

### `ur7e_gazebo_esdf_sage_continuous/online.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/online.py)

线程式新增障碍层：深度端点加入占据，有效自由射线清除占据；与静态场保守合成。

源码说明：Sensor-only online added-obstacle layer over an immutable static policy.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `DynamicBlocked` | 15 | 相关状态与操作的封装；方法见下一行。 |
| `update_occupancy(previous, free, hit, shape)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `OnlineLayer` | 23 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L25）、`submit`（L33）、`read`（L36）、`status`（L41）、`worker`（L43）、`close`（L101） |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_perception_planning_core.grid`。

### `ur7e_gazebo_esdf_sage_continuous/online_v2.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/online_v2.py)

将 ROS 感知和几何处理放到独立进程，按完整版本提交在线占据和距离快照。

源码说明：Sensor-only online added-obstacle layer over an immutable static policy.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `update_occupancy(previous, free, hit, shape)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `init_geometry_worker(config, extra, base, static_obstacle)` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `process_geometry(depth, pose, intr, q, occupied, old_distance, ray_depth=None, ray_intr=None)` | 36 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `_LayerWorker` | 71 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L73）、`submit`（L86）、`read`（L89）、`status`（L94）、`diagnostic`（L96）、`worker`（L99）、`close`（L154） |
| `_sensor_process(config, extra, signature, session, base, output, shared, header, mutex, stop)` | 158 | Own ROS executor and geometry worker; never shares a GIL with SAGE/servo. |
| 类 `OnlineLayer` | 206 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L208）、`frames`（L216）、`version`（L219）、`submit`（L221）、`read`（L222）、`status`（L230）、`close`（L234） |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_perception_planning_core.grid`, `ur7e_real_static.model`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `config['joint_topic']` | 187 |
| `create_subscription` | `CameraInfo` | `config['camera_info_topic']` | 188 |
| `create_subscription` | `Image` | `config['depth_topic']` | 189 |

### `ur7e_gazebo_esdf_sage_continuous/optimizer.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/optimizer.py)

单个常驻 SAGE 工作进程，最多一个未完成任务；地图按版本传输，新目标重置分布，后续 optimize 使用 shift_steps。

源码说明：One persistent SAGE process; bounded one-job mailbox, no servo/GPU GIL sharing.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `initialize(config, directory, session)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `compute(job)` | 22 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `Planner` | 39 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L40）、`submit`（L43）、`poll`（L49）、`warm`（L52）、`close`（L55） |

直接项目/规划依赖：`ur7e_real_static.model`。

### `ur7e_gazebo_esdf_sage_continuous/ownership.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/ownership.py)

在冻结姿态下证明完整体素属于 URDF 本体实体，生成绑定原地图的派生检查场；不改变环境观测证据。

源码说明：Frozen robot-owned solids, distinct from sensed environmental free space.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `contained_cells(points, resolution, vertices, normals, face_points)` | 19 | Convex half-space proof for the entire axis-aligned voxel, not its centre. |
| 类 `RobotOwnership` | 31 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L32）、`mask`（L69） |
| `collision_snapshot(frozen, owned, native, known)` | 79 | Keep environmental evidence unchanged; derive one conservative check field. |
| `prepare(frozen, geometry, q, session)` | 99 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `load_snapshot(frozen, path)` | 126 | Read an immutable derived field bound to the exact sensing map and meshes. |

直接项目/规划依赖：`ur7e_nvblox.depth`。

### `ur7e_gazebo_esdf_sage_continuous/partial_map.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/partial_map.py)

生成显式有限未知准入的仿真可行性场与不确定性代价；正距离不等于已观测自由。

源码说明：Explicit bounded-unknown planning policy, separate from environmental evidence.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `fields(frozen, owned, raw, known, band=BAND)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `prepare(frozen, session, band=BAND)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `load_snapshot(frozen, path)` | 54 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `attach_cost(rollout, session)` | 65 | Add uncertainty ranking to the unchanged SAGE optimizer/collision costs. |

直接项目/规划依赖：`ur7e_nvblox.depth`。

### `ur7e_gazebo_esdf_sage_continuous/planning.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/planning.py)

关节空间引导搜索及路径检查；不同包的 SAGE 调用方式见下文流程。

源码说明：Bounded ESDF guide search; reserve time for connections after goal IK.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `route(g, snapshot, start, xyz, anchors, c, cancel=lambda : False)` | 11 | 在给定模型、地图与预算内寻找可检查的关节引导路径；不能搜索成功时抛错/返回失败。 |
| `shortcut(g, snapshot, path, c)` | 68 | Remove detours only after checking the complete replacement edge. |

直接项目/规划依赖：`ur7e_real_static.trajectory`。

### `ur7e_gazebo_esdf_sage_continuous/scene.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/scene.py)

展开带手眼外参的统一机器人模型，供 Gazebo、TF、CPU 几何和 SAGE 使用。

源码说明：Build one calibrated model for Gazebo, TF, depth pose and SAGE FK.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `build_robot(share)` | 11 | 根据包内资源生成含相机安装外参的统一机器人描述。 |

直接项目/规划依赖：`ur7e_real_static.camera_collision`。

### `ur7e_gazebo_esdf_sage_continuous/self_filter.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/self_filter.py)

沿相机射线按机器人碰撞球屏蔽本体像素，避免将本体当作环境障碍。

源码说明：Mask measured robot pixels, including the fixed base excluded from moving balls.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `robot_depth_mask(depth, pose, intr, centers, radii, link_ids, padding=0.025, depth_tolerance=0.06)` | 4 | Same ray/sphere rule as the shared filter, evaluated in array batches. |
| `mask(depth, pose, intr, centers, radii, link_ids, extra)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_gazebo_esdf_sage_continuous/stationary.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/stationary.py)

核查计算期间机器人仍停稳，判断延迟完成的采集结果是否可接受。

源码说明：Validate delayed computation only while a static-scene capture stays still.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `result_rejection(c, states, q, stamp, received, now, wall_now)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_gazebo_esdf_sage_continuous/task.py`

[查看源码](../../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/task.py)

构建连续模式的 SAGE task 和预测参数，复用碰撞代价及机器人模型。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `build_task(g, c, directory)` | 4 | 按本包模型和配置构建 SAGE 任务，具体控制参数由配置决定。 |

直接项目/规划依赖：`storm_kit.mpc.rollout.sage_arm_reacher`, `ur7e_perception_planning_core.sage`。

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_gazebo_esdf_sage_continuous/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/collision_spheres.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/collision_spheres.yaml) | 配置分区/键：`robot_name` |
| [config/gazebo.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/gazebo.yaml) | 配置分区/键：`gazebo` |
| [config/initial_positions.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/initial_positions.yaml) | 配置分区/键：`elbow_joint` |
| [config/mapping.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/mapping.yaml) | 配置分区/键：`robot_urdf` |
| [config/moving_stage2.rviz](../../src/ur7e_gazebo_esdf_sage_continuous/config/moving_stage2.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [config/real_calibrated.urdf](../../src/ur7e_gazebo_esdf_sage_continuous/config/real_calibrated.urdf) | 展开后的机器人几何/运动学模型。 |
| [config/scan.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/scan.yaml) | 配置分区/键：`views` |
| [config/scan_coverage.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/scan_coverage.yaml) | 配置分区/键：`views` |
| [config/scan_high_central.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/scan_high_central.yaml) | 配置分区/键：`views` |
| [config/scan_oblique.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/scan_oblique.yaml) | 配置分区/键：`views` |
| [config/tabletop.yaml](../../src/ur7e_gazebo_esdf_sage_continuous/config/tabletop.yaml) | 配置分区/键：`policy` |
| [config/view.rviz](../../src/ur7e_gazebo_esdf_sage_continuous/config/view.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [package.xml](../../src/ur7e_gazebo_esdf_sage_continuous/package.xml) | ROS 包名、依赖及构建类型声明。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_gazebo_esdf_sage_continuous/README.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
