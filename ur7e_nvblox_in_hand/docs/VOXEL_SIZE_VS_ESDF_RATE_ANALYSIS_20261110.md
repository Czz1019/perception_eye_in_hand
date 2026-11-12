# 为什么体素从 1 cm 改为 2 cm，ESDF 更新频率没有明显提升

日期：2026-11-10。对象：ur7e_nvblox_in_hand 当前 Python ROS 前端、nvblox_torch 后端与项目原生导出桥。本次仅分析代码和已有记录，没有改代码、改参数或新增测试。

## 1. 结论

在相同物理区域内，较大的体素通常会减少地图计算量，但它不保证完整管线达到 30 Hz。当前“ESDF 更新频率”统计的是共享工作线程调用更新函数的次数，不是 ESDF CUDA 内核自身的最大吞吐能力。输入、融合、更新调度、同步导出和 Mesh 读回都可能成为限制。

如果同一份可靠输入、相同版本、相同物理截断和范围、相同订阅及显示条件下，反复观察到 2 cm 比 1 cm 明显更慢，应继续定位，不能把它当成正常必然现象。本轮没有这样的严格配对日志，不能确定用户当前观察的唯一原因。

## 2. 当前代码如何决定更新次数

[node.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:215) 中的工作线程顺序是：

```text
处理服务任务
 → 取一帧深度并融合
 → 取一帧颜色并融合
 → 地图有变化且 ESDF 到期时更新
 → 有订阅且有新版本时导出 ESDF
 → Mesh 到期且有订阅时生成、读回
 → 按需发布其他层和维护
 → 等待/继续循环
```

depth 成功融合才设置 dirty_depth；update_esdf() 清除此标记。多次深度融合可以合并为一次 ESDF 更新。正常关闭衰减和清理时，输入或融合不足 30 Hz，不可能获得 30 Hz 的独立新观测地图。

这不是独立运行的 ESDF 高频线程；缩短 ESDF 内核耗时并不会让线程跳过其他工作。

## 3. 调度上限并不是固定频率保证

现有 due() 的核心为：

```python
if now < self.deadlines.get(name, 0):
    return False
self.deadlines[name] = now + 1.0 / hz
return True
```

其含义是“现在允许再执行一次吗”，不是固定相位的周期调度器。工作线程忙时错过的执行时刻不会补回，每次按实际执行时刻重新设置期限，会受到轮询和其他任务阻塞影响。

重要限定：如果深度地图仍然 dirty，下次工作循环会再次检查，不是一次未到门限就永远丢掉该深度变化。因此不能简单声称当前代码必然把 30 Hz 砍成 15 Hz。本次 33.4 Hz 上限约对应 29.94 ms，但实际有新观测的更新间隔还包括线程调度和串行工作。

## 4. 原生导出中有与 ESDF 内核不同的开销

[export.cu](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/native/export.cu:38) 的 run()：

1. 首先 cudaDeviceSynchronize()，等待此前设备任务完成；它不只代表 ESDF 计算时间。
2. getAllBlockIndices() 获取当前层全部已分配块，而不是只导出本轮变化块。
3. 每批至多 512 个块，组织主机索引与指针，复制到 GPU，运行压缩 kernel。
4. 把输出计数复制回 CPU 并同步，再复制点数据并同步。
5. 返回完整结果，Python 中继续执行 numpy().copy()，之后打包 PointCloud2。

stride 在 kernel 内筛选索引，但不会免掉遍历所有块、组织批次、分配输出缓冲或入口设备同步。增大 voxel_size 会减少部分规模相关工作，却不能消除这些步骤。

不能为了提频直接删除同步：该同步用于 wheel 私有流和项目导出流之间的所有权边界。若要优化，应建立正确的 CUDA stream/event 依赖及一致的读写快照，否则可能读到尚未完成的数据。

## 5. Mesh 也会阻塞更新线程

[backend.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:107) 每次 Mesh 任务执行 update_color_mesh()、get_color_mesh()，并把顶点、三角形和颜色搬到 CPU。

已有修复后 1 cm、stride=2 运行中，最后统计窗口的 Mesh 生成/读回平均约 36.71 ms；30 Hz 每帧预算仅 33.33 ms。虽然 Mesh 只设为 5 Hz，但它仍会周期性阻塞同一个更新线程，不是自动分摊成每帧几个毫秒。

点云和 Mesh 打包发布在另一个线程，仍会竞争 CPU、Python 执行时间、内存复制和 DDS 资源。不能把并发阶段所有耗时直接相加作为端到端延迟。

## 6. voxel_size 变大不代表所有工作减少 8 倍

固定物理体积且完全稠密覆盖时，体素边长加倍的理想体素数约为原来的 1/8。但本项目采用稀疏分块地图和增量 ESDF：实际成本取决于已分配块、发生变化的块、传播范围、几何表面积与观测轨迹，不是每次都处理整个固定稠密立方体。

保持输入 640×480 时，图像接收、部分图像转换与 CPU→GPU 输入复制的像素数量完全没有减少。TF 查询、ROS 调度、发布次数及若干固定开销也不按体素体积缩放。

还要核对以下关联参数：

| 条件 | 1 cm | 2 cm | 影响 |
|---|---:|---:|---|
| truncation_vox 固定 4 | 物理截断 4 cm | 物理截断 8 cm | 同时改变融合的物理距离带，非严格同一几何设置。 |
| esdf_max_site_vox 固定 2 | 对应 2 cm 距离阈值参数 | 对应 4 cm | 同时改变 ESDF 种子判定尺度，不能只理解为降低计算分辨率。 |
| stride 都为 2 | 显示典型间距 2 cm | 显示典型间距 4 cm | 导出点数通常会下降，但不会消除同步与调度开销。 |
| 旧 2 cm stride=1 对比新 1 cm stride=2 | 新显示间距 2 cm | 旧显示间距也为 2 cm | 两者导出点密度可能相近，DDS 负载不会自动减少 8 倍。 |

如果希望保持截断物理距离为 4 cm，2 cm 体素应对应 truncation_vox=2；这只是参数关系说明，本次没有修改配置，且这种变化还需结合融合质量需求决定。

## 7. 已有记录不能直接证明大体素反而更慢

本地找到的 2 cm 记录包含：

- [静止场景计数](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110/counter_rates.json)：深度融合约 18.70 Hz，ESDF 更新约 18.54 Hz。
- [多视角计数](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/lab_multiview_20261110/counter_rates.json)：深度融合约 13.37 Hz，ESDF 更新约 13.25 Hz。

这些配置没有新增 input_qos、esdf_view_stride 参数，属于可靠传输与显示抽样修复之前的记录；输入、运行过程、订阅/显示和代码状态均不完全相同。修复后的 1 cm 正常组输入约 29.95 Hz、融合约 28.7 Hz、ESDF 更新约 23.9 Hz，不能直接与前面的旧记录解释为体素变化的因果关系。

旧 2 cm 多视角记录还存在累计 depth_overwritten=1052、ESDF 输出覆盖 332 次，说明当时已经有明显的管线积压。其约 13 Hz 更新几乎跟随约 13 Hz 深度融合，并不能据此得出 ESDF 内核计算能力只有 13 Hz。

## 8. 为什么会出现小幅下降

在当前共享线程设计中，体素变化会改变计算和导出的完成时刻，进而改变 Mesh、ESDF、输入到达之间的相位关系。某次执行可能更容易与 Mesh 读回、CPU 发布或 DDS 调度重叠，导致次数小幅变化。地图历史、可见范围、运行长度、是否新建空图、订阅开关、GUI 和 GPU 其他负载也都会改变结果。

这些是代码允许的解释，不是本轮已经追踪证实的单一根因。只有重复稳定且足够大的下降，才应视为需要进一步追踪的反常性能现象，不能简单以噪声打发。

## 9. 如何分析才能判断 30 Hz 被谁限制

不新增实验的情况下，优先检查现有同轮日志：

- depth_received 是否接近 30 Hz；否则先看输入。
- depth_integrated 是否接近 30 Hz，overwritten、stale、gate_skipped 是否增长；否则先看融合调度和输入排队。
- esdf_updated 是否明显低于 depth_integrated；重点检查共享线程、due()、导出和 Mesh 阻塞。
- 发布次数是否接近更新次数、实际点云接收是否接近发布次数；否则重点检查输出覆盖、DDS 和接收端。
- esdf_submit_ms 只能代表主机调用耗时；需要 GPU event 或 profiler 才能分离真正 CUDA 计算、等待与数据传输时间。

若下一步获得用户授权继续优化，应先建立上述阶段的时间归因，再决定减少全量导出、复用缓冲、降低 Mesh 干扰或建立带正确 CUDA 依赖的解耦路径。不要靠调大更新参数或重复计算未改变的地图，制造 30 Hz 计数。

## 10. 当前能作出的判断

体素变大通常有利于底层地图计算，但当前实现的性能并不只由该计算决定。达到稳定 30 Hz 要求输入、融合、更新调度、同步导出和需要的输出送达都满足时间预算。现在不能承诺只把 voxel_size 改成 0.02 就能达到 30 Hz，也不能仅凭话题频率没有升高就认定底层 ESDF 没有变快。
