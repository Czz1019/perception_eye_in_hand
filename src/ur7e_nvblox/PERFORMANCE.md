# 借鉴 realtime_nvblox_2：实现与验证

2026-09-23。完整实验记录见同工作区 `ur7e_perception_planning/validation/NVBLOX2_LEARNING.md`。

## 保留与借鉴

原实现已经有图像时刻 TF、最新帧队列和独立融合/发布线程。没有把这些已有功能算作新改进。
新实现借鉴参考包的 GPU 已观测体素筛选、减少复制、按订阅需求发布，并增加固定位置查询缓冲复用。
原始 `realtime_nvblox` 与参考 `realtime_nvblox_2` 均保持不变，优化封装在本包 `esdf.py` 中。

默认 `dense` 保留规则网格及未知 sentinel；可选 `sparse` 输出原生已观测候选体素上的有效 ESDF。
稀疏输出并不是整张地图的完整查询接口，不能根据没有输出某个位置推断该位置自由。

## 同输入 ROS 测试

whole_control，真实 nvBlox/CUDA，合成平面、移动相机、图像时间戳 TF；30 Hz 输入、160×90、3 cm 体素、15 Hz 更新配置、相同机器，依次独立运行。

| 实现 | 实测输出 Hz | 延迟 P50/P95 ms | 每帧点数 |
|---|---:|---:|---:|
| 修改前 dense | 10.09 | 61.44 / 64.71 | 52,111 |
| 修改后 dense | 13.99 | 60.45 / 72.17 | 52,111 |
| 修改后 sparse | 13.75 | 24.29 / 26.58 | 19,714 |
| 未修改参考前端 sparse，同参数 | 11.43 | 42.99 / 46.93 | 19,714 |

这是统一输入和参数的测试，不是用户原来 Gazebo 高分辨率场景的 15 Hz 复现。参考代码通过其 Python 前端运行；根目录参考脚本依赖的 source_gazebo_env.sh 缺失，未修改或补写该文件。
延迟从本地合成图像时间到接收点云计算，不代表真实相机曝光到控制执行延迟。dense 更新频率提高，但其 P95 延迟没有改善。

查询测试中，本包 sparse 输出的位置和距离与参考后端逐点一致；prepared query 与原生查询一致，复用输出后之前返回的 CPU 数组不变，空地图不会残留旧距离。

## 复现

ROS 和 install 已 source，使用 whole_control；用空闲隔离 ROS_DOMAIN_ID：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=106 \
  /home/wqj/miniconda3/envs/whole_control/bin/python -B \
  src/ur7e_nvblox/test/run_stream_benchmark.py --output /tmp/esdf_dense.json
# 稀疏模式：同上，另加 --mode sparse
```

生产仍严格查询图像时刻 TF，不改标定，不使用 latest TF，不放宽过期帧限制。
