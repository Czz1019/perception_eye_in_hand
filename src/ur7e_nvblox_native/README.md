# UR7e 原生感知：官方 ROS 结构对齐（0.4.0 / 接口 v4）

[修改、目录、统计语义与完整操作说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。

主要参考及链接依赖为 workspaces 官方 ROS 移植库，应用修改均在 perception_eye_in_hand：

- native_mapper.cpp 直接调用 NvbloxNode::tick，复用官方输入、图像时刻 TF、TSDF/颜色/三维 ESDF、Mesh 与服务。
- mesh_view.h/cpp 封装按需标准 RViz 显示、删除与重连；快照不再强制全地图 updateMesh(kYes)。
- gpu_timings.h 用有界、复用的 CUDA 事件统计，移除应用逐帧深度/ESDF 强制同步；读回和显式清图仍有必要等待。
- 原生已分配块遍历导出 ESDF，不生成未知占位。
- TSDF 加权和 ESDF 最低权重对齐参考；保留 3 cm、640×360 深度、1280×720 RGB 和原 ROI。

默认 gazebo.yaml 的 ESDF 目标为 5 Hz；gazebo_fast_esdf.yaml 保留 30 Hz 目标。两者算法、精度、输入和范围相同。配置不代表实际 Hz。场景新增 gazebo_ros_init 时钟发布目标 100 Hz，不改变相机频率或物理步长。

显示话题保持接口 v3 的名称：esdf_occupied、esdf_distance_band、mesh、mesh_markers、color_layer_marker、camera_pose、camera_path；stats 和 topic_info 升级为接口 v4。统计区分提交、事件确认完成、实际发布；旧同步耗时字段不能沿用解释。

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./build_ur7e_native.sh
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_perception.sh gazebo_gui:=true
```

本轮完成编译/链接/安装和 host-only Mesh 协议检查；没有执行 CUDA 事件设备验收、仿真或机械臂运动，实际频率、平整度和完整性仍待同输入验证。旧版本性能数据不代表本版。
