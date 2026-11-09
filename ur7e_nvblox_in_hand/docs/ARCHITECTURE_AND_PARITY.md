# 代码结构、功能等价性与两类接口的区别

## 计算与接口的准确关系

官方ROS接口和nvblox_torch都调用nvblox C++/CUDA核心，两者都可用于实时系统。前者已有ROS图像队列、TF、调度、增量层发布；后者提供张量和Mapper接口，由应用组织ROS输入、时序、GPU工作及发布。底层版本/构建可能不同，不能认为两个.so只是同一个二进制换一种语言调用，也不能说Python版天生不考虑实时性。

本工程以Python组织应用，在单个GPU工作线程中调用wheel Mapper；小型桥接直接获得同一个Python Mapper持有的native Mapper，不创建第二张地图，不读取私有对象内存偏移，不用查询网格伪装原生体素。桥接头文件固定到wheel源码提交，链接wheel中的libpy_nvblox/libnvblox，不加载原workspaces。

## 目录与模块

```text
ur7e_nvblox_in_hand/
├── src/
│   ├── ur7e_d435_gazebo/              机械臂、D435、场景、控制器和模型副本
│   ├── ur7e_nvblox_interfaces/        FilePath与QueryEsdf服务
│   └── ur7e_nvblox_in_hand/
│       ├── ur7e_nvblox_in_hand/
│       │   ├── node.py               ROS输入、图像时刻TF、调度、服务与统计
│       │   ├── backend.py            wheel版本检查、参数传递、GPU调用
│       │   └── messages.py           PointCloud2、MarkerArray批量编码
│       ├── launch/bringup.launch.py  场景/实机/rosbag/Mapper/RViz编排
│       ├── config/                  base、realtime、reference、D435和DDS
│       └── rviz/                    3D ESDF、Mesh、2D切片三种标准插件视图
├── native/                          小型Torch C++/CUDA桥接
├── vendor/                          匹配wheel的官方/第三方编译头文件
├── tools/                           环境、GPU检查、场景测试、截图与观察路径
├── tests/                           ROS消息逐字节等价与字段契约检查
├── provenance/                      来源与最终源码哈希
├── docs/                            操作、结构、验收文档
└── build.sh / env.sh / run.sh        独立构建和启动入口
```

父目录COLCON_IGNORE防止原项目递归构建重复包；本工程通过--base-paths src显式构建自身三个包。`build/install/build_native/install_native`均为可重建产物，不是源码。

## 数据流和线程边界

```text
ROS深度/颜色图 + 各自CameraInfo → 有界输入队列
  → 按该图像header查机械臂TF（不使用最新TF冒充）
  → 单GPU所有者：深度TSDF / 颜色 / ESDF / Mesh / 服务操作
  → GPU批量压缩已观测体素并读回，或获取当前Mesh
  → CPU拥有的数据快照，按话题保留最新待发布结果
  → 独立发布线程：批量编码 / ROS发布 / RViz
```

队列容量和输入最大年龄有界，发生覆盖/过期/限频跳过时计数。动态参数仅支持频率、显示选择与主要开关；体素、坐标系、模式和算法参数需重启新地图，避免静默更改地图几何语义。

默认ESDF仅在新深度融合或合法地图维护使地图dirty时更新，发布按ESDF版本推进，不靠重复发布维持Hz。显示上限大于等于更新上限时直接跟随已有更新版本；显示更慢时另行限频。显式查询更新、衰减或清图可能在没有新相机stamp时改变地图，不能把这些维护更新当新相机观测。

TSDF仍为投影距离加权融合，颜色仍基于TSDF可见性和颜色体素平均；Mesh仍来自零交叉。更换接口不解决自体污染、图像内容时间误差、遮挡或粗体素的棱角损失。默认没有机器人自体遮罩，不宣称质量问题已全部修复。

## 功能对应与协议边界

| 功能 | 新Python工程实现 | 与原工程差异 |
| --- | --- | --- |
| UR7e场景、D435和机械臂pose | 本地场景副本、按图像时刻TF | 不依赖兄弟目录安装环境 |
| 深度/颜色融合 | wheel add_depth_frame/add_color_frame | wheel核心版本与ROS迁移库可能不同 |
| 3D ESDF | update_esdf + native EsdfVoxel导出 | 非固定ROI点阵；保留observed/inside/site |
| 2D ESDF | 桥接调用native updateEsdfSlice | 原生单层切片，不把3D随机点简单画平 |
| Mesh | wheel更新/获取完整彩色网格 | 标准MarkerArray整图替换，非官方nvblox增量消息 |
| TSDF/颜色层 | native已分配层体素，按权重筛选 | 标准PointCloud2，非官方自定义分块消息 |
| 相机位姿/轨迹 | camera_pose/camera_path | 来自机械臂TF，不称为VSLAM或发布假SLAM路径 |
| 衰减/半径清图 | wheel decay / native clearOutsideRadius | 默认关闭，可配置开启 |
| 保存/加载/清空 | 本地FilePath/Trigger服务 | 加载先检查体素一致；不是原nvblox_msgs类型 |
| ESDF距离/梯度查询 | QueryEsdf，显式位置或受限AABB | 自定义服务字段，旧客户端需要适配 |
| RGB输入、二维/三维/网格视图 | 标准RViz插件 | 不依赖nvblox_rviz_plugin/GXF材质资源 |
| 开关与参数 | depth/color/esdf/mesh/layers/pose/path等 | 部分ROS端口内部调试/容器参数不适用 |
| 实机D435/rosbag | 可选launch入口 | 代码入口保留，本轮验收以Gazebo为主 |

这里的“等价”指上述应用功能，不代表旧话题类型、服务请求字段、插件、统计文本格式或所有内部参数逐字相同。旧规划客户端/旧RViz配置不能不作适配直接使用；跨不同core版本的旧.blox地图兼容性未验收；新配置与服务例子已随工程提供。人物分割、任务识别、多相机和雷达不是本工程主体。

当前未增加官方人/动态地图工作流。`profile=reference`仅为2cm、较低颜色/ESDF频率的参考数值，不是整个workspaces任务工作流的复刻。颜色投影内部参数并非所有都由wheel封装；本版本不支持的配置会明确拒绝或不暴露，不把未生效参数写成有效优化。

## 实时性处理

1. GPU张量与native Mapper保留在wheel中，不经ROS C++节点或Isaac传输层。
2. PointCloud2使用array('B')批量赋值，避免Humble对bytes逐字节Python校验。
3. Mesh从本机ROS标准序列化器产生模板，用NumPy批量填充Point/Color数组，再发布标准CDR字节；不逐三角形创建数万个Python消息对象。与标准序列化逐字节一致测试覆盖空网格、多三角形、不同frame长度、UTF-8及时间戳；非标准/不兼容布局会断言失败，不静默发送错误消息。
4. 原生导出每批512块，显式等待wheel工作再压缩读回，仍有同步与传输成本，不能称为全链路零拷贝。
5. Mesh当前整图读取，地图变大时可成为瓶颈；没有声称达到官方分块增量协议的相同扩展性。

详细实测以验收文档为准；GPU提交耗时、ESDF调用Hz、发布Hz、新输入关联Hz与输入年龄分别报告，不能混为一个“实时频率”。
