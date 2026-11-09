# 表面模糊、颜色不正确及不同物体混合：代码分析

日期：2026-11-05。对象为当前 `/home/czz/perception_eye_in_hand/nvblox_in_hand`，以高位左右扫描时实际使用的代码、运行时参数和已有结果为依据。本轮仅阅读源码和已有文件、生成说明；没有启动仿真、没有重新采集/测试，没有修改感知程序或参数。

## 1. 结论

当前问题不是单纯“显示颜色不好看”，也不能由ESDF更新达到约30Hz推断重建质量合格。代码中最明确的组合是：**3cm体素的静态TSDF地图，将进入视场的活动机械臂一并融合；历史衰减关闭；颜色又依据这张TSDF的表面来投影和加权融合。** 几何一旦存在混合、残影或遮挡误判，颜色会跟着被涂到错误的表面，再被多帧平均，最终表现为边缘变钝、颜色扩散和局部表面混在一起。

但不能把截图中的所有连接或变色都判为错误：场景地面本来连接墙体，官方输出是单张几何地图而非分物体模型；RViz光照也影响明暗。需要区分真实接触、错误桥接、几何位置错误、颜色投影错误与显示差异。

本次代码分析能确定相关机制和配置，不能在没有新对照采集的情况下确定每一处伪影的唯一原因。

## 2. 当前数据流：几何和颜色不是同一个处理过程

```text
Gazebo深度传感器 → 深度图 + 深度CameraInfo + 图像时间TF
                                       ↓
                                TSDF几何融合
                                       ↓
Gazebo彩色传感器 → RGB图 + 彩色CameraInfo + 图像时间TF
                                       ↓
                 在现有TSDF上从彩色视角生成合成深度
                                       ↓
              表面/遮挡门限 → RGB插值 → ColorVoxel加权融合
                                       ↓
               TSDF零交叉生成Mesh → 从颜色体素给顶点着色
                                       ↓
                                   RViz显示
```

深度建立几何，颜色给已有几何着色；颜色本身不新增TSDF几何。看到物体颜色跑到另一个面上，不等于RGB代码直接把两个物体的几何拼接了。

入口 `launch/perception/nvblox.launch.py` 强制static_tsdf，使用官方NvbloxNode；诊断派生节点的processDepthImage/processColorImage仍调用父类，没有另一套私有融合算法。ESDF从几何地图生成距离，当前彩虹ESDF色带也不参与真实Mesh颜色融合。

源码定位：

- [后端与静态地图选择](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/launch/perception/nvblox.launch.py:58)
- [诊断节点调用官方融合](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:36)
- [官方深度处理](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:966)
- [官方颜色处理](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:1176)

## 3. 表面为什么会模糊、变厚或出现假连接

### 3.1 体素3cm，10cm厚墙只有约3.3个体素

当前realtime的voxel_size=0.03m，场景两面墙的厚度均为0.1m，厚度方向只有约3.3格。棱角、窄面和遮挡边界在这样的网格上很难保持锐利。Mesh是从离散TSDF零交叉插值提取，不是直接把640×480深度像素当成高分辨率表面。

相机内参fx约320.25，举例深度0.8m处一个像素约对应2.5mm，而一个3cm体素覆盖约12个像素的线性尺度。这个例子说明提高输入消息频率或维持640×480，并不能消除地图体素造成的细节限制；实际不同表面的相机深度不同，应按具体深度计算。

参考档继承workspaces的2cm体素，当前3cm在每个方向粗50%。因此即使调用相同官方核心，也不应默认二者具有相同棱角和颜色细节。

### 3.2 TSDF截断距离12cm，相对薄墙偏大

配置为projective_integrator_truncation_distance_vox=4，乘3cm得到12cm。它是射线方向有符号距离的截断/更新尺度，不是“把所有表面直接做12cm高斯模糊”。不过在薄物体、遮挡边缘、多视角不一致或错误深度条件下，较宽的更新范围会让互相矛盾的观测影响同一带内的体素，造成零交叉位置不稳、圆角或局部桥接。

必须注意：两墙之间净间距约40cm，不能仅凭12cm截断距离就解释整个40cm空隙被错误填满。若出现这种情况，还要检查地面是否本来存在、机械臂残留、边缘深度或位姿错误。两墙厚度10cm与墙间40cm是不同问题。

参数依据：[realtime体素与历史处理](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/profiles/realtime.yaml:6)、[截断距离与Mesh门限](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/nvblox/nvblox_base.yaml:45)、[场景几何](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/config/collision_world_gazebo_tall.yml)。

### 3.3 活动机械臂被写入静态地图，是重点问题

现有RGB截图和Mesh里都能看到机器人自体。当前运行时use_segmentation=false，入口没有机器人深度/RGB自体遮罩；地图使用static_tsdf，不按物体身份区分机器人、墙和地面。

相机左右移动时，机械臂的关节、连杆也在世界中运动。同一位置可能先看到连杆、后看到墙面或地面。这些互相矛盾的测量被送入同一张静态TSDF；对应颜色也会累积。它可以形成自体残影、假表面和颜色混杂，不需要RGB/深度外参出错也会发生。

realtime关闭TSDF衰减和半径清图；invalid_depth_decay_factor=-1也表示这条无效深度衰减途径未启用。旧误观测不一定快速消失。但这不等于TSDF永远不更新：后续有效深度仍可能沿射线修正或清除部分旧几何；遮挡或未重访位置则可能持续保留。

不能因为“任务/人物识别属于冗余功能”就把机器人自体几何过滤也当作冗余。自体过滤可以基于URDF、关节状态和相机投影实现，无需人物检测网络或实例识别。

### 3.4 低权重边缘和不完整观测仍可进入网格

当前static_mapper.mesh_integrator_min_weight=0.1，do_depth_preprocessing=false。边缘、少量观测或不稳定深度更容易留下可见片段；关闭预处理并不自动构成错误，但缺少专门的深度边缘/自体质量控制。

不能笼统认为“提高min_weight一定更好”：它可能减少碎片，也可能进一步制造缺口；过强深度滤波同样可能把不同表面跨边界平滑。本次不改参数，只说明取舍。

## 4. 颜色为什么会错位和扩散

### 4.1 可见性由已重建TSDF推断，几何错会传递成颜色错

官方ProjectiveColorIntegrator先从彩色相机视角对TSDF进行sphere tracing，生成合成深度，然后对投影体素进行遮挡测试。这里没有拿“一张严格同步的真实深度图”直接逐像素约束每张RGB。

底层颜色更新内核：

1. 将体素投影到彩色图像。
2. 双线性采样合成深度。
3. 当体素深度与合成表面距离不超过截断带时允许着色。
4. 双线性采样RGB。
5. 与历史ColorVoxel做加权融合。

如果TSDF中已有假表面、缺面或机械臂残影，射线命中的表面可能就不正确，后续颜色即使来自正确RGB，也可能被写到错误体素。它是“几何错误 → 可见性/投影错误 → 颜色污染”的放大链。

代码依据：

- [从TSDF生成合成深度与颜色融合](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_color_integrator.cu:98)
- [颜色遮挡门限及双线性采样](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh:173)

### 4.2 低分辨率合成深度和边缘插值有串色风险

颜色积分器sphere_tracing_ray_subsampling_factor默认4，合成深度相对彩色图像降采样；内核对其双线性插值。这个参数与base.yaml中的view-calculator raycast_subsampling_factor不是同一个成员，不能只改后者就认为颜色射线精度已提高。

在前景/背景深度突变处，低分辨率合成深度、双线性深度和RGB插值，加上较宽的表面门限，会增加跨边缘串色风险。代码没有物体标签边界约束；是否造成当前具体位置的错误仍需后续针对边缘对照，不能凭代码存在插值就认定全部颜色错误由它造成。

还有一个后续修改时容易忽略的实现细节：本地Mapper::setMapperParams把截断参数传给TSDF等积分器，但该段没有同步调用color_integrator().truncation_distance_vox(...)。颜色积分器当前继承默认4体素，因此目前TSDF与颜色都是约12cm；以后只调TSDF截断参数，颜色遮挡带未必一起变窄，需要明确核对两个积分器，不能只看YAML名称。

源码：[颜色射线默认降采样](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/integrators/projective_color_integrator.h:143)、[Mapper参数传递](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mapper/mapper.cpp:121)。

### 4.3 ColorVoxel存平均颜色，Mesh不是原始RGB高清贴图

blendTwoColors将新旧颜色按权重平均。若相同体素在不同帧分别采到灰地面、紫色墙或机器人颜色，结果会是它们的混合。max_weight=5限制历史权重，仍会更新，并不是权重饱和后冻结；但错误颜色也不会自动被判为另一个物体而隔离。

颜色体素空间分辨率同样受3cm网格限制。GPU Mesh着色采用当前网格块中的近邻颜色体素，块边界索引会被夹到本块范围；这是官方代码明确采用的性能取舍，边界处偶有误配风险，不是精细纹理坐标贴图。颜色块不存在时还有默认颜色路径。因此大块平均色、灰色区域或局部块状边界，不一定是RGB未输入，也可能是体素平均、缺少着色或取色策略。

源码：[颜色加权平均](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_color_integrator.cu:206)、[Mesh近邻颜色体素](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mesh/mesh_integrator_color.cu:54)。

## 5. 外参、时间和编码：哪些尚不能认定出错

### 5.1 仿真RGB/深度固定安装关系初步相符

Gazebo中深度传感器位于camera_link，彩色传感器相对camera_link位移为(0,0.015,0)m。手眼安装链中，color_optical→bottom_screw的位移(0.0325,0.0125,−0.0106)m，结合本地D435宏bottom_screw→camera_link的(0.0106,0.0175,0.0125)m及对应旋转，复合得到color_optical→camera_link位移(0.015,0,0)m；反向转回普通相机坐标后，正对应camera_link→color_frame的(0,0.015,0)m。深度光学旋转也按相同约定设置。

因此，静态代码没有显示“Gazebo的15mm基线与TF完全不符”的直接证据。此前核对两路CameraInfo尺寸/内参一致，也不能单独证明所有外参正确，但应避免未经证实地把当前问题直接归因于某个手眼偏移。

文件：[Gazebo传感器位置](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro:108)、[手眼及光学链](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/urdf/d435_handeye_mount.urdf.xacro:201)、[实际引用的D435宏](/home/czz/workspaces/isaac_ros-dev/src/realsense-ros/realsense2_description/urdf/_d435.urdf.xacro:58)。

### 5.2 两路异步不等于必然错误，时间戳准确性仍需后续核验

官方节点对深度和颜色分别排队，各自同步对应CameraInfo，并按各自图像时间戳查TF；没有强制RGB和深度一一配成同一时间的帧。这是官方支持的异步着色方式，静态场景、准确位姿和准确时间戳条件下可以工作。不能说“不做RGB-D强同步所以一定串色”。

但是当前Gazebo ROS插件在颜色和深度回调中都使用world->SimTime()填header，而不是显式使用传感器曝光/渲染帧时刻；两传感器也独立触发。它存在需要核验的帧内容时间与标记时间差风险。实际偏差大小及其是否造成当前错位，本轮未采样，不能下定论。

此外，D435实机驱动配置中的enable_sync=true不控制Gazebo传感器；本轮实际启动的是Gazebo场景，不能用那份实机驱动YAML证明仿真已经同步。

现有约51ms点云输入年龄不能直接等同于51ms位姿错配。只要图像时间戳与TF正确，处理延迟不必然使融合几何偏移；需要检查的是时间对应关系，而不是看到延迟就认定位姿用了最新值。

源码：[仿真颜色时间戳](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp:88)、[仿真深度时间戳](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp:234)。

### 5.3 没有发现直接RGB/BGR颠倒证据

插件把RGB_INT8标为rgb8，官方Nitros转换针对rgb8按Rgb读取；已有运行记录也是rgb8。当前链路没有直接支持“红蓝通道统一颠倒”这一结论。个别表面的异色更应先检查几何、遮挡、空间取样和时序。

URDF里camera/noise配置stddev=0.02，但仅凭此处配置不能断言实际深度噪声标准差为2cm；需进一步确认Gazebo版本将该噪声应用于哪个输出，并检查静态原始深度。本轮不做这项测试。

## 6. 哪些“融合在一起”可能本来就是正常输出

- 地面与墙体实际接触/相交，Mesh连通属于合理几何结果。场景墙底约−0.1m，地面顶为0m，不能要求它们在几何上自动分成独立实例。
- 静态TSDF没有“这是A物体、那是B物体”的身份信息。若需要分别导出各物体，属于后续实例/几何分割；不是换一个ESDF配色能解决。
- 如果颜色从墙扩散到不应同色的地面、出现悬空膜片、机器人离开后原地仍留表面，则更符合颜色污染、假几何或动态残留，需要区别于正常连通。
- ESDF彩虹点色表示距离，不是物体RGB；修改它的Min/Max Intensity不修正Mesh里的真实颜色。判断原始颜色应关闭ESDF叠加单看RGB/Mesh。

## 7. 现有数据能支持什么

使用已经完成的高位左右扫描结果：

| 指标 | 高位静止 | 左右扫描后 |
| --- | ---: | ---: |
| observed ESDF点 | 24184 | 33352 |
| Mesh三角形 | 4306 | 6156 |
| 墙体邻域顶点误差中位数 | 2.29mm | 3.73mm |
| 墙体邻域顶点误差P95 | 35.00mm | 49.94mm |

数据支持“观测/网格规模扩大，但局部误差尾部变大”，不能支持“点数越多重建越准确”。新增顶点来自不同区域，因此也不能单凭P95变化证明每个原有表面都恶化。没有同区域几何/颜色逐点真值误差，不能给各原因分配百分比。

已有真实截图，未重新运行：

![高位左右扫描已有Mesh截图](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/raised_lateral_scan_20261105/mesh_after.png)

[该轮原始结果及截图](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/RAISED_LATERAL_RECONSTRUCTION_TEST_20261105.md)。

## 8. 改善优先级：先几何正确，再颜色，再精细度

| 优先级 | 方向 | 解决的问题 | 注意事项 |
| --- | --- | --- | --- |
| P0 | 基于URDF/关节状态建立深度与颜色一致的机器人自体过滤 | 活动物体污染静态TSDF、颜色残影 | 需要遮挡一致性，不能只在RViz隐藏机器人；无需人物识别网络 |
| P0 | 核验渲染/图像时间戳、图像时刻TF和RGB/深度真实投影 | 错位、运动时表面分裂与跨表面涂色 | 当前静态外参初步相符，先用证据定位，不盲目改手眼参数 |
| P1 | 将TSDF与颜色的表面带分别核对、合理收紧，增加深度突变边缘约束 | 薄墙、棱角混合与颜色跨边缘 | 本地颜色截断参数传递需注意；过窄会减少有效融合/颜色覆盖 |
| P1 | 核对颜色sphere-tracing降采样，改进遮挡与颜色置信度筛选 | 边界颜色泄漏 | 不等同于简单把颜色Hz调高；提高采样精度会增加GPU开销 |
| P1 | 对运动自体和旧误几何设计针对性清理/衰减 | 长尾残影与错误历史 | 全图统一强衰减可能删除真实静态障碍物，自体过滤应优先 |
| P2 | 在上述问题明确后比较2cm/更细体素、Mesh置信门限、边缘保留处理 | 表面颗粒、圆角与纹理模糊 | 需要相同输入/范围对照，细化不能补救错误位姿或动态污染 |
| P2 | 若目标是高清视觉模型，单独考虑纹理映射/更高精度表面表示 | ColorVoxel平均色与低频顶点色的限制 | nvblox导航/距离地图的彩色Mesh不等同于摄影测量高清纹理模型 |

后续若获准测试，先做同一轨迹的静态逐点采集/连续运动、自体过滤开关、几何与颜色分离检查，再调整体素和截断距离。当前只完成原因分析，不实施这些修改或对照测试。
