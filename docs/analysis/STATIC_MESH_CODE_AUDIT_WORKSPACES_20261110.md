# 静止状态下 RGB/Depth 与 Mesh 不一致：代码链路审查及 workspaces 对照

日期：2026-11-10。分析对象：`ur7e_nvblox_in_hand`、`nvblox_in_hand`，以及 `/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox` 中的主体实现和启动组织。仅新增分析文档与既有Mesh渲染图副本，没有修改源码、配置或运行参数，没有在本轮启动新测试。

## 1. 对问题的重新界定

用户确认：机械臂与相机静止，真实 RGB/Depth 可辨认目标；仿真也出现类似重建异常。最新截图中，灯板和背景结构在输入图像中明显，但 Mesh 大幅简化、缺失并出现错误连接。因此，不能继续把物理相机质量或运动误差当成已经确定的主要原因。

需要调查的是：图像进入程序后，是否使用正确的相机模型与坐标关系反投影；TSDF 是否保留对应结构；Mesh 是否正确表达已建立的几何；颜色投影是否覆盖对应表面。

用户最新RGB/Depth/Mesh对照图来自本次会话附件；附件所给临时文件路径当前不可读，本文不伪造该截图副本。下图为此前实际PLY数据的独立渲染，不能当作最新截图同一时刻的同步数据。

![此前导出Mesh的原始颜色多视角渲染](/home/czz/perception_eye_in_hand/docs/analysis/assets/saved_mesh_views_20261109.png)

图像能证明输入具有可辨认结构，但灰度深度窗口不是数值真值：它不能单独验证单位、有效值比例、K、图像行跨度、与RGB的光心关系。以下将“输入结构清楚”作为调查前提，不把尚未检查的数值问题写成相机故障。

另一个必要条件：当前RViz是基座下的俯视，RGB是相机自身视角。轮廓一致性必须在同一相机视角下测量；然而视角不同不能解释所有几何孔洞，且用户已在导出PLY中观察到异常。

## 2. 先纠正前面分析的边界

1. 修正话题前缀、RViz重映射和相机启动参数，只证明实机接入被修正，不证明重建算法质量被修复。
2. 颜色约30Hz是接口融合调用成功计数，不是颜色覆盖率，更不是几何精度。
3. 固定手眼变换错误在完全静止时，通常造成整个几何地图的刚体平移/旋转，不能单独解释内部大面积孔洞。相对color/depth外参错误仍可使颜色错位，但不等价于深度几何变形。
4. 静止不是“所有输入都恒定”：代码仍可能发生相机模型变化、错误消息组合、时钟重置或位姿反馈抖动。这些是应核查的软件状态，不是已证实的现场事实。
5. 3cm网格不适合毫米级灯珠、孔洞，但不能用这一点替所有明显错误连接开脱。需要先完成同视角的数值一致性检查。

## 3. workspaces 的实际代码组织

```text
nvblox_examples/nvblox_examples_bringup
 ├─ launch/realsense_example.launch.py             应用启动编排
 ├─ launch/sensors/realsense.launch.py             驱动、投射器配置、splitter
 ├─ launch/perception/vslam.launch.py              官方示例的位姿来源
 ├─ launch/perception/nvblox.launch.py             选择模式、配置层次、输入重映射
 ├─ launch/visualization/...                      显示
 └─ config/nvblox/{nvblox_base,specializations/...}.yaml
       ↓
nvblox_ros/src/lib/nvblox_node.cpp                配对、队列、TF、转换、调度
nvblox_ros/src/lib/conversions/image_conversions.cpp
nvblox_ros/src/lib/transformer.cpp
       ↓
nvblox_core/nvblox/src/mapper/mapper.cpp          保留地图、变化块跟踪
 ├─ integrators/projective_tsdf_integrator.cu      深度投影融合
 ├─ integrators/projective_color_integrator.cu     基于TSDF可见性的颜色融合
 ├─ mesh/mesh_integrator.cu                       网格提取
 └─ integrators/esdf_integrator.cu                距离传播
```

必须区分：examples 是应用入口，不是 Mesh 算法本身。替换VSLAM为机械臂位姿后，仍应满足同样的图像、内参、外参和时间接口条件。

### 3.1 workspaces 也不是完全未经修改的官方基准

本地 `gazebo_realsense_example.launch.py`、`sensors/gazebo_realsense.launch.py` 含固定相机、跳过splitter、静态TF别名等本地适配。后者还有两项明确的移植风险：

- 将 `/camera/depth/camera_info` 转发到 `/camera0/camera/color/camera_info`。它可以出于历史几何兼容而存在，但不能一般化地用于真实独立RGB相机的颜色融合；焦距、主点、frame与光心都可能不同。
- 发布 `camera_link → base_link` 单位变换别名。这是固定相机示例的特殊假设，不能迁移为UR机械臂的基座关系，否则可能与机器人TF产生冲突。

本地 `nvblox_base.yaml` 还包含 use_color=false、v=0.02m，以及针对本地场景的参数修改。因此“workspaces正常”必须附带当时的实际launch、最终参数和输出，不能把整个目录视作一组统一、未经改动的官方配置。

当前两套新包的真实话题命令没有使用上述Gazebo CameraInfo转发链，不能把这两个风险直接写成当前实机故障根因。

## 4. 逐段代码审查

### 4.1 深度图解析与单位

Python `backend.py:76–84` 使用编码、is_bigendian、step构造二维数组，16UC1乘0.001，32FC1保持米值，剔除非有限及超出量程的深度，再传给add_depth_frame。

本轮没有发现行列反转、忽略step或反复乘1000的直接证据。仿真插件的RealSensePlugin.cpp确实将米深度按DEPTH_SCALE_M转成16位深度；接口应再核对该常量与实际消息。固定16UC1=毫米是输入合同，不是任意自定义16位深度都适用。

这段判断应通过已知深度的平面数据验证。仅看灰度图无法证明单位解释正确，但也不能据此宣称单位已经错了。

### 4.2 相机模型：确实存在代码合同不足

Python `backend.py:69–74` 注释提到真实校正输入可能使用P，实际代码始终只读取K，并调用无畸变的Sensor.from_camera；没有根据图像是否经过校正、裁剪或缩放选择与之匹配的模型，也没有验证P/D/ROI。

这是明确的实现边界和注释与实现不一致，但不代表所有K-only使用都错误：在K就是输入针孔模型、D为零、无额外图像变换时可以正确工作。本轮RGB CameraInfo的D全零、K/P焦距主点相同，因此没有证据把该限制直接判为这次RGB异常根因。深度及历史仿真每次实际模型仍需各自核对。

本地workspaces官方 `cameraFromMessage()` 同样只读取K。说明盲目复制官方转换函数不能自动解决这一类输入合同问题；官方应用必须提供已经符合约定的图像。

Python `camera_info()` 仅检查fx/fy>0，缓存最新CameraInfo；candidate检查尺寸/frame，但未验证内参有限性、模型与该图像的配置代际一致。若分辨率不变而相机模型在运行中切换，缓存策略缺少可追溯性。相比之下，本地官方ROS使用Image与CameraInfo的ExactTime配对。

### 4.3 位姿方向：没有发现直接取反错误

Python用lookup_transform(global_frame, image.header.frame_id, image_stamp)，构造相机到全局的T_L_C，传给wheel。已安装wheel接口约定也是sensor到world。workspaces的Transformer使用同方向TF，并在底层需要时取逆投影。

所以目前没有依据改成inverse(pose)或转置矩阵。这样的盲改可能直接破坏几何。要排除二进制桥的矩阵布局问题，应做已知平移/旋转的合成平面接口测试；本轮仅审查代码，没有声称已完成该测试。

### 4.4 颜色数据解析与投影：调用次数不足以评价效果

Python支持rgb8/bgr8，按step读取，bgr8才交换通道；转连续uint8后调用add_color_frame。Mesh顶点色除255的做法与wheel自带Mesh.to_open3d一致，没有找到明显归一化重复错误。

官方color_integrator不是简单贴RGB：它选取已有TSDF截断带内的块，从彩色相机视角对TSDF执行sphere tracing生成合成深度，然后只为满足可见性条件的体素融合颜色。如果TSDF存在孔洞、彩色相机模型/外参不符，可能调用正常但大量体素没有被着色。

Python和C++诊断中的color_integrated都没有直接统计“本帧实际增加颜色权重的体素数量”。因此即使达到30Hz，也可能有效着色很少。这是已确认的监测语义不足，不是已经找到某个具体CUDA算子错误。

上轮PLY有150种RGB，但约56.5%顶点恰为(127,127,127)。默认灰色是异常排查线索，真实灰色也可能相同；不能把灰值比例精确当作未着色率。

### 4.5 TSDF及Mesh提取条件可以解释“静止仍缺失”的机制

TSDF是离散网格，不是RGB纹理重建。当前v=0.03、截断4voxel、Wmax=5、mesh_min_weight=0.1。workspaces本地基准v=0.02，其余主要权重设置相近；两者并非同分辨率比较。

官方mesh_integrator.cu在Marching Cubes单元角点或邻块缺失、角点权重低于门限时，拒绝生成该单元三角面。因此只要少数关键体素未被正确更新，输入深度中能看见一片表面，也未必能得到连续Mesh。粗网格附近的薄结构、遮挡边界和缺乏跨边界观测会扩大这种影响。

这解释了可能机制，不等于当前所有孔洞都是正常现象。应统计拒绝单元的原因、权重、有效TSDF角点与深度支持率。只重复融合同一静止视角，也不能生成从未获得有效观测的几何。

此外，3cm下4voxel=12cm是截断尺度，不是固定平滑半径，不应把它写成必然产生12cm误差。Wmax限制累计权重也不会停止后续融合。

### 4.6 Mesh消息与PLY路径：没有证实索引打乱

Python backend.mesh()获取vertices/triangles/vertex_colors并逐项CPU复制；messages.py按三角索引展开点与颜色。单位scale、identity pose，没有再次施加相机TF。CDR快速打包已有与标准ROS序列化的等价测试。

save_ply经wheel自带get_color_mesh().save()，绕过项目MarkerArray编码。用户在独立渲染PLY中仍看到异常，因此自定义CDR编码不能单独解释全部现象。但等价序列化测试仅证明传输一致，不证明Mesh几何准确。

C++官方增量Mesh插件与Python完整MarkerArray不同，可能出现显示差异；两者几何数据都差时，应优先追溯共同输入模型/TSDF条件，而不是只切换渲染器。

### 4.7 确认的额外代码缺陷：实机仍响应外部/clock回退

Python node初始化无条件订阅/clock；clock()发现回退超过1ms时会清输入、重置last_seen并提交clear_map，但没有检查use_sim_time是否启用。

这意味着真实时间模式也可能被同ROS域内无关仿真/回放的时钟回退清图。它是明确的模式隔离缺陷，未来应修正；本轮已有采样没有clock_resets证据，不能宣称本次Mesh孔洞就是它造成的。

### 4.8 诊断与测试覆盖不足是最明确的项目级问题

当前测试覆盖了消息格式、话题选择、基础几何等，但没有提供“已知平面/台阶深度→TSDF→Mesh→同相机投影误差”的端到端几何合同。TF查询失败也大多静默等待，缺少按原因、持续时间和每路输入区分的质量状态。

因此当前程序容易出现“输入Hz、融合Hz均正常，但几何质量没有被检查”的假阳性。这需要几何和颜色支持率诊断，不能再通过增加Hz来宣布修复。

## 5. 仿真代码的具体核查

- ur7e_d435.urdf.xacro中depth传感器位于camera_link，color传感器pose为(0,0.015,0)，两个FOV均为1.57，分辨率来自同一camera_width/height参数。
- 对d435_handeye_mount与_d435宏的固定变换进行了矩阵组合：camera_link→color_optical的平移为(0,0.015,0)，旋转与标准optical轴变换一致，和传感器定义在数值精度内相符。不能声称这里已发现“15mm偏移方向写反”。
- 仿真插件用各相机HFOV生成CameraInfo，fx=fy=0.5*width/tan(HFOV/2)，主点width/2,height/2。插件自身点云使用(width-1)/2、(height-1)/2，存在半像素约定差异；它可造成小投影差，但通常不足以解释大面积几何崩坏。
- xacro显式配置了gaussian噪声、stddev=0.02；其是否及如何作用于深度应依据Gazebo实现核实，不能把这个仿真先验认作完全无噪声真值。
- near=0.1而插件rangeMinDepth=0.2，插件先屏蔽0.2m以内深度；Mapper的min_depth=0.1无法恢复被上游置零的观测。这是明确的量程合同差异，仅当目标在该范围内时影响当前结果。
- 插件还可以用同像素位置给自身点云着色，未按15mm基线做完整RGB投影；这个点云的颜色不能作为严格对齐真值。但当前Mapper订阅Image而非该点云，不能把它误说成Mapper正在使用的颜色算法。

仿真和实机共用相机模型解释、TSDF参数与Mesh输出路径，因而二者都出现问题时，软件层调查应优先。上述仿真差异不能直接外推成实机故障原因。

## 6. 问题分级

| 级别 | 发现 | 本次因果判断 |
|---|---|---|
| 已确认代码缺陷 | 实机模式仍会因/clock回退触发清图 | 未见本轮触发证据 |
| 已确认实现边界 | K-only针孔模型；注释提到P但没有实现；内参验证不完整 | 当前RGB K/P一致，尚不能锁定其致因 |
| 已确认监测不足 | “融合成功”计数不等于实际更新体素/颜色覆盖 | 此前用30Hz证明整体质量是不充分的 |
| 已确认基准风险 | workspaces有本地Gazebo适配，depth_info转color_info及camera_link→base_link别名 | 不可原样搬到当前机械臂实机 |
| 已确认条件差异 | workspaces基准2cm，当前3cm；仿真上下游近距离阈值不同 | 需同条件核对，不能默认等价 |
| 待定位实际根因 | TSDF覆盖/权重、颜色可见性、图像模型、实际运行TF/参数与输入对应 | 尚未得到一条足以解释全部异常的证据链 |

## 7. 后续建议：先定位，后修改

1. 录制一段静止且新建地图的RGB、Depth、各自CameraInfo和TF；记录所有节点包路径及最终参数。排除重复Mapper、错误配置和旧地图遗留。
2. 在depth optical坐标系直接按实际模型反投影；同时从同一相机视角渲染Mesh深度，比较有效区域覆盖率与深度残差。不要只比较基座俯视与RGB。
3. 用identity相机位姿输入已知平面/台阶合成深度，经当前后端导出Mesh；再用已知刚体变换重复。它可以分离图像解析、矩阵布局、网格提取与硬件TF。
4. 导出相同区域TSDF距离、权重与observed掩码：原始深度有值而TSDF未更新，查投影/块选择；TSDF有效而Mesh缺失，查单元角点/门限/提取；Mesh几何正确而颜色差，查sphere tracing及颜色权重。
5. 严格使用相同输入、2cm/3cm、截断米值/体素值、量程、权重和输出负载，对比workspaces官方核心路径和Python接口。不能将旧版不同输入的“好看”作为定量结论。
6. 根因明确后再设计修改：相机模型合同、时钟隔离、有效体素统计、已知几何端到端测试；性能优化放在几何正确性之后。

以上均是后续方案，本轮没有执行新的采集或修改。

## 8. 关键代码定位

项目根目录 `/home/czz/perception_eye_in_hand`：

- [Python相机模型与图像解析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:69)
- [Python时钟、内参缓存与图像时刻位姿](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:126)
- [Python Mesh打包](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/messages.py:35)
- [仿真传感器定义](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro:55)
- [仿真相机插件](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp:296)
- [workspaces应用输入组织](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/perception/nvblox.launch.py)
- [workspaces本地Gazebo适配风险](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/launch/sensors/gazebo_realsense.launch.py:101)
- [官方颜色积分器](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_color_integrator.cu:45)
- [官方Mesh角点有效性条件](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mesh/mesh_integrator.cu:250)

官方原理参考：[nvblox论文](https://arxiv.org/html/2311.00626v2)、[Isaac ROS技术说明](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)。本报告的具体代码判断以读取的本地版本为准。

最终判断：现有证据足以要求优先审查代码及其数据合同，不足以把整体现象归咎于相机；也尚不足以宣布某个矩阵或算子就是唯一根因。软件质量检查缺失、模型合同边界和本地基准差异已明确，下一步应建立同输入、同视角、逐层的数值验证链。
