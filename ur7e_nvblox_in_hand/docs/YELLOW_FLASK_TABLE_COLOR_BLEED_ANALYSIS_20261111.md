# 黄色瓶体的颜色为什么落到工作台：截图、保存Mesh与官方实现分析

日期：2026-11-11。对象：`ur7e_nvblox_in_hand`当前5mm仿真重建。本文只分析现有图片、保存数据、项目代码及官方资料；没有修改感知源代码、参数或场景，没有执行新的机械臂运动。

## 1. 对这两张图的判断

右侧主要立体结构对应黄色瓶体；左侧靠近台面的黄色长条/瓶形区域，应优先作为**颜色跨深度边界污染**检查，不能当作正常瓶体结构，也不能把全部异常笼统解释成“体素太大”或“融合次数太多”。

二维截图不能确定黄色区域的真实高度。为区分“瓶子被重建成两个实体”和“工作台被染成瓶子的颜色”，本轮读取了上一轮已保存的真实PLY检查三维坐标，结果支持后者确实存在。该PLY来自同一场景此前完成的扫描，不是与本次两张截图严格同一时刻的快照，以下数字用于确认同类现象，不能当作截图逐像素测量。

![整体截图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/user_full.png)

![用户指出的黄色瓶体局部](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/user_detail.png)

### 1.1 场景中真正的黄色瓶子是什么

[lab_scene.py:107](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/scripts/lab_scene.py:107)生成黄色瓶体：

- 中心位置为x=0.59m、y=−0.39m。
- 底部半径45mm、高25mm。
- 瓶身为6段逐渐缩小的圆柱，每段高18mm。
- 瓶颈半径18mm、高45mm，顶部高度178mm。
- 整体黄色，工作台几何顶面z=0，原本为灰色材料。

因此，右侧瓶身的**部分规则阶梯**来自仿真真值本身，不全是重建错误。但场景没有在旁边再放一个黄色瓶子，也没有把该处桌面设置成黄色。

### 1.2 保存数据中发现了什么

分析来源：[static_after/mesh.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/live_5mm_scan_20261111/static_after/mesh.ply)。

采用瓶体中心±12cm局部ROI，以R>0.4、G>0.3、B<0.5且G>1.5B筛选黄色候选。它是诊断规则，不是语义分割，也不代表所有黄色像素都准确归属于瓶子。

| 检查项 | 结果 | 含义 |
|---|---:|---|
| 局部黄色候选顶点 | 2,309 | 包含正常瓶体及疑似污染 |
| 半径>50mm且接近台面的黄色顶点 | 173 | 已超出瓶体最大45mm底部半径 |
| 上述候选高度 | −7.5…+7.5mm | 属于接近台面的表面，不是178mm高的瓶颈 |
| 上述候选离瓶中心的最大水平距离 | 149.54mm | 不能只用一个5mm体素的边界误差解释 |
| 三顶点均满足上述条件的三角面 | 203，面积约25.34cm² | 仅是该筛选规则下的面积，不是全部污染面积 |
| 距真实黄色瓶实体>20mm的黄色顶点 | 114 | 部分候选远离真实瓶体 |
| 同时距实体>20mm且z>20mm的黄色顶点 | 0 | 本份数据未在这一规则下检出明显抬高的黄色复制体；不排除其他颜色或未采样位置的多余几何 |

![实际PLY几何与颜色诊断](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/flask_geometry_color_diagnosis.png)

左图为实际Mesh顶点颜色与真实最大底部轮廓，中央为侧向投影及真实瓶体分段轮廓，右图红点为瓶体外围的低位黄色候选。绘图显示真实三维坐标，未改变原始Mesh。该图含局部ROI内其他物体投影，不应把每个点都看作瓶体。

这证明**至少一部分异常颜色已写入保存的Mesh数据，不能靠更换RViz或CloudCompare渲染器解决**。它也提示：从斜视图看上去像另一只瓶颈的色块，可能实际上是台面上的颜色投影。是否另有几何复制，仍须用灰色Mesh和截面单独检查。

## 2. 当前代码中颜色怎样进入地图

[backend.py:76](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:76)负责深度，[backend.py:86](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:86)负责颜色：

```text
深度图 + Depth CameraInfo + 图像时刻Depth optical pose
    → add_depth_frame → TSDF几何

RGB图 + Color CameraInfo + 图像时刻Color optical pose
    → add_color_frame → 颜色体素

TSDF提取三角网格 + 颜色层取色
    → 带顶点色的Mesh → PLY或ROS MarkerArray
```

颜色接口目前接收RGB、位姿和相机模型，**未传入实测Depth**。两路输入独立排队、按各自图像时间戳查TF；不是在同一对RGBD图像上先完成逐像素可见性验证再融合。[node.py:154](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:154)

这并不意味着接口天然用错。官方支持独立颜色观测，但其有效性依赖几何、投影和遮挡关系一致。

## 3. 关键原因：遮挡判断使用的是哪一张深度

固定版本[projective_appearance_integrator.h:56](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/projective_appearance_integrator.h:56)说明：有对齐深度时使用它做遮挡检查；否则从TSDF通过sphere tracing生成合成深度。当前Python入口走后者。

随后[颜色CUDA模板:150](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh:150)先判断体素与该表面的深度差，再从RGB图像双线性采样：

```cpp
voxel_distance_from_surface = surface_depth_m - voxel_depth_m;
if (fabsf(voxel_distance_from_surface) > op->truncation_distance_m_) return;
// 通过后，投影到RGB并线性插值，更新该体素外观。
```

这是一种**几何可见性门限**，没有物体身份识别。它不会直接知道“黄色属于瓶子，不能属于台面”。

可能的错误链路是：

```text
RGB在某像素看到黄色瓶子
    ↓
TSDF合成深度在相应射线上却命中台面/错误表面
    ↓
台面体素与合成深度很接近，遮挡检查通过
    ↓
该体素取得同一RGB像素的黄色
    ↓
台面几何仍在低处，但其Mesh显示为黄色
```

举一个仅解释机制的数值例子：实际瓶体在相机轴向0.65m，台面在0.82m；如果合成深度错误地返回0.82m，台面体素与合成表面的差约0，而不是0.17m，因此即使容差只有0.02m也可能通过。这里的数值不是本次已测到的单帧内核记录；本轮没有捕获颜色更新瞬间的合成深度，故不能把这一链路当作已完全定位的唯一根因。

另一个必须检查的方向是RGB相机实际投影、CameraInfo与TF是否一致。如果RGB投影错位，使用的深度即使在自身坐标下正确，也可能被配到不对应的颜色射线。静止能排除运动时间误差的很多影响，但不能排除固定投影/外参错误。

## 4. 为什么不能只怪4体素截断或融合次数

当前voxel_size=0.005m，4体素为20mm。该尺度会影响表面附近颜色接纳范围，可能放大边缘污染，但**20mm门限不能单独解释正确投影和正确遮挡条件下，178mm高的瓶颈颜色落到台面**。

同样，149.54mm的水平范围不能直接与20mm相减来推导深度错误：前者是世界水平距离，后者是相机深度方向门限。它只能提示现象超过一个体素的局部模糊，必须检查投影和可见性。

需要分清：

| 因素 | 可能作用 | 不能据此推出什么 |
|---|---|---|
| 颜色接纳带较宽 | 接纳表面附近错误位置 | 不能单独证明跨很大深度的串色必然发生 |
| 合成深度边缘或射线命中错误 | 错误表面通过遮挡检查 | 当前尚未抓取到对应像素的内部合成深度证据 |
| RGB/Depth内参外参不一致 | 系统性投影错位，静止也会出现 | 不能仅因有15mm基线就认定外参错误 |
| 颜色射线低分辨率采样 | 细物体/遮挡边界可能受影响 | sphere-tracing抽样、分配射线抽样与ESDF stride不是同一参数 |
| 外观更新权重较大 | 新的错误颜色容易进入并改变历史颜色 | 降低权重不会修复投影位置 |
| 衰减和清图关闭 | 已污染的颜色可继续保留 | 开清图并不保证正确重建，还可能丢失需要保留的地图 |
| Mesh体素取色和三角形插值 | 局部颜色扩散、视觉边界增宽 | 颜色操作不会单独生成第二个TSDF几何瓶体 |
| 增量融合 | 在既有状态上更新受影响区域 | 不包含自动的语义纠错或全局纹理重优化保证 |

固定版本外观参数采用指数更新语义，默认α=0.8，项目未显式覆盖；其参数含义来自[projective_integrator_params.h:65](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/projective_integrator_params.h:65)。如果观测本身错位，继续融合可能更新或保留错误，不能指望帧数增加自动恢复材质边界。

## 5. 仿真相机配置核对与局限

源码中Color传感器相对camera_link的y偏移为+15mm；Depth在camera_link原点。[ur7e_d435.urdf.xacro:116](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro:116)

将当前安装结构中的固定变换组合后，理论Color optical原点与Color传感器相符，Depth optical相对Color optical为+x方向15mm。**静态源码中未发现简单的“把深度当彩色”这一基线符号错误**。这不是对实际运行TF、传感器插件输出和CameraInfo的一次完整投影验收。

本轮还作了辅助投影诊断：使用保存Depth位姿、源码15mm基线、相同仿真K近似Color位姿，将173个低位黄色候选投回保存RGB，其中166个落在黄色候选像素；场景真值射线检查有58个被前景挡住超过2cm，其中46个首先命中黄色瓶体。

该结果提示“后方表面沿RGB射线取得前景颜色”的机制值得优先检查，但**它不是颜色融合瞬间的回放证据**。RGB/Depth快照非严格同时，Color位姿与K为源码推导，没有单独保存该时刻Color CameraInfo和TF，因此不能由此断言真实运行外参错误，或所有173个点均由一次特定错误观测造成。后续应以同一帧的真实Color TF、K和合成深度验证。

## 6. 官方文档是否说会出现这种问题

需要区分“官方实现允许这种误差机制”和“官方已确认本项目这张图的特定缺陷”。

1. 官方[核心接口说明](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/core_library_interface.html)要求调用方提供图像、相机模型和位姿，并说明Mesh/ESDF基于地图增量更新。它没有承诺物体材质边界不会跨表面污染，也没有提供瓶子身份约束。
2. 官方固定提交[颜色融合CUDA代码](https://github.com/nvidia-isaac/nvblox/blob/e4a2b00f45ecf989682d72a33a2c29defe39fd2d/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh)明确使用表面深度门限和RGB线性采样。这说明边界错误可能来自几何、遮挡与采样链路，而不是只有本项目Python显示层才有的可能性。
3. 官方[Limitations](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/limitations.html)说明重建质量受输入深度影响，并列出Python接口的一些限制。该页面**没有明确确认“黄色瓶体被涂到台面”这一特定问题**，不能把它引述为官方已承认同一bug；也不能反过来将所有问题推给相机硬件质量。
4. 官方[Isaac ROS Nvblox概览](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)描述深度、颜色和位姿的建图管线。ROS封装、核心版本、颜色更新策略和输入预处理可能不同，“都用官方nvBlox”不代表当前自写接入与workspaces在同一条件下必然得到完全相同结果。

结论：**在投影、合成几何或可见性不一致时，官方这一类投影式颜色融合也可能产生串色；但本轮没有官方材料可以证明这是该固定版本无条件必现的已知bug。正确遮挡深度应拒绝相差明显的后方台面。**

资料访问日期2026-11-11。网页文档为v0.0.10说明，具体内核以项目锁定e4a2b00提交和vendor头文件核对，不能用其他版本实现替代当前二进制行为。在线已核对固定提交CUDA模板；同提交完整appearance .cu页面未成功获取，未据其他版本补写该部分结论。Firecrawl CLI未安装，本轮改用网页检索工具获取官方资料，未安装或改变环境。

## 7. 下一步如何定位并优化

建议先做证据隔离，再动参数：

| 顺序 | 验证 | 如何判断 | 对应改善 |
|---|---|---|---|
| 1 | 同一保存Mesh用统一灰色与RGB显示 | 黄色长条是否仍是抬高的实体 | 分离TSDF几何问题和颜色污染 |
| 2 | 固定地图、固定相机，只检查一次颜色投影 | 同像素RGB、实测对齐深度、TSDF合成深度是否属于同一表面 | 定位遮挡误判，避免混入运动历史 |
| 3 | 用真实Color K/TF投影瓶体和台面 | 是否存在固定方向的系统偏移 | 修正经验证的相机模型/坐标链问题，而不是改frame名字 |
| 4 | 直接输出当前颜色用的合成深度与有效遮挡掩码 | 黄色RGB像素是否错误匹配台面深度 | 优先修正合成深度边缘与可见性判断 |
| 5 | 同几何下比较颜色射线采样、深度边缘拒绝与颜色专用容差 | 台面误色减少且正常表面颜色覆盖未大幅下降 | 避免为改颜色同时破坏TSDF几何 |
| 6 | 若接口需要，接入真正对齐到Color视角的实测深度 | 后方表面能否被当前观测正确拒绝 | 扩展Python/C++绑定；不能直接传原始Depth图冒充对齐深度 |
| 7 | 正确观测链路确认后，再清除局部历史颜色并重新融合 | 污染是否还会再出现 | 避免旧错误让修复效果不可见 |

注意：实测对齐深度不是万能补丁。Depth相机没看到但Color看到的区域，应保持不可用或保守处理，不能通过插值跨越前景/背景空洞人为补齐。严格时序和边缘一致性同样需要保留。

本轮没有调小截断、没有改α、没有清图，也没有声称已经修复。优先目标应是定位一次错误颜色更新为何通过遮挡检查，而不是继续增加融合帧数或仅调整RViz配色。

## 8. 分析产物

- [数值诊断](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/flask_diagnosis.json)
- [只读PLY分析脚本](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/analyze_flask.py)
- [局部候选顶点与标记](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/yellow_flask_bleed_20261111/flask_vertices.npz)
- [上轮5mm测试报告及数据说明](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/LIVE_5MM_SCAN_MESH_ESDF_ACCURACY_REALTIME_20261111.md)
