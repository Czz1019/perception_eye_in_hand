# 瓶盖颜色沿瓶身扩展、Mesh阶梯变形：代码与已有数据分析

日期：2026-11-11。分析对象：`ur7e_nvblox_in_hand` 的颜色/深度接入、nvblox_torch接口、配套仿真模型以及已保存1cm/2cm Mesh。本轮**不修改生产代码、不重跑仿真、不重新融合数据**，只读取已有PLY和参数、检查源代码及生成分析图表。

## 1. 主要判断

用户看到的“瓶盖颜色出现在瓶身下方、侧面呈阶梯”确实值得排查。它不能简单归结为融合次数太多，也不能只归结为RViz。

1. **先区分合理的模型特征。** 仿真瓶盖是带厚度的彩色圆柱，侧壁也有颜色；托盘两排小瓶本来相差15mm高度。因此不要求所有彩色像素只在一张水平顶面上。
2. **但存在超出上述合理范围的颜色扩展。** 已保存1cm单帧Mesh中，托盘瓶盖附近已有高饱和彩色顶点位于对应真实瓶盖底面下方超过20mm，静止60帧后基本不变；这排除了“必须经过很多帧或机械臂运动才会出现”的解释。
3. **当前着色依赖重建出来的几何。** Python `add_color_frame`未传入实测对齐深度；固定版本核心接口说明该路径用TSDF生成合成深度来判断遮挡。TSDF表面不准确，会进一步影响颜色投影的有效性。
4. **颜色更新并非只允许发生在零表面。** 固定版本投影内核按截断带检验体素与渲染表面的距离，再从RGB双线性取色。4体素对应1cm地图的4cm，或2cm地图的8cm；相对12mm瓶盖，这个范围值得优先验证。
5. **阶梯形状还涉及几何表示，颜色不能单独移动顶点。** TSDF/零面提取决定形状，Color层影响表面外观，体素取色和三角面颜色插值进一步使色块扩展显眼。

本次数据支持优先检查“**单帧TSDF几何 → 合成深度/遮挡判定 → 颜色带宽 → Mesh取色**”。尚未独立改变这些因素进行对照，不能断言其中某一项就是唯一根因。

![用户提供的局部现象](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/user_cap_mesh.png)

截图没有附PLY文件名、坐标与实际参数，所以不能将截图本身强行认定为某一轮配置；下面的定量结果明确来自已归档的1cm/2cm测试数据。

## 2. 真实模型中的瓶盖到底在哪里

### 2.1 托盘内六只小瓶

[lab_scene.py:102](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/scripts/lab_scene.py:102)：

```python
h = 0.055 + 0.015*j
# 瓶身：底部z=0.006，顶部z=0.006+h
b.cylinder(..., radius=0.018, height=h, ...)
# 瓶盖：半径0.020、厚度0.012
b.cylinder(..., radius=0.020, height=0.012,
           xyz=(tx, ty, 0.006+h+0.006), color=CAPS[i+j])
```

上面为依据实际代码语义整理的示意，具体调用见链接。

| 对象 | 瓶身顶部/瓶盖底部 | 瓶盖顶部 | 瓶盖真实厚度 |
|---|---:|---:|---:|
| 第一排j=0 | 61mm | 73mm | 12mm |
| 第二排j=1 | 76mm | 88mm | 12mm |

所以两排本来有15mm高度差，且彩色圆柱侧壁是合理外观。**但第一排瓶身30–40mm高度处出现大量瓶盖色，不能由12mm瓶盖厚度解释。**

### 2.2 试管架的高试管

[lab_scene.py:86](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/scripts/lab_scene.py:86) 另有 `h=tube_height+0.012*((i+j)%3)`，瓶盖厚14mm。其试管原本就有每档12mm的高度变化。不能把不同试管顶部高低变化直接当作重建错位。

应检查的是：同一支瓶子上，重建表面与颜色是否越出了该瓶盖的真实三维范围；不要只凭截图旋转后的视觉上下关系判断。

## 3. 对已有Mesh数据做了什么检查

使用前两轮归档的单帧、静止60帧、多视角PLY，不重新建图。

- 限定托盘六只小瓶附近：距最近瓶中心XY小于28mm，高度15–130mm。
- 用顶点RGB筛出饱和度≥0.55、色差幅度≥0.2的高饱和彩色候选点，尽量排除灰白瓶身与托盘。
- 按最近小瓶中心匹配真实瓶盖底部，计算候选顶点低于瓶盖底部的距离。
- 这是**候选顶点诊断**，不是完美的材质语义分割；数量不是表面积、像素占比或统一精度分数。跨体素尺寸顶点密度不同，不能直接比较点数优劣。

| 配置/阶段 | 彩色候选顶点数 | 低于瓶盖底部>10mm | 低于瓶盖底部>20mm | 向下超出量P95 |
|---|---:|---:|---:|---:|
| 1cm / 单帧 | 203 | 56 | 33 | 36.0mm |
| 1cm / 静止60帧 | 203 | 56 | 33 | 36.0mm |
| 1cm / 多视角 | 231 | 71 | 46 | 41.0mm |
| 2cm / 单帧 | 74 | 35 | 25 | 46.0mm |
| 2cm / 静止60帧 | 74 | 35 | 25 | 46.0mm |
| 2cm / 多视角 | 74 | 35 | 25 | 46.0mm |

1cm单帧203个候选中，有33个低于对应瓶盖底部超过20mm；多视角变为231个候选、46个超过20mm。2cm也在单帧阶段已出现。这支持“初次投影、体素表示或单帧几何已经造成异常”，不能用多帧数目独立解释。

上表向下超出量按`max(瓶盖底部z−顶点z,0)`计算，顶面或盖内点记0；它不是全局Mesh P95，也不是TSDF截断距离。数值有筛选范围限制，不能把其最大值理解成整幅地图的最大颜色错位。

![彩色Mesh顶点与真实瓶盖高度的对照](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/cap_height_stages.png)

图只画托盘第一排三个小瓶的附近顶点，灰点为Mesh顶点，彩点保留原始顶点RGB；虚线框为真实瓶盖x/z范围，z=61–73mm。上排1cm、下排2cm，三列依次为单帧、静止60帧、多视角。没有对网格平滑或重新上色。

这幅图直接在base_link坐标中比较，不受截图视角、RViz光照或显示点大小影响。它同时显示：一些彩色顶点在真实盖范围之外，而且单帧和静止60帧相似。

## 4. 当前数据流为什么可能产生这种现象

### 4.1 深度与颜色走两条接入路径

[backend.py:76](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:76) 的深度路径：

```text
Depth图像 + 自身CameraInfo + 图像时刻的Depth optical pose
 → 深度解码/毫米转米/无效值过滤
 → add_depth_frame
 → TSDF距离与权重更新
```

[backend.py:86](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/backend.py:86) 的颜色路径：

```text
RGB图像 + 自身CameraInfo + 图像时刻的Color optical pose
 → add_color_frame(RGB, pose, sensor)
 → 核心对已有TSDF进行颜色更新
 → Color体素
```

这里没有把彩色纹理逐像素贴到一个已经完全正确的表面上。颜色要依赖当前三维体素与RGB的投影关系。

两路各自使用自己的frame和内参是正确方向，不能为了“统一”就把原始深度的frame改成color。未对齐深度的内参和坐标系不能靠改字符串变成对齐输入。

### 4.2 当前Python颜色入口未接收真实深度

本机安装的 [mapper.py:122](/home/czz/.local/lib/python3.10/site-packages/nvblox_torch/mapper.py:122) 以及固定版本桥接头文件 [py_mapper.h:53](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox_torch/cpp/include/nvblox_torch/py_mapper.h:53) 表明：颜色接口接收RGB、位姿、sensor、可选mask及map id，**没有实测Depth参数**。

固定版本 [projective_appearance_integrator.h:50](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/projective_appearance_integrator.h:50) 说明：若有对齐深度则可直接用于遮挡检查；没有时通过sphere tracing生成合成深度。本项目调用的是没有实测Depth的路径。

因此：

```text
原始Depth可以很清晰
    ↓ TSDF空间离散、投影融合后，细瓶颈/盖边缘可能变形
当前TSDF表面不够准确
    ↓ 从RGB相机视角生成的合成深度也受影响
颜色体素的可见性判断和取色范围偏差
    ↓
瓶盖色可能落在错误侧面或阶梯状表面上
```

**依赖合成深度是官方支持的实现方式，不是仅凭这一点就能宣布代码错误。** 但它意味着原始深度图好看，不能保证颜色融合所使用的遮挡几何同样准确。

### 4.3 截断带并非零厚度表面

固定版本内核 [projective_integrator_impl.cuh:156](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/internal/cuda/impl/projective_integrator_impl.cuh:156)：

```cpp
voxel_distance_from_surface = surface_depth_m - voxel_depth_m;
if (fabsf(voxel_distance_from_surface) > op->truncation_distance_m_) return;
// 随后在RGB图像中双线性插值取色
```

它检验的是体素沿相机深度方向与合成表面的接近程度，不是“材质是否属于这个瓶盖”，也不是“这个体素是否恰好位于TSDF零表面”。

对于4体素截断：

| 配置 | 截断尺度 | 相对托盘瓶盖12mm厚度 |
|---|---:|---|
| 1cm体素 | 40mm | 约3.3倍 |
| 2cm体素 | 80mm | 约6.7倍 |

几何/投影有误时，较宽接纳范围可能使原本在瓶盖附近的颜色观测影响错误位置。对薄盖、紧邻瓶身和遮挡边界尤其需要验证。

但不能据此说“4cm以内所有格点必定涂上盖色”：还要满足视锥、深度有效性、遮挡和图像投影等条件。表中40/80mm是相机深度方向的检验尺度，也不是严格的世界z轴颜色扩展上限；顶点/体素位置、不同视角和合成表面误差都会影响最终结果。

### 4.4 体素取色与Mesh顶点颜色

固定版本 [voxel_access.cuh:85](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/mesh/internal/cuda/voxel_access.cuh:85) 包含按位置找到一个AppearanceVoxel并取其颜色的路径。Mesh使用顶点颜色，最终三角形内还可能插值，**没有使用高分辨率UV纹理图集**。

1cm体素只能沿12mm盖厚提供约1.2个体素尺度，2cm只有约0.6个。盖顶、盖侧与瓶身的材质边界容易落在同一体素或相邻粗单元中。一个顶点被赋上瓶盖颜色，三角面视觉上可把颜色继续铺开。

这能解释色块边界和分辨率限制，但单个体素的取色限制本身不能充分解释数厘米以上的所有错位；必须与上游几何和可见性一起看。

## 5. 为什么表面还会呈“阶梯”

需要把两类现象拆开：

| 现象 | 首先影响它的部分 | 说明 |
|---|---|---|
| 顶点真的偏到较低位置、表面鼓包或成片连接 | 深度投影、TSDF离散/融合、零面提取 | 颜色不会通过add_color_frame直接修改TSDF几何 |
| 几何位置大体正确，但瓶身被涂上瓶盖色 | 颜色投影、遮挡带、Color体素与Mesh取色 | 这是材质/着色对应关系问题 |
| 只在有色图中像有台阶 | 顶点颜色、三角面插值、光照 | 可用相同PLY的统一灰色/法线视图分辨 |
| 不同瓶盖本来高低不同 | 场景模型 | 托盘差15mm，架上试管每档差12mm，不是算法错误 |

前两轮已检查：拥有8个有效TSDF邻居的Mesh顶点基本贴合TSDF零面；1cm最终约96.55%的顶点通过该检验。该结果说明已经检测到的几何模糊不是单纯RViz把网格画错；不能把一致性残差误读成对场景的亚微米精度。

小圆柱、薄盖和6mm架条在1cm/2cm地图中仍属于难以稳定表示的结构。Marching Cubes做的是离散TSDF零交叉插值，不会自动恢复不存在的亚体素细节，也不保证产生规则的圆柱。

## 6. 是否属于“过度融合”

### 6.1 如果指“帧数太多把正确结果融坏”

现有证据**不支持它是主要解释**：

- 单帧已经存在颜色下移和几何损失。
- 静止60帧后的颜色候选分布几乎一致。
- 静止输入只有约0.6165%的像素变化过，主体深度重复。
- 多视角增加覆盖，也可能增加错误投影或历史污染，但不是产生这个现象的必要条件。

正确的融合不应凭空让静止物体不断漂移。若错误对应关系已经进入地图，重复融合可能持续保留、更新或强化错误，而不是因“次数多”自动导致错误。

### 6.2 如果指“空间融合/着色带比物体细节宽”

这是**值得优先验证的解释**：瓶盖厚12mm，而当前截断尺度40/80mm；再结合变形的TSDF合成表面和体素着色，可能造成盖色扩展、盖身边界糊化。

需要把几何TSDF截断、颜色可见性容差和颜色采样分辨率分别验证；不应只减少融合帧数，也不应不加验证地把截断距离调到极小。

### 6.3 max_weight不是融合次数开关

`max_weight=5`限制累积置信权重，不表示最多融合5帧，也不表示达到5就完全停止更新。对于TSDF，新旧距离仍可按观测权重更新，不能用“权重满了所以表面必然过度融合”解释。

还有一个版本差异：固定版本 [projective_integrator_params.h:65](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/integrators/projective_integrator_params.h:65) 将外观更新参数定义为：

```text
C_new = α × C_measured + (1−α) × C_old
默认α=0.8
```

本轮仅构造参数类读取到默认α约0.80000001，没有创建地图或执行融合；记录在 [wheel_default_parameters.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/wheel_default_parameters.json)。项目backend没有显式覆盖这个参数。也就是说，当前外观更新的参数语义包含较强的新观测权重，不能把它笼统等同于旧版ROS颜色权重累计平均。

错误的新投影可能快速改变体素颜色；把α调低可以降低单次变化，但**不会纠正错误投影位置**。本轮没有更改它，也没有进行α对照，尚不能认定0.8是截图错误的唯一原因。

## 7. 时间、内参与外参是否可能有问题

[node.py:154](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py:154) 按每张图自己的时间戳查TF，不使用“最新TF”替代；深度和颜色各有队列、频率门限及自己的CameraInfo。当前没有要求每张RGB与同一时刻实测深度成对后再调用颜色融合。

对于静态场景，分路时间匹配并非天然错误；但运动时，RGB时刻、最新TSDF内容、队列延迟及遮挡关系可能不完全一致。相机看到移动机械臂时，静态地图假设也会受影响。

还应核对相机模型：backend.sensor当前使用CameraInfo.K，没有自动进行去畸变，也没有根据原始/rectified话题选择P。这是实机迁移时要明确的输入契约；但仿真单帧已出现问题，本轮没有证据证明K/P或手眼外参就是当前截图的根因。

不能通过把depth frame名字改成color，或把两路位姿强行设成同一个，来修正投影。真正需要的是正确内参、外参、时间戳以及必要的深度重投影。

## 8. 与workspaces参考实现的关系与差异

| 项目 | workspaces本地旧ROS核心 | 当前ur7e_nvblox_in_hand |
|---|---|---|
| 几何/颜色主体 | nvBlox C++/CUDA | wheel内nvBlox C++/CUDA，Python仅接入/调度 |
| 彩色投影 | 旧ProjectiveColorIntegrator用TSDF sphere tracing生成遮挡深度 | 固定头文件支持合成/实测对齐深度，当前Python入口未传实测深度 |
| 颜色更新语义 | 本地旧源码采用观测权重累计混合 | 固定新版本参数文档定义外观α混合，默认0.8 |
| Mesh颜色 | 旧核心按ColorVoxel赋色 | 当前也保留体素外观/顶点着色表示，不是UV纹理系统 |
| 调度接入 | 官方ROS节点封装 | 本项目图像队列、TF、门控和发布线程自行组织 |

读取的旧参考源码：

- [projective_color_integrator.cu](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/integrators/projective_color_integrator.cu)。
- [mesh_integrator_color.cu](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/src/mesh/mesh_integrator_color.cu)。

**版本证据边界：** 当前wheel固定提交e4a2b00，项目vendor头文件与之匹配；另一本地nvblox_ws源码HEAD为3a97d1f，只作实现思路参考，不能冒充相同编译提交。在线获取固定提交的完整CUDA源文件未成功，本文关于当前内核门限的结论依据本项目固定版本的CUDA模板头文件，关于默认外观参数依据固定头文件和实际参数类读取，不以另一版本源码替代二进制级验证。

因此“都调用官方核心”不意味着两套代码版本、参数、相机轨迹和输出表现必然完全相同；反过来，也不能只因为效果差就认定没有使用官方核心。

## 9. 建议的排查和优化顺序（本轮不实施）

| 顺序 | 验证内容 | 解决什么问题 | 判据 |
|---|---|---|---|
| 1 | 同一Mesh统一灰色与原始顶点色并排 | 分开几何台阶与着色台阶 | 几何截面是否偏离真实瓶盖/瓶身 |
| 2 | 直接投影真实瓶盖位置到原始RGB/Depth | 核对K、frame、外参和深度解释 | 原始图上的盖轮廓与投影是否一致 |
| 3 | 从Color视角比较原始重投影深度和TSDF合成深度 | 验证颜色实际使用的遮挡表面 | 瓶盖边界是否有系统偏差/跨物体连接 |
| 4 | 几何固定，仅缩小颜色表面容差并检查轮廓边缘 | 降低错误侧面着色 | 盖下方彩色候选的面积/距离减少，同时颜色完整性不明显下降 |
| 5 | 比较颜色sphere tracing抽样4与1/2 | 检查小盖边界的合成深度分辨率 | 边缘误着色下降，并记录GPU耗时 |
| 6 | 若需要，扩展绑定传入真正对齐的实测Depth | 降低着色对模糊TSDF合成表面的依赖 | 遮挡一致性改善；不可直接传未对齐原始深度 |
| 7 | 分离TSDF截断与颜色容差，逐项调参 | 避免一个宽尺度同时支配几何与外观 | 在同一输入/体素精度下改善材质边界且孔洞不增加 |
| 8 | 处理动态自体和过期颜色历史 | 减少移动观察后错误保留 | 同轨迹残影与颜色污染减少 |

sphere tracing默认抽样4来自固定版本ProjectiveAppearanceIntegrator头文件，项目未提供独立ROS参数覆盖；**它与raycast_subsampling_factor=4不是同一个开关**，更与ESDF显示stride=2无关。需要核对/扩展对应接口后再对照，不能误改ESDF stride来修复Mesh。

固定版本 [mapper.h:211](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/vendor/nvblox/include/nvblox/mapper/mapper.h:211) 已有“Color+对齐Depth”核心重载，但当前Python绑定签名没有该入口。因此若选择该优化，需要适配绑定/桥接，同时完成Depth→Color的正确重投影和时间匹配；仅换topic名并不能实现。

以上为后续方案，**不是本次已经完成的修复或复测结果**。本次只确认现象在数据中存在、已在单帧出现，并给出最应优先检查的代码阶段。

## 10. 本次产物

- [分析脚本](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/analyze_saved_caps.py)：读取六份既有PLY，按真实瓶盖位置进行候选诊断。
- [原始统计JSON](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/cap_color_vertex_summary.json)。
- [高度图生成脚本](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/plot_cap_heights.py)。
- [参数类默认值读取](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/cap_color_geometry_analysis_20261111/wheel_default_parameters.json)。
- [前一轮1cm完整精度/实时性报告](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/VOXEL001_UNKNOWN_MESH_ESDF_STAGE_TEST_20261111.md)。

本轮未修改融合、接入、配置或显示代码，没有新增运动、仿真或实机操作。后续修改应以本报告列出的分离验证为依据，避免把所有现象简单归为“过度融合”。
