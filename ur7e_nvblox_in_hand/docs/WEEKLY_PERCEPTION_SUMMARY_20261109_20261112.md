# 感知与重建本周工作总结：问题、处理、效果与原始照片

统计时间：2026年11月9日—11月12日（截至本次整理）。项目：`perception_eye_in_hand`；主要测试对象：`ur7e_nvblox_in_hand`。本报告整理本周已有代码变更记录、仿真与实机测试数据，不新增测试，不修改感知源代码。

本周的主要进展是：打通UR7e机械臂位姿、D435i图像与重建流程，修复已复现的仿真输入缺帧问题，完成不同体素尺度、颜色开关、运动扫描和实机短测，并保存Mesh、原生地图和ESDF数据用于复查。**实机静止30秒窗口中，深度融合约29.55 Hz，ESDF更新和实际接收约29.03 Hz；大累积地图仿真扫描仍只有约10.47 Hz实际接收。** 两者负载不同，不能合并为“所有场景稳定30 Hz”。

本周尚未解决的重点是颜色跨物体边界污染、细小物体重建不足、遮挡造成的unknown、少量ESDF距离严重高估，以及地图扩大后的导出与发布成本。较高更新频率不代表重建准确性或抓取安全性已经验收。

## 一、本周问题与处理状态

| 本周提出的问题 | 本周已做的工作 | 结果与状态 |
|---|---|---|
| 相机、机器人和RGB可见，但不生成Mesh/ESDF | 核查输入话题、运行环境、图像时刻TF与ROS域 | 诊断发现手眼静态TF默认域0，而相机/Mapper在135；修正运行环境后，后续实机记录恢复持续融合。已验证恢复。 |
| 真实相机话题前缀与默认输入不一致、颜色未参与融合 | Python包实机默认话题适配`/camera/camera`；RViz输入跟随实际话题；相机启动参数整理 | 实机后续颜色融合约29.28 Hz，颜色数据已参与；颜色边界准确性仍未解决。 |
| 仿真CameraInfo存在，但同时间戳图像缺失，融合只有约18 Hz | 仿真相机加入可靠发布和历史深度配置，Mapper增加可选可靠订阅，配套启动匹配QoS | 相同静止输入对照中，深度24.71→29.97 Hz，彩色24.17→29.97 Hz；缺配对93/131→0/0。已修复该次复现。 |
| 深度图闪黑、偏暗，左上角有缺失区域 | 记录原始深度有效率、范围、图像年龄；区分显示归一化和无效数值 | 已采样窗口未出现整帧全无效；显示偏黑有量程因素。固定三角区域的唯一根因尚未确认，不能宣称修复。 |
| 启动后Mapper与RViz突然关闭 | 根据报错检查参数合法性 | `clearing_hz=1`但半径非正，触发启动校验；文档改为关闭清图或配置正半径。属于配置错误，不是GPU崩溃。 |
| `topic hz`无输出，内部指标却显示更新 | 区分更新调用、发布调用、订阅实际收到和RViz刷新；补充可靠raw接收测量工具 | 已记录实际非空ESDF接收频率。普通测频无输出不能据此认定没有发布，也不能用内部计数代替当前接收结果。 |
| Mesh模糊、边缘错位、瓶盖颜色下移或台面被染色 | 比较原始深度、单帧/多帧、颜色开关、不同体素；核查颜色投影和遮挡路径 | 确认部分异常写入保存PLY，并非仅RViz渲染问题；具体颜色污染根因尚未闭环。 |
| ESDF有死角、长尾、多余点和少量异常距离 | 读取原生observed/site/parent状态，比较仿真真值，检查原始深度长距离返回 | 区分自由空间距离点、unknown和真正距离错误；仿真确认少量严重高估仍存在，尚未修复。 |
| 5mm是否能兼顾精度与实时性 | 完成有界回放、现场仿真扫描、实机静止短测 | 精细轮廓有所改善，但无界5mm测试曾显存不足；大地图输出仍明显降频。 |

以上“解决”仅限对应测试复现，不代表另一套功能包及全部运行条件都已修复。`ur7e_nvblox_in_hand`是Python ROS接入与调度、nvblox_torch中的C++/CUDA计算；`nvblox_in_hand`是另一套C++ ROS路径。两者不能混写成同一次修改或同一次测试。

## 二、已解决的关键阻塞与具体修改

### 2.1 图像输入和实机接入

仿真图像与CameraInfo虽然由同一帧产生，但作为独立ROS消息传输。此前Best Effort及有限队列条件下，存在CameraInfo送达、Image未送达的记录，不能将此直接归因于相机生成了错误时间戳。此次对图像发布可靠性和队列配置进行调整，并提供匹配的订阅选项，没有伪造时间戳或补空图像。

| 代码位置（相对于本功能包目录） | 本周记录中的修改及作用 |
|---|---|
| `src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp` | Image和CameraInfo发布QoS可配置，配套仿真默认可靠传输、历史深度10。 |
| 同目录`RealSensePlugin.cpp` | 解析新增可靠性配置。 |
| `src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro`、`launch/scene.launch.py` | 将启动配置传给相机插件。 |
| `src/ur7e_nvblox_in_hand/ur7e_nvblox_in_hand/node.py` | 加入`input_qos`选择，图像与CameraInfo使用匹配策略。 |
| `src/ur7e_nvblox_in_hand/launch/bringup.launch.py`、相机配置及RViz配置 | 适配真实`/camera/camera`输入、显示输入及仿真可靠性；保留显式参数优先。 |

可靠性与历史深度同时改变，所以不能把改善精确拆分成某一个DDS参数的贡献。可靠传输仍受资源限制，不能保证任意负载下绝不丢帧。详见[输入缺帧修复报告](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/INPUT_DELIVERY_FIX_AND_ESDF_RETEST_20261110.md)及[实机颜色输入修复记录](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_COLOR_INPUT_FIX_AND_RETEST_20261109.md)。

### 2.2 TF、环境和启动参数

无重建诊断时，图像回调持续增长，图像时刻TF查询却为0/320成功，深度与颜色只是在队列中被覆盖。手眼TF进程存在不代表它与Mapper处于同一ROS域。统一环境后，后续实机测试已确认图像参与融合。此前这一故障的直接条件是位姿不连通，不能把0 Hz解释成融合算力不足。

`clearing requires positive radius`则是另一类问题：开启清图而未提供合法半径时，Mapper启动即退出，launch继而关闭RViz。运行说明采用`clearing_hz:=0.0`保留地图；若确需半径清图，应同时设置正半径。半径清图不等于去除运动残影，不能作为通用修复。

本周还整理了依赖迁移、UR驱动、相机、手眼TF、实机运行和保存操作说明。用户反馈相机环境更新后启动恢复；没有足够证据把所有相机启动问题都归为硬件损坏或某一次SDK升级。

## 三、重建与颜色问题：对应原始照片

以下P01—P05均为用户提出问题时提供、项目中已保存的原图副本，不是本次重新生成的效果图。仅归档和核对文件一致性，没有修改照片内容。照片展示现象，不单独作为精度真值。

### P01：瓶盖颜色向下扩展、侧面呈阶梯状

![P01 用户原图：瓶盖颜色与阶梯状重建](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/P01_cap_color_staircase.png)

对应问题：黄、蓝、绿本应主要出现在瓶盖顶部，却扩展到侧面及其他深度位置。颜色可能被错误投影到几何表面，也可能受到几何离散、遮挡判断和表面混合影响，不能仅凭截图认定“融合帧数太多”。

本周核查的颜色流程是：RGB、Color CameraInfo和图像时刻相机位姿进入`add_color_frame`，当前调用未提供实测对齐深度，底层使用TSDF生成合成深度进行遮挡判断，通过表面距离门限后取RGB颜色，再形成带顶点色的Mesh。若合成深度命中错误表面或投影不一致，错误表面仍可能通过判断。双线性取色、体素和截断带可能扩大边缘混色，但不能独立解释全部大范围污染。

颜色关闭对照中，同一输入的Python/C++四组测试Mesh几何指标接近，说明至少该轮模糊几何并非仅由颜色开启造成。**颜色参与融合已经恢复，不等于颜色融合质量已修复。** 详见[瓶盖分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/CAP_COLOR_BLEED_AND_STAIRCASE_CODE_ANALYSIS_20261111.md)及[两后端颜色开关对照](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/COLOR_ON_OFF_TWO_BACKENDS_TEST_20261110.md)。

### P02、P03：黄色瓶体旁的工作台被错误染黄

![P02 用户原图：整体实验台重建](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/P02_yellow_flask_full.png)

![P03 用户原图：黄色瓶体局部](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/P03_yellow_flask_detail.png)

本周读取此前保存PLY，在瓶体外围近台面位置检出173个黄色候选顶点，高度约−7.5～+7.5mm，超出真实瓶体底部最大半径。支持“至少部分工作台几何被染成黄色”，因此更换CloudCompare或RViz不能消除已经写入数据的颜色错误。该PLY与照片不是同一时刻快照，173不是对照片逐像素的计数。

另外，场景中的黄色瓶体本来由分段圆柱构成，部分规则阶梯来自场景真值；不能把所有阶梯都算成算法缺陷。5mm×4体素=20mm截断尺度值得检查，但不能单独解释178mm高瓶颈颜色落到台面的情况。后续需记录实际融合瞬间的RGB射线、合成深度、实测深度和被更新体素，定位投影与可见性，而不是仅继续加帧或调小体素。详见[黄色瓶体与台面串色分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/YELLOW_FLASK_TABLE_COLOR_BLEED_ANALYSIS_20261111.md)。

## 四、ESDF与深度问题：对应原始照片

### P04：ESDF左上角多余区域、长尾和漂移疑问

![P04 用户原图：实机ESDF左上角异常区域](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/P04_real_esdf_protrusion.png)

ESDF点并非全部是障碍物表面。已观测自由空间也有“距障碍物多远”的有效值，因此大面积点不直接等于虚假障碍物；observed显示只筛选已有信息的体素，没有把unknown补成完整障碍物。

本周用原生地图状态、连通分量、深度端点和Mesh做了联合检查。抽查的一部分孤立区域在当前深度表面之前，属于自由空间距离点；不能把每一块彩色区域当作新增实体。另一方面，仿真真值对照确实发现少量ESDF高估，例如真实距离约9.25cm却输出2m，已进入实际发布点云，不能归因于显示配色。

实机额外深度采样有14/59帧出现1～2.05m长距离返回，但这些端点未与旧TSDF零面匹配；深度与保存地图并非同时刻，尚不能证明照片左上方全部由这些返回造成。真实漂移还需检查历史坏观测、TF/投影一致性及无效深度不清除历史的影响。**当前是完成分类诊断，尚未完成统一修复。** 详见[深度与ESDF突出区域分析](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_DEPTH_VS_ESDF_PROTRUSION_ANALYSIS_20261111.md)。

### P05：深度图偏黑、边缘缺失及固定黑区

![P05 用户原图：2026-11-11深度显示](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/P05_real_depth.png)

该图对应本周“深度闪黑、缺失区域是否导致ESDF多余点”的问题。原始数值0代表无效，与较小有效深度显示成深灰不是同一情况。原图自动归一化可能被远处大深度拉大量程，近处工作台因此很暗；已采样320帧没有整帧全无效，另一次轻量可靠接收296帧、约29.56 Hz。

这些结果只证明采样窗口没有复现全无效，不证明闪黑永不存在。RGB与Depth光学中心、内参和视场不同，未对齐时不能用相同像素直接比较缺失；固定在图像坐标的三角区域还需检查相机/支架遮挡、红外匹配及原始有效值。关闭infra1/2的ROS输出不等于关闭设备内部双目红外测深。该问题尚无唯一根因，不标记为已修复。详见[无重建与深度闪黑诊断](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_NO_MAP_DEPTH_FLICKER_DIAGNOSIS_20261111.md)。

本次归档未找到另一次“RGB和三角黑区同屏”的临时原文件，未用其他图片冒充。P05是已保存的同周深度原图；启动报错和缺TF等问题主要依据用户文字日志。

## 五、本周实际测得的实时性效果

下表每行来自不同条件，**用于说明负载与阶段差异，不是严格单变量性能排名**。更新为节点已有调用计数；实际接收为订阅者收到完整ESDF消息；均不是RViz画面FPS。

| 已完成测试 | 体素与条件 | 深度融合 | ESDF更新 | ESDF实际接收 | 延迟代理P95 |
|---|---|---:|---:|---:|---:|
| 有界仿真录包回放，2cm | 640×480，stride2，无GUI | 29.37 Hz | 28.59 Hz | 26.17 Hz | 59.59ms，内参接收→点云接收 |
| 有界仿真录包回放，5mm | 640×480，stride2，无GUI | 26.21 Hz | 23.38 Hz | 22.97 Hz | 172.17ms，内参接收→点云接收 |
| 现场Gazebo左右扫描，5mm | 累积大地图、GUI、约63.77万显示点 | 10.71 Hz | 10.69 Hz | 10.47 Hz | 305.90ms，内参接收→点云接收 |
| 实机静止30秒，5mm | UR7e＋D435i，约14.69万显示点 | 29.55 Hz | 29.03 Hz | 29.03 Hz | 85.48ms，输入时间戳→点云接收；同帧内参接收代理为51.47ms |

实机记录收到871条ESDF，具有871个不同输入时间戳，三个10秒窗口接收分别为29.2、29.2、28.7 Hz；接收间隔最大约104ms。**本周有证据支持“这一静止短窗口接近30Hz”，没有证据支持“所有场景、手拖运动和RViz渲染均稳定30Hz”。** 若把采样前后及保存阶段一起计入，完整接收日志口径约27.82Hz，不能只报较高窗口而不说明边界。

![R01 测试生成图：实机实时性分段统计](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/R01_real_realtime_summary.png)

R01来自实机记录数据，并非用户提出问题时的截图。输入时间戳年龄和内参接收代理不是曝光到屏幕的完整延迟；P95表示95%样本不超过的值，不是最大值，也不是准确率。

### 为什么大地图还是慢

现场扫描点云约15.3MB/条，实机静止约3.53MB/条；地图规模、场景、GUI和订阅者不同。大地图扫描时ESDF导出平均约34.90ms、打包发布约41.29ms，完整Mesh回读约59.84ms、打包约93.91ms；实机静止ESDF导出约3.21ms、打包约6.37ms。不能把这种差异全部归为代码加速。

代码工作线程还要处理深度、颜色、ESDF导出与Mesh回读；原生导出存在全层遍历、设备同步和分批读回，Mesh使用完整数据发送。**底层增量融合/ESDF更新，不等于ROS导出与Mesh发布也是增量。** 输入可达30Hz而处理队列被覆盖，是当前大图降频的一条已观测链路；不能只将定时器写成33.4Hz就保证输出33.4Hz。

普通`topic hz`本机版本使用Best Effort；Reliable发布者与其并非天然不兼容，大消息送达和订阅调度仍可能有波动。本周采用独立可靠raw接收工具核实数据，但尚未用逐包跟踪证明每次无输出的唯一原因。详见[实机原始结果与测量口径](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_TEST_RESULTS_AND_ANALYSIS_20261111.md)。

## 六、重建和ESDF精度：取得了什么效果

本周加入包含工作台、瓶体、试管架和不同尺度物体的实验室场景，避免仅在简单平面上评价效果；并进行单帧、静止多帧、多视角、颜色开关及不同体素对照。5mm不意味着每一处表面误差都小于5mm，ESDF stride2也只是输出采样，未把原生地图改成1cm。

| 现场5mm仿真最终结果 | 数值 | 解读 |
|---|---:|---|
| Mesh可见区域轴向误差P95 | 3.21mm | 大面积台面占比较大，不代表全部细物体。 |
| 去除台面的物体区域P95 | 11.18mm | 瓶盖、细杆、边缘仍存在明显重建不足。 |
| 可见参考覆盖率 | 99.92% | 是该视角可见参考区域，不是全场景无死角。 |
| ESDF已观测自由空间MAE / P95 | 3.01 / 6.77mm | 评价ROI内原生体素的stride2子集，未知位置另计。 |
| ROI unknown | 15.31% | 主要存在未分配TSDF块或权重为0的区域。 |
| 输出+2m的异常自由空间样本 | 18个 | 仿真评价中存在严重距离高估，最大误差约1.907m；不能只看P95忽略。 |

![R02 测试生成图：现场保存Mesh与输入视图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/R02_live_mesh_comparison.png)

R02为实际保存PLY的渲染与输入视图，不是原始投诉照片。RGB和深度相机视角不同，不可直接按两图像素差测纹理误差。本轮扫描延续已有地图，并非空图单帧/多帧公平对照。

无界5mm、较大范围测试还出现过CUDA显存不足；限制到工作台范围后才完成对应测试。有界5mm回放中Mesh整体P95约3.56mm、物体P95约18.51mm、可见覆盖99.50%；其输入与现场累积地图测试不同，不能用两组数字宣称同场景误差下降。

实机已保存65,130顶点、102,822三角面的Mesh，及原生地图、最后ESDF点云和分阶段CSV。实机没有独立几何真值，因此本周**没有给出实机毫米级精度结论，也没有完成实际抓取验收**。实机静止测量不能替代操作员手拖时的运动重建测试。

## 七、后续优先解决的工作

1. **先检查距离正确性。** 对仿真固定输入比较增量更新与全量重算，追踪异常ESDF体素的site、parent和更新状态；修复少量严重高估，再谈用于碰撞或抓取。
2. **拆开几何与颜色。** 固定同一输入保存原始深度点云、无色Mesh和彩色Mesh，记录污染位置对应的RGB投影、实测深度与合成深度；避免仅凭“更细体素”推断颜色已正确。
3. **做同负载实时性对照。** 固定地图、输入、GUI与订阅条件，分别关闭/降低完整Mesh回读和ESDF导出，定位计算、同步、打包和送达成本，再考虑按块增量输出与线程解耦。
4. **完成实机手拖与长时间测试。** 保持既有配置，记录静止→手拖→停止，检查残影、队列覆盖、时间戳年龄及地图增长；本周约30秒静止结果不能替代该验收。
5. **同步捕获黑区。** 保存异常瞬间原始深度、RGB、内参和TF；区分固定图像区域、真实遮挡/材质以及仅显示量程问题。unknown应保留未知语义，不能填成自由空间制造覆盖率。

## 八、原始照片、数据和详细报告位置

原始问题照片副本位于[本周图片目录](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112)。[sources.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/images/weekly_20261112/sources.json)记录每张图的来源、类别及SHA-256；P01—P05与归档源文件逐字节一致，R01—R02明确标为测试生成图。

| 证据 | 路径 |
|---|---|
| 实机原始频率、延迟和配置 | [result.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/result.json)、[stage_rates.csv](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/stage_rates.csv)、[params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/params.yaml) |
| 实机保存重建 | [mesh.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/mesh.ply)、[map.blox](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/map.blox)、[esdf_points.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/esdf_points.ply) |
| 现场仿真完整报告 | [5mm扫描、精度与阶段性能](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/LIVE_5MM_SCAN_MESH_ESDF_ACCURACY_REALTIME_20261111.md) |
| 5mm回放与显存限制 | [5mm精度及流水线测试](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/VOXEL0005_MESH_ESDF_ACCURACY_AND_PIPELINE_TEST_20261111.md) |
| 2cm测试与指标定义 | [2cm Mesh/ESDF精度及性能](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/VOXEL002_MESH_ESDF_ACCURACY_AND_REALTIME_20261111.md) |
| 实机完整运行与保存说明 | [README_REAL_TEST.md](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/README_REAL_TEST.md) |

本周完成的是输入链路修复、接入恢复、测试场景与证据完善，并确认了特定负载下接近30Hz的实际输出；颜色质量、距离异常与大地图吞吐仍列为待解决项。本次仅生成总结及照片归档，没有重新运行机器人、改变参数或修改融合算法。
