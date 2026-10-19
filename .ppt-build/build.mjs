import fs from 'node:fs/promises';
import {Presentation,PresentationFile} from '@oai/artifact-tool';
import {finalizePresentation} from '/home/czz/.codex/plugins/cache/openai-primary-runtime/presentations/26.1007.11041/skills/presentations/container_tools/artifact_tool_utils.mjs';
const root='/home/czz/perception_eye_in_hand', build=root+'/.ppt-build';
const p=Presentation.create({slideSize:{width:1280,height:720}});
const font='Noto Sans CJK SC', ink='#142D42', teal='#007E87', muted='#536779';
function tx(s,t,x,y,w,h,size=26,color=ink,bold=false){const a=s.shapes.add({geometry:'textbox',position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});a.text=t;a.text.style={typeface:font,fontSize:size,color,bold};return a;}
function slide(title,n){let s=p.slides.add();s.background.fill='#F7F9FA';tx(s,title,64,48,1150,70,43,ink,true);tx(s,'Unitree G1 · 3D 激光导航调研',64,666,900,25,16,muted);tx(s,`${n} / 4`,1150,666,70,25,16,muted);return s;}
function table(s,values,widths,y,height){const t=s.tables.add({rows:values.length,columns:values[0].length,left:64,top:y,width:1152,height,columnWidths:widths,values});t.borders.assign({fill:'#D6E0E5',width:1,style:'solid'});for(let r=0;r<values.length;r++){t.rows[r].height=height/values.length;for(let c=0;c<values[0].length;c++){const cell=t.getCell(r,c);cell.fill=r===0?ink:(r%2?'#FFFFFF':'#EEF3F5');cell.text.style={typeface:font,fontSize:23,color:r===0?'#FFFFFF':ink,bold:r===0};}}return t;}
let s=slide('Unitree G1 三维激光定位与避障',1);
tx(s,'调研结论与 baseline 方案',64,124,1100,46,28,muted);
tx(s,'3D 定位 + 2D 平地导航',64,228,1152,75,56,teal,true);
tx(s,'以室内平整地面为起点，先验证预建地图中的自主到点导航',64,320,1140,50,29);
tx(s,'定位与建图',64,431,310,43,28,ink,true);
tx(s,'Mid-360 + IMU\nFAST-LIO2 + 地图配准',64,487,400,95,25,muted);
tx(s,'障碍感知与规划',490,431,390,43,28,ink,true);
tx(s,'三维点云 + Voxel Layer\n二维代价地图 + MPPI',490,487,410,95,25,muted);
tx(s,'机器人执行',940,431,280,43,28,ink,true);
tx(s,'平面速度指令\nG1 运动控制器',940,487,280,95,25,muted);
s.speakerNotes.text='汇报范围：根据用户提供的调研材料整理，属于方案调研，尚无本项目实测结果。第一阶段假设室内平整地面。六自由度定位支持三维姿态估计，平地导航在 x、y、yaw 空间规划。\n原材料参考：FAST-LIO2 https://arxiv.org/abs/2107.06829\nNav2 Voxel Layer https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/';
s=slide('定位方案：FAST-LIO2 为主，分模块验证',2);
table(s,[['模块 / 方案','作用与优势','边界及选型建议'],['FAST-LIO2','LiDAR–IMU 紧耦合里程计\n增量构建三维点云地图','主 baseline\n回环与全局优化需额外集成'],['Point-LIO','面向高动态运动的 LIO','作为行走振动场景对照'],['预建地图定位','scan-to-map 配准\nFAST-LIO 地图定位实现','先采用人工粗初值\n再评估全局重定位'],['LIO-SAM / Leg-KILO','因子图优化 / 运动学融合','分别作为全局一致性、\n腿式运动鲁棒性扩展']],[280,435,437],157,380);
tx(s,'关键区分：连续里程计、回环建图、预建地图定位属于不同模块',64,575,1152,48,27,teal,true);
s.speakerNotes.text='FAST-LIO2 是里程计和增量建图前端，不能直接等同于带回环优化的完整 SLAM。地图定位通常需要较好的初值。所列工程实现的 ROS 版本、传感器和 G1 SDK 适配需要在实验阶段确认。\n原材料参考：\nFAST-LIO2 https://arxiv.org/abs/2107.06829\nPoint-LIO https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/aisy.202200459\n地图定位 https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID\nLIO-SAM https://ieeexplore.ieee.org/abstract/document/9341176/\nLeg-KILO https://ieeexplore.ieee.org/document/10631676/';
s=slide('避障方案：三维感知支撑平面规划',3);
table(s,[['处理环节','baseline 配置','输出 / 集成要点'],['障碍感知','实时点云，地面与自身过滤','保留与机器人碰撞相关的高度范围'],['导航地图','Nav2 Voxel Layer + 2D Costmap','体素观测投影为二维障碍代价'],['路径与局部避障','Smac / A* + MPPI','结合路径与障碍代价生成速度'],['G1 执行接口','速度桥接至运动控制器','vx、vy、ωz，按实机能力限速']],[250,425,477],155,355);
tx(s,'导航层决定行进方向和速度，G1 运动控制器负责步态执行',64,548,1152,47,28,teal,true);
tx(s,'集成重点：TF 与时间同步、机身碰撞轮廓、速度超时停车和急停',64,602,1152,39,24,muted);
s.speakerNotes.text='MPPI 在此承担高层平面导航，不是足步规划器。是否开放侧向速度、速度范围及接口由实机控制模式决定。二维地图不能完整刻画全身三维碰撞，因此第一阶段限制在适合该表示的平地场景。\n原材料参考：\nVoxel Layer https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/\nMPPI https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/\nG1 工程参考 https://github.com/binbin2002/Unitree_G1_Navigation/blob/main/README.md';
s=slide('验证计划与后续研究方向',4);
table(s,[['实验阶段','场景与任务','主要指标'],['① 定位稳定性','静止、直线、转向、启停\n对比 FAST-LIO2 与 Point-LIO','z / roll / pitch 波动\nATE / RPE、延迟与漂移'],['② 地图定位','保存地图后重启\n逐步增加位置与航向初值偏差','重定位成功率与耗时\n定位连续性'],['③ 平地自主导航','纸箱、桌腿、窄通道\n在受控条件下加入移动障碍','到达率、碰撞率、最小间距\n路径效率、急停次数']],[235,480,437],146,337);
tx(s,'阶段扩展',64,523,210,44,28,ink,true);
tx(s,'坡道 / 台阶：Elevation Map + 可通行性评估\n全身空间约束：ESDF；更复杂环境再研究语义与学习策略',290,520,920,90,25,muted);
tx(s,'先完成可复现 baseline，再用实测结果决定扩展方向',64,616,1152,39,26,teal,true);
s.speakerNotes.text='建议执行顺序：校准与同步检查、LIO 对照、已知地图定位、导航闭环、地形扩展。ATE/RPE 需要可用的参考轨迹；没有真值时应明确记录闭环终点误差、重复轨迹一致性等代理指标，不能称为 ATE。指标门槛应结合场地、参考系统和 G1 运动能力另行确定。本页为实验计划，无性能数据。\n原材料参考：\nElevation Mapping https://arxiv.org/abs/2204.12876\nnvblox https://arxiv.org/abs/2311.00626';
for(let i=0;i<p.slides.items.length;i++){let b=await p.export({slide:p.slides.items[i],format:'png',scale:1});await fs.writeFile(`${build}/slide-${i+1}.png`,new Uint8Array(await b.arrayBuffer()));}
const candidatePath=build+'/candidate.pptx';await (await PresentationFile.exportPptx(p)).save(candidatePath);
const skill='/home/czz/.codex/plugins/cache/openai-primary-runtime/presentations/26.1007.11041/skills/presentations';
console.log(await finalizePresentation({workspaceDir:root,candidatePath,finalPath:root+'/output/G1三维激光导航调研汇报.pptx',pythonExecutable:'/home/czz/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3',integrityValidatorPath:skill+'/container_tools/inspect_presentation_package_integrity.py',layoutValidatorPath:skill+'/container_tools/inspect_presentation_layout_geometry.py',layoutArgs:['--expected-slide-size-emu','12192000,6858000','--validate-bullet-geometry','--validate-heading-fit','--require-native-table-slide','2','--require-native-table-slide','3','--require-native-table-slide','4'],explicitTotalSlideCount:4,requiredNativeTableOwnerSlides:[2,3,4],fontPolicy:{basis:'design',families:[font]},verifyArtifactToolImport:true,receiptPath:build+'/validation.json'}));
