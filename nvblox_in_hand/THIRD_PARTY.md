# 来源和许可证

本工程新增适配代码采用 Apache-2.0；官方源码的 NVIDIA 版权标头和原 LICENSE 保留。`upstream/` 来自本机 isaac_ros_nvblox，包含其 Apache-2.0 及 nvblox_core 自身许可证/第三方声明。根目录 LICENSE 为 Apache-2.0 文本副本。

机器人、UR/D435 mesh、Gazebo RealSense 插件和标定配置从原项目复制，具体来源及 BSD/Apache/其他资产声明仍见 `src/ur7e_d435_gazebo/THIRD_PARTY.md`、licenses/、third_party/。复制不改变原资产所有权或授权条款。

未复制任务识别、人物分割/检测模型及其示例工作流；共享 nvblox 算法框架中的通用类型保留，见 docs/ARCHITECTURE.md。

2026-11-04：config/nvblox/nvblox_base.yaml提取本机nvblox_examples_bringup静态参数，新增launch参考其组件编排结构；保留Apache-2.0声明，参考来源、哈希和删减键见provenance/example_refactor_20261104.json。新增适配文件不声称是官方原样示例。
