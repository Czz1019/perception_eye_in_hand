# 旧编译产物清理说明

日期：2026-11-09。

本次只删除父工作区 `/home/czz/perception_eye_in_hand` 下的四个生成目录：`build/`、`install/`、`build_native/`、`install_native/`。它们不是当前 `nvblox_in_hand/run_esdf.sh` 测试入口使用的构建与安装目录。

保留父工程 `src/`、全部构建与启动脚本、历史日志、测试资料，以及新工程 `nvblox_in_hand/` 的源码、官方源码副本、构建和安装目录。`/home/czz/workspaces/isaac_ros-dev` 中的官方依赖也保留。

清理前检查当前新工程的包解析与动态库路径；没有进程正在执行或映射待清理目录中的程序、共享库。对父工程源码、新工程源码、upstream及新工程install记录内容哈希和符号链接，删除后再次比较，确认内容一致。证据见 `../provenance/build_cleanup_20261109.json`。本次不启动仿真，也不修改感知实现或测试参数。

当前启动入口保持为：

清理后验证通过：nvblox_in_hand与ur7e_d435_gazebo仍解析到新工程install；nvblox_ros解析到workspaces官方install；mapper_node及RealSense Gazebo插件动态库无缺失，且未引用删除的四个目录。`./run_esdf.sh --show-args`解析成功。这是依赖及启动参数检查，未进行仿真重测。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./run_esdf.sh gazebo_gui:=true
```

旧入口依赖的安装文件已经清除，使用旧入口前需要执行对应旧构建脚本重新生成。新工程依旧使用自身install，并依赖workspaces中的官方安装环境。

父仓库新增.gitignore，忽略上述四个目录未来生成的未跟踪文件。此前已经被Git跟踪的编译产物，在本次删除后会显示为删除变更，需要随清理提交审阅；忽略规则不会自动取消已有跟踪。本次没有执行git add、commit、push，也没有更改Git索引。
