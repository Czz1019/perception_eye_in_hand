# 将perception_eye_in_hand共享到自己的GitHub：VS Code操作说明

日期：2026-11-09。本文根据当前本地Git状态整理，界面名称按VS Code常见英文/中文名称说明。本轮没有修改remote、索引或分支，没有执行commit、push或创建GitHub仓库。

## 1. 你的情况不需要原作者仓库管理权限

你可以保留克隆得到的提交历史与原作者信息，将本地修改提交后推送到自己有写入权限的GitHub仓库。只有向原作者仓库直接push才需要相应写入权限；同步原作者更新一般只需fetch/pull，贡献给原作者则可通过fork/PR流程。

本机现在已配置：

```text
origin   https://github.com/Czz1019/perception_eye_in_hand.git
upstream https://github.com/emmmwqj/perception_eye_in_hand.git
当前分支：main
跟踪分支：origin/main
```

**如果Czz1019是你的GitHub账号，这已经是“自己的仓库＋原作者参考仓库”的结构。** 接下来主要是整理、提交、推送，不必再次git init、不必删除.git、不必重新clone覆盖现有目录。

以上是本地配置事实，未联网验证该账号身份、远端当前分支状态或写权限。remote里有一个地址并不证明你当前登录的账号能推送。

| 名称 | 这里的含义 | 本次使用 |
| --- | --- | --- |
| origin | 你的发布仓库地址（需确认账号） | push目标 |
| upstream | 原作者仓库地址 | 以后按需fetch更新，不往这里直接push |
| main | 本地分支 | 当前提交所在分支 |
| origin/main | 本地记录的远端分支状态 | fetch后才刷新，不是实时联网状态 |

补充：remote名字upstream与“分支的upstream跟踪关系”是两个概念。当前main跟踪origin/main是正确的；不应因为名字相似把它改成跟踪原作者upstream/main。

## 2. 当前需要特别处理的内容

只读核查时：原仓库跟踪了约1240个build文件、1052个install文件和2171个log文件。旧build/install已在此前清理中删除，Git显示大量D，是等待提交的删除记录，并不表示源码丢失。

新增 `nvblox_in_hand`、`ur7e_nvblox_in_hand` 与不少说明文档仍是未跟踪文件，**不提交它们，推送不会自动把这些新代码带到GitHub**。

根.gitignore目前只列旧build/install相关四个目录；log/tmp还需整理。子工程各自已有忽略规则，但不能替根工作区自动排除所有临时文件。

检查到历史可达提交中有约69.85MiB的tmp/pptx/1105/source.json和54.49MiB的tmp/docx_speech/media2.mp4，当前未发现超过100MiB的历史blob。这些文件不属于感知运行源码。保留历史时，即使新提交不再跟踪它们，旧对象仍可能随历史传输。

官方对普通Git文件超过50MiB给出警告，超过100MiB阻止上传；更大的必要资产应使用LFS或Release等适当分发方式，见[GitHub大文件说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)。不要把“加入.gitignore”误认为自动清除了历史。

## 3. 在VS Code打开正确的目录并登录

1. 打开VS Code，选择“文件→打开文件夹”，打开 `/home/czz/perception_eye_in_hand`，不要只打开某个src子目录。
2. 按Ctrl+Shift+G打开“源代码管理 / Source Control”。若有多个仓库，确认当前选中perception_eye_in_hand。
3. 左下角应显示main。点击某个改动文件，查看左右diff；U为未跟踪、M为修改、D为删除。
4. 点击左下角“账户 / Accounts”，按提示登录GitHub；或在首次push时完成浏览器授权。登录你拥有目标仓库写权限的账号。
5. Ctrl+Shift+P运行“Git: Show Git Output”可查看Git执行日志；菜单名称随版本略有差异。

VS Code内置Git已能完成提交/推送，GitHub Pull Requests扩展可选，不是必需品。[官方GitHub接入说明](https://code.visualstudio.com/docs/sourcecontrol/github)

在“终端→新建终端”做只读核查：

```bash
cd /home/czz/perception_eye_in_hand
git remote -v
git branch -vv
git status --short
```

如果Commit提示身份未设置，可只对当前仓库设置提交作者（自行替换姓名/邮箱）：

```bash
git config user.name "你的姓名或GitHub用户名"
git config user.email "你的GitHub验证邮箱或noreply邮箱"
```

提交作者信息不等于GitHub登录凭据。

## 4. 先决定是否继续使用现有origin

### 路线A：Czz1019/perception_eye_in_hand就是你的仓库

推荐走此路线，跳到第5节。无需再创建仓库、添加remote或点击Initialize Repository。

### 路线B：确实要发布到另一个全新仓库

1. 在GitHub登录自己的账号，右上角“＋→New repository”。
2. Owner选择自己，填写新名称，例如 `ur7e-eye-in-hand-perception`。
3. 选择Public或Private。分享给所有人用Public；限定合作者可用Private并邀请协作者。可先Private整理，但私有可见性不代替第三方授权核对。
4. 因本地已有Git历史，创建**空仓库**：不要勾选初始化README、.gitignore或LICENSE，避免引入另一条初始历史。
5. 创建后复制HTTPS地址。
6. 在VS Code的命令面板找“Git: Add Remote”，输入新地址，remote名称使用 `personal`。某些版本可从源代码管理“⋯→Remote→Add Remote”进入。
7. 后续Push To明确选择personal，而不是upstream。此方式无需覆盖当前origin。

对应终端替代操作，只有选择路线B时才执行：

```bash
git remote add personal https://github.com/你的用户名/你的新仓库名.git
git remote -v
```

若personal已存在，应先核对其地址，不反复添加。不要把含用户名/仓库名占位词的URL原样执行。

### 为什么这里不推荐“Publish to GitHub”作为当前第一步

VS Code支持把未连接远端的本地仓库发布为新GitHub仓库；“Publish Branch”则将分支上传到已有远端。当前本地已经有origin/upstream，使用现有目标或新增personal更清晰，避免重复建仓库。[官方remote与发布说明](https://code.visualstudio.com/docs/sourcecontrol/repos-remotes)

若你希望GitHub明确显示“forked from原作者”，应先在原仓库网页点Fork到自己的账号，再把该fork地址作为自己的目标。单独New repository再push虽然可保留历史，但不会仅凭remote名称自动建立GitHub的fork关系。两条路线不必同时做。

## 5. 整理要共享的文件，保留可运行的完整源码

建议先在文件管理器做一份工程备份，包含未提交源码和.git；备份不要放在当前仓库内部。不要通过“丢弃所有更改”做清理。

### 5.1 需要提交

- 根目录src：原有功能包和本轮确实修改/新增的源码。
- `ur7e_nvblox_in_hand`：三个ROS包、native、vendor、配置、launch、rviz、工具、测试、许可证及provenance。
- `nvblox_in_hand`：如果共享完整项目，保留其src/upstream及依赖说明。
- 启动/编译脚本、README、docs及必要文档图片。
- URDF、mesh、world、标定说明等复现所需资产。不要因为是.dae/.stl/.png就一概忽略。

Python工程不能只提交node.py；缺native/vendor/接口包会导致其他人无法编译。不要把系统安装的Torch、CUDA、wheel或整个workspaces复制进来，依赖通过安装文档获取。

### 5.2 通常不提交

编译目录、install、log、.ros、Python缓存、临时PPT/视频、大型bag/地图/数据集。历史测试结论与精选截图放在docs，原始大数据另行分发。包含真实设备配置和历史记录时核对哪些信息适合公开；GitHub token、SSH私钥、密码和.env私密配置不要提交。

根.gitignore可在保留现有内容基础上补充，例如：

```gitignore
# Existing generated outputs
/build/
/install/
/build_native/
/install_native/
# Parent-workspace temporary outputs
/log/
/tmp/
/.ros/
__pycache__/
*.pyc
# Local credentials; use separately reviewed templates if needed
.env
```

不要直接忽略所有yaml或模型文件。根/artifacts是否保留应按文件用途审查，尤其已有文档引用的截图；不是看到artifacts就全部删除。子工程历史artifacts也要单独检查，只留对复现有价值的结果，避免把大量地图放进Git。

### 5.3 .gitignore不影响已经跟踪的文件

对于已经进入Git索引的旧生成物，需要从跟踪中移除。可在VS Code集成终端执行以下**仅移除Git跟踪、不删除本地文件**的操作；执行前确认路径确实都是生成目录：

```bash
git rm -r --cached --ignore-unmatch -- build install build_native install_native log tmp
```

随后VS Code的“已暂存的更改”会显示这些文件的删除，表示下一次提交不再携带它们；对于此前已清理的build/install，工作目录本来就没有。src与新工程源码不在该命令范围内。

`.gitignore`要一同提交，避免后续重新加入。**不要使用不带--cached的删除命令、不要删除.git，也不需要重新初始化仓库。** 本文没有替你执行上述命令。

### 5.4 保留来源与许可证

保留原作者提交历史、版权标头、包内LICENSE/THIRD_PARTY与资产许可，README注明原仓库链接及你的新增模块。当前根目录未发现统一LICENSE，不能据此假定所有原代码都允许任意再许可；发布/分发前核对各部分已有许可，缺失部分向作者确认，不要给整个混合仓库直接套一个新许可证。

新包的Apache声明不覆盖第三方ROS/模型/核心文件的各自条款。关于共享与许可的基本区别见[GitHub许可说明](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository)。

## 6. 用VS Code暂存和提交

1. 保存修改后的.gitignore与说明文档。
2. Ctrl+Shift+G，逐项检查Changes。重点确认新 `ur7e_nvblox_in_hand`、`nvblox_in_hand` 和docs都出现了。
3. 对要提交的文件点击右侧“＋ / Stage Changes”。可以按功能包分批暂存。
4. 前一步生成物取消跟踪的删除记录应保留在Staged Changes；确认没有误删src和模型。
5. 点击Staged Changes中的文件审查diff。不要仅因改动多就直接点“暂存所有”，否则会把未审查的视频、日志或本机配置一起加入。
6. 输入提交说明，例如：`Add standalone UR7e D435i mapping and deployment guides`。
7. 点击“提交 / Commit”。若询问“没有暂存的文件，是否自动暂存全部”，先取消并明确选择要提交内容。

提交只写入本地Git，不会自动上传GitHub。建议将“生成物取消跟踪”和“新感知模块与文档”分成两次清晰提交；二者都审核完再push也可以。

提交前可在集成终端核查：

```bash
git diff --cached --stat
git diff --cached --name-status
git diff --cached --check
```

如果文件显示被忽略却认为它是必需源码，使用 `git check-ignore -v 文件路径`定位规则；先修正规则，不盲目git add -f所有文件。

## 7. 用VS Code推送到自己的仓库

### 当前路线A：origin就是你的目标

1. 在源代码管理点“⋯→Pull, Push→Push”（菜单可能显示“推送”）。
2. 多remote时选择“Push To…”并明确选origin；不要选upstream。
3. 首次认证按浏览器提示授权GitHub，确认登录账号正确。
4. main已跟踪origin/main，普通Push即可。也可在核对后用状态栏同步按钮，但“Sync Changes”可能先pull再push，第一次迁移建议明确Push，便于确认发生了什么。
5. 若提示远端有新提交，先审查远端变更，再正常合并/解决冲突；不要直接force push。

终端等价命令（选择路线A且目标确认后）：

```bash
git push -u origin main
```

### 路线B：新增personal目标

用“Push To…”选personal并推送main；如果界面没有清晰设置跟踪分支，用集成终端完成首次设置：

```bash
git push -u personal main
```

此后main跟踪personal/main是这条路线的预期；origin仍保留原先地址。不要把两个路线的push命令都执行一遍，除非你明确要维护两个发布仓库。

Push只上传已提交的内容，不会上传Changes中尚未提交的文件。HTTPS不能用GitHub账号密码直接做Git认证，采用VS Code浏览器授权/凭据管理，或按需配置SSH；不要把token拼进remote URL。参考[GitHub认证说明](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/about-authentication-to-github)。

## 8. 在GitHub上确认共享是否完整

浏览器打开目标仓库，检查：

1. 分支为刚推送的main，最新提交说明与VS Code一致。
2. `ur7e_nvblox_in_hand/src`、native、vendor、docs、build.sh都存在。
3. README中能点到迁移说明、实机测试和算法文档，图片能打开。
4. 不出现新生成的build/install/log；如果保留历史，旧提交里仍能看到以前的产物是正常的。
5. 仓库可见性符合预期。公开仓库可直接分享链接；Private需从Settings→Collaborators等入口邀请合作者。

建议README写明：

```text
项目来源：emmmwqj/perception_eye_in_hand
主要新增：独立UR7e/D435i感知工程、原生ESDF导出、运行/迁移/测试文档
推荐入口：ur7e_nvblox_in_hand/README.md
验证环境：Ubuntu22.04、ROS2 Humble、NVIDIA GPU及固定Torch/nvBlox wheel
测试边界：明确区分仿真结果、历史旧模块实机结果及尚未验收的部分
许可证：按原项目与第三方组件说明保留
```

为新clone用户提供相对路径入口和依赖安装方法，不只写/home/czz或/home/wqj本机绝对路径。实机IP和标定可以作为历史示例保留，但需要注明用户必须按自己的设备配置。

## 9. 建议做一次新的clone验收

push成功后，用VS Code“Git: Clone”把自己的仓库克隆到另一个空目录，再检查能否按部署文档找到所有源文件、许可证与配置。不要覆盖当前开发目录。

示例（仅路线A且确认仓库是你的时候）：

```bash
mkdir -p "$HOME/github_share_check"
git clone https://github.com/Czz1019/perception_eye_in_hand.git \
  "$HOME/github_share_check/perception_eye_in_hand"
```

本工程build默认架构89，对其他GPU用户要说明如何调整。源码clone成功不等于无依赖可运行；需按 `ur7e_nvblox_in_hand/docs/MIGRATION_DEPLOYMENT_20261109.md` 安装系统/库依赖并编译。

## 10. 常见问题

| 现象 | 处理 |
| --- | --- |
| 403 / Permission denied | 核对push目标和登录账号；不要向原作者upstream推送 |
| Repository not found | URL拼写、仓库是否创建、私有仓库访问权限 |
| remote origin already exists | 当前已配置，不重复add；确需换目标时审查后set-url，或新增personal |
| VS Code没有Publish按钮 | 已有remote属于正常；用Push/Push To |
| Everything up-to-date但网页无新代码 | 新文件可能没stage/commit，或推到其他remote/分支 |
| non-fast-forward / rejected | 远端含额外提交；fetch后审查并合并，不默认强推 |
| 大量D | 此前删除了受跟踪生成物；区分源码与产物后提交删除记录 |
| ignore后日志仍在Changes | 已被Git跟踪，按--cached取消跟踪 |
| GitHub有旧编译内容 | 新提交删除不清历史；只为首次共享无需立刻重写历史 |
| 大文件阻止push | 先定位文件是否进入提交/历史；不要只在工作目录删除后反复push |
| .git对象较大、上传慢 | 旧历史/大文件仍在；源码快照新仓库是另一方案，但会丢原提交历史，不建议未备份就重做当前仓库 |
| 克隆后缺机械臂模型或bridge | 暂存漏了mesh/native/vendor/接口包，补齐源码后再提交 |

## 11. 针对当前仓库最短操作路线

确认Czz1019为你的账号与origin目标→VS Code打开工程根目录→审核忽略规则→取消跟踪旧生成物→暂存新包和文档→Commit→Push To origin→GitHub检查→另目录clone验收。

不需要原作者管理员权限，不需要删除本地旧源码，不需要重装Git或重新创建.git。本文只生成操作文档；所有Git修改与远端发布仍由你按上述步骤操作。
