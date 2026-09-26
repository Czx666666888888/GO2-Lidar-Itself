# GO2-Lidar-Itself 协作规则

本文件定义 ChatGPT 网页版、Codex 与 GitHub 在本项目中的长期协作边界。GitHub 仓库是双方共享的项目状态源；任何结论都应能追溯到仓库文件、明确的测试输出或已标注来源的实验记录。

## 项目角色

### ChatGPT 网页版

负责：

- 问题定义与目标澄清
- 系统方案与技术路线
- 任务拆解与优先级
- 科研方法与实验设计
- 审查 Codex 的实现和验证证据
- 决定下一阶段工作

### Codex

负责：

- 阅读仓库并分析现有代码
- 实现边界明确的任务
- 调试、测试和 Git 操作
- 更新工程文档与项目状态
- 将未验证事实、风险和阻塞明确反馈给 ChatGPT

Codex 不应替代 ChatGPT 擅自改变系统目标、研究路线或整体架构。

## 每次任务开始前

Codex 必须按顺序优先阅读：

1. `AGENTS.md`
2. `PROJECT_STATUS.md`
3. `TASKS.md`
4. 与当前任务相关的 `docs/`
5. 与当前任务相关的源码、配置、launch 和脚本

不得仅根据单次 prompt 直接修改代码。开始修改前应检查 `git status`、当前分支和最近提交，避免覆盖用户已有改动。

## 修改原则

- 先理解现有实现、数据流和安全边界，再修改。
- 使用最小侵入改动，避免无关重构。
- 尽量保持现有接口和兼容性。
- 不随意修改 ROS topic、frame、service、action、消息类型或 QoS。
- 不虚构实验结果，不把源码存在写成运行验证通过。
- 不把成功 build、节点启动、DDS 发现、端口打开或安全门 ARMED 单独当作真机运动有效的证据。
- 发现架构级问题时，先记录到 `PROJECT_STATUS.md`、`TASKS.md` 或 `docs/decisions.md`，交由 ChatGPT 决策；不得擅自大规模重构。
- 保留第三方代码的版权、许可证和来源说明。

涉及真实 GO2 时，默认采用只读检查、离线测试或 `--dry-run --no-arm`。发送真实运动命令必须得到用户明确授权，并具备物理急停、空旷环境和可确认的停止路径。

## 验证原则

每次代码修改后，应根据改动范围尽可能执行：

- build
- lint
- unit test
- integration test
- launch / dry-run
- 现有项目测试脚本

报告必须区分：静态检查、构建成功、节点启动、话题存在、收到连续数据、离线回放、真机执行。

如果因缺少 GO2、UTLiDAR、rosbag、网络环境或真实硬件无法验证，必须标记：

`NOT VERIFIED`

不得改写成“已经正常工作”。失败、跳过和待验证项应给出具体原因。

## Git 原则

原则上：

一个 ChatGPT 明确任务 = 一个 Codex implementation cycle = 一个逻辑清晰的 Git commit。

- 不在一个任务中混入无关修改。
- 提交前检查 `git diff`、`git status` 和敏感信息。
- 提交信息应描述本轮逻辑目标。
- 未经明确要求，不重写公共历史、不强推、不删除远程分支。
- push 后核对本地与远程 commit hash。

## 文档原则

每轮任务结束前必须判断是否需要同步更新：

- `PROJECT_STATUS.md`：当前可确认状态、风险和验证边界
- `TASKS.md`：任务完成情况、阻塞和下一项工作
- `docs/architecture.md`：已确认的数据流、模块和接口
- `docs/decisions.md`：已决定且有证据的架构选择
- `docs/experiments.md`：实验配置、观测、原始数据位置和证据等级

文档中的事实应注明来自源码、README、测试输出还是实机记录。无法确认的内容使用 `Unknown / needs verification` 或 `TODO: verify`。
