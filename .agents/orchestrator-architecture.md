# SiFli SDK 多代理软件开发架构设计

> 状态：Phase 1 核心派发路径在 DeepSeek Harness 与 Claude Code 两侧均已验证，
> 证据与未实测项见附录 C 第 4–6 条。
> 适用范围：本仓库的 DSH 会话；同时兼容 Claude Code 的角色文件格式。
> 相关文件：[AGENTS.md](../AGENTS.md)（调度规则）、[.claude/agents/](../.claude/agents/)（角色定义）、
> [skills/sifli-code-review/](../skills/sifli-code-review/)（审查规则）。

## 0. 快速开始

不需要额外命令：在本仓库的会话里直接提需求，主会话按 `AGENTS.md` 自行判断是否派发。

- 想强制某个角色：「用 Researcher 先查清楚 X」「让 Reviewer 审一下这次改动」
  —— 主会话按 `AGENTS.md` 的 "Harness adapter" 启动该角色：Claude Code 直接按注册名派发，
  DSH 读取 `.claude/agents/<role>.md` 并内联正文。
- 想看审查规则：`skill` 工具加载 `sifli-code-review`，或直接读
  `skills/sifli-code-review/SKILL.md`（本工作区暂不拆分 `references/`）。
- 想调整调度：改 `AGENTS.md` 的派发表；想调整角色行为：改对应的角色文件。
- 本设计只落在**工作区**内：所有文件都在仓库内，不新增 `~/.dsh` 或 profile 级配置
  （该 profile 已有的 bundle 仍会生效）。


## 1. 设计目标

为了提高 AI Agent 在 SiFli SDK 开发中的效率、代码质量与任务可靠性，采用
**Orchestrator + Specialized Agents** 的多代理协作模式：

- **Orchestrator**：理解需求、制定计划、拆分任务、调度子代理、集成结果、最终验收。
- **Specialized Agents**：按专业领域负责调查、设计、实现、测试、审查和文档。

设计原则：

1. **职责分离**：每个代理有明确边界，不越界做别人的事。
2. **按需调度**：不是每个任务都启动全部代理；简单任务主会话直接完成。
3. **专业化**：不同领域（固件/构建/工具）使用不同的提示词与检查清单。
4. **并行执行**：相互独立的任务并行，有依赖的串行。
5. **独立审查**：实现与审查必须是不同代理、不同上下文。
6. **统一协调**：所有子代理由 Orchestrator 派发，最终结果由 Orchestrator 验收。

非目标：不替代 CI 的机械检查，不替代人工对安全/兼容性/预算的风险决策。

## 2. 总体架构

采用**固定核心代理 + 按需启用专业代理**：角色全部预先定义，但只在需要时启动。

```
                         User
                           |
                           v
                  +------------------+
                  |   Orchestrator   |   主会话（不另建子代理）
                  | 需求分析 / 规划   |
                  | 任务拆分 / 调度   |
                  | 结果集成 / 验收   |
                  +------------------+
                           |
              +------------+------------+
              |            |            |
              v            v            v
         Researcher    Developer     Reviewer      ← 核心代理，默认可用
              |            |            |
              +------------+------------+
                           |
                    按需启用的专业代理
             +-------------+-------------+
             |             |             |
             v             v             v
          Architect      Tester     Security Reviewer
                                         |
                                  Documentation Engineer
```

### 2.1 代理分类

| 类型 | 代理 | 说明 | 定义文件 |
|---|---|---|---|
| 主代理 | Orchestrator | 每次开发任务的统一协调者 | `AGENTS.md`（本仓库根） |
| 核心代理 | Researcher | 代码调查与技术信息收集（只读） | `.claude/agents/researcher.md` |
| 核心代理 | Developer | 代码实现（Embedded / Build-Tooling 两种模式） | `.claude/agents/developer.md` |
| 核心代理 | Reviewer | 独立代码审查（只读，全新上下文） | `.claude/agents/reviewer.md` |
| 专业代理 | Architect | 技术方案与架构设计 | `.claude/agents/architect.md` |
| 专业代理 | Tester | 测试设计与执行 | `.claude/agents/tester.md` |
| 专业代理 | Security Reviewer | 安全专项审查（只读） | `.claude/agents/security-reviewer.md` |
| 专业代理 | Documentation Engineer | 文档与示例维护 | `.claude/agents/documentation-engineer.md` |

### 2.2 DeepSeek Harness 运行模型

DSH 没有 Claude Code 的 `Agent` 子代理工具与 `.claude/agents` 自动注册表，对应关系如下：

| 方案概念 | Claude Code 做法 | 本仓库 DSH 实现 |
|---|---|---|
| Orchestrator | 主会话 + `CLAUDE.md` | 主会话 + 根 `AGENTS.md` 的 "Multi-Agent Collaboration (Orchestrator)" 章节 |
| 子代理定义 | `.claude/agents/*.md` 自动注册 | 同一目录存放 markdown；Orchestrator 读取正文作为子代理提示词 |
| 启动子代理 | `Agent(subagent_type=...)` | `subagent` 工具（全新上下文） |
| 继承上下文 | `Agent(subagent_type="fork")` | `subagent_fork`（继承已完成轮次，不含当前进行中的轮） |
| 继续/追问 | `SendMessage`（同一 agent id） | `send_message` 继续同一子代理 |
| 并行 | 一条消息多个 `Agent` 调用 | 一条消息多个 `subagent`；大规模同质扇出且用户要求或同意时才用 `workflow` |
| 技能 | `.claude/skills`（Claude Code 默认位置）与仓库根 `skills/`（本仓实测直接可见） | `.dsh/skills/<name>/SKILL.md`（DSH 自动发现，已验证）+ `skills/` 规则正文 |
| 工具权限 | frontmatter `tools:` | 模型侧无参数；`persona`/`toolFilter` 由 profile 级 Agent Preset 提供，当前未启用 |
| 模型选择 | frontmatter `model:` | `subagent` 的 `provider`/`model` 参数（可选） |

**实现要点**：

- 在 DSH 里，子代理是"一段提示词 + 一个独立上下文"，不是可注册的实体。因此**角色文件必须被
  内联**进 `subagent` 的 `prompt`，不能假设子代理会自动读取 `.claude/agents/`；Claude Code
  则相反，注册后只传任务即可。
- 两个 harness 都会给子代理加载同一份工作区指令（`AGENTS.md`；DSH 已验证子代理能复述其标题与本节，
  Claude Code 见附录 C）。因此 `AGENTS.md` 明确写了：**被派发的一方不再委派**，避免递归。
- 委派深度有上限（会话配置），子代理不应再调用 `subagent`。
- `.claude/agents/*.md` 的 frontmatter 采用 Claude Code 兼容格式，Claude Code 可直接复用；
  DSH 侧只把正文当提示词，忽略 `tools`/`model` 字段。
- 两侧的机制差异集中在根 `AGENTS.md` 的 "Harness adapter" 表里；角色文件**正文**不写工具名——
  同一份文件在 Claude Code 是自动注册的系统提示，在 DSH 是被内联的用户消息；frontmatter 的
  `tools:` 只供 Claude Code 注册使用，DSH 忽略。
- Claude Code 的角色定义在会话启动时加载，改动后如果看起来没生效，先开新会话再判断；DSH 每次
  派发现读，改完立即生效。

**子代理上下文的实际构成**（DSH 对子会话转录的实测，按到达顺序；Claude Code 下角色定义在系统
提示里，不在用户消息里）：

| 序号 | 内容 | 来源 | 权限层级 |
|---|---|---|---|
| 1 | 通用系统提示词 | DSH 内置，不入转录 | system（最高） |
| 2 | 角色定义 + 任务 | `subagent` 的 `prompt` 参数，成为首条 `user/message` | user |
| 3 | 工作区指令（`AGENTS.md`） | 指令插件注入的 `<system-reminder>` | user |
| 4 | 运行时上下文（cwd、时间、深度） | 宿主注入 | user |
| 5 | 技能目录 | 技能插件注入 | user |

由此有两条必须记住的结论：

1. 角色提示词在 DSH 里是 **user 级消息**，不是 system prompt。它可以被直接用户指令覆盖，
   也不如 system 约束"硬"——所以"只读"这类约束要短、要显眼，并由 Orchestrator 事后查 diff 兜底。
2. 子代理天然带着 `AGENTS.md`，所以角色提示词里不需要重复项目概览，只需要写专业规则。

### 2.3 为什么用"内联提示词"而不是插件化代理

DSH 的 Agent Preset 是**插件组合**（Cordis YAML，profile 级），用于给子代理换工具集/人格，
不是"按任务类型选角色提示词"。对本需求：

- 角色是**任务级**的（同一个 Developer 一会儿写驱动、一会儿改 SCons）；
- 角色文件需要**跟仓库一起版本化**、可 review、可 diff；
- 不能引入 profile 级配置（要求"只在当前工作区生效"）。

因此选择：**markdown 角色文件 + Orchestrator 内联组装**。代价是每次多读一个文件（~2–6 KB），
收益是零配置、可移植、可审查。

## 3. 角色职责

每个角色文件的完整提示词在 `.claude/agents/`，这里只记录契约（输入/输出/边界）。

### 3.1 Orchestrator

由主会话承担，**不额外创建 Orchestrator 子代理**。规则写在根 `AGENTS.md`。

职责：

- 理解需求与预期交付物，判断复杂度与涉及领域。
- 决定是否需要调查、是否需要方案设计。
- 拆分任务，明确每个子任务的输入、输出、边界。
- 选择子代理，决定 fresh 还是 fork，决定并行还是串行。
- 集成结果，解决代理间冲突。
- 组织修复回路，控制轮次。
- **最终验收**并对用户报告。

原则：

1. 简单任务直接做，不派发。
2. 复杂任务先计划，再派发。
3. 不为并行而拆强依赖任务。
4. 不把验收责任交给子代理。
5. 子代理说"完成"不等于任务完成。

### 3.2 Researcher（只读）

- 输入：问题、已知线索、范围、期望产出。
- 输出：结论、关键位置（`path:line`）、调用链、配置依赖、疑点、未确认项。
- 边界：不改代码、不写方案、不猜 API。

### 3.3 Developer

同一角色，两种模式：

| 模式 | 触发 | 关注 |
|---|---|---|
| Embedded | `drivers/`、`rtos/`、`middleware/`、`customer/`、`example/` 的 C/H | 分层、并发、资源、时序、Kconfig、多芯片 |
| Build/Tooling | `SConscript`、`SConstruct`、`Kconfig*`、`tools/`、`tests/` | 可复现构建、配置优先级、Python 兼容与可诊断 |

- 输入：任务与验收标准、方案要点、涉及/禁止触碰的文件、可用构建板型。
- 输出：改动清单、行为变化、构建结果、未验证项、风险。
- 边界：最小改动、不顺手重构/格式化、不改公共 API（除非任务要求）、不 commit。

后续可扩展 Graphics Developer、Connectivity Developer，只需新增角色文件并在
`AGENTS.md` 的派发表中登记。

### 3.4 Reviewer（只读，必须全新上下文）

- 输入：diff 范围、变更目的、关联需求。
- **必读**：`skills/sifli-code-review/SKILL.md`（本仓唯一规则正文）：Step 0–2 通用流程，
  再按文件类型追加 Step 3（例程 README）/ Step 3b（`docs/`，同口径用于 `.agents/`、`AGENTS.md`
  等 agent 文档）/ Step 4（例程代码）/ Step 5（提交说明）/ Step 6（测试资产）。报告骨架见
  `.claude/agents/reviewer.md` 的「输出格式」，严重度与评分口径见 Step 7。
- DSH 会话中也可用 `skill` 工具加载 `sifli-code-review`；`.dsh/skills/` 下只有入口壳，正文仍指向 `skills/`。
- 输出：`verdict`、带严重度（严重/问题/建议/OK）与 `path:line` 的发现、待确认、需人工复核、已检查项。
- 边界：不改代码；不信任实现者自述；无法验证的硬件行为标"待确认"。

### 3.5 Architect

- 启用：跨 ≥3 模块、跨层、公共 API/格式变更、构建配置重构、需要方案对比。
- 输出：≥2 个方案与推荐、分层与接口、文件范围、兼容性、扩展性判定、实施步骤、风险、验证计划。
- 边界：只读分析，不实现；方案必须落到文件与接口。

### 3.6 Tester

- 级别：只有单元测试与端到端（E2E）两级，按被测范围判定；载体（宿主/QEMU/真板）与级别正交。
- 覆盖：每个真实接缝至少一条用真实一侧验证的测试；高风险接缝加负向用例。
- 铁律：编译成功 ≠ 硬件正确；单元测试通过 ≠ 集成正确；必须区分"已执行"与"未执行"。
- 反向验证：新用例必须给证据（每个用例至少一次破坏）；改既有断言时，破坏点必须落在被改的断言上。
- 无执行环境时：仍要输出测试计划、失败模式/接缝、未验证清单与原因，不得以"没板"跳过测试设计。
- 允许写测试代码；不改产品代码。

### 3.7 Security Reviewer

- 触发：TrustZone/安全域、Secure/Non-secure 接口、PSA、Flash/密钥、OTA/DFU、网络与外部输入解析。
- 输出：攻击面前提、影响、触发路径、修复建议、置信度。
- 边界：只读；不构造利用代码；无可达路径的理论风险标"待确认"。

### 3.8 Documentation Engineer

- 触发：公开 API、Kconfig、构建流程、工具命令、例程变化。
- 输出：文档变更表、中英一致性、芯片范围一致性、命令可执行性、未覆盖项。
- 边界：不改代码逻辑；只改受影响章节。

### 3.9 权限模型（与 Claude Code 的差异）

DSH 的 `subagent` 工具**不接受** `tools:` 或权限参数；工具集来自子代理运行时的组合：
`persona` 与 `toolFilter { allow, deny }` 是 `subagent/descriptor` 的正式字段，但只能由
**Agent Preset**（profile 级 Cordis 插件组合）提供，模型不能按次指定。

当前工作区没有配置自定义 preset，所有子代理继承父会话的 `standard`，因此：

- "只读"靠角色提示词 + Orchestrator 约束（不派发写任务给 Researcher/Reviewer），属于**约定**而非强制。
- 写操作仍受会话 sandbox 策略约束（当前 workspace-write / 全访问由会话决定）。
- 高风险操作（`git reset`、删除、发布）必须由 Orchestrator 在执行前确认。
- 若将来接受 profile 级配置，可建一个 `readonly` preset（`toolFilter.deny: ["write","edit"]`），
  在 profile 侧把 Reviewer 的派发绑定到它（模型侧没有 `agentPreset` 参数，`subagent` 也无此字段）
  ——这是唯一能做到"硬只读"的路径，角色文本无需改动。

Claude Code 侧不同：`.claude/agents/*.md` 的 `tools:` 会被强制执行——`architect` 只有
Read/Grep/Glob，是硬只读；Researcher/Reviewer/Security Reviewer 带 `Bash`（审查需要
`git diff`/`git show`），所以仍是"软只读"，角色文件约束命令行只用于只读操作。

## 4. 项目文件组织

```
SiFli-SDK/
├── AGENTS.md                         # Orchestrator 规则（DSH 自动加载）
├── .agents/
│   └── orchestrator-architecture.md  # 本文档（多代理方案设计，agent 通用可读目录）
├── .claude/
│   ├── CLAUDE.md                     # 指向 AGENTS.md 的兼容入口（Claude Code）
│   └── agents/                       # 角色定义（提示词库）
│       ├── researcher.md
│       ├── developer.md
│       ├── reviewer.md
│       ├── architect.md
│       ├── tester.md
│       ├── security-reviewer.md
│       └── documentation-engineer.md
├── .dsh/
│   └── skills/
│       └── sifli-code-review/
│           └── SKILL.md              # DSH 技能入口，指向 skills/ 规则正文
└── skills/
    └── sifli-code-review/
        └── SKILL.md                  # 审查规则正文（唯一真源；本工作区暂不拆分 references/）
```

### 4.1 为什么角色文件放在 `.claude/agents/`

- Claude Code 原生读取该目录，同一套文件两边可用；
- 与本仓库既有 `.claude/` 约定一致；
- DSH 侧不依赖该目录的自动加载，只当普通 markdown 读取；Claude Code 侧则原生自动注册。

### 4.2 AGENTS.md 与角色文件的职责边界

- `AGENTS.md`：**只放调度规则**（何时派发、派给谁、怎么组装提示词、验收口径）。
- 角色文件：**只放该角色的专业规则**（步骤、检查点、输出格式）。
- 公共规范（编码风格、构建命令、审查分级）不重复写在每个角色里，用路径引用。

### 4.3 为什么设计文档放在 `.agents/`

- `.agents/` 作为 agent 文档的约定目录，人和 agent 都容易直接找到方案文档；注意 DSH 只自动加载
  `AGENTS.md` 与 `.dsh/skills`，本文件需要显式 `read`，不要指望被自动注入；
- 与角色定义、技能入口同属"agent 可见区"，改方案时不用跨目录找；
- 代价：不在 Sphinx 文档树内——本文档面向 agent 而不是最终用户，不需要进文档站。

## 5. Orchestrator 调度策略

### 5.1 判定表

| 任务特征 | 派发 |
|---|---|
| 单文件、≤ ~20 行、不涉及公共 API/配置 | 主会话直接做 |
| 需要定位实现、梳理调用链、确认根因 | Researcher |
| 已确定方案，需要写代码 | Developer |
| 任意非平凡代码改动（> ~20 行或跨文件） | Reviewer（fresh） |
| 跨 ≥3 模块、跨层、公共接口/格式变更 | Architect → Developer → Reviewer |
| 有可执行测试环境或需要测试设计 | Tester |
| 命中安全触发条件 | Security Reviewer |
| 公开 API/配置/流程/例程变化 | Documentation Engineer |
| 几十个同质、可枚举的独立子任务（用户明确要求或同意） | `workflow` 脚本编排；少量任务用一条消息多个 `subagent` |

### 5.2 简单任务

```
User → Orchestrator → 直接读改 → 必要验证 → 完成
```

例：改一个宏、修局部逻辑、改注释、修文档错字。不派发，避免额外上下文成本。

### 5.3 普通开发任务

```
Orchestrator → Researcher → Orchestrator 定方案 → Developer
            → Orchestrator 集成/编译 → Reviewer → 修复 → 验收
```

例：给现有驱动加功能、改初始化流程、加一个配置项。

### 5.4 复杂开发任务

```
Orchestrator
   → Researcher
   → Architect（方案与文件范围）
   → 拆分：Developer A / Developer B / Tester(测试设计)
   → 集成与编译
   → Tester（执行）
   → Reviewer（必要时 Security Reviewer）
   → 修复 → 复审 → 验收
```

Tester 可以在开发前参与测试设计，也可以在实现后执行；不绑定单一阶段。
涉及公开接口/配置/流程时追加 Documentation Engineer。

### 5.5 提示词组装（Orchestrator 的固定动作）

派发前必须：

1. 判断 harness 是否自动注册角色（见 §2.2）：Claude Code **是**，DSH **否**。
2. 自动注册时按注册名派发，**不要把正文内联**——重复定义，而且退化成通用代理时会丢掉
   `tools:` 限制。
3. 未注册时 `read` 对应的 `.claude/agents/<role>.md`，把**正文原样**放到 `prompt` 开头。
4. 追加任务、上下文、验收标准、禁止项。

模板（`<role file 正文，原样>` 只在角色未被注册的 harness 里需要；Claude Code 用注册名派发时
删掉这一段）：

```text
你是 SiFli SDK 的 <Role>。严格按以下角色定义工作。

<role file 正文，原样>

## 任务
<一句话目标 + 可验证的完成标准>

## 上下文
- 相关文件：<path:line>
- 已知结论：<Researcher/Architect 的关键结论>
- 约束：<不许改什么、必须用什么机制、构建板型>
- Tester 证据：<级别/接缝表、反向验证表；Tester 跑过时必填>

## 验收
<Orchestrator 将如何检查：编译、测试、diff 范围、输出格式>

## 边界
- 不调用子代理。
- 超出任务范围时停止并报告，不要自行扩大。
```

组装要求：

- **自包含**：子代理看不到主会话历史（除非 fork），路径、板型、目标都必须写清。
- **可验证**：验收标准写成"运行什么命令 / 检查什么文件"，不写"做好一点"。
- **有边界**：明确禁止项，防止顺手重构。

### 5.6 何时不派发

- 派发成本（读角色文件 + 子代理上下文）大于任务本身；
- 需要主会话连续判断的探索性工作；
- 强依赖当前对话细节，且 `subagent_fork` 也无法提供（当前进行中的轮次）。

## 6. 任务拆分与并行执行规则

拆分时逐条检查：

1. 每个子任务有明确目标与交付物。
2. 每个子任务有明确输入（文件、结论、约束）。
3. **尽量减少多个代理修改同一文件**。
4. 明确依赖关系；有依赖的串行。
5. 独立任务并行；一条消息内启动多个 `subagent`，然后继续做别的事。
6. 子任务失败时不静默跳过，报告给 Orchestrator 决定重派或降级。

### 6.1 并行与写冲突

- 并行子任务若触及同一文件，改为串行，或由 Orchestrator 重新划分边界。
- 子代理的写操作在同一工作区立即可见，没有文件锁；Orchestrator 必须在集成时
  检查 `git status`/`git diff`，确认没有互相覆盖。
- Claude Code 侧可对写任务使用 `isolation: "worktree"` 做隔离；DSH 侧没有该能力，只能串行（Phase 4）。

### 6.2 用 workflow 编排大规模同质扇出

`workflow` 适合"同构、可枚举"的**大批量**扇出（几十个文件/模块级审计、批量迁移），
**且仅在用户明确要求或同意时使用**；它运行 JS 脚本、按阶段并发调用子代理、返回结构化结果。
三五个小任务用一条消息发多个 `subagent` 更省：`workflow` 的脚本、阶段与结果复核是额外成本。

约束：

- 只用于**互相独立**的任务；有依赖的用主会话顺序派发。
- 脚本里的每个 `agent()` 提示词同样要内联角色定义。
- 结果需要 Orchestrator 复核，不直接把脚本结果当最终结论。

## 7. 代码审查与返工机制

### 7.1 审查必须独立

- Reviewer 在**全新上下文**中启动，不继承实现过程的历史（Claude Code 用
  `Agent(subagent_type="reviewer")`；DSH 用 `subagent`，并需按 §5.5 内联角色正文）。
- 不把 Developer 的自述作为事实；先读代码。
- Reviewer 只读，不修代码；修复由 Orchestrator 决定后派回 Developer。

### 7.2 严重度映射

本仓库审查规则用四级，与常见 Critical/Major/Minor/Suggestion 对应：

| 本仓库 | 等价 | 合并影响 |
|---|---|---|
| 严重 | Critical | 阻塞；必须修复或人工显式豁免 |
| 问题 | Major | 需回应：修复，或说明为何不改 |
| 建议 | Minor / Suggestion | 不阻塞，可延后 |
| 待确认 | — | 不计入问题数，需补信息 |

`skills/sifli-code-review/SKILL.md` Step 7 用「严重 / 问题 / 建议 / OK」四档；"待确认"是代理层
为证据不足的条目增设的标签（不计入问题数），不属于技能正文。
Security Reviewer 自带 Critical/High/Medium/Low：Critical、High → 严重，Medium → 问题，
Low → 建议，合并 verdict 时按此折算（AGENTS.md 的 Review gate 也写了同一映射）。

### 7.3 返工回路

```
Developer 完成 → Orchestrator 检查 diff/编译 → Reviewer
    → 有"严重"或必须处理的"问题"？
        是 → Developer 修复 → 重新编译 → 同一 Reviewer 复核
        否 → 验收
```

- 最多 **2 轮**修复；第 2 轮后仍有"严重"，停下来向用户报告冲突与选项。
- 不为"报告里没有建议"而无限返工；"建议"级不进入回路。
- 复审优先用同一 Reviewer（它知道要验证什么）；最终高风险变更可再开 fresh Reviewer 签收。

## 8. 最终验收标准

Orchestrator 在报告完成前逐项检查：

### 代码

- 满足需求与验收标准。
- 遵循 `.clang-format` 与 `docs/source/en/contribute/coding_style.md`。
- 无不必要修改（格式化、重命名、无关文件）。
- 公共 API/配置/格式兼容性已确认。

### 构建

- 执行了受影响项目/板型的编译，记录命令与结果。
- 覆盖相关芯片系列与关键配置组合；未覆盖的明确列出。
- 无新增告警/链接问题。

### 测试

- 执行了可执行的测试（`pytest tests/` 或 `python -m unittest discover ...`、交叉编译）。
- 本仓没有可用的 cppcheck / CodeChecker，静态分析不作为基线，也不允许当作"已检查"的证据。
- 区分"已执行"与"未执行"；不用编译通过冒充功能正确。
- 未验证场景有清单与原因。

### 文档

- 公开 API/Kconfig/流程/例程变化已同步文档与示例。
- 中英文一致（如适用）。

### 交付报告

向用户说明：

1. 实现内容；
2. 主要修改文件；
3. 重要技术决策（与方案的差异）；
4. 已执行的构建与测试（命令 + 结果）；
5. 未执行的验证及原因；
6. 已知问题与限制；
7. 审查结论（verdict + 问题数）。

## 9. 实施计划

### Phase 1：基础架构（已完成）

- 根 `AGENTS.md` 调度规则。
- Researcher、Developer、Reviewer 角色文件。
- `sifli-code-review` 在 DSH 中可发现（`.dsh/skills`；已验证：技能目录出现该条目）。
- 验证：技能可加载；角色提示词可驱动子代理并按格式输出。

### Phase 2：专业化分工（部分完成）

- 已定义 Architect、Tester 角色。
- 待做：Embedded/Build 开发模式的提示词细化；按板型沉淀构建矩阵；
  为常见任务积累"派发样例"。

### Phase 3：扩展专业能力

- 已定义 Security Reviewer、Documentation Engineer。
- 待做：按需增加领域 Developer（Graphics、Connectivity），在 `AGENTS.md` 派发表登记。

### Phase 4：协作效率

- 多任务并行与写作用域管理。
- `git worktree` 隔离（DSH 支持 worktree 工具时）。
- Token/成本控制：角色文件瘦身、按需加载 references。
- 构建与测试自动化（把常用编译/测试命令固化为脚本，减少子代理试错）。
- 多芯片/多 SDK 版本适配。

## 10. 总结

本架构是**一个 Orchestrator 主会话 + 三个核心代理 + 四个按需代理**：

- Orchestrator 负责规划、协调、验收；
- Researcher 负责事实，Developer 负责实现，Reviewer 负责独立审查；
- Architect、Tester、Security Reviewer、Documentation Engineer 按需启用。

所有角色预先定义但不必每次运行。架构以任务为中心，不以代理数量为中心；
通过职责边界、任务拆分、有限并行与独立审查，提高可靠性和效率。

---

## 附录 A：角色文件契约

每个角色文件建议包含以下要素（标题可按角色调整，例如 Reviewer 用"独立性要求/第一步…"，
Security Reviewer 用"检查清单"）：

```markdown
---
name: <kebab-case>
description: <一句话：做什么、何时用>
tools: <Claude Code 工具列表，DSH 忽略>
model: inherit
---

# <Role> — <中文名>

## 角色边界        # 能做什么、不能做什么
## 输入            # Orchestrator 必须提供什么
## 工作步骤        # 可执行的顺序
## 输出格式        # 固定小节，便于 Orchestrator 解析
## 约束            # 反模式与禁止项
```

要求：

- `description` 要能让 Orchestrator 在一秒内判断"该不该派这个角色"。
- 输出格式用固定标题，便于跨角色汇总。
- 角色文件不重复公共规范，只写路径引用。

## 附录 B：与 Claude Code 方案的差异与限制

| 项 | Claude Code | 本仓库 DSH | 影响 |
|---|---|---|---|
| 代理注册 | 目录自动注册 | 手动内联提示词 | 每次派发多一次 `read` |
| 工具白名单 | frontmatter `tools:` | 无模型参数；需 profile 级 preset | "只读"靠提示词约束 + diff 复核 |
| 模型选择 | `model:` | `subagent` 参数 | 可为 Reviewer 指定更强模型 |
| 上下文继承 | `Agent(subagent_type="fork")` | `subagent_fork` | 审查用 fresh，实现可用 fork |
| 技能 | `.claude/skills` + 仓库根 `skills/` | `.dsh/skills` | 规则正文仍在 `skills/` |
| 并行 | 多 `Agent` 调用 | 多 `subagent` / `workflow` | 需自己控制写冲突 |
| 子代理递归 | 注册角色的工具集不含 Agent，天然不能递归 | 受深度上限限制 | `AGENTS.md` 明确禁止再委派 |

已知限制：

1. DSH 侧子代理无独立权限，无法强制"只读"，只能靠约束与事后 diff 检查；Claude Code 侧
   `tools:` 生效，但带 `Bash` 的只读角色仍是软只读。
2. 子代理不共享主会话的中间结论（除非 fork），提示词必须自包含。
3. `workflow` 无文件系统访问，脚本只能编排代理，不能直接读写。
4. 角色文件与 `AGENTS.md` 存在少量重复（派发表 vs 角色定位），修改时需同步。

## 附录 C：当前实现清单

| 路径 | 作用 | 状态 |
|---|---|---|
| `AGENTS.md` | Orchestrator 调度规则 | 已加入 |
| `.claude/agents/researcher.md` | 调查（只读） | 已创建 |
| `.claude/agents/developer.md` | 实现（Embedded / Build） | 已创建 |
| `.claude/agents/reviewer.md` | 独立审查 | 已创建 |
| `.claude/agents/architect.md` | 方案设计 | 已创建 |
| `.claude/agents/tester.md` | 测试 | 已创建 |
| `.claude/agents/security-reviewer.md` | 安全审查 | 已创建 |
| `.claude/agents/documentation-engineer.md` | 文档维护 | 已创建 |
| `.dsh/skills/sifli-code-review/SKILL.md` | DSH 技能入口（只做路由） | 已创建，技能目录可见 |
| `skills/sifli-code-review/SKILL.md` | 审查规则正文（唯一真源） | 已存在 |
| `skills/sifli-code-review/references/*` | 拆分后的规则册 | 本工作区暂不引入（沿用单文件） |

### 验证记录

1. **技能发现**：`.dsh/skills/sifli-code-review/SKILL.md` 创建后，会话技能目录立即出现
   `sifli-code-review`，`skill` 工具可按名加载；修改其 frontmatter 的 `description` 后，
   技能目录同轮内即时更新，说明入口壳确实被读取。正文按设计指向 `skills/` 下的规则真源，
   没有复制第二份规则；本工作区为单文件正文，未拆分 `references/`。
2. **角色派发**：把 `researcher.md` 正文内联进 `subagent` 的提示词，子代理按角色输出格式
   返回了 7 个小节、全部带 `path:line` 的调查结论，并主动给出"未确认"项。
   另用一次探测确认：子代理上下文里确实有 `AGENTS.md`（含"Multi-Agent Collaboration"与
   "If you are a subagent"两节），且技能目录里能看到 `sifli-code-review`。
3. **结论复核**：对子代理的两条关键结论做了抽查——`CommandRegistry.api_version`
   仅在 `registry.py:202` 赋值、无读取点；`report.failed` 在
   `sdk.py:358`/`362` 追加后立即 raise，`print_summary` 的 failed 分支不可达。
   两条均成立，说明"不信任子代理自述、回归代码复核"的流程有效。
4. **Claude Code 侧已验证**（2026-11-08）：7 个角色全部出现在可用代理列表里，`description` 与
   frontmatter 一致、`tools:` 生效（`architect` 只有 Read/Grep/Glob）；派发 `researcher` 后，
   子代理的系统提示即角色文件正文，上下文里有 `AGENTS.md` 的多代理章节；仓库根 `skills/` 下的
   5 个技能直接出现在技能列表里（`sifli-code-review` 用的是根文件描述），`.dsh/skills` 的入口壳
   在该会话中被遮蔽。
5. **依据工具契约、未实测**：Claude Code 的 `Agent(subagent_type="fork")`、`SendMessage`、
   `isolation: "worktree"`，以及 `Workflow` 的用户 opt-in 要求。这些随 harness 版本变化，
   文档写的是当前版本的契约，不作为跨版本保证。
6. **未验证**：多代理并行写冲突、`workflow` 脚本编排，留待实际任务检验。

