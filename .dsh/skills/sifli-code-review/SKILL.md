---
name: sifli-code-review
description: SiFli SDK 代码审查规则入口。审查 C/固件、Python 工具、例程、文档或测试资产时使用；含分级门禁、通用代码/例程/文档/测试审查步骤与报告格式。
whenToUse: 需要对 SiFli SDK 的 diff、commit 或文件做 code review；或需要判断改动是否符合分层、并发安全、例程与文档规范。
---

# SiFli SDK 代码审查（入口）

本仓库的审查规则正文只有一份：`skills/sifli-code-review/SKILL.md`。
本 skill 只负责入口与路由，不复制第二份规则（本工作区暂不拆分 `references/`）。

## 加载顺序

1. 必读：`skills/sifli-code-review/SKILL.md` 的 Step 0–Step 2
   —— 确定审查目标、文件分类、通用代码检查（风格 / 逻辑边界 / RTOS 并发 / 无关改动 / 清晰度 / 功能正确性）。
2. 按变更类型追加同一文件中的步骤：
   - `example/` 下的 C/H：Step 4（例程结构、`proj.conf` 最小化、README 一致性、评分），同时适用 Step 2
   - `example/` 下的 `README*.md`：Step 3
   - `docs/` 下的文档：Step 3b
   - 本地 commit 或 Gerrit 变更：Step 5（提交说明）
   - 测试资产（`pytest/`、`testcase.yaml`、`tools/sdk_test/tests/**`、`tools/lab_agent/tests/**`、`tests/**`）：Step 6（测试代码审查）
   - `.agents/`、`AGENTS.md` 等 agent 文档：Step 3b 的清晰度/一致性口径，不做评分
3. 报告格式与严重度标注：Step 7（中文报告；严重 / 问题 / 建议 / OK；总结统计问题数）。

## 审查顺序

按 Step 2.1–2.6 逐项过一遍；发现按下面的顺序归类与排序：
分层与公共面 → 并发与资源 → 错误与恢复 → 局部正确性 → 设计取舍（扩展是否过度或不足）。

注意：Step 0 的审查目标由 Orchestrator 的任务给定，不需要向用户反问；目标缺失时按本地未提交改动处理，并在报告开头声明假设。

## 结论口径（供 Orchestrator 汇总）

- `verdict`：`block` / `comment` / `approve`。
- 严重度沿用 Step 7：严重（阻塞）、问题（需回应）、建议（不阻塞）、OK（已确认）。
- 每条发现带 `severity: critical|high|medium|low`（critical/high → 严重，medium → 问题，low → 建议）。
- 证据不足、无法判定的条目放"待确认"，不计入问题数。
- 每条发现必须含 `path:line`、触发条件、具体后果、最小修复；同一模式只报一次。
