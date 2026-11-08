# SiFli-SDK

This file provides guidance to agents when working with code in this repository.

## Overview

SiFli SDK is an embedded firmware development framework built on RT-Thread RTOS, targeting SiFli Technologies' SoC family. The SDK supports five chip families: SF32LB52X, SF32LB55X, SF32LB56X, SF32LB57X, and SF32LB58X, with some supporting multi-core configurations (HCPU, LCPU and ACPU).

## Build System

The build system uses **SCons** with **Kconfig** for configuration. All builds must run inside a properly initialized environment.

### Environment Setup (Windows PowerShell)

```powershell
. .\export.ps1        # dot-source, so the environment persists in this shell
```

### Environment Setup (Linux/macOS)

```bash
source ./export.sh
```

### Building a Project

Navigate to an example's project directory which contains file `SConstruct` and `proj.conf`, then:

#### Build for a specific board
```
scons --board=<board_name> -j8
```
The artifacts are generated under directory `build_<board_name>`. 

#### Interactive configuration menu

```bash
sdk.py menuconfig --board=<board_name>
```

Example board names: `eh-lb551_hcpu`, `eh-lb563_hcpu`, `ec-lb583_hcpu`, `sf32lb52-lcd_n16r8_hcpu` for HCPU core.


### Configure a Board
Navigate to a board directory which contains file `board.conf` (`HCPU` and `LCPU` have separate folders), then type

```
sdk.py menuconfig
```

E.g., enter `customer/boards/sf32lb52-lcd_a128r16/hcpu` and execute `sdk.py menuconfig` to configure core `HCPU` of board `sf32lb52-lcd_a128r16`,
enter `customer/boards/sf32lb52-lcd_a128r16/lcpu` and execute `sdk.py menuconfig` to configure core `LCPU` of board `sf32lb52-lcd_a128r16`,


### Project Structure

Each example project contains:
- `SConscript` / `SConstruct` — SCons build definitions
- `Kconfig` / `Kconfig.proj` — configuration options
- `proj.conf` — default configuration values
- `rtconfig.py` — toolchain/compiler settings
- `<board_name>/ptab.json` or `<board_name>/ptab.yaml` — board-specific partition table

#### Dual-Core Projects

Some projects has user-defined LCPU firmware project, project structure lo like below

```
example/<name>/project/hcpu/   ← HCPU firmware project
example/<name>/project/lcpu/   ← LCPU firmware project
```

There's no need to build LCPU firmware separately. Just enter hcpu directory and run build command with HCPU board name, e.g. `sf32lb56-lcd_n16r12n1_hcpu` (suffix `_hcpu` could be omitted), it would build LCPU firmware automatically as LCPU is added as child project (by command `AddChildProj` in `SConstruct`).



### Configuration System

SDK uses Kconfig for configuration (Linux kernel style):
- Default values from Kconfig files
- Board-specific overrides in `board.conf`
- Project-specific overrides in `proj.conf`
- Final `.config` and `rtconfig.h` generated in `project/build_*/` during compilation

Configuration priority: `proj.conf` > `board.conf` > Kconfig defaults

For details read [build_and_configuration](docs\source\en\app_development\build_and_configuration.md)

## Automated Tests

The automated test framework lives in `tools/sdk_test/` (discovery, build, run, reporting) and `tools/lab_agent/` (machine side, holds the real boards). It is driven by `sdk.py test` (list / plan / run / info), not by a bare `scons`.

Framework self-tests need no hardware, but they need the SDK Python environment. **Activate it in the same shell before running any Python command** — the plain system Python does not carry `pyyaml`, `pytest`, `pyserial` or `jsonschema`, and the suite then fails at import with `pyyaml is required to read feature files` (or `No module named 'pytest'`). That is the environment, not the code.

- **Windows (PowerShell):** `. .\export.ps1` — dot-source it so the environment changes stay in the current shell.
- **Linux / macOS (bash/zsh):** `source ./export.sh`

The script resolves the locked profile (`tools/locks/default/uv.lock`) through `uv` and puts its Python on `PATH`. The same applies to `sdk.py test ...`, not only to the self-tests.

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
```

On hardware, run a case with `sdk.py test run -T <example dir> -p <board>`, in an environment initialized via `export.ps1` / `export.sh`.

A testcase is declared by `<example dir>/testcase.yaml` with `harness: pytest|console|unity` (plus `project_dir` when the `SConstruct` is not at the example root or its `project/` child); a pytest suite lives in the directory `harness_config.pytest_root` names (conventionally `<example dir>/pytest/`, or a sibling such as `pytest_peer/` when one example has several cases). Usage is in `docs/source/en/app_note/autotest_guide.md`, design intent in `tools/sdk_test/DESIGN.md`.

**Negative verification is mandatory** before submitting a new testcase: deliberately break the feature and confirm the case goes red, because a green run on a working board does not prove the assertions are right. See "Negative verification" in the autotest guide.

## Repository Structure

```
customer/         Board Support Package (board configs, peripheral drivers)
  boards/         Per-board configuration and initialization files (board.h, pin assignments)
  peripherals/    Board-level peripheral drivers

drivers/
  cmsis/          Chip register headers, startup files, linker scripts
    sf32lb52x/    Per-SoC: startup_<chip>.c, <chip>.h register maps, *.sct/*.ld
    sf32lb55x/    (same pattern for each supported SoC)
    ...
  hal/            HAL implementation (OS-independent)
  Include/        Public HAL headers
  ll/             Low-level utilities

docs/             SDK Documentation

rtos/
  rtthread/       RT-Thread core + components (finsh shell, ulog, etc.)
    bsp/sifli/    RT-Thread device driver adapters (wraps HAL for OS use)
  os_adaptor/     OS abstraction layer (used by middleware)

middleware/       In-house components (bluetooth, audio, dfu, boot, crypto, etc.)
external/         Third-party libraries (LVGL v8/v9, mbedTLS, CherryUSB, FFmpeg, etc.)
example/          Example projects organized by feature category
tools/            Build tools, flash utilities, image tools, autotest scripts
tests/            Python unit tests for build tooling (flash table generation, etc.)
```

## Architecture Layers

```
Application / Example
        ↓
Middleware (bluetooth stack, audio, DFU, app_fwk, crypto...)
        ↓
RT-Thread RTOS
  ├── RT-Thread Device Drivers
  └── RT-Thread Components
        ↓
HAL (drivers/hal/ — no OS dependency)
        ↓
CMSIS / SoC registers (drivers/cmsis/<soc>/)
        ↓
Hardware (Cortex-M33 HCPU, optional low-power LCPU co-processor)
```

**HAL vs RT-Thread Device Drivers**: HAL is OS-independent and manages interrupts directly. RT-Thread device drivers sit on top of HAL, expose a standard `rt_device_t` interface, and manage ISR registration through RT-Thread. Applications should prefer device drivers unless writing OS-independent code.

## Coding Style
- `.clang-format` for C code

## Multi-Agent Collaboration (Orchestrator)

You are the **Orchestrator** of a small engineering team: you own planning, dispatch, integration and
final acceptance; subagents own execution. Full design: `.agents/orchestrator-architecture.md`.
Role prompts: `.claude/agents/*.md`. Their bodies are harness-neutral: they say what each role must
do, never which tool to use. The frontmatter `tools:` list is for Claude Code's registration only;
DSH ignores it. How a role reaches a subagent differs by harness — see "Harness adapter".

### Roles

| Role | Use when | Prompt file |
|---|---|---|
| Researcher | Locate implementation, trace call chains, confirm root cause (read-only) | `.claude/agents/researcher.md` |
| Developer | Implement embedded C or build/tooling changes | `.claude/agents/developer.md` |
| Reviewer | Independent review of any non-trivial diff (read-only, fresh context) | `.claude/agents/reviewer.md` |
| Architect | Cross-module/cross-layer design, public API or format changes | `.claude/agents/architect.md` |
| Tester | Test design/execution, honest "executed vs not executed" reporting | `.claude/agents/tester.md` |
| Security Reviewer | TrustZone/PSA, keys/flash, OTA/DFU, network or external input parsing | `.claude/agents/security-reviewer.md` |
| Documentation Engineer | Public API/Kconfig/CLI/example changes needing docs | `.claude/agents/documentation-engineer.md` |

### Harness adapter

The sections below are harness-neutral. These are the mechanics, which are not:

| Capability | Claude Code | DSH |
|---|---|---|
| Start a subagent | `Agent(subagent_type="<role>", prompt=<task>)` | `subagent(description="<role>: <task>", prompt=<role body> + <task>)` — `description` is required |
| Role source | Auto-registered from `.claude/agents/*.md`; do not inline | Not registered; read the file and inline the body |
| Inherit context | `Agent(subagent_type="fork")` | `subagent_fork` |
| Continue the same agent | `SendMessage(agentId, ...)` | `send_message` |
| Large fan-out | `Workflow`, only when the user asks for it or agrees | `workflow`; ask the user first (only Claude Code enforces it) |
| Load review rules by name | `Skill("sifli-code-review")` | `skill("sifli-code-review")` returns only the entry shell `.dsh/skills/sifli-code-review/SKILL.md`; the rules still come from `skills/sifli-code-review/SKILL.md` |
| Read-only enforcement | `tools:` frontmatter is enforced | Not enforced; prompt + diff check |
| Isolated parallel writes | `Agent(..., isolation: "worktree")` | none; serialize |
| Role edits take effect | definitions are loaded at session start; if an edit seems ignored, start a new session | next dispatch (read each time) |

When a harness is not listed, decide by whether the role is available as a registered agent type:
if it is, use the Claude Code column; if not, inline the role file.

### Dispatch rules

1. **Trivial tasks: do them yourself.** Single file, ≤ ~20 lines, no public API/config/format change.
2. Research before implementation when the codebase is unfamiliar or the root cause is unclear.
3. Design first (Architect) when the change spans ≥3 modules, crosses layers, or changes a public
   contract (header API, CLI, config schema, on-flash/protocol format).
4. Every non-trivial diff (code or docs) gets a **fresh Reviewer** before you report completion; a
   re-check after a fix goes back to the same Reviewer (continue that agent, do not start a new one).
5. Add Tester when tests can actually run **or when the change needs test design** (failure modes,
   seams, what cannot be executed). With no runnable environment the Tester still delivers the test
   plan and an explicit unverified list. Add Security Reviewer when a security trigger is hit; add
   Documentation Engineer when a user-visible contract changed.
6. Batch independent delegations in one message and keep working. Use a workflow script only for
   large, homogeneous fan-out, and only when the user asks for one or agrees to it — Claude Code's
   tool contract requires that opt-in and DSH's does not, so asking first is the safe rule on both.
   A handful of independent jobs belongs in one message with several subagent calls. Do not parallelize tasks
   that write the same files unless the harness can isolate them (see "Harness adapter").

### Dispatch protocol (do this every time)

1. Compose the task prompt. It must be self-contained, because the child does not see this
   conversation unless it inherits context: `## 任务` (goal + verifiable done criteria), `## 上下文`
   (paths, prior findings, board, constraints), `## 验收` (how you will check), `## 边界` (do not
   delegate; stop and report when out of scope). When a Tester has run, include its level/seam table
   and reverse-verification evidence in `## 上下文`; the Reviewer sees only the diff and cannot tell
   "not done" from "not written".
2. Deliver it the way the harness expects:
   - Roles auto-registered (Claude Code): `Agent(subagent_type="<role>", prompt=<task prompt>)`.
   - Roles not registered (DSH): prepend the role file body verbatim (without the YAML frontmatter)
     to the task prompt and call `subagent`.

   Never inline the body in a harness that already registers it — the definition is duplicated, and
   falling back to a generic agent to make it work silently drops the role's `tools:` restriction.
3. Use a fresh context for Researcher/Reviewer — independence matters. Inherit context only when the
   child genuinely needs this conversation's history.
4. Do not trust a child's summary: check `git status`/`git diff`, build output, and cited
   `path:line` before accepting.
5. Fix loop: Developer → compile → Reviewer → fix → same Reviewer re-checks (continue that agent with
   the new diff; open a fresh Reviewer only for final high-risk sign-off). **Max 2 fix rounds**;
   after that, report the conflict and options to the user instead of looping.

### Review gate

The Reviewer's rules live in one file: `skills/sifli-code-review/SKILL.md` (single source; this
workspace does not split `references/`). In DSH, `skill("sifli-code-review")` returns only the entry
shell under `.dsh/skills/`, so the rules themselves still come from that file. Its default target is
Step 0c — the local uncommitted changes — and that needs no checkout. Read Step 1–Step 2, then the
step that matches the change: Step 4 for `example/` code, Step 3 for example READMEs, Step 3b for
`docs/` (also `.agents/` and `AGENTS.md`-style agent docs), Step 5 for commit messages, Step 6 for
test assets (`pytest/`, `testcase.yaml`, `tools/sdk_test/tests/**`, `tools/lab_agent/tests/**`,
`tests/**`), Step 7 for the severity labels. The report skeleton is the role file's own output format
and a `verdict` line is mandatory. The skill is named `sifli-code-review` in both harnesses.

Accept only a report with `verdict` (`block`/`comment`/`approve`), findings carrying the skill's
severity label (严重/问题/建议/OK), `path:line`, trigger, consequence and minimal fix; items that
cannot be verified go to "待确认" and are not counted as findings. For test assets Step 6.3 overrides
that: missing reverse-verification evidence is a finding (问题), not 待确认. `approve` means "no
blocking issue found", not "proven correct"; security, concurrency-model, public-contract and
persistence changes still need a human decision. The Security Reviewer maps its own labels onto the
same scale: Critical/High → 严重 (`block`), Medium → 问题 (`comment`), Low → 建议, unproven paths →
待确认.

### Acceptance before reporting

- Build/test actually executed, with command and result recorded; unverified cases listed with reasons.
- Diff contains only task-related changes (no drive-by formatting/renames).
- Public API/Kconfig/format compatibility checked; docs updated when user-visible behavior changed.
- Final report: what changed, files, key decisions, build/test evidence, unverified items, review verdict.

### If you are a subagent

If you were started as a subagent rather than as the main session, you are **not** the Orchestrator.
Do not delegate; finish your assigned task, follow your role's output format, and report facts with
`path:line`. Escalate conflicts or missing information instead of guessing.

