---
name: sifli-code-review
description: Review SiFli SDK code changes for coding style compliance, logic correctness, RTOS concurrency safety, unnecessary modifications, and clarity. For example code, also checks structure compliance, proj.conf minimality, and readability. For README docs, checks template compliance, clarity, CN/EN consistency, and scores the document. Supports reviewing local uncommitted changes, a local commit SHA, or fetching a specific commit from Gerrit for review.
---

# SiFli SDK Code Review

Comprehensive code review for SiFli SDK projects covering general code, example projects, and documentation.

## Trigger

When the user says "review下代码", "审阅下代码", "审查下代码", "review this code", "从gerrit拉取", "审查gerrit", "review这个commit", "审查这个提交", "审阅这个sha", or similar phrases, use this skill to perform a structured review of the current changes, a local commit, or a Gerrit commit.

## Step 0: Determine the Review Target

Ask the user to clarify whether the review is for:

1. **Local changes** — uncommitted working tree changes (default if user just says "review下代码")
2. **Local commit SHA** — a commit that already exists in the local repository. The user provides a SHA hash (full or short, e.g. `a1b2c3d` or `028f3a59f`).
3. **Gerrit commit** — a specific commit on Gerrit. The user must provide one of:
   - A Gerrit change URL, e.g. `http://10.21.10.179:8080/c/SiFli-SDK/+/12345` or `ssh://10.21.10.179:29418/SiFli-SDK`
   - A Gerrit change number, e.g. `12345`
   - A `refs/changes/...` refspec
   - A commit SHA (40-char hash)

> **How to distinguish local SHA vs Gerrit SHA**: If the user provides a SHA hash, first check if it exists locally with `git cat-file -t <sha>`. If it exists locally, treat it as a local commit SHA (option 2). If it does not exist locally, treat it as a Gerrit SHA and attempt to fetch it from Gerrit (option 3).

### Step 0a: Review Local Commit SHA

When the user provides a local SHA to review, use these commands:

**Step 1 — Verify the commit exists locally:**
```bash
git cat-file -t <sha>
```
Expected output: `commit`. If the command fails, the SHA doesn't exist locally — fall through to the Gerrit fetch flow (Step 0b).

**Step 2 — Determine the diff base:**

The diff shows what the commit introduced. Use the commit's first parent:
```bash
git diff <sha>~1..<sha>
```

If the commit is a merge commit (multiple parents), you may want to use a different base:
```bash
# For merge commits, diff against the first parent
git diff <sha>~1..<sha>
# Or diff against a specific branch
git diff <branch>..<sha>
```

**Step 3 — Check out the commit to read files:**
```bash
git checkout <sha>
```

> **IMPORTANT**: Checking out a detached HEAD will move away from the current branch. After the review is complete, return to the original branch:
> ```bash
> git checkout -
> ```

**Step 4 — Read the commit message:**
```bash
git log -1 --format=full <sha>
```

**Step 5 — Show the diff for analysis:**
```bash
git diff <sha>~1..<sha>
git diff --name-only <sha>~1..<sha>
```

### Step 0b: Fetch from Gerrit

The Gerrit server is at `ssh://10.21.10.179:29418/SiFli-SDK`. To fetch a change:

**If given a Gerrit URL** like `http://10.21.10.179:8080/c/SiFli-SDK/+/12345` or `http://10.21.10.179:8080/c/SiFli-SDK/+/12345/5`:
- Extract the change number (e.g. `12345`) and optional patchset number (e.g. `5`)
- The last 2 digits of the change number determine the ref directory: `refs/changes/<last2>/<change>/<patchset>`
- Example: change `12345` → `refs/changes/45/12345/`

**Step 1 — Fetch the change:**

If a specific patchset is given:
```bash
git fetch origin refs/changes/<last2>/<change_number>/<patchset>
```

If no patchset is given, fetch all patchsets for that change:
```bash
git fetch origin refs/changes/<last2>/<change_number>/*
```

Then find the latest patchset from the fetched refs and checkout:
```bash
git checkout FETCH_HEAD
```

**If given a commit SHA**, try fetching it directly:
```bash
git fetch origin <sha>
git checkout FETCH_HEAD
```

**Step 2 — Determine the diff base:**

After fetching and checking out the commit, determine what to diff against. Use the **actual parent commit** (FETCH_HEAD's first parent):

```bash
git diff FETCH_HEAD~1..FETCH_HEAD
```

> **IMPORTANT — Use `FETCH_HEAD~1`, not `HEAD~1`**: After `git fetch` (without checkout), `FETCH_HEAD` points to the fetched commit. `HEAD` is your current branch, which is unrelated. Using `HEAD~1..FETCH_HEAD` will give you the diff between your branch and the fetched commit — NOT the actual changes in the commit!
>
> After `git fetch && git checkout FETCH_HEAD`, `HEAD` now equals `FETCH_HEAD`, so `HEAD~1..HEAD` works. But `git diff FETCH_HEAD~1..FETCH_HEAD` is always safe regardless of checkout state.

Or if the user specifies a branch to compare against:
```bash
git diff <branch>..HEAD
```

Also show the commit message (always reference FETCH_HEAD explicitly):
```bash
git log -1 --format=full FETCH_HEAD
```

> **IMPORTANT — Always use `FETCH_HEAD` in `git log`**: After `git fetch`, `HEAD` is still your original branch. Running `git log -1` without `FETCH_HEAD` will show the wrong commit message. Even after `git checkout FETCH_HEAD`, using `FETCH_HEAD` is safer — there's no ambiguity about what you're reading.

### Step 0c: Review Local Changes (default)

If no Gerrit reference or local SHA is provided, review the current working tree.

First, identify what files are changed:

```bash
git diff --name-only
```

Also check for untracked files:

```bash
git status -s
```

## Step 1: Classify Changed Files

Classify each changed file into one of four categories:
- **General code**: C source/header files, build scripts (SConscript, SConstruct, Kconfig), configuration files
- **Example README**: `README.md` or `README_EN.md` files in an example directory
- **Example code**: C source/header files in `example/` directories (which also requires the general code checks)
- **SDK documentation**: files under `docs/` directory (`.md`, `.rst`)

> **CRITICAL — Read source files yourself; do NOT delegate to sub-agents.**
>
> Sub-agents may lack access to `FETCH_HEAD` or the working tree. If they silently fail, the review proceeds with no actual code reading.
>
> 1. **For Gerrit commits**: `git checkout FETCH_HEAD`, then read files directly from disk. **For local SHA commits**: `git checkout <sha>`, then read files directly from disk. This also gives accurate line numbers.
> 2. **For new files or files with >100 lines changed, read the FULL source file** — not just the diff. Diffs lack surrounding context (control flow, callers, variable scope).
> 3. **Read every line** of substantial files (>50 lines changed). Do not skim — trace control flow through each function, check every condition and error path.

> **IMPORTANT — For new chip support changes, review must verify "the entire driver works correctly on the new chip", not just "the few lines changed are free of typos".**
>
> When the diff adds `#if defined(NEW_CHIP)` / `#if !defined(NEW_CHIP)` guards to add support for a new chip family, the diff itself only shows where the guards are placed. It does **not** show the execution flow for the new chip. You must reconstruct it mentally:
>
> 1. **Traverse all execution paths for the new chip**: For every function touched by the diff, trace through the function with the new chip's macros enabled, and verify each path is semantically correct. Do not assume that "skipping the old code" is sufficient — the new chip must still produce correct behavior (timeout values, register configurations, interrupt setup, etc.).
>
> 2. **Contrast key parameters between old and new paths**: Compare critical values (timeout durations, reload values, clock frequencies, interrupt windows) between the existing chip path and the new chip path. If they differ, ask whether the difference is intentional (hardware difference) or accidental (bug). For example: if the existing chip sets Reload2 to 50 seconds but the new chip path sets it to 5 seconds, flag it for confirmation.
>
> 3. **Check macro naming consistency**: Newly introduced condition macros (e.g. `MY_CHIP`) must be checked against existing macros in the same file (e.g. `SOC_MY_CHIP`). Inconsistent naming leads to silent compilation logic errors where the guard never activates.
>
> 4. **Evaluate un-guarded code paths**: Code paths that remain unchanged (not wrapped in any `#if`) may still be wrong for the new chip. Check whether functions called from modified code access registers or hardware blocks that don't exist on the new chip.
>
> 5. **Check arithmetic consistency**: When the diff involves expressions with mixed operators (e.g. `freq * time + offset` vs `freq * (time + offset)`), verify operator precedence produces the intended result. These bugs are invisible on a quick read.

**For local changes**:
```bash
git diff --name-only && git status -s   # identify files
git diff                                # read the diff
```

**For local commit SHA**:
```bash
git diff <sha>~1..<sha>                 # read the diff
git diff --name-only <sha>~1..<sha>     # identify changed files
```
Then checkout the commit (`git checkout <sha>`) and read each changed source file directly from disk with the Read tool.

**For Gerrit changes**:
```bash
git fetch origin refs/changes/<last2>/<change_number>/<patchset>
git checkout FETCH_HEAD
git diff HEAD~1..HEAD                   # read the diff
```
Then read each changed source file directly from disk with the Read tool.

## Step 2: General Code Review (applies to all code files)

For each code file changed, check the following items:

### 2.1 Coding Style Compliance

Check against the project's `.clang-format` (Allman brace style, 4-space indent) and `docs/source/en/contribute/coding_style.md`:

- **Braces**: Allman style — braces on their own line, not at end of statement. Exception: `switch-case` aligns `case` with `switch`.
- **Indentation**: 4 spaces. Preprocessor directives indented with `BeforeHash` style.
- **Naming**:
  - Variables: lowercase with `_` (snake_case). Global variables prefixed with module name. Static variables preferably with `s_` prefix.
  - Functions: lowercase with `_`. Internal static functions prefixed with `_`.
  - Macros/Enum values: UPPERCASE with `_`.
  - Types: struct/enum name + `_t` suffix.
- **Spacing**: Space before `(` in `if`, `for`, `while`, `switch`. No spaces inside parentheses of expressions. Operators have spaces around them.
- **Header guards**: `#ifndef __FILE_H__` / `#define __FILE_H__` format.
- **Comments**: Must use **English** (not Chinese). Function comments use doxygen style (`/** ... */`). Statement comments above or to the right only.
- **SortIncludes**: `false` — includes should be preserved as-is, do not reorder.
- **No consecutive blank lines**: At most one blank line unless special need.

### 2.2 Logic Correctness & Boundary Conditions

- Are all error paths handled? Check return values of functions that can fail.
- Are array indices bounds-checked?
- Are pointers checked for NULL before dereference?
- Are buffer sizes validated to prevent overflow?
- Are loop termination conditions correct?
- Are integer overflow/underflow cases considered?
- Are `switch` statements exhaustive (including `default` case)?
- Are configuration macros used instead of hard-coded values?

### 2.3 RTOS Multi-Thread Safety

- Identify all global/static variables accessed in the changed code.
- For each global/static variable: can it be accessed from multiple threads or from both thread and ISR context?
- If yes: is access protected by a mutex, semaphore, critical section, or atomic operations?
- Are RT-Thread IPC calls used correctly (message queues, mailboxes, events)?
- Are calls to non-reentrant functions (e.g., `malloc`, `printf`) safe in the context they're called from?
- Check for potential deadlocks: consistent lock ordering, no nested locks that could invert.

### 2.4 Unnecessary Changes

- Are there any formatting-only changes mixed with logic changes? Flag them — they should be separate.
- Are there commented-out code blocks left in? They should be removed.
- Are there leftover debug `LOG_D` / `rt_kprintf` statements?
- Are there changes that don't relate to the stated purpose of the commit?
- Are there whitespace-only lines added or removed?

### 2.5 Code Clarity & Comments

- Is the logic easy to follow? Complex algorithms should have explanatory comments.
- Are magic numbers replaced with named constants or macros?
- Are function names descriptive of what they do?
- Is there appropriate use of `LOG_D`/`LOG_I`/`LOG_W`/`LOG_E` (via ulog, not `rt_kprintf`) for logging?
- Are key design decisions or non-obvious behaviors commented?

### 2.6 Functional Correctness

- Does the code do what it claims to do?
- Are there off-by-one errors?
- Are conditional expressions correct (not accidentally using `=` instead of `==`)?
- Are bitwise operations correct?
- Are type casts safe (especially pointer casts)?
- Is `const` used where appropriate for pointer parameters?
- **For arithmetic expressions**: check operator precedence when mixing `*`, `+`, `-`, especially between `freq * time + offset` and `freq * (time + offset)` patterns — verify intent matches execution.

## Step 3: Example README Review (for `README.md` / `README_EN.md` in example directories)

Compare against the README template at `docs/TEMPLATE_EXAMPLE_README.md` (EN) and `docs/TEMPLATE_EXAMPLE_README_CN.md` (CN).

### 3.1 Template Compliance

Check that the README has these sections (matching the template):
- Title (using "example" not "demo"/"test")
- Source code path (format: `Source code path: example/xxxxx` or `源码路径: example/xxxxx`)
- Overview (what is this example, what does it do, what SiFli-SDK features does it use)
- Usage section with link to SiFli-SDK Getting Started
- Supported Boards
- Hardware Required (detailed for beginner examples, concise for advanced ones)
- Configure the Project (with `scons --board=<board_name> --menuconfig`)
- Example Output (expected console output in code block)
- Example Breakdown (optional — needed for complex examples, remove for simple ones)
- Troubleshooting (or "异常诊断" in CN)
- Reference (links to important documents)

### 3.2 Clarity of Purpose

- Can a reader understand the main feature and use case of the example from the overview?
- Is it clear what SiFli-SDK features are demonstrated?

### 3.3 Language Fluency & Readability

Applies to both CN and EN READMEs:

- Is the writing clear, natural, and easy to follow?
- Are sentences well-structured and not overly long or tangled?
- Are there typos, grammatical errors, or awkward phrasing?
- For EN: see 3.6 for detailed English-specific checks.
- For CN: check for typos (错别字), run-on sentences (一逗到底), and word-for-word translations from English that sound unnatural.

### 3.4 Usage & Expected Results

- Are compilation and flashing steps clear?
- Is the expected output shown and sufficient to verify correct execution?
- For examples requiring user interaction (press button, send BLE command, etc.), is it explained?

### 3.5 CN/EN Consistency

If both `README.md` (Chinese) and `README_EN.md` (English) exist:
- Do they convey the same information?
- Are there any sections that exist in one but not the other?
- Are code snippets, commands, and output consistent across both?

### 3.6 English Expression Quality (for `README_EN.md`)

For the English version of the README, evaluate the quality of English expression:

- **Accuracy**: Is the technical meaning correctly conveyed? Are there mistranslations or misunderstandings of technical terms? Check that terms match standard SiFli SDK terminology (e.g., "development board" not "development kit" unless appropriate, "flash" not "burn", "compile" not "translate").
- **Naturalness**: Is the English idiomatic and natural-sounding to a native speaker? Flag:
  - "Chinglish" expressions — direct translations from Chinese that sound unnatural in English
  - Run-on sentences — Chinese often uses commas to join clauses; English requires proper sentence boundaries
  - Missing articles (a, an, the) or incorrect article usage
  - Word-for-word translations that lose meaning (e.g., "according to the actual situation" → "as needed" / "accordingly")
  - Passive voice overuse where active voice would be clearer
- **Grammar**: Subject-verb agreement, tense consistency, plural forms, prepositions.
- **Technical conventions**: Code blocks use correct syntax highlighting markers. Commands are correctly formatted. Paths use correct slash direction. Acronyms are properly capitalized.

### 3.7 Score (1-5)

Rate the README from a reader's perspective on:
- **Fluency** — is the language natural and easy to read? (For English docs, this includes the English expression quality from 3.6)
- **Completeness** — is all necessary information present?
- **Readability** — is it well-structured with proper formatting?

## Step 3b: SDK Documentation Review (for files under `docs/`)

Unlike example READMEs, SDK docs do not follow a fixed template. Review focuses on language quality and consistency.

### 3b.1 Clarity & Fluency

- Is the writing clear and easy to follow?
- Are complex concepts explained well?
- Is the English natural and idiomatic (for EN docs)? Check for:
  - "Chinglish" word-for-word translations
  - Run-on sentences
  - Missing articles (a, an, the)
  - Awkward phrasing
- Are technical terms used correctly and consistently?

### 3b.2 CN/EN Consistency

If both Chinese and English versions exist:
- Do they convey the same information?
- Are code snippets, commands, and output examples identical across both?
- Are there sections in one that are missing from the other?
- Pay attention to chip-scoping directives (`{only} SF32LB52X` etc.) — the same content should have the same scope in both languages. If EN is chip-agnostic but CN is inside a `{only}` block, flag it.

### 3b.3 Grammar & Formatting

- Spelling and grammar errors
- Code blocks use correct syntax highlighting markers
- Commands and paths are correctly formatted
- Links are valid (within reason — check that referenced files exist)

### 3b.4 Score (1-5)

Rate the SDK documentation from a reader's perspective on:
- **Fluency** — is the language natural and easy to read? (For EN docs, this includes English expression quality from 3b.1)
- **CN/EN Consistency** — do both language versions convey the same information?
- **Readability** — is it well-structured with proper formatting?

## Step 4: Example Code Review (for code in `example/` directories)

In addition to all items in Step 2 (General Code Review), also check:

### 4.1 Code Structure Compliance

**Single-core example** — compare against `example/get-started/hello_world/rtt/`:
```
example/<category>/<name>/
  README.md          (Chinese)
  README_EN.md       (English)
  src/
    main.c           (main functionality)
    SConscript       (Glob('*.c') then DefineGroup)
  project/
    SConstruct       (PrepareEnv → SifliEnv → DoBuilding → AddFTAB → GenDownloadScript)
    SConscript       (imports SIFLI_SDK + src SConscript)
    Kconfig          (source "$SIFLI_SDK/Kconfig.v2" + rsource "Kconfig.proj")
    Kconfig.proj     (CUSTOM_MEM_MAP config)
    rtconfig.py
```
- Does `src/main.c` exist?
- Does `src/SConscript` use `DefineGroup` with proper depend?
- Are `SConstruct` and `SConscript` present in the project directory?
- Are `Kconfig` and `Kconfig.proj` present?

**Dual-core example** — compare against `example/get-started/dualcore/`:
```
example/<category>/<name>/
  project/
    hcpu/
      SConstruct       (PrepareEnv → AddChildProj/AddLCPU → AddBootLoader → SifliEnv → DoBuilding → AddFTAB → GenDownloadScript)
      SConscript       (includes SDK + hcpu src SConscript)
      Kconfig, Kconfig.proj, rtconfig.py
      proj.conf
    lcpu/
      SConstruct       (PrepareEnv → SifliEnv → DoBuilding)
      SConscript       (scans subdirs for SConscript, includes SDK + lcpu src)
      Kconfig, Kconfig.proj, rtconfig.py
      proj.conf
  src/
    hcpu/
      main.c
      SConscript
    lcpu/
      main.c
      SConscript
```
- Are both `hcpu/` and `lcpu/` project directories present?
- Does each have `SConstruct`, `SConscript`, `Kconfig`, `Kconfig.proj`?
- Does `hcpu/SConstruct` call `AddChildProj()` or `AddLCPU()` for the LCPU?
- Does each `src/` side have `main.c` and `SConscript`?

### 4.2 Proj.conf Minimality

- Does `proj.conf` only enable the **necessary** config options for this example?
- Are there any config options that are not needed for this example's functionality? Flag them.
- Are config options that duplicate Kconfig defaults included unnecessarily?
- For dual-core: check both `hcpu/proj.conf` and `lcpu/proj.conf`.

### 4.3 Readability & README Consistency

- Is the example code easy to understand? Follow the natural execution flow?
- Are key sections commented (in English)?
- Do non-static file-local functions/variables use `example_` prefix?
- Is there any dead code (old debug logs, commented-out code)?
- Are options hard-coded instead of using config macros/constants?
- Does the code behavior match what the README describes?
- Are displayed strings/log messages in the code consistent with the expected output shown in the README?
- Does each source file have a license header (Apache 2.0 or CC0)?

### 4.4 Score (1-5)

Rate the example code from a reader's perspective on:
- **Readability** — can a developer unfamiliar with the code understand it quickly?
- **Structure** — is the code well-organized?
- **Documentation** — are comments helpful and sufficient?

## Step 5: Commit Message Review (local SHA and Gerrit commits)

When reviewing a specific commit (local SHA or Gerrit), also check the commit message. First, read it:

**For local commit SHA:**
```bash
git log -1 --format=full <sha>
```

**For Gerrit commit:**
```bash
git log -1 --format=full FETCH_HEAD
```

The project follows a commit template (from `.commit_template`):

```
[bug|new|opt|chore][module_name] one line description

redmine: #id, REDMINE-id
ext-redmine: #id, REDMINE-id

[Description in detail]
Describe the change in more detail, e.g. how is the problem fixed, why do you make the modification in this way, explain the feature in more details
```

### 5.1 Subject Line (First Line)

- **Tag format**: Should start with one of `[bug]`, `[new]`, `[opt]`, `[chore]` followed by `[module_name]`.
  - `[bug]` — bug fix
  - `[new]` — new feature
  - `[opt]` — optimization/refinement
  - `[chore]` — chore/maintenance
- **Module name**: The tag should identify the module being changed, e.g. `[ble]`, `[drv][adc]`, `[hal][gpio]`, `[rtos]`.
- **One-line description**: After the tags, a concise one-line summary of what the change does. Should clearly state what bug was fixed or what feature was added. Not too vague ("fix bug" is bad; "fix null pointer dereference in audio callback" is good).
- **Length**: The subject line should generally be under 72 characters (the tags help categorize, so the description part should be brief).

### 5.2 Detailed Description

- **Explanation of WHY**: The description should explain **why** the change was made, not just restate what the diff shows. What problem existed? How was it discovered?
- **Explanation of HOW**: For non-trivial fixes, explain the approach: why was this specific fix chosen over alternatives? Are there any trade-offs?
- **Clarity**: The description should be understandable to someone not familiar with the specific bug/feature. Avoid internal jargon without explanation.
- **Redmine links**: If applicable, include `redmine:` and `ext-redmine:` references.

### 5.3 English Quality

- **Grammar**: Check for subject-verb agreement, tense consistency, article usage.
- **Spelling**: Check for typos and misspelled words.
- **Expression**: Is the English natural and idiomatic? Flag awkward phrasing or "Chinglish" expressions.
- **Technical accuracy**: Are technical terms used correctly (e.g., "register" vs "registry", "interrupt" vs "exception")?

### 5.4 Completeness

- Does the commit message cover all the changes in the diff?
- If the commit touches multiple modules, does the description address each?
- Are any significant side-effects or behavioral changes mentioned?

## Step 6: Output Format

**审查报告请使用中文撰写。**

Present the review as a structured report:

```
## 代码审查报告

### 审查文件
- [list of files]

### 类别: [通用代码 / 例程代码 / README / SDK 文档]

#### 1. 代码风格
[Findings, each marked as OK / 问题 / 建议]

#### 2. 逻辑与边界条件
[Findings]

#### 3. RTOS 多线程安全
[Findings or "不适用 — 未访问全局状态"]

#### 4. 不必要的修改
[Findings]

#### 5. 代码清晰度与注释
[Findings]

#### 6. 功能正确性
[Findings]

[If example code, add:]
#### 7. 例程结构
[Findings]

#### 8. proj.conf 最小化
[Findings]

#### 9. 与 README 一致性
[Findings]

#### 10. 例程评分: X/5
[Brief justification]

[If README, add:]
#### 模板合规性
[Findings]

#### 语句通顺与可读性
[Findings on clarity, grammar, typos — both CN and EN]

#### 中英文一致性
[Findings]

#### 英文表达质量 (针对 README_EN.md)
[Findings on accuracy, naturalness, grammar, technical conventions]

#### README 评分: X/5
[Brief justification]

[If SDK documentation, add:]
#### 语句通顺与可读性
[Findings on clarity, grammar, typos — both CN and EN]

#### 中英文一致性
[Findings, including chip-scoping directive checks]

#### 英文表达质量 (针对英文文档)
[Findings on accuracy, naturalness, grammar, technical conventions]

#### SDK 文档评分: X/5
[Brief justification on clarity, CN/EN consistency, and English quality]
```

**[If local SHA or Gerrit commit, add:]**

```
### Commit Message 审查

#### 标题行
[Findings on tag format, description clarity, length]

#### 详细说明
[Findings on why/how explanation, completeness]

#### 英语质量
[Grammar, spelling, expression issues]

#### Commit Message 评分: X/5
[Brief justification on overall quality]
```

```
### 总结
- 总问题数: X
- 严重: X
- 一般问题: X
- 建议: X
```

每条发现标注严重程度：
- **严重**: Bug、安全隐患、竞态条件 — 合并前必须修复
- **问题**: 风格违规、缺失章节、非最优写法 — 应当修复
- **建议**: 改进机会 — 建议考虑修复
- **OK**: 确认正确，未发现问题
