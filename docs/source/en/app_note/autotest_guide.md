# Automated Test Framework User Guide

> For the design background, the layering model and the decision record, see
> `tools/sdk_test/DESIGN.md` in the repository (not published with the documentation site).
> This document only covers **how to use** the framework.

> **There are now examples you can actually run**: `example/test/unit_test/` (on-board Unity
> unit tests) and `example/get-started/hello_world/` (the console and pytest harnesses).
>
> Still **not in this batch of commits**: the QEMU carrier from section 4
> (`customer/boards/qemu_*`, `rtos/rtthread/bsp/sifli/drivers_qemu/`, `tools/qemu/`,
> `example/qemu/`), and the **two CMock example projects** from section 7.1 (the next
> change). Each place that mentions those two sections belongs to its own batch.
> For the full boundary, see the table at the top of `tools/sdk_test/DESIGN.md`.

---

## 1. The Two Halves

```
+-- SDK side (tools/sdk_test/, not bound to hardware) ---------------------+
| testcase.yaml -> discovery -> matching -> build -> run -> logs + report  |
| binding.yaml (hardware mapping) -> testbed list                          |
+--------------------------------------------------------------------------+
                                   | local HTTP
                                   v
                  +--------------------------------+
                  | Lab Agent (machine side)       |  owns the boards: serial / probe / relay
                  | tools/lab_agent/               |  hands out one testbed at a time
                  +--------------------------------+
                                   |
                              real board
```

**The division of labour is one sentence**: a test case declares only its **needs** (which fixtures, which peripherals it requires) and never contains a port number, a probe serial number or a mux channel. The physical reality all lives in the binding file on the Lab Agent side.

---

## 2. Quick Start (No Hardware Needed)

> **You can type this section out as it stands.** The tree contains two example test case projects
> you can list and plan: `example/get-started/hello_world/` (console and pytest)
> and `example/test/unit_test/` (on-board Unity unit tests).
> Only `example/qemu/` is in another batch along with the QEMU carrier, so `-T example` will be
> missing that one entry.

```bash
./export.ps1                      # Windows; on Linux/macOS use source ./export.sh
```

```bash
# See which test cases there are
sdk.py test list -T example
sdk.py test list -T example --json          # for CI to generate a matrix
sdk.py test info                            # which harnesses / backends this build supports

# See what hardware a lab has
sdk.py test list-fixtures --binding tools/sdk_test/examples/binding.local.yaml

# Dry run: work out the plan only, no build, no flashing
sdk.py test plan -T example/get-started/hello_world \
    --binding tools/sdk_test/examples/binding.local.yaml
```

Run the framework's own tests (no hardware needed, and no Lab Agent either):

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
# You can also collect them with pytest; the two are compatible
python -m pytest tools/sdk_test/tests -v
```

---

### 2.1 After a Run: Where the Logs and Reports Are

`test run` **produces artifacts by default** (aligned with twister), with no extra flags. When it finishes, the terminal prints:

```
[outdir] E:\...\test-out\2026-10-03T17-09-22     ← printed before the run starts; you can tail device.log from it
[plan]  3 test(s) to run:                        ← lists what is about to run, for watching progress
        1/3  rtt/hello_world.console    (sf32lb52-lcd_a128r16, console)
        2/3  rtt/pytest/hello_world.pytest  (sf32lb52-lcd_a128r16, pytest)
        3/3  no-os/hello_world.noos     (sf32lb52-lcd_a128r16, console)

------- [1/3] rtt/hello_world.console  (sf32lb52-lcd_a128r16, console) -------
[build] rtt/hello_world.console for sf32lb52-lcd_a128r16
[run]   rtt/hello_world.console via board
[PASS ] rtt/hello_world.console on sf32lb52-lcd_a128r16_01  (25.5s)

------- [2/3] rtt/pytest/hello_world.pytest  (sf32lb52-lcd_a128r16, pytest) -------
...
[log]    E:\...\test-out\2026-10-03T17-09-22\runner.log
[report] E:\...\test-out\2026-10-03T17-09-22\sifli.json
[report] E:\...\test-out\2026-10-03T17-09-22\sifli.xml

================= rtt  (2 test(s): 2 passed) =================                 ← the tail groups by testcase
[PASS ] rtt/hello_world.console on sf32lb52-lcd_a128r16_01  (25.5s)
[PASS ] rtt/pytest/hello_world.pytest on sf32lb52-lcd_a128r16_01  (21.9s)

================ no-os  (1 test(s): 1 passed) ================
[PASS ] no-os/hello_world.noos on sf32lb52-lcd_a128r16_01  (22.3s)

3 test(s): 3 passed, 0 failed, 0 errored, 0 skipped  in 69.8s
artifacts: E:\...\test-out\2026-10-03T17-09-22
```

**Listing the cases before the run starts, a separator line between cases, and a verdict the moment a case finishes** — all three are there for the sake of "watching progress":
in a long run the only question anyone asks is "how many are left", and the summary at the end cannot answer it.
The summary at the end is in turn **grouped by `testcase.yaml`** (one directory, one file, and usually one firmware),
one summary line per group, with the details of a failure indented directly beneath its line — no hunting back and forth.

**Each run gets its own timestamped directory**, so the previous run's logs are not overwritten — the log of a passing test case is exactly the baseline you want when chasing a flaky one.
The timestamp is local time in the `2026-10-03T17-09-22` format, and **sorting lexicographically sorts by time**, so the newest comes last in an `ls`.
Two runs started within the same second automatically get `-2`, `-3` suffixes instead of being merged into one tree.

```
test-out/                        ← --outdir, defaults to the directory you ran sdk.py from (gitignored)
  2026-10-03T17-09-22/
    runner.log                   run level: what this run did, and what it ran against
    sifli.json                   the main document
    sifli.xml                    JUnit, derived from the JSON
    qemu_sf32lb52x/
      hal_mock.spi_flash_status/
        build.log                the compiler verbatim (absent with --no-build)
        device.log               the device verbatim, including the boot banner
        runner.log               this instance's record
  2026-10-03T16-40-05/           ← the previous run, still there
    ...
```

**Reports and logs are two independent paths**: a report can be pinned to a fixed directory with `--report-dir` (CI needs a fixed path to collect the junit),
while logs still get a directory per run. When you do not specify it, the report lands in this run's timestamped directory too.

**Compiler output does not scroll through the console; it lands in `build.log`.** One `scons` run is hundreds of lines and the vast majority of it is noise,
and the moment you actually want to look at it (a build failure) is exactly when it has already scrolled past. So by default only one line is printed,
`[build] <id> for <board>`, and the full text goes into the file.

What each of the three files is for, and when to look at which:

| File | The question it answers |
|---|---|
| `build.log` | **What the compiler said**. The only place with an answer when a build fails — exit code 2 does not tell you whether a header is missing or a type is wrong |
| `device.log` | **What the device actually said**. Verbatim, byte for byte, no header and no separator lines, so you can `diff` it, `grep` it, or paste it into a ticket as-is |
| instance `runner.log` | **How the harness reached its verdict**: the verdict, the per-case table, the harness's own words, plus how much serial output was retained and whether any of it was trimmed |
| run-level `runner.log` | **What this run did**: how many were discovered and how many ran, which were filtered out and why, the time and the environment |

The one-sentence rule for troubleshooting: when a harness reports "I saw nothing", **only `device.log` can tell "the device produced no output" apart from "the device produced output but the harness did not read it"**.

On failure the terminal gives you the list directly, with no digging through files:

```
[FAIL ] hal_mock.spi_flash_status  (4.0s)  at run
        1 of 3 Unity test(s) failed: status_returned_as_reported
        FAIL  status_returned_as_reported (spi_flash:170): Expected 3 Was -559038737
        logs: <the directory above>
```

`at run` is the **failure stage**: `plan` / `build` / `open` / `run` / `close`. `at build` means this run never got as far as running,
and the problem is in the compilation rather than in the test case — that is the distinction most worth spotting at a glance in the pile of `ERROR`s. Adding `-i` replaces the `logs:` line with the full serial output.

**Retention policy**: nothing is ever deleted — the logs of passing test cases stay, and so do those of the previous run (twister deletes both).
To clean up, delete by directory: the timestamped directories under `test-out/` are independent of one another, so deleting one leaves the others alone.

---

## 3. Running on Real Hardware

Two terminals.

**Terminal A** — start the Lab Agent on the machine with the board plugged in:

```bash
cp tools/lab_agent/examples/runner.local.yaml my-runner.yaml   # edit it following the comments
python tools/lab_agent/runner.py -c my-runner.yaml --check     # validate the config first
python tools/lab_agent/runner.py -c my-runner.yaml --list      # see which testbeds were recognised
python tools/lab_agent/runner.py -c my-runner.yaml             # start the service
```

**Terminal B** — run the test cases:

```bash
sdk.py test plan -T example/get-started/hello_world --binding my-binding.yaml   # look at the plan first
sdk.py test run  -T example/get-started/hello_world --binding my-binding.yaml
```

Exit codes: `0` everything passed, `1` some test case failed, `2` the request itself is not usable.

### Running by hand: name a platform, skip the binding

**Most manual runs are this kind** — "grab a free board and run this batch". The `--binding`
is superfluous then; `-p/--platform` is enough:

```bash
sdk.py test run -T example -p sf32lb52-lcd_a128r16 --agent-url http://127.0.0.1:8760
```

It works without a binding because **allocation is the agent's job**: when the SDK side calls
`allocate`, it passes `testbed_id=None` anyway and lets the agent pick a free candidate. So the
allocation path is *identical* with or without a binding — what a binding adds is an **early
filter** on the SDK side.

The price is three checks that then happen *before the build* no longer happen at all, and are
deferred to the moment hardware is requested:

| What is lost | Consequence |
|---|---|
| fixture matching | a case needing a fixture nobody offers fails at `allocate`, not at `plan` |
| the binding ↔ `features.yaml` contradiction check | "there is a flash on spi1 but the board declares no spi" is no longer caught early |
| platform filter, phase two | whatever a case declares is overridden by `-p` (below) |

**`-p` overrides a case's own `integration_platforms`** — the flag wins. The whole tree is
pulled onto the platform you name, including cases pinned to a different one. That is exactly
what "run everything on this board" wants, but it does bypass the cases' own declarations, so
do not be surprised when a cross-platform case turns up.

**To run on several platforms**: the command line takes only one `-p` today. Running the whole
tree on two platforms means naming them in the cases' `integration_platforms:`, or running twice.

Common flags:

| Flag | What it does |
|---|---|
| `-T/--testsuite-root` | The directory to search recursively for `testcase.yaml` (default: the SDK's `example/`) |
| `-t/--testcase` | Run only the given id (repeatable). Both the full id and just the tests key are supported |
| `--tag` | Select by tag (union) |
| `-b/--binding` | The hardware mapping file |
| `-p/--platform` | Name a board; without it the binding decides |
| `--no-build` | Skip the build and use what is already in `build_<board>/` |
| `-j/--jobs` | Number of parallel build jobs |
| `--timeout` | Override the timeout of every test case |
| `--outdir` | Artifact root, default `test-out/`. **Each run creates a timestamped directory under it**, holding this run's run-level `runner.log`, the per-instance `device.log`/`runner.log`, and the reports |
| `-i/--inline-logs` | On failure, print the serial log **into the terminal** instead of only giving the path |
| `--report-dir` | Where the reports land; defaults to `--outdir` |
| `--report-name` | The report filename stem, default `sifli` (producing `<name>.json` and `<name>.xml`) |
| `--report-suffix` | Appended to the report filename to tell builds apart in CI (→ `sifli_ci123.xml`) |
| `--report` | **Compatibility alias**: pins the JUnit directory and filename (the extension is stripped, and the JSON lands under the same stem). Reports are produced by default now, so you normally do not need it |
| `--fail-fast` | Stop at the first failure (by default the run continues; nightly needs the full picture more) |
| `--no-confirm` | Never stop to ask mid-run. It only affects boards with manual reset (see section 8), and it also disables `unity` — that harness needs someone to confirm the board has booted |
| `--confirm-reset` | The other way round: do not stop to ask, but **declare the board ready**. On a manual-reset board this is what lets `unity` run (unattended); mutually exclusive with `--no-confirm` |

---

## 4. Running Without Hardware: the QEMU Carrier

> **The content of this section is in another batch of commits, along with the QEMU carrier.**
> The framework side's `backends/qemu.py` is already in the tree, but the boards these commands
> need (`customer/boards/qemu_*`), the BSP (`rtos/rtthread/bsp/sifli/drivers_qemu/`) and the
> examples (`example/qemu/`) are all still missing, so **typing them out will fail**. And the
> QEMU route itself is not settled yet, so this section describes the design intent.

A real board is not the only carrier. `qemu_cortex_m33` is an ordinary board name in the SDK,
so **the same test cases, with the same harnesses, can run with no hardware at all** — no Lab Agent,
no binding file, no wiring.

```bash
sdk.py test run -T example/qemu                      # console harness
sdk.py test run -T example/test/unit_test --no-build -t unit_test.cjson_qemu
```

The second one runs **the on-board Unity unit tests** (the cJSON suite), just on a different carrier.
The test case declares where it runs:

```yaml
  - id: unit_test.cjson_qemu
    harness: unity
    integration_platforms: [qemu_cortex_m33]   # with this, no binding is needed
```

You need QEMU installed locally; when it is not on PATH, point at it with `SIFLI_QEMU_EXE`
(on this machine it is at `E:\Program Files\qemu\qemu-system-arm.exe`).

### 4.1 QEMU Boards with a Chip Identity

`qemu_cortex_m33` is a **fake SoC** — it has not selected any chip. To touch anything chip-related
(`bf0_hal.h`, register maps, any code under `#ifdef SOC_SF32LB5xx`), you need these five boards:

| Board name | Chip identity |
|---|---|
| `qemu_sf32lb52x` | SF32LB52X |
| `qemu_sf32lb55x` | SF32LB55X |
| `qemu_sf32lb56x` | SF32LB56X |
| `qemu_sf32lb57x` | SF32LB57X |
| `qemu_sf32lb58x` | SF32LB58X |

The same QEMU machine (mps2-an505), the same `link.lds`, **only the chip identity differs**.

```bash
scons --board=qemu_sf32lb52x -j8            # build directly
sdk.py test run -T example/test/hal_mock               # the test cases declare their own platform
sdk.py test run -T example/test/hal_mock --no-build -p qemu_sf32lb55x   # run on another series
```

**What it does for you**: the board's `Kconfig.board` has `select SOC_SF32LB5<nnn>X`,
and the carrier side (`rtos/rtthread/bsp/sifli/drivers_qemu/SConscript`) uses that to add the chip's
include paths and the three macros `SF32LB<nnn>X` / `SOC_BF0_HCPU` / `USE_HAL_DRIVER`,
**and to guarantee the include order** — `bf0_hal.h` does `#include "register.h"` by name,
and the chip's copy has to win over the carrier's own `drivers_qemu/register.h`.

**So do not write any of this in a test project.** Injecting `-DSF32LB52X` yourself has two traps:
a compiler macro is **not the same as** a Kconfig symbol (the `#ifndef SOC_SF32LB55X` in `drv_spi_flash.c`
will not be satisfied), and you do not control the include order — what comes out is "missing register map" or
"unknown type name", which looks nothing like the root cause. `example/test/cmock_unit/` and
`example/test/hal_mock/` both follow this rule, so you can copy them directly.

**Note**: these boards only exist in the KCONFIG_V2 flow that `--board=` goes through; the old
`sdk.py menuconfig` without `--board=` cannot see them (in v1 the chip is a choice, mutually exclusive with `SOC_QEMU`).

**Why QEMU matters**: like the `pc` simulator, `BSP_USING_QEMU` makes `drivers/SConscript`
**skip the whole `drivers/` tree**. That is, the image contains no HAL implementation at all,
and a CMock-generated mock is the **only definition** of the symbol it replaces —
which is exactly the precondition requirement 3 (stubbing) needs. See `middleware/test_fwk/mocks/README.md` for details.

**A few traps (all of them silent failures)**:

- **Writes of a command must be rate-limited character by character** (the backend defaults to 20 ms/character). Writing a whole line at once drops characters,
  and what you see is a command that echoes back fine but never executes — because what got dropped is exactly the trailing carriage return and line feed.
- **Line endings must be CRLF**. Sending only `\r` gets dropped.
- A session is not handed over as soon as QEMU starts; it waits **until the device has finished booting and gone quiet** before handing over,
  otherwise the command is sent at a moment when the UART is not yet enabled, the whole line is dropped,
  and what you see is "no Unity summary line found" after waiting out the full timeout.
- The console **does not go through interrupts**; it is polled (measured: enabling the UART receive interrupt and unmasking
  every NVIC line produces no device interrupt at all). So do not expect it to reflect real interrupt behaviour.
- The `qemu_run.*` and `qemu_smoke_test.py` under `tools/qemu/` still use the old semihosting calls and are
  **out of date**; running them produces no output.

## 5. Writing a `testcase.yaml`

Put it in the directory of the test case it measures. **The id is generated from "relative path + tests key"**, so do not assemble it yourself.

```yaml
tests:
  - id: my_case
    description: one sentence on what this case verifies
    harness: console          # console | unity | pytest (shell not implemented)
    type: functional          # unit | integration | functional
    tags: [smoke]
    timeout: 60
    harness_config:
      ...
```

The full set of fields is defined by `tools/sdk_test/schema/testcase.schema.json`, which is **the single source of truth**:
a wrong field name is rejected outright rather than silently ignored.

Other usable fields: `platform_allow` / `platform_exclude` / `integration_platforms` /
`depends_on` (matches the instance types declared in the board's `features.yaml`, see section 8) / `extra_args` (passed through to scons) /
`skip` / `build_only` / `project_dir`.

**These fields are not all in the same state**, and it is worth knowing that before you write them — several are accepted by the schema but read by no code:

| Field | State |
|---|---|
| `depends_on` | Works. Matches the board's `features.yaml` and filters **before the build** (stage one). If it is absent, that file is not read at all. |
| `integration_platforms` | Works, and it is **the whole list**: write three and it runs three times. It is not a "preferred platform". |
| `platform_allow` / `platform_exclude` | Work; they still apply when `--platform` is given explicitly. |
| `extra_args` | Works, but what it is passed through to is the **scons build**, not the run. |
| `build_only` / `skip` | Work. `skip` is decided **before the build**, so it saves you a build. |
| `min_flash` / `min_ram` / `slow` / `type` | **Inert**: the schema accepts them and they are parsed into the model, but no code reads them. Writing them has no effect. |
| `harness: shell` | Legal in the schema but **not implemented**; selecting it raises an error rather than passing silently. |
| `dut` / `dut_client` | Parsed into the model, not yet consumed by the `runner`; multi-device currently has to be orchestrated by the `pytest` harness itself. |

### `project_dir`: the test case and the project are not at the same level

A `testcase.yaml` usually sits next to the project rather than inside it. By default it looks, by convention, for
"the directory `testcase.yaml` is in" or "its `project/` subdirectory";
when neither matches it searches within the two levels below for a unique one, and finding several is an **error rather than a guess**.
To point elsewhere, write `project_dir` (relative to the testcase file):

```yaml
    project_dir: ../project     # used by example/get-started/hello_world/rtt/pytest/
```

---

## 6. Writing Each of the Three Harnesses

### `console` — no test code on the device, the host matches serial output

```yaml
  - id: hello_world.console
    harness: console
    timeout: 60
    harness_config:
      record:
        - "- SiFli Corporation"     # regex, matched in declaration order
        - "Hello world!"
        - "msh />"
```

- Matching is done **on the accumulated buffer**, not line by line. `RT_CONSOLEBUF_SIZE=128` splits long lines apart,
  and there is no global console lock, so noise gets interleaved between lines — ignore it and write normally.
- The entries are matched **in order** by default (sharing one cursor). To demote an entry to "appears anywhere",
  write `ordered: false`.
- Repeat N times: `{regex: "tick", repeat: 3}`; a shortfall is reported as `(2/3)` rather than just "missing".
- Matching across lines: `{regex: "Hello world!", type: "multi_line"}` (line breaks are folded into spaces;
  `separator` lets you use something else).
- The failure message names **which entry is missing** and appends what the device actually output.

**When to use it**: the assertion is "several strings appear in order". Three lines of YAML is enough, and no dependency is needed.

### `unity` — Unity runs on the board, the host parses its own summary

```yaml
  - id: unit_test.lz4
    harness: unity
    timeout: 120
    harness_config:
      suite: lz4          # optional; without it, unity_run with no arguments

      prompt: "msh />"    # optional; default msh />. Waiting for it is what gates
                          # the send. Yours replaces it -- it does not add to it.
      # cmd: unity_run    # optional; default unity_run. What gets sent.
```

The two fields divide the work and fix the order: **`prompt` answers when it is safe to send, `cmd` answers
what gets sent**. `prompt` **replaces** rather than adds: write your own pattern and `msh />` is no longer
waited for — it is not "the default first, then yours". To accept several markers, alternate them in one
regex, e.g. `prompt: "msh />|READY"`, which means "either one opens the gate" — again not one after the
other. `cmd` replaces in the same way.

The host waits for `prompt` to match on the console output, and only then writes `cmd` out; it then parses
Unity's output contract:
`<file>:<line>:<case>:PASS|FAIL`, `<n> Tests <n> Failures <n> Ignored`, `OK|FAIL`.

Waiting for `prompt` before sending is load-bearing, not a safety net. On a board with no probe, the console is opened while
**the download has just finished writing and the board is still booting**; sending the command out at that moment means the board never receives it,
and the symptom is `no Unity summary line found` after the timeout is exhausted — **which looks like broken firmware**.

The two defaults are a matched pair rather than two arbitrary names: `cmd` sends `unity_run` by default, and
that is an **msh command**, which can only run if msh is already up; so `prompt` waits for `msh />` by default,
which is exactly that precondition — **assuming nothing beyond what sending an msh command already assumed**.
For a bare-metal firmware, or a different shell, point `prompt` at the readiness marker the firmware itself prints.

Not finding the prompt **is not a failure**: the command is sent anyway and the existing checks decide — otherwise a new heuristic
would invent a new kind of failure, misjudging firmware with a different prompt as broken.

**Two gates against a no-op run**: no summary line seen → failure; the summary says 0 test cases → failure.
(The latter is the most dangerous false green: linking Unity but registering no test case at all prints a clean `OK`.)

`harness: unity` does not accept `record` — the verdict belongs to Unity, and forcing one in raises an error telling you to use `console` instead.

#### How to run it on real hardware

`example/test/unit_test/` is a ready-made project: four test cases in one `testcase.yaml`, sharing one firmware.
Three of them can run on real hardware; `unit_test.cjson_qemu` is pinned to QEMU:

```powershell
# Dry run: confirm what gets planned first (no hardware touched)
sdk.py test plan -T example/test/unit_test -t unit_test.lz4 -b <binding.yaml>

# Real hardware: a manual-reset board needs --confirm-reset, otherwise unity is refused before flashing
sdk.py test run  -T example/test/unit_test `
    -t unit_test.lz4 -t unit_test.cjson -t unit_test.circular_buf `
    -b <binding.yaml> --confirm-reset
```

**Watch the scope of `-T`**: a bare `-T example/test/unit_test` also selects `unit_test.cjson_qemu`,
and since that one declares `integration_platforms: [qemu_cortex_m33]` — a platform this lab does not have —
`test plan` **aborts as a whole** (it is strict), while `test run` records it as an ERROR and carries on.
So either pick the three with `-t`, or accept one ERROR.

**With all three passing it looks like this**:

```
=================== unit_test  (3 test(s): 3 passed) ====================
[PASS ] unit_test.circular_buf on sf32lb52-lcd_a128r16_01  (23.4s)
[PASS ] unit_test.cjson on sf32lb52-lcd_a128r16_01  (18.6s)
[PASS ] unit_test.lz4 on sf32lb52-lcd_a128r16_01  (18.6s)
```

> As of this document it is actually **2 passed / 1 failed**: the `boundaries` case in `unit_test.lz4` is not closed yet,
> see risk 9 in `tools/sdk_test/DESIGN.md`. The above is the shape when everything is green.

On failure the terminal gives you **which sub-case** went red directly, with no digging through logs — **the one below is the actual current output**:

```
[FAIL ] unit_test.lz4 on sf32lb52-lcd_a128r16_01  (18.6s)  at run
        1 of 3 Unity test(s) failed: boundaries
        FAIL  boundaries (lz4:129): Expected 1024 to be less than 0
```

### `pytest` — the test case directory brings its own Python; use it when you need control flow

```
example/get-started/hello_world/rtt/pytest/
  testcase.yaml
  test_hello_world.py
```

```yaml
  - id: hello_world.pytest
    harness: pytest
    timeout: 180
    project_dir: ../project
    harness_config:
      pytest_root: .              # relative to the testcase file
      # pytest_args: [-x, -k, "smoke"]
```

```python
def test_the_console_is_bidirectional(dut):
    dut.send_line("")                      # ask first
    assert dut.expect(r"msh />", timeout=10), "the shell did not answer"
```

The `dut` fixture is loaded automatically by the framework (no `conftest.py` needed), and what it receives is
**the testbed the runner has already claimed**, not a new board request.

Available methods: `expect(regex, timeout)`, `send_line()`, `write()`, `read()`, `drain()`,
`reset()`, `expect_unity_test_output(timeout)`, plus `dut.testbed_id` / `dut.platform` /
`dut.build_dir`.

**Two semantics of `expect`** (both load-bearing; keep them in mind when writing test cases):

- **The cursor only moves forward, and it carries across cases.** `dut` is session-scoped, so "the device said X"
  means **from this case onward**. A prompt left over from boot does **not** satisfy the next case's
  `expect` — that would be a green light proving nothing. If you want a fresh prompt, `send_line` first.
- **A single serial read may deliver the text this case expects and the next case's text at the same time.** Matching happens on the accumulated buffer,
  searching forward from the cursor, so the earlier one does not swallow the later one's content. Do not write an implementation where one `expect`
  reads once and returns — that was a real bug once: the later one would wait out the entire timeout and then
  report that "the device said nothing", pointing in a completely wrong direction.

**When to use it**: the assertion depends on "ask, then answer", or you need to coordinate several boards, drive instruments, or retry.
For pure assertions it is only more verbose, and it costs you an extra pytest install.

> `pytest` is not a hard dependency of the framework: `console` and `unity`, and the framework's own tests, are pure standard library.
> pytest is only there if you installed it with `install.ps1` (it is already in `tools/locks/default`).

---

## 7. Writing On-Board Unity Unit Tests

For a project template, see `example/test/unit_test/`.

```c
/* One test case is just a function */
void test_lz4_roundtrip(void)
{
    TEST_ASSERT_EQUAL_INT(orig_len, LZ4_compress_default(src, dst, orig_len, cap));
}

/* See the Unity documentation for how assertions are written; a failure carries file:line */
```

**The registry is generated; do not write it by hand.** `src/SConscript` uses `test_registry.action(...)`
to read the `test_*.c` that **actually take part in this project's build**, and produces `unity_suites_gen.c`
(the `sifli_unity_cases[]` array — an ordinary array, not a linker section, so no linker script needs changing)
and a `.json` manifest of the same name.

So a test case declares only one thing: its definition. The suite name and the case name are derived by the rules below,
and you only need an extra line of hinting when the name you want differs from the derived one:

| You want | How to write it |
|---|---|
| suite name | Name the file `test_<suite>.c`; when the filename does not fit, write a line `suite: <name>` in the file |
| case name | The function name minus `test_<suite>_` (`test_lz4_roundtrip` → `roundtrip`); for another name, write a line `case: <name>` in the comment block above the function |

```c
/* Malformed input has no partial success: the parser must report NULL.
 *
 * case: malformed
 */
void test_cjson_malformed_returns_null(void)
```

The hint must be **on a line of its own** (`/* case: malformed */` also works), so that a
"case:" appearing in prose is not taken for a hint, and it applies only to the function directly below it.

**Platform gating is automatic too**: if a suite's module cannot compile on a given carrier, `SrcRemove` it in
`src/SConscript` and it disappears from the registry along with it — no more writing an `#if` in the registry.

Turn these on in `proj.conf`:

```
CONFIG_RT_USING_UNITY=y
CONFIG_USING_TEST_FWK=y
```

Device-side commands: `unity_run [suite|case|suite.case]`, `unity_list`.

**Mind the EOL**: `rt_kprintf` expands `\n` into `\r\n`, `middleware/test_fwk` already handles that,
and the host-side parser tolerates both LF and CRLF.

### 7.1 Stubbing: CMock

> CMock itself and the `mock_*.c/h` generated artifacts **are committed along with this change**; the two projects below are for the next one.
> And the `BSP_USING_QEMU` they depend on (section 4) is not in this batch either, so these commands cannot run yet
> — they wait for the QEMU carrier to catch up.

When the branch you want to test cannot be reproduced on the bench ("erase failure", "write timeout", "ID mismatch"),
replace that dependency with a CMock-generated mock. Two ready-made projects to copy from:

| Project | What it stubs | Cases |
|---|---|---|
| `example/test/cmock_unit/` | middleware functions (`circular_buf_*`, `ipc_hw_*`) | 5 |
| `example/test/hal_mock/` | **HAL functions** (`HAL_QSPI_GET_SR`) | 3 |

```bash
sdk.py test run -T example/test/cmock_unit
sdk.py test run -T example/test/hal_mock
```

Both need a **board with a chip identity** (see 4.1), because the module under test includes `bf0_hal.h`.

**The mechanism is "do not compile the mocked implementation", not interception**: `BSP_USING_QEMU` makes
`drivers/SConscript` skip the whole `drivers/` tree, the image contains no HAL implementation at all,
so the mock is the only definition of that symbol — no duplication, no weak symbols, no touching the ROM table.

Three points that really do bite:

1. **The size of a mock.** CMock has no switch for "generate only these few functions"; it can only generate a whole header.
   `bf0_hal_mpi_ex.h` declares 593 APIs, the generated artifact is 588 KB, and the test case uses 1 of them.
   This is a deliberate trade-off (the generated artifacts are committed and never hand-edited), but be aware of it.
2. **`GlobalExpectCount` / `GlobalVerifyOrder` must be defined by the test project itself.** With
   `:enforce_strict_ordering` turned on, the generated mock only `extern`-declares and assigns them, and never defines them.
   The link error names only the symbol, which is easy to misread as a missing library.
3. **`_CallCount()` is not a call count**, it counts callback invocations, and under an ordinary `_ExpectAndReturn` it is always 0.
   To prove "it was called", rely on `_Verify()` consuming the expectations (call it in tearDown), plus
   `:fail_on_unexpected_calls`.

**Always do the reverse verification after writing one**: deliberately make a mock expectation mismatch, and the test case must FAIL.
Without that step you cannot prove the mock is really linked in — if what got linked is the real implementation, changing the mock would produce no reaction at all.

For the details (how the generator is run, how `:strippables` is written, why the chip identity must come from the board,
why the link-time closure is only 2 symbols), see `middleware/test_fwk/mocks/README.md`.

---

## 8. The binding File: Describing a Lab

```yaml
testbed:
  - id: sf32lb52-lcd_n16r8_01
    platform: sf32lb52-lcd_n16r8      # must correspond to customer/boards/<name>
    runner: local                     # which Lab Agent holds it
    console:
      port: COM13
      # baud defaults to 1000000, the SDK console's baud rate
    connections:                       # corresponds to the testcase's harness_config.fixture
      - fixture: spi_nor_flash:spi1
        pinmap:
          - {pin: PA00, signal: CS}
```

**What a board can do is not written here.** `depends_on` matches **the board's own declaration** —
`customer/boards/<board>/<core>/features.yaml`, which says which bus instances this board has
(`spi1`, `i2c2`; an empty list is a legal declaration too, which is what the QEMU carrier has). The binding only says
**what is connected to what**. Writing the same fact in two places is exactly how the two come to contradict each other.

```yaml
# testcase.yaml: what it wants
    depends_on: [spi]                  # “there must be an spi” — not which bus
    harness_config:
      fixture: spi_nor_flash           # “a NOR flash is attached to the board” — not where it is attached
```

The test case writes a **type name**, the board writes **instance names**, and a type match accepts any instance of that type;
writing an instance name (`depends_on: [spi2]`, `fixture: spi_nor_flash:spi2`) anchors it exactly.
Both axes use the same rule — because which bus has what attached to it is the board's business, and the test case's own code
(`BSP_USING_SPI1` and the like) has already settled it.

**Same-named fixture = equivalent**: when several testbeds offer the same fixture, a test case must reach the same conclusion whichever of them it lands on.
If two boards do not satisfy that, they should not declare the same fixture.

The Lab Agent's own settings (probe serial numbers, flashing method) are written in this file too;
`tools/sdk_test` only parses the parts it needs and keeps the rest as-is:

```yaml
    jlink:
      # device: SF32LB52X_NOR     # if absent, read from customer/boards/<platform>/hcpu/rtconfig.py
      serial: "59605049"          # use -SelectEmuBySN to pick when there are several probes
      # wdt_script: tools/segger/halt_all_cpu_and_disable_all_wdt_56x.jlink
    flash:
      method: jlink               # jlink (probe) | sftool (UART)
    reset:
      method: jlink               # jlink (probe, default) | manual (a person presses the reset button)
```

### Manual reset (`reset.method: manual`)

For boards **without a probe**: the framework does not touch a probe at all, and the board is started externally (a download, opening a port, or a person).

```yaml
    reset:
      method: manual
```

#### With a terminal: it stops once for confirmation

```powershell
sdk.py test run ...        # run it in a terminal
```

After the console opens:

```
[reset] sf32lb52-lcd_a128r16_01 (sf32lb52-lcd_a128r16) has no probe.
        Press the board's RESET button if it has not already booted,
        then press Enter to run the test.
  continue? [Y/n]
```

**You most likely do not need to press it.** When a UART download (`flash.method: sftool`) finishes, the ROM jumps to the new image —
the board has already started itself once; and opening the console port makes pyserial drive DTR/RTS, which may reset it once more.
So pressing Enter is usually all it takes, and this prompt is only a fallback.

Answering `n` (or Ctrl-D) records this test case as **skipped**, not failed — the case never ran,
and calling it broken would be an assertion with nothing behind it.

#### Three modes, one table

| Command | Does it stop | How the board comes up | `unity` |
|---|---|---|---|
| default (with a terminal) | stops and waits for Enter | a person watches | ✅ |
| `--confirm-reset` | does not stop | the caller declares it has booted | ✅ |
| `--no-confirm` / no terminal (CI) | does not stop | nobody is watching | ❌ refused **before** the download |

The last two **do not block**; they print a single line of notice, and the window is the test case's own `timeout` (60s by default):

```
[reset] sf32lb52-lcd_a128r16_01 (sf32lb52-lcd_a128r16) has no probe -
waiting for the board to boot; press its RESET button if nothing appears (60s)
```

Giving `--no-confirm` together with `--confirm-reset` is an outright error — one says "nobody is watching", the other says
"it has already been taken care of", and both cannot be true at once.

#### Why `unity` can run on this board

`unity_run` is an **msh command**; send it too early and the shell does not exist yet, so the command is lost — the symptom is
`no Unity summary line found` after the timeout is exhausted, **which looks like broken firmware**.

So the `unity` harness **waits for the shell prompt first** (default `msh />`) before sending the command.

**Why not "wait for the serial port to go quiet"**: we tried it, and it does not work. The board is also quiet before it has started booting,
and at that moment the console has just been opened and the boot triggered by the download has not arrived yet — "silence" was taken for
"finished talking", and the command was lost just the same. **What is needed is a positive signal, not the absence of one.**

When the prompt is already in the buffer (a probe reset, QEMU) this step passes immediately, at zero cost.

```yaml
      prompt: "msh />"    # optional; default msh />. Yours replaces it, it does
                          # not get waited for in addition.
```

Not finding the prompt **is not a failure**: the command is sent anyway and the existing checks decide — otherwise a new heuristic
would invent a new kind of failure, misjudging firmware with a different prompt as broken.

#### Three things to watch

- **With no terminal, passing does not mean somebody pressed it.** A green light only means "the expected content appeared on the serial port after the console opened" —
  this boot may have come from the download, from opening the port, or from a person. The framework cannot tell them apart, and does not pretend to.
  With a terminal, that Enter at least means somebody looked.
- **`wdt_script` cannot run**: the watchdog will not be disabled. If a test case runs long, the board may be reset by the watchdog partway through.
- **Pressing late pollutes the next test case**: press after the previous case has `release`d and the boot log lands in the buffer
  the next case has just opened — a probe reset does not have this problem.

Templates: `tools/sdk_test/examples/binding.local.yaml` (the SDK side)
and `tools/lab_agent/examples/runner.local.yaml` (the machine side, with an extra global `runner:` block).
**Do not commit real COM ports.**

---

## 9. Wiring It Into CI

Reports are produced by default, so there is no need to write `--report` in your script; the logs and the report are both in this run's timestamped directory,
and the `device.log` / `runner.log` you need to look at on every failure are collected along with the ordinary artifacts (`when: always`, otherwise you get nothing precisely when it fails).
`test list --json` can be used to generate the matrix (so board names are not hard-coded in the CI configuration).

**In CI, use `--report-dir` to pin the report to a fixed path** — once the directory carries a timestamp, the path `test-out/sifli.xml` no longer exists.
Logs still get a directory per run (artifact globs are recursive, so they are unaffected):

```yaml
test:
  script:
    - sdk.py test run -T example -b $BINDING --outdir test-out --report-dir test-out/reports
  artifacts:
    when: always
    reports:
      junit: test-out/reports/sifli.xml
    paths:
      - test-out/**/*.log
      - test-out/**/device.log
```

`--report-suffix` is for telling report filenames apart across parallel or repeated runs (the same as twister);
without it the report overwrites the file of the same name under `test-out/reports/` every time — that is deliberate, CI only cares about the last one.
`test-out/` is already in `.gitignore`.

(Only the hook is left in this change; no CI stage was enabled.)

---

## 10. Where Test Data Lives

**Test code lives in the same repository as the code under test** (one review has to be able to look at them together);
**test data goes through external storage**, and both are expressed with one mechanism: **a vector is a fixture**.

```yaml
    harness_config:
      fixture: audio_sample_16k     # declare the need only, no path
```

The test does not know the path, so the same test case runs both on a laptop with a cache and in CI against remote storage.
This repository already has `jfrog.sifli.com/artifactory` (a conan remote) that can carry it.

**Do not commit large vectors to git**: git history only grows, and a 200 MB file committed once
stays in **every** clone forever, and deleting it afterwards does not help.

> A resolver that turns `fixture:` into "download + verify the hash + land in a local cache" is **not implemented yet**;
> the interface is left in `tools/lab_agent/fixture_drivers/`.
> None of the acceptance cases need a vector at present, so there is no urgency yet.

---

## 11. Known Limitations and Troubleshooting

**Not implemented**

| Item | State |
|---|---|
| `harness: shell` | raises an explicit error, will not spin silently |
| `backends/pc_sim.py` | stub (the `pc` simulator is still at the design stage) |
| Multi-device scheduling (requirement 4) | the schema and the plugin interface support it; the actual scheduling is not done |
| Relay / power control | `transport/relay.py` has only the interface and raises `RelayNotAvailable` |
| `backends/qemu.py` | **implemented**: the QEMU carrier can run console/unity/pytest, see section 4 |
| CMock stubbing cases | **delivered** as two projects (middleware + HAL), see section 7.1 and `middleware/test_fwk/mocks/README.md` |
| Per-instance `build.log` | **implemented**: compiler output goes to disk instead of flooding the console |

**Troubleshooting**

- `Cannot reach the Lab Agent` → terminal A was not started, or `--agent-url` is wrong.
- `every testbed matching the request is busy` → somebody is holding it; `sdk.py test list-fixtures -a <url>` shows who.
- `missing fixture(s) [...] (testbed offers: ...)` → this lab does not have what the test case needs;
  both sides are listed in the message, so compare them directly.
- A build failure is recorded as an ERROR for that test case and does **not** abort the whole round — so one round shows you every problem at once.
  **`at build` is that case**: the `stage` field in the terminal and in the artifacts tells you directly which stage failed (`plan`/`build`/`open`/`run`/`close`),
  with no guessing. The compiler's own words are in **`build.log`** in the same directory (the failure message points at it too).
- The serial port will not open: the Lab Agent side reports the port as occupied — check whether another terminal tool is connected to the same board.
- Seeing `console: N bytes retained, M trimmed from the head` in a log → the serial output exceeded the backend's 4 MB
  retention window and **the head was cut off**. This line appears only in the header of the instance `runner.log`; do not treat a trimmed `device.log` as the whole thing.

**Two things you must remember**

1. **Do not derive the expected strings from the source code**; fix them in `testcase.yaml` and keep them aligned with the actual output.
2. **Do not use mocks for on-board unit tests**: a real board reports its results over the serial console, and "not compiling `drivers/hal`"
   takes out the clock/UART/MPI along with everything else. Stubbing can only live on a non-board carrier.
