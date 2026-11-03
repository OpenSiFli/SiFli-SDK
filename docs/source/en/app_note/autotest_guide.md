# Automated Test Framework User Guide

> For the design background, the layering model and the decision record, see
> `tools/sdk_test/DESIGN.md` in the repository (not published with the documentation site).
> This document only covers **how to use** the framework.

> **There are now examples you can actually run**: `example/test/unit_test/` (on-board Unity
> unit tests) and `example/get-started/hello_world/` (the console and pytest harnesses).
>
> The QEMU carrier from section 4 (`customer/boards/qemu_*`,
> `rtos/rtthread/bsp/sifli/drivers_qemu/`, `tools/qemu/`, `example/qemu/`) and the **two CMock
> example projects** from section 7.1 are in the tree now too, so the commands run.
> For the full boundary, see the table at the top of `tools/sdk_test/DESIGN.md`.

---

## 1. The Two Halves

```
+-- SDK side (tools/sdk_test/, not bound to hardware) ---------------------+
| testcase.yaml -> discovery -> matching -> build -> run -> logs + report  |
| (instance, platform) pairs -> the Lab Agent                              |
+--------------------------------------------------------------------------+
                                   | local HTTP
                                   v
                  +--------------------------------+
                  | Lab Agent (machine side)       |  holds the binding and the boards: serial / probe / relay
                  | tools/lab_agent/               |  hands out one testbed at a time
                  +--------------------------------+
                                   |
                              real board
```

**The division of labour is one sentence**: a test case declares only its **needs** (which fixtures, which peripherals it requires) and never contains a port number, a probe serial number or a mux channel. The physical reality all lives in the binding file on the Lab Agent side.

---

## 2. Quick Start (No Hardware Needed)

> **You can type this section out as it stands.** The tree contains three example test case projects
> you can list and plan: `example/get-started/hello_world/` (console and pytest),
> `example/test/unit_test/` (on-board Unity unit tests) and `example/qemu/`.

```bash
./export.ps1                      # Windows; on Linux/macOS use source ./export.sh
```

```bash
# See which test cases there are
sdk.py test list -T example
sdk.py test list -T example --json          # for CI to generate a matrix
sdk.py test info                            # which harnesses / backends this build supports

# Dry run: work out the plan only, no build, no flashing
sdk.py test plan -T example/get-started/hello_world \
    -p sf32lb52-lcd_a128r16
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

**Terminal B** — run the test cases. **The only two things you state are which platform to run on, and where
the agent is**:

```bash
sdk.py test plan -T example -p sf32lb52-lcd_a128r16     # look at the plan first (no hardware touched)
sdk.py test run  -T example -p sf32lb52-lcd_a128r16     # the agent allocates a free board
```

`--agent-url` already defaults to `http://127.0.0.1:8760`, so it can be omitted while the agent is on the
same machine as you.

Exit codes: `0` everything passed, `1` some test case failed, `2` the request itself is not usable.

**The SDK holds no inventory of the lab.** A run is (instance, platform, backend, build dir) and nothing
finer: which *board* of that platform will host it is not decided here and cannot be, because the agent
allocates from its own configuration at the moment hardware is requested. So naming a platform is the
whole of what you say — the hardware map never enters this side at all (it belongs to the hardware team,
see section 8).

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
| `-p/--platform` | Name a board to build for; needed unless every test case names its own `integration_platforms` |
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
| `--no-confirm` | Never stop to ask mid-run. It only affects boards with `reset.method: manual` (see section 8). Giving it means **nobody answers** "has the board booted", so the harnesses that write before they read are refused, that is, **`unity` without `autorun`**; `console`, `pytest` and unity with autorun are all unaffected |

### 3.1 How the build reaches the board

**It is sent as one package, so the two machines need not share a filesystem.** The build happens
here; the SDK side packs the **two manifests** in `build_<board>/` (`download.jlink`,
`sftool_param.json`) together with the images they name into one tar and POSTs it to the agent. The
agent unpacks it into a directory of its own and **flashes from there**. The rest of
`build_<board>/` -- tens of thousands of object files -- is not sent; for this board the whole
package comes to 420 KB.

So the agent can run on **another machine**: it never needs to see your build directory. Uploading
the same build twice is fine -- the agent names a package after its contents, so it unpacks one
copy. A testbed's packages go away when it is released.

The package carries its `platform`, and **the agent checks it**: only the side that built the image
knows which board it was for, and without the check an A-board image programmed into a B board
succeeds all the way to a result nobody can explain. It also carries the **device name the probe
needs** (`SF32LB52X_NAND` and the like) -- that is not in `download.jlink`, and without it a remote
agent can only read the board file in its own checkout, which may be a different revision.

### 3.2 When the agent is not the same code as your SDK

The agent is a **long-running process**, quite possibly started days ago from another checkout,
while your SDK keeps changing. Two layers:

- **The protocol version**: a mismatch is **refused outright**, and the message names both sides.
  It only moves when the interface genuinely stops being compatible -- "a field I send that you
  cannot read" -- which is not something a version number can guess at.
- **The two revisions**: **recorded, never checked**. A dirty worktree is a normal working state,
  and making it a gate would refuse to run exactly when somebody is mid-change. `sifli.xml` carries
  `sdk_revision`, `agent_revision` and `sdk_protocol` in its `<properties>`, which is where "which
  two revisions produced this result" is answered afterwards. Differing revisions are not an error
  by themselves, but they are the only clue to the most common real accident here: a long-running
  agent still holding old code.

---

## 4. Running Without Hardware: the QEMU Carrier

> **Everything this section needs is in the tree now**: the framework side's `backends/qemu.py`,
> the boards (`customer/boards/qemu_*`), the BSP (`rtos/rtthread/bsp/sifli/drivers_qemu/`) and the
> examples (`example/qemu/`), so the commands below run as written.

A real board is not the only carrier. `qemu_sf32lb52x` is an ordinary board name in the SDK,
so **the same test cases, with the same harnesses, can run with no hardware at all** — no Lab Agent,
no binding file, no wiring.

```bash
sdk.py test run -T example/qemu                      # console harness
sdk.py test run -T example/test/unit_test --no-build -t unit_test.qemu
```

The second one runs **the on-board Unity unit tests** (the three suites in the same image), just on a different carrier.
The test case declares where it runs:

```yaml
  - id: unit_test.qemu
    harness: unity
    timeout: 120
    integration_platforms: [qemu_sf32lb52x]   # with this, no binding is needed
    harness_config:
      autorun: true
```

You need QEMU installed locally; when it is not on PATH, point at it with `SIFLI_QEMU_EXE`
(on this machine it is at `E:\Program Files\qemu\qemu-system-arm.exe`).

### 4.1 QEMU Boards with a Chip Identity

All five QEMU boards **select a chip** — there is no chipless carrier. To touch anything
chip-related (`bf0_hal.h`, register maps, any code under `#ifdef SOC_SF32LB5xx`), pick the board
whose series matches:

| Board name | Chip identity |
|---|---|
| `qemu_sf32lb52x` | SF32LB52X |
| `qemu_sf32lb55x` | SF32LB55X |
| `qemu_sf32lb56x` | SF32LB56X |
| `qemu_sf32lb57x` | SF32LB57X |
| `qemu_sf32lb58x` | SF32LB58X |

The same QEMU machine (mps2-an505), the same `link.lds`, **only the chip identity differs** — and
that identity has to be in the board name, since `--board=<name>` resolves to
`customer/boards/<name>/hcpu` and a testcase names its platform by board name.

> There used to be a sixth carrier, `qemu_cortex_m33`, that selected no chip and served as a fake
> SoC. It is **gone**: the cases that need `bf0_hal.h` (CMock stubs, the HAL tests, `unit_test`)
> could not compile against it at all, and the rest run just as well on a board that names a chip.

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

**Why QEMU matters**: like the `pc` simulator, `SOC_QEMU` makes `drivers/SConscript`
**skip the whole `drivers/` tree**. That is, the image contains no HAL implementation at all,
and a CMock-generated mock is the **only definition** of the symbol it replaces —
which is exactly the precondition requirement 3 (stubbing) needs. See `middleware/test_fwk/mocks/README.md` for details.
(Skipping is the default: a project that declares `QEMU_HAL_DRIVER_ENABLED` takes the tree back out of
it — all of `drivers/` but `cmsis/`, see 7.2 — that is the route the real-HAL cases take.)

**A few traps (all of them silent failures)**:

- **Writes of a command must be rate-limited character by character** (the backend defaults to 20 ms/character). Writing a whole line at once drops characters,
  and what you see is a command that echoes back fine but never executes — because what got dropped is exactly the trailing carriage return and line feed.
- **Line endings must be CRLF**. Sending only `\r` gets dropped.
- A session is not handed over as soon as QEMU starts; it waits **until the device has finished booting and gone quiet** before handing over,
  otherwise the command is sent at a moment when the UART is not yet enabled, the whole line is dropped,
  and what you see is "no Unity summary line found" after waiting out the full timeout.
- The console **does not go through interrupts**; it is polled (measured: enabling the UART receive interrupt and unmasking
  every NVIC line produces no device interrupt at all). So do not expect it to reflect real interrupt behaviour.
- The `qemu_run.*` and `qemu_smoke_test.py` under `tools/qemu/` match the backend: the same
  `-display none -monitor none -serial stdio`, with `SIFLI_QEMU_EXE` pointing at the binary when
  QEMU is not on PATH. Use them to run an image outside `sdk.py test`.

## 5. Writing a `testcase.yaml`

Put it in the directory of the test case it measures. **The id is generated from "relative path + tests key"**, so do not assemble it yourself.

```yaml
tests:
  - id: my_case
    description: one sentence on what this case verifies
    harness: console          # console | unity | pytest (shell not implemented)
    tags: [smoke]
    timeout: 60
    harness_config:
      ...
```

The full set of fields is defined by `tools/sdk_test/schema/testcase.schema.json`, which is **the single source of truth**:
a wrong field name is rejected outright rather than silently ignored.

**Both ways of writing it are supported.** `tests:` can be a list (each entry carrying its own `id`), or a
mapping (**the key is the id**):

```yaml
tests:
  my_case:                   # the key is the id, no separate id: line needed
    harness: console
    harness_config:
      record: ["Hello world!"]
  my_other_case: ~           # an empty entry = id only, everything else takes the default
```

**There is also a top-level `common:`**: a set of values written there merges into **every** entry of
`tests:`, and **each entry's own key of the same name wins**:

```yaml
common:
  tags: [smoke]
  timeout: 60
tests:
  - id: my_case              # inherits the tags and timeout above
    harness: console
```

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
| `min_flash` / `min_ram` / `slow` / `type` / `dut` / `dut_client` | **Removed**: the schema no longer accepts them, so writing one is rejected outright. They used to be parsed and read by nothing. Why they went, and what they were for (resource limits, marking slow cases, multi-device), is recorded in the TODO section of `tools/sdk_test/DESIGN.md`. |
| `harness: shell` | Legal in the schema but **not implemented**; selecting it raises an error rather than passing silently. |

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

A test case looks like this:

```yaml
  - id: unit_test.board
    harness: unity
    timeout: 120
    harness_config:
      autorun: true       # the firmware runs every suite itself at power-on; the host only reads the port
      # suite: lz4        # optional. Write it and you claim only lz4's results, see below
```

**One test case = one build + one flash + one boot.** The project calls `sifli_unity_autorun()` once in
`main()` (`CONFIG_TEST_FWK_AUTORUN`, already turned on in the test project's `proj.conf`), so the board runs
**every** suite in the registry at power-on, prints all the results into one transcript, and the host just
reads the serial port — **you send no command, and you do not have to worry about what moment to send it at**.

**So do not split entries by suite.** Which suites an image contains is a **build fact**: `src/SConscript`
has already scanned the `test_*.c` actually compiled into `unity_suites_gen.json`. Listing them again here
would write one fact in two places; and splitting them into several entries only turns into several more
flashes and several more boots.

#### When several test cases share one image: `suite` claims results

One autorun writes the results of **all** suites into a single transcript, so a test case may have to
declare **who** it is:

```yaml
      suite: lz4          # optional. Write it and you claim only lz4's results
```

The host dispatches by the suite name in the result line `<file>:<line>:<suite.case>:PASS|FAIL`. A test case
that does not write `suite` is responsible for the whole transcript — **which is how it is written when a
project has one test case** (see the real-hardware example below).

Two things to remember:

- **Not a single result appeared for your own suite → failure.** A transcript has only one summary line
  (`<n> Tests <n> Failures <n> Ignored` — one `UnityBegin`/`UnityEnd` around the whole loop, so that line
  belongs to the **firmware**, not to any single suite). Handing the firmware's overall `OK` to a suite that
  ran nothing is a green light that proves nothing.
- **Another suite failing does not affect you.** That is the actual shape on real hardware: in one
  transcript cjson and circular_buf come out PASS and lz4 comes out FAIL, independent of each other.

**Two gates against a no-op run**: no summary line seen → failure; the summary says 0 test cases → failure.
(The latter is the most dangerous false green: linking Unity but registering no test case at all prints a clean `OK`.)

> **Why the host is not made to send `unity_run`**: there is no reliable moment to send it — the boot banner
> is printed once, and before the prompt; and the prompt's text in turn follows the build configuration.
> Send the command too early and it is lost, and the symptom is `no Unity summary line found`, which looks
> like broken firmware. Handing the decision to the firmware makes that timing problem go away. For the full
> argument, see `tools/sdk_test/DESIGN.md`.

#### `prompt` / `cmd` are needed only when the firmware waits for a command

Turn `CONFIG_TEST_FWK_AUTORUN` off (set it to `n` in `proj.conf`) and the board stops at the shell waiting
for a command; only then does the host need those two fields. Running a single suite by hand goes that way
too: flash once, then type `unity_run lz4`.

**The reverse holds too**: writing `cmd` or `prompt` alongside `autorun: true` is **refused outright** —
that path does not read those two keys, and a key that has no effect is an error in this framework, never
silently ignored.

```yaml
    harness_config:
      prompt: "msh />"    # optional; the default is the msh prompt (msh [^>\n]*>, which
                          # takes both `msh >` and `msh />`). Wait for it to appear in the
                          # console output before sending the command; writing your own
                          # string replaces it, it does not add to it
      # cmd: unity_run    # optional; default unity_run. The command itself, sent once the wait is over
```

`prompt` answers **when it is safe to send**, `cmd` answers **what gets sent**. Both are **replacing**:
write your own string and the default is no longer used — it is not "the default first, then yours". To
accept several markers, alternate them in one regex, e.g. `prompt: "msh />|READY"`, which means "either one
opens the gate" — again not one after the other.

Waiting for `prompt` is load-bearing, not a safety net: on a manual-reset board, the console is opened
while **the download has just finished writing and the board is still booting**, and sending the command
out at that moment means the board never receives it. The two defaults are a matched pair — `cmd` sends
`unity_run` by default, and that is an **msh command**, which can only run if msh is already up; so `prompt`
waits for the msh prompt by default (`msh [^>\n]*>`, which takes both spellings), which is exactly that
precondition. For a different shell, point `prompt` at the readiness marker the firmware itself prints.

Not finding the prompt **is not a failure**: the command is sent anyway and the existing checks decide.

**Two gates against a no-op run**: no summary line seen → failure; the summary says 0 test cases → failure.
(The latter is the most dangerous false green: linking Unity but registering no test case at all prints a clean `OK`.)

`harness: unity` does not accept `record` — the verdict belongs to Unity, and forcing one in raises an error telling you to use `console` instead.
`autorun` is in turn unity-only: `console` and `pytest` never read it, and receiving it raises an error.

#### How to run it on real hardware

`example/test/unit_test/` is a ready-made project. One `testcase.yaml` holds **two test cases** — one for real
hardware, one pinned to QEMU — sharing one firmware, and that firmware contains three suites (lz4, cjson,
circular_buf).

**One test case = one build + one flash + one boot**, and every suite in the image runs within that one boot.
So do not split entries by suite: splitting would mean three flashes and three boots.

```powershell
# Dry run: confirm what gets planned first (no hardware touched)
sdk.py test plan -T example/test/unit_test -t unit_test.board -p sf32lb52-lcd_a128r16

# Real hardware. This bench's reset.method is `auto` — the download brings the board up by
# itself, so the run needs no extra action. Autorun's results start from the first byte the
# board boots, so that boot has to happen after the serial port is open, or nobody sees it.
sdk.py test run  -T example/test/unit_test -t unit_test.board `
    -p sf32lb52-lcd_a128r16
```

**The `unit_test.qemu` one is pinned to `qemu_sf32lb52x`**, a platform this lab does not have, so
`test plan` **aborts as a whole** (it is strict), while `test run` records it as an ERROR and carries on.
So pick the real-hardware one with `-t unit_test.board`.

**With it passing it looks like this** (one test case, nine cases):

```
=================== unit_test  (1 test(s): 1 passed) ====================
[PASS ] unit_test.board on sf32lb52-lcd_a128r16_01  (19.0s)
```

> As of this document it is actually **1 failed**: `lz4.boundaries` is not closed yet, see
> risk 9 in `tools/sdk_test/DESIGN.md`. The above is the shape when everything is green.

On failure the terminal gives you **which sub-case** went red directly, with no digging through logs — **the one below is the actual current output**:

```
[FAIL ] unit_test.board on sf32lb52-lcd_a128r16_01  (19.0s)  at run
        1 of 9 Unity test(s) failed: lz4.boundaries
        FAIL  lz4.boundaries (test_lz4.c:129): Expected 1024 to be less than 0
```

In the report each case is identified as **`suite.case`** (for example `lz4.boundaries`): case names repeat
across suites (cjson and lz4 both have `boundaries`), and without the suite there is no telling them apart
— which is also exactly the filter syntax `unity_run` accepts. The parentheses hold the real source file
and the line the assertion tripped on, so a failure reads as a plain `file:line` location.

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

### Negative verification: do it before submitting

An over-permissive test case also passes on a working device, so "it is green on
the board" is not evidence that the assertions are right. Before submitting,
verify in the other direction and answer one question: **what broken behaviour
would still let this case pass?** If you can answer, the assertion is too weak.

`console` cases can do this entirely offline, without occupying a board: feed
synthetic transcripts to the real `ConsoleMatcher` and confirm the cases that
should fail do fail. Cover all four defects -- first, middle or last occurrence
missing content, and out-of-order output.

**The multi-occurrence trap**: `ordered` does not bind entries to the *same*
occurrence. A later entry only has to find its text after the cursor, so when an
earlier occurrence is missing content it borrows that text from the next one.
For example, splitting a report into `CPU Usage:` + `run_time` + `tidle` groups
still passes when the first report printed only its title and the second is
complete: `run_time` and `tidle` are taken from the second. To assert that one
occurrence is complete, anchor it inside a single match (e.g. use the boot
banner as the only anchor and `(?!...).` to forbid crossing the next title), or
switch to `pytest` and split the transcript into blocks -- usually more readable.

For `pytest` / `unity` cases, deliberately break one expectation (or make the
command fail) once and confirm the run goes `FAIL`, then put it back.

Before submitting:

- [ ] the normal path passes on a real board (or the intended carrier)
- [ ] at least one "feature is broken" input was tried, and the case did fail
- [ ] multi-occurrence assertions survive the first/middle/last-defect and shuffled matrices
- [ ] expected strings come from the actual `device.log`, not from source or README
- [ ] you can answer: what broken behaviour would still let this pass?

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

> CMock itself and the `mock_*.c/h` generated artifacts came in with an earlier commit; the two projects below
> and the `SOC_QEMU` they depend on (section 4) are in the tree now too, so these commands run locally.

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

**The mechanism is "do not compile the mocked implementation", not interception**: `SOC_QEMU` makes
`drivers/SConscript` skip the whole `drivers/` tree, the image contains no HAL implementation at all,
so the mock is the only definition of that symbol — no duplication, no weak symbols, no touching the ROM table.
(Skipping is the default: a project that declares `QEMU_HAL_DRIVER_ENABLED` takes the tree back out of
it — all of `drivers/` but `cmsis/`, see 7.2 — and `hal_mock` declares no such thing, so the paragraph above is its case.)

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

### 7.2 Running the Real HAL: Registers in RAM

7.1 replaces a HAL **function** with a mock and measures whoever calls it; this one goes the other way, compiling
the HAL **implementation** into the image and executing it. The first answers "what does the driver do when the
HAL reports this state", the second "are the words the driver writes to the peripheral right". Neither covers
the other: the second's assertion grain — the exact word written, the exact place a channel lands — is not
something a bench can give you.

**The mechanism**: SiFli's HAL is STM32-style — the handle's `Instance` is the register base, and every access in
the driver is written `hptc->Instance->FIELD`. So a test can declare a register block of its own and hand over its
address:

```c
static PTC_TypeDef g_ptc_regs;
g_ptc.Instance = &g_ptc_regs;
HAL_PTC_Enable(&g_ptc, 1);
TEST_ASSERT_EQUAL_HEX32(expected, g_ptc_regs.IER);
```

Every word the driver writes lands in that variable. **No linker script change, no reserved memory and no address
remapping** are involved: the block is an ordinary variable whose address the linker chooses, and the carrier's own
console at `0x40200000` is nowhere near it.

**Getting the HAL into the image**: `SOC_QEMU` makes `drivers/SConscript` skip the whole `drivers/` tree,
so running the real HAL in a carrier image means the project has to narrow that skip. It does so by declaring a
switch of its own in `Kconfig.proj`:

```kconfig
config QEMU_HAL_DRIVER_ENABLED
    bool
    depends on SOC_QEMU
    default y
```

`drivers/SConscript` reads it and compiles the tree back out of the skip, by the SDK's own walk — all of
`drivers/` but `cmsis/`, whose chip bring-up does not compile on a carrier and whose `startup_bf0_hcpu.S`
would be a second `Reset_Handler` beside the carrier's own — **the project lists no files at all**, and the
objects land under `build_qemu_sf32lb52x_hcpu/sifli_sdk/drivers/<subdir>/` by the same rule every other SDK
module follows.
`example/test/hal_real/project/Kconfig.proj` holds those three lines.

**Why the project declares that switch, and the driver layer does not offer it**: it has to be promptless,
or it would appear in `menuconfig`, which it must not; but a promptless symbol **cannot** be assigned in a
configuration file — `check_no_promptless_assign` in `tools/kconfig/kconfig.py` (invoked with
`--handwritten-input-configs` from `tools/build/building.py:3185`) fails the build outright with

```
is assigned in a configuration file, but is not directly user-configurable (has no prompt)
```

Satisfying both at once leaves one route: the value has to come from a `default`, and that means the project
that wants it declares it — the arrangement `example/lcpu_rom/Kconfig.proj:6-8` uses for `ROM_ATTR`. The cost
is that every HAL-testing project copies those lines again; the benefit is that a project which does not
declare it **cannot see the symbol at all**, which is how `example/test/hal_mock` keeps its image free of any
HAL implementation so that its CMock mock stays the sole definition of a HAL symbol (see 7.1).

**What that costs**: the switch brings the whole of `drivers/hal` back, not the two modules a case touches — 68
of its 76 `.c` on `qemu_sf32lb52x` (the other 8 are excluded by the existing per-chip logic), all 68 compiling
with zero diagnostics and no unresolved symbol at the link. The cost is build time: a clean `-j8` build goes
from ~18–21 s to ~28–31 s. It does not mean the whole subtree reaches the image either: gcc sections each
function (`-ffunction-sections`) and the link discards what nothing reaches (`--gc-sections`), so 3 of the 68
keep any code at all, 1,118 bytes. Three things you would not guess without measuring:

- **The image comes out about 4 KB *smaller*** (`.text` 33,453 → 29,501 B): `bf0_hal.c:56` defines
  `__WEAK __aeabi_unwind_cpp_pr0`, and libgcc's unwind objects (`libunwind.o` + `pr-support.o` +
  `unwind-arm.o`, 4,039 B) existed only to satisfy that reference, so `bf0_hal.o` on the link line retires
  them. It is not a benefit this project asked for — on a real board `bf0_hal.o` is always linked.
- **`drivers/hal` registers no init entries at all** (no `INIT_*_EXPORT` anywhere in it), so the usual worry
  that "the kept init table drags things in" does not apply here.
- **What does reach in is the vector table**: `bf0_hal.c:575` defines a **strong** `NMI_Handler` while
  `drivers_qemu/startup_qemu.S:112` declares it weak, so on this carrier the HAL's handler takes the slot.
  That is a real behaviour change and the tests cannot see it — NMI never fires under mps2-an505.

**The limit**: only entries that **take `Instance` from the caller** can be driven this way. A HAL file that
reaches for a global `hwp_*` pointer (RCC, pinmux, AON) has no handle to point anywhere, and needs a different
mechanism.

```bash
sdk.py test run -T example/test/hal_real     # no hardware, no Lab Agent, no binding
```

The project's two suites (`ptc`, `crc`) share one image and the host dispatches by suite name — the same rule as
section 6.

**What it establishes**: the driver's decision logic and its register choreography — which bits a value goes to,
which block a channel lands in, whether the table's constants are right, which word a polling pass writes. The
values written are readable afterwards, so a case asserts the **exact word** instead of inferring it from behaviour.

**What it does not establish** (these matter more than the lines above, and each case says so in its own header):

- **Timing, handshakes, interrupts, DMA completion: none of it.** A polling path is testable only because the test
  can **write the status bit beforehand**: that is what `test_crc.c` does with `CRC_SR_DONE`, and the wait loop in
  `HAL_CRC_Accumulate` reads back exactly what the test put there. The other way round, a driver that **polled the
  wrong bit, or never exited**, cannot be caught here.
- **RAM cannot tell a write-1-to-clear register from a plain one.** `ICR |= 1 << channel` clears an interrupt flag
  on silicon and merely sets a bit in RAM. That case therefore pins the **store** itself, not its effect.

**Three things the first run turned up** (they all cost time):

1. **`setUp`/`tearDown` is one pair per image, not one per suite.** Unity reaches the pair by name from
   `UnityDefaultTestRun()`, and `middleware/test_fwk/sifli_unity_runner.c` defines it `RT_WEAK` so a project can
   replace it — two suites in one image each defining it is two strong definitions, and the link stops:

   ```
   test_ptc.c:100: multiple definition of `setUp';
   test_crc.c:56: first defined here
   ```

   So the pair lives in `src/unity_main.c` (which already owns the other image-wide concern, autorun), and it calls
   the `*_fixture_reset()` each suite exports, with the registers staying private to their own file.

2. **`HAL_PTC_Enable()` without `HAL_PTC_Init()` writes to address 0, and says nothing.** `Chn` is derived by `Init`
   alone (`(PTC_ChnTypeDef *)&Instance->TCR1 + Channel`), and `Enable` never derives it; with `Init` left out the
   handle still holds the NULL `setUp` memsets into it, so `hptc->Chn->TCR = ...` lands at 0 — which on this carrier
   is **real memory** (`0x0` is an alias of `0x10000000`), not a fault. The assertion then reads zeros out of a
   register nothing wrote, and a case can pass **vacuously**: asserting "RCR stays 0" while the driver under test is
   writing to 0. The arrange step is part of the test.

3. **This carrier's shell prompt has no slash, so the host default could not match it and every instance
   burned its whole `timeout`.** `finsh` spells the prompt `"msh " + cwd + ">"`, and the cwd only appears
   when `RT_USING_DFS && DFS_USING_WORKDIR` is on — which the QEMU board's `board.conf` turns off, so what
   gets printed is `msh >` while the default was pinned to the cwd spelling and could never match. The symptom
   reads as "a slow device": the command does not go out until the timeout expires, which does not look like a
   pattern that never matched. Measured on this project: 132.5 s per instance → 13.8 s, run total 265 s → 27 s.
   The fix went into the framework, not the test case: `DEFAULT_PROMPT` in `tools/sdk_test/harness/unity.py`
   now accepts `msh [^>\n]*>`, which takes both spellings (`msh ` alone, or a `>` further down the log, does
   not), so this project needs no `harness_config.prompt` at all — that is a property of one carrier and does
   not belong in a test case file.

One thing worth knowing, though it is not about this section's mechanism: `HAL_PTC_GetData` in
`drivers/Include/bf0_hal_ptc.h` used to be spelled `(*(hptc->Chn->TDR))`, while `PTC_ChnTypeDef.TDR` is a plain
`__IO uint32_t`, so expanding it **could not compile**. Nothing in the SDK ever expanded it, which is how it
survived; `hal_real` is the first thing to — the last case in `test_ptc.c`, which reads a register back through
the HAL, is what turned it up, and the header has since been fixed (the stray `*` gone, with a comment naming the
project that found it). The case stays: it is the only one in the file that reads a register **through** the HAL
rather than writing one, and breaking the macro again would take it out.

The full account of the mechanism and its limits is in the project's own file headers: `testcase.yaml` (the whole
idea), `project/Kconfig.proj` (why the switch is the project's to declare), and `src/test_ptc.c` /
`src/test_crc.c` (what each case establishes and what it does not).

---

## 8. The binding File: Describing a Lab (the hardware team's)

**This file is usually not one you write.** It describes which boards are plugged into that machine, with
their serial ports and wiring — that is the job of whoever maintains the lab. A user only needs to know two
things: that it exists, and that **the `platform` names in it have to match what `-p` is given**; if you
would rather not touch it, just run with `-p` (see section 3).

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
the agent reads all of it, and no part of it reaches the SDK side:

```yaml
    jlink:
      # device: SF32LB52X_NOR     # if absent, read from customer/boards/<platform>/hcpu/rtconfig.py
      serial: "59605049"          # use -SelectEmuBySN to pick when there are several probes
      # wdt_script: tools/segger/halt_all_cpu_and_disable_all_wdt_56x.jlink
    flash:
      method: jlink               # jlink (probe) | sftool (UART)
    reset:
      method: jlink               # jlink (probe, default) | auto (the download brings it up) | manual (a person may press the button)
```

### 8.1 Field Quick Reference

**If you are the one writing this file (you are the hardware team, say), only two things must be right**:
`id` and `platform`. The rest is either the agent's own, or makes no difference to behaviour when read — the
table marks which, so copying it will not go wrong.

**A user's only correspondence is one line**: the name you give to `-p` has to equal some testbed's
`platform`.

| Key | Who reads it | Notes |
|---|---|---|
| `testbed[].id` | agent | The testbed's identifier within this run. It must not repeat within one file |
| `testbed[].platform` | **agent** | **Decides whether a test case can land on this board.** Must be named the same as `customer/boards/<name>`; without the core suffix is fine, it gets completed to `_hcpu` |
| `testbed[].console` | **agent** | `{port, baud}`. `baud` defaults to 1000000 (the SDK console is fixed at 1 Mbaud) |
| `testbed[].connections[].fixture` | agent | **Only consulted when a test case writes `harness_config.fixture`**. Write `name`, or `name:param` (`spi_nor_flash:spi2`) to anchor it exactly |
| `testbed[].connections[].pinmap` | none | Records how the wiring is done (which pin is CS/CLK). **No code consumes it yet**; writing it is for the hardware team to check the wiring against |
| `testbed[].connections[].mux` | agent (validation only) | Of the form `mux_01.channel.3`; used only to check that the mux it references really is declared under `muxes:` |
| `testbed[].muxes[]` | agent (`id` only) | Declares out-of-band switches (a relay and the like). Only `id` is read (for `connections[].mux` to reference); `type` / `channel_num` / `runner` are read by no code yet, and are kept for the fixture driver that will drive a relay later |
| `testbed[].runner` | agent (reporting only) | Which Lab Agent holds this testbed. It is reported in `/testbeds`, **but takes no part in the allocation decision** |
| `testbed[].jlink` / `flash` / `reset` | **agent only** | Probe, flashing, reset. The SDK side never sees this file — **so a probe serial number in it is not a mistake**; it is the agent's own configuration |
| `runner:` (top level) | **agent only** | Machine-level settings (listen address, poll interval, buffer cap, probe path). **The SDK side never reads them** |

> **One file, one reader**: `python tools/lab_agent/runner.py -c <f>` reads all of it — both the `testbed:`
> part and the machine-level `runner:` block. So one file serves as both the lab's hardware map and that
> machine's runner config. **Do not commit real COM ports.**

The sub-keys of `jlink:` / `flash:` / `reset:` are in the annotated template
`tools/lab_agent/examples/runner.local.yaml` — that one is written for people to copy from.

### Three reset modes: `jlink` / `auto` / `manual`

`reset:` says more than who presses the button — it also says **whether anything vouches for the boot**:

| `reset.method` | who resets it | does the run stop and ask | `unity` without `autorun` |
|---|---|---|---|
| `jlink` (default) | the probe | no | ✅ |
| `auto` | nobody — and nothing needs to: the download runs the new image, and that is the reset | no | ✅ |
| `manual` | nobody, **and nothing guarantees it** (a person may have to press the button) | yes, on a terminal | ❌ refused **before** the download |

`auto` and `manual` are both no-ops on the framework's side; the difference is the promise — **whether the
boot is bound to happen**. That promise is the only thing deciding whether an operator is asked and whether
a harness that writes to the device is allowed.

```yaml
    reset:
      method: auto      # the download reliably brings the board up (a UART download ends
                        # by running the new image, and opening the console drives DTR/RTS,
                        # which often resets it once more)
      # method: manual  # not guaranteed; somebody presses the button
```

**`auto` prints nothing.** The `[reset]` line is addressed **to an operator** ("press reset, then Enter"),
and `auto` has none to address — `jlink` says nothing either.

#### `manual`: with a terminal it stops once for confirmation

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
**When that holds every time, this bench's `reset.method` should say `auto`** — the same thing, with the asking taken out.

Answering `n` (or Ctrl-D) records this test case as **skipped**, not failed — the case never ran,
and calling it broken would be an assertion with nothing behind it.

That Enter is not a formality: on a `manual` bench **nothing else can bring the board up**, so the press
*is* the reset. That is exactly why somebody standing there is what lets a harness which writes to the
device run here at all.

#### `manual` with nobody to ask (`--no-confirm`, or no terminal)

It does not block; it prints a single line of notice, and the window is the test case's own `timeout`
(60s by default):

```
[reset] sf32lb52-lcd_a128r16_01 (sf32lb52-lcd_a128r16) has no probe -
waiting for the board to boot; press its RESET button if nothing appears (60s)
```

Here the harnesses that write before they read are refused — that is, **`unity` without `autorun`** —
and the refusal lands **before** the download.

#### Why `unity` can run on this board

**Because it no longer writes a command.** With `autorun` off, `unity_run` is only valid once the shell is
up, and sending it too early loses it — the symptom is `no Unity summary line found` after the timeout is
exhausted, **which looks like broken firmware**. With autorun the firmware prints the results itself and the
host only reads, so that timing problem no longer exists.

**What actually has to be guarded against on this board is something else**: an autorun's results start from
**the first byte the board boots**, so that boot must happen **after the serial port is open**. The console
is opened only once flashing has finished, and opening the serial port tends to drive DTR/RTS and reset the
board once more — the boot log is usually already sitting in the buffer. That "press reset if needed, then
Enter" confirmation is the fallback kept for exactly this.

(With `autorun` off and back on the send-a-command path, the host first waits for the shell prompt in the
console output (default `msh [^>\n]*>`, which takes both `msh >` and `msh />`) — that is the positive signal
for "is it safe to send?". **Not "wait for the serial port to go quiet"**: the board is also quiet before it
has started booting, we tried it, "silence" was taken for "finished talking", and the command was lost just
the same. Not finding the prompt is not a failure: the command is sent anyway and the existing checks decide.)

#### Three things to watch

- **With no terminal, passing does not mean somebody pressed it.** A green light only means "the expected content appeared on the serial port after the console opened" —
  this boot may have come from the download, from opening the port, or from a person. The framework cannot tell them apart, and does not pretend to.
  With a terminal, that Enter at least means somebody looked.
- **`wdt_script` cannot run**: the watchdog will not be disabled. If a test case runs long, the board may be reset by the watchdog partway through.
- **Pressing late pollutes the next test case**: press after the previous case has `release`d and the boot log lands in the buffer
  the next case has just opened — a probe reset does not have this problem.

Templates: `tools/sdk_test/examples/binding.local.yaml` (the hardware map)
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
    - sdk.py test run -T example -p $PLATFORM --outdir test-out --report-dir test-out/reports
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
| Real-HAL cases | **delivered**: `example/test/hal_real/` with two instances (`hal_real.ptc`, `hal_real.crc`), `sdk.py test run -T example/test/hal_real`, see section 7.2 |
| Per-instance `build.log` | **implemented**: compiler output goes to disk instead of flooding the console |

**Troubleshooting**

- `Cannot reach the Lab Agent` → terminal A was not started, or `--agent-url` is wrong.
- `every testbed matching the request is busy` → somebody is holding it; the Lab Agent's own
  `python tools/lab_agent/runner.py -c <cfg> --list` marks which testbeds are busy.
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
