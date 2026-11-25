# SF32LB57 SMP example

[中文](README.md)

Source code path: example/multicore/smp

This example demonstrates symmetric multiprocessing (SMP) on SF32LB57 HCPU and ACPU using one application image and a shared RT-Thread 3.1.3 kernel. It uses the existing thread, semaphore, mutex and heap APIs to check cross-core data handoff, thread migration, floating-point/stack context and dynamic thread reclamation. It provides a starting point for writing explicitly shared worker threads.

The port is experimental. New threads are bound to HCPU by default; only threads with explicitly changed affinity can run on ACPU. See the [SMP port notes](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md) for the complete scope.

## Usage

Install the SDK environment and GCC toolchain using [SiFli-SDK Getting Started](../../../docs/source/en/quickstart/index.md). Open PowerShell in the SDK root and run:

```powershell
. ./export.ps1
cd example/multicore/smp/project
```

The example uses the standard `project/` layout and builds one HCPU application, `main.bin`, shared by both cores. No separate `acpu.bin` build or download is needed.

## Supported Boards

- `dpi-hdk_lb57gyd7n6_epd_hcpu`: SF32LB57GYD7N6; see the [board description](../../../customer/boards/dpi-hdk_lb57gyd7n6_epd/doc/index.md).

### Hardware Required

Use the board above, a power supply and a USB cable/serial adapter connected to the HCPU debug UART. The board's 16 MiB PSRAM is used for shared-heap checks. The example does not require a display connection or operation. No buttons or other interaction are needed after startup.

### Configure the Project

[proj.conf](project/proj.conf) enables SMP, shared Flash execution and memheap. It disables incompatible features including the AMP ACPU service, PM, software timers, signals and modules.

| Option | Example default | Description |
| --- | --- | --- |
| `RT_USING_SMP` | `y` | Enable SF32LB57 HCPU/ACPU SMP. |
| `RT_SMP_SHARED_XIP` | `y` | Execute shared code through the MPI2 Flash address accessible to both cores. |
| `RT_SMP_PSRAM_NONCACHE` | `y` | Use the Kconfig default: noncacheable shared PSRAM on both cores. |
| `RT_SMP_PSRAM_WT_SYNC` | `n` | Optional write-through PSRAM synchronization; first disable the noncacheable PSRAM option. |
| `RT_SMP_SRAM_WT_SYNC` | `n` | Optional write-through synchronization for ACPU SRAM data/stacks. |
| `RT_USING_MEMPOOL` | `y` | SDK default; enables the example's cross-core fixed-block memory-pool checks. |
| `RT_USING_HOOK` | `y` | SDK default; enables the example's scheduler-switch callback argument checks. |
| `EXAMPLE_SMP_ROUNDS` | `32` | Number of dynamic thread-creation rounds, from 1 to 10000. |

To change the configuration, run from `project/`:

```powershell
sdk.py menuconfig --board=dpi-hdk_lb57gyd7n6_epd_hcpu
```

The SDK linker scripts place shared code and constants. `RT_SMP_TEXT` and `RT_SMP_RODATA` mark only the specified function or constant. Callees, string pools and compiler/library helpers must also be placed at addresses accessible to both cores. These attributes do not automatically adapt an entire application for SMP.

### Optional WT Configuration

Either WT mode selects the internal `RT_SMP_CACHE_SYNC` option and requires `CONFIG_PSRAM_CACHE_WB=n`, including SRAM-only WT. The board's PSRAM configuration inherits a default of `y` for `PSRAM_CACHE_WB`. Leaving it enabled causes the compile-time error `WT synchronization cannot coexist with writable write-back mappings.`.

Choose one policy below, replace the corresponding settings in `project/proj.conf`, retain the other project settings and rebuild all affected code. The WT options can be enabled independently or together; do not set `RT_SMP_CACHE_SYNC` manually.

PSRAM WT only:

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=n
CONFIG_RT_SMP_PSRAM_WT_SYNC=y
CONFIG_RT_SMP_SRAM_WT_SYNC=n
CONFIG_PSRAM_CACHE_WB=n
```

SRAM WT only, keeping PSRAM noncacheable:

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=y
CONFIG_RT_SMP_PSRAM_WT_SYNC=n
CONFIG_RT_SMP_SRAM_WT_SYNC=y
CONFIG_PSRAM_CACHE_WB=n
```

Both PSRAM and SRAM WT:

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=n
CONFIG_RT_SMP_PSRAM_WT_SYNC=y
CONFIG_RT_SMP_SRAM_WT_SYNC=y
CONFIG_PSRAM_CACHE_WB=n
```

### Build and Flash

From `project/` with the SDK environment initialized, run:

```powershell
scons --board=dpi-hdk_lb57gyd7n6_epd_hcpu -j8
```

Artifacts and download scripts are generated under `project/build_dpi-hdk_lb57gyd7n6_epd_hcpu/`. Rebuild all affected code after changing SMP or cache configuration; do not reuse RTOS objects from the previous configuration.

Use the generated download scripts as described in [SDK Build and Download](../../../docs/source/en/quickstart/build.md). For Windows UART download, run from the same `project/` directory:

```powershell
.\build_dpi-hdk_lb57gyd7n6_epd_hcpu\uart_download.bat
```

Enter the board's actual serial port number and reset it as prompted. After downloading, open the HCPU console using the baud rate in the current board console configuration.

### Automated Execution

After configuring a lab-agent connected to the board, run from the SDK root with the SDK environment initialized:

```powershell
sdk.py test run -T example/multicore/smp -p dpi-hdk_lb57gyd7n6_epd_hcpu
```

See the [Automated Testing Guide](../../../docs/source/en/app_note/autotest_guide.md) for setup and report details. The console testcase checks the default 32 rounds; if `EXAMPLE_SMP_ROUNDS` changes, update its expected totals accordingly.

## Example Output

This output was captured from the current example on SF32LB57GYD7N6 using the GCC `link.lds` path, shared XIP and noncacheable PSRAM, with SRAM WT disabled. Only line endings and the shell prompt have been normalized:

```text
SMP_BEGIN
SMP_RESULT rounds=32 threads=64 migrations=2048 errors=0 heap_restored=1 cpus=3 ticks=354
SMP_PASS
```

The same firmware passed three reset runs, reporting `ticks` values of 354, 352 and 354. Each run completed 32 rounds, 64 threads and 2048 migrations with no reported errors and restored heap accounting. `cpus=3` is the bitmask indicating that both cores were visited.

The `ticks` field is the elapsed RT-Thread tick count. It depends on clocks, cache policy and scheduling load, so it is not a fixed acceptance value. The values above apply only to the default configuration.

A failed check prints `SMP_FAIL` with the failing stage. Disabling SMP produces `SMP_DISABLED`.

All example output comes from HCPU; the ACPU workers do not print logs.

### Validation Scope

The current source completed hardware validation of these 10 configurations on SF32LB57GYD7N6:

| Linker path / configuration | Coverage | Result |
| --- | --- | --- |
| GCC `link.lds`, SMP enabled | Default noncacheable policy, PSRAM+SRAM WT, shared code in SRAM with XIP disabled | 3 passed |
| GCC `link.jinja2`, SMP enabled | Default noncacheable policy, PSRAM-only WT, SRAM-only WT, both WT | 4 passed |
| Both linker paths, SMP disabled | Single-core configuration regression | 2 produced the expected `SMP_DISABLED` |
| GCC `link.lds`, minimal SLAB configuration | Run the example with the SLAB allocator | 1 passed, subject to the condition below |

The SLAB pass applies only to the reduced test configuration with `CONFIG_RT_USING_DFS=n`, `CONFIG_RT_USING_FINSH=n` and `CONFIG_RT_USING_AUDIO=n`. The original full board configuration ran out of memory and reported `SMP_FAIL thread_create`; it is not counted as a pass. The project's default configuration was not changed for this check.

The native SDK [smp.console testcase](testcase.yaml) passed. A separate Jinja configuration with both WT policies completed 1000 stress rounds: 2000 worker threads, 64000 migrations, `errors=0`, `heap_restored=1` and `ticks=90492`, taking approximately 91 seconds including startup.

The hook negative-control firmware emitted `SMP_FAIL scheduler_hook` and was rejected by the console matcher. Targeted host checks passed 24 tests and 153 subtests. These results do not establish endurance qualification.

## Example Breakdown

[src/main.c](src/main.c) follows this sequence:

1. HCPU initializes IPC objects and the configured PSRAM heap, fixed-block memory pool and scheduler hook, then calls `rt_smp_start_secondary()` to start ACPU.
2. Each round dynamically creates two threads, initially bound to HCPU and ACPU respectively. They pass a patterned payload through semaphore handoffs and protect a counter with a mutex.
3. Each thread switches to the other core 32 times, checking stack data and a floating-point accumulator. The default 32 rounds create 64 threads and perform 2048 migrations.
4. One core allocates and fills a block; the other receives it through a semaphore, checks it and frees it. Round numbers start at zero. Odd rounds use the 16 KiB example PSRAM heap; with `RT_USING_MEMPOOL` enabled, some even rounds use a fixed-block pool backed by 2 KiB of storage. The remaining rounds use the SRAM heap. The PSRAM path is compiled only when PSRAM, shared XIP and a valid shared-PSRAM policy are enabled. Disabling the corresponding feature uses the SRAM heap path instead.
5. Workers return to exit, and the HCPU `reaper` reclaims them. The main thread waits for cleanup notifications and restored heap accounting before starting the next round. At completion, it checks the PSRAM heap balance, that all memory-pool blocks were returned, and aggregate counters.
6. With `RT_USING_HOOK` enabled, the scheduler-switch callback checks that both thread pointers it receives are non-`NULL`. This check does not count callback invocations.

## Troubleshooting

- `SMP_DISABLED`: check that the final build configuration enables `RT_USING_SMP`.
- `SMP_FAIL secondary_start=...`: check shared code placement, MPU attributes and whether the AMP ACPU service was enabled accidentally. Return values are described in the [SMP port notes](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md).
- `SMP_FAIL completion_timeout`, or only `SMP_BEGIN`: check ACPU startup and reachable code. Inspect `ap_fault_cfsr`, `ap_fault_hfsr`, `ap_fault_pc` and `ap_fault_lr` with a debugger.
- `SMP_FAIL round=...`, `psram_leak`, `pool_leak` or `counters`: inspect data handoff, thread reclamation and heap/pool accounting. Retain the full log and final configuration; successful thread creation alone does not establish a successful run.
- `SMP_FAIL scheduler_hook`: the scheduler-switch callback received a null thread pointer; inspect the conditions under which the callback is invoked.

The example does not cover DMA ownership handoff, peripheral-driver concurrency, Flash programming/erasing while the peer executes XIP, lock-free shared data or endurance qualification.

For technical questions, open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference

- [SMP port notes](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md)
- [SMP extension header](../../../rtos/rtthread/include/rtsmp.h)
- [Project configuration](project/Kconfig.proj) and [defaults](project/proj.conf)
