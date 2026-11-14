# SF32LB57 HCPU/ACPU SMP prototype

This experimental GCC port extends RT-Thread 3.1.3 to schedule threads on
HCPU (CPU 0) and ACPU (CPU 1) from one application image. Both cores share
kernel objects and ready queues. It targets SF32LB57 HCPU projects and is
disabled by default; it is not a production SMP release.

The repository includes an [SMP example](../../example/multicore/smp/README_EN.md)
([中文](../../example/multicore/smp/README.md)) for
`dpi-hdk_lb57gyd7n6_epd_hcpu`. Build and flash that HCPU project normally;
there is no separate `acpu.bin`.

## Configuration

Options are defined in [the kernel Kconfig](src/Kconfig). Defaults below are
SDK defaults; the example explicitly enables SMP and shared XIP.

| Option | Default | Effect / requirement |
| --- | --- | --- |
| `RT_USING_SMP` | `n` | SF32LB57 HCPU with the GCC Cortex-M33 port; selects semaphore support. |
| `RT_SMP_SHARED_XIP` | `n` | Execute shared code through the common MPI2 Flash system-bus alias. Without it, shared code is copied to SRAM. |
| `RT_SMP_PSRAM_NONCACHE` | `y` when shared XIP is enabled | Map shared PSRAM as noncacheable on both cores. |
| `RT_SMP_PSRAM_WT_SYNC` | `n` | Write-through PSRAM with kernel synchronization; requires shared XIP and `RT_SMP_PSRAM_NONCACHE=n`. |
| `RT_SMP_SRAM_WT_SYNC` | `n` | Cache ACPU application BSS, SRAM heap and stacks as write-through; requires shared XIP and one of the two PSRAM policies above. |
| `RT_SMP_CACHE_SYNC` | Internal | Selected by either WT option; applications do not enable it directly. |

Either WT option requires `CONFIG_PSRAM_CACHE_WB=n`, even with noncacheable
PSRAM and SRAM-only WT. The PSRAM setting defaults to `y` in the
[board-driver Kconfig](../../customer/boards/Kconfig_drv); leaving it enabled
with `RT_SMP_CACHE_SYNC` triggers a compile-time error. The example READMEs
provide complete settings for PSRAM-only WT, SRAM-only WT and both together.

SMP excludes the AMP ACPU service (`ACPU_CALLER_ENABLED`), PM, signals,
modules, LWP, software timers and scheduler-lock debugging. ROM RTOS and
precompiled RTOS objects are unsupported. Existing public function signatures
remain available, but `struct rt_thread` gains `bind_cpu` and `oncpu` fields.
Rebuild all code that allocates or accesses RTOS objects; binary compatibility
with the single-core thread layout is not preserved.

## Startup, scheduling and thread lifetime

HCPU initializes the C runtime, memory and kernel objects once. From an HCPU
thread, call `rt_err_t rt_smp_start_secondary(void)` after preparing shared
application state. ACPU starts through a vector table in SRAM and joins the
same scheduler without repeating C runtime initialization. The call returns
`RT_EOK` on startup, `-RT_EBUSY` when called on ACPU or after startup,
`-RT_EINVAL` for an incompatible shared-XIP memory policy, or `-RT_ETIMEOUT`
if ACPU does not start scheduling within one second.

Include [rtsmp.h](include/rtsmp.h) for the affinity extensions. New threads
default to HCPU, keeping existing SDK services on that core. Use the existing
`rt_thread_control()` API to change affinity:

```c
rt_thread_control(thread, RT_THREAD_CTRL_BIND_CPU, (void *)0); /* HCPU */
rt_thread_control(thread, RT_THREAD_CTRL_BIND_CPU, (void *)1); /* ACPU */
rt_thread_control(thread, RT_THREAD_CTRL_BIND_CPU,
                  (void *)RT_CPU_DETACHED); /* Either CPU */
```

The affinity command returns `RT_EOK`, or `-RT_EINVAL` for an invalid CPU
value. ACPU execution requires shared placement of the entire reachable code
and data described below, regardless of whether affinity is fixed or detached.

PendSV saves the outgoing stack and FP context before the scheduler releases
the thread's `oncpu` ownership under the global lock. Eager FP stacking avoids
migrating an unfinished lazy FP frame. The global tick and timer list advance
on HCPU; ACPU SysTick advances its current thread's time slice. Each core has
its own interrupt nesting state and idle thread.

Forced `rt_thread_delete()` or `rt_thread_detach()` of a thread that still has
CPU ownership returns `-RT_EBUSY`. Running threads exit by returning from
their entry function or calling `rt_thread_exit()`. An internal HCPU `reaper`
thread, with a 2 KiB noncacheable stack, waits for ownership release and
reclaims retired threads. It can wait on allocator synchronization without
suspending idle. Cleanup callbacks run inside the existing kernel critical
section and must remain nonblocking.

## Shared code and constants

Both SDK GCC HCPU linker templates,
[link.lds](../../drivers/cmsis/sf32lb57x/Templates/gcc/hcpu/link.lds) and
[link.jinja2](../../drivers/cmsis/sf32lb57x/Templates/gcc/hcpu/link.jinja2),
place the shared kernel objects, including mempool and SLAB allocator objects,
and application `.smp_text` / `.smp_rodata` sections. No example-specific
linker rewriting is required. Shared placement alone does not validate every
allocator configuration.

- With `RT_SMP_SHARED_XIP=y`, shared code and constants execute at the common
  system-bus VMA (`0x620xxxxx` for the example's MPI2 Flash partition) and are
  programmed at the corresponding LMA (`0x120xxxxx`). The alias offset is
  exactly `0x50000000`. ACPU cannot execute from the HCPU-only code-bus window.
- With shared XIP disabled, the same shared inputs are placed in the SRAM
  copy region. Secondary boot, low-level IRQ/lock routines and vectors also
  remain in SRAM.
- `RT_SMP_TEXT` marks an application function and `RT_SMP_RODATA` marks a
  constant for shared placement; both become empty when SMP is disabled.
  See the worker, helper and seed table in the
  [example source](../../example/multicore/smp/src/main.c).

These attributes do not relocate a function's callees, callbacks, string
pools, other referenced constants or compiler/library helpers automatically.
Check the final map and disassembly for the complete ACPU call and data-access
paths. HCPU-only drivers and arbitrary precompiled libraries must not become
reachable from ACPU merely because the thread entry has `RT_SMP_TEXT`.
Custom linkers must preserve the shared sections and SRAM cache boundary.
Kernel LTO and non-GCC linkers are outside this port's build scope.

## Shared data and existing IPC/heap APIs

Mailbox C4EXR supplies the recursive global kernel lock; local interrupts are
masked while it is held. Mailbox C3/C4 interrupt bit 15 supplies interprocessor
interrupts. These resources are reserved for SMP and cannot be shared with
the AMP ACPU service.

The existing [IPC implementation](src/ipc.c) still uses
`rt_hw_interrupt_disable()` / `rt_hw_interrupt_enable()`, which map to the
global lock in an SMP build. The existing [heap](src/mem.c) and
[memheap](src/memheap.c) implementations retain their semaphore locks.
Memheap checks available capacity after acquiring its heap semaphore, so
the check sees shared state refreshed at the SMP acquisition boundary.
`rt_enter_critical()` / `rt_exit_critical()` also hold the global lock and
mask local interrupts until the outer matching exit, protecting legacy SDK
object/heap traversal. No separate SMP allocator or IPC API is introduced.
Keep global critical sections short and nonblocking. A local-only interrupt
mask is not a cross-core synchronization boundary.

Noncacheable PSRAM maps `0x60000000-0x61ffffff` as shareable Normal memory,
read/write and execute-never, while leaving I/D caches enabled. The example
board has 16 MiB of physical PSRAM. Use its canonical `0x600xxxxx` addresses;
cached aliases of shared data are unsupported.

The optional WT policies use the same `RT_SMP_CACHE_SYNC` protocol. On an
outer kernel unlock, the core drains stores and marks its peer's cache stale
before releasing the hardware lock. The peer invalidates its local D-cache
on its next outer acquisition if needed. Nested acquisitions do not repeat
the maintenance. Synchronization flags and kernel control state stay in
noncacheable SRAM. Writable write-back mappings are incompatible with this
protocol and are rejected by configuration/startup checks.

With `RT_SMP_SRAM_WT_SYNC=y`, the linker supplies the 32-byte-aligned
`__smp_sram_cache_start__` boundary. Initialized data and static kernel control
BSS remain noncacheable. Ordinary application BSS and the following SRAM
heap/stacks are write-through on ACPU; HCPU keeps its direct SRAM path.
The `.smp_nocache` section is reserved for low-level state that must be read
before acquiring the kernel lock. Startup checks the selected MPU policy
before allowing ACPU scheduling.

Shared writable data must use RT-Thread IPC or the global kernel critical
sections to publish and acquire ownership. The protocol also brackets saved
thread contexts before migration and reclamation. It does not make
unsynchronized `volatile` polling, lock-free atomics or cached aliases coherent.
Whole-cache invalidation adds lock hold time and interrupt latency; the WT
options are experiments, not general performance guarantees.

## Example coverage and limits

The in-tree example exercises semaphore payload handoff, mutex-protected
updates, forced migration with stack/FP checks, dynamic thread reclamation,
and cross-core allocation/free in SRAM and, when configured, PSRAM and a
fixed-block memory pool. It checks that all pool blocks are returned. With
`RT_USING_HOOK` enabled, the scheduler callback checks that both thread
arguments are non-null without counting callback invocations. Its
README documents the commands and current validation status. The example is
not a benchmark or an endurance qualification.

The current example passed three default-configuration reset runs on
SF32LB57GYD7N6, each completing 32 rounds, 64 threads and 2048 migrations
with zero errors and restored heap accounting. The example READMEs list the
10 hardware configurations covering both GCC linker paths, NC/WT policies,
SRAM execution, SMP-disabled regression and a reduced SLAB configuration.
The native SDK console testcase also passed.

SLAB passed only with DFS, Finsh and Audio disabled for the check. The full
board configuration reported `SMP_FAIL thread_create` due to insufficient
memory and is not counted as passing; the project defaults were not changed.
A Jinja build with both WT policies completed 1000 rounds, 2000 worker
threads and 64000 migrations with zero errors and restored heap accounting
in approximately 91 seconds including startup. This is not an endurance claim.

The hook negative-control firmware emitted `SMP_FAIL scheduler_hook` and
was rejected by the console matcher. Targeted host checks passed 24 tests
and 153 subtests. These checks do not extend the scope to the exclusions below.

ACPU fault handling retains four 32-bit debugger fields (`ap_fault_cfsr`,
`ap_fault_hfsr`, `ap_fault_pc`, `ap_fault_lr`); the last records EXC_RETURN.
It does not call HCPU logging or acquire the kernel lock.

Other SDK driver/middleware concurrency, DMA ownership, Flash program/erase
while the peer executes XIP, executable PSRAM, runtime MPU changes and
production endurance remain outside the validation scope. Default HCPU
affinity alone is not a concurrency audit of all SDK services.

With `RT_USING_SMP=n`, the build selects the original `scheduler.c` and
single-core context-switch assembly. The shared suspend/resume atomicity
fixes also apply to that configuration.
