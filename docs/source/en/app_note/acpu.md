# ACPU Usage Guide

## 1. ACPU Overview

The SF32LB57x and SF32LB58x series adopt a tri-core architecture: HCPU (high-performance computing), ACPU and LCPU (power-sensitive tasks such as BLE and sensor data processing). See [HAL index](../hal/index.md) for the full architecture description.

The ACPU role differs by chip series:

| Chip series | ACPU role |
|-------------|-----------|
| SF32LB57x | Audio and graphics processing |
| SF32LB58x | Audio DSP processing |

The ACPU runs an independent firmware image (RT-Thread based), which is built as a child project of the HCPU project and loaded into ACPU RAM by the second-stage bootloader at startup. The HCPU and ACPU communicate through the inter-core IPC Queue built on top of the mailbox.

### 1.1 Typical Use Cases

- **Audio codec offloading**: opus encoder/decoder runs on ACPU, e.g. `example/multimedia/audio/opus`.
- **Voice 3A processing**: ACPU tasks `ACPU_TASK_audio_3a_open/uplink/downlink/close` handle 3A processing.
- **Graphics rendering**: EPIC render list execution on ACPU (`ACPU_TASK_epic_rl`), e.g. `example/rt_device/gpu/render_list_mode`.
- **Custom computation tasks**: user-defined tasks dispatched by `acpu_main()`, e.g. `example/multicore/acpu_task`.

### 1.2 How HCPU and ACPU Interact

- The HCPU initiates a task with `acpu_run_task()`, which is a blocking call that returns after the ACPU finishes the task.
- The ACPU receives the task in `acpu_main()`, executes it, and sends the result back with `acpu_send_result()`.
- The underlying transport is the IPC Queue (bidirectional queues `sys_get_ha_ipc_queue` / `sys_get_ah_ipc_queue`) with a semaphore for synchronization.

See {ref}`Chapter 4 <acpu-api-usage>` for the API details.

## 2. Enable and Configuration

An ACPU-enabled project consists of two parts: the HCPU project and the ACPU child project (see {ref}`Chapter 3 <acpu-project-build>`). Each project has its own configuration, which is edited with menuconfig.

### 2.1 Enable ACPU in the HCPU Project

In the `project/hcpu` directory, start menuconfig with the board name:

```bash
sdk.py menuconfig --board=ec-lb587
```

In the menuconfig interface:

1. Enable **SiFli Built-in Components → ACPU Control Framework** (`USING_ACPU_CTRL_FWK`). This automatically selects **On-chip Peripherals → ACPU** (`BSP_USING_ACPU`) and the IPC Queue (`USING_IPC_QUEUE`).
2. (Optional) Under **SiFli Built-in Components → ACPU Control Framework**, enable **Enable Caller** (`ACPU_CALLER_ENABLED`, enabled by default on HCPU), which allows ACPU code to call HCPU services such as memory allocation and console output.

Press `D` to save the minimal configuration to `proj.conf`.

Notes:

- `BSP_USING_ACPU` is only available on SF32LB57x and SF32LB58x chips (`customer/boards/Kconfig_drv`).
- Without a core suffix, `--board` defaults to the HCPU core: `ec-lb587` and `ec-lb587_hcpu` are equivalent.

### 2.2 Configure the ACPU Project

The ACPU child project has its own configuration. In the `project/acpu` directory, start menuconfig with the `_acpu` board suffix so that the ACPU-core board configuration is used:

```bash
sdk.py menuconfig --board=ec-lb587_acpu
```

Enable **SiFli Built-in Components → ACPU Control Framework** (`USING_ACPU_CTRL_FWK`; the IPC Queue is selected automatically).

### 2.3 Feature-Specific Options

| Feature | HCPU project (`project/hcpu`) | ACPU project (`project/acpu`) |
|---------|------------------------------|-------------------------------|
| EPIC dual-core render list mode | **On-chip Peripherals → EPIC → Using epic render list API → EPIC Render List API Mode** → select *Using epic render list API in dual core HCPU mode* (`DRV_EPIC_NEW_API_DUAL_CORE_HCPU`) | Same menus → select *Using epic render list API in dual core ACPU mode* (`DRV_EPIC_NEW_API_DUAL_CORE_ACPU`) |
| opus codec on ACPU | – | **Third-Party Components → libopus** (`PKG_LIB_OPUS`) |
| ACPU calling HCPU services (memory allocation, console output) | See 2.1 step 2 | – |

The EPIC dual-core options automatically select `USING_ACPU_CTRL_FWK` (`customer/boards/Kconfig_drv`). The **EPIC Render List API Mode** choice depends on the core, so the HCPU and ACPU sides must each select their own dual-core mode in their own menuconfig session (with the `_hcpu` / `_acpu` board suffix). Refer to `example/acpu/project/proj.conf` for a complete EPIC-enabled ACPU project configuration.

(acpu-project-build)=
## 3. Project Creation, Build and Flash

The ACPU runs a separate firmware image. In the SDK, the ACPU project is managed as a child project of the HCPU project and is built together with it.

### 3.1 Project Structure

Take `example/multicore/acpu_task` as an example:

```text
example/multicore/acpu_task/
├── project/hcpu/      # HCPU project
├── project/acpu/      # ACPU child project
├── src/acpu/          # ACPU application code
└── src/hcpu/          # HCPU application code
```

### 3.2 Add the ACPU Child Project

In the HCPU project's `SConstruct.py`, call `AddChildProj` with `core="ACPU"` to add the ACPU child project (`example/multicore/acpu_task/project/hcpu/SConstruct.py`):

```python
acpu_proj_path = os.path.join(proj_path, '../acpu')
acpu_proj_name = 'acpu'
AddChildProj(acpu_proj_name, acpu_proj_path, False, core="ACPU")
```

### 3.3 Build and Flash

In the `project/hcpu` directory, build with the board name (only boards based on SF32LB57x/SF32LB58x support ACPU, e.g. `ec-lb583`, `ec-lb587`):

```bash
scons --board=ec-lb587
```

The build artifacts are generated under `build_ec-lb587`. Flash the image with:

```bash
build_ec-lb587/download.bat
```

When the board name carries the `_hcpu` suffix, the suffix can be omitted since HCPU is the default core (e.g. both `ec-lb587` and `ec-lb587_hcpu` use the HCPU configuration).

### 3.4 ACPU Image Loading

The ACPU image is stored in the Flash and loaded into ACPU RAM by the second-stage bootloader at startup:

1. The flash table maps the ACPU image to the `DFU_FLASH_HCPU_EXT2` partition. For example, the generated `ftab.c` places the image at flash address `0x69100000` and sets `xip_base = 0x20200000`, which corresponds to address 0 of the ACPU instruction space:

```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```

2. During startup, the second-stage bootloader copies the ACPU image from Flash to ACPU RAM (`example/boot_loader/project/sf32lb58x_v2/board/main.c`, function `boot_images`):

```c
/* load extended img if present, such as ACPU img */
if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```

> `example/acpu` is a generic ACPU project template used by other projects; modifying it affects all projects that reference it.

(acpu-api-usage)=
## 4. Communication and API Usage

The complete API is defined in `middleware/include/acpu_ctrl.h` (Doxygen reference: [acpu_ctrl API](../api/middleware/acpu_ctrl.md)). The transport between the two cores is the IPC Queue built on top of the mailbox (see [IPC Queue](../middleware/ipc_queue.md) and [Mailbox](../hal/mailbox.md)).

### 4.1 HCPU-Side API

| Function | Description |
|----------|-------------|
| `acpu_init()` | Initializes the framework: creates the HCPU→ACPU IPC queue and powers on the ACPU. It is exported with `INIT_PRE_APP_EXPORT`, so it runs automatically during system initialization and does not need to be called by the user. |
| `acpu_run_task(task_name, param, param_size, error_code)` | Sends a task to the ACPU and blocks until the task finishes. Returns the ACPU result buffer pointer and writes the error code to `error_code` (`ACPU_ERR_OK` = 0, `ACPU_ERR_ASSERT` = 0xff). |
| `acpu_power_on()` / `acpu_power_off()` | Controls the ACPU power domain. Power-on is normally performed by `acpu_init()`; a console command `acpu on` / `acpu off` is also available for debugging. |

`acpu_run_task()` is implemented with a blocking semaphore: the caller thread is suspended until the ACPU sends the response, or the 1-second wait expires and the SDK raises an assertion (see `middleware/acpu_ctrl/acpu_ctrl.c`).

### 4.2 ACPU-Side API

| Function | Description |
|----------|-------------|
| `acpu_main(task_name, param)` | The ACPU entry function. The weak default implementation dispatches the predefined tasks (opus, audio 3A, read/write, EPIC render list). Override it to handle custom tasks (see {ref}`Chapter 5 <acpu-examples>`). |
| `acpu_send_result(err_code, ret_value)` | Sends the task result back to the HCPU, which unblocks the pending `acpu_run_task()`. |
| `acpu_printf()` | Forwards a string to the HCPU console (requires `ACPU_CALLER_ENABLED` on the HCPU side). |
| `acpu_call_hcpu_malloc()` / `acpu_call_hcpu_free()` | Allocate/free memory on the HCPU side from ACPU code (requires `ACPU_CALLER_ENABLED` on the HCPU side). |
| `acpu_send_assert(file, line)` | Sends an assert notification to the HCPU. |

### 4.3 Task IDs

Task IDs are defined in `middleware/include/acpu_ctrl.h`:

| Macro | Value | Description |
|-------|-------|-------------|
| `ACPU_TASK_INVALID` | 0 | Invalid task |
| `ACPU_TASK_0` | 1 | User-defined task 0 |
| `ACPU_TASK_1` | 2 | User-defined task 1 |
| `ACPU_TASK_opus_encoder_init` | 3 | opus encoder init |
| `ACPU_TASK_opus_encoder_ctl` | 4 | opus encoder control |
| `ACPU_TASK_opus_encode` | 5 | opus encode |
| `ACPU_TASK_opus_decoder_init` | 6 | opus decoder init |
| `ACPU_TASK_opus_decoder_ctl` | 7 | opus decoder control |
| `ACPU_TASK_opus_decode` | 8 | opus decode |
| `ACPU_TASK_audio_3a_open` | 9 | audio 3A open |
| `ACPU_TASK_audio_3a_close` | 10 | audio 3A close |
| `ACPU_TASK_audio_3a_downlink` | 11 | audio 3A downlink |
| `ACPU_TASK_audio_3a_uplink` | 12 | audio 3A uplink |
| `ACPU_TASK_read` | 13 | read |
| `ACPU_TASK_write` | 14 | write |
| `ACPU_TASK_epic_rl` | 15 | EPIC render list |
| `ACPU_TASK_COUNT` | 16 | Task count |

HCPU-side task IDs used when the ACPU calls back:

| Macro | Value | Description |
|-------|-------|-------------|
| `HCPU_TASK_INVALID` | 0 | Invalid task |
| `HCPU_TASK_MALLOC` | 1 | Allocate memory on HCPU |
| `HCPU_TASK_FREE` | 2 | Free memory on HCPU |
| `HCPU_TASK_PRINTF` | 3 | Print to HCPU console |

### 4.4 Notes and Limitations

- The task message (`acpu_ctrl_ipc_msg_t`) carries the parameter buffer pointer and the result value through the IPC queue. Per the `acpu_run_task()` documentation in `acpu_ctrl.h`, the parameter size is limited by `ACPU_TASK_INPUT_PARAM_SIZE` and the result length by `ACPU_TASK_OUTPUT_VAL_SIZE`.
- Do not call `acpu_run_task()` from an interrupt context: it is a blocking call that suspends the calling thread.
- While waiting for the ACPU result, the framework requests `PM_SLEEP_MODE_IDLE`, so the system does not enter a deeper sleep state during the call.
- If the ACPU returns `ACPU_ERR_ASSERT`, the HCPU prints `acpu assert:` followed by the `<file> <line>` reported by the ACPU, and raises its own assertion.
- The ACPU image must be flashed correctly; otherwise the ACPU never responds and `acpu_run_task()` times out and asserts.

(acpu-examples)=
## 5. Example Walkthrough

### 5.1 `acpu_task`: HCPU Dispatches a Task to ACPU

`example/multicore/acpu_task` is the minimal ACPU example. The HCPU registers a console command `run_acpu` (`src/hcpu/main.c`) that calls `acpu_run_task()`:

```c
static int run_acpu(int argc, char *argv[])
{
    char *s;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }

    /* config ACPU to run specified task */
    s = acpu_run_task(atoi(argv[1]) + ACPU_TASK_0, NULL, 0, NULL);

    LOG_I(s);

    return 0;
}
MSH_CMD_EXPORT(run_acpu, "run acpu")
```

The call blocks until the ACPU finishes the task. The ACPU entry `acpu_main()` has a weak default implementation in `middleware/acpu_ctrl/acpu_main.c` that handles the predefined task IDs; for task 0/1 it replies with `"task_0"` / `"task_1"`, and for unknown IDs with `"unknown task"`. The HCPU then logs the returned string:

```text
msh />run_acpu 0
run_acpu 0
[I/main] task_0
msh />run_acpu 1
[I/main] task_1
msh />run_acpu 2
[I/main] unknown task
```

To implement custom tasks, define `acpu_main(task_name, param)` in `src/acpu/main.c` (the example ships an empty `__acpu_main()` placeholder there; a strong definition of `acpu_main` overrides the framework's weak default), dispatch on `task_name`, execute the task, and return the result with `acpu_send_result()`.

### 5.2 `opus`: Audio Codec on ACPU

`example/multimedia/audio/opus` runs the opus encoder/decoder on the ACPU. The ACPU project enables `CONFIG_PKG_LIB_OPUS=y`, and the HCPU delegates the codec operations through the `ACPU_TASK_opus_*` task IDs, e.g. (`src/main.c`):

```c
#if defined(USING_ACPU_CTRL_FWK)
    uint8_t error_code = 1;
    opus_encode_ctl_arg_t arg;
    arg.st = encoder;
    arg.id = 0;
    acpu_run_task(ACPU_TASK_opus_encoder_ctl, &arg, sizeof(arg), &error_code);
    RT_ASSERT(error_code == 0);
#else
    opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    ...
#endif
```

The same pattern applies to `ACPU_TASK_opus_encode`, `ACPU_TASK_opus_decode`, etc.

### 5.3 `4pdm`: Audio Capture with 3A on ACPU

`example/multimedia/audio/4pdm` captures 4-channel PDM audio. When `CONFIG_ANYKA_RUN_IN_ACPU=y` is set in the HCPU project, the 3A processing runs on the ACPU through the `ACPU_TASK_audio_3a_*` tasks, and the ACPU child project is added in `project/hcpu/SConstruct`.

### 5.4 `render_list_mode`: GPU Rendering on ACPU

`example/rt_device/gpu/render_list_mode` runs EPIC render list execution on the ACPU. The HCPU project sets `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_HCPU=y` and reuses the generic ACPU template `example/acpu/project` as the child project (which enables `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_ACPU=y`). Render list jobs are delivered to the ACPU as `ACPU_TASK_epic_rl`.

## 6. Debugging and FAQ

### 6.1 ACPU Assert

When the ACPU hits an assertion, it calls `acpu_send_assert(file, line)` to report the location to the HCPU. The HCPU prints:

```text
acpu assert:<file> <line>
```

and raises its own assertion (`RT_ASSERT(0)`). Check the reported ACPU source location to find the failure.

### 6.2 ACPU Logs

ACPU code can forward strings to the HCPU console with `acpu_printf()`. This requires `ACPU_CALLER_ENABLED` on the HCPU side (default enabled on HCPU). The console command `acpu on` / `acpu off` powers the ACPU domain on/off for debugging.

### 6.3 Memory

- The ACPU image is copied to ACPU RAM at `0x20200000` (address 0 of the ACPU instruction space) by the second-stage bootloader.
- Board-specific memory maps live under `customer/boards/<board>/{hcpu,lcpu,acpu}/` (e.g. `custom_mem_map.h`).

### 6.4 Common Problems

| Symptom | Possible cause |
|---------|----------------|
| `acpu_run_task()` times out and asserts | ACPU image not flashed, or `USING_ACPU_CTRL_FWK`/`BSP_USING_ACPU` not enabled in the HCPU project |
| The ACPU option does not appear in menuconfig | The board is based on SF32LB52x/55x/56x, which have no ACPU core |
| `acpu assert: ...` | ACPU code failed at the reported file:line |
| EPIC rendering is wrong or hangs | `DRV_EPIC_NEW_API_DUAL_CORE_*` not enabled on both the HCPU and ACPU sides |

## 7. References

- [acpu_ctrl API](../api/middleware/acpu_ctrl.md)
- [IPC Queue API](../api/middleware/inter-processor_queue_library.md)
- [Mailbox HAL](../hal/mailbox.md)
- Dual-Core Development note (SDK path: `docs/source/en/app_note/dualcore.md`, SF32LB55x HCPU/LCPU)
- Memory Usage guide (SDK path: `docs/source/en/app_note/memory_usage.md`, SF32LB55x)
- `acpu_task` example (SDK path: `example/multicore/acpu_task`)
