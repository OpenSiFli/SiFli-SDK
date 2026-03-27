# ACPU Custom Task Execution. Source code path: `example/multicore/acpu_task`

## Usage

### Supported Boards
<!-- 支持哪些板子和芯片平台 -->
+ ec-lb583
+ ec-lb587

## Overview

This example demonstrates how to configure the ACPU to execute custom tasks,
with the HCPU issuing task commands and receiving execution results. The
implementation leverages the multi-core communication framework and task
scheduling capabilities of the SiFli-SDK. Developers can use this as a reference
for heterogeneous multi-core collaboration scenarios, such as offloading
computationally intensive tasks to the ACPU to enhance overall system
performance.
<!-- 例程简介 -->
This example demonstrates how to configure the ACPU to execute custom tasks.

### Hardware Requirements

No specialized hardware is required; the example runs on any supported
development board.

### Compilation and Flashing

## Directory Structure

- `project/hcpu`: HCPU project
- `project/acpu`: ACPU project
- `src/acpu`: ACPU application code
- `src/hcpu`: HCPU application code

### Compilation and Flashing

Navigate to the `project/hcpu` directory and execute the `scons
--board=<board_name>` command to compile the image for the target board. For
example, run `scons --board=ec-lb587` to generate the image for the `587-evb`
development board. Once compilation is complete, run
`build_<board_name>\download.bat` to flash the image (e.g.,
`build_ec-lb587\download.bat`).</board_name></board_name>

## Expected Results

Send the command `run_acpu <task_id>` (followed by a carriage return) via the
serial console. The `<task_id>` should be an integer starting from 0,
corresponding to TASK_0, TASK_1, etc. The expected output is as
follows:</task_id></task_id>
```
12-28 20:17:23:794    msh /&gt;
12-28 20:17:23:844    msh /&gt;
12-28 20:17:26:560 TX:run_acpu 0
12-28 20:17:26:732    run_acpu 0
12-28 20:17:26:772    [I/main] task_0
12-28 20:17:26:790    msh /&gt;
12-28 20:17:26:809    msh /&gt;
12-28 20:17:29:006 TX:run_acpu 1
12-28 20:17:29:149    run_acpu 1
12-28 20:17:29:160    [I/main] task_1
12-28 20:17:29:179    msh /&gt;
12-28 20:17:29:194    msh /&gt;
12-28 20:17:30:203 TX:run_acpu 2
12-28 20:17:30:332    run_acpu 2
12-28 20:17:30:347    [I/main] unknown task
12-28 20:17:30:358    msh /&gt;
12-28 20:17:30:366    msh /&gt;
12-28 20:17:31:285 TX:run_acpu 3
12-28 20:17:31:425    run_acpu 3
12-28 20:17:31:437    [I/main] unknown task
12-28 20:17:31:464    msh /&gt;
```

## Code Description
The function `acpu_main` in `src/acpu/main.c` serves as the entry point for ACPU
task processing, executing the corresponding code block based on the received
task ID.

The `acpu_run_task` function in `src/hcpu/main.c` is called to configure a task
for ACPU execution. This function operates in blocking mode and does not return
until the ACPU provides a result; during this execution, the calling thread is
suspended while waiting for a semaphore.

The ACPU image is stored in Flash and programmed via a flashing script. The
secondary bootloader copies the ACPU code to the RAM address corresponding to
address 0 in the ACPU instruction space. For example, the following code is
excerpted from the generated file `ftab.c`: `.base=0x69100000` indicates that
the ACPU image starts at Flash address 0x69100000, while `xip_base=0x20200000`
specifies that the secondary bootloader will copy the ACPU code to RAM starting
at 0x20200000, which maps to address 0 in the ACPU instruction space.

```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```

The implementation for the secondary bootloader copying the ACPU code can be
found in the `boot_images` function within
`example\boot_loader\project\sf32lb58x_v2\board\main.c`, as shown below:
```c
if (g_sec_config-&gt;imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```

## Troubleshooting

- **Compilation Error**: Verify that the SiFli-SDK development environment is
  correctly configured and ensure the board model name is correct.
- **Flashing Failed**: Confirm the development board is properly connected and
  try reconnecting the USB cable.

## References

- [SiFli-SDK Quick Start
  Guided](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [Multicore Communication Development
  Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)

