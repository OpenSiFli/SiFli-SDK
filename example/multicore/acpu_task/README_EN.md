# ACPU Executes Custom Tasks
Source Code Path: `example/multicore/acpu_task`

## Usage

### Supported Development Boards
<!-- Which boards and chip platforms are supported -->
+ ec-lb583
+ ec-lb587

## Overview

This example demonstrates how to configure ACPU to execute custom tasks, send task instructions through HCPU, and receive execution results. This example mainly uses the multi-core communication framework and task scheduling functions of SiFli-SDK.
Based on this example, developers can build application scenarios that require heterogeneous multi-core collaboration, such as assigning computationally intensive tasks to ACPU for processing to improve overall system performance.
<!-- Example Introduction -->
This example demonstrates how to configure ACPU to execute custom tasks

### Hardware Requirements

No special hardware requirements, just use a supported development board to run normally.

### Compilation and Flashing

## Directory Structure

- `project/hcpu`: HCPU project
- `project/acpu`: ACPU project
- `src/acpu`: ACPU application code
- `src/hcpu`: HCPU application code

### Compilation and Flashing

Execute the `scons --board=<board_name>` command in the `project/hcpu` directory to compile and generate the image file for the required board. For example, execute the `scons --board=ec-lb587` command to generate the image file for the `587-evb` development board. After compilation, run the command `build_<board_name>\download.bat` to flash the image file, such as `build_ec-lb587\download.bat`

## Expected Result of the Example

Send the `run_acpu <task_id>` command (with carriage return) in the serial console window, where `<task_id>` is a number greater than or equal to 0, corresponding to TASK_0, TASK_1, etc. in sequence. The running result is as follows
```
12-28 20:17:23:794    msh />
12-28 20:17:23:844    msh />
12-28 20:17:26:560 TX:run_acpu 0
12-28 20:17:26:732    run_acpu 0
12-28 20:17:26:772    [I/main] task_0
12-28 20:17:26:790    msh />
12-28 20:17:26:809    msh />
12-28 20:17:29:006 TX:run_acpu 1
12-28 20:17:29:149    run_acpu 1
12-28 20:17:29:160    [I/main] task_1
12-28 20:17:29:179    msh />
12-28 20:17:29:194    msh />
12-28 20:17:30:203 TX:run_acpu 2
12-28 20:17:30:332    run_acpu 2
12-28 20:17:30:347    [I/main] unknown task
12-28 20:17:30:358    msh />
12-28 20:17:30:366    msh />
12-28 20:17:31:285 TX:run_acpu 3
12-28 20:17:31:425    run_acpu 3
12-28 20:17:31:437    [I/main] unknown task
12-28 20:17:31:464    msh />
```

## Code Explanation

The function `acpu_main` in `src/acpu/main.c` is the entry function for ACPU to process tasks, executing corresponding code according to the received task ID
The `acpu_run_task` function is called in `src/hcpu/main.c` to configure ACPU to execute a certain task. This function runs in blocking mode and will not return until ACPU returns the result. During this period, the thread calling this function will be suspended due to waiting for a semaphore
The ACPU image file is stored in Flash and programmed into Flash by the programming script. The secondary boot copies the ACPU code to the RAM corresponding to address 0 of the ACPU instruction space.
For example, the following code is excerpted from the compiled ftab.c. `.base=0x69100000` indicates that the ACPU image file is stored in Flash starting from address 0x69100000. `xip_base=0x20200000` indicates that the secondary boot will copy the ACPU code to the RAM starting from 0x20200000. 0x20200000 corresponds to address 0 of the ACPU instruction space.
```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```
The implementation of the secondary boot copying ACPU code can be found in the `boot_images` function in `example\boot_loader\project\sf32lb58x_v2\board\main.c` as follows:
```c
if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```

## Troubleshooting

- **Compilation Errors**: Ensure the SiFli-SDK development environment is configured correctly and check that the board name is correct
- **Programming Failure**: Confirm the development board is connected correctly and try re-plugging the USB cable

## Reference Documents

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [Multi-core Communication Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)