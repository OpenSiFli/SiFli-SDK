# ACPU Custom Task Execution
Source code path: `example/multicore/acpu_ctrl`
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ ec-lb583
+ ec-lb587

## Overview
<!-- Example introduction -->
This example demonstrates how to configure ACPU to execute custom tasks.

## Directory Structure
- `project/hcpu`: HCPU project
- `project/acpu`: ACPU project
- `src/acpu`: ACPU application code
- `src/hcpu`: HCPU application code

### Compilation and Programming
Execute the `scons --board=<board_name>` command in the `project/hcpu` directory to compile and generate image files for the required board, such as executing `scons --board=ec-lb587` to generate image files for the `587-evb` development board. After compilation, run the command `build_<board_name>\download.bat` to flash the image file, for example `build_ec-lb587\download.bat`.

## Expected Results of the Example

Send the `run_acpu <task_id>` command (requires carriage return) in the serial console window, where `<task_id>` takes values of numbers greater than or equal to 0, corresponding to TASK_0, TASK_1, etc. respectively. The running results are as follows:
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

## Code Description
The function `acpu_main` in `src/acpu/main.c` serves as the entry function for ACPU task processing, executing corresponding code based on the received task ID.

In `src/hcpu/main.c`, the `acpu_run_task` function is called to configure ACPU to execute a specific task. This function runs in blocking mode and only returns after ACPU returns results. During this period, the thread calling this function will be suspended while waiting for the semaphore.

ACPU's image file is stored in Flash and burned to Flash by the programming script. The secondary bootloader copies ACPU code to the RAM corresponding to address 0 in ACPU instruction space.
For example, the following code excerpt from the generated ftab.c shows `.base=0x69100000` indicating that ACPU's image file is stored in Flash starting from address 0x69100000, and `xip_base=0x20200000` indicates that the secondary bootloader will copy ACPU code to RAM starting from 0x20200000, where 0x20200000 corresponds to address 0 in ACPU instruction space.

```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```

The implementation of secondary bootloader copying ACPU code can be found in the function `boot_images` in `example\boot_loader\project\sf32lb58x_v2\board\main.c` in the following section:
```c
if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```

## Exception Diagnosis

## Reference Documentation

## Update History