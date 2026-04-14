# Partition Table Syntax Version 2.0

The partition table description file `ptab.json` is a text file in JSON format. It follows standard JSON syntax and can be edited with any text editor.
The partition table uses a list to define all memory devices used in the system, such as `flash2`, `psram1`, etc. In version 2.0 of the partition table, a special list element (referred to as the header element) is used to specify the version number with the `version` property set to `2`, as shown below:
```json
[
    {"version": "2"},
    {
        "mem": "flash2", 
        "base": "0x12000000", 
        "regions": []
    },
    {
        "mem": "psram1", 
        "base": "0x60000000", 
        "regions": []
    }    
]    
```
Each element in the list (excluding the header) corresponds to a memory block. For example, in the following example, there are memories such as `flash5`, `psram1`, `flash4`, and `hpsys_ram`. Each memory has the following properties:

* `mem`: The name of the memory.
* `base`: The base address of the memory, in hexadecimal. Must be correct.
* `regions`: The list of partitions within the memory.

`regions` defines the partitions within the memory. Each partition has the following properties:

* `offset`: The offset in bytes (hexadecimal).
* `max_size`: The size in bytes (hexadecimal).
* `tags`: Optional. A list of tags. Each tag is used to generate corresponding macros representing the partition’s size and starting address. For example, a tag `FLASH_BOOT_LOADER` generates `FLASH_BOOT_LOADER_START_ADDR`, `FLASH_BOOT_LOADER_OFFSET`, and `FLASH_BOOT_LOADER_SIZE` macros. `START_ADDR` is `base + offset`, `OFFSET` equals the `offset` value, and `SIZE` equals the `max_size` value.
* `name`: Partition name
* `type`: Optional. Partition type
* `core`: Optional. When the partition type is `app_exec`, it indicates the core that runs the code in this partition. The available names are `hcpu`, `lcpu`, and `acpu`.
* `custom`: Optional. A custom macro dictionary


````{dropdown} ptab.json Example
```json
[
    {
        "mem": "flash5", 
        "base": "0x1C000000", 
        "regions": [
            {
                "offset": "0x00020000", 
                "max_size": "0x00020000", 
                "tags": ["FLASH_BOOT_LOADER"], 
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
        ]
    }, 
    {
        "mem": "psram1", 
        "base": "0x60000000", 
        "regions": [
            {
                "offset": "0x00000000", 
                "max_size": "0x00200000", 
                "tags": ["HCPU_FLASH_CODE"],
                "name": "main",
                "type": ["app_exec"]
            }, 
            {
                "offset": "0x00200000", 
                "max_size": "0x00200000", 
                "tags": ["PSRAM_DATA"]
            },
        ]
    }, 
    {
        "mem": "flash4", 
        "base": "0x18000000", 
        "regions": [
            {
                "offset": "0x00000000", 
                "max_size": "0x00200000", 
                "name": "main", 
                "type": ["app_img"],
            }, 
            {
                "offset": "0x00200000", 
                "max_size": "0x00100000", 
                "tags": ["FS_REGION"]
            }
        ]
    }, 
    {
        "mem": "hpsys_ram", 
        "base": "0x20000000", 
        "regions": [
            {
                "offset": "0x00000000", 
                "max_size": "0x0006BC00", 
                "tags": ["HCPU_RAM_DATA"]
            }, 
            {
                "offset": "0x0006BC00", 
                "max_size": "0x00014000", 
                "tags": ["HCPU_RO_DATA"]
            }, 
        ]
    }
]
```
````

## Property Details

### name

The `name` property defines the name of the partition. For partitions of type `app_img` or `app_exec` (see [](#type) for details), it indicates which program the partition is associated with. The value `bootloader` refers to the secondary bootloader, `main` refers to the main application, `dfu` refers to the DFU (Device Firmware Upgrade) program, and `ftab` stands for the flash table. These are reserved names and should not be used arbitrarily. Other names can be freely defined by the user. For example, `acpu` can be used to represent a code partition for ACPU.

It’s important to note that for program image partitions, the partition name must match the program name. For instance, if the ACPU and LCPU programs are added to the main project as sub-projects using the `AddChildProj` function, and are named `acpu` and `lcpu` respectively, then their corresponding partition names must also be `acpu` and `lcpu`.

Another case is when a partition stores a precompiled binary file, which is added using the `AddCustomImg` function. In this case, the `name` parameter specified in `AddCustomImg` must match the partition’s `name` property.

Additionally, a program might consist of multiple files. For example, the main program `main` might include two binary files: one for code (`ER_IROM1.bin`) and one for resources (`ER_IROM2.bin`). In this case, the `name` property of the corresponding partitions can be set to `main:1` and `main:2` respectively, where `main:1` corresponds to `ER_IROM1.bin`, and `main:2` to `ER_IROM2.bin`.

In summary, the value of `name` should follow the format `proj_name:num`, where `proj_name` is the program name and `num` is an index starting from 1. If the program consists of only one file, the `num` part can be omitted.

### type

The `type` property defines the type of the partition. It is a list of strings, and the supported types include:

* `app_img`: The partition is used to store the program image (primary partition). The flashing tool will download the program to this address.
* `app_img2`: The partition is used to store a backup of the program image (backup partition), typically used for upgrades where it acts as a backup to the primary partition.
* `app_exec`: The partition is used to execute program code, meaning the code runs in the address space of this partition.

Below is a description snippet for `flash5`. From the code, we can see that the `bootloader` (i.e., the secondary bootloader) is stored in `flash5`, with the address range from `0x1C020000` to `0x1C03FFFF`. At the same time, the bootloader also executes from `flash5`, meaning the storage address and execution address are the same. The program executes in-place (XIP) on `flash5`.

```json
{
    "mem": "flash5", 
    "base": "0x1C000000", 
    "regions": [
        {
            "offset": "0x00020000", 
            "max_size": "0x00020000", 
            "tags": ["FLASH_BOOT_LOADER"], 
            "name": "bootloader",
            "type": ["app_img", "app_exec"]
        }
    ]
}, 
```

Program partition name that are not system-reserved (i.e., `name` is not `main`, `bootloader`, `ftab`, or `dfu`) will use the following entries in the flash table to describe the image information stored in flash:

* `DFU_FLASH_HCPU_EXT2`: 11
* `DFU_FLASH_LCPU_EXT1`: 12
* `DFU_FLASH_LCPU_EXT2`: 13

For example, in the `ptab.json` snippet below, the partition `acpu` is defined to use an offset of `0x00300000` in `flash4` as the storage address for the ACPU program, and an offset of `0x00200000` in `hpsys_ram` as the execution address.
The generated `ftab.c` file will use the `DFU_FLASH_HCPU_EXT2` entry to describe the storage and execution addresses of the ACPU image.
The `length` (i.e., the amount of data to be relocated) is determined based on the actual file size of the program specified by the `name` property (see [](#name) for details). In this example, the actual length is 0x7d4 bytes.

````{dropdown} ACPU Program Partition Example
```json
{
    "mem": "flash4", 
    "base": "0x18000000", 
    "regions": [
        {
            "offset": "0x00300000", 
            "max_size": "0x00010000", 
            "name": "acpu", 
            "type" ["app_img"]
        }
    ]
}, 
{
    "mem": "hpsys_ram", 
    "base": "0x20000000", 
    "regions": [
        {
            "offset": "0x00200000", 
            "max_size": "0x00020000", 
            "name": "acpu",
            "type": ["app_exec"]
        }
    ]
}, 
```
````

The `ftab.c` generated from this example is as follows:

```{code-block} c
:lineno-start: 1
:emphasize-lines: 20,21

RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[0] = {.base = FLASH_TABLE_START_ADDR,      .size = FLASH_TABLE_SIZE,      .xip_base = 0, .flags = 0},
    .ftab[1] = {.base = FLASH_CAL_TABLE_START_ADDR,  .size = FLASH_CAL_TABLE_SIZE,  .xip_base = 0, .flags = 0},
    .ftab[3] = {.base = 0x1C020000, .size = 0x00020000,  .xip_base = 0x1C020000, .flags = 0},
    .ftab[4] = {.base = 0x18000000, .size = 0x00200000,  .xip_base = 0x60000000, .flags = 0},
    .ftab[5] = {.base = FLASH_BOOT_PATCH_START_ADDR, .size = FLASH_BOOT_PATCH_SIZE, .xip_base = BOOTLOADER_PATCH_CODE_ADDR, .flags = 0},
    .ftab[7] = {.base = 0x1C020000, .size = 0x00020000,  .xip_base = 0x1C020000, .flags = 0},
    .ftab[8] = {.base = 0x18000000, .size = 0x00200000,  .xip_base = 0x60000000, .flags = 0},
    .ftab[9] = {.base = BOOTLOADER_PATCH_CODE_ADDR,  .size = FLASH_BOOT_PATCH_SIZE, .xip_base = BOOTLOADER_PATCH_CODE_ADDR, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)] = {.length = 0x000FB1F8, .blksize = 512, .flags = DFU_FLAG_AUTO},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BL)] = {.length = 0x80000, .blksize = 512, .flags = DFU_FLAG_AUTO},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BOOT)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU2)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BCPU2)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BOOT2)] = {.length = 0xFFFFFFFF},
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x18300000, .size = 0x00020000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x000007D4, .blksize = 512, .flags = DFU_FLAG_AUTO},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_LCPU_EXT1)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_LCPU_EXT2)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_RESERVED)] = {.length = 0xFFFFFFFF},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_SINGLE)] = {.length = 0xFFFFFFFF},
    .running_imgs[CORE_HCPU] = (struct image_header_enc *) &sec_config.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)],
    .running_imgs[CORE_LCPU] = (struct image_header_enc *)0xFFFFFFFF,
    .running_imgs[CORE_BL] = (struct image_header_enc *) &sec_config.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BL)],
    .running_imgs[CORE_BOOT] = (struct image_header_enc *)0xFFFFFFFF,
};
```

### custom

The `custom` property allows adding custom macros to `ptab.h`. The value must be a dictionary. For example:

```json
"custom": {
    "PSRAM_BL_MODE": 3, 
    "PSRAM_BL_SIZE": 8, 
    "PSRAM_BL_MPI": 2
}
```

Generates:

```c
#define PSRAM_BL_MODE  (3)
#define PSRAM_BL_SIZE  (8)
#define PSRAM_BL_MPI   (1)
```

```{note}
Note: Dictionary values must be integers.
```

Here is the English translation:

---

## Default Partition Configuration

If the partition table does not define the `ftab` and `bootloader` partition configurations, the tool will generate `ptab.h` using default settings. Users can check the generated file to understand these defaults. In general, the default configuration is sufficient for most application development needs. However, users can also add `ftab` and `bootloader` configurations to the partition table to override the default parameters. Note that `ftab` must be placed in the correct location to ensure it can be read by the chip's Boot ROM. The specific requirements are as follows:

* **SF32LB55X series**: Must be placed at the base address of flash1 (i.e., the NOR Flash embedded in the high-performance core, connected to the QSPI1 interface).
* **SF32LB58X series**: Must be placed at the base address of flash5 (i.e., the NOR Flash embedded in the low-power core, connected to the MPI5 interface).
* **SF32LB56X series**: Must be placed at the base address of flash5 (i.e., the NOR Flash embedded in the low-power core, connected to the MPI5 interface).
* **SF32LB52X series**:

  * For chips with internal Flash, `ftab` must be placed at the base address of the internal Flash (i.e., flash1, connected to the MPI1 interface).
  * Otherwise, it should be placed at the base address of the external SPI NOR or SPI NAND Flash.
  * For eMMC, SD-NAND, and SD cards, `ftab` must be placed starting at an offset of 0x1000.
