# SF32LB55x Memory usage guide

## 1. Memory address space

### HCPU Memory address space

|name             |starting address| end address| size (bytes)| describe
|-----------------|----------------|----------------|------------|--------
|HPSYS ROM        | 0x00000000     | 0x0000FFFF     | 64*1024    | |
|HPSYS ITCM RAM   | 0x00010000     | 0x0001FFFF     | 64*1024    | |
|HPSYS Retention RAM   | 0x00020000     | 0x0002FFFF     | 64*1024    | It can be powered off when it is in standby mode with low power consumption|
|HPSYS SRAM       | 0x20000000     | 0x200FFFFF     | 1024*1024  | |
|LPSYS ROM        | 0x0B000000     | 0x0B05FFFF     | 384*1024    | |
|LPSYS ITCM RAM   | 0x0B0FC000     | 0x0B0FFFFF     | 16*1024   | |
|LPSYS DTCM RAM   | 0x2B0FC000     | 0x2B0FFFFF     | 16*1024   | |
|LPSYS SRAM       | 0x20100000     | 0x20137FFF     | 224*1024   | |
|QSPI1            | 0x10000000     | 0x11FFFFFF     | |The EVB has a capacity of 4MByte |
|QSPI2            | 0x64000000     | 0x67FFFFFF     | |The EVB has a capacity of 32MByte|
|QSPI3            | 0x68000000     | 0x68FFFFFF     | |EVB ununited|
|QSPI4            | 0x12000000     | 0x13FFFFFF     | |LCPU is optional flash memory,EVB ununited|
|PSRAM-OPI        | 0x60000000     | 0x61FFFFFF     | |The EVB has a capacity of 4MByte |

```{note} 
The first 128KB of HPSYS SRAM is DTCM, which is faster than the subsequent address space access
```

HCPU is used to access the address translation interface of LPSYS Memory
- LCPU SRAM address conversion: LCPU_ADDR_2_HCPU_ADDR, for example, call LCPU_ADDR_2_HCPU_ADDR (0x20100000) to get 0x20100000
- LCPU ROM address conversion: LCPU_ROM_ADDR_2_HCPU_ADDR, for example, call LCPU_ROM_ADDR_2_HCPU_ADDR (0x00000000) to get 0x0B000000
- LCPU ITCM address conversion: LCPU_ITCM_ADDR_2_HCPU_ADDR, for example, call LCPU_ITCM_ADDR_2_HCPU_ADDR (0x000FC000) to get 0x0B0FC000
- LCPU DTCM address conversion: LCPU_DTCM_ADDR_2_HCPU_ADDR, for example, call LCPU_DTCM_ADDR_2_HCPU_ADDR (0x200FC000) to get 0x2B0FC000

HCPU code can be executed on QSPI1/QSPI2/QSPI3 by XIP, or it can run on HPSYS ITCM/SRAM/Retention RAM


### LCPU Memory address space

| Name | S-Bus start address | S-Bus end address | C-Bus start address | C-Bus end address | Size (bytes) | Description
|-----------------|----------------|----------------|----------------|----------------|------------|------
|HPSYS SRAM       | 0x2A000000     | 0x2A0FFFFF     |                |                | 1024*1024  | |
|LPSYS ROM        |                |                | 0x00000000     |  0x0005FFFF    | 384*1024   | |
|LPSYS ITCM RAM   |                |                | 0x000FC000     |  0x000FFFFF    | 16*1024    | |
|LPSYS DTCM RAM   | 0x200FC000     | 0x200FFFFF     |                |                | 16*1024    | |
|LPSYS SRAM       | 0x20100000     | 0x20137FFF     | 0x00100000     |  0x00137FFF    | 224*1024   | | 
|QSPI4            |                |                | 0x12000000     |  0x13FFFFFF    |            |The LCPU is optional flash memory, and the EVB is not connected |


In order to improve performance, in addition to using the 0x20100000~0x20137FFF address range to access LPSYS SRAM, you can also use the 0x00100000~0x00137FFF address range to access,
The former uses S-Bus to access SRAM, and the latter uses C-Bus to access SRAM. They both access the same physical SRAM.
For example, the code can be allocated to an address starting at 0x00100000, and the remaining space in the SRAM can be allocated to the data segment using the S-Bus address space,
In this way, fetching and fetching can be parallelized to improve the execution efficiency of the code.
It is also possible to combine ITCM and SRAM. For example, if the code starts at 0x000FC000, the code that exceeds 16K bytes will be placed in the area starting at SRAM address 0x00100000, and the remaining SRAM space will still be accessed using the S-Bus address space.
See sections 4 and 5 for specific examples.

The LCPU is used as a conversion interface for accessing HPSYS SRAM addresses,
- HCPU_ADDR_2_LCPU_ADDR, for example, call HCPU_ADDR_2_LCPU_ADDR(0x20000000) to get 0x2A000000

The LCPU code can run on the LPSYS' ITCM/DTCM/SRAM and needs to be moved from the user program on the HCPU to the corresponding address space

### QSPI1 address space notes
The Bootloader uses 12kbyte starting with 0x10000000 as the Flash configuration table. The default Flash configuration table specifies the Flash1 address planning as follows, and the user program starting address is 0x10020000.
If a custom Flash configuration table is used, it must be ensured that the first 12kbyte is not used and that the Flash configuration table is not rewritten. Other address Spaces can be used at will.

| Name | Starting address | Ending address | Size (bytes) | Description
|-----------------|----------------|----------------|------------|--------
|Flash configuration table      | 0x10000000     | 0x10004FFF     | 20*1024    | |
|Calibration Table | 0x10005000     | 0x10006FFF     | 8*1024     | |
|Reserved          | 0x10007000     | 0x1000FFFF     | 36*1024    | |
|bootrom patch    | 0x10010000     | 0x1001FFFF     | 64*1024    | |
|User Code        | 0x10020000     | | | | 

### HPSYS SRAM address space notes
The Bootloader uses the RAM address space shown in the following table. When the Bootloader is used to boot the user program to execute in RAM, the RAM program cannot use the address 0x20040000 to avoid being rewritten by the bootloader after moving.
This restriction does not apply if the user program is executed on Flash.

bootloader RAM address space

| Name | Starting address | Ending address | Size (bytes) | Description
|-----------------|----------------|----------------|------------|--------------
|DATA             | 0x20040000     | 0x2004FFFF     | 64*1024    | bootrom data |
|PATCH CODE       | 0x20050000     | 0x2005FFFF     | 64*1024    | Patch code   |
|PATCH DATA       | 0x20060000     | 0x2006FFFF     | 64*1024    | Patch data   |

## 2. HPSYS Memory performance comparison

| Name | Frequency (MHz) | Description
|---------------|--------------|-------
|HCLK           | 240          |      |
|PSRAM-OPI      | 120          | DDR  |
|FLASH1         | 96           |      |
|FLASH2         | 80           |      |

Generally speaking, DMA has the highest efficiency in accessing SRAM, and eDMA has the highest efficiency in accessing PSRAM/FLASH. The specific throughput performance is shown in the following table. In this table, memcpy cannot use microlib, FLASH1 and FLASH2 enable CACHE, and PSRAM does not enable CACHE.


| Source | Purpose  | memcpy(MB/s) | eDMA(MB/s) | DMA(MB/s) 
|----------|------------|--------------|------------|-------------
| SRAM     | SRAM       | 405.54       | 430.17     | 446.35    |
| PSRAM    | SRAM       | 43.34        | 125.54     | 53.23     |
| FLASH1   | SRAM       | 46.41        | 43.17      | 25.50     |
| FLASH2   | SRAM       | 36.30        | 36.13      | 21.24     |
| SRAM     | PSRAM      | 48.05        | 187.01     | 47.70     |
| PSRAM    | PSRAM      | 20.99        | 76.72      | 12.96     |
| FLASH1   | PSRAM      | 26.84        | 42.97      | 12.51     |
| FLASH2   | PSRAM      | 24.94        | 36.14      | 21.25     |

## 3. HCPU EPIC performance

| Name | Frequency (MHz) | Description
|---------------|--------------|-------
|HCLK           | 240          |      |
|PSRAM-OPI      | 120          | DDR  |
|FLASH1         | 96           |      |
|FLASH2         | 80           |      |

Unless otherwise specified, the color format is RGB565

### Alpha mix
| Vision | Output | Performance (cycles/pixel) | Throughput (MPixels/s)
|----------|------------|--------------------|------------------
| SRAM     | SRAM       | 1.41               | 170.16      |
| PSRAM    | SRAM       | 4.80               | 50.05       |
| FLASH1   | SRAM       | 14.93              | 16.07       |
| FLASH2   | SRAM       | 17.83              | 13.46       |
| SRAM     | PSRAM      | 2.54               | 94.51       |
| PSRAM    | PSRAM      | 7.40               | 32.46       |
| FLASH1   | PSRAM      | 14.90              | 16.11       |
| FLASH2   | PSRAM      | 17.83              | 13.46       |

```{note} 
The above table shows the performance of using HAL_EPIC_BlendStart/HAL_EPIC_BlendStart_IT interface. If you use HAL_EPIC_Rotate_IT/HAL_EPIC_Rotate,
SRAM will lose 50% performance in the foreground and 20% in other situations
```

### Zoom in
| Vision | Output | Performance (cycles/pixel) | Throughput (MPixels/s)
|----------|------------|--------------------|---------------
| SRAM     | SRAM       | 4.59               | 52.24      | 
| PSRAM    | SRAM       | 12.82              | 18.72      | 
| FLASH1   | SRAM       | 41.19              | 5.82       | 
| FLASH2   | SRAM       | 49.43              | 4.85       | 
| SRAM     | PSRAM      | 4.63               | 51.88      | 
| PSRAM    | PSRAM      | 12.81              | 18.73      | 
| FLASH1   | PSRAM      | 41.22              | 5.82       | 
| FLASH2   | PSRAM      | 49.47              | 4.85       | 

```{note}
The size of the foreground image is 88*88, and the zoom factor is 1400
```

### Revolve
| Vision | Output | Performance (cycles/pixel) | Throughput (MPixels/s)
|----------|------------|--------------------|--------------
| SRAM     | SRAM       | 15.90              | 15.69     | 
| PSRAM    | SRAM       | 136.27             | 1.76      | 
| FLASH1   | SRAM       | 254.80             | 0.94      | 
| FLASH2   | SRAM       | 313.40             | 0.76      | 
| SRAM     | PSRAM      | 15.29              | 15.7      | 
| PSRAM    | PSRAM      | 139.11             | 1.72      | 
| FLASH1   | PSRAM      | 254.80             | 0.94      | 
| FLASH2   | PSRAM      | 313.42             | 0.76      | 

```{note}
The rotation Angle is 45°
```

### Filling
| Vision | Output | Performance (cycles/pixel) | Throughput (MPixels/s)
|------------|--------------------|---------------
| SRAM       | 1.31               | 183.55      | 
| PSRAM      | 2.79               | 85.95       | 

```{note}
Fill the monochrome with a transparency of 255
```


## 4. Application Memory Planning and Custom Interfaces
The following interfaces are defined in _$SDK_ROOT/drivers/cmsis/sf32lb55x/mem_map.h_ to plan the Memory usage of applications.
In addition to header files and C files, these macros can also be used in linker files to ensure consistency in address planning.

HCPU Flash and SRAM Address Planning Table
| Macro | Description | Default Value | Description |
|-----------------------------|-------------------------------|------------|---------|
| HCPU_FLASH_CODE_START_ADDR  | Starting address of the XIP code segment for the HCPU application | 0x10020000 (no DFU) or 0x10100000 (with DFU) | |
| HCPU_FLASH_CODE_SIZE        | Size of the XIP code segment for the HCPU application | 896KB | |
| HCPU_FLASH2_IMG_START_ADDR  | Starting address of the image resource | 0x64000000 | |
| HCPU_FLASH2_IMG_SIZE        | Image resource size | 4MB | |
| HCPU_FLASH2_FONT_START_ADDR | Starting address of the font resource | 0x64400000 | |
| HCPU_FLASH2_FONT_SIZE       | Font resource size | 12MB | |
| HCPU_RAM_DATA_START_ADDR    | Starting address of the data segment for the HCPU application | 0x20000000 | |
| HCPU_RAM_DATA_SIZE          | Size of the data segment for the HCPU application | 1007KB | |
| HCPU_RO_DATA_START_ADDR     | Starting address of the U application RAM code segment | 0x200FBC00 | Used to store code that needs to be executed in SRAM |
| HCPU_RO_DATA_SIZE           | Size of the RAM code segment for the U application | 16KB | |
| HCPU2LCPU_MB_CH2_BUF_START_ADDR | Starting address of the shared buffer for HCPU2LCPU | 0x200FFC00 | |
| HCPU2LCPU_MB_CH2_BUF_SIZE  | Size of mailbox2 shared buffer for HCPU2LCPU | 512B | |
| HCPU2LCPU_MB_CH1_BUF_START_ADDR | Starting address of the shared buffer for mailbox1 in HCPU2LCPU | 0x200FFE00 | |
| HCPU2LCPU_MB_CH1_BUF_SIZE  | Size of mailbox1 shared buffer for HCPU2LCPU | 512B | |

The heap space size that HCPU can allocate depends on how much space static variables occupy in the HCPU application data segment.
For example, if a static variable takes up 500KB, the remaining heap space will be 1007KB - 500KB = 507KB.

OTA Address Planning Table
| Macro | Description | Default Value | Description |
|-----------------------------|---------------------------|------------|---------|
| DFU_FLASH_CODE_START_ADDR   | Starting address of the DFU program code segment | 0x10020000 | |
| DFU_FLASH_CODE_SIZE         | Size of the DFU program code segment | 256KB | |
| DFU_RES_FLASH_CODE_START_ADDR | Starting address of the application upgrade area | 0x10060000 | |
| DFU_RES_FLASH_CODE_SIZE     | Size of the application upgrade area | 640KB | |
| HCPU_FLASH2_IMG_UPGRADE_START_ADDR | Starting address of the image resource upgrade area | 0x65000000 | |
| HCPU_FLASH2_IMG_UPGRADE_SIZE | Size of the image resource upgrade area | 1MB | |
| HCPU_FLASH2_FONT_UPGRADE_START_ADDR | Starting address of the font resource upgrade area | 0x65100000 | |
| HCPU_FLASH2_FONT_UPGRADE_SIZE | Size of the font resource upgrade area | 3MB | |

LCPU SRAM Address Planning Table
| Macro | Description | Default Value |
|-----------------------------|---------------------------|------------------|
| LCPU_RAM_CODE_START_ADDR    | Starting address of the LCPU application code segment | 0x000FC000 |
| LCPU_RAM_CODE_SIZE          | Size of the LCPU application code segment | 140KB |
| LPSYS_RAM_SIZE              | Total size of the LCPU application code and data segment | 160KB |
| LCPU2HCPU_MB_CH1_BUF_START_ADDR | Starting address of the shared buffer for LCPU2HCPU mailbox2 | 0x20123C00 |
| LCPU2HCPU_MB_CH1_BUF_SIZE  | Size of the shared buffer for LCPU2HCPU mailbox2 | 512B |
| LCPU2HCPU_MB_CH2_BUF_START_ADDR | Starting address of the shared buffer for LCPU2HCPU mailbox1 | 0x20123E00 |
| LCPU2HCPU_MB_CH2_BUF_SIZE  | Size of the shared buffer for LCPU2HCPU mailbox1 | 512B |

The heap space that LCPU can allocate depends on how much space the LCPU application code and static variables occupy.
For instance, if the code and static variables occupy 120KB, the remaining heap space will be 160KB - 120KB = 40KB.

If you need to modify the default values, you can add the `CUSTOM_MEM_MAP` option in the project's Kconfig.
After running menuconfig and saving the settings, the `CUSTOM_MEM_MAP` switch will be added in the `rtconfig.h`, and `mem_map.h` will include `custom_mem_map.h`, where you can redefine the macros that need modification.

```c
config CUSTOM_MEM_MAP
    bool 
    default y  
```

### 4.1 Low Power Related Interfaces
To implement the low power Standby mode, the link.sct file for the HCPU project defines several interfaces used by the PM framework. For example, in _$SDK_ROOT/example/pm/ble/ec-lb555/hcpu/linker_scripts/link_flash.sct_, if the hardware platform supports PSRAM, the following regions need to be defined in the linker file. The SDK uses RW_PSRAM2 as the backup area for data during sleep, and the data in this region will not be saved during sleep. For data that needs to be saved, it can be placed in the RW_P...

```c
  RW_PSRAM1 PSRAM_BASE UNINIT{  ; ZI data, retained
    *.o (.l2_ret_data_*)
    *.o (.l2_ret_bss_*)
    *.o (.l2_cache_ret_data_*)
    *.o (.l2_cache_ret_bss_*)    
  }
  RW_PSRAM2 +0  UNINIT{  ; ZI data, not retained and reused by SRAM retention
    *.o (.nand_cache)
    *.o (.l2_non_ret_data_*)
    *.o (.l2_non_ret_bss_*)
    *.o (.l2_cache_non_ret_data_*)
    *.o (.l2_cache_non_ret_bss_*)  
  }
  ScatterAssert((ImageLength(RW_PSRAM1)+ImageLength(RW_PSRAM2))<PSRAM_SIZE)
```

Similarly, SRAM is divided into RW_IRAM0 and RW_IRAM1 regions. Data in RW_IRAM0 will be lost in standby sleep mode, while data in RW_IRAM1 will not be lost during standby sleep.

```c
  RW_IRAM0 HCPU_RAM_DATA_START_ADDR UNINIT {  ; ZI data, not retained
#ifdef BSP_USING_PM  
    *.o (non_ret) ; non-retention section
    *.o (STACK)   ; ISR stack
#endif

    *.o (.l1_non_ret_data_*)
    *.o (.l1_non_ret_bss_*)
#ifndef BSP_USING_PSRAM
    *.o (.l2_non_ret_data_*)
    *.o (.l2_non_ret_bss_*)
    *.o (.l2_cache_non_ret_data_*)
    *.o (.l2_cache_non_ret_bss_*)
    *.o (.nand_cache)
#endif
  }  

  RW_IRAM1 +0  {  ; RW data  retained

    *.o (.l1_ret_data_*)
    *.o (.l1_ret_bss_*)
   .ANY (+RW +ZI)
  }
```

In addition, HCPU has 64KB of Retention RAM, specified in the linker file by the RW_IRAM_RET region.
For hardware platforms without PSRAM, this region is used to back up SRAM data that needs to be preserved during sleep.

```c
  RW_IRAM_RET HPSYS_RETM_BASE HPSYS_RETM_SIZE {  

   *.o (.l1_ret_text_*)
   *.o (.l1_ret_rodata_*)
   *.o (.retm_bss_*)
   *.o (.retm_data_*)

   idle.o (.bss.rt_thread_stack)
   bf0_hal_rcc.o   (.text.*)
#ifdef BSP_USING_PM   
   bf0_pm_a0.o        (.text.sifli_light_handler)
   bf0_pm_a0.o        (.text.sifli_deep_handler)
   bf0_pm_a0.o        (.text.sifli_standby_handler)
   drv_io.o           (.text.*)
   bf0_hal_gpio.o     (.text.*)
#endif  

    drv_psram.o(.bss.bf0_psram_handle)
  }
```

To easily specify the memory sections where variables should be stored, for example, some variables that need to be preserved during sleep, you can use macros defined in _$SDK_ROOT/middleware/include/mem_section.h_ to specify where the variables should reside.
For example, the following code assigns the variable `g_ble_db` to the `L1_NON_RET_BSS` section, which is in RW_IRAM0, so that the data of this variable will not be preserved during sleep.

```c
L1_NON_RET_BSS_SECT_BEGIN(g_ble_db)
static struct fdb_kvdb g_ble_db;
L1_NON_RET_BSS_SECT_END
```

## 5. Memory Planning Example for HCPU and LCPU Projects
Taking _$SDK_ROOT/example/ble/ancs_dualcore/project/ec-lb555/_ as an example,

### 5.1 HCPU
In this example, HCPU uses the default macro definitions in `mem_map.h`, with the code being stored and executed at address 0x10020000. If you want to compile the code to the address 0x10060000 and increase the BIN size to 1024KB, follow these steps:

1) Modify `hcpu/Kconfig`

```c
mainmenu "Sifli Configuration"

config SIFLI_SDK
    string 
    option env="SIFLI_SDK"
    default "."

# SDK configuration
source "$SIFLI_SDK/Kconfig"

config BF0_HCPU
    bool
    default y

config CUSTOM_MEM_MAP
    bool 
    default y      
```

2) Execute menuconfig, save the configuration, and exit. Confirm that the `CUSTOM_MEM_MAP` has been added in the updated `rtconfig.h`.

3) Create a new file `_custom_mem_map.h_` under the `_hcpu/linker_scripts_` directory with the following content:

```c
#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#undef  HCPU_FLASH_CODE_START_ADDR
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_START_ADDR      0x10060000
#define HCPU_FLASH_CODE_SIZE            (1024*1024)

#endif  /* __CUSTOM_MEM_MAP__ */
```

4) Modify `_hcpu/board/SConscript_` to add the following path to the list of existing paths `[cwd + '/../linker_scripts']`:

```python
path =  [cwd]
path += [cwd + '/ports']
path += [cwd + '/../linker_scripts']  # <- add
```

5) Run `scons --clean` to delete the previous compilation results, then execute `scons -j8` to compile the new program.  
Check the generated `_bf0_ap.map_` in the build directory, where you can see that the HCPU code's load and execution addresses have changed to 0x10060000.

The starting address of the system heap is indicated by the segment name `Image$$RW_IRAM1$$ZI$$Limit` in the map file, and the end address is the starting address of the `ER_IROM1_EX` segment, which is 0x200FBC00.  
```c
    Image$$RW_IRAM1$$ZI$$Limit               0x2000721c   Number         0  anon$$obj.o ABSOLUTE
    Execution Region ER_IROM1_EX (Exec base: 0x200fbc00, Load base: 0x10095e6c, Size: 0x00001ab4, Max: 0x00004000, ABSOLUTE)    
```

```c
    Execution Region ER_IROM1 (Exec base: 0x10060000, Load base: 0x10060000, Size: 0x00032d28, Max: 0x00100000, ABSOLUTE)

    Exec Addr    Load Addr    Size         Type   Attr      Idx    E Section Name        Object

    0x10060000   0x10060000   0x000007c0   Data   RO          175    RESET               startup_bf0_hcpu.o
    0x100607c0   0x100607c0   0x00000008   Code   RO         5163  * !!!main             c_w.l(__main.o)
    0x100607c8   0x100607c8   0x00000034   Code   RO         5592    !!!scatter          c_w.l(__scatter.o)
```

### 5.2 LCPU
In general, the default memory planning of LCPU can meet the needs of most applications without customization.  
For example, in `_lcpu/linker_scripts/link_lcpu.sct_`, you can see that the code and data segments are closely stored. Although the macro `LCPU_RAM_CODE_SIZE` limits the code segment size to a maximum of 140KB, if the actual code segment only uses 100KB, the data segment's starting address will be `(0x200FC000 + 100*1024) = 0x20115000` and there will be no space wasted.

```c
LR_IROM1 LCPU_RAM_CODE_START_ADDR LCPU_RAM_CODE_SIZE {    ; load region size_region
  ER_IROM1 LCPU_RAM_CODE_START_ADDR LCPU_RAM_CODE_SIZE  {  ; load address = execution address
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
  }
  RW_IRAM1 AlignExpr(LPSYS_RAM_BASE+ImageLength(ER_IROM1), 16) ALIGN 16 NOCOMPRESS  {  ; RW data
   .ANY (+RW +ZI)
  }
  ; Load Address must be equal to Exec Address
  ScatterAssert((LoadBase(RW_IRAM1) OR 0x20000000) == ImageBase(RW_IRAM1))   ; <- Check that load address and exec address are the same
  ScatterAssert((ImageLength(ER_IROM1)+ImageLength(RW_IRAM1)+LCPU_MBOX_SIZE)<LPSYS_RAM_SIZE)
}
```

Here is a map file snippet where the code's load and execution address starts at 0x000FC000. The execution address of the data segment starts at 0x201095A0 and the load address is 0x001095A0, pointing to the same physical memory.

```c
    Execution Region ER_IROM1 (Exec base: 0x000fc000, Load base: 0x000fc000, Size: 0x0000d598, Max: 0x00023800, ABSOLUTE)
...
    Execution Region RW_IRAM1 (Exec base: 0x201095a0, Load base: 0x001095a0, Size: 0x000045b8, Max: 0xffffffff, ABSOLUTE)
```

The system heap's starting address is indicated by `Image$$RW_IRAM1$$ZI$$Limit` in the map file, and the ending address is 0x20123C00, the starting address of the mailbox buffer. Thus, the available heap size for LCPU is 90280 bytes.

```c
    Image$$RW_IRAM1$$ZI$$Limit               0x2010db58   Number         0  anon$$obj.o ABSOLUTE
```




