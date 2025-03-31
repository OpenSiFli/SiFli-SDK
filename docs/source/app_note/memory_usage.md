# SF32LB55x内存使用指南

## 1. Memory地址空间

### HCPU Memory地址空间

|名称             | 起始地址       | 结束地址       | 大小(字节) | 描述
|-----------------|----------------|----------------|------------|--------
|HPSYS ROM        | 0x00000000     | 0x0000FFFF     | 64*1024    | |
|HPSYS ITCM RAM   | 0x00010000     | 0x0001FFFF     | 64*1024    | |
|HPSYS Retention RAM   | 0x00020000     | 0x0002FFFF     | 64*1024    | 可以在standby低功耗时不掉电|
|HPSYS SRAM       | 0x20000000     | 0x200FFFFF     | 1024*1024  | |
|LPSYS ROM        | 0x0B000000     | 0x0B05FFFF     | 384*1024    | |
|LPSYS ITCM RAM   | 0x0B0FC000     | 0x0B0FFFFF     | 16*1024   | |
|LPSYS DTCM RAM   | 0x2B0FC000     | 0x2B0FFFFF     | 16*1024   | |
|LPSYS SRAM       | 0x20100000     | 0x20137FFF     | 224*1024   | |
|QSPI1            | 0x10000000     | 0x11FFFFFF     | |EVB的容量为4MByte |
|QSPI2            | 0x64000000     | 0x67FFFFFF     | |EVB的容量为32MByte|
|QSPI3            | 0x68000000     | 0x68FFFFFF     | |EVB未接|
|QSPI4            | 0x12000000     | 0x13FFFFFF     | |LCPU可选flash,EVB未接|
|PSRAM-OPI        | 0x60000000     | 0x61FFFFFF     | |EVB的容量为4MByte |

```{note} 
HPSYS SRAM前128KB为DTCM，相比其后的地址空间访问速度更快
```

HCPU用于访问LPSYS Memory的地址转换接口
- LCPU SRAM地址转换: #LCPU_ADDR_2_HCPU_ADDR , 例如调用LCPU_ADDR_2_HCPU_ADDR(0x20100000)可得到0x20100000
- LCPU ROM地址转换: #LCPU_ROM_ADDR_2_HCPU_ADDR, 例如调用LCPU_ROM_ADDR_2_HCPU_ADDR(0x00000000)可得到0x0B000000
- LCPU ITCM地址转换: #LCPU_ITCM_ADDR_2_HCPU_ADDR, 例如调用LCPU_ITCM_ADDR_2_HCPU_ADDR(0x000FC000)可得到0x0B0FC000
- LCPU DTCM地址转换: #LCPU_DTCM_ADDR_2_HCPU_ADDR, 例如调用LCPU_DTCM_ADDR_2_HCPU_ADDR(0x200FC000)可得到0x2B0FC000

HCPU的代码可XIP执行在QSPI1/QSPI2/QSPI3上，也可以跑在HPSYS ITCM/SRAM/Retention RAM上


### LCPU Memory地址空间

|名称             | S-Bus 起始地址 | S-Bus 结束地址 | C-Bus 起始地址 | C-Bus 结束地址 | 大小(字节) | 描述
|-----------------|----------------|----------------|----------------|----------------|------------|------
|HPSYS SRAM       | 0x2A000000     | 0x2A0FFFFF     |                |                | 1024*1024  | |
|LPSYS ROM        |                |                | 0x00000000     |  0x0005FFFF    | 384*1024   | |
|LPSYS ITCM RAM   |                |                | 0x000FC000     |  0x000FFFFF    | 16*1024    | |
|LPSYS DTCM RAM   | 0x200FC000     | 0x200FFFFF     |                |                | 16*1024    | |
|LPSYS SRAM       | 0x20100000     | 0x20137FFF     | 0x00100000     |  0x00137FFF    | 224*1024   | | 
|QSPI4            |                |                | 0x12000000     |  0x13FFFFFF    |            |LCPU可选flash,EVB未接 |


为了提高性能，除了可以使用0x20100000~0x20137FFF地址段访问LPSYS SRAM外，还可以使用0x00100000~0x00137FFF访问，
前者使用S-Bus访问SRAM，后者使用C-Bus访问SRAM，他们访问的都是同一片物理SRAM。
例如，可以将代码分配在0x00100000开始的地址，SRAM剩余的空间则使用S-Bus的地址空间分配给数据段使用，
这样就可以使取指和取数并行，提高代码的执行效率。
也可以将ITCM和SRAM合并使用，比如代码起始地址为0x000FC000，超出16K字节的代码就会接着放在SRAM地址0x00100000开始的区域，剩余的SRAM空间仍旧使用S-Bus地址空间访问。
具体示例见第4和第5节。

LCPU用于访问HPSYS SRAM地址的转换接口，
- #HCPU_ADDR_2_LCPU_ADDR, 例如调用HCPU_ADDR_2_LCPU_ADDR(0x20000000)可得到0x2A000000

LCPU的代码可以运行在LPSYS的ITCM/DTCM/SRAM上，需要由HCPU上的用户程序搬到对应的地址空间

### QSPI1地址空间注意事项
Bootloader使用0x10000000开始的12kbyte作为Flash配置表，默认的Flash配置表指定的Flash1地址规划如下，用户程序起始地址为0x10020000。
如果使用自定义的Flash配置表，则必须保证不使用最开始的12kbyte，避免改写Flash配置表，其它地址空间可随意使用。

|名称             | 起始地址       | 结束地址       | 大小(字节) | 描述
|-----------------|----------------|----------------|------------|--------
|Flash配置表      | 0x10000000     | 0x10004FFF     | 20*1024    | |
|校准表           | 0x10005000     | 0x10006FFF     | 8*1024     | |
|保留             | 0x10007000     | 0x1000FFFF     | 36*1024    | |
|bootrom patch    | 0x10010000     | 0x1001FFFF     | 64*1024    | |
|User Code        | 0x10020000     | | | | 

### HPSYS SRAM地址空间注意事项
Bootloader使用下表所示的RAM地址空间，当使用Bootloader引导用户程序在RAM中执行时，RAM程序不能使用0x20040000地址，避免搬移后又被bootloader改写。
如果引导用户程序在Flash上执行，则没有这个限制。

bootloader RAM地址空间

|名称             | 起始地址       | 结束地址       | 大小(字节) | 描述
|-----------------|----------------|----------------|------------|--------------
|DATA             | 0x20040000     | 0x2004FFFF     | 64*1024    | bootrom数据 |
|PATCH CODE       | 0x20050000     | 0x2005FFFF     | 64*1024    | Patch代码   |
|PATCH DATA       | 0x20060000     | 0x2006FFFF     | 64*1024    | Patch数据   |

## 2. HPSYS Memory性能对比

|名称           | 频率(MHz)    | 说明 
|---------------|--------------|-------
|HCLK           | 240          |      |
|PSRAM-OPI      | 120          | DDR  |
|FLASH1         | 96           |      |
|FLASH2         | 80           |      |

一般而言，DMA访问SRAM效率最高，eDMA访问PSRAM/FLASH效率最高，具体吞吐率性能如下表所示，其中memcpy不能使用microlib，FLASH1和FLASH2使能了CACHE，PSRAM未使能CACHE。


| 源       | 目的       | memcpy(MB/s) | eDMA(MB/s) | DMA(MB/s) 
|----------|------------|--------------|------------|-------------
| SRAM     | SRAM       | 405.54       | 430.17     | 446.35    |
| PSRAM    | SRAM       | 43.34        | 125.54     | 53.23     |
| FLASH1   | SRAM       | 46.41        | 43.17      | 25.50     |
| FLASH2   | SRAM       | 36.30        | 36.13      | 21.24     |
| SRAM     | PSRAM      | 48.05        | 187.01     | 47.70     |
| PSRAM    | PSRAM      | 20.99        | 76.72      | 12.96     |
| FLASH1   | PSRAM      | 26.84        | 42.97      | 12.51     |
| FLASH2   | PSRAM      | 24.94        | 36.14      | 21.25     |

## 3. HCPU EPIC性能

|名称           | 频率(MHz)    | 说明 
|---------------|--------------|-------
|HCLK           | 240          |      |
|PSRAM-OPI      | 120          | DDR  |
|FLASH1         | 96           |      |
|FLASH2         | 80           |      |

以下未作说明，颜色格式均为RGB565

### Alpha混叠
| 前景     | 输出       | 性能(cycle/pixel ) | 吞吐率(Mpixel/s) 
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
上表为使用HAL_EPIC_BlendStart/HAL_EPIC_BlendStart_IT接口的性能，若使用HAL_EPIC_Rotate_IT/HAL_EPIC_Rotate，
SRAM作为前景时性能会有50%%的损失，其它情况会有20%%损失
```

### 缩小
| 前景     | 输出       | 性能(cycle/pixel ) | 吞吐率(Mpixel/s) 
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
前景图大小为88*88，缩放系数为1400
```

### 旋转
| 前景     | 输出       | 性能(cycle/pixel ) | 吞吐率(Mpixel/s)  
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
旋转角度为45°
```

### 填充
| 输出       | 性能(cycle/pixel ) | 吞吐率(Mpixel/s) 
|------------|--------------------|---------------
| SRAM       | 1.31               | 183.55      | 
| PSRAM      | 2.79               | 85.95       | 

```{note}
填充单色且透明度为255
```

## 4. 应用程序Memory规划及定制接口
在 _$SDK_ROOT/drivers/cmsis/sf32lb55x/mem_map.h_ 中定义了以下一些接口用于规划应用程序的Memory使用，
除了在头文件和C文件，也可以在link文件中使用这些宏，这样便于保证地址规划的一致性。

HCPU Flash和SRAM地址规划表
| 宏                          | 说明                          | 默认值     |   说明
|-----------------------------|-------------------------------|------------|---------
| HCPU_FLASH_CODE_START_ADDR  | HCPU应用程序XIP代码段起始地址 | 0x10020000（没有DFU）或者0x10100000 （有DFU）  | |
| HCPU_FLASH_CODE_SIZE        | HCPU应用程序XIP代码段大小     | 896KB        | |
| HCPU_FLASH2_IMG_START_ADDR  | 图片资源起始地址              | 0x64000000   | |
| HCPU_FLASH2_IMG_SIZE        | 图片资源大小                  | 4MB          | |
| HCPU_FLASH2_FONT_START_ADDR | 字体资源起始地址              | 0x64400000   | |
| HCPU_FLASH2_FONT_SIZE       | 字体资源大小                  | 12MB         | |
| HCPU_RAM_DATA_START_ADDR    | HCPU应用程序数据段起始地址    | 0x20000000   | |
| HCPU_RAM_DATA_SIZE          | HCPU应用程序数据段大小        | 1007KB       | |
| HCPU_RO_DATA_START_ADDR     | HCPU应用程序RAM代码段起始地址 | 0x200FBC00   | 用于存放需要做在SRAM中执行的代码 |
| HCPU_RO_DATA_SIZE           | HCPU应用程序RAM代码段大小     | 16KB         | |
| HCPU2LCPU_MB_CH2_BUF_START_ADDR           | HCPU2LCPU mailbox2共享buffer起始地址    | 0x200FFC00   | |
| HCPU2LCPU_MB_CH2_BUF_SIZE                 | HCPU2LCPU mailbox2共享buffer大小        | 512B   | |
| HCPU2LCPU_MB_CH1_BUF_START_ADDR           | HCPU2LCPU mailbox1共享buffer起始地址    | 0x200FFE00   | |
| HCPU2LCPU_MB_CH1_BUF_SIZE                 | HCPU2LCPU mailbox1共享buffer大小        | 512B   | |

HCPU由rt_malloc可分配的heap空间大小取决于HCPU应用程序数据段中静态变量占用了多大的空间，
比如静态变量如果占用了500KB，则heap空间就剩下1007-500=507KB。


OTA地址规划表
| 宏                          | 说明                      | 默认值     |  说明
|-----------------------------|---------------------------|------------|---------
| DFU_FLASH_CODE_START_ADDR   | DFU程序代码段起始地址     | 0x10020000        | |
| DFU_FLASH_CODE_SIZE         | DFU程序代码段大小         | 256KB   | |
| DFU_RES_FLASH_CODE_START_ADDR   | 应用程序升级区起始地址| 0x10060000   | |
| DFU_RES_FLASH_CODE_SIZE         | 应用程序升级区大小    | 640KB   | |
| HCPU_FLASH2_IMG_UPGRADE_START_ADDR   | 图片资源升级区起始地址  | 0x65000000   | |
| HCPU_FLASH2_IMG_UPGRADE_SIZE         | 图片资源升级区大小      | 1MB          | |
| HCPU_FLASH2_FONT_UPGRADE_START_ADDR   | 字体资源升级区起始地址 | 0x65100000   | |
| HCPU_FLASH2_FONT_UPGRADE_SIZE         | 字体资源升级区大小     | 3MB          | |



LCPU SRAM地址规划表
| 宏                          | 说明                      | 默认值  
|-----------------------------|---------------------------|------------------
| LCPU_RAM_CODE_START_ADDR    | LCPU应用程序代码段起始地址      | 0x000FC000    |
| LCPU_RAM_CODE_SIZE          | LCPU应用程序代码段大小          | 140KB   |
| LPSYS_RAM_SIZE              | LCPU应用程序代码与数据段总大小  | 160KB   |
| LCPU2HCPU_MB_CH1_BUF_START_ADDR           | LCPU2HCPU mailbox2共享buffer起始地址    | 0x20123C00   |
| LCPU2HCPU_MB_CH1_BUF_SIZE                 | LCPU2HCPU mailbox2共享buffer大小        | 512B   | 
| LCPU2HCPU_MB_CH2_BUF_START_ADDR           | LCPU2HCPU mailbox1共享buffer起始地址    | 0x20123E00   | 
| LCPU2HCPU_MB_CH2_BUF_SIZE                 | LCPU2HCPU mailbox1共享buffer大小        | 512B   |


LCPU由rt_malloc可分配的heap空间大小取决于LCPU应用程序代码与静态变量占用了多大的空间，
比如代码和静态变量总共占用了120KB，则heap空间就剩下160-120=40KB。

如果需要修改默认值，可以在工程的Kconfig中增加如下的`CUSTOM_MEM_MAP`选项，
运行menucofig并保存设置后rtconfig.h中就会增加一个CUSTOM_MEM_MAP开关，
这样mem_map.h就能包含custom_mem_map.h，可以在custom_mem_map.h中重定义需要修改的宏。

```c
config CUSTOM_MEM_MAP
    bool 
    default y  
```



### 4.1 低功耗相关接口
为了实现低功耗的Standby模式，HCPU工程的link.sct文件定义一些PM框架使用的接口, 例如 _$SDK_ROOTexample/pm/ble/ec-lb555/hcpu/linker_scripts/link_flash.sct_ 如果硬件平台支持PSRAM，则需要在link文件中定义如下的区域，SDK会使用RW_PSRAM2作为睡眠时数据的备份区域，
该区域中的数据在睡眠时无法保存，对于需要保存的数据可以将其放在RW_PSRAM1区。

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

类似的，SRAM也分为RW_IRAM0和RW_IRAM1区，RW_IRAM0中的数据在standby睡眠模式时会丢失，而RW_IRAM1中数据在standby睡眠时不会丢失

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

此外，HCPU还有64KB的Retention RAM，在link.sct文件中由RW_IRAM_RET区域指定，
对于没有PSRAM的硬件平台，该区域被用来备份睡眠时需要保存的SRAM数据。

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


为了便于指定变量存放的区域，比如有些变量的值在睡眠时需要保存，而有些变量不需要，可以使用 _$SDK_ROOT/middleware/include/mem_section.h_ 中定义的宏来指示变量存放的位置。
例如，下面的代码将变量g_ble_db指定到了L1_NON_RET_BSS段，也就是RW_IRAM0中，这样睡眠时就不会保存该变量的数据。

```c
L1_NON_RET_BSS_SECT_BEGIN(g_ble_db)
static struct fdb_kvdb g_ble_db;
L1_NON_RET_BSS_SECT_END
```

## 5. HCPU与LCPU项目内存规划实例
以 _$SDK_ROOT/example/ble/ancs_dualcore/project/ec-lb555/_ 为例，
### 5.1 HCPU
示例的HCPU使用mem_map.h中默认的宏定义，代码存放和执行地址为0x10020000，如果想将代码编译到0x10060000地址，BIN大小增加到1024KB，可以按以下几个步骤修改代码，

1) 修改`hcpu/Kconfig`

```c
mainmenu "Sifli Configuration"

config SIFLI_SDK
    string 
    option env="SIFLI_SDK"
    default "."

#SDK configuration	
source "$SIFLI_SDK/Kconfig"

config BF0_HCPU
    bool
    default y
    
config CUSTOM_MEM_MAP
    bool 
    default y      
    
```


2) 执行menuconfig，保存配置并退出，确认更新后的rtconfig.h里增加了CUSTOM_MEM_MAP

3) 在 _hcpu/linker_scripts_ 目录下新建 _custom_mem_map.h_ ，内容如下

```c

#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#undef  HCPU_FLASH_CODE_START_ADDR
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_START_ADDR      0x10060000
#define HCPU_FLASH_CODE_SIZE            (1024*1024)


#endif  /* __CUSTOM_MEM_MAP__ */
```

4) 修改 _hcpu/board/SConscript_ ，给已有的path列表，增加一个路径`[cwd + '/../linker_scripts']`

```python
path =  [cwd]
path += [cwd + '/ports']
path += [cwd + '/../linker_scripts']  # <-增加
```

5) 在命令行执行scons --clean删除原来的编译结果，再执行scons -j8编译新的程序，
查看build目录下生成 _bf0_ap.map_ 可以看到HCPU代码的加载和执行地址都已经变为0x10060000

系统heap的起始地址由map文件里的段名`Image$$RW_IRAM1$$ZI$$Limit`指示，结束地址为`ER_IROM1_EX`段的起始地址，即0x200FBC00
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
一般而言，LCPU默认的memory规划已经可以满足大部分应用的需求，无需定制。
例如在 _lcpu/linker_scripts/link_lcpu.sct_ 中可以看到，代码和数据段是紧密存放，虽然使用了宏`LCPU_RAM_CODE_SIZE`限定了代码段最大不能超过140KB，
但如果实际代码段只使用100KB，则数据段起始地址就是(0x200FC000+100*1024)=0x20115000，并不会浪费空间，
同时为了避免RW变量load地址和exec地址的错位导致带初值的变量初始化错误，RW区使用不压缩格式

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
  ScatterAssert((LoadBase(RW_IRAM1) OR 0x20000000) == ImageBase(RW_IRAM1))   ; <- 检查load地址与exec地址是否相同
  ScatterAssert((ImageLength(ER_IROM1)+ImageLength(RW_IRAM1)+LCPU_MBOX_SIZE)<LPSYS_RAM_SIZE)
}


```

以下为生成的map文件片段，代码段的加载和执行地址从0x000FC000开始，数据段的执行地址从0x201095A0开始，加载地址为0x001095A0，
由前文的LCPU的Memory地址空间可知，他们指向了同一片物理memory。

```c
    Execution Region ER_IROM1 (Exec base: 0x000fc000, Load base: 0x000fc000, Size: 0x0000d598, Max: 0x00023800, ABSOLUTE)

    Exec Addr    Load Addr    Size         Type   Attr      Idx    E Section Name        Object

    0x000fc000   0x000fc000   0x00000100   Data   RO          178    RESET               startup_bf0_lcpu.o
    0x000fc100   0x000fc100   0x00000000   Code   RO         3293  * .ARM.Collect$$$$00000000  mc_w.l(entry.o)
    0x000fc100   0x000fc100   0x00000004   Code   RO         3346    .ARM.Collect$$$$00000001  mc_w.l(entry2.o)
    0x000fc104   0x000fc104   0x00000004   Code   RO         3349    .ARM.Collect$$$$00000004  mc_w.l(entry5.o)
    0x000fc108   0x000fc108   0x00000000   Code   RO         3351    .ARM.Collect$$$$00000008  mc_w.l(entry7b.o)
...

    0x00109560   0x00109560   0x00000008   Data   RO          988    .sifli_reg.1        ble_connection_manager.o
    0x00109568   0x00109568   0x00000008   Data   RO          326    .sifli_reg.1.end    bf0_ble_common.o
    0x00109570   0x00109570   0x00000020   Data   RO         3419    Region$$Table       anon$$obj.o
    0x00109590   0x00109590   0x00000008   Data   RO          877    SerialTranExport    bf0_sibles_weather_service.o

    Execution Region RW_IRAM1 (Exec base: 0x201095a0, Load base: 0x001095a0, Size: 0x000045b8, Max: 0xffffffff, ABSOLUTE)

    Exec Addr    Load Addr    Size         Type   Attr      Idx    E Section Name        Object

    0x201095a0   0x001095a0   0x00000004   Data   RW         3299    .data               mc_w.l(rand.o)
    0x201095a4   0x001095a4   0x0000006c   Data   RW           42    .data..L_MergedGlobals  main.o
    0x20109610   0x00109610   0x00000024   Data   RW         1204    .data..L_MergedGlobals  drv_usart.o
    0x20109634   0x00109634   0x00000020   Data   RW         1320    .data..L_MergedGlobals  drv_i2c.o
...    
    0x201098a8   0x001098a8   0x00000010   Data   RW          267    .data.pin_service_cb  pin_service.o
    0x201098b8   0x001098b8   0x00000014   Data   RW         1093    .data.sifli_pm      bf0_pm_a0.o
    0x201098cc   0x001098cc   0x00000010   Data   RW         3273    .data.sys_service_config  sifli_lib.lib(data_service.o)
    0x201098dc        -       0x0000001c   Zero   RW          824    .bss..L_MergedGlobals  bf0_sibles_serial_trans_service.o

```

系统heap的起始地址由map文件里的段名`Image$$RW_IRAM1$$ZI$$Limit`指示，结束地址为0x20123C00，即mailbox buffer的起始地址，可见LCPU可用的heap大小为90280字节。
```c
    Image$$RW_IRAM1$$ZI$$Limit               0x2010db58   Number         0  anon$$obj.o ABSOLUTE
```



