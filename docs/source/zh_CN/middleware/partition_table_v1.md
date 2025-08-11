# 分区表语法1.0版本

分区表描述文件`ptab.json`是一个json格式的文本文件，遵循json语法，可以使用任意文本编辑器编辑。
分区表使用列表定义了系统中用到的所有memory，如flash2、psram1等。1.0版本的分区表没有专门的属性定义版本号，或者使用一个特殊的列表元素（称为头信息）并携带了`version`属性指定版本号为`1`，如下面这样，

```json
[
    {"version": "1"},
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

列表中每个元素（头信息除外）对应一块memory，比如下面的示例中有flash5、psram1、flash4、hpsys_ram等memory，每个memory有如下属性：
- `mem`：memory的名称
- `base`：memory的基地址，十六进制数值，需保证正确性
- `regions`：memory中的分区列表

`regions`定义了memory中有哪些分区，每个分区有如下属性：
- `offset`：偏移，字节单位，十六进制
- `max_size`：大小，字节单位，十六进制
- `tags`：标签列表，列表中的名称用于生成相应的宏定义，用于表示分区大小和起始地址，比如tag为`FLASH_BOOT_LOADER`，生成的头文件会定义`FLASH_BOOT_LOADER_START_ADDR`、`FLASH_BOOT_LOADER_OFFSET`和`FLASH_BOOT_LOADER_SIZE`三个宏, `START_ADDR`由`base`加上`offset`的属性值得到，`OFFSET`等于`offset`属性值，`SIZE`等于`max_size`属性值
- `ftab`：flash table条目信息，可选
- `img`：本分区存放的镜像文件名，用于生成烧写脚本，可选
- `exec`：本分区上运行的程序名，可选
- `custom`：自定义宏字典，可选


````{dropdown} ptab.json示例
```json
[
    {
        "mem": "flash5", 
        "base": "0x1C000000", 
        "regions": [
            {
                "offset": "0x00020000", 
                "max_size": "0x00020000", 
                "tags": [
                    "FLASH_BOOT_LOADER"
                ], 
                "ftab": {
                    "name": "bootloader", 
                    "address": [
                        "base", 
                        "xip"
                    ]
                }, 
                "img": "bootloader",
                "exec": "bootloader"
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
                "tags": [], 
                "ftab": {
                    "name": "main", 
                    "address": [
                        "xip"
                    ]
                }
            }, 
            {
                "offset": "0x00200000", 
                "max_size": "0x00200000", 
                "tags": [
                    "PSRAM_DATA"
                ]
            },
        ]
    }, 
    {
        "mem": "psram1_cbus", 
        "base": "0x10000000", 
        "regions": [
            {
                "offset": "0x00000000", 
                "max_size": "0x00200000", 
                "tags": [
                    "HCPU_FLASH_CODE"
                ],
                "exec": "main"
            }
        ]
    }, 
    {
        "mem": "flash4", 
        "base": "0x18000000", 
        "regions": [
            {
                "offset": "0x00000000", 
                "max_size": "0x00200000", 
                "tags": [
                    "HCPU_FLASH_CODE_LOAD_REGION"
                ], 
                "img": "main", 
                "ftab": {
                    "name": "main", 
                    "address": [
                        "base"
                    ]
                }
            }, 
            {
                "offset": "0x00200000", 
                "max_size": "0x00100000", 
                "tags": [
                    "FS_REGION"
                ]
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
                "tags": [
                    "HCPU_RAM_DATA"
                ]
            }, 
            {
                "offset": "0x0006BC00", 
                "max_size": "0x00014000", 
                "tags": [
                    "HCPU_RO_DATA"
                ]
            }, 
        ]
    }
]
```
````

## 属性详解

###  ftab
`ftab`属性定义了flash table中某一个条目的参数，以此生成flash table的描述文件`ftab.c`用于编译。flash table会影响程序的启动，因此需要确保正确填写`ftab`属性。ftab包含以下属性：
- `name`：程序名称，`bootloader`表示二级bootloader，`main`表示主程序，`dfu`表示dfu程序，他们是系统预留的名称，不能随便使用，除此以外的名字可以由用户随意指定，如`acpu_region1`可以用来表示ACPU的代码分区
- `address`：地址类型列表，如果分区是用于存放程序的镜像文件，则填写`"base"`，如果程序运行在分区所在的memory上，则填写`"xip"`，如果两者都是（即存储地址与执行地址相同，程序可XIP执行（execution in place）），则需要同时填写`"base"`和`"xip"`

下面为flash5的一段描述，由代码可知，`bootloader`即二级bootloader存放于flash5中，地址段为`0x1C020000`~`0x1C03FFFF`，同时二级bootloader的代码也运行在flash5上，即XIP执行，

```json
{
    "mem": "flash5", 
    "base": "0x1C000000", 
    "regions": [
        {
            "offset": "0x00020000", 
            "max_size": "0x00020000", 
            "tags": [
                "FLASH_BOOT_LOADER"
            ], 
            "ftab": {
                "name": "bootloader", 
                "address": [
                    "base", 
                    "xip"
                ]
            }, 
            "img": "bootloader",
            "exec": "bootloader"
        }
    ]
}, 
```

非系统预留的程序（即`name`不是`main`、`bootloader`和`dfu`）将依次使用flash table中以下条目描述flash中存放的镜像信息
- DFU_FLASH_HCPU_EXT2： 11
- DFU_FLASH_LCPU_EXT1： 12
- DFU_FLASH_LCPU_EXT2： 13

例如下面这段ptab.json中定义了`acpu_region1`使用flash4的`0x00300000`偏移地址作为ACPU程序的base地址，hpsys_ram的`0x00200000`偏移地址作为ACPU程序的xip地址，生成的ftab.c文件使用`DFU_FLASH_HCPU_EXT2`项描述了ACPU程序镜像文件的存放和执行地址，length（需要搬移的数据长度）则由`img`属性（详细说明见[](#img)）指定的程序文件名按照实际文件大小填写（示例中实际长度为0x7d4字节）。

````{dropdown} acpu_region1示例代码
```json
{
    "mem": "flash4", 
    "base": "0x18000000", 
    "regions": [
        {
            "offset": "0x00300000", 
            "max_size": "0x00010000", 
            "tags": [
                "ACPU_CODE_LOAD_REGION1"
            ], 
            "img": "acpu", 
            "ftab": {
                "name": "acpu_region1", 
                "address": [
                    "base"
                ]
            }
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
            "tags": [
                "ACPU_CODE_REGION1_SBUS"
            ], 
            "ftab": {
                "name": "acpu_region1", 
                "address": [
                    "xip"
                ]
            }
        }
    ]
}, 
```
````

下面是由以上的ptab.json示例代码生成的ftab.c文件的内容

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


下图说明了ftab.c中ACPU程序的存放与执行地址是如何从ptab.json提取的。


![](../../assets/partition_table/ptab_ftab.png)


### img

`img`属性指定分区存放的程序镜像文件名，烧写脚本根据该属性得到镜像文件的烧写地址，当ftab属性描述的base与xip地址相同时不必使用img属性，因为编译生成的hex文件中包含的地址信息可以直接用于烧写。例如下面的示例代码中，由主程序main的ftab定义可知，它的base地址为0x18000000，xip地址为0x60000000，即二级bootloader会将主程序代码由0x18000000搬到0x60000000地址后跳转执行，因此flash4的第一个分区使用img属性表示该分区存放了主程序main的镜像文件。指定的镜像文件名必须是当前工程生成的二进制镜像文件，如果生成的二进制文件只有一个，可以省略扩展名`.bin`，直接以`main`、`bootloader`和`lcpu`等命名，如果有多个文件，则以`proj_name:binary_name`格式指定文件名，proj_name为工程名，主工程为`main`，子工程则与编译脚本配置的名称一致，比如acpu作为子工程加入编译，子工程名为acpu，生成的两个二进制文件分别为`ER_IROM1.bin`和`ER_IROM2.bin`，可以用`acpu:ER_IROM1.bin`表示ACPU编译生成的ER_IROM1.bin文件，用`acpu:ER_IROM2.bin`表示ACPU编译生成的ER_IROM2.bin文件。


```json
{
    "mem": "flash4", 
    "base": "0x18000000", 
    "regions": [
        {
            "offset": "0x00000000", 
            "max_size": "0x00200000", 
            "tags": [
                "HCPU_FLASH_CODE_LOAD_REGION"
            ], 
            "img": "main", 
            "ftab": {
                "name": "main", 
                "address": [
                    "base"
                ]
            }
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
            "tags": [], 
            "ftab": {
                "name": "main", 
                "address": [
                    "xip"
                ]
            }
        }
    ]
}, 
{
    "mem": "psram1_cbus", 
    "base": "0x10000000", 
    "regions": [
        {
            "offset": "0x00000000", 
            "max_size": "0x00200000", 
            "tags": [
                "HCPU_FLASH_CODE"
            ],
            "exec": "main"
        }
    ]
}, 
```

### exec 
`exec`属性指定分区上存放的可执行程序的名称，其目的是为了复用同一个链接脚本编译主程序和子程序，比如二级bootloder、dfu和主程序main都使用同一个链接脚本，代码地址由`CODE_START_ADDR`指定，当编译二级bootloader时，生成的`ptab.h`如下，`CODE_START_ADDR`指向了`FLASH_BOOT_LOADER_START_ADDR`

```c
#undef  FLASH_BOOT_LOADER_START_ADDR
#define FLASH_BOOT_LOADER_START_ADDR                       (0x1C020000)
#undef  FLASH_BOOT_LOADER_SIZE
#define FLASH_BOOT_LOADER_SIZE                             (0x00020000)
#undef  FLASH_BOOT_LOADER_OFFSET
#define FLASH_BOOT_LOADER_OFFSET                           (0x00020000)
#define CODE_START_ADDR                                    (FLASH_BOOT_LOADER_START_ADDR)
#define CODE_SIZE                                          (FLASH_BOOT_LOADER_SIZE)```
```

而编译主程序生成的`ptab.h`则是这样：

```c
#undef  HCPU_FLASH_CODE_START_ADDR
#define HCPU_FLASH_CODE_START_ADDR                         (0x10000000)
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_SIZE                               (0x00200000)
#undef  HCPU_FLASH_CODE_OFFSET
#define HCPU_FLASH_CODE_OFFSET                             (0x00000000)
#define CODE_START_ADDR                                    (HCPU_FLASH_CODE_START_ADDR)
#define CODE_SIZE                                          (HCPU_FLASH_CODE_SIZE)
```

```{note}
需要注意的是，对于PSRAM分区，存在CBUS和SBUS两个地址空间，例如上面的例子中CBUS地址是`0x10000000`，SBUS地址是`0x60000000`，`exec`属性需要填写在CBUS分区中，以确保代码执行地址使用CBUS地址空间，而`ftab`属性的`"xip"`类型则填写在SBUS分区中，以确保使用SBUS地址搬移代码
```

### custom
使用custom属性可在生成的ptab.h中加入自定义的宏定义，custom属性值为字典数据类型，例如下面这行代码，定义了三个宏，分别为PSRAM_BL_MODE、PSRAM_BL_SIZE和PSRAM_BL_MPI。

```json
"custom": {
    "PSRAM_BL_MODE": 3, 
    "PSRAM_BL_SIZE": 8, 
    "PSRAM_BL_MPI": 2
}
```

```c
#define PSRAM_BL_MODE  (3)
#define PSRAM_BL_SIZE  (8)
#define PSRAM_BL_MPI   (1)
```

```{note}
字典key的value值必须为整数
```

