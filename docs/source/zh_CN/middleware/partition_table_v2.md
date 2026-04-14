# 分区表语法2.0版本
分区表描述文件`ptab.json`是一个json格式的文本文件，遵循json语法，可以使用任意文本编辑器编辑。
分区表使用列表定义了系统中用到的所有memory，如flash2、psram1等。2.0版本的分区表使用一个特殊的列表元素（称为头信息）并携带了`version`属性指定版本号为`2`，如下面这样，
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

列表中每个元素（头信息除外）对应一块memory，比如下面的示例中有flash5、psram1、flash4、hpsys_ram等memory，每个memory有如下属性：
- `mem`：memory的名称
- `base`：memory的基地址，十六进制数值，需保证正确性
- `regions`：memory中的分区列表

`regions`定义了memory中有哪些分区，每个分区有如下属性：
- `offset`：偏移，字节单位，十六进制
- `max_size`：大小，字节单位，十六进制
- `tags`：可选，标签列表，列表中的名称用于生成相应的宏定义，用于表示分区大小和起始地址，比如tag为`FLASH_BOOT_LOADER`，生成的头文件会定义`FLASH_BOOT_LOADER_START_ADDR`、`FLASH_BOOT_LOADER_OFFSET`和`FLASH_BOOT_LOADER_SIZE`三个宏, `START_ADDR`由`base`加上`offset`的属性值得到，`OFFSET`等于`offset`属性值，`SIZE`等于`max_size`属性值
- `name`：分区名称
- `type`：可选，分区类型
- `core`: 可选，当分区类型为`app_exec`时，表示运行该分区代码的内核，可用的名称有`hcpu`、`lcpu`和`acpu`
- `custom`：可选，自定义宏字典


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

## 属性详解
### name
`name`属性定义了分区名称，对于`app_img`或者`app_exec`类型的分区（具体参见[](#type)），它表示该分区与哪个程序关联，`bootloader`表示二级bootloader，`main`表示主程序，`dfu`表示dfu程序，`ftab`为flash table，他们是系统预留的名称，不能随便使用，除此以外的名字可以由用户随意指定，如`acpu`可以用来表示ACPU的代码分区。需要注意的是，对于程序镜像文件分区，分区名必须与程序名相同，例如ACPU和LCPU的程序作为子工程（使用`AddChildProj`函数）加入主程序编译时使用的名称分别为`acpu`和`lcpu`，那么他们的程序名就是`acpu`和`lcpu`，指定他们的程序分区时使用的名称也应该是`acpu`和`lcpu`。另外一种情况是分区存放预编译好的二进制文件，使用`AddCustomImg`函数添加了二进制镜像文件，需要确保`AddCustomImg`指定的`name`参数与分区的`name`是相同的。

此外，程序文件可能有多个文件组成，比如主程序`main`包含了2个bin文件，一个是代码（ER_IROM1.bin），另一个是资源（ER_IROM2.bin），这时对应分区的`name`属性值可以分别设置为`main:1`和`main:2`，`main:1`对应文件`ER_IROM1.bin`，`main:2`对应文件`ER_IROM2.bin`。总结起来，`name`的取值为`proj_name:num`，`proj_name`为程序名，`num`为1、2、3，依次类推，如果只有一个文件，num可以省略。

### type
`type`属性定义的分区的类型，他是一个字符串列表变量，支持的类型有：
- `app_img`：分区用于存放程序镜像文件（主分区），烧写工具会往该地址下载程序
- `app_img2`: 分区用于存放程序镜像文件的备份（备份分区），一般用于升级时与主分区互为备份
- `app_exec`：分区用于执行程序代码，即代码运行在该分区的地址空间

下面为flash5的一段描述，由代码可知，`bootloader`即二级bootloader存放于flash5中，地址段为`0x1C020000`~`0x1C03FFFF`，同时二级bootloader的代码也运行在flash5上，即存储地址与执行地址相同，程序XIP执行（execution in place）在flash5上，
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

非系统预留的程序代码分区名（即`name`不是`main`、`bootloader`、`ftab`和`dfu`）将依次使用flash table中以下条目描述flash中存放的镜像信息
- DFU_FLASH_HCPU_EXT2： 11
- DFU_FLASH_LCPU_EXT1： 12
- DFU_FLASH_LCPU_EXT2： 13

例如下面这段ptab.json中定义了`acpu`使用flash4的`0x00300000`偏移地址作为ACPU程序的存放地址，hpsys_ram的`0x00200000`偏移地址作为ACPU程序的执行地址，生成的ftab.c文件使用`DFU_FLASH_HCPU_EXT2`项描述了ACPU程序镜像文件的存放和执行地址，length（需要搬移的数据长度）则由`name`属性（详细说明见[](#name)）指定的程序文件名按照实际文件大小填写（示例中实际长度为0x7d4字节）。

````{dropdown} ACPU程序分区示例
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

## 缺省的分区配置
如果分区表中未定义`ftab`和`bootloader`的分区配置，工具会使用默认配置生成`ptab.h`，用户可以查看生成的文件了解默认配置，一般而言，默认配置已能满足大部分应用的开发需求，用户也可以在分区表中加入`ftab`和`bootloader`的配置以替换默认参数，但需要注意，`ftab`必须放在正确位置以确保能被芯片的bootrom读取到，具体要求如下：
- SF32LB55X系列：flash1（即大核内置的NOR Flash，接在QSPI1接口上）的首地址
- SF32LB58X系列：flash5（即小核内置的NOR Flash，接在MPI5接口上）的首地址
- SF32LB56X系列：flash5（即小核内置的NOR Flash，接在MPI5接口上）的首地址
- SF32LB52X系列：
    - 有内置Flash的芯片，`ftab`需放在内置Flash（即flash1，接在MPI1接口上）的首地址，
    - 没有内置Flash且外置了SPI NOR或者SPI NAND Flash的芯片，放在外置Flash的首地址，
    - 对于eMMC、SD-NAND和SD卡，需要从0x1000偏移地址开始放置

