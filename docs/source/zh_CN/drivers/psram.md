# PSRAM

PSRam 驱动包括两层：硬件访问层（HAL）和 RT-Thread 的适配层。<br>
HAL 提供用于访问 psram 外设寄存器的基本 API。 有关详细信息，请参阅 psram HAL 的 API 文档。<br>
适配层提供硬件初始接口。 初始化后，用作sram存储器。 

## 驱动配置

硬件驱动仅使用一个实例，其基地址为 0x60000000，不可更改。 它可以使用 menuconfig 工具为每个项目启用，通常保存在 C 头文件中。 默认情况下，配置保存为 _rtconfig.h_。 

下面的例子显示了在一个项目头文件中定义的标志，项目是启用 PSRAM 控制器并设置它的内存大小（以 MB 为单位）。 步骤选择配置：
- 在项目下的命令中输入“menuconfig”
- 选择“RTOS --->”
- 选择“On-chip Peripheral Driver--->”	
- 选择“Enable PSRAM --->”             使用 psram 驱动，定义宏BSP_USING_PSRAM
- 输入“PSram full chip size(MB)”      以MB为单位设置psram内存大小，定义宏PSRAM_FULL_SIZE
```c
#define BSP_USING_PSRAM
#define PSRAM_FULL_SIZE 4
```
配置完成后，用户需要在所有需要访问驱动程序的源代码中包含头文件。

## 内存地址和初始接口
使用 psram 内存时，其基地址定义在内存映射中：
```c
#define PSRAM_BASE          (0x60000000)
```

初始接口，使用psram前调用一次。
```c
/**
 * @brief psram hardware initial.
 * @param[none] .
 * @return 0 if initial success.
 */
int rt_psram_init(void);
```

## 使用 PSRAM

初始化后，PSRAM 内存可以像普通的 sram 内存一样被 CPU 和 DMA 访问，如下所示：

```c

// Initial PSRAM hardware before using it
rt_psram_init(); 

// Define PSRAM base address for memory access, it can not be changed
#define PSRAM_BASE_ADDR             PSRAM_BASE

int *buf = (int *)PSRAM_BASE_ADDR;
int i;

// Write psram memory
for(i=0; i<1000; i++)
    buf[i] = i*6543;

// Read psram
int value = *buf;

// Read and Write
int *src = (int *)PSRAM_BASE_ADDR;
int *dst = (int *)(PSRAM_BASE_ADDR + 0x100000);
memcpy(dst, src, 1000);


...

// Close device, keep it open for more user.


```


