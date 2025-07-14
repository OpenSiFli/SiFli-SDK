# PSRAM

PSRAM HAL 提供用于访问 psram 外设寄存器的基本 API。
该模块只存在与A0系列，到PRO之后的版本，PSRAM功能由MPI模块代替，HAL层不再独立存在。

## 使用PSRAM
初始化后，PSRAM 内存可以像普通的 sram 内存一样被 CPU 和 DMA 访问，如下所示：

```c

// Initial PSRAM hardware before using it
PSRAM_HandleTypeDef hpsram;
HAL_PSRAM_Init(&hpsram);

// Define PSRAM base address for memory access, it can not be changed
#define PSRAM_BASE_ADDR             (0x60000000)

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


## API参考
[](../api/hal/psram.md)

