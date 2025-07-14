# PSRAM

PSRAM HAL provides basic APIs for accessing PSRAM peripheral registers.
This module only exists in the A0 series. After the PRO version, PSRAM functionality is replaced by the MPI module, and the HAL layer no longer exists independently.

## Using PSRAM
After initialization, PSRAM memory can be accessed by CPU and DMA like normal SRAM memory, as shown below:

```c
// Initial PSRAM hardware before using it
PSRAM_HandleTypeDef hpsram;
HAL_PSRAM_Init(&hpsram);

// Define PSRAM base address for memory access, it cannot be changed
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

// Close device, keep it open for more users.
```

## API Reference
[](../api/hal/psram.md)