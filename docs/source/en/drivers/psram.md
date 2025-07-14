# PSRAM

PSRam driver includes two layers: hardware access layer (HAL) and RT-Thread adaptation layer.<br>
HAL provides basic APIs for accessing psram peripheral registers. For details, please refer to the API documentation of psram HAL.<br>
The adaptation layer provides hardware initial interface. After initialization, it is used as sram memory.

## Driver Configuration

The hardware driver uses only one instance with a base address of 0x60000000, which cannot be changed. It can be enabled for each project using the menuconfig tool, usually saved in C header files. By default, the configuration is saved as _rtconfig.h_.

The following example shows the flags defined in a project header file, where the project enables the PSRAM controller and sets its memory size (in MB). Configuration selection steps:
- Enter "menuconfig" in the command under the project
- Select "RTOS --->"
- Select "On-chip Peripheral Driver--->"	
- Select "Enable PSRAM --->"             Use psram driver, define macro BSP_USING_PSRAM
- Enter "PSram full chip size(MB)"      Set psram memory size in MB, define macro PSRAM_FULL_SIZE
```c
#define BSP_USING_PSRAM
#define PSRAM_FULL_SIZE 4
```
After configuration, users need to include header files in all source code that needs to access the driver.

## Memory Address and Initial Interface
When using psram memory, its base address is defined in the memory map:
```c
#define PSRAM_BASE          (0x60000000)
```

Initial interface, called once before using psram.
```c
/**
 * @brief psram hardware initial.
 * @param[none] .
 * @return 0 if initial success.
 */
int rt_psram_init(void);
```

## Using PSRAM

After initialization, PSRAM memory can be accessed by CPU and DMA like normal sram memory, as shown below:

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