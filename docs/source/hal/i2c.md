# I2C

I2C HAL 提供用于访问 I2C 外设寄存器的基本 API。
主要功能包括：
 - 仅限主模式 (I2C Master)。
 - 最多支持 6 个实例，并且3个在HCPU上，3个在 LCPU 上使用。
 - 10 位和 7 位地址支持。
 - DMA/中断模式支持
 - 内存模式访问
 
## 使用 I2C HAL 驱动程序
I2C 可以支持使用 DMA 和中断模式，它们需要在 I2C 启动之前进行配置。

将 I2C HAL 与内存模式和轮询一起使用的示例：

```c

I2C_HandleTypeDef i2c_Handle = {0};
uint16_t DevAddress, MemAddress, MemAddSize, Size;
uint32_t Timeout;
uint8_t *pData;

/* initial I2C controller */
i2c_Handle.Mode = HAL_I2C_MODE_MASTER;
i2c_Handle.Instance = hwp_i2c1;
i2c_Handle.core = CORE_ID_LCPU;
i2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
i2c_Handle.Init.ClockSpeed = 400000;
i2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
ret = HAL_I2C_Init(&i2c_Handle);
if (ret != HAL_OK)
{
	return;
}

DevAddress = 0x76;// slave device 7 bit or 10 bit address 
MemAddress = 0x10;	// address on slave device
MemAddSize = I2C_MEMADD_SIZE_8BIT;	// I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT
pData = (uint8_t *)malloc(256);
Size = 256;
Timeout = 100;	// 100ms

/* I2C Write */
HAL_I2C_Mem_Write(&i2c_Handle, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);

/* I2C read */
HAL_I2C_Mem_Read(&i2c_Handle, DevAddress, MemAddress, MemAddSize, pData,Size, Timeout);

...
```

Sample for using I2C HAL with register mode and DMA:

```c

uint16_t DevAddress, Size;
uint8_t *pData;
I2C_HandleTypeDef handle = {0};
static struct dma_config i2c_dmarx;	// allocated buffer for dma handle in I2C instance
static struct dma_config i2c_dmatx;
struct dma_config dma_set;

/* initial I2C DMA info */
dma_set.dma_rcc = I2C1_DMA_RCC;
dma_set.Instance = I2C1_DMA_INSTANCE;
dma_set.dma_irq = I2C1_DMA_IRQ;
dma_set.request = I2C1_DMA_REQUEST;


/* bind dma handle */
//__HAL_LINKDMA(handle, hdmarx, i2c_dmarx);
//__HAL_LINKDMA(handle, hdmatx, i2c_dmatx);
handle.hdmarx = &i2c_dmarx;
handle.hdmatx = &i2c_dmatx;
HAL_I2C_DMA_Init(&handle, &dma_set, &dma_set);

/* Enable RX dma interrupt by default */
HAL_NVIC_SetPriority(dma_set.dma_irq, 0, 0);
HAL_NVIC_EnableIRQ(dma_set.dma_irq);

handle.Instance = hwp_i2c1;
handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
handle.Init.ClockSpeed = 400000;
handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
handle.core = CORE_ID_LCPU;
handle.Mode = HAL_I2C_MODE_MASTER;
HAL_I2C_Init(&handle);

/* I2C Write */
HAL_DMA_Init(handle.dma_tx);
DevAddress = 0x76; // set slave i2c device id
pData = malloc(256);
Size = 256;
HAL_I2C_Master_Transmit_DMA(&handle, DevAddress,pData, Size);

/* I2C read */
HAL_DMA_Init(handle.dma_rx);
HAL_I2C_Master_Receive_DMA(&handle, DevAddress,pData, Size);

...
```

## API参考
[](#hal-i2c)
