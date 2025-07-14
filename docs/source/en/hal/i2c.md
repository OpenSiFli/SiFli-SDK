
# I2C

I2C HAL provides basic APIs for accessing I2C peripheral registers.
Main features include:
 - Master mode only (I2C Master).
 - Supports up to 6 instances, with 3 on HCPU and 3 on LCPU.
 - 10-bit and 7-bit address support.
 - DMA/interrupt mode support
 - Memory mode access
 
## Using I2C HAL Driver
I2C supports DMA and interrupt modes, which need to be configured before I2C startup.

Example of using I2C HAL with memory mode and polling:

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

## API Reference
[](#hal-i2c)
