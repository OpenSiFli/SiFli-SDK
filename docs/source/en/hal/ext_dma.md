# EXTDMA

EXTDMA is in HCPU, used for efficient data transfer between memory-to-memory (especially data transfer related to PSRAM, more efficient than DMA memory-to-memory transfer), and can perform real-time lossy compression of images during transfer (52x does not support).

## Main Features
 - Both source and destination addresses use 4-byte access, and addresses must be 4-byte aligned, length must also be 4-byte aligned
 - Source and destination addresses independently support BURST access (BURST1/4/8/16), and support automatic address increment
 - Single configuration maximum transfer unit count is 2^20-1, each unit is 4-byte transfer, i.e., single maximum transfer 4M bytes
 - Supports 3 event flags - transfer complete, half transfer, transfer error
 - Can be configured as image compression mode, supports RGB565/RGB888/ARGB8888 format input

## Compression Ratios for Various Color Formats (10 levels)
 - rgb565    {1.33, 1.47, 1.6, 1.73, 1.87, 1.93, 2, 2.13, 2.26, 2.4}
 - rgb888    {2, 2.2, 2.4, 2.6, 2.8, 2.9, 3, 3.2, 3.4, 3.6}
 - argb8888  {2.67, 2.93, 3.2, 3.47, 3.73, 3.86, 4.0, 4.27, 4.53, 4.8}

## EXTDMA Example 1

Compress and transfer 100x100 size RGB888 image at address 0x20000000 to 0x60000000, compression level 1 (i.e., 2x)

```c
static volatile HAL_StatusTypeDef endflag;
static EXT_DMA_HandleTypeDef DMA_Handle = {0};

void EXTDMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_EXT_DMA_IRQHandler(&DMA_Handle);

    /* leave interrupt */
    rt_interrupt_leave();
}


static void dma_done_cb()
{
    endflag = HAL_OK;
}

static void dma_err_cb()
{
    endflag = HAL_ERROR;
}


void main(void)
{
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t len;

    /*Data copy config    */
    DMA_Handle.Init.SrcInc = HAL_EXT_DMA_SRC_INC | HAL_EXT_DMA_SRC_BURST16; //Source address auto-increment and burst 16
    DMA_Handle.Init.DstInc = HAL_EXT_DMA_DST_INC | HAL_EXT_DMA_DST_BURST16; //Dest address auto-increment and burst 16

    /*Compression config  */
    DMA_Handle.Init.cmpr_en = true;
    if(DMA_Handle.Init.cmpr_en)
    {
	    DMA_Handle.Init.src_format = EXTDMA_CMPRCR_SRCFMT_RGB888;
	    DMA_Handle.Init.cmpr_rate = 1;
	    DMA_Handle.Init.col_num = 100;
	    DMA_Handle.Init.row_num = 100;
    }

    len = DMA_Handle.Init.col_num * DMA_Handle.Init.row_num * get_byte_per_pixel(DMA_Handle.Init.src_format) / 4;

    res = HAL_EXT_DMA_Init(&DMA_Handle);

    HAL_EXT_DMA_RegisterCallback(&DMA_Handle, HAL_EXT_DMA_XFER_CPLT_CB_ID, dma_done_cb);
    HAL_EXT_DMA_RegisterCallback(&DMA_Handle, HAL_EXT_DMA_XFER_ERROR_CB_ID, dma_err_cb);
    
    /* NVIC configuration for EXTDMA transfer complete interrupt */
    HAL_NVIC_SetPriority(EXTDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTDMA_IRQn);

    HAL_EXT_DMA_Start_IT(&DMA_Handle, 0x20000000, 0x60000000, len);
}


```

## API Reference
[](../api/hal/ext_dma.md)
