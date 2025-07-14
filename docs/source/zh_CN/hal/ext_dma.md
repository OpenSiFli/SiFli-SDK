# EXTDMA


EXTDMA 在HCPU, 用于内存-内存之间数据的高效率搬运工作（特别是与PSRAM相关的数据搬运, 比DMA内存-内存的传输更高效），并且在搬运的同时可实时进行图像的有损压缩(52x不支持)。


## 主要特性
 - 源地址和目标地址均为4字节访问, 且地址均需要4字节对齐, 长度也要4字节对齐
 - 源地址和目标地址独立支持BURST访问(BURST1/4/8/16)，并支持地址自动递增
 - 单次配置最大传输单元数为2^20-1，每单元为4字节传输，即单次最大传输4M bytes
 - 支持3种事件标志-传输完成，过半传输，传输出错
 - 可配置为图像压缩模式，支持RGB565/RGB888/ARGB8888格式输入


## 各种颜色格式下的压缩档位对应压缩率(共10档)
 - rgb565    {1.33, 1.47, 1.6, 1.73, 1.87, 1.93, 2, 2.13, 2.26, 2.4}
 - rgb888    {2, 2.2, 2.4, 2.6, 2.8, 2.9, 3, 3.2, 3.4, 3.6}
 - argb8888  {2.67, 2.93, 3.2, 3.47, 3.73, 3.86, 4.0, 4.27, 4.53, 4.8}


## 使用EXTDMA示例1

将0x20000000地址上100x100大小的RGB888图片压缩并搬运到0x60000000，压缩档位1(即2倍)

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

## API参考
[](../api/hal/ext_dma.md)

