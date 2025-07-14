# EZIP

HAL EZIP 模块提供抽象的软件接口操作硬件EZIP模块，EZIP模块支持对ezip格式的数据进行解压缩。
支持的特性有:
- AHB和EPIC输出模式，AHB模式指解压后的数据保存到指定memory地址，EPIC模式指解压后的数据送入EPIC模块，
  使得EPIC可读取ezip格式图像数据实现混叠和缩放操作（不支持ezip格式数据的旋转），
  对于EPIC输出模式，用户可使用HAL EPIC接口配置EPIC读取ezip格式数据，HAL EPIC会配置EZIP进入EPIC输出模式，用户不必调用HAL EZIP接口
- 可指定图形的部分区域解压输出
- 支持中断和轮询模式
- AHB模式下支持LZ4和GZIP4的解压
- 除了SF32LB55X，其他芯片系列支持EZIPA动画数据的解压缩，中间件的ezipa_dec模块基于EPIC和EZIP驱动提供了更上层的软件支持，使用方法参考ezipa_dec模块的头文件


详细的API说明参考 [EZIP](#hal-ezip) .

## 使用HAL EZIP

首先调用 `HAL_EZIP_Init` 初始化HAL_EZIP, 需要在 `EZIP_HandleTypeDef` 中指定使用的EZIP硬件模块（即EZIP实例），芯片只有一个EZIP实例，由 `hwp_ezip` 表示。
初始化只需做一次，之后就可以调用 `HAL_EZIP_Decode` 和 `HAL_EZIP_Decode_IT` 解压数据。

```{note}
源和目的地址需要保证4字节对齐
```


例如,
```c
static EZIP_HandleTypeDef ezip_handle;

void init_ezip(void) 
{ 	// Initialize driver and enable EZIP IRQ
	HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EZIP_IRQn);
	
	ezip_handle.instance = hwp_ezip ;
	HAL_EZIP_Init(&ezip_handle);
}
```

### AHB output (polling mode)

```c
void example_ahb(void)
{
    EZIP_DecodeConfigTypeDef config;
    HAL_StatusTypeDef res;
    uint16_t width = 68;
    uint16_t height = 37;

    /* image size if 68*37, color format is ARGB565A, output entire image, output region is (0,0,)~(67,36) */
    config.input = (uint8_t *)img;
    config.output = malloc(width * height * 3);
    memset(config.output, 0, width * height * 3);
    
    config.start_x = 0;
    config.start_y = 0;
    config.width = 68;
    config.height = 37;
    config.work_mode = HAL_EZIP_MODE_EZIP;
    config.output_mode = HAL_EZIP_OUTPUT_AHB;

    res = HAL_EZIP_Decode(&ezip_handle, &config);
    
    /* image size if 68*37, color format is ARGB565A, output partial image, output region is (10,20)~(39,31) */
    config.input = (uint8_t *)img;
    config.output = malloc(width * height * 3);
    memset(config.output, 0, width * height * 3);
    
    config.start_x = 10;
    config.start_y = 20;
    config.width = 30;
    config.height = 12;
    config.work_mode = HAL_EZIP_MODE_EZIP;
    config.output_mode = HAL_EZIP_OUTPUT_AHB;

    res = HAL_EZIP_Decode(&ezip_handle, &config);    
}
```

### AHB output (interrupt mode)

```c
/* EZIP IRQ ISR in vector table */
void EZIP_IRQHandler(void)
{
    HAL_EZIP_IRQHandler(ezip_handle);
}

static uint8_t ezip_done_flag;

static void ezip_done(EZIP_HandleTypeDef *ezip)
{
    ezip_done_flag = 1;
}


void example_ahb_it(void)
{
    ZIP_DecodeConfigTypeDef config;
    HAL_StatusTypeDef res;
    uint16_t width = 68;
    uint16_t height = 37;

    /* image size if 68*37, color format is ARGB565A, output entire image, output region is (0,0,)~(67,36) */
    config.input = (uint8_t *)img;
    config.output = malloc(width * height * 3);
    memset(config.output, 0, width * height * 3);

    config.start_x = 0;
    config.start_y = 0;
    config.width = 68;
    config.height = 37;
    config.work_mode = HAL_EZIP_MODE_EZIP;
    config.output_mode = HAL_EZIP_OUTPUT_AHB;

    ezip_done_flag = 0;
    ezip->CpltCallback = ezip_done;    
    res = HAL_EZIP_Decode_IT(&ezip_handle, &config);
    
    /* wait for interrupt, ezip_done_flag is changed to 1 in ezip_done */
    while (0 == ezip_done_flag)
    {
    }
}
```
## API参考
[](#hal-ezip)

