# FFT

HAL FFT 模块提供抽象的软件接口操作硬件FFT模块，实现定点数的FFT和DCT变换

```{note}
SF32LB55x不支持FFT
```

```{note}
SF32LB58x不支持DCT
```

详细的API说明参考[FFT](#hal-fft) .

## 使用HAL FFT

首先调用 `HAL_FFT_Init` 初始化HAL_FFT, 需要在 #FFT_HandleTypeDef 中指定使用的FFT硬件模块（即FFT实例），FFT实例个数随芯片有所不同，如SF32LB58x系列有两个FFT实例，#hwp_fft1和#hwp_fft2，而
SF32LB56x系列则只有一个FFT实例，即#hwp_fft1。初始化只需做一次，之后就可以调用 `HAL_FFT_StartFFT` 、`HAL_FFT_StartDCT` 等函数处理数据。

```{note}
源和目的地址需要保证4字节对齐，支持源和目的地址相同
```

```{note}
在初始化HAL_FFT之前需先执行`HAL_RCC_EnableModule`使能相应的FFT模块
```

例如,
```c
static FFT_HandleTypeDef fft_handle;

void init_fft(void) 
{ 	// Initialize driver and enable FFT IRQ
	HAL_NVIC_SetPriority(FFT1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(FFT1_IRQn);
	
    HAL_RCC_EnableModule(RCC_MOD_FFT1);
    
	fft_handle.Instance = hwp_fft1 ;
	HAL_FFT_Init(&fft_handle);
}
```

### 轮询模式
```c
static uint32_t input_data[512];
static uint32_t output_data[512];

void fft_example_polling(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;

    /* 初始化 */
    memset(&config, 0, sizeof(config));

    /* 512点16比特复数FFT */
    config.bitwidth = FFT_BW_16BIT;
    config.fft_length = FFT_LEN_512;
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = input_data;
    config.output_data = output_data;

    res = HAL_FFT_StartFFT(&fft_handle, &config);
}
```

### 中断模式

```c
volatile static uint8_t fft_done_flag;
static uint32_t input_data[512];
static uint32_t output_data[512];

/* FFT1 IRQ ISR in vector table */
void FFT1_IRQHandler(void)
{
    HAL_FFT_IRQHandler(&fft_handle);
}

static void fft_done(FFT_HandleTypeDef *fft)
{
    fft_done_flag = 1;
}

void fft_example_it(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;

    /* 初始化 */
    memset(&config, 0, sizeof(config));

    /* 512点16比特的复数FFT */
    config.bitwidth = FFT_BW_16BIT;
    config.fft_length = FFT_LEN_512;
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = input_data;
    config.output_data = output_data;

    fft_done_flag = 0;
    fft_handle.CpltCallback = fft_done;    
    res = HAL_FFT_StartFFT_IT(&fft_handle, &config);
    
    /* wait for interrupt, fft_done_flag is changed to 1 in fft_done */
    while (0 == fft_done_flag)
    {
    }
}
```

## API参考
[](#hal-fft)
