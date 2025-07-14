# FFT

HAL FFT module provides abstract software interface to operate hardware FFT module, implementing FFT and DCT transforms for fixed-point numbers.

```{note}
SF32LB55x does not support FFT
```

```{note}
SF32LB58x does not support DCT
```

For detailed API documentation, refer to [FFT](#hal-fft).

## Using HAL FFT

First call `HAL_FFT_Init` to initialize HAL_FFT. Need to specify the FFT hardware module used (i.e., FFT instance) in #FFT_HandleTypeDef. The number of FFT instances varies by chip. For example, SF32LB58x series has two FFT instances, #hwp_fft1 and #hwp_fft2, while SF32LB56x series has only one FFT instance, #hwp_fft1. Initialization only needs to be done once, then `HAL_FFT_StartFFT`, `HAL_FFT_StartDCT` and other functions can be called to process data.

```{note}
Source and destination addresses need to be 4-byte aligned, supporting same source and destination addresses
```

```{note}
Before initializing HAL_FFT, need to execute `HAL_RCC_EnableModule` to enable the corresponding FFT module
```

For example,
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

### Polling Mode
```c
static uint32_t input_data[512];
static uint32_t output_data[512];

void fft_example_polling(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;

    /* Initialize */
    memset(&config, 0, sizeof(config));

    /* 512-point 16-bit complex FFT */
    config.bitwidth = FFT_BW_16BIT;
    config.fft_length = FFT_LEN_512;
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = input_data;
    config.output_data = output_data;

    res = HAL_FFT_StartFFT(&fft_handle, &config);
}
```

### Interrupt Mode

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

    /* Initialize */
    memset(&config, 0, sizeof(config));

    /* 512-point 16-bit complex FFT */
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

## API Reference
[](#hal-fft)