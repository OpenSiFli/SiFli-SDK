
# EZIP

HAL EZIP module provides abstract software interface to operate hardware EZIP module. EZIP module supports decompression of ezip format data.
Supported features:
- AHB and EPIC output modes. AHB mode means decompressed data is saved to specified memory address. EPIC mode means decompressed data is sent to EPIC module, allowing EPIC to read ezip format image data for blending and scaling operations (rotation of ezip format data is not supported).
  For EPIC output mode, users can use HAL EPIC interface to configure EPIC to read ezip format data. HAL EPIC will configure EZIP to enter EPIC output mode, users don't need to call HAL EZIP interface.
- Can specify partial region of graphics for decompression output
- Supports interrupt and polling modes
- AHB mode supports LZ4 and GZIP4 decompression
- Except SF32LB55X, other chip series support decompression of EZIPA animation data. The ezipa_dec module in middleware provides higher-level software support based on EPIC and EZIP drivers. Usage reference ezipa_dec module header file

For detailed API documentation, refer to [EZIP](#hal-ezip).

## Using HAL EZIP

First call `HAL_EZIP_Init` to initialize HAL_EZIP. Need to specify the EZIP hardware module used (i.e., EZIP instance) in `EZIP_HandleTypeDef`. The chip has only one EZIP instance, represented by `hwp_ezip`.
Initialization only needs to be done once, then `HAL_EZIP_Decode` and `HAL_EZIP_Decode_IT` can be called to decompress data.

```{note}
Source and destination addresses need to be 4-byte aligned
```

For example,
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
## API Reference
[](#hal-ezip)
