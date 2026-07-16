# ADC Example
Source code path: example\hal\adc\multichannel
## Supported Platforms
The example can run on the following development boards
- sf32lb52-lcd_n16r8

## Overview
* Use HAL unified calibration interface for single-channel ADC or DMA six-channel sampling demonstration (single-channel mode triggers one conversion by default)

## Example Usage
### Compilation and Programming
The demonstration code defaults to single-channel ADC sampling (trigger one conversion and print after startup). If you want to run the DMA six-channel demonstration, please enable the macro in `main.c` under the project path:  
```c
#define BSP_GPADC_USING_DMA 1
```
Switch to the example project directory and run the scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j8

Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:

> build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

>Uart Download

>please input the serial port num:5

### Hardware Connection
* PA28 is the fixed output IO for ADC1 Channel 0 (DMA six-channel demonstration uses PA28-PA33, not marked here)  
![alt text](assets/SF32LB52_DevKit_ADC.png)
#### Example Output Results Display:
* Single-channel ADC mode log output (PA28 input 2.5V):
```
    SFBL
    Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000080
     \ | /
    - SiFli Corporation
     / | \     build on Nov  6 2024, 2.2.0 build 00000000
     2020 - 2022 Copyright by SiFli team
    mount /dev sucess
    [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [I/drv.rtc] RTC use LXT RTC_CR=00000001
    [I/drv.rtc] Init RTC, wake = 0
    [I/drv.audprc] init 00 ADC_PATH_CFG0 0x606
    [I/drv.audprc] HAL_AUDPRC_Init res 0
    [I/drv.audcodec] HAL_AUDCODEC_Init res 0
    [32m][I/TOUCH] Regist touch screen driver, probe=1203c2d5 [0m]
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 2050
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 2050
    Start adc demo!
    ADC Get calibration res 0
    ADC reg value 3192 voltage 2519.12 mv
    adc demo end!
```
The log prints `ADC reg value` as raw register value, `voltage` as voltage (mV) converted by calibration context.
* DMA six-channel mode log output (PA28 input 2.5V, PA29 connected to GND, other IOs based on board internal voltage):
```
   SFBL
   Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000080
    \ | /
   - SiFli Corporation
    / | \     build on Nov  6 2024, 2.2.0 build 00000000
    2020 - 2022 Copyright by SiFli team
   mount /dev sucess
   [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [I/drv.rtc] RTC use LXT RTC_CR=00000001
   [I/drv.rtc] Init RTC, wake = 0
   [I/drv.audprc] init 00 ADC_PATH_CFG0 0x606
   [I/drv.audprc] HAL_AUDPRC_Init res 0
   [I/drv.audcodec] HAL_AUDCODEC_Init res 0
   [32m][I/TOUCH] Regist touch screen driver, probe=1203c685 [0m]
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2050
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2050
   Start adc demo!
   ADC Get calibration res 0
   ADC reg value[0] 3199 voltage 2526.56 mv
   ADC reg value[1] 831 voltage 10.98 mv
   ADC reg value[2] 3950 voltage 3324.36 mv
   ADC reg value[3] 1757 voltage 994.69 mv
   ADC reg value[4] 1082 voltage 277.62 mv
   ADC reg value[5] 3951 voltage 3325.43 mv
   Loop 0 done ===
   adc demo end!
```
The log prints `ADC reg value[i]` as corresponding channel raw register value, `voltage` as voltage (mV) converted by calibration context.

#### ADC Configuration Flow

* Set the corresponding ADC IO port (single-channel ADC mode)
```c
    /* 52 chip default channel 0 is PA28, set PA28 to analog input mode, disable internal pull-up/pull-down */
    HAL_PIN_Set_Analog(PAD_PA28, 1);
```
* Set the corresponding ADC IO ports (DMA six-channel mode)
```c
    /* 52 chip demonstration uses channels 0-5, all configured as analog input mode, disable internal pull-up/pull-down */
    HAL_PIN_Set_Analog(PAD_PA28, 1);  /* channel 0 */
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    HAL_PIN_Set_Analog(PAD_PA33, 1);  /* channel 5 */
```
**Note**: 
1. ADC input ports are fixed IO ports, as shown in the following diagram:<br>52 chip ADC CH1-7 distribution, corresponding to software configured Channel0-6. The last channel CH8 (Channel 7) is internally connected to battery Vbat detection and not mapped to external IO<br>
![alt text](assets/ADC_MAP.png)
2. `HAL_PIN_Set` `HAL_PIN_Set_Analog` last parameter is for hcpu/lcpu selection, 1: select hcpu, 0: select lcpu<br>
* Enable the corresponding ADC clock source (enabled by default in code, not mandatory here)
```
    /* 2, open adc clock source  */
    HAL_RCC_EnableModule(RCC_MOD_GPADC);
```
* ADC initialization settings
1. ADC channel modification<br>
```c
   #define ADC_DEV_CHANNEL     0           /* Single-channel ADC mode ADC channel */
    /* 52 chip default channel 0 is PA28, set PA28 to analog input mode, disable internal pull-up/pull-down */
    HAL_PIN_Set_Analog(PAD_PA28, 1);
```
```c
    /* Select which channels to sample with DMA  */
    /* set pinmux of channel 0 to analog input */
    HAL_PIN_Set_Analog(PAD_PA28, 1);  /* channel 0 */
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    HAL_PIN_Set_Analog(PAD_PA33, 1);  /* channel 5 */
    /* ADC has 8 slots total, can be used to simultaneously sample 8-channel ADC. Channel 0 corresponds to slot0. To simultaneously sample which channels, enable the corresponding slots */
    ADC_ChanConf.Channel = 0; /* channel 0 */
    ADC_ChanConf.pchnl_sel = 0; /* channel 0 */
    ADC_ChanConf.slot_en = 1; /* Enable slot 0 to sample channel 0 */
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);
```
* ADC Calibration
1. To improve ADC accuracy, SiFli series chips undergo ADC calibration at the factory (calibration parameters are written to the chip's internal OTP area). Different series have different calibration methods.  
To ensure ADC accuracy, call the calibration function once per power-on. Use unified context structure to manage calibration parameters:

```c
/* Define calibration context */
static HAL_ADC_CalibContextTypeDef g_adc_calib_ctx;

/* Load calibration parameters */
static int utest_adc_calib(void)
{
    HAL_ADC_CalibFactoryInfoTypeDef factory_info;

    /* Load calibration context: init defaults + read factory config */
    if (HAL_ADC_CalibLoad(NULL,
                          &g_adc_calib_ctx,
                          HAL_ADC_CALIB_SOURCE_BSP,
                          HAL_ADC_CALIB_F_INIT) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    /* Get factory config info (for log output) */
    if (HAL_ADC_CalibGetFactoryInfo(HAL_ADC_CALIB_SOURCE_BSP, &factory_info) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}
```

2. After ADC sampling obtains the raw register value, call `HAL_ADC_RegToVoltageFloat` to calculate the final voltage value using calibration context:

```c
/* Read register value */
dst = HAL_ADC_GetValue(&hadc, lslot);

/* Convert to voltage (mV) */
float voltage = HAL_ADC_RegToVoltageFloat((float)dst, &g_adc_calib_ctx);
```

3. For 52 series chips, Channel 7 is internally connected to Vbat through voltage divider resistors. To get Vbat value, multiply by calibration factor:

```c
/* VBAT channel special handling */
if (channel == 7)
{
    /* Actual battery voltage = ADC voltage × vbat_factor */
    voltage *= g_adc_calib_ctx.vbat_factor;
}
```

4. Apply calibration parameters to hardware (such as LDO Vref setting):

```c
/* Apply calibration parameters to hardware */
HAL_ADC_CalibApply(&hadc, &g_adc_calib_ctx);
```

## Calibration API Reference

### Data Structures

**HAL_ADC_CalibContextTypeDef** - Calibration context
```c
typedef struct 
{ 
    float    offset;              /* 0V offset (register value) */
    float    ratio;               /* Gain/slope (mV/bit × 1000) */
    uint32_t threshold_reg;       /* Overvoltage threshold (register value) */
    uint8_t  range_mode;          /* Range mode: 0=big range(X3), 1=small range(X1) */
    float    vbat_factor;         /* VBAT divider correction factor */
    uint8_t  ldovref_sel;         /* LDO reference voltage selection */
    uint8_t  flags;               /* Status flags */
} HAL_ADC_CalibContextTypeDef;
```

**HAL_ADC_CalibFactoryInfoTypeDef** - Factory config info
```c
typedef struct 
{ 
    uint32_t voltage1_mv;         /* Calibration point 1 voltage (mV) */
    uint32_t reg_value1;          /* Calibration point 1 register value */
    uint32_t voltage2_mv;         /* Calibration point 2 voltage (mV) */
    uint32_t reg_value2;          /* Calibration point 2 register value */
    uint16_t vbat_reg;            /* VBAT reference register value (SF32LB52X only) */
    uint16_t vbat_mv;             /* VBAT reference voltage (SF32LB52X only) */
    uint8_t  ldovref_flag;        /* LDO Vref calibrated flag (SF32LB52X only) */
    uint8_t  ldovref_sel;         /* LDO Vref selection value (SF32LB52X only) */
    uint8_t  range_mode;          /* Range mode */
} HAL_ADC_CalibFactoryInfoTypeDef;
```

### API Functions

| Function | Description |
|------|------|
| `HAL_ADC_CalibLoad()` | Load calibration context |
| `HAL_ADC_CalibInit()` | Initialize context with defaults |
| `HAL_ADC_CalibApply()` | Apply calibration parameters to hardware |
| `HAL_ADC_CalibGetFactoryInfo()` | Get factory config info |
| `HAL_ADC_RegToVoltageFloat()` | Register value to voltage (float) |
| `HAL_ADC_RegToVoltage()` | Register value to voltage (integer) |
| `HAL_ADC_CalibSetCustom()` | Set custom calibration parameters |

## Exception Diagnosis
* ADC sampled voltage value is incorrect
1. Check if ADC hardware is connected correctly. ADC sampling channels are fixed IO ports and cannot be arbitrarily assigned. For which IO corresponds to CH0-7, refer to the chip manual.  
2. ADC input voltage range is 0V - reference voltage (52 defaults to 3v3), cannot exceed the input range  
3. Use debugging tools like Ozone or LightWork. After starting ADC sampling, connect online and check the corresponding register configuration status against the chip manual.
* ADC accuracy is insufficient
1. Check if ADC calibration parameters are obtained and used
2. Check if voltage divider resistor accuracy meets requirements
3. Check if ADC reference voltage is stable and has excessive ripple (refer to ADC voltage reference chip manual for specifics) 

  
## Reference Documents
* EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx
* DS0052-SF32LB52x-Chip Technical Specification V0p3.pdf
## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
|0.0.2 |03/2025 |Updated to new HAL calibration API |
