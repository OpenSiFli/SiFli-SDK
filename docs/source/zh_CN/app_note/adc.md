# ADC使用指南

## 1. ADC 介绍 
目前ADC 有8个通道可以使用，单元测量范围 0 ~ 1.0 V， 可以单独通道测试，也可以多通道循环测试；可以测试一组数据后结束，也可以一直测试输出数据。

## 2. ADC 配置 
ADC的配置包括PINMUX的配置和ADC时钟相关的配置，时钟等参数的配置在adc_config.h中 定义，用户可以根据需要进行修改。<br/>
PINMUX的设置目前A0/Z0的方式稍有不同， 在Z0上，如果PIN当作ADC使用，可以在设置PIN时直接选择ADC功能（比如HAL_PIN_Set(PAD_PB_04, ADC_PIN, PIN_PULLUP, 0)这种方式）；<br/>
而在A0上，每个可配置成GPADC的管脚，都对应设置了不同的FUNCTION: GPADC_CH0/GPADC_CH1 ...... <br/>
在使用时，要按照各自PIN来设置，比如 PAD_PB8 对应的GPADC 为通道0，那设置应该为： HAL_PIN_Set(PAD_PB08, GPADC_CH0, PIN_NOPULL, 0)  <br/>
而PAD_PB13 对应的GPADC为通道3， 则应该设置为： HAL_PIN_Set(PAD_PB13, GPADC_CH3, PIN_NOPULL, 0) <br/>
也可以直接设置PIN为模拟功能，不再具体关心每个PIN对应的具体模拟功能，如HAL_PIN_Set_Analog(PAD_PB08, 0) / HAL_PIN_Set_Analog(PAD_PB13, 0)<br/> 
在A0上各个可用做GPADC的PAD 及功能对应表：<br/>
|PAD 号             | 通道号       |  描述
|-----------------|----------------|--------
|PAD_PB08           | GPADC_CH0     | 挂载到LCPU |
|PAD_PB10           | GPADC_CH1     | 挂载到LCPU |
|PAD_PB12           | GPADC_CH2     | 挂载到LCPU |
|PAD_PB13           | GPADC_CH3     | 挂载到LCPU |
|PAD_PB16           | GPADC_CH4     | 挂载到LCPU |
|PAD_PB17           | GPADC_CH5     | 挂载到LCPU |
|PAD_PB18           | GPADC_CH6     | 挂载到LCPU |
|PAD_PB19           | GPADC_CH7     | 挂载到LCPU |
 
## 3.ADC 使用

在我们系统中，默认将ADC注册成了电池电压设备， 可以使用设备方式进行操作读写， 默认设备名为 “bat1"：<br/>

```c
uint32_t chnl = 1;
uint32_t value;

// find device
rt_device_t dev = rt_device_find(argv[2]);
if (dev)
{
    // open device
    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
	// enable adc
    rt_device_control(dev, RT_ADC_CMD_ENABLE, (void *)chnl);
	// read adc value
	rt_device_read(dev, chnl, &value, 1);
}
......
```

## 4.电压值的计算
关于通过ADC获取电压，可以通过接口（HAL_ADC_GetValue， 如果用rt_device 则用read接口）获取ADC值. <br/>
寄存器数值每增加1，电压增加1mv左右(每颗芯片不会完全相同，需要每个芯片都做出厂校准). <br/>
ADC值和电压之间是线性关系，而如何校验获取精准的偏移和斜率，可以参考函数（sifli_adc_calibration）。<br/>
其原理和方法为：通过两点确定一条直线，后面通过新的一个坐标值（寄存器值）在直线上算出另外一个坐标----即电压值<br/>
具体操作为：<br/>
ATE几台将已知两个准确的电压（比如0.3， 0.8， 不能为0或者最大电压值）， 接入ADC后，读取ADC值，将两个电压值和寄存器值保存，后续应用通过两点确定一条直线的方式，从而可以获取电压的偏移（0V时对应的电压值，不过在0附近时并不精确）和斜率（寄存器值每增加1对应的电压增加值），可以将这组数值保存以便后续电压测试使用。<br/>

寄存器值转换电压的参考代码如下：<br/>

```c

// default value, they should be over write by calibrate
// it should be register value offset vs 0 v value.
static uint32_t adc_vol_offset = 200;
// mv per bit, if accuracy not enough, change to 0.1 mv or 0.01 mv later
static uint32_t adc_vol_ratio = 3930; //6;

/**
* @brief  Get voltage by register value.
* @param[in]  value register value.
* @retval voltage in mv.

int sifli_adc_get_mv(uint32_t value)
{
    uint32_t offset, ratio;
    // get offset
    offset = adc_vol_offset;
    // get ratio
    ratio = adc_vol_ratio;

    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

/**
* @brief  Get voltage offset and ratio.
* @param[in]  value1 register value 1.
* @param[in]  value2 register value 2.
* @param[in]  vol1  voltage 1 in mv.
* @param[in]  vol2  voltage 2 in mv.
* @param[out]  offset, reg value offset vs 0v value.
* @param[out]  ratio, voltage (mv)per bit, ratio with 100 based or 0.01 mv, 1 base for 1 mv
* @retval offset.
*/
int sifli_adc_calibration(uint32_t value1, uint32_t value2,
                          uint32_t vol1, uint32_t vol2, uint32_t *offset, uint32_t *ratio)
{
    uint32_t gap1, gap2;

    if (offset == NULL || ratio == NULL)
        return 0;

    gap1 = value1 > value2 ? value1 - value2 : value2 - value1; // register value gap
    gap2 = vol1 > vol2 ? vol1 - vol2 : vol2 - vol1; // voltage gap -- mv

    if (gap1 != 0)
    {
        *ratio = gap2 * ADC_RATIO_ACCURATE / gap1; // gap2 * 10 for 0.1mv, gap2 * 100 for 0.01mv
        adc_vol_ratio = *ratio;
    }
    *offset = value1 - vol1 * ADC_RATIO_ACCURATE / adc_vol_ratio;
    adc_vol_offset = *offset;

    return adc_vol_offset;
}

```