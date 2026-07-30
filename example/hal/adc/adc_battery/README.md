# ADC_battery示例
源码路径：example/hal/adc/adc_battery
## 支持的平台
例程可以运行在以下开发板
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列
+ sf32lb57 系列 

## 概述
* 操作Hal函数单路ADC读取电池电压

## 例程的使用
### 编译和烧录
演示代码默认为单路ADC采样演示（启动后触发一次转换并打印）

切换到例程project目录，运行scons命令执行编译：

```
scons --board=sf32lb52-lcd_n16r8 -j8
```

执行烧写命令
```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
```

按提示选择端口即可进行下载：

```none
please input the serial port num:5
```

#### 例程输出结果展示:
每秒循环打印读取的电压值

* 接入电池前读取的电压log与接入电池后读取的电压log对比

![alt text](assets/before_after.png)

* 58_lcd 以及 56_lcd 的测量引脚点位为：

58的测量点位：

![58](assets/58.png)

56的测量点位：

![56](assets/56.png)

#### ADC配置流程

* 设置电池Vbat接口对应的通道，根据自己的板子平台修改，此处以52为例为通道7

![alt text](assets/1.png)

* 在menuconfig中打开adc device

```
sdk.py menuconfig --board=sf32lb52-lcd_n16r8
```

![alt text](assets/2.png)

* 将需要测量的ADC通道引脚设置为模拟输入模式(非52平台通道7)

![alt text](assets/pin.png)

**注意**: 
* 打开对应的ADC的时钟源（默认代码开启，此处不是必须）
```c
/* 2, open adc clock source  */
HAL_RCC_EnableModule(RCC_MOD_GPADC);
```

* ADC校准
1. 为了提高ADC精度，SiFli系列芯片出厂都进行了ADC校准（校准参数写入了芯片内的OTP区域），不同系列校准方法会有区别，  
为了确保ADC精确度，每次上电，需要调用一次校准函数。使用统一的上下文结构体管理校准参数：

```c
/* 定义校准上下文 */
static HAL_ADC_CalibContextTypeDef g_adc_calib_ctx;

/* 加载校准参数 */
static int utest_adc_calib(void)
{
    HAL_ADC_CalibFactoryInfoTypeDef factory_info;

    /* 加载校准上下文：初始化默认值 + 读取工厂配置 */
    if (HAL_ADC_CalibLoad(NULL,
                          &g_adc_calib_ctx,
                          HAL_ADC_CALIB_SOURCE_BSP,
                          HAL_ADC_CALIB_F_INIT) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    /* 获取工厂配置信息（用于日志输出） */
    if (HAL_ADC_CalibGetFactoryInfo(HAL_ADC_CALIB_SOURCE_BSP, &factory_info) != HAL_OK)
    {
        rt_kprintf("Get ADC configure fail\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}
```

2. ADC采用得到寄存器原始值后，调用函数`HAL_ADC_RegToVoltageFloat`根据校准上下文算出最终的电压值：

```c
/* 读取寄存器值 */
dst = HAL_ADC_GetValue(&hadc, lslot);

/* 转换为电压值 (mV) */
float voltage = HAL_ADC_RegToVoltageFloat((float)dst, &g_adc_calib_ctx);
```

3. 52系列芯片 Channel 7 内部通过分压电阻连接到Vbat，要得到Vbat值需要乘以校准系数：

```c
/* VBAT通道特殊处理 */
if (channel == 7)
{
    /* 实际电池电压 = ADC电压 × vbat_factor */
    voltage *= g_adc_calib_ctx.vbat_factor;
}
```

4. 应用校准参数到硬件（如LDO Vref设置）：

```c
/* 应用校准参数到硬件 */
HAL_ADC_CalibApply(&hadc, &g_adc_calib_ctx);
```

## 校准API参考

### 数据结构

**HAL_ADC_CalibContextTypeDef** - 校准上下文
```c
typedef struct 
{ 
    float    offset;              /* 0V偏移值（寄存器值） */
    float    ratio;               /* 增益/斜率（mV/bit × 1000） */
    uint32_t threshold_reg;       /* 过压阈值（寄存器值） */
    uint8_t  range_mode;          /* 范围模式: 0=大范围(X3), 1=小范围(X1) */
    float    vbat_factor;         /* VBAT分压校正系数 */
    uint8_t  ldovref_sel;         /* LDO参考电压选择值 */
    uint8_t  flags;               /* 状态标志位 */
} HAL_ADC_CalibContextTypeDef;
```

**HAL_ADC_CalibFactoryInfoTypeDef** - 工厂配置信息
```c
typedef struct 
{ 
    uint32_t voltage1_mv;         /* 校准点1电压 (mV) */
    uint32_t reg_value1;          /* 校准点1寄存器值 */
    uint32_t voltage2_mv;         /* 校准点2电压 (mV) */
    uint32_t reg_value2;          /* 校准点2寄存器值 */
    uint16_t vbat_reg;            /* VBAT参考寄存器值 (仅SF32LB52X) */
    uint16_t vbat_mv;             /* VBAT参考电压 (仅SF32LB52X) */
    uint8_t  ldovref_flag;        /* LDO Vref已校准标志 (仅SF32LB52X) */
    uint8_t  ldovref_sel;         /* LDO Vref选择值 (仅SF32LB52X) */
    uint8_t  range_mode;          /* 范围模式 */
} HAL_ADC_CalibFactoryInfoTypeDef;
```

### API函数

| 函数 | 说明 |
|------|------|
| `HAL_ADC_CalibLoad()` | 加载校准上下文 |
| `HAL_ADC_CalibInit()` | 初始化上下文为默认值 |
| `HAL_ADC_CalibApply()` | 应用校准参数到硬件 |
| `HAL_ADC_CalibGetFactoryInfo()` | 获取工厂配置信息 |
| `HAL_ADC_RegToVoltageFloat()` | 寄存器值转电压（浮点） |
| `HAL_ADC_RegToVoltage()` | 寄存器值转电压（整型） |
| `HAL_ADC_CalibSetCustom()` | 自定义校准参数 |

## 异常诊断
* ADC采样的电压值不对
1. 检查ADC硬件是否连接正确，ADC采样的通道为固定IO口，不能任意指定，具体CH0-7为哪个IO，参照芯片手册  
2. ADC输入电压范围为0V - 参考电压（52默认为3v3），不能超出输入范围  
3. 采用Ozone或者LightWork等调试工具，在启动ADC采样后，在线连接，对照芯片手册，查看对应的寄存器配置状态
* ADC精确度不够
1. ADC校准参数是否获取和使用
2. 分压电阻的精度是否达到要求
3. ADC参考电压是否稳定和是否有过大纹波(具体参考ADC电压参考芯片手册) 

  
## 参考文档
* EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx
* DS0052-SF32LB52x-芯片技术规格书 V0p3.pdf
## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
|0.0.2 |03/2025 |更新为新版HAL校准API |
|0.0.3 |05/2026 |增加对56，58的说明 |
| | | |
