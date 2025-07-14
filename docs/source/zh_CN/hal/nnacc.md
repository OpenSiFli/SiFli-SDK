# NNACC

SiFli实现了一个 NNACC（神经网络加速器）来加速IOT设备上的人工智能。 它可以用于语音/视觉识别应用程序和其他使用卷积神经网络（CNN）的人工智能应用程序。
支持：<br>
 - 支持 CMSIS-NN， 比 CMSIS-NN 中的 arm DSP 实现快 8 倍。 <br>
 - 可以实现低MIPS需求的关键词识别。<br>

## 驱动配置
固件可以在启用 CMSIS-NN 的情况下使用 NNACC。 配置是使用menuconfig 工具完成的，通常保存在一个C 头文件中。 默认情况下，配置保存为 rtconfig.h。
以下显示在一个项目标题文件中定义的标志，该项目将重新计算NNACC支持。

```{note}
`SF32LB55X/56X`的HCPU有一个NNACC, NNACC1， SF32LB58X HCPU/LCPU各有一个NNACC, 分别为NNACC1/NNACC2.
```

配置完成后，用户需要在所有需要访问驱动程序的源代码中包含头文件。

```c
#define BSP_USING_CMSIS_NN
#define BSP_USING_NN_ACC
```
Once configuration is done, user need to include the header file in all source code that need to access the driver.

## 使用 NNACC
SiFli SDK 在 CMSIS-NN 中实现 CNN 功能。 用户仍然可以在他们的 AI 应用程序中使用 CMSIS-NN 接口。 Accelerator API 的详细介绍请参考 CNN 加速器。
以下 CMSIS-NN 函数具有来自 SiFli SDK 的加速：
- arm_convolve_1x1_HWC_q7_fast_nonsquare
- arm_convolve_HWC_q7_basic_nonsquare
- arm_depthwise_separable_conv_HWC_q7_nonsquare


## API参考
[](../api/hal/nnacc.md)
