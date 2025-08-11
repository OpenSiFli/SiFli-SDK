# Simple FFT Example

源码路径: example/hal/fft/simple

## 支持的开发板
此示例可在以下开发板上运行：
- sf32lb56-lcd_a128r12n1

## 示例概述

本示例演示了以下基础FFT操作：
- 实数FFT（Real FFT - 将实数序列转换为频域
- 复数FFT（Complex FFT - 将复数序列转换为频域  
- 逆FFT（Inverse FFT - 将频域数据转换回时域

## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

### 硬件连接
本例程不需要特殊的硬件连接，只需将开发板连接到电脑。

### 预期结果

预期情况下，您将看到以下输出：

```log
=========================================
        FFT HAL Module Example          
=========================================
This example demonstrates:
- Real FFT operation
- Complex FFT operation
- Inverse FFT operation
=========================================

FFT Example: Using FFT1 (HCPU)
FFT initialization successful
=== Real FFT Test ===
Real FFT completed successfully
[0] = 0x0B73
[1] = 0x0000
[2] = 0x0349
[3] = 0x0641
[4] = 0xFE9D
[5] = 0xFBF7
[6] = 0xF7E2
[7] = 0x067A
[8] = 0xF961
...

=== Complex FFT Test ===
Complex FFT completed successfully
[0] = 0x0599
[1] = 0x077B
[2] = 0x0BA7
[3] = 0xFF15
[4] = 0x04C7
[5] = 0xF4D7
[6] = 0xFC33
[7] = 0xF75A
[8] = 0x000A
...

=== Inverse FFT Test ===
Inverse FFT completed successfully
[0] = 0x0599
[1] = 0x077B
[2] = 0x0BA7
[3] = 0xFF15
[4] = 0x04C7
[5] = 0xF4D7
[6] = 0xFC33
[7] = 0xF75A
[8] = 0x000A
...

=========================================
          FFT Example Complete          
=========================================
```

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档

- [SiFli-SDK 官方文档](https://docs.sifli.com/)

## 更新记录
|版本  |日期    |发布说明 |
|:---  |:---    |:---    |
|1.0.0 |6/2025 |初始版本 |
|      |        |        |