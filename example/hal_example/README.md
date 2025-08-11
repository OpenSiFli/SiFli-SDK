# HAL综合示例

源码路径: example/hal_example

## 概述
HAL综合示例展示了如何使用SiFli-SDK的HAL（硬件抽象层）功能。该示例提供了一系列测试命令，允许用户通过串口交互方式运行不同模块的测试用例。

### 硬件需求
无特殊硬件需求，使用标准开发板即可运行。

## 工程编译及下载：
工程可以通过指定board来编译可运行的对应板子的程序
- 比如想编译可以在HDK 525上运行的工程，执行scons --board=eh-lb525即可生成镜像文件
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的525工程，可以执行.`build_eh-lb525\download.bat`来通过jlink下载
- 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载

## 示例使用
在串口使用以下命令进行交互:
1. `help` - 显示所有支持的命令
2. `utest_list` - 显示所有支持的测试用例
3. `utest_run example_xxx` - 运行指定模块的测试用例

## 异常诊断
暂无特定异常诊断信息。如有问题，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)