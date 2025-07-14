# 上手指南

[SDK]: https://github.com/OpenSiFli/SiFli-SDK
[Trace]: https://webfile.lovemcu.cn/file/sdk/SifliTrace_v2.2.6.7z
[52DevKit]: https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-LCD.html
[52Module]: https://wiki.sifli.com/silicon/%E6%A8%A1%E7%BB%84%E5%9E%8B%E5%8F%B7%E6%8C%87%E5%8D%97.html

## 准备工作

### 硬件

- 一款开发板，例如[SF32LB52-DevKit-LCD 开发板][52DevKit]
- 一条USB Type-C数据线，连接开发板与电脑，注意不能是只有充电功能的Type-C线，插入开发板的USB-to-UART接口（注意不要插入专用USB功能接口）
- 电脑（Windows、Linux 或 macOS）

### 软件

如需在 SF32 上使用 SiFli-SDK，请安装以下软件：
- 设置 工具链，用于编译 SF32 代码；
- 编译构建工具 —— 在SDK2.x中使用Scons作为构建工具，用于编译SF32应用程序；
- 获取 SiFli-SDK 软件开发框架。该框架已经基本包含 SF32 使用的 API（软件库和源代码）和运行 **工具链** 的脚本；

### 安装

为安装所需软件，SiFli 提供了以下方法，可根据需要选择其中之一。

#### 手动安装

参考手动安装：[手动安装](install/index.md)
