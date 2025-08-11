# SiFli-SDK

[English](README_EN.md) | 中文

## 上手指南
SiFli SDK是官方提供的基于RT-Thread定制开发的一套软件开发框架，使用它可以快速开发运行于思澈科技芯片平台的应用程序。

配套文档链接：
- https://wiki.sifli.com/
- https://docs.sifli.com/projects/sdk/latest/sf32lb52x/index.html

可参考文档进行快速上手开发

**注意**：由于SDK包含了子模块，克隆仓库时需要添加`--recursive`参数，如下：
```bash
git clone --recursive https://github.com/OpenSiFli/SiFli-SDK
```

软件框架如下图，

![sdk_arch_diagram](img/sdk_arch_diagram.png)

- HAL为硬件抽象层，提供不依赖于操作系统服务的驱动功能
- RT-Thread设备驱动（Device Driver）基于HAL实现，提供了更高层的封装，用户无需实现中断服务程序，更易于使用，HAL与RT-Thread设备驱动更详细的比较请参考[芯片外设驱动](https://docs.sifli.com/projects/sdk/v2.3/sf32lb55x/app_development/drivers.html)。
- 中间件（components）包括RT-Thread自带的软件组件（如finsh、ulog）、第三方组件（位于`external`目录下）以及自研组件（位于`middleware`目录下）。应用程序可以使用包括HAL在内的所有服务接口开发应用程序。

SDK的目录结构

```
+---customer                 // 板级支撑包
|   +---boards               // 板子配置文件
|   |
|   +---peripherals          // 板级外设驱动
|
|
+---drivers
|   +---cmsis                // 芯片寄存器头文件、启动文件、链接脚本
|   |   +---Include
|   |   +---sf32lb52x
|   |   |     
|   |   +---sf32lb55x
|   |   | 
|   |   +---sf32lb56x
|   |   |
|   |   \---sf32lb58x
|   |     
|   +---hal                  // HAL实现代码
|   |
|   \---Include              // HAL头文件
| 
+---example                  // 例程
|
+---external                 // 第三方组件
|
+---middleware               // 自研组件
|
+---rtos                     // 操作系统
|   +---freertos             // FreeRTOS
|   |
|   +---os_adaptor           // OS抽象层
|   |
|   |
|   \---rtthread             // RT-Thread
|       \---bsp
|           \---sifli
|               \---drivers  // RT-Thread设备驱动适配代码
|       
|       
|
\---tools                    // 工具
```

## 版本发布规则
版本号遵循[语义化版本命名规则](https://semver.org/)

版本号格式：主版本号.次版本号.修订号，版本号递增规则如下：
- 主版本号：不兼容的API修改，比如 v1.0.0, v2.0.0
- 次版本号：向前兼容的功能性新增和修改，比如 v2.3.0, v2.4.0
- 修订号：向前兼容的问题修正和功能性的小修改，比如 v2.3.1, v2.3.2

每次发布都会创建`vX.Y.Z`形式的标签。

### 发布周期
- 修订版本：按需发布，最快一周
- 次版本: 4~5个月
- 主版本：一年甚至更长


### 分支
仓库包含以下分支：
- `main`：开发分支，所有新功能、修订都会进入该分支，**不建议**作为项目开发使用
- `release/vX.Y`：次版本分支，包含了次版本的所有修订，所有版本均从该分支发布（即标签会打在该分支的节点上），**建议**项目开发使用最新发布版本或者次版本分支的最新节点
- 其他：临时或者特性开发分支，一般为内部开发用途，不建议使用

### 版本支持期限
|版本      | 发布日期       | 支持结束日期    | 维护中
|----------|---------------|----------------|------
|v2.3      | 2025年1月21日  |               | 是
|v2.4      | 2025年6月5日   |                | 是