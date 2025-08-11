# LVGL 扫雷游戏

源码路径: example/multimedia/lvgl/games/minesweeper

这是一个基于LVGL实现的扫雷游戏示例，所有操作均通过LVGL图形库完成，无需对接底层硬件接口。该示例展示了如何使用LVGL创建交互式游戏应用，包括图形界面渲染、用户输入处理和游戏逻辑实现。
## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

## 支持的开发板

此示例可在以下开发板上运行：
- eh-lb563
- SF32LB52x系列
- SF32LB56x系列

本实例演示扫雷小游戏，游戏所有操作均基于LVGL实现，无需对接任何底层接口

## 工程编译及下载：
板子工程在project目录下可以通过指定board来编译适应相对board的工程，
- 比如想编译可以在HDK 563上运行的工程，执行scons --board=eh-lb563即可生成工程
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的563工程，可以执行.\build_eh-lb563\download.bat来通过jlink下载
- 特别说明下，对于SF32LB52x/SF32LB56x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
模拟器工程在simulator目录下，
- 使用 scons 进行编译，simulator/msvc_setup.bat文件需要相应修改，和本机MSVC配置对应
- 也可以使用 scons --target=vs2017 生成 MSVC工程 project.vcxproj, 使用Visual Studio 进行编译。

```{note}
如果不是使用VS2017, 例如 VS2022, 加载工程的时候，会提示升级MSVC SDK, 升级后就可以使用了。
```

## 异常诊断

如有任何技术疑问，请在GitHub上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
            
      
