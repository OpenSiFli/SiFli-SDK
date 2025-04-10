# 编译下载

## 编译工程

先按照安装环境的要求安装好依赖包，并配置好环境变量。

执行命令`cd example\get-started\hello_world\rtt\project`进入RT-Thread版本的hello_world示例的工程目录，运行命令`scons --board=em-lb525 -j8`编译可运行在SF32LB52x-DevKit-LCD开发板的hello_world程序，其中`em-lb525`为开发板的名称，可用的板子名称见[](/supported_boards/index.md)，可将em-lb525替换为其他开发板的名字。编译生成的文件存放在`build_em-lb525_hcpu`目录下。

```{note}
编译命令格式：`scons --board=<board_name> -jN`，其中`<board_name>`为板子名称，可用的板子名称见[](/supported_boards/index.md)，如果board_name未指定内核，则默认使用HCPU的配置编译，<board_name>会扩展成<board_name>_hcpu，`-jN`为多线程编译参数，`N`为线程数，例如上面的例子中`-j8`表示开启8个线程编译

编译生成的文件存放在`build_<board_name>`目录下，包含了需要下载的二进制文件和下载脚本，其中`<board_name>`为以内核为后缀的板子名称，例如`em-lb525_build`
```

## 下载程序

保持开发板与电脑的USB连接，运行`build_em-lb525_hcpu\uart_download.bat`下载程序到开发板，当提示`please input serial port number`，输入开发板实际，例如COM19就输入19，输入完成后敲回车即开始下载程序，完成后按提示按任意键回到命令行提示符。
```{note}
Linux和macOS用户建议直接使用`sftool`工具下载，使用方法可参考[sftool](https://wiki.sifli.com/tools/SFTool.html)。需要下载的文件在有bootloader.elf、ftab.elf、main.elf
```

## 运行程序

下载完成后，会自动执行软件复位，或者也可以按下开发板上的RESET键，程序会自动运行，串口助手会打印出hello world的提示信息。
