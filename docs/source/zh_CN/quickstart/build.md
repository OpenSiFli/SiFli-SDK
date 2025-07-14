# 编译下载

## 编译工程

先按照安装环境的要求安装好依赖包，并配置好环境变量。

执行命令`cd example\get-started\hello_world\rtt\project`进入RT-Thread版本的hello_world示例的工程目录，运行命令`scons --board=sf32lb52-lcd_n16r8 -j8`编译可运行在SF32LB52x-DevKit-LCD开发板的hello_world程序，其中`sf32lb52-lcd_n16r8`为开发板的名称，可用的板子名称见[](/supported_boards/index.md)，可将sf32lb52-lcd_n16r8替换为其他开发板的名字。编译生成的文件存放在`build_sf32lb52-lcd_n16r8_hcpu`目录下。

```{note}
编译命令格式：`scons --board=<board_name> -jN`，其中`<board_name>`为板子名称，可用的板子名称见[](/supported_boards/index.md)，如果board_name未指定内核，则默认使用HCPU的配置编译，<board_name>会扩展成<board_name>_hcpu，`-jN`为多线程编译参数，`N`为线程数，例如上面的例子中`-j8`表示开启8个线程编译

编译生成的文件存放在`build_<board_name>`目录下，包含了需要下载的二进制文件和下载脚本，其中`<board_name>`为以内核为后缀的板子名称，例如`build_sf32lb52-lcd_n16r8_hcpu`
```

## 配置选项

对不同的工程来说，可能有不同的定制化的配置选项。我们使用menuconfig工具来配置工程。打开menuconfig的方法如下：

在工程目录下运行命令`scons --board=<board_name> --menuconfig`，会弹出一个配置界面，可以根据需要修改配置选项。修改完成后，按下`ESC`键两次，选择保存配置并退出。

```{note}
`--board`是必选参数，指定要编译的板子名称，可用的板子名称见[](/supported_boards/index.md)，如果未指定将无法正常配置。
```

## 下载程序

保持开发板与电脑的USB连接，运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`下载程序到开发板，当提示`please input serial port number`，输入开发板实际串口号，例如COM19就输入19，输入完成后敲回车即开始下载程序（建议在敲回车开始下载前重启开发板，并在重启开发板后的2秒内敲回车开始下载），完成后按提示按任意键回到命令行提示符。
```{note}
Linux和macOS用户使用的脚本文件为`build_sf32lb52-lcd_n16r8_hcpu/uart_download.sh`，使用方法与Windows下的脚本相同。需要注意的是macOS用户的串口号请使用`/dev/cu.`开头的设备名，例如`/dev/cu.usbserial-12345678`，而不是`/dev/tty.`开头的设备名。
```

## 运行程序

下载完成后，会自动执行软件复位，或者也可以按下开发板上的RESET键，程序会自动运行，串口助手会打印出hello world的提示信息。
