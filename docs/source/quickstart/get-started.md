# 上手指南

[SDK]: https://github.com/OpenSiFli/SiFli-SDK
[Env]: https://webfile.lovemcu.cn/file/sdk/SiFli_Env.exe
[Trace]: https://webfile.lovemcu.cn/file/sdk/SifliTrace_v2.2.6.7z
[52DevKit]: https://wiki.sifli.com/board/index.html
[52Module]: https://wiki.sifli.com/silicon/%E6%A8%A1%E7%BB%84%E5%9E%8B%E5%8F%B7%E6%8C%87%E5%8D%97.html

## 准备工作

- 一款开发板，例如[SF32LB52x-DevKit-LCD开发板][52DevKit]
- 一条USB Type-C数据线，连接开发板与电脑，注意不能是只有充电功能的Type-C线，插入开发板的USB-to-UART接口（注意不要插入专用USB功能接口）
- 一台Windows电脑 (目前仅支持Windows作为软件开发环境)


## 安装工具链
首先，安装如下两个工具用于应用的开发
- SiFli Env：包含了应用配置、编译和烧写所需的所有工具，提供一个命令行开发环境
- SiFliTrace：串口日志查看工具

(install-sifli-env)=
### 安装SiFli Env
点击链接[SiFli Env][Env]，下载最新版本的SiFli Env安装包，下载完成后双击运行安装程序，按提示完成安装，下图中的路径填写本机已安装的Keil、JLink、Visual Studio等软件的路径，将鼠标移动到`i`图标上可以查看提示，如果未安装相应软件，可以不填写。
```{image} assets/sifli_env_ui.png
:alt: env install
:scale: 70%
```

安装完成后，在任意目录点击鼠标右键打开快捷菜单，将看到如图新增的`ConEmu Here`菜单
```{image} assets/contextmenu.png
:alt: contextmenu
:scale: 90%
```

点击`ConEmu Here`菜单启动Env命令行窗口，命令行所在的目录为点击右键菜单所在的目录，显示下图窗口即表示Env命令行已正常打开，后续的应用配置、编译和烧写都将在该命令行窗口下完成。
```{image} assets/env_window.png
:alt: env window
```

```{note}
SiFli Env命令行窗口类似Windows的命令行，可以执行常用的Windows命令。
```

### 安装SiFliTrace
点击链接[SiFliTrace][Trace]下载SiFliTrace，因为压缩包为7z格式，推荐使用7z解压，解压后双击目录中的`SifliTrace.exe`即可启动SiFliTrace。


## 获取SiFli SDK
点击链接[SiFli SDK][SDK]下载SiFli SDK，下载完成后解压到任意目录，例如解压到C盘work目录下，解压后的目录结构如下图，下文将该目录称为SDK根目录，该目录包含了SDK环境设置脚本`set_env.bat`
```{image} assets/sdk_folder.png
:alt: sdk folder
```

```{important}
建议使用SDK的版本号命名解压后的文件夹，避免使用中文，路径中不能包含空格，尽量缩短路径长度，若路径过长可能会导致编译报错
```

## 编译并运行第一个工程
下面以[SF32LB52x-DevKit-LCD开发板][52DevKit]（板载[SF32LB52x-MOD-N16R8][52Module]模组）为例，介绍如何编译和下载hello_world程序到开发板中运行，并使用SiFliTrace串口调试工具查看log。
1. 启动Env命令行

    在SDK根目录点击右键菜单`ConEmu Here`打开Env命令行窗口，Env命令行路径自动定位到对应的SDK根目录，后续的命令均在Env命令行窗口执行

    ```{image} assets/env_sdk.png
    :alt: env sdk
    ```

1. 设置编译环境
    
    ````{tab-set}

    ```{tab-item} GCC
    执行命令`set_env gcc`设置GCC编译环境，设置完成后提示`set_env DONE`
    ```

    ```{tab-item} Keil
    执行命令`set_env`设置Keil编译环境，设置完成后提示`set_env DONE`
    ```

    ````

    ```{note}
    需要在SDK根目录（包含文件`set_env.bat`的目录）执行`set_env`命令，每次打开新的Env窗口都需要执行一次`set_env`命令，如果要使用其他目录的SDK代码，不能在已有的命令行中执行`cd`切换目录，而是需要在对应的SDK根目录开启一个新的Env命令行，并执行`set_env.bat`设置环境
    ```
1. 编译
    
    执行命令`cd example\get-started\hello_world\rtt\project`进入RT-Thread版本的hello_world示例的工程目录，运行命令`scons --board=em-lb525 -j8`编译可运行在SF32LB52x-DevKit-LCD开发板的hello_world程序，其中`em-lb525`为开发板的名称，可用的板子名称见[](/supported_boards/index.md)，可将em-lb525替换为其他开发板的名字。编译生成的文件存放在`build_em-lb525_hcpu`目录下。下图列出了以上几个步骤所用的命令和运行结果
    ```{image} assets/build_hello_world.png
    :alt: build hello_world
    ```
    ```{note}
    编译命令格式：`scons --board=<board_name> -jN`，其中`<board_name>`为板子名称，可用的板子名称见[](/supported_boards/index.md)，如果board_name未指定内核，则默认使用HCPU的配置编译，<board_name>会扩展成<board_name>_hcpu，`-jN`为多线程编译参数，`N`为线程数，例如上面的例子中`-j8`表示开启8个线程编译

    编译生成的文件存放在`build_<board_name>`目录下，包含了需要下载的二进制文件和下载脚本，其中`<board_name>`为以内核为后缀的板子名称，例如`em-lb525_build`
    ```
1. 连接开发板与电脑，确认开发板所用的调试串口（COM口）

    开发板使用CH343串口-USB转换芯片，连接开发板USB-to-UART接口（注意不是专用USB口）与电脑的USB口（连接方法参考[SF32LB52x-DevKit-LCD开发板][52DevKit]），Windows会自动安装USB驱动，并枚举出一个串口，开发板中运行的程序会通过这个串口号输出日志，从电脑下载程序到开发板也使用该串口。
    
    打开设备管理器并查看端口列表中新增的COM口获取串口号，具体方法为：打开资源管理器，右键**此电脑**打开右键菜单，点击**管理**打开计算机管理窗口，点击左侧的**设备管理器**，中间的端口列表中列出了当前系统中所有可用的COM口，可以插拔一下USB线以确认新增的COM口，新增的COM口即是开发板的调试串口，例如下图中的COM19就是开发板的调试串口
    ```{image} assets/windows_manager.png
    :alt: windows manager
    ```
    ```{image} assets/device_manager.png
    :alt: device manager
    ```
    ```{note}
    显示的端口设备名称可能随操作系统版本不同而有所差异，只需关注COM口的编号即可
    ```
1. 下载程序到开发板

    {#download-program}    
    保持开发板与电脑的USB连接，运行`build_em-lb525_hcpu\uart_download.bat`下载程序到开发板，当提示`please input serial port number`，输入上面步骤查询到的串口号，例如COM19就输入19，输入完成后敲回车即开始下载程序，下载过程中会以百分比提示下载进度，完成后按提示按任意键回到命令行提示符，下面的日志为一次成功下载的日志，使用串口19下载，如果下载失败请看下文，

        user@DESKTOP-2AB1CDE c:\work\release_v2.2.0\example\hello_world\rtt\project
        > build_em-lb525\uart_download.bat

            Uart Download

        please input the serial port num:19
        com19
        19:47:03 cur version 2.8, driver_20240726
        19:47:03 uart com19 open success
        19:47:03 RAM_PATCH: ram_patch_52X.bin
        19:47:03 SIG_PUB: sig_pub.der
        19:47:03 uart com19 open success
        19:47:03 EnterDebugMode success: curbaund (1000000)
        19:47:03 WriteMemSingle success: 0xf000edf0 0xa05f0003
        19:47:03 ReadMemSingle success: 0xf000ee08 0x00000000
        19:47:03 WriteMemSingle success: 0xf000ee08 0x00010000
        19:47:03 WriteMemSingle success: 0xf000edfc 0x01000001
        19:47:05 WriteMemSingle error: 0xf000ed0c 0x05fa0004
        19:47:05 ReadMemSingle success: 0xf000edf0 0x02030003
        19:47:05 ReadMemSingle success: 0xf000edf0 0x00030003
        19:47:05 WriteMemSingle success: 0xf000edfc 0xa05f0003
        19:47:05 WriteMemSingle success: 0xf000edfc 0x01000000
        19:47:05 [R] PC: 0x000007d4  MSP: 0x20001020
        19:47:05 DownLoadFileRam start
        19:47:05 use driver: internal
        19:47:06 DownLoadFileRam over
        19:47:06 WriteMemSingle success: 0x5000202c 0xfffffc00
        19:47:06 WriteMemSingle success: 0x50002030 0x00000000
        19:47:06 [R] PC: 0x000007d4  MSP: 0x20001020
        19:47:06 [R] PC: 0x2005ba94  MSP: 0x20043168
        19:47:11 downloadfile: bootloader\bootloader.bin  addr: 0x12010000  len: 62928 Byte
        19:47:11 percent:18
        19:47:11 downloadfile: main.bin  addr: 0x12020000  len: 267784 Byte
        19:47:12 percent:56
        19:47:12 percent:95
        19:47:12 percent:96
        19:47:13 downloadfile: ftab\ftab.bin  addr: 0x12000000  len: 11288 Byte
        19:47:13 percent:100
        19:47:13 percent:100
        Download Successful
        Press any key to continue . . .
        user@DESKTOP-2AB1CDE c:\work\release_v2.2.0\example\hello_world\rtt\project
        >
        user@DESKTOP-2AB1CDE c:\work\release_v2.2.0\example\hello_world\rtt\project
        >

    如果串口打开失败，请检查串口是否已被其他程序占用，若串口打开成功但下载失败，可以尝试进入下载模式再启动下载，进入下载模式的步骤为：

    {#siflitrace-connect}
    1. 打开SiFliTrace，如下图，从下拉框选择开发板调试串口（如COM19），波特率保持默认值`1000000`，勾选`BOOT`，最后点击**连接**按钮，串口若打开成功，连接按钮左侧的**红灯**变为**绿灯**
    
        ```{image} assets/siflitrace_open2.png
        :alt: siflitrace open
        :scale: 30%
        ```

        ```{image} assets/siflitrace_open_succ.png
        :alt: siflitrace open succ
        :scale: 40%
        ```

    2. 短按开发板的复位按键让板子重新上电，出现如图提示`receive boot info1, stop it!!!`表示进入了下载模式，点击**断开**按钮断开串口，**绿灯**变为**红灯**，此时就可以按前文[下载程序到开发板](#download-program)的步骤，运行脚本再次下载程序

        ```{image} assets/siflitrace_download_mode.png
        :alt: siflitrace download mode
        :scale: 40%
        ```
        ```{note}
        `SFBL`为BOOTROM启动日志，如果复位后看不见这条日志，请检查板子供电
        ```
1. 查看hello_world日志

    打开SiFliTrace串口日志查看软件，参考上文的[SiFliTrace配置方法](#siflitrace-connect)，区别在于**不勾选**`BOOT`，连接串口成功后（绿灯亮起），短按开发板的复位按键让板子重新上电，出现下图的日志即表示hello_world程序已正常运行
    ```{image} assets/siflitrace_helloworld_log.png
    :alt: siflitrace helloworld log
    :scale: 40%
    ```
