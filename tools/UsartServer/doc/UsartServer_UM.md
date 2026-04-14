
# UsartServer

## 1. 概述

UsartServer 是思澈公司自研工具，是基于芯片内置的 DEBUG IP 进行调试的配套工具，思澈公司量产的SF32LB52X和SF32LB56X系列芯片内置有DEBUG IP，可以通过串口进行调试。\
工具路径：`tools/UsartServer`

## 2. 环境配置

UsartServer 免安装，可直接运行于WINDOWS系统，WINXP/WIN7/WIN10/WIN11…

## 3. 功能介绍

<img src="png/UsartServer_001.png"/>  
  
工具主界面如图所示，主要控件功能描述如下：  

- **平台**  
  选择芯片类型，LB52X / LB56X。
- **BOOT**  
  该功能用于LB52X系列芯片强制进入BOOT模式，LB52X系列芯片无BOOT MODE管脚，需要在芯片启动前2S内通过UART1发送命令进入BOOT模式，该功能对其他型号的芯片无效。可以根据控件前面的数字增加来判断工具检测到启动信号并强制芯片进入BOOT模式。正常使用时需关闭此开关，否则目标板启动后无法进入用户程序。
- **串口时延**  
  FTDI的USB转串口芯片在PC上安装驱动后，会有一个时延参数，该参数比较大时会影响交互速率，建议通过工具设置为1ms，该参数对于其他公司的USB转串口芯片可能无效。
- **H 串口**  
  工具打开默认提供一套调试接口，即只显示UART1。如果需要同时使用两个调试口进行调试，可以双击该控件打开UART4调试界面。
- **串口号**  
  UART1/UART4对应的串口号。
- **波特率**  
  选择串口的波特率，初始波特率需要同固件中配置的波特率一致，一般为1000000。串口连接后选择波特率会修改固件中串口的波特率，可以通过提高波特率来加快交互速度，目标板重新上电后波特率会恢复为固件默认设置。
- **SERVER**  
  工具提供服务的地址端口，供其他工具连接使用，该地址端口配置了默认值。每个SERVER支持最多2条链路，有设备接入时指示灯会显示绿色。
- **连接/断开**  
  连接串口并发送命令打开调试功能，串口连接成功且调试功能打开交互正常，后面指示灯显示为绿色，如果只是串口连接成功但是打开调试功能的交互失败则显示为灰色。

## 4. 使用方法

工具使用比较简单，直接双击运行，执行步骤如下：

- 根据目标板选择芯片类型；
- 选择UART1 串口号，选择波特率，选择调试的CPU；
- 点击 **连接**，指示灯显示为绿色；
- 使用JLink调试的第三方工具配置为IP方式连接调试器（配置方法参考后面描述）；
- 第三方工具连后，SERVER 后面的状态指示灯变绿色表示正常，第三方工具可以正常进行调试；

常用的调试ARM芯片的工具使用UsartServer的配置方法：

- **JLink Commander**  
  打开 JLink Commander工具，如果电脑上连接有其他JLink设备，则会默认选择USB JLink设备，可以在工具输入 ip 127.0.0.1:19025命令切换到串口调试器；如果电脑上没有连接其他JLink设备，则会有弹框来输入IP设备的地址，在Idertifier窗口输入127.0.0.1:19025即可（UsartServer工具连接设备后会把地址放在剪切板，所以这里直接粘贴就行）。填完信息后点击 Yes 界面会显示连接上设备，UsartServer 的 SERVER 指示灯变为绿色表示连接成功。

- **Kei 软件**  
Keil软件打开工程文件后，在 Project -> Opentions for Target -> Debug -> Use J-LINK/J-TRACE Cortex -> Settings -> Debug -> Interface 选择 TCP/IP，并在该页面设置 TCP/IP -> IP-Address为127.0.0.1，设置 TCP/IP -> Port为 19025。设置完后在该页面点击 Interface -> Connect， 如果 UsartServer 的 SERVER 指示灯显示为绿色表示连接成功。

- **Ozone 软件**  
Ozone软件打开后，会弹出 New Project Wizard 界面，或者通过 File -> New -> New Project Wizard 打开该界面，在第二个设置页签 Connection Settings中，设置 Target Interface 为 SWD, 设置 Host Interface 为 IP, 设置 IP Address为 127.0.0.1:19025（UsartServer工具连接设备后会把地址放在剪切板，所以这里直接粘贴就行），完成创建流程，进行调试时，UsartServer 的 SERVER 指示灯变为绿色表示连接成功。

- **命令行调用JLink.exe**  
当使用命令行调用JLink.exe来执行JLink脚本来完成读写数据等操作时，也可以在JLink.exe 后面加上 -ip 127.0.0.1:19025 来使用串口实现该功能。eg： JLink.exe -ip 127.0.0.1:19025 -device SF32LB52X_NOR -if SWD -speed 4000 -autoconnect 1 -CommandFile SF32LB52X_BURN.jlink
