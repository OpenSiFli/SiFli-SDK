(quick_start)=
# SDK快速入门(SF52X SF56X)

## 1. 开发工具准备
### 需要安装的软件
1. Sifli env和SiFli SDK(从微盘下载)。
2. Keil V5.32或以上，armcc V6版本或以上。
3. Segger Jlink，V6.80a版本或以上。
   [Jlink下载地址](https://www.segger.com/downloads/jlink/JLink_Windows.exe)
4. 代码编辑软件vscode或者source insight。
5. 串口日志查看工具，使用sdk自带的sifliTrace或者sscom。

### 需要的硬件
1. Windows PC x1
2. ARM仿真器 x1  
```{note} 仿真器的供电和HDK板有冲突，需要关闭仿真器的供电。如果出现仿真器连接不成功，可以检查一下Jlink的供电硬件跳线。
下图是常见Jlink的供电跳线，需要移除。 
![](../../assets/disable_jlink_power_supply.png)
```
3. USB type-C 线 x 2\
   用于链接ARM仿真器和连接USB2UART小板给HDK板子供电，抓取HDK串口的LOG数据。
4. USB2UART debug小板 x1 (SIFLI-Debug001-V1.1.0)。<br>
>  USB2UART debug小板一览图 ![](../../assets/hdk_sifli_debug_overview.png)
5. HDK开发板 x 1套<br> 
   包含一个HDK CPU板子SF56X或者SF52X，以及屏幕子板（屏幕不同，相应的子板型号不一样）。<br>
>  HDK52x 开发板接口一览图 ![](../../assets/hdk_micro_overview.png)\
>  HDK56x 开发板接口一览图 ![](../../assets/hdk_lite_overview.png)\
>  屏幕子板接口一览图 ![](../../assets/hdk_screen_overview.png)

## 2. 开发环境配置
### ENV的配置
1. 解压env到PC上，如 _D:\work\env_ 。
2. 进入env目录，可以运行本目录下的 _env.exe_ ，如果打开失败可以尝试使用 _env .bat_ 。
3. 关联文件夹右键ConEmu Here菜单。
>  添加ConEmu Here到右键菜单 ![](../../assets/add_env_conemu_1.png)
>  任意文件夹右键可打开ConEmu Here ![](../../assets/add_env_conemu_2.png)

### 安装Keil
双击Keil安装包，按照提示点击下一步进行安装，需要注意的是keil的安装路径必须是 _C:\Keil_v5_ 目录。启动Keil，打开 _Help->>About_ 窗口，
如果显示的版本不是v5.32则需要卸载并安装新版Keil。<br> 
![](../../assets/keil_version_about.png)\
安装完Keil后将arm的路径加入Windows环境变量PATH，如： _C:\Keil_v5\ARM\ARMCLANG\bin_ 。

### 安装JLink
Jlink需要安装新版v6.80a，双击安装包 _JLink_Windows_V680a.exe_ ，安装路径选择默认路径，将JLink的路径加入Windows环境变量PATH，
如：_C:\Program Files (x86)\SEGGER\JLink_ 。
将 _$SDK_ROOT\tools\flash\jlink_drv\sf32lbxx_ 目录下的所有elf文件都拷贝到JLink安装目录下的 _Devices\SiFli_ 目录中，SiFli目录要手动创建，
拷贝完成后如下图所示。<br>
添加jlink驱动到Jlink安装目录 ![](../../assets/JLink_drv_1.png)

将jlink_drv目录下 _JLinkDevices.xml_ 中有关SiFli的内容合并到JLink安装目录下的 _JlinkDevices.xml_ 中。![](../../assets/JLink_drv_2.png)


修改JlinkDevice文件 ![](../../assets/JLink_drv_3.png)

### 配置系统环境变量
在系统环境变量的PATH变量中添加如图几个路径，实际路径需要本机工具与代码路径一致。
添加环境变量 PATH![](../../assets/add_env_path.png)

## 3. 编译工程代码
### SF52X工程代码编译
以hal_example下面的sf525工程为例编译步骤如下：
1. 进入sdk根目录 _$SDK_ROOT_，鼠标右键选择ConEmu Here菜单。
2. 执行脚本 `.\set_env.bat_` ，配置环境。
3. 进入到编译目录下：`cd example\hal_example\project\common`
4. 执行编译命令 ：`scons --board=eh-lb525_v2_hcpu -j8`
```{note} 
`--board=xx`, 需要指定编译的board信息，可以选择目录 _customer\\boards_ 下带v2的目录名对应的board，后面再加上_hcpu ，如eh-lb525_v2_hcpu。
编译结束后会在common目录下生成文件夹build_eh-lb525_v2_hcpu，存放编译生成的image等文件。
![sf525工程编译成功](../../assets/sf525_compile_success.png)
```

### SF56X工程代码编译
以hal_example下面的sf561工程为例编译步骤如下：
1. 进入sdk根目录，鼠标右键选择ConEmu Here菜单。
2. 执行脚本 `.\set_env.bat`，配置环境。
3. 进入到编译目录下：`cd example\hal_example\project\common`
4. 执行编译命令 ：`scons --board=eh-lb561_v2_hcpu -j8`
```{note} 
`--board= xx`, 需要指定编译的board信息，可以选择目录 _customer\\boards_ 下带v2的目录名对应的board，后面再加上_hcpu ，如eh-lb561_v2_hcpu。
编译结束后会在common目录下生成文件夹build_eh-lb561_v2_hcpu，存放编译生成的image等文件。
![sf561工程编译成功](../../assets/sf561_compile_success.png)
```

## 4，下载烧写HDK
### SF52X工程image下载烧写
sf525使用的是uart接口下载镜像，将HDK板子连接siflideubg小板，再将siflideubg小板使用typec线连接电脑，需要先确认一下CPU的日志输出的com口，比如为COM4。<br>
1. 串口日志工具关闭串口。
2. 在编译目录下运行下载脚本 `.\build_eh-lb525_v2_hcpu\uart_download.bat`。
3. 此时会让输入下载使用的COM口， 输入4。按enter键，下载开始。
> sf525下载image ![](../../assets/sf525_download_img.png)

### SF561工程image下载烧写
sf561使用的是JLink下载镜像，将HDK板子连接siflideubg小板，将siflideubg小板使用typec线连接电脑用于串口打印日志，再将ARM仿真器通过JLink排线连接到siflideubg小板上，
ARM仿真器的另一端使用typec连接到电脑。
1. 使用J-Link Commander发送`connect`命令确认开发板是否能连接成功。
```{note}
connect时选择的Device名与工程目录下rtconfig.py中的JLINK_DEVICE保持一致，比如sf561选择SF32LB56X。选择完device后会选择interface（SWD），
接下来配置传输速率，默认即可。可以看到连接成功。
>sf561连接JLink 
![](../../assets/sf561_jlink_connect_1.png)
![](../../assets/sf561_jlink_connect_2.png)
```
2.  在编译目录下运行下载脚本 `.\build_eh-lb561_v2_hcpu\download.bat`，开始启动JLink下载。
>sf561下载image 
![](../../assets/sf561_download_img_1.png)
![](../../assets/sf561_download_img_2.png)
