# SDK快速入门

## 1. 开发环境准备
### 需要安装的软件列表
1. Sifli env 和 SiFli SDK (从微盘下载)
1. Keil V5.32或以上, armcc V6 版本或以上
   将arm和jlink的路径加入Windows环境变量PATH， 如 _C:\Keil_v5\ARM\ARMCLANG\bin_ 和 _C:\Program Files(x86)\SEGGER\JLink_v672b_ , 有助于一些脚本的使用
1. Segger Jlink,  V6.72b 版本或以上[Jlink下载地址](https://www.segger.com/downloads/jlink/JLink_Windows.exe)
1. Visual Studio 2017或更高版本

### 需要的硬件
1. Windows PC x1
1. ARM仿真器 x1   
```{note} 
  仿真器的供电和EVB板有冲突，需要关闭仿真器的供电。如果出现仿真器连接不成功，可以检查一下Jlink的供电硬件跳线。下图是常见Jlink的供电跳线，需要移除 
  ![](../../assets/disable_jlink_power_supply.png)
```  
1. USB type-C 线 x1     
   用于给EVB供电，以及连接EVB上的FTDI USB2UART芯片，用于抓取EVB串口数据
1. EVB开发板 x1套\
   包含一个接口底板EI-LB55XXXX001, 和一个CPU子板LB551, 以及屏幕子板(屏幕不同，相应的子板型号不一样。)
>  EVB 开发板接口一览图  ![](../../assets/evb_a0_overview.png)

### 编译必须的配置
1. 解压env到PC上，如 _c:\work\env_
2. 进入env目录，可以运行本目录下的 _env.exe_ ，如果打开失败可以尝试使用 _env.bat_ 
3. 关联文件夹右键ConEmu Here菜单
![ ](../../assets/Add_Env_To_Right-click_Menu.png)

### 添加Keil Flash烧写驱动
  将 _tools/flash/keil_drv/sf32lb55x_ 目录下sf32lb55x.FLM，复制到 _C:\Keil_v5\ARM\Flash_ 下 (此处为默认的Keil安装目录，如果安装到其它目录则放到相应的文件夹下)

## 2. 编译示例工程

### 编译evb工程(DSI屏)
1. 打开SifliSDK目录，右键ConEmu Here打开ConEmu，执行批处理`set_env.bat`(设置相关的环境变量，如果不关闭ConEmu，只需设置一次，打开新的ConEmu需要再次设置)
```{note} _set_env.bat_ 里面有设置Keil的安装目录(默认是 _C:/Keil_v5_ )，用户需要根据自己的安装目录进行修改。
2. 切换到 _example/watch_demo/project/ec-lb551_
3. 用命令 `scons --target=mdk5 –s` 生成Keil工程
4.  然后用Keil打开 _project.uvprojx_ 编译
```{note} 
ec-lb555的工程类似编译
```

### 编译模拟器工程
1. 打开SifliSDK目录，右键ConEmu Here打开ConEmu，执行批处理 _set_env.bat_
2. 切换到 _example\watch_demo\project\watch_simu_
3. 命令`scons --target=vs2017 -s` 生成vs工程， 然后用visual studio2017打开工程 _project.vcxproj_ 编译运行
```{note} 
运行时如果Visual studio提示找不到 _SDL2.DLL_，则需要添加路径 _env\tools\Python27_ 到环境变量PATH，然后重启下Visula studio 
```
```{note} 
如果安装的是其他版本的visual studio, 打开工程时，需要按照提示改变windows SDK的版本。
```
## 3. 烧写EVB

SDK提供三种烧录EVB的方式，其中一种使用Keil的环境，另外两种使用Jlink的工具。

```{note}
正常情况可以不用改变跳线boot_mode, 如果烧写持续出现问题, 或者用户程序死机，可以将跳线boot_mode置于VDD一侧，然后按reset重启进入boot模式，再次尝试烧写。烧写完成之后，跳线boot_mode置于GND一侧，按reset重启正常运行。
```
### 3.1 用Keil烧录EVB
使用Keil烧录EVB flash需要首先添加Keil Flash烧写驱动, 将 _tools\flash\keil_drv\sf32lb55x_ 目录下 _sf32lb55x.FLM_ ，复制到 _C:\Keil_v5\ARM\Flash_ 下 (此处为默认的Keil安装目录，如果安装到其它目录则放到相应的文件夹下)

EVB至少需要烧写2部分： 
- Flash Table (只需要烧录一次，用于复位后ROM从FlashTable读取地址然后跳转到用户代码)， 具体用法参考 [安全引导加载](../../bootloader.md) 用户收到EVB的时候，flash table 已经烧录为默认，除非flash被破坏，一般情况下，客户可以忽略这个步骤。
- 工程代码

#### 烧录Flash table
1. 打开工程 _example/flash_table/project.uvprojx_
2. 按照最后一节选择Keil Flash烧写驱动
3. 选择flash1作为编译目标，然后编译，烧写
![ ](../../assets/keil_download_flash_table.png)

#### 烧录工程代码
1. 打开 _example/hal_example/project/ec_lb551/project.uvprojx_
2. 按照最后一节选择Keil Flash烧写驱动
3. 编译后，烧写


#### 选择Keil Flash烧写驱动
- 打开工程选项，按下图选择Flash驱动：
![ ](../../assets/keil_flash_download_config_a0.png)


### 3.2 用Jlink烧写EVB

Jlink 的版本此处以v672b为例，安装路径 _D:\Software\JLink_v672b_
#### 添加Jlink Flash烧写驱动
1. 在Jlink程序目录 _D:\Software\JLink_v672b\Devices_ 目录下新建目录 _SiFli_
2. 将 _tools/flash/jlink_drv/sf32lb55x_ 下的elf文件拷贝到前面新建的SiFli目录(每个elf对应一个flash的烧写驱动)
![](../../assets/add_sifli_jlink_device_A0_1.png)
3. 修改Jlink注册的Device列表，增加前面添加的文件路径，运行参数等,如下图:
![](../../assets/add_sifli_jlink_device_2.png)

附图中添加的xml内容：
```
  <!--                                    -->
  <!-- SiFli Z0(Cortex-M33 devices)-->
  <!--                                    -->
  <Device>
    <ChipInfo Vendor="SiFli" Name="SF32LB5XX" Core="JLINK_CORE_CORTEX_M33" WorkRAMAddr="0x20000000" WorkRAMSize="0x40000" />
    <FlashBankInfo Name="Internal Flash1" BaseAddr="0x10000000" MaxSize="0x400000" Loader="Devices/SiFli/SF32LB5XX_INT_FLASH1.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash2" BaseAddr="0x18000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB5XX_EXT_FLASH2.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
  <!--                                    -->
  <!-- SiFli SF32LB55X(Cortex-M33 devices)-->
  <!--                                    -->
  <Device>
    <ChipInfo Vendor="SiFli" Name="SF32LB55X" Core="JLINK_CORE_CORTEX_M33" WorkRAMAddr="0x20000000" WorkRAMSize="0x40000" />
    <FlashBankInfo Name="Internal Flash1" BaseAddr="0x10000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_INT_FLASH1.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash2" BaseAddr="0x64000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH2.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash3" BaseAddr="0x68000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH3.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash4" BaseAddr="0x12000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH4.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
```


#### 烧写Bin/hex文件到Flash
1. 打开Jlink，连接，并选择SiFli的device(注意SF32LB55X代表EVB的flash驱动)
![](../../assets/download_with_jlink_a0_1.png)

2. 选择swd接口，配置速度，选择对应的bin烧写到某一地址(可以烧ROM/RAM/FLASH都可以)，Jlink速度取决于Jlink的硬件，通常可以设置到4MHz以上，芯片可以支持最高10MHz.
![](../../assets/download_with_jlink_2.png)

3. 烧写结果
![](../../assets/download_with_jlink_3.png)

4. Hex文件的烧写，只需将loadbin 改成loadfile即可，<b>且不需要带地址</b>
![](../../assets/download_with_jlink_4.png)


### 3.3 集成到windows，右键烧写hex文件
添加完了的Jlink 的驱动以后，为了平时调试方便，可以将hex文件的烧写集成到windows的右键菜单，但只能烧写hex文件因为其自带地址，bin文件不带地址所以不支持。

1. 添加右键菜单
![](../../assets/integrate_jlink_download_to_right_click_menu_A0_1.png)

2. 右键目录烧写，将会把目录内的文件改为.hex后缀，并逐个烧写
![](../../assets/integrate_jlink_download_to_right_click_menu_A0_2.png)

3. 也可以单独选择烧录其中一个hex文件(只支持.hex后缀的文件)
![](../../assets/integrate_jlink_download_to_right_click_menu_3.png)

```{note} 为了使得BLE 应用运行正常，请在串口输入\
       1. nvds reset_all 1\
       2. nvds update addr 6 \<蓝牙地址\>
          例如 nvds update addr 6 1234567890C8，注意蓝牙地址分类需要保持一定的格式，建议C8不要改动，其他的用户可以自行改变
        然后按reset重启
```

```{warning}
56和58系列工程，编译完后在build目录下会生成 _download.bat_ 和 _download.jlink_ 文件，运行 _download.bat_ 就可以下载，不要用鼠标右键模式下载hex文件。如果点了，会更改 _download.jlink_ 文件内容导致下载失败，请删除被更改的download.jlink文件，并重新编译用 _download.bat_ 下载。
```


## 4. 上电引导流程
| 阶段                  | 函数                           | 文件路径                                                       |
| :----                 | :----                          | :----                                                          |
| 中断向量表            | ResetHandler                   | SifliSDK\\drivers\\cmsis\\sf32lb55x\\Templates\\arm\\startup_bf0_hcpu.S |
| MPU设置, BOOTMODE检查 | SystemInit()                   | SifliSDK\\drivers\\cmsis\\sf32lb55x\\Templates\\system_bf0_ap.c        |
| RO/RW/ZI初始化        | __main()                       |                                                                |
| RT_THREAD_MAIN入口    | `$Sub$$main`                     | SifliSDK\\rtos\\rtthread\\src\\components.c                       |
| ^                     | rtthread_startup()             | SifliSDK\\rtos\\rtthread\\src\\components.c                       |
| 硬件初始化            | rt_hw_board_init()             |                                                                |
|                       | HAL_Init();                    |                                                                |
| RCC 配置              | HAL_PreInit()                  | SifliSDK\\drivers\\boards\\ec-lb551XXX\\drv_io.c     |
|                       | HAL_MspInit()                  |                                                                |
| PIN脚配置             | BSP_IO_Init()                  |                                                                |
|                       | rt_system_heap_init            |                                                                |
| LOG输出口初始化       | rt_console_set_device          |                                                                |
| 驱动初始化            | rt_components_board_init();    |                                                                |
| 上层初始化            | rt_application_init()          | SifliSDK\\rtos\\rtthread\\src\\components.c                       |
|                       | main_thread_entry              | SifliSDK\\rtos\\rtthread\\src\\components.c                       |
|  上层中间件           | rt_components_init             |                                                                |
|                       | Main()                         |   工程main函数                                               |
| 启动线程调度          | rt_system_scheduler_start()    |                                                                |
 
```{note} 
EVB开发板插入PC后，PC会枚举四个个串口，其中编号最小的是Boot/LCPU的终端，第二个大的是HCPU的终端，目前建议用户关注HCPU的UART输出。其余两个串口暂时没有分配。
 ```

