# 通用工程构建方法

## 1. 背景
  
 在v2.1.5之前的SDK版本里，每个示例目录下都会按板子创建不同目录，放置对应的工程文件，比如 _$SDK_ROOT/hal_example/project_ 下有eh-lb561、eh-lb563和eh-lb523的目录，分别对应三种板子
 的工程，每个目录下都有一套类似的SCons脚本以及 _.config_ 和 _rtconfig.h_ 配置文件，如果需要编译板子eh-lb561的软件，就进到eh-lb561目录执行scons进行编译。
 这种方式存在的问题是每增加
 一块板子就要新建一个工程目录，示例一多就要增加很多工程，一旦某个示例有公共配置需要修改，又要修改所有的工程，维护工作量很大。
 因此，从v2.1.5开始，SDK引入一种新工程构建方式，称为通用工程，每个示例只需要维护一个工程文件夹，工程目录下不再保存.config和rtconfig文件，
 编译时根据选择的板子就可以生成对应板子的软件，_.config_ 和 _rtconfig.h_ 也会生成在build目录下，可以同时编译多个板子，生成的文件不会相互覆盖。
 
 
## 2. 编译方法
 在通用工程目录下，执行如下命令可以编译生成指定的板子的镜像文件，board_name是板子的名称，他后面仍旧可以跟其他scons参数，如-j8指定使用8个线程编译。
 对于双核芯片的板子，board_name需要指定是哪个核，比如eh-lb523_hcpu，表示编译可运行在523 HDK的HCPU上的程序。
 通用工程为了减少重复文件的维护工作量，很多文件都会在编译时生成，如 _.config_、 _rtconfig_ 和链接脚本等，具体的生成方法见3.3节。
```python    
    scons --board=board_name
```

以下命令编译可运行于板子eh-lb523_v2 HCPU上的程序

```python    
    scons --board=eh-lb523_v2_hcpu -j8
```

编译生成的文件存放在build_eh-lb523_v2_hcpu目录下，如下图所示

![](../../assets/an_28_build_output.png)


## 3. 如何创建通用工程

通用工程的创建分为板子和应用工程两部分，如果是在已有的板子上创建新的应用，可以跳过3.1节创建板子，直接新建一个应用工程

### 3.1 板子

板子的目录结构如下图所示，一级目录下放板子相关的源文件，包括
-	IO相关的代码，如bsp_pinmux.c， bsp_power.c
-	初始化代码，bsp_init.c
-	ptab.json，memory规划
-	SConscript，编译脚本
-	Kconfig.board，板子的Kconfig配置文件


二级目录则是每个核的配置文件，包括
-	_board.conf_ ，由kconfig生成的板子最小配置文件
-	_Kconfig.board_ ，对应核的Kconfig配置文件，可以source一级目录下的 _Kconfig.board_
-	_link.sct_ ，可选，Keil的链接脚本文件
-	_rtconfig.py_ ，可以指定JLINK_DEVICE、优化等级等编译参数（优化等级推荐使用新增的kconfig配置）
-	_custom_mem_map.h_ ，打开CUSTOM_MEM_MAP时使用

![](../../assets/an_28_board.png)


有时同一个型号的HDK会贴不同的芯片，为了避免在不同的板子目录下放置相同的IO设置代码，可以将公共代码放到一个目录，
再由不同板子来引用，如下图所示，公共代码放在了eh-lb56xu_v2，开发板HDK561对应文件夹eh-lb561_v2，开发板HDK563对应文件夹eh-lb563_v2，
他们的一级目录可以简化，引用eh-lb56xu_v2下的文件，HCPU和LCPU的仍旧放在这两个开发板目录下。

![](../../assets/an_28_board2.png)

#### 3.1.1 board.conf

在二级目录运行menuconfig打开配置窗口，这个配置界面会加载SDK中除应用之外的所有Kconfig，
修改配置后选择“[D] Save minimal config”即可保存最小的差异化配置（与Kconfig的默认配置相比有变化的部分），
文件名填写board.conf。选择“[S] Save”则会生成 .config和 _rtconfig.h_ ，方便下次打开时能显示正确的配置，如果不生成.config，
关闭menuconfig后再打开，并不会加载 _board.conf_ 中的配置，看到的仍旧是所有的缺省配置，为了避免 _rtconfig.h_ 被误引用，
建议删除生成的 _rtconfig.h_ 。也可以执行 _gen_board_config.bat_ 基于 _board.conf_ 生成.config和config.h，
生成的文件内容与选择“[S] Save”相同。

建议用board.conf配置一个可用于一般应用开发的基本运行环境，比如保证console(选择合适的串口)和文件系统(选择合适的文件系统类型)可以工作，
应用相关的配置则留到应用工程的配置文件中设置。

![](../../assets/an_28_board2.png)

#### 3.1.2 rtconfig.py

rtconfig.py增加CORE和CHIP两个变量，CORE用于指示对应的核，可以是”HCPU”或者”LCPU”，CHIP为芯片系列名，如”SF32LB56X”、”SF32LB52X”

![](../../assets/an_28_board_rtconfig.png)


### 3.2 应用工程
对于要用到双核的应用，工程目录也分为hcpu和lcpu，如下图

![](../../assets/an_28_app_folder.png)
   
对于SF32LB52X，如果只需要开发HCPU上的程序，不使用LCPU或者使用预生成好的LCPU镜像文件，应用工程就只需要创建一个目录放置HCPU的工程文件，例如下图的hal_example示例工程

![](../../assets/an_28_52x_app.png)

目录下的文件有：
-	_Kconfg.proj_ ，应用的Kconfig配置文件
-	_proj.conf_ ，应用的最小配置文件
-	_rtconfig.py_ ，应用自定义编译选项，如果与板子目录下的 _rtconfig.py_ 重复，则覆盖同名配置，可以是空文件
-	Sconscript/Sconstruct，编译脚本
-	应用相关的源代码

#### 3.2.1 SConstruct

区别于以前的工程，需要在SConstruct脚本的开始调用`PrepareEnv()`， 准备板子的编译环境，如下图

![](../../assets/an_28_sconstruct.png)

由于子工程可能运行在不同的核上，需要在SConstruct脚本添加子工程的时候增加参数指定子工程运行在哪个核上，不指定则与主工程相同，例如下图添加了LCPU子工程

![](../../assets/an_28_app_child_proj.png)
   
   
#### 3.2.2 proj.conf

类似 _board.conf_ 的生成，在工程目录下运行menuconfig可以保存最小配置到 _proj.conf_ ，如果保存了全部配置，同样要删除 _rtconfig.h_ 。
可以执行 _gen_proj_config.bat_ 基于 _proj.conf_ 生成.config和 _config.h_ ，生成的文件内容与选择“[S] Save”相同。
build目录下也会生成一个Kconfig，内容类似下面，相比应用工程目录下的Kconfig，会多加载一个板子的Kconfig文件，以便生成完整的配置文件。


```python    
source "$SIFLI_SDK/Kconfig.v2"
source "$SIFLI_SDK/customer/boards/eh-lb56xu_v2/hcpu/Kconfig.board"
source "D:/Users/xiaomingxi/code/bt7/example/multicore/data_service/common/hcpu/Kconfig.proj"
```   
   
以下为proj.conf的示例，如果要将某个选项关闭，则以\#开始，跟着选项名（加上CONFIG_前缀），然后写上“is not set”，例如下面的配置关闭了ULOG_USING_COLOR。  
  
```python   
CONFIG_RT_MAIN_THREAD_STACK_SIZE=4096
CONFIG_RT_MAIN_THREAD_PRIORITY=15
CONFIG_RT_USING_ULOG=y
CONFIG_ULOG_USING_ISR_LOG=y
## CONFIG_ULOG_USING_COLOR is not set
CONFIG_ULOG_OUTPUT_THREAD_NAME=y
CONFIG_RT_TICK_PER_SECOND=1000
CONFIG_IDLE_THREAD_STACK_SIZE=512
CONFIG_RT_TIMER_THREAD_STACK_SIZE=1024
CONFIG_RT_CONSOLE_DEVICE_NAME="uart1"
CONFIG_BSP_USING_TOUCHD=y
CONFIG_BSP_USING_LCPU_PATCH=y
CONFIG_BSP_USING_LCD=y
CONFIG_BLUETOOTH=y
CONFIG_BSP_USING_FULL_ASSERT=y
## CONFIG_BSP_USING_LOOP_ASSERT is not set
## CONFIG_BSP_USING_EMPTY_ASSERT is not set
```   
  
### 3.3 文件生成规则

为了减少重复文件，以下几个文件在编译时会根据选择的板子动态生成在build目录下
-	_rtconfig.h（.config）_
-	_ptab.json/ptab.h_
-	_custom_mem_map.h_
-	_link.sct/link.lds_ (链接脚本)

生成的一般原则是按照优先级选择相应目录下的文件作为最终使用的文件或者覆盖低优先级的配置，优先级由高到低依次是：
1.	应用工程目录下的板子目录
2.	应用工程目录下的芯片目录(由 _rtconfig.CHIP_ 指定的芯片系列名称，如sf32lb56x)
3.	应用工程目录
4.	板子目录
5.	芯片系列目录

#### 3.3.1 custom_mem_map.h

比如，在应用工程目录 _data_service/common/hcpu_ 下有个eh-lb561_v2_hcpu的板子目录，放了 _custom_mem_map.h_  ，
那么build目录下的 _custom_mem_map.h_ 就拷贝自 _data_service/common/hcpu/eh-lb561_v2_hcpu/custom_mem_map.h_ ，如果应用的板子目录下没有 _custom_mem_map.h_ ，
但应用工程目录下有，那么build目录下的 _custom_mem_map.h_ 就拷贝自 _data_service/common/hcpu/custom_mem_map.h_ ，只有在应用工程目录下也没有该文件时，
才会把 _$SDK_ROOT/customer/boards/eh-lb561_v2/hcpu/custom_mem_map.h_ 拷贝到builld目录下。

#### 3.3.2 ptab.json

按以下顺序选择靠前目录下的 _ptab.json_ 作为编译使用的文件
1.	应用工程目录下的板子目录
2.	应用工程目录下的芯片目录
3.	应用工程目录
4.	板子目录
_ptab.h_ 由 _ptab.json_ 生成并保存在build目录下



#### 3.3.3 link script (链接脚本)

按以下顺序选择靠前目录下的link script文件(keil编译对应 _link.sct_ , gcc编译对应 _link.lds_ )作为编译使用的文件
1.	应用工程目录下的板子目录
2.	应用工程目录下的芯片目录
3.	应用工程目录
4.	板子目录
  
#### 3.3.4 rtconfig.h
  
_rtconfig.h_ 是由Kconfig的默认值、_board.conf_ 和 _proj.conf_ 合并而成。 _board.conf_ 和 _proj.conf_ 中记录了需要修改的配置（与默认值相比），
如果相同的配置同时出现在 _board.conf_ 和 _proj.conf_ 中，则使用 _proj.conf_ 定义的配置。还可以在应用录下添加板子目录，放置该板子专属的proj.conf，
比如不同板子大部分软件配置都一样，只有个别参数不同，可以在根目录的proj.conf定义所有板子都会用到的配置，
在个别需要使用不同参数值的板子目录下放置 _proj.conf_ ，在其中只定义需要修改的参数值。归纳起来，_rtconfig.h_ 和 _.config_ 中的参数取值来源，
按优先级由高到低依次如下，如果多个文件中都对同一个变量进行了设置，优先级高的将覆盖优先级低的：
1.	应用的板子目录下的 _proj.conf_
2.	应用的芯片名目录下的 _proj.conf_
3.	应用目录下的 _proj.conf_
4.	板子目录下的 _board.conf_
5.	Kconfig的默认值
  
  
### 3.4 link script的复用
多工程编译时，同样运行于HCPU的主工程和子工程可能都会使用同一个板子目录下的链接脚本，但他们的代码编译地址又不一样，
比如下图是 _link.sct_ 的代码段部分，起始地址都是CODE_START_ADDR，大小为CODE_SIZE，但在编译主程序和二级bootloader时，
这两个宏的取值是不同的。  
  
  
![](../../assets/an_28_link_script.png)  


为此，ptab.h的生成脚本增加了一个功能，在 _ptab.json_ 中使用属性”exec”定义该region是哪个工程的代码执行段，
属性值为工程名，主工程是“main”，子工程与AddChildProj时指定的名称一致。例如下面的 _ptab.json_ 中，
二级bootloader的代码段以FLASH_BOOT_LOADER命名，主工程的以HCPU_FLASH_CODE命名。

![](../../assets/an_28_ptab_exec_attr.png)  
  
  
加上了exec属性后，主工程生成的 _ptab.h_ 会把CODE_START_ADDR定义为HCPU_FLASH_CODE_START_ADDR，
二级bootloader子工程则会把CODE_START_ADDR定义为FLASH_BOOT_LOADER_START_ADDR  
  
  
![](../../assets/an_28_ptab_header.png)  
  
  
## 4. 改造现有工程

按以下步骤将现有工程改造为支持任意板子编译的通用工程

1.	创建板子文件夹，从原工程的.config里提取板子相关的配置，写入board.conf（如板子已存在则跳过此步骤），拷贝工程目录下的 _ptab.json_ 到新建的板子目录
2.	创建应用工程文件夹，从原工程的.config里提取应用的共性配置，写入应用工程根目录下的 _proj.conf_ ，将板子专有的配置写入到应用板子目录下（板子名需以_hcpu或者_lcpu结尾）的 _proj.conf_
3.	拷贝原工程目录下的Sconscript和SConstruct，在SConstruct里增加11行的“PrepareEnv()”调用，以支持指定板子的编译

![](../../assets/an_28_sconstruct.png)  
  
