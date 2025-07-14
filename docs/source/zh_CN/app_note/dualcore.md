# 双核的开发

## 1. 双核介绍
  
 SF32LB55X为双核架构，提供两个Cortex MStar处理器，其中
 - 大核HCPU <br>
   HCPU主要完成高性能的计算和图像处理，最高可以运行到240MHz, 有1M内存。 <br>
   HCPU有三个QSPI，可以连接NOR Flash, NAND flash, PSRAM。不同的封装拥有不同大小的Flash和PSRAM。通常代码运行在NOR Flash中。
   HCPU有64K ROM, 用于上电启动。
 - 小核LCPU <br>
   LCPU 是一个低功耗内核，做一些相对简单传感器采集缓冲，简单处理，以及低功耗蓝牙等任务, 最高可以运行到48MHz, 有256KB内存。 <br>
   通常代码运行在RAM中，由HCPU加载。LCPU也有一个QSPI, 可以连接NOR Flash运行代码，但由于管脚限制，不是所有封装都可以访问。
   LCPU有384K ROM, 包含了RT-Thread操作系统以及思澈的低功耗蓝牙协议栈。
 当LCPU处于唤醒状态的时候，HCPU可以使用LCPU的硬件模块，也能收到所有硬件模块的中断。<br>
 当HCPU处于唤醒状态的时候，LCPU可以使用HCPU的硬件模块，但无法收到这些模块的中断。<br>
 HCPU/LCPU的硬件资源，请参考[](../hal/index.md) <br>
 用户设计的时候，首先需要将不同的功能根据大小核的特点进行分类，确定每个核需要实现的功能，以及需要使用到的硬件模块。

## 2. 双核启动

### 系统启动
  SF32LB55X上电后，系统从HCPU启动，ROM启动代码读取启动Flash上的flash table, 加载指定的用户程序运行. 通常用户代码从第三个64K地址开始。详细过程，请参考[](../bootloader.md) 。<br>
  上电后，LCPU默认处于有电唤醒状态，但是没有运行，这个时候，HCPU可以使用LCPU的硬件模块。
 
### 双核程序编译
 LCPU的程序文件被作为一个常数表编译到HCPU的程序文件中，参考 _$SDK_ROOT/example/multicore/ipc_queue_ 中的示例，编译LCPU工程除了生成LCPU的程序文件外，还会将生成的bin文件
 转换成 _lcpu_img.c_ 存放在hcpu工程目录下, _lcpu_img.c_ 中使用常数表定义了LCPU的二进制代码，并实现了 _lcpu_img_install_ 安装函数用于加载LCPU代码到RAM中。

 LCPU编译脚本使用了SDK提供的工具 _$SDK_ROOT/tools/patch/gen_src.py_ ，将编译生成的LCPU bin转换为C数组，并实现安装函数，_gen_src.py_ 脚本的用法如下：
```python 
    python gen_src.py lcpu <usr image.bin> <output folder> 
```
 
### LCPU代码加载
　LCPU的代码由HCPU加载，拷贝到LCPU的RAM中，配置LCPU的Stack和PC后，启动LCPU开始运行。

  还是以 _$SDK_ROOT/example/multicore/ipc_queue_ 为例，在HCPU的console中执行`lcpu on`命令启动LCPU，该命令会调用函数`lcpu_power_on`, 该函数会调用`lcpu_img_install`搬移代码到LCPU RAM中，
  并配置好相应的寄存器启动LCPU。


## 3. LCPU代码的开发
  由上述介绍可知，相对于HCPU，LCPU工作在更低的频率(48MHz)以及更低的RAM空间(256KByte)，并且BLE的协议栈也运行在LCPU。所以LCPU代码开发主要侧重于以下几个方面：
  - 功能规划
  - 内存规划
  - 双核数据交互
 
### 功能规划
  LCPU相对于HCPU最大的优势是更低的功耗，功能规划的核心也在于功耗的规划。下面分为蓝牙部分和非蓝牙部分来进行说明：
   - 蓝牙部分：SF32LB55X的协议栈运行在LCPU，同时在协议栈的基础上，SF32LB55X SDK提供了更简单的蓝牙服务层。蓝牙服务层可以通过menuconfig配置，选择运行在LCPU或者HCPU。<br>
               运行在LCPU，可以在不唤醒HCPU的情况下，与对端设备进行蓝牙数据的交互；但需要考虑数据的夸核操作，以及双核之间的蓝牙状态同步等。<br>
               运行在HCPU，不用考虑数据的夸核操作；但所有蓝牙的数据交互及处理都会运行在更高功耗的HCPU上。
   - 非蓝牙部分：适合做唤醒频率高且运算相对简单的动作，比如前文提到的传感器数据的采集和简单的数据处理，可以充分利用LCPU的低功耗优势。<br>
     需要注意的是LCPU的RAM空间是有限的，要结构内存规划来考量。

### 内存规划
  LCPU在没有外接flash的情况下，有256K的RAM，内存的整体Layout可以参考[](../app_note/memory_usage.md)。需要注意的是256K包含了所有的RO和RW，需要谨慎规划。<br>
  同时LCPU的ROM提供了RT-Thread操作系统，BLE的协议栈以及大部分的HAL层接口，开发者可以选择直接调用ROM的函数来节约RO/RW，但ROM本身以及蓝牙协议栈也会消耗一定的RAM空间，其具体规划如下：

|名称               | 起始地址       | 结束地址       | 大小(字节)     | 描述
|-------------------|----------------|----------------|----------------|----------------
|ROM used RAM area  | 0x20124000     | 0x20130000     | 48*1024       | |
|BLE stack area     | 0x20130000     | 0x20138000     | 32*1024       | |

#### ROM说明
  LCPU采用了symbol definition(symdefs) file的机制，LCPU的RAM工程可以通过导入ROM的sym文件来访问ROM的函数。ROM的sym文件包括：
- 整个RT-Thread操作系统
- LCPU支持的HAL模块
- 部分driver
  - alarm
  - flash
  - i2c
  - spi
  - timer
- BLE协议栈API
 
  RAM工程只需要正常config相应的模块并打开LCPU_ROM的宏，这样RAM工程会在编译时遇到同名函数，会直接选择ROM的函数进行链接。开发者也可以通过恢复删除sym文件的symbol来指定最终链接的函数是ROM还是RAM。<br>
  
  可以按照以下步骤添加symdefs：<br>
 
  1. 在Link选项中指定ROM的symdefs文件可以获取到ROM的符号表
      - ROM的symdefs文件放在 __$SDK_ROOT/example/rom_bin/lcpu_boot_loader/rom.sym_ 。可以通过工程的prebuild在LCPU编译前拷贝到LCPU的工程目录下。
      - 在工程的 _rtconfig.py_ 中加入“CUSTOM_LFLAGS = 'rom.sym'”，SCONS会自动帮忙把工程目录下的 _rom.sym_ 加入到Link选项中。
  2. 并在LCPU的工程下的Kconfig下使能LCPU ROM的宏：
```python
     config LCPU_ROM 
     bool 
     default y
```
  
  注意事项：
  1. ROM函数是Read-Only，所以无法通过menuconfig配置参数的开关。
  2. ROM函数只会使用ROM定义的全局变量，RAM工程可以选择使用RAM工程定义的变量也可以使用ROM工程定义的变量。在通过sym文件改变链接函数时要特别注意这一点，避免修改后期望ROM使用RAM的全局变量不生效等异常状态。
  

### 双核数据交互
  
  1. 通信方式：请参考下一章节核间通信的介绍。
  2. 通信节点：参考双核启动的介绍，LCPU是由HCPU启动，因此LCPU的启动时间点一定是晚于HCPU，取决于HCPU启动LCPU的时间点。
     - 双核都是按照操作系统的启动顺序独立运行
     - 核间通信是通过核间共享内存进行数据交互，只要保证发送数据被覆盖前，数据接收方的接收模块已经完成初始化并且能够接收并处理该数据，就可完成数据的交互。<br>
       比如HCPU要基于数据服务向LCPU发送数据，只要确保同样基于数据服务的接收端在LCPU的数据服务开始接收该数据之前，已经完成数据服务的注册，就可以在触发LCPU power on后，不用等待LCPU状态更新就可以发送该数据。
  3. 其他注意事项：
     - 基于核间通信的数据交互，会自动唤醒另外一个核心再交互数据，不需要考虑另外一个核的睡眠状态。


## 4. 核间通信
 HCPU和LCPU之间的通信，可以有三个级别，用户可以根据自己的需求使用不同的接口，这三个基本从低到高分别是：
 
 - Mailbox <br>
   HCPU和LCPU可以互相访问对方的内存，但是访问的速度比本地内存速度低，而且需要保证对方处于唤醒状态。
   同一个内存，在HCPU和LCPU有不同的地址，SDK提供了HCPU_ADDR_2_LCPU_ADDR，以及LCPU_ADDR_2_HCPU_ADDR来转换不同的地址空间，详细的地址空间可以参考[](../app_note/memory_usage.md)<br>
   Mailbox主要实现了中断触发功能，当一个CPU需要触发另一个CPU执行某个任务时，只需触发该CPU的MAILBOX中断，该CPU就会收到中断，具体API可以参考[](../hal/mailbox.md).
   双方还可以约定共享内存的地址，用于交换数据，但由HCPU发送给LCPU的数据，必须存放在HCPU的RAM中，反之亦然，因为在收到MAILBOX中断前，LCPU可能处于睡眠状态，HCPU无法访问LCPU的memory。
   
 - IPC Queue/IPC Queue Device 
   - IPC Queue是在Mailbox的基础上，封装了循环buffer和中断服务，便于在双核之间进行数据交换。
   - IPC Queue模式是操作系统无关的，但提供了对RT-Thread低功耗框架的支持，在发送buffer没有被读空前，发送方不会进入睡眠模式。
   - IPC Queue Device在IPC Queue的基础上实现了RT-Thread的设备接口，便于做console的重定向。
   - IPC Queue的具体用法可以参考 _$SDK_ROOT/example/multicore/ipc_queue_
 - 数据服务   <br>
   - 数据服务(data service) 在IPC Queue的基础上，进一步提供一种client/server结构的分布式数据业务。数据业务的提供者(server)通常运行在LCPU, 使用者(client)通常运行在HCPU。
   - 两者之间的连接由数据服务框架提供。无论是提供者还是使用者都可以单独进行开发/模拟，尤其是LCPU的代码，开发的时候都可以先使用HCPU，更容易调试，完成后再在真正的使用核上进行集成验证。
   - 具体用法可以参考 _$SDK_ROOT/example/multicore/data_service_

## 5. 调试和日志
  请参考[](../app_note/debug_logging.md)
   
