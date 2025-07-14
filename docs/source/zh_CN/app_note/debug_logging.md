# 调试和日志

## 1. 硬件接口
  SF32FB55X采用SWD作为调试接口。用户可以通过配置，来切换选择HCPU或者LCPU。

  系统上电默认选择HCPU，如果想要调试LCPU，可以调用SDK的工具 _$SDK_ROOT/tools/segger/jlink_lcpu_a0.bat_ ，将SWD切换到LCPU。

  同样，如果目前SWD连接LCPU，可以调用SDK的工具 _$SDK_ROOT/tools/segger/jlink_hcpu_a0.bat_ ，将SWD切换到HCPU。
  
  ```{note} 
    1. 由于SWD使用PB IO，当使用SWD进行调试时，需确保LPSYS处于active或者light sleep状态，无论当前SWD连接至HCPU还是LCPU<br>
    2. Jlink发送reset命令不会改变SWD当前连接的CPU<br>
    3. LPSYS从Standby唤醒后， SWD会切换回默认的HCPU<br>
  ```
### LCPU日志接口
  系统ROM在初始化的时候，使用属于LCPU的UART3作为console接口，波特率为1000000bps，用来打印日志或者输入命令。建议这个接口保留给LCPU作为日志接口。<br>
  
### HCPU日志接口
  HCPU的日志接口可以选择UART1/2或者SWD，如果需要选择属于LCPU的UART3/4/5,则需要确保使用时，LCPU属于唤醒状态。
  
## 2. 调试方法
  这里主要讲一下常见的Assert或者HardFault的分析方法，以及死机解决办法，这里调试器都是使用的Jlink。
  
### 设置断点
当Jlink连接到HCPU/LCPU的时候，通常系统已经初始化完成，如果需要调试初始化，例如冷启动或者standby睡眠唤醒, 需要将系统停留在尽早的地方。<br>
建议用户可以修改系统初始化程序， 
 - HCPU<br>
   _$SDK_ROOT/drivers/cmsis/sf32lb55x/Templates/arm/startup_bf0_hcpu.S_ <br>
 - LCPU<br>
   _$SDK_ROOT/drivers/cmsis/sf32lb55x/Templates/arm/startup_bf0_lcpu.S_ <br>
在Reset_Handler中的第一条指令去掉注释 ';', 变为 <br>
  B  . <br>
这样CPU启动，就会停留在第一条指令，当Jlink连接成功后，可以改变PC寄存器 (+2), 设置所需断点，从而调试初始化过程。

同样的方法也可以在其他的地方使用，使系统停留在某个事件发生的时刻，在所怀疑有问题的地方，如果是C文件，加入 <br>
  _asm("B .");  <br>
可以使系统停留在这个指令，这个时候，再连接Jlink, 可以改变PC寄存器 (+2), 继续调试。
```{note} 
不能使用while(1); 否则系统会优化，将while(1)之后的语句都无效了。
```

### Assert/HardFault 错误分析
当错误发生的时候，如果开发板有连接SWD到Jlink工具，可以使用 _$SDK_ROOT/tools/crash_dump_analyser/script/save_ram_a0.bat_ 保存RAM，EPIC寄存器和PSRAM的内容到当前路径，有助于分析死机的原因
```{note} 
需要将jlink的路径加入Windows环境变量PATH， 如 _C:/Program Files (x86)/SEGGER/JLink_v672b_ ，以后可以通过Jlink加载RAM回复死机现场。
```
#### 通过日志分析
默认SDK会在Assert时通过日志接口输出断点行，以及最后的CPU寄存器，根据内容分析即可。注意，如果日志接口是异步输出，可能出现没有输出完整的情况。
```
Assertion failed at function:app_exit, line number:704 ,(app_node->next != &running_app_list)
===================
Thread Info        
===================
thread   pri  status      sp     stack size max used left tick  error
-------- ---  ------- ---------- ----------  ------  ---------- ---
app_watc  25  ready   0x00000100 0x00002800    26%   0x00000008 000
tshell    20  suspend 0x000000f4 0x00001000    13%   0x00000008 000
ble_app   15  suspend 0x000001b4 0x00000400    54%   0x00000007 000
mbox_th   10  suspend 0x00000110 0x00001000    51%   0x00000006 000
ds_proc   12  suspend 0x0000011c 0x00000800    24%   0x00000005 000
ds_mb     11  suspend 0x00000148 0x00000400    32%   0x0000000a 000
touch_th  10  suspend 0x000000ec 0x00000200    59%   0x00000006 000
test      15  suspend 0x0000011c 0x00000400    27%   0x0000000a 000
alarmsvc   8  suspend 0x00000074 0x00000200    22%   0x00000001 000
ulog_asy  30  ready   0x000000ec 0x00000400    36%   0x0000000b 000
tidle     31  ready   0x00000064 0x00000200    19%   0x00000008 000
timer      4  suspend 0x000000e0 0x00000400    23%   0x00000003 000
main      10  suspend 0x000000ec 0x00000800    31%   0x0000000c 000
===================
Mailbox Info       
===================
mailbox  entry size suspend thread
-------- ----  ---- --------------
g_bf0_si 0000  0016 0
ble_app  0000  0008 1:ble_app
===================
MessageQueue Info  
===================
msgqueue entry suspend thread
-------- ----  --------------
uisrv    0000  0
mq_guiap 0000  0
data_mb_ 0000  1:ds_mb
dserv    0000  1:ds_proc
test     0000  1:test
===================
Mutex Info         
===================
mutex      owner  hold suspend thread
-------- -------- ---- --------------
dserv    (NULL)   0000 0
tmalck   (NULL)   0000 0
alarmsvc (NULL)   0000 0
alm_mgr  (NULL)   0000 0
ulog loc (NULL)   0000 0
i2c_bus_ (NULL)   0000 0
i2c_bus_ (NULL)   0000 0
i2c_bus_ (NULL)   0000 0
i2c_bus_ (NULL)   0000 0
spi1     (NULL)   0000 0
===================
Semaphore Info     
===================
semaphore v   suspend thread
-------- --- --------------
app_tran 000 0
lv_data  001 0
lv_lcd   001 0
lv_epic  001 0
drv_lcd  000 0
fb_sem   000 0
lvlargef 001 0
lvlarge  001 0
btn      001 0
shrx     000 1:tshell
g_sifli_ 000 0
tma525b  000 1:touch_th
aw_tim   000 0
cons_be  000 0
ulog     150 0
heap     001 0
===================
Memory Info     
===================
total memory: 260784 used memory : 69096 maximum allocated memory: 96768
===================
MemoryHeap Info     
===================
memheap   pool size  max used size available size
-------- ---------- ------------- --------------
lvlargef 309172     301588        309124
lvlarge  2473392    2201700       2473344
=====================
 sp: 0x2006ec08
psr: 0x60000000
r00: 0x00000000
r01: 0x00000000
r02: 0x200bc8f8
r03: 0x0000002a
r12: 0x10069305
 lr: 0x100642e9
 pc: 0x10020bfa
=====================
fatal error on thread: app_watc?
```



#### 通过Ozone查看死机现场

如果日志打印没法分析出死机问题，则可以通过Ozone 这个Segger提供的调试工具分析。它在死机时比Keil更容易通过Jlink Attach到芯片(Keil的配置很容易使芯片重启，从而破坏死机现场).

> Ozone 在板子没有死机的情况下，也可以通过以下方法Attach上去，并单步调试，类似Keil的功能。但是它的栈解析好像不如Keil的好。
- 新建一个工程，选择适当的Device驱动(ButterFlier 是SF32LB6XX), CPU型号(ButterFlier 是CM33), 以及外设SVD文件(用于看外设寄存器内容，仅供Sifli内部使用，用户可以忽略，设为空)
![](../../assets/Ozone_debug_Step1.png)

- 下一步选择Jlink的连接方式，SWD接口
![](../../assets/Ozone_debug_Step2.png)

- 选择烧录程序的ELF文件，读取符号信息
![](../../assets/Ozone_debug_Step3.png)

- 工程建立完毕，然后将Ozone 通过Jlink Attach到死机的板子并且Halt住板子
![](../../assets/Ozone_debug_Step4.png)

- 然后就可以通过菜单里面的功能，单步调试、变量查看、栈解析等操作，跟Keil类似。
![](../../assets/Ozone_debug_Step5.png)



## 3. 日志接口

### 通过UART口作为日志输出
UART口的pinmux配置此处不做赘述，详见 [](../hal/uart.md)

如果使用的是RT-Thread RTOS, 则配置好UART的pinmux后，可以通过如下menuconfig的配置，选择不同的uart device
![](../../assets/config_rtt_console.png)

另外，SDK 采用ULOG作为通用的日志输出接口，详见 [](../middleware/logger.md)

### 通过Jlink作为日志输出
如果管脚不够用，则可以通过Jlink的RTT功能作为console口，配置步骤如下(SDK自带的RT-Thread RTOS已经集成了Segger RTT功能)：

1. 通过menuconfig，打开Segger RTT功能(它将自动注册一个名为segger的rt-device)
![](../../assets/jlink_trace_config_step1.png)

2. 将RT-Thread的默认console口指定到segger这个rt-device上
![]../../assets/jlink_trace_config_step2.png)

3. 通过Ozone连接到板子，如果已经指定了ELF文件Ozone将自动寻找RTT_Ctrlb，否则需要自己指定
![](../../assets/jlink_trace_config_step3.png)

## 4. 使用总线监视器

总线监视器可以监视总线上面的访问，当条件满足的时候，产生中断回掉，这个可以在调试中检测某块内存，或者外围设备的访问。

### 使能总线监视器
可以通过如下menuconfig的配置，使能总线控制器功能
![](../../assets/config_busmon.png)

### 使用总线监视器

在代码中，可以添加如下代码，实现特定的功能：
```c

void busmon_cbk()
{
    rt_kprintf("Busmon captured\n");        // 当总线特定访问时，产生回掉处理。用户可以在这里Assert，进一步调试分析。
}

...

    dbg_busmon_reg_callback(busmon_cbk);       // 注册回掉
    dbg_busmon_read(0x20080000,1);             // 第一次读取0x20080000地址的时候，触发总线监视器
    
    // 重新配置
    dbg_busmon_reg_callback(busmon_cbk);       // 注册回掉
    dbg_busmon_write(0x20080004,3);            // 第三次写0x20080004地址的时候，触发总线监视器

    // 重新配置
    dbg_busmon_reg_callback(busmon_cbk);       // 注册回掉
    dbg_busmon_write(0x20080008,2);            // 第二次读或者写0x20080008地址的时候，触发总线监视器

```

