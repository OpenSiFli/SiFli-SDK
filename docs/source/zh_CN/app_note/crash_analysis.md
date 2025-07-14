# 死机分析指南

## 1. 介绍

SDK集成了一些工具用于分析由assert或hardfault引起的死机问题，为了便于分析内存泄漏，需要打开`RTOS->RT-Thread Kernel->Memory Management`中的`Enable memory trace`

![](../../assets/crash_analysis_mem_trace.png)

下面时ASSERT发生时的打印信息，显示ASSERT发生的函数和行号，以及线程、消息队列和heap等信息。
```
16:21:48:257        Assertion failed at function:wait_power_on_anim_done, line number:32 ,(RT_EOK == err)
16:21:48:258        ===================
16:21:48:258        Thread Info        
16:21:48:259        ===================
16:21:48:260        thread          pri  status      sp      top     stack size max used left tick  error
16:21:48:260        --------------- ---  ------- ---------- ---------- ----------  ------  ---------- ---
16:21:48:261        power_on_thread  18  suspend 0x200a342c 0x200a3520 0x00002800    05%   0x00000013 000
16:21:48:262        tpread           10  suspend 0x20003acc 0x20003bfc 0x00000800    25%   0x00000008 000
16:21:48:262        tshell           20  suspend 0x200a04fc 0x200a0670 0x00001000    09%   0x0000000a 000
16:21:48:263        app_bg           19  suspend 0x200042d4 0x200043fc 0x00000800    48%   0x00000001 000
16:21:48:264        app_watch        19  ready   0x2008f9e4 0x2008faec 0x00004000    06%   0x00000003 000
16:21:48:264        ds_proc          14  suspend 0x20037e14 0x20037f30 0x00001000    26%   0x00000002 000
16:21:48:265        ds_mb            13  suspend 0x20036e24 0x20036f30 0x00001000    51%   0x00000001 000
16:21:48:266        nvds_srv         20  suspend 0x20002ee4 0x20002ffc 0x00000800    33%   0x00000005 000
16:21:48:266        alarmsvc         12  suspend 0x200993f4 0x20099464 0x00000800    06%   0x00000005 000
16:21:48:267        tidle            31  ready   0x20096914 0x20096954 0x00000400    06%   0x0000001a 000
16:21:48:268        timer             4  suspend 0x200033bc 0x200033fc 0x00000400    06%   0x00000009 000
16:21:48:268        main             10  suspend 0x20002714 0x200027fc 0x00000800    34%   0x0000000f 000
16:21:48:269        ===================
16:21:48:269        Mailbox Info       
16:21:48:270        ===================
16:21:48:271        mailbox entry size suspend thread
16:21:48:271        ------- ----  ---- --------------
16:21:48:272        ===================
16:21:48:273        MessageQueue Info  
16:21:48:274        ===================
16:21:48:274        msgqueue             entry suspend thread
16:21:48:275        -------------------- ----  --------------
16:21:48:276        mq_guiapp            0001  0
16:21:48:276        app_preprocess_queue 0000  1:app_bg
16:21:48:277        application_lv_mq    0000  0
16:21:48:277        data_mb_mq           0000  1:ds_mb
16:21:48:278        dserv                0000  1:ds_proc
16:21:48:279        nvds_srv             0000  1:nvds_srv
16:21:48:279        ===================
16:21:48:280        Mutex Info         
16:21:48:281        ===================
16:21:48:281        mutex          owner  hold suspend thread
16:21:48:282        ------------ -------- ---- --------------
16:21:48:282        app_db       (NULL)   0000 0
16:21:48:283        app_db       (NULL)   0000 0
16:21:48:284        tplck        (NULL)   0000 0
16:21:48:284        hw_alarm     (NULL)   0000 0
16:21:48:285        ui_pm        (NULL)   0000 0
16:21:48:286        fat1         (NULL)   0000 0
16:21:48:286        fat0         (NULL)   0000 0
16:21:48:287        ds_ipc       (NULL)   0000 0
16:21:48:288        dserv        (NULL)   0000 0
16:21:48:288        flash_mutex  (NULL)   0000 0
16:21:48:289        alarmsvc     (NULL)   0000 0
16:21:48:290        ulog lock    (NULL)   0000 0
16:21:48:291        fslock       (NULL)   0000 0
16:21:48:291        i2c_bus_lock (NULL)   0000 0
16:21:48:292        ===================
16:21:48:293        Semaphore Info     
16:21:48:293        ===================
16:21:48:294        semaphore           v   suspend thread
16:21:48:295        ------------------- --- --------------
16:21:48:296        lv_data             001 0
16:21:48:297        power_on_anim       000 0
16:21:48:298        dfu_gui_epic        000 0
16:21:48:299        lv_lcd              001 0
16:21:48:300        drv_lcd             000 0
16:21:48:301        ui_pm               000 0
16:21:48:301        shrx                000 0
16:21:48:302        message_falsh       001 0
16:21:48:303        poweron             000 0
16:21:48:304        llt                 001 0
16:21:48:305        app_ft_memheap      001 0
16:21:48:305        app_message_memheap 001 0
16:21:48:311        ft3168              000 1:tpread
16:21:48:311        lv_copy             000 0
16:21:48:311        epic                001 0
16:21:48:312        flash1              001 0
16:21:48:312        heap                001 0
16:21:48:313        ===================
16:21:48:313        Memory Info     
16:21:48:313        ===================
16:21:48:314        total memory: 409740 used memory : 73808 maximum allocated memory: 83516
16:21:48:314        ===================
16:21:48:314        MemoryHeap Info     
16:21:48:315        ===================
16:21:48:315        memheap              pool size  max used size available size
16:21:48:316        ------------------- ---------- ------------- --------------
16:21:48:316        llt                 8192       2332          5860 
16:21:48:317        app_ft_memheap      160000     9052          150948
16:21:48:317        app_message_memheap 18000      80            17920
16:21:48:317        =====================
16:21:48:318        PSP: 0x2008fa50, MSP: 0x20090ad0
16:21:48:318        =====================
16:21:48:318         sp: 0x2008fab0
16:21:48:319        psr: 0x40000000
16:21:48:319        r00: 0x00000000
16:21:48:319        r01: 0x40002000
16:21:48:320        r02: 0x00000000
16:21:48:320        r03: 0x00000000
16:21:48:320        r12: 0x10106ab9
16:21:48:320         lr: 0x1010012d
16:21:48:321         pc: 0x10060c1a
16:21:48:321        =====================
16:21:48:321        fatal error on thread: app_watch

```


hardfault发生时会打印如下信息，最后打印hardfault的类型，如果busfault、mem manage fault等，
下面的例子为mem manage fault的死机，`DACCVIOL SCB->MMAR:00000000`表示MPU发现了对0地址的非法访问，访问它的指令地址由pc寄存器记录，是0x100c6426

```
00:48:26:197         sp: 0x200a00d8
00:48:26:199        psr: 0x41000000
00:48:26:203        r00: 0x00000001
00:48:26:206        r01: 0x000000ff
00:48:26:208        r02: 0x00000000
00:48:26:212        r03: 0x00000000
00:48:26:215        r04: 0x00000008
00:48:26:217        r05: 0x00000000
00:48:26:221        r06: 0x00000000
00:48:26:224        r07: 0x00000000
00:48:26:226        r08: 0x000000ff
00:48:26:231        r09: 0x0000000c
00:48:26:233        r10: 0x00000000
00:48:26:236        r11: 0x6020641c
00:48:26:238        r12: 0x10137332
00:48:26:240         lr: 0x00000000
00:48:26:244         pc: 0x100c6426
00:48:26:247        hard fault on thread: app_watch
00:48:26:250        
00:48:26:252        =====================
00:48:26:256        PSP: 0x200a0048, MSP: 0x200a1544
00:48:26:259        ===================
00:48:26:264        Thread Info        
00:48:26:265        ===================
00:48:26:269        thread      pri  status      sp      top     stack size max used left tick  error
00:48:26:272        ----------- ---  ------- ---------- ---------- ----------  ------  ---------- ---
00:48:26:275        tpread       10  suspend 0x20002acc 0x20002bfc 0x00000800    17%   0x00000001 000
00:48:26:277        tshell       20  suspend 0x200bd824 0x200bd994 0x00001000    11%   0x00000008 000
00:48:26:282        app_bg       19  suspend 0x200032d4 0x200033fc 0x00000800    35%   0x00000001 000
00:48:26:285        app_watch    19  ready   0x200a0374 0x200a0578 0x00004000    52%   0x00000007 -02
00:48:26:288        g_sifli_tid  12  suspend 0x200b3534 0x200b363c 0x00001000    16%   0x00000003 000
00:48:26:290        ds_proc      14  suspend 0x200052e4 0x200053fc 0x00001000    17%   0x0000000a 000
00:48:26:295        ds_mb        13  suspend 0x200042f4 0x200043fc 0x00001000    06%   0x00000005 000
00:48:26:297        alarmsvc     12  suspend 0x200b5d14 0x200b5d88 0x00000800    05%   0x00000005 000
00:48:26:303        tidle        31  ready   0x200b1fcc 0x200b2094 0x00000200    80%   0x0000001e 000
00:48:26:305        timer         4  suspend 0x200023bc 0x200023fc 0x00000400    17%   0x00000009 000
00:48:26:312        main         10  suspend 0x200b1a14 0x200b1afc 0x00000800    11%   0x0000000f 000
00:48:26:314        ===================
00:48:26:319        Mailbox Info       
00:48:26:321        ===================
00:48:26:325        mailbox        entry size suspend thread
00:48:26:328        -------------- ----  ---- --------------
00:48:26:331        g_bf0_sible_mb 0000  0016 0
00:48:26:333        ===================
00:48:26:336        MessageQueue Info  
00:48:26:339        ===================
00:48:26:343        msgqueue             entry suspend thread
00:48:26:347        -------------------- ----  --------------
00:48:26:351        mq_guiapp            0000  0
00:48:26:353        app_preprocess_queue 0000  1:app_bg
00:48:26:356        application_lv_mq    0000  0
00:48:26:359        data_mb_mq           0000  1:ds_mb
00:48:26:361        dserv                0000  1:ds_proc
00:48:26:365        ===================
00:48:26:368        Mutex Info         
00:48:26:370        ===================
00:48:26:372        mutex          owner  hold suspend thread
00:48:26:376        ------------ -------- ---- --------------
00:48:26:378        app_db       (NULL)   0000 0
00:48:26:383        app_db       (NULL)   0000 0
00:48:26:385        tplck        (NULL)   0000 0
00:48:26:390        hw_alarm     (NULL)   0000 0
00:48:26:393        ui_pm        (NULL)   0000 0
00:48:26:396        fat1         (NULL)   0000 0
00:48:26:398        fat0         (NULL)   0000 0
00:48:26:402        ds_ipc       (NULL)   0000 0
00:48:26:405        dserv        (NULL)   0000 0
00:48:26:408        flash_mutex  (NULL)   0000 0
00:48:26:410        alarmsvc     (NULL)   0000 0
00:48:26:415        ulog lock    (NULL)   0000 0
00:48:26:418        fslock       (NULL)   0000 0
00:48:26:421        i2c_bus_lock (NULL)   0000 0
00:48:26:422        ===================
00:48:26:426        Semaphore Info     
00:48:26:428        ===================
00:48:26:431        semaphore            v   suspend thread
00:48:26:434        -------------------- --- --------------
00:48:26:437        flash2               001 0
00:48:26:442        flash1               001 0
00:48:26:445        app_trans            000 0
00:48:26:448        lv_data              001 0
00:48:26:449        lv_lcd               001 0
00:48:26:454        cfbdma               000 0
00:48:26:456        drv_lcd              001 0
00:48:26:459        ui_pm                000 0
00:48:26:461        shrx                 000 0
00:48:26:465        message_falsh        001 0
00:48:26:470        poweron              000 0
00:48:26:472        btn                  001 0
00:48:26:476        g_sifli_sem          000 0
00:48:26:479        llt                  001 0
00:48:26:483        app_ft_memheap       001 0
00:48:26:485        app_message_memheap  001 0
00:48:26:489        app_image_psram_memh 001 0
00:48:26:492        it7259e              000 1:tpread
00:48:26:494        lv_copy              000 0
00:48:26:499        epic                 001 0
00:48:26:502        heap                 001 0
00:48:26:504        ===================
00:48:26:507        Memory Info     
00:48:26:510        ===================
00:48:26:513        total memory: 292712 used memory : 258480 maximum allocated memory: 292324
00:48:26:515        ===================
00:48:26:519        MemoryHeap Info     
00:48:26:521        ===================
00:48:26:524        memheap               pool size  max used size available size
00:48:26:526        -------------------- ---------- ------------- --------------
00:48:26:531        llt                  8192       2732          5760 
00:48:26:538        app_ft_memheap       400000     305016        213016
00:48:26:541        app_message_memheap  96000      80            95920
00:48:26:544        app_image_psram_memh 1100000    801444        430952
00:48:26:546        FPU active!
00:48:26:550        mem manage fault:
00:48:26:553        SCB_CFSR_MFSR:0x82 DACCVIOL SCB->MMAR:00000000

```


## 2. 准备工作
对于ASSERT类型的死机，可以从打印大致知道问题发生的位置，但对于hardfault死机或者比较复杂的死机，这些信息就不够了，需要借助工具获取更多线索。一个方法是使用调试器attach上被测设备，
查看全局变量和memory（如果是hardfault，attach上之后需要用打印出来的SP/LR/PC改写当前的寄存器，就能看到函数调用栈了，如果是ASSERT，不需要修改寄存器也能看到函数调用栈），
但这样就会占用了被测设备，也不利于多人一起分析。SDK提供的`crash_dump_analyser`工具可以保存和恢复问题现场，开发人员在PC上也可以分析问题，而不需要连上目标设备。
所需的工具有：
- JLink仿真器和JLink软件包
- _$SDK_ROOT/tools/crash_dump_analyser/script_：现场保存和恢复脚本
- _$SDK_ROOT/tools/crash_dump_analyser/simarm/t32marm.exe_：执行现场恢复脚本的Trace32软仿工具

## 3. 保存现场 
### 通过BAT脚本保存现场
以55x芯片为例：
- 连接JLink仿真器到目标板(没有Jlink的芯片需要打开 _SifliUsartServer.exe_ 用DBGUART模拟Jlink)
- 双击执行 _tools/crash_dump_analyser/script/save_ram_55x.bat_ 读取目标板的数据，
- 也可以在命令行，以watch_demp为例，在SDK根目录，调用 _SDK_ROOT/tools/crash_dump_analyser/script/save_ram_55x.bat_ ， _$SDK_ROOT/example/watch_demo/project/eh-lb555/build_ 这样可以把生成文件放入 _SDK_ROOT/example/watch_demo/project/eh-lb555/build_

成功后会生成以下几个文件(由对应的sf32lb55x.jlink的内容而定)：
- _hcpu_ram.bin_：1Mbyte的HCPU RAM数据
- _psram.bin_: 32Mbyte的PSRAM数据
- _ret_ram.bin_: 64Kbyte的retention RAM数据
- _hcpu_itcm.bin_: 16Kbyte的retention RAM数据
- _epic_reg.bin_: EPIC寄存器
- _ezip_reg.bin_: EZIP寄存器
- _dsi_host_reg.bin_: DSI HOST寄存器
- _dsi_phy_reg.bin_: DSI HOST寄存器
- _dsi_phy_reg.bin_: DSI HOST寄存器
- _dsi_phy_reg.bin_: DSI HOST寄存器
- _gpio1_reg.bin_: GPIO1寄存器
- _gpio2_reg.bin_: GPIO2寄存器
- _lcpu_ram.bin_: 224Kbyte的LCPU RAM数据
- _lcpu_dtcm.bin_: 16Kbyte的LCPU DTCM数据

### 通过AssertDumpUart工具保存现场
该工具直接连接debuguart口，然后执行对应的jlink脚本保存现场，无需 _SifliUsartServer.exe_ 模拟Jlink.
以52x芯片为例：
- 打开 _$SDK_ROOT/tools/crash_dump_analyser/script/AssertDumpUart.exe_
- 设置对应的jlink脚本、芯片型号、串口号、波特率、串口设备（注意是MCU上的USART设备，HCPU一般是UART1, LCPU是UART4）
- 点击导出即可

![](../../assets/crash_analysis_AssertDumpUsart.png)


## 4. 恢复现场
### 4.1 HCPU恢复现场
#### 双击运行t32marm.exe,如下图：

![](../../assets/crash_analysis_default_view.png)


#### 点击HA按钮(HCPU assertion)
- 选择当前的芯片，设置前面保存现场导出的bin所在的路径(注意路径最后没有带斜杠)，以及手动放置的axf文件，来查看HCPU的死机现场。
- 如果有些bin不存在（例如有的dump没有PSRAM），可以勾掉。

![](../../assets/crash_analysis_load_hcpu_assertion_ui.png)

#### 点击 “run_next_step”按钮加载
加载成功后显示下图的现场信息

![](../../assets/crash_analysis_hcpu_window.png)

可以在Window菜单切换显示的窗口

![](../../assets/crash_analysis_hcpu_window_select.png)

heapAllocation窗口显示了系统中所有heap pool的分配情况，包括system heap以及memheap_pool：
- system heap：rt_malloc和lv_mem_alloc使用的pool
- 各个memheap_pool: 使用`rt_memheap_init`创建的pool，分配和释放使用`rt_memheap_alloc`和`rt_memheap_free`


分配信息列表中的字段含义为：
- BLOCK_ADDR: 分配的内存块的起始地址，包括了管理项
- BLOCK_SIZE: 申请的内存大小，不包括管理项长度
- USED：是否已分配，1表示已分配，0表示未分配
- TICK: 申请时间，单位为OS tick，即1ms
- RETURN ADDR: 申请者地址

#### 没有显示异常栈的处理
做完前面3个步骤，有时候不会显示死机的现场栈，可能是dump内容中没有保存或者保存的异常，可以尝试以下2种办法：
- 从Jlink halt的log信息加载现场栈
HR(HCPU Registers)按钮用于恢复没有走到异常处理程序的CPU寄存器
点击按钮后选择导出现场的 _log.txt_ 文件，他将把里面HCPU的16个寄存器回填到trace32
![](../../assets/crash_analysis_toolsbar_HR.png)

- 从log里面打印的16个寄存器中，回填到trace32的register窗口中
![](../../assets/crash_analysis_restore_registers_from_log.png)

### 4.2 LCPU恢复现场

与HCPU的恢复现场类似，先将需要用到的文件copy到scripy目录中来(lcpu.axf和rom_axf文件)
![](../../assets/add4.png)

需要的文件路径如下图，rom_axf文件按照自己的型号选择对应文件即可，lcpu文件要根据自己是keil编译还是gcc编译来选择
#### 需要注意：！！！
-  keil编译出来的lcpu文件后缀名为axf，gcc编译出来的lcpu文件后缀名为elf，注意区分
-  选择rom_axf文件时注意区分板子型号
![](../../assets/add2.png)
![](../../assets/add3.png)

再打开Trace32选择LA按钮，在弹出的窗口中进行如下配置：
![](../../assets/add1.png)



## 5. Trace32常用命令

除了已打开的窗口，可以使用View菜单打开新的窗口，如下图所示
- Registers：查看CPU寄存器
- Dump：查看指定地址数据
- List Source: 查看汇编代码
- Watch：查看变量

![](../../assets/crash_analysis_cmd_view.png)


变量窗口支持通配符，例如输入*error*rea*并回车，会提示error_reason，双击可选择变量假如watch窗口

![](../../assets/crash_analysis_cmd_watch.png)

![](../../assets/crash_analysis_cmd_watch2.png)

也可以在下方的命令窗口输入命令（命令不区分大小写）打开调试窗口

![](../../assets/crash_analysis_cmd_input.png)


例如：
- V.W: 打开watch窗口
- D.DUMP address: 查看指定地址数，例如D.DUMP 20000000， 查看0x20000000地址开始的数据
- L address/symbol：查看指定地址的汇编，例如L 1011D888，打开汇编窗口显示0x1011D888地址开始的汇编代码；`L rt_thread_stack_restore`， 打开汇编窗口显示函数`rt_thread_stack_restore`的汇编代码。


## 6. HEAP分析示例

下图为一个检测到heap有泄漏的现场，callstack窗口显示了assert的函数调用栈，heapAllocation窗口的system heap列表显示由`rt_malloc`申请的内存块的列表，
RETURN ADDR显示调用`rt_malloc`的函数名，TICK为申请时的`rt_tick_get`时间，

![](../../assets/crash_analysis_heap_callstack.png)

![](../../assets/crash_analysis_heap_detail.png)


System heap内存管理项的结构如图所示，第一个uint16为特殊字0x1EA0，所有内存管理项都是这个值，如果该变量的值不是0x1EA0，
那么就是被非法改写了，第二个uint16为used标志，1表示已分配，0表示未分配，如果出现0和1以外值，也意味着发生了非法改写，也会造成申请不到内存的假象。

![](../../assets/crash_analysis_heap_struct.png)


例如HEAP窗口中第一行地址为0x200A27EC的内存块，是由函数`rt_serial_open`中的指令地址0x1011B5FB前的指令申请，申请的内存大小为4108字节。
由`strutc heap_mem`结构可知，System heap管理项长度为28字节，所以由内存块起始地址偏28字节就是申请者使用的内存地址，
例如下图中`_lv_ll_ins_head`函数申请了88字节的内存，内存块起始地址为0x200B08E0，在变量查看窗口中可以用(lv_obj_t*)(0x200B08E0+28)查看这个变量的值，
分析LVGL代码可知只有lv_obj_t的大小是88字节（还需要加上8字节的LVGL链表的关联项，跟在了lv_obj_t之后）。
signal_cb是函数地址，在窗口最下方的命令行中输入L 100DC9A1命令，
打开反汇编窗口显示该地址对应的汇编代码，可知这个函数是lv_img_signal，所以是lv_img控件申请的内存。当出现内存泄漏时，可以结合申请者地址和申请时间分析哪个地方申请了内存但没有释放。

![](../../assets/crash_analysis_heap_example.png)


