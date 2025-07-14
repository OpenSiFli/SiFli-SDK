
# Debugging and Logging

## 1. Hardware Interface
The SF32FB55X uses SWD as the debugging interface. Users can configure it to switch between HCPU or LCPU.

By default, the system powers up with HCPU. If you need to debug LCPU, you can use the SDK tool _$SDK_ROOT/tools/segger/jlink_lcpu_a0.bat_ to switch the SWD to LCPU.

Similarly, if SWD is currently connected to LCPU, you can use the SDK tool _$SDK_ROOT/tools/segger/jlink_hcpu_a0.bat_ to switch the SWD back to HCPU.

```{note} 
1. Since SWD uses PB IO, when debugging with SWD, make sure that LPSYS is in active or light sleep mode, whether the SWD is connected to HCPU or LCPU.<br>
2. Sending a reset command from JLink will not change the current CPU connected via SWD.<br>
3. After LPSYS wakes up from standby, SWD will automatically switch back to HCPU.
```

### LCPU Log Interface
The system ROM, during initialization, uses UART3 of LCPU as the console interface with a baud rate of 1,000,000 bps for logging and command input. It is recommended to retain this interface for LCPU logs.<br>

### HCPU Log Interface
The HCPU log interface can be selected between UART1/2 or SWD. If you need to use UART3/4/5 of LCPU, ensure that the LCPU is in an active state.

## 2. Debugging Methods
This section covers common methods for analyzing Assert or HardFault errors and solving system crashes. The JLink debugger is used for all methods.

### Setting Breakpoints
When JLink connects to HCPU/LCPU, the system is usually already initialized. If you need to debug initialization processes, such as cold starts or standby wake-ups, you need to halt the system as early as possible.  
We suggest users modify the system initialization code:
- HCPU  
  _$SDK_ROOT/drivers/cmsis/sf32lb55x/Templates/arm/startup_bf0_hcpu.S_  
- LCPU  
  _$SDK_ROOT/drivers/cmsis/sf32lb55x/Templates/arm/startup_bf0_lcpu.S_

In the `Reset_Handler`, uncomment the first instruction `;` to change it to:  
  B  .  
This will halt the CPU at the first instruction. Once JLink is connected, you can adjust the PC register (+2) and set breakpoints to debug the initialization process.

This method can also be used at other points to halt the system at a specific event and debug at the suspected location. In C files, you can add:  
  _asm("B .");  
This will stop the system at that instruction. Then, reconnect JLink, adjust the PC register (+2), and continue debugging.

```{note} 
Do not use `while(1);` because the system will optimize it and make the statements after `while(1)` ineffective.
```

### Assert/HardFault Error Analysis
When an error occurs, if the development board is connected via SWD to JLink, you can use the tool _$SDK_ROOT/tools/crash_dump_analyser/script/save_ram_a0.bat_ to save RAM, EPIC registers, and PSRAM content to the current path. This is helpful for analyzing the cause of the crash.

```{note} 
Make sure to add the JLink path to your Windows PATH environment variable, e.g., _C:/Program Files (x86)/SEGGER/JLink_v672b_, so you can load RAM via JLink to restore the crash context.
```

#### Analyzing Logs
By default, the SDK will output the breakpoint line and the last CPU registers during an Assert to the log interface. Analyze the contents of the log. Note that if the log interface is asynchronous, it may not output the complete log.

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

#### Analyzing Crash Context with Ozone

If the log analysis cannot pinpoint the crash issue, the Ozone debugging tool provided by Segger can be used. Ozone is easier to attach to the chip via JLink during a crash compared to Keil (as Keil configurations can easily reset the chip and destroy the crash context).

> Ozone can also be used in cases where the board is not crashed and can be attached for step-by-step debugging, similar to Keil, although its stack analysis seems to be less effective.

- Create a new project, select the appropriate device driver (ButterFlier is SF32LB6XX), CPU model (ButterFlier is CM33), and peripheral SVD files (used to view peripheral registers, only for internal Sifli use, users can ignore it).
![Ozone Debug Step 1](../../assets/Ozone_debug_Step1.png)

- Select the connection method for JLink, using SWD interface.
![Ozone Debug Step 2](../../assets/Ozone_debug_Step2.png)

- Choose the ELF file for flashing and read the symbol information.
![Ozone Debug Step 3](../../assets/Ozone_debug_Step3.png)

- Once the project is set up, attach Ozone to the crashed board via JLink and halt the board.
![Ozone Debug Step 4](../../assets/Ozone_debug_Step4.png)

- Now you can perform step-by-step debugging, variable inspection, stack analysis, etc., similar to Keil.
![Ozone Debug Step 5](../../assets/Ozone_debug_Step5.png)

## 3. Log Interface

### Logging via UART
The pinmux configuration for UART is not discussed here. Please refer to [](../hal/uart.md) for details.

If using RT-Thread RTOS, after configuring the UART pinmux, you can choose different UART devices through the following menuconfig configuration:
![RTT Console Configuration](../../assets/config_rtt_console.png)

Additionally, the SDK uses ULOG as the general logging output interface, detailed in [](../middleware/logger.md).

### Logging via JLink
If there are not enough pins available, you can use JLink's RTT functionality as a console interface. The configuration steps are as follows (Segger RTT functionality is integrated into the RT-Thread RTOS provided by the SDK):

1. Enable the Segger RTT functionality through menuconfig (it will automatically register an `rt-device` named `segger`).
![JLink RTT Configuration Step 1](../../assets/jlink_trace_config_step1.png)

2. Assign the default console of RT-Thread to the `segger` rt-device.
![JLink RTT Configuration Step 2](../../assets/jlink_trace_config_step2.png)

3. Connect to the board using Ozone. If the ELF file is already specified, Ozone will automatically search for `RTT_Ctrlb`. Otherwise, you need to specify it manually.
![JLink RTT Configuration Step 3](../../assets/jlink_trace_config_step3.png)

## 4. Using Bus Monitors

Bus monitors can track access on the bus and trigger interrupts or callbacks when specific conditions are met. This is useful for debugging memory or peripheral device accesses during development.

### Enabling the Bus Monitor
Enable the bus controller functionality through the following menuconfig configuration:
![Bus Monitor Config](../../assets/config_busmon.png)

### Using the Bus Monitor
In the code, you can add the following code to implement specific functionality:
```c
void busmon_cbk()
{
    rt_kprintf("Busmon captured
");        // Trigger callback when a specific bus access occurs. Users can assert here and further debug.
}

...

dbg_busmon_reg_callback(busmon_cbk);       // Register the callback
dbg_busmon_read(0x20080000,1);             // Trigger bus monitor on first read of address 0x20080000

// Reconfigure
dbg_busmon_reg_callback(busmon_cbk);       // Register the callback
dbg_busmon_write(0x20080004,3);            // Trigger bus monitor on third write of address 0x20080004

// Reconfigure
dbg_busmon_reg_callback(busmon_cbk);       // Register the callback
dbg_busmon_write(0x20080008,2);            // Trigger bus monitor on second read or write of address 0x20080008
```

