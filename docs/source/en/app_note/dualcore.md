
# Dual-Core Development

## 1. Introduction to Dual-Core

The SF32LB55X features a dual-core architecture with two Cortex MStar processors:
- **Main Core HCPU**  
  HCPU is responsible for high-performance computation and image processing, capable of running up to 240 MHz, with 1MB of memory.  
  HCPU has three QSPI interfaces that can connect to NOR Flash, NAND Flash, and PSRAM. The size of Flash and PSRAM varies depending on the package. Typically, code runs from NOR Flash.  
  HCPU has 64KB of ROM for boot-up purposes.
- **Sub-Core LCPU**  
  LCPU is a low-power core designed for tasks such as simple sensor data collection, buffering, basic processing, and low-power Bluetooth tasks. It runs at a maximum frequency of 48 MHz, with 256KB of memory.  
  Code typically runs from RAM, loaded by HCPU. LCPU also has a QSPI interface for connecting to NOR Flash and running code. However, due to pin limitations, not all packages allow access to this interface.  
  LCPU has 384KB of ROM, which contains the RT-Thread OS and the Bluetooth Low Energy (BLE) protocol stack developed by Sifli.  
  When LCPU is awake, HCPU can access LCPU's hardware modules and receive interrupts from all hardware modules.  
  When HCPU is awake, LCPU can use HCPU's hardware modules but cannot receive interrupts from these modules.  

For hardware resource details for HCPU and LCPU, please refer to [](../hal/index.md).  
During user design, it is essential to categorize different tasks based on the capabilities of each core and determine which core will handle which tasks and which hardware modules will be used.

## 2. Dual-Core Startup

### System Boot
After power-up, the SF32LB55X system starts from the HCPU. The ROM startup code reads the Flash table from the boot Flash and loads the specified user program to run. Typically, user code starts at the third 64K address. For detailed steps, refer to [](..\bootloader.md).  
After power-up, LCPU is in a powered wake-up state but not running. At this point, HCPU can use LCPU's hardware modules.

### Dual-Core Program Compilation
The LCPU program file is compiled as a constant table inside the HCPU program file. Refer to the example in _$SDK_ROOT/example/multicore/ipc_queue_.  
When compiling the LCPU project, in addition to generating the LCPU program file, the resulting BIN file is converted into a _lcpu_img.c_ file, which is stored in the HCPU project directory. This file uses a constant table to define the LCPU binary code and implements the _lcpu_img_install_ function to load the LCPU code into RAM.

The LCPU compile script uses the SDK tool _$SDK_ROOT/tools/patch/gen_src.py_ to convert the generated LCPU BIN file into a C array and implement the installation function. The usage of the _gen_src.py_ script is as follows:
```python 
    python gen_src.py lcpu <usr image.bin> <output folder> 
```

### LCPU Code Loading
The LCPU code is loaded by HCPU, copied into LCPU's RAM, and configured with LCPU's Stack and PC before starting the LCPU to run.

Taking _$SDK_ROOT/example/multicore/ipc_queue_ as an example, to start LCPU, run the `lcpu on` command from HCPU's console. This command calls the `lcpu_power_on` function, which invokes `lcpu_img_install` to transfer the code into LCPU's RAM, and configures the registers to start LCPU.

## 3. LCPU Code Development
As introduced above, compared to HCPU, LCPU operates at a lower frequency (48MHz) and has less RAM (256KB). Additionally, the BLE protocol stack runs on LCPU. Therefore, the development of LCPU code primarily focuses on the following aspects:
- Function planning
- Memory planning
- Dual-core data interaction

### Function Planning
LCPU's main advantage over HCPU is its lower power consumption, and the core focus of function planning is on power consumption. This can be divided into Bluetooth and non-Bluetooth parts:
- **Bluetooth part**: The SF32LB55X protocol stack runs on LCPU, and on top of the protocol stack, the SF32LB55X SDK provides a simpler Bluetooth service layer. The Bluetooth service layer can be configured via menuconfig to run on either LCPU or HCPU.  
  Running on LCPU allows Bluetooth data exchange with the peer device without waking up HCPU. However, it requires consideration of cross-core operations and Bluetooth state synchronization between the two cores.  
  Running on HCPU eliminates the need to manage cross-core data operations but results in higher power consumption due to the use of HCPU for all Bluetooth operations.
- **Non-Bluetooth part**: Suitable for tasks that require frequent wake-ups with relatively simple computations, such as sensor data collection and basic data processing, which can take full advantage of LCPU's low power.  
  It is important to note that LCPU's RAM is limited, so careful memory planning is needed.

### Memory Planning
LCPU has 256KB of RAM when no external flash is connected. The overall memory layout can be referenced from [](../app_note/memory_usage.md). The 256KB includes all RO (read-only) and RW (read-write) data, so careful planning is required.  
LCPU's ROM includes the RT-Thread OS, BLE protocol stack, and most of the HAL interfaces. Developers can choose to call ROM functions to save RO/RW space. However, both ROM and the Bluetooth protocol stack consume some RAM, and specific memory planning is required as follows:

| Name               | Start Address   | End Address     | Size (bytes)  | Description |
|--------------------|-----------------|-----------------|---------------|-------------|
| ROM used RAM area  | 0x20124000      | 0x20130000      | 48*1024       |             |
| BLE stack area     | 0x20130000      | 0x20138000      | 32*1024       |             |

#### ROM Explanation
LCPU uses a symbol definition (symdefs) file mechanism, and the LCPU RAM project can access ROM functions by importing the ROM sym file. The ROM sym file includes:
- The entire RT-Thread OS
- Supported HAL modules for LCPU
- Some drivers (alarm, flash, i2c, spi, timer)
- BLE protocol stack API

The RAM project only needs to configure the relevant modules and enable the LCPU_ROM macro. When compiling, if a function name conflicts with ROM functions, it will link directly to the ROM functions. Developers can also specify whether the final linked function is from ROM or RAM by restoring deleted symbols from the sym file.

The following steps outline how to add symdefs:
1. In the Link options, specify the ROM symdefs file to obtain the ROM symbol table.
    - The ROM symdefs file is located in __$SDK_ROOT/example/rom_bin/lcpu_boot_loader/rom.sym__. It can be copied to the LCPU project directory before LCPU compilation through the prebuild step.
    - Add `CUSTOM_LFLAGS = 'rom.sym'` in the project's _rtconfig.py_ file, and SCONS will automatically include the _rom.sym_ in the Link options.
2. Enable the LCPU ROM macro in the Kconfig of the LCPU project:
```python
     config LCPU_ROM 
     bool 
     default y
```

#### Notes:
1. ROM functions are read-only and cannot be configured through menuconfig options.
2. ROM functions only use global variables defined in ROM. RAM projects can choose to use either RAM or ROM variables. Special attention should be given when modifying the sym file to avoid unexpected behavior where ROM may not work with RAM-defined global variables.

### Dual-Core Data Interaction
1. **Communication method**: Refer to the next section on inter-core communication for details.
2. **Communication nodes**: As LCPU is started by HCPU, LCPU's startup time will always be later than HCPU's, depending on when HCPU starts LCPU.
   - Both cores operate independently according to the operating system's startup sequence.
   - Inter-core communication uses shared memory to exchange data. As long as data is not overwritten before the receiving module is initialized and ready to handle it, data can be exchanged.  
     For example, if HCPU wants to send data to LCPU through a data service, as long as the receiving side in LCPU has registered the data service before LCPU begins receiving data, HCPU can send data after triggering LCPU power-on without waiting for LCPU's state update.
3. **Other considerations**:
   - Data exchange based on inter-core communication will automatically wake up the other core for data interaction. The sleep state of the other core does not need to be considered.

## 4. Inter-Core Communication
There are three levels of communication between HCPU and LCPU. Users can choose different interfaces based on their needs, ranked from low to high:

- **Mailbox**  
  HCPU and LCPU can access each other's memory, but the access speed is slower than local memory, and it is necessary to ensure that the other core is awake.  
  The same memory has different addresses in HCPU and LCPU. The SDK provides functions like HCPU_ADDR_2_LCPU_ADDR and LCPU_ADDR_2_HCPU_ADDR to convert between different address spaces. For detailed address space information, refer to [](../app_note/memory_usage.md).  
  The Mailbox provides interrupt trigger functionality. When one CPU needs the other CPU to perform a task, it triggers the other CPU's Mailbox interrupt, which will then handle the interrupt. Detailed API can be found in [](../hal/mailbox.md).  
  Both cores can agree on a shared memory address to exchange data, but data sent from HCPU to LCPU must be stored in HCPU's RAM, and vice versa. This is because LCPU may be in sleep mode before receiving the Mailbox interrupt, and HCPU cannot access LCPU's memory.

- **IPC Queue/IPC Queue Device**  
   - IPC Queue is built on top of Mailbox, wrapping a circular buffer and interrupt service for easier data exchange between cores.
   - IPC Queue is OS-independent but supports the RT-Thread low-power framework. The sender does not enter sleep mode before the buffer is read.
   - IPC Queue Device builds on IPC Queue, implementing an RT-Thread device interface for console redirection.
   - The usage of IPC Queue can be found in _$SDK_ROOT/example/multicore/ipc_queue_.

- **Data Service**  
   - Data Service builds on IPC Queue and offers a client-server distributed data service architecture. The service provider (server) typically runs on LCPU, and the client (user) runs on HCPU.
   - The connection between them is provided by the Data Service framework. Both the provider and the user can be developed and simulated independently, especially for LCPU code, which can be debugged on HCPU before final integration and validation on the actual core.
   - Specific usage can be found in _$SDK_ROOT/example/multicore/data_service_.

## 5. Debugging and Logging
Please refer to [](../app_note/debug_logging.md).
