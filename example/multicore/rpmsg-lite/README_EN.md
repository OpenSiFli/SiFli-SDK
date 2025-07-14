# RPMsg-Lite Example
## Overview
This example demonstrates the basic usage of the RPMsg-Lite component.

## How to Use the Example
- RPMsg-Lite uses queue4 and queue5 as bidirectional communication channels. HCPU acts as master and LCPU acts as remote. The master's endpoint is 30, and the remote's endpoint is 40.
  The shared buffer must be allocated in LCPU's RAM, with the address specified by the macro `RPMSG_BUF_ADDR_MASTER`, and buffer size specified by the macro `RPMSG_BUF_SIZE`.
  These macros are defined in header files `src\common\ipc_config.h`, `project\hcpu\custom_mem_map.h`, and `project\lcpu\custom_mem_map.h`.
- It is recommended to initialize the RPMsg-Lite module during the `INIT_APP_EXPORT` stage to avoid premature mailbox interrupt activation affecting the data_service module.
- HCPU's main function is located in `src/hcpu/main.c`, and LCPU's main function is in `src/lcpu/main.c`.
- For compilation methods, refer to the general project compilation. For example, execute `scons --board=eh-lb551 -j8` in the `project\hcpu` directory to compile programs running on the eh-lb551 board, where --board is followed by the board name.
  To compile programs for 555hdk, execute the command `scons --board=eh-lb555 -j8`. After compilation, use the `build_eh-lb551/download.bat` command to download the bin file to the board.

## Example Output
- Select the serial ports corresponding to HCPU and LCPU logs according to the development board documentation. Some development boards use separate serial ports for HCPU and LCPU log output, while others multiplex the same serial port for log output.
- After power-on, HCPU automatically calls lcpu_power_on to start LCPU. After successful startup, boot logs can be seen on LCPU's console.
- HCPU and LCPU automatically send timed messages to each other. When LCPU receives a message, it prints `rx: hello_from_hcpu`. When HCPU receives a message, it prints `rx: hello_from_lcpu`.
- The big core can send strings to the other core through the console command `send message`, where `message` is the content to be sent. If the string contains spaces, it needs to be enclosed in double quotes.
  The other core will then print the received string.
  For example, sending `send "Hello LCPU, this is HCPU"` in HCPU's console will result in the print `rx: Hello LCPU, this is HCPU` appearing in LCPU's console.
  To demonstrate the sleep functionality, HCPU will enter sleep state after executing the send command, and LCPU will also go to sleep accordingly, each being woken up by timers or messages.