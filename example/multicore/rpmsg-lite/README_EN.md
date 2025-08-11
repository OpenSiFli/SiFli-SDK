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
HCPU and LCPU automatically send messages to each other at regular intervals:
- When LCPU receives a message, it prints: `rx: hello_from_hcpu`
- When HCPU receives a message, it prints: `rx: hello_from_lcpu`
- The master core can send a string to the other core using the console command `send message`, where `message` is the content to send. If the string contains spaces, it must be enclosed in double quotes. The other core will print the received string.
```bash
# Send message in HCPU console
send "Hello LCPU, this is HCPU"
```
The LCPU console will display:
```
rx: Hello LCPU, this is HCPU
```

> To demonstrate sleep functionality, HCPU will enter sleep state after executing the send command, and LCPU will also sleep. Both will be woken up by timers or messages.

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [Multi-core Communication Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)