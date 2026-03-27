# RPMsg-Lite Example

(For a general overview of the examples and their usage, please refer to the
`README.md` file in the parent "examples" directory.)

Source Path: example/multicore/rpmsg-lite

The RPMsg-Lite example demonstrates bidirectional communication between the HCPU
and LCPU within the SiFli-SDK. Leveraging the SDK's RPMsg-Lite API, the example
utilizes shared memory and mailbox interrupt mechanisms for inter-processor
communication (IPC), making it suitable for multi-core applications requiring
high-efficiency data transfer.

RPMsg-Lite is a lightweight Remote Processor Messaging protocol that supports
asynchronous cross-core communication, using an endpoint mechanism to achieve
multi-channel communication isolation.
## Overview

### Supported Boards
This example can be executed on the following development boards:
- eh-lb551
- eh-lb555
- sf32lb56-lcd series
- sf32lb58-lcd series

The following sections provide only essential information. For complete steps on
configuring the SiFli-SDK and using it to build and run projects, please refer
to the [SiFli-SDK Quick Start
Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Hardware Requirements
No special hardware is required; this example runs directly on the supported
development boards.

### Project Configuration
- RPMsg-Lite utilizes queue4 and queue5 as bidirectional communication channels,
  where the HCPU acts as the master and the LCPU as the remote. The master
  endpoint is set to 30, and the remote endpoint is 40. The shared buffer must
  be allocated in the LCPU RAM. The buffer address is specified by the
  `RPMSG_BUF_ADDR_MASTER` macro, and the size is defined by the `RPMSG_BUF_SIZE`
  macro. These macros are defined in `src\common\ipc_config.h`,
  `project\hcpu\custom_mem_map.h`, and `project\lcpu\custom_mem_map.h`.
- It is recommended to initialize the RPMsg-Lite module during the
  `INIT_APP_EXPORT` stage to prevent early mailbox interrupts from affecting the
  data_service module.
- The HCPU main function is located in `src/hcpu/main.c`, and the LCPU main
  function is in `src/lcpu/main.c`.

### Compilation and Flashing
Refer to the standard project compilation process. Execute the following in the
`project/hcpu` directory:
```bash
scons --board=<board_name> -j8</board_name>
```
The `board_name` parameter specifies the target board. For example, to compile
the program for the `eh-lb551` board, use the following command:
```bash
scons --board=eh-lb551 -j8
```
After compilation is complete, use the following command to flash the binary
file to the board:
```bash
build_<board_name>/download.bat</board_name>
```

### Console Configuration
- Select the serial ports for HCPU and LCPU logging according to the development
  board documentation. Some boards use separate serial ports for HCPU and LCPU
  logs, while others multiplex a single serial port for both.
- Upon power-up, the HCPU automatically invokes `lcpu_power_on` to boot the
  LCPU. Once the boot process completes, the startup logs will be displayed on
  the LCPU console.

## How to Use the Example
The HCPU and LCPU exchange periodic messages automatically:
- RPMsg-Lite uses queue4 and queue5 as bidirectional communication channels.
  HCPU acts as master and LCPU acts as remote. The master's endpoint is 30, and
  the remote's endpoint is 40. The shared buffer must be allocated in LCPU's
  RAM, with the address specified by the macro `RPMSG_BUF_ADDR_MASTER`, and
  buffer size specified by the macro `RPMSG_BUF_SIZE`. These macros are defined
  in header files `src\common\ipc_config.h`, `project\hcpu\custom_mem_map.h`,
  and `project\lcpu\custom_mem_map.h`.
- It is recommended to initialize the RPMsg-Lite module during the
  `INIT_APP_EXPORT` stage to avoid premature mailbox interrupt activation
  affecting the data_service module.
- HCPU's main function is located in `src/hcpu/main.c`, and LCPU's main function
  is in `src/lcpu/main.c`.
```bash
# Send a message from the HCPU console
send "Hello LCPU, this is HCPU"
```
The LCPU console will display:
```
rx: Hello LCPU, this is HCPU
```

> To demonstrate power management features, the HCPU enters a sleep state after
> executing the `send` command, and the LCPU follows suit. Both processors can
> only be awakened by timers or incoming messages; they cannot be awakened via
> the `send` command.

## Example Output
- When LCPU receives a message, it prints: `rx: hello_from_hcpu`
- When HCPU receives a message, it prints: `rx: hello_from_lcpu`

