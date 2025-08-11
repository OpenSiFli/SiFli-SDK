# IPC Queue Example
Source Code Path: example/multicore/ipc_queue
The IPC Queue example demonstrates how to use inter-core queues (IPC Queue) in SiFli-SDK to implement bidirectional communication between HCPU and LCPU. This example utilizes the SDK's IPC queue management API, supporting reliable inter-core message delivery, suitable for multi-core application scenarios requiring efficient inter-core communication.

## Usage
The following sections provide only essential information. For complete steps on configuring SiFli-SDK and building/running projects, please refer to the [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Supported Development Boards
This example can run on the following development boards:
- eh-lb551
- eh-lb555
- ec-lb583
- ec-lb587
- eh-lb561
- eh-lb563

### Hardware Requirements
This example requires no special hardware and can run directly on supported development boards.

### Compilation and Flashing
Execute the compilation command in the hcpu directory:
```bash
scons --board=<board_name> -j8
```
Where board_name is the board model. For example, to compile for the eh-lb561 board, use the complete command:
```bash
scons --board=eh-lb561 -j8
```
The compiled image files are stored in the HCPU's build_<board_name> directory. Use the following command to flash:
```bash
build_<board_name>/download.bat
```

### Console Configuration
- For 55x platforms, HCPU uses UART1 (second enumerated serial port) as console, LCPU uses UART3 as console (first enumerated serial port).
  For 58x platforms, HCPU uses UART1 (first enumerated serial port) as console, LCPU uses UART4 as console (third enumerated serial port).
- Send command `lcpu on` in HCPU console to start LCPU. After successful startup, you can see the startup log on LCPU console.
- Both HCPU and LCPU consoles can use the command `send message` to send a string to the other core,
  where `message` is the content to send. If the string contains spaces, it needs to be enclosed in double quotes.
  The other core will print the received string.
  For example, sending `send "Hello LCPU, this is HCPU"` in HCPU console will result in `rx: Hello LCPU, this is HCPU` being printed in LCPU console.

## Troubleshooting
- **Compilation Errors**: Ensure the SiFli-SDK development environment is properly configured and check that the board name is correct.
- **Flashing Failures**: Verify the development board is properly connected and try reconnecting the USB cable.
- **Communication Issues**: Check IPC queue configuration and confirm macro definitions in `src/common/ipc_config.h` and `linker_scripts/custom_mem_map.h`.

## Reference Documentation
- [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [Multi-core Communication Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)
- [IPC Queue API Documentation](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/api/ipc_queue.html)