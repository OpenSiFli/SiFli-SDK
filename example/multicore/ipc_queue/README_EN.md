# IPC Queue Example
- For 55x platform, HCPU uses UART1 (second enumerated serial port) as console, and LCPU uses UART3 as console (first enumerated serial port).
  For 58x platform, HCPU uses UART1 (first enumerated serial port) as console, and LCPU uses UART4 as console (third enumerated serial port).
- Send the command `lcpu on` in HCPU's console to start LCPU. After successful startup, startup logs can be seen on LCPU's console.
- Both HCPU and LCPU consoles can use the command `send message` to send strings to the other core.
  `message` is the content to be sent. If the string contains spaces, it needs to be enclosed in double quotes.
  The other core will then print the received string.
  For example, sending `send "Hello LCPU, this is HCPU"` in HCPU's console will result in the print `rx: Hello LCPU, this is HCPU` appearing in LCPU's console.

## Project Description
- The project supports the following development boards:
    - eh-lb551
    - eh-lb555
    - ec-lb583
    - ec-lb587
    - eh-lb561
    - eh-lb563
- Compilation method: Enter the hcpu directory and execute the command `scons --board=<board_name> -j8`, where board_name is the board name. For example, to compile for eh-lb561 board, the complete command is `scons --board=eh-lb561 -j8`.
  The generated image files are stored in the HCPU's build_<board_name> directory. For project usage, refer to <<General Project Build Method>>.
- HCPU and LCPU use queue 2 as the communication channel. The send buffer size from HCPU to LCPU is 256 bytes, and the send buffer from LCPU to HCPU is also 256 bytes.
  The corresponding macro definitions are in `/src/common/ipc_config.h` and `linker_scripts/custom_mem_map.h` in their respective projects. HCPU's main function is in `src/hcpu/main.c`, and LCPU's main function is in `src/lcpu/main.c`.