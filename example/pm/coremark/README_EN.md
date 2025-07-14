# Processor Power Consumption Test
## Introduction:
Processor power consumption test routine, including the following scenarios:
- One core executes CoreMark benchmark program
- One core executes while loop for a period of time, executing nop instructions in the loop
- System shutdown, can be woken up by RTC timer
- System shutdown, woken up by button press
The wake-up PIN varies on different development boards, corresponding relationships are as follows:
- EH-LB551: Uses PA80, corresponding to the INT pin on the external Bluetooth or GPS interface on the HDK board
- EH-LB555: Uses PA79, corresponding to TP_INT on the bottom HDK board
- EC-LB58X: Uses PA64
- EH-LB561/EH-LB563: Uses PB34, corresponding to HR_INT on the HDK board
- EH-ss6500/EH-LB52x: Uses PA24, corresponding to GPS_PEN on the HDK
When the wake-up PIN is at low level, HCPU cannot enter low power mode.
At this time, commands can be sent to HCPU through console to execute specified tasks. When the wake-up PIN is connected to high level (i.e., 1.8V voltage), HCPU enters low power mode, and at this time HCPU cannot respond to console commands.
LCPU never enters low power mode. If LCPU is started, when not executing tasks, LCPU is in WFI state and can respond to commands from console.
When LCPU is not started, LCPU is considered to be in halt state and cannot process console commands.

PC and the baseboard are connected using USB Type-C cable, which will enumerate two serial ports. HCPU uses UART1 as Console port, LCPU uses UART4 as Console port.

## Related Console Commands
- Both HCPU and LCPU support
    - 'run_coremark freq_in_mhz': Modify main frequency and execute CoreMark, 'freq_in_mhz' is the frequency in MHz, for example 'run_coremark 48', execute CoreMark at 48MHz main frequency
    - 'run_while_loop freq_in_mhz': Modify main frequency and execute while loop for a period of time, 'freq_in_mhz' is the frequency in MHz, `run_while_loop 48`, execute while loop at 48MHz main frequency
    - HCPU supported main frequencies: 240MHz/192MHz/144MHz/96MHz
    - LCPU supported main frequencies: 48MHz/24MHz
- HCPU supports
    - 'lcpu on': Start LCPU, after successful startup, LCPU can receive console commands to execute specified tasks
    - 'shutdown [wakeup_time_in_sec]': Shutdown, 'wakeup_time_in_sec' is an optional parameter in seconds, indicating how long after shutdown to automatically power on. If no parameter is provided, after shutdown it can only be woken up by PIN.
                                        On 551 and 555 platforms, PIN is KEY1 button, press to power on.
                                        On 557 platform, PIN corresponds to SPI3_INT pin next to J0804, this pin needs to be connected to high level before shutdown, and connecting to low level after shutdown will trigger power on.
                                        On 58X platform, PIN corresponds to KEY_HOME_RST button, press to power on.

## Project Description
- Development boards supported by the project:
    - eh-lb551
    - eh-lb555
    - ec-lb583
    - ec-lb587
    - eh-lb561
    - eh-lb563
    - eh-lb523
- Compilation method: Enter hcpu directory and execute command `scons --board=<board_name> -j8`, where board_name is the board name. For example, to compile eh-lb561 board, the complete command is `scons --board=eh-lb561 -j8`.
The compiled image file is stored in HCPU's build_<board_name> directory. For project usage, refer to <<General Project Build Method>>