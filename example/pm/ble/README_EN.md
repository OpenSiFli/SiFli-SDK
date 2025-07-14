# BLE Broadcast and Connection Power Consumption Test
## Introduction:
BLE broadcast and connection scenario power consumption test routine (LCPU main frequency at 24MHz).
The wake-up PIN varies on different development boards, with the following corresponding relationships:
- EH-LB551: Uses PA80, corresponding to the INT pin on the external Bluetooth or GPS interface on the HDK board
- EH-LB555: Uses PA79, corresponding to TP_INT on the bottom HDK board

When the wake-up PIN is at low level, both HCPU and LCPU cannot enter low power mode.
At this time, commands can be sent to LCPU through Console to modify parameters. When the wake-up PIN is connected to high level (i.e., 1.8V voltage),
HCPU enters low power mode, while LCPU periodically enters and exits low power mode. At this time, Console cannot be used.
After connecting PC and HDK with USB Type-C cable, two serial ports will be enumerated. HCPU uses the second serial port as Console port, and LCPU uses the first serial port as Console port.

## Related Console Commands
- HCPU
    1. Factory reset BLE-related Flash data: nvds reset_all 1
            - To avoid Flash conflicts, it is best to execute this command first when using for the first time.
    2. Set Bluetooth MAC address: `nvds update addr 6 [addr]`. Example: `nvds update addr 6 2345670123C3`
- LCPU
    - `ble_config adv interval_in_ms`: Modify broadcast period, where `interval_in_ms` is the broadcast interval in milliseconds
    - `ble_config conn interval_in_ms`: Modify connection period, where `interval_in_ms` is the connection interval in milliseconds,
      command needs to be sent after connecting with phone

## Mobile Phone Recommendations:
1. For iPhone, third-party software LightBlue is recommended; for Android, use nRF Connect for BLE testing.

## Project Description
- Supported development boards include
    - eh-lb551
    - eh-lb555
- Compilation method: Enter the hcpu directory and execute the command `scons --board=<board_name> -j8`, where board_name is the board name. For example, to compile eh-lb551 board, the complete command is `scons --board=eh-lb551 -j8`
  The compiled image files are stored in the build_<board_name> directory of HCPU. For project usage, refer to <<General Project Construction Method>>

- LCPU main frequency is 24MHz. You can modify the 'board/bf0_ap_hal_msp.c' file in the LCPU project directory and remove the statement 'HAL_RCC_LCPU_SetDiv(2, 0, 3)' in function HAL_MspInit,
  this way it will maintain the 48MHz frequency set by HAL_PreInit.
- 32K crystal default configuration: EH-LB551 does not use it, EH-LB555 uses it. Related configuration modification method: menuconfig -->select board peripherals -->Low power crystal disabled