# BLE/BT Power Consumption Test
Source Path: example/pm/bt
## Introduction:
BLE/BT power consumption test routine (LCPU main frequency at 24MHz). BLE can
test broadcast and connection mode power consumption, BT can test Scan and Sniff
mode power consumption. The wake-up PIN varies on different development boards,
with the following corresponding relationships: - EC-LB58X: Uses PA64 -
EH-LB561/EH-LB563: Uses PB34, corresponding to HR_INT on HDK -
EH-ss6500/EH-LB523/EH-LB520: Uses PA24, corresponding to GPS_PEN on HDK

When the wake-up PIN is at low level, both HCPU and LCPU cannot enter low power
mode. At this time, commands can be sent through HCPU Console to modify
parameters. When the wake-up PIN is connected to high level, HCPU can enter low
power mode, while LCPU periodically enters and exits low power mode. At this
time, Console cannot be used. HCPU uses UART1 as Console port, LCPU uses UART4
as Console port.

## Related Console Commands
- HCPU
    1. Factory reset BLE-related Flash data: `nvds reset_all 1`
        - To avoid Flash conflicts, it is best to execute this command first
          when using for the first time.
    2. Set Bluetooth MAC address: nvds update addr 6 [addr]. Example: nvds
       update addr 6 2345670123C3
    3. 'ble_config adv interval_in_ms': Modify broadcast period, where
       'interval_in_ms' is the broadcast interval in milliseconds
    4. 'ble_config conn interval_in_ms': Modify connection period, where
       'interval_in_ms' is the connection interval in milliseconds, command
       needs to be sent after connecting with phone
    5. 'btskey': BTS menu control command, can modify BT parameters. After
       startup, it defaults to BTS main menu. BTS menu is a multi-level menu.
       You can send 'btskey s' command to display current menu content, then
       send commands according to menu prompts to enter next level menu or
       execute certain menu functions. For example, in the main menu, sending
       the following three commands in sequence can turn on Page Scan and turn
       off Inquiry Scan
        1) btskey 1
        2) btskey 7
        3) btskey 2

Main menu is shown below
```
######################################################
                    ##                                                  ##
                    ##           BTS2 Demo Main Menu                    ##
                    ##   1. Generic Command                             ##
                    ##   2. SPP Client                                  ##
                    ##   3. SPP Server                                  ##
                    ##   4. HFP HF                                      ##
                    ##   6. A2DP Sink                                   ##
                    ##   8. L2CAP bqb test                              ##
                    ##   p. AVRCP                                       ##
                    ##   s. Show Menu                                   ##
                    ##   q. Exit                                        ##
                    ##                                                  ##
                    ######################################################
```

After sending 'btskey 1', the Generic Command Menu submenu is displayed
```
######################################################
                    ##                                                  ##
                    ##           Generic Command Menu                   ##
                    ##   1. Inquiry start                               ##
                    ##   2. Inquiry cancel                              ##
                    ##   3. Select device from Inquiry list             ##
                    ##   4. Sc menu                                     ##
                    ##   5. Local device info                           ##
                    ##   6. Get remote device info                      ##
                    ##   7. Scan mode                                   ##
                    ##   8. Link menu                                   ##
                    ##   9. Service Browse                              ##
                    ##   a. Set IO Settings                             ##
                    ##   s. Show Menu                                   ##
                    ##   r. Return to last menu                         ##
                    ##                                                  ##
                    ######################################################
```

## Mobile Phone Recommendations:
1. For iPhone, third-party software LightBlue is recommended; for Android, use
   nRF Connect for BLE testing.

## Project Description
- Supported development boards:
    - ec-lb583
    - ec-lb587
    - eh-lb561
    - eh-lb563
    - eh-lb523
- Compilation Method: Navigate to the `hcpu` directory and execute the command
  `scons --board=<board_name> -j8`. Replace `<board_name>` with the name of the
  target board. For example, to compile for the `eh-lb561` board, the full
  command is `scons --board=eh-lb561 -j8`. The generated image files are stored
  in the `build_<board_name>` directory within `HCPU`. For further details on
  usage, refer to the "General Project Build Methodology" guide. For development
  boards utilizing NAND Flash (such as `ec-lb583`, `ec-lb587`, and `eh-lb563`),
  the first-time setup requires executing the `download_fs.bat` script located
  in the board-specific directory under the `HCPU` project to flash the
  filesystem image. Failure to do so will result in a filesystem mount error
  upon boot. The filesystem image only needs to be flashed once; however,
  reflashing the filesystem will erase the Bluetooth address, necessitating
  reconfiguration.</board_name></board_name></board_name>

