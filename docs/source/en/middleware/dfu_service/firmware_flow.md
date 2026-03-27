# Process Introduction

## Flash Layout

As shown in the figure below, by default, Bootrom will jump to 0x10020000
according to the flash config table (for details, refer to []{1}). The firmware
upgrade service requires creating a separate OTA manager project, and the user
project (User Code) needs to be placed after the OTA manager.

After Bootrom jumps to OTA manager, it will act according to the current state:
if no upgrade is needed, it jumps to the user project; if an upgrade is needed,
it waits for the upgrade.


The starting address of the user project and the Upgrade bin address where
firmware upgrade packages are stored can be configured through the project's
_memory_map.h_ and _custom_memory_map.h_. It is recommended to place the Upgrade
bin at the end of Flash.


## Process Overview

1. Step 1. The remote device sends _ctrl_packet.bin_ to user bin to confirm
   whether OTA can proceed. If yes, continue to step 2
2. Step 2. User bin needs to restart and enter ota manager bin, then
   re-establish Bluetooth connection with the remote device. If no connection is
   made within a certain time, it will return to user bin.
3. Step 3. After establishing Bluetooth connection, the remote device will
   continuously transmit upgrade packages until all upgrade packages are
   transmitted.
4. Step 4. Decompress the firmware upgrade package and complete installation.

The starting address of the user project and the Upgrade bin address where
firmware upgrade packages are stored can be configured through the project's
_memory_map.h_ and _custom_memory_map.h_. It is recommended to place the Upgrade
bin at the end of Flash.


## Important Notes


Refer to the `example/ble/peripheral_with_ota` project and its corresponding
README: {doc}`../../example/ble/peripheral_with_ota/README`
