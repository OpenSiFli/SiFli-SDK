# BT PBAP Client Example

Source code path: example/bt/pbap

{#Platform_bt_pbap}
## Supported Platforms
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview

This example demonstrates using the PBAP client to connect to a mobile phone and access its phonebook. It includes:
+ PBAP client connection and disconnection
+ Selecting a phonebook repository and phonebook object
+ Pulling a phonebook
+ Looking up a contact by phone number
+ Pulling contact information by vCard handle
+ Processing PBAP operation results through asynchronous notifications

## Example Usage

The example enables Bluetooth by default at startup and controls the PBAP client through FINSH commands. Before running it, pair the phone with the development board and allow contact sharing when requested by the phone.

1. Connect to a phone

Start a PBAP connection using the phone Bluetooth address. The optional `auth` argument is `0` or `1`:

```text
pbap_cmd connect 11:22:33:44:55:66 [0|1]
```

The connection state is printed through PBAP asynchronous notifications. Use the following command to print the command help:

```text
pbap_cmd help
```

2. Disconnect from a phone

```text
pbap_cmd disconnect 11:22:33:44:55:66
```

3. Select the phonebook path

The `set` command supports the `local`, `sim1`, and `current` repositories, and the `pb`, `ich`, `och`, `mch`, and `cch` phonebook objects:

```text
pbap_cmd set local pb
pbap_cmd set sim1 ich
```

| Argument | Description |
| --- | --- |
| `pb` | Main phonebook |
| `ich` | Incoming call history |
| `och` | Outgoing call history |
| `mch` | Missed call history |
| `cch` | Combined call history |

4. Pull a phonebook

```text
pbap_cmd pull_pb
pbap_cmd pull_pb 100
```

The optional argument specifies the maximum contact count and defaults to `65535`. Contact information, the total contact count, and the operation completion result are printed through asynchronous notifications.

5. Look up a contact by number

```text
pbap_cmd list 10086
```

`list` uses the PBAP client number lookup API. It does not pull the complete phonebook.

6. Pull a vCard

```text
pbap_cmd vcard 123
```

The command return value only indicates whether the request was accepted. Check the serial output for the asynchronous PBAP completion or result event.

### Hardware Requirements

Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_bt_pbap)).
+ One Bluetooth phone that supports PBAP.
+ A serial connection to the development board for entering FINSH commands and reading logs.

### menuconfig Configuration

1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enables Bluetooth functionality
2. Enable the PBAP client:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh (Optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enables the FINSH command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually selects the enabled profiles
    - Enable: Enable PBAP client
        - Macro switch: `CONFIG_CFG_PBAP_CLT`
        - Description: Enables PBAP client functionality

### Compilation and Flashing

Run `set_env.bat` from the SDK root to set up the build environment. Then run:

```bat
cd example\bt\pbap\project
scons --board=<board> -j32
```

Switch to `project/build_<board>` and run the generated download script to flash the image. For details, refer to the [Quick Start](/quickstart/get-started.md) documentation.

## Expected Results

After the example starts, `pbap_cmd` commands can be entered through the serial console. After the development board connects to a PBAP-capable phone, it can select a phonebook path, pull the phonebook, look up contacts, and print operation results through asynchronous notifications.

## Exception Diagnosis

- If connection fails, confirm that the phone is paired and that contact sharing is allowed on the phone.
- If a phonebook operation fails, confirm that the PBAP connection is active and that a valid path has been selected with `set`.
- The phone number for `list` must not be empty, and the `vcard` handle cannot exceed 25 characters.
- PBAP operations are asynchronous. A successful FINSH return value only means that the request was submitted; continue checking the PBAP result logs.

## Reference Documentation

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |11/2026 |Initial version |
| | | |