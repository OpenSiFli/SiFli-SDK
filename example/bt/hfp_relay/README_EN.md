# BT HFP Relay Example

Source code path: `example/bt/hfp_relay`

{#Platform_bt_hfp_relay}
## Supported Platforms

+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview

This example demonstrates HFP (Hands-Free Profile) call relay functionality. The device runs two HFP roles at the same time:

+ **HF (Hands-Free) role**: works as a hands-free device and connects to a mobile phone (the phone side works as AG).
+ **AG (Audio Gateway) role**: works as an audio gateway and accepts the connection from a Bluetooth headset (the headset side works as HF).

After the mobile phone and Bluetooth headset establish HFP connections with the development board respectively, the development board synchronizes the call status, incoming call number, signal/battery indicators, and other information from the phone side to the headset. It also forwards dialing, answering, hanging up, DTMF key, volume adjustment, battery status update, and other requests initiated by the headset to the phone. In this way, HFP control relay between the phone and headset is implemented.

> This example is used to verify the relay flow of HFP control and status information. The actual call audio establishment and transmission depend on the peer devices and Bluetooth protocol stack negotiation result.

## Example Usage

### Connection Topology

The following three devices are required to run this example:

1. A development board flashed with this example;
2. A mobile phone that supports HFP;
3. A Bluetooth headset that supports HFP.

The connection relationship is shown below. The development board works as two HFP roles at the same time: **HF** for the phone and **AG** for the Bluetooth headset.

```text
Mobile Phone (AG) <--HFP--> Development Board (HF/AG Relay) <--HFP--> Bluetooth Headset (HF)
```

The development board relays call status, phone numbers, volume, and control requests between the mobile phone and Bluetooth headset.

The basic interaction is as follows:

+ Phone → Headset: incoming call status, call status, phone number, and other information synchronization.
+ Headset → Phone: answer, hang up, dial, DTMF, volume adjustment, and other control request forwarding.

After the Bluetooth protocol stack is ready, this example sets the local Bluetooth name of the development board. The default name is `sifli_hfp_relay_service`. If `BT_DEVICE_NAME` is defined in the project configuration, that configured name is used instead.

### Operation Steps

Bluetooth is enabled by default after the example starts, and the FINSH command `hfp_cmd` is registered. In the following commands, `[addr]` is the Bluetooth device address in the format of `xx:xx:xx:xx:xx:xx`, and `[phone_number]` is the phone number to dial.

1. **Clear bonded devices (optional)**

   When testing for the first time or when re-pairing is required, run the following command to clear saved bonded devices:
   ```shell
   hfp_cmd c
   ```

2. **Search for the mobile phone and Bluetooth headset**

   After running the inquiry command, this example searches for phone-class devices and audio-class devices. When a device is found, the serial log prints the device name, COD, and Bluetooth address. Use `hfp_cmd stop_inquiry` to stop inquiry if necessary.
   ```shell
   hfp_cmd start_inquiry
   hfp_cmd stop_inquiry
   ```

   Typical inquiry logs are shown below. The `addr` value is used as `[addr]` in subsequent connection commands:
   ```text
   device <name> searched
   device COD is <cod>, addr is xx:xx:xx:xx:xx:xx
   ```

3. **Connect to the mobile phone (HF role)**

   The development board connects to the mobile phone as the HF role. Use the phone address found during inquiry in the command below:
   ```shell
   hfp_cmd hfp_connect [phone Bluetooth address]
   ```

   Alternatively, the phone can search for and actively connect to the development board. For the first connection, complete pairing according to the phone prompt. When the connection succeeds, the serial log prints:
   ```text
   HFP HF connected
   ```

4. **Connect to the Bluetooth headset (AG role)**

   The development board accepts the headset connection as the AG role. The headset can search for and connect to the development board named `sifli_hfp_relay_service`. If the headset address is known or found during inquiry, the development board can also actively initiate the connection:
   ```shell
   hfp_cmd hfp_ag_connect [headset Bluetooth address]
   ```

   When the connection succeeds, the serial log prints:
   ```text
   HFP AG connected
   ```

5. **Verify information synchronization from phone to headset**

   Initiate an incoming call, make a call, or answer a call on the phone side, and check whether the headset receives the incoming call number and call status changes. Signal, battery, and other indicator information from the phone side is also synchronized to the headset. To actively query phone information, run:
   ```shell
   hfp_cmd local_phone_number
   hfp_cmd remote_calls_info
   hfp_cmd remote_calls_status
   ```

   When the local phone number is received successfully, the serial log prints `the remote phone local number:<number>`. When the call status changes, the log prints `the remote phone call_status`, `callsetup_status`, and `callheld_status`.

6. **Verify call control from headset to phone**

   On the headset side, answer, hang up, dial, send a DTMF key, or adjust call volume, and confirm that the request is forwarded to the phone and takes effect. You can also directly verify phone-side HFP control through FINSH commands:
   ```shell
   hfp_cmd make_call [phone_number]
   hfp_cmd call_back
   hfp_cmd answer_call
   hfp_cmd handup_call
   hfp_cmd dtmf_key [0-9#*ABCD]
   hfp_cmd volume_control [0-15]
   hfp_cmd voice_recognition [0|1]
   hfp_cmd battery_update [0-9]
   ```

   The results of dialing, redialing, answering, hanging up, and volume adjustment are printed as `make a call complete`, `make a callback complete`, `answer a call complete`, `hangup a call complete`, and `change volume value complete`, respectively.

7. **Verify the call audio link**

   During a call, run the following commands to establish or disconnect the SCO audio link:
   ```shell
   hfp_cmd audio_connect
   hfp_cmd audio_disconnect
   ```

   When the call audio link is established or disconnected, the serial log prints:
   ```text
   HFP HF audio_connected
   HFP HF audio_disconnected
   ```

8. **Disconnect the HFP connection**

   Disconnect the corresponding HFP connection by device address:
   ```shell
   hfp_cmd hfp_disconnect [device Bluetooth address]
   ```

   After the phone or headset is disconnected, the serial log prints `HFP HF disconnected` or `HFP AG disconnected`.

> To use the relay function, both the phone-side HF connection and the headset-side AG connection must be established. Call control commands directly executed through `hfp_cmd` act on the phone-side HF connection.

### Typical Function Verification

| Test Item | Operation | Expected Result |
|:---|:---|:---|
| Incoming call status synchronization | The phone receives an incoming call | The headset receives the incoming call status; the serial log prints phone-side call status updates. |
| Answer/Hang up | Answer or hang up on the headset side | The request is forwarded to the phone, and the phone call status changes accordingly. |
| Headset dialing | Initiate dialing on the headset side | The dialed number is forwarded to the phone for execution. |
| DTMF key | Send a key from the headset during a call | The key is forwarded to the phone. |
| Volume synchronization | Adjust call volume on the phone or headset side | The peer device receives the corresponding volume update; when Audio Manager is enabled, the Bluetooth voice volume is also updated. |
| Information query | The headset queries operator/call/local phone number information | The development board obtains the information from the phone side and replies to the headset. |

### Hardware Requirements

Before running this example, prepare:

+ One development board supported by this example ([Supported Platforms](#Platform_bt_hfp_relay)).
+ One mobile phone that supports HFP.
+ One Bluetooth headset that supports HFP.

### menuconfig Configuration

The project `project/proj.conf` already contains the required default configuration. If you manually create or adjust the project, make sure the following options are enabled:

1. Enable Bluetooth (`BLUETOOTH`):
   - Path: `Sifli middleware → Bluetooth`
   - Enable: `Enable bluetooth`
     - Macro switch: `CONFIG_BLUETOOTH`
     - Description: Enables Bluetooth functionality.
2. Enable HFP HF and AG roles:
   - Path: `Sifli middleware → Bluetooth → Bluetooth service → Classic BT service`
   - Enable: `Enable BT finsh` (optional)
     - Macro switch: `CONFIG_BT_FINSH`
     - Description: Enables FINSH Bluetooth control commands.
   - Enable: `Manually select profiles`
     - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
     - Description: Allows manually selecting Classic Bluetooth profiles.
   - Enable: `Enable Handsfree HF`
     - Macro switch: `CONFIG_CFG_HFP_HF`
     - Description: Enables the development board to connect to the phone as the HF role.
   - Enable: `Enable Handsfree AG`
     - Macro switch: `CONFIG_CFG_HFP_AG`
     - Description: Enables the development board to serve the Bluetooth headset as the AG role.
3. Auto reconnect (optional):
   - Macro switch: `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
   - Description: Automatically reconnects to the most recently paired device after startup.
4. Audio function:
   - Path: `Sifli middleware`
   - Enable: `Enable Audio`
     - Macro switch: `CONFIG_AUDIO`
     - Description: Provides Bluetooth voice related audio capability.

### Compilation and Flashing

Switch to the example `project` directory and run SCons to compile. For example:

```shell
> scons --board=eh-lb525 -j32
```

Switch to the example `project/build_xx` directory and run `uart_download.bat`. Select the serial port as prompted to download:

```shell
> uart_download.bat
```

For detailed compilation and download steps, refer to [Quick Start](/quickstart/get-started.md).

## Expected Results

After both the mobile phone and Bluetooth headset are connected to the development board successfully:

+ The serial log prints `HFP HF connected` and `HFP AG connected` in sequence.
+ Incoming calls, dialing, and call status changes on the phone side can be synchronized to the headset.
+ Answering, hanging up, dialing, DTMF, and volume control requests initiated by the headset can be forwarded to the phone.
+ When the SCO call audio link is established or disconnected, the serial log prints the corresponding `audio_connected` or `audio_disconnected` log.

## Exception Diagnosis

1. **The phone or headset cannot connect**: Confirm that both peer devices support HFP, delete historical pairing records and pair again, and confirm that `CONFIG_CFG_HFP_HF` and `CONFIG_CFG_HFP_AG` are both enabled.
2. **Only one side connects successfully**: HFP relay requires both the phone-side HF connection and headset-side AG connection to be established. Check the `HFP HF connected` and `HFP AG connected` logs in the serial output.
3. **Call control does not take effect**: Confirm that the phone HFP connection is not disconnected, and check whether `HFP HF disconnected` appears in the serial log. Some phones or headsets may differ in support for optional HFP features such as dialing, phone number query, and voice recognition.
4. **Auto reconnect does not work**: Complete one successful pairing first, and confirm that `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE` is enabled.

## Reference Documentation

+ [BT HFP HF Example](../hfp/README_EN.md)
+ [Quick Start](/quickstart/get-started.md)

## Update History

| Version | Date | Release Notes |
|:---|:---|:---|
| 0.0.1 | 07/2026 | Initial version |
