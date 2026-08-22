# DFU V2 Examples

DFU V2 is the firmware update (DFU / OTA) middleware of the SiFli SDK. It supports three transport channels: USB-CDC, BLE, and Bluetooth PAN (network). This directory groups the matching examples by channel.

## Directory layout

```
dfu_v2/
├── cdc/      USB-CDC channel
│   ├── app/      user application + PC upgrade tool (dfu_v2_cdc_tool.py)
│   └── loader/   CDC DFU loader subprogram
├── ble/      BLE channel
│   ├── app/      BLE peripheral app (in-app real-time update; HCPU self-update via the loader)
│   └── loader/   BLE DFU loader subprogram
└── bt_pan/   Bluetooth PAN (network / HTTP) channel
    ├── app/      app that pulls firmware from a server over the network
    └── loader/   PAN DFU loader subprogram
```

## Choosing a channel

- **USB-CDC**: upgrade from a PC over a USB virtual COM port; good for production lines and wired debugging.
- **BLE**: upgrade from a phone app over Bluetooth, with the device acting as a peripheral.
- **PAN**: the device goes online over Bluetooth PAN and pulls firmware from a server itself.

## How app and loader relate

An app is used together with the loader of the same channel. When you build an app, the SDK automatically pulls in the matching loader as a child project through `AddDFU_*` in `building.py`, so there is no need to build the loader separately.

## Build

Enter an example's `project/` directory and run:

```
scons --board=<board> -j8
```

See each example's own README for the boards it supports.

## Example index

| Channel | User application | Loader subprogram |
|---|---|---|
| USB-CDC | [cdc/app](cdc/app/README.md) | [cdc/loader](cdc/loader/README.md) |
| BLE | [ble/app](ble/app/README.md) | [ble/loader](ble/loader/README.md) |
| Bluetooth PAN | [bt_pan/app](bt_pan/app/README.md) | [bt_pan/loader](bt_pan/loader/README.md) |

For the DFU V2 middleware configuration options, see `middleware/dfu_v2`.

```{toctree}
:hidden:

cdc/app/README
cdc/loader/README
ble/app/README
ble/loader/README
bt_pan/app/README
bt_pan/loader/README
```
