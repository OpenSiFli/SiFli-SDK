# Bluetooth Low Energy Service

Silfi Bluetooth is divided into two parts: one is the Bluetooth LE (BLE) service, which provides standard Bluetooth LE APIs and enhanced services for user development; the other is a fully tested BLE 5.2 protocol stack with host and controller components.

The BLE service runs on HCPU or LCPU. It provides standard Generic Access Profile (GAP) APIs. It also provides a series of sibles BLE services to help users easily develop GATT services, clients, and GAP. Additionally, the Bluetooth LE service provides multiple GATT-based services. The BLE protocol stack runs on the Bluetooth stack core (LCPU), and HCPU can communicate with the BLE service through inter-core communication mechanisms such as hardware mailbox.

![](../../assets/ble_structure.png)

```{toctree}
:maxdepth: 1
:titlesonly:

ble_gap.md
ble_service.md
ble_profile.md
```

## BLE Service Configuration

Users can enable the Bluetooth LE service using the menuconfig tool. Configuration is typically saved in C header files. By default, the configuration is saved as rtconfig.h.

The following example shows flags defined in a project header file:
```c
#define BSP_BLE_SIBLES
```

After configuration is complete, users need to include the header file in all source code that requires access to data services. Users can also use menuconfig to enable BLE services.
![](../../assets/ble_menuconfig.png)