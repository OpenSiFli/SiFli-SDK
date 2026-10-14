# USB-BT Transparent Bridge (No IP Translation Solution)

Source code path: example/cherryusb/host/usb_bt_bridge_l2

## Overview

This example implements a **USB-BT transparent bridge**. The Dongle forwards Ethernet frames received on USB RNDIS and Bluetooth PAN/BNEP **directly to each other**, completely bypassing the lwIP IP layer. There is no routing or NAT, and the Dongle itself does not take part in the IP stack. The earbud **gets an address in the same subnet as the phone's USB network** and lives on the same L2/L3 network as the phone.

Unlike the NAT solution ([`example/cherryusb/host/usb_bt_bridge_nat`](../usb_bt_bridge_nat/README.md), lwIP NAT), the Dongle here has no IP and does not go online by itself; it acts as a pure transparent pipe.


## Supported Boards

Verified on the following development boards:
- sf32lb52 LCD series

### Hardware Required

- A SiFli development board with USB Host support
- USB Type-C cable
- A HarmonyOS/Android phone (with "USB tethering" support)
- A Bluetooth PAN (PANU) earbud

## Technical Principle

### Transparent Forwarding Path

```
Earbud PANU (10.196.255.x)
    │
    │ DHCP Discover
    ▼
BT PAN / BNEP ──► Dongle direct forward ──► USB RNDIS ──► Phone DHCP Server
The phone replies with DHCP Offer / ACK.
These replies travel back along the reverse path to the earbud
```

When forwarding succeeds, the earbud gets an address in the same subnet as the phone's USB network, e.g.:
- Phone gateway: `10.196.255.57`
- Earbud address: `10.196.255.x`

Because the earbud and the phone are on the same IP network, the following are **no longer needed**:
- IPv4 forwarding (`IP_FORWARD`)
- NAT
- DHCP Server on the Dongle's `b0`
- DHCP Client on the Dongle's `u2`
- lwIP IP-layer routing

**Note**: `u2`/`b0` are lwIP interface names (the netdev names shown by `ifconfig`), not USB/Bluetooth spec terms: `u2` is the USB RNDIS netif and `b0` is the Bluetooth PAN/BNEP netif.

### Weak Hook Mechanism (Core)

The **CherryUSB Kconfig option `PKG_CHERRYUSB_HOST_CDC_RNDIS_NO_NETIF`** (C macro `USBHOST_RNDIS_NO_NETIF`) decides whether a frame goes through the bridge:

- **Bridge mode (macro on)**: the `u2` netif is **not** registered, and every frame received over USB goes **directly** to the hook `usbh_rndis_on_raw_rx()` — lwIP is never touched.
- **Non-bridge mode (macro off)**: the `u2` netif is registered normally and frames take the normal lwIP netif path (the hook is not called).

- **USB → BT**: `external/cherryusb/class/wireless/usbh_rndis.c` defines the weak function `usbh_rndis_on_raw_rx(buf, len)`; this project overrides it in `main.c`: when a PAN connection exists (`s_pan_bnep_id` is valid), it calls `bt_pan_send_data(s_pan_bnep_id, buf, len)` to send the USB frame to the earbud over the BNEP channel.
- **BT → USB**: `middleware/bluetooth/service/bt/bt_finsh/bts2_app_pan.c` defines the weak function `bt_pan_on_raw_rx(buff, len)`; this project overrides it in `main.c`: it takes the RNDIS TX buffer via `usbh_rndis_get_eth_txbuf()`, `memcpy`s the frame, and sends it via `usbh_rndis_eth_output()`.

### Automatic PAN Re-sync (Phone Change / Reboot)

In transparent-bridge mode the **earbud's IP address is assigned by the phone's DHCP server** and must be in the same subnet as the phone's current USB network. Each "USB tethering" session uses a subnet that may differ from the previous one (switching phones or rebooting the phone); the Bluetooth link, however, does not drop with USB, so the earbud keeps its old DHCP lease and never re-acquires an address by itself — after a phone swap or phone reboot it simply loses Internet access until the lease expires or the earbud is restarted.

To handle this, the example performs an automatic re-sync: when the Dongle detects a **new RNDIS session** (USB re-enumeration caused by switching phones, a phone reboot or a cable re-plug) while PAN is connected, it disconnects and reconnects the PAN link once. The peer then rebuilds its netif and re-runs DHCP, automatically adopting the new phone's subnet without restarting the earbud.

- Every USB session change triggers one re-connect (including re-plugging the same phone); the bridge is interrupted for a few seconds — this is expected.
- `pan_net disc` on the Dongle disconnects PAN manually (re-connect with `pan_net conn <earbud MAC>`), useful for debugging.
- This mechanism relies on the peer re-running DHCP after a PAN reconnect (SDK-based earbud firmwares such as `example/bt/pan` do this by default).
- Expected log:

```none
[USB] RNDIS link lost, bridge inactive
[USB] RNDIS link up, USB<->BT bridge active
[BT] New USB session: re-sync PAN, peer will re-run DHCP
[BT] PAN disconnected
[BT] PAN re-sync: reconnecting
[BT] PAN connect request accepted: ...
[BT] PAN connected, USB<->BT bridge ready
```

### Key Configurations

- `PKG_CHERRYUSB_HOST_CDC_RNDIS_NO_NETIF` (provided by CherryUSB, enabled by this project): the lwIP netif `u2` is **not registered** after RNDIS enumeration; the RNDIS driver itself tracks the link state (`usbh_rndis_is_link_up()`), so the Dongle never touches lwIP.
- `BT_PAN_NO_B0_NETIF` (provided by the Bluetooth component, enabled by this project): the lwIP netif `b0` is **not created** on PAN connect; received frames are handed to the application hook `bt_pan_on_raw_rx()`.

### Limitations

- The Dongle has no IP and cannot go online locally (e.g. `ping`).


## Using the Example

### Compile and Flash
Change to the example `project` directory and run `scons` to compile:
```
scons --board=sf32lb52-lcd_a128r16 -j8
```
Run the flash command:
```
build_sf32lb52-lcd_a128r16_hcpu\uart_download.bat
```
Select the serial port as prompted to download:
```none
please input the serial port num:6
```
### Earbud Side
You can use another development board as the earbud side. For the earbud firmware, use the `example\bt\pan` example and flash it into the second board using the same compile/flash procedure above for testing.

## Usage Steps

1. Flash the firmware, connect the phone via USB and enable "USB tethering"
2. Observe `[USB] RNDIS link up, USB<->BT bridge active`
3. Flash the earbud-side firmware and power it on
4. On the Dongle, start the inquiry: `pan_net inquiry start`
5. On the Dongle, connect: `pan_net conn <earbud MAC>` (e.g. `pan_net conn 12345678abcd`)
6. After the Bluetooth connection is established, the earbud should obtain an IP in the same subnet as the phone network (e.g. `10.196.255.x`) and be able to access the Internet; verify with `ping 8.8.8.8`.

## Example Output

```
TX:ping 8.8.8.8
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   D/NO_TAG bts: Active mode st: 0, inv: 0.00
   D/btapp_pan bts:  BTS2MU_PAN_STS_IND
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=0 ttl=106 time=901 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=1 ttl=106 time=283 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=2 ttl=106 time=286 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=3 ttl=106 time=284 ms
   msh />msh />
```

## Troubleshooting

- **Dongle does not respond when connected to the phone**: make sure the USB cable supports data transfer.
- **Earbud has no Internet after switching phones or rebooting the phone**: the earbud still holds the old phone's DHCP lease. The example automatically re-syncs PAN on a new USB session (see "Automatic PAN Re-sync"); if it does not recover, run `pan_net disc` followed by `pan_net conn <earbud MAC>` on the Dongle to verify manually, and check the earbud's `ifconfig` for an address in the phone's current subnet.


- [CherryUSB](https://github.com/cherry-embedded/CherryUSB)


