# USB-BT Network Bridge (lwIP NAT Solution)

Source code path: example/cherryusb/host/usb_bt_bridge_nat

## Overview

This example implements a **USB-BT network bridge**: the Dongle (SiFli SoC) acts as a **USB RNDIS host** connecting to a phone (HarmonyOS/Android), using the phone's "USB tethering", while it connects to a Bluetooth earbud in the classic Bluetooth **PAN NAP** role. The Dongle translates traffic from the earbud with **lwIP NAT (IPv4 forwarding + network address translation)** and forwards it out through the USB RNDIS uplink, giving the earbud access to the Internet.

This solution bridges at the IP layer: the Dongle itself has two IP interfaces (`b0` on the BT side, `u2` on the USB side) and NATs the earbud's private subnet `192.168.43.0/24` to the phone's USB network (e.g. `10.196.255.x`).


## Supported Boards

Verified on the following development boards:
- sf32lb52 LCD series

## Hardware Required

- A SiFli development board with USB Host support
- USB Type-C cable
- A HarmonyOS/Android phone (with "USB tethering" support)
- A Bluetooth PAN (PANU) earbud


## Technical Principle

### System Topology

```
Internet (WiFi/4G/5G)
        │
   ┌────┴───────┐
   │ Phone (RNDIS Device) │  USB tethering (DHCP Server)
   └─────┬──────┘
         │ USB Type-C
   ┌─────┴──────┐
   │ Dongle (this example) │
   │  USB RNDIS Host       │  u2: 10.196.255.x (DHCP Client)
   │  BT PAN NAP           │  b0: 192.168.43.1 (DHCP Server)
   │  lwIP NAT             │  NAT: 192.168.43.0/24 → u2
   └─────┬──────┘
         │ BT PAN / BNEP
   ┌─────┴──────┐
   │ Earbud (PANU)          │  192.168.43.x (DHCP Client)
   └────────────┘
```

### Data Flow

Uplink (earbud → Internet):

```
Earbud 192.168.43.x
  → b0 interface (received over PAN BNEP)
  → lwIP ip4_forward
  → ip_nat_out: source 192.168.43.0/24 hits the NAT entry → source rewritten to u2's IP
  → u2 interface (sent over USB RNDIS)
  → phone USB network → Internet
```

Downlink (Internet → earbud): `ip_nat_input` performs reverse NAT (rewrites the destination back to `192.168.43.x`) on received replies and sends them back to the earbud over `b0`.

### Key Implementation Points

- **USB side (`u2`)**: after enumerating the phone, the CherryUSB RNDIS Host registers the lwIP netif `u2` through `usbh_lwip.c`; `eth_device_init()` automatically starts the DHCP client to obtain IP/gateway (e.g. `10.196.255.3 / gw 10.196.255.57`).
- **BT side (`b0`)**: when the PAN connection is established, the `bt_pan` stack registers the lwIP netif `b0` (NAP mode) and starts the DHCP server (`LWIP_USING_DHCPD`), assigning `192.168.43.2 ~ .254` to the earbud with gateway `192.168.43.1` (b0's address).
- **NAT**: the `rndis_monitor_thread` in `main.c` waits for `u2` DHCP to bind, then calls `update_nat()` to install the NAT rule `source=192.168.43.0/24 → out_if=u2` (`LWIP_USING_NAT`). NAT only applies to the **forwarding path** (`ip4_forward`) and does not affect traffic originating from the Dongle itself.
- **Dongle's own connectivity**: once the RNDIS uplink gets an address, the application sets it as the default netdev (`netdev_set_default(u2)`), so the Dongle's own ping/DNS/sockets go out the phone uplink.


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


### Usage Steps

**Stage 1: Verify USB tethering**
1. Flash the firmware and open the serial terminal
2. Connect the phone via USB and enable "USB tethering" on the phone
3. Observe the `[USB] DHCP bound ...` log
4. Verify Dongle connectivity: `ping 8.8.8.8`

**Stage 2: Connect the Bluetooth PAN earbud**
1. Flash the earbud-side firmware and power it on
2. On the Dongle, start the inquiry: `pan_net inquiry start`
3. On the Dongle, connect: `pan_net conn <earbud MAC>` (e.g. `pan_net conn 12345678abcd`)
4. On the Dongle, observe `[BT] PAN connected ...` and `[NAT] Enabled 192.168.43.0/24 b0 -> u2`

**Stage 3: Verify earbud connectivity**
- The earbud should obtain a `192.168.43.x` address from the `b0` DHCP server and be able to access the Internet; verify with `ping 8.8.8.8`.

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


- [CherryUSB](https://github.com/cherry-embedded/CherryUSB)
