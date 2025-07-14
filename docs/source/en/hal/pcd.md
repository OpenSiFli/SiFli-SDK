# USBC

The USB Device Module (also known as Peripheral Controller Device, PCD) module implements USB device functionality between the Full-Speed USB 2.0 bus and APB bus.

## PCD Key Features
 - USB SF32LB55x/56x specification version 2.0 Full-Speed compatible, SF32LB58x supports USB 2.0 High-Speed transmission.
 - Configurable endpoint count up to 8 endpoints.
 - Cyclic Redundancy Check (CRC) generation/verification, Non-Return-to-Zero Inverted (NRZI) encoding/decoding and bit stuffing.
Please note that PCD is tested with RT-Thread USB device stack, with composite device configuration including USB Mass Storage and USB CDC UART.

## Using PCD
PCD can only be tested with upper-layer USB device stack. At HAL level, SiFli SDK provides support for:
 - USBD hardware module initialization.
 - EP0 read/write.
 - EPx read/write.
 - USB interrupt handler.
 - Endpoint halt/unhalt.
Upper-layer USB device stack can call USB device functions and implement callbacks defined by the PCD HAL module. For detailed usage of PCD, please refer to RT-Thread SiFli USB driver tools (in _/rtos/rtthread/bsp/sifli/drivers/drv_usbd.c_)

## API Reference
[](../api/hal/pcd.md)
