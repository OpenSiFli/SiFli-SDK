# cdc_acm_uart

A USB CDC ACM to UART bridge driver based on CherryUSB that transparently forwards USB virtual serial port data to physical UART devices.

## Features

- **USB CDC ACM Device**: Implements standard USB Communication Device Class, appears as a virtual serial port on PC
- **Bidirectional Data Bridge**: Transparent USB ↔ UART data transmission
- **Dynamic Parameter Configuration**: Supports runtime modification of UART parameters (baud rate, data bits, stop bits, parity)
- **High-Performance Transmission**: Uses DMA/interrupt reception

## Hardware Connections

### Supported Development Boards

- **SF32LB52_LCD_N16R8**: UART2 uses PA03(RX), PA04(TX)
- **SF32LB58_LCD_N16R64N4**: UART2 uses PA29(RX), PA28(TX)

## Compilation and Usage

### 1. Compilation

Switch to the example project directory and run the scons command to compile:

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat` and select the port as prompted for downloading:

```bash
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
Uart Download
please input the serial port num:5
```

### 2. Usage Instructions

1. **Hardware Connection**: Connect the configured UART2 pins RX and TX to the TX and RX of a USB-to-serial module respectively, then connect the module to the computer's USB port. Connect the Type-C interfaces labeled USB and UART on the development board to the computer's USB ports separately. The UART interface is used for downloading and debugging, while the USB interface is used for running the example program.

2. **Flash Program**: Flash the compiled firmware to the development board. See "Compilation and Usage" section for flashing steps.

3. **Open Serial Port**: To test transparent transmission between UART2 and USB virtual serial port, the host needs to run two serial debugging assistants, opening the "USB Serial Device" and the COM port where the USB-to-serial module is located respectively. After configuring the same baud rate, they can send and receive messages to each other. At this point, the development board functions as a USB-to-serial module.

## Troubleshooting

### Common Issues

1. **Device Not Recognized**
   - Check USB connection
   - Verify VID/PID configuration
   - Check USB driver

2. **Data Transmission Abnormal**
   - Check UART pin configuration
   - Verify baud rate settings
   - Check hardware connections

## Reference Documentation

* CDC ACM Device Class: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB Documentation: https://cherryusb.readthedocs.io/
* CherryUSB Official Repository: https://github.com/cherry-embedded/CherryUSB
* RT-Thread Official Website: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## Update Log

| Version | Date   | Release Notes |
|:--------|:-------|:--------------|
| 0.0.1   | 9/2025 | Initial version |