# SiFli-SDK Programming Guide
SiFli-SDK is a software development framework customized based on RT-Thread, which enables rapid development of applications running on SiFli Technology chip platforms.

```{only} SF32LB55X
This document describes how to use the SDK on SF32LB55x chips.
```

```{only} SF32LB56X
This document describes how to use the SDK on SF32LB56x chips.
```
```{only} SF32LB58X
This document describes how to use the SDK on SF32LB58x chips.
```

```{only} SF32LB52X
This document describes how to use the SDK on SF32LB52x chips.
```

The SDK uses Kconfig as the software configuration tool and SCons for command-line compilation. It can also generate projects for Keil, Eclipse, and GNU Makefile for compilation and debugging, and supports generating MS Visual Studio projects for PC simulation.

The SDK features the following characteristics:
- Hardware Abstraction Layer (HAL) for accessing chipset hardware\
    This component is independent of the Real-Time Operating System (RTOS) and can be integrated with different RTOS or used without an RTOS.
- RT-thread device drivers
- LVGL GUI library\
    Deep optimization of LVGL, utilizing the chip's built-in GPU to accelerate graphics rendering, achieving smooth graphical interface effects
- BT/BLE protocol stack and other middleware, including:
    - HFP/A2DP host protocol stack and application interfaces
    - PAN protocol stack and corresponding TCP/IP protocol stack integration
    - Rapid BLE service generation to accelerate BLE GATT server development
    - CMSIS DSP and CMSIS NN with hardware acceleration
    - Encryption/decryption using Flash XIP hardware acceleration
    - File system support on NOR and NAND flash
    - USB device stack with comprehensive USB class support
- Secure Boot\
    Provides firmware updates through secure transmission and encrypted firmware updates. The SDK includes scripts responsible for firmware encryption, signing, and transmission.


```{toctree}
:hidden:

quickstart/index
app_development/index
hal/index
drivers/index
app_note/index
middleware/index
supported_boards/index
bootloader
example/index
api/index
contribute/index
```