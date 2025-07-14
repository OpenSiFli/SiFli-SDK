# Getting Started Guide

[SDK]: https://github.com/OpenSiFli/SiFli-SDK
[Trace]: https://webfile.lovemcu.cn/file/sdk/SifliTrace_v2.2.6.7z
[52DevKit]: https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-LCD.html
[52Module]: https://wiki.sifli.com/silicon/%E6%A8%A1%E7%BB%84%E5%9E%8B%E5%8F%B7%E6%8C%87%E5%8D%97.html

## Preparation

### Hardware

- A development board, such as [SF32LB52-DevKit-LCD Development Board][52DevKit]
- A USB Type-C data cable to connect the development board to the computer. Note that it cannot be a Type-C cable with only charging function. Insert it into the USB-to-UART interface of the development board (do not insert it into the dedicated USB function interface)
- Computer (Windows, Linux, or macOS)

### Software

To use SiFli-SDK on SF32, please install the following software:
- Set up toolchain for compiling SF32 code;
- Build tool — SDK2.x uses Scons as the build tool for compiling SF32 applications;
- Obtain SiFli-SDK software development framework. This framework basically includes SF32 APIs (software libraries and source code) and scripts for running the **toolchain**;

### Installation

To install the required software, SiFli provides the following methods, you can choose one of them as needed.

#### Manual Installation

Refer to manual installation: [Manual Installation](install/index.md)
