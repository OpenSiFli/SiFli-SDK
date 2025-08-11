# SiFli-SDK

English | [中文](README.md)

## Getting Started

The SiFli SDK is an official software development framework, custom-built on RT-Thread, that enables rapid development of applications running on SiFli Technology’s chip platforms.

Relevant documentation:
- https://wiki.sifli.com/
- https://docs.sifli.com/projects/sdk/latest/sf32lb52x/index.html

You can refer to these documents for a quick start.

**Note**: Since the SDK includes submodules, you must clone the repository with the `--recursive` flag, for example:
```bash
git clone --recursive https://github.com/OpenSiFli/SiFli-SDK
```

The software framework is illustrated below:

![sdk_arch_diagram](img/sdk_arch_diagram.png)

- **HAL** is the Hardware Abstraction Layer, providing driver functionality without relying on any operating system services.  
- **RT-Thread Device Drivers** are built on top of HAL and offer a higher-level abstraction; users do not need to write interrupt service routines, making them easier to use. For a detailed comparison between HAL and RT-Thread device drivers, see [Chip Peripheral Drivers](https://docs.sifli.com/projects/sdk/v2.3/sf32lb55x/app_development/drivers.html).  
- **Middleware (components)** includes RT-Thread’s built-in software components (e.g., finsh, ulog), third-party components (under the `external` directory), and in-house components (under the `middleware` directory). Applications can use all service interfaces, including HAL, to build their software.

### SDK Directory Structure

```
+---customer                 // Board Support Package
|   +---boards               // Board configuration files
|   +---peripherals          // Board-level peripheral drivers
|
+---drivers
|   +---cmsis                // Chip register headers, startup files, linker scripts
|   |   +---Include
|   |   +---sf32lb52x
|   |   +---sf32lb55x
|   |   +---sf32lb56x
|   |   +---sf32lb58x
|   +---hal                  // HAL implementation code
|   \---Include              // HAL header files
|
+---example                  // Examples
+---external                 // Third-party components
+---middleware               // In-house components
+---rtos                     // Operating systems
|   +---freertos             // FreeRTOS
|   +---os_adaptor           // OS abstraction layer
|   \---rtthread             // RT-Thread
|       \---bsp
|           \---sifli
|               \---drivers  // RT-Thread device driver adapters
\---tools                    // Tools
```

## Versioning Policy

Version numbers follow the [Semantic Versioning](https://semver.org/) scheme.

The version format is **MAJOR.MINOR.PATCH**, with the following increment rules:

- **MAJOR** version: incompatible API changes (e.g., v1.0.0 → v2.0.0)  
- **MINOR** version: backward-compatible new features or enhancements (e.g., v2.3.0 → v2.4.0)  
- **PATCH** version: backward-compatible bug fixes or small functional tweaks (e.g., v2.3.1 → v2.3.2)  

Each release is tagged as `vX.Y.Z`.

### Release Cadence

- **Patch releases**: as needed, typically as fast as one week  
- **Minor releases**: every 4–5 months  
- **Major releases**: once a year or longer  

### Branches

The repository contains the following branches:

- `main`: development branch where all new features and fixes land. **Not recommended** for production development.  
- `release/vX.Y`: minor release branches containing all patches for that minor version. All official releases are made from these branches (tags are placed on commits in these branches). **Recommended** for project development is either the latest release tag or the tip of the corresponding `release/vX.Y` branch.  
- Other branches: temporary or feature branches for internal development. **Not recommended** for general use.  

### Support Timeline

| Version | Release Date     | End-of-Support Date | Supported |
|---------|------------------|---------------------|-----------|
| v2.3    | January 21, 2025 |                     | Yes       |
| v2.4    | June 5, 2025     |                     | Yes       |
