# Hardware Abstraction Layer

```{toctree}
:hidden:
:glob:

*

```

The Hardware Abstraction Layer (HAL) defines chip hardware access interfaces that users can use to access underlying hardware resources. HAL interfaces are compatible across different chip series, facilitating application portability across different chip platforms.

```{only} SF32LB55X
The SF32LB55x series has two CPUs: HCPU for high-performance computing and LCPU for executing power-sensitive tasks. Users can run high-performance processing on HCPU, such as graphics display, while implementing low-power functions on LCPU, such as BLE and sensor data processing. This allows HCPU to be shut down when processing low-power tasks, reducing system power consumption.
```

```{only} SF32LB56X
The SF32LB56x series has two CPUs: HCPU for high-performance computing and LCPU for executing power-sensitive tasks. Users can run high-performance processing on HCPU, such as graphics display, while implementing low-power functions on LCPU, such as BLE and sensor data processing. This allows HCPU to be shut down when processing low-power tasks, reducing system power consumption.
```

```{only} SF32LB52X
The SF32LB52x series has two CPUs: HCPU for high-performance computing, and LCPU runs the Bluetooth protocol stack and is not open to applications.
```

```{only} SF32LB58X
The SF32LB58x series has three CPUs: HCPU for high-performance computing, ACPU can be used for Audio DSP processing, and LCPU for executing power-sensitive tasks. Users can run high-performance processing on HCPU, such as graphics display, while implementing low-power functions on LCPU, such as BLE and sensor data processing. This allows HCPU to be shut down when processing low-power tasks, reducing system power consumption.
```

Software running on HCPU can typically access LCPU hardware resources, but needs to ensure that LCPU is in an active running state and LCPU is not using these resources. When HCPU/ACPU software uses LCPU hardware, it needs to wake up LCPU first during sleep-wake cycles to ensure access to LCPU hardware resources, otherwise it will cause system access failures and crashes.

Software running on LCPU cannot use HCPU hardware resources and can only access some HCPU registers when HCPU/ACPU is in an active running state. This type of access is not recommended.

## HAL Porting

The following functions need to be implemented when porting to other real-time operating systems:
<!-- @ref HAL_PORT -->
