# Low-Power Development Guide

## 1 Introduction
```{only} SF32LB52X or SF32LB56X or SF32LB55X
The SiFli MCU is a dual-core Cortex‑M33 STAR SoC. The big core HCPU runs at 0~240 MHz and belongs to the HPSYS subsystem, suitable for high-performance tasks such as graphics, audio, and neural networks. The small core LCPU runs at 0~48 MHz and belongs to the LPSYS subsystem, suitable for tasks such as Bluetooth, sensor data collection, and computation.
```
```{only} SF32LB58X
The SF32LB58X is a tri-core chip (dual high-performance big cores + one low-power small core). The dual big cores run at up to 240 MHz with a single-core CoreMark score of up to 984 and power efficiency of 8.29uA/CoreMark, providing the graphics and audio computing power required by rich applications and smooth human-machine interaction. The small low-power core runs at up to 96 MHz with a CoreMark score of 394 and power efficiency of 3.88uA/CoreMark, serving as a low-power sensor hub while also running the Bluetooth protocol stack.
```
```{only} SF32LB57X
The SF32LB57X is a tri‑core SoC consisting of two high‑performance main cores plus one low‑power auxiliary core.
The dual high‑performance main cores run at a maximum frequency of 240 MHz, delivering a single‑core CoreMark score of up to 984. They provide the graphics and audio computing power required for feature‑rich applications and smooth human‑computer interaction. The low‑power LCPU operates at 0‑48 MHz. It belongs to the LPSYS subsystem and is dedicated to Bluetooth processing.
```

Refer to `example\pm\classical` for low-power development examples.

:::{only} SF32LB56X
For the power measurement report, see: [Power Measurement Report](https://docs.sifli.com/projects/rpt5602_sf32lb56x-low-power-measurement-report/latest/zh_CN/index.html)
:::

:::{only} SF32LB52X
For the power measurement report, see: [Power Measurement Report](https://docs.sifli.com/projects/rpt5202_sf32lb52x-low-power-measurement-report/latest/zh_CN/index.html)
:::

## 2 Configuring Low-Power Modes

### 2.1 Enabling Low-Power Mode
```{only} SF32LB52X or SF32LB57X
Run `sdk.py menuconfig` in the project directory to open the software configuration menu:

1. Enable the low-power feature (`Enable Low power support`):
    - Path: Sifli middleware → Enable Low power support
    - Enable: Enable Low power support
        - Macro switch: `BSP_USING_PM`
        - Effect: Enables the low-power feature

```{figure} ../../assets/enable_pm.png
:align: center
Figure  Enable Low Power configuration menu
```
```{only} SF32LB52X or SF32LB57X
2. Select the low-power mode (`Enable Deep Mode`):
    - Path: RTOS → RT-Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - Select: Enable Deep Mode
        - Macro switch: `PM_DEEP_ENABLE`
        - Effect: Selects Deep Sleep mode as the low-power mode

```{figure} ../../assets/deep.png
:align: center
Figure  Deep Sleep configuration menu
```
```{only} SF32LB52X or SF32LB57X
3. Enable the low-power debug switch. Once enabled, low-power related logs will be printed (optional; the low-power log printing takes time and affects power consumption) (`Enable PM Debug`)
    - Path: Sifli middleware → Enable Low power support -> Enable PM Debug
        - Select: Enable PM Debug
            - Macro switch: `BSP_PM_DEBUG`
            - Effect: Enables the low-power log debug switch

```{figure} ../../assets/low_power11.png
:align: center
Figure  Debug configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
Run `sdk.py menuconfig` in the project directory to open the software configuration menu:

1. Enable the low-power feature (`Enable Low power support`):
    - Path: Sifli middleware → Enable Low power support
    - Enable: Enable Low power support
        - Macro switch: `BSP_USING_PM`
        - Effect: Enables the low-power feature

```{figure} ../../assets/enable_pm.png
:align: center
Figure Enable Low Power configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
2. Select the low-power mode (`Enable Standby Mode`):
    - Path: RTOS → RT-Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - Select: Enable Standby Mode
        - Macro switch: `PM_Standby_ENABLE`
        - Effect: Selects Standby Sleep mode as the low-power mode

```{figure} ../../assets/stabdby.png
:align: center
Figure  Standby Sleep configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
3. Enable the low-power debug switch. Once enabled, low-power related logs will be printed (optional; the low-power log printing takes time and affects power consumption) (`Enable PM Debug`)
    - Path: Sifli middleware → Enable Low power support -> Enable PM Debug
        - Select: Enable PM Debug
            - Macro switch: `BSP_PM_DEBUG`
            - Effect: Enables the low-power log debug switch

```{figure} ../../assets/low_power11.png
:align: center
Figure  Debug configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
- Note: If STANDBY mode is enabled on the LCPU, STANDBY mode must also be enabled on the HCPU.
```
4) After configuration, confirm that the project configuration file `rtconfig.h` contains the following definitions:
```{only} SF32LB52X or SF32LB57X
```c
#define RT_USING_PM 1           // Enable the PM module
#define PM_DEEP_ENABLE 1        // DEEP sleep mode
#define BSP_USING_PM 1          // Enable the PM module
#define BSP_PM_DEBUG 1          // Print PM[S], PM[W] logs (optional)
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
```c
#define RT_USING_PM 1           // Enable the PM module
#define PM_STANDBY_ENABLE 1     // STANDBY sleep mode
#define BSP_USING_PM 1          // Enable the PM module
#define BSP_PM_DEBUG 1          // Print PM[S], PM[W] logs (optional)
```

### 2.2 Disabling Low-Power Mode

Run `sdk.py menuconfig` in the project directory, then reverse the steps described in Enabling Low-Power Mode to disable the corresponding options.

## 3 Key Wakeup Configuration

Refer to the `SDK\example\pm\classical` example for the configuration method.

**Standby wakeup configuration (for standby/deep wakeup)**

The following is an example of the HAL-layer standby wakeup APIs. You can use `HAL_LPAON_QueryWakeupPin()` and `HAL_HPAON_QueryWakeupPin()` to get the PIN number corresponding to a wakeup pin. If IO wakeup needs to process events, a GPIO interrupt must also be configured:

```{only} SF32LB55X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 55x PA80 #WKUP_A3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 55x PB48 #WKUP_PIN5
// To check whether the configuration takes effect, refer to the corresponding registers in the chip manual:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

```{only} SF32LB52X
For the 52 series, no additional wakeup pin configuration is required for sleep in deepsleep mode; all pins can wake up the system, and the wakeup source goes through WSR_GPIO1.
```

```{only} SF32LB56X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 56x PB35 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 56x PA50 #WKUP_PIN5
// To check whether the configuration takes effect, refer to the corresponding registers in the chip manual:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```
```{only} SF32LB58X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 58x PB57 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 58x PB59 #WKUP_PIN5
// To check whether the configuration takes effect, refer to the corresponding registers in the chip manual:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```
```{only} SF32LB57X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN10, AON_PIN_MODE_LOW);        // 57x PA24 #WKUP_PIN10
// To check whether the configuration takes effect, read the PMUC registers: on the 57 series, PIN wakeup sources
// go through the PMUC (enable in PMUC WER, trigger mode in PMUC WKUP_MODE, status in PMUC WSR, address
// 0x500ca008); HPSYS AON WER/WSR only covers RTC/LPTIM/IWDT/Mailbox and other non-PIN sources:
rt_kprintf("pmuc wsr:0x%x,wer:0x%x,mode:0x%x,\n", hwp_pmuc->WSR, hwp_pmuc->WER, hwp_pmuc->WKUP_MODE); // hcpu
```

**Power-off wakeup configuration (for hibernate wakeup)**
```{only} SF32LB55X
For 55 series MCUs: In Hibernate mode, only the LCPU wakeup PIN0‑5 have wakeup capability; see the PMUC WER register configuration in the 55 series user manual for details.
```c
// Configuration method for 55x:
HAL_PMU_EnablePinWakeup(5, AON_PIN_MODE_NEG_EDGE); // 55x PB48 #WKUP_PIN5
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```

```{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X
For MCUs after the 55 series: Two wakeup sources, PIN0 and PIN1, can exist simultaneously, and each wakeup source can be mapped to any HCPU/LCPU wakeup pin; see the PMUC CR register configuration in the user manual for details.
```c
// Configuration method for 58x/56x/52x/57x:
HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); // select PA34 to wake_pin0
HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);                                  // enable wake_pin0 
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```



After Hibernate power-off, waking up is equivalent to a cold boot (but with the `PM_HIBERNATE_BOOT` flag set), unlike Standby wakeup, which resumes the original program from where it left off. The wakeup PIN and level mode are controlled by PMU registers; you can print the WER/CR registers to verify. If the same IO is used for both standby and power-off wakeup, both configurations must take effect.

## 4 Debugging Low-Power Modes

### 4.1 Low-Power Modes

- `PM_SLEEP_MODE_IDLE`:
The CPU enters idle mode. The CPU stops at WFI or WFE, and high-speed clocks (HRC/HXT/DBLR/DLL) are present in the system. All peripherals can be enabled and generate interrupts.
```{only} SF32LB52X 
- `PM_SLEEP_MODE_LIGHT`:
The CPU enters light sleep. The CPU stops at the WFI instruction, high-speed clocks are switched off, and CPU-related peripherals stop working (but remain powered). The system switches to the 32K clock.
The system can be woken up by the low-power timer (LPTIM), RTC, BLE MAC (LCPU only), Mailbox (from the other CPU), or a pin. Wakeup time is 30us-100us.
After wakeup, execution continues from the instruction after WFI.
```
```{only} SF32LB56X or SF32LB55X or SF32LB58X or SF32LB57X 
- `PM_SLEEP_MODE_LIGHT`:
The CPU enters light sleep. The CPU stops at the WFI instruction, high-speed clocks are switched off, and CPU-related peripherals stop working (but remain powered). The system switches to the 32K clock.
The system can be woken up by the low-power timer (LPTIM), RTC, BLE MAC (LCPU only), Mailbox (from the other CPU), or a specific wakeup pin. Wakeup time is 30us-100us.
After wakeup, execution continues from the instruction after WFI.
```
- `PM_SLEEP_MODE_DEEP`:
Same as PM_SLEEP_MODE_LIGHT, except that the system power supply switches to RET_LDO and the wakeup time increases to 100us-1ms.
After wakeup, execution continues from the instruction after WFI.

- `PM_SLEEP_MODE_STANDBY`:
The CPU enters standby mode. High-speed clocks are switched off, CPU-related peripherals are powered down, all RAM except the system-configured portion is unpowered, pin states are retained, and the system power supply switches to RET_LDO.
The system can be woken up by the low-power timer, RTC, BLE MAC (LCPU only), Mailbox (from the other CPU), or a specific wakeup pin. Wakeup time is 1ms-2ms.
After wakeup, the system reboots. Based on the low-power mode indication in the AON registers, the software determines that the system has booted from standby mode, distinguishing it from a cold boot.

See Table 4‑1 for the current consumption of each low-power mode. If PSRAM is present, the HCPU backs up the data that must be retained in the power-down RAM to PSRAM before entering sleep and restores it after wakeup; without PSRAM, the data is backed up to the 64KB Retention RAM. Unless otherwise specified below, "entering sleep" refers to entering a low-power mode other than IDLE, and "wakeup" refers to exiting a low-power mode other than IDLE.

Table 4‑1: Low-Power Modes
```{only} SF32LB52X 
| Low-Power Mode        | CPU state | Peripheral state | SRAM                                                     | Wakeup sources   | Wakeup time     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | Accessible                                               | Any interrupt | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained | RTC, wakeup PIN, IO(PA),<br>LPTIM1, Bluetooth | ~ 250us    |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, only 384KB retained | RTC, wakeup PIN, <br>LPTIM1, Bluetooth | ~ 1ms    | 
```
```{only} SF32LB55X 
| Low-Power Mode        | CPU state | Peripheral state | SRAM                                                     | Wakeup sources   | Wakeup time     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | Accessible                                               | Any interrupt | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained |RTC, wakeup PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2| ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, only 64KB retained | RTC, wakeup PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```
```{only} SF32LB56X 
| Low-Power Mode        | CPU state | Peripheral state | SRAM                                                     | Wakeup sources   | Wakeup time     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | Accessible                                               | Any interrupt | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained |RTC, wakeup PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2| ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, only 64KB retained | RTC, wakeup PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```
```{only} SF32LB58X 
| Low-Power Mode        | CPU state | Peripheral state | SRAM                                                     | Wakeup sources   | Wakeup time     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | Accessible                                               | Any interrupt | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained | RTC, wakeup PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2 | ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, only 64KB retained | RTC, wakeup PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```
```{only} SF32LB57X 
| Low-Power Mode        | CPU state | Peripheral state | SRAM                                                     | Wakeup sources   | Wakeup time     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | Accessible                                               | Any interrupt | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained |RTC, wakeup PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2| ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS: inaccessible, all retained<br>HPSYS: inaccessible, all retained | RTC, wakeup PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```

### 4.2 Power-Off Modes

In addition to the four low-power modes provided by each subsystem, the chip also provides two system-level power-off modes:
- Hibernate: All subsystems are powered down and the system switches to the 32K crystal. It can be woken up by PIN and RTC (RTC wakeup time is accurate). Interface: `HAL_PMU_EnterHibernate`.
- Shutdown: All subsystems are powered down and the system switches to RC10K. It can be woken up by PIN and RTC (RTC wakeup time is inaccurate). Interface: `HAL_PMU_EnterShutdown`.

Table 4‑2: Power-Off Modes

| Low-Power Mode | CPU state | Peripheral state | SRAM        | IO  | Wakeup sources    | Wakeup time |
|------------|----------|----------|-------------|-----|-----------|----------|
| Hibernate  | reset    | reset    | Data not retained  | High-impedance| RTC and PIN| >2ms     | 
| Shutdown   | reset    | reset    | Data not retained  | High-impedance| RTC and PIN| >2ms     | 

Note: The current figures are for reference only; actual values vary with peripheral enablement and IO settings. "stop" means the module stops working and can resume without reconfiguration after exiting low power; "reset" means the module has been reset after exiting (the CPU starts from ROM and peripherals need to be reconfigured).

If sleep conditions are not met but power still needs to be reduced, refer to WFI Automatic Frequency Scaling and Scenario-Based Dynamic Frequency Scaling.
### 4.3 WFI Automatic Frequency Scaling
When the `IDLE` thread is entered but sleep conditions are not met, the current during `WFI` can be reduced by lowering the HCPU frequency. The precondition for frequency reduction: no high-speed peripheral is active. High-speed peripherals include:
- EPIC
- EZIP
- LCDC
- USB
- SD

Note: The busy detection for EPIC/EZIP is integrated into the LVGL implementation bundled with the SDK; if that implementation is not used, call `rt_pm_hw_device_start`/`rt_pm_hw_device_stop` respectively when a peripheral starts/stops working to indicate busy/idle, avoiding WFI frequency reduction while busy. The busy detection for LCDC/USB/SD is integrated into the RT-Thread LCD Device driver.
```{only} SF32LB52X 
The WFI frequency after reduction is configured by `HAL_RCC_HCPU_SetDeepWFIDiv`. When an audio peripheral is active, the frequency can only be reduced to 48MHz; otherwise it can be reduced to 4MHz. In addition, the `HPSYS_RCC_DBGR_FORCE_HP` bit of `hwp_hpsys_rcc->DBGR` must be set to 1.
```
```{only} SF32LB57X
The WFI frequency after reduction is configured by `HAL_RCC_HCPU_SetDeepWFIDiv`. When an audio peripheral is active, the frequency can only be reduced to 48MHz; otherwise it can be reduced to 4MHz.
```
```{only} SF32LB56X or SF32LB58X
The WFI frequency after reduction is configured by `HAL_RCC_HCPU_SetDeepWFIDiv`. When an audio peripheral is active, the frequency can only be reduced to 48MHz; otherwise it can be reduced to 1MHz.
```
### 4.4 Scenario-Based Dynamic Frequency Scaling
For scenarios that do not require high performance, the big core can reduce its frequency and voltage to lower active power consumption (for example, after the screen is off, running only the wrist-raise algorithm can drop to 48MHz). Although a lower frequency lengthens execution time, the total energy (current × time) may be lower. The optimal mode can be measured in different scenarios and chosen accordingly.

Use `rt_pm_run_enter` to configure the current run mode. The HCPU supports the following four modes (the application starts with `PM_RUN_MODE_HIGH_SPEED` by default); switching to a high-speed mode takes effect immediately, while switching to a low-speed mode is deferred until the IDLE thread runs:

| Mode                      | System clock (MHz) |
|---------------------------|-----------------|
| PM_RUN_MODE_HIGH_SPEED    | 240             |
| PM_RUN_MODE_NORMAL_SPEED  | 144             |
| PM_RUN_MODE_MEDIUM_SPEED  | 48              |
| PM_RUN_MODE_LOW_SPEED     | 24              |

The SDK also provides `pm_scenario_start`/`pm_scenario_stop` for convenient per-scenario switching. Currently `UI` and `Audio` are supported:
- If either is active → HIGH_SPEED is used;
- If neither `UI` nor `Audio` is active → MEDIUM_SPEED is used.
### 4.5 Low-Power Flow

In this solution, HPSYS can enter sleep mode only after the screen is off; with the screen on, HPSYS can only enter IDLE mode when the HCPU is idle. LPSYS can enter sleep only after HPSYS has entered sleep; if HPSYS is not asleep, LPSYS can only enter IDLE even when the LCPU is idle (with the exception of the 52 series, where the HCPU and LCPU can sleep independently). Once HPSYS is asleep, LPSYS can freely enter and exit sleep without waking HPSYS.

#### 4.5.1 Screen Off

The screen lock time can be selected in the settings interface. When the screen has been idle for longer than the lock time, the screen turns off. Sleep conditions are checked in the IDLE thread; if they are met, HPSYS enters sleep, and LPSYS can then also enter sleep.

```{figure} ../../assets/low_power13.png
:align: center
Figure 4.1 Screen-off flow
```

#### 4.5.2 HPSYS Wakeup
```{only} SF32LB52X 
HPSYS can be woken up by the low-power timer (LPTIM), RTC, BLE MAC (LCPU only), Mailbox (from the other CPU), or any pin in deepsleep mode. For example, when a key is pressed, HPSYS wakes up.
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X or SF32LB57X
HPSYS can be woken up in the following ways: low-power timer (LPTIM), RTC, BLE MAC (LCPU only), Mailbox (from the other CPU), or a specific wakeup pin. For example, after the key pin wakeup function is enabled, HPSYS wakes up when the key is pressed.
```
Taking key-wakeup screen-on as an example, the flow is shown in Figure 4.2; after the screen turns on, a new round of screen-off judgment begins. The wakeup flow triggered by the setting event from the phone APP is shown in Figure 4.3; after the Setting request is processed, the system returns to the IDLE thread and can enter sleep immediately.

```{figure} ../../assets/low_power14.png
:align: center
Figure 4.2 Key screen-on/screen-off flow
```

```{figure} ../../assets/low_power25.png
:align: center
Figure 4.3 Wakeup flow upon receiving a phone Setting event
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X or SF32LB57X
#### 4.5.3 LPSYS Wakeup

LPSYS can be woken up by the following events:
- Key press
- HPSYS wakeup
- Sensor data acquisition timer timeout
- BLE periodic timer timeout


#### 4.5.3 Cross-Power-Domain Peripheral Sharing and Sleep Constraints

The SiFli HCPU and LCPU belong to different power domains and respectively control the peripherals in their own power domains. To support resource sharing:
- When the LCPU is in IDLE/ACTIVE state (not asleep), the HCPU can use peripherals in the LCPU power domain; however, both cores must be prevented from using the same peripheral at the same time, otherwise hardware resource conflicts occur during parallel execution.
- A common scenario is the HCPU using an LCPU peripheral. In this case, as long as the HCPU needs the peripheral, the LCPU must stay awake (must not enter sleep); otherwise the access will fail.
- If the LCPU enters standby mode, its related peripherals lose power, which in turn constrains the minimum low-power mode of the HCPU: the HCPU must also take the standby path. In terms of wakeup order, the LCPU must first be woken up and the related peripherals (including LCPU peripherals) reinitialized; only then can the HCPU use these resources normally.
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
Regarding context retention in standby:
- HCPU: Before standby, the data/context that must be retained is backed up to PSRAM; after wakeup, the running context is restored from PSRAM (without PSRAM, it relies only on the HPSYS Retention area).
- LCPU: All RAM remains powered, and the CPU register context is saved in RAM; after wakeup, the context is restored directly from RAM.
```
```{only} SF32LB52X or SF32LB57X
Regarding context retention in standby:
- HCPU: In DEEPSLEEP mode, all RAM is retained, and after wakeup the context is restored directly from RAM.
- LCPU: All RAM remains powered, and the CPU register context is saved in RAM; after wakeup, the context is restored directly from RAM.
```
### 4.6 Log Interpretation

The HCPU and LCPU output logs through the console. After enabling the low-power debug switch as described in Section 2.1, you can search for the following keywords in the logs to analyze the flow.

Table 4‑3: Log Keyword Interpretation

| Log          | Meaning                                         |
|---------------|----------------------------------------------|
| gui_suspend   | Screen off                                    |
| gui_resume    | Screen on                                     |
| [pm]S: mode,gtime | Enter sleep; mode=2 means LIGHT, 4 means STANDBY; gtime unit is 32768Hz |
| [pm]W: gtime  | Exit sleep; gtime unit is 32768Hz             |
| [pm]WSR:0xXXX | Wakeup reason (interpreted by register bit meaning) |

`gtime` is synchronized between the HCPU and LCPU sides. For example, if the system enters sleep at 2136602 and wakes up at 2142330, the sleep duration is `sleep_time=(2142330-2136602)/32768=175ms`; if `WSR=0x200`, it means the wakeup was triggered by a mailbox interrupt from LPSYS. See the chip manual for the meaning of each WSR bit.

```{figure} ../../assets/low_power15.png
:align: center
Figure 4.4 Low-power log example
```
```{only} SF32LB55X
* HPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | LPTIM1 wakeup            |
| [2]    | PIN0 wakeup              |
| [3]    | PIN1 wakeup              |
| [4]    | PIN2 wakeup              |
| [5]    | PIN3 wakeup              |
| [8]    | LPSYS wakes up HPSYS manually   |
| [9]    | LPSYS wakes up HPSYS via Mailbox |

* LPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | LPTIM2 wakeup            |
| [2]    | LPCOMP1 wakeup           |
| [3]    | LPCOMP2 wakeup           |
| [4]    | BLE wakeup               |
| [5]    | PIN0 wakeup              |
| [6]    | PIN1 wakeup              |
| [7]    | PIN2 wakeup              |
| [8]    | PIN3 wakeup              |
| [9]    | PIN4 wakeup              |
| [10]   | PIN5 wakeup              |
| [11]   | HPSYS wakes up LPSYS manually   |
| [12]   | HPSYS wakes up LPSYS via Mailbox |
```
```{only} SF32LB56X
* HPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | GPIO1 wakeup            |
| [2]    | LPTIM1 wakeup              |
| [6]    | LPSYS wakes up HPSYS manually           |
| [7]    | LPSYS wakes up HPSYS via Mailbox  |
| [8]    | PIN0 wakeup   |
| [9]    | PIN1 wakeup   |
| [10]    | PIN2 wakeup   |
| [11]    | PIN3 wakeup   |
| [12]    | PIN4 wakeup   |
| [13]    | PIN5 wakeup   |
| [14]    | PIN6 wakeup   |
| [15]    | PIN7 wakeup   |
| [16]    | PIN8 wakeup   |
| [17]    | PIN9 wakeup   |
| [18]    | PIN10 wakeup   |
| [19]    | PIN11 wakeup   |
| [20]    | PIN12 wakeup   |
| [21]    | PIN13 wakeup   |


* LPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | GPIO2 wakeup            |
| [2]    | LPTIM2 wakeup              |
| [3]    | LPCOMP1 wakeup              |
| [4]    | LPCOMP2 wakeup            |
| [5]    | BT wakeup  |
| [6]    | HPSYS wakes up LPSYS manually   |
| [7]    | HPSYS wakes up LPSYS via Mailbox |
| [8]    | PIN0 wakeup   |
| [9]    | PIN1 wakeup   |
| [10]    | PIN2 wakeup   |
| [11]    | PIN3 wakeup   |
| [12]    | PIN4 wakeup   |
| [13]    | PIN5 wakeup   |
| [14]    | PIN6 wakeup   |
| [15]    | PIN7 wakeup   |
| [16]    | PIN8 wakeup   |
| [17]    | PIN9 wakeup   |
| [18]    | PIN10 wakeup   |
| [19]    | PIN11 wakeup   |
| [20]    | PIN12 wakeup   |
| [21]    | PIN13 wakeup   |
```
```{only} SF32LB58X
* HPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | GPIO1 wakeup            |
| [2]    | LPTIM1 wakeup              |
| [3]    | LPCOMP wakeup              |
| [6]    | LPSYS wakes up HPSYS manually           |
| [7]    | LPSYS wakes up HPSYS via Mailbox  |
| [8]    | PIN0 wakeup   |
| [9]    | PIN1 wakeup   |
| [10]    | PIN2 wakeup   |
| [11]    | PIN3 wakeup   |
| [12]    | PIN4 wakeup   |
| [13]    | PIN5 wakeup   |
| [14]    | PIN6 wakeup   |
| [15]    | PIN7 wakeup   |
| [16]    | PIN8 wakeup   |
| [17]    | PIN9 wakeup   |
| [18]    | PIN10 wakeup   |
| [19]    | PIN11 wakeup   |
| [20]    | PIN12 wakeup   |
| [21]    | PIN13 wakeup   |
| [22]    | PIN14 wakeup   |
| [23]    | PIN15 wakeup   |
| [24]    | PIN16 wakeup   |
| [25]    | PIN17 wakeup   |


* LPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | GPIO2 wakeup            |
| [2]    | LPTIM2 wakeup              |
| [3]    | LPCOMP1 wakeup              |
| [4]    | LPCOMP2 wakeup            |
| [5]    | BT wakeup  |
| [6]    | HPSYS wakes up LPSYS manually   |
| [7]    | HPSYS wakes up LPSYS via Mailbox |
| [8]    | PIN0 wakeup   |
| [9]    | PIN1 wakeup   |
| [10]    | PIN2 wakeup   |
| [11]    | PIN3 wakeup   |
| [12]    | PIN4 wakeup   |
| [13]    | PIN5 wakeup   |
| [14]    | PIN6 wakeup   |
| [15]    | PIN7 wakeup   |
| [16]    | PIN8 wakeup   |
| [17]    | PIN9 wakeup   |
| [18]    | PIN10 wakeup   |
| [19]    | PIN11 wakeup   |
| [20]    | PIN12 wakeup   |
| [21]    | PIN13 wakeup   |
| [22]    | PIN14 wakeup   |
| [23]    | PIN15 wakeup   |
| [24]    | PIN16 wakeup   |
| [25]    | PIN17 wakeup   |
```
```{only} SF32LB57X
* HPSYS WSR meaning (PIN wakeup bits come from PMUC_WSR)

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | PIN0 wakeup (PA33)        |
| [1]    | PIN1 wakeup (PA34)        |
| [2]    | PIN2 wakeup (PA35)        |
| [3]    | PIN3 wakeup (PA36)        |
| [4]    | PIN4 wakeup (PA37)        |
| [5]    | PIN5 wakeup (PA38)        |
| [6]    | PIN6 wakeup (PA39)        |
| [7]    | PIN7 wakeup (PA40)        |
| [8]    | PIN8 wakeup (PA41)        |
| [9]    | PIN9 wakeup (PA42)        |
| [10]   | PIN10 wakeup (PA24)       |
| [11]   | PIN11 wakeup (PA25)       |
| [12]   | PIN12 wakeup (PA26)       |
| [13]   | PIN13 wakeup (PA27)       |
| [16]   | RTC wakeup                |
| [17]   | IWDT wakeup               |
| [18]   | GPIO1 wakeup              |
| [19]   | LPTIM1 wakeup             |
| [21]   | LP2HP_WDT wakeup          |
| [22]   | LPSYS wakes up HPSYS manually    |
| [23]   | LPSYS wakes up HPSYS via Mailbox |
| [25]   | CHG wakeup                |

* LPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | BT wakeup                 |
| [1]    | WDT2 wakeup               |
| [2]    | LPTIM3 wakeup             |
| [6]    | HPSYS wakes up LPSYS manually    |
| [7]    | HPSYS wakes up LPSYS via Mailbox |
```
```{only} SF32LB52X
* HPSYS WSR meaning

| Bit field | Meaning                    |
|--------|-------------------------|
| [0]    | RTC wakeup                |
| [1]    | GPIO1 wakeup            |
| [2]    | LPTIM1 wakeup              |
| [3]    | PMUC wakeup              |
| [6]    | LPSYS wakes up HPSYS manually           |
| [7]    | LPSYS wakes up HPSYS via Mailbox  |
| [8]    | PIN0 wakeup   |
| [9]    | PIN1 wakeup   |
| [10]    | PIN2 wakeup   |
| [11]    | PIN3 wakeup   |
| [18]    | PIN10 wakeup   |
| [19]    | PIN11 wakeup   |
| [20]    | PIN12 wakeup   |
| [21]    | PIN13 wakeup   |
| [22]    | PIN14 wakeup   |
| [23]    | PIN15 wakeup   |
| [24]    | PIN16 wakeup   |
| [25]    | PIN17 wakeup   |
| [26]    | PIN18 wakeup   |
| [27]    | PIN19 wakeup   |
| [28]    | PIN20 wakeup   |
```

```{only} SF32LB58X
Table 4‑4: 55 Series HPSYS Wakeup PIN Mapping Table

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PA77     |
| PIN1 | PA78     |
| PIN2 | PA79     |
| PIN3 | PA80     |

Table 4‑5: 55 Series LPSYS Wakeup PIN Mapping Table

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PB43     |
| PIN1 | PB44     |
| PIN2 | PB45     |
| PIN3 | PB46     |
| PIN4 | PB47     |
| PIN5 | PB48     |
```
```{only} SF32LB56X
Table 4‑4: 56 Series HPSYS Wakeup PIN Mapping Table (partial)

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PB32     |
| PIN1 | PB33     |
| PIN2 | PB34     |
| PIN3 | PB35     |

Table 4‑5: 56 Series LPSYS Wakeup PIN Mapping Table (partial)

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PB54     |
| PIN1 | PB55     |
| PIN2 | PB56     |
| PIN3 | PB57     |
| PIN4 | PB58     |
| PIN5 | PB59     |
```
```{only} SF32LB58X
Table 4‑4: 58 Series HPSYS Wakeup PIN Mapping Table (partial)

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PB54     |
| PIN1 | PB55     |
| PIN2 | PB56     |
| PIN3 | PB57     |

Table 4‑5: 58 Series LPSYS Wakeup PIN Mapping Table (partial)

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PB32     |
| PIN1 | PB33     |
| PIN2 | PB34     |
| PIN3 | PB35     |
| PIN4 | PB36     |
| PIN5 | PA50     |
```
```{only} SF32LB57X
Table 4‑4: 57 Series HPSYS Wakeup PIN Mapping Table

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PA33     |
| PIN1 | PA34     |
| PIN2 | PA35     |
| PIN3 | PA36     |
| PIN4 | PA37     |
| PIN5 | PA38     |
| PIN6 | PA39     |
| PIN7 | PA40     |
| PIN8 | PA41     |
| PIN9 | PA42     |
| PIN10| PA24     |
| PIN11| PA25     |
| PIN12| PA26     |
| PIN13| PA27     |

```

```{only} SF32LB52X
Table 4‑4: 52 Series HPSYS Wakeup PIN Mapping Table (partial)

| Wakeup | PIN Meaning |
|------|----------|
| PIN0 | PA24     |
| PIN1 | PA25     |
| PIN10| PA34     |
| PIN11| PA35     |
| PIN19| PA43     |
```

### 4.7 Common Issues Analysis

Because SWD cannot connect after entering sleep mode, UART must be used as the console port to capture logs for analysis.

#### 4.7.1 Has the System Entered Sleep Mode?

If any of the following conditions is met, HPSYS has most likely entered sleep:
- SWD cannot connect
- The HCPU console does not respond
- The HCPU log contains "S: mode, gtime"

If any of the following conditions is met, LPSYS has most likely entered sleep:
- The LCPU console does not respond
- The LCPU log contains "S: mode, gtime"

Make sure the finsh shell option in the LCPU Command shell has been enabled.

The current low-power mode can also be determined by measuring the chip power pin voltages:
- HPSYS in active/sleep/deepsleep: `LDO1_VOUT`≈1.1V; HPSYS in standby: `LDO1_VOUT` gradually drops to 0V.
- LPSYS in active/sleep/deepsleep: `LDO2_VOUT` or `BUCK2_VOUT`≈0.9V; LPSYS in standby: the corresponding voltage gradually drops to 0V.
- Entering hibernate: `LDO1_VOUT/LDO2_VOUT/BUCK2_VOUT/VDD_RET` all drop to 0V.

```{only} SF32LB55X
See Figure 4.5 for the power pin voltages in low-power modes of the 55 series (source: SF32LB55x User Manual 4.2.9).

```{figure} ../../assets/low_power16.png
:align: center
Figure 4.5 Power pin voltages in low-power modes
```

#### 4.7.2 Why Didn't the System Enter Sleep Mode?

**Sleep entry conditions**

For applications, the switching between sleep and active modes is transparently controlled by the lowest-priority `IDLE` thread: when all higher-priority threads have no work, `IDLE` is scheduled and checks whether all of the following conditions are met before entering sleep:

- Sleep mode is not blocked (no unreleased `rt_pm_request(PM_SLEEP_MODE_IDLE)` exists)
- The time of the nearest OS timer that will expire is greater than the threshold (100ms by default)
- No wakeup condition is active (for example, enabled wakeup sources are currently inactive)
- Data sent to the small core has been consumed (the IPC queue has no unconsumed data)

**Timer wakeup configuration before sleep**

Before entering sleep, `LPTIM` is configured according to the timeout of the "nearest timer" so that it generates an interrupt to wake up the big core at that moment. For example, if the nearest timer expires in 200ms, `LPTIM` is configured to interrupt after 200ms, ensuring that the OS timer callback still fires on time during sleep.

**Sleep block/release APIs**

An application can explicitly block sleep in a critical section, and the driver framework also blocks it automatically while a peripheral is active, to avoid accidentally falling asleep during interrupt processing:

```c
// Block sleep until released
rt_pm_request(PM_SLEEP_MODE_IDLE);
// ... critical section / peripheral operations ...
rt_pm_release(PM_SLEEP_MODE_IDLE);
```

**Timer threshold and policy**

Refer to the `configuration` in Power Management to enable low-power support for the project. Different low-power policies can enter different low-power modes. The current default policy of the system is:

Default policy example:

```c
static const pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},                  // Enter light sleep when idle for more than 15ms
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},             // Enter standby when idle for more than 10s
#endif /* PM_STANDBY_ENABLE */
};
```

Common cause checklist (similar for HCPU/LCPU):

1) Enable the PM module and confirm the macro definitions:

```{only} SF32LB52X or SF32LB57X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // Enable low-power mode
#define PM_DEEP_ENABLE 1       // Enter Deep low power
#define BSP_PM_DEBUG 1          // Enable low-power debug logs
```
```{only} SF32LB56X or SF32LB58X or SF32LB55X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // Enable low-power mode
#define PM_STANDBY_ENABLE 1     // Enter standby low power
#define BSP_PM_DEBUG 1          // Enable low-power debug logs
```
2) Confirm the CPU is idle and has entered the idle thread:
- Use the finsh command `list_thread` to check thread states; except `tshell` and `tidle`, which are ready, all others should be suspend, otherwise the IDLE thread cannot run.

```{figure} ../../assets/low_power17.png
:align: center
Figure 4.6 Information returned by the list_thread command
```

3) Confirm sleep is not blocked:
- Run `pm_dump` in the console; if the "Idle Mode Counter" > 0, some module has called `rt_pm_request(PM_SLEEP_MODE_IDLE)` to block sleep and must be released with the corresponding `rt_pm_release(PM_SLEEP_MODE_IDLE)`.

```{figure} ../../assets/low_power18.png
:align: center
Figure 4.7 Information returned by the pm_dump command
```

4) Confirm the OS timer timeout is greater than the sleep threshold:
Send the command `list_timer` in the console to display all created OS timers. Compare the timeout of activated timers with the sleep threshold; if a timeout is smaller than the threshold, that timer prevents the system from entering sleep. The unit of the OS timer timeout is ms.
```{figure} ../../assets/low_power19.png
:align: center
Figure 4.8 Information returned by the list_timer command
```
See the configuration below: the HPSYS sleep threshold is 100ms by default, and the LPSYS sleep threshold is 10ms.
```c
RT_WEAK const pm_policy_t pm_policy[] =
{
#ifdef PM_STANDBY_ENABLE
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_STANDBY}, //Hcpu: if no timer wakeup within 100ms, enter standby sleep
#else
    {10, PM_SLEEP_MODE_STANDBY}, //Lcpu: if no timer wakeup within 10ms, enter standby sleep
#endif /* SOC_BF0_HCPU */
#elif defined(PM_DEEP_ENABLE)
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_DEEP}, //Hcpu: if no timer wakeup within 100ms, enter Deep sleep
#else
    {10, PM_SLEEP_MODE_DEEP}, //Lcpu: if no timer wakeup within 10ms, enter Deep sleep
#endif /* SOC_BF0_HCPU */
#else
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_LIGHT},
#else
```
If the HCPU code contains a 90ms periodic delay, the system will never sleep:

```c
while (1)
{
    rt_thread_delay(90); // 90ms delay
}
```

Note the difference between the delay functions:
- HAL layer (no thread switching during the delay):

```c
HAL_Delay(10);     // 10ms
HAL_Delay_us(10);  // 10us
```

- RT-Thread APIs (switch to other threads, may trigger idle → sleep):

```c
rt_thread_delay(100); // 100ms
```

5) Confirm there is no pending wakeup source:
- Read the registers with commands to verify `WER/WSR`:
```{only} SF32LB55X
55 series WSR addresses:
```text
regop unlock 0000
regop read 4007001c 1   # LPSYS WSR
regop read 4003001c 1   # HPSYS WSR
```
```{only} SF32LB52X
52 series WSR addresses:
```text
regop unlock 0000
regop read 40040024 1   # LPSYS WSR
regop read 500c0024 1   # HPSYS WSR
```
```{only} SF32LB56X
56 series WSR addresses:
```text
regop unlock 0000
regop read 50040020 1   # LPSYS WSR
regop read 40040020 1   # HPSYS WSR
```
```{only} SF32LB58X
58 series WSR addresses:
```text
regop unlock 0000
regop read 50040020 1   # LPSYS WSR
regop read 40040020 1   # HPSYS WSR
```
```{only} SF32LB57X
57 series WSR addresses:
```text
regop unlock 0000
regop read 4004001c 1   # LPSYS WSR
regop read 500c001c 1   # HPSYS WSR
regop read 500ca008 1   # PMUC WSR (PIN wake‑up bits are located here, corresponding to WKUP_PIN0‑13)

Note: For the SF32LB57X series, the PIN wake‑up bits of HPSYS are **not located in HPSYS AON WSR**, but are mapped to `PMUC WSR` (`hwp_pmuc->WSR`, address `0x500ca008`), which corresponds to WKUP_PIN0‑13. Therefore, this register shall be read when troubleshooting PIN wake‑up issues. The `HPSYS AON WSR` and `LPSYS AON WSR` only contain wake‑up bits for RTC/LPTIM/IWDT/Mailbox and other sources.
```

- You can also read registers via Jlink/SifliUsartServer or print them through logs:

```c
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

Common issue: the wakeup pin level is misconfigured (for example, low-level wakeup but the level stays low).

6) Confirm the data sent to the other core has been consumed:
- You can connect via Ozone and dump memory to inspect with trace32, or print the `read_idx_mirror/write_idx_mirror` of the `ipc_ctx` queues through logs; if they are not equal, there is uncollected data that blocks sleep.

```{figure} ../../assets/low_power20.png
:align: center
Figure 4.9 Non-empty ring buffer case
```

```{figure} ../../assets/low_power21.png
:align: center
Figure 4.10 Empty ring buffer case
```

Example printing the active queue indices:

```c
for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
{
    if (ipc_ctx.queues[i].active == true)
    {
        if (ipc_ctx.queues[i].rx_ring_buffer && ipc_ctx.queues[i].tx_ring_buffer)
        {
            LOG_I("ipc_ctx.queues[%d].tx read_idx_mirror=0x%x, write_idx_mirror=0x%x\n",
                  i,
                  ipc_ctx.queues[i].tx_ring_buffer->read_idx_mirror,
                  ipc_ctx.queues[i].tx_ring_buffer->write_idx_mirror);
        }
    }
}
```

```{figure} ../../assets/low_power22.png
:align: center
Figure 4.11 Example of a missing channel caused by the LCPU data service not being enabled
```


## 5 Power Optimization

### 5.1 Standby Leakage Analysis

When both HPSYS and LPSYS have entered sleep, the focus of whole-device power optimization is:
- Remove detachable components such as the screen, sensors, and charging IC first and measure the minimum system current;
- Software IO level misconfiguration causing voltage difference/floating leakage;
- The on-chip PSRAM/Flash and external NAND/Flash/eMMC not entering sleep.

If the hardware can measure current per rail, you can locate which rail of `VSYS/ VLDO2/ VLDO3/ VDD_SIP/ VDDIOA` is leaking to narrow down the scope.

#### 5.1.1 Peripheral Leakage

Common causes:
1) Board-level devices not powered off;
2) Board-level devices powered off, but improper chip pin settings cause current to back-feed from the chip pins into the board-level devices.

For cause 2), avoid: chip pins connected to power-off devices outputting high or enabling pull-ups. The recommended configurations of common peripherals in active/sleep states are as follows (no pin change is needed when the external circuit stays powered; switch to the low-power configuration when the external circuit is powered off).

Table 5‑1: Recommended Pin Settings

| Peripheral      | Pin           | Direction | Active state   | Sleep (external circuit powered) | Sleep (external circuit power-off)      |
|-----------|----------------|------|------------|-------------------------|----------------------------|
| PSRAM     | PSRAM_CLK      | O    | Digital output   | Digital output                | GPIO mode output low            |
| PSRAM     | PSRAM_CLKB     | O    | Digital output   | Digital output                | GPIO mode output low            |
| PSRAM     | PSRAM_CS       | O    | Digital output   | Digital output                | GPIO mode output low            |
| PSRAM     | PSRAM_DM0      | O    | Digital output   | Digital output                | GPIO mode output low            |
| PSRAM     | PSRAM_DM1      | O    | Digital output   | Digital output                | GPIO mode output low            |
| PSRAM     | PSRAM_DQS0     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| PSRAM     | PSRAM_DQS1     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| PSRAM     | PSRAM_DQx      | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| QSPI      | QSPIx_CLK      | O    | Digital output   | Digital output                | GPIO mode output low            |
| QSPI      | QSPIx_CS       | O    | Digital output   | Digital output                | GPIO mode output low            |
| QSPI      | QSPIx_DIO0     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| QSPI      | QSPIx_DIO1     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| QSPI      | QSPIx_DIO2     | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| QSPI      | QSPIx_DIO3     | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| QSPI      | QSPIx_DIO4     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| QSPI      | QSPIx_DIO5     | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| QSPI      | QSPIx_DIO6     | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| QSPI      | QSPIx_DIO7     | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| USART     | USARTx_RXD     | I    | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| USART     | USARTx_TXD     | O    | Digital output   | Digital output                | Digital output                   |
| USART     | USARTx_CTS     | I    | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| USART     | USARTx_RTS     | O    | Digital output   | Digital output                | Digital output                   |
| I2C       | I2Cx_SCL       | I/O  | Digital input   | Digital input                | Digital input pull-down               |
| I2C       | I2Cx_SDA       | I/O  | Digital input   | Digital input                | Digital input pull-down               |
| SPI M     | SPIx_CLK       | O    | Digital output   | Digital output                | GPIO mode output low            |
| SPI M     | SPIx_CS        | O    | Digital output   | Digital output                | GPIO mode output low            |
| SPI M     | SPIx_DI        | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| SPI M     | SPIx_DO        | O    | Digital output   | Digital output                | GPIO mode output low            |
| SPI M     | SPIx_DIO       | I/O  | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| LCDC SPI  | LCDCx_SPI_CS   | O    | Digital output   | Digital output                | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_CLK  | O    | Digital output   | Digital output                | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_DIO0 | I/O  | Digital input pull-down | Digital input pull-down          | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_DIO1 | O    | Digital output   | Digital output                | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_DIO2 | O    | Digital output   | Digital output                | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_DIO3 | O    | Digital output   | Digital output                | GPIO mode input pull-down          |
| LCDC SPI  | LCDCx_SPI_RSTB | O    | Digital output   | Digital output                | GPIO output low                |
| LCDC SPI  | LCDCx_SPI_TE   | I    | Digital input   | Digital input                | GPIO mode input pull-down          |
| SDIO      | SD_CLK         | O    | Digital output   | Digital output                | GPIO mode output low            |
| SDIO      | SD_CMD         | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| SDIO      | SD_DIOx        | I/O  | Digital input pull-up | Digital input pull-up          | Digital input pull-down               |
| I2S       | I2S1_BCK       | O    | Digital output   | Digital output                | GPIO mode output low            |
| I2S       | I2S1_LRCK      | O    | Digital output   | Digital output                | GPIO mode output low            |
| I2S       | I2S1_SDI       | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| I2S       | I2S2_BCK       | O    | Digital output   | Digital output                | GPIO mode output low            |
| I2S       | I2S2_LRCK      | O    | Digital output   | Digital output                | GPIO mode output low            |
| I2S       | I2S2_SDI       | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| I2S       | I2S2_SDO       | O    | Digital output   | Digital output                | GPIO mode output low            |
| PDM       | PDM_CLK        | O    | Digital output   | Digital output                | GPIO mode output low            |
| PDM       | PDM_DATA       | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| GPTIM out | GPTIMx_CHx     | O    | Digital output   | Digital output                | GPIO mode output low            |
| GPTIM in  | GPTIMx_CHx     | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| GPTIM     | GPTIMx_ETR     | I    | Digital input pull-down | Digital input pull-down          | Digital input pull-down               |
| GPIO In   | GPIO           | I    | Digital input   | Digital input                | GPIO output low or digital input pull-down |
| GPIO Out  | GPIO           | O    | Digital output   | Digital output                | GPIO mode output low            |

#### 5.1.2 On-Chip IO Internal Leakage

Common models (see FAQ 8.7/8.8 for details):
1) Input pins floating (a powered-off device on the other end is equivalent to floating) causing an undetermined level;
2) The IO output level not matching the internal/external pull-up/pull-down.

The following figure shows the pin internal structure (indicative functions: DS/OE/O/IE/PE/PS, etc.).

```{figure} ../../assets/low_power23.png
:align: center
Figure 5.1 Pin internal structure diagram
```
```{only} SF32LB55X
Note: The `PA01` pin of the 55 series USB has a default internal 18K pull-down. Outputting high or connecting an external high level causes leakage; refer to the FAQ "Leakage risk of PA01/PA03 multiplexed with USB on 55 series MCUs" for handling.
```
#### 5.1.3 On-Chip and External Memory Chip Leakage

Example of PSRAM entering/exiting Half_sleep:

```c
void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#if defined(BSP_USING_PSRAM1)
        rt_psram_exit_low_power("psram1"); // Exit half_sleep
#endif
    }
    // ...
}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
#if defined(BSP_USING_PSRAM1)
        rt_psram_enter_low_power("psram1");  // Enter half_sleep
#endif
    }
#else
    // ...
#endif
}
```

Example of Flash power-off and Deep Sleep:

```c
HAL_RAM_RET_CODE_SECT(BSP_PowerDownCustom, void BSP_PowerDownCustom(int coreid, bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
#ifdef BSP_USING_NOR_FLASH2
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, true); // Power off nor flash supply

    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1); // Change IO to pull-down after power-off
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1);
#elif defined(BSP_USING_NOR_FLASH1)
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle); // nor flash enters deep sleep; no IO change needed
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH2 */
#else
    { ; }
#endif
}

HAL_RAM_RET_CODE_SECT(BSP_PowerUpCustom, void BSP_PowerUpCustom(bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#ifdef BSP_USING_NOR_FLASH2
        HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1); // Restore IO to active state before powering on
        HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
        HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP,   1);

        HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLUP, 1);

        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, true, true); // Power on nor flash supply
        BSP_Flash_hw2_init(); // nor flash must be reinitialized after power-off
#elif defined(BSP_USING_NOR_FLASH1)
        FLASH_HandleTypeDef *flash_handle;
        flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
        HAL_FLASH_RELEASE_DPD(flash_handle); // Exit deep sleep
        HAL_Delay_us(20); // See the chip manual for tRES1
#endif
    }
    else if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
    }
#elif defined(SOC_BF0_LCPU)
    { ; }
#endif
}
```

Note: When XIP runs from nor flash, the code that puts nor flash to sleep/wakes it up must be placed in RAM (`HAL_RAM_RET_CODE_SECT`).

### 5.2 Code Implementation
```{only} SF32LB55X
The pin configuration code is located in the board's `pinmux.c` and `drv_io.c`. Implement the `BSP_PIN_Init`, `BSP_Power_Up`, `BSP_IO_Power_Down` and other interfaces based on the IO definitions and hardware.
```
```{only} SF32LB58X or SF32LB56X or SF32LB57X or SF32LB52X
The pin configuration code is located in the board's `pinmux.c` and `bsp_power.c`. Implement the `BSP_PIN_Init`, `BSP_Power_Up`, `BSP_IO_Power_Down` and other interfaces based on the IO definitions and hardware.
```
#### 5.2.1 Pin Configuration in Active State

`BSP_PIN_Init` runs once on both cold boot and STANDBY wakeup, and can set the function/input-output mode of pins in the active state there. For example, configure `PB46` as `USART3_RX`, digital input with pull-up:

```c
HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);
```

For output IOs, configuring only `PIN_NOPULL` without setting the GPIO output leaves the pin in the "default input state" and causes floating leakage. Set the output level, for example:

```c
HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_NOPULL, 1);
// Then configure it as an explicit high/low level output, or call HAL_GPIO_DeInit to restore the input state when needed
```

#### 5.2.2 Pin Configuration in Sleep State
```{only} SF32LB55X
The following weak functions can be implemented in `drv_io.c` to dynamically switch pin settings when entering/exiting sleep:
```
```{only} SF32LB58X or SF32LB56X or SF32LB57X or SF32LB52X
The following weak functions can be implemented in `bsp_power.c` to dynamically switch pin settings when entering/exiting sleep:
```
Table 5‑2: Pin Configuration APIs for the Sleep State

| Function name       | Description                                       |
|--------------------|--------------------------------------------|
| BSP_IO_Power_Down  | Executed before entering sleep                        |
| BSP_Power_Up       | Executed after wakeup (STANDBY wakeup runs after `BSP_PIN_Init`) |
| BSP_TP_PowerDown   | Executed after the screen turns off                    |
| BSP_TP_PowerUp     | Executed before the screen turns on                    |
| BSP_LCD_PowerDown  | Executed after the screen turns off                    |
| BSP_LCD_PowerUp    | Executed before the screen turns on                    |

If the power-off and power-on control of board-level devices both accompany sleep, you can power off the board-level devices and change the corresponding pin settings in BSP_IO_Power_Down, and do the reverse in BSP_Power_Up. However, this approach lacks fine-grained control. For example, after the screen turns off, HPSYS may still take some time before entering sleep; if the LCD remains powered during that period, power consumption increases. Or when HPSYS is woken up to run tasks for a while without needing the screen on, turning on the screen power in BSP_Power_Up also increases power consumption. For this reason, you can implement more complex control logic in BSP_IO_Power_Down and BSP_Power_Up. Taking the screen and touch as an example, put the power-down handling of the screen and touch into BSP_TP_PowerDown and BSP_LCD_PowerDown, so that the screen and touch chips are powered off immediately once the screen turns off; in BSP_Power_Up, call BSP_TP_PowerDown and BSP_LCD_PowerDown again, so that even when HPSYS is woken up, the pin settings are restored to the power-off state. When the screen-on condition is met, the system calls BSP_TP_PowerUp and BSP_LCD_PowerUp before turning on the screen to restore the power supply of the screen and touch and the active-state pin settings.

Call order and coreid description:
- `void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)` is called twice before the HCPU enters sleep:
    - First call with `coreid=CORE_ID_LCPU`: executed before revoking the LCPU wakeup request, used to turn off "the pins related to LCPU peripherals used by the HCPU". After revocation, the LCPU may enter low power, and the HCPU can no longer access the LCPU power domain registers.
    - Second call with `coreid=CORE_ID_HCPU`: executed right before the HCPU actually falls asleep, used to turn off the other pins used by the HCPU itself.
- In the LCPU project, this function is called once before the LCPU falls asleep, used to turn off the pins used by the LCPU.
- Different peripherals have different low-power pin configurations. Generally, pull-ups/pull-downs should be turned off to avoid loop leakage; whether a pin outputs high or low depends on the board design (whether it matches the peripheral power state).

### 5.3 Sleep Flow

```{only} SF32LB56X

• HCPU sleep/wakeup (simplified flow):
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` log `[pm]S:4,11620140` → device `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` enter standby → timer/IO wakeup → `SystemInitFromStandby → HAL_Init → BSP_IO_Init → restore_context` (PC returns to after WFI) → `BSP_Power_Up →` device `RT_DEVICE_CTRL_RESUME` → log `[pm]W:11620520`, `[pm]WSR:0x80`.

• LCPU sleep/wakeup is similar to HCPU, with the differences: `sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` same logs as above.
```
```{only} SF32LB55X
• HCPU sleep/wakeup (simplified flow):
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` log `[pm]S:4,11620140` → device `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` enter standby → timer/IO wakeup → `SystemPowerOnModeInit → HAL_Init → BSP_IO_Init → restore_context` (PC returns to after `sifli_standby_handler`) → `BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` log `[pm]W:11620520`, `[pm]WSR:0x80`.

• LCPU sleep/wakeup is similar to HCPU, with the differences: `sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` same logs as above.
```
```{only} SF32LB52X
It is recommended to use the deepsleep low-power mode (sleep mode). In this mode all RAM data and hardware configurations are retained, and the recovery time from sleep mode back to the active state is also shorter. During sleep, IO levels can remain in the active state. However, peripherals stop working in sleep mode, and the CPU can only be woken up by a limited set of wakeup sources, including GPIO interrupts, RTC interrupts, LPTIM interrupts, and inter-core communication interrupts.

• HCPU sleep/wakeup (simplified flow):
Entering `sifli_deep_handler()` with no peripheral SUSPEND/RESUME or context restore makes wakeup faster:
`sifli_sleep →` log `[pm]S:3,11620140` → `sifli_deep_handler → BSP_IO_Power_Down → WFI` enter deep → timer/IO wakeup → continue after WFI → `BSP_Power_Up →` log `[pm]W:11620520`, `[pm]WSR:0x80`.

Note: The LCPU code of the 52 series is not open for modification.
```
```{only} SF32LB57X
It is recommended to use the deepsleep low-power mode (sleep mode). In this mode all RAM data and hardware configurations are retained, and the recovery time from sleep mode back to the active state is also shorter. During sleep, IO levels can remain in the active state. However, peripherals stop working in sleep mode, and the CPU can only be woken up by a limited set of wakeup sources, including GPIO interrupts, RTC interrupts, LPTIM interrupts, and inter-core communication interrupts.

• HCPU sleep/wakeup (simplified flow):
Entering `sifli_deep_handler()` with no peripheral SUSPEND/RESUME or context restore makes wakeup faster:
`sifli_sleep →` log `[pm]S:3,11620140` → `sifli_deep_handler → BSP_IO_Power_Down → WFI` enter deep → timer/IO wakeup → continue after WFI → `BSP_Power_Up →` log `[pm]W:11620520`, `[pm]WSR:0x80`.
```
### 5.4 Hibernate Power-Off Leakage Analysis

#### 5.4.1 Hibernate Power-Off Flow
```{only} SF32LB55X
Entering Hibernate: call `HAL_PMU_EnterHibernate()`. Before hibernation, configure the PMU wakeup PIN and level for Hibernate.
```
```{only} SF32LB52X or SF32LB57X
Entering Hibernate: call `HAL_PMU_EnterHibernate()`. Before hibernation, configure the PMU wakeup PIN and level for Hibernate. The 52/57 series have 3 built-in LDOs; disable the unused LDOs via `HAL_PMU_ConfigPeriLdo` based on the hardware.
```
```{only} SF32LB56X or SF32LB58X
Entering Hibernate: call `HAL_PMU_EnterHibernate()`. Before hibernation, configure the PMU wakeup PIN and level for Hibernate. Because the 56/58 series add a PMU pull-up/pull-down system in Hibernate, it is recommended to use `HAL_PIN_Set` to configure the wakeup PIN pull-up/pull-down.
```
Hibernate wakeup: The system wakes up after the wakeup PIN is pressed. You can use `PM_HIBERNATE_BOOT == SystemPowerOnModeGet()` to determine whether it is a hibernate boot, and combine it with the key press duration to decide whether to power on.

#### 5.4.2 Hibernate Power-Off Configuration

Before entering Hibernate:
- Call `HAL_PMU_EnterHibernate()`;
- Configure the PMU wakeup PIN and level to ensure the system can be woken up;
```{only} SF32LB55X
- 55 series: In Hibernate, all wakeup PINs are floating inputs; external pull-ups/pull-downs are required to prevent floating leakage;
```
```{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X
- 58/56/52 series: In Hibernate, the PMU side provides pull-ups/pull-downs (`hwp_rtc->PAWK1R/PAWK2R`); it is recommended to configure them with `HAL_PIN_Set`;
- `hwp_pmuc->WKUP_CNT` can configure the external signal duration threshold (58/56/52 series only).
```
```{only} SF32LB52X
- 52 series: There are 3 built-in LDOs (`PMU_PERI_LDO_1V8/PMU_PERI_LDO2_3V3/PMU_PERI_LDO3_3V3`); decide whether to turn them off based on the hardware.

Example:

```c
rt_kprintf("SF32LB52X entry_hibernate\n");
HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); // select PA34 → wake_pin0
HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);                                 // enable wake_pin0 
hwp_pmuc->WKUP_CNT = 0x50005; // 31-16bit: PIN1 wake CNT, 15-0bit: PIN0 wake CNT
rt_kprintf("SF32LB52X CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);

HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1); // #WKUP_PIN0
HAL_PIN_Set(PAD_PA25, GPIO_A25, PIN_PULLDOWN, 1); // #WKUP_PIN1
HAL_PIN_Set(PAD_PA26, GPIO_A26, PIN_PULLDOWN, 1); // #WKUP_PIN2
HAL_PIN_Set(PAD_PA27, GPIO_A27, PIN_PULLDOWN, 1); // #WKUP_PIN3

HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1); // #WKUP_PIN10
HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1); // #WKUP_PIN11
HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1); // #WKUP_PIN12
HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1); // #WKUP_PIN13
HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1); // #WKUP_PIN14
HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1); // #WKUP_PIN15
HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_PULLDOWN, 1); // #WKUP_PIN16
HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1); // #WKUP_PIN17
HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1); // #WKUP_PIN18
HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_PULLDOWN, 1); // #WKUP_PIN19
HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_PULLDOWN, 1); // #WKUP_PIN20

rt_hw_interrupt_disable();
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, false);
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, false, false);
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8,  false, false);
HAL_PMU_EnterHibernate();
```

Notes:
```{only} SF32LB55X or SF32LB57X 
- 55/57 series MCUs: Each wakeup pin can be enabled individually; only `HAL_PMU_EnablePinWakeup` is needed;
```
```{only} SF32LB52X or SF32LB56X or SF32LB58X
- 58/56/52 series: Only 2 wakeup sources `pin0/pin1` are allowed simultaneously; use `HAL_PMU_SelectWakeupPin` to specify the mapping;
```
```{only} SF32LB52X
- On the 52 series, `#WKUP_PIN4-9 (PA28-PA33)` are multiplexed with ADC and their wakeup function has been removed; handle them by disconnecting the external IO and using internal pull-down (no handling is needed in Hibernate and no leakage occurs). Do not directly set the pull-ups of `hwp_rtc->PAWK1R/PAWK2R` to avoid leakage.

```{figure} ../../assets/low_power24.png
:align: center
Figure 5.2 Handling of #WKUP_PIN4-9 in Hibernate on the 52 series
```

Hibernate wakeup determination:

```c
if (PM_HIBERNATE_BOOT == SystemPowerOnModeGet())
{
    // Decide whether to power on based on the key press duration etc.
}
```






