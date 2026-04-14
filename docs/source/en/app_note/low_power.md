# Low Power Development Guide

## 1 Introduction
```{only} SF32LB52X or SF32LB56X or SF32LB55X
SiFli MCUs are dual‑core Cortex‑M33 STAR SoCs. The high‑performance core (HCPU) runs at 0–240 MHz and belongs to the HPSYS subsystem, suitable for graphics, audio, neural networks, and other compute‑intensive workloads. The low‑power core (LCPU) runs at 0–48 MHz and belongs to the LPSYS subsystem, suitable for Bluetooth, sensor data collection, and processing tasks.
```
```{only} SF32LB58X
SF32LB58X is a tri‑core chip (dual high‑performance cores + one low‑power core). Each high‑performance core runs up to 240 MHz and scores up to 984 CoreMark with an energy efficiency of 8.29 µA/CoreMark, providing graphics and audio compute for rich applications and smooth UI/UX. The low‑power core runs up to 96 MHz, reaches 394 CoreMark with 3.88 µA/CoreMark, serving as a Sensor Hub while also running the Bluetooth protocol stack.
```
Refer to the low‑power example at `example\pm\classical`.

## 2 Configure Low‑Power Mode

### 2.1 Enable Low‑Power Mode
```{only} SF32LB52X
Run `sdk.py menuconfig` in the project directory to open the configuration menu:

1. Enable low‑power support (`Enable Low power support`):
    - Path: Sifli middleware → Enable Low power support
    - Toggle: Enable Low power support
        - Macro: `BSP_USING_PM`
        - Purpose: Enable low‑power features

```{figure} ../../assets/enable_pm.png
:align: center
Figure: Enable Low Power configuration menu
```
```{only} SF32LB52X
2. Select low‑power mode (`Enable Deep Mode`):
    - Path: RTOS → RT‑Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - Select: Enable Deep Mode
        - Macro: `PM_DEEP_ENABLE`
        - Purpose: Use Deep Sleep mode as the low‑power mode

```{figure} ../../assets/deep.png
:align: center
Figure: Deep Sleep configuration menu
```
```{only} SF32LB52X
3. Enable PM debug to print low‑power logs (optional; printing logs consumes time and affects power) (`Enable PM Debug`):
    - Path: Sifli middleware → Enable Low power support → Enable PM Debug
        - Select: Enable PM Debug
            - Macro: `BSP_PM_DEBUG`
            - Purpose: Enable low‑power debug logging

```{figure} ../../assets/low_power11.png
:align: center
Figure: PM debug configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
Run `sdk.py menuconfig` in the project directory to open the configuration menu:

1. Enable low‑power support (`Enable Low power support`):
    - Path: Sifli middleware → Enable Low power support
    - Toggle: Enable Low power support
        - Macro: `BSP_USING_PM`
        - Purpose: Enable low‑power features
        
```{figure} ../../assets/enable_pm.png
:align: center
Figure: Enable Low Power configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
2. Select low‑power mode (`Enable Standby Mode`):
    - Path: RTOS → RT‑Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - Select: Enable Standby Mode
        - Macro: `PM_Standby_ENABLE`
        - Purpose: Use Standby Sleep mode as the low‑power mode

```{figure} ../../assets/stabdby.png
:align: center
Figure: Standby Sleep configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
3. Enable PM debug to print low‑power logs (optional; printing logs consumes time and affects power) (`Enable PM Debug`):
    - Path: Sifli middleware → Enable Low power support → Enable PM Debug
        - Select: Enable PM Debug
            - Macro: `BSP_PM_DEBUG`
            - Purpose: Enable low‑power debug logging

```{figure} ../../assets/low_power11.png
:align: center
Figure: PM debug configuration menu
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
- Note: If LCPU enables STANDBY mode, HCPU must also enable STANDBY mode.
```
4) After configuration, verify `rtconfig.h` includes the following:
```{only} SF32LB52X
```c
#define RT_USING_PM 1           // Enable PM module
#define PM_DEEP_ENABLE 1        // Use DEEP sleep mode
#define BSP_USING_PM 1          // Enable PM module
#define BSP_PM_DEBUG 1          // Print PM[S], PM[W] logs (optional)
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X
```c
#define RT_USING_PM 1           // Enable PM module
#define PM_STANDBY_ENABLE 1     // Use STANDBY sleep mode
#define BSP_USING_PM 1          // Enable PM module
#define BSP_PM_DEBUG 1          // Print PM[S], PM[W] logs (optional)
```

### 2.2 Disable Low‑Power Mode

Run `sdk.py menuconfig` and uncheck the same options used to enable low power.

## 3 Wake‑Up by Button Configuration

Refer to `SDK\example\pm\classical` for the configuration method.

**Standby wake‑up configuration (for standby/deep wake‑up)**

Below is an HAL‑level standby wake‑up API example. You can use `HAL_LPAON_QueryWakeupPin()` and `HAL_HPAON_QueryWakeupPin()` to get the wake‑up pin index. If the IO wake‑up needs event handling, also configure the GPIO interrupt:

```{only} SF32LB55X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 55x PA80 #WKUP_A3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 55x PB48 #WKUP_PIN5
// Check if configuration takes effect; compare with datasheet registers:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

```{only} SF32LB52X
In 52 series DeepSleep mode, no extra wake‑up pin configuration is required. Any pin can wake up, via WSR_GPIO1.
```

```{only} SF32LB56X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 56x PB35 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 56x PA50 #WKUP_PIN5
// Check if configuration takes effect; compare with datasheet registers:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```
```{only} SF32LB58X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 58x PB57 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 58x PB59 #WKUP_PIN5
// Check if configuration takes effect; compare with datasheet registers:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_k_printf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

**Power‑off wake‑up configuration (for hibernate wake‑up)**
```{only} SF32LB55X
55 series MCU: In Hibernate, only LCPU wake‑up PIN0–PIN5 support wake‑up. See PMUC WER register in the 55 series User Manual.
```c
// 55x configuration:
HAL_PMU_EnablePinWakeup(5, AON_PIN_MODE_NEG_EDGE); // 55x PB48 #WKUP_PIN5
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```

```{only} SF32LB52X or SF32LB56X or SF32LB58X
Post‑55 series MCUs: Allow two wake‑up sources PIN0 and PIN1 at the same time; each source can map to any HCPU/LCPU wake pin. See PMUC CR register in the User Manual.
```c
// 58x/56x/52x configuration:
HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); // select PA34 to wake_pin0
HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);                                  // enable wake_pin0 
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```



After Hibernate power‑off, wake‑up is equivalent to a cold boot (but with the `PM_HIBERNATE_BOOT` flag), unlike Standby which resumes previous context. Wake‑up pin and level mode are controlled by PMU registers; you can print WER/CR to verify. If the same IO is used for both standby and hibernate wake‑up, both configurations must apply.

## 4 Low‑Power Debugging Methods

### 4.1 Low‑Power Modes

- `PM_SLEEP_MODE_IDLE`:
CPU enters idle (WFI/WFE). High‑speed clocks (HRC/HXT/DBLR/DLL) remain on; all peripherals can stay enabled and generate interrupts.
```{only} SF32LB52X 
- `PM_SLEEP_MODE_LIGHT`:
CPU enters light sleep (WFI). High‑speed clocks are turned off. CPU‑related peripherals stop but remain powered. The system clock switches to 32K.
Wake‑up sources: LPTIM, RTC, BLE MAC (LCPU Only), Mailbox (other CPU), or pin. Wake‑up latency: 30–100 µs.
Execution resumes at the instruction after WFI.
```
```{only} SF32LB56X or SF32LB55X or SF32LB58X 
- `PM_SLEEP_MODE_LIGHT`:
CPU enters light sleep (WFI). High‑speed clocks are turned off. CPU‑related peripherals stop but remain powered. The system clock switches to 32K.
Wake‑up sources: LPTIM, RTC, BLE MAC (LCPU Only), Mailbox (other CPU), or specific wake‑up pins. Wake‑up latency: 30–100 µs.
Execution resumes at the instruction after WFI.
```
- `PM_SLEEP_MODE_DEEP`:
Same as LIGHT, but system power switches to RET_LDO; wake‑up latency increases to 100 µs–1 ms.
Execution resumes at the instruction after WFI.

- `PM_SLEEP_MODE_STANDBY`:
CPU enters standby. High‑speed clocks are off; CPU‑related peripherals lose power; RAM power is removed except for configured retention parts; pin states are retained; system power switches to RET_LDO.
Wake‑up sources: LPTIM, RTC, BLE MAC (LCPU only), Mailbox (other CPU), or specific wake‑up pins. Wake‑up latency: 1–2 ms.
After wake‑up the system restarts; software checks AON registers to determine standby boot vs. cold boot.

Typical currents for each mode are shown in Table 4‑1. If PSRAM exists, HCPU backs up data that must be retained from powered‑down RAM to PSRAM and restores it after wake‑up; otherwise it backs up to 64 KB Retention RAM. Unless otherwise stated, “enter sleep” refers to entering low‑power modes other than IDLE, and “wake‑up” refers to exiting low‑power modes other than IDLE.

Table 4‑1: Low‑power modes
```{only} SF32LB52X 
| Low‑Power Mode         | CPU State | Peripheral State | SRAM                                                     | Wake‑up Source                 | Wake‑up Latency |
|------------------------|-----------|------------------|----------------------------------------------------------|--------------------------------|-----------------|
| PM_SLEEP_MODE_IDLE     | stop      | run              | Accessible                                               | Any interrupt                 | <1 µs           |
| PM_SLEEP_MODE_DEEP     | stop      | stop             | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, fully retained | RTC, wake pin, IO(PA),<br>LPTIM1, BLE | ~250 µs        |
| PM_SLEEP_MODE_STANDBY  | reset     | reset            | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, 384 KB retained only | RTC, wake pin,<br>LPTIM1, BLE | ~1 ms           |
```
```{only} SF32LB55X 
| Low‑Power Mode         | CPU State | Peripheral State | SRAM                                                     | Wake‑up Source                 | Wake‑up Latency |
|------------------------|-----------|------------------|----------------------------------------------------------|--------------------------------|-----------------|
| PM_SLEEP_MODE_IDLE     | stop      | run              | Accessible                                               | Any interrupt                  | <1 µs           |
| PM_SLEEP_MODE_DEEP     | stop      | stop             | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, fully retained | RTC, wake pin, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2 | ~250 µs |
| PM_SLEEP_MODE_STANDBY  | reset     | reset            | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, 64 KB retained only | RTC, wake pin,<br>LPTIM1, LPSYS, MAILBOX2 | ~1.5 ms |
```
```{only} SF32LB56X 
| Low‑Power Mode         | CPU State | Peripheral State | SRAM                                                     | Wake‑up Source                 | Wake‑up Latency |
|------------------------|-----------|------------------|----------------------------------------------------------|--------------------------------|-----------------|
| PM_SLEEP_MODE_IDLE     | stop      | run              | Accessible                                               | Any interrupt                  | <1 µs           |
| PM_SLEEP_MODE_DEEP     | stop      | stop             | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, fully retained | RTC, wake pin, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2 | ~250 µs |
| PM_SLEEP_MODE_STANDBY  | reset     | reset            | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, 64 KB retained only | RTC, wake pin,<br>LPTIM1, LPSYS, MAILBOX2 | ~1.5 ms |
```
```{only} SF32LB58X 
| Low‑Power Mode         | CPU State | Peripheral State | SRAM                                                     | Wake‑up Source                                   | Wake‑up Latency |
|------------------------|-----------|------------------|----------------------------------------------------------|--------------------------------------------------|-----------------|
| PM_SLEEP_MODE_IDLE     | stop      | run              | Accessible                                               | Any interrupt                                   | <1 µs           |
| PM_SLEEP_MODE_DEEP     | stop      | stop             | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, fully retained | RTC, wake pin, IO(PA), LPTIM1,<br>LPSYS, MAILBOX2 | ~250 µs        |
| PM_SLEEP_MODE_STANDBY  | reset     | reset            | LPSYS: inaccessible, fully retained<br>HPSYS: inaccessible, 64 KB retained only | RTC, wake pin, LPTIM1,<br>LPSYS, MAILBOX2       | ~1.5 ms        |
```
### 4.2 Power‑Off Modes

In addition to the four low‑power modes per subsystem, the chip provides two system‑level power‑off modes:
- Hibernate: All subsystems powered down; system clock switches to 32K crystal; wake‑up by PIN and RTC (RTC wake‑up time accurate). API: `HAL_PMU_EnterHibernate`.
- Shutdown: All subsystems powered down; system clock switches to RC10K; wake‑up by PIN and RTC (RTC wake‑up time not accurate). API: `HAL_PMU_EnterShutdown`.

Table 4‑2: Power‑off modes

| Mode      | CPU State | Peripheral State | SRAM         | IO   | Wake‑up Source | Wake‑up Time |
|-----------|-----------|------------------|--------------|------|----------------|--------------|
| Hibernate | reset     | reset            | Not retained | Hi‑Z | RTC and PIN    | >2 ms        |
| Shutdown  | reset     | reset            | Not retained | Hi‑Z | RTC and PIN    | >2 ms        |

Note: Current values are for reference only; actual numbers vary with enabled peripherals and IO settings. “stop” means halted but resumes without reconfiguration; “reset” means CPU resets (starts from ROM) and peripherals need reinitialization.

If sleep is not possible but further power reduction is needed, see WFI auto down‑clocking and scenario‑based DVFS.
### 4.3 WFI Auto Down‑Clocking
When the `IDLE` thread runs but sleep conditions are not met, reduce HCPU frequency to lower current during `WFI`. Prerequisite: all high‑speed peripherals are idle. High‑speed peripherals include:
- EPIC
- EZIP
- LCDC
- USB
- SD

Notes: Busy detection for EPIC/EZIP is built into the SDK’s LVGL implementation. If using a custom implementation, call `rt_pm_hw_device_start`/`rt_pm_hw_device_stop` when peripherals start/stop to avoid down‑clocking during busy periods. Busy detection for LCDC/USB/SD is integrated in RT‑Thread’s LCD Device driver.
```{only} SF32LB52X 
The WFI frequency after down‑clocking is configured by `HAL_RCC_HCPU_SetDeepWFIDiv`. When audio peripherals are active, it can only drop to 48 MHz; otherwise down to 4 MHz. Also set `HPSYS_RCC_DBGR_FORCE_HP` in `hwp_hpsys_rcc->DBGR` to 1.
```
```{only} SF32LB56X or SF32LB58X  
The WFI frequency after down‑clocking is configured by `HAL_RCC_HCPU_SetDeepWFIDiv`. When audio peripherals are active, it can only drop to 48 MHz; otherwise down to 1 MHz.
```
### 4.4 Scenario‑Based DVFS
In low‑performance scenarios, the high‑performance core can reduce frequency/voltage to cut active power (e.g., after screen off, wrist‑raise algorithm only could run at 48 MHz). Although lower frequency increases execution time, total energy (current × time) may be lower. Measure and pick optimal modes per scenario.

Use `rt_pm_run_enter` to set the current run mode. HCPU supports four modes (app default is `PM_RUN_MODE_HIGH_SPEED`); switching to higher speed takes effect immediately, switching to lower speed is deferred until the IDLE thread:

| Mode                     | System Clock (MHz) |
|--------------------------|--------------------|
| PM_RUN_MODE_HIGH_SPEED   | 240                |
| PM_RUN_MODE_NORMAL_SPEED | 144                |
| PM_RUN_MODE_MEDIUM_SPEED | 48                 |
| PM_RUN_MODE_LOW_SPEED    | 24                 |

SDK also provides `pm_scenario_start`/`pm_scenario_stop` for scenario toggles. Currently supports `UI` and `Audio`:
- Either on → use HIGH_SPEED
- Both off → use MEDIUM_SPEED
### 4.5 Low‑Power Flow

In this solution, HPSYS can enter sleep only after the screen turns off. With screen on, when HCPU is idle, HPSYS can only enter IDLE. LPSYS can enter sleep only after HPSYS sleeps; if HPSYS is awake, LPSYS stays in IDLE even if LCPU is idle (52 series exception: HCPU/LCPU can sleep independently). When HPSYS is asleep, LPSYS can freely enter/exit sleep without waking HPSYS.

#### 4.5.1 Screen‑Off

Lock‑screen timeout is configurable in settings. When no interaction exceeds the timeout, the screen turns off; the IDLE thread checks sleep conditions. If met, HPSYS sleeps, and LPSYS can then sleep as well.

```{figure} ../../assets/low_power_english13.png
:align: center
Figure 4.1 Screen‑off flow
```

#### 4.5.2 HPSYS Wake‑Up
```{only} SF32LB52X 
HPSYS can be woken by LPTIM, RTC, BLE MAC (LCPU only), Mailbox (other CPU), or any pin in DeepSleep. For example, press a button to wake HPSYS.
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
HPSYS wake‑up sources: LPTIM, RTC, BLE MAC (LCPU only), Mailbox (from another CPU), or specific wake‑up pins. For example, enabling wake on a button pin allows a button press to wake HPSYS.
```
For button wake to turn the screen on, see Figure 4.2. After the screen turns on, a new screen‑off cycle begins. A wake‑up flow triggered by a smartphone app “setting” event is shown in Figure 4.3; after handling the request, the IDLE thread can immediately re‑enter sleep.

```{figure} ../../assets/low_power_english14.png
:align: center
Figure 4.2 Button wake and screen‑off cycle
```

```{figure} ../../assets/low_power_english25.png
:align: center
Figure 4.3 Wake‑up flow on phone Setting event
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
#### 4.5.3 LPSYS Wake‑Up

LPSYS can be woken by:
- Button
- HPSYS wake‑up
- Sensor acquisition timer timeout
- BLE periodic timer timeout


#### 4.5.3 Cross Power‑Domain Sharing and Sleep Constraints

HCPU and LCPU belong to different power domains and control peripherals in their own domains. To support resource sharing:
- When LCPU is IDLE/ACTIVE (not sleeping), HCPU can use peripherals in the LCPU domain. Avoid both cores using the same peripheral simultaneously to prevent hardware contention.
- Common case: HCPU uses an LCPU peripheral. As long as HCPU needs it, LCPU must stay awake (no sleep), otherwise access will fail.
- If LCPU enters STANDBY, its peripherals lose power, constraining HCPU’s minimum low‑power mode: HCPU must also take the STANDBY path. On wake‑up, first wake LCPU and re‑initialize related peripherals (including those in LCPU), then HCPU can safely use these resources.
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
Context retention in STANDBY:
- HCPU: Before standby, backs up required data/context to PSRAM; after wake‑up, restores from PSRAM (without PSRAM, relies on HPSYS retention region only).
- LCPU: Keeps all RAM powered; saves CPU registers/context in RAM; restores directly from RAM on wake‑up.
```
```{only} SF32LB52X
Context retention in DEEPSLEEP/STANDBY:
- HCPU: In DEEPSLEEP, all RAM retained; restore directly from RAM after wake‑up.
- LCPU: Keeps all RAM powered; saves CPU registers/context in RAM; restores directly from RAM on wake‑up.
```
### 4.6 Log Interpretation

HCPU and LCPU print logs via console. After enabling low‑power debug in Section 2.1, search the following keywords to analyze flows.

Table 4‑3: Log keywords

| Log            | Meaning                                        |
|----------------|-------------------------------------------------|
| gui_suspend    | Screen off                                      |
| gui_resume     | Screen on                                       |
| [pm]S: mode,gtime | Enter sleep; mode=2 means LIGHT, 4 means STANDBY; gtime unit is 32768 Hz |
| [pm]W: gtime   | Exit sleep; gtime unit is 32768 Hz              |
| [pm]WSR:0xXXX  | Wake‑up reason (decode by register bitfields)   |

`gtime` is synchronized between HCPU and LCPU. Example: sleep at 2136602 and wake at 2142330 → `sleep_time=(2142330-2136602)/32768=175 ms`. If `WSR=0x200`, wake‑up was by LPSYS mailbox interrupt. See the datasheet for bit meanings.

```{figure} ../../assets/low_power15.png
:align: center
Figure 4.4 Low‑power log example
```
```{only} SF32LB55X
* HPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | LPTIM1 wake‑up              |
| [2]  | PIN0 wake‑up                |
| [3]  | PIN1 wake‑up                |
| [4]  | PIN2 wake‑up                |
| [5]  | PIN3 wake‑up                |
| [8]  | LPSYS manual wake‑up HPSYS  |
| [9]  | LPSYS wake HPSYS via Mailbox|

* LPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | LPTIM2 wake‑up              |
| [2]  | LPCOMP1 wake‑up             |
| [3]  | LPCOMP2 wake‑up             |
| [4]  | BLE wake‑up                 |
| [5]  | PIN0 wake‑up                |
| [6]  | PIN1 wake‑up                |
| [7]  | PIN2 wake‑up                |
| [8]  | PIN3 wake‑up                |
| [9]  | PIN4 wake‑up                |
| [10] | PIN5 wake‑up                |
| [11] | HPSYS manual wake‑up LPSYS  |
| [12] | HPSYS wake LPSYS via Mailbox|
```
```{only} SF32LB56X
* HPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | GPIO1 wake‑up               |
| [2]  | LPTIM1 wake‑up              |
| [6]  | LPSYS manual wake‑up HPSYS  |
| [7]  | LPSYS wake HPSYS via Mailbox|
| [8]  | PIN0 wake‑up                |
| [9]  | PIN1 wake‑up                |
| [10] | PIN2 wake‑up                |
| [11] | PIN3 wake‑up                |
| [12] | PIN4 wake‑up                |
| [13] | PIN5 wake‑up                |
| [14] | PIN6 wake‑up                |
| [15] | PIN7 wake‑up                |
| [16] | PIN8 wake‑up                |
| [17] | PIN9 wake‑up                |
| [18] | PIN10 wake‑up               |
| [19] | PIN11 wake‑up               |
| [20] | PIN12 wake‑up               |
| [21] | PIN13 wake‑up               |


* LPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | GPIO2 wake‑up               |
| [2]  | LPTIM2 wake‑up              |
| [3]  | LPCOMP1 wake‑up             |
| [4]  | LPCOMP2 wake‑up             |
| [5]  | BT wake‑up                  |
| [6]  | HPSYS manual wake‑up LPSYS  |
| [7]  | HPSYS wake LPSYS via Mailbox|
| [8]  | PIN0 wake‑up                |
| [9]  | PIN1 wake‑up                |
| [10] | PIN2 wake‑up                |
| [11] | PIN3 wake‑up                |
| [12] | PIN4 wake‑up                |
| [13] | PIN5 wake‑up                |
| [14] | PIN6 wake‑up                |
| [15] | PIN7 wake‑up                |
| [16] | PIN8 wake‑up                |
| [17] | PIN9 wake‑up                |
| [18] | PIN10 wake‑up               |
| [19] | PIN11 wake‑up               |
| [20] | PIN12 wake‑up               |
| [21] | PIN13 wake‑up               |
```
```{only} SF32LB58X
* HPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | GPIO1 wake‑up               |
| [2]  | LPTIM1 wake‑up              |
| [3]  | LPCOMP wake‑up              |
| [6]  | LPSYS manual wake‑up HPSYS  |
| [7]  | LPSYS wake HPSYS via Mailbox|
| [8]  | PIN0 wake‑up                |
| [9]  | PIN1 wake‑up                |
| [10] | PIN2 wake‑up                |
| [11] | PIN3 wake‑up                |
| [12] | PIN4 wake‑up                |
| [13] | PIN5 wake‑up                |
| [14] | PIN6 wake‑up                |
| [15] | PIN7 wake‑up                |
| [16] | PIN8 wake‑up                |
| [17] | PIN9 wake‑up                |
| [18] | PIN10 wake‑up               |
| [19] | PIN11 wake‑up               |
| [20] | PIN12 wake‑up               |
| [21] | PIN13 wake‑up               |
| [22] | PIN14 wake‑up               |
| [23] | PIN15 wake‑up               |
| [24] | PIN16 wake‑up               |
| [25] | PIN17 wake‑up               |


* LPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | GPIO2 wake‑up               |
| [2]  | LPTIM2 wake‑up              |
| [3]  | LPCOMP1 wake‑up             |
| [4]  | LPCOMP2 wake‑up             |
| [5]  | BT wake‑up                  |
| [6]  | HPSYS manual wake‑up LPSYS  |
| [7]  | HPSYS wake LPSYS via Mailbox|
| [8]  | PIN0 wake‑up                |
| [9]  | PIN1 wake‑up                |
| [10] | PIN2 wake‑up                |
| [11] | PIN3 wake‑up                |
| [12] | PIN4 wake‑up                |
| [13] | PIN5 wake‑up                |
| [14] | PIN6 wake‑up                |
| [15] | PIN7 wake‑up                |
| [16] | PIN8 wake‑up                |
| [17] | PIN9 wake‑up                |
| [18] | PIN10 wake‑up               |
| [19] | PIN11 wake‑up               |
| [20] | PIN12 wake‑up               |
| [21] | PIN13 wake‑up               |
| [22] | PIN14 wake‑up               |
| [23] | PIN15 wake‑up               |
| [24] | PIN16 wake‑up               |
| [25] | PIN17 wake‑up               |
```
```{only} SF32LB52X
* HPSYS WSR bits

| Bit  | Description                 |
|------|-----------------------------|
| [0]  | RTC wake‑up                 |
| [1]  | GPIO1 wake‑up               |
| [2]  | LPTIM1 wake‑up              |
| [3]  | PMUC wake‑up                |
| [6]  | LPSYS manual wake‑up HPSYS  |
| [7]  | LPSYS wake HPSYS via Mailbox|
| [8]  | PIN0 wake‑up                |
| [9]  | PIN1 wake‑up                |
| [10] | PIN2 wake‑up                |
| [11] | PIN3 wake‑up                |
| [18] | PIN10 wake‑up               |
| [19] | PIN11 wake‑up               |
| [20] | PIN12 wake‑up               |
| [21] | PIN13 wake‑up               |
| [22] | PIN14 wake‑up               |
| [23] | PIN15 wake‑up               |
| [24] | PIN16 wake‑up               |
| [25] | PIN17 wake‑up               |
| [26] | PIN18 wake‑up               |
| [27] | PIN19 wake‑up               |
| [28] | PIN20 wake‑up               |
```

```{only} SF32LB58X
Table 4‑4: 55 series HPSYS wake‑up PIN mapping

| Wake | PIN |
|------|-----|
| PIN0 | PA77|
| PIN1 | PA78|
| PIN2 | PA79|
| PIN3 | PA80|

Table 4‑5: 55 series LPSYS wake‑up PIN mapping

| Wake | PIN |
|------|-----|
| PIN0 | PB43|
| PIN1 | PB44|
| PIN2 | PB45|
| PIN3 | PB46|
| PIN4 | PB47|
| PIN5 | PB48|
```
```{only} SF32LB56X
Table 4‑4: 56 series HPSYS wake‑up PIN mapping (partial)

| Wake | PIN |
|------|-----|
| PIN0 | PB32|
| PIN1 | PB33|
| PIN2 | PB34|
| PIN3 | PB35|

Table 4‑5: 56 series LPSYS wake‑up PIN mapping (partial)

| Wake | PIN |
|------|-----|
| PIN0 | PB54|
| PIN1 | PB55|
| PIN2 | PB56|
| PIN3 | PB57|
| PIN4 | PB58|
| PIN5 | PB59|
```
```{only} SF32LB58X
Table 4‑4: 58 series HPSYS wake‑up PIN mapping (partial)

| Wake | PIN |
|------|-----|
| PIN0 | PB54|
| PIN1 | PB55|
| PIN2 | PB56|
| PIN3 | PB57|

Table 4‑5: 58 series LPSYS wake‑up PIN mapping (partial)

| Wake | PIN |
|------|-----|
| PIN0 | PB32|
| PIN1 | PB33|
| PIN2 | PB34|
| PIN3 | PB35|
| PIN4 | PB36|
| PIN5 | PA50|
```

```{only} SF32LB52X
Table 4‑4: 52 series HPSYS wake‑up PIN mapping (partial)

| Wake | PIN |
|------|-----|
| PIN0 | PA24|
| PIN1 | PA25|
| PIN10| PA34|
| PIN11| PA35|
| PIN19| PA43|
```

### 4.7 Common Issues Analysis

Since SWD cannot connect in sleep, use UART console logs to analyze issues.

#### 4.7.1 Whether Sleep Was Entered

Likely HPSYS slept if any of:
- SWD cannot connect
- HCPU console no response
- HCPU logs show “S: mode, gtime”

Likely LPSYS slept if any of:
- LCPU console no response
- LCPU logs show “S: mode, gtime”

Ensure the finsh shell option is enabled in LCPU Command shell.

You can also check chip power pins to determine current low‑power mode:
- HPSYS active/sleep/deepsleep: `LDO1_VOUT` ≈ 1.1 V; HPSYS standby: `LDO1_VOUT` gradually drops to 0 V.
- LPSYS active/sleep/deepsleep: `LDO2_VOUT` or `BUCK2_VOUT` ≈ 0.9 V; LPSYS standby: the voltage gradually drops to 0 V.
- Hibernate: `LDO1_VOUT/LDO2_VOUT/BUCK2_VOUT/VDD_RET` all drop to 0 V.

```{only} SF32LB55X
55 series power pins in low‑power modes, see Figure 4.5 (from SF32LB55x User Manual 4.2.9).

```{figure} ../../assets/low_power16.png
:align: center
Figure 4.5 Power pin voltages in low‑power modes
```

#### 4.7.2 Why Sleep Didn’t Happen

**Sleep entry conditions**

For applications, sleep/work switching is transparently controlled by the lowest priority `IDLE` thread: when all higher‑priority threads are idle, `IDLE` runs and checks if all the following are met before entering sleep:

- Sleep not forbidden (no outstanding `rt_pm_request(PM_SLEEP_MODE_IDLE)`)
- The soonest OS timer expiration > threshold (default 100 ms)
- No wake condition currently satisfied (e.g., enabled wake source not active)
- Data sent to the other core has been consumed (no unread data in IPC queues)

**Timed wake configuration before sleep**

Before sleep, configure `LPTIM` to interrupt at the soonest OS timer expiration so the timer callback fires on time during sleep. Example: if the next timer expires in 200 ms, set `LPTIM` to interrupt in 200 ms.

**Forbid/release sleep APIs**

Apps can explicitly suppress sleep in critical sections; the driver framework also suppresses sleep while peripherals are active to avoid sleeping during interrupt operations:

```c
// Forbid sleep until release
rt_pm_request(PM_SLEEP_MODE_IDLE);
// ... critical section/peripheral operations ...
rt_pm_release(PM_SLEEP_MODE_IDLE);
```

**Timer thresholds and policy**

Refer to Power Management configuration for enabling low‑power support and selecting policies to enter different low‑power modes. The current default policy is:

Default policy example:

```c
static const pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},                  // Idle > 15 ms → Light sleep
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},             // Idle > 10 s → Standby
#endif /* PM_STANDBY_ENABLE */
};
```

Common checks (HCPU/LCPU similar):

1) Enable PM and verify macros:

```{only} SF32LB52X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // Enable low‑power
#define PM_DEEP_ENABLE 1        // Use Deep low‑power
#define BSP_PM_DEBUG 1          // Enable low‑power debug logs
```
```{only} SF32LB56X or SF32LB58X or SF32LB55X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // Enable low‑power
#define PM_STANDBY_ENABLE 1     // Use Standby low‑power
#define BSP_PM_DEBUG 1          // Enable low‑power debug logs
```
2) Confirm CPU is idle and in the idle thread:
- Use finsh `list_thread` to check threads; except `tshell` and `tidle` ready, others should be suspend, otherwise IDLE cannot run.

```{figure} ../../assets/low_power_english17.png
:align: center
Figure 4.6 Output of list_thread
```

3) Confirm sleep is not forbidden:
- In console, run `pm_dump`. If “Idle Mode Counter” > 0, some module called `rt_pm_request(PM_SLEEP_MODE_IDLE)` to forbid sleep; call `rt_pm_release(PM_SLEEP_MODE_IDLE)` to release.

```{figure} ../../assets/low_power18.png
:align: center
Figure 4.7 Output of pm_dump
```

4) Confirm OS timer expirations exceed the sleep threshold:
In console, run `list_timer` to show created timers. Compare the timeout of timers with flag `activated` against the sleep threshold; if smaller, that timer prevents sleep. Timeout unit is ms.
```{figure} ../../assets/low_power19.png
:align: center
Figure 4.8 Output of list_timer
```
Default thresholds: HPSYS sleep threshold 100 ms; LPSYS sleep threshold 10 ms.
```c
RT_WEAK const pm_policy_t pm_policy[] =
{
#ifdef PM_STANDBY_ENABLE
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_STANDBY}, // HCPU: no timer wake within 100 ms → Standby
#else
    {10, PM_SLEEP_MODE_STANDBY},  // LCPU: no timer wake within 10 ms → Standby
#endif /* SOC_BF0_HCPU */
#elif defined(PM_DEEP_ENABLE)
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_DEEP},    // HCPU: no timer wake within 100 ms → Deep
#else
    {10, PM_SLEEP_MODE_DEEP},     // LCPU: no timer wake within 10 ms → Deep
#endif /* SOC_BF0_HCPU */
#else
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_LIGHT},
#else
}
```
If HCPU code uses a 90 ms periodic delay, the system will never enter sleep:

```c
while (1)
{
    rt_thread_delay(90); // 90ms delay
}
```

Differences between delay functions:
- HAL layer (no thread switch during delay):

```c
HAL_Delay(10);     // 10ms
HAL_Delay_us(10);  // 10us
```

- RT‑Thread interface (switches to other threads; may trigger idle→sleep):

```c
rt_thread_delay(100); // 100ms
```

5) Ensure no pending wake‑up sources:
- Read registers via console commands to check `WER/WSR`:
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

- You can also use Jlink/SifliUsartServer to read registers or print via logs:

```c
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

Common issue: wake‑up pin level misconfigured (e.g., configured for low‑level wake‑up but the pin remains low).

6) Ensure data sent to the other core has been consumed:
- Use Ozone, memory dump with trace32, or log `ipc_ctx` queue `read_idx_mirror/write_idx_mirror`; inequality indicates unread data that blocks sleep.

```{figure} ../../assets/low_power_english20.png
:align: center
Figure 4.9 Non‑empty ring buffer
```

```{figure} ../../assets/low_power_english21.png
:align: center
Figure 4.10 Empty ring buffer
```

Print activation queue index:

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

```{figure} ../../assets/low_power_english22.png
:align: center
Figure 4.11 Example: LCPU data service disabled causing missing channel
```


## 5 Power Optimization Methods

### 5.1 Standby Leakage Analysis

When both HPSYS and LPSYS are in sleep, focus on system power optimization:
- Remove detachable components (display, sensors, charging IC) and measure the minimum system current.
- Misconfigured software IO levels can cause voltage differences/floating leakage.
- On‑chip PSRAM/Flash and external NAND/Flash/eMMC may not have entered sleep.

If hardware allows measuring current per rail, identify the leaking rail (`VSYS/ VLDO2/ VLDO3/ VDD_SIP/ VDDIOA`) to narrow the scope.

#### 5.1.1 Peripheral Leakage

Common causes:
1) Board‑level components are not powered off.
2) Board‑level components are powered off, but chip pin configuration causes back‑powering from the chip pins.

For case 2), avoid: driving high or enabling pull‑up on pins connected to powered‑off devices. Recommended pin settings for common peripherals in active/sleep are below (no change needed if external circuits stay powered; switch to low‑power configuration if external circuits are powered off).

Table 5‑1: Recommended Pin Settings

| Peripheral| Pin            | Dir  | Active     | Sleep (external on)     | Sleep (external off)       |
|-----------|----------------|------|------------|-------------------------|----------------------------|
| PSRAM     | PSRAM_CLK      | O    | Digital out| Digital out             | GPIO mode output low       |
| PSRAM     | PSRAM_CLKB     | O    | Digital out| Digital out             | GPIO mode output low       |
| PSRAM     | PSRAM_CS       | O    | Digital out| Digital out             | GPIO mode output low       |
| PSRAM     | PSRAM_DM0      | O    | Digital out| Digital out             | GPIO mode output low       |
| PSRAM     | PSRAM_DM1      | O    | Digital out| Digital out             | GPIO mode output low       |
| PSRAM     | PSRAM_DQS0     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| PSRAM     | PSRAM_DQS1     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| PSRAM     | PSRAM_DQx      | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| QSPI      | QSPIx_CLK      | O    | Digital out| Digital out             | GPIO mode output low       |
| QSPI      | QSPIx_CS       | O    | Digital out| Digital out             | GPIO mode output low       |
| QSPI      | QSPIx_DIO0     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| QSPI      | QSPIx_DIO1     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| QSPI      | QSPIx_DIO2     | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| QSPI      | QSPIx_DIO3     | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| QSPI      | QSPIx_DIO4     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| QSPI      | QSPIx_DIO5     | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| QSPI      | QSPIx_DIO6     | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| QSPI      | QSPIx_DIO7     | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| USART     | USARTx_RXD     | I    | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| USART     | USARTx_TXD     | O    | Digital out| Digital out             | Digital out                |
| USART     | USARTx_CTS     | I    | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| USART     | USARTx_RTS     | O    | Digital out| Digital out             | Digital out                |
| I2C       | I2Cx_SCL       | I/O  | Digital in | Digital in              | Digital in‑pd              |
| I2C       | I2Cx_SDA       | I/O  | Digital in | Digital in              | Digital in‑pd              |
| SPI M     | SPIx_CLK       | O    | Digital out| Digital out             | GPIO mode output low       |
| SPI M     | SPIx_CS        | O    | Digital out| Digital out             | GPIO mode output low       |
| SPI M     | SPIx_DI        | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| SPI M     | SPIx_DO        | O    | Digital out| Digital out             | GPIO mode output low       |
| SPI M     | SPIx_DIO       | I/O  | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| LCDC SPI  | LCDCx_SPI_CS   | O    | Digital out| Digital out             | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_CLK  | O    | Digital out| Digital out             | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_DIO0 | I/O  | Digital in‑pd | Digital in‑pd       | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_DIO1 | O    | Digital out| Digital out             | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_DIO2 | O    | Digital out| Digital out             | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_DIO3 | O    | Digital out| Digital out             | GPIO mode input pull‑down  |
| LCDC SPI  | LCDCx_SPI_RSTB | O    | Digital out| Digital out             | GPIO output low            |
| LCDC SPI  | LCDCx_SPI_TE   | I    | Digital in | Digital in              | GPIO mode input pull‑down  |
| SDIO      | SD_CLK         | O    | Digital out| Digital out             | GPIO mode output low       |
| SDIO      | SD_CMD         | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| SDIO      | SD_DIOx        | I/O  | Digital in‑pu | Digital in‑pu       | Digital in‑pd              |
| I2S       | I2S1_BCK       | O    | Digital out| Digital out             | GPIO mode output low       |
| I2S       | I2S1_LRCK      | O    | Digital out| Digital out             | GPIO mode output low       |
| I2S       | I2S1_SDI       | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| I2S       | I2S2_BCK       | O    | Digital out| Digital out             | GPIO mode output low       |
| I2S       | I2S2_LRCK      | O    | Digital out| Digital out             | GPIO mode output low       |
| I2S       | I2S2_SDI       | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| I2S       | I2S2_SDO       | O    | Digital out| Digital out             | GPIO mode output low       |
| PDM       | PDM_CLK        | O    | Digital out| Digital out             | GPIO mode output low       |
| PDM       | PDM_DATA       | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| GPTIM Out | GPTIMx_CHx     | O    | Digital out| Digital out             | GPIO mode output low       |
| GPTIM In  | GPTIMx_CHx     | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| GPTIM     | GPTIMx_ETR     | I    | Digital in‑pd | Digital in‑pd       | Digital in‑pd              |
| GPIO In   | GPIO           | I    | Digital in | Digital in              | GPIO output low or digital in‑pd |
| GPIO Out  | GPIO           | O    | Digital out| Digital out             | GPIO mode output low       |

#### 5.1.2 On‑Chip IO Internal Leakage

Common patterns (see FAQ 8.7/8.8):
1) Input pins floating (peer device powered off equals floating), causing undefined levels.
2) IO output levels mismatch with internal/external pull‑ups or pull‑downs.

Below shows the pin internal structure (functional blocks: DS/OE/O/IE/PE/PS, etc.).

```{figure} ../../assets/low_power23.png
:align: center
Figure 5.1 Pin internal structure
```
```{only} SF32LB55X
Note: In 55 series, USB `PA01` has an internal 18K pull‑down by default; driving high or externally pulling high may cause leakage. See FAQ “Leakage risk on 55 series MCU when multiplexing USB PA01/PA03”.
```
#### 5.1.3 On‑chip/External Memory Leakage

PSRAM half‑sleep entry/exit example:

```c
void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#if defined(BSP_USING_PSRAM1)
        rt_psram_exit_low_power("psram1"); // exit half_sleep
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
        rt_psram_enter_low_power("psram1");  // enter half_sleep
#endif
    }
#else
    // ...
#endif
}
```

NOR Flash power‑off and Deep Sleep example:

```c
HAL_RAM_RET_CODE_SECT(BSP_PowerDownCustom, void BSP_PowerDownCustom(int coreid, bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
#ifdef BSP_USING_NOR_FLASH2
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, true); // turn off NOR flash power

    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1); // after power off, set IO to pull‑down
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
    HAL_FLASH_DEEP_PWRDOWN(flash_handle); // NOR flash enters deep sleep; IO state unchanged
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
        HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1); // restore IO to working state before power on
        HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
        HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP,   1);

        HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLUP, 1);

        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, true, true); // turn on NOR flash power
        BSP_Flash_hw2_init(); // reinitialize NOR flash after power cycling
#elif defined(BSP_USING_NOR_FLASH1)
        FLASH_HandleTypeDef *flash_handle;
        flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
        HAL_FLASH_RELEASE_DPD(flash_handle); // exit deep sleep
        HAL_Delay_us(20); // delay per datasheet tRES1
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

Note: When running XIP from NOR flash, code controlling NOR sleep/wake must be placed in RAM (`HAL_RAM_RET_CODE_SECT`).

### 5.2 Code Implementation
```{only} SF32LB55X
Pin configuration code resides in the board’s `pinmux.c` and `drv_io.c`. Implement `BSP_PIN_Init`, `BSP_Power_Up`, `BSP_IO_Power_Down` per IO definitions and hardware.
```
```{only} SF32LB58X or SF32LB56X or SF32LB52X
Pin configuration code resides in the board’s `pinmux.c` and `bsp_power.c`. Implement `BSP_PIN_Init`, `BSP_Power_Up`, `BSP_IO_Power_Down` per IO definitions and hardware.
```
#### 5.2.1 Pin Configuration in Active State

`BSP_PIN_Init` runs once on cold boot and after STANDBY wake‑up. Set functional modes and IO directions for active state here. For example, configure `PB46` as `USART3_RX` with digital input pull‑up:

```c
HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);
```

For output IOs, configuring only `PIN_NOPULL` without setting a GPIO output leaves the pin in default input state, potentially causing floating leakage. Set an explicit output level, e.g.:

```c
HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_NOPULL, 1);
// Then configure as explicit high/low output, or HAL_GPIO_DeInit to restore input when needed
```

#### 5.2.2 Pin Configuration in Sleep State
```{only} SF32LB55X
In `drv_io.c`, implement the following virtual functions to dynamically switch pin settings when entering/exiting sleep:
```
```{only} SF32LB58X or SF32LB56X or SF32LB52X
In `bsp_power.c`, implement the following virtual functions to dynamically switch pin settings when entering/exiting sleep:
```
Table 5‑2: Pin Configuration APIs for Sleep State

| Function           | Description                                 |
|--------------------|--------------------------------------------|
| BSP_IO_Power_Down  | Executed before entering sleep              |
| BSP_Power_Up       | Executed after wake‑up (STANDBY after `BSP_PIN_Init`) |
| BSP_TP_PowerDown   | Executed after screen off                   |
| BSP_TP_PowerUp     | Executed before screen on                   |
| BSP_LCD_PowerDown  | Executed after screen off                   |
| BSP_LCD_PowerUp    | Executed before screen on                   |

If board‑level power control should align with sleep, you can power off components and adjust pins in `BSP_IO_Power_Down`, then reverse in `BSP_Power_Up`. This is coarse‑grained: e.g., after screen‑off, HPSYS may enter sleep later, and keeping LCD powered until then wastes energy; or when HPSYS wakes to run tasks without needing the screen, powering the screen in `BSP_Power_Up` also wastes energy. To refine control, handle display/touch power‑down in `BSP_TP_PowerDown` and `BSP_LCD_PowerDown` right after screen‑off, and call them again in `BSP_Power_Up` to keep pins in power‑down state until screen‑on. If screen‑on conditions are met, the system calls `BSP_TP_PowerUp`/`BSP_LCD_PowerUp` before lighting the screen to restore power and active pin settings.

Call order and coreid notes:
- `void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)` is called twice before HCPU sleeps:
    - First with `coreid=CORE_ID_LCPU`: before revoking LCPU wake requests, to close pins of LCPU peripherals used by HCPU. After revocation LCPU may enter low‑power and HCPU cannot access LCPU domain registers.
    - Second with `coreid=CORE_ID_HCPU`: right before HCPU sleep, to close pins used by HCPU itself.
- In the LCPU project, this function is called once before LCPU sleeps to close pins used by LCPU.
- Low‑power pin configurations differ per peripheral. Generally disable pull‑ups/downs to avoid leakage loops; output levels depend on board design and external device power state.

### 5.3 Sleep Flow

```{only} SF32LB56X

• HCPU sleep/wake (simplified):
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` 日志 `[pm]S:4,11620140` → 设备 `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` 进入 standby → 定时器/IO 唤醒 → `SystemInitFromStandby → HAL_Init → BSP_IO_Init → restore_context`（PC 回到 WFI 后）→ `BSP_Power_Up →` 设备 `RT_DEVICE_CTRL_RESUME` → 日志 `[pm]W:11620520`、`[pm]WSR:0x80`。

• LCPU sleep/wake is similar to HCPU. Differences: `sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` logs as above.
```
```{only} SF32LB55X
• HCPU sleep/wake (simplified):
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` 日志 `[pm]S:4,11620140` → 设备 `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` 进入 standby → 定时器/IO 唤醒 → `SystemPowerOnModeInit → HAL_Init → BSP_IO_Init → restore_context`（PC 回到 `sifli_standby_handler` 之后）→ `BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` 日志 `[pm]W:11620520`、`[pm]WSR:0x80`。

• LCPU sleep/wake is similar to HCPU. Differences: `sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` logs as above.
```
```{only} SF32LB52X
It is recommended to use DEEPSLEEP (sleep mode). In this mode all RAM data and hardware configurations are retained, wake‑up time back to active is shorter, and IO levels remain as in active state. Peripherals stop in sleep; CPU can only be woken by limited sources: GPIO interrupt, RTC interrupt, LPTIM interrupt, and inter‑core communication interrupt.

• HCPU sleep/wake (simplified):
Enter `sifli_deep_handler()`; without peripheral SUSPEND/RESUME and context restore, wake‑up is faster:
`sifli_sleep →` log `[pm]S:3,11620140` → `sifli_deep_handler → BSP_IO_Power_Down → WFI` enter deep → timer/IO wake‑up → continue after returning from WFI → `BSP_Power_Up →` logs `[pm]W:11620520`, `[pm]WSR:0x80`.

Note: LCPU code is not open for modification on 52 series.
```
### 5.4 Hibernate Power‑Off Leakage Analysis

#### 5.4.1 Hibernate Power‑Off Flow
```{only} SF32LB55X
Enter Hibernate: call `HAL_PMU_EnterHibernate()`. Before sleep, configure PMU wake‑up pins and levels for Hibernate.
```
```{only} SF32LB52X
Enter Hibernate: call `HAL_PMU_EnterHibernate()`. Before sleep, configure PMU wake‑up pins and levels for Hibernate. For 52 series, three internal LDOs exist; consider turning off unused LDOs via `HAL_PMU_ConfigPeriLdo` per hardware.
```
```{only} SF32LB56X or SF32LB58X
Enter Hibernate: call `HAL_PMU_EnterHibernate()`. Before sleep, configure PMU wake‑up pins and levels for Hibernate. For 56/58 series, PMU pull‑up/down is available in Hibernate; use `HAL_PIN_Set` to set wake‑up pin pulls.
```
Hibernate wake‑up: press the wake‑up pin to wake. Use `PM_HIBERNATE_BOOT == SystemPowerOnModeGet()` to determine hibernate boot, and combine with button duration to decide whether to power on.

#### 5.4.2 Hibernate Power‑Off Configuration

Before entering Hibernate:
- Call `HAL_PMU_EnterHibernate()`.
- Configure PMU wake‑up pins and levels to ensure wake‑up.
```{only} SF32LB55X
- 55 series: Wake‑up pins are floating inputs in Hibernate; external pull‑ups/downs are required to avoid leakage.
```
```{only} SF32LB52X or SF32LB56X or SF32LB58X
- 58/56/52 series: PMU provides pull‑up/down in Hibernate (`hwp_rtc->PAWK1R/PAWK2R`); configure via `HAL_PIN_Set`.
- `hwp_pmuc->WKUP_CNT` sets external signal duration thresholds (58/56/52 series only).
```
```{only} SF32LB52X
– 52 series: Three internal LDOs (`PMU_PERI_LDO_1V8/PMU_PERI_LDO2_3V3/PMU_PERI_LDO3_3V3`); consider turning them off per hardware.

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
```{only} SF32LB55X
- 55 series MCU: Each wake‑up pin can be enabled individually with `HAL_PMU_EnablePinWakeup`.
```
```{only} SF32LB52X or SF32LB56X or SF32LB58X
- 58/56/52 series: Only two wake‑up sources (`pin0/pin1`) are allowed at the same time; map via `HAL_PMU_SelectWakeupPin`.
```
```{only} SF32LB52X
– 52 series: `#WKUP_PIN4-9 (PA28-PA33)` are multiplexed with ADC and no longer support wake‑up; disconnect external IO and use internal pull‑down (no handling needed in Hibernate, no leakage). Do not set pull‑ups via `hwp_rtc->PAWK1R/PAWK2R` to avoid leakage.

```{figure} ../../assets/low_power24.png
:align: center
Figure 5.2 Handling of #WKUP_PIN4‑9 in 52 series Hibernate
```

Hibernate wake‑up detection:

```c
if (PM_HIBERNATE_BOOT == SystemPowerOnModeGet())
{
    // Decide whether to power on based on button duration, etc.
}
```






