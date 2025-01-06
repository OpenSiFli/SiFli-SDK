::     WHAT IS THIS BAT DO
:: 1. switch to HCPU
:: 2. halt cpu
:: 3. Release WDT1 protect, Disable it, Enable protect
:: 4. Release IWDT protect, Disable it, Enable protect
:: 5. switch to LCPU SWD
:: 6. halt cpu
:: 7. Release WDT2 protect, Disable it, Enable protect


jlink.exe -device CORTEX-M33 -if SWD -speed 4000 -autoconnect 1 -CommandFile %~dp0\halt_all_cpu_and_disable_all_wdt_56x.jlink
