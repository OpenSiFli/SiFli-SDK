if NOT [%1]==[] cd %1
JLink.exe -Device CORTEX-M33 -CommanderScript %~dp0\sf32lb55x.jlink >log.txt
if exist bf0_ap.axf cp bf0_ap.axf hcpu.axf