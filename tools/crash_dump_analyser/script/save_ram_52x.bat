if NOT [%1]==[] cd %1
JLink.exe -ip 127.0.0.1:19025 -Device CORTEX-M33 -CommanderScript %~dp0\sf32lb52x.jlink >log.txt
if exist bf0_ap.axf cp bf0_ap.axf hcpu.axf
