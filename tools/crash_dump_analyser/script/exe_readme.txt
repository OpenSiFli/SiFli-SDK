1) Context2Mem.exe 保存在FLASH中的死机数据，用手机APP导出后，用该工具转换为分析使用的bin文件.
eg: Context2Mem.exe d:\20230402234930_ble_assert_log1431734617.txt

2) SetCpuReg.exe 使用导死机保存的log.txt文件中寄存器信息，配置CPU寄存器，由trace32界面的 HR/LR 按钮调用。

