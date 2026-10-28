# SF32LB57-EPD_N16R16

`sf32lb57-epd_n16r16` 对应 SF32LB57-DevKit-EPD_MB_V1.0 墨水屏开发板，使用 SF32LB57GYD7N6 芯片，配有 16 MB NOR Flash 和 16 MB PSRAM。

## 板载硬件

| 硬件 | 接口与引脚 |
| --- | --- |
| E0470A03 墨水屏 | 4.7 英寸，竖屏分辨率 684 × 1216；LCDC1 8 位 TCON 接口 |
| TPS65185 屏幕电源 | I2C1，地址 0x68；WAKEUP PA42、PWRCOM PA43、PWRUP PA44、PGOOD PA52 |
| FT5446U 触摸 | I2C2，地址 0x38；中断 PA3、复位 PA4、电源 PA5 |
| AW99703 前光 | I2C1，地址 0x36；使能 PA53 |
| Wi-Fi | SDMMC1：CLK PA14、CMD PA15、D0–D3 PA16/PA17/PA12/PA13；电源 PA8、PDN PA9 |
| TF 卡 | SDMMC2：CLK PA47、CMD PA48、D0–D3 PA49/PA50/PA45/PA46；检测 PA51 |
| NS4150B 功放 | 使能 PA54 |
| 数字麦克风 | PDM1：CLK PA55、DATA PA56；电源 PA57 |
| 按键 | PA34 中断，ADC 通道 6 区分三个按键 |
| 电源检测 | USB 接入 PA6、充电状态 PA7、电池电压 ADC 通道 11 |

I2C1 的 SCL/SDA 为 PA20/PA21，I2C2 的 SCL/SDA 为 PA1/PA2。

屏幕信号映射为：SDCLK PA24、SDLE PA25、SDOE PA26、SDSTL PA27、SDAT0–SDAT7 PA28/PA29/PA30/PA31/PA32/PA33/PA37/PA38、GMODE PA39、GDCLK PA40、GDSP PA41。

## 编译

在 SDK 根目录加载构建环境，然后编译 Hello World：

```powershell
. ./export.ps1
cd example/get-started/hello_world/rtt/project
scons --board=sf32lb57-epd_n16r16_hcpu -j8
```

HCPU 配置默认选择 SDK 的 E0470A03 显示驱动和 FT5446U 触摸驱动。ACPU 配置位于 `acpu` 子目录，供 SDK 多核工程的 ACPU 子工程使用。

Wi-Fi 和 TF 卡由具体工程按需启用。启用 SDMMC1 时使用 SDIO 模式，板级初始化会打开 PA8 电源，PA9 由接入的 Wi-Fi 驱动控制。启用 SDMMC2 时使用 SD 卡模式。数字麦克风录音前调用 `BSP_Microphone_PowerOn()`，结束后调用 `BSP_Microphone_PowerOff()`；上电函数需要在线程中调用。

## 屏幕波形

本板配置使用二进制波形表。使用默认 `ptab.json` 时，将 SDK 文件 `customer/peripherals/display/epd_e0470a03/waveform/epd_waveform.bin` 烧录到 **0x12CE8000**，对应的 `wave_table` 分区容量为 256 KiB。

工程覆盖分区表时，波形烧录地址以该工程生成的 `CUSTOM_EPD_WAVE_TABLE_START_ADDR` 为准。
