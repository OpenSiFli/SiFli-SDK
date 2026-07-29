# EPD OPM060DA 波形表切换说明

[English](README_EN.md) | [中文](README.md)

## 概述

OPM060DA 墨水屏驱动支持两种波形表工作模式：

| 模式 | Kconfig | 存储位置 | 波形来源 |
|---|---|---|---|
| 明码波形（源码） | `EPD_WAVEFORM_USE_BIN=n`（默认） | 编译进固件 | `epd_waveform.c` 中的 C 数组 |
| 打库波形（二进制） | `EPD_WAVEFORM_USE_BIN=y` | NOR Flash `wave_table` 分区 | 原厂提供的 `epd_waveform.bin` |

两种模式在存储方式、LUT 数据格式、刷新行为（帧数、温区适配）上均有差异。

---

## 1. 明码波形（默认）

### 1.1 存储方式

波形数据以 C 源码数组形式存放于 `epd_waveform.c`：

```c
// 全刷波形：32 帧，单一温区 (0-100C)
static const uint8_t yzc085_wave_full_0_100[32][256] = { ... };

// 局刷波形：12 帧，单一温区 (0-100C)
static const uint8_t yzc085_wave_partial_0_100[12][256] = { ... };
```

每帧 256 字节，表示一个 16×16 的查找表（高 4 位旧像素值，低 4 位新像素值 → 2-bit 波形输出）。

### 1.2 LUT 数据格式

波形值直接放在 32-bit LUT 条目的 **bit[1:0]** 位置，与 EPD_8BIT + F2_SWAP 硬件读取位置一致：

```c
// 明码波形：波形值直接写入 bit[1:0]
for (uint16_t i = 0; i < 256; i++)
    p_argb8888_lut[i] = 0xFF000000 | p_frame_wave[i];
```

### 1.3 刷新模式

| 模式 | 枚举值 | 帧数 | 行为 |
|---|---|---|---|
| AUTO | `EPD_DRAW_MODE_AUTO = 1` | 全刷 32 / 局刷 12 | 根据刷新计数自动选择全刷或局刷（每 10 次局刷后执行 1 次全刷） |
| FULL | `EPD_DRAW_MODE_FULL = 2` | 32 | 始终全刷 |
| PARTIAL | `EPD_DRAW_MODE_PARTIAL = 3` | 12 | 始终局刷 |

### 1.4 全刷与局刷效果分析

**全刷（32 帧）：**

32 帧波形序列逐帧驱动墨水粒子从当前状态过渡到目标状态。每帧施加略微不同的电压模式，逐步构建最终图像：

- 第 0-7 帧：预调节阶段（清除残影）—— 主要输出值 2（白驱动）
- 第 8-15 帧：写入阶段（驱动到目标灰度）—— 输出值 1 和 2 混合（黑/白驱动）
- 第 16-23 帧：后调节阶段（稳定图像）—— 主要输出值 1（黑驱动）
- 第 24-31 帧：最终稳定阶段 —— 均衡输出

这种多阶段方式确保了残影的完全清除和整屏对比度的一致性。全刷总耗时约为 32 × 单帧周期。

**局刷（12 帧）：**

12 帧波形仅驱动新旧图像之间有变化的像素，不变区域保持不动。局刷速度快（12 帧 vs 32 帧），但在细微灰度过渡区域可能留下轻微残影。

**明码模式的局限性：** 仅支持单一温区 (0-100C)。在极端温度下，波形有效性可能下降，导致对比度减弱或残影加重。

---

## 2. 打库波形（`EPD_WAVEFORM_USE_BIN=y`）

### 2.1 存储方式

打库波形文件 `epd_waveform.bin` 由原厂提供，存放在 `waveform/` 目录。编译时通过构建脚本自动打包到固件镜像中：

```python
# SConstruct
if GetDepend('EPD_WAVEFORM_USE_BIN'):
    AddCustomImg("wave_table", bin=[epd_wave_bin])
```

运行时，预编译库 `libepd_waveform_bin_reader_gcc.a` 通过 RT-Thread 的 flash 读取接口从 NOR Flash 的 `wave_table` 分区读取波形数据：

```c
// epd_waveform_bin_reader_port.c
int waveform_bin_reader_read_data(uint32_t offset, uint8_t *buf, uint32_t size) {
    return rt_flash_read(CUSTOM_EPD_WAVE_TABLE_START_ADDR + offset, buf, (int)size);
}
```

分区地址和大小由 `ptab.json` 自动生成：
- `CUSTOM_EPD_WAVE_TABLE_START_ADDR` — 例如 `0x14CE8000`
- `CUSTOM_EPD_WAVE_TABLE_SIZE` — 例如 `0x00020000` (128 KB)

### 2.2 LUT 数据格式

打库波形库输出的波形值位于 **bit[4:3]**（EPIC 数据格式约定）。由于 EPD_8BIT + F2_SWAP 硬件从 **bit[1:0]** 读取，需要进行右移转换：

```c
// 打库波形：库填充 bit[4:3]，右移 3 位到 bit[1:0] 供硬件读取
waveform_bin_reader_fill_lut(p_argb8888_lut, frame_num);
for (uint16_t i = 0; i < 256; i++)
    p_argb8888_lut[i] = (p_argb8888_lut[i] >> 3) | 0xFF000000;
```

### 2.3 多温区支持

与明码波形（仅 0-100C 单一温区）不同，打库波形支持**多个温度分段**。帧数和波形数据根据当前温度动态选择：

```c
uint32_t epd_wave_table_get_frames(int temperature, EpdDrawMode mode) {
    // 打库波形：根据温度和模式动态获取帧数
    return waveform_bin_reader_get_frames(temperature, mode);
}
```

这确保了在全工作温度范围内都能获得最佳刷新效果。

### 2.4 打库波形全刷与局刷效果优势

相比明码波形，打库波形在全刷和局刷方面具有以下显著优势：

**1. 温区自适应驱动**

原厂提供的打库波形包含多个温度区段各自的波形序列。不同于明码波形只有一个 0-100C 的通用波形，打库波形针对不同温度区间（如 0-5C、5-10C、10-15C...50-100C）分别设计了专门的驱动序列。温度越低，墨水粒子活性越差，需要更长的驱动时间和更强的电压序列；温度越高，粒子响应越快，可以适当缩短驱动时间。温区自适应确保了从低温到高温都能保持一致的对比度和残影控制效果。

**2. 动态帧数优化**

打库波形的帧数不再固定为 32（全刷）或 12（局刷），而是根据温度和模式动态变化。在适宜温度下，帧数可能更少，刷新更快；在极端温度下，帧数可能增加以保障刷新质量。这种灵活性兼顾了速度和效果。

**3. 原厂专业调校**

打库波形由面板原厂针对具体面板型号进行专业设计和验证。原厂拥有面板的完整物理模型和测试数据，能够针对该面板的墨水特性（粒子尺寸、粘度、介电常数等）精确计算每个温度点、每个灰度转换所需的最佳电压波形序列。这是通用明码波形无法达到的精度。

**4. 低温残影显著抑制**

这是打库波形最明显的优势之一。明码波形的单一温区设计在低温（< 10C）环境下容易出现较严重的残影（ghost image），表现为前一幅画面的"影子"叠加在当前显示内容上。打库波形的低温专用波形序列通过更长的预调节阶段和更强的驱动电压，大幅减少低温残影现象。

**5. 维护方便**

当面板供应商提供更新版本的波形文件时，只需要替换 `epd_waveform.bin` 文件并重新烧录，无需修改任何源代码。这对于批量生产和后期维护非常友好。

---

## 3. 切换方法

### 3.1 通过 menuconfig

```bash
scons --board=<你的板卡名称> --board_search_path=.. --menuconfig
```

导航路径：`Board → Built-in LCD module driver → 6inch EPD OPM060DA 8bit → Use binary waveform table`

勾选即启用打库波形，取消勾选即使用明码波形。

### 3.2 手动修改配置

在 `rtconfig.h` 中：

```c
// 启用打库波形
#define EPD_WAVEFORM_USE_BIN 1

// 禁用打库波形（使用明码波形）
// #define EPD_WAVEFORM_USE_BIN
```

### 3.3 编译与烧录

切换模式后，需要全量重新编译并完整烧录：

```bash
scons --board=<你的板卡名称> --board_search_path=.. -j8
```

> **重要提示：** 切换波形模式会改变编译产物（波形初始化等代码路径不同）和固件镜像内容（打库模式需要将 `epd_waveform.bin` 打包进 `wave_table` 分区）。建议执行全擦除后再烧录，避免新旧数据混合。

### 3.4 例程 SConstruct 添加打库下载

在使用打库波形的例程 `project/SConstruct` 中，需要添加以下代码，否则 `epd_waveform.bin` 不会被打包下载到对应的 `wave_table` 分区，导致运行时初始化失败：

```python
# Add EPD wave form image bin
if GetDepend('EPD_WAVEFORM_USE_BIN'):
    epd_wave_bin = SIFLI_SDK + "/customer/peripherals/display/epd_opm060da/waveform/epd_waveform.bin"
    AddCustomImg("wave_table", bin=[epd_wave_bin])
```

这段代码会根据 `EPD_WAVEFORM_USE_BIN` 宏自动判断是否需要将打库波形文件加入镜像。参考路径：`example/rt_driver/project/SConstruct`。

---

## 4. 运行时验证

启动时，驱动会打印波形初始化状态日志：

**打库波形初始化成功：**
```
EPD wave table BIN init OK, size=131072
```

**打库波形初始化失败（自动回退到明码波形）：**
```
Failed to initialize custom EPD wave table reader! err=X
```

如果打库波形初始化失败，驱动会静默回退到编译进固件的明码波形，不会导致系统崩溃。常见失败原因：

- `epd_waveform.bin` 未烧录到 Flash，或烧录地址与 `ptab.json` 中 `wave_table` 分区不一致
- `libepd_waveform_bin_reader_gcc.a` 库文件缺失或损坏
- NOR Flash 硬件异常

---

## 5. 对比总表

| 对比项 | 明码波形 | 打库波形 |
|---|---|---|
| 存储位置 | 固件 .text/.rodata（约 10 KB） | NOR Flash 分区（128 KB） |
| 温区数量 | 1 个（0-100C） | 多个（原厂定义） |
| 全刷新帧数 | 固定 32 帧 | 按温度动态调整 |
| 局刷新帧数 | 固定 12 帧 | 按温度动态调整 |
| 残影控制 | 一般 | 优秀（温区自适应） |
| 低温显示效果 | 较差（残影明显） | 良好（专用低温波形） |
| Flash 开销 | 无 | 需要 `wave_table` 分区 + BIN 文件 |
| 波形更新方式 | 修改源码 + 重新编译 | 替换 BIN 文件即可 |
| 编译依赖 | 无 | 需要 `libepd_waveform_bin_reader_gcc.a` |

---

## 6. 相关文件一览

| 文件 | 作用 |
|---|---|
| `epd_waveform.h` | 公共 API 声明 |
| `epd_waveform.c` | 明码波形数据 + 打库/明码分发逻辑 |
| `epd_opm060da.c` | LCD 初始化、波形表分发、帧渲染管线 |
| `epd_waveform_bin_reader_port.c` | RT-Thread 适配层：malloc/free/read_data 接口 |
| `waveform/epd_waveform.bin` | 原厂提供的二进制波形文件 |
| `waveform/libepd_waveform_bin_reader_gcc.a` | 波形读取预编译库 |
| `waveform/epd_waveform_bin_reader.h` | 波形读取库的头文件 |
| `SConscript` | 编译配置：`EPD_WAVEFORM_USE_BIN=y` 时链接 .a 库 |
| `../boards/Kconfig_lcd` | menuconfig 中 `EPD_WAVEFORM_USE_BIN` 开关定义 |
