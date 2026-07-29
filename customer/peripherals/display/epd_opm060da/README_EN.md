# EPD OPM060DA Waveform Table Switching Guide

[English](README_EN.md) | [中文](README.md)

## Overview

The OPM060DA EPD driver supports two waveform table modes:

| Mode | Kconfig | Storage | Waveform Source |
|---|---|---|---|
| Source-embedded waveform | `EPD_WAVEFORM_USE_BIN=n` (default) | Compiled into firmware | C arrays in `epd_waveform.c` |
| Binary pre-built waveform | `EPD_WAVEFORM_USE_BIN=y` | NOR Flash `wave_table` partition | Vendor-provided `epd_waveform.bin` |

The two modes differ in storage, LUT data format, and refresh behavior (frame count, temperature zone adaptation).

---

## 1. Source-Embedded Waveform (Default)

### 1.1 Storage

Waveform data is stored as C source arrays in `epd_waveform.c`:

```c
// Full refresh waveform: 32 frames, single temperature zone (0-100C)
static const uint8_t yzc085_wave_full_0_100[32][256] = { ... };

// Partial refresh waveform: 12 frames, single temperature zone (0-100C)
static const uint8_t yzc085_wave_partial_0_100[12][256] = { ... };
```

Each frame is 256 bytes, representing a 16x16 lookup table (high 4 bits = old pixel value, low 4 bits = new pixel value -> 2-bit waveform output).

### 1.2 LUT Data Format

Waveform values are placed directly at **bit[1:0]** of 32-bit LUT entries, matching the EPD_8BIT + F2_SWAP hardware read position:

```c
// Source-embedded waveform: values written directly to bit[1:0]
for (uint16_t i = 0; i < 256; i++)
    p_argb8888_lut[i] = 0xFF000000 | p_frame_wave[i];
```

### 1.3 Refresh Modes

| Mode | Enum Value | Frames | Behavior |
|---|---|---|---|
| AUTO | `EPD_DRAW_MODE_AUTO = 1` | Full 32 / Partial 12 | Automatically selects full or partial based on refresh counter (1 full refresh after every 10 partial refreshes) |
| FULL | `EPD_DRAW_MODE_FULL = 2` | 32 | Always full refresh |
| PARTIAL | `EPD_DRAW_MODE_PARTIAL = 3` | 12 | Always partial refresh |

### 1.4 Full vs. Partial Refresh Effect Analysis

**Full Refresh (32 frames):**

The 32-frame waveform sequence drives ink particles frame-by-frame from the current state to the target state. Each frame applies slightly different voltage patterns, gradually constructing the final image:

- Frames 0-7: Pre-conditioning phase (ghost clearing) — predominantly output value 2 (white drive)
- Frames 8-15: Writing phase (driving to target grayscale) — mixed output values 1 and 2 (black/white drive)
- Frames 16-23: Post-conditioning phase (image stabilization) — predominantly output value 1 (black drive)
- Frames 24-31: Final stabilization phase — balanced output

This multi-stage approach ensures complete ghost removal and consistent full-screen contrast. Total full refresh time is approximately 32 x single-frame period.

**Partial Refresh (12 frames):**

The 12-frame waveform only drives pixels that have changed between the old and new images, leaving unchanged areas untouched. Partial refresh is faster (12 frames vs. 32 frames) but may leave subtle ghosting in fine grayscale transition areas.

**Limitations of source-embedded mode:** Supports only a single temperature zone (0-100C). At extreme temperatures, waveform effectiveness may degrade, leading to reduced contrast or increased ghosting.

---

## 2. Binary Pre-built Waveform (`EPD_WAVEFORM_USE_BIN=y`)

### 2.1 Storage

The binary waveform file `epd_waveform.bin` is provided by the panel vendor and stored in the `waveform/` directory. It is automatically packaged into the firmware image at build time:

```python
# SConstruct
if GetDepend('EPD_WAVEFORM_USE_BIN'):
    AddCustomImg("wave_table", bin=[epd_wave_bin])
```

At runtime, the pre-compiled library `libepd_waveform_bin_reader_gcc.a` reads waveform data from the NOR Flash `wave_table` partition via RT-Thread's flash read interface:

```c
// epd_waveform_bin_reader_port.c
int waveform_bin_reader_read_data(uint32_t offset, uint8_t *buf, uint32_t size) {
    return rt_flash_read(CUSTOM_EPD_WAVE_TABLE_START_ADDR + offset, buf, (int)size);
}
```

Partition address and size are auto-generated from `ptab.json`:
- `CUSTOM_EPD_WAVE_TABLE_START_ADDR` — e.g. `0x14CE8000`
- `CUSTOM_EPD_WAVE_TABLE_SIZE` — e.g. `0x00020000` (128 KB)

### 2.2 LUT Data Format

The binary waveform library outputs waveform values at **bit[4:3]** (EPIC data format convention). Since EPD_8BIT + F2_SWAP hardware reads from **bit[1:0]**, a right-shift conversion is required:

```c
// Binary waveform: library fills bit[4:3], right-shift by 3 to bit[1:0] for hardware
waveform_bin_reader_fill_lut(p_argb8888_lut, frame_num);
for (uint16_t i = 0; i < 256; i++)
    p_argb8888_lut[i] = (p_argb8888_lut[i] >> 3) | 0xFF000000;
```

### 2.3 Multi-Temperature Zone Support

Unlike the source-embedded waveform (single 0-100C zone), the pre-built waveform supports **multiple temperature segments**. Frame count and waveform data are dynamically selected based on current temperature:

```c
uint32_t epd_wave_table_get_frames(int temperature, EpdDrawMode mode) {
    // Pre-built waveform: dynamically get frame count by temperature and mode
    return waveform_bin_reader_get_frames(temperature, mode);
}
```

This ensures optimal refresh quality across the full operating temperature range.

### 2.4 Advantages of Pre-built Waveform for Full and Partial Refresh

Compared to source-embedded waveforms, pre-built waveforms offer the following significant advantages:

**1. Temperature-Adaptive Driving**

The vendor-provided pre-built waveform contains separate waveform sequences for multiple temperature zones. Unlike the source-embedded waveform which has only a single 0-100C generic waveform, the pre-built waveform provides dedicated drive sequences for different temperature ranges (e.g., 0-5C, 5-10C, 10-15C...50-100C). At lower temperatures, ink particle activity decreases, requiring longer drive times and stronger voltage sequences; at higher temperatures, particles respond faster, allowing shorter drive times. Temperature adaptation ensures consistent contrast and ghosting control from low to high temperatures.

**2. Dynamic Frame Count Optimization**

Pre-built waveform frame counts are no longer fixed at 32 (full) or 12 (partial), but vary dynamically with temperature and mode. At favorable temperatures, frame counts may be lower for faster refresh; at extreme temperatures, frame counts may increase to ensure refresh quality. This flexibility balances speed and quality.

**3. Professional Vendor Tuning**

The pre-built waveform is professionally designed and validated by the panel vendor for the specific panel model. The vendor possesses the panel's complete physical model and test data, enabling precise calculation of optimal voltage waveform sequences for each temperature point and each grayscale transition based on the panel's ink characteristics (particle size, viscosity, dielectric constant, etc.). This level of precision is unattainable with generic source-embedded waveforms.

**4. Significant Low-Temperature Ghosting Suppression**

This is one of the most noticeable advantages of pre-built waveforms. The single temperature zone design of source-embedded waveforms tends to produce more severe ghost images in low-temperature (< 10C) environments, where the "shadow" of the previous image overlays the current display content. The pre-built waveform's low-temperature-specific sequences, with longer pre-conditioning phases and stronger drive voltages, substantially reduce low-temperature ghosting.

**5. Easy Maintenance**

When the panel vendor provides an updated waveform file, simply replace the `epd_waveform.bin` file and re-flash — no source code changes required. This is highly convenient for mass production and ongoing maintenance.

---

## 3. Switching Methods

### 3.1 Via menuconfig

```bash
scons --board=<your_board_name> --board_search_path=.. --menuconfig
```

Navigation path: `Board -> Built-in LCD module driver -> 6inch EPD OPM060DA 8bit -> Use binary waveform table`

Check to enable pre-built waveform, uncheck to use source-embedded waveform.

### 3.2 Manual Configuration

In `rtconfig.h`:

```c
// Enable pre-built waveform
#define EPD_WAVEFORM_USE_BIN 1

// Disable pre-built waveform (use source-embedded waveform)
// #define EPD_WAVEFORM_USE_BIN
```

### 3.3 Build and Flash

After switching modes, perform a full rebuild and complete flash:

```bash
scons --board=<your_board_name> --board_search_path=.. -j8
```

> **Important:** Switching waveform modes changes compiled output (different code paths for waveform initialization) and firmware image content (pre-built mode requires packaging `epd_waveform.bin` into the `wave_table` partition). A full erase before flashing is recommended to avoid mixing old and new data.

### 3.4 Add Binary Download to Example SConstruct

In the example project's `project/SConstruct`, the following code must be added; otherwise `epd_waveform.bin` will not be packaged and downloaded to the corresponding `wave_table` partition, causing runtime initialization failure:

```python
# Add EPD wave form image bin
if GetDepend('EPD_WAVEFORM_USE_BIN'):
    epd_wave_bin = SIFLI_SDK + "/customer/peripherals/display/epd_opm060da/waveform/epd_waveform.bin"
    AddCustomImg("wave_table", bin=[epd_wave_bin])
```

This code automatically determines whether to include the pre-built waveform file based on the `EPD_WAVEFORM_USE_BIN` macro. Reference path: `example/rt_driver/project/SConstruct`.

---

## 4. Runtime Verification

At startup, the driver prints waveform initialization status logs:

**Pre-built waveform initialization successful:**
```
EPD wave table BIN init OK, size=131072
```

**Pre-built waveform initialization failed (auto-fallback to source-embedded waveform):**
```
Failed to initialize custom EPD wave table reader! err=X
```

If pre-built waveform initialization fails, the driver silently falls back to the source-embedded waveform compiled into firmware without causing a system crash. Common failure causes:

- `epd_waveform.bin` not flashed, or flash address inconsistent with `wave_table` partition in `ptab.json`
- `libepd_waveform_bin_reader_gcc.a` library missing or corrupted
- NOR Flash hardware anomaly

---

## 5. Comparison Summary

| Item | Source-Embedded | Pre-built Binary |
|---|---|---|
| Storage | Firmware .text/.rodata (~10 KB) | NOR Flash partition (128 KB) |
| Temperature zones | 1 (0-100C) | Multiple (vendor-defined) |
| Full refresh frames | Fixed 32 | Dynamic by temperature |
| Partial refresh frames | Fixed 12 | Dynamic by temperature |
| Ghosting control | Average | Excellent (temperature-adaptive) |
| Low-temp display | Poor (visible ghosting) | Good (dedicated low-temp waveform) |
| Flash overhead | None | Requires `wave_table` partition + BIN file |
| Waveform update method | Modify source + recompile | Replace BIN file only |
| Build dependency | None | Requires `libepd_waveform_bin_reader_gcc.a` |

---

## 6. Related Files

| File | Purpose |
|---|---|
| `epd_waveform.h` | Public API declarations |
| `epd_waveform.c` | Source-embedded waveform data + binary/source dispatch logic |
| `epd_opm060da.c` | LCD initialization, waveform table dispatch, frame rendering pipeline |
| `epd_waveform_bin_reader_port.c` | RT-Thread adaptation layer: malloc/free/read_data interfaces |
| `waveform/epd_waveform.bin` | Vendor-provided binary waveform file |
| `waveform/libepd_waveform_bin_reader_gcc.a` | Pre-compiled waveform reader library |
| `waveform/epd_waveform_bin_reader.h` | Waveform reader library header |
| `SConscript` | Build config: links .a library when `EPD_WAVEFORM_USE_BIN=y` |
| `../boards/Kconfig_lcd` | `EPD_WAVEFORM_USE_BIN` switch definition in menuconfig |
