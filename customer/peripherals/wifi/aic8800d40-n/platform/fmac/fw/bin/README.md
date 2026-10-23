# AIC8800 固件 .bin 文件目录

> **用途**: `CONFIG_LOAD_FW=1` (flashmode) 模式下，这些 .bin 文件需预先烧录到 flash 分区；
> `CONFIG_LOAD_FW=0` (filemode) 模式下，平台需将 `AICBSP_FW_PATH` 指向对应 family 目录。
>
> `CONFIG_LOAD_FW=2` (innermode) 模式不使用本目录，固件从 `fw/*.h` 内嵌数组加载。

---

## 1. 目录结构（按 family 分组）

```
fw/bin/
├── D80/    # AIC8800D80 family (AIC_CHIP_MODEL=D80)
├── DC/     # AIC8800DC/DL family (AIC_CHIP_MODEL=DC/DC_H/DL/DL_H)
└── DW/     # AIC8800DW family (AIC_CHIP_MODEL=DW/DW_H, 复用 DC 固件)
```

> **DW 说明**: AIC8800DW 与 AIC8800DC 固件完全相同，Linux 驱动未提供独立 aic8800DW 固件目录。
> DW/DW_H 型号请使用 `DC/` 目录下的对应固件。本目录保留为空，仅作 family 占位。

---

## 2. AIC_CHIP_MODEL 与 family 映射

| AIC_CHIP_MODEL | 编译宏 | AIC_LIB_FAMILY | 固件目录 | 说明 |
|----------------|--------|----------------|----------|------|
| `D80` | `CONFIG_AIC8800D80` | D80 | `D80/` | 支持 5G |
| `DC` | `CONFIG_AIC8800DC` | DC | `DC/` | |
| `DC_H` | `CONFIG_AIC8800DC` + `CONFIG_AIC8800DC_H` | DC | `DC/` | sub_id=2, 仅 _h_u02 固件 |
| `DL` | `CONFIG_AIC8800DC` + `CONFIG_NO_SDIO_FUNC_RELEASE` | DC | `DC/` | |
| `DL_H` | `CONFIG_AIC8800DC` + `CONFIG_AIC8800DC_H` + `CONFIG_NO_SDIO_FUNC_RELEASE` | DC | `DC/` | |
| `DW` | `CONFIG_AIC8800DW` | DW | `DC/` | 复用 DC 固件 |
| `DW_H` | `CONFIG_AIC8800DW` + `CONFIG_AIC8800DW_H` | DW | `DC/` | 复用 DC 固件, sub_id=2 |

---

## 3. 固件文件命名规则

```
fmacfw_<chip>[_h]_u02[_usb|_ipc].bin      # WiFi 固件 (正常模式)
lmacfw_rf_<chip>[_u02].bin                # WiFi 固件 (RF 测试模式)
fmacfw_patch_<chip>[_h]_u02.bin           # Patch 固件
fmacfw_patch_tbl_<chip>[_h]_u02.bin       # Patch Table 固件
fmacfw_calib_<chip>[_h]_u02.bin           # 校准固件 (仅 DC/DW)
fw_patch_<chip>_u02[_ext0|_ext1].bin      # Patch 数据
fw_patch_table_<chip>_u02.bin             # Patch Table 数据
fw_adid_<chip>_u02.bin                    # ADID 数据
```

**后缀说明**:
- `_h` : H 版本 (chip sub_id=2, 对应 DC_H/DW_H/DL_H)
- `_u02`: 固件版本号 U02
- `_usb`: USB 总线专用固件
- `_ipc`: IPC 模式 (CONFIG_FOR_IPCAM)
- `_ext0`/`_ext1`: 扩展 patch

---

## 4. 各型号实际加载的固件

### 4.1 D80 family (`AIC_CHIP_MODEL=D80`)

**D80 (非 H 版本, U02)** — SDIO 总线, 正常模式:

| 固件类型 | 文件名 | 加载入口 |
|---------|--------|---------|
| WiFi 固件 | `fmacfw_8800d80_u02.bin` | `aicwf_rwnx_load_firmware()` |
| WiFi RF测试固件 | `lmacfw_rf_8800d80_u02.bin` | `aicwf_rwnx_load_firmware(mode=RFTEST)` |
| Patch 数据 | `fw_patch_8800d80_u02.bin` | `aic_fw_ptr_get(FW_PATCH)` (innermode) |
| Patch Table | `fw_patch_table_8800d80_u02.bin` | `aic_fw_ptr_get(FW_PATCH_TABLE)` (innermode) |
| ADID 数据 | `fw_adid_8800d80_u02.bin` | `aic_fw_ptr_get(FW_ADID)` (innermode) |

**D80 H 版本** — SDIO 总线, 正常模式:
- WiFi 固件: `fmacfw_8800d80_h_u02.bin` (非 IPC) / `fmacfw_8800d80_h_u02_ipc.bin` (IPC)
- WiFi RF测试固件: `lmacfw_rf_8800d80_u02.bin` (与非 H 共用)

**D80 U04 版本** (patch 更新):
- Patch 数据: `fw_patch_8800d80_u04.bin`
- Patch Table: `fw_patch_table_8800d80_u04.bin`

### 4.2 DC family (`AIC_CHIP_MODEL=DC/DC_H/DL/DL_H`)

**DC (sub_id=0, U01)** :
- WiFi 固件: `fmacfw_patch_8800dc.bin`
- WiFi RF测试: `lmacfw_rf_8800dc.bin` 或 `fmacfw_rf_patch_8800dc.bin`

**DC (sub_id=1, U02)** :
- WiFi 固件: `fmacfw_patch_8800dc_u02.bin`
- WiFi RF测试: `lmacfw_rf_8800dc.bin`
- Patch Table: `fmacfw_patch_tbl_8800dc_u02.bin`
- 校准固件: `fmacfw_calib_8800dc_u02.bin`

**DC_H (sub_id=2, H版本)** :
- WiFi 固件: `fmacfw_patch_8800dc_h_u02.bin`
- WiFi RF测试: `lmacfw_rf_8800dc.bin`
- Patch Table: `fmacfw_patch_tbl_8800dc_h_u02.bin`
- 校准固件: `fmacfw_calib_8800dc_h_u02.bin`

### 4.3 DW family (`AIC_CHIP_MODEL=DW/DW_H`)

> DW 与 DC 固件完全相同，加载逻辑通过 `chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW` 共用。
> 请使用 `DC/` 目录下的对应固件。

---

## 5. 总线与 IPC 模式对固件选择的影响

D80 H 版本的 WiFi 固件按总线/IPC 模式区分（见 `aic_fw_port.c` `aicwf_rwnx_load_firmware()`）:

| 总线 | CONFIG_FOR_IPCAM | 固件文件名 |
|------|------------------|-----------|
| SDIO | 未定义 | `fmacfw_8800d80_h_u02.bin` |
| SDIO | 已定义 | `fmacfw_8800d80_h_u02_ipc.bin` |
| USB | 未定义 | `fmacfw_8800d80_h_u02_usb.bin` |
| USB | 已定义 | `fmacfw_8800d80_h_u02_usb_ipc.bin` |

> 非 H 版本的 D80 (U02) 固件不区分总线/IPC，统一使用 `fmacfw_8800d80_u02.bin`。

---

## 6. 来源与同步

- **来源**: Linux/Android 驱动 `USB/driver_fw/fw/aic8800D80/` 和 `USB/driver_fw/fw/aic8800DC/`
- **同步规则**: 基准驱动更新固件时，按 `docs/平台驱动更新原则.md` 第 3.4 节，固件文件全量覆盖
- **生成工具**: `.h` 数组文件由 `tools/py/bin2hdr_py3.py` 从 .bin 生成（innermode 使用）

---

## 7. 文件清单

### D80/ (14 个)

```
fmacfw_8800d80_h_u02.bin           # D80 H版本 SDIO 固件
fmacfw_8800d80_h_u02_ipc.bin       # D80 H版本 SDIO IPC 固件
fmacfw_8800d80_u02.bin             # D80 U02 SDIO 固件
fmacfw_8800d80_u02_ipc.bin         # D80 U02 SDIO IPC 固件
lmacfw_rf_8800d80_u02.bin          # D80 RF测试固件
fw_adid_8800d80_u02.bin            # ADID 数据
fw_patch_8800d80_u02.bin           # Patch 数据
fw_patch_8800d80_u02_ext0.bin      # Patch 扩展0
fw_patch_8800d80_u02_ext1.bin      # Patch 扩展1
fw_patch_8800d80_u04.bin           # U04 Patch 数据
fw_patch_table_8800d80_u02.bin     # U02 Patch Table
fw_patch_table_8800d80_u04.bin     # U04 Patch Table
fw_ble_scan_ad_filter.bin          # BLE 扫描过滤
calibmode_8800d80.bin              # 校准模式固件
```

### DC/ (16 个)

```
fmacfw_patch_8800dc_u02.bin        # DC U02 固件
fmacfw_patch_8800dc_h_u02.bin      # DC H版本 U02 固件
fmacfw_patch_8800dc_ipc_u02.bin    # DC U02 IPC 固件
fmacfw_patch_tbl_8800dc_u02.bin    # DC U02 Patch Table
fmacfw_patch_tbl_8800dc_h_u02.bin  # DC H版本 Patch Table
fmacfw_patch_tbl_8800dc_ipc_u02.bin # DC IPC Patch Table
fmacfw_calib_8800dc_u02.bin        # DC U02 校准固件
fmacfw_calib_8800dc_h_u02.bin      # DC H版本 校准固件
lmacfw_rf_8800dc.bin               # DC RF测试固件
fw_adid_8800dc_u02.bin             # ADID 数据 (U02)
fw_adid_8800dc_u02h.bin            # ADID 数据 (U02H)
fw_patch_8800dc_u02.bin            # Patch 数据
fw_patch_8800dc_u02_ext0.bin       # Patch 扩展0
fw_patch_8800dc_u02h.bin           # Patch 数据 (U02H)
fw_patch_table_8800dc_u02.bin      # Patch Table
fw_patch_table_8800dc_u02h.bin     # Patch Table (U02H)
```

### DW/ (空, 复用 DC/)

```
# DW 与 DC 固件相同, 请使用 DC/ 目录下的文件
```
