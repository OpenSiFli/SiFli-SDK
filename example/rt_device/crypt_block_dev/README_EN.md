# Crypt Block Device Example
Source path: `example/rt_device/crypt_block_dev`

## Overview
This example demonstrates how to use the `crypt_block_dev` middleware to create a transparent AES encryption layer on top of an SD/eMMC block device.
All data written to the underlying device is encrypted with AES-CTR-128, and automatically decrypted on read. The upper file system is completely unaware of the encryption layer.

```
  Application Layer (DFS / FatFs)
        ↓
  crypt_root / crypt_misc   (crypt_block_dev, AES-CTR-128 transparent encryption/decryption)
        ↓
  root / misc                (mmcsd partition devices, stores ciphertext)
        ↓
  sd0                         (underlying SD/eMMC block device, stores ciphertext and plaintext MBR)
```

## Supported Platforms
* sf32lb56-lcd_a128r12n1
* sf32lb58-lcd_n16r64n4
* sf32lb58-lcd_a128r32n1_dsi
* sf32lb58-lcd_n16r32n1_qspi

## Usage Guide

### Build
```powershell
cd example/rt_device/crypt_block_dev/project
scons --board=<board_name> -j8
```

### Runtime
After flashing, the system automatically initializes via `INIT_ENV_EXPORT`:
1. Waits for the SD/eMMC device to be ready (timeout ~8 seconds)
2. Creates `root` and `misc` partitions on the raw SD card
3. Wraps each partition with an AES-CTR-128 encryption layer (`crypt_root`, `crypt_misc`)
4. Mounts `/` to `crypt_root` and `/misc` to `crypt_misc` (automatically runs `mkfs` on first use)

### Partition Layout
| Partition | Start Offset | Size |
|-----------|-------------|------|
| root      | 0x00081000  | 48 MB |
| misc      | 0x00F81000  | 500 MB |

### Finsh Commands

#### File Read/Write Speed Test (through filesystem)
| Command | Description |
|---------|-------------|
| `crypt_write <path> <num_MB> [blocks_per_call]` | Encrypted write speed test, N sectors per call (default 1) |
| `crypt_read <path> <num_MB> [blocks_per_call]` | Encrypted read speed test, N sectors per call (default 1) |

Examples:
```
crypt_write /test.bin 10       # Encrypted write 10MB, 1 sector per call
crypt_write /test.bin 10 64    # Encrypted write 10MB, 64 sectors per call (32KB)
crypt_read /test.bin 10       # Encrypted read 10MB
```

#### Raw Block Device Speed Test (bypasses filesystem)
| Command | Description |
|---------|-------------|
| `crypt_raw_read <num_sectors> [blocks_per_call]` | Raw encrypted block device read speed test |
| `crypt_raw_write <num_sectors> [blocks_per_call]` | Raw encrypted block device write speed test |

Examples:
```
crypt_raw_read 1024           # Read 1024 encrypted sectors
crypt_raw_write 2048 128      # Write 2048 encrypted sectors, 128 sectors per call (64KB)
```

#### Data Integrity Verification
| Command | Description |
|---------|-------------|
| `crypt_verify <path>` | Writes 1MB of random data, then reads back 512KB at offset 512KB and compares byte-by-byte |

Example:
```
crypt_verify /verify.bin      # Write 1MB, verify the last 512KB
```

Filesystem commands (`ls`, `df`, `mkdir`, etc.) work exactly as with a normal filesystem.

## menuconfig Configuration
```powershell
sdk.py menuconfig --board=<board_name>
```
Ensure the following options are enabled:
- `BSP_USING_SDIO` / `BSP_USING_SDMMC1` — SD card driver
- `BSP_USING_HW_AES` — AES hardware module
- `RT_USING_DFS_ELMFAT` — FAT filesystem
- `USING_CRYPT_BLOCK_DEV` — crypt block device middleware

## Notes
- The `aes_key` (16 bytes) and `aes_iv` in this example use hardcoded test values. **Replace them with secure keys in production environments.**
- An encrypted SD card cannot be read directly on other devices (data is ciphertext), though the MBR partition table remains in plaintext.
- In AES-CTR-128 mode, each partition is encrypted independently using the same key and IV.
- Partition sector offsets and sizes are defined in `main.c` via macros `FS_ROOT_OFFSET` / `FS_ROOT_LEN` / `FS_MISC_OFFSET` / `FS_MISC_LEN`. Adjust these according to your storage medium.
- The `blocks_per_call` parameter can be used to test the impact of buffer size on read/write performance — larger values transfer more data per call and may improve throughput.
