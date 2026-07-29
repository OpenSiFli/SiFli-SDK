# 加密块设备示例
源码路径: `example/rt_device/crypt_block_dev`

## 概述
例程展示了如何使用 `crypt_block_dev` 中间件在 SD/eMMC 块设备上创建一层透明的 AES 加密层，
所有写入底层设备的数据都会被 AES-CTR-128 加密，读取时自动解密，上层文件系统无需感知加密逻辑。

```
  应用层 (DFS / FatFs)
        ↓
  crypt_root / crypt_misc   (crypt_block_dev, AES-CTR-128 透明加解密)
        ↓
  root / misc                (mmcsd 分区设备, 存储密文)
        ↓
  sd0                         (底层 SD/eMMC 块设备, 存储密文及明文分区表)
```

## 支持平台
* sf32lb56-lcd_a128r12n1
* sf32lb58-lcd_n16r64n4
* sf32lb58-lcd_a128r32n1_dsi
* sf32lb58-lcd_n16r32n1_qspi

## 使用指南

### 编译
```powershell
cd example/rt_device/crypt_block_dev/project
scons --board=<board_name> -j8
```

### 运行
烧录后启动，系统通过 `INIT_ENV_EXPORT` 自动执行初始化：
1. 等待 SD/eMMC 设备就绪（超时约 8 秒）
2. 在原始 SD 卡上创建 `root` 和 `misc` 分区
3. 对每个分区分别创建加密层 `crypt_root` 和 `crypt_misc`（AES-CTR-128）
4. `/` 挂载到 `crypt_root`，`/misc` 挂载到 `crypt_misc`（首次使用自动执行 `mkfs`）

### 分区布局
| 分区 | 起始偏移 | 大小 |
|------|---------|------|
| root | 0x00081000 | 48 MB |
| misc | 0x00F81000 | 500 MB |

### Finsh 命令

#### 文件读写速度测试（通过文件系统）
| 命令 | 说明 |
|------|------|
| `crypt_write <path> <num_MB> [blocks_per_call]` | 加密写入速度测试，每调用写入 N 个扇区（默认 1） |
| `crypt_read <path> <num_MB> [blocks_per_call]` | 解密读取速度测试，每调用读取 N 个扇区（默认 1） |

示例：
```
crypt_write /test.bin 10       # 加密写入 10MB 文件，每次 1 扇区
crypt_write /test.bin 10 64    # 加密写入 10MB 文件，每次 64 扇区（32KB）
crypt_read /test.bin 10       # 解密读取 10MB 文件
```

#### 原始块设备读写速度测试（绕过文件系统）
| 命令 | 说明 |
|------|------|
| `crypt_raw_read <num_sectors> [blocks_per_call]` | 原始加密块设备读取速度测试 |
| `crypt_raw_write <num_sectors> [blocks_per_call]` | 原始加密块设备写入速度测试 |

示例：
```
crypt_raw_read 1024           # 读取 1024 个加密扇区
crypt_raw_write 2048 128      # 写入 2048 个加密扇区，每次 128 扇区（64KB）
```

#### 数据完整性验证
| 命令 | 说明 |
|------|------|
| `crypt_verify <path>` | 写入 1MB 随机数据，然后读取偏移 512KB 处的 512KB 进行逐字节比对 |

示例：
```
crypt_verify /verify.bin      # 写 1MB，验证后 512KB 是否正确
```

文件系统命令（`ls`, `df`, `mkdir` 等）与普通文件系统完全一致。

## menuconfig 配置
```powershell
sdk.py menuconfig --board=<board_name>
```
需确保以下配置开启：
- `BSP_USING_SDIO` / `BSP_USING_SDMMC1` — SD 卡驱动
- `BSP_USING_HW_AES` — AES 硬件模块
- `RT_USING_DFS_ELMFAT` — FAT 文件系统
- `USING_CRYPT_BLOCK_DEV` — 加密块设备中间件

## 注意事项
- 密钥 `aes_key`（16 字节）和初始向量 `aes_iv` 示例中使用的是硬编码测试值，**生产环境务必替换为安全密钥**
- 加密后的 SD 卡在其他设备上无法直接读取分区数据（数据为密文），但分区表（MBR）以明文存储
- AES-CTR-128 模式下，每个分区独立加密，使用相同的密钥和 IV
- 分区的扇区偏移和大小在 `main.c` 中通过 `FS_ROOT_OFFSET` / `FS_ROOT_LEN` / `FS_MISC_OFFSET` / `FS_MISC_LEN` 宏定义，可根据实际存储介质调整
- `blocks_per_call` 参数可用于测试不同缓冲区大小对读写性能的影响，值越大单次传输数据量越大，可能提升吞吐量
