# WPA AES 接口冲突评估与 aic_wpa 前缀重命名清单

> 日期: 2026-08-19 | 范围: host/thirdparty/wpa_supplicant AES 相关接口 | 验证: D80 host 全量编译 + 链接库符号核对

## 一、背景

WPA 内部 AES 接口（如 `aes_encrypt_init`、`omac1_aes_128`、`aes_wrap`）在链接到真实平台 SDK 时，可能与平台自带 AES 接口产生符号冲突（平台 SDK 常见自带 wpa_supplicant 精简版或 mbedtls/openssl 加密库，其导出的 `aes_*` 符号与本项目同名的概率极高）。统一添加 `aic_wpa` 前缀以隔离命名空间。

## 二、冲突评估结论

### 2.1 命名冲突（高风险，主要冲突类型）

| 符号类别 | 数量 | 风险说明 |
|---|---|---|
| AES 核心函数 `aes_encrypt/aes_decrypt/aes_*_init/aes_*_deinit` | 6 | 与上游 wpa_supplicant、openssl (`AES_encrypt`)、mbedtls 内部 `aes_*` 极易撞名 |
| AES 模式函数 `aes_wrap/aes_unwrap/omac1_aes_*/aes_128_cbc_*/aes_128_ctr_encrypt` 等 | 19 | 上游 wpa_supplicant 官方同名 API，平台自带 wpa 时 100% 冲突 |
| Rijndael 核心 `rijndaelKeySetupEnc` | 1 | 与任何携带 rijndael 参考实现的平台库冲突 |
| AES S 盒/轮常量表 `Te0~Te4/Td0~Td4/Td4s/rcon/rcons` | 13 | 导出为 R 符号，与平台侧同源 rijndael 实现的表名冲突 |

### 2.2 参数定义冲突（中风险）

上游 wpa 的 `aes_encrypt_init(const u8 *key, size_t len)` 与平台常见 `aes_encrypt_init(const unsigned char *key, int bits)` 参数类型/语义不同。同名不同参时 C 链接器不报错（无类型校验），静默错误调用导致密钥初始化错误——比编译期报错更危险，前缀隔离后彻底消除。

### 2.3 返回值类型冲突（中风险）

如 `omac1_aes_128` 上游返回 `int`（0 成功），部分平台实现返回 `void` 或布尔。链接期无法发现，运行期行为未定义。

### 2.4 功能实现冲突（低风险）

上游 `aes_wrap/unwrap` 遵循 RFC3394，语义标准化，撞名后功能本身等价，但 key 长度单位（字节 vs 位）、输入布局差异仍可能引发错误。

### 2.5 本项目内部冲突检查结果（重命名前实测）

| 检查项 | 结果 |
|---|---|
| 本工程 host/driver、platform 层对 AES 函数的调用 | 零引用（重命名完全封闭在 wpa_supplicant 内） |
| libaic_mbedtls.a 内 AES 符号 | 无（仅 bignum/ecp，且已带 aic_mbedtls_ 前缀） |
| 平台适配层 platform/ 内 aes_* 定义 | 无 |
| 结论 | 当前代码库内部无冲突；冲突风险全部来自**外部平台 SDK 链接期**，前缀化后消除 |

## 三、重命名清单

规则：**仅重命名链接期符号**（非 static 函数 + 导出全局表）。static 函数（如 `rijndaelEncrypt`、`gf_mulx`、`aes_gctr` 等）与宏（`AES_BLOCK_SIZE`、`AES_PRIV_SIZE`、`AES_SMALL_TABLES`、`TE0()` 等）不参与链接，保持原名。文件名与 `#include "aes_wrap.h"` 等头文件名不变。

### 3.1 AES 核心函数（6 个，定义于 aes-internal-enc/dec.c，声明于 aes.h + crypto.h）

| 原始名称 | 重命名后 |
|---|---|
| `aes_encrypt_init` | `aic_wpa_aes_encrypt_init` |
| `aes_encrypt` | `aic_wpa_aes_encrypt` |
| `aes_encrypt_deinit` | `aic_wpa_aes_encrypt_deinit` |
| `aes_decrypt_init` | `aic_wpa_aes_decrypt_init` |
| `aes_decrypt` | `aic_wpa_aes_decrypt` |
| `aes_decrypt_deinit` | `aic_wpa_aes_decrypt_deinit` |

### 3.2 Rijndael 核心（1 个，定义于 aes-internal.c，声明于 aes_i.h）

| 原始名称 | 重命名后 |
|---|---|
| `rijndaelKeySetupEnc` | `aic_wpa_rijndaelKeySetupEnc` |

### 3.3 AES 模式函数（19 个，声明于 aes_wrap.h / aes_siv.h）

| 原始名称 | 重命名后 | 定义文件 | 是否参与编译 |
|---|---|---|---|
| `aes_wrap` | `aic_wpa_aes_wrap` | aes-wrap.c | 是 |
| `aes_unwrap` | `aic_wpa_aes_unwrap` | aes-unwrap.c | 是 |
| `omac1_aes_vector` | `aic_wpa_omac1_aes_vector` | aes-omac1.c | 是 |
| `omac1_aes_128_vector` | `aic_wpa_omac1_aes_128_vector` | aes-omac1.c | 是 |
| `omac1_aes_128` | `aic_wpa_omac1_aes_128` | aes-omac1.c | 是 |
| `omac1_aes_256` | `aic_wpa_omac1_aes_256` | aes-omac1.c | 是 |
| `aes_128_cbc_encrypt` | `aic_wpa_aes_128_cbc_encrypt` | aes-cbc.c | 是 |
| `aes_128_cbc_decrypt` | `aic_wpa_aes_128_cbc_decrypt` | aes-cbc.c | 是 |
| `aes_128_encrypt_block` | `aic_wpa_aes_128_encrypt_block` | aes-encblock.c | 否（同步改名保持一致性） |
| `aes_ctr_encrypt` | `aic_wpa_aes_ctr_encrypt` | aes-ctr.c | 否（同上） |
| `aes_128_ctr_encrypt` | `aic_wpa_aes_128_ctr_encrypt` | aes-ctr.c | 否（同上） |
| `aes_128_eax_encrypt` | `aic_wpa_aes_128_eax_encrypt` | aes-eax.c | 否（同上） |
| `aes_128_eax_decrypt` | `aic_wpa_aes_128_eax_decrypt` | aes-eax.c | 否（同上） |
| `aes_gcm_ae` | `aic_wpa_aes_gcm_ae` | aes-gcm.c | 否（同上） |
| `aes_gcm_ad` | `aic_wpa_aes_gcm_ad` | aes-gcm.c | 否（同上） |
| `aes_gmac` | `aic_wpa_aes_gmac` | aes-gcm.c | 否（同上） |
| `aes_ccm_ae` | `aic_wpa_aes_ccm_ae` | aes-ccm.c | 否（同上） |
| `aes_ccm_ad` | `aic_wpa_aes_ccm_ad` | aes-ccm.c | 否（同上） |
| `aes_siv_encrypt` | `aic_wpa_aes_siv_encrypt` | aes-siv.c | 否（同上） |
| `aes_siv_decrypt` | `aic_wpa_aes_siv_decrypt` | aes-siv.c | 否（同上） |

### 3.4 AES 查找表（13 个，定义于 aes-internal.c，声明于 aes_i.h）

| 原始名称 | 重命名后 | 备注 |
|---|---|---|
| `Te0/Te1/Te2/Te3/Te4` | `aic_wpa_Te0~aic_wpa_Te4` | 当前 AES_SMALL_TABLES 配置下仅 Te0 实际导出 |
| `Td0/Td1/Td2/Td3/Td4` | `aic_wpa_Td0~aic_wpa_Td4` | 当前仅 Td0 实际导出 |
| `Td4s` | `aic_wpa_Td4s` | 实际导出 |
| `rcon` | `aic_wpa_rcon` | 大表分支用，当前不导出 |
| `rcons` | `aic_wpa_rcons` | 实际导出 |

aes_i.h 中的宏 `TE0()/TD0()/RCON(i)` 等引用表名处已随表名同步更新（74 处替换，含宏展开体内的引用）。

### 3.5 未重命名项及理由

| 项 | 理由 |
|---|---|
| static 函数（rijndaelEncrypt/rijndaelKeySetupDec/rijndaelDecrypt/gf_mulx/aes_gctr/aes_s2v 等） | 编译单元内部符号，不参与链接，无冲突可能 |
| 宏 AES_BLOCK_SIZE/AES_PRIV_SIZE/AES_PRIV_NR_POS/AES_SMALL_TABLES | 编译期文本替换，不进符号表 |
| 文件名 aes-*.c 与头文件名 aes*.h、`#include` 语句 | 非链接期符号；保留可降低与上游 diff 噪音 |
| `crypto_module_tests.c` 等 5 个未编译 crypto 后端（gnutls/libtomcrypt/linux/nettle/openssl/wolfssl）中的调用点 | 已同步重命名（符号引用一致性），但文件本身不参与编译 |

## 四、修改位置汇总（38 文件、390 处替换）

**核心实现（15 文件）**：aes-internal.c(26) aes_i.h(74) aes_wrap.h(18) aes.h(6) aes_siv.h(2) crypto.h(16) aes-cbc.c(10) aes-ccm.c(15) aes-ctr.c(8) aes-eax.c(12) aes-encblock.c(5) aes-gcm.c(12) aes-omac1.c(16) aes-siv.c(9) aes-wrap.c(5) aes-unwrap.c(5) aes-internal-enc.c(4) aes-internal-dec.c(6)

**生效调用点（9 文件）**：wpa.c(4) wpa_ft.c(2) wpa_auth.c(5) wpa_common.c(4) wps_common.c(1) wps_attr_build.c(1) milenage.c(7) crypto_internal-cipher.c(7) 及未编译的 tdls.c/eap_psk.c/pae/*/mesh_rsn.c/crypto 后端(其余)

**关键声明头**：aes.h、aes_wrap.h、aes_i.h、aes_siv.h、crypto.h（外部若 include 这些头，调用自动跟随新名）

## 五、验证结果

| 验证项 | 结果 |
|---|---|
| 源码残留检查（旧函数名/表名，全 wpa 树） | 零残留（仅 #include 文件名保留，属预期） |
| 双重前缀误伤（aic_wpa_aic_wpa_） | 零 |
| host 全量编译（D80, 174 objects） | 通过 |
| libaic8800_driver.a 符号核对（评估时为分型号库 libaic8800_driver_D80.a，现已单库统一为 libaic8800_driver.a） | 新符号 `aic_wpa_aes_encrypt_init/aic_wpa_omac1_aes_128/aic_wpa_aes_wrap` 等正常定义(T)与引用(U)；**旧符号零残留** |
| 表符号核对 | `aic_wpa_Te0/aic_wpa_Td0/aic_wpa_Td4s/aic_wpa_rcons` 为 R 符号正常导出 |
| host/driver、platform 外部引用 | 零（重命名无外部破坏面） |

**功能测试说明**：编译期与链接期验证已完成；运行期功能验证依赖真实平台联调（WPA2/PSK 连接、WPS、FT 等 AES 路径为现网回归项），本次重命名为纯符号名替换（签名/实现体零改动），不引入行为变化。

## 六、平台集成注意事项

1. 平台侧若有代码直接调用 wpa 的 AES 函数（如自研 supplicant glue），需改用 `aic_wpa_` 前缀名。
2. 若平台用 `USE_BUILTIN_MBEDTLS=0` 自实现 crypto（见 platform_config.mk），不受本次影响（35 个 `aic_mbedtls_*` 接口与 AES 前缀体系独立）。
3. 上游 wpa_supplicant 升级合并时，AES 相关文件冲突需按本清单重新应用前缀。
