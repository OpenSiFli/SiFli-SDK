
# CA 证书文件

- `certs/default` 目录中存储着常用的 CA 证书文件
- `certs` 目录下存储着用户增加的 CA 证书文件

如果 `certs/default` 目录下没有包含用户需要的 CA 根证书文件，则需要用户将自己的 PEM 格式的 CA 证书拷贝 `certs` 根目录下。（仅支持 PEM 格式证书，不支持 DER 格式证书）

## 说明

- `PEM 格式证书`

    **PEM 格式证书** 通常是以 **.pem** 和 **.cer** 后缀名结尾的文件。

    使用文本编辑器打开后，文件内容以 `-----BEGIN CERTIFICATE-----` 开头，以 `-----END CERTIFICATE-----` 结尾。
- `DER 格式证书`

    **DER 格式证书** 是二进制文件类型。


## 额外证书搜索目录（可选）

当证书不便于集中拷贝到 `certs` 目录时（例如多个板型共用同一份代码库、证书由其他子工程维护、或希望直接引用外部路径），可以通过 Kconfig 选项 `PKG_USING_MBEDTLS_EXTRA_CERT_DIRS` 指定额外的搜索目录。

### 配置方法

在 menuconfig 中：

```
RT-Thread online packages --->
  security packages  --->
        mbedtls lts version (latest)  --->
        ( ) Extra CA cert search directories (semicolon separated)
```

填入分号 `;` 分隔的目录列表，每个条目可以是：

- **绝对路径**：例如 `/etc/ssl/certs`、`/Volumes/work/my_board_certs`
- **相对路径**：相对于 `mbedtls_228` 软件包根目录（即包含本 Kconfig 的目录），例如 `certs/my_board`、`certs/customer_a`

### 示例

```
certs/my_board;certs/customer_a
```

或

```
/etc/ssl/certs
```

### 收集规则

- 构建时 SConscript 会遍历每个配置的目录，仅收集 `.pem` / `.cer` / `.crt` 后缀文件
- 读取每个文件首行，必须以 `-----BEGIN CERTIFICATE` 开头才被采纳，其他文件会被静默跳过
- 与 `certs/default/`（受 `PKG_USING_MBEDTLS_USE_ALL_CERTS` 控制）和 `certs/`（受 `PKG_USING_MBEDTLS_USER_CERTS` 控制）的证书列表合并后去重
- 配置项留空时行为与未引入该选项前完全一致（向后兼容）

### 与其他证书选项的关系

| Kconfig 选项 | 扫描目录 | 触发条件 |
|---|---|---|
| `PKG_USING_MBEDTLS_USE_ALL_CERTS` | `certs/default/` | 选中后扫描整个目录 |
| `PKG_USING_MBEDTLS_USER_CERTS` | `certs/`（根目录） | 选中后扫描整个目录 |
| `PKG_USING_MBEDTLS_<CA>_ROOT_CA`（12 个单选） | `certs/default/<file>.cer` | 选中对应 CA 后追加 |
| `PKG_USING_MBEDTLS_EXTRA_CERT_DIRS` | 用户指定的任意目录 | 填入非空字符串后扫描 |

多个选项可叠加使用，最终证书列表为各选项收集结果去重后的并集。