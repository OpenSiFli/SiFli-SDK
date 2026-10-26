# 安装环境

本节介绍 SiFli-SDK 开发环境的安装方法。我们提供了两种安装方式：

- **脚本安装**：使用自动化脚本快速安装开发环境
- **使用VSCode插件 CodeKit**：通过 VSCode 插件一键安装和管理 SiFli-SDK 开发环境，参考：[CodeKit 插件安装指南](https://docs.sifli.com/projects/codekit/get-started/)
- **ENV工具安装**：使用 ENV 工具进行环境配置和管理

请根据您的需求选择合适的安装方式：

```{toctree}
:maxdepth: 2

script/index
./lacey_install
```

(file-encryption-policies-in-development-environments)=
## 研发环境中的文件加密策略

如果研发电脑安装了文件透明加密或加密读取管控软件，请在安装和使用 SDK 前，联系企业 IT 管理员为相关工具配置加密策略白名单或解密策略，确保工具能够读取所需文件的明文内容。否则，安装脚本、Python 脚本或编译工具可能因读取到加密内容而报错。

### SDK 安装阶段：Python 与 uv 的文件访问权限

使用脚本或 CodeKit 安装 SDK 时，需要检查 **`uv.exe` 和实际使用的 `python.exe`** 是否均已加入加密策略白名单/解密策略范围内。Python 依赖构建过程中，Python 子进程会生成临时结果文件，供 `uv.exe` 读取；如果两个程序的策略不一致，可能出现文件已生成、但另一个程序无法读取明文的情况。仅放行 `python.exe` 对 `*.py` 的访问权限并不足以覆盖这一过程。

请根据安装日志中的实际路径，检查 SDK 管理的 Python 运行时、虚拟环境以及 uv 缓存和临时构建目录的策略，确保相关进程可以正常读写构建结果文件，包括 `*.txt` 等临时文件。例如，安装 Python 依赖时可能出现以下报错：

```text
failed to read ...\prepare_metadata_for_build_wheel.txt
stream did not contain valid UTF-8
```

已排查确认的客户案例中，加密软件导致工具读取到的文件内容为乱码，使 uv 无法按 UTF-8 读取上述构建结果文件，最终中断 SDK 安装。遇到此类问题时，请为 `uv.exe` 和实际使用的 `python.exe` 配置对应文件的加密豁免或解密权限，确保 Python 写出的文件能够被 uv 以明文读取。退出加密软件界面不一定会停止后台服务或文件过滤驱动，应在策略生效后验证文件是否能正常读取，再重新执行安装。

### 工具策略参考

请将实际使用的工具加入加密策略白名单或解密策略范围。SDK 安装时需配置 `uv.exe` 和实际使用的 `python.exe`；编译时按所用工具链配置相应策略。

| 需要加入加密策略的工具文件名 | 需要允许明文访问的文件类型 |
| --- | --- |
| `armclang.exe` | `*.c`、`*.s`、`*.h` |
| `armlink.exe` | `*.c`、`*.s`、`*.h` |
| `arm-none-eabi-gcc.exe` | `*.c`、`*.s`、`*.h` |
| `arm-none-eabi-ld.exe` | `*.c`、`*.s`、`*.h` |
| `lang.exe` | `*.c` |
| `lang_gen.exe` | `*.c`、`*.xlsx` |
| `uv.exe` | `*.txt` 等临时构建结果文件 |
| `python.exe` | `*.py`、`*.txt` 等临时构建结果文件 |

```{note}
请按实际使用的工具及其安装路径配置策略。SDK 使用的 Python 可能位于 SDK 管理的运行时或虚拟环境中；仅放行系统安装的 Python 可能无法解决问题。上表为参考范围，其他工具和文件请按实际情况配置。

如果已经出现报错，请在策略生效后确认相关文件能被对应工具正常解密读取，再重新执行安装或编译。对于仍保留加密内容的文件，请联系企业 IT 管理员处理。
```
