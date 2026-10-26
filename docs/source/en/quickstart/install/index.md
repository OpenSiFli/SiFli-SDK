# Installation Environment

This section introduces the installation methods for the SiFli-SDK development environment. We provide two installation approaches:

- **Script Installation**: Use automated scripts to quickly install the development environment
- **Using the CodeKit VSCode Extension**: Install and manage the SiFli-SDK development environment with one click through the VSCode extension. Refer to the [CodeKit extension installation guide](https://docs.sifli.com/projects/codekit/get-started/)
- **ENV Tool Installation**: Use the ENV tool for environment configuration and management

Please select the appropriate installation method based on your requirements:

```{toctree}
:maxdepth: 2

script/index
./lacey_install
```

(file-encryption-policies-in-development-environments)=
## File Encryption Policies in Development Environments

If your development computer uses transparent file encryption or software that controls access to encrypted files, contact your IT administrator before installing or using the SDK. Add the relevant tools to the encryption policy allowlist or grant them decryption access so they can read the required files as plaintext. Otherwise, installation scripts, Python scripts, or build tools may fail when they read encrypted content.

### SDK Installation: File Access for Python and uv

When installing the SDK through scripts or CodeKit, check that both **`uv.exe` and the `python.exe` actually used by the SDK** are covered by the encryption allowlist or decryption policy. During Python dependency builds, Python subprocesses generate temporary result files that `uv.exe` reads. If the two programs have different policies, one program may be unable to read the plaintext written by the other. Allowing `python.exe` to access only `*.py` files does not cover this process.

Use the actual paths in the installation log to check the policies for the SDK-managed Python runtime, virtual environment, uv cache, and temporary build directories. Ensure that the relevant processes can read and write build result files, including temporary files such as `*.txt`. For example, Python dependency installation may report:

```text
failed to read ...\prepare_metadata_for_build_wheel.txt
stream did not contain valid UTF-8
```

In a confirmed customer case, encryption software caused tools to read garbled file content. As a result, uv could not read the build result file above as UTF-8, and SDK installation stopped. To resolve this issue, configure encryption exemptions or decryption permissions for the relevant files for both `uv.exe` and the actual `python.exe`, so uv can read the files written by Python as plaintext. Closing the encryption software's interface may leave its background services or file filter drivers running. After the policy takes effect, verify that the files can be read correctly, then run the installation again.

### Tool Policy Reference

Add the tools used by your workflow to the encryption allowlist or decryption policy. For SDK installation, configure `uv.exe` and the `python.exe` actually in use. For builds, configure the tools used by your selected toolchain.

| Tool executable to include in the policy | File types requiring plaintext access |
| --- | --- |
| `armclang.exe` | `*.c`, `*.s`, `*.h` |
| `armlink.exe` | `*.c`, `*.s`, `*.h` |
| `arm-none-eabi-gcc.exe` | `*.c`, `*.s`, `*.h` |
| `arm-none-eabi-ld.exe` | `*.c`, `*.s`, `*.h` |
| `lang.exe` | `*.c` |
| `lang_gen.exe` | `*.c`, `*.xlsx` |
| `uv.exe` | Temporary build result files such as `*.txt` |
| `python.exe` | `*.py` and temporary build result files such as `*.txt` |

```{note}
Configure policies for the tools and installation paths actually in use. The SDK's Python interpreter may be in an SDK-managed runtime or virtual environment, so allowing only the system Python may not resolve the issue. Build tool policies must also cover the selected toolchain paths. The table above is a reference; configure policies for other scripts, configuration files, and tools used by your project as needed.

If an error has already occurred, confirm that the corresponding tools can decrypt and read the affected files after the policy takes effect, then run the installation or build again. Contact your IT administrator to handle any files that remain encrypted and unreadable by the tools.
```
