# Windows Installation Process

We recommend using the [CodeKit](https://marketplace.visualstudio.com/items?itemName=SiFli.sifli-sdk-codekit) VSCode extension to install SiFli-SDK and related tools.

## Installation Prerequisites

### `uv` Environment

Windows users do not need to pre-install system Python for the SDK scripts anymore. The supported workflow uses `uv` to provision the locked Python runtime on demand.

After installing `uv`, run the following command in PowerShell to verify it is available:

```powershell
uv --version
```

```{note}
`uv` is an extremely fast Python package and project management tool written in Rust. For installation instructions, refer to the [official uv documentation](https://docs.astral.sh/uv/getting-started/installation).
```

### Git Environment

If Git is not installed, please refer to the [Git official website](https://git-scm.com/download/win) to download and install Git. After installation, make sure to add Git to the system's environment variables.

```{note}
For domestic users in China, you can use the following domestic mirror link to download Git installer: <https://mirrors.huaweicloud.com/git-for-windows/v2.47.0.windows.1/Git-2.47.0-64-bit.exe>. Keep the default installation options each time.
```

After installation, you can run the `git --version` command in the terminal to check if Git is installed successfully. Normally, it should output Git version information, such as:

```powershell
git version 2.47.0.windows.1
```

### Terminal Setup

SiFli-SDK script installation only supports `powershell`.

For terminal choice, we recommend using [Windows Terminal](https://aka.ms/terminal). Users can also choose other terminals, such as the integrated terminal that comes with VSCode. However, Windows Terminal is more recommended. Note that in some newer Windows 10/11 versions, Windows Terminal is already pre-installed.

To open `PowerShell`, you can use the following methods:

- Press the Win key or click the Windows icon in the lower left corner, type `powershell`, then click to open PowerShell terminal.
- Press Win + R key combination to open the Run window, type `powershell`, then click OK.

If you are using Windows Terminal, you can directly open PowerShell in the terminal. To open the terminal, you can press the Win key or click the Windows icon in the lower left corner, type `terminal`, then click to open Windows Terminal.

If you encounter the error message 
`Cannot load file C:\OpenSiFli\SiFli-SDK\export.ps1 because running scripts is disabled on this system.` in the subsequent script running steps, or if you have never heard of or run `.ps1` scripts before, please open PowerShell terminal in **administrator mode** and run the following command:

```powershell
Set-ExecutionPolicy RemoteSigned
```

Then type `Y` and press Enter to gain permission to run scripts.

## Get SiFli-SDK

Before building applications around SF32, please first obtain the software library files provided by SiFli from the [SiFli-SDK repository](https://github.com/OpenSiFli/SiFli-SDK).

Get a local copy of SiFli-SDK: Open terminal, switch to the working directory where you want to save SiFli-SDK, and use the `git clone` command to clone the remote repository. Generally, we recommend using code from the release branch to get the latest stable version.

```{warning}

Since SiFli-SDK contains submodules, you cannot obtain the complete code by downloading the zip package.

```

Open PowerShell terminal and run the following commands:

```powershell
mkdir -p C:\OpenSiFli
cd C:\OpenSiFli
git clone --recursive -b release/v2.4 https://github.com/OpenSiFli/SiFli-SDK
```

````{note}
The above SDK path is for example only, users can choose the path according to their needs.

If accessing GitHub is slow in China, you can use the `gitee` mirror to clone SiFli-SDK. Please use the following command:
```powershell
git clone --recursive -b release/v2.4 https://gitee.com/SiFli/sifli-sdk
```

Note that the SiFli-SDK repository path on gitee is all lowercase, so you need to pay attention to case sensitivity when `SiFli-SDK` appears later.
````

````{note}
If you want to switch to other branches (e.g., development branch), you can use the `checkout` command, for example:


```powershell
git checkout main
```
or
```powershell
git checkout release/v2.3
```

````

````{note}
Note that SiFli-SDK contains some submodules, so you need to use the `--recursive` parameter to clone all submodules. If you forgot this parameter when cloning, you can run the following command after cloning to initialize the submodules:

```bash
git submodule update --init --recursive
```
````

## Install Tools

In addition to SiFli-SDK itself, you also need to install various tools used by SiFli-SDK for projects supporting SF32, such as compilers, debuggers, Python packages, etc.

```powershell
cd C:\OpenSiFli\SiFli-SDK
.\install.ps1
```

`install.ps1` will:

- use `uv` to provision the locked Python runtime
- sync the locked Python dependency graph from `tools/locks/default/pyproject.toml` and `tools/locks/default/uv.lock`
- install the SDK toolchain versions bound by `tools/locks/default/lock.json`
- initialize the profile-specific Conan home under `SIFLI_SDK_TOOLS_PATH`

If you need to build with Keil/ARMCLANG, record the Keil root during installation. The directory must already exist and contain `ARM\ARMCLANG\bin`:

```powershell
.\install.ps1 --keil C:\Keil_v5
```

If the SDK environment is already installed, run the same command again. `install.ps1` is idempotent: it reuses the existing Python, tool, and Conan state, and records the Keil path in `${SIFLI_SDK_TOOLS_PATH}\sifli-sdk-env.json`.

````{note}
Domestic users in China can use the following commands to enable the bundled China mirror preset and avoid slow downloads from default sources. Note that if you choose to execute the following commands, you do not need to execute the commands in the above code block.

```powershell
cd C:\OpenSiFli\SiFli-SDK
$env:SIFLI_SDK_MIRROR_CHINA="1"
.\install.ps1
```

When enabled, this preset forcefully overrides the following environment variables:

- `SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"`
- `SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"`
- `UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"`
- `UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"`

If you do not want the bundled preset, you can still set the fine-grained mirror variables manually.

````

### Custom Tool Installation Path (Optional)

The script described in this step installs the compilation tools required by SiFli-SDK in the user's home directory by default, which is the `$HOME/.sifli` directory in *nix systems, or `C:\Users\<name>\.sifli` in Windows. We can choose to install tools to other directories, but please export the environment variable `SIFLI_SDK_TOOLS_PATH` before running the installation script. Note that please ensure the user account has read and write permissions for that path.

```powershell
$env:SIFLI_SDK_TOOLS_PATH="D:\SIFLI\tools"
.\install.ps1

.\export.ps1
```

If you modified the `SIFLI_SDK_TOOLS_PATH` variable, please export this variable to the environment variables before running any SiFli-SDK tools or scripts.

```{note}
For Windows users, if your username contains spaces, Chinese characters, or other non-English characters, it is strongly recommended to set `SIFLI_SDK_TOOLS_PATH` to specify the tool installation path, otherwise it may cause installation failure or compilation errors. It is recommended to set this path to a pure English path, such as `D:\SIFLI\tools`.
```

```{note}
If environment variables are not exported, most shells will not support using `SIFLI_SDK_TOOLS_PATH` in variable assignments, such as `SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path" ./install.ps1`. Because even if the variable is exported or modified in the source script, the current execution environment is not affected by variable assignment.
```

## Set Environment Variables

After the above steps, the SDK and related tools are installed, but their paths are not in the environment variables, so they cannot be used in any directory. Therefore, some environment variables must be set. This can be done through another script provided by SiFli-SDK.

Please run the following command in terminal windows where you need to use compilation or download commands:

```powershell
cd C:\OpenSiFli\SiFli-SDK
.\export.ps1
```

Without `-t`, `export.ps1` exports the GCC environment. This is equivalent to:

```powershell
.\export.ps1 -t gcc
```

If you have recorded a Keil root with `install.ps1 --keil`, switch to Keil/ARMCLANG with:

```powershell
.\export.ps1 -t keil
```

`export.ps1` now invokes `tools/sdk_env.py export` through `uv run`. The environment manager resolves the current `profile + lock` snapshot to the matching SDK environment instance and uses the Python virtual environment recorded there. If that instance is missing or invalid, `export.ps1` will either reconcile it according to the saved preference or fail and ask you to run `.\install.ps1` again.

````{note}
If you have set a custom tool installation path according to the above instructions, then you **must** set the `SIFLI_SDK_TOOLS_PATH` variable before running the `export.ps1` script
```powershell
cd C:\OpenSiFli\SiFli-SDK
$env:SIFLI_SDK_TOOLS_PATH="D:\SIFLI\tools"
.\export.ps1
```
````

```{note}
Each time you open a new terminal window, you need to run the `export.ps1` script once in the SDK root directory to set environment variables. Note that this script must be run in the SDK root directory, otherwise it will cause execution failure or compilation errors.
```

```{note}
`export.ps1` now validates the resolved environment instance before exporting. If the local Python environment, tools, or Conan config do not match the current repo lock, `export.ps1` may prompt to reconcile the environment or fail deterministically in non-interactive shells.

`export.ps1` requires `uv` in PATH because it launches `tools/sdk_env.py` through `uv run`.
```

### Windows Terminal Quick Configuration

If you need to run SiFli-SDK frequently and want to automatically set environment variables each time you open the terminal, you can create a new Windows Terminal profile by following these steps:

Press `Ctrl+,` in Windows Terminal to open settings, click to add a new profile, select duplicate profile `Windows PowerShell`, then follow these steps:
![](image/Windows-T1.png)
1. Change the name to SiFli-SDK
2. Change the command line configuration to the following, change the final export.ps1 file location to your SDK path
```powershell
%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe  -ExecutionPolicy Bypass -NoExit -File  D:\SIFIL\SiFli-SDK\export.ps1
```
![](image/Windows-T2.png)

3. Change the starting directory to use parent process directory

![](image/Windows-T3.png)

4. Other configurations can remain unchanged
5. Click Save

Subsequently, you only need to open Windows Terminal in any code directory, click the dropdown menu in the upper right corner, select SiFli-SDK, and the environment variables will be set automatically. In the newly opened window, you can use the SDK's compilation and download commands.
![](image/Windows-T4.png)

### Check if Environment Setup is Successful
You can try compiling and downloading to see if it's successful. For compilation and downloading, please refer to [](../../build.md)
