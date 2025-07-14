# Windows Installation Process

## Installation Prerequisites

### Python Environment

For Windows users, you need to ensure that the `Python` environment variable exists in the environment variables.

If Python is not installed, please refer to the [Python official website](https://www.python.org/downloads/) to download and install Python version 3.9 or above, below 3.14. After installation, make sure to add Python to the system's environment variables.

![](image/2025-05-26-13-39-17.png)

```{note}
For domestic users in China, you can use the following domestic mirror link to download Python installer: <https://mirrors.ustc.edu.cn/python/3.12.0/python-3.12.0.exe>
```

After installation, you can run the `python --version` command in the terminal to check if Python is installed successfully. Normally, it should output Python version information, such as:

```powershell
Python 3.12.0
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

SiFli-SDK script installation only supports `powershell`, and we recommend using `PowerShell 7` version. For specific installation procedures, please refer to the Microsoft documentation [PowerShell Installation](https://learn.microsoft.com/zh-cn/powershell/scripting/install/installing-powershell-on-windows?view=powershell-7.5).

```{note}
For domestic users in China, you can directly download PowerShell 7 installer from the following mirror link: <https://mirror4.lzu.edu.cn/github-release/PowerShell/PowerShell/v7.4.10%20Release%20of%20PowerShell/PowerShell-7.4.10-win-x64.msi>
```

For terminal choice, we recommend using [Windows Terminal](https://aka.ms/terminal). Users can also choose other terminals, such as the integrated terminal that comes with VSCode. However, Windows Terminal is more recommended. Note that in some newer Windows 10/11 versions, Windows Terminal is already pre-installed.

To open `PowerShell`, you can use the following methods:

- Press the Win key or click the Windows icon in the lower left corner, type `powershell`, then click to open PowerShell terminal.
- Press Win + R key combination to open the Run window, type `powershell`, then click OK.

If you are using Windows Terminal, you can directly open PowerShell in the terminal. To open the terminal, you can press the Win key or click the Windows icon in the lower left corner, type `terminal`, then click to open Windows Terminal.

````{warning}
Note that if you are using PowerShell 5, running the install script in Windows Terminal may result in errors, prompting that `PowerShell.exe` is not supported. In this case, please upgrade to PowerShell 7 or use the PowerShell native terminal (the one with blue background after opening).

To check PowerShell version, you can use the following command:

```powershell
$PSVersionTable.PSVersion
```

````

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

````{note}
Domestic users in China can use the following commands to install tool packages through domestic mirror sources to avoid slow download speeds from default sources. Note that if you choose to execute the following commands, you do not need to execute the commands in the above code block.

```powershell
cd C:\OpenSiFli\SiFli-SDK
$env:SIFLI_SDK_GITHUB_ASSETS="downloads.sifli.com/github_assets"
$env:PIP_INDEX_URL="https://mirrors.ustc.edu.cn/pypi/simple"
.\install.ps1
```

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
The current script may have some occasional bugs. If you get prompts like "command not found `arm-none-eabi-gcc`" during compilation, you can try running `. export.ps1` twice to resolve it.
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