# 安装环境

## 安装准备

为了安装SiFli-SDK，需要根据操作系统安装一些软件包。可以参考以下安装指南，安装 Linux 和 macOS 的系统上所有需要的软件包。

```{important}
对 Windows 用户来说，只需要保证环境变量中存在 `Python` 环境变量即可。

如果没有安装 Python，请参考 [Python 官网](https://www.python.org/downloads/) 下载并安装 Python 3.9 以上版本。安装完成后，确保将 Python 添加到系统的环境变量中。
```

::::::{tab-set}
:sync-group: os

:::::{tab-item} Linux
:sync: Linux

::::{tab-set}
:sync-group: linux

:::{tab-item} Ubuntu 和 Debian

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

:::

:::{tab-item} CentOS 7 & 8

```bash
sudo yum -y update && sudo yum install git wget flex bison gperf python3 python3-setuptools cmake ninja-build ccache dfu-util libusbx
```

:::

:::{tab-item} Arch

```bash
sudo pacman -S --needed gcc git make flex bison gperf python cmake ninja ccache dfu-util libusb python-pip
```

:::

::::

:::::

:::::{tab-item} macOS
:sync: macOS

SiFli-SDK 将使用 macOS 上默认安装的 Python 版本。

- 安装 CMake 和 Ninja 编译工具：
  - Homebrew 用户：

        ```bash
        brew install cmake ninja
        ```

  - MacPort 用户

        ```bash
        sudo port install cmake ninja
        ```

  - 都不是
        若以上均不适用，请访问 CMake 和 Ninja 主页，查询有关 macOS 平台的下载安装问题。

:::{note}
如在上述任何步骤中遇到以下错误:

```
xcrun: error: invalid active developer path (/Library/Developer/CommandLineTools), missing xcrun at: /Library/Developer/CommandLineTools/usr/bin/xcrun
```

则必须安装 XCode 命令行工具，可运行 `xcode-select --install` 命令进行安装。
:::

:::::

::::::

## 获取 SiFli-SDK

在围绕 SF32 构建应用程序之前，请先获取 SiFli 提供的软件库文件 [SiFli-SDK 仓库](https://github.com/OpenSiFli/SiFli-SDK)。

获取 SiFli-SDK 的本地副本：打开终端，切换到要保存 SiFli-SDK 的工作目录，使用 git clone 命令克隆远程仓库。针对不同操作系统的详细步骤，请见下文。

打开终端，运行以下命令：

::::{tab-set}
:sync-group: os

:::{tab-item} Windows

```powershell
mkdir -p C:\OpenSiFli
cd C:\OpenSiFli
git clone --recursive https://github.com/OpenSiFli/SiFli-SDK
```

:::

:::{tab-item} Linux & macOS

```bash
mkdir -p ~/OpenSiFli
cd ~/OpenSiFli
git clone --recursive https://github.com/OpenSiFli/SiFli-SDK
```

:::

::::

```{note}
上面的SDK路径仅做示例，用户可以根据自己的需要选择路径。

如果在国内访问 GitHub 较慢，可以使用以下命令替换上面的命令：
git clone --recursive https://gitee.com/SiFli/sifli-sdk
```

## 安装工具

除了 SiFli-SDK 本身，还需要为支持 SF32 的项目安装 SiFli-SDK 使用的各种工具，比如编译器、调试器、Python 包等。

:::: {tab-set}
:sync-group: os

:::{tab-item} Windows

```powershell
cd C:\OpenSiFli\SiFli-SDK
.\install.ps1
国内用户可以使用如下命令通过国内镜像源安装工具包
cd C:\OpenSiFli\SiFli-SDK
set SIFLI_SDK_GITHUB_ASSETS="downloads.sifli.com/github_assets"
./install.ps1
```

:::

:::{tab-item} Linux & macOS

```bash
cd ~/OpenSiFli/SiFli-SDK
./install.sh
国内用户可以使用如下命令通过国内镜像源安装工具包
cd ~/OpenSiFli/SiFli-SDK
export SIFLI_SDK_GITHUB_ASSETS="downloads.sifli.com/github_assets"
./install.sh
```

:::

::::

## 设置环境变量

通过以上步骤，SDK和相关工具就安装好了，但是他们的路径并不在环境变量里，没办法在任意目录使用。因此，必须设置一些环境变量。这可以通过 SiFli-SDK 提供的另一个脚本进行设置。

请在需要使用编译或下载命令的终端窗口运行以下命令：
::::{tab-set}
:sync-group: os

:::{tab-item} Windows

```powershell
export.ps1
```

如果需要经常运行 SiFli-SDK，并且希望在每次打开终端时自动设置环境变量，可以新建一个 PowerShell 配置文件，具体步骤如下：
在 PowerShell中按下 `Ctrl+,` 打开设置，点击添加新的配置文件，选择复制配置文件 `Windows PowerShell`，然后按照以下步骤进行操作：

1. 将名称改为SiFli-SDK
2. 把命令行的配置改为如下,最后的export.ps1文件位置改成你的SDK路径
```powershell
%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe  -ExecutionPolicy Bypass -NoExit -File  D:\SIFIL\SiFli-SDK\export.ps1
```
3. 启动目录改为使用父进程目录
4. 其他配置可以不改动
5. 点击保存

后续只需要在任意代码目录下打开PowerShell，点击右上角的下拉菜单，选择SiFli-SDK，就可以自动设置环境变量了。在新打开的窗口中就可以使用SDK的编译和下载命令了。

:::

:::{tab-item} Linux & macOS

```bash
source export.sh
```

如果需要经常运行 SiFli-SDK，可以为执行 export.sh 创建一个别名，具体步骤如下：

1. 复制并粘贴以下命令到 shell 配置文件中（.profile、.bashrc、.zprofile 等）

```bash
alias get_sf32='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. 通过重启终端窗口或运行 `source [path to profile]`，如 `source ~/.bashrc` 来刷新配置文件

现在可以在任何终端窗口中运行 get_sf32 来设置或刷新 SiFli-SDK 环境。

不建议直接将 export.sh 添加到 shell 的配置文件。这样做会导致在每个终端会话中都激活 SDK 虚拟环境（包括无需使用 SiFli-SDK 的会话）。这违背了使用虚拟环境的目的，还可能影响其他软件的使用。
:::

::::
