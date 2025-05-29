# macOS 或 Linux 安装流程

## 安装准备

为了安装SiFli-SDK，需要根据操作系统安装一些软件包。可以参考以下安装指南，安装 Linux 和 macOS 的系统上所有需要的软件包。

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

获取 SiFli-SDK 的本地副本：打开终端，切换到要保存 SiFli-SDK 的工作目录，使用 git clone 命令克隆远程仓库。

打开终端，运行以下命令：

```bash
mkdir -p ~/OpenSiFli
cd ~/OpenSiFli
git clone --recursive https://github.com/OpenSiFli/SiFli-SDK
```

````{note}
需要注意的是，SiFli-SDK中存在一些子模块，因此需要使用 `--recursive` 参数来克隆所有子模块。如果你在克隆时忘记了这个参数，可以在克隆后运行以下命令来初始化子模块：

```bash
git submodule update --init --recursive
```

````

````{note}
上面的SDK路径仅做示例，用户可以根据自己的需要选择路径。

如果在国内访问 GitHub 较慢，可以使用我们的gitee镜像：
```bash
git clone --recursive https://gitee.com/SiFli/sifli-sdk
```
````

## 安装工具

除了 SiFli-SDK 本身，还需要为支持 SF32 的项目安装 SiFli-SDK 使用的各种工具，比如编译器、调试器、Python 包等。

```bash
cd ~/OpenSiFli/SiFli-SDK
./install.sh
```

对于国内用户来说，可以使用如下命令来添加国内镜像源：

```bash
cd ~/OpenSiFli/SiFli-SDK
export SIFLI_SDK_GITHUB_ASSETS="downloads.sifli.com/github_assets"
export PIP_INDEX_URL="https://mirrors.ustc.edu.cn/pypi/simple"
./install.sh
```

## 设置环境变量

通过以上步骤，SDK和相关工具就安装好了，但是他们的路径并不在环境变量里，没办法在任意目录使用。因此，必须设置一些环境变量。这可以通过 SiFli-SDK 提供的另一个脚本进行设置。

请在需要使用编译或下载命令的终端窗口运行以下命令：

```bash
. export.sh
```

```{note}
目前的脚本可能有一些偶现的bug，如果在编译的时候提示找不到`arm-none-eabi-gcc`等命令，可以尝试运行两次`. export.sh`解决。
```

如果需要经常运行 SiFli-SDK，可以为执行 export.sh 创建一个别名，具体步骤如下：

1. 复制并粘贴以下命令到 shell 配置文件中（.profile、.bashrc、.zprofile 等）

```bash
alias sf32sdk='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. 通过重启终端窗口或运行 `source [path to profile]`，如 `source ~/.bashrc` 来刷新配置文件

现在可以在任何终端窗口中运行 `sf32sdk` 来设置或刷新 SiFli-SDK 环境。

不建议直接将 export.sh 添加到 shell 的配置文件。这样做会导致在每个终端会话中都激活 SDK 虚拟环境（包括无需使用 SiFli-SDK 的会话）。这违背了使用虚拟环境的目的，还可能影响其他软件的使用。
