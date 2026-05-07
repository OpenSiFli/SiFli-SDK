# macOS 或 Linux 安装流程

我们推荐使用 [CodeKit](https://marketplace.visualstudio.com/items?itemName=SiFli.sifli-sdk-codekit) VSCode 插件来安装 SiFli-SDK 和相关工具。

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

当前的 SiFli-SDK 安装流程不再依赖系统 Python。`install.sh` 会通过 `uv` 准备锁定的 Python 运行时和依赖。

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

## 安装 `uv`

当前 install/export 主链路只支持通过 `uv` 引导。请先安装 `uv`，并确保终端中可以正常执行：

```bash
uv --version
```

```{note}
`uv` 是一个用Rust编写的、速度极快的Python包和项目管理工具。安装方法可以参考 [uv 官方文档](https://docs.astral.sh/uv/getting-started/installation)。
```

## 获取 SiFli-SDK

在围绕 SF32 构建应用程序之前，请先获取 SiFli 提供的软件库文件 [SiFli-SDK 仓库](https://github.com/OpenSiFli/SiFli-SDK)。

获取 SiFli-SDK 的本地副本：打开终端，切换到要保存 SiFli-SDK 的工作目录，使用 `git clone` 命令克隆远程仓库。一般来说，我们建议使用release分支上的代码以获取最新的稳定版本。

```{warning}

由于SiFli-SDK中包含子模块，不能通过下载zip包获取完整的代码。

```

打开 PowerShell 终端，运行以下命令：

```powershell
mkdir -p C:\OpenSiFli
cd C:\OpenSiFli
git clone --recursive -b release/v2.4 https://github.com/OpenSiFli/SiFli-SDK
```

````{note}
上面的SDK路径仅做示例，用户可以根据自己的需要选择路径。

如果在国内访问 GitHub 较慢，可以使用 `gitee` 镜像来克隆 SiFli-SDK。请使用以下命令：
```powershell
git clone --recursive -b release/v2.4 https://gitee.com/SiFli/sifli-sdk
```

需要注意，gitee的SiFli-SDK仓库的路径是全小写的，在后续出现`SiFli-SDK`时需要注意大小写。
````

````{note}
如果想要切换到其他分支（例如开发分支），可以使用 `checkout` 命令，例如：


```powershell
git checkout main
```
或者
```powershell
git checkout release/v2.3
```

````

````{note}
需要注意的是，SiFli-SDK中存在一些子模块，因此需要使用 `--recursive` 参数来克隆所有子模块。如果你在克隆时忘记了这个参数，可以在克隆后运行以下命令来初始化子模块：

```bash
git submodule update --init --recursive
```
````

## 安装工具

除了 SiFli-SDK 本身，还需要为支持 SF32 的项目安装 SiFli-SDK 使用的各种工具，比如编译器、调试器、Python 包等。

```bash
cd ~/OpenSiFli/SiFli-SDK
./install.sh
```

`install.sh` 会自动完成以下工作：

- 通过 `uv` 准备锁定的 Python 运行时
- 根据 `tools/locks/default/pyproject.toml` 和 `tools/locks/default/uv.lock` 同步锁定的 Python 依赖
- 根据 `tools/locks/default/lock.json` 安装当前 profile 绑定的工具版本
- 根据当前 lock 快照在 `SIFLI_SDK_TOOLS_PATH` 下实例化当前 profile 对应的环境
- 在 `SIFLI_SDK_TOOLS_PATH` 下初始化该环境实例对应的 Conan 环境

Keil/ARMCLANG 路径记录和 `export -t keil` 仅支持 Windows；macOS 和 Linux 的脚本默认导出 GCC 工具链。

对于国内用户，可以通过 `SIFLI_SDK_MIRROR_CHINA` 一键启用国内镜像预设：

```bash
cd ~/OpenSiFli/SiFli-SDK
export SIFLI_SDK_MIRROR_CHINA=1
./install.sh
```

该预设开启后会强制覆盖以下环境变量：

- `SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"`
- `SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"`
- `UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"`
- `UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"`

如果不想使用整组预设，也可以继续手工设置这些细粒度变量。

### 自定义工具安装路径（可选）

本步骤中介绍的脚本将 SiFli-SDK 所需的编译工具默认安装在用户的根目录中，即 *inx 系统中的 `$HOME/.sifli` 目录，或者是windows下的`C:\Users\<name>\.sifli`。我们可以选择将工具安装到其他目录中，但请在运行安装脚本前，导出环境变量 `SIFLI_SDK_TOOLS_PATH`。注意，请确保用户账号已经具备了读写该路径的权限。

```powershell
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
./install.sh

. ./export.sh
```

如果修改了 `SIFLI_SDK_TOOLS_PATH` 变量，请在运行任意 SiFli-SDK 工具或脚本前，将该变量导出到环境变量中。

```{note}
如未导出环境变量，大多数 shell 将不支持在变量赋值中使用 `SIFLI_SDK_TOOLS_PATH`，例如 `SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path" ./install.sh`。因为即便在源脚本中导出或修改了该变量，当前的执行环境也不受变量赋值影响。
```

## 设置环境变量

通过以上步骤，SDK和相关工具就安装好了，但是他们的路径并不在环境变量里，没办法在任意目录使用。因此，必须设置一些环境变量。这可以通过 SiFli-SDK 提供的另一个脚本进行设置。

请在需要使用编译或下载命令的终端窗口运行以下命令：

```bash
. export.sh
```

`export.sh` 现在会通过 `uv run` 调用 `tools/sdk_env.py export`。环境管理器会根据当前 `profile + lock` 快照解析目标 SDK 环境实例；如果该实例不存在或已损坏，`export.sh` 会按已保存的偏好自动修复，或者直接失败并提示重新执行 `./install.sh`。

````{note}
如果按照上述说明设置过自定义工具安装路径，那么在运行 `export.sh` 脚本之前**必须**设置`SIFLI_SDK_TOOLS_PATH` 变量
```powershell
cd C:\OpenSiFli\SiFli-SDK
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
. export.sh
```
````

```{note}
`export.sh` 现在会在导出环境前检查当前解析出来的环境实例是否仍与仓库锁文件一致。如果本地 Python 环境、工具版本或 Conan 配置不匹配，交互式终端可能会提示修复；非交互场景下会直接以确定性错误退出。

`export.sh` 需要 PATH 中存在 `uv`，因为它会通过 `uv run` 启动 `tools/sdk_env.py`。
```

如果需要经常运行 SiFli-SDK，可以为执行 export.sh 创建一个别名，具体步骤如下：

1. 复制并粘贴以下命令到 shell 配置文件中（.profile、.bashrc、.zprofile 等）

```bash
alias sf32sdk='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. 通过重启终端窗口或运行 `source [path to profile]`，如 `source ~/.bashrc` 来刷新配置文件

现在可以在任何终端窗口中运行 `sf32sdk` 来设置或刷新 SiFli-SDK 环境。

不建议直接将 export.sh 添加到 shell 的配置文件。这样做会导致在每个终端会话中都激活 SDK 虚拟环境（包括无需使用 SiFli-SDK 的会话）。这违背了使用虚拟环境的目的，还可能影响其他软件的使用。
