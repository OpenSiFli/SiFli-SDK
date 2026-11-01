# macOS 或 Linux 安装流程

我们推荐使用 [CodeKit](https://marketplace.visualstudio.com/items?itemName=SiFli.sifli-sdk-codekit) VSCode 插件来安装 SiFli-SDK 和相关工具。

## 安装准备

为了安装SiFli-SDK，需要根据操作系统安装一些软件包。macOS 和 Linux 的安装流程不再依赖系统 Python，但需要先安装 `uv`。可以参考以下安装指南，安装 Linux 和 macOS 的系统上所有需要的软件包。

::::::{tab-set}
:sync-group: os

:::::{tab-item} Linux
:sync: Linux

::::{tab-set}
:sync-group: linux

:::{tab-item} Ubuntu 和 Debian

```bash
sudo apt-get install git wget flex bison gperf cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

:::

:::{tab-item} CentOS 7 & 8

```bash
sudo yum -y update && sudo yum install git wget flex bison gperf cmake ninja-build ccache dfu-util libusbx
```

:::

:::{tab-item} Arch

```bash
sudo pacman -S --needed gcc git make flex bison gperf cmake ninja ccache dfu-util libusb
```

:::

::::

:::::

:::::{tab-item} macOS
:sync: macOS

当前的 SiFli-SDK 安装流程不再依赖系统 Python。`install.sh` 会通过 `uv` 准备 SDK 管理的 Python 运行时和依赖。

- 安装 CMake、Ninja 编译工具和 `uv`：
  - Homebrew 用户：

        ```bash
        brew install cmake ninja uv
        ```

  - MacPort 用户

        ```bash
        sudo port install cmake ninja uv
        ```

  - 都不是
        若以上均不适用，请访问 CMake、Ninja 和 uv 主页，查询有关 macOS 平台的下载安装问题。

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
curl -LsSf https://astral.sh/uv/install.sh | sh
```

如果系统中没有 `curl`，也可以使用 `wget`：

```bash
wget -qO- https://astral.sh/uv/install.sh | sh
```

安装完成后，重新打开终端，或按安装脚本的提示刷新当前 shell 配置，然后检查版本：

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

- 安装 SDK 运行需要的 Python 环境和依赖
- 安装当前 SDK 版本匹配的编译器、调试器等工具
- 准备构建依赖使用的 Conan 环境
- 在 `SIFLI_SDK_TOOLS_PATH` 下保存 SDK 环境信息，后续导出环境变量时会继续使用

这里的 SDK 环境指一套可供当前 SDK 使用的 Python、工具链、调试工具和构建依赖。首次安装请直接运行 `./install.sh`。

SDK 更新后，普通 `./install.sh` 会为更新后的 SDK 准备或切换到匹配的环境；如果当前已经选中了旧环境，它不会直接修改旧环境。这样旧版本 SDK 仍可继续使用。

如果你确认要在当前已选中的环境上原地更新，可以运行：

```bash
./install.sh update
```

如果这个环境也被其他 SDK 工作目录使用，脚本会改为新建一个环境，避免影响其他 SDK。

Keil/ARMCLANG 路径记录和 `export -t keil` 仅支持 Windows；macOS 和 Linux 的脚本默认导出 GCC 工具链。

### 清理旧环境和缓存（可选）

如果 SDK 更新后不再需要旧环境，可以运行：

```bash
./install.sh uninstall
```

该命令会直接删除当前 profile 下不再被任何 SDK 工作目录使用的旧环境，并保留当前 SDK 匹配的环境和其他工作目录仍在使用的环境。若只想预览将删除的内容，可以运行：

```bash
./install.sh uninstall --dry-run
```

如果要清理所有 profile 下不再被任何 SDK 工作目录选中的旧环境，可以运行：

```bash
./install.sh uninstall --all
```

也可以先预览：

```bash
./install.sh uninstall --all --dry-run
```

如需删除所有由 SDK 管理的环境，包括当前环境和仍在状态文件中选中的环境，可以运行：

```bash
./install.sh uninstall --all --force
```

`--all --force` 会让所有 SDK 环境失效，并清空状态文件中的环境选择；之后需要重新运行 `./install.sh`。这里的“仍在使用”按 `SIFLI_SDK_TOOLS_PATH` 下的状态文件判断，不检测已经打开的 shell 或进程。

如果还需要清理下载缓存和残留的 staging 临时目录，可以增加 `--cache`：

```bash
./install.sh uninstall --cache
```

`--cache` 会保留当前 SDK 仍需要的工具归档和 Conan 配置包，不会删除已经安装好的工具目录。若使用了自定义 `SIFLI_SDK_TOOLS_PATH`，清理前也必须先导出同一个环境变量。

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

`export.sh` 会切换到当前 SDK 已安装的环境；如果这个环境不存在或已损坏，脚本会按已保存的偏好提示是否更新。选择更新等价于运行 `./install.sh update`；选择不更新时，可以手动运行普通 `./install.sh` 为当前 SDK 安装新环境，或运行 `./install.sh update` 尝试更新当前已选中的旧环境。

````{note}
如果按照上述说明设置过自定义工具安装路径，那么在运行 `export.sh` 脚本之前**必须**设置`SIFLI_SDK_TOOLS_PATH` 变量
```powershell
cd C:\OpenSiFli\SiFli-SDK
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
. export.sh
```
````

```{note}
`export.sh` 会在导出环境前检查当前 SDK 环境是否仍然完整。如果本地 Python 环境、工具版本或 Conan 配置不匹配，交互式终端可能会提示修复；非交互场景下会直接以确定性错误退出。

`export.sh` 需要 PATH 中存在 `uv`，以便运行 SDK 环境管理流程。
```

### 断网或内网环境下使用

如果电脑无法访问外网，但已经成功运行过 `./install.sh`，且当前 SDK 所需的环境仍然完整、兼容，可以使用本地环境离线激活。SDK 代码更新本身不一定需要重新安装环境，是否需要更新取决于环境兼容性检查结果，而不只是编译器版本是否相同。

建议在断网前，使用同一个用户完成安装并成功运行一次 `export.sh`，保留 SDK 工具目录（默认是 `$HOME/.sifli`）、`uv` 缓存和 `uv` 管理的 Python 运行时。如果项目还需要下载外部构建依赖，也应提前准备好。

在断网或只能访问内网的终端中，运行：

```bash
cd ~/OpenSiFli/SiFli-SDK
export UV_OFFLINE=1
. ./export.sh --offline
```

如果使用了自定义工具安装路径，请先导出安装时使用的 `SIFLI_SDK_TOOLS_PATH`。

`UV_OFFLINE=1` 禁止外层 `uv` 启动流程访问网络，只使用本地缓存和文件；`--offline` 让 SDK 环境管理流程使用离线模式。激活成功后，可以在同一个终端中进入工程目录，正常执行编译命令。

```{warning}
离线模式不会自动补齐缺失的依赖。`sifli-sdk-env.json`（默认位于 `$HOME/.sifli`）记录的是已安装环境的信息，文件存在并不代表 Python、工具链和缓存仍然完整。

如果提示找不到 Python、缺少 `rich` 或 `tomli_w` 缓存，或者环境不兼容，应在可联网环境中重新准备所需依赖。仅添加 `--offline` 不能解决外层 `uv` 启动时的联网问题，因此需要同时设置 `UV_OFFLINE=1`。

首次安装不能仅靠上述命令在完全断网环境中完成。内网机器需要提前准备完整的安装依赖，或使用可访问的内网依赖源；切换到公网国内镜像并不能解决完全断网的问题。
```

恢复联网后，如需允许 `uv` 下载或更新依赖，可以在当前终端中取消离线设置：

```bash
unset UV_OFFLINE
```

### 设置快捷命令（可选）

如果需要经常运行 SiFli-SDK，可以为执行 export.sh 创建一个别名，具体步骤如下：

1. 复制并粘贴以下命令到 shell 配置文件中（.profile、.bashrc、.zprofile 等）

```bash
alias sf32sdk='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. 通过重启终端窗口或运行 `source [path to profile]`，如 `source ~/.bashrc` 来刷新配置文件

现在可以在任何终端窗口中运行 `sf32sdk` 来设置或刷新 SiFli-SDK 环境。

不建议直接将 export.sh 添加到 shell 的配置文件。这样做会导致在每个终端会话中都激活 SDK 虚拟环境（包括无需使用 SiFli-SDK 的会话）。这违背了使用虚拟环境的目的，还可能影响其他软件的使用。
