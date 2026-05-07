# Windows 安装流程

我们推荐使用 [CodeKit](https://marketplace.visualstudio.com/items?itemName=SiFli.sifli-sdk-codekit) VSCode 插件来安装 SiFli-SDK 和相关工具。

## 安装准备

### `uv` 环境

Windows 脚本主链路不再要求用户预装系统 Python。当前支持的方式是通过 `uv` 按需准备锁定的 Python 运行时。

安装好 `uv` 后，请在 PowerShell 中执行以下命令确认可用：

```powershell
uv --version
```

```{note}
`uv` 是一个用Rust编写的、速度极快的Python包和项目管理工具。安装方法可以参考 [uv 官方文档](https://docs.astral.sh/uv/getting-started/installation)。
```

### Git 环境

如果没有安装 Git，请参考 [Git 官网](https://git-scm.com/download/win) 下载并安装 Git。安装完成后，确保将 Git 添加到系统的环境变量中。

```{note}
对于国内用户来说，可以使用如下国内镜像链接下载 Git 安装包：<https://mirrors.huaweicloud.com/git-for-windows/v2.47.0.windows.1/Git-2.47.0-64-bit.exe>。每次均保持默认安装选项即可。
```

安装完成之后，可以在终端中运行`git --version`命令来检查 Git 是否安装成功。正常情况下，应该会输出 Git 的版本信息，例如：

```powershell
git version 2.47.0.windows.1
```

### 终端设置

SiFli-SDK 脚本安装仅支持`powershell`。

对于终端的选择，我们建议使用 [Windows Terminal](https://aka.ms/terminal)，用户也可以自行选择其他终端，例如VSCode自带的集成终端。但是更推荐使用 Windows Terminal。需要注意的是，在一些较新的 Windows 10/11 版本中，Windows Terminal 已经预装了。

想要打开`PowerShell`，可以使用如下方式：

- 按 Win键 或点击左下角Windows图标，输入 `powershell`，然后点击打开 PowerShell 终端。
- 按下 Win + R 组合键，打开运行窗口，输入 `powershell`，然后点击确定。

如果您使用的是 Windows Terminal，可以直接在终端中打开 PowerShell。想要打开终端，可以按 Win键 或点击左下角Windows图标，输入 `终端`，然后点击打开 Windows Terminal。

如果在接下来运行脚本的步骤中出现
`无法加载文件 C:\OpenSiFli\SiFli-SDK\export.ps1，因为在此系统上禁止运行脚本。` 的错误提示，或者你从未听说且从未运行过`.ps1`脚本，请使用 **管理员模式** 打开 PowerShell 终端，并运行以下命令：

```powershell
Set-ExecutionPolicy RemoteSigned
```

然后输入`Y`命令后，回车即可获得运行脚本的权限。

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

```powershell
cd C:\OpenSiFli\SiFli-SDK
.\install.ps1
```

`install.ps1` 会自动完成以下工作：

- 通过 `uv` 准备锁定的 Python 运行时
- 根据 `tools/locks/default/pyproject.toml` 和 `tools/locks/default/uv.lock` 同步锁定的 Python 依赖
- 根据 `tools/locks/default/lock.json` 安装当前 profile 绑定的工具版本
- 在 `SIFLI_SDK_TOOLS_PATH` 下初始化 profile 级别的 Conan 环境

如果需要使用 Keil/ARMCLANG 编译，请在安装时记录 Keil 根目录。该目录必须已经存在，并包含 `ARM\ARMCLANG\bin`：

```powershell
.\install.ps1 --keil C:\Keil_v5
```

已经安装过 SDK 环境的用户也可以直接重新运行上面的命令。`install.ps1` 是幂等的，会复用已有 Python、工具和 Conan 状态，并把 Keil 路径补录到 `${SIFLI_SDK_TOOLS_PATH}\sifli-sdk-env.json`。

````{note}
国内用户可以改用下面的命令一键启用国内镜像预设，避免默认源下载速度慢。注意，选择执行下述命令的时候不需要再执行上述代码块中的命令。

```powershell
cd C:\OpenSiFli\SiFli-SDK
$env:SIFLI_SDK_MIRROR_CHINA="1"
.\install.ps1
```

该预设开启后会强制覆盖以下环境变量：

- `SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"`
- `SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"`
- `UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"`
- `UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"`

如果不想使用整组预设，也可以继续手工设置这些细粒度变量。

````

### 自定义工具安装路径（可选）

本步骤中介绍的脚本将 SiFli-SDK 所需的编译工具默认安装在用户的根目录中，即 *inx 系统中的 `$HOME/.sifli` 目录，或者是windows下的`C:\Users\<name>\.sifli`。我们可以选择将工具安装到其他目录中，但请在运行安装脚本前，导出环境变量 `SIFLI_SDK_TOOLS_PATH`。注意，请确保用户账号已经具备了读写该路径的权限。

```powershell
$env:SIFLI_SDK_TOOLS_PATH="D:\SIFLI\tools"
.\install.ps1

.\export.ps1
```

如果修改了 `SIFLI_SDK_TOOLS_PATH` 变量，请在运行任意 SiFli-SDK 工具或脚本前，将该变量导出到环境变量中。

```{note}
对Windows用户来说，如果你的用户名中包含空格、汉字等非英文字符，则强烈建议设置`SIFLI_SDK_TOOLS_PATH`用来指定工具安装路径，否则可能会导致安装失败或编译错误。建议将该路径设置为纯英文路径，例如 `D:\SIFLI\tools`。
```

```{note}
如未导出环境变量，大多数 shell 将不支持在变量赋值中使用 `SIFLI_SDK_TOOLS_PATH`，例如 `SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path" ./install.ps1`。因为即便在源脚本中导出或修改了该变量，当前的执行环境也不受变量赋值影响。
```

## 设置环境变量

通过以上步骤，SDK和相关工具就安装好了，但是他们的路径并不在环境变量里，没办法在任意目录使用。因此，必须设置一些环境变量。这可以通过 SiFli-SDK 提供的另一个脚本进行设置。

请在需要使用编译或下载命令的终端窗口运行以下命令：

```powershell
cd C:\OpenSiFli\SiFli-SDK
.\export.ps1
```

默认不传 `-t` 时导出 GCC 环境，等价于：

```powershell
.\export.ps1 -t gcc
```

如果已经通过 `install.ps1 --keil` 记录过 Keil 根目录，可以切换到 Keil/ARMCLANG：

```powershell
.\export.ps1 -t keil
```

`export.ps1` 现在会通过 `uv run` 调用 `tools/sdk_env.py export`。环境管理器会根据当前 `profile + lock` 快照解析目标 SDK 环境实例，并使用其中记录的 Python 虚拟环境；如果该实例不存在或已损坏，`export.ps1` 会按已保存的偏好自动修复，或者直接失败并提示重新执行 `.\install.ps1`。

````{note}
如果按照上述说明设置过自定义工具安装路径，那么在运行 `export.ps1` 脚本之前**必须**设置`SIFLI_SDK_TOOLS_PATH` 变量
```powershell
cd C:\OpenSiFli\SiFli-SDK
$env:SIFLI_SDK_TOOLS_PATH="D:\SIFLI\tools"
.\export.ps1
```
````

```{note}
每次打开新的终端窗口都需要在SDK根目录下运行一次 `export.ps1` 脚本设置环境变量。注意，必须要在SDK根目录下运行该脚本，否则会导致运行失败或者编译错误。
```

```{note}
`export.ps1` 现在会在导出环境前检查当前解析出来的环境实例是否仍与仓库锁文件一致。如果本地 Python 环境、工具版本或 Conan 配置不匹配，交互式终端可能会提示修复；非交互场景下会直接以确定性错误退出。

`export.ps1` 需要 PATH 中存在 `uv`，因为它会通过 `uv run` 启动 `tools/sdk_env.py`。
```

### Windows Terminal 快捷配置

如果需要经常运行 SiFli-SDK，并且希望在每次打开终端时自动设置环境变量，可以新建一个 Windows Terminal 配置文件，具体步骤如下：

在 Windows Terminal 中按下 `Ctrl+,` 打开设置，点击添加新的配置文件，选择复制配置文件 `Windows PowerShell`，然后按照以下步骤进行操作：
![](image/Windows-T1.png)
1. 将名称改为SiFli-SDK
2. 把命令行的配置改为如下,最后的export.ps1文件位置改成你的SDK路径
```powershell
%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe  -ExecutionPolicy Bypass -NoExit -File  D:\SIFIL\SiFli-SDK\export.ps1
```
![](image/Windows-T2.png)

3. 启动目录改为使用父进程目录

![](image/Windows-T3.png)

4. 其他配置可以不改动
5. 点击保存

后续只需要在任意代码目录下打开Windows Terminal，点击右上角的下拉菜单，选择SiFli-SDK，就可以自动设置环境变量了。在新打开的窗口中就可以使用SDK的编译和下载命令了。
![](image/Windows-T4.png)

### 环境搭建是否成功
可以进行编译下载看看是否成功，编译下载可以参见[](../../build.md)
