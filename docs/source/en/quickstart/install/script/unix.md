# macOS or Linux Installation Process

We recommend using the [CodeKit](https://marketplace.visualstudio.com/items?itemName=SiFli.sifli-sdk-codekit) VSCode extension to install SiFli-SDK and related tools.

## Installation Prerequisites

To install SiFli-SDK, you need to install some software packages according to your operating system. You can refer to the following installation guide to install all required software packages on Linux and macOS systems.

::::::{tab-set}
:sync-group: os

:::::{tab-item} Linux
:sync: Linux

::::{tab-set}
:sync-group: linux

:::{tab-item} Ubuntu and Debian

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

SiFli-SDK no longer relies on the system Python installation. `install.sh` uses `uv` to prepare the locked Python runtime and dependencies for the active SDK profile.

- Install CMake and Ninja build tools:
  - Homebrew users:

        ```bash
        brew install cmake ninja
        ```

  - MacPort users

        ```bash
        sudo port install cmake ninja
        ```

  - Neither of the above
        If neither of the above applies, please visit the CMake and Ninja homepages to find download and installation information for the macOS platform.

:::{note}
If you encounter the following error in any of the above steps:

```
xcrun: error: invalid active developer path (/Library/Developer/CommandLineTools), missing xcrun at: /Library/Developer/CommandLineTools/usr/bin/xcrun
```

You must install XCode command line tools by running the `xcode-select --install` command.
:::

:::::

::::::

## Install `uv`

`uv` is the only supported bootstrap tool for the current install/export workflow. Please install `uv` first and ensure the following command works in your terminal:

```bash
uv --version
```

```{note}
`uv` is an extremely fast Python package and project management tool written in Rust. For installation instructions, refer to the [official uv documentation](https://docs.astral.sh/uv/getting-started/installation).
```

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

```bash
cd ~/OpenSiFli/SiFli-SDK
./install.sh
```

`install.sh` will:

- use `uv` to provision the locked Python runtime
- sync the locked Python dependency graph from `tools/locks/default/pyproject.toml` and `tools/locks/default/uv.lock`
- install the SDK toolchain versions bound by `tools/locks/default/lock.json`
- instantiate the current profile environment under `SIFLI_SDK_TOOLS_PATH` based on the active lock snapshot
- initialize the environment-specific Conan home under `SIFLI_SDK_TOOLS_PATH`

Keil/ARMCLANG path recording and `export -t keil` are Windows-only; the macOS and Linux scripts export the GCC toolchain by default.

For domestic users in China, you can enable the bundled China mirror preset with `SIFLI_SDK_MIRROR_CHINA`:

```bash
cd ~/OpenSiFli/SiFli-SDK
export SIFLI_SDK_MIRROR_CHINA=1
./install.sh
```

When enabled, this preset forcefully overrides the following environment variables:

- `SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"`
- `SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"`
- `UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"`
- `UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"`

If you do not want the bundled preset, you can still set the fine-grained mirror variables manually.

### Custom Tool Installation Path (Optional)

The script described in this step installs the compilation tools required by SiFli-SDK in the user's home directory by default, which is the `$HOME/.sifli` directory in *nix systems, or `C:\Users\<name>\.sifli` in Windows. We can choose to install tools to other directories, but please export the environment variable `SIFLI_SDK_TOOLS_PATH` before running the installation script. Note that please ensure the user account has read and write permissions for that path.

```powershell
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
./install.sh

. ./export.sh
```

If you modified the `SIFLI_SDK_TOOLS_PATH` variable, please export this variable to the environment variables before running any SiFli-SDK tools or scripts.

```{note}
If environment variables are not exported, most shells will not support using `SIFLI_SDK_TOOLS_PATH` in variable assignments, such as `SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path" ./install.sh`. Because even if the variable is exported or modified in the source script, the current execution environment is not affected by variable assignment.
```

## Set Environment Variables

After the above steps, the SDK and related tools are installed, but their paths are not in the environment variables, so they cannot be used in any directory. Therefore, some environment variables must be set. This can be done through another script provided by SiFli-SDK.

Please run the following command in terminal windows where you need to use compilation or download commands:

```bash
. export.sh
```

`export.sh` now invokes `tools/sdk_env.py export` through `uv run`. The environment manager resolves the current `profile + lock` snapshot to the matching SDK environment instance. If that instance is missing or invalid, `export.sh` will either reconcile it according to the saved preference or fail and ask you to run `./install.sh` again.

````{note}
If you have set a custom tool installation path according to the above instructions, then you **must** set the `SIFLI_SDK_TOOLS_PATH` variable before running the `export.sh` script
```powershell
cd C:\OpenSiFli\SiFli-SDK
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
. export.sh
```
````

```{note}
`export.sh` now validates the resolved environment instance before exporting. If the local Python environment, tools, or Conan config do not match the current repo lock, `export.sh` may prompt to reconcile the environment or fail deterministically in non-interactive shells.

`export.sh` requires `uv` in PATH because it launches `tools/sdk_env.py` through `uv run`.
```

If you need to run SiFli-SDK frequently, you can create an alias for executing export.sh by following these steps:

1. Copy and paste the following command into your shell configuration file (.profile, .bashrc, .zprofile, etc.)

```bash
alias sf32sdk='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. Refresh the configuration file by restarting the terminal window or running `source [path to profile]`, such as `source ~/.bashrc`

Now you can run `sf32sdk` in any terminal window to set or refresh the SiFli-SDK environment.

It is not recommended to add export.sh directly to the shell's configuration file. This will cause the SDK virtual environment to be activated in every terminal session (including sessions that don't need to use SiFli-SDK). This goes against the purpose of using virtual environments and may affect the use of other software.
