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

SiFli-SDK no longer relies on the system Python installation. `install.sh` uses `uv` to prepare the SDK-managed Python runtime and dependencies.

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

- install the Python runtime and dependencies required by the SDK
- install the compiler, debugger, and other tools that match the current SDK version
- prepare the Conan environment used for build dependencies
- save SDK environment information under `SIFLI_SDK_TOOLS_PATH` for later environment export

An SDK environment is the Python runtime, toolchain, debug tools, and build dependencies prepared for the current SDK. For the first install, run `./install.sh`.

After updating the SDK, plain `./install.sh` prepares or switches to the matching environment for the updated SDK. If an older environment is currently selected, it is not modified in place, so older SDK checkouts can keep using it.

If you explicitly want to update the currently selected environment in place, run:

```bash
./install.sh update
```

If that environment is also used by another SDK checkout, the script creates a new environment instead so the other checkout is not affected.

Keil/ARMCLANG path recording and `export -t keil` are Windows-only; the macOS and Linux scripts export the GCC toolchain by default.

### Clean Old Environments and Cache (Optional)

After SDK updates, you can remove old environments that are no longer needed:

```bash
./install.sh uninstall
```

This command deletes old environments for the current profile immediately, while keeping the environment that matches the current SDK and any environment still selected by another SDK checkout. To preview the cleanup without deleting anything, run:

```bash
./install.sh uninstall --dry-run
```

To clean old environments for all profiles when they are not selected by any SDK checkout, run:

```bash
./install.sh uninstall --all
```

You can preview the all-profile cleanup first:

```bash
./install.sh uninstall --all --dry-run
```

To remove every SDK-managed environment, including the current environment and environments still selected in the state file, run:

```bash
./install.sh uninstall --all --force
```

`--all --force` makes every SDK environment invalid and clears environment selections in the state file. Run `./install.sh` again before exporting. “Still in use” is based on the state file under `SIFLI_SDK_TOOLS_PATH`; open shells or running processes are not detected.

To also clean download cache files and leftover staging directories, add `--cache`:

```bash
./install.sh uninstall --cache
```

`--cache` keeps tool archives and the Conan config package required by the current SDK, and it does not remove installed tool directories. If you use a custom `SIFLI_SDK_TOOLS_PATH`, export the same variable before running cleanup.

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

`export.sh` switches to the SDK environment installed for the current checkout. If that environment is missing or invalid, the script prompts according to the saved preference. Choosing to update is equivalent to running `./install.sh update`. If you decline, you can run plain `./install.sh` to install a new environment for the current SDK, or run `./install.sh update` to try updating the currently selected old environment.

````{note}
If you have set a custom tool installation path according to the above instructions, then you **must** set the `SIFLI_SDK_TOOLS_PATH` variable before running the `export.sh` script
```powershell
cd C:\OpenSiFli\SiFli-SDK
export SIFLI_SDK_TOOLS_PATH="$HOME/required_sdk_tools_path"
. export.sh
```
````

```{note}
`export.sh` validates the current SDK environment before exporting. If the local Python environment, tools, or Conan config do not match the current SDK, `export.sh` may prompt to repair the environment or fail deterministically in non-interactive shells.

`export.sh` requires `uv` in PATH to run the SDK environment management flow.
```

### Offline or Intranet Use

If your computer cannot access the internet but you have successfully run `./install.sh`, you can activate the local environment offline as long as it remains complete and compatible with the current SDK. Updating SDK code does not necessarily require reinstalling the environment. This depends on the environment compatibility check, not just whether the compiler version is unchanged.

Before disconnecting, complete the installation and successfully run `export.sh` once using the same user account. Keep the SDK tools directory (default: `$HOME/.sifli`), the `uv` cache, and the Python runtime managed by `uv`. Also prepare any external build dependencies required by your project.

In a terminal without internet access or with intranet-only access, run:

```bash
cd ~/OpenSiFli/SiFli-SDK
export UV_OFFLINE=1
. ./export.sh --offline
```

If you use a custom tools directory, first export the same `SIFLI_SDK_TOOLS_PATH` used during installation.

`UV_OFFLINE=1` prevents the outer `uv` bootstrap process from accessing the network and restricts it to local cached data and files. `--offline` enables offline mode for SDK environment management. Once activation succeeds, enter your project directory and run the build commands in the same terminal.

```{warning}
Offline mode does not supply missing dependencies. `sifli-sdk-env.json` (located under `$HOME/.sifli` by default) records installed environment information. Its presence does not guarantee that the Python runtime, toolchain, and caches are still complete.

If Python cannot be found, cached packages such as `rich` or `tomli_w` are missing, or the environment is incompatible, prepare the required dependencies in an environment with internet access. Adding only `--offline` does not prevent network access during the outer `uv` bootstrap process, so also set `UV_OFFLINE=1`.

The commands above are not sufficient for a first-time installation on a completely disconnected machine. Intranet machines need all installation dependencies prepared in advance or accessible internal dependency sources. Switching to a public mirror does not solve a lack of internet access.
```

After restoring internet access, allow `uv` to download or update dependencies again by clearing the offline setting in the current terminal:

```bash
unset UV_OFFLINE
```

### Set Up an Alias (Optional)

If you need to run SiFli-SDK frequently, you can create an alias for executing export.sh by following these steps:

1. Copy and paste the following command into your shell configuration file (.profile, .bashrc, .zprofile, etc.)

```bash
alias sf32sdk='. $HOME/OpenSiFli/SiFli-SDK/export.sh'
```

2. Refresh the configuration file by restarting the terminal window or running `source [path to profile]`, such as `source ~/.bashrc`

Now you can run `sf32sdk` in any terminal window to set or refresh the SiFli-SDK environment.

It is not recommended to add export.sh directly to the shell's configuration file. This will cause the SDK virtual environment to be activated in every terminal session (including sessions that don't need to use SiFli-SDK). This goes against the purpose of using virtual environments and may affect the use of other software.
