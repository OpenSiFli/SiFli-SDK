# 在工程中使用 SiFli 组件注册表

以下步骤说明如何在现有工程中安装和使用 SiFli 组件注册表 依赖。

```{note}
当前页面中的部分截图仍来自旧版命令界面；若截图中的命令格式与正文不同，请以当前 `sdk.py sf-pkg ...` 命令为准。
```

## 初始化依赖（sf-pkg init）

```bash
sdk.py sf-pkg init
```

执行成功后，会在 `project` 目录下生成一份 `sf-pkg.yaml` 声明文件（YAML 格式，支持注释），例如：

```yaml
# 工程级固定依赖：每一项为 <包名>/<版本号>@<用户名>，版本可写区间
requires:
  - sht30/0.0.4@caisong123
  # - battery_calculator/[^1.2]@sifli

# 可选：工程要求的 SDK 区间（npm semver，如 ^2.4）
# support_sdk_version: "^2.4"
```

`requires` 为空即表示不依赖任何包；需要时按 `<包名>/<版本号>@<用户名>` 格式添加即可，例如添加 SHT30 传感器驱动包：`sht30/0.0.4@caisong123`。

```{note}
早期版本通过 `conanfile.py` 声明依赖。若工程根存在 `conanfile.py` 且没有 `sf-pkg.yaml`，`sf-pkg install` 会视为"高级模式"，直接对它执行 `conan install`（保持旧行为，不参与模块级依赖聚合）。建议新工程使用 `sf-pkg.yaml`。
```

### sf-pkg.yaml 字段说明

`sf-pkg.yaml` 同时用于工程根与 SDK 内置模块，字段说明如下：

- `requires`（列表，可选）：本工程/本模块需要的外置组件。每一项格式为 `<包名>/<版本号>@<用户名>`，版本可写区间，例如 `sht30/0.0.4@caisong123`、`battery_calculator/[^1.2]@sifli`。为空或不写等价于无依赖。
- `support_sdk_version`（字符串，可选）：声明要求的 SDK 版本区间，使用 npm 风格 semver（如 `^2.4`、`~2.4.1`、`>=2.4,<3`）；不写则不检查。安装/构建时会用环境变量 `SIFLI_SDK_VERSION` 校验，不满足即在联网前报错并指明来源文件。工程根与每个模块各自声明、分别校验。
- `enable`（列表，可选，**仅模块清单使用**）：模块的 Kconfig 使能符号，列表中**任一**为 `y` 即视为"本模块参与编译"，此时它的 `requires` 才会并入拉取集合；全部不满足则不拉取。这里填 Kconfig 中定义的**真实符号名**（`.config` 行首的 `CONFIG_` 是 kconfig 写文件时自动加的，不算符号名的一部分；若符号名本身就以 `CONFIG_` 开头，则如实填写）。若省略/为空则视为"总是参与"，仅适合确实没有开关的固定模块。

工程根 `sf-pkg.yaml` 中 `enable` 不生效——工程根是固定依赖，`sf-pkg.yaml` 只需写 `requires`（和可选的 `support_sdk_version`）。

### 模块级外置依赖（模块清单）

SDK 内置模块（如 `middleware/*`、板级驱动等）也可以各自携带一份 `sf-pkg.yaml`，声明"仅当本模块参与编译时才需要拉取"的外置组件：

```yaml
# middleware/<module>/sf-pkg.yaml
# 必须放在"定义该模块使能符号的 Kconfig"同目录
enable:
  - BLE_STACK            # Kconfig 使能符号，任一为 y 即参与
requires:
  - mesh-lib/1.3.0@acme  # conan 引用，格式同工程根
support_sdk_version: "^2.4"   # 可选，同工程根语义
```

- 这份清单必须与其模块的 Kconfig 放在**同一目录**，且该 Kconfig 要在当前板子的配置树内被解析到，否则清单不会被发现（不参与就不会拉取）。
- 模块启用时，除拉取外还要让包参与链接：在模块自己的 Kconfig 里对包在 `Kconfig.conandeps` 中声明的使能符号做 `select`。
- 更完整的机制与约定见 [模块级依赖设计文档](design/module_deps.md)。

## 搜索可用的包

如果不确定包名或版本号，可以搜索：

```bash
sdk.py sf-pkg search <package_name>
```

示例：

```bash
sdk.py sf-pkg search sht30
```
也可以直接在组件注册表官网进行搜索：点此访问 [SiFli组件注册表](https://packages.sifli.com/)


## 安装依赖（sf-pkg install）

在工程的 `project` 目录下执行：

```bash
sdk.py sf-pkg install
```

- 不带 `--board`：只按工程根 `sf-pkg.yaml` 的 `requires` 安装，不合并模块级依赖。
- 带 `--board <board>`：额外把该板配置中"启用模块"声明的外置组件合并安装；会先自动按 board.conf + proj.conf 解析该板配置（生成构建目录内的 `kconfiglist`/`.config`），无需先编译。
- 带 `--board-search-path <dir>`：在 SDK `customer/boards` 之外附加板子搜索目录（同 scons `--board_search_path`；也可用环境变量 `SIFLI_SDK_BOARD_SEARCH_PATH`）。

![安装依赖](./assets/sf-pkg-install.png)

安装成功后，会在 `project` 目录下生成 `sf-pkgs` 文件夹，其中包含了所安装的包。

### 构建时自动拉取

只要工程根存在 `sf-pkg.yaml`，执行 `scons --board=<b>` 时构建入口会自动检测"本次板配置下启用模块"的依赖集合是否有变化，有变化则自动安装后再继续编译（幂等，未变化会跳过）。需要关闭自动拉取时：

```bash
SIFLI_SF_PKG_OFFLINE=1 scons --board=<b>     # 或使用 scons --no-sf-pkg
```

## 使用驱动

安装完成后即可直接使用驱动：

- 可以直接编译
- include 头文件时无需填写绝对路径，Conan 会自动处理路径配置

### Kconfig 配置注意事项

- `menuconfig` 会自动整合 `sf-pkgs` 文件夹下的所有 `Kconfig` 文件
- 这些配置项会出现在 `menuconfig` 的 **SiFli External Components** 菜单中
