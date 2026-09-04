# 内置模块可选外置依赖（模块级依赖聚合）

本文说明 SDK 内置模块如何就地声明"仅当自己参与编译时才需要拉取"的外置组件，以及该机制如何随工程一起安装/构建生效。既作为设计说明，也作为模块作者的写作与约定参考。

## 背景与动机

`sf-pkg`（`tools/sdk_py_actions/sf_pkg_ext.py`）是 Conan 2 的封装：
- 工程根目录 `conanfile.py` 里声明外置依赖（`sf-pkg init` 生成）；
- `sf-pkg install` 等价于 `conan install . --output-folder=sf-pkgs --deployer=full_deploy`，把包源码以及生成的 `sf-pkgs/Kconfig.conandeps`、`sf-pkgs/SConscript_conandeps` 落盘；
- 构建侧（`tools/build/building.py`）只消费 `sf-pkgs/` 里已部署的内容（`AddExternalComponents`、Kconfig 的 `osource`），本身不触发 conan。

痛点：SDK 内置模块（如 `middleware/*`、`customer/peripherals/*`）自己需要的外置组件，只能在工程根 `conanfile.py` 集中声明，导致即使模块没参与编译也会被拉取/暴露，并且项目开发者需要代管 SDK 模块的依赖。

## 目标与约束

1. 解析方式：**单图聚合**——各模块依赖汇总到一张 consumer conanfile，仍只做一次 `conan install`，由 Conan 统一版本解析与去重。
2. 参与判定：**配置静态解析**——读取本次构建已解析的 `.config`，而非在 SCons 运行时判定。
3. 粒度：**随模块使能符号**——某模块的使能符号为 `y` 时，其依赖才并入拉取集合。
4. 编译门控：**模块自身 Kconfig `select` 包符号**——拉取层只保证包源码进 `sf-pkgs/`，包是否编入由模块 Kconfig 决定，两者解耦。
5. 触发点：**构建入口自动触发**（board 已知、`.config` 已解析），独立的 `sf-pkg install` 保留并支持 `--board`。

## 工程根声明改 YAML

工程根的依赖声明由 `conanfile.py` 改为 `sf-pkg.yaml`：

```yaml
# 工程根固定依赖
requires:
  - sht30/0.0.4@caisong123
support_sdk_version: "^2.4"   # 可选：工程要求的 SDK 区间（npm semver），缺省不检查
```

- 选择 YAML 而非 JSON：可写注释、无尾逗号/引号这类易错语法，工程开发者与模块作者都可读可改。
- 与模块清单共用 schema（`requires` / `support_sdk_version` 字段一致）；工程根没有 `enable`，视为工程级固定依赖。
- Conan 本身不认 YAML：`sf-pkg` 在拉取前把“工程根 requires + 命中模块 requires”合并，生成一份标准 consumer conanfile——基座由官方 `conan new sf-pkg-project` 模板产出（随 conan config bundle 一起更新），SDK 仅把 requires 列表注入其中，落于 `.sf-pkg/conanfile.py`（gitignore），再对它执行 `conan install`。
- 好处：聚合降级为纯数据合并，不再需要“import/继承用户任意 python recipe”；`conanfile.py` 的用户入口消失，不会与手写 recipe 打架。
- 逃生口：若个别工程确需 python 级高级能力（条件依赖、`tool_requires`、复杂 conf），保留“高级模式”——工程根存在 `conanfile.py` 且无 `sf-pkg.yaml` 时，`sf-pkg install` 直接对它执行 `conan install`（旧行为），模块级依赖不参与。`sf-pkg init` 只生成 `sf-pkg.yaml`；老工程没有该文件即自动回落到旧行为，无破坏性迁移。

## 模块依赖清单

每个需要外置依赖的 SDK 内置模块，就地放一份 `sf-pkg.yaml`，**与定义该模块使能符号的 Kconfig 放在同一目录**：

```yaml
# 本模块参与编译时所需的 sf-pkg 外置组件
enable:
  - SF32LB5XX_MOD_ENABLE   # Kconfig 使能符号，任一为 y 即视为参与
requires:
  - mesh-lib/1.3.0@acme    # conan 引用串，可写版本区间
  - crypto/2.0.1@sifli
support_sdk_version: "^2.4"   # 可选：本模块要求的 SDK 区间（npm semver），缺省不检查
```

字段说明：

- `enable`：本模块的 Kconfig 使能符号。数组内任一为 `y` 即视为参与；缺失视为“总是参与”，仅用于确实无开关的固定模块。
- `requires`：与工程根相同的 Conan 包引用格式。
- `support_sdk_version`：npm 风格 semver 区间（如 `^2.4`、`~2.4.1`、`>=2.4,<3`）。语义与包 recipe 的类属性 `support_sdk_version`、以及基类 `sf-pkg-base.SourceOnlyBase.validate()` 完全一致：用 `semantic_version.NpmSpec` 解析，版本取环境变量 `SIFLI_SDK_VERSION`（由 `export.ps1` / `set_env.bat` 设置，形如 `2.4` / `2.4.1`）。

### 候选模块发现：基于 `kconfiglist`，不做全盘扫描

每次构建 `InitBuild` 都会把本次 board 实际解析过的全部 Kconfig 文件（绝对路径、去重、按 chip/board 配置树门控）写入 `build_dir/kconfiglist`。因此：

> 候选清单 = 遍历 `kconfiglist` 中每个文件所在目录，检查是否存在同级的 `sf-pkg.yaml`。

- 开销为 O(本次解析过的 Kconfig 文件数)，无递归、无全盘遍历。
- 不属于当前 chip/board 树的模块（其它芯片族、未 source 子树、docs/tools 等）根本不会出现在 `kconfiglist` 中，天然对齐 SConscript/Kconfig 树。
- 若某类模块的使能符号定义在共享/上层聚合 Kconfig（如 `drivers/hal/Kconfig` 集中定义多个开关、源码在各子目录），只需在该聚合 Kconfig 所在目录放一份清单，`enable` 列出其下各符号即可覆盖，不需要扫描源码树。

### SDK 版本检查

不依赖旧的 conan hook / `required_sdk_version` 机制，而是在 SDK 侧**复刻** `SourceOnlyBase.validate()` 的校验逻辑：

1. 取环境变量 `SIFLI_SDK_VERSION`；存在任一 `support_sdk_version` 声明但环境变量缺失、或版本不是合法 semver，直接报错。
2. `semantic_version.Version.coerce(sdk_ver)` 解析实际版本。
3. `semantic_version.NpmSpec(range)` 解析声明区间，`sdk_version not in spec` 即报错，指明来源文件与要求/实际版本。

工程根与**每个启用模块**的 `support_sdk_version` 都在 SDK 侧统一校验，先于任何网络 / install 动作。各**包**自身的 `support_sdk_version` 仍由它们经 `sf-pkg-base.SourceOnlyBase.validate()` 在 Conan 解析时自行校验，本层不重复。

## 数据流

```
scons --board=<b>
  └─ PrepareEnv
      ├─ LoadRtconfig / InitBuild(#1)          生成 build_dir/{.config,rtconfig.h,kconfiglist}
      ├─ BuildOptionUpdate(BuildOptions)
      ├─ EnsureSfPkgDepsForBoard(b)
      │    ├─ manifests_from_kconfiglist(kconfiglist)        → 候选模块清单
      │    ├─ load_root_manifest(project)                    → 工程根 requires
      │    ├─ collect_requires(启用模块)                     → 模块 requires
      │    ├─ check_sdk_versions(...)                        → 失败即报错
      │    ├─ fingerprint(merged_requires, sdk_ver)
      │    ├─ 与 .sf-pkg/installed.fingerprint 相同? → 直接返回
      │    └─ 不同：
      │         清空 project/sf-pkgs/                        避免残留包被编入
      │         generate_consumer_conanfile() → .sf-pkg/conanfile.py
      │         conan install <consumer>                     落 sf-pkgs/
      │         写 .sf-pkg/installed.fingerprint
      │         InitBuild(#2) + BuildOptionUpdate            包 Kconfig 就位、模块 select 生效
      └─ 正常 SConscript 构建（读 sf-pkgs/SConscript_conandeps）
```

- 指纹包含 SDK 版本：SDK 升级即使依赖区间未变也会触发一次重解析/重装，避免 Conan 区间解析结果陈旧。
- 无 `sf-pkg.yaml` 且无模块命中、或为“高级模式”时完全跳过，保持离线与向后兼容。
- 提供 `SIFLI_SF_PKG_OFFLINE=1`（或等价开关）跳过自动拉取，便于 CI / 离线。

## 模块 Kconfig 编译门控（模块作者约定）

拉取层只保证包源码在 `sf-pkgs/` 里；包是否真正编入，由模块自己的 Kconfig 在其启用时 `select` 该包在 `Kconfig.conandeps` 中声明的使能符号（符号名是包作者契约，模块作者按包文档填写）。Kconfig 树里模块符号在前、包符号由 `osource sf-pkgs/Kconfig.conandeps` 在后，kconfiglib 按符号名建表，后置定义会合入前置引用，`select` 可跨文件生效。

## 交付物

- `tools/sdk_py_actions/sf_pkg_deps.py`：核心收集/聚合/consumer 生成/校验逻辑，同时被 `building.py` 与 `sf_pkg_ext.py` 复用。
- `tools/build/building.py`：`PrepareEnv` 中插入 `EnsureSfPkgDepsForBoard`，必要时二次 `InitBuild`。
- `tools/sdk_py_actions/sf_pkg_ext.py`：`init` 生成 `sf-pkg.yaml`；`install` 支持 `--board`；抽出共享 conan 命令。
- 工程根 `sf-pkg.yaml` 模板（`sf-pkg init` 生成）。
- 使用示例：若干确有外置依赖的模块加 `sf-pkg.yaml`。
- `.gitignore` 增加 `.sf-pkg/`。
- 本文档对应的用户指南。

## 验证

1. 无外置依赖的常规工程构建行为不变（无 `sf-pkg.yaml` / 聚合为空 → 跳过）。
2. `sf-pkg init` 生成 `sf-pkg.yaml`；`sf-pkg install` 在 YAML 模式下正常落 `sf-pkgs/`。
3. 最小试验：某工程内放一个测试用模块清单，requires 指向 registry 上一个已知小包 → 构建后确认 `sf-pkgs` 出现该包、`Kconfig.conandeps` / `SConscript_conandeps` 生成、模块 Kconfig `select` 的包符号为 `y`。
4. 关闭该模块使能符号再构建 → 指纹变化触发重装，该包从 `sf-pkgs` 移除，不参与链接。
5. 无网络 / `SIFLI_SF_PKG_OFFLINE=1` → 构建正常、跳过拉取。
6. 遗留工程（只有 `conanfile.py`）走高级模式，行为与旧版一致。
7. SDK 版本检查：工程根或启用模块声明不被当前 `SIFLI_SDK_VERSION` 满足的 `support_sdk_version` → 在联网前报错并指明来源文件。
