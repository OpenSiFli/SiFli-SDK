# 许可证和版权检查器

该脚本根据提供的配置文件以及 `scancode-toolkit` 的 JSON 输出，分析被扫描文件的许可证与版权信息，并生成违规报告。

## 概述

[`license_check.py`](license_check.py:1) 读取 [`license_config.yml`](license_config.yml:1) 的规则，并解析 `scancode-toolkit` 的 JSON 结果：

- 许可证检查：
  - 缺失许可证（可选）
  - 无效许可证（不在允许列表中，可选）
- 版权检查：
  - 缺失版权声明（可选）

脚本会输出一份包含所有违规项的报告；若存在违规则以退出码 `1` 结束，否则以 `0` 结束。

## 依赖项

- Python 3
- [`scancode-toolkit`](https://github.com/nexB/scancode-toolkit)
- PyYAML（`pip install pyyaml`）

## 配置文件

脚本行为通过 YAML 文件（例如 [`license_config.yml`](license_config.yml:1)）配置。

示例：

```yaml
license:
  main: Apache-2.0
  additional: 
    - MIT
  report_invalid: true
  report_unknown: true
  report_missing: true
copyright:
  check: true
exclude:
  extensions:
    - yml
    - yaml
    - html
    - rst
    - conf
    - cfg
    - config
    - bat
    - md
  special_file:
    - SConscript
    - SConstruct
    - ChangeLog
    - Kconfig
    - rtconfig.h
    - config.h
  langs:
    - HTML
extensions:
  - path:
      "external/ffmpeg"
    license:
      - LGPL-2.1-only
      - LGPL-2.1-or-later
      - LGPL-3.0-only
      - LGPL-3.0-or-later
  - path:
      "external/faad2"
    license:
      - GPL-2.0-only
      - GPL-2.0-or-later
  - path:
      "filesystems/dhara/dhara"
    ignore_license: true
    ignore_copyright: true
```

### 配置选项

- `license`
  - `main`：主要允许的 SPDX 许可证标识符（例如 `Apache-2.0`）。
  - `additional`：额外允许的 SPDX 许可证标识符列表。
  - `report_invalid`：是否报告“检测到的许可证不在允许列表中”的文件。
  - `report_missing`：是否报告“未检测到许可证”的文件。
  - `report_unknown`：保留字段，当前脚本未使用该开关。

- `copyright`
  - `check`：是否启用缺失版权声明检查。

- `exclude`
  - `extensions`：按扩展名排除（不含点号，例如 `yml`）。
  - `special_file`：按文件名排除（完整文件名匹配，例如 `Kconfig`、`config.h`）。
  - `langs`：按 `scancode-toolkit` 识别的编程语言排除（例如 `HTML`）。

- `extensions`
  - 用于为特定路径配置“例外规则”。每条规则包含：
    - `path`：路径子串匹配（例如 `external/ffmpeg`）。
    - `license`：该路径下允许的 SPDX 许可证列表（可选）。
    - `ignore_license`：是否忽略该路径下的许可证检查（可选）。
    - `ignore_copyright`：是否忽略该路径下的版权检查（可选）。

## 检查逻辑说明

脚本核心逻辑在 [`analyze_file()`]：

- 仅对 `scancode-toolkit` 结果中被判定为“需要检查”的文件执行规则校验：
  - 如果文件扩展名 / 编程语言 / 文件名命中 `exclude` 则跳过。
  - 若路径包含 `Kconfig` 且 `file_type` 为 `ASCII text`，强制纳入检查。
  - 其余情况下，当 `scancode-toolkit` 将文件标记为 `is_script` 或 `is_source` 时纳入检查。

- 许可证允许列表：
  - 全局允许列表 = `license.additional + license.main`，并额外允许 `unknown-license-reference`（脚本默认加入，用于规避某些包含“license”字样导致的误报）。
  - 若文件路径命中 `extensions` 的某条规则：
    - 当该规则提供 `license` 时，使用该列表作为该路径的允许许可列表。
    - 当该规则未提供 `license` 时，回退到全局允许列表。
    - 若设置了 `ignore_license: true`，则跳过该路径下的许可证检查与缺失许可证检查。
  - 当 `scancode-toolkit` 返回组合表达式（例如 `A AND B AND C`）：脚本不会严格按 SPDX 逻辑求值，而是采用“反向包含”策略——只要该长字符串中包含任意一个允许的许可证 ID，即判定为 PASS。

- 版权缺失检查：
  - 仅在 `copyright.check: true` 时启用。
  - 若文件未检测到版权且其 `programming_language` 不是 `CMake`，则报告缺失版权。
  - 命中 `extensions` 的规则且设置 `ignore_copyright: true` 时，跳过该路径下的版权检查。

## 使用方法

1. 使用 `scancode-toolkit` 扫描项目（示例）：

   ```bash
   scancode -clipeu --license --license-text --license-text-diagnostics --classify --summary --verbose /path/to/your/project --processes 4 --json "scancode.json" --html "scancode.html"
   ```

   说明：脚本依赖 `scancode.json` 中的 `files[*].license_detections`、`files[*].copyrights`、`files[*].is_source/is_script` 等字段。

2. 运行许可证/版权检查脚本：

   ```bash
   python license_check.py \
       -c license_config.yml \
       -s scancode.json \
       -f /path/to/your/project \
       -o license_report.txt
   ```

### 命令行参数

- `-c, --config_file`：YAML 配置文件路径。
- `-s, --scancode-output`：`scancode-toolkit` 生成的 JSON 输出文件路径。
- `-f, --scanned_files`：被扫描的项目目录路径（用于从 `scancode.json` 的 `path` 中剥离前缀）。
- `-o, --output_file`：违规报告输出文件路径。

## 输出

- 若发现违规项：
  - 将违规详情写入 `-o/--output_file` 指定的文件
  - 打印 `Copyright check FAILED.`
  - 以退出码 `1` 结束

- 若未发现违规项：
  - 打印 `Copyright check PASSED.`
  - 以退出码 `0` 结束
