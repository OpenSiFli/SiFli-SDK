# 创建示例项目

每个 SiFli-SDK 的示例都是一个完整的项目，其他人可以将示例复制至本地，并根据实际情况进行一定修改。请注意，示例项目主要是为了展示 SiFli-SDK 的功能。

## 示例项目结构

- `src` 目录需要包含一个名为 `main.c` 的源文件，里面包含示例项目的主要功能。
- 如果该示例项目的子任务比较多，请根据逻辑将其拆分为 `main` 目录下的多个 C 或者 C++ 源文件，并将对应的头文件也放在同一目录下。
- 示例项目需要包含一个 ``README.md`` 文件，建议使用 [示例项目README模板 (英文)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README.md)或者[示例项目README模板 (中文)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README_CN.md)，并根据项目实际情况进行修改。

## 一般准则

示例代码需要遵循 [SiFli-SDK 编码规范](coding_style.md) 中的要求。示例项目的代码需要清晰易读，且具有良好的可维护性。

## 检查清单
提交一个新的示例项目之前，需要检查以下内容：

- 每个示例项目只能有一个主要功能。如果某个示例项目有多个主要功能，请将其拆分为两个或更多示例项目。
- 每个示例项目应包含一个 `README.md` 文件，建议使用 [示例项目README模板 (英文)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README.md)或者[示例项目README模板 (中文)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README_CN.md)。
- 示例项目中的函数和变量的命令要遵循 [SiFli-SDK 编码规范](coding_style.md) 中的要求。对于仅在示例项目源文件中使用的非静态变量/函数，请使用 ``example`` 或其他类似的前缀。
- 示例项目中的所有代码结构良好，关键代码要有详细注释。
- 示例项目中所有不必要的代码（旧的调试日志，注释掉的代码等）都必须清除掉。
- 示例项目中使用的选项（比如网络名称，地址等）不得直接硬编码，应尽可能地使用配置项，或者定义为宏或常量。
- 所有的源代码都需要在文件开头指定许可信息（表示该代码是 `in the public domain CC0`）和免责声明。或者，源代码也可以应用 `Apache License 2.0` 许可条款。请查看现有示例项目的许可信息和免责声明，并根据实际情况进行修改。
- 任何第三方代码（无论是直接使用，还是进行了一些改进）均应保留原始代码中的许可信息，且这些代码的许可必须兼容 `Apache License 2.0` 协议。