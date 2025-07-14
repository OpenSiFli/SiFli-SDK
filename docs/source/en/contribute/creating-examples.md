
# Creating Example Projects

Each SiFli-SDK example is a complete project that others can copy to their local machine and modify according to their needs. Please note that example projects are primarily meant to showcase the functionality of SiFli-SDK.

## Example Project Structure

- The `src` directory must contain a source file named `main.c`, which includes the main functionality of the example project.
- If the example project has multiple sub-tasks, split them into multiple C or C++ source files under the `main` directory and place the corresponding header files in the same directory.
- The example project must include a `README.md` file. It is recommended to use the [Example Project README Template (English)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README.md) or the [Example Project README Template (Chinese)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README_CN.md) and modify it according to the project's actual situation.

## General Guidelines

The example code must adhere to the requirements in the [SiFli-SDK Coding Guidelines](coding_style.md). The code in the example project must be clear, readable, and maintainable.

## Checklist
Before submitting a new example project, ensure the following:

- Each example project should focus on one main feature. If an example project has multiple main features, it should be split into two or more projects.
- Each example project must include a `README.md` file. It is recommended to use the [Example Project README Template (English)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README.md) or the [Example Project README Template (Chinese)](https://github.com/OpenSiFli/SiFli-SDK/blob/main/docs/TEMPLATE_EXAMPLE_README_CN.md).
- Functions and variable names in the example project should follow the rules outlined in the [SiFli-SDK Coding Guidelines](coding_style.md). Non-static variables/functions that are used only within the example project's source file should be prefixed with `example` or another similar prefix.
- All code in the example project should be well-structured, with key sections clearly commented.
- Any unnecessary code in the example project (such as old debug logs, commented-out code, etc.) should be removed.
- Options used in the example project (such as network names, addresses, etc.) must not be hard-coded. Configuration options, or defined macros/constants, should be used wherever possible.
- All source code must specify licensing information at the beginning of the file (indicating that the code is `in the public domain CC0`) and include a disclaimer. Alternatively, the source code can apply the `Apache License 2.0`. Please refer to the licensing information and disclaimers of existing example projects and modify them as necessary.
- Any third-party code (whether used directly or with modifications) must retain the original licensing information, and the licensing must be compatible with the `Apache License 2.0` agreement.
