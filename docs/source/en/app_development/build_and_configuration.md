
# Configuration and Compilation

Through [create_application.md](create_application.md) and [create_board.md](create_board.md), you have already gained an intuitive understanding of the application and the board. This section further explains the relationship and operational mechanism between the two.

A traditional RT-Thread project directory corresponds to an application, but it is already linked to a specific board. The `rtconfig.h` in the directory defines the complete configuration parameters for the project. To run this application on another board, you would need to create a new project directory based on the corresponding board’s BSP package and port the application code and related configurations over. This process is cumbersome and prone to errors. The SiFli-SDK follows Zephyr's approach of separating the board (hardware runtime environment) from the application. As long as the board provides the necessary hardware capabilities for the application, it becomes easy to compile the target files for any board. This type of application project that can compile for any target board is called a **generic project**. The compilation method for a specified target board is as follows:

```shell
scons --board=<board_name> -jN
```

Where `<board_name>` is the name of the board. The selection method is described in [supported_boards/index.md](../supported_boards/index.md), and `-jN` is a multi-thread compilation parameter, where N represents the number of threads. For example, the following command uses 8 threads to compile the target file for the board `sf32lb52-lcd_n16r8`:

```shell
scons --board=sf32lb52-lcd_n16r8 -j8
```

The existing `--target=<target_name>` parameter can also be combined with `--board`. For instance, to create the corresponding Keil project file for the `sf32lb52-lcd_n16r8` board, the following command can be executed:

```shell
scons --board=sf32lb52-lcd_n16r8 --target=mdk5 -s
```

```{note}
It should be noted that the SDK uses multi-project compilation. The application project is the main project, which will trigger the compilation of corresponding sub-projects, such as the secondary bootloader, ftab, and others. However, using `--target` will only generate the Keil project corresponding to the main project. Directly compiling using this project might cause issues, and it can only be used for code review.
```

## Export Codebase Index

If you want to export all source files and header files used by the current project for external analysis tools, run the following command in the project directory:

```shell
sdk.py export-codebase --board=<board_name>
```

If the target board has already been saved with `sdk.py set-target`, you can omit the `--board` parameter:

```shell
sdk.py export-codebase
```

This command internally invokes `scons --target=json` and generates `codebase_index.json` in the build directory, for example `build_sf32lb52-lcd_n16r8_hcpu/codebase_index.json`.

The exported JSON contains the following top-level fields:

- `system_construction`: currently fixed to `scons`
- `projects`: per-project file lists for the main project and its sub-projects
- `all_sources`: merged list of all source files
- `all_headers`: merged list of all header files
- `all_files`: merged list of all source and header files
- `all_include_paths`: merged list of include paths
- `all_defines`: merged list of preprocessor definitions

In addition to using the SDK’s built-in board configurations, you can use `--board_search_path` to specify a directory as a search path for third-party boards. This directory can be outside the SDK, and it can be either a relative or absolute path. When the search path is specified, the compiler will not only look for boards in the SDK’s board directory but will also check this directory for board configurations. If a board with the same name exists in both directories, the board in the `--board_search_path` specified directory will be used. For example, to compile using a relative path for the board search path in the `app1` project directory, you can run the following command:

```shell
scons --board=test_board --board_search_path=../../boards -j8
```

Of course, you can also use the `SIFLI_SDK_BOARD_SEARCH_PATH` environment variable to specify the search path, so you don't have to input the `--board_search_path` parameter every time.

```shell
export SIFLI_SDK_BOARD_SEARCH_PATH="../../boards" # unix
$env:SIFLI_SDK_BOARD_SEARCH_PATH="../../boards" # powershell
scons --board=test_board -j8
```

The directory structure of the code is as follows. The above command is executed in the `app1/project` directory, where the `test_board` is located in the `boards` directory. The `workspace` is an arbitrary working directory, which can be outside the SDK.

```
+--workspace
|
├─app1
│  ├─project
|  |
│  └─src
|
├─app2
│  ├─project
|  |
│  └─src
└─boards
    ├─test_board
    |
    └─test_board2
```

## Project Settings

The SDK uses menuconfig (a graphical interface tool within the kconfiglib package) to manage project settings. During compilation, it reads all macro switches from `rtconfig.h` to instruct SCons on which modules to compile and their parameters. The corresponding kconfig configuration is stored in `.config`. To address the previously mentioned issues, `rtconfig.h` and `.config` are no longer stored in the generic project directory. Instead, these files are dynamically generated in the build directory during compilation based on the selected board. The generated `.config` file is merged from Kconfig's default values, `board.conf`, and `proj.conf`. `board.conf` and `proj.conf` document the configurations requiring modification (those differing from default values). If identical configurations appear in both `board.conf` and `proj.conf`, the settings defined in `proj.conf` take precedence.

To modify `proj.conf`, execute `sdk.py menuconfig --board=<board_name>` in the project directory. If `<board_name>` lacks the `_hcpu` suffix, it defaults to HCPU configuration. To use LCPU settings, append `_lcpu` to the board name, e.g., `sf32lb52-lcd_n16r8_lcpu`. The parameter settings displayed in the menuconfig interface are identical to those used during actual compilation. After modifying settings, press {kbd}`D` to save the minimal configuration to `proj.conf`. To modify `board.conf`, switch to the board directory and execute `sdk.py menuconfig` (no any param). For example, navigate to `boards/sf32lb52-lcd_n16r8/hcpu` and run `sdk.py menuconfig`.

```{note}
If the configurations stored in `proj.conf` do not apply to all boards, you can create a subdirectory for the board in the project directory. Place the specific `proj.conf` for that board in this subdirectory for differentiated configuration. For more details, refer to [](../app_note/common_project.md).
```

For further reading, refer to [](../app_note/common_project.md).
