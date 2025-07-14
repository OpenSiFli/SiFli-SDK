
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

In addition to using the SDK’s built-in board configurations, you can use `--board_search_path` to specify a directory as a search path for third-party boards. This directory can be outside the SDK, and it can be either a relative or absolute path. When the search path is specified, the compiler will not only look for boards in the SDK’s board directory but will also check this directory for board configurations. If a board with the same name exists in both directories, the board in the `--board_search_path` specified directory will be used. For example, to compile using a relative path for the board search path in the `app1` project directory, you can run the following command:

```shell
scons --board=test_board --board_search_path=../../boards -j8
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

The SDK uses `menuconfig`, a graphical interface tool from the kconfiglib package, to manage project settings. During compilation, all macro switches are read from `rtconfig.h`, which directs SCons on which modules to compile and their corresponding parameters. The corresponding Kconfig configuration is stored in `.config`. To address the issue mentioned earlier, the generic project directory no longer contains `rtconfig.h` and `.config`. Instead, these files are dynamically generated in the build directory during compilation based on the selected board. The generated `.config` is a merged result of Kconfig’s default values, `board.conf`, and `proj.conf`. Both `board.conf` and `proj.conf` contain modifications compared to the default values, with `proj.conf` taking precedence if the same configuration appears in both files.

If you wish to modify `proj.conf`, you can execute `menuconfig --board=<board_name>` in the project directory. If the board name does not have the `_hcpu` suffix, it defaults to configuring for the HCPU. To use the LCPU configuration, the board name should include the `_lcpu` suffix, such as `sf32lb52-lcd_n16r8_lcpu`. The parameter settings displayed in the menuconfig interface are the same as those used during actual compilation. After making modifications, press {kbd}`D` to save the minimal configuration to `proj.conf`. If you want to modify `board.conf`, you need to switch to the board directory and execute `menuconfig` (without any parameters). For example, switch to the `boards/sf32lb52-lcd_n16r8/hcpu` directory and run `menuconfig`.

```{note}
If the configurations stored in `proj.conf` do not apply to all boards, you can create a subdirectory for the board in the project directory. Place the specific `proj.conf` for that board in this subdirectory for differentiated configuration. For more details, refer to [common_project.md](app_note/common_project.md).
```

For further reading, refer to [common_project.md](app_note/common_project.md).
