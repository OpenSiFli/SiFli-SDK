# Watch Interface

Using LVGL v8, the included interfaces are:

## Specifying Fonts
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb525
+ eh-lb523

## Overview
<!-- 例程简介 -->
This example demonstrates the usage of dynamic modules based on the RT-Thread
dynamic library framework. In this demonstration, the application code (CODE)
and resources are decoupled and packaged into separate shared objects (.so
files):
+ `module_name_res.so`: Resource component
+ `module_name.so`: Code component

Upon execution, the example dynamic module creates a demonstration page.

## Directory Structure

```c
|--module                           -- Dynamic module project directory
    |--module_name.h                -- Header file for the dynamic module name (customizable)
    |--src
    |    |--app                     -- Source code for the dynamic module (generates module_name.so)
              |--app_demo.c         -- Initializes the module and creates a page
              |--SConscript
    |    |--resource                -- Resource directory for the dynamic module (generates module_name_res.so)
              |--font_bitmap        -- Bitmap fonts
              |--images             -- Image assets
              |--strings            -- Multi-language translation files
              |--SConscript
    |--rtconfig.py
    |--SConstruct
|--project                          -- Firmware project directory (host environment for the dynamic module)
|--src                              -- Firmware source directory
|--disk                             -- Pre-mounted files (used to pre-index .so files into the root partition)
|--README.md                        -- Documentation (Chinese)
|--README_EN.md                     -- Documentation (English)
```

## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Prerequisites:
+ A supported development board ([Supported Platforms](quick_start)).

### menuconfig Configuration
Firmware side:
1. Enable `RT_USING_MODULE`:\
   ![RT_USING_MODULE](./assets/conf_dlmodule.png)

    ```{tip}
    + `use custom memory interface`: Allows overriding the default memory management interface for dynamic modules. To customize this, locate and modify `RT_MODULE_MEM_CUSTOM`.
    ```

2. Enable `USING_MOD_INSTALLER`:\
   ![USING_MOD_INSTALLER](./assets/conf_mod_installer.png)
3. As dynamic modules rely on a file system, ensure that file system support is
   enabled.
4. `RT_NAME_MAX` Configuration:\
   ![11](./assets/config_rt_name_max.png)

    ```{warning}
    If the allocated configuration is too small, long module names may cause identification failures, leading to system exceptions.
    ```

### Compilation and Flashing
Dynamic modules only support compilation using the `GCC` toolchain.
1. Compiling the Firmware: Navigate to the sample `project` directory and run
   the scons command:
```c
&gt; scons --board=eh-lb525 -j8
```
2. Generate dynamic module dependency files:
```c
&gt; scons --board=eh-lb525 --target=ua -s
```
3. Navigate to the `module` directory to compile the module:
```c
&gt; scons --board=eh-lb525 -j8
```

```{tip}
+ Dependency files are copied from the firmware directory before compilation:  
    ![USING_MOD_INSTALLER](./assets/ua_files_copy.png)
+ Upon successful compilation, the module .so and _res.so files are generated in /module/output/.
```


4. Packaging .so files into the file system partition:\
   Place the module .so and _res.so files into the /disk/app directory, then
   recompile the firmware.
5. Flashing after compilation: Navigate to the sample `project/build_xx`
   directory, run `uart_download.bat`, and select the appropriate port as
   prompted to start the download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download procedures, please refer to the [Quick
Start](quick_start) guide.

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the sample starts, use the following commands to test the dynamic module:
| Purpose              | Command                                        | Example                                              |
| -------------------- | ---------------------------------------------- | ---------------------------------------------------- |
| Open dynamic module  | mod_open [module_res.so_path] [module.so_path] | `mod_open "/app/mod_demo_res.so" "/app/mod_demo.so"` |
| Close dynamic module | mod_close [module_name]                        | `mod_close "mod_demo"`                               |

Example:
```c
/* 1. Run the sample module */
08-11 18:58:14:025 TX:mod_open /app/mod_demo_res.so /app/mod_demo.so
08-11 18:58:14:044    mod_open argc=3
08-11 18:58:14:048    /app/mod_demo_res.so
08-11 18:58:14:053    dlmodule_find name=/app/mod_demo_res.so
08-11 18:58:14:064    dlmodule_load /app/mod_demo_res.so fd=4
08-11 18:58:14:069    dlmodule_load_shared_object: invalid mod_demo_res, 0x6060429c, 44316
08-11 18:58:14:078    dlmodule_load_shared_object: clean mod_demo_res, 0x6060429c, 44316
08-11 18:58:14:083    /app/mod_demo.so
08-11 18:58:14:089    dlmodule_find name=/app/mod_demo.so
08-11 18:58:14:098    dlmodule_load /app/mod_demo.so fd=4
08-11 18:58:14:102    dlmodule_load_shared_object: invalid mod_demo, 0x605f9d5c, 2204
08-11 18:58:14:111    dlmodule_load_shared_object: clean mod_demo, 0x605f9d5c, 2204
08-11 18:58:14:117    match lang
/* 2. Module executed successfully; creating [demo_p] page */ 
08-11 18:58:14:122    app_init_func create demo page.
08-11 18:58:14:130    [20537952] D/APP.FWK tshell: send msg[GUI_APP_MSG_OPEN_PAGE] [0x2003f868] to gui_app_mbx tick:629133.
08-11 18:58:14:135    msh /&gt;msh /&gt;[20538378] D/APP.SCHE app_watch: ----------------app_schedule_task---------------start
08-11 18:58:14:144    [20538410] I/APP.SCHE app_watch: &gt;&gt;Execute msg[GUI_APP_MSG_OPEN_PAGE] tick:629133
08-11 18:58:14:150    [20538440] D/APP.SCHE app_watch: app[Main] create page[demo_p] 2003e134
```

## Troubleshooting

1. Missing EXPORT for .so dependencies:
```c
08-11 10:04:12:999    /app/mod_demo.so
08-11 10:04:13:003    dlmodule_find name=/app/mod_demo.so
08-11 10:04:13:007    dlmodule_load /app/mod_demo.so fd=4
08-11 10:04:13:017    dlmodule_load_shared_object: invalid mod_demo, 0x605f9d30, 2160
08-11 10:04:13:020    [3253182] E/DLMD tshell: Module: can't find lv_ext_set_local_font_bitmap in kernel symbol table
08-11 10:04:13:024    [3253218] E/DLMD tshell: Module: can't find img_red_heart in kernel symbol table
08-11 10:04:13:032    [3253249] E/DLMD tshell: Module: can't find lv_font_montserrat_28_compressed in kernel symbol table
08-11 10:04:13:037    [3253286] E/DLMD tshell: Module: can't find lv_i18n_lang_pack in kernel symbol table
08-11 10:04:13:043    app_open open /app/mod_demo.so failed.
```
Solution: Use EXPORT to expose the required interfaces on the firmware side, for
example:
```c
RTM_EXPORT(lv_ext_set_local_font_bitmap);
... ...
```

2. Insufficient memory:
```c
dlmodule_load length=500000 malloc fail.
```
Memory required for module execution is allocated via `dlm_malloc`. The error
above indicates an allocation failure. To resolve this, either reduce the size
of the .so file, increase the memory pool available for modules, or enable
`RT_MODULE_MEM_CUSTOM` to redefine the module memory interface.


## References
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

1. [RT-Thread Documentation Center - Dynamic
   Modules](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/libc/posix/dlmodule)

## Revision History
| Version | Date     | Release Notes   |
| ------- | -------- | --------------- |
| 0.0.1   | May 2025 | Initial release |
|         |          |                 |
|         |          |                 |
