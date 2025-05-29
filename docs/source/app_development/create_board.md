# 创建板子
创建新板子最快捷的方法是基于一块硬件配置相近的板子修改，下面以板子sf32lb52-lcd_n16r8（即sf32lb52-devkit-lcd）为例，演示如何创建一块板子，新板子与sf32lb52-lcd_n16r8同样使用了SF32LB525芯片，外置了NOR Flash，因此选择sf32lb52-lcd_n16r8会是一个比较好的起点。

操作步骤如下：
1. 在`boards`目录下创建文件夹`testboard_525`，将`sf32lb52-lcd_n16r8`下的文件拷贝到`testboard_525`下
1. 在`boards`目录下创建文件夹`testboard_525_base`，将`sf32lb52-devkit-lcd`下的文件拷贝到`testboard_525_base`下
1. 修改`testboard_525/SConscript`，把图中1和2的宏开关名字改为新板子，图中3引用的SConscript文件路径改为`testboard_525_base`
    ```{image} assets/create_board_sconscript.png
    ```
    修改后的代码如下
    ```{image} assets/create_board_sconscript_new.png
    ```
  
1. 修改`testboard_525/hcpu/Kconfig.board`和`testboard_525/lcpu/Kconfig.board`中板子的宏开关，与上一个步骤中的名称保持一致，修改后的代码如下
    ```{code-block} kconfig
    :caption:   hcpu/Kconfig.board

    config BSP_USING_BOARD_TESTBOARD_525
        bool
        select SOC_SF32LB52X
        select BF0_HCPU
        default y

    rsource "../Kconfig.board"
    ```

    ```{code-block} kconfig
    :caption:    lcpu/Kconfig.board

    config BSP_USING_BOARD_TESTBOARD_525
        bool
        select SOC_SF32LB52X
        select BF0_LCPU
        default y

    rsource "../Kconfig.board"

    ```
1. 修改`testboad_525/Kconfig.board`引用`testboard_525_base`下的`Kconfig.board`
    ```kconfig
    source "$SIFLI_SDK/customer/boards/testboard_525_base/Kconfig.board"
    ```
1. 至此，一块新的板子已经创建完成，可以切到`hello_world/rtt/project`执行命令`scons --board=testboard_525`使用新板子编译了
    ```none
    > scons --board=testboard_525 -j8
    scons: Reading SConscript files ...
    Board: testboard_525_hcpu
    ========
    Multi-Project Info
    --------
    full_name       main.bootloader
    parent          main
    bsp_root        D:\code\release_v2.3\release_v2.3.0_test\example\boot_loader\project\butterflmicro\ram_v2
    build_dir       build_testboard_525_hcpu/bootloader
    link_script     D:/code/release_v2.3/release_v2.3.0_test/example/boot_loader/project/butterflmicro/ram_v2\link
    ptab            D:/code/release_v2.3/release_v2.3.0_test/customer/boards/testboard_525\ptab.json
    embedded:       False
    --------
    full_name       main
    parent
    bsp_root        D:\code\release_v2.3\release_v2.3.0_test\example\get-started\hello_world\rtt\project
    build_dir       build_testboard_525_hcpu/
    link_script     D:/code/release_v2.3/release_v2.3.0_test/drivers/cmsis/sf32lb52x/Templates/gcc/HCPU/link
    ptab            D:/code/release_v2.3/release_v2.3.0_test/customer/boards/testboard_525\ptab.json
    --------
    full_name       main.ftab
    parent          main
    bsp_root        D:\code\release_v2.3\release_v2.3.0_test\example\flash_table\sf32lb52x_common_v2
    build_dir       build_testboard_525_hcpu/ftab
    link_script     D:/code/release_v2.3/release_v2.3.0_test/drivers/cmsis/sf32lb52x/Templates/gcc/HCPU/link
    ptab            D:/code/release_v2.3/release_v2.3.0_test/customer/boards/testboard_525\ptab.json
    embedded:       False
    ========
    ```

## 板子的目录结构
板子的目录结构说明如下

```{code-block} none
:caption: testboard_525

|   Kconfig.board          // 板子的Kconfig配置文件
|   ptab.json              // memory分区表
|   SConscript             // 编译脚本
|   
+---hcpu
|       board.conf         // 由menuconfig生成的HCPU最小配置文件
|       custom_mem_map.h   // 打开CUSTOM_MEM_MAP时使用，用于自定义memory规划
|       Kconfig            
|       Kconfig.board      // HCPU的Kconfig配置文件，可以source上级目录下的`Kconfig.board`
|       rtconfig.py        // 可以指定JLINK_DEVICE，生成下载脚本时使用
|       
\---lcpu    
        board.conf         // 由menuconfig生成的LCPU最小配置文件
        custom_mem_map.h
        Kconfig
        Kconfig.board
        rtconfig.py
```

```{code-block} none
:caption: testboard_525_base

|   board.h
|   bsp_board.h
|   bsp_init.c        // HAL_PreInit实现
|   bsp_lcd_tp.c      // IO相关的配置代码
|   bsp_pinmux.c      // IO相关的配置代码
|   bsp_power.c       // IO相关的配置代码
|   Kconfig.board     // 板子的Kconfig配置文件
|   
\---script
        SConscript
```        

`testboard_525_base`单独分离出去是为了提高代码的复用性，可能同一块板子有多种变体，比如搭载了不同的芯片，这样可以创建多块板子，都引用`testboard_525_base`，避免重复修改。
如果不考虑板子有多种变体，也可以将`testboard_525_base`里的文件合并到`testboard_525`目录下，修改文件中的引用路径

## 修改板子配置
板子的配置包含以下几个：
1. 硬件连接由`bsp_pinmux.c`、`board.conf`和`Kconfig.board`等文件定义，
    - `board.conf`存放可由menuconfig配置的选项，比如console使用的串口，修改方法为在`board.conf`所在目录下执行`menuconfig`，并按{kbd}`D`以最小配置保存
    - `Kconfig.board`存放menuconfig不可见的选项，比如下面的代码定义了触控中断使用的管脚、背光使用的PWM设备编号等
        ```kconfig
        config ASIC
            bool 
            default y 
        
        config TOUCH_IRQ_PIN
            int
            default 26

        config LCD_PWM_BACKLIGHT_INTERFACE_NAME
            string
            default "pwm3"

        config LCD_PWM_BACKLIGHT_CHANEL_NUM
            int
            default 4

        config LCD_BACKLIGHT_CONTROL_PIN
            int
            default 1
            
        config RGBLED_CONTROL_PIN
            int
            default 32  

        ```
    - `bsp_pinmux.c`等文件通过实现BSP_PIN_Init等函数，配置管脚的功能与上下拉属性（使用`HAL_PIN_Set`接口）
1. memory分区表ptab.json\
    该文件可以描述所有memory的分区信息，包括NOR Flash、NAND Flash、PSRAM、片内SRAM、SD卡等。
    编译时会由ptab.json生成`ptab.h`到build目录下，`ptab.h`中定义了一组以`_START_ADDR`、`_OFFSET`、`_SIZE`为后缀的宏，代码中可以使用这些宏获取分区信息

更多信息可参考[](/app_note/common_project.md)