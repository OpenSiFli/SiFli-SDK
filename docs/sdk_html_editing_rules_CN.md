1.	斜体
    1.1.使用情景
        1.1.1.文件路径
            例： 
            _$SDK_ROOT\example\flash_table\project.uvprojx_
        1.1.2.	完整文件名
            例: 
            _download.bat_ 
            _download.jlink_ 
    1.2.规范
        在此类字段前后加入空格，与正文隔开
        例：
        入env目录，可以运行本目录下的 _env.exe_ ，如果打开失败可以尝试使用 _env .bat_ 。
    1.3.语法
        在所需改为斜体的字段前后加入 “_”

2.	Code—block
    2.1.使用情景
        2.1.1.命令行
            例： 
            进入到编译目录下：`cd example\hal_example\project\common`
            执行编译命令 ：`scons --board=eh-lb525_v2_hcpu -j8`
        2.1.2.出现在代码里的，函数名，结构体，宏，等
            例：
            线程中调用`rt_hw_watchdog_pet()`, 确保
            通过`rt_flash_erase`进行擦除，对
            虽然使用了宏`LCPU_RAM_CODE_SIZE`限定了
    2.2.规范
        前后无需添加空格
    2.3.语法
        在所需改为斜体的字段前后加入 “`”


注：正文中的英文数字等无需任何修改，前后不要加空格。
例：
 