1.	Italic
    1.1. Use Cases
        1.1.1. File Paths
            Example: 
            _$SDK_ROOT\example\flash_table\project.uvprojx_
        1.1.2.	Complete Filenames
            Example: 
            _download.bat_ 
            _download.jlink_ 
    1.2. Guidelines
        Add spaces before and after such fields to separate them from the main text
        Example:
        Enter the env directory, you can run _env.exe_ in this directory. If it fails to open, you can try using _env.bat_ .
    1.3. Syntax
        Add "_" before and after the field to be italicized

2.	Code—block
    2.1. Use Cases
        2.1.1. Command Line
            Example: 
            Enter the compilation directory: `cd example\hal_example\project\common`
            Execute the compilation command: `scons --board=eh-lb525_v2_hcpu -j8`
        2.1.2. Elements appearing in code: function names, structures, macros, etc.
            Example:
            Call `rt_hw_watchdog_pet()` in the thread to ensure
            Erase through `rt_flash_erase`, for
            Although the macro `LCPU_RAM_CODE_SIZE` is used to limit
    2.2. Guidelines
        No need to add spaces before or after
    2.3. Syntax
        Add "`" before and after the field to be formatted as code

Note: Letters and numbers in the main text do not require any modifications. Do not add spaces before or after them.
Example: