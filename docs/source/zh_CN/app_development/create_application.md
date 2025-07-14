# 创建应用程序
创建一个应用程序最快捷的方法是复制一份已有的示例，下面以hello_world例程为例，说明如何基于该示例创建一个新的应用程序

## 拷贝例程
1. 拷贝示例：将`example/get-started/hello_world/rtt`拷贝到任意目录，比如`c:\work\ws\hello_world`，

    ```{image} ../../assets/app_folder.png
    :scale: 70%
    ```

    应用的目录结构

    ```none
    +---project                // 工程目录
    |       Kconfig
    |       Kconfig.proj
    |       proj.conf          // 工程配置文件
    |       rtconfig.py
    |       SConscript
    |       SConstruct         
    |       
    \---src                    // 代码目录 
            main.c
            SConscript
            
    ```



1. 打开Env命令行窗口：在SDK根目录右键菜单打开Env命令行窗口，运行`set_env gcc`设置环境参数
1. 进入到工程目录：`cd c:\work\ws\hello_world\project`切换路径到拷贝的示例工程目录
1. 编译：`scons --board=sf32lb52-lcd_n16r8 -j8`，这里选择的板子为`sf32lb52-lcd_n16r8`，参考[](/supported_boards/index.md)
    ```{image} ../../assets/create_app_and_build.png
    ```
1. 烧写：`build_sf32lb52-lcd_n16r8_hcpu\uart_download`    

至此，一个新的hello_world程序已经创建完成，新工程的编译、烧写方法与SDK自带例程完全相同。

## 修改代码
hello_world的应用代码在`src/main.c`文件中，可以看到他只在第5行打印了一次`Hello world`

{lineno-start=1}
```c
int main(void)
{
    rt_thread_mdelay(3000);      
    /* Output a message on console using printf function */
    rt_kprintf("Hello world!\n");

    /* Infinite loop */
    while (1)
    {
    }
    return 0;
}
```
现在我们在第10~11行加上如下的两句，每3秒打印一次`Hello SiFli`

{lineno-start=1 emphasize-lines="10,11"}
```c
int main(void)
{
    rt_thread_mdelay(3000);         
    /* Output a message on console using printf function */
    rt_kprintf("Hello world!\n");

    /* Infinite loop */
    while (1)
    {
        rt_kprintf("Hello SiFli!\n");
        rt_thread_mdelay(3000);  
    }
    return 0;
}
```
修改之后编译并下载程序，连接串口调试工具，复位后可以看到如图的打印输出

```{image} ../../assets/print_log.png
:scale: 70%
```


除了在`main.c`里添加代码以外，也可以在`src`目录下添加C文件，因为src目录下的Sconscript已经使用`Glob`命令把当前目录下的所有C文件都加入了编译，更多Scons脚本的用法请参考[](/app_note/scons.md)
```python
import os
from building import *

# Add source code
src = Glob('*.c')
group = DefineGroup('Applications', src, depend = [''])

Return('group')

```

## 修改设置
hello_world例程只有最简单的打印功能，很多组件都没有使能，即使在`main.c`里包含了组件的头文件，也无法调用相应函数，需要使用`menuconfig`工具修改
工程配置，方法是在工程目录下执行`menuconfig --board=<board_name>`，这里的`<board_name`替换为所使用的板子名称，如`menuconfig --board=sf32lb52-lcd_n16r8`，与`scons`编译时指定板子的名称类似，缺省使用hcpu，也就是`menuconfig --board=sf32lb52-lcd_n16r8`与`menuconfig --board=sf32lb52-lcd_n16r8_hcpu`效果相同，都是以板子sf32lb52-lcd_n16r8的HCPU为对象配置当前工程。

执行menuconfig后打开如图的界面，可以使用{kbd}`⇧`和{kbd}`⇩`方向键在不同菜单项间移动，按{kbd}`Enter`键进入子菜单，按{kbd}`Space`选中菜单项，确认修改都完成后，按{kbd}`D`保存最小配置到工程目录下的`proj.conf`文件中（即`project\proj.conf`），该文件为文本文件，可以打开查看修改前后的差异，menuconfig的详细用法参见[](/app_note/menuconfig.md)

```{image} ../../assets/menuconfig.png
```

更详细的配置与编译机制请阅读[](build_and_configuration.md)
