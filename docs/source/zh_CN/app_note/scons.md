# Scons和Kconfig快速入门

SifliSDK提供了一个Env配置工具，这个工具里就已经带有SCons和Python，因此在Windows平台使用SCons并不需要安装。SifliSDK在scons基础上添加了一些脚本库函数，简化用户的项目配置。

## 1. 项目文件结构

SCons使用SConscript和SConstruct文件来组织源码结构，通常来说一个项目只有一 SConstruct，在项目根目录中，但是会有多个SConscript。一般情况下，每个存放有源代码的子目录下都会放置一个SConscript。用户可以参考 _example\\uart\\project\\ec-lbxxx_ ，看到项目文件结构。

SifliSDK 为每个项目单独创建了一个名为 _rtconfig.py_ 的文件。因此每一个项目目录下都会存在下面三个文件：_rtconfig.py_ ，_SConstruct_ 和 _SConscript_ ，它们控制项目的编译。一个项目中只有一个 _SConstruct_ 文件，但是却可能会有多个 _SConscript_ 文件，可以说 _SConscript_ 文件是组织源码的主力军。

SifliSDK使用menuconfig来配置项目，每个项目有一个Kconfig，在项目根目录中，作为配置选项的起点，配置完成后会在项目根目录生成 _.config_ 和 _rtconfig.h_ ，分别用于Scons脚本和c文件。

SifliSDK大部分源码文件夹下也存在 _SConscript_ 文件，这些文件会被项目根目录下的 _SConscript_ 文件 “找到” 从而将 _.config/rtconfig.h_ 中定义的宏对应的源代码加入到编译器中来。

SCons编译项目首先执行的是 _SConstruct._ 在用户项目的 _Sconstruct_ 可以配置一些项目的选项，和menuconfig工具生成的项目配置.config配合, `SifliEnv()`函数会设置相应的默认编译选项。用户如果需要强制修改相关编译/链接的选项，可以改变`Environment`()函数中相应的参数。

## 2. Menuconfig工具

Kconfig文件是各种配置界面的源文件，在项目目录下使用env工具执行`menuconfig`命令调出的系统配置界面，就是通过读取当前项目目录下的Kconfig文件来生成的配置界面，这个文件是所有配置的总入口。不同层级目录下的Kconfig文件可以通过这个入口包含到配置界面上来，配置工具通过读取各个目录下的Kconfig文件，从而生成配置界面供开发人员配置系统，最终生成RT-Thread系统的配置文件 _rtconfig.h_

类型：每个config菜单项都要有类型定义，变量共有四种种类型：bool，string，hex，int

反向依赖关系：select语句表示反向依赖关系，即当前配置选项被选中

默认值：表示当前配置选项的默认值，很好理解

通过以上语句配置之后，最终就会在 _rtconfig.h_ 文件中生成如下两个宏：

```c
#define ASIC 1
#define BSP_USING_EMPTY_ASSERT 1
```

- menu语句\
menu语句用于生成菜单，使用menu和endmenu定义一个菜单，其中可以包含很多条config语句。这个很简单，不做详细展开。

- if/endif语句\
用于定义条件判断语句，通过判断一个标志，来决定是否显示某个配置项。

- menuconfig语句\
表示带菜单的配置项，menuconfig语句和config语句相似，但它在config的基础上要求所有的子选项作为独立的行显示。用depends on表示依赖某个配置选项。

- choice/endchoice语句\
将多个类似的配置选项组合在一起，供用户选择一组配置项。用prompt给出提示信息，作为弹出框的标题，在一组choice语句中，可以给出多个配置项供选择，但是在配置界面只能选择一个配置项，相当于是一个单选框。

- comment语句\
comment 语句出现在界面的第一行，用于定义一些提示信息。

- source语句\
用于读取另一个文件中的 Kconfig 文件

更详细的配置请参考[](../app_note/menuconfig.md)

## 3. SCons的基本功能

在Env工具中直接输入`scons`命令直接编译项目，默认使用ARM CLANG编译器。SifliSDK根目录有 _set_env.bat_ , 里面会设定编译器目录。

如果需要自己制定编译器，可以在修改 _set_env.bat_ 里面的命令指定：

    set RTT_CC = keil
    set RTT_EXEC = C:/Keilv5

scons命令
该命令不仅可以完成基本的编译，还可以生成MDK/IAR/VS 工程，在后边增加不同参数可以实现不同效果。

- `-s` 参数，不会打印具体的内部命令。
- `-c` 参数，清除编译目标，清除临时文件和目标文件。
- `--target=XXX` 参数，如果使用mdk/iar来进行项目开发，当修改了 _rtconfig.h_ 打开或者关闭某些组件时，需要使用该参数来重新生成对应的定制化工程。然后在mdk/iar中进行编译下载。

```{warning}
注意：要生成MDK或者IAR的工程文件，前提条件是项目目录存在一个工程模版文件，然后`scons`才会根据这份模版文件加入相关的源码，头文件搜索路径，编译参数，链接参数等。而至于这个工程是针对哪颗芯片的，则直接由这份工程模版文件指定。所以大多数情况下，这个模版文件是一份空的工程文件，用于辅助 _SCons_ 生成 _project.uvprojx_ 或者 _project.eww_ 。为了方便客户，如果没有找到 _project.uvprojx_ , SiFliSDK会使用一份默认的模板用于生成 _project.uvprojx_ ，用户如果不想使用默认的模板（例如添加自己的编译选项），可以将 _SifliSDK\\tools\\build\\template_ 下面的 _template.uvprojx_ 拷贝到项目目录，作为自己的项目模板，添加自己特定的改变。
```

- `-jN` 参数，多线程编译目标，在多核计算机上可以使用此命令加快编译速度。一般来说一颗 cpu 核心可以支持 2 个线程。双核机器上使用`scons -j4`命令即可。
- `--verbose`参数，默认情况下，使用`scons`命令编译的输出不会显示编译参数，使用该参数，会显示编译参数。

## 4. SCons 内置函数
如果想要将自己的一些源代码加入到 _SCons_ 编译环境中，一般可以创建或修改已有 _SConscript_ 文件。

SConscript 文件可以控制源码文件的加入，并且可以指定文件的Group（与MDK/IAR等IDE中的Group的概念类似）。

SCons提供了很多内置函数可以帮助我们快速添加源码程序，利用这些函数，再配合一些简单的Python语句我们就能随心所欲向项目中添加或者删除源码。下面将简单介绍一些常用函数。实际上每个 _SConscript_ 文件就是一个python文件。

- `GetCurrentDir()` 函数，获取当前路径。
- `Glob('*.c')`函数，获取当前目录下所有C文件。修改参数中的后缀，可以匹配当前目录下的所有某类型的文件。
- `GetDepend(macro)`函数，该函数定义在tools目录下的脚本文件中，它会从 _rtconfig.h_ 文件读取配置信息，其参数为 _rtconfig.h_ 中的宏名。如果 _rtconfig.h_ 打开了某个宏，则这个方法（函数）返回真，否则返回假。
- `Split(str)`函数，将字符串str分割成一个列表list。
- `DefineGroup(name， src， depend，**parameters)`函数，这是基于 _SCons_ 扩展的一个方法（函数）。DefineGroup用于定义一个组件。组件可以是一个目录（下的文件或子目录），也是后续一些IDE工程文件中的一个Group或文件夹。parameter可以添加：

  - LIBS=\<some.lib\>  将特定lib链接进入项目 <br>
  - LIBPATH=\<some path\> 在特定目录中放入.lib  <br>
  - LIBRARY=\<some.lib\> 将所有group 中源代码编译链接，生成 .lib <br>
  - INSTALL_PATH=\<some path\> 将生成的.lib安装到特定目录中 <br>
  - CCFLAGS=\<more CC flags\> 添加编译选项 <br>
  - CPPPATH=\<some folder\> 添加C语言头文件搜索目录 <br>
  - CPPDEFINES=\<more macro\> 添加C语言宏定义 <br>
  - LINKFLAGS=\<more link flags\> 添加链接选项 <br>
  - ASFLAGS=\<more assembly flags\> 添加汇编选项 <br>
    

