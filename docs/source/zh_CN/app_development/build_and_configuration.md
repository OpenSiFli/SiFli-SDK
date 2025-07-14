# 配置与编译
通过[](create_application.md)与[](create_board.md)，您已经对应用程序与板子有了直观的认识，这里将对两者的关系和工作机制做进一步的介绍。

传统的RT-Thread工程目录对应一个应用程序，但他已经与特定的板子关联了起来，目录下的`rtconfig.h`定义了完整的工程配置参数，如果想让这个应用程序运行在其他板子上，需要基于对应板子的BSP包新建一个工程目录，将应用程序的代码和相关配置移植过去，这个过程比较繁琐也容易出错。SiFli-SDK参考了Zephyr的做法，将板子（硬件运行环境）与应用程序分离，只要板子具备应用程序所需的硬件能力，就很容易的编译任意板子的目标文件，这个可以编译任意目标板程序的应用程序工程称为**通用工程**。指定目标板的编译方法是：
```shell
scons --board=<board_name> -jN
```
其中，`<board_name>`是板子名称，选择方法参见[](../supported_boards/index.md)，`-jN`为多线程编译参数，N为线程数，比如下面的命令使用8个线程编译板子sf32lb52-lcd_n16r8的目标文件
```shell
scons --board=sf32lb52-lcd_n16r8 -j8
```
原有的`--target=<target_name>`参数也可以和`--board`结合使用，比如想创建sf32lb52-lcd_n16r8板子相应的Keil工程文件，可以执行以下命令

```shell
scons --board=sf32lb52-lcd_n16r8 --target=mdk5 -s
```

```{note}
需要注意的是，SDK采用多工程编译，应用程序的工程只是主工程，会联动编译相应的子工程，如二级boot、ftab等工程，但使用`--target`仅生成主工程对应的Keil工程，直接使用该工程编译会有问题，只能用于阅读代码
```

除了使用SDK自带的板子配置，还可以使用`--board_search_path`指定一个目录作为第三方板子的搜索路径，这个目录可以在SDK之外，可以是相对路径也可以是绝对路径，当指定了搜索路径后，编译时除了从SDK的板子目录查找板子，还会从这个目录获取板子配置，如果两个目录下有同名的板子，会使用`--board_search_path`指定目录下的板子。例如在app1的project目录下执行如下的编译命令，以相对路径指定板子的搜索路径

```shell
scons --board=test_board --board_search_path=../../boards -j8
```

代码的目录结构如下，以上命令是在`app1/project`目录下执行，板子`test_board`在`boards`目录下，`workspace`是任意路径的一个工作目录，可以在SDK之外

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




## 项目设置
SDK使用menuconfig（kconfiglib包中的一个图形化界面工具）管理项目设置，编译时从`rtconfig.h`中读取所有的宏开关，指示SCons需要编译哪些模块、模块参数如何，相对应的kconfig配置存放在`.config`中。为了解决前面提到的问题，通用工程目录下不再存放`rtconfig.h`和`.config`，而是在编译时根据选择的板子将这两个文件动态生成在build目录下，生成的`.config`是由Kconfig的默认值、`board.conf`和`proj.conf`三者合并而成。`board.conf`和`proj.conf`中记录了需要修改的配置（与默认值相比有变化的部分），如果相同的配置同时出现在`board.conf`和`proj.conf`中，则使用`proj.conf`定义的配置。

如果想修改`proj.conf`，可以在工程目录下执行`menuconfig --board=<board_name>`，其中`<board_name>`如果不带`_hcpu`后缀则默认按HCPU来配置，如果想使用LCPU的配置，则需要在板子名称后加上`_lcpu`后缀，如`sf32lb52-lcd_n16r8_lcpu`。menuconfig界面上看到的参数设置与实际编译时用到的相同，修改设置后按{kbd}`D`保存最小配置到`proj.conf`中。如果想修改`board.conf`，则需要切换到板子目录下执行`menuconfig`（不带任何参数），如切换到`boards/sf32lb52-lcd_n16r8/hcpu`目录下运行`menuconfig`

```{note}
`proj.conf`中的存放的配置如果不适用于所有板子，可以在工程目录下创建板子的子目录，在这个目录下放置该板子专属的`proj.conf`，以做差异化配置，具体方法参考[](../app_note/common_project.md)
```

扩展阅读请参考[](../app_note/common_project.md)