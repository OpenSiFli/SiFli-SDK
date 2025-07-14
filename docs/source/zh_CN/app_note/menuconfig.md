# 项目设置

SiFli SDK通过menuconfig工具来配置用户应用和SDK的选项。 Menuconfig使用和Linux核心配置相同的语法，生成和工程相关的 _.config_ 和 _rtconfig.h_ , 可以分别应用于编译脚本和C代码。有关Menuconfig的语法，可以参考 https://www.kernel.org/doc/html/latest/kbuild/kconfig-language.html

## menuconfig启动： 
在用户工程目录（通常有Sconscruct文件在相同目录）启动menuconfig，设置好SDK环境之后，执行menuconfig，出现menuconfig界面。<br>
![ ](../../assets/start.png)

### 修改configuration

更改菜单配置界面中的配置值，如下所示：
- 使用箭头键浏览菜单。也支持常用的Vim键绑定。 
- 使用空格键和`Enter`键进入菜单并切换值。菜单旁边显示—>。按`ESC`返回到父菜单。 
    布尔型配置选项用[]括号显示，而数字和字符串值配置符号用（）括号显示。不能更改的符号值显示为–或-*-。 
    你还可以按Y或N将布尔配置符号设置为相应的值。 
- 按`?`显示有关当前所选符号的信息，包括其帮助文本。按ESC或Q从信息显示返回菜单。 
- 在menuconfig界面中按Q将打开“保存并退出”对话框（如果有要保存的更改）：

![](../../assets/save.png)
按Y将内核配置选项保存为默认文件名（.config）。 除非你使用不同的配置，否则通常将保存为默认文件名。
```{note}
    构建期间使用的配置文件始终是 _.config_ 。 如果要使用其他已保存的配置，请将其复制到 .config。 确保备份原始配置文件。 <br>
```

![](../../assets/search.png)
如果你跳到当前不可见的符号（例如，由于依赖关系不满意），则将启用“全部显示”模式。 在全部显示模式下，将显示所有符号，包括当前不可见的符号。 要关闭全部显示模式，请在menuconfig中按ESC。

- 如果当前菜单中没有可见的项目，则无法关闭“全部显示”模式。
  要弄清楚为什么跳转到的符号不可见，请按一下来检查其依赖性。如果发现该符号依赖于另一个未启用的符号，则可以依次跳至该符号以查看是否可以启用它
- 在menuconfig中，您可以按'F'在跳出对话框中查看当前所选项目的帮助，而无需退出该对话框。

## 设置Kconfig配置值

menuconfig接口可用于在应用程序开发期间配置测试。 本页说明如何使设置永久化。
在Kconfig符号参考中可以找到所有Kconfig选项的自动生成的列表。
```{note} 
在更改Kconfig文件之前，最好先浏览一下Kconfig-提示和最佳做法页面。
```

### Kconfig配置项

进行Kconfig更改时，重要的是要了解配置项和不可见配置项之间的区别。可见符号是用提示定义的符号。可见符号显示在交互式配置界面中（因此可见），可以在配置文件中进行设置。

这是可见配置的示例：<br>
![](../../assets/seeable.png)

该配置项在menuconfig中显示如下，可以在其中进行切换：<br>
不可见的配置项是没有提示的符号。交互式配置界面中未显示不可见的配置项，用户无法直接控制其值。相反，它们从默认值或其他配置项中获取值。
![](../../assets/unseeable.png)

在这种情况下，通过具有选择BF0_HCPU的其他符号来启用BF0_HCPU。

### 通过配置文件中设置配置项
可见配置项可以通过在配置文件中进行设置来配置。初始配置是通过将板子的 _.config_ 文件与应用程序设置合并而产生的。有关更多详细信息，请参见下面的初始配置。 <br>
配置文件中的分配使用以下语法： <br>
CONFIG=\<value>
```{note} 
等号两边不能有空格。
```

可以通过将bool符号分别设置为y或n来启用或禁用它们。上面的示例中的FPU符号可以这样启用：

  CONFIG_FPU=y

```{note}
1.布尔符号也可以设置为n，其注释格式如下：
    #CONFIG_SOME_OTHER_BOOL is not set
2.这个在 .config中的合并配置中看到的格式。
```

其他符号类型的分配如下：
- CONFIG_SOME_STRING="cool value"
- CONFIG_SOME_INT=123

### 注释使用#
  \#This is a comment

## SiFli SDK的配置
SDK包含多种选项，可以提供给用户进行配置选择，包括芯片，电路板，驱动，RTOS，外围电路，中间件，三方软件等等。用户可以根据需要定制自己选择的功能。
用户可以在自己工程的Kconfig中，添加SDK的配置，
用户可以在自己的KConfig中间加入

```kconfig
config SIFLI_SDK
    string 
    option env="SIFLI_SDK"
    default "."

# SDK configuration	
source "$SIFLI_SDK/Kconfig"
```

这样就可以引入SDK的配置，在调用menuconfig之后，可以看到

    ------------End of SDK configuration ----------- 

作为SDK配置的分隔符。

SDK的配置主要有以下几个部分：
- 芯片选择 （Select chipset）
  选择芯片，目前请选择SF32LB55X。
- 操作系统选择 （RTOS）
  RT-Thread是目前主要支持的操作系统，建议使用，里面也包含了芯片内部IP 的RT-Thread 驱动配置
- 板级外设（Select board peripherals）
  里面提供思澈科技支持的一些外部设备的配置。
- 中间件 （Sifli middleware）
  里面包含了思澈科技开发的中间件配置
- 第三方软件 （Third party package）
  里面包含了思澈科技验证和优化的第三方软件。
  
用户也可以自己定制一些Kconfig配置，建议不要和SDK配置混杂在一起，可以单独使用不同的Kconfig文件。  


