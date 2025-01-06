这是一个简单的手表应用程序，包含思澈科技开发的主界面，表盘，和一些其他应用，作为示例展示思澈科技基于RT-thread和LVGL开发的GUI应用。

请进入：

1. ./project/ec-lb551目录
    编译支持LB551硬件的手表应用程序
    1.1 行命令编译
        scons -j8
    1.2 Keil工程编译   
    1.2.1 生成资源
        scons resource --no_cc
    1.2.2 生成Keil工程
        scons --target=mdk5 -s
    1.2.3 打开Keil工程编译
    如果要验证Javascript程序，还需要调用./project/ec-lb551/jsroot.bat, 下载JS程序的文件系统
2. ./project/ec-lb555目录
    编译支持LB555硬件的手表应用程序
    2.1 行命令编译
        scons -j8
    2.2 Keil工程编译   
    2.2.1 生成资源
        scons resource --no_cc
    2.2.2 生成Keil工程
        scons --target=mdk5 -s
    2.2.3 打开Keil工程编译
    如果要验证Javascript程序，还需要调用./project/ec-lb555/jsroot.bat, 下载JS程序的文件系统
3. ./project/ec-lb583_v11，ec-lb587_v11，ec-lb563_nand目录
    编译支持LB583硬件的手表应用程序
    2.1 行命令编译
        scons -j8
    2.2 Keil工程编译   
    2.2.1 生成资源
        scons resource --no_cc
    2.2.2 生成lcpu
        scons build/lcpu --no_cc -j8
    2.2.3 生成Keil工程
        scons --target=mdk5 -s
    2.2.4 打开Keil工程编译
    如果要验证Javascript程序，还需要调用./project/ec-lb583_nand/jsroot.bat, 下载JS程序的文件系统
4. ./project/simulator目录
    编译支持PC模拟器的手表应用程序
    3.1 生成资源
        scons resource --no_cc
    3.2 生成MSVC工程
        scons --target=vs2017 -s
    3.3 打开MSVC工程编译    
    
    也可以在行命令编译支持PC模拟器的手表应用程序， 这种方式编译速度更快。
    3.1.1 生成资源
        scons resource --no_cc
    3.2.1 修改 ./project/simulator/msvc_setup.bat, 指向MSVC和Windows SDK安装目录
    3.3.1 进行编译
        scons -j8 
    3.4.1 调试
        启动Visual Studio, 打开 ./project/simulator/debug_sim_cmdbuild.vcxproj进行调试。
           
    