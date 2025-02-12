# HAL综合示例

## 介绍
在串口使用命令：
1. help 可以显示所有命令
2. utest_list 可以显示所有支持的example.
3. utest_run example_xxx 可以运行某一个模块的example.

## 工程编译及下载：
Common工程可以通过指定board来编译可运行的对应板子的程序
- 比如想编译可以在HDK 525上运行的工程，执行scons --board=eh-lb525即可生成镜像文件
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的525工程，可以执行.`build_eh-lb525\download.bat`来通过jlink下载
- 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载