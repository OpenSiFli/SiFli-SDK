这个示例展示UART的功能，在UART3打印 (EVB 板子最小的串口)

UART Printf Example: retarget the C library printf function to the UART

在串口的输入, 示例会反馈串口输出输入数据的16进制代码

@工程编译及下载：Common工程可以通过指定board来编译适应相对board的工程
                - 比如想编译可以在SF32LB555上运行的工程，执行scons --board=eh-lb555即可生成工程
                - 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的555工程，可以执行.\build_eh-lb555\download.bat来通过jlink下载
                - 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
