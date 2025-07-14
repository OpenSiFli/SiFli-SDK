# USBC

USB 设备模块（又名外设控制器设备，PCD）模块在全速 USB 2.0 总线和 APB 总线之间实现 USB 设备功能。

## PCD主要特点 
 - USB SF32LB55x/56x规范版本 2.0 全速兼容, SF32LB58x支持USB 2.0高速传输。  
 - 可配置的端点数量最多到8个。  
 - 循环冗余校验 (CRC) 生成/校验、不归零反转 (NRZI) 编码/解码和位填充。  
请注意，PCD 使用 RT-Thread USB 设备堆栈进行测试，复合设备设置包括 USB 存储和 USB CDC UART。 

## 使用PCD
PCD 只能使用上层 USB 设备堆栈进行测试。 HAL级别，SiFli SDK 提供支持  
 - USBD 硬件模块初始化。  
 - EP0 读/写。  
 - EPx 读/写。  
 - USB中断处理程序。  
 - 端点停止/取消停止。  
USB 上层设备栈可以调用 USB 设备函数并实现 PCD HAL 模块定义的回调。 PCD的详细使用请参考RT-Thread SiFli USB驱动工具（在 _/rtos/rtthread/bsp/sifli/drivers/drv_usbd.c_ ）



## API参考
[](../api/hal/pcd.md)
