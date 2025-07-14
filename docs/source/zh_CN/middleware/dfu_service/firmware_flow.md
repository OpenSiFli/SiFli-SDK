# 流程介绍

## Flash布局

如下图所示，默认Bootrom会根据flash config table跳转到0x10020000（具体参考 [](/bootloader.md)），固件升级服务需要单独创建工程OTA manager，用户工程User Code需要放在OTA manager后。

Bootrom跳转到OTA manager以后，再根据当前状态：若不需要升级则跳转到用户工程；若需要升级则等待升级。

用户工程的起始地址、固件升级包存放的Upgrade bin地址均可通过工程的 _memory_map.h_ 、 _custom_memory_map.h_ 进行配置。Upgrade bin建议放在Flash的最后。

 ![](../../../assets/ota_flash_layout.png)


## 流程简介

下图介绍了OTA的基本流程。完整的OTA升级文件包含携带验证信息的 _ctrl_packet.bin_ 以及随后的固件包，资源包和字体包。

![](../../../assets/ota_flow.png)


- 步骤1. 远端设备将 _ctrl_packet.bin_ 传给user bin来确认是否可以进行OTA，如果可以，继续步骤2
- 步骤2. user bin需要重启进入ota manger bin，并和远端设备重新建立蓝牙连接。如果一定时间内没有连接，会重新返回user bin。
- 步骤3. 建立蓝牙连接后，远端设备会一直传输升级包直到所有升级包传输完成。
    - 如果固件升级包下载失败，会重新回到user bin。
    - 如果资源和字体下载失败，会强制留再ota manager bin，直到下载成功为止。
    - 下载失败后，OTA支持resume和强制从头升级两种方式，由远端设备决定采用哪种方式。
- 步骤4. 解压缩固件升级包并完成安装。
- 步骤5. 安装完成后跳转到user bin，完成OTA。


## 注意事项

- OTA的升级包采用加密方式，须确保使用携带加密信息的flash config table；如果解密存在efuse，须确保efuse有正确烧录相关内容。若校验失败，远端设备会收到09（DFU_ERR_CONTROL_PACKET_INVALID）的错误代码。
- 如果OTA整个过程中因为BLE断连或其他原因导致失败，远端设备会收到10（DFU_ERR_OTA_ONGOING）的错误代码，远端设备可以决定resume或从头升级。


## 工程配置

用户工程配置：
- 双核均需要配置：
    ![](../../../assets/ota_hcpu_config1.png)
    ![](../../../assets/ota_hcpu_config2.png)
- LCPU配置：
    如果ble service放在LCPU，需要配置LCPU的port service以便于OTA manager能获取到BLE数据
    ![](../../../assets/ota_lcpu_config1.png)

OTA Manager工程配置：
- 除了用户工程双核均需配置的内容为，还需要加入zlib
    ![](../../../assets/ota_lcpu_config2.png)
