# coredump 示例
源码路径:example/system/coredump

## 概述
- 该例程演示崩溃信息的保存流程，并支持通过 BLE 与手机连接，将死机现场数据传输到手机；设备上电后会广播名称形如 `COREDUMP-xx-xx-xx-xx-xx-xx`。

## 支持的平台
在以下平台验证通过：
- sf32lb52-lcd_n16r8
- sf32lb52-lcd_a128r16

## 配置与 menuconfig
例程支持四种模式组合：
1) 分区模式，不开minidump
2) 分区模式，开minidump
3) 文件模式，不开minidump
4) 文件模式，开minidump
默认为 1.分区模式，不开minidump，可通过以下配置修改模式
2. 分区模式，开minidump
![alt text](asserts/partition.png)
![alt text](asserts/mini_enable.png)
3. 文件模式，不开minidump
![alt text](asserts/file_mode.png)
![alt text](asserts/mini_disable.png)
4. 文件模式，开minidump
![alt text](asserts/file_mode.png)
![alt text](asserts/mini_enable.png)

### 编译和烧录
以sf32lb52-lcd_n16r8为例，按照以下步骤，可以完成编译和烧录。
```
scons --board=sf32lb52-lcd_n16r8

.\build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
```

## 例程的使用
程序上电后，当发生死机情况时(可手动assert触发死机)会出现如下log表明将死机时的现场数据进行保存
![alt text](asserts/assert.png)
重新复位开发板，打开手机 sifli ble APP。找到名称形如 `COREDUMP-xx-xx-xx-xx-xx-xx`的蓝牙进行连接，再通过手机APP将死机现场数据进行导出即可
![alt text](asserts/ble.png)
导出的过程中会有对应大量log出现
![alt text](asserts/dump.png)
最后在手机端会生成一个bin文件，可以将这个文件发送到电脑使用Context2Mem.exe工具转换.bin文件为dump文件http://docs.sifli.cc/test_doc/methods/%E6%95%B4%E6%9C%BAlog%E5%AF%BC%E5%87%BA%E4%B8%8E%E8%BD%AC%E6%8D%A2.html

