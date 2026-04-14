# 动态加载ezip示例(LVGL V9)

[English](README_EN.md) | 简体中文

源码路径: example/multimedia/lvgl/ezip_v9

## 概述
本示例演示如何在RT-Thread环境中使用LVGL动态加载ezip格式的图片文件。示例支持：
- 从Flash文件系统或SD卡加载ezip图片
- 选择将图片加载到SRAM或PSRAM
- 通过文件路径或预加载描述符两种方式显示图片
- 通过finsh命令行动态切换图片源

## 功能特性
- **多种加载方式**：支持直接从文件加载或预加载到内存
- **灵活内存管理**：可选择SRAM或PSRAM存储图片数据
- **文件系统支持**：支持从Flash文件系统和SD卡读取图片
- **命令行控制**：通过finsh命令动态切换图片源

## 用法

### 支持的平台
例程可以运行在以下开发板
* sf32lb52-lchspi-ulp

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lchspi-ulp -j8
```

### 准备图片文件

默认提供的图片文件：
- LVGL V9: `example.ezip`

如需使用自定义图片，可以：
- 使用sdk/tools/png2ezip/ezip.exe工具将PNG图片转换为ezip格式
  ```
  ezip -convert xxx\yyy.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8/9
  ```
  **命令参数说明**：
  - `-convert`: 指定要转换的PNG图片路径
  - `-rgb565`: 设置输出颜色格式为RGB565
  - `-binfile 2`: 设置输出为二进制文件格式
  - `-binext .ezip`: 指定输出文件扩展名为.ezip，可以自定义为不超过20个字符的扩展名
  - `-lvgl_version`: 指定目标LVGL版本，支持8或9
  
  **使用示例**：
  ```
  # 转换为LVGL V8格式
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8
  
  # 转换为LVGL V9格式
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 9
  ```
- 将图片文件放入`disk`目录，重新编译烧录
- 或将图片文件复制到SD卡根目录（需要插入SD卡）

## 示例输出
如果示例运行成功，您将在串口看到以下输出：
```
(...省略系统初始化信息...)

Register root to mtd device with base addr 0x12820000
mount fs on flash to root success
dynamic ezip loading example.
mount fs on tf card to /sdcard success
```

LCD屏幕将显示加载的ezip图片，图片居中显示。

## 使用finsh命令

程序启动后，可以通过串口命令动态切换图片源：

### 命令格式
```
set_image [path] [file/dsc] [sram/psram]
```

### 参数说明
- `path`: 图片文件路径
- `file/dsc`: 加载方式
  - `file`: 直接使用文件路径（不预加载到内存）
  - `dsc`: 预加载到内存作为描述符
- `sram/psram`: 内存类型（可选，默认sram）
  - `sram`: 使用SRAM
  - `psram`: 使用PSRAM

**重要提示**：LVGL V8 和 V9 对文件路径格式要求不同：
- **LVGL V8**：直接使用文件名，如 `example.ezip` 或 `/sdcard/image.ezip`
- **LVGL V9**：需要添加盘符前缀，如 `A:/example.ezip` 或 `A:/sdcard/image.ezip`

### 使用示例
```
# LVGL V8 - 从文件直接加载（不占用额外内存）
set_image example.ezip file

# LVGL V8 - 预加载到PSRAM（提高显示性能）
set_image example.ezip dsc psram

# LVGL V9 - 从文件加载（注意需要 A: 前缀）
set_image A:/example.ezip file

# 从SD卡加载（LVGL V8）
set_image /sdcard/image.ezip dsc sram

# 从SD卡加载（LVGL V9，注意需要 A: 前缀）
set_image A:/sdcard/image.ezip dsc sram
```

## API说明

### load_ezip
```c
int load_ezip(lv_img_dsc_t *img_dsc, const char *ezip_path, int use_psram)
```
加载ezip图片文件到内存。

**参数**：
- `img_dsc`: LVGL图片描述符指针
- `ezip_path`: 图片文件路径
- `use_psram`: 1使用PSRAM，0使用SRAM

**返回值**：成功返回0，失败返回-1

### unload_ezip
```c
void unload_ezip(lv_img_dsc_t *img_dsc, int use_psram)
```
卸载ezip图片并释放内存。

**参数**：
- `img_dsc`: LVGL图片描述符指针
- `use_psram`: 1表示从PSRAM释放，0表示从SRAM释放

## 注意事项
1. 确保开发板有足够的SRAM或PSRAM空间存储图片数据
2. ezip格式需要匹配LVGL版本（V8或V9）
3. **LVGL V8 和 V9 文件路径格式不同**：
   - V8：直接使用路径，如 `example.ezip`
   - V9：需要加盘符前缀，如 `A:/example.ezip`
4. 使用SD卡时需要确保卡已正确格式化为FAT32
5. 预加载方式（dsc）性能更好但占用内存，文件方式（file）节省内存但性能较低

## 异常诊断
- **图片无法显示**：检查ezip文件是否存在，路径是否正确
- **内存分配失败**：检查SRAM或PSRAM是否有足够空间
- **SD卡挂载失败**：确保SD卡已格式化为FAT32格式

如有其他问题，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [LVGL 文档](https://docs.lvgl.io/)
