# LVGL v8 FreeType例程

源码路径: example/multimedia/lvgl/lvgl_v8_freetype

## 概述
本示例演示如何在RT-Thread环境中使用LVGL的FreeType字体引擎，在运行时从文件系统读取TTF字体并实时切换。示例支持：
- 开机加载默认字体，缺失或损坏时自动回退到内置字体
- 扫描字体目录，在界面上列出可用字体并点选切换
- 字号12~60连续调节，预览区内容超高时可上下滑动
- 加载即校验：字体过大或格式不支持时明确报错，界面维持原字体
- 单字体驻留：切换完成后释放上一个字体，内存不随切换次数增长

## 功能特性
- **文件系统字体**：字体以文件形式随镜像烧录，无需转换为C数组，更换字体不必重新编译固件
- **实时切换**：切换与字号调节均在运行时完成，无需重启
- **资源守护**：字体对象的创建受堆余量下限保护，避免"加载成功、稍后因内存耗尽而崩溃"
- **安全卸载**：卸载前会检查字体是否仍被界面引用，仍在显示的字体不会被释放
- **命令行控制**：通过finsh命令重启示例、验证与启用字体

## 用法

### 支持的平台
例程已在以下开发板验证：
* sf32lb52-lchspi-ulp

其它开发板需满足两个条件：分区表中存在 `FS_REGION` 文件系统分区（示例将其挂载为根目录），且系统堆能容纳所用字体（见[注意事项](#注意事项)）。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lchspi-ulp -j8
```
其它支持板子替换 `--board` 即可。

执行烧写命令：
```
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```
按提示选择端口即可进行下载：
```none
please input the serial port num:5
```

```{note}
字体随文件系统镜像 `fs_root.bin` 一同烧录，修改 `disk/` 下的字体后必须重新编译并**完整烧录**，只烧 `main.bin` 不会更新板上的字体文件。
```

### 准备字体文件
`disk/` 目录的内容会被打包成文件系统镜像，`disk/font/` 下的字体即为板上可用字体，示例默认提供：

| 字体文件 | 说明 |
|---|---|
| `DroidSansFallback_Simplified.ttf` | 默认字体，子集化后约1.0MB，含GB2312全部汉字与数字 |
| `A-SourceHanSansCN-M_Simplified.ttf` | 思源黑体，子集化并转换为TrueType轮廓后约2.0MB |
| `FontSwitchDemoCN.ttf` | 精简测试字体，仅含少量字形，用于验证缺字回退 |

放入 `disk/font/` 的 `.ttf` / `.otf` 文件会在打开字体列表时自动出现，无需修改代码。更换默认字体请修改 `src/font_switch_demo.c` 中的宏：
```c
#define FONT_DEMO_DEFAULT_FONT "/font/DroidSansFallback_Simplified.ttf"
```

准备自定义字体时，请注意本SDK的两条限制：

* **只支持TrueType轮廓**。固件的FreeType仅注册了TrueType驱动，CFF轮廓的OTF字体无法打开（串口会提示 `unsupported outline format`）。可用 [fontTools](https://github.com/fonttools/fonttools) 将其转换为TrueType轮廓。
* **全量中文字体需要子集化**。字体打开后其字形索引表常驻内存，一个五万字形的中文字体仅索引表就要数百KB，小内存板卡无法承受。建议先子集化再放入 `disk/font/`：
  ```
  pip install fonttools
  pyftsubset YourFont.ttf --output-file=YourFont_Simplified.ttf \
      --unicodes=U+0020-007E,U+00A0-00FF,U+2000-206F,U+3000-303F,U+FF00-FFEF \
      --text-file=gb2312.txt --drop-tables+=GSUB,GPOS,GDEF
  ```
  其中 `gb2312.txt` 为需要保留的汉字清单。

### 关键配置项
以下配置已在 `project/proj.conf` 中设置好，移植到自己的工程时需要一并带上：

| 配置项 | 取值 | 说明 |
|---|---|---|
| `CONFIG_LV_USING_FREETYPE_ENGINE` | y | 启用FreeType字体引擎 |
| `CONFIG_FREETYPE_FONT_BPP_8` | y | 字形位图使用8bpp灰度 |
| `CONFIG_LV_FREETYPE_CACHE_FT_FACES` | 4 | 同时打开的字体文件数上限 |
| `CONFIG_LV_FREETYPE_CACHE_FT_SIZES` | 16 | 字号缓存数量上限 |
| `CONFIG_LVSF_FONT_MIN_FREE_HEAP` | 32768 | 创建字体后须保留的系统堆下限，低于此值则拒绝加载 |
| `CONFIG_RT_USING_DFS_ELMFAT` | y | 字体从FAT文件系统读取 |

## 示例输出
如果示例运行成功，您将在串口看到以下输出：
```
(...省略系统初始化信息...)

[lvsf_font] freetype init cache=64000 faces=4 sizes=16
mount fs on flash to root success
[font_demo] start
[font_demo] load: path=/font/DroidSansFallback_Simplified.ttf name=DroidSansFallback_Simplified
[lvsf_font] create ok name=DroidSansFallback_Simplified size=24 font=0x200498ac dsc=0x200498ec
[font_demo] enable/order ok: DroidSansFallback_Simplified
[font_demo] mem demo ready total=343704 used=183540 free=160164 max=183540
[font_demo] ready
```
LCD屏幕显示预览文本、当前字体名与字号，底部为字号加减与字体列表按钮。

点选新字体后，串口会输出切换与回收过程：
```
[font_demo] list select: /font/A-SourceHanSansCN-M_Simplified.ttf
[lvsf_font] create ok name=A-SourceHanSansCN-M_Simplified size=24 ...
[font_demo] enable/order ok: A-SourceHanSansCN-M_Simplified
[font_demo] mem after unload previous font total=343704 used=... free=...
```

## 使用finsh命令

| 命令 | 说明 |
|---|---|
| `lv_example_font_switch_demo` | 重启示例（重建界面，并释放上一轮选中的字体） |
| `font_demo_enable_font` | 启用当前选中的字体并置于优先级首位 |
| `font_demo_verify_font` | 按当前字号创建一次字体对象，用于诊断字体是否可用 |

```{note}
以上命令在finsh线程中只投递请求，真正的界面与字体操作由示例的定时器在LVGL线程中执行——LVGL不是线程安全的，应用中也应遵循同样的做法。
```

## API说明

### lvsf_font_load_ex
```c
int lvsf_font_load_ex(char *font_path, uint16_t *size);
```
注册文件系统中的字体并创建字体对象。`size` 传 `NULL` 时按默认字号创建一个对象以校验字体文件；也可传入以0结尾的字号数组，一次创建多个字号。

**返回值**：成功返回1，失败返回-1（文件打不开、格式不支持或内存不足）。

### lvsf_get_font_by_name
```c
lv_font_t *lvsf_get_font_by_name(char *font_name, int size);
```
按字体名（文件名去掉扩展名）取得指定字号的 `lv_font_t`，可直接设置到LVGL样式上。字号对象按需创建。

**返回值**：成功返回字体指针，失败返回 `NULL`（此时应回退到其它字体）。

### lvsf_font_set_enable / lvsf_font_set_order
```c
int  lvsf_font_set_enable(char *font_name, int enable);
void lvsf_font_set_order(char **font_name, uint16_t font_num);
```
启用/停用字体，并调整按字号取字体时的优先级顺序。

### lvsf_font_unload_ex
```c
int lvsf_font_unload_ex(char *font_path);
```
卸载该路径注册的字体，释放其字体对象与常驻内存。

**参数**：`font_path` 字体文件路径

**返回值**：全部释放返回0；仍有字体在显示、被保留时返回-1

```{warning}
调用前必须先把引用了该字体的样式改指到其它字体（本示例的做法是先把界面交还内置字体，再卸载）。仍在显示的字体会被保留并在串口报错，而不会被释放。
```

## 注意事项
1. **卸载前先改引用**：字体管理器交出的是裸指针，应用把它存进LVGL样式；卸载一个仍在显示的字体等于访问已释放内存。SDK会在释放前检查引用并拒绝释放，但改指仍是应用的责任。
2. **给板子设定字体内存下限**：一个字体打开后其索引表常驻内存。设置 `CONFIG_LVSF_FONT_MIN_FREE_HEAP` 后，创建字体时若剩余堆低于该值会直接失败，应用可回退到其它字体，而不是稍后在无关的分配上崩溃。
3. **字体格式**：仅支持TrueType轮廓，CFF轮廓的OTF需先转换。
4. **文件系统容量**：`disk/` 的总大小受分区表中 `FS_REGION` 的容量限制（默认4MB），超出时镜像打包会失败。
5. **字号与内存**：每个字号都会创建一个字体对象并占用字形缓存，连续浏览大量字号会显著抬高堆水位。

## 异常诊断
* **串口提示 `Default font not found`**：`disk/font/` 中没有默认字体文件，或烧录时未更新 `fs_root.bin`。
* **串口提示 `unsupported outline format: convert CFF/OTF fonts to TrueType`**：字体为CFF轮廓，需转换为TrueType轮廓。
* **串口提示 `leaves less than 32768 bytes of heap, refused`**：字体对本板过大，请先子集化，或改用带PSRAM的板型。
* **切换字体后仍显示旧字体**：检查是否有同名字体（去掉扩展名后重名），管理器会拒绝重复注册并在串口报错。
* **部分字符显示为方框或明显变小**：该字符不在当前字体的字符集中，已回退到内置字体，请在子集化时收入所需字符。

如有其他问题，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [LVGL 文档](https://docs.lvgl.io/)
- [FreeType 文档](https://freetype.org/freetype2/docs/documentation.html)
