# LVGL v8 官方示例

源码路径：`example/multimedia/lvgl/lvgl_v8_examples`

## 支持的平台

- `sf32lb52-lchspi-ulp`
- SF32LB52x LCD 系列开发板
- SF32LB56x LCD 系列开发板
- SF32LB58x LCD 系列开发板

以上平台均可运行基础 LVGL 示例。使用 SJPG 示例从 TF 卡读取图片时，开发板还需具备 SPI TF 卡接口，并完成 SPI、TF 卡和文件系统配置。本文以 `sf32lb52-lchspi-ulp` 为例，其他开发板请替换为对应的板名和硬件配置。

## 介绍

本工程用于运行和验证 LVGL v8 官方示例。示例源码位于 `src/examples`，可调用的示例函数可参考 `src/examples/lv_examples.h`。

`src/main.c` 默认调用 `lv_example_scroll_1()`，用于演示对象在内容超出可视区域时的横向和纵向滚动效果。

## 切换不同演示

打开 `src/main.c`，找到示例函数调用：

```c
lv_example_scroll_1();
// lv_example_grid_1();
```

注释当前函数并取消注释需要运行的函数。例如，切换到 `lv_example_grid_1()`：

```c
// lv_example_scroll_1();
lv_example_grid_1();
```

修改后重新编译并烧录工程。

## 编译和烧录

切换到例程 `project` 目录，运行 SCons 命令进行编译：

```sh
scons --board=sf32lb52-lchspi-ulp -j8
```

执行烧录脚本：

```none
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

按提示输入串口号：

```none
please input the serial port num:5
```

### PC 模拟器

PC 配置位于 `project/pc_hcpu`。先根据本机的 MSVC 安装修改 SDK 根目录下的 `msvc_setup.bat`，再在 `project` 目录执行：

```sh
scons --board=pc_hcpu -j8
```

编译完成后运行 `build_pc_hcpu/main.exe`。

## 可选示例：使用 SJPG 显示图片

本节说明如何切换到 `lv_example_sjpg_1()`，从 TF 卡文件系统读取并显示 `small_image.sjpg`。该示例不是默认显示内容，需要手动完成以下配置并切换示例函数。

### 硬件需求

- 支持平台中带 SPI TF 卡接口的开发板
- 一根支持数据传输的 USB 线
- 一张 TF 卡和一个 TF 读卡器

### 准备图片

将工程中的 `src/examples/libs/sjpg/small_image.sjpg` 复制到 TF 卡文件系统的根目录。示例源码使用以下路径读取图片：

```c
lv_img_set_src(wp, "A:small_image.sjpg");
```

### 配置工程

在 `project` 目录执行：

```sh
sdk.py menuconfig --board=sf32lb52-lchspi-ulp
```

确认生成的配置包含以下选项：

```ini
CONFIG_RT_USING_SPI_MSD=y
CONFIG_RT_USING_DFS_ELMFAT=y
CONFIG_LV_USE_FS_POSIX=y
CONFIG_LV_FS_POSIX_LETTER=65
CONFIG_LV_USE_SJPG=y
```

`CONFIG_LV_FS_POSIX_LETTER=65` 表示 LVGL 文件系统使用盘符 `A`。

具体配置项如下：

1. 启用 SPI 总线。

   ![启用 SPI 总线](assets/V8_SPI.png)

2. 将 SD/TF 设备挂载到 SPI 总线。

   ![配置 TF 卡设备](assets/V8_tf.png)

3. 启用 ELM FAT 文件系统。

   ![配置 ELM FAT 文件系统](assets/V8_elm.png)

4. 启用 LVGL POSIX 文件系统接口和 SJPG 解码器，并将盘符配置为 `A`。

   ![配置 LVGL 文件系统和 SJPG](assets/V8_posix.png)

### 切换示例

修改 `src/main.c`：

```c
// lv_example_scroll_1();
lv_example_sjpg_1();
```

### 编译和烧录

在 `project` 目录执行：

```sh
scons --board=sf32lb52-lchspi-ulp -j8
```

使用串口烧录：

```none
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

按提示输入串口号：

```none
please input the serial port num:5
```

> **注意：** `src/main.c` 在 TF 卡根逻辑区域挂载失败时会调用 `dfs_mkfs()` 自动格式化该区域，可能清除该逻辑区域中的数据。运行示例前请备份 TF 卡中的重要数据，并确认 TF 卡和 SPI 接口工作正常。

### 运行结果

插入 TF 卡后，日志中出现 `mount fs on flash to root success` 表示文件系统挂载成功。

![文件系统挂载日志](assets/log1.png)

在 Finsh 中输入 `ls` 可查看 TF 卡根目录中的图片文件。

![TF 卡文件列表](assets/log2.png)

图片显示效果如下：

![SJPG 示例效果](assets/demo.jpg)

### 异常诊断

如果出现以下日志，请检查 TF 卡是否插入或松动，以及 TF 卡与 SPI 总线是否可以正常通信。

![异常日志](assets/log3.png)
