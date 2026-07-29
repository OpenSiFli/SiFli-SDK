# LVGL v9 官方示例

源码路径：`example/multimedia/lvgl/lvgl_v9_examples`

## 支持的平台

- `sf32lb52-lchspi-ulp`
- SF32LB52x LCD 系列开发板
- SF32LB56x LCD 系列开发板
- SF32LB58x LCD 系列开发板

以上平台均可运行基础 LVGL 示例。使用 TJPGD 示例从 TF 卡读取图片时，开发板还需具备 SPI TF 卡接口，并完成 SPI、TF 卡和文件系统配置。本文以 `sf32lb52-lchspi-ulp` 为例，其他开发板请替换为对应的板名和硬件配置。

## 介绍

本工程用于运行和验证 LVGL v9 官方示例。示例源码位于 `src/examples`，可调用的示例函数可参考 `src/examples/lv_examples.h`。

`src/main.c` 默认调用 `lv_example_scroll_1()`，用于演示对象在内容超出可视区域时的横向和纵向滚动效果。

## 切换不同演示

打开 `src/main.c`，找到示例函数调用：

```c
lv_example_scroll_1();
// lv_example_tiny_ttf_1();
// lv_example_file_explorer_1();
// lv_example_tjpgd_1();
```

注释当前函数并取消注释需要运行的函数。例如，切换到 `lv_example_tiny_ttf_1()`：

```c
// lv_example_scroll_1();
lv_example_tiny_ttf_1();
// lv_example_file_explorer_1();
// lv_example_tjpgd_1();
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

## 可选示例：使用 TJPGD 显示 TF 卡中的 JPG 图片

本节说明如何切换到 `lv_example_tjpgd_1()`，从 TF 卡文件系统读取并显示 `flower.jpg`。该示例不是默认显示内容，需要手动完成以下配置并切换示例函数。

### 硬件需求

- 支持平台中带 SPI TF 卡接口的开发板
- 一根支持数据传输的 USB 线
- 一张 TF 卡和一个 TF 读卡器

### 准备图片

将工程中的 `src/examples/libs/libjpeg_turbo/flower.jpg` 复制到 TF 卡文件系统的根目录。示例源码使用以下路径读取图片：

```c
lv_image_set_src(wp, "A:flower.jpg");
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
CONFIG_LV_USE_TJPGD=y
# CONFIG_LV_USE_FS_MEMFS is not set
```

`CONFIG_LV_FS_POSIX_LETTER=65` 表示 LVGL 文件系统使用盘符 `A`。运行文件模式的 `lv_example_tjpgd_1()` 时应保持 `LV_USE_FS_MEMFS` 关闭；启用该选项后，源码会切换为后文的内存图片模式。

具体配置项如下：

1. 启用 SPI 总线。

   ![启用 SPI 总线](assets/V9_SPI.png)

2. 将 SD/TF 设备挂载到 SPI 总线。

   ![配置 TF 卡设备](assets/V9_tf.png)

3. 启用 ELM FAT 文件系统。

   ![配置 ELM FAT 文件系统](assets/V9_elm.png)

4. 启用 LVGL POSIX 文件系统接口和 TJPGD 解码器，并将盘符配置为 `A`。

   ![配置 LVGL 文件系统和 TJPGD](assets/V9_posix.png)

### 切换示例

修改 `src/main.c`：

```c
// lv_example_scroll_1();
lv_example_tjpgd_1();
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

> **注意：** `src/main.c` 在 TF 卡的根逻辑区域或 `/misc` 逻辑区域挂载失败时会调用 `dfs_mkfs()` 自动格式化相应区域，可能清除其中的数据。运行示例前请备份 TF 卡中的重要数据，并确认 TF 卡和 SPI 接口工作正常。

### 运行结果

插入 TF 卡后，日志中出现 `mount fs on flash to root success` 表示根文件系统挂载成功。

![文件系统挂载日志](assets/log1.png)

在 Finsh 中输入 `ls` 可查看 TF 卡根目录中的图片文件。

![TF 卡文件列表](assets/log2.png)

图片显示效果如下：

![TJPGD 文件示例效果](assets/demo.jpg)

### 异常诊断

如果出现以下日志，请检查 TF 卡是否插入或松动，以及 TF 卡与 SPI 总线是否可以正常通信。

![异常日志](assets/log3.png)

## TJPGD 扩展：显示固件中的 JPG 图片

该模式将 JPG 数据以 `LV_IMAGE_SRC_VARIABLE` 类型编译到固件中，不需要 TF 卡。工程已经提供可直接使用的图片资源：

- 原始图片：`src/examples/libs/tjpgd/img_lvgl_logo.jpg`
- C 数组：`src/examples/libs/tjpgd/ui_image_logo.c`
- 图片变量：`img_logo`

### 配置工程

在 `project` 目录打开配置界面：

```sh
sdk.py menuconfig --board=sf32lb52-lchspi-ulp
```

在 LVGL v9 的第三方库配置中启用 TJPGD 和 MEMFS，并为 MEMFS 设置盘符 `M`：

```ini
CONFIG_LV_USE_TJPGD=y
CONFIG_LV_USE_FS_MEMFS=y
CONFIG_LV_FS_MEMFS_LETTER=77
```

![启用 LV_USE_FS_MEMFS](assets/use_fs_memfs.png)

`CONFIG_LV_FS_MEMFS_LETTER=77` 表示 MEMFS 使用盘符 `M`。工程中的 POSIX 文件系统已经使用盘符 `A`，不要为两个文件系统配置相同的盘符。

此模式不需要 TF 卡。如果工程不再使用其他 TF 卡功能，可关闭 `CONFIG_RT_USING_SPI_MSD`，避免执行 `src/main.c` 中的 TF 卡挂载和自动格式化流程。

### 声明并调用内存图片示例

启用 `LV_USE_FS_MEMFS` 后，`src/examples/libs/tjpgd/lv_example_tjpgd_1.c` 会编译 `lv_example_tjpgd_2()`。先在 `src/examples/libs/tjpgd/lv_example_tjpgd.h` 中补充函数声明：

```c
void lv_example_tjpgd_1(void);
void lv_example_tjpgd_2(void);
```

再修改 `src/main.c`，调用内存图片示例：

```c
// lv_example_scroll_1();
// lv_example_tjpgd_1();
lv_example_tjpgd_2();
```

完成以上修改后，按“编译和烧录”一节重新编译并烧录，即可显示 `ui_image_logo.c` 中的 `img_logo`。

### 替换为自定义图片

1. 使用 EEZ Studio 将 JPG 图片转换为 LVGL C 数组，颜色格式选择 RAW，并记录生成的图片变量名。

   ![在 EEZ Studio 中选择 JPG 图片](assets/jpg_awitch_RAW.png)

   ![将图片转换为 RAW 数据](assets/build_raw.png)

2. 将生成的 `.c` 文件放到 `src/examples/libs/tjpgd` 目录。

   ![复制生成的 C 文件](assets/copy_raw.png)

3. 在 `src/examples/libs/tjpgd/lv_example_tjpgd_1.c` 中，将 `img_logo` 替换为生成的图片变量名：

```c
LV_IMG_DECLARE(your_image);

void lv_example_tjpgd_2(void)
{
    lv_obj_t * wp = lv_image_create(lv_screen_active());
    lv_image_set_src(wp, &your_image);
    lv_obj_center(wp);
}
```

重新编译并烧录后，示例将显示自定义图片。
