# LVGL v9 Official Examples

Source path: `example/multimedia/lvgl/lvgl_v9_examples`

## Supported Platforms

- `sf32lb52-lchspi-ulp`
- SF32LB52x LCD development boards
- SF32LB56x LCD development boards
- SF32LB58x LCD development boards

All platforms listed above can run the standard LVGL examples. The TJPGD file example
additionally requires a board with an SPI-connected TF card slot and the corresponding SPI
bus, TF card device, and file-system configuration. The instructions below use
`sf32lb52-lchspi-ulp`; select the appropriate board name and hardware settings when using
another board.

## Introduction

This project builds and runs the official LVGL v9 examples. The example sources are under
`src/examples`, and `src/examples/lv_examples.h` provides the available example functions.

By default, `src/main.c` calls `lv_example_scroll_1()`. This example demonstrates horizontal
and vertical scrolling when child content extends beyond the visible area.

## Switching Examples

Open `src/main.c` and locate the example calls:

```c
lv_example_scroll_1();
// lv_example_tiny_ttf_1();
// lv_example_file_explorer_1();
// lv_example_tjpgd_1();
```

Comment out the current example and uncomment the one to run. For example, to select
`lv_example_tiny_ttf_1()`:

```c
// lv_example_scroll_1();
lv_example_tiny_ttf_1();
// lv_example_file_explorer_1();
// lv_example_tjpgd_1();
```

Rebuild and flash the project after changing the example.

## Build and Flash

Change to the example's `project` directory and run the SCons build:

```sh
scons --board=sf32lb52-lchspi-ulp -j8
```

Run the UART flashing script:

```none
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

Enter the serial port number when prompted:

```none
please input the serial port num:5
```

### PC Simulator

The PC configuration is under `project/pc_hcpu`. First update `msvc_setup.bat` in the SDK root
to match the local MSVC installation, then run the following command from `project`:

```sh
scons --board=pc_hcpu -j8
```

When the build completes, run `build_pc_hcpu/main.exe`.

## Optional Example: Display a JPG from a TF Card with TJPGD

This section switches the project to `lv_example_tjpgd_1()`, which loads and displays
`flower.jpg` from a TF card. This is not the default example, so configure the required
features and select the example function as described below.

### Hardware Requirements

- A supported board with an SPI-connected TF card interface
- A USB cable that supports data transfer
- A TF card and a card reader

### Preparing the Image

Copy `src/examples/libs/libjpeg_turbo/flower.jpg` to the root of the TF card file system. The
example opens the image from the following path:

```c
lv_image_set_src(wp, "A:flower.jpg");
```

### Configuring the Project

Run the following command from `project`:

```sh
sdk.py menuconfig --board=sf32lb52-lchspi-ulp
```

Confirm that the resulting configuration contains:

```ini
CONFIG_RT_USING_SPI_MSD=y
CONFIG_RT_USING_DFS_ELMFAT=y
CONFIG_LV_USE_FS_POSIX=y
CONFIG_LV_FS_POSIX_LETTER=65
CONFIG_LV_USE_TJPGD=y
# CONFIG_LV_USE_FS_MEMFS is not set
```

`CONFIG_LV_FS_POSIX_LETTER=65` assigns drive letter `A` to the LVGL file-system driver. Keep
`LV_USE_FS_MEMFS` disabled when running the file-backed `lv_example_tjpgd_1()`. Enabling it
selects the memory-backed example described later in this document.

Configure the options as follows:

1. Enable the SPI bus.

   ![Enable the SPI bus](assets/V9_SPI.png)

2. Configure the TF card device on the SPI bus.

   ![Configure the TF card device](assets/V9_tf.png)

3. Enable the ELM FAT file system.

   ![Configure the ELM FAT file system](assets/V9_elm.png)

4. Enable the LVGL POSIX file-system interface and TJPGD decoder, then set the drive letter to
   `A`.

   ![Configure the LVGL file system and TJPGD decoder](assets/V9_posix.png)

### Switching the Example

Update `src/main.c`:

```c
// lv_example_scroll_1();
lv_example_tjpgd_1();
```

### Building and Flashing

Run the following command from `project`:

```sh
scons --board=sf32lb52-lchspi-ulp -j8
```

To flash over UART, run:

```none
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

Enter the serial port number when prompted:

```none
please input the serial port num:5
```

> **Warning:** If mounting either the TF card's root file-system region or the `/misc`
> file-system region fails, `src/main.c` calls `dfs_mkfs()` and formats the affected region
> automatically.
> This may erase data in that region. Back up important data before running the example, and
> verify that the TF card and SPI interface are working correctly.

### Expected Output

After inserting the TF card, the log message `mount fs on flash to root success` indicates
that the root file system was mounted successfully.

![File-system mount log](assets/log1.png)

At the Finsh prompt, run `ls` to list the image files in the TF card root directory.

![TF card file list](assets/log2.png)

The displayed image should look like this:

![TJPGD file example output](assets/demo.jpg)

### Troubleshooting

If the following log appears, check that the TF card is inserted securely and can communicate
over the SPI bus.

![Error log](assets/log3.png)

## TJPGD Extension: Display a JPG Embedded in Firmware

This mode embeds the JPEG byte stream in firmware as an `lv_image_dsc_t`. LVGL treats the
descriptor as an `LV_IMAGE_SRC_VARIABLE` source and uses TJPGD to decode the data, so no TF
card is required. The project includes a ready-to-use image resource:

- Source image: `src/examples/libs/tjpgd/img_lvgl_logo.jpg`
- Generated C image asset: `src/examples/libs/tjpgd/ui_image_logo.c`
- Image descriptor: `img_logo`

### Configuring the Project

Open the configuration menu from `project`:

```sh
sdk.py menuconfig --board=sf32lb52-lchspi-ulp
```

Enable TJPGD and MEMFS under the LVGL v9 third-party library options, and assign drive letter
`M` to MEMFS:

```ini
CONFIG_LV_USE_TJPGD=y
CONFIG_LV_USE_FS_MEMFS=y
CONFIG_LV_FS_MEMFS_LETTER=77
```

![Enable LV_USE_FS_MEMFS](assets/use_fs_memfs.png)

`CONFIG_LV_FS_MEMFS_LETTER=77` assigns drive letter `M` to MEMFS. The POSIX file-system driver
already uses `A`, so the two drivers must not use the same letter.

This mode does not use a TF card. If no other feature requires TF card storage, disable
`CONFIG_RT_USING_SPI_MSD` so that `src/main.c` does not run its TF card mount and automatic
formatting path.

### Declaring and Calling the Memory-Backed Example

When `LV_USE_FS_MEMFS` is enabled, `src/examples/libs/tjpgd/lv_example_tjpgd_1.c` builds
`lv_example_tjpgd_2()` instead of `lv_example_tjpgd_1()`. Add its declaration to
`src/examples/libs/tjpgd/lv_example_tjpgd.h`:

```c
void lv_example_tjpgd_1(void);
void lv_example_tjpgd_2(void);
```

Then update `src/main.c` to call the memory-backed example:

```c
// lv_example_scroll_1();
// lv_example_tjpgd_1();
lv_example_tjpgd_2();
```

Rebuild and flash the project as described in the "Build and Flash" section. The display will
show `img_logo` from `ui_image_logo.c`.

### Replacing the Image

1. Use EEZ Studio, a third-party LVGL image-conversion tool, to export a JPG as a C source file
   with the `RAW` color format. Note the generated image descriptor name.

   ![Select the JPG file in EEZ Studio](assets/jpg_awitch_RAW.png)

   ![Export the image in RAW format](assets/build_raw.png)

2. Copy the generated `.c` file to `src/examples/libs/tjpgd`.

   ![Copy the generated C file](assets/copy_raw.png)

3. In `src/examples/libs/tjpgd/lv_example_tjpgd_1.c`, replace `img_logo` with the generated
   image descriptor name:

```c
LV_IMG_DECLARE(your_image);

void lv_example_tjpgd_2(void)
{
    lv_obj_t * wp = lv_image_create(lv_screen_active());
    lv_image_set_src(wp, &your_image);
    lv_obj_center(wp);
}
```

Rebuild and flash the project. The example will display the replacement image.
