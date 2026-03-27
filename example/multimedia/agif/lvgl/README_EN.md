# AGIF/APNG+LVGL Animation Example

Source Code Path: example/multimedia/agif/lvgl

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb525

## Overview
<!-- 例程简介 -->
This example contains two watch faces with GIF animations and one watch face
with APNG animation, demonstrating animation implementation based on agif/apng +
lvgl, including:
+ .gif conversion to .c using eZIP.exe:
    - Placement location: `src/resource/images/common/gif/`
    - Resource processing: `src/resource/images/SConscript` will compile .gif
      files in the above path. The generated .c files can be found in
      `project/build_xxx/src/resource/images/common/gif` path.
    ```{tip}
    Resources can also be processed manually using `/tools/png2ezip/eZIP.exe`. Run eZIP.exe to view help for command format.
    ```

+ gif display
    - `src/gui_apps/clock/app_clock_agif.c`:
        * Resource declaration:
        ```c
        /* * Image declaration * */
        LV_IMG_DECLARE(agif_icon);
        ```
        * gif widget creation and configuration:
        ```c
        /* * Create animated GIF * */
        lv_color_t bg_color;
        p_clk_agif-&gt;gif = lv_gif_dec_create(parent, LV_EXT_IMG_GET(agif_icon), &amp;bg_color, LV_COLOR_DEPTH);
        RT_ASSERT(p_clk_agif-&gt;gif);
        lv_obj_align(p_clk_agif-&gt;gif, LV_ALIGN_CENTER, 0, 0);

        /* * Looping is enabled by default * */
        lv_gif_dec_loop(p_clk_agif-&gt;gif, 1, 16);
        /* * This callback is executed when GIF playback completes * */
        lv_gif_dec_end_cb_register(p_clk_agif-&gt;gif, agif_loop_end_func);
        ```
        * gif refresh pause and resume:
        ```c
        static rt_int32_t resume_callback(void)
        {
            /* * Resume GIF animation refresh * */
            lv_gif_dec_task_resume(p_clk_agif-&gt;gif);
            return RT_EOK;
        }

        static rt_int32_t pause_callback(void)
        {
            /* * Pause GIF animation refresh * */
            lv_gif_dec_task_pause(p_clk_agif-&gt;gif, 0);
            return RT_EOK;
        }
        ```
        * gif destruction
        ```c
        /* * Release GIF context * */
        lv_gif_dec_destroy(p_clk_agif-&gt;gif);
        p_clk_agif-&gt;gif = NULL;
        ```
    - `src/gui_apps/clock/app_clock_agif_2.c`:\
      `lv_gif_dec_create` automatically creates an lv timer for periodic GIF
      refreshing. This example demonstrates pausing (`lv_gif_dec_task_pause`)
      the automatically created lv timer and creating an external lv timer for
      refreshing. The refresh code is as follows:
        ```c
        static void agif_refresh_timer_cb(struct _lv_timer_t * t)
        {
            /* * Advance to the next frame * */
            int ret = lv_gif_dec_next_frame(p_clk_agif-&gt;gif);

            /* * A return value of 0 indicates the last frame has been reached * */
            if (0 == ret)
            {
                /* * Playback complete * */
                agif_loop_end_func();
                /* * Restart playback */
                lv_gif_dec_restart(p_clk_agif-&gt;gif);
            }
        }
        ```

+ apng resource：
    - Same as PNG, placed in: ` src/resource/images/common/ezip/`
    - APNG image production: can be made online:` https://ezgif.com/ `

+ apng display：
    - `src/gui_apps/clock/app_clock_apng.c`:
        * Resource declaration:
        ```c
        /* * Image declaration * */
        LV_IMG_DECLARE(apng_dice);
        ```
        * apng widget creation and configuration：
        ```c
        /* * Create APNG * */
        lv_obj_t *dice_img = lv_ezipa_create(parent);
        RT_ASSERT(dice_img);
        /* * Set image source * */
        lv_ezipa_set_src(dice_img, apng_dice.data);
        /* * Set surface * */
        // lv_ezipa_set_surface(dice_img, xxx);
        /* * Set interval */
        lv_ezipa_set_interval(dice_img, 60);
        ```
        * apng refresh pause and resume：
        ```c
        static rt_int32_t resume_callback(void)
        {
            /* * Resume APNG animation refresh * */
            lv_ezipa_resume(p_clk_apng-&gt;apng);
            return RT_EOK;
        }

        static rt_int32_t pause_callback(void)
        {
            /* * Pause APNG animation refresh * */
            lv_ezipa_pause(p_clk_apng-&gt;apng);
            return RT_EOK;
        }
        ```

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Running this example requires:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).

### menuconfig Configuration

1. Enable LVGL:\
   ![RTT_LVGL](./assets/agif_cfg_lvgl.png)
2. Enable EPIC/EZIP:\
   ![EPIC](./assets/agif_cfg_epic.png) ![EZIP](./assets/agif_cfg_ezip.png)
3. Enable `USING_EZIPA_DEC` (If apng is used.)：\
   ![EZIPA](./assets/apng_cfg_ezipa_dec.png)
4. Configure the LCD driver according to the LCD used.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
then select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the relevant
introduction in [Quick Start](quick_start).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
+ Defaults to the `agif` watch face, with `agif_icon.gif` refreshing and
  displaying in a loop.
+ Swipe left and right to switch between `aigf` \ `agif02` \ `apng` watch faces.
+ `agif.h` \ `lvsf_ezipa.h` also provides some other control APIs that can be
  modified in the example to see the effects.

## Troubleshooting

+ Compilation error, gif resource not found: As described in [Overview]{1},
  confirm whether the .c file of the gif is generated normally.

## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date     | Release Notes   |
| ------- | -------- | --------------- |
| 0.0.1   | May 2025 | Initial version |
|         |          |                 |
|         |          |                 |
