# 基准测试演示

## 概述

基准测试演示在各种情况下测试性能。
例如矩形、边框、阴影、文本、图像混合、图像变换、混合模式等。
所有测试都使用50%透明度重复进行。

测试期间对象的大小和位置使用伪随机数设置，以使基准测试可重复。

在屏幕顶部显示当前测试步骤的标题和上一步的结果。

## 运行基准测试
- 在 `lv_conf.h` 或等效位置设置 `LV_USE_DEMO_BENCHMARK 1`
- 在 `lv_init()` 和初始化驱动程序后调用 `lv_demo_benchmark()`
- 如果您只想出于任何目的运行特定场景（例如调试、性能优化等），可以调用 `lv_demo_benchmark_run_scene()` 而不是 `lv_demo_benchmark()` 并传递场景编号。
- 如果您通过将宏 `LV_USE_LOG` 设置为 `1` 并将跟踪级别 `LV_LOG_LEVEL` 设置为 `LV_LOG_LEVEL_USER` 或更高来启用跟踪输出，基准测试结果将以 `csv` 格式打印出来。
- 如果您想知道测试何时完成，可以在调用 `lv_demo_benchmark()` 或 `lv_demo_benchmark_run_scene()` 之前通过 `lv_demo_benchmark_register_finished_handler()` 注册回调函数。
- 如果您想知道系统的最大渲染性能，请在 `lv_demo_benchmark()` 之前调用 `lv_demo_benchmark_set_max_speed(true)`。

## 解释结果

FPS 的测量方式如下：
- 加载下一步
- 在显示驱动程序的 `monitor_cb` 中累积渲染时间和周期数
- 测量1秒
- 计算 `FPS = time_sum / render_cnt`

请注意，对于简单情况，它可能产生非常高的FPS结果。
例如，如果一些简单的矩形在5毫秒内绘制，基准测试将显示它是200 FPS。
因此它忽略了 `LV_DISP_REFR_PERIOD`，该参数告诉LVGL应该多久刷新一次屏幕。
换句话说，基准测试显示的是纯渲染时间的FPS。

默认情况下，只刷新更改的区域。这意味着如果在1毫秒内只有几个像素发生变化，基准测试将显示1000 FPS。要测量全屏刷新的性能，请在 `lv_demo_benchmark.c` 中的 `monitor_cb()` 中取消注释 `lv_obj_invalidate(lv_scr_act())`。

![LVGL基准测试运行](screenshot1.png)

如果您正在进行2D图像处理优化的性能分析，`disp_flush()` 引入的LCD延迟（将数据刷新到LCD）可能会稀释LVGL绘制过程的性能结果，因此更难看到您的优化结果（增益或损失）。为避免此类问题，请：

1. 在 `disp_flush()` 内部使用标志控制LCD刷新。例如：

```c
volatile bool disp_flush_enabled = true;

/* 当LVGL调用disp_flush()时启用屏幕更新（刷新过程）
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* 当LVGL调用disp_flush()时禁用屏幕更新（刷新过程）
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        GLCD_DrawBitmap(area->x1,                   //!< x
                        area->y1,                   //!< y
                        area->x2 - area->x1 + 1,    //!< width
                        area->y2 - area->y1 + 1,    //!< height
                        (const uint8_t *)color_p);
    }

    /*重要！！！
     *通知图形库您已完成刷新*/
    lv_disp_flush_ready(disp_drv);
}
```

2. 在调用 `lv_demo_benchmark()` 或 `lv_demo_benchmark_run_scene()` 之前禁用刷新，例如：

```c
extern void disp_enable_update(void);
extern void disp_disable_update(void);

static void on_benchmark_finished(void)
{
    disp_enable_update();
}

int main(void)
{    
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    LV_LOG("运行LVGL基准测试...");
    LV_LOG("请稍候...");
    LV_LOG("注意：在结束之前您将看不到任何内容。");

    disp_disable_update();
    
    lv_demo_benchmark_set_finished_cb(&on_benchmark_finished);
    lv_demo_benchmark_set_max_speed(true);
    lv_demo_benchmark();
    
    //lv_demo_benchmark_run_scene(43);      // 运行场景编号31

    ...
    while(1){
        lv_timer_handler();                 //! 以最大速度运行lv任务
    }
}
```



3. 或者，您可以通过以下方式使用跟踪输出来获得csv格式的基准测试结果：
   - 将宏 `LV_USE_LOG` 设置为 `1`
   - 将跟踪级别 `LV_LOG_LEVEL` 设置为 `LV_LOG_LEVEL_USER` 或更高。




## 结果摘要
最后，创建一个表格来显示测量的FPS值。

在摘要屏幕顶部，显示"加权FPS"值。
其中，更常见情况的结果以更高的权重考虑。

"透明度速度"显示使用透明度测量的速度与完全不透明相比。
例如，"透明度速度 = 90%"意味着使用透明度渲染慢10%。

在表格的第一部分"慢但常见情况"中，显示那些被认为是常见但慢于20 FPS的情况。

在此下方的"所有情况部分"中显示所有结果。小于10 FPS的结果用红色显示，大于等于10但小于20 FPS的值用橙色显示。


![LVGL基准测试结果摘要](https://github.com/lvgl/lvgl/tree/master/demos/benchmark/screenshot2.png?raw=true)
