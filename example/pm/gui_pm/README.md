# GUI_PM 低功耗示例说明

源码路径: example/pm/gui_pm

## 支持的平台
例程可以运行在以下开发板
* sf32lb52-lchspi-ulp

## 背景问题
- 启用低功耗相关宏开关后，主核（HCPU）迟迟无法进入睡眠模式。
- 典型场景发生在应用持续刷新屏幕时，LCD 的刷新线程持续活跃导致睡眠条件被破坏。

发生这种问题可以排查当前程序是否满足睡眠条件：
1. **没有禁止睡眠模式**：不存在主动阻止睡眠的逻辑或标志位，例如调用了 `rt_pm_request(PM_SLEEP_MODE_IDLE);`。可以通过在console中发送命令pm_dump，出现如下图信息，如果 Idle Mode 的 Counter 大于 0，则表示有模块调用`rt_pm_request(PM_SLEEP_MODE_IDLE)`禁止了睡眠。如果为 0，则表示没有禁止睡眠。
![alt text](assets/sleep.png)

2. **操作系统超时门限满足**：系统内最近一个将要到期的定时器剩余时间大于门限（默认 100 ms）。
3. **无唤醒源被触发**：已配置的唤醒源保持未激活状态，例如按键、传感器中断等。
4. **小核消息队列清空**：HCPU 发送给 LCPU 的数据已被小核读取处理。

当其中任意一条未满足时，系统都会停留在活跃态，无法进入睡眠。

## 示例概述
本示例基于 `gui_app_pm` 的接口，在 GUI 线程空闲时主动让出资源、关闭屏幕背光，进而满足睡眠条件，实现如下目标：
- 在检测到显示层长时间无用户输入后，通过调用 `gui_pm_fsm(GUI_PM_ACTION_SLEEP)` 将状态从 `GUI_STATE_ACTIVE` 变更为 `GUI_STATE_INACTIVE_PENDING`
并发出`GUI_PM_EVT_SUSPEND `事件通知， 进入休眠流程。

- 程序原本因为频繁刷新的线程导致没有满足：`操作系统超时门限满足：系统内最近一个将要到期的定时器剩余时间大于门限（默认 100 ms）`这个要求。当调用 `gui_suspend()` 之后使得频繁刷新的线程被挂起，从而满足了睡眠要求进入休眠。

- 唤醒源（按键）触发后，调用 `gui_pm_fsm(GUI_PM_ACTION_WAKEUP)` ：如果当前状态为 `GUI_STATE_INACTIVE` 或 `GUI_STATE_INACTIVE_PENDING`，则转换到 `GUI_STATE_ACTIVE` 并发出 `GUI_PM_EVT_RESUME `事件通知，恢复被挂起的线程继续运行并重新开启 LCD 显示。

## 关键代码说明
- `lcd_display_task()`：周期检测 LVGL 的 inactivity 时间，超过 5 s 时进入 GUI 休眠逻辑；唤醒后触发重绘以恢复显示内容。
- `gui_is_force_close()`：判断当前 GUI 是否需要强制关闭，判断逻辑为：当前 GUI 状态是否为 `GUI_STATE_INACTIVE_PENDIN`。取决于是否有执行`gui_pm_fsm(GUI_PM_ACTION_SLEEP)`操作。

> **重点**：主要核心代码在 lcd_display_task 中，主要职责包括监控用户活动时间并执行相应休眠逻辑处理。若希望复用示例中的休眠逻辑，请重点阅读 `src/main.c` 中被 `#ifdef GUI_APP_PM` 包裹的代码段。
核心流程如下：
```c
if (lv_display_get_inactive_time(NULL) > 5000)
{
	gui_pm_fsm(GUI_PM_ACTION_SLEEP);
}
if (gui_is_force_close())
{
	gui_suspend();               // 挂起当前 GUI 线程、关闭屏幕
	draw(lcd_device);            // 重新刷底色
	lv_obj_invalidate(lv_screen_active());
	lv_display_trigger_activity(NULL);
}
```

## 编译和烧录
### 以sf32lb52-lchspi-ulp_hcpu工程代码为例编译
52平台默认配置为Deep Sleep模式休眠<br>
* 编译：
    切换到例程project目录，运行scons命令执行编译：

> scons --board=sf32lb52-lchspi-ulp -j8
* 烧入：
    1. 切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
    2. 切换到例程project目录，运行： build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
    
```c
 build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat

 Uart Download

 please input the serial port num:5
```
## 例程输出结果

![alt text](assets/log.png)

* 运行时板子现象
![alt text](assets/open.png)
* 进入睡眠时板子现象
![alt text](assets/off.png)
* 进入睡眠时功耗数据
![alt text](assets/sju.png)
