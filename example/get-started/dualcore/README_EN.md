# Dual-Core Comprehensive Example
Source code path: example/get-started/dualcore

## Supported Platforms
This example can run on the following development boards:
* sf32lb52-lcd_n16r8
* eh-lb561
* ec-lb587
* eh-lb523

## Overview
This example demonstrates how to develop dual-core applications. The project supports Bluetooth and low power consumption, builds user interface based on LVGL v8 (referencing `multimedia/lvgl/watch`), with big and little cores running their respective programs (SF32LB52 series chips' little core is dedicated to Bluetooth and does not run user programs).

## Directory Structure
```
.
└── hmi_demo
    ├── project
    │   ├── hcpu    // HCPU project directory
    │   └── lcpu    // LCPU project directory
    └── src
        ├── hcpu    // HCPU code directory
        └── lcpu    // LCPU code directory
```        

## Example Usage
### Compilation and Flashing

Switch to the `project/hcpu` directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:

Executing the `scons` command in the HCPU project directory will automatically compile the LCPU project, and the download script will download all firmware including the little core.

### Code Analysis
#### Big Core

The big core's `main` function is in `src/hcpu/main.c`, completing Bluetooth initialization:
```c
int main(void)
{
    int count = 0;

    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
    env->time_handle  = rt_timer_create("app", app_timeout_handler,  NULL,
                                        rt_tick_from_millisecond(BLE_APP_TIMEOUT_INTERVAL), RT_TIMER_FLAG_SOFT_TIMER);
#ifdef SF32LB52X
    env->rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                              rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
    rt_timer_start(env->rc10k_time_handle);
#endif

    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            ble_app_service_init();
            /* First enable connectable adv then enable non-connectable. */
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}
```

Graphics initialization is triggered by `app_watch_init` in `src/hcpu/gui_apps/watch_demo.c`, which is registered as an `APP` level [automatic initialization function](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/basic/basic?id=rt-thread-%e8%87%aa%e5%8a%a8%e5%88%9d%e5%a7%8b%e5%8c%96%e6%9c%ba%e5%88%b6), executing earlier than the `main` function in `main.c`. In `app_watch_init`, the `app_watch` thread is created, and graphics initialization is actually completed in the `app_watch` thread.

```c
INIT_APP_EXPORT(app_watch_init);
```

#### Little Core
The little core's `main` function is in `src/lcpu/main.c`, as follows:

```c
int main(void)
{
    if (HAL_LXT_DISABLED())
    {
        rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                                            rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
        RT_ASSERT(rc10k_time_handle);
        rt_timer_start(rc10k_time_handle);
    }

    while (1)
    {
        rt_kprintf("main loop\n");
        rt_thread_mdelay(3000);
    }
    return RT_EOK;
}
```

#### Button
If button Key1 is connected to the little core's pin, the little core handles button events and forwards button events to the big core through data service.
The little core's button initialization code is in the `init_pin` function in `src/lcpu/main.c`. At the same time, the little core project configuration `proj.conf` enables `CONFIG_BUTTON_SERVICE_ENABLED`, which registers the button service, and the big core will subscribe to this service to receive button events.

````{note}
Since most boards' little core configuration files (i.e., `board.conf` in the board's lcpu directory) do not define button pins, a `proj.conf` is added for each supported board in the lcpu project directory to define button pin numbers. For example, in the `project/lcpu/eh-lb561_lcpu` directory, `proj.conf` has the following KEY1 configuration parameters, which are effective for the eh-lb561 development board:
```c
CONFIG_BSP_USING_KEY1=y
CONFIG_BSP_KEY1_PIN=128
CONFIG_BSP_KEY1_ACTIVE_HIGH=y
```

````

The big core's button initialization function is `init_pin` in `src/hcpu/gui_apps/watch_demo.c`. You can see that buttons are only initialized in the big core program when the button's corresponding GPIO pin is a PA pin, otherwise it just subscribes to the button data service.
```c
#if (SLEEP_CTRL_PIN < GPIO1_PIN_NUM)
    button_cfg_t cfg;
#if defined(BSP_USING_PM) && !defined(SF32LB52X)
    int8_t wakeup_pin;
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio;
#endif /* BSP_USING_PM && !SF32LB52X */
    
    cfg.pin = SLEEP_CTRL_PIN;
    cfg.active_state = BUTTON_ACTIVE_POL;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
    key1_button_handle = id;

#endif /* SLEEP_CTRL_PIN < GPIO1_PIN_NUM */

    button_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != button_handle);
    datac_subscribe(button_handle, "btn0", button_service_callback, SLEEP_CTRL_PIN);    
```

For SF32LB52 series chips, the little core is dedicated to Bluetooth and does not run user programs. The button is connected to the big core's pin and is directly handled by the big core GPIO interrupt, but button events can still be handled uniformly by subscribing to the button service, where the button service is also located on the big core.

```{note}
SF32LB52 series needs to enable `DATA_SVC_MBOX_THREAD_DISABLED` to disable cross-core service
```

## Expected Results
After power-on, a honeycomb interface appears. Clicking the clock icon opens the watch face. If there is no operation on the interface or button press for 10 seconds, the screen will automatically turn off. At this time, the big core sleeps and enters low power mode. Button (Key1) wakes up the big core and turns on the screen.

## Exception Diagnosis

  
## Reference Documentation