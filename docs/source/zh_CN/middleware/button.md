# 按键组件
源码路径example\button\src[butto示例](example-button)
按钮库使用引脚设备来检测各种按钮动作。 它使用一个计时器来处理去抖动，而另一个计时器来检测长按动作。

它支持以下操作
- 按下：按钮被按下
- 释放：按钮被释放
- 长按：按钮被按下并保持由 `BUTTON_ADV_ACTION_CHECK_DELAY` 指定的时间
- 点击：按钮被按下和释放，中间不会触发长按动作

用户可以注册每个按钮的处理程序以在发生任何操作时获得通知。 按钮活动状态是可配置的。

## 配置
支持的最大按键数和长按动作检测阈值由以下宏配置
```c
#define BUTTON_MAX_NUM 2
#define BUTTON_ADV_ACTION_CHECK_DELAY 3000
```

它们可以在 `SiFli Middleware->Enable button library` 下配置。
可以在编译界面输入`menuconfig`进入
![图 1：按钮库配置菜单](../../assets/button_menu.png)

## 使用按钮库
在下面的示例中，它首先初始化按钮，即将按钮与 pin 设备绑定并注册按钮动作处理程序。 然后启用按钮检测，即启用引脚 IRQ。




## 主体代码部分
```c
static void button_event_handler(int32_t pin, button_action_t action)
{
    printf("button:%d,%d\n", pin, action);
}



button_cfg_t cfg;

#if defined(BSP_USING_BOARD_EM_LB525XXX)
    cfg.pin = 34;
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    cfg.pin = 152;
#endif
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    int32_t id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));


#if defined(BSP_USING_BOARD_EM_LB525XXX)
    cfg.pin = 11;
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    cfg.pin = 150;
#endif
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT;
    cfg.button_handler = button_event_handler;
    id = button_init(&cfg);
    RT_ASSERT(id >= 0);
    RT_ASSERT(SF_EOK == button_enable(id));
```
## 按键引脚模式设置

```c
#if defined(BSP_USING_BOARD_EM_LB525XXX)
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLDOWN, 1);
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    HAL_PIN_Set(PAD_PB56, GPIO_B56, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PB54, GPIO_B54, PIN_PULLDOWN, 1);
#endif

```

## API参考
[](middleware-button)
