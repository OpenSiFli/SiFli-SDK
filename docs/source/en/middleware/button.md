
# Button Component

Source Path: `example\button\src` [Button Example](example-button)

The button library uses pin devices to detect various button actions. It utilizes one timer to handle debouncing and another timer to detect long press actions.

It supports the following actions:
- **Press**: The button is pressed.
- **Release**: The button is released.
- **Long Press**: The button is pressed and held for a duration specified by `BUTTON_ADV_ACTION_CHECK_DELAY`.
- **Click**: The button is pressed and released without triggering a long press action.

Users can register handlers for each button to be notified when any action occurs. The button activity state is configurable.

## Configuration
The maximum number of buttons supported and the threshold for detecting long press actions are configured by the following macros:

```c
#define BUTTON_MAX_NUM 2
#define BUTTON_ADV_ACTION_CHECK_DELAY 3000
```

These can be configured under `SiFli Middleware -> Enable button library`. You can enter the compilation interface and type `menuconfig` to access it.

![Figure 1: Button Library Configuration Menu](../../assets/button_menu.png)

## Using the Button Library

In the example below, the button is first initialized by binding the button to a pin device and registering a button action handler. Then, button detection is enabled, which involves enabling the pin IRQ.

## Main Code Section

```c
static void button_event_handler(int32_t pin, button_action_t action)
{
    printf("button:%d,%d
", pin, action);
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

## Button Pin Mode Configuration

```c
#if defined(BSP_USING_BOARD_EM_LB525XXX)
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLDOWN, 1);
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    HAL_PIN_Set(PAD_PB56, GPIO_B56, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PB54, GPIO_B54, PIN_PULLDOWN, 1);
#endif
```

## API Reference
[Middleware Button](middleware-button)
