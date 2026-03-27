# GPADC button
Source Path: example/misc/adc_button

## Supported Platforms
* sf32-oed-epd_v11

## Overview
GPADC (General Purpose ADC) button is a technology that realizes multi-key
detection through ADC sampling, and its core principle is to use a resistive
voltage divider network to convert the state of different keys into different
voltage values, and then identify specific key actions through ADC sampling.
GPADC keys significantly reduce pin footprint compared to traditional standalone
GPIO keys (one ADC pin can support multiple keys), especially for embedded
devices with tight pin resources. This document describes the GPADC buttons,
which can detect the key state through ADC, support two basic operations: click
and long-press, and can be mapped to specific operation actions (up, down,
select, and exit).

## Explanation of principle and function
- When a key is pressed, the corresponding resistor branch is turned on with
  GND, and the voltage of the ADC pin is determined by the resistance of the
  conduction branch and the pull-up resistor (or series resistor). For example:

When key 1 is pressed, ADC voltage = VCC × (R1 parallel other resistors) / total
resistance;\
When key 2 is pressed, ADC voltage = VCC × (R2 parallel other resistors) / total
resistance;\
When different keys are pressed, the ADC pins produce a unique and
distinguishable voltage value.

By properly designing the resistance value, it is possible to ensure that the
voltage value range corresponding to each button does not overlap, so that the
specific button can be identified by the ADC sample value.

- This routine mainly implements the following functions:
1. Initialize the GPADC button hardware and driver configuration;
2. Recognize two basic key actions: short click and long press;
3. Mapping into four actions: UP, DOWN, SELECT, and EXIT;
4. Support business logic processing of custom key actions.

## Use of routines
### Compilation and burning
The hardware mainly uses the development board (SF32-OED-6'-EPD_V1.1)

#### Program compilation and burning
Switch to the routine project directory and run the scons command to perform the
compilation ('.. 'Add the sf32-oed-epd_v11 storage path for the development
board):
```
scons --board=sf32-oed-epd_v11 --board_search_path=.. -j8
```
run`build_sf32-oed-epd_hcpu\uart_download.bat`，Follow the prompts to select a
port to download：
```
build_sf32l-oed-epd_hcpu\uart_download.bat
Uart Download
please input the serial port num: 5  (Fill in the appropriate port number)
```
This is only a brief explanation, please check for details[Compile the burning
link](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/build.html)

#### menuconfig configuration
Run the menuconfig command to perform the compilation ('..'Add the
sf32-oed-epd_v11 storage path for the development board):
```
menuconfig --board=sf32-oed-epd_v11 --board_search_path=..
```
- If you want to change the time of the long press, you can set it by modifying
  the macro 'BUTTON_ADV_ACTION_CHECK_DELAY' in Menuconfig. Open the Menuconfig
  configuration under 'SiFli Middleware->Enable button library'. ![alt
  text](assets/menuconfig_action.png)
#### Configuration process
- First, check whether the hardware supports the use of GPADC buttons, as shown
  in the hardware schematic diagram below, mark the voltage range and reference
  voltage of the GPADC buttons, for example, set the reference voltage under the
  directory of the board according to the figure below (take the middle value of
  the voltage range)`CONFIG_ADC_BUTTON_GROUP1_BUTTON1_VOLT=2992`、
  `CONFIG_ADC_BUTTON_GROUP1_BUTTON2_VOLT=2345`, and the amplitude of the voltage
  range for each ADC button (take half of the voltage
  range)`CONFIG_ADC_BUTTON_GROUP1_BUTTON1_RANGE=315`、`CONFIG_ADC_BUTTON_GROUP1_BUTTON2_RANGE=245`。
  ![alt text](assets/adc.png)

## Software design
1. ADC key event processing function to parse key actions and trigger
   corresponding callbacks. First, determine whether the key action type is a
   click or a long press, then determine which button is based on the key index
   (pin value), and finally trigger the corresponding operation action by
   calling the global callback function action_cbk, so as to realize the mapping
   and transmission of key hardware events to business logic.
```c
static void adc_button_handler(uint8_t group_idx, int32_t pin, button_action_t action)
{
    / Handling click events (short press)
    if (action == BUTTON_CLICKED) {
        if (pin == 0) {
            rt_kprintf("The first key clicks → up\n");
            if (action_cbk) action_cbk(Action_UP);
        } else if (pin == 1) {
            rt_kprintf("The second button clicks → down\n");
            if (action_cbk) action_cbk(Action_DOWN);
        }
    }
    / Handle long press events
    else if (action == BUTTON_LONG_PRESSED) {
        if (pin == 0) {
            rt_kprintf("Press and hold the first button → to select\n");
            if (action_cbk) action_cbk(Action_SELECT);
        } else if (pin == 1) {
            rt_kprintf("Press and hold the second button → to exit\n");
            if (action_cbk) action_cbk(Action_EXIT);
        }
    }
}
```
2. Create and initialize a key controller, complete the key hardware
   initialization, and bind the event handling function. Create a ButtonControls
   instance by allocating memory, configure the keystroke pins, activation
   level, mode, and other parameters, invoke button_init initialize the key
   driver, bind the ADC key processing function to the corresponding button,
   enable the key detection function, and save the callback function and key ID
   to complete the creation and initialization of the button controller.
```c
static ButtonControls* button_controls_create(ActionCallback_t on_action)
{
    ButtonControls *ctrl = (ButtonControls*)malloc(sizeof(ButtonControls));
    if (!ctrl) {
        rt_kprintf("Memory allocation failed\n");
        return NULL;
    }
    int32_t id;
    button_cfg_t cfg; 
    // Initialize the key configuration
    cfg.pin = EPD_KEY_GPADC; 
    cfg.active_state = BUTTON_ACTIVE_HIGH;
    cfg.mode = PIN_MODE_INPUT; 
    cfg.button_handler = dummy_button_event_handler;

    //...
}
```

3. Action processing functions that execute specific business logic based on key
   actions. As a logical entry point, specific functions can be implemented here
   according to actual needs.
```c
static void handle_action(ActionType action) {
    switch(action) {
        case Action_UP:
            // Add the specific code for the upward action here
            break;
        case Action_DOWN:
            // Add the specific code for the downward action here
            break;
        case Action_SELECT:
            // Add the specific code for the selection action here
            break;
        case Action_EXIT:
            // Add the specific code for the exit action here
            break;
        default:
            break;
    }
}
```

#### Troubleshooting
| problem                     | possible cause                      | Workaround                                                                                                      |
| --------------------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| Button unresponsive         | pin definition error                | Check if the 'EPD_KEY_GPADC' definition is correct                                                              |
| Button unresponsive         | driver not enabled                  | Confirm that 'RT_USING_BUTTON' and 'USING_ADC_BUTTON'                                                           |
| Key recognition error       | ADC sample value interval overlap   | Adjust the hardware voltage divider resistor to increase the difference in sampling values of different buttons |
| Long press does not trigger | Long press time setting is too long | Reduce the 'BUTTON_ADV_ACTION_CHECK_DELAY' value                                                                |
| Frequent false triggers     | Insufficient debrating time         | Increased key debounce time (adjusted in button driver)                                                         |
| initialization failure      | insufficient memory                 | Check the system memory usage and adjust the thread stack size                                                  |

#### Routine output results display:
You can see the successful initialization of the button controller, as well as
the printing effect of the button presses by short and long presses, as well as
the current voltage value.
```
Button controller initialized successfully, ID:0
msh /&gt;adc control origin data 3194, Voltage 24863
Second button clicked → Down
adc control origin data 3853, Voltage 31785
First button clicked → Up
adc control origin data 3194, Voltage 24863
Second button long pressed → Exit
adc control origin data 3853, Voltage 31785
First button long pressed → Select
```


## Update the record
| version | date    | release notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 08/2025 | initial version |
``
