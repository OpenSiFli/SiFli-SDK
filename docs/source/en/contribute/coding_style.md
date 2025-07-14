
# Coding Style (Non-HAL Modules)
This is a development guideline for SiFli-SDK developers. Since SiFli-SDK is based on RT-Thread, the coding style for modules other than HAL is consistent with that of RT-Thread.

## 1. Directory Names

Directory names should be in all lowercase unless there is a special requirement. The directory name should reflect the content, such as chip-specific directories named after the chip or chip category, and components directories should reflect the purpose of the component.

## 2. File Names

File names should be in all lowercase unless there is a special requirement (if referencing files from elsewhere, the original name may be kept). To avoid name conflicts, avoid using common or frequently used names.

Device driver source files: Naming convention follows `drv_class.c`, for example:

- drv_spi.c
- drv_gpio.c

## 3. Header File Definitions

To avoid multiple inclusions of C language header files, a symbol must be defined. The format for defining this symbol should follow this style:

```c
    #ifndef __FILE_H__
    #define __FILE_H__
    /* header file content */
    #endif
```

Symbols should be wrapped with "__" to avoid conflicts. If the file name contains multiple words, they can be connected with underscores.

## 4. Variables
Variable names should use lowercase letters and "_" to connect them. To avoid name conflicts, global variables should start with the module name, for example, `pm_reg_ctx` is a global variable for the `pm` module. Static variables can omit the module name and should preferably begin with `s_`, for example, `static bool s_freq_scaling_enabled = true;`

## 5. Enum Definitions
Enum values should use uppercase, and enum types should be in lowercase with "_" connecting the words.

```c
enum rt_object_class_type
{
    RT_Object_Class_Null   = 0,                         /**< The object is not used. */
    RT_Object_Class_Thread,                             /**< The object is a thread. */
    RT_Object_Class_Semaphore,                          /**< The object is a semaphore. */
    RT_Object_Class_Mutex,                              /**< The object is a mutex. */
    RT_Object_Class_Event,                              /**< The object is an event. */
    RT_Object_Class_MailBox,                            /**< The object is a mail box. */
    RT_Object_Class_MessageQueue,                       /**< The object is a message queue. */
    RT_Object_Class_MemHeap,                            /**< The object is a memory heap */
    RT_Object_Class_MemPool,                            /**< The object is a memory pool. */
    RT_Object_Class_Device,                             /**< The object is a device */
    RT_Object_Class_Timer,                              /**< The object is a timer. */
    RT_Object_Class_Module,                             /**< The object is a module. */
    RT_Object_Class_Unknown,                            /**< The object is unknown. */
    RT_Object_Class_Static = 0x80                       /**< The object is a static object. */
};
```

## 6. Structure Definitions

Structure names should be in lowercase letters, with words connected by "_". For example:

```c
    struct rt_list_node
    {
        struct rt_list_node *next;
        struct rt_list_node *prev;
    };
```

Where **"{" and "}"** should be placed on separate lines, and subsequent member definitions should be indented.

Type definitions should use the structure or enum name followed by "_t", such as:

```c
    typedef struct rt_list_node rt_list_t;
```

## 7. Macro Definitions

Macro definitions should use uppercase letters with words connected by "_". For example:

```c
    #define RT_TRUE                         1
```

## 8. Function Names and Declarations

Function names should be in lowercase letters, with words connected by "_". APIs provided for upper-level applications must be declared in the corresponding header file. If the function's entry parameter is empty, `void` should be used as the entry parameter.

```c
rt_thread_t rt_thread_self(void);
```

Internal static functions should be named with an underscore prefix, such as `_class_method`, without the `_rt_` prefix, for example:

```c
/* IPC object init */
static rt_err_t _ipc_object_init()

/* UART driver ops */
static rt_err_t _uart_configure()
static rt_err_t _uart_control()                    
```

Functions calling the device registration interface should follow the naming pattern `rt_hw_class_init()`, for example:

```c
int rt_hw_uart_init(void)
int rt_hw_spi_init(void)
```

Function names should follow the module name + verb pattern. For example, in the `rt_timer` module, function names are composed of the module name `rt_timer` + verb phrase:

```c
    struct rt_timer
    {
        struct rt_object parent;
        /* other fields */
    };
    typedef struct rt_timer* rt_timer_t;
```

```c
rt_timer_t rt_timer_create(const char* name,
                           void (*timeout)(void* parameter), 
                           void* parameter,
                           rt_tick_t time, rt_uint8_t flag);
rt_err_t rt_timer_delete(rt_timer_t timer);
rt_err_t rt_timer_start(rt_timer_t timer);
rt_err_t rt_timer_stop(rt_timer_t timer);
```

## 9. Commenting

Please use **English** for comments. Writing comments in Chinese would require constant switching between input languages, disrupting coding flow. Using English comments also facilitates better communication with developers outside China.

**Statement Comments**:

Comments in the source code should not be excessive. Most comments should explain what the code does, and only complex algorithms should have explanatory comments on how they work. Comments should only appear **above or to the right** of the statement, any other positions are invalid.

```c
/* Your English comment */
```

**Function Comments**:

Use the [doxygen style](https://www.doxygen.nl/manual/commands.html), starting with `/**` and ending with `*/`, with the function comment in between. The structure should align the description elements, and each element should be separated by a blank line. Example:

```C
/**
 * @brief    The function will initialize a static event object.
 *
 * @note     For the static event object, its memory space is allocated by the compiler during compiling,
 *           and shall be placed on the read-write data segment or on the uninitialized data segment.
 *           By contrast, the rt_event_create() function will allocate memory space automatically
 *           and initialize the event.
 *
 * @see      rt_event_create()
 *
 * @param    event is a pointer to the event to initialize. It is assumed that storage for the event
 *           will be allocated in your application.
 *
 * @param    name is a pointer to the name that given to the event.
 *
 * @param    value is the initial value for the event.
 *           If you want to share resources, initialize the value to the number of available resources.
 *           If you want to signal the occurrence of an event, initialize the value to 0.
 *
 * @param    flag is the event flag, which determines the queuing method of how multiple threads wait
 *           when the event is not available.
 *
 * @return   Return the operation status. When the return value is RT_EOK, the initialization is successful.
 *           Any other value indicates initialization failure.
 *
 * @warning  This function can ONLY be called from threads.
 */
rt_err_t rt_event_init(rt_event_t event, const char *name, rt_uint8_t flag)
{
   ... 
}
```

### Structure Members and Enum Value Comments
For alignment purposes, structure member and enum value comments should be placed on the right side of the code, also following the [doxygen style](https://www.doxygen.nl/manual/commands.html), for example:

```c
enum rt_object_class_type
{
    RT_Object_Class_Null   = 0,                         /**< The object is not used. */
    RT_Object_Class_Thread,                             /**< The object is a thread. */
    RT_Object_Class_Semaphore,                          /**< The object is a semaphore. */
    RT_Object_Class_Mutex,                              /**< The object is a mutex. */
    RT_Object_Class_Event,                              /**< The object is an event. */
    RT_Object_Class_MailBox,                            /**< The object is a mail box. */
    RT_Object_Class_MessageQueue,                       /**< The object is a message queue. */
    RT_Object_Class_MemHeap,                            /**< The object is a memory heap */
    RT_Object_Class_MemPool,                            /**< The object is a memory pool. */
    RT_Object_Class_Device,                             /**< The object is a device */
    RT_Object_Class_Timer,                              /**< The object is a timer. */
    RT_Object_Class_Module,                             /**< The object is a module. */
    RT_Object_Class_Unknown,                            /**< The object is unknown. */
    RT_Object_Class_Static = 0x80                       /**< The object is a static object. */
};
```

## 10. Indentation and Line Breaks

Indentation should use 4 spaces. If there is no special meaning, please break the line after "{" and indent subsequent lines, for example:

```c
    if (condition)
    {
        /* others */
    }
```

The only exception is the `switch` statement, where `switch-case` statements should align with the `switch`, for example:

```c
    switch (value)
    {
    case value1:
        break;
    }
```

Do not use more than one blank line consecutively in the code unless there is a special need.

## 11. Braces and Spaces

From a readability perspective, each brace should occupy its own line, not be placed at the end of a statement:

```c
    if (condition)
    {
        /* others */
    }
```

Matching braces should be on separate lines to provide structure and avoid confusion. A space should be added before parentheses in non-function calls to distinguish them, for example:

```c
    if (x <= y)
    {
        /* others */
    }

    for (index = 0; index < MAX_NUMBER; index++)
    {
        /* others */
    }
```

No spaces should appear inside parentheses of expressions, for example:

```c
    if (x <= y)
    {
        /* other */
    }
```

It is recommended to use the `.clang-format` configuration file from the SDK root directory for code formatting, ensuring consistency in coding style.

## 12. Logging Information

Use `ulog` for logging output, for example:

```c
#define DBG_TAG "Driver"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

LOG_D("this is a debug log.");
```

- In RT-Thread, commonly used log levels are `LOG_D`, `LOG_I`, `LOG_W`, `LOG_E`, and they can be differentiated by `DBG_TAG` and controlled by `DBG_LVL`.
- Logs should be output in a way that is easy to understand and diagnose.
- Do not redefine `DBG_TAG` in header files to prevent uncontrollable issues when other modules include it.
- Avoid excessive logging in timers or interrupts, and keep logs lightweight.
- Avoid using `rt_kprintf` for logging, as it is generally used for terminal command-line interactions.

## 13. Functions

Functions should be as concise as possible, performing only relatively independent, simple tasks. The implementation of a function should not be too long. If a function becomes too lengthy, consider refactoring or splitting it into smaller, more understandable parts.
