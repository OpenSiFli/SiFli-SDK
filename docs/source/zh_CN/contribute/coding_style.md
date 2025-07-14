# 编程风格（非HAL模块）
这是一份 SiFli-SDK 开发人员的开发指引，由于SiFli-SDK基于RT-Thread，因此HAL以外的模块的编程风格都与RT-Thread保持一致。

## 1.目录名称

目录名称如果无特殊的需求，请使用全小写的形式；目录名称应能够反映部分的意思，例如各芯片移植由其芯片名称构成或芯片类别构成；components 目录下能够反映组件的意义。

## 2.文件名称

文件名称如果无特殊的需求(如果是引用其他地方，可以保留相应的名称)，请使用全小写的形式。另外为了避免文件名重名的问题，一些地方请尽量不要使用通用化、使用频率高的名称。

设备驱动源码文件：`drv_class.c` 的命名方式，如：

- drv_spi.c
- drv_gpio.c

## 3.头文件定义

C 语言头文件为了避免多次重复包含，需要定义一个符号。这个符号的定义形式请采用如下的风格：

```c
    #ifndef __FILE_H__
    #define __FILE_H__
    /* header file content */
    #endif
```

即定义的符号两侧采用 "__" 以避免重名，另外也可以根据文件名中是否包含多个词语而采用 "_" 连接起来。

## 4.变量
变量名采用小写和"_"连接，为避免名字冲突，全局变量使用模块名开头，例如`pm_reg_ctx`为`pm`模块的全局变量；静态变量可以不加模块名，建议以`s_`开始，例如`static bool s_freq_scaling_enabled = true;`


## 5.枚举定义
枚举值使用大写，枚举类型使用小写和"_"连接

```
enum rt_object_class_type
{
    RT_Object_Class_Null   = 0,                         /**< The object is not used. */
    RT_Object_Class_Thread,                             /**< The object is a thread. */
    RT_Object_Class_Semaphore,                          /**< The object is a semaphore. */
    RT_Object_Class_Mutex,                              /**< The object is a mutex. */
    RT_Object_Class_Event,                              /**< The object is a event. */
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

## 6.结构体定义

结构体名称请使用小写英文名的形式，单词与单词之间采用 "_" 连接，例如：

```c
    struct rt_list_node
    {
        struct rt_list_node *next;
        struct rt_list_node *prev;
    };
```

其中，**"{"，"}" 独立占用一行**，后面的成员定义使用缩进的方式定义。

类型定义请以结构体/枚举名称加上 "_t" 的形式作为名称，例如：

```c
    typedef struct rt_list_node rt_list_t;
```

## 7.宏定义

请使用大写英文名称作为宏定义，单词之间使用 "_" 连接，例如：

```c
    #define RT_TRUE                         1
```

## 8.函数名称、声明

函数名称请使用小写英文的形式，单词之间使用 "_" 连接。提供给上层应用使用的 API接口，必须在相应的头文件中声明；如果函数入口参数是空，必须使用 void 作为入口参数，例如：

```c
rt_thread_t rt_thread_self(void);
```

内部静态函数命名：以下划线开头，使用 `_class_method` 格式，不携带`_rt_`开头，如内核或驱动文件中的函数命名：

```c
/* IPC object init */
static rt_err_t _ipc_object_init()

/* UART driver ops */
static rt_err_t _uart_configure()
static rt_err_t _uart_control()                    
```

调用注册设备接口的函数命名：使用 `rt_hw_class_init()` 格式，举例：

```c
int rt_hw_uart_init(void)
int rt_hw_spi_init(void)
```

函数名称以模块名+动词的形式，以`rt_timer`为例，rt_timer+动词短语构成了timer模块的函数名

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

## 9.注释编写

请使用**英文**做为注释，使用中文注释将意味着在编写代码时需要来回不停的切换中英文输入法从而打断编写代码的思路。并且使用英文注释也能够比较好的与中国以外的技术者进行交流。

**语句注释**：

源代码的注释不应该过多，更多的说明应该是代码做了什么，仅当个别关键点才需要一些相应提示性的注释以解释一段复杂的算法它是如何工作的。对语句的注释只能写在它的**上方或右方**，其他位置都是非法的。

```c
/* 你的英文注释 */
```

**函数注释**：

注释采用[doxygen风格](https://www.doxygen.nl/manual/commands.html)，以 `/**` 开头，以 `  */` 结尾，中间写入函数注释，组成元素如下，每个元素描述之间空一行，且首列对齐：

- @brief + 简述函数作用。在描述中，着重说明该函数的作用，每句话首字母大写，句尾加英文句号。
- @note + 函数说明。在上述简述中未能体现到的函数功能或作用的一些点，可以做解释说明，每句话首字母大写，句尾加英文句号。
- @see + 相关 API 罗列。若有与当前函数相关度较高的 API，可以进行列举。
- @param + 以参数为主语 + be 动词 + 描述，说明参数的意义或来源。
- @return + 枚举返回值 + 返回值的意思，若返回值为数据，则直接介绍数据的功能。
- @warning + 函数使用注意要点。在函数使用时，描述需要注意的事项，如使用环境、使用方式等。每句话首字母大写，句尾加英文句号。

注释模版请参见：rt-thread/src/ipc.c 源码文件，英文注释请参考使用 grammarly 以及谷歌翻译。

```C
/**
 * @brief    The function will initialize a static event object.
 *
 * @note     For the static event object, its memory space is allocated by the compiler during compiling,
 *           and shall placed on the read-write data segment or on the uninitialized data segment.
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
 *           If want to share resources, you should initialize the value as the number of available resources.
 *           If want to signal the occurrence of an event, you should initialize the value as 0.
 *
 * @param    flag is the event flag, which determines the queuing way of how multiple threads wait
 *           when the event is not available.
 *           The event flag can be ONE of the following values:
 *
 *               RT_IPC_FLAG_PRIO          The pending threads will queue in order of priority.
 *
 *               RT_IPC_FLAG_FIFO          The pending threads will queue in the first-in-first-out method
 *                                         (also known as first-come-first-served (FCFS) scheduling strategy).
 *
 *               NOTE: RT_IPC_FLAG_FIFO is a non-real-time scheduling mode. It is strongly recommended to
 *               use RT_IPC_FLAG_PRIO to ensure the thread is real-time UNLESS your applications concern about
 *               the first-in-first-out principle, and you clearly understand that all threads involved in
 *               this event will become non-real-time threads.
 *
 * @return   Return the operation status. When the return value is RT_EOK, the initialization is successful.
 *           If the return value is any other values, it represents the initialization failed.
 *
 * @warning  This function can ONLY be called from threads.
 */
rt_err_t rt_event_init(rt_event_t event, const char *name, rt_uint8_t flag)
{
   ... 
}
```

### 结构体成员和枚举值注释
为了方便对齐，建议将结构体成员和枚举值的注释置于代码右侧，也采用[doxygen风格](https://www.doxygen.nl/manual/commands.html)，如下，
```
enum rt_object_class_type
{
    RT_Object_Class_Null   = 0,                         /**< The object is not used. */
    RT_Object_Class_Thread,                             /**< The object is a thread. */
    RT_Object_Class_Semaphore,                          /**< The object is a semaphore. */
    RT_Object_Class_Mutex,                              /**< The object is a mutex. */
    RT_Object_Class_Event,                              /**< The object is a event. */
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


## 10.缩进及分行

缩进请采用 4 个空格的方式。如果没有什么特殊意义，请在 "{" 后进行分行，并在下一行都采用缩进的方式，例如：

```c
    if (condition)
    {
        /* others */
    }
```

唯一的例外是 switch 语句，switch-case 语句采用 case 语句与 switch 对齐的方式，例如：

```c
    switch (value)
    {
    case value1:
        break;
    }
```

case 语句与前面的 switch 语句对齐，后续的语句则采用缩进的方式。分行上，如果没有什么特殊考虑，请**不要在代码中连续使用两个以上的空行**。

## 11.大括号与空格

从代码阅读角度，建议每个大括号单独占用一行，而不是跟在语句的后面，例如：

```c
    if (condition)
    {
        /* others */
    }
```

匹配的大括号单独占用一行，代码阅读起来就会有相应的层次而不会容易出现混淆的情况。空格建议在非函数方式的括号调用前留一个空格以和前面的进行区分，例如：

```c
    if (x <= y)
    {
        /* others */
    }

    for (index = 0; index < MAX_NUMBER; index ++)
    {
        /* others */
    }
```

建议在括号前留出一个空格(涉及的包括 if、for、while、switch 语句)，而运算表达式中，运算符与字符串间留一个空格。另外，不要在括号的表达式两侧留空格，例如：

```c
    if ( x <= y )
    {
        /* other */
    }
```

这样括号内两侧的空格是不允许的。

推荐使用SDK根目录下的.clang-format文件作为clang-format格式化的配置文件对代码进行格式化，确保代码风格的一致性。

## 12.日志信息

代码中多使用ulog的方式来输出日志，例如：

```
#define DBG_TAG "Driver"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

LOG_D("this is a debug log.");
```

- 在 RT-Thread 中，普遍使用的日志输出方式是通过`LOG_D` 、`LOG_I` 、`LOG_W` 、`LOG_E`的方式来输出日志，同时它也可以通过DBG_TAG来区分日志类别，DBG_LVL控制日志输出的等级。
- 日志应该是以输出易懂易定位问题的方式。"天书式"的日志系统是糟糕的，不合理的，不应该出现在代码中。
- 禁止在头文件中重定义DBG_TAG，防止其他模块包含时DBG_TAG出现不可控。
- 严禁在timer或者中断打印大量日志，尽可能的避免或轻量化。
- 不建议使用rt_kprintf来作为日志输出方式，rt_kprintf一般作为终端命令行交互使用。

## 13.函数

函数应该尽量精简，仅完成相对独立的简单功能。函数的实现不应该太长，函数实现太长，应该反思能够如何修改(或拆分)使得函数更为精简、易懂。



