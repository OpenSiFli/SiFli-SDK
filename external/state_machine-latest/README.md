# state_machine

## 1、介绍

state_machine是基于RT-Thread格式移植的状态机软件包。
state_machine的作者是misje, github地址: https://github.com/misje/stateMachine
对该软件包进行了如下修改:
1.修复部分函数反馈，由void改为int，如果异常反馈负数；
2.修改状态儿子数的判断，如果这个状态没有儿子，还需要判断它的父亲（原作者不判断父亲，该状态会停掉）；


## 使用方法
1.先申请一个状态结构，定义事件

```
enum event_type {
    EVENT_KEYBOARD,
};
struct state_machine m;
```
2.初始化状态对象

```
static struct state state_idle = {
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[]){
        { EVENT_KEYBOARD, (void *)(intptr_t)'t', &keyboard_char_compare, NULL, &state_next },// turn next state
		{ EVENT_KEYBOARD, (void *)(intptr_t)'d', &keyboard_char_compare, &action_fun, &state_idle },// do action
    },
    .transition_nums = 1,
    .data = "idle",
    .action_entry = &print_msg_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_error = { 
    .data = "Error",
    .action_entry = &print_msg_err
};
```
3.初始化状态机

```
statem_init( &m, &state_idle, &state_error );
...
```
4.执行事件

```
statem_handle_event( &m, &(struct event){ EVENT_KEYBOARD, (void *)(intptr_t)ch } );
```


## 特性

state_machine 使用C语言实现，基于面向对象方式设计思路，每个状态对象单独用一份数据结构管理：



## Examples

使用示例在 [examples](./examples) 下。

