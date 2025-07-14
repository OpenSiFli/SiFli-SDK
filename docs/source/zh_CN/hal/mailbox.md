# MAILBOX

Mailbox HAL 驱动程序提供高级 API 以使用硬件邮箱模块。 每个子系统都有一个硬件邮箱模块。 它可用于触发中断以通知其他子系统或保护多个子系统共享的硬件资源。\
主要功能包括：
- 触发中断通知其他子系统。 例如HPSYS mailbox组1（#H2L_MAILBOX）用于触发LPSYS中断。
- 每个mailbox组有16个通道，可以同时触发所有中断。
- Mailbox中断可以自动唤醒处于  _LIGHT/DEEP/STANDBY_ 低功耗模式的子系统。
- Mailbox模块有两个互斥通道来保护共享资源。 所有子系统都可以访问。
```{note}
当所有者子系统未处于低功耗模式时应访问mailbox模块，否则行为未定义。
```

HPSYS 中的可用资源：
1. `H2L_MAILBOX` 
1. `HMUTEX_CH1` 
1. `HMUTEX_CH2` 
LPSYS 中的可用资源：
1. `L2H_MAILBOX` 
1. `LMUTEX_CH1` 
1. `LMUTEX_CH2` 
mailbox驱动的详细API请参考[mailbox](#hal-mailbox) 。

## 使用mailbox HAL 驱动程序

### 通知功能
可以通过调用一些宏来使用mailbox通知功能，例如 调用 `__HAL_MAILBOX_TRIGGER_CHANNEL_IT` 触发中断，由于邮mailbox块寄存器内容在从待机模式唤醒后会丢失，建议发送者（即所有者）子系统屏蔽/取消屏蔽中断。
在下面的示例中，HCPU触发了LCPU的中断：

```c
static MAILBOX_HandleTypeDef mailbox_handle;
void hcpu_mailbox_ch1_init(void)
{
    mailbox_handle.Instance = H2L_MAILBOX;
    /* unmask HCPU2LCPU channel1 interrupt */
    __HAL_MAILBOX_UNMASK_CHANNEL_IT(&mailbox_handle, MAILBOX_CHANNEL_1);
}

void hcpu_trigger_mailbox_ch1_interrupt(void) 
{
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&mailbox_handle, MAILBOX_CHANNEL_1);
}

void lcpu_mailbox_ch1_init(void)
{
    /* set the mailbox priority */
    HAL_NVIC_SetPriority(HCPU2LCPU_IRQn, 3, 0);
    /* enable the mailbox global Interrupt */
    HAL_NVIC_EnableIRQ(HCPU2LCPU_IRQn);
}

void lcpu_mailbox_isr(void)
{
    uint32_t status;
    status = __HAL_MAILBOX_GET_STATUS(&mailbox_handle);
    
    //do something according to status
    //...
    
    __HAL_MAILBOX_CLEAR_STATUS(status);
}

```

### 互斥功能

可以通过调用 `HAL_MAILBOX_Lock` 和 `HAL_MAILBOX_UnLock` 来使用互斥功能。 使用 `HAL_MAILBOX_Lock` 来锁定互斥通道。 如果锁定成功，则返回 `MUTEX_UNLOCKED` ，否则返回哪个内核正在占用互斥通道。 使用 `HAL_MAILBOX_UnLock` 来解锁互斥通道。通道只能由之前锁定通道的所有者解锁。 以下示例演示了 HCPU 锁定和解锁 LPSYS 互斥锁通道 


```c
void test_hmutex_ch1(void) 
{ 	
    MUTEX_HandleTypeDef handle;
    
    handle.Instance = HMUTEX_CH1;
    core = HAL_MAILBOX_Lock(&handle, 0);
    //core should be MUTEX_UNLOCKED
    core = HAL_MAILBOX_Lock(&handle, 0);
    // core should be MUTEX_HCPU_LOCKED
    HAL_MAILBOX_UnLock(&handle, 0);
}
```


## API参考
[](#hal-mailbox)

