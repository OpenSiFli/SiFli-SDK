
# MAILBOX

The Mailbox HAL driver provides high-level APIs to use the hardware mailbox module. Each subsystem has a hardware mailbox module. It can be used to trigger interrupts to notify other subsystems or protect hardware resources shared by multiple subsystems.

Main features include:
- Trigger interrupts to notify other subsystems. For example, HPSYS mailbox group 1 (#H2L_MAILBOX) is used to trigger LPSYS interrupts.
- Each mailbox group has 16 channels and can trigger all interrupts simultaneously.
- Mailbox interrupts can automatically wake up subsystems in _LIGHT/DEEP/STANDBY_ low-power modes.
- The Mailbox module has two mutex channels to protect shared resources. All subsystems can access them.

```{note}
The mailbox module should be accessed when the owner subsystem is not in low-power mode, otherwise behavior is undefined.
```

Available resources in HPSYS:
1. `H2L_MAILBOX` 
1. `HMUTEX_CH1` 
1. `HMUTEX_CH2` 

Available resources in LPSYS:
1. `L2H_MAILBOX` 
1. `LMUTEX_CH1` 
1. `LMUTEX_CH2` 

For detailed mailbox driver APIs, please refer to [mailbox](#hal-mailbox).

## Using Mailbox HAL Driver

### Notification Function
The mailbox notification function can be used by calling some macros, such as calling `__HAL_MAILBOX_TRIGGER_CHANNEL_IT` to trigger interrupts. Since mailbox register contents are lost after waking up from standby mode, it's recommended that the sender (i.e., owner) subsystem mask/unmask interrupts.

In the following example, HCPU triggers an interrupt for LCPU:

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

### Mutex Function

The mutex function can be used by calling `HAL_MAILBOX_Lock` and `HAL_MAILBOX_UnLock`. Use `HAL_MAILBOX_Lock` to lock a mutex channel. If locking succeeds, it returns `MUTEX_UNLOCKED`, otherwise it returns which core is occupying the mutex channel. Use `HAL_MAILBOX_UnLock` to unlock a mutex channel. A channel can only be unlocked by the owner that previously locked the channel. The following example demonstrates HCPU locking and unlocking LPSYS mutex channel:

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

## API Reference
[](#hal-mailbox)
