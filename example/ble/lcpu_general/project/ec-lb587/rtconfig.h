/*Auto generated configuration*/
#ifndef RT_CONFIG_H
#define RT_CONFIG_H

#define SOC_SF32LB58X 1
#define CORE "LCPU"
#define CPU "Cortex-M33"
#define BSP_USING_RTTHREAD 1
#define RT_USING_COMPONENTS_INIT 1
#define RT_USING_USER_MAIN 1
#define RT_MAIN_THREAD_STACK_SIZE 4096
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_FINSH 1
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY 1
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB 1
#define FINSH_USING_DESCRIPTION 1
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH 1
#define FINSH_USING_MSH_DEFAULT 1
#define FINSH_ARG_MAX 10
#define RT_USING_DEVICE_IPC 1
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL 1
#define RT_SERIAL_USING_DMA 1
#define RT_SERIAL_RB_BUFSZ 4096
#define RT_SERIAL_DEFAULT_BAUDRATE 1000000
#define RT_USING_HWTIMER 1
#define RT_USING_HWMAILBOX 1
#define RT_USING_PIN 1
#define RT_USING_PM 1
#define PM_STANDBY_ENABLE 1
#define RT_USING_AUDIO 1
#define RT_USING_LIBC 1
#define RT_USING_RYM 1
#define RT_USING_ULOG 1
#define ULOG_OUTPUT_LVL_D 1
#define ULOG_OUTPUT_LVL 7
#define ULOG_USING_ISR_LOG 1
#define ULOG_ASSERT_ENABLE 1
#define ULOG_LINE_BUF_SIZE 128
#define ULOG_USING_ASYNC_OUTPUT 1
#define ULOG_ASYNC_OUTPUT_BUF_SIZE 8192
#define CUSTOM_ULOG_ASYNC_OUTPUT_STORE_LINES 1
#define ULOG_ASYNC_OUTPUT_STORE_LINES 96
#define ULOG_ASYNC_OUTPUT_BY_THREAD 1
#define ULOG_ASYNC_OUTPUT_THREAD_STACK 1024
#define ULOG_ASYNC_OUTPUT_THREAD_PRIORITY 30
#define ULOG_OUTPUT_FLOAT 1
#define ULOG_OUTPUT_TIME 1
#define ULOG_OUTPUT_LEVEL 1
#define ULOG_OUTPUT_TAG 1
#define ULOG_OUTPUT_THREAD_NAME 1
#define ULOG_BACKEND_USING_CONSOLE 1
#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32 1
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK 1
#define RT_USING_HOOK 1
#define RT_USING_IDLE_HOOK 1
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 512
#define RT_USING_TIMER_SOFT 1
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 1024
#define RT_DEBUG 1
#define RT_USING_SEMAPHORE 1
#define RT_USING_MUTEX 1
#define RT_USING_EVENT 1
#define RT_USING_MAILBOX 1
#define RT_USING_MESSAGEQUEUE 1
#define RT_USING_MEMPOOL 1
#define RT_USING_MEMHEAP 1
#define RT_USING_SMALL_MEM 1
#define RT_USING_MEMTRACE 1
#define RT_USING_HEAP 1
#define RT_USING_DEVICE 1
#define RT_USING_CONSOLE 1
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart4"
#define RT_VER_NUM 0x30103
#define BSP_USING_GPIO 1
#define BSP_GPIO_HANDLE 2
#define BSP_USING_DMA 1
#define BSP_USING_UART 1
#define BSP_USING_UART4 1
#define BSP_UART4_RX_USING_DMA 1
#define BSP_USING_TIM 1
#define BSP_USING_LPTIM2 1
#define BSP_USING_PINMUX 1
#define BSP_USING_BOARD_EC_LB587XXX 1
#define LXT_FREQ 32768
#define LXT_LP_CYCLE 200
#define BLE_TX_POWER_VAL 0
#define BSP_USING_KEY1 1
#define BSP_KEY1_PIN 99
#define ASIC 1
#define BSP_USING_EMPTY_ASSERT 1
#define BLUETOOTH 1
#define ROM_ATT_BUF_SIZE 3084
#define ROM_LOG_SIZE 4096
#define BT_DUAL_FULL_MEM 1
#define MB_PORT 1
#define UART_PORT1 1
#define UART_PORT1_PORT "uart4"
#define BSP_USING_PM 1
#define BSP_PM_STANDBY_SHUTDOWN 1
#define USING_IPC_QUEUE 1
#define USING_CONTEXT_BACKUP 1
#define AUDIO 1
#define AUDIO_PATH_USING_HCPU 1
#define AUDIO_USING_AUDPROC 1
#define AUDIO_BT_AUDIO 1
#define AUDIO_SPEAKER_USING_I2S2 1
#define AUDIO_MIC_USING_PDM 1
#define USING_PARTITION_TABLE 1
#define RT_USING_RTT_CMSIS 1
#define PKG_USING_SEGGER_RTT 1
#define PKG_SEGGER_RTT_MAX_NUM_UP_BUFFERS 3
#define PKG_SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 3
#define PKG_SEGGER_RTT_BUFFER_SIZE_UP 2048
#define PKG_SEGGER_RTT_BUFFER_SIZE_DOWN 1024
#define PKG_SEGGER_RTT_PRINTF_BUFFER_SIZE 256
#define PKG_SEGGER_RTT_AS_SERIAL_DEVICE 1
#define PKG_SERIAL_DEVICE_NAME "segger"
#define PKG_SEGGER_RTT_MODE_ENABLE_NO_BLOCK_SKIP 1
#define PKG_SEGGER_RTT_MAX_INTERRUPT_PRIORITY 0x20
#define BF0_LCPU 1
#define LCPU_ROM 1
#define CUSTOM_MEM_MAP 1
#endif