/*Auto generated configuration*/
#ifndef RT_CONFIG_H
#define RT_CONFIG_H

#define SOC_SF32LB55X 1
#define CORE "HCPU"
#define CPU "Cortex-M33"
#define BSP_USING_RTTHREAD 1
#define RT_USING_COMPONENTS_INIT 1
#define RT_USING_USER_MAIN 1
#define RT_MAIN_THREAD_STACK_SIZE 4096
#define RT_MAIN_THREAD_PRIORITY 15
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
#define RT_USING_I2C 1
#define RT_USING_PIN 1
#define RT_USING_PM 1
#define PM_STANDBY_ENABLE 1
#define RT_USING_RTC 1
#define RT_USING_ALARM 1
#define RT_USING_SPI 1
#define RT_USING_SENSOR 1
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
#define RT_USING_MEMHEAP 1
#define RT_USING_SMALL_MEM 1
#define RT_USING_MEMTRACE 1
#define RT_USING_HEAP 1
#define RT_USING_DEVICE 1
#define RT_USING_CONSOLE 1
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x30103
#define BSP_USING_GPIO 1
#define BSP_GPIO_HANDLE 2
#define BSP_USING_DMA 1
#define BSP_USING_UART 1
#define BSP_USING_UART1 1
#define BSP_UART1_RX_USING_DMA 1
#define BSP_USING_SPI 1
#define BSP_USING_SPI1 1
#define BSP_USING_I2C 1
#define BSP_USING_I2C1 1
#define BSP_USING_I2C2 1
#define BSP_USING_TIM 1
#define BSP_USING_GPTIM2 1
#define BSP_USING_GPTIM3 1
#define BSP_USING_BTIM1 1
#define BSP_USING_LPTIM1 1
#define BSP_USING_SPI_FLASH 1
#define BSP_ENABLE_QSPI1 1
#define BSP_QSPI1_MODE 0
#define BSP_QSPI1_USING_DMA 1
#define BSP_QSPI1_MEM_SIZE 4
#define BSP_USING_PSRAM 1
#define BSP_USING_ONCHIP_RTC 1
#define BSP_USING_PINMUX 1
#define BSP_USING_LCPU_PATCH 1
#define BSP_USING_BUTTON 1
#define BSP_USING_GPT3_BUTTON 1
#define SINGLE_AND_DOUBLE_TRIGGER 1
#define BUTTON_DEBOUNCE_TIME 2
#define BUTTON_CONTINUOS_CYCLE 1
#define BUTTON_LONG_CYCLE 1
#define BUTTON_DOUBLE_TIME 15
#define BUTTON_LONG_TIME 50
#define BSP_USING_QSPI 1
#define BSP_QSPI1_CHIP_ID 0
#define BSP_USING_PSRAM0 1
#define PSRAM_FULL_SIZE 32
#define BSP_USING_EXT_PSRAM 1
#define BSP_USING_XCCELA_PSRAM 1
#define BSP_USING_DUAL_PSRAM 1
#define RT_USING_MOTOR 1
#define BSP_USING_BOARD_EC_LB557XXX 1
#define LXT_FREQ 32768
#define LXT_LP_CYCLE 200
#define BLE_TX_POWER_VAL 0
#define BSP_LB55X_CHIP_ID 2
#define BSP_USING_KEY1 1
#define BSP_KEY1_PIN 99
#define ASIC 1
#define HRS3300_POW_PIN 105
#define BSP_USING_EMPTY_ASSERT 1
#define BSP_USING_PM 1
#define BSP_USING_DATA_SVC 1
#define DATA_SVC_MBOX_THREAD_STACK_SIZE 1024
#define DATA_SVC_MBOX_THREAD_PRIORITY 13
#define DATA_SVC_PROC_THREAD_STACK_SIZE 4096
#define DATA_SVC_PROC_THREAD_PRIORITY 14
#define BSP_USING_BLE_NVDS_SVC 1
#define BSP_SHARE_PREFS 1
#define USING_IPC_QUEUE 1
#define USING_CONTEXT_BACKUP 1
#define RT_USING_RTT_CMSIS 1
#define PKG_USING_FLASHDB 1
#define PKG_FLASHDB_PATH "/external/FlashDB"
#define FDB_KV_CACHE_TABLE_SIZE 16
#define PKG_FLASHDB_ERASE_GRAN 4096
#define PKG_FLASHDB_START_ADDR 0
#define PKG_FLASHDB_DEBUG 1
#define PKG_FDB_USING_FAL_MODE 1
#define BF0_HCPU 1
#endif