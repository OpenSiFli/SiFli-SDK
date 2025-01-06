#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* Sifli Configuration */

#define SOC_SF32LB52X
#define CORE "HCPU"
#define CPU "Cortex-M33"

/* RTOS */

#define BSP_USING_RTTHREAD

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 8192
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_ARG_MAX 10

/* Device virtual file system */


/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 64

/* Using WiFi */


/* Using USB */


/* POSIX layer and C standard library */


/* Utilities */


/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_IDLE_HOOK
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_SMALL_MEM
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart4"
#define RT_VER_NUM 0x30103
#define BSP_USING_UART

/* On-chip Peripheral Drivers */

#define BSP_USING_DMA
#define BSP_USING_UART4
#define BSP_UART4_RX_USING_DMA
#define BSP_USING_FLASH
#define BSP_USING_NOR_FLASH
#define BSP_ENABLE_FLASH1
#define BSP_FLASH1_MEM_SIZE 2
#define BSP_USING_EXT_DMA
#define BSP_USING_HW_AES
#define BSP_USING_PINMUX

/* Select board peripherals */

#define BSP_USING_BOARD_MPW_EVB
#define ASIC
#define LXT_FREQ 32768

/* Key config */


/* Sifli Middleware */

#define BSP_USING_EMPTY_ASSERT

/* RW Bluetooth */

#define BSP_USING_DFU

/* Third party packages */

#define PKG_SIFLI_MBEDTLS_BOOT
#define BF0_HCPU
#define CFG_BOOTLOADER

#endif
