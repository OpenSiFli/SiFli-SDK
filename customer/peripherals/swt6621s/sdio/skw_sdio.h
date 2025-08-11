/*
 * Copyright (C) 2022 Seekwave Tech Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __SKW_SDIO_H__
#define __SKW_SDIO_H__

#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>
#include <drivers/mmcsd_host.h>
#include <sdio_func.h>
#include <skw_platform_data.h>
#include <drivers/sdio.h>

//#include "../skwutil/skw_boot.h"
#include "skw_sdio_log.h"
#define skwsdio_log(fmt, args...) \
    rt_kprintf("[SKWSDIO]:" fmt, ## args)

#define skwsdio_err(fmt, args...) \
    rt_kprintf("[SKWSDIO_ERR]:" fmt, ## args)

//#define skwsdio_data_pr(level, prefix_str, prefix_type, rowsize,\
//      groupsize, buf, len, asscii)\
//      do{if(loglevel) \
//          print_hex_dump(level, prefix_str, prefix_type, rowsize,\
//                  groupsize, buf, len, asscii);\
//      }while(0)

#define scatterlist char

#define BOOT_DRAM_PATH "/wifi/boot"
#define BOOT_IRAM_PATH "/wifi/boot"
#define CALIBRATION_IRAM_PATH "/wifi/calibration"


#define BOOT_DRAM_FILE_NAME "SWT6621S_DRAM_SDIO.bin"
#define BOOT_IRAM_FILE_NAME "SWT6621S_IRAM_SDIO.bin"
#define CALIBRATION_FILE_NAME "SWT6621S_SEEKWAVE_R00001.bin"

#define BOOT_FILE_NAME_LEN 128
#define BOOT_DRAM_ADDR   0X20200000
#define BOOT_IRAM_ADDR   0X00100000
#define HZ 1000
#define SKW_AP2CP_IRQ_REG 0x1B0

#define SKW_BUF_SIZE    2048

#define SKW_SDIO_SDMA   0
#define SKW_SDIO_ADMA   1

#define SKW_SDIO_INBAND_IRQ 0
#define SKW_SDIO_EXTERNAL_IRQ   1

#define SDIO_RX_TASK_PRIO   90
#define SDIO_UPDATE_TASK_PRIO   91

#define SKW_SDIO_BLK_SIZE   skw_sdio_blk_size
#define MAX_PAC_SIZE        0x700
#define MAX2_PAC_SIZE       0x600
#define MAX_PAC_COUNT       20

#define SKW_SDIO_NSIZE_BUF_SIZE SKW_SDIO_BLK_SIZE

#define SKW_SDIO_READ       0
#define SKW_SDIO_WRITE      1

#define SKW_SDIO_DATA_FIX   0
#define SKW_SDIO_DATA_INC   1

#define MAX_IO_RW_BLK       511

#define FUNC_0          0
#define FUNC_1          1
#define MAX_FUNC_NUM        2

#define SKW_SDIO_DT_MODE_ADDR   0x0f//
#define SKW_SDIO_PK_MODE_ADDR   0x20

#define SKW_SDIO_RESET_MODE_ADDR    0x1C
#define SKW_SDIO_CCCR_ABORT     0x06
#define SDIO_INT_EXT            0x16
#define SDIO_ABORT_TRANS        0x01

#define SKW_SDIO_FBR_REG        0x15C

#define SKW_CHIP_ID0        0x40000000      //SV6160 chip id0
#define SKW_CHIP_ID1        0x40000004      //SV6160 chip id1
#define SKW_CHIP_ID2        0x40000008      //SV6160 chip id2
#define SKW_CHIP_ID3        0x4000000C      //SV6160 chip id3
#define SKW_SMEM_POWERON1 0x40104000  //SMEM power on
#define SKW_SMEM_POWERON2 0x40108000  //SMEM power on
#define SKW_CHIP_ID_LENGTH  16          //SV6160 chip id lenght

#define SKW_SDIO_ALIGN_4BYTE(a)  (((a)+3)&(~3))
#define SKW_SDIO_ALIGN_BLK(a) (((a)%SKW_SDIO_BLK_SIZE) ? \
    (((a)/SKW_SDIO_BLK_SIZE + 1)*SKW_SDIO_BLK_SIZE) : (a))

#define SDIO_VER_CCCR   (0)


#define SKW_SDIO_CARD_OFFLINE 0x8000
#define SKW_CARD_ONLINE(skw_sdio) \
    (atomic_read(&skw_sdio->online) < SKW_SDIO_CARD_OFFLINE)

#define SKW_SDIO_RESET_CARD_VAL 0x08
#define SKW_SDIO_RESET_CP   0x20

#define WIFI_SERVICE    0
#define BT_SERVICE  1

#define SERVICE_START   0
#define SERVICE_STOP    1

#define SKW_SDIO_V10 0
#define SKW_SDIO_V20 1


#define BSP_ATC_PORT    0
#define BSP_LOG_PORT    1
#define BT_DATA_PORT    2
#define BT_CMD_PORT 3
#define BT_AUDIO_PORT   4
#define WIFI_CMD_PORT   5
#define WIFI_DATA_PORT  6
#define LOOPCHECK_PORT  7
#define MAX_CH_NUM  8

#define SDIO2_BSP_ATC_PORT  0
#define SDIO2_LOOPCHECK_PORT    1
#define SDIO2_BT_CMD_PORT   2
#define SDIO2_BT_AUDIO_PORT 3
#define SDIO2_BT_ISOC_PORT  4
#define SDIO2_BT_DATA_PORT      5
#define SDIO2_WIFI_CMD_PORT 6
#define SDIO2_WIFI_DATA_PORT    7
#define SDIO2_WIFI_DATA1_PORT   8
#define SDIO2_BSP_LOG_PORT  9
#define SDIO2_BT_LOG_PORT   10
#define SDIO2_BSP_UPDATE_PORT   11
#define SDIO2_MAX_CH_NUM    12

#define DEVICE_ASSERT_EVENT 0
#define DEVICE_BSPREADY_EVENT   1
#define DEVICE_DUMPDONE_EVENT   2
#define DEVICE_BLOCKED_EVENT    3
#define DEVICE_DISCONNECT_EVENT 4
#define DEVICE_DUMPMEM_EVENT 5

#ifndef u8
    #define u8 unsigned char
#endif

#ifndef u16
    #define u16 unsigned short
#endif

#ifndef u32
    #define u32 unsigned int
#endif

#ifndef s32
    #define s32 int
#endif

#ifndef u64
    #define u64 long long
#endif

#ifndef atomic_t
    #define atomic_t int
#endif
#ifndef atomic_inc
    #define atomic_inc(atom) (*atom)++
#endif
#ifndef atomic_dec
    #define atomic_dec(atom) (*atom)--
#endif
#ifndef atomic_read
    #define atomic_read(atom) (*atom)
#endif

#define SKW_WIFI_LOG     0
#if SKW_WIFI_LOG
    #define skw_printf rt_kprintf
#else
    #define skw_printf(...)
#endif
#define SKW_WIFI_MAX_PACKET_SIZE  1024 * 32//最大每次读20个packet
struct skw_wifi_rx_data
{
    uint16_t flags;
    uint16_t flags0;
    uint16_t flags1;
};

struct skw_sdio_data_t
{
    rt_thread_t rx_thread;
    struct rt_semaphore rx_completed;
//    struct rt_semaphore rx_completed;
    rt_thread_t update_thread;
    struct rt_completion *update_completed;
#if 0
#ifdef  CONFIG_WAKELOCK
    struct wake_lock rx_wl;
#else
    struct wakeup_source *rx_ws;
#endif
#endif
    atomic_t rx_wakelocked;
    struct rt_mutex transfer_mutex;
    struct rt_mutex except_mutex;
    struct rt_mutex service_mutex;
    struct rt_mutex rx_tx_mutex;
    atomic_t resume_flag;
    atomic_t online;
    bool threads_exit;
    bool adma_rx_enable;
    bool pwrseq;
    bool blk_size;
    /* EXTERNAL_IRQ 0, INBAND_IRQ 1. */
    unsigned char irq_type;

    atomic_t suspending;
    int gpio_out;
    int gpio_in;
    unsigned int irq_num;
    unsigned int irq_trigger_type;
    unsigned int smem_poweron;
    struct sdio_func *sdio_func[MAX_FUNC_NUM];
    struct rt_mmcsd_host *sdio_dev_host;
    unsigned char *eof_buf;

    unsigned int next_size;
    unsigned int remain_packet;
    unsigned long long rx_packer_cnt;
    char *next_size_buf;

    struct rt_completion scan_done;
    struct rt_completion remove_done;
    struct rt_completion download_done;
    int host_active;
    int host_state;
    int device_active;
    struct rt_completion device_wakeup;
    char tx_req_map;
    int resume_com;
    int cp_state;
    int chip_en;
    unsigned char chip_id[SKW_CHIP_ID_LENGTH];
    struct seekwave_device *boot_data;
    unsigned int service_state_map;
    struct rt_delayed_work skw_except_work;
    int power_off;
    int service_index_map;
    //wait_queue_head_t wq;
};

struct debug_vars
{
    u16 cmd_timeout_cnt;
    u32 rx_inband_irq_cnt;
    u32 rx_gpio_irq_cnt;
    u32 rx_irq_statistics_cnt;
    u32 rx_read_cnt;
    u32 last_sent_wifi_cmd[3];
    u64 last_sent_time;
    u64 last_rx_submit_time;
    u64 host_assert_cp_time;
    u64 cp_assert_time;
    u64 last_irq_time;
    u64 rx_irq_statistics_time;
    u32 chn_irq_cnt[SDIO2_MAX_CH_NUM];
#define CHN_IRQ_RECORD_NUM 3
    u64 chn_last_irq_time[SDIO2_MAX_CH_NUM][CHN_IRQ_RECORD_NUM];
    u64 last_irq_times[CHN_IRQ_RECORD_NUM];
    u64 last_clear_irq_times[CHN_IRQ_RECORD_NUM];
    u64 last_rx_read_times[CHN_IRQ_RECORD_NUM];
};
struct sdio_port
{
    rt_device_t pdev;
    uint8_t *sg_rx;//消息事件的传递？
    uint32_t sg_len;
    int  sg_index;
    int  total;
    int sent_packet;
    unsigned int type;
    unsigned int channel;
    rx_submit_fn rx_submit;
    void *rx_data;
    int rx_data_len;
    int state;
    char *read_buffer;
    int rx_rp;
    int rx_wp;
    char *write_buffer;
    int  length;
    struct rt_completion rx_done;
    struct rt_completion tx_done;
    struct rt_mutex rx_mutex;
    int rx_packet;
    int rx_count;
    int     tx_flow_ctrl;
    int  rx_flow_ctrl;
    u16 next_seqno;
};

//=======================================================
//debug sdio macro and Variable
//int glb_wifiready_done;
#define SKW_WIFIONLY_DEBUG 1
//=======================================================
void skw_resume_check(void);
struct skw_sdio_data_t *skw_sdio_get_data(void);

void skw_sdio_rx_up(struct skw_sdio_data_t *skw_sdio);
void skw_sdio_rx_thread(void *parameter);

void skw_sdio_unlock_rx_ws(struct skw_sdio_data_t *skw_sdio);
int skw_recovery_mode(void);
int skw_sdio_sdma_write(unsigned char *src, unsigned int len);
int skw_sdio_sdma_read(unsigned char *src, unsigned int len);
int skw_sdio_adma_write(int portno, uint8_t *sgs, int sg_count, int mode_addr);
int skw_sdio_adma_read(struct skw_sdio_data_t *skw_sdio, uint8_t *sgs, int sg_count, int mode_addr);
int skw_sdio_dt_read(unsigned int address, void *buf, unsigned int len);
int skw_sdio_dt_write(unsigned int address, void *buf, unsigned int len);
int skw_sdio_readb(unsigned int address, unsigned char *data);
int skw_sdio_writeb(unsigned int address, unsigned char data);
int skw_sdio_writel(unsigned int address, void *data);
int skw_sdio_readl(unsigned int address, void *data);
int send_modem_service_command(u16 service, u16 command);
int send_modem_assert_command(void);
int skw_sdio_bind_platform_driver(struct sdio_func *func);
int skw_sdio_bind_WIFI_driver(struct sdio_func *func);

int skw_sdio_bind_btseekwave_driver(struct sdio_func *func);
int skw_sdio_unbind_platform_driver(struct sdio_func *func);
int skw_sdio_unbind_WIFI_driver(struct sdio_func *func);
int skw_sdio_unbind_BT_driver(struct sdio_func *func);
int skw_boot_loader(struct seekwave_device *boot_data);
void send_host_suspend_indication(struct skw_sdio_data_t *skw_sdio);
void send_host_resume_indication(struct skw_sdio_data_t *skw_sdio);
int try_to_wakeup_modem(int portno);
int wakeup_modem(int portno);
void host_gpio_in_routine(int value);
void skw_sdio_inband_irq_handler(struct sdio_func *func);
int loopcheck_send_data(char *buffer, int size);
void skw_get_port_statistic(char *buffer, int size);
int skw_sdio_cp_log_disable(int disable);
int skw_sdio_recovery_debug(int disable);
int skw_sdio_recovery_debug_status(void);
int skw_sdio_wifi_serv_debug(int enable);
int skw_sdio_wifi_serv_debug_status(void);
int skw_sdio_bt_serv_debug(int enable);
int skw_sdio_bt_serv_debug_status(void);
void reboot_to_change_bt_antenna_mode(char *mode);
void reboot_to_change_bt_uart1(char *mode);
void get_bt_antenna_mode(char *mode);
int skw_sdio_wifi_power_on(int power_on);
int skw_sdio_wifi_status(void);
int skw_sdio_dloader(int index);
int skw_sdio_poweron_mem(int index);
void skw_get_sdio_debug_info(char *buffer, int size);
void skw_get_assert_print_info(char *buffer, int size);
int skw_sdio_debug_log_open(void);
int skw_sdio_debug_log_close(void);
/************************/
void cancel_delayed_work_sync(struct rt_work *work);
void schedule_delayed_work(struct rt_work *work, rt_tick_t time);

#define INIT_DELAYED_WORK(work,func) rt_delayed_work_init(work, func, RT_NULL)
typedef void(*aicwf_sdio_irq_handler_t)(struct sdio_func *);

#define PAGE_SIZE 2048
void sdio_claim_host(struct sdio_func *func);
void sdio_release_host(struct sdio_func *func);

void mutex_lock(rt_mutex_t mutex);
void mutex_unlock(rt_mutex_t mutex);
void mutex_init(rt_mutex_t mutex);
void mutex_destroy(rt_mutex_t mutex);
rt_err_t  wait_for_completion_timeout(struct rt_completion *completion, rt_int32_t            timeout);
rt_err_t wait_for_completion_interruptible_timeout(struct rt_completion *completion, rt_int32_t timeout);
rt_err_t wait_for_completion_interruptible(struct rt_completion *completion);
rt_err_t init_completion(struct rt_completion *complet);
rt_err_t complete(struct rt_completion *complet);
uint32_t msecs_to_jiffies(uint32_t time);
rt_time_t ktime_get(void);
rt_time_t ktime_sub(rt_time_t lhs, rt_time_t rhs);

void atomic_set(int var, int i);
void WARN_ON(int i);
rt_err_t platform_device_add(rt_device_t dev);
rt_err_t platform_device_unregister(rt_device_t dev);
rt_int32_t sdio_claim_irq(struct sdio_func *func, rt_sdio_irq_handler_t   *handler);
rt_int32_t rt_skw_sdio_probe(struct rt_mmcsd_card *card);
rt_int32_t rt_skw_sdio_remove(struct rt_mmcsd_card *card);

int xhci_sdio_release_irq(struct sdio_func *func);
int sdio_release_irq(struct sdio_func *func);


u64 skw_local_clock(void);
uint32_t local_clock(void);
void skw_sdio_transfer_exit(void);
void skw_sdio_transfer_enter(void);
__weak int gpio_set_value(int i, int j)
{
    return 0;
}
__weak int gpio_get_value(int i)
{
    return 0;
}
__weak void page_frag_free(void *head)
{
    //rt_free(head);
}
__weak void sg_set_buf(uint8_t *sg_rx, void *head, int len)
{

}
__weak int sdio_reset_comm(struct rt_mmcsd_card *card)
{
    return 0;
}
__weak void *kzalloc(uint32_t size, int i)
{
    return rt_malloc(size);
}
__weak void *skw_sdio_alloc_frag(uint32_t size, int i)
{
    return rt_malloc(size);
}

__weak void kfree(void *head)
{
    rt_free(head);
}
__weak void udelay(int time)
{
    rt_thread_mdelay(time);
}
__weak void msleep(int time)
{
    rt_thread_mdelay(time);
}
__weak void modem_notify_event(int i)
{

}
static inline void skw_ether_copy(u8 *dst, const u8 *src)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
    *(u32 *)dst = *(const u32 *)src;
    *(u16 *)(dst + 4) = *(const u16 *)(src + 4);
#else
    u16 *a = (u16 *)dst;
    const u16 *b = (const u16 *)src;

    a[0] = b[0];
    a[1] = b[1];
    a[2] = b[2];
#endif
}

struct sv6160_platform_data *skw_sdio_get_pdata(void);
struct sdio_port *skw_get_sdio_ports(uint8_t port_num);

void *sg_virt(void *buff);

void inv_sleep(int mSecs);
#ifndef min
    #define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
    #define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef  BIT
    #define  BIT(x)                 (1 << x)
#endif
void mutex_lock(rt_mutex_t mutex);

#endif /* SKW_SDIO_H */
