#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>

#include "skw_sdio.h"
#include "skw_platform_data.h"
#include "skw_boot.h"
#include "string.h"
//#include "wifi-internal.h"

#define MODEM_ASSERT_TIMEOUT_VALUE  2*1000
#define MAX_SG_COUNT    100
#define SDIO_BUFFER_SIZE    (16*1024)
#define MAX_FIRMWARE_SIZE 256
#define PORT_STATE_IDLE 0
#define PORT_STATE_OPEN 1
#define PORT_STATE_CLSE 2
#define PORT_STATE_ASST 3

#define CRC_16_L_SEED   0x80
#define CRC_16_L_POLYNOMIAL  0x8000
#define CRC_16_POLYNOMIAL  0x1021

//#define platform_device rt_device
int recovery_debug_status = 0;
int wifi_serv_debug_status = 0;
int bt_serv_debug_status = 0;
static struct skw_wifi_rx_data rx_data_t = {0};

/***********************************************************/
char firmware_version[128];
char assert_context[1024];
int  assert_context_size = 0;
static int assert_info_print;
static u8 fifo_ind;
//static u64 port_dmamask = DMA_BIT_MASK(32);
static u64 port_dmamask ;
struct sdio_port sdio_ports[SDIO2_MAX_CH_NUM];
static u8 cp_fifo_status;
struct debug_vars debug_infos;
#if 0
    static BLOCKING_NOTIFIER_HEAD(modem_notifier_list);
    #if KERNEL_VERSION(4,4,0) <= LINUX_VERSION_CODE
        static DEFINE_PER_CPU(struct page_frag_cache, skw_sdio_alloc_cache);
    #endif
#endif
unsigned int crc_16_l_calc(char *buf_ptr, unsigned int len);
static int skw_sdio_rx_port_follow_ctl(int portno, int rx_fctl);
//add the crc api the same as cp crc_16 api
extern void kernel_restart(char *cmd);
static int bt_service_start(void);
static int bt_service_stop(void);
static int wifi_service_start(void);
static int wifi_service_stop(void);
static void send_cp_wakeup_signal(struct skw_sdio_data_t *skw_sdio);
char skw_cp_ver = SKW_SDIO_V10;
int max_ch_num = MAX_CH_NUM;
int max_pac_size = MAX_PAC_SIZE;
int skw_sdio_blk_size = 256;

#define IS_LOG_PORT(portno)  ((skw_cp_ver == SKW_SDIO_V10)?(portno==1):(portno==SDIO2_BSP_LOG_PORT))


void skw_get_assert_print_info(char *buffer, int size)
{

}

void skw_get_sdio_debug_info(char *buffer, int size)
{
}
//=======================================================
//debug sdio macro and Variable
int glb_wifiready_done;
//#define SKW_WIFIONLY_DEBUG 1
//=======================================================

/********************************************************
 * skw_sdio_update img crc checksum
 * For update the CP IMG
 *Author: JUNWEI JIANG
 *Date:2022-08-11
 * *****************************************************/

int skw_log_port(void)
{
    return (skw_cp_ver == SKW_SDIO_V10) ? (1) : (SDIO2_BSP_LOG_PORT);
}

void skw_get_port_statistic(char *buffer, int size)
{
    int ret = 0;
    int i;

    if (!buffer)
        return;

    for (i = 0; i < SDIO2_MAX_CH_NUM; i++)
    {
        if (ret >= size)
            break;

        if (sdio_ports[i].state)
            ret += rt_sprintf(&buffer[ret], "port%d: rx %d %d, tx %d %d\n",
                              i, sdio_ports[i].rx_count, sdio_ports[i].rx_packet,
                              sdio_ports[i].total, sdio_ports[i].sent_packet);

    }
}

unsigned int crc_16_l_calc(char *buf_ptr, unsigned int len)
{
    unsigned int i;
    unsigned short crc = 0;

    while (len-- != 0)
    {
        for (i = CRC_16_L_SEED; i != 0; i = i >> 1)
        {
            if ((crc & CRC_16_L_POLYNOMIAL) != 0)
            {
                crc = crc << 1;
                crc = crc ^ CRC_16_POLYNOMIAL;
            }
            else
            {
                crc = crc << 1;
            }

            if ((*buf_ptr & i) != 0)
            {
                crc = crc ^ CRC_16_POLYNOMIAL;
            }
        }
        buf_ptr++;
    }
    return (crc);
}

static int skw_sdio_rx_port_follow_ctl(int portno, int rx_fctl)
{
    char ftl_val = 0;
    int ret = 0;

    skw_sdio_info(" portno:%d, rx_fctl:%d \n", portno, rx_fctl);

    if ((portno < 0) || (portno > max_ch_num))
        return -1;

    if (portno < 8)
    {
        ret = skw_sdio_readb(SKW_SDIO_RX_CHANNEL_FTL0, &ftl_val);
        if (ret)
            return -1;

        if (rx_fctl)
            ftl_val = ftl_val | (1 << portno);
        else
            ftl_val = ftl_val & (~(1 << portno));
        ret = skw_sdio_writeb(SKW_SDIO_RX_CHANNEL_FTL0, ftl_val);
    }
    else
    {
        portno = portno - 8;
        ret = skw_sdio_readb(SKW_SDIO_RX_CHANNEL_FTL1, &ftl_val);
        if (ret)
            return -1;

        if (rx_fctl)
            ftl_val = ftl_val | (1 << portno);
        else
            ftl_val = ftl_val & (~(1 << portno));
        ret = skw_sdio_writeb(SKW_SDIO_RX_CHANNEL_FTL1, ftl_val);
    }
    return ret;
}

void skw_sdio_exception_work(struct rt_delayed_work *work)
{
    int i = 0;
    int port_num = 5;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info(" entern..cp_state = %d.\n", skw_sdio->cp_state);
    mutex_lock(&skw_sdio->except_mutex);
    if (skw_sdio->cp_state != 1)
    {
        mutex_unlock(&skw_sdio->except_mutex);
        return;
    }
    skw_sdio->cp_state = DEVICE_BLOCKED_EVENT;
    mutex_unlock(&skw_sdio->except_mutex);
    if (!skw_sdio->host_state)
        modem_notify_event(DEVICE_BLOCKED_EVENT);

    skw_printf("%s %d\n", __func__, __LINE__);

    for (i = 0; i < port_num; i++)
    {
        if (!sdio_ports[i].state || sdio_ports[i].state == PORT_STATE_CLSE)
            continue;

        sdio_ports[i].state = PORT_STATE_ASST;
        complete(&sdio_ports[i].rx_done);

        if (i != 1)
            complete(&sdio_ports[i].tx_done);
        if (i == 0 || i == skw_log_port())
            sdio_ports[i].next_seqno = 1;

    }
    skw_printf("%s %d\n", __func__, __LINE__);
    skw_sdio->service_state_map = 0;
    skw_recovery_mode();
}
static void skw_sdio_rx_down(struct skw_sdio_data_t *skw_sdio)
{
    rt_sem_take(&skw_sdio->rx_completed, RT_WAITING_FOREVER);
}
void skw_sdio_rx_up(struct skw_sdio_data_t *skw_sdio)
{
    rt_sem_release(&skw_sdio->rx_completed);
}
void skw_sdio_dispatch_packets(struct skw_sdio_data_t *skw_sdio)
{
    int i;
    struct sdio_port *port;

    for (i = 0; i < max_ch_num; i++)
    {
        port = &sdio_ports[i];
        if (!port->state)
            continue;
        if (port->rx_rp != port->rx_wp)
            skw_sdio_dbg("port[%d] sg_index=%d (%d,%d)\n", i,
                         port->sg_index, port->rx_rp, port->rx_wp);
        if (port->rx_submit && port->sg_index)
        {
            debug_infos.last_rx_submit_time = skw_local_clock();
            port->rx_submit(port->channel, port->sg_rx, port->sg_index, port->rx_data);//注册的 rx 数据接收回调
            skw_sdio_dbg("port[%d] sg_index=%d (%d,%d)\n", i,
                         port->sg_index, port->rx_rp, port->rx_wp);
            port->sg_index = 0;
        }
    }
}
static void skw_sdio_sdma_set_nsize(unsigned int size)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int count;

    if (size == 0)
    {
        skw_sdio->next_size = max_pac_size;
        return;
    }

    count = (size >> 10) + 9;
    size = SKW_SDIO_ALIGN_BLK(size + (count << 3));
    skw_sdio->next_size = (size > SDIO_BUFFER_SIZE) ? SDIO_BUFFER_SIZE : size;
}

static void skw_sdio_adma_set_packet_num(unsigned int num)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    if (num == 0)
        num = 1;

    if (num >= MAX_SG_COUNT)
        skw_sdio->remain_packet = MAX_SG_COUNT;
    else
        skw_sdio->remain_packet = num;
}

/************************************************************************
 *Decription:release debug recovery auto test
 *Author:junwei.jiang
 *Date:2023-05-30
 *Modfiy:
 *
 ********************************************************************* */
int skw_sdio_recovery_debug(int disable)
{
    recovery_debug_status = disable;
    skw_sdio_info("the recovery status =%d\n", recovery_debug_status);
    return 0;
}

int skw_sdio_recovery_debug_status(void)
{
    skw_sdio_info("the recovery val =%d\n", recovery_debug_status);
    return recovery_debug_status;
}

int skw_sdio_bt_serv_debug(int enable)
{
    bt_serv_debug_status = enable;

    skw_sdio_info("the bt_service status =%d\n", bt_serv_debug_status);
    if (enable)
    {
        bt_service_start();
    }
    else
    {
        bt_service_stop();
    }
    return 0;
}

int skw_sdio_bt_serv_debug_status(void)
{
    skw_sdio_info("the bt_service val =%d\n", bt_serv_debug_status);

    return bt_serv_debug_status;
}
int skw_sdio_wifi_serv_debug(int enable)
{
    wifi_serv_debug_status = enable;

    skw_sdio_info("the wifi_service status =%d\n", wifi_serv_debug_status);
    if (enable)
    {
        wifi_service_start();
    }
    else
    {
        wifi_service_stop();
    }
    return 0;
}

int skw_sdio_wifi_serv_debug_status(void)
{
    skw_sdio_info("the wifi_service val =%d\n", wifi_serv_debug_status);
    return wifi_serv_debug_status;
}

static int force_cp_wakeup(void)
{
    int ret;
    u32 val;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    send_cp_wakeup_signal(skw_sdio);
    wait_for_completion_interruptible_timeout(&skw_sdio->device_wakeup, HZ / 100);
    val = gpio_get_value(skw_sdio->gpio_in);
    if (val)
        return 0;
    else
    {
        send_cp_wakeup_signal(skw_sdio);
        ret = wait_for_completion_interruptible_timeout(&skw_sdio->device_wakeup, HZ / 100);
        if (ret == 0)
            return -ETIMEDOUT;
        val = gpio_get_value(skw_sdio->gpio_in);
        if (val)
            return 0;
        else
            return -ETIMEDOUT;
    }

    return -ETIMEDOUT;
}

int update_download_flag(bool enable)
{
    int ret;
    u32 buffer;

    /* 1. read DL_FLAG reg */
    ret = force_cp_wakeup();
    if (ret)
    {
        skw_sdio_err("force cp wakeup failed %d\n", __LINE__);
        return ret;
    }
    ret = skw_sdio_dt_read(SKW_DL_FLAG_BASE, &buffer, 4);
    if (ret)
    {
        ret = force_cp_wakeup();
        if (ret)
        {
            skw_sdio_err("force cp wakeup failed %d\n", __LINE__);
            return ret;
        }
        ret = skw_sdio_dt_read(SKW_DL_FLAG_BASE, &buffer, 4);
        if (ret)
        {
            skw_sdio_err("read dl flag failed\n");
            return ret;
        }
    }

    if (enable == 1)
        buffer |= SKW_DL_FLAG_BIT_MASK;
    else
        buffer &= ~SKW_DL_FLAG_BIT_MASK;

    /* 2. update DL_FLAG bit */
    ret = force_cp_wakeup();
    if (ret)
    {
        skw_sdio_err("force cp wakeup failed %d\n", __LINE__);
        return ret;
    }
    ret = skw_sdio_dt_write(SKW_DL_FLAG_BASE, &buffer, 4);
    if (ret)
    {
        ret = force_cp_wakeup();
        if (ret)
        {
            skw_sdio_err("force cp wakeup failed %d\n", __LINE__);
            return ret;
        }
        ret = skw_sdio_dt_write(SKW_DL_FLAG_BASE, &buffer, 4);
        if (ret)
        {
            skw_sdio_err("write dl flag failed\n");
            return ret;
        }
    }
    /* 3. Make sure CP update flag successfully */
    ret = force_cp_wakeup();
    if (ret)
    {
        skw_sdio_err("force cp wakeup failed %d\n", __LINE__);
        return ret;
    }

    return 0;
}

static int skw_pin_config(void)
{
    return 0;
}

static int skw_sdio_handle_packet(struct skw_sdio_data_t *skw_sdio,
                                  scatterlist *sg, struct skw_packet_header *header, int portno)
{
    struct sdio_port  *port;
    int buf_size, i, ret;
    char *addr;
    u32 *data;
    if (portno >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[portno];
    port->rx_packet++;
    port->rx_count += header->len;
    if (portno == LOOPCHECK_PORT)
    {
        char *cmd = (char *)(header + 4);
        cmd[header->len - 12] = 0;
        skw_sdio_info("LOOPCHECK channel received: %s\n", (char *)cmd);
        if (header->len == 19 && !strncmp(cmd, "BTREADY", 7))
        {
            skw_sdio->service_state_map |= 2;
            //kernel_restart(0);
            skw_sdio->device_active = 1;
            complete(&skw_sdio->download_done);
        }
        else if (header->len == 21 && !strncmp(cmd, "WIFIREADY", 9))
        {
            skw_sdio->service_state_map |= 1;
            //kernel_restart(0);
            skw_sdio->device_active = 1;
            complete(&skw_sdio->download_done);
        }
        else if (!strncmp((char *)cmd, "BSPASSERT", 9))
        {
            debug_infos.cp_assert_time = skw_local_clock();
            if (!skw_sdio->cp_state)
                schedule_delayed_work(&skw_sdio->skw_except_work.work, msecs_to_jiffies(8000));

            mutex_lock(&skw_sdio->except_mutex);
            if (skw_sdio->cp_state == DEVICE_BLOCKED_EVENT)
            {
                if (skw_sdio->adma_rx_enable)
                    page_frag_free(header);

                mutex_unlock(&skw_sdio->except_mutex);
                return 0;
            }
            skw_sdio->cp_state = 1; /*cp except set value*/
            mutex_unlock(&skw_sdio->except_mutex);
            skw_sdio->service_state_map = 0;
            rt_memset(assert_context, 0, 1024);
            assert_context_size = 0;
            modem_notify_event(DEVICE_ASSERT_EVENT);
            skw_sdio_err(" bsp RT_ASSERT !!!\n");
        }
        else if (header->len == 20 && !strncmp(cmd, "DUMPDONE", 8))
        {
            mutex_lock(&skw_sdio->except_mutex);
            if (skw_sdio->cp_state == DEVICE_BLOCKED_EVENT)
            {
                if (skw_sdio->adma_rx_enable)
                    page_frag_free(header);

                mutex_unlock(&skw_sdio->except_mutex);
                return 0;
            }
            skw_sdio->cp_state = DEVICE_DUMPDONE_EVENT; /*cp except set value 2*/
            mutex_unlock(&skw_sdio->except_mutex);
            cancel_delayed_work_sync(&skw_sdio->skw_except_work.work);
#ifdef CONFIG_SEEKWAVE_PLD_RELEASE
            modem_notify_event(DEVICE_DUMPDONE_EVENT);
#else
            if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12) && !recovery_debug_status)
            {
                modem_notify_event(DEVICE_DUMPDONE_EVENT);
            }
#endif
            skw_sdio_err("The CP DUMPDONE OK : \n %d::%s\n", assert_context_size, assert_context);
            for (i = 0; i < 5; i++)
            {
                if (!sdio_ports[i].state || sdio_ports[i].state == PORT_STATE_CLSE)
                    continue;

                sdio_ports[i].state = PORT_STATE_ASST;
                complete(&(sdio_ports[i].rx_done));
                if (i != 1)
                    complete(&(sdio_ports[i].tx_done));
                if (i == 0 || i == skw_log_port())
                    sdio_ports[i].next_seqno = 1;
            }
#ifdef CONFIG_SEEKWAVE_PLD_RELEASE
            skw_recovery_mode();//recoverymode open api
#else
            if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12) && !recovery_debug_status
                    && skw_sdio->cp_state != DEVICE_BLOCKED_EVENT)
            {
                skw_recovery_mode();//recoverymode open api
            }
#endif
        }
        else if (!strncmp("trunk_W", cmd, 7))
        {
            if (!skw_sdio->cp_state)
                complete(&skw_sdio->download_done);

            if (skw_sdio->cp_state)
            {
                assert_info_print = 0;
                if (sdio_ports[0].state == PORT_STATE_ASST)
                    sdio_ports[0].state = PORT_STATE_OPEN;
                modem_notify_event(DEVICE_BSPREADY_EVENT);
                skw_sdio_info("send the bsp state to log service or others\n");
            }
            skw_sdio->host_state = 0;
            skw_sdio->cp_state = 0;
            //wake_up(&skw_sdio->wq);
            skw_sdio_info("cp_state = %d \n", skw_sdio->cp_state);
            rt_memset(firmware_version, 0, sizeof(firmware_version));
            strncpy(firmware_version, cmd, strlen(cmd));
            skw_sdio_info("firmware version: %s:%s \n", cmd, firmware_version);
            if (skw_sdio->boot_data->nv_mem_pnfg_data != NULL && skw_sdio->boot_data->nv_mem_pnfg_size != 0)
            {
                skw_sdio_info("UPDATE '%s' PINCFG from %s\n", (char *)skw_sdio->chip_id, skw_sdio->boot_data->skw_nv_name);
                ret = skw_pin_config();
                if (ret)
                    skw_sdio_err("Update pin config failed!!!\n");
            }
        }
        else if (!strncmp(cmd, "BSPREADY", 8))
        {
            loopcheck_send_data("RDVERSION", 9);
        }
        skw_sdio_dbg("Line:%d the port=%d \n", __LINE__, port->channel);
        if (skw_sdio->adma_rx_enable)
            page_frag_free(header);
        return 0;
    }
    if (!port->state)
    {
        if (skw_sdio->adma_rx_enable)
        {
            if (!IS_LOG_PORT(portno))
                skw_sdio_err("port%d discard data for wrong state\n", portno);
            page_frag_free(header);
            return 0;
        }
    }
    if (port->sg_rx && port->rx_data)
    {
        if (port->sg_index >= MAX_SG_COUNT)
        {
            skw_sdio_err(" rx sg_buffer is overflow!\n");
        }
        else
        {
            //sg_set_buf(&port->sg_rx[port->sg_index++], header, header->len+4);//需要具体实现
        }
    }
    else
    {
        int packet = 0, total = 0;
        mutex_lock(&port->rx_mutex);
        buf_size = (port->length + port->rx_wp - port->rx_rp) % port->length;
        buf_size = port->length - 1 - buf_size;
        addr = (char *)(header + 1);
        data = (u32 *) addr;
        if (((data[2] & 0xffff) != port->next_seqno) &&
                (header->len > 12) && !IS_LOG_PORT(portno))
        {
            skw_sdio_err("portno:%d, packet lost recv seqno=%d expected %d\n", port->channel,
                         data[2] & 0xffff, port->next_seqno);
            if (skw_sdio->adma_rx_enable)
                page_frag_free(header);
            mutex_unlock(&port->rx_mutex);
            return 0;
        }
        if (header->len > 12)
        {
            port->next_seqno++;
            addr += 12;
            header->len -= 12;
            total = data[1] >> 8;
            packet = data[2] & 0xFFFF;
        }
        else if (header->len == 12)
        {
            header->len = 0;
            port->tx_flow_ctrl--;
            complete(&port->tx_done);
            skw_port_log(portno, "%s link msg: 0x%x 0x%x port%d: %d \n", __func__,
                         data[0], data[1], portno, port->tx_flow_ctrl);
        }
        if (skw_sdio->cp_state)
        {
            if (header->len != 245 || buf_size < 2048)
            {
                if (assert_info_print++ < 28 && strncmp((const char *)addr, "+LOG", 4))
                {
                    if (assert_context_size + header->len < sizeof(assert_context))
                    {
                        memcpy(assert_context + assert_context_size, addr, header->len);
                        assert_context_size += header->len;
                    }
                }
            }
            if (buf_size < 2048)
                msleep(10);
        }
        if (port->rx_submit && !port->sg_rx)
        {
            if (header->len && port->pdev)
                port->rx_submit(portno, port->rx_data, header->len, addr);
        }
        else if (buf_size < header->len)
        {
            skw_port_log(portno, "%s port%d overflow:buf_size %d-%d, packet size %d (w,r)=(%d, %d)\n",
                         __func__, portno, buf_size, port->length, header->len,
                         port->rx_wp,  port->rx_rp);
        }
        else if (port->state && header->len)
        {
            if (port->length - port->rx_wp > header->len)
            {
                memcpy(&port->read_buffer[port->rx_wp], addr, header->len);
                port->rx_wp += header->len;
            }
            else
            {
                memcpy(&port->read_buffer[port->rx_wp], addr, port->length - port->rx_wp);
                memcpy(&port->read_buffer[0], &addr[port->length - port->rx_wp],
                       header->len - port->length + port->rx_wp);
                port->rx_wp = header->len - port->length + port->rx_wp;
            }

            if (!port->rx_flow_ctrl && buf_size - header->len < (port->length / 3))
            {
                port->rx_flow_ctrl = 1;
                skw_sdio_rx_port_follow_ctl(portno, port->rx_flow_ctrl);
            }
            mutex_unlock(&port->rx_mutex);
            complete(&port->rx_done);
            if (skw_sdio->adma_rx_enable)
                page_frag_free(header);
            return 0;
        }
        mutex_unlock(&port->rx_mutex);
        if (skw_sdio->adma_rx_enable)
            page_frag_free(header);
    }
    return 0;
}
//extern wm_wifi_t wm_wifi;
//#include "wifi-internal.h"
#include "fsl_os_abstraction.h"
#include "skw_type_msg.h"
#include "dfs_file.h"
extern struct dfs_fd wifi_log_fd;
static int skw_sdio2_handle_packet(struct skw_sdio_data_t *skw_sdio,
                                   scatterlist *sg, struct skw_packet2_header *header, int portno)
{
    struct sdio_port  *port;
    int buf_size, i, ret;
    char *addr;
    u32 *data;

    if (portno >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[portno];
    port->rx_packet++;
    port->rx_count += header->len;
    if (portno == SDIO2_LOOPCHECK_PORT)
    {
        char *cmd = (char *)(header + 4);
        cmd[header->len - 12] = 0;
        skw_sdio_info("LOOPCHECK channel received: %s\n", (char *)cmd);
        skw_printf("%s %d header.len=%x pad=%x,eof=%x,channel=%x\n", __func__, __LINE__, header->len, header->pad, header->eof, header->channel);
        if (header->len == 19 && !strncmp(cmd, "BTREADY", 7))
        {
            skw_sdio->service_state_map |= 2;
            //kernel_restart(0);
            skw_sdio->device_active = 1;
            complete(&skw_sdio->download_done);
        }
        else if (header->len == 21 && !strncmp(cmd, "WIFIREADY", 9))
        {
            skw_sdio->service_state_map |= 1;
            //kernel_restart(0);
            skw_sdio->device_active = 1;
            complete(&skw_sdio->download_done);
            skw_printf("%s %d header.len=%x pad=%x,eof=%x,channel=%x\n", __func__, __LINE__, header->len, header->pad, header->eof, header->channel);
            goto end;
            return 0;
        }
        else if (!strncmp((char *)cmd, "BSPASSERT", 9))
        {
            RT_ASSERT(0);
            debug_infos.cp_assert_time = skw_local_clock();
            if (!skw_sdio->cp_state && (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12)))
                schedule_delayed_work(&skw_sdio->skw_except_work.work, msecs_to_jiffies(8000));

            mutex_lock(&skw_sdio->except_mutex);
            if (skw_sdio->cp_state == DEVICE_BLOCKED_EVENT)
            {
                if (skw_sdio->adma_rx_enable)
                    page_frag_free(header);

                mutex_unlock(&skw_sdio->except_mutex);
                goto end;
                return 0;
            }
            skw_sdio->cp_state = 1; /*cp except set value*/
            mutex_unlock(&skw_sdio->except_mutex);
            skw_sdio->service_state_map = 0;
            rt_memset(assert_context, 0, 1024);
            assert_context_size = 0;
            modem_notify_event(DEVICE_ASSERT_EVENT);
            skw_sdio_err(" bsp RT_ASSERT !!!\n");
        }
        else if (header->len == 20 && !strncmp(cmd, "DUMPDONE", 8))
        {
            mutex_lock(&skw_sdio->except_mutex);
            if (skw_sdio->cp_state == DEVICE_BLOCKED_EVENT)
            {
                if (skw_sdio->adma_rx_enable)
                    page_frag_free(header);

                mutex_unlock(&skw_sdio->except_mutex);
                goto end;
                return 0;
            }
            skw_sdio->cp_state = DEVICE_DUMPDONE_EVENT; /*cp except set value 2*/
            mutex_unlock(&skw_sdio->except_mutex);
            cancel_delayed_work_sync(&skw_sdio->skw_except_work.work);
#ifdef CONFIG_SEEKWAVE_PLD_RELEASE
            modem_notify_event(DEVICE_DUMPDONE_EVENT);
#else
            if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12) && !recovery_debug_status)
            {
                modem_notify_event(DEVICE_DUMPDONE_EVENT);
            }
#endif
            skw_sdio_err("The CP DUMPDONE OK : \n %d::%s\n", assert_context_size, assert_context);
            for (i = 0; i < 5; i++)
            {
                if (!sdio_ports[i].state || sdio_ports[i].state == PORT_STATE_CLSE)
                    continue;

                sdio_ports[i].state = PORT_STATE_ASST;
                complete(&(sdio_ports[i].rx_done));
                if (i != 1)
                    complete(&(sdio_ports[i].tx_done));
                if (i == 1)
                    sdio_ports[i].next_seqno = 1;
            }
#ifdef CONFIG_SEEKWAVE_PLD_RELEASE
            skw_recovery_mode();//recoverymode open api
#else
            if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12) && !recovery_debug_status
                    && skw_sdio->cp_state != DEVICE_BLOCKED_EVENT)
            {
                skw_recovery_mode();//recoverymode open api
            }
#endif

        }
        else if (!strncmp("trunk_W", cmd, 7))
        {
            rt_memset(firmware_version, 0, sizeof(firmware_version));
            strncpy(firmware_version, cmd, strlen(cmd));
            skw_sdio_info("firmware version: %s:%s \n", cmd, firmware_version);
            if (!skw_sdio->cp_state)
                complete(&skw_sdio->download_done);

            if (skw_sdio->cp_state)
            {
                assert_info_print = 0;
                if (sdio_ports[0].state == PORT_STATE_ASST)
                    sdio_ports[0].state = PORT_STATE_OPEN;
                modem_notify_event(DEVICE_BSPREADY_EVENT);
            }
            skw_sdio->host_state = 0;
            skw_sdio->cp_state = 0;
            if (skw_sdio->boot_data->nv_mem_pnfg_data != NULL && skw_sdio->boot_data->nv_mem_pnfg_size != 0)
            {
                skw_sdio_info("UPDATE '%s' PINCFG from %s\n", (char *)skw_sdio->chip_id, skw_sdio->boot_data->skw_nv_name);
                ret = skw_pin_config();
                if (ret)
                    skw_sdio_err("Update pin config failed!!!\n");
            }
        }
        else if (!strncmp(cmd, "BSPREADY", 8))
        {
            loopcheck_send_data("RDVERSION", 9);
        }
        skw_sdio_dbg("Line:%d the port=%d \n", __LINE__, port->channel);
//      if(skw_sdio->adma_rx_enable)
//          page_frag_free(header);
        goto end;
        return 0;
    }
    // skw_sdio_info("%s %d wifi_read buff port=%d len=%d rx_data_len=%d,rx_data=%p\n",__func__, __LINE__,
    //                          port->channel,header->len,port->rx_data_len,port->rx_data);
    if (header->len == 12)
    {
        header->len = 0;
        port->tx_flow_ctrl--;
        skw_port_log(portno, "%s link msg: 0x%x 0x%x 0x%x: %d\n", __func__,
                     data[0], data[1], data[2], port->tx_flow_ctrl);
        complete(&port->tx_done);
    }
    else if (SDIO2_WIFI_CMD_PORT == portno)
    {
        //cmd 的回调
        port->rx_count = header->len;
        struct skw_msg *msg = (struct skw_msg *)(sg + 16);
        switch (msg->type)
        {
        case SKW_MSG_CMD_ACK:
            if (port->rx_data) rt_memcpy(port->rx_data, sg + 16 + sizeof(struct skw_msg), port->rx_data_len);
            complete(&port->rx_done);
            break;
        case SKW_MSG_EVENT:
            break;
        }
        void wm_wifi_event(void *sg, int len, int type);
        wm_wifi_event((void *)sg, header->len, SDIO2_WIFI_CMD_PORT);
        return 0;
    }
    else if (SDIO2_WIFI_DATA_PORT == portno)
    {
        //data的回调
        void wm_wifi_event(void *sg, int len, int type);
        wm_wifi_event((void *)sg, header->len, SDIO2_WIFI_DATA_PORT);
        return 0;
    }
    else if (SDIO2_BSP_LOG_PORT == portno)
    {
        // skw_printf("%s %d TX log data header->len=%d\n",__func__, __LINE__, header->len);
        // skw_printf("%s\n",sg + 16);
        extern uint8_t wifi_dfs_log;
        if (wifi_dfs_log)
        {
            dfs_file_write(&wifi_log_fd, "\r\n", 2);
            dfs_file_write(&wifi_log_fd, sg + 16, header->len - 12);
        }
        goto end;
        return 0;
    }
end:
    //rt_free(header);
    return 0;
}
int send_modem_assert_command(void)
{
    int ret = 0;

    return ret;
}

/* for adma */
static int skw_sdio_adma_parser(struct skw_sdio_data_t *skw_sdio, scatterlist *sgs,
                                int packet_count)
{
    struct skw_packet_header *header = NULL;
    unsigned int i;
    int channel = 0;
    unsigned int parse_len = 0;
    uint32_t *data;
    struct sdio_port *port;

    port = &sdio_ports[0];
    for (i = 0; i < packet_count; i++)
    {
        header = (struct skw_packet_header *)sg_virt(sgs + i);
        data = (uint32_t *)header;
        if (atomic_read(&skw_sdio->suspending))
            skw_sdio_info("ch:%d len:%d 0x%x 0x%x\n", header->channel, header->len, data[2], data[3]);
        skw_port_log(header->channel, "%s[%d]:ch:%d len:0x%0x 0x%08X 0x%08X : 0x%08X 0x%08x 0x%08X\n", __func__,
                     i,  header->channel, header->len, data[2], data[3], data[5], data[6], data[7]);
        channel = header->channel;

        if (!header->eof && (channel < max_ch_num) && header->len)
        {
            parse_len += header->len;
            data = (uint32_t *)(header + 1);
            if ((channel >= max_ch_num) || (header->len >
                                            (max_pac_size - sizeof(struct skw_packet_header))) ||
                    (header->len == 0))
            {
                skw_sdio_err("%s invalid header[%d]len[%d]: 0x%x 0x%x\n",
                             __func__,  header->channel, header->len, data[0], data[1]);
                page_frag_free(header);
                continue;
            }
            skw_sdio->rx_packer_cnt++;
            skw_sdio_handle_packet(skw_sdio, sgs + i, header, channel);
        }
        else
        {
            skw_sdio_err("%s[%d]:ch:%d len:0x%0x 0x%08X 0x%08X : 0x%08X 0x%08x 0x%08X\n", __func__,
                         i,  header->channel, header->len, data[2], data[3], data[5], data[6], data[7]);

            page_frag_free(header);
            continue;
        }
    }
    if (debug_infos.last_irq_time && (channel > 0 && channel < max_ch_num))
    {
        if (channel > SDIO2_MAX_CH_NUM)
            skw_sdio_err("line: %d channel number error %d %d\n", __LINE__, channel, SDIO2_MAX_CH_NUM);
        debug_infos.chn_last_irq_time[channel][debug_infos.chn_irq_cnt[channel] % CHN_IRQ_RECORD_NUM] = debug_infos.last_irq_time;
        debug_infos.chn_irq_cnt[channel]++;
    }
    //atomic_set(skw_sdio->suspending, 0);
    skw_sdio->suspending = 0;
    return 0;
}

static int skw_sdio2_adma_parser(struct skw_sdio_data_t *skw_sdio, scatterlist *sgs,
                                 int packet_count)
{
    struct skw_packet2_header *header = NULL;
    unsigned int i;
    int channel = 0;
    unsigned int parse_len = 0;
    uint32_t *data;
    struct sdio_port *port;

    port = &sdio_ports[0];
    skw_printf("%s %d packet_count=%d,skw_sdio->rx_packer_cnt=%d\n", __func__, __LINE__, packet_count, skw_sdio->rx_packer_cnt);
    for (i = 0; i < packet_count; i++)
    {
        header = (struct skw_packet2_header *)sg_virt(sgs + i * 1536);
        skw_printf("%s %d header->len =0x%x header=%p\n", __func__, __LINE__, header->len, header);
        data = (uint32_t *)header;
        channel = header->channel;
        if (!header->eof && (channel < max_ch_num) && header->len)
        {
            data = (uint32_t *)(header + 1);
            if ((channel >= max_ch_num) || (header->len > (max_pac_size - sizeof(struct skw_packet2_header))) ||
                    (header->len == 0))
            {
                skw_printf("%s %d eof=%x,channel=%x,pad=%x len=%x\n", __func__, __LINE__, header->eof, header->channel, header->pad, header->len);
                //page_frag_free(sgs);
                continue;
            }
            //skw_sdio->rx_packer_cnt++;
            skw_sdio2_handle_packet(skw_sdio, sgs + i * 1536, header, channel);
            // parse_len += header->len;
            // parse_len += 4; // add 4 for the header
        }
        else break;
        // {
        //     skw_sdio_err("%s[%d]:ch:%d len:0x%0x 0x%08X 0x%08X : 0x%08X 0x%08x 0x%08X\n", __func__,
        //                  i,  header->channel, header->len, data[2], data[3], data[5], data[6], data[7]);
        //     //RT_ASSERT(0);
        //     continue;
        // }
    }
    if (rx_data_t.flags == 0) rx_data_t.flags0 = 0;
    else if (rx_data_t.flags == 1) rx_data_t.flags1 = 0;
    skw_sdio->suspending = 0;
    return 0;
}
/* for normal dma */
static int skw_sdio_sdma_parser(char *data_buf, int total)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct skw_packet_header *header = NULL;
    int channel;
    uint32_t *data;
    unsigned char *p = NULL;
    unsigned int parse_len = 0;
    int current_len = 0;
#if 0
    print_hex_dump(KERN_ERR, "skw_rx_buf:", 0, 16, 1,
                   data_buf, total, 1);
#endif
    header = (struct skw_packet_header *)data_buf;
    for (parse_len = 0; parse_len < total;)
    {
        if (header->eof != 0)
            break;
        p = (unsigned char *)header;
        data = (uint32_t *)header;
        if (atomic_read(&skw_sdio->suspending))
            skw_sdio_info("ch:%d len:%d 0x%x 0x%x\n", header->channel, header->len, data[2], data[3]);
        skw_port_log(header->channel, "%s:ch:%d len:0x%0x 0x%08X 0x%08X : 0x%08X 0x%08x 0x%08X\n", __func__,
                     header->channel, header->len, data[1], data[2], data[3], data[4], data[5]);
        channel = header->channel;
        current_len = header->len;
        parse_len += current_len;
        if ((channel >= max_ch_num) || (current_len == 0) ||
                (current_len > (max_pac_size - sizeof(struct skw_packet_header))))
        {
            skw_sdio_err("%s skip [%d]len[%d]\n", __func__, header->channel, current_len);
            break;
        }
        skw_sdio->rx_packer_cnt++;
        skw_sdio_handle_packet(skw_sdio, NULL, header, channel);
        skw_port_log(header->channel, "the -header->len----%d\n", current_len);
        /* pointer to next packet header*/
        p += sizeof(struct skw_packet_header) + SKW_SDIO_ALIGN_4BYTE(current_len);
        header = (struct skw_packet_header *)p;
    }
    if (debug_infos.last_irq_time && (channel > 0 && channel < max_ch_num))
    {
        if (channel > SDIO2_MAX_CH_NUM)
            skw_sdio_err("line: %d channel number error %d %d\n", __LINE__, channel, SDIO2_MAX_CH_NUM);
        debug_infos.chn_last_irq_time[channel][debug_infos.chn_irq_cnt[channel] % CHN_IRQ_RECORD_NUM] = debug_infos.last_irq_time;
        debug_infos.chn_irq_cnt[channel]++;
    }
    //atomic_set(skw_sdio->suspending, 0);
    skw_sdio->suspending = 0;
    return 0;
}
static int skw_sdio2_sdma_parser(char *data_buf, int total)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct skw_packet2_header *header = NULL;
    int channel;
    uint32_t *data;
    unsigned char *p = NULL;
    unsigned int parse_len = 0;
    int current_len = 0;
#if 0
    print_hex_dump(KERN_ERR, "skw_rx_buf:", 0, 16, 1,
                   data_buf, total, 1);
#endif
    header = (struct skw_packet2_header *)data_buf;
    for (parse_len = 0; parse_len < total;)
    {
        if (header->eof != 0)
            break;
        p = (unsigned char *)header;
        data = (uint32_t *)header;
        if (atomic_read(&skw_sdio->suspending))
            skw_sdio_info("ch:%d len:%d 0x%x 0x%x\n", header->channel, header->len, data[2], data[3]);
        skw_port_log(header->channel, "ch:%d len:0x%0x 0x%08X 0x%08X : 0x%08X 0x%08x 0x%08X\n",
                     header->channel, header->len, data[1], data[2], data[3], data[4], data[5]);
        channel = header->channel;
        current_len = header->len;
        parse_len += current_len;
        if ((channel >= max_ch_num) || (current_len == 0) ||
                (current_len > (max_pac_size - sizeof(struct skw_packet2_header))))
        {
            skw_sdio_err("%s skip [%d]len[%d]\n", __func__, header->channel, current_len);
            break;
        }
        //skw_sdio->rx_packer_cnt++;
        skw_sdio2_handle_packet(skw_sdio, NULL, header, channel);
        skw_port_log(header->channel, "the -header->len----%d\n", current_len);
        /* pointer to next packet header*/
        p += sizeof(struct skw_packet2_header) + SKW_SDIO_ALIGN_4BYTE(current_len);
        header = (struct skw_packet2_header *)p;
    }
    if (debug_infos.last_irq_time && (channel > 0 && channel < max_ch_num))
    {
        if (channel > SDIO2_MAX_CH_NUM)
            skw_sdio_err("line: %d channel number error %d %d\n", __LINE__, channel, SDIO2_MAX_CH_NUM);
        debug_infos.chn_last_irq_time[channel][debug_infos.chn_irq_cnt[channel] % CHN_IRQ_RECORD_NUM] = debug_infos.last_irq_time;
        debug_infos.chn_irq_cnt[channel]++;
    }
    //atomic_set(skw_sdio->suspending, 0);
    skw_sdio->suspending = 0;
    return 0;
}
scatterlist *skw_sdio_prepare_adma_buffer(struct skw_sdio_data_t *skw_sdio, int *sg_count, int *nsize_offset)
{
    scatterlist *sgs;
    void    *buffer;
    int i, j, data_size;
    int alloc_size = PAGE_SIZE;

    sgs = kzalloc((*sg_count) * sizeof(scatterlist), 0);

    if (sgs == NULL)
        return NULL;

    for (i = 0; i < (*sg_count) - 1; i++)
    {
        buffer = skw_sdio_alloc_frag(alloc_size, 0);
        if (buffer)
            sg_set_buf(&sgs[i], buffer, max_pac_size);
        else
        {
            *sg_count = i + 1;
            break;
        }
    }

    if (i <= 0)
        goto err;
    //sg_mark_end(&sgs[*sg_count - 1]);
    data_size = max_pac_size * ((*sg_count) - 1);
    data_size = data_size % SKW_SDIO_NSIZE_BUF_SIZE;
    *nsize_offset = SKW_SDIO_NSIZE_BUF_SIZE - data_size;
    if (*nsize_offset < 8)
        *nsize_offset = SKW_SDIO_NSIZE_BUF_SIZE + *nsize_offset;
    *nsize_offset = *nsize_offset + SKW_SDIO_NSIZE_BUF_SIZE;
    sg_set_buf(sgs + i, skw_sdio->next_size_buf, *nsize_offset);
    return sgs;
err:
    skw_sdio_err("%s failed\n", __func__);
    for (j = 0; j < i; j++)
        page_frag_free(sg_virt(sgs + j));
    kfree(sgs);
    return NULL;

}
//static uint8_t rx_wifi_data_t[2048] = {0};
ALIGN(32) static  uint8_t data0[SKW_WIFI_MAX_PACKET_SIZE];
ALIGN(32) static  uint8_t data1[SKW_WIFI_MAX_PACKET_SIZE];
void skw_sdio_rx_thread(void *parameter)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int read_len, buf_num;
    int ret = 0;
    unsigned int rx_nsize = 0;
    unsigned int valid_len = 0;
    char *rx_buf;
    scatterlist *sgs = NULL;
    char fifo_ind;
    unsigned char reg = 0;
    //skw_printf("%s %d\n",__func__,__LINE__);

    skw_sdio_sdma_set_nsize(0);
    skw_sdio_adma_set_packet_num(1);
    cp_fifo_status = 0;
    while (1)
    {
        /* Wait the semaphore */
        skw_sdio_rx_down(skw_sdio);//等待中断

        if (skw_sdio->threads_exit)
        {
            skw_sdio_err("line %d threads exit\n", __LINE__);
            break;
        }
        cp_fifo_status = fifo_ind;
receive_again:
        if (skw_sdio->adma_rx_enable)
        {
            int nsize_offset, read_buff_size = 0;
            buf_num = skw_sdio->remain_packet;
            if (buf_num > MAX_PAC_COUNT)//每次最大读20个包 20 * 1536 = 30720
                buf_num = MAX_PAC_COUNT;
            //buf_num = 1;
            read_buff_size = (buf_num * max_pac_size) + SKW_SDIO_BLK_SIZE;

data_buff_wait:
            if (rx_data_t.flags0 == 0)
            {
                sgs = &data0[0];
                rx_data_t.flags0 = 1;
                rx_data_t.flags = 0;
                skw_printf("%s %d sgs=%p\n", __func__, __LINE__, sgs);
            }
            else if (rx_data_t.flags1 == 0)
            {
                sgs = &data1[0];
                rx_data_t.flags1 = 1;
                rx_data_t.flags = 1;
                skw_printf("%s %d sgs=%p\n", __func__, __LINE__, sgs);
            }
            else
            {
                skw_printf("%s %d sgs=%p\n", __func__, __LINE__, sgs);
                goto data_buff_wait;
            }

            skw_printf("%s %d buf_num=%d read_buff_size=%d\n", __func__, __LINE__, buf_num, read_buff_size);
            ret = skw_sdio_adma_read(skw_sdio, sgs, read_buff_size, SKW_SDIO_PK_MODE_ADDR); // 一次读2048个
            //skw_printf("%s %d end\n",__func__,__LINE__);
            if (ret != 0)
            {
                skw_sdio_err("%s adma read fail ret:%d\n", __func__, ret);
                goto submit_packets;
            }

            rx_nsize = *((uint32_t *)(sgs + (buf_num * max_pac_size) + SKW_PACKET_NUM_OFFSET)); //还剩余多少个packet

#if SKW_WIFI_DUMP
            struct skw_packet2_header *h = (struct skw_packet2_header *)sgs;
            skw_printf("%s %d rx_nsize =%d h->len=%d\n", __func__, __LINE__, rx_nsize, h->len + 4);
            if (h->channel != 9)
            {
                for (int i = 0; i < read_buff_size; i++)
                {
                    if (!(i % 8)) skw_printf("    ");
                    if (!(i % 16)) skw_printf("\n");
                    skw_printf("%02x ", sgs[i]);
                }
                skw_printf("\n");
            }
#endif
            skw_sdio2_adma_parser(skw_sdio, sgs, buf_num);

            //rt_free(sgs);
        }

submit_packets:
        skw_sdio_dispatch_packets(skw_sdio);//接收回调
        skw_printf("%s %d rx_nsize:%d adma_rx_enable=%d\n", __func__, __LINE__, rx_nsize, skw_sdio->adma_rx_enable);
        if (skw_sdio->adma_rx_enable)
            skw_sdio_adma_set_packet_num(rx_nsize);

        if (rx_nsize > 0)
            goto receive_again;//继续读

        debug_infos.last_irq_time = 0;
        skw_sdio_unlock_rx_ws(skw_sdio);
    }
    skw_sdio_info("%s exit\n", __func__);

    return;
}
int skw_sdio_port_init(int id, void *callback, void *data)
{
    struct sdio_port *port;

    if (id >= max_ch_num)
        return -EINVAL;

    port = &sdio_ports[id];
    if ((port->state == PORT_STATE_OPEN) || port->rx_submit)
        return -EBUSY;
    port->rx_submit = callback;
    port->rx_data = data;
    init_completion(&port->rx_done);
    init_completion(&port->tx_done);
    mutex_init(&port->rx_mutex);
    port->state = PORT_STATE_OPEN;
    port->tx_flow_ctrl = 0;
    port->rx_flow_ctrl = 0;
    return 0;
}

static int open_sdio_port(int id, void *callback, void *data)
{
    struct sdio_port *port;

    if (id >= max_ch_num)
        return -EINVAL;

    port = &sdio_ports[id];
    if ((port->state == PORT_STATE_OPEN) || port->rx_submit)
        return -EBUSY;
    port->rx_submit = callback;
    port->rx_data = data;
    init_completion(&port->rx_done);
    init_completion(&port->tx_done);
    mutex_init(&port->rx_mutex);
    port->state = PORT_STATE_OPEN;
    port->tx_flow_ctrl = 0;
    port->rx_flow_ctrl = 0;
    if (id && id != skw_log_port())
    {
        port->next_seqno = 1; //cp start seqno default no 1
        port->rx_wp = port->rx_rp = 0;
    }
    if (id == skw_log_port())
    {
        skw_sdio_cp_log_disable(0);
    }
    //skw_sdio_info("%s(%d) %s portno = %d\n", current->comm, current->pid, __func__, id);
    return 0;
}
static int close_sdio_port(int id)
{
    struct sdio_port *port;

    if (id >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[id];
    //skw_sdio_info("%s(state=%d) portno = %d\n", current->comm, port->state, id);
    if (!port->state)
        return -ENODEV;
    port->state = PORT_STATE_CLSE;
    port->rx_submit = NULL;
    if (id == skw_log_port())
    {
        skw_sdio_cp_log_disable(1);
    }
    complete(&port->rx_done);
    return 0;
}

void send_host_suspend_indication(struct skw_sdio_data_t *skw_sdio)
{
    uint32_t value;
    uint32_t timeout = 2000;
    if (skw_sdio->gpio_out && skw_sdio->resume_com)
    {
        skw_sdio_dbg("%s enter gpio=0\n", __func__);
        skw_sdio->host_active = 0;
        gpio_set_value(skw_sdio->gpio_out, 0);
        skw_sdio->device_active = 0;
        do
        {
            value = gpio_get_value(skw_sdio->gpio_in);
            if (value == 0)
                break;
            udelay(10);
        }
        while (timeout--);
    }
    else
        skw_sdio_dbg("%s enter\n", __func__);
}

void send_host_resume_indication(struct skw_sdio_data_t *skw_sdio)
{
    if (skw_sdio->gpio_out >= 0)
    {
        skw_sdio_dbg("%s enter\n", __func__);
        skw_sdio->host_active = 1;
        gpio_set_value(skw_sdio->gpio_out, 1);
        skw_sdio->resume_com = 1;
    }
}

static void send_cp_wakeup_signal(struct skw_sdio_data_t *skw_sdio)
{
    if (skw_sdio->gpio_out < 0)
        return;

    gpio_set_value(skw_sdio->gpio_out, 0);
    udelay(5);
    gpio_set_value(skw_sdio->gpio_out, 1);
}

extern int skw_sdio_enable_async_irq(void);
int try_to_wakeup_modem(int portno)
{
    int ret = 0;
    int val;
    unsigned long flags;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    rt_completion_init(&skw_sdio->device_wakeup);
    skw_sdio->tx_req_map |= 1 << portno;
    skw_sdio_info("%s enter gpio_val=%d : %d\n", __func__, skw_sdio->device_active, skw_sdio->resume_com);
    skw_port_log(portno, "%s enter device_active=%d : %d\n", __func__, skw_sdio->device_active, skw_sdio->resume_com);

    if (val && !skw_sdio->sdio_func[FUNC_1]->irq_handler &&
            !skw_sdio->resume_com && skw_sdio->irq_type == SKW_SDIO_INBAND_IRQ)
    {
        sdio_claim_host(skw_sdio->sdio_func[FUNC_1]);
        ret = sdio_claim_irq(skw_sdio->sdio_func[FUNC_1], skw_sdio_inband_irq_handler);
        //ret = skw_sdio_enable_async_irq();//PM
        if (ret < 0)
            skw_sdio_err("enable sdio async irq fail ret = %d\n", ret);
        sdio_release_host(skw_sdio->sdio_func[FUNC_1]);
        skw_port_log(portno, "%s enable SDIO inband IRQ ret=%d\n", __func__, ret);
    }
    return ret;
}

int wakeup_modem(int portno)
{
    int ret = 0;

    return ret;
}

void host_gpio_in_routine(int value)
{

}

static int setup_sdio_packet(void *packet, u8 channel, char *msg, int size)
{
    struct skw_packet_header *header = NULL;
    u32 *data = packet;

    data[0] = 0;
    header = (struct skw_packet_header *)data;
    header->channel = channel;
    header->len = size;
    memcpy(data + 1, msg, size);
    data++;
    data[size >> 2] = 0;
    header = (struct skw_packet_header *)&data[size >> 2];
    header->eof = 1;
    size += 8;
    return size;
}
static int setup_sdio2_packet(void *packet, u8 channel, char *msg, int size)
{
    struct skw_packet2_header *header = NULL;
    u32 *data = packet;

    data[0] = 0;
    header = (struct skw_packet2_header *)data;
    header->channel = channel;
    header->len = size;
    memcpy(data + 1, msg, size);
    data++;
    data[size >> 2] = 0;
    header = (struct skw_packet2_header *)&data[size >> 2];
    header->eof = 1;
    size += 8;
    return size;
}
int loopcheck_send_data(char *buffer, int size)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_port *port;
    int ret, count;
    port = &sdio_ports[LOOPCHECK_PORT];
    count = (size + 3) & 0xFFFFFFFC;
    if (count + 8 < port->length)
    {
        if (skw_cp_ver == SKW_SDIO_V10)
        {
            count = setup_sdio_packet(port->write_buffer, port->channel, buffer, count);
        }
        else
        {
            count = setup_sdio2_packet(port->write_buffer, port->channel, buffer, count);
        }
        try_to_wakeup_modem(LOOPCHECK_PORT);
        if (!(ret = skw_sdio_sdma_write(port->write_buffer, count)))
        {
            port->total += count;
            port->sent_packet++;
            ret = size;
        }
        skw_sdio->tx_req_map &= ~(1 << LOOPCHECK_PORT);
        return ret;
    }
    return -ENOMEM;
}
static int send_data(int portno, char *buffer, int size)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_port *port;
    int ret, count, i;
    u32 *data = (u32 *)buffer;
    if (size == 0)
        return 0;
    if (portno >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[portno];
    if (!port->state || skw_sdio->cp_state)
        return -EIO;

    if (port->state == PORT_STATE_CLSE)
    {
        port->state = PORT_STATE_IDLE;
        return -EIO;
    }

    count = (size + 3) & 0xFFFFFFFC;
    if (count + 8 < port->length)
    {
        if (skw_cp_ver == SKW_SDIO_V10)
        {
            count = setup_sdio_packet(port->write_buffer, port->channel, buffer, count);
        }
        else
        {
            count = setup_sdio2_packet(port->write_buffer, port->channel, buffer, count);
        }
        //skw_reinit_completion(port->tx_done);
        init_completion(&port->tx_done);
        for (i = 0; i < 2; i++)
        {
            try_to_wakeup_modem(portno);

            if (skw_sdio->cp_state)
                return -EIO;

            if (!(ret = skw_sdio_sdma_write(port->write_buffer, count)))
            {
                port->tx_flow_ctrl++;
                if (sdio_ports[portno].state != PORT_STATE_ASST)
                {
                    ret = wait_for_completion_interruptible_timeout(&port->tx_done,
                            msecs_to_jiffies(100));
                    if (!ret && port->tx_flow_ctrl)
                    {
                        skw_sdio_info("%s ret=%d:%d and retry again\n", __func__, ret, port->tx_flow_ctrl);
                        port->tx_flow_ctrl--;
                        continue;
                    }
                }
                port->total += count;
                port->sent_packet++;
                ret = size;
                break;
            }
            else
            {
                skw_sdio_info("%s ret=%d\n", __func__, ret);
                if (ret == -ETIMEDOUT && !skw_sdio->device_active)
                    continue;
            }
        }
        skw_sdio->tx_req_map &= ~(1 << portno);
        skw_port_log(portno, "%s port%d size=%d 0x%x 0x%x\n",
                     __func__, portno, size, data[0], data[1]);
        return ret;
    }
    else
    {
        for (i = 0; i < 2; i++)
        {
            try_to_wakeup_modem(portno);
            if (!(ret = skw_sdio_sdma_write(buffer, count)))
            {
                port->total += count;
                port->sent_packet++;
                ret = size;
                break;
            }
            else
            {
                skw_sdio_info("%s ret=%d\n", __func__, ret);
                if (ret == -ETIMEDOUT && !skw_sdio->device_active)
                    continue;
            }
        }
        skw_sdio->tx_req_map &= ~(1 << portno);
        skw_port_log(portno, "%s port%d size=%d 0x%x 0x%x\n",
                     __func__, portno, size, data[0], data[1]);
        return ret;
    }
    return -ENOMEM;
}
static int sdio_read(struct sdio_port *port, char *buffer, int size)
{
    int data_size;
    int ret = 0;
    int buffer_size;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_dbg("%s buffer size = %d , (wp, rp) = (%d, %d), state %d\n",
                 __func__, size, port->rx_wp, port->rx_rp, port->state);
    if (port->state == PORT_STATE_ASST)
    {
        skw_sdio_err("Line:%d The CP RT_ASSERT  portno =%d error code =%d cp_state=%d !!\n", __LINE__,
                     port->channel, ENOTCONN, skw_sdio->cp_state);
        if (skw_sdio->cp_state != 0)
        {
            if (port->channel == skw_log_port())
                port->state = PORT_STATE_OPEN;

            return -ENOTCONN;
        }
    }
try_again0:
    //skw_reinit_completion(port->rx_done);
    init_completion(&port->rx_done);
    if (port->rx_wp == port->rx_rp)
    {

        if ((port->state == PORT_STATE_CLSE) || ((port->channel > 0 && port->channel != skw_log_port())
                && !(skw_sdio->service_state_map & (1 << BT_SERVICE))))
        {
            skw_sdio_err("the log port or at port ---%d --%d\n", port->channel, skw_log_port());
            return -EIO;
        }
        wait_for_completion_interruptible(&port->rx_done);//一直等待
//      if(ret)
//          return ret;
        if (port->state == PORT_STATE_CLSE)
        {
            port->state = PORT_STATE_IDLE;
            return -EAGAIN;
        }
        else if (port->state == PORT_STATE_ASST)
        {
            skw_sdio_err("The CP RT_ASSERT  portno =%d error code =%d!!!!\n", port->channel, ENOTCONN);
            if (skw_sdio->cp_state != 0)
            {
                if (port->channel == skw_log_port())
                    port->state = PORT_STATE_OPEN;

                return -ENOTCONN;
            }
        }
    }
    mutex_lock(&port->rx_mutex);
    data_size = (port->length + port->rx_wp - port->rx_rp) % port->length;
    if (data_size == 0)
    {
        skw_sdio_info("%s buffer size = %d , (wp, rp) = (%d, %d)\n",
                      __func__, size, port->rx_wp, port->rx_rp);
        mutex_unlock(&port->rx_mutex);
        goto try_again0;
    }
    if (size > data_size)
        size = data_size;
    data_size = port->length - port->rx_rp;
    if (size > data_size)
    {
        memcpy(buffer, &port->read_buffer[port->rx_rp], data_size);
        memcpy(buffer + data_size, &port->read_buffer[0], size - data_size);
        port->rx_rp = size - data_size;
    }
    else
    {
        skw_sdio_dbg("size1 = %d , (wp, rp) = (%d, %d) (packet, total)=(%d, %d)\n",
                     size, port->rx_wp, port->rx_rp, port->rx_packet, port->rx_count);
        memcpy(buffer, &port->read_buffer[port->rx_rp], size);
        port->rx_rp += size;
    }

    if (port->rx_rp == port->length)
        port->rx_rp = 0;

    if (port->rx_rp == port->rx_wp)
    {
        port->rx_rp = 0;
        port->rx_wp = 0;
    }
    if (port->rx_flow_ctrl)
    {
        buffer_size = (port->length + port->rx_wp - port->rx_rp) % port->length;
        buffer_size = port->length - 1 - buffer_size;

        if (buffer_size > (port->length * 2 / 3))
        {
            port->rx_flow_ctrl = 0;
            skw_sdio_rx_port_follow_ctl(port->channel, port->rx_flow_ctrl);
        }
    }
    mutex_unlock(&port->rx_mutex);
    return size;
}

int recv_data(int portno, char *buffer, int size)
{
    struct sdio_port *port;
    int ret;
    if (size == 0)
        return 0;
    if (portno >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[portno];
    if (!port->state)
        return -EIO;
    if (port->state == PORT_STATE_CLSE)
    {
        port->state = PORT_STATE_IDLE;
        return -EIO;
    }
    ret = sdio_read(port, buffer, size);
    return ret;
}
#include <drivers/mmcsd_host.h>
void wifi_host_read_en(uint8_t en)
{
    // struct rt_mmcsd_host *sdhci_get_emmchost(void);
    // struct rt_mmcsd_host * host = sdhci_get_emmchost();
    // host->ops->enable_sdio_irq(host, en);
}
int wifi_send_cmd(int portno, scatterlist *sg, int sg_num, int total)
{
    struct sdio_port *port;
    int ret, i;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    u32 *data;

    if (total == 0)
        return 0;
    if (portno >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[portno];
//  if(!port->state)
//      return -EIO;
    data = (u32 *)sg_virt(sg);
    //skw_printf("%s %d stater\n",__func__,__LINE__);
    //mutex_lock(&skw_sdio->rx_tx_mutex);
    for (i = 0; i < 2 ; i++) //只发一次
    {
        //try_to_wakeup_modem(portno);

        //skw_printf("cmd tx: len=%d\n",sg_num);
#if SKW_WIFI_DUMP
        for (int i = 0; i < 100; i++)
        {
            if (!(i % 8)) skw_printf("    ");
            if (!(i % 16)) skw_printf("\n");
            skw_printf("%02x ", sg[i]);
        }
        skw_printf("\n");
#endif
        ret = skw_sdio_adma_write(portno, sg, sg_num, SKW_SDIO_PK_MODE_ADDR);
        if (!ret)
            break;
        skw_sdio_info("timeout gpioin value\n");
    }
    //skw_printf("%s %d end\n",__func__,__LINE__);
    //mutex_unlock(&skw_sdio->rx_tx_mutex);
    skw_sdio->tx_req_map &= ~(1 << portno);

    port->total += total;
    port->sent_packet += sg_num;
    return ret;
}
static int register_rx_callback(int id, void *func, void *para)
{
    struct sdio_port *port;

    if (id >= max_ch_num)
        return -EINVAL;
    port = &sdio_ports[id];
    if (port->state && func)
        return -EBUSY;
    port->rx_submit = func;
    port->rx_data = para;
    if (func)
    {
        port->sg_rx = rt_malloc(MAX_SG_COUNT * sizeof(scatterlist));
        if (port->sg_rx == NULL)
            return -ENOMEM;
        port->state = PORT_STATE_OPEN;
    }
    else
        port->state = PORT_STATE_IDLE;
    return 0;
}
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int bt_service_start(void)
{
    int ret = 0;
    rt_time_t cur, start_poll;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("the ---debug---line:%d \n", __LINE__);

    cur = ktime_get();
    if (skw_sdio->boot_data == NULL || (skw_sdio->service_state_map & (1 << BT_SERVICE)))
        return ret;

    skw_sdio_info("the ---debug---line:%d \n", __LINE__);
    mutex_lock(&skw_sdio->service_mutex);
    if (skw_sdio->boot_data->iram_img_data && skw_sdio->service_index_map)
    {
        skw_sdio_info("just download the BT img!!\n");
        skw_sdio->service_index_map = SKW_BT;
        skw_sdio_poweron_mem(SKW_BT);
        skw_sdio->boot_data->skw_dloader_module(SKW_BT);
    }
    ret = skw_sdio->boot_data->bt_start();
    skw_sdio->service_index_map = SKW_BT;
    start_poll = ktime_get();
    skw_sdio_info("the start service time =%lld", ktime_sub(start_poll, cur));

    mutex_unlock(&skw_sdio->service_mutex);
    return ret;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int bt_service_stop(void)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("the ---debug---line:%d \n", __LINE__);

    if (skw_sdio->boot_data == NULL)
        return ret;
    mutex_lock(&skw_sdio->service_mutex);
    ret = skw_sdio->boot_data->bt_stop();
    mutex_unlock(&skw_sdio->service_mutex);
    return ret;
}


/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int wifi_service_start(void)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("the ---debug---line:%d \n", __LINE__);

    if (skw_sdio->boot_data == NULL || (skw_sdio->service_state_map & (1 << WIFI_SERVICE)))
        return 0;

    skw_sdio_info("the ---debug---line:%d \n", __LINE__);
    mutex_lock(&skw_sdio->service_mutex);
#if SKW_WIFIONLY_DEBUG//for the debug wifi only  setvalue 0
    if (skw_sdio->boot_data->iram_img_data && glb_wifiready_done)
    {
#else
    if (skw_sdio->boot_data->iram_img_data && skw_sdio->service_index_map)
    {
#endif
        //skw_sdio->service_index_map &= SKW_WIFI;
        skw_sdio_info("the ---debug---line:%d \n", __LINE__);

        skw_sdio_info("just download the WIFI img!!\n");
        skw_sdio->service_index_map = SKW_WIFI;
        skw_sdio_poweron_mem(SKW_WIFI);
        skw_sdio->boot_data->skw_dloader_module(SKW_WIFI);
    }
    skw_sdio_info("the ---debug---line:%d \n", __LINE__);
    if (skw_sdio->boot_data->wifi_start)
        ret = skw_sdio->boot_data->wifi_start();
    skw_sdio->service_index_map = SKW_WIFI;
    //skw_sdio->service_index_map = SKW_WIFI;
    glb_wifiready_done = 1;
    mutex_unlock(&skw_sdio->service_mutex);
    return ret;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int wifi_service_stop(void)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("the ---debug---line:%d \n", __LINE__);
    //debug code end
    if (skw_sdio->boot_data == NULL)
    {
        skw_sdio_info("no wifi service start before!!");
        return ret;
    }
    //mutex_lock(&skw_sdio->service_mutex);
    if (skw_sdio->boot_data->wifi_stop)
        ret = skw_sdio->boot_data->wifi_stop();
    //mutex_unlock(&skw_sdio->service_mutex);
    return ret;
}

static int wifi_get_credit(void)
{
    char val;
    int err;

    err = skw_sdio_readb(SDIOHAL_PD_DL_CP2AP_SIG4, &val);
    if (err)
        return err;
    return val;
}
static int wifi_store_credit_to_cp(unsigned char val)
{
    int err;

    err = skw_sdio_writeb(SKW_SDIO_CREDIT_TO_CP, val);

    return err;
}

void kick_rx_thread(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

}

struct sv6160_platform_data wifi_pdata =
{
    .data_port =  WIFI_DATA_PORT,
    .cmd_port =  WIFI_CMD_PORT,
    .bus_type = SDIO_LINK | TX_ADMA | RX_ADMA | CP_DBG,
    .max_buffer_size = 84 * 1536,
    .align_value = 256,
    .hw_adma_tx = wifi_send_cmd,
    .hw_sdma_tx = send_data,
    .callback_register = register_rx_callback,
    .modem_assert = send_modem_assert_command,
    .service_start = wifi_service_start,
    .service_stop = wifi_service_stop,
    .skw_dloader = skw_sdio_dloader,

    .wifi_power_on = skw_sdio_wifi_power_on,

    .wifi_get_credit = wifi_get_credit,
    .wifi_store_credit = wifi_store_credit_to_cp,
    .debug_info = assert_context,
    .rx_thread_wakeup = kick_rx_thread
};
struct sv6160_platform_data ucom_pdata =
{
    .data_port = 2,
    .cmd_port  = 3,
    .audio_port = 4,
    .bus_type = SDIO_LINK,
    .max_buffer_size = 0x1000,
    .align_value = 4,
    .hw_sdma_rx = recv_data,
    .hw_sdma_tx = send_data,
    .open_port = open_sdio_port,
    .close_port = close_sdio_port,
    .modem_assert = send_modem_assert_command,
    .service_start = bt_service_start,
    .service_stop = bt_service_stop,
    .skw_dump_mem = skw_sdio_dt_read,
};
struct sv6160_platform_data *skw_sdio_get_pdata(void)
{
    return &wifi_pdata;
}
struct sdio_port *skw_get_sdio_ports(uint8_t port_num)
{
    return &sdio_ports[port_num];
}

int skw_sdio_bind_platform_driver(struct sdio_func *func)
{
    rt_device_t pdev;
    char    pdev_name[32];
    struct sdio_port *port;
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    rt_memset(sdio_ports, 0, sizeof(struct sdio_port)*MAX_CH_NUM);
    rt_sprintf(pdev_name, "skw_ucom");
    pdev->user_data = (void *)&ucom_pdata;

    return ret;
}

#include "rtdef.h"
struct rt_device sdio_wifi_dev;

int skw_sdio_bind_WIFI_driver(struct sdio_func *func)//未完成读写动作
{
    rt_device_t pdev;
    char    pdev_name[32];
    struct sdio_port *port;
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    if (sdio_ports[WIFI_DATA_PORT].pdev)
        return 0;

    rt_sprintf(pdev_name, "%s%d", SV6160_WIRELESS, func->num);
    pdev = &sdio_wifi_dev;//(rt_device_t)malloc(sizeof(rt_device));//platform_device_alloc(pdev_name, PLATFORM_DEVID_AUTO);
    if (!pdev)
        return -ENOMEM;
    func->priv = &sdio_wifi_dev;
    //pdev->dev.dma_mask = &port_dmamask;
    //pdev->dev.coherent_dma_mask = port_dmamask;
#ifdef CONFIG_SEEKWAVE_PLD_RELEASE
    wifi_pdata.bus_type |= CP_RLS;
#else
    if (!strncmp((char *)skw_sdio->chip_id, "SV6160", 6))
    {
        wifi_pdata.bus_type |= CP_RLS;
    }
#endif
    /*support the sdma type bus*/
    if (!skw_sdio->adma_rx_enable)
    {
        if (skw_cp_ver == SKW_SDIO_V10)
            wifi_pdata.bus_type = SDIO_LINK | TX_ADMA | RX_SDMA;
        else
            wifi_pdata.bus_type = SDIO2_LINK | TX_ADMA | RX_SDMA;
    }
    else
    {
        if (skw_cp_ver == SKW_SDIO_V10)
            wifi_pdata.bus_type = SDIO_LINK | TX_ADMA | RX_ADMA | CP_DBG;
        else
            wifi_pdata.bus_type = SDIO2_LINK | TX_ADMA | RX_ADMA | CP_DBG;
    }
    wifi_pdata.align_value = skw_sdio_blk_size;
    skw_sdio_info(" wifi_pdata bus_type:0x%x \n", wifi_pdata.bus_type);
    if (skw_cp_ver == SKW_SDIO_V20)
    {
        if (!strncmp((char *)skw_sdio->chip_id, "SV6316", 12))
        {
            wifi_pdata.data_port = (SDIO2_WIFI_DATA1_PORT << 4) | SDIO2_WIFI_DATA_PORT;
            wifi_pdata.cmd_port = SDIO2_WIFI_CMD_PORT;
        }
        else if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12))
        {
            wifi_pdata.data_port = SDIO2_WIFI_DATA_PORT;
            wifi_pdata.cmd_port = SDIO2_WIFI_CMD_PORT;
        }
        else
        {
            wifi_pdata.data_port = (SDIO2_WIFI_DATA1_PORT << 4) | SDIO2_WIFI_DATA_PORT;
            wifi_pdata.cmd_port = SDIO2_WIFI_CMD_PORT;
        }
    }
    memcpy(wifi_pdata.chipid, skw_sdio->chip_id, SKW_CHIP_ID_LENGTH);

    pdev->user_data = &wifi_pdata;
    if (skw_cp_ver == SKW_SDIO_V20)
    {
        if (!strncmp((char *)skw_sdio->chip_id, "SV6316", 12))
        {
            port = &sdio_ports[(wifi_pdata.data_port >> 4) & 0x0F];
            port->pdev = pdev;
            port->channel = (wifi_pdata.data_port >> 4) & 0x0F;
            port->rx_wp = 0;
            port->rx_rp = 0;
            port->sg_index = 0;
            port->state = 0;
        }
    }
    uint8_t *cmd_rx_data = rt_malloc(8 * 1024);
    uint8_t *data_rx_data = rt_malloc(8 * 1024);
    skw_sdio_port_init(wifi_pdata.data_port, RT_NULL, data_rx_data);
    skw_sdio_port_init(wifi_pdata.cmd_port, RT_NULL, cmd_rx_data);

    port = &sdio_ports[wifi_pdata.data_port & 0x0F];
    port->pdev = pdev;
    port->channel = wifi_pdata.data_port & 0x0F;
    port->rx_wp = 0;
    port->rx_rp = 0;
    port->sg_index = 0;
    port->state = 0;

    port = &sdio_ports[wifi_pdata.cmd_port];
    port->pdev = pdev;
    port->channel = wifi_pdata.cmd_port;
    port->rx_wp = 0;
    port->rx_rp = 0;
    port->sg_index = 0;
    port->state = 0;

    ret = platform_device_add(pdev);
#if 0
    if (ret)
    {
        dev_err(&func->dev, "failt to register platform device\n");
        platform_device_put(pdev);
    }
#endif
    return ret;
}
int skw_sdio_wifi_status(void)
{
    struct sdio_port *port = &sdio_ports[wifi_pdata.cmd_port];
    if (port->pdev == NULL)
        return 0;
    return 1;
}
int skw_sdio_wifi_power_on(int power_on)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int ret;
    if (power_on)
    {
        if (skw_sdio->power_off)
            skw_recovery_mode();

        ret = skw_sdio_bind_WIFI_driver(skw_sdio->sdio_func[FUNC_1]);
    }
    else
    {
        ret = skw_sdio_unbind_WIFI_driver(skw_sdio->sdio_func[FUNC_1]);
    }
    return ret;
}

static int skw_sdio_unbind_sdio_port_driver(struct sdio_func *func, int portno)
{
    int i;
    struct rt_device *pdev;
    struct sdio_port *port;
    struct sv6160_platform_data *pdata = NULL;

    for (i = portno; i < max_ch_num;)
    {
        port = &sdio_ports[i];
        pdev = port->pdev;
        pdata = (struct sv6160_platform_data *)(pdev->user_data);
        skw_sdio_info("port name %s %d\n", pdata->port_name, portno);
        port->pdev = NULL;
        if (port->read_buffer)
            kfree(port->read_buffer);
        if (port->write_buffer)
            kfree(port->write_buffer);
        if (port->sg_rx)
            kfree(port->sg_rx);
        if (pdev)
            platform_device_unregister(pdev);
        port->sg_rx = NULL;
        port->read_buffer = NULL;
        port->write_buffer = NULL;
        port->pdev = NULL;
        port->rx_wp = 0;
        port->rx_rp = 0;
        port->sg_index = 0;
        port->state = 0;
        break;
    }
    return 0;
}

int skw_sdio_unbind_platform_driver(struct sdio_func *func)
{
    int ret;

    ret = skw_sdio_unbind_sdio_port_driver(func, SDIO2_BSP_ATC_PORT);
    if (skw_cp_ver == SKW_SDIO_V20)
    {
        ret |= skw_sdio_unbind_sdio_port_driver(func, SDIO2_BSP_LOG_PORT);
    }
    else
    {
        ret |= skw_sdio_unbind_sdio_port_driver(func, BSP_LOG_PORT);
    }
    return ret;
}

int skw_sdio_unbind_WIFI_driver(struct sdio_func *func)
{
    int ret;

    ret = skw_sdio_unbind_sdio_port_driver(func, SDIO2_WIFI_CMD_PORT);
    return ret;
}

int skw_sdio_unbind_BT_driver(struct sdio_func *func)
{
    int ret = 0;

    if (skw_cp_ver == SKW_SDIO_V20)
    {
        ret = skw_sdio_unbind_sdio_port_driver(func, SDIO2_BT_LOG_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, SDIO2_BT_DATA_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, SDIO2_BT_CMD_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, SDIO2_BT_AUDIO_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, SDIO2_BT_ISOC_PORT);
    }
    else
    {
        ret = skw_sdio_unbind_sdio_port_driver(func, BT_DATA_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, BT_CMD_PORT);
        ret |= skw_sdio_unbind_sdio_port_driver(func, BT_AUDIO_PORT);
    }
    return ret;
}
