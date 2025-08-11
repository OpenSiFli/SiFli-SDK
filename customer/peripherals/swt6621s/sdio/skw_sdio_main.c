/*
 * Copyright (C) 2021 Seekwave Tech Inc.
 *
 * Filename : skw_sdio.c
 * Abstract : This file is a implementation for Seekwave sdio  function
 *
 * Authors  :
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>
#include <drivers/mmcsd_card.h>
#include <drivers/mmcsd_core.h>
#include <drivers/mmcsd_host.h>
#include <drivers/sdio.h>
#include <sdio_func.h>
#include "skw_sdio_log.h"
#include "skw_sdio.h"
#ifdef RT_USING_DFS_ELMFAT
    #include "dfs_file.h"
#endif
#include "skw_boot.h"
#ifdef BSP_USING_SDHCI
    #include "drv_sdhci.h"
#elif BSP_USING_SD_LINE
    #include "drv_sdio.h"
#endif
#include "skw_type_msg.h"
int bind_device = 0;
struct rt_mmcsd_card *g_wifi_if_sdio = NULL;
static struct rt_sdio_device_id wifi_if_sdio_id = {0, 0x1FFE, 0};

static struct rt_sdio_driver wifi_if_sdio =
{
    "xwwfsdio",
    rt_skw_sdio_probe,
    rt_skw_sdio_remove,
    &wifi_if_sdio_id,
};

#ifndef MMC_CAP2_SDIO_IRQ_NOTHREAD
    #define MMC_CAP2_SDIO_IRQ_NOTHREAD (1 << 17)
#endif

static int check_chipid(void);
static int skw_sdio_cp_reset(void);
static int skw_sdio_cp_service_ops(int service_ops);
static int skw_sdio_cpdebug_boot(void);
struct skw_sdio_data_t g_skw_sdio_data;

static struct rt_mutex dloader_mutex;
static int skw_sdio_set_dma_type(unsigned int address, unsigned int dma_type);
static int skw_sdio_slp_feature_en(unsigned int address, unsigned int slp_en);
static int skw_sdio_host_irq_init(unsigned int irq_gpio_num);
static int skw_WIFI_service_start(void);
static int skw_WIFI_service_stop(void);
static int skw_BT_service_start(void);
static int skw_BT_service_stop(void);
static int skw_sdio_host_check(struct skw_sdio_data_t *skw_sdio);
extern void kernel_restart(char *cmd);
extern void skw_sdio_exception_work(struct rt_work *work, void *work_data);

extern char skw_cp_ver;
extern int max_ch_num;
extern int max_pac_size;
extern int skw_sdio_blk_size;
extern char assert_context[];
extern int  assert_context_size;
extern struct debug_vars debug_infos;

uint8_t wifi_test_t = 0;

struct skw_sdio_data_t *skw_sdio_get_data(void)
{
    return &g_skw_sdio_data;
}

void skw_sdio_unlock_rx_ws(struct skw_sdio_data_t *skw_sdio)
{

}
static void skw_sdio_lock_rx_ws(struct skw_sdio_data_t *skw_sdio)
{

}
static void skw_sdio_wakeup_source_init(struct skw_sdio_data_t *skw_sdio)
{

}
static void skw_sdio_wakeup_source_destroy(struct skw_sdio_data_t *skw_sdio)
{

}

void skw_resume_check(void)
{

}

static void skw_sdio_abort(void)
{
    skw_sdio_info("%s %d\n", __func__, __LINE__);
}

int skw_sdio_sdma_write(unsigned char *src, unsigned int len)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    int blksize = func->cur_blk_size;
    int ret = 0;

    if (!src || len % 4)
    {
        skw_sdio_err("%s invalid para %p, %d\n", __func__, src, len);
        return -1;
    }

    len = (len + blksize - 1) / blksize * blksize;

    skw_resume_check();
    skw_sdio_transfer_enter();
    sdio_claim_host(func);
    ret = sdio_io_writeb(func, SKW_SDIO_PK_MODE_ADDR, *src);
    if (ret < 0)
        skw_sdio_err("%s  ret = %d\n", __func__, ret);
    sdio_release_host(func);
    if (ret)
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    skw_sdio_transfer_exit();

    return ret;
}

int skw_sdio_sdma_read(unsigned char *src, unsigned int len)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    rt_uint32_t ret = 0;
    rt_uint8_t read_ret = 0;

    skw_resume_check();
    skw_sdio_transfer_enter();
    sdio_claim_host(func);
    read_ret = sdio_io_readb(func, SKW_SDIO_PK_MODE_ADDR, &ret);
    if (src) *src = read_ret;
    sdio_release_host(func);
    if (ret != 0)
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    skw_sdio_transfer_exit();
    return ret;
}

void *skw_get_bus_dev(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int time_count = 0;
    if ((!skw_sdio->sdio_dev_host) || (!skw_sdio))
    {
        skw_sdio_err("%d try again get sdio bus dev  \n", __LINE__);
        do
        {
            msleep(10);
            time_count++;
        }
        while (!skw_sdio->sdio_dev_host && time_count < 50);
    }
    if ((!skw_sdio->sdio_dev_host) || (!skw_sdio))
    {
        skw_sdio_err("sdio_dev_host is NULL!\n");
        return NULL;
    }
    return &skw_sdio->sdio_func[FUNC_1]->priv;
}

static int skw_sdio_start_transfer(scatterlist *sgs, int sg_count,
                                   int total, struct sdio_func *sdio_func, int fix_inc, bool dir, int addr)
{
    return 0;
}

int skw_sdio_adma_write(int portno, uint8_t *sgs, int sg_count, int mode_addr)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int ret = 0, op_code = 0;

    skw_resume_check();
    skw_sdio_transfer_enter();
    if (skw_sdio->resume_com == 0)
        skw_sdio->resume_com = 1;

    op_code = mode_addr == SKW_SDIO_DT_MODE_ADDR ? 1 : 0;
    ret = sdio_io_rw_extended_block(skw_sdio->sdio_func[FUNC_1],
                                    SKW_SDIO_WRITE, mode_addr, op_code, sgs, sg_count);

    if (ret)
    {
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }
    skw_sdio_transfer_exit();

    return ret;
}

int skw_sdio_adma_read(struct skw_sdio_data_t *skw_sdio, uint8_t *sgs, int sg_count, int mode_addr)
{
    int ret = 0, op_code = 0;

    skw_resume_check();
    skw_sdio_transfer_enter();
    op_code = mode_addr == SKW_SDIO_DT_MODE_ADDR ? 1 : 0;
    ret = sdio_io_rw_extended_block(skw_sdio->sdio_func[FUNC_1],
                                    SKW_SDIO_READ, mode_addr, op_code, sgs, sg_count);
    if (ret)
    {
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }
    skw_sdio_transfer_exit();
    return ret;
}

static int skw_sdio_dt_set_address(unsigned int address)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_0];
    unsigned int value ;
    rt_int32_t err = 0;
    int i;
    rt_uint16_t data = 0;
    for (i = 0; i < 4; i++)
    {
        value = (address >> (8 * i)) & 0xFF;
        skw_sdio_info("value=%x card=%p,num=%d\n", value, func->card, func->num);
        err = sdio_io_writeb(func, SKW_SDIO_FBR_REG + i, value); //15c
        if (err != 0)
            break;
    }
    skw_sdio_info("err=%d \n", err);
    return err;
}

int skw_sdio_writel(unsigned int address, void *data)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    int ret = 0;

    skw_resume_check();
    skw_sdio_transfer_enter();

    ret = skw_sdio_dt_set_address(address);
    if (ret != 0)
    {
        skw_sdio_transfer_exit();
        return ret;
    }

    sdio_claim_host(func);
    ret = sdio_io_writel(func, *(unsigned int *)data, SKW_SDIO_DT_MODE_ADDR);
    sdio_release_host(func);
    skw_sdio_transfer_exit();

    if (ret)
    {
        skw_sdio_err("%s fail ret:%d, addr=0x%x\n", __func__,
                     ret, address);
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }
    skw_sdio_info("debug----the address=0x%x \n", address);
    return ret;
}

int skw_sdio_readl(unsigned int address, void *data)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    rt_int32_t ret = 0;

    skw_resume_check();
    skw_sdio_transfer_enter();
    ret = skw_sdio_dt_set_address(address);
    if (ret != 0)
    {
        skw_sdio_transfer_exit();
        return ret;
    }

    sdio_claim_host(func);

    data = (void *)sdio_io_readl(func, SKW_SDIO_DT_MODE_ADDR, &ret);

    sdio_release_host(func);
    skw_sdio_transfer_exit();
    if (0 != ret)
    {
        skw_sdio_err("%s fail ret:%d, addr=0x%x\n", __func__, ret, address);
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }

    return ret;
}

int send_modem_service_command(u16 service, u16 command)
{
    u16 cmd;
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    if (command)
        skw_sdio->service_state_map &= ~(1 << service);
    cmd = (service << 1) | command;
    cmd = 1 << cmd;
    if (cmd >> 8)
    {
        skw_sdio_err("service command error 0x%x!", cmd);
        return -EINVAL;
    }
    skw_sdio_info("service = %d cmd %x,skw_sdio->cp_state=%d\n", service, cmd, skw_sdio->cp_state);
    BSP_GPIO_Set(17, 0, 1);
    ret = skw_sdio_writeb(SKW_AP2CP_IRQ_REG, cmd & 0xff);
    BSP_GPIO_Set(17, 1, 1);
    skw_sdio_info("ret = %d command %x\n", ret, command);
    return ret;
}

static unsigned int max_bytes(struct sdio_func *func)
{
    unsigned int mval = func->card->host->max_blk_size;
    return min(mval, 512u);
}

//对wifi模块的boot固件进行下载
int skw_sdio_dt_write(unsigned int address, void *buf, unsigned int len)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    unsigned int remainder = len;
    unsigned int trans_len;
    int ret = 0;
    char *data = skw_sdio->next_size_buf;
    ret = skw_sdio_dt_set_address(address);
    if (ret != 0)
    {
        skw_sdio_err("%s set address error!!!", __func__);
        return ret;
    }

    sdio_claim_host(func);
    char boot_file_path[BOOT_FILE_NAME_LEN] = {0};
    struct stat boot_file_stat;
    struct dfs_fd fd;
    uint32_t read_len = 0, page_num = 0;
    char *read_bin_buff = (char *)rt_malloc(SKW_SDIO_BLK_SIZE);
    if (address == BOOT_DRAM_ADDR)
        rt_sprintf(boot_file_path, "%s/%s\0", BOOT_DRAM_PATH, BOOT_DRAM_FILE_NAME);
    else if (address == BOOT_IRAM_ADDR)
        rt_sprintf(boot_file_path, "%s/%s\0", BOOT_IRAM_PATH, BOOT_IRAM_FILE_NAME);
#ifdef RT_USING_DFS_ELMFAT
    dfs_file_stat(boot_file_path, &boot_file_stat);

    skw_sdio_info("%s %d boot_file_path=%s size=%d,read_bin_buff=%p\n", __func__, __LINE__, boot_file_path, boot_file_stat.st_size, read_bin_buff);
    if (dfs_file_open(&fd, boot_file_path, O_RDWR | O_CREAT) == 0)
    {
        while (1)
        {
            rt_memset(read_bin_buff, 0xf0, SKW_SDIO_BLK_SIZE);
            read_len = dfs_file_read(&fd, read_bin_buff, SKW_SDIO_BLK_SIZE);
            if (!read_len)
            {
                skw_sdio_info("%s %d boot over \n", __func__, __LINE__);
                dfs_file_close(&fd);
                break;
            }
            //skw_sdio_info("read_len=%d read_bin_buff=%p\n", read_len, read_bin_buff);
            skw_sdio_adma_write(0, read_bin_buff, read_len, SKW_SDIO_DT_MODE_ADDR);
            page_num ++;
        }
    }
    else
        skw_sdio_info(" open %s fialed\n", boot_file_path);
#endif
    rt_free(read_bin_buff);

    sdio_release_host(func);

    if (0 != ret)
    {
        skw_sdio_err("dt write fail ret:%d, address=0x%x\n", ret, address);
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }
    skw_sdio_dbg("debug----the address=0x%x \n", address);
    return ret;
}
void BSP_GPIO_Set(int pin, int val, int is_porta);

int skw_sdio_dt_read(unsigned int address, void *buf, unsigned int len)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_1];
    unsigned int remainder = len;
    unsigned int trans_len;
    int ret = 0;
    ret = skw_sdio_dt_set_address(address);
    if (skw_sdio->resume_com == 0)
        skw_sdio->resume_com = 1;
    sdio_claim_host(func);
    while (remainder > 0)
    {
        if (remainder >= func->cur_blk_size)
            trans_len = func->cur_blk_size;
        else
            trans_len = min(remainder, max_bytes(func));
        skw_sdio_info("trans_len=%d buf=%p\n", trans_len, buf);
        ret = skw_sdio_adma_read(skw_sdio, buf, trans_len, SKW_SDIO_DT_MODE_ADDR);
        skw_sdio_info("ret =%d\n", ret);
        if (ret)
        {
            skw_sdio_info("sdio_memcpy_fromio: %p 0x%x ret=%d\n", buf, *(uint32_t *)buf, ret);
            break;
        }
        remainder -= trans_len;
        buf += trans_len;
    }
    sdio_release_host(func);
    if (ret)
    {
        skw_sdio_info("dt read fail ret:%d, address=0x%x\n", ret, address);
        skw_sdio_info("%s %d ret=%d\n", __func__, __LINE__, ret);
    }

    return ret;
}

int skw_sdio_readb(unsigned int address, unsigned char *value)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_0];
    unsigned char reg = 0;
    rt_int32_t err = 0;
    skw_sdio_transfer_enter();
    sdio_claim_host(func);
    reg = sdio_io_readb(func, address, &err);
    if (value)
        *value = reg;
    sdio_release_host(func);
    skw_sdio_transfer_exit();
    return err;
}

int skw_sdio_writeb(unsigned int address, unsigned char value)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func = skw_sdio->sdio_func[FUNC_0];
    int err = 0;
    skw_sdio_transfer_enter();
    wakeup_modem(MAX_CH_NUM);
    sdio_claim_host(func);
    err = sdio_io_writeb(func, address, value);
    sdio_release_host(func);
    skw_sdio_transfer_exit();

    return err;
}

static int skw_sdio_get_dev_func(struct sdio_func *func)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    if (func->num >= MAX_FUNC_NUM)
    {
        skw_sdio_err("func num err!!! func num is %d!!!",
                     func->num);
        return -1;
    }
    skw_sdio_dbg("func num is %d.", func->num);

    skw_sdio->sdio_func[FUNC_1] = func;

    return 0;
}

void skw_sdio_inband_irq_handler(struct sdio_func *func)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct sdio_func *func0 = skw_sdio->sdio_func[FUNC_0];
    int ret;
    if (!debug_infos.cp_assert_time)
    {
        debug_infos.last_irq_time = skw_local_clock();
        debug_infos.last_irq_times[debug_infos.rx_inband_irq_cnt % CHN_IRQ_RECORD_NUM] = debug_infos.last_irq_time;
    }
    skw_sdio_lock_rx_ws(skw_sdio);
    skw_sdio_rx_up(skw_sdio);
}

static int skw_check_cp_ready(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int ret = rt_completion_wait(&skw_sdio->download_done, 3000);
    if (ret != RT_EOK)
    {
        skw_sdio_err("check CP-ready time out ret=%d\n", ret);
        RT_ASSERT(0);
        return -ETIME;
    }

    return 0;
}

static int skw_sdio_probe(struct sdio_func *func, const struct rt_sdio_device_id *id)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct rt_mmcsd_host *host = func->card->host;
    int ret;
    skw_sdio_info("%s %d \n", __func__, __LINE__);

    skw_sdio_log(SKW_SDIO_INFO, "%s: device=0x%p, "
                 "func_num=0x%04x, clock=%d blksize=0x%x max_blkcnt %d\n", __func__,
                 func->priv, func->num, host->io_cfg.clock,
                 func->cur_blk_size, func->card->host->max_blk_count);

    ret = skw_sdio_get_dev_func(func);
    if (ret < 0)
    {
        skw_sdio_err("get func err\n");
        return ret;
    }

    skw_sdio->sdio_dev_host = skw_sdio->sdio_func[FUNC_1]->card->host;
    if (skw_sdio->sdio_dev_host == NULL)
    {
        skw_sdio_err("get host failed!!!");
        return -1;
    }

    if (!skw_sdio->pwrseq)
    {
        struct sdio_func *func1 = skw_sdio->sdio_func[FUNC_1];
        sdio_claim_host(func1);
        ret = sdio_enable_func(func1);

        skw_sdio_info("sdio_enable_func ret=%d type %d\n", ret, skw_sdio->irq_type);
        if (!ret)
        {
            sdio_set_block_size(func1, SKW_SDIO_BLK_SIZE);
            func1->max_blk_size = SKW_SDIO_BLK_SIZE;
            if (skw_sdio->irq_type == SKW_SDIO_INBAND_IRQ)
            {
                if (sdio_claim_irq(func1, skw_sdio_inband_irq_handler))
                {
                    skw_sdio_err("sdio_claim_irq failed\n");
                }
                else
                {
                    skw_sdio_info("\n");
                }
            }
            sdio_release_host(func1);
        }
        else
        {
            sdio_release_host(func1);
            skw_sdio_err("enable func1 err!!! ret is %d\n", ret);
            return ret;
        }
        skw_sdio->resume_com = 1;
        skw_sdio_info("enable func1 done\n");
    }

    rt_completion_done(&skw_sdio->scan_done);
    skw_sdio_info("\n");

    check_chipid();
    if (strncmp((char *)skw_sdio->chip_id, "SV6160", 12))
    {
        struct sdio_func *func1 = skw_sdio->sdio_func[FUNC_1];
        sdio_claim_host(func1);
        skw_sdio->sdio_func[FUNC_0]->max_blk_size = SKW_SDIO_BLK_SIZE;
        skw_sdio_info("\n");
        sdio_set_block_size(func1, SKW_SDIO_BLK_SIZE);
        skw_sdio_info("\n");
        func1->max_blk_size = SKW_SDIO_BLK_SIZE;
        func1->cur_blk_size = SKW_SDIO_BLK_SIZE;
        sdio_release_host(func1);
    }

    if (bind_device == 1)
    {
        skw_sdio->adma_rx_enable = 1;
        if (ret != 0)
        {
            skw_sdio_err("the dma type write fail ret:%d\n", ret);
            return -1;
        }
        skw_sdio_info("line%d,adma type \n",  __LINE__);
    }
    else if (bind_device == 2)
    {
        ret = skw_sdio_writeb(SKW_SDIO_PLD_DMA_TYPE, SDMA);
        skw_sdio->adma_rx_enable = 0;
        if (ret != 0)
        {
            skw_sdio_err("the dma type write fail: %d\n", ret);
            return -1;
        }
        send_modem_service_command(WIFI_SERVICE, SERVICE_START);
        skw_sdio_info("the skw_sdio sdma write the pass\n");
    }

    skw_sdio->service_state_map = 0;
    skw_sdio->service_index_map = 0;
    skw_sdio->host_active = 1;
    skw_sdio->power_off = 0;
    return 0;
}

static void skw_sdio_remove(struct sdio_func *func)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    skw_sdio_info("Enter\n");

    rt_completion_done(&skw_sdio->remove_done);

    if (skw_sdio->irq_type == SKW_SDIO_INBAND_IRQ)
    {
        sdio_claim_host(skw_sdio->sdio_func[FUNC_1]);
        xhci_sdio_release_irq(skw_sdio->sdio_func[FUNC_1]);
        sdio_release_host(skw_sdio->sdio_func[FUNC_1]);
    }

    skw_sdio->host_active = 0;
    skw_sdio_unbind_WIFI_driver(skw_sdio->sdio_func[FUNC_1]);
    rt_free(skw_sdio->sdio_func[FUNC_0]);
}

void skw_sdio_launch_thread(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    rt_sem_init(&skw_sdio->rx_completed, "rx_completed_sem", 0, RT_IPC_FLAG_FIFO);
    skw_sdio_info("%s %d\n", __func__, __LINE__);

    skw_sdio->rx_thread =
        rt_thread_create("sdio_wifi_thread", skw_sdio_rx_thread, NULL, 2048, 20, RT_THREAD_TICK_DEFAULT * 2);
    if (skw_sdio->rx_thread)
    {
        skw_sdio_info("%s %d\n", __func__, __LINE__);
        rt_thread_startup(skw_sdio->rx_thread);
    }
}

void skw_sdio_stop_thread(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    if (skw_sdio->rx_thread)
    {
        skw_sdio->threads_exit = 1;
        skw_sdio_rx_up(skw_sdio);
        rt_thread_delete(skw_sdio->rx_thread);
        skw_sdio->rx_thread = NULL;
        skw_sdio_wakeup_source_destroy(skw_sdio);
    }
    skw_sdio_info("done\n");
}

void skw_sdio_remove_card(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    rt_completion_init(&skw_sdio->remove_done);
    sdio_unregister_driver(&wifi_if_sdio);
    skw_sdio_info(" sdio_unregister_driver\n");
    if (rt_completion_wait(&skw_sdio->remove_done,
                           5000) != 0)
        skw_sdio_err("remove card time out\n");
    else
        skw_sdio_info("remove card end\n");

}

int skw_sdio_scan_card(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int ret = 0;

    skw_sdio_info("sdio_scan_card\n");
    rt_completion_init(&skw_sdio->scan_done);
    rt_completion_init(&skw_sdio->download_done);
    rt_completion_init(&skw_sdio->device_wakeup);
    skw_sdio->irq_type = SKW_SDIO_INBAND_IRQ;
    ret = sdio_register_driver(&wifi_if_sdio);
    if (ret != 0)
    {
        skw_sdio_err("sdio_register_driver error :%d\n", ret);
        return ret;
    }
    if (rt_completion_wait(&skw_sdio->scan_done, 2000) != RT_EOK)
    {
        skw_sdio_err("wait scan card time out\n");
        return -ENODEV;
    }
    if (!skw_sdio->sdio_dev_host)
    {
        skw_sdio_err("sdio_dev_host is NULL!\n");
        return -ENODEV;
    }
    skw_sdio_info("scan end!\n");

    return ret;
}

static int skw_sdio_slp_feature_en(unsigned int address, unsigned int slp_en)
{
    int ret = 0;
    ret = skw_sdio_writeb(address, slp_en);
    if (ret != 0)
    {
        skw_sdio_err("no-sleep support en write fail, ret=%d\n", ret);
        return -1;
    }
    skw_sdio_info("no-sleep_support_enable:%d\n ", slp_en);
    return 0;
}

static int skw_sdio_set_dma_type(unsigned int address, unsigned int dma_type)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    if (dma_type == SDMA)
    {
        skw_sdio->adma_rx_enable = 0;
    }
    if (!bind_device)
    {
        ret = skw_sdio_writeb(address, dma_type);
        if (ret != 0)
        {
            skw_sdio_err("dma type write fail, ret=%d\n", ret);
            return -1;
        }
    }
    skw_sdio_info("dma_type=%d,adma_rx_enable:%d\n ", dma_type, skw_sdio->adma_rx_enable);
    return 0;
}
static int skw_sdio_set_slp_mode(unsigned int address, unsigned int slp_en)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    ret = skw_sdio_writeb(address, slp_en);
    if (ret != 0)
    {
        skw_sdio_err("slp_en type write fail, ret=%d\n", ret);
        return -1;
    }

    return 0;
}

static int skw_sdio_boot_cp(int boot_mode)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    skw_sdio_info("DOWNLOAD BIN TO CP\n");
    skw_sdio_set_dma_type(SKW_SDIO_PLD_DMA_TYPE, ADMA);
    skw_sdio_set_slp_mode(SKW_SDIO_CP_SLP_SWITCH, 1);

    ret = skw_sdio_dt_write(BOOT_DRAM_ADDR, NULL, 0);
    ret = skw_sdio_dt_write(BOOT_IRAM_ADDR, NULL, 0);
    if (ret != 0)
        goto FAIL;

    ret = skw_sdio_writeb(SKW_SDIO_PD_DL_AP2CP_BSP, BIT(0));
    ret |= skw_check_cp_ready();
    if (ret != 0)
        goto FAIL;
    skw_sdio->cp_state = 1;

    skw_sdio_info("line:%d boot cp pass!!\n", __LINE__);
    check_chipid();
    return ret;
FAIL:
    skw_sdio_info("line:%d  fail ret=%d\n", __LINE__, ret);
    return ret;
}

int skw_sdio_mem_poweron_cp(void)
{
    int ret = 0;
    unsigned char value = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("Enter\n");
    if (!strncmp((char *)skw_sdio->chip_id, "SV6160", 10))
    {
        skw_sdio->smem_poweron = SKW_SMEM_POWERON1;
    }
    else if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12))
    {
        skw_sdio->smem_poweron = SKW_SMEM_POWERON2;
    }
    else if (!strncmp((char *)skw_sdio->chip_id, "SV6316", 10))
    {
        skw_sdio->smem_poweron = SKW_SMEM_POWERON2;
    }
    else
    {
        skw_sdio_err("not support memdump\n");
        return -1;
    }
    skw_sdio_info(" the chip id:%s\n", (char *)skw_sdio->chip_id);
    ret = skw_sdio_readb(skw_sdio->smem_poweron, &value);
    if (ret < 0)
    {
        skw_sdio_err("read cp reg fail ret=%d\n", ret);
        return ret;
    }
    skw_sdio_info("poweron cp reg  addr:%x value:%x\n",
                  skw_sdio->smem_poweron, value);
    value &= ~(0x3 << 2);
    skw_sdio_info("poweron cp reg  addr:%x value:%x\n",
                  skw_sdio->smem_poweron, value);
    ret = skw_sdio_writeb(skw_sdio->smem_poweron, value);
    if (ret < 0)
    {
        skw_sdio_err("write	poweron cp reg  fail ret=%d\n", ret);
        return ret;
    }
    return 0;
}

int skw_sdio_cp_log_disable(int disable)
{
    int ret = 0;
    skw_sdio_info("Enter\n");
    ret = skw_sdio_writeb(SDIOHAL_CPLOG_TO_AP_SWITCH, disable);
    if (ret < 0)
    {
        skw_sdio_err("close the log signal send fail ret=%d\n", ret);
        return ret;
    }
    skw_sdio_writeb(SKW_AP2CP_IRQ_REG, BIT(5));
    if (!disable)
        skw_sdio_info(" enable the CP log \n");
    else
        skw_sdio_info(" disable the CP log !!\n");

    return 0;
}

static int skw_WIFI_service_start(void)
{
    int ret;

    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("Enter STARTWIFI cp_state:%d\n", skw_sdio->cp_state);
    if (skw_sdio->service_state_map & (1 << WIFI_SERVICE))
        return 0;

    mutex_lock(&skw_sdio->except_mutex);
    rt_completion_init(&skw_sdio->download_done);
    ret = send_modem_service_command(WIFI_SERVICE, SERVICE_START);
    if (ret == 0)
        ret = skw_check_cp_ready();
    mutex_unlock(&skw_sdio->except_mutex);
    return ret;
}

static int skw_WIFI_service_stop(void)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("Enter,STOPWIFI  cp_state:%d", skw_sdio->cp_state);
    mutex_lock(&skw_sdio->except_mutex);
    if (skw_sdio->service_state_map & (1 << WIFI_SERVICE))
        ret = send_modem_service_command(WIFI_SERVICE, SERVICE_STOP);
    mutex_unlock(&skw_sdio->except_mutex);
    return ret;
}

static int skw_BT_service_start(void)
{
    int ret;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("Enter cpstate=%d\n", skw_sdio->cp_state);
    if (assert_context_size)
        skw_sdio_info("%s\n", assert_context);
    if (skw_sdio->service_state_map & (1 << BT_SERVICE))
        return 0;

    mutex_lock(&skw_sdio->except_mutex);
    if (skw_sdio->service_state_map == 0 && skw_sdio->power_off)
    {
        rt_completion_init(&skw_sdio->download_done);
        skw_recovery_mode();
    }
    rt_completion_init(&skw_sdio->download_done);
    ret = send_modem_service_command(BT_SERVICE, SERVICE_START);
    if (!ret)
        ret = skw_check_cp_ready();
    mutex_unlock(&skw_sdio->except_mutex);
    return ret;
}

static int skw_BT_service_stop(void)
{
    int ret = 0;

    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("Enter cpstate=%d\n", skw_sdio->cp_state);

    mutex_lock(&skw_sdio->except_mutex);
    if (skw_sdio->service_state_map & (1 << BT_SERVICE) && !skw_sdio->cp_state)
    {
        rt_completion_init(&skw_sdio->download_done);
        ret = send_modem_service_command(BT_SERVICE, SERVICE_STOP);
    }
    mutex_unlock(&skw_sdio->except_mutex);
    return ret;
}

static int skw_sdio_cp_service_ops(int service_ops)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    switch (service_ops)
    {
    case SKW_WIFI_START:
        BSP_GPIO_Set(17, 0, 1);
        ret = skw_WIFI_service_start();
        BSP_GPIO_Set(17, 1, 1);
        skw_sdio_dbg("-----WIFI SERIVCE START\n");
        break;
    case SKW_WIFI_STOP:
        ret = skw_WIFI_service_stop();
        skw_sdio_dbg("----WIFI SERVICE---STOP\n");
        break;
    default:
        skw_sdio_err("service not support!\n");
        break;
    }
    return ret;
}

int skw_boot_loader(struct seekwave_device *boot_data)
{
    int ret = 0;
    struct sdio_func *func;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    skw_sdio_boot_cp(RECOVERY_BOOT);
    skw_sdio_bind_WIFI_driver(skw_sdio->sdio_func[FUNC_1]);
    ret = skw_sdio_cp_service_ops(skw_sdio->boot_data->service_ops);
    return ret;
}

void get_bt_antenna_mode(char *mode)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct seekwave_device *boot_data = skw_sdio->boot_data;
    u32 bt_antenna = boot_data->bt_antenna;

    if (bt_antenna == 0)
        return;
    bt_antenna--;
    if (!mode)
        return;
    if (bt_antenna)
        sprintf(mode, "bt_antenna : alone\n");
    else
        sprintf(mode, "bt_antenna : share\n");
}

void reboot_to_change_bt_antenna_mode(char *mode)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct seekwave_device *boot_data = skw_sdio->boot_data;
    u32 *data = (u32 *) &boot_data->iram_img_data[boot_data->head_addr - 4];
    u32 bt_antenna;

    if (boot_data->bt_antenna == 0)
        return;

    bt_antenna = boot_data->bt_antenna - 1;
    bt_antenna = 1 - bt_antenna;
    data[0] = (bt_antenna) | 0x80000000;
    if (!mode)
        return;
    if (bt_antenna == 1)
    {
        boot_data->bt_antenna = 2;
        sprintf(mode, "bt_antenna : alone\n");
    }
    else
    {
        boot_data->bt_antenna = 1;
        sprintf(mode, "bt_antenna : share\n");
    }
    send_modem_assert_command();
}

void reboot_to_change_bt_uart1(char *mode)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    struct seekwave_device *boot_data = skw_sdio->boot_data;
    u32 *data = (u32 *) &boot_data->iram_img_data[boot_data->head_addr - 4];

    if (data[0] & 0x80000000)
        data[0] |=  0x0000008;
    else
        data[0] = 0x80000008;
    send_modem_assert_command();
}

int skw_reset_bus_dev(void)
{
    return 0;
}

static void skw_sdio_reset_card(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    if (skw_sdio->sdio_dev_host && skw_sdio->sdio_dev_host->card)
    {
        sdio_reset_comm((skw_sdio->sdio_dev_host->card));
    }
    else
    {
        return;
    }
    skw_sdio_info("the reset sdio host pass \n");
}

static int skw_sdio_cp_reset(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    int ret;

    if (skw_sdio->irq_type == SKW_SDIO_INBAND_IRQ)
    {
        sdio_claim_host(skw_sdio->sdio_func[FUNC_1]);
        ret = sdio_release_irq(skw_sdio->sdio_func[FUNC_1]);
        sdio_release_host(skw_sdio->sdio_func[FUNC_1]);
        if (ret < 0)
            skw_sdio_err("%s sdio_release_irq ret = %d\n", __func__, ret);
    }

    skw_sdio_reset_card();
    msleep(5);
    sdio_claim_host(skw_sdio->sdio_func[FUNC_1]);
    ret = sdio_enable_func(skw_sdio->sdio_func[FUNC_1]);
    sdio_set_block_size(skw_sdio->sdio_func[FUNC_1],
                        SKW_SDIO_BLK_SIZE);
    skw_sdio->sdio_func[FUNC_1]->max_blk_size = SKW_SDIO_BLK_SIZE;
    if (skw_sdio->irq_type == SKW_SDIO_INBAND_IRQ)
    {
        ret = sdio_claim_irq(skw_sdio->sdio_func[FUNC_1], skw_sdio_inband_irq_handler);
        if (ret < 0)
        {
            skw_sdio_err("%s sdio_claim_irq ret = %d\n", __func__, ret);
        }
        else
        {
            if (ret < 0)
                skw_sdio_err("enable sdio async irq fail ret = %d\n", ret);
        }
    }
    sdio_release_host(skw_sdio->sdio_func[FUNC_1]);
    if (ret < 0)
    {
        skw_sdio_err("enable func1 err!!! ret is %d\n", ret);
        return -1;
    }
    skw_sdio_info("CP RESET OK!\n");
    return 0;
}

static int skw_sdio_cpdebug_boot(void)
{
    int ret = 0;
    skw_sdio_info(" CP DUEBGBOOT Done!!!\n");
    return 0;
}

int skw_recovery_mode(void)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("the CHIPID:%s \n", (char *)&skw_sdio->chip_id);
    if (!skw_sdio->boot_data->dram_dl_size || !skw_sdio->boot_data->iram_dl_size
            || (skw_sdio_recovery_debug_status() && skw_sdio->cp_state))
    {
        skw_sdio_err("CP DEBUG BOOT,AND NO NEED RECOVERY!!! \n");
        return -1;
    }

    ret = skw_sdio_cp_reset();
    if (ret != 0)
    {
        skw_sdio_err("CP RESET fail \n");
        return -1;
    }
    skw_sdio->power_off = 0;
    skw_sdio->service_index_map = 0;
    ret = skw_sdio_boot_cp(RECOVERY_BOOT);
    if (ret != 0)
    {
        skw_sdio_err("CP RESET fail \n");
        return -1;
    }
    skw_sdio_info("Recovery ok\n");
    return ret;
}

static int poweron_wifi_mem(struct seekwave_device *boot_data)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    rt_time_t cur, stop;

    unsigned char wifi_poweron_reg = 0;
    unsigned char tmp_val = 0;

    if (skw_sdio->service_state_map & (1 << WIFI_SERVICE))
    {
        skw_sdio_info(" No need power on WIFI mem!!! \n");
        return ret;
    }

    skw_sdio_info("---Enter---\n");
    msleep(5);
    skw_sdio_readb(SKW_SDIO_DL_CP2AP_BSP, &tmp_val);
    ret = skw_sdio_writeb(SKW_SDIO_DL_POWERON_MODULE, SKW_WIFI);
    ret = skw_sdio_writeb(SKW_AP2CP_IRQ_REG, BIT(6));
    if (ret != 0)
    {
        skw_sdio_err("%s poweron fail \n", __func__);
        return -1;
    }
    do
    {
        ret = skw_sdio_readb(SKW_SDIO_DL_CP2AP_BSP, &wifi_poweron_reg);
        cur = ktime_get();
        rt_thread_mdelay(10);
    }
    while (tmp_val == wifi_poweron_reg);

    skw_sdio->boot_data->dl_module = SKW_WIFI;
    skw_sdio_info("%s power on wifi pass \n", __func__);
    return 0;
}

int skw_sdio_poweron_mem(int index)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info(" Enter\n");
    switch (index)
    {
    case SKW_WIFI:
        poweron_wifi_mem(skw_sdio->boot_data);
        break;
    case SKW_BT:
        break;
    default:
        skw_sdio_info("no need poweron service mem\n");
        break;
    }
    return ret;

}

int skw_sdio_dloader(int service_index)
{
    int ret = 0;
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info(" Enter\n");
    mutex_lock(&dloader_mutex);
    ret = skw_sdio_poweron_mem(service_index);
    if (ret)
    {
        skw_sdio_err("power the module=%d fail\n", service_index);
        mutex_unlock(&dloader_mutex);
        return ret;
    }
    switch (service_index)
    {
    case SKW_BOOT:
        skw_recovery_mode();
        break;
    case SKW_WIFI:
        skw_sdio->boot_data->skw_dloader_module(SKW_WIFI);
        break;
    case SKW_BT:
        skw_sdio->boot_data->skw_dloader_module(SKW_BT);
        break;
    case SKW_ALL:
        skw_sdio->boot_data->skw_dloader_module(SKW_ALL);
        break;
    default:
        skw_sdio_info("no need the dloader servce mem\n");
        break;
    }
    mutex_unlock(&dloader_mutex);
    return ret;

}

static int check_chipid(void)
{
    int ret;
    skw_sdio_info("\n");
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    skw_sdio_info("skw_sdio->chip_id=%p\n", skw_sdio->chip_id);
    rt_memset(skw_sdio->chip_id, 0, SKW_CHIP_ID_LENGTH);
    ret = skw_sdio_dt_read(SKW_CHIP_ID0, skw_sdio->chip_id, SKW_CHIP_ID_LENGTH);
    skw_sdio_info("skw_sdio->chip_id= %s \n", skw_sdio->chip_id);
    if (!strncmp((char *)skw_sdio->chip_id, "SV6160", 12))
    {
        skw_cp_ver = SKW_SDIO_V10;
        max_ch_num = MAX_CH_NUM;
        max_pac_size = MAX_PAC_SIZE;
        skw_sdio_blk_size = 256;
        for (int i = 0; i < 16; i++) skw_sdio_info("%02x ", skw_sdio->chip_id[i]);
        skw_sdio_info("\n");
        skw_sdio_info("Chip id:%s used SDIO10", (char *)skw_sdio->chip_id);
    }
    else if (!strncmp((char *)skw_sdio->chip_id, "SV6160LITE", 12))
    {
        skw_cp_ver = SKW_SDIO_V20;
        max_ch_num = SDIO2_MAX_CH_NUM;
        max_pac_size = MAX2_PAC_SIZE;
        skw_sdio_blk_size = 512;
        skw_sdio_info("Chip id:%s used SDIO20 ", (char *)skw_sdio->chip_id);
    }
    else
    {
        skw_cp_ver = SKW_SDIO_V20;
        max_ch_num = SDIO2_MAX_CH_NUM;
        max_pac_size = MAX2_PAC_SIZE;
        skw_sdio_blk_size = 512;
        skw_sdio_info("Chip id:%s used SDIO20 ", (char *)skw_sdio->chip_id);
    }
    if (ret < 0)
    {
        skw_sdio_err("Get the chip id fail!!\n");
        RT_ASSERT(0);
        return ret;
    }
    return 0;
}

static int skw_sdio_host_check(struct skw_sdio_data_t *skw_sdio)
{
    int ret = 0;
    return ret;
}
int skw_add_vif(void);
int skw_sdio_io_init(void)
{
    struct skw_sdio_data_t *skw_sdio;
    int ret = 0;
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    memset(&debug_infos, 0, sizeof(struct debug_vars));
    skw_sdio = &g_skw_sdio_data;
    rt_mutex_init(&skw_sdio->transfer_mutex, "transfer_mutex", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&skw_sdio->except_mutex, "except_mutex", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&dloader_mutex, "dloader_mutex", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&skw_sdio->service_mutex, "service_mutex", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&skw_sdio->rx_tx_mutex, "rx_tx_mutex", RT_IPC_FLAG_FIFO);
    skw_sdio->resume_flag = 1;
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    skw_sdio->next_size_buf = rt_malloc(SKW_BUF_SIZE);
    if (skw_sdio->next_size_buf == NULL)
    {
        rt_free(skw_sdio);
        return -ENOMEM;
    }
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    skw_sdio->eof_buf = rt_malloc(SKW_BUF_SIZE);
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    skw_sdio->online = SKW_SDIO_CARD_OFFLINE;
    skw_sdio_info("%s %d,skw_sdio->online=%d\n", __func__, __LINE__, skw_sdio->online);
    if (!bind_device)
    {
        skw_sdio->adma_rx_enable = 1;
    }
    rt_delayed_work_init(&skw_sdio->skw_except_work, skw_sdio_exception_work, RT_NULL);
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    skw_sdio_launch_thread();
#ifdef BSP_USING_SDHCI
    rt_hw_sdmmc_init();
#elif BSP_USING_SD_LINE

    //rt_hw_sdio_init();
#endif
    skw_sdio_info("%s %d\n", __func__, __LINE__);
    rt_thread_mdelay(2000);
    skw_sdio_scan_card();

    skw_sdio_info("%s %d\n", __func__, __LINE__);

    skw_sdio_info(" OK\n");
    ret = skw_sdio_boot_cp(1);
    if (RT_EOK == ret)
    {
        ret = skw_sdio_cp_service_ops(SKW_WIFI_START);
        skw_sdio_bind_WIFI_driver(skw_sdio->sdio_func[FUNC_1]);
    }
    skw_add_vif();

    skw_sdio_writeb(SDIOHAL_CPLOG_TO_AP_SWITCH, 1);
    skw_sdio_writeb(SKW_AP2CP_IRQ_REG, BIT(5));
    wifi_info_init();
    return ret;
}

static void  skw_sdio_io_exit(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();

    skw_sdio_stop_thread();
    if (SKW_CARD_ONLINE(skw_sdio))
    {
        skw_sdio_remove_card();
    }
    skw_sdio_reset_card();
    cancel_delayed_work_sync(&skw_sdio->skw_except_work.work);
    mutex_destroy(&skw_sdio->transfer_mutex);
    mutex_destroy(&skw_sdio->except_mutex);
    mutex_destroy(&dloader_mutex);
    mutex_destroy(&skw_sdio->service_mutex);
    if (skw_sdio)
    {
        rt_free(skw_sdio->next_size_buf);
        rt_free(skw_sdio->eof_buf);
        skw_sdio->boot_data = NULL;
        skw_sdio->sdio_dev_host = NULL;
        rt_free(skw_sdio);
        skw_sdio = NULL;
    }
    skw_sdio_info(" OK\n");
}

struct sdio_func g_wifi_if_sdio_func1;

void cancel_delayed_work_sync(struct rt_work *work)
{
    rt_work_cancel(work);
}
void schedule_delayed_work(struct rt_work *work, rt_tick_t time)
{
    rt_work_submit(work, time);
}

void sdio_claim_host(struct sdio_func *func)
{
    if (g_wifi_if_sdio->host)
        mmcsd_host_lock(g_wifi_if_sdio->host);
}

void sdio_release_host(struct sdio_func *func)
{
    if (g_wifi_if_sdio->host)
        mmcsd_host_unlock(g_wifi_if_sdio->host);
}
void mutex_lock(rt_mutex_t mutex)
{
    rt_mutex_take(mutex, RT_WAITING_FOREVER);
}
void mutex_unlock(rt_mutex_t mutex)
{
    rt_mutex_release(mutex);
}
void mutex_init(rt_mutex_t mutex)
{
    rt_mutex_init(mutex, "awifi", RT_IPC_FLAG_FIFO);
}
void mutex_destroy(rt_mutex_t mutex)
{
    rt_mutex_detach(mutex);
}
void skw_sdio_transfer_enter(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    rt_mutex_take(&skw_sdio->transfer_mutex, RT_WAITING_FOREVER);
}
void skw_sdio_transfer_exit(void)
{
    struct skw_sdio_data_t *skw_sdio = skw_sdio_get_data();
    rt_mutex_release(&skw_sdio->transfer_mutex);
}
rt_err_t  wait_for_completion_timeout(struct rt_completion *completion,
                                      rt_int32_t            timeout)
{
    return rt_completion_wait(completion, timeout);
}
rt_err_t wait_for_completion_interruptible_timeout(struct rt_completion *completion, rt_int32_t timeout)
{
    return rt_completion_wait(completion, timeout);
}
rt_err_t wait_for_completion_interruptible(struct rt_completion *completion)
{
    return rt_completion_wait(completion, RT_WAITING_FOREVER);
}
rt_err_t init_completion(struct rt_completion *complet)
{
    rt_completion_init(complet);
    return RT_EOK;
}
rt_err_t complete(struct rt_completion *complet)
{
    rt_completion_done(complet);
    return RT_EOK;
}
uint32_t msecs_to_jiffies(uint32_t time)
{
    return time;
}
rt_time_t ktime_get(void)
{
    return rt_tick_get();
}
void *sg_virt(void *buff)
{
    return buff;
}
rt_time_t ktime_sub(rt_time_t lhs, rt_time_t rhs)
{
    rt_time_t result;
    result = lhs - rhs;
    return result;
}

void atomic_set(int var, int i)
{
    var = i;
}
void WARN_ON(int i)
{
    return ;
}
rt_err_t platform_device_add(rt_device_t dev)
{
    return rt_device_register(dev, dev->parent.name,
                              RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);

}
rt_err_t platform_device_unregister(rt_device_t dev)
{
    return rt_device_unregister(dev);
}
rt_int32_t sdio_claim_irq(struct sdio_func *func, rt_sdio_irq_handler_t   *handler)
{
    int ret = 0;
    int func_num = func->num;
    if (func->irq_handler == NULL)
    {
        func->irq_handler = handler;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = (int)sdio_attach_irq(g_wifi_if_sdio->sdio_function[func_num], handler);
    }
    return ret;
}

int xhci_sdio_release_irq(struct sdio_func *func)
{
    int ret = 0;
    int func_num = func->num;
    if (func->irq_handler)
    {
        func->irq_handler = NULL;
    }
    if ((func_num <= SDIO_MAX_FUNCTIONS) && g_wifi_if_sdio)
    {
        ret = (int)sdio_detach_irq(g_wifi_if_sdio->sdio_function[func_num]);
    }
    return ret;
}
int sdio_release_irq(struct sdio_func *func)
{
    return xhci_sdio_release_irq(func);
}

u64 skw_local_clock(void)
{
    return local_clock();
}
uint32_t local_clock(void)
{
    return rt_tick_get();
}

rt_int32_t rt_skw_sdio_probe(struct rt_mmcsd_card *card)
{
    skw_sdio_info("%s %d sdio_function_num=%d\n", __func__, __LINE__, card->sdio_function_num);
    g_wifi_if_sdio = card;
    g_wifi_if_sdio_func1.num = g_wifi_if_sdio->sdio_function[1]->num;
    g_wifi_if_sdio_func1 = *card->sdio_function[1];
    g_skw_sdio_data.sdio_func[FUNC_0] = g_wifi_if_sdio->sdio_function[0];
    g_skw_sdio_data.sdio_func[FUNC_1] = g_wifi_if_sdio->sdio_function[1];
    skw_sdio_probe(&g_wifi_if_sdio_func1, &wifi_if_sdio_id);
    return 0;
}
rt_int32_t rt_skw_sdio_remove(struct rt_mmcsd_card *card)
{
    skw_sdio_remove(&g_wifi_if_sdio_func1);
    g_wifi_if_sdio = NULL;
    return 0;
}

// void wifi_open(void)
// {
//     skw_sdio_info("%s %d\n", __func__, __LINE__);
//     skw_sdio_io_init();
//     skw_sdio_info("%s %d\n", __func__, __LINE__);
// }
// MSH_CMD_EXPORT(wifi_open, wifi_open);