/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_log_process.c
 *Authors:seekwave platform
 *
 * This software is licensed under the terms of the the GNU
 * General Public License version 2, as published by the Free
 * Software Foundation, and may be copied, distributed, and
 * modified under those terms.
 *
 * This program is distributed in the hope that it will be usefull,
 * but without any warranty;without even the implied warranty of
 * merchantability or fitness for a partcular purpose. See the
 * GUN General Public License for more details.
 * **************************************************************/

/* #define DEBUG */
/* #define VERBOSE_DEBUG */
#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>
#if 0
#include "skw_boot.h"

#include "skw_log_to_file.h"
#include "skw_mem_map.h"
extern int cp_exception_sts;
static char *log_path = "/data";
module_param(log_path, charp, 0644);

int skw_log_num = 2;
#ifdef MIN_LOG_SIZE
    int skw_log_size = (1 * 1024 * 1024);
#else
    int skw_log_size = (100 * 1024 * 1024);
#endif
#define SKW_LOG_READ_BUFFER_SIZE (8*1024)

#ifndef CONFIG_NO_GKI
    #define CONFIG_NO_GKI
#endif

module_param(skw_log_size, int, 0644);
module_param(skw_log_num, int, 0644);

struct skw_log_data
{
    spinlock_t lock;

    int state;

    /* synchronize access to our device file */
    atomic_t open_excl;
    /* to enforce only one ioctl at a time */
    atomic_t ioctl_excl;


    int rx_done;
    /* for processing MTP_SEND_FILE, MTP_RECEIVE_FILE and
     * MTP_SEND_FILE_WITH_HEADER ioctls on a work queue
     */
    struct workqueue_struct *wq;
    struct work_struct log_to_file_work;
    struct file *xfer_file;
    loff_t xfer_file_offset;
    int64_t xfer_file_length;
    unsigned xfer_send_header;
    uint16_t xfer_command;
    uint32_t xfer_transaction_id;
    int xfer_result;
};

struct log_com_dev
{
    atomic_t open;
    spinlock_t lock;
    int     rx_busy;
    int tx_busy;
    int devno;
    int portno;
    struct sv6160_platform_data *pdata;
    wait_queue_head_t wq;
    struct cdev cdev;
    char    *rx_buf;
    char    *tx_buf;
    struct notifier_block notifier;
};
struct skw_log_read_buffer
{
    int     lenth;
    char    *buffer;
};


void skw_modem_log_to_file_work(struct work_struct *data);
extern int skw_cp_exception_reboot(void);

int skw_modem_save_dumpmem(void);
static uint32_t record_flag = 0;
static uint32_t cp_assert_status = 0;
struct file *log_fp = NULL;
struct log_com_dev *log_com = NULL;
struct sv6160_platform_data *port_data = NULL;
struct skw_log_data *skw_log_dev = NULL;
struct skw_log_read_buffer log_read_buffer;

char *log0_file = "log000";
char *log1_file = "log111";
char *log_file;

char *skw_code_mem = "code_mem_100000_7a000";
char *skw_data_mem = "data_mem_20200000_40000";
char *skw_cscb_mem = "cscb_mem_e000ed00_300";
char *skw_wreg_mem = "wreg_mem_40820000_4000";
char *skw_phyr_mem = "phyr_mem_40830000_4000";
char *skw_smem_mem = "smem_mem_40a00000_58000";
char *skw_umem_mem = "umem_mem_40b00000_c000";
char *skw_modem_mem = "sdio_mem_401e0000_800";
char *skw_btdm_mem = "btdm_mem_41000000_400";
char *skw_btbt_mem = "btbt_mem_41000400_400";
char *skw_btle_mem = "btle_mem_41000800_400";
char *skw_btem_mem = "btem_mem_41022000_c000";

int modem_event_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
    unsigned long flags;
    skwboot_log("%s event = %d\n", __func__, (int)action);
    switch (action)
    {
    case DEVICE_ASSERT_EVENT:
    {
        struct log_com_dev *ucom = log_com;
        skwboot_log("the BSPASSERT EVENT Comming in !!!!\n");
        skw_modem_log_start_rec();
        spin_lock_irqsave(&ucom->lock, flags);
        if (ucom->tx_busy)
        {
            spin_unlock_irqrestore(&ucom->lock, flags);
            skwboot_err("%s error 0\n", __func__);
            return NOTIFY_OK;
        }
        ucom->tx_busy = 1;
        spin_unlock_irqrestore(&ucom->lock, flags);
        skw_modem_log_set_assert_status(1);
    }
    break;
    case DEVICE_BSPREADY_EVENT:
    {
        skwboot_log("the BSPREADY EVENT Comming in !!!!\n");
        skw_modem_log_start_rec();
    }
    break;
    case DEVICE_DUMPDONE_EVENT:
    {
        skwboot_log("the DUMPDONE EVENT Comming in !!!!\n");
        skw_modem_log_stop_rec();
        skw_modem_log_set_assert_status(0);
    }
    break;
    case DEVICE_BLOCKED_EVENT:
    {
        skwboot_log("the BLOCKED EVENT Comming in !!!!\n");
        skw_modem_dumpmodem_start_rec();
    }
    break;
    case DEVICE_DUMPMEM_EVENT:
    {
        skw_modem_dumpmodem_start_rec();
    }
    break;
    default:
    {

    }
    break;

    }
    return NOTIFY_OK;
}

void skw_modem_log_to_file_work(struct work_struct *data)
{
#ifdef CONFIG_NO_GKI
    //int log_len = 0;
    struct file *fp = log_fp;
    struct file *log_store_fp = NULL;
    loff_t offset = 0;
    loff_t log_store_offset = 0;
    unsigned long flags;
    uint32_t *rx_data;
    size_t count = 0;
    int ret = 0;
    int i = 0;
    int log_cnt = 0;
    int sdma_rx_error_cnt = 0;
    char *log_store;
    int  log_path_len;
    char rd_buff[50];

    if (record_flag)
        return;
    record_flag = 1;

    skwlog_log("log path = %s \n", log_path);
    log_path_len = strlen(log_path);

    log_file = kzalloc(log_path_len + 16, GFP_KERNEL);

    if (log_file == NULL)
        return;
    log_store = kzalloc(log_path_len + 16, GFP_KERNEL);
    if (log_store == NULL)
    {
        kfree(log_file);
        return;
    }

    sprintf(log_store, "%s/log_store", log_path);
    log_store_fp = filp_open(log_store, O_RDWR, 0777);
    while (IS_ERR(log_store_fp))
    {
        skwlog_err("open log_store %s failed:%d	\n", log_store, (int)PTR_ERR(log_store_fp));
        msleep(1000);
        i++;
        if (i > 8)
        {
            kfree(log_store);
            kfree(log_file);
            return;
        }
        if (i > 5)
        {
            skwlog_err("%s open log_store file failed, create file:%s\n", __func__, log_store);
            log_store_fp = filp_open(log_store, O_CREAT | O_RDWR | O_TRUNC, 0777);
        }
        else
        {
            log_store_fp = filp_open(log_store, O_RDWR, 0777);
        }
    }

    ret = skw_read_file(log_store_fp, rd_buff, 30, &log_store_offset);
    if (ret < 0)
    {
        skwlog_err("%s Read file:%s failed, err:%d \n", __func__, log_store, ret);
    }

    rd_buff[0] = rd_buff[0] + 1;
    if (rd_buff[0] > skw_log_num)
    {
        rd_buff[0] = 0;
    }
    sprintf(log_file, "%s/%s", log_path, log0_file);
    log_path_len = strlen(log_file);
    *(log_file + log_path_len - 1) = ((rd_buff[0] % 10) | 0x30);
    *(log_file + log_path_len - 2) = (((rd_buff[0] / 10) % 10) | 0x30);

    log_store_offset = 0;
    ret = skw_write_file(log_store_fp, rd_buff, 1, &log_store_offset);
    if (ret < 0)
    {
        skwlog_err("%s write file:%s failed, err:%d \n", __func__, log_store, ret);
    }
    ret = skw_write_file(log_store_fp, log_file, strlen(log_file), &log_store_offset);
    if (ret < 0)
    {
        skwlog_err("%s write f name to file:%s failed, err:%d \n", __func__, log_store, ret);
    }

    log_fp = filp_open(log_file, O_CREAT | O_RDWR | O_TRUNC, 0777);
    fp = log_fp;

    while (IS_ERR(fp))
    {
        skwlog_err("open rec file %s failed :%d	\n", log_file, (int)PTR_ERR(fp));
        msleep(500);
        i++;
        if (i > 10)
        {
            kfree(log_store);
            kfree(log_file);
            return;
        }

        log_fp = filp_open(log_file, O_CREAT | O_RDWR | O_TRUNC, 0777);
        fp = log_fp;
    }
    atomic_inc(&log_com->open);
    if (atomic_read(&log_com->open) == 1)
    {
        init_waitqueue_head(&log_com->wq);
        spin_lock_init(&log_com->lock);
        log_com->pdata->open_port(log_com->portno, NULL, NULL);
    }

    log_read_buffer.lenth = 0;
    log_read_buffer.buffer = kzalloc(SKW_LOG_READ_BUFFER_SIZE, GFP_KERNEL);
    if (!log_read_buffer.buffer)
    {
        kfree(log_store);
        kfree(log_file);
        return;
    }
    skwlog_log(" open %s for CP log record \n", log_file);
    while (record_flag || cp_assert_status)
    {
        ret = 0;
        if (log_com)
        {
check_rx_busy:
            spin_lock_irqsave(&log_com->lock, flags);
            if (log_com->rx_busy)
            {
                spin_unlock_irqrestore(&log_com->lock, flags);
                mdelay(5);
                goto check_rx_busy;
            }
            log_com->rx_busy = 1;
            count = log_com->pdata->max_buffer_size;
            spin_unlock_irqrestore(&log_com->lock, flags);
            ret = log_com->pdata->hw_sdma_rx(log_com->portno, (log_read_buffer.buffer + log_read_buffer.lenth), count);

            if (ret > 0)
            {
                log_cnt++;
                sdma_rx_error_cnt = 0;
                log_read_buffer.lenth = log_read_buffer.lenth + ret;
                //skwlog_log("hw_sdma_rx:%s read len:%d buffer len:%d \n", skw_log, ret, log_read_buffer.lenth);
                if (ret >= 0x1000)
                    skwlog_err("%s get too long data , err:%d \n", __func__, ret);

                if (log_cnt > 1000)
                {
                    skwlog_log("%s log_file:%s offset:%lld data:0x%x 0x%x 0x%x 0x%x 0x%x	\n", __func__, log_file, offset, *(log_read_buffer.buffer),
                               *(log_read_buffer.buffer + 1), *(log_read_buffer.buffer + 2), *(log_read_buffer.buffer + 3), *(log_read_buffer.buffer + 4));
                    log_cnt = 0;
                }
            }
            else
            {
                skwlog_err("%s read log data err:%d \n", __func__, ret);
                sdma_rx_error_cnt++;
                if (sdma_rx_error_cnt > 5)
                {
                    skwlog_err("%s sdma_rx_error_cnt over:%d, stop log work \n", __func__, sdma_rx_error_cnt);
                    skw_modem_log_set_assert_status(0);
                    skw_modem_log_stop_rec();
                }
            }

            if (port_data->bus_type == USB_LINK)
            {
                if (ret < 0)
                {
                    skwlog_err("%s read log data err:%d, stop log work \n", __func__, ret);
                    skw_modem_log_set_assert_status(0);
                    skw_modem_log_stop_rec();
                }
            }
            else if (port_data->bus_type == SDIO_LINK)
            {
                if (ret == -ENOTCONN)
                {
                    skw_modem_log_set_assert_status(0);
                    skw_modem_log_stop_rec();
                }
            }
            //skwlog_log("read log from SDIO len:%d  ----- \n", log_read_buffer.lenth);
            log_com->rx_busy = 0;
            rx_data = (uint32_t *)log_read_buffer.buffer;
        }

        if (((log_read_buffer.lenth > 0) && cp_assert_status)
                || ((SKW_LOG_READ_BUFFER_SIZE - log_read_buffer.lenth) <= (log_com->pdata->max_buffer_size)))
        {
            //skwlog_log("skw_write_file:%s offset:%lld lenth:%d \n", skw_log, offset, log_read_buffer.lenth);
            ret = skw_write_file(fp, log_read_buffer.buffer, log_read_buffer.lenth, &offset);
            if (ret < 0)
            {
                skwlog_err("%s write file failed, err:%d \n", __func__, ret);
            }

            log_read_buffer.lenth = 0;
            if (ret == -ENOSPC)
            {
                skwlog_err("%s no space, stop CP log record \n", __func__);
                skw_modem_log_stop_rec();
            }

            if (offset > skw_log_size && (!cp_assert_status))
            {
                if (!IS_ERR(fp))
                    filp_close(fp, NULL);

                rd_buff[0] = rd_buff[0] + 1;
                if (rd_buff[0] > skw_log_num)
                {
                    rd_buff[0] = 0;
                }
                sprintf(log_file, "%s/%s", log_path, log0_file);
                log_path_len = strlen(log_file);
                *(log_file + log_path_len - 1) = ((rd_buff[0] % 10) | 0x30);
                *(log_file + log_path_len - 2) = (((rd_buff[0] / 10) % 10) | 0x30);

                log_store_offset = 0;
                ret = skw_write_file(log_store_fp, rd_buff, 1, &log_store_offset);
                if (ret < 0)
                {
                    skwlog_err("%s write file:%s failed, err:%d \n", __func__, log_store, ret);
                }
                ret = skw_write_file(log_store_fp, log_file, strlen(log_file), &log_store_offset);
                if (ret < 0)
                {
                    skwlog_err("%s write f name to file:%s failed, err:%d \n", __func__, log_store, ret);
                }

                log_fp = filp_open(log_file, O_CREAT | O_RDWR | O_TRUNC, 0777);
                fp = log_fp;
                if (IS_ERR(fp))
                {
                    skwlog_err("%s switch record file to:%s failed: %d \n", __func__, log_file, (int)PTR_ERR(fp));
                    kfree(log_file);
                    kfree(log_store);
                    kfree(log_read_buffer.buffer);
                    return;
                }
                else
                {
                    skwlog_err("%s switch record file to:%s sucess \n", __func__, log_file);
                }

                offset = 0;
            }
#if 0
            ret = vfs_fssync(fp, 0);
            if (ret < 0)
            {
                skwlog_err("%s fssync file failed, err:%d	\n", __func__, ret);
            }
#endif
        }
    }
    atomic_dec(&log_com->open);
    log_com->pdata->close_port(log_com->portno);

    if (!IS_ERR(fp))
    {
        filp_close(fp, NULL);
        skwlog_log("%s close file %s before stop work.\n", __func__, log_file);
        log_fp = NULL;
        fp = log_fp;
    }

    if (!IS_ERR(log_store_fp))
    {
        filp_close(log_store_fp, NULL);
        skwlog_log("%s close file %s before stop work.\n", __func__, log_store);
        log_store_fp = NULL;
    }
    kfree(log_file);
    kfree(log_store);
    kfree(log_read_buffer.buffer);
    skwlog_log("%s work exit\n", __func__);
#endif
    return;
}

/***************************************************************************
 *Description:dump modem memory
 *Seekwave tech LTD
 *Author:JunWei Jiang
 *Date:2022-11-14
 *Modify:
 **************************************************************************/
int skw_modem_save_mem(char *mem_path, unsigned int mem_len, unsigned int mem_addr)
{
    int ret = 0;
#ifdef CONFIG_NO_GKI
    char dump_mem_file[128];
    //dump code data
    char *data_mem = NULL;
    struct file *fp = NULL;
    loff_t pos = 0;
    int nwrite = 0;
    unsigned int read_len = 0;
    unsigned int mem_size = mem_len;

    memset(dump_mem_file, 0, sizeof(dump_mem_file));
    sprintf(dump_mem_file, "%s/%s", log_path, mem_path);
    data_mem = kzalloc(SKW_MAX_BUF_SIZE, GFP_KERNEL);
    if (!data_mem)
    {
        skwlog_log("the kzalloc dump buffer fail");
        return -2;
    }
    /* open file to write */
    fp = filp_open(dump_mem_file, O_CREAT | O_RDWR | O_TRUNC, 0777);
    if (IS_ERR(fp))
    {
        skwlog_log("open file %s fail try again!!!\n", dump_mem_file);
        fp = filp_open(mem_path, O_CREAT | O_RDWR | O_TRUNC, 0777);
        if (IS_ERR(fp))
        {
            skwlog_log("open file error\n");
            ret = -1;
            goto exit;
        }
    }

    while (mem_size)
    {
        if (mem_size < SKW_MAX_BUF_SIZE)
        {
            read_len = mem_size;
        }
        else
        {
            read_len = SKW_MAX_BUF_SIZE;
        }
        //skwlog_log("the read_len =0x%x mem_size= 0x%x\n", read_len, mem_size);
        ret = log_com->pdata->skw_dump_mem(mem_addr + (mem_len - mem_size), (void *)data_mem, read_len);
        if (ret < 0)
            break;

        //print_hex_dump(KERN_ERR, "img data ", 0, 16, 1,data_mem, 32, 1);
        //pos=(unsigned long)offset;
        /* Write buf to file */
        nwrite = skw_write_file(fp, data_mem, read_len, &pos);
        //offset +=nwrite;

        if (mem_size >= SKW_MAX_BUF_SIZE)
        {
            mem_size = mem_size - SKW_MAX_BUF_SIZE;
        }
        else
        {
            mem_size = 0;
        }
    }
    skwlog_log("Dump %s memory done !!\n", mem_path);
    if (fp)
    {
        filp_close(fp, NULL);
        skwlog_log("the file close!!!\n");
    }
exit:
    if (fp)
        filp_close(fp, NULL);
    kfree(data_mem);
#endif
    return ret;
}

/***************************************************************************
 *Description:dump modem memory
 *Seekwave tech LTD
 *Author:JunWei Jiang
 *Date:2022-11-14
 *Modify:
 **************************************************************************/
int skw_modem_save_dumpmem(void)
{
    int ret = 0;
    if (!log_com->pdata->skw_dump_mem)
        return 0;
    skwlog_log("%s [Enter]\n", __func__);
    //DATA MEM
    ret = skw_modem_save_mem(skw_data_mem, DATA_MEM_SIZE, DATA_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_data_mem, ret);
        return -1;
    }
    //CODE MEM
    ret = skw_modem_save_mem(skw_code_mem, CODE_MEM_SIZE, CODE_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_code_mem, ret);
        return -1;
    }
    //CSCB MEM
    ret = skw_modem_save_mem(skw_cscb_mem, CSCB_MEM_SIZE, CSCB_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_cscb_mem, ret);
        return -1;
    }

    //UMEM MEM
    ret = skw_modem_save_mem(skw_umem_mem, UMEM_MEM_SIZE, UMEM_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_umem_mem, ret);
        return -1;
    }
    //SDIO MEM
    ret = skw_modem_save_mem(skw_modem_mem, SDIO_MEM_SIZE, SDIO_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_modem_mem, ret);
        return -1;
    }
    //BTDM MEM
    ret = skw_modem_save_mem(skw_btdm_mem, BTDM_MEM_SIZE, BTDM_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_btdm_mem, ret);
        return -1;
    }
    //BTBT MEM
    ret = skw_modem_save_mem(skw_btbt_mem, BTBT_MEM_SIZE, BTBT_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_btbt_mem, ret);
        return -1;
    }
    //BTLE MEM
    ret = skw_modem_save_mem(skw_btle_mem, BTLE_MEM_SIZE, BTLE_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_btle_mem, ret);
        return -1;
    }
    //BTEM MEM
    ret = skw_modem_save_mem(skw_btem_mem, BTEM_MEM_SIZE, BTEM_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_btem_mem, ret);
        return -1;
    }
    //WREG MEM
    ret = skw_modem_save_mem(skw_wreg_mem, WREG_MEM_SIZE, WREG_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_wreg_mem, ret);
        return -1;
    }
    //PHYR MEM
    ret = skw_modem_save_mem(skw_phyr_mem, PHYR_MEM_SIZE, PHYR_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_phyr_mem, ret);
        return -1;
    }
    //SMEM MEM
    ret = skw_modem_save_mem(skw_smem_mem, SMEM_MEM_SIZE, SMEM_MEM_BASE_ADDR);
    if (ret != 0)
    {
        skwlog_log("%s dump fail ret: %d\n", skw_smem_mem, ret);
        return -1;
    }
    return ret;
}

int skw_modem_log_init(struct sv6160_platform_data *p_data, struct file *fp, void *ucom)
{

    int ret = 0;
#ifdef CONFIG_NO_GKI
    if (skw_log_dev)
        return ret;
    if (skw_log_size > (200 * 1024 * 1024))
        skw_log_size = (200 * 1024 * 1024);
    if (skw_log_size < (512 * 1024))
        skw_log_size = (512 * 1024);
    if ((skw_log_size * skw_log_num) > (1024 * 1024 * 1024))
        skw_log_num = (1024 * 1024 * 1024) / skw_log_size;
    if (skw_log_num > 99)
        skw_log_num = 99;

    skwlog_log("%s enter skw_log_num:%d skw_log_size:%d   \n", __func__, skw_log_num, skw_log_size);
    log_fp = fp;
    log_com = ucom;
    port_data = p_data;

    skw_log_dev = (struct skw_log_data *)kzalloc(sizeof(*skw_log_dev), GFP_KERNEL);
    if (!skw_log_dev)
    {
        ret = -ENOMEM;
        skwlog_err("%s line:%d,can't malloc \n", __func__, __LINE__);
        goto err1;
    }

    spin_lock_init(&skw_log_dev->lock);
    atomic_set(&skw_log_dev->open_excl, 0);
    atomic_set(&skw_log_dev->ioctl_excl, 0);
    //INIT_LIST_HEAD(&skw_log_dev->tx_idle);


    skw_log_dev->wq = create_singlethread_workqueue("skw_log");
    if (!skw_log_dev->wq)
    {
        ret = -ENOMEM;
        goto err2;
    }
    INIT_WORK(&skw_log_dev->log_to_file_work, skw_modem_log_to_file_work);

    if (log_com->pdata->modem_register_notify)
    {
        if (log_com->notifier.notifier_call == NULL)
        {
            log_com->notifier.notifier_call = modem_event_notifier;
            log_com->pdata->modem_register_notify(&log_com->notifier);
        }
    }

    skw_modem_log_start_rec();
    return 0;

err2:
    destroy_workqueue(skw_log_dev->wq);
    kfree(skw_log_dev);
    skwlog_err("mtp gadget driver failed to initialize\n");

err1:
#else
    skwlog_warn("GKI Not support log2file!\n");
#endif
    return ret;
}

void skw_modem_log_set_assert_status(uint32_t cp_assert)
{
    cp_assert_status = cp_assert;
    if (cp_assert_status)
    {
        skwlog_log("%s CP in ASSERT, dump log in %s \n", __func__, log_file);
    }
}

void skw_modem_log_start_rec(void)
{
#ifdef CONFIG_NO_GKI
    skwlog_log("%s enter  \n", __func__);
    if (atomic_read(&log_com->open) > 1)
    {
        skwlog_warn("%s:log port is busy\n", __func__);
        return;
    }
    if (!skw_log_dev)
    {
        skwlog_warn("%s no mem ready, can't start \n", __func__);
        return;
    }
    if (record_flag)
    {
        skwlog_warn("%s lof2file already \n", __func__);
        return;
    }
    cp_assert_status = 0;
    queue_work(skw_log_dev->wq, &skw_log_dev->log_to_file_work);
#else
    skwlog_warn("GKI Not support log2file!\n");
#endif

}

/***************************************************************************
 *Description:dump modem memory
 *Seekwave tech LTD
 *Author:JunWei Jiang
 *Date:2022-11-14
 *Modify:
 **************************************************************************/
void skw_modem_dumpmodem_start_rec(void)
{
#ifdef CONFIG_NO_GKI
    skw_modem_save_dumpmem();
#else
    skwlog_warn("GKI Not support dump!\n");
#endif

}

/***************************************************************************
 *Description:dump modem memory
 *Seekwave tech LTD
 *Author:JunWei Jiang
 *Date:2022-11-14
 *Modify:
 **************************************************************************/

void skw_modem_log_stop_rec(void)
{
    skwlog_log("%s enter \n", __func__);
    if (record_flag)
        record_flag = 0;
    if (log_com && log_com->pdata && log_com->pdata->close_port)
        log_com->pdata->close_port(log_com->portno);
    return;
}

void skw_modem_log_exit(void)
{
#ifdef CONFIG_NO_GKI
    skwlog_log("%s enter\n", __func__);
    if (!log_com) return;
    log_com->pdata->modem_unregister_notify(&log_com->notifier);
    skw_modem_log_stop_rec();
    cancel_work_sync(&skw_log_dev->log_to_file_work);
    destroy_workqueue(skw_log_dev->wq);
    kfree(skw_log_dev);
    skw_log_dev = NULL;
    log_com = NULL;
#endif
}

//DECLARE_USB_FUNCTION_INIT(mtp, mtp_alloc_inst, mtp_alloc);
MODULE_LICENSE("GPL");
#endif
