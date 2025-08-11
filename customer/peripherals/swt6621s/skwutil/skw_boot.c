/*****************************************************************
 *Copyright (C) 2021 Seekwave Tech Inc.
 *Filename : skw_boot.c
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
#if 0
#include "drv_config.h"
#include "drv_io.h"
#include <rtthread.h>

#include <drivers/sdio_func_ids.h>
#include "skw_mem_map.h"
#include "skw_boot.h"
#include "boot_config.h"
/**************************sdio boot start******************************/
extern int cp_exception_sts;
int test_debug = 0;
unsigned char dl_signal_acount = 0;
struct rt_device_t *btboot_pdev;
static u64 port_dmamask = DMA_BIT_MASK(32);
static struct mutex boot_mutex;

#ifdef CONFIG_OF
    #undef CONFIG_OF
#endif
//#define SDIO_BUFFER_SIZE   (16*1024)
/*
 *add the little endian
 * */
#define _LITTLE_ENDIAN  1

#define CP_IMG_HEAD0    "kees"       //"6B656573"
#define CP_IMG_HEAD1    "0616"      //"30363136"
#define CP_IMG_TAIL0    "evaw"      //"65766177"
#define CP_IMG_TAIL1    "0616"      //"30363136" //ASCII code 36 31 36 30
#define CP_NV_HEAD    "TSVN"        //"5453564E" //ASCII code 36 31 36 30
#define CP_NV_TAIL       "DEVN"     //"4445564E" //ASCII code 36 31 36 30

#define IMG_HEAD_OPS_LEN    4
#define RAM_ADDR_OPS_LEN    8
#define MODULE_INFO_LEN     12

#define IMG_HEAD_INFOR_RANGE    0x200  //10K Byte

static unsigned int EndianConv_32(unsigned int value);
/***********sdio drv extern interface **************/
/* driect mode,reg access.etc */
//extern int skw_get_chipid(char *chip_id);
extern int skw_boot_loader(struct seekwave_device *boot_data);
extern void *skw_get_bus_dev(void);
extern int skw_reset_bus_dev(void);
static int skw_first_boot(struct seekwave_device *boot_data);
static int skw_boot_init(struct seekwave_device *boot_data);
static int skw_download_signal_ops(void);
static int get_sleep_status(int portno, char *buffer, int size);
static int set_sleep_status(int portno, char *buffer, int size);
//int skw_cp_exception_reboot(void);
static int skw_start_bt_service(void);
static int skw_stop_bt_service(void);
/**************************sdio boot end********************************/
struct seekwave_device *boot_data;

//=======================================================
//debug sdio macro and Variable
//int glb_wifiready_done;
#define SKW_CPBOOT_DEBUG 0
//=======================================================
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static unsigned int crc_16_l_calc(char *buf_ptr, unsigned int len)
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


static int skw_request_firmwares(struct seekwave_device *boot_data,
                                 const char *dram_image_name, const char *iram_image_name, const char *nv_mem_name)
{
    int ret;
    const struct firmware *fw;

    skwboot_log("request_firmware %s\n", dram_image_name);
    ret = request_firmware(&fw, dram_image_name, NULL);
    if (ret)
    {
        pr_err("request_firmware %s fail\n", dram_image_name);
        goto ret;
    }

    boot_data->dram_img_data = (char *)kzalloc(fw->size, GFP_KERNEL);
    if (boot_data->dram_img_data == NULL)
    {
        pr_err("alloc dram memory failed\n");
        ret = -ENOMEM;
        goto relese_fw;
    }
    skwboot_log("boot data dram_img_data %p\n", boot_data->dram_img_data);
    memcpy(boot_data->dram_img_data, fw->data, fw->size);
    boot_data->dram_dl_size = fw->size;
    release_firmware(fw);
    //dram crc16
    boot_data->dram_crc_en = 1;
    boot_data->dram_crc_offset = 0;
    boot_data->dram_crc_val = crc_16_l_calc(boot_data->dram_img_data + boot_data->dram_crc_offset, boot_data->dram_dl_size);

    skwboot_log("request_firmware %s\n", iram_image_name);
    ret = request_firmware(&fw, iram_image_name, NULL);
    if (ret)
    {
        kfree(boot_data->dram_img_data);
        boot_data->dram_img_data = NULL;
        pr_err("request_firmware %s fail\n", iram_image_name);
        goto ret;
    }

    boot_data->iram_img_data = (char *)kzalloc(fw->size, GFP_KERNEL);
    if (boot_data->iram_img_data == NULL)
    {
        kfree(boot_data->dram_img_data);
        boot_data->dram_img_data = NULL;
        pr_err("alloc iram memory failed\n");
        ret = -ENOMEM;
        goto relese_fw;
    }
    memcpy(boot_data->iram_img_data, fw->data, fw->size);
    boot_data->iram_dl_size = fw->size;
    release_firmware(fw);
    //iram crc16
    boot_data->iram_crc_en = 1;
    boot_data->iram_crc_offset = 0;
    boot_data->iram_crc_val = crc_16_l_calc(boot_data->iram_img_data + boot_data->iram_crc_offset, boot_data->iram_dl_size);

    skwboot_log("boot data iram_img_data %p\n", boot_data->iram_img_data);

    skwboot_log("request_firmware %s\n", nv_mem_name);
    ret = request_firmware(&fw, nv_mem_name, NULL);
    if (ret)
    {
        skwboot_err("request_firmware %s fail\n", nv_mem_name);
        ret = ENOENT;
        goto ret;
    }

    boot_data->nv_mem_data = (char *)kzalloc(fw->size, GFP_KERNEL);
    if (boot_data->nv_mem_data == NULL)
    {
        kfree(boot_data->iram_img_data);
        boot_data->iram_img_data = NULL;
        kfree(boot_data->dram_img_data);
        boot_data->dram_img_data = NULL;
        skwboot_err("alloc nv memory failed\n");
        goto relese_fw;
    }
    memcpy(boot_data->nv_mem_data, fw->data, fw->size);
    boot_data->nv_mem_size = fw->size;
    if (boot_data->nv_mem_size > 20)  //new nv
    {
        boot_data->nv_mem_cmfg_data = boot_data->nv_mem_data + *(u32 *)(boot_data->nv_mem_data + NV_CMFG_OFFSET);
        boot_data->nv_mem_cmfg_size = *(u32 *)(boot_data->nv_mem_data + NV_CMFG_SIZE);

        if (*(u32 *)(boot_data->nv_mem_data + NV_PNFG_OFFSET) != 0)
            boot_data->nv_mem_pnfg_data = boot_data->nv_mem_data + *(u32 *)(boot_data->nv_mem_data + NV_PNFG_OFFSET);
        else
            boot_data->nv_mem_pnfg_data = NULL;
        boot_data->nv_mem_pnfg_size = *(u32 *)(boot_data->nv_mem_data + NV_PNFG_SIZE);
    }
    ret = 0;
    boot_data->nvmem_crc_en = 1;
    boot_data->nvmem_crc_offset = 0;
    boot_data->nvmem_crc_val = crc_16_l_calc(boot_data->nv_mem_data + boot_data->nvmem_crc_offset, boot_data->nv_mem_size);
    //print_hex_dump(KERN_ERR, "nvdata:", 0, 16, 1, boot_data->nv_mem_data, boot_data->nv_mem_size, 1);

relese_fw:
    release_firmware(fw);
ret:
    return ret;
}

static int seekwave_boot_parse_dt(struct platform_device *pdev, struct seekwave_device *boot_data)
{
    int ret = 0;

    boot_data->bt_antenna = SKW_BT_ANTENNA_CFG;
    boot_data->dma_type = SKW_DMA_TYPE_CFG;
    boot_data->chip_en = MODEM_ENABLE_GPIO;
    boot_data->host_gpio =  HOST_WAKEUP_GPIO_IN;
    boot_data->chip_gpio =  MODEM_WAKEUP_GPIO_OUT;

    skwboot_log("%s,Only support no DTS setting::=>\n", __func__);
    skwboot_log("chipen=%d\n", boot_data->chip_en);
    skwboot_log("hst_wak_wf=%d\n", boot_data->chip_gpio);
    skwboot_log("wf_wak_hst=%d\n", boot_data->host_gpio);
    skwboot_log("dma_type=%d\n", boot_data->dma_type);
    skwboot_log("bt_antenna=%d\n", boot_data->bt_antenna);

    if (test_debug == 1) //test debug inband irq and nosleep en
    {
        boot_data->chip_gpio = -1;
        boot_data->host_gpio = -1;
    }

    if (boot_data->host_gpio >= 0)
    {
        ret = devm_gpio_request_one(&pdev->dev, boot_data->host_gpio, GPIOF_IN, "HOST_WAKE");
        if (ret < 0)
            skwboot_err("%s:gpio_host request fail ret=%d\n", __func__, ret);

        if (boot_data->chip_gpio >= 0)
            ret = devm_gpio_request_one(&pdev->dev, boot_data->chip_gpio, GPIOF_OUT_INIT_HIGH, "CHIP_WAKE");

        skwboot_log("%s,chipen=%d, gpio_out:%d gpio_in:%d ret = %d\n", __func__, boot_data->chip_en,
                    boot_data->chip_gpio, boot_data->host_gpio, ret);
    }

    if (boot_data->chip_gpio >= 0 && boot_data->host_gpio >= 0)
    {
        boot_data->slp_disable = 0;
    }
    else
    {
        boot_data->slp_disable = 1;
    }
    if (boot_data->chip_en >= 0)
        ret = devm_gpio_request_one(&pdev->dev, boot_data->chip_en, GPIOF_OUT_INIT_HIGH, "CHIP_EN");
    return ret;
}

/************************************************************************/
//Description: BT start service
//Func: BT start service
//Call：
//Author:junwei.jiang
//Date:2021-110
//Modify:
/************************************************************************/
static int bt_start_service(int id, void *callback, void *data)
{
    int ret = 0;
    if (cp_exception_sts)
        return -1;

    skwboot_log("%s pid:%d %s\n", current->comm, current->pid, __func__);
    ret = skw_start_bt_service();
    if (ret < 0)
    {
        skwboot_err("%s boot bt fail \n", __func__);
        return -1;
    }
    return 0;
}

/************************************************************************/
//Description: BT stop service
//Func: BT stop service
//Call：
//Author:junwei.jiang
//Date:2021-11-1
//Modify:
/************************************************************************/
static int bt_stop_service(int id)
{
    int ret = 0;

    if (cp_exception_sts)
        return 0;

    ret = skw_stop_bt_service();
    if (ret < 0)
    {
        skwboot_err("%s boot bt fail \n", __func__);
        return -1;
    }
    skwboot_log("%s OK\n", __func__);
    return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int seekwave_boot_probe(struct  platform_device *pdev)
{
    int ret;
    int time_count = 0;
    struct device *io_bus;

    boot_data = devm_kzalloc(&pdev->dev, sizeof(struct seekwave_device), GFP_KERNEL);
    if (!boot_data)
    {
        skwboot_err("%s :kzalloc error !\n", __func__);
        return -ENOMEM;
    }
    mutex_init(&boot_mutex);
    seekwave_boot_parse_dt(pdev, boot_data);
    io_bus = skw_get_bus_dev();
    if (!io_bus)
    {
        skwboot_log("%s :CHIP_RESET AGAIN!\n", __func__);
        skw_chip_power_reset();
        do
        {
            msleep(10);
            io_bus = skw_get_bus_dev();
        }
        while (!io_bus && time_count++ < 50);
    }
    if (!io_bus)
    {
        devm_kfree(&pdev->dev, boot_data);
        skwboot_err("%s get bus dev fail !\n", __func__);
        return -ENODEV;
    }
    if (!strncmp(io_bus->bus->name, "usb", 3))
    {
        boot_data->iram_file_path = "SWT6621S_IRAM_USB.bin";
        boot_data->dram_file_path = "SWT6621S_DRAM_USB.bin";
        boot_data->skw_nv_name = "SWT6621S_NV_USB.bin";
    }
    else
    {
        boot_data->iram_file_path = "SWT6621S_IRAM_SDIO.bin";
        boot_data->dram_file_path = "SWT6621S_DRAM_SDIO.bin";
        boot_data->skw_nv_name = "SWT6621S_NV_SDIO.bin";
    }
    skw_boot_init(boot_data);
#ifndef CONFIG_SEEKWAVE_USB_STR
    boot_data->pdev = pdev;
#endif
    ret = skw_first_boot(boot_data);
    if (strncmp(io_bus->bus->name, "usb", 3))
        skw_bind_boot_driver(io_bus);
    return ret;
}
/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int seekwave_boot_remove(struct  rt_device_t *pdev)
{
    skwboot_log("%s the Enter \n", __func__);

    if (btboot_pdev)
    {
        platform_device_unregister(btboot_pdev);
        btboot_pdev = NULL;
    }
    if (boot_data)
    {
        if (boot_data->iram_img_data)
        {
            kfree(boot_data->iram_img_data);
            boot_data->iram_img_data = NULL;
        }
        if (boot_data->dram_img_data)
        {
            kfree(boot_data->dram_img_data);
            boot_data->dram_img_data = NULL;
        }
        if (boot_data->dl_bin)
        {
            kfree(boot_data->dl_bin);
            boot_data->dl_bin = NULL;
        }
        if (boot_data->img_data)
        {
            kfree(boot_data->img_data);
            boot_data->img_data = NULL;
        }
        if (boot_data->dl_base_img)
        {
            kfree(boot_data->dl_base_img);
            boot_data->dl_base_img = NULL;
        }
        if (boot_data->nv_mem_size > 20)  //new nv
        {
            if (boot_data->nv_mem_data)
            {
                kfree(boot_data->nv_mem_data);
                boot_data->nv_mem_data = NULL;
            }
        }
        boot_data->skw_nv_name = NULL;
        boot_data->iram_file_path = NULL;
        boot_data->dram_file_path = NULL;
        boot_data->wifi_start = NULL;
        boot_data->wifi_stop = NULL;
        boot_data->bt_start = NULL;
        boot_data->bt_stop = NULL;
        boot_data->skw_dloader_module = NULL;
        devm_kfree(&pdev->dev, boot_data);
        boot_data = NULL;
    }
    mutex_destroy(&boot_mutex);
    return 0;
}
extern void skw_modem_log_stop_rec(void);
static void seekwave_boot_shutdown(struct platform_device *pdev)
{
    skwboot_log("%s enter ...\n", __func__);
    skw_modem_log_stop_rec();
    skw_reset_bus_dev();
}
static const struct of_device_id seekwave_match_table[] =
{

    { .compatible = "seekwave,sv6160lite"},
    { },
};

static struct platform_driver seekwave_driver =
{

    .driver = {
        .owner = THIS_MODULE,
        .name  = "sv6160lite",
        .of_match_table = seekwave_match_table,
    },
    .probe = seekwave_boot_probe,
    .remove = seekwave_boot_remove,
    .shutdown = seekwave_boot_shutdown,
};

/***********************************************************************
 *Description:BT download boot pdata
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:2021-11-3
 *Modify:
 ***********************************************************************/
static int get_sleep_status(int portno, char *buffer, int size)
{
    memcpy(buffer, "WAKE", 4);
    if (boot_data->host_gpio >= 0)
    {
        if (gpio_get_value(boot_data->host_gpio) == 0)
            memcpy(buffer, "DOWN", 4);
    }
    return 4;
}
static int set_sleep_status(int portno, char *buffer, int size)
{
    int i, count;

    for (i = 0; i < 2; i++)
    {
        if (gpio_get_value(boot_data->host_gpio))
            return 1;
        if (buffer && !strncmp(buffer, "WAKE", 4))
        {
            gpio_set_value(boot_data->chip_gpio, 0);
            udelay(10);
            gpio_set_value(boot_data->chip_gpio, 1);
        }
        count = 0;
        do
        {
            if (count++ < 100)
                udelay(20);
        }
        while (gpio_get_value(boot_data->host_gpio) == 0);
        if (gpio_get_value(boot_data->host_gpio))
            return 1;
        udelay(100);
    }
    if (gpio_get_value(boot_data->host_gpio) == 0)
        skwboot_log("wakeup CHIP timeout!!! \n");
    return 1;
}
struct sv6160_platform_data boot_pdata =
{
    .data_port = 8,
    .bus_type = SDIO_LINK,
    .max_buffer_size = 0x800,
    .align_value = 4,
    .hw_sdma_rx = get_sleep_status,
    .hw_sdma_tx = set_sleep_status,
    .open_port = bt_start_service,
    .close_port = bt_stop_service,
};

/***************************************************************
 *Description:BT bind boot driver
 *Seekwave tech LTD
 *Author:junwei.jiang
 *Date:2021-11-3
 *Modify:
***************************************************************/
int skw_bind_boot_driver(struct device *dev)
{
    struct platform_device *pdev;
    char    pdev_name[32];
    int ret = 0;
    sprintf(pdev_name, "skw_ucom");
    if (!dev)
    {
        skwboot_err("%s the dev fail \n", __func__);
        return -1;
    }
    if (btboot_pdev)
        return ret;

    pdev = platform_device_alloc(pdev_name, PLATFORM_DEVID_AUTO);
    if (!pdev)
        return -ENOMEM;
    pdev->dev.parent = dev;
    pdev->dev.dma_mask = &port_dmamask;
    pdev->dev.coherent_dma_mask = port_dmamask;
    boot_pdata.port_name = "BTBOOT";
    boot_pdata.data_port = 8;
    //skw_get_chipid((char *)&boot_data->chip_id);
    ret = platform_device_add_data(pdev, &boot_pdata, sizeof(boot_pdata));
    if (ret)
    {
        dev_err(dev, "failed to add boot data \n");
        platform_device_put(pdev);
        return ret;
    }
    ret = platform_device_add(pdev);
    if (ret)
    {
        platform_device_put(pdev);
        skwboot_err("%s,line:%d the device add fail \n", __func__, __LINE__);
        return ret;
    }
    btboot_pdev = pdev;
    return ret;
}
/****************************************************************
 *Description:the data Little Endian process interface
 *Func:EndianConv_32
 *Calls:None
 *Call By:The img data process
 *Input:value
 *Output:the Endian data
 *Return：value
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2021-08-26
 * **************************************************************/
static unsigned int EndianConv_32(unsigned int value)
{
#ifdef _LITTLE_ENDIAN
    unsigned int nTmp = (value >> 24 | value << 24);
    nTmp |= ((value >> 8) & 0x0000FF00);
    nTmp |= ((value << 8) & 0x00FF0000);
    return nTmp;
#else
    return value;
#endif
}

/****************************************************************
 *Description:dram read the double img file
 *Func:
 *Calls:
 *Call By:
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
static int skw_download_signal_ops(void)
{
    unsigned int tmp_signal = 0;
    //download done flag ++
    boot_data->dl_done_signal ++;
    tmp_signal = boot_data->dl_done_signal;
    boot_data->dl_done_signal = 0xff & tmp_signal;
    boot_data->dl_acount_addr = SKW_SDIO_PD_DL_AP2CP_BSP;

    //gpio need set high or low power interrupt to cp wakeup
    boot_data->gpio_out = boot_data->chip_gpio;
    if (boot_data->gpio_val)
        boot_data->gpio_val = 0;
    else
        boot_data->gpio_val = 1;
    skwboot_log("%s line:%d download data ops done the dl_count=%d \n", __func__, __LINE__, boot_data->dl_done_signal);
    return 0;
}

/****************************************************************
 *Description:analysis the double img dram iram
 *Func:
 *Calls:
 *Call By:
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
static int skw_boot_init(struct seekwave_device *boot_data)
{
    int i = 0;
    int k = 0;
    unsigned int head_offset = 0;
    unsigned int tail_offset = 0;
    int ret = 0;
    struct img_head_data_t dl_data_info;
    unsigned int *data = NULL;
    unsigned int *nvdata = NULL;
    unsigned int *dl_addr_data = NULL;

    ret = skw_request_firmwares(boot_data, boot_data->dram_file_path,
                                boot_data->iram_file_path, boot_data->skw_nv_name);
    if (ret == ENOENT)
    {
        ret = skw_request_firmwares(boot_data, boot_data->dram_file_path,
                                    boot_data->iram_file_path, SEEKWAVE_NV_NAME);
    }
    skwboot_log("image_size=%d,%d, ret=%d\n", boot_data->iram_dl_size, boot_data->dram_dl_size, ret);
    if (ret < 0)
    {
        return ret;
    }
    boot_data->head_addr = 0;
    boot_data->tail_addr = 0;
    boot_data->bsp_head_addr = 0;
    boot_data->bsp_tail_addr = 0;
    boot_data->wifi_head_addr = 0;
    boot_data->wifi_tail_addr = 0;
    boot_data->bt_head_addr = 0;
    boot_data->bt_tail_addr = 0;
    boot_data->nv_head_addr = 0;
    boot_data->nv_tail_addr = 0;
    boot_data->nv_data_size = 0;

    if (boot_data->iram_img_data != NULL)
    {
        /*analysis the img*/
        for (i = 0; i * IMG_HEAD_OPS_LEN < IMG_HEAD_INFOR_RANGE; i++)
        {
            if (!head_offset)
            {
                if ((0 == memcmp(CP_IMG_HEAD0, boot_data->iram_img_data + i * IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)) &&
                        (0 == memcmp(CP_IMG_HEAD1, boot_data->iram_img_data + (i + 1)*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)))
                    head_offset = (i + 1) * IMG_HEAD_OPS_LEN;
            }
            else if (!tail_offset)
            {
                if ((0 == memcmp(CP_IMG_TAIL0, boot_data->iram_img_data + i * IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)) &&
                        (0 == memcmp(CP_IMG_TAIL1, boot_data->iram_img_data + (i + 1)*IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)))
                {
                    tail_offset = (i - 1) * IMG_HEAD_OPS_LEN;
                    break;
                }
            }
        }

        /*analysis the nv*/
        for (k = 0; k * IMG_HEAD_OPS_LEN < IMG_HEAD_INFOR_RANGE; k++)
        {
            if (!boot_data->nv_head_addr)
            {
                if (0 == memcmp(CP_NV_HEAD, boot_data->iram_img_data + k * IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN))
                    boot_data->nv_head_addr = k * IMG_HEAD_OPS_LEN;
            }
            else if (!boot_data->nv_tail_addr)
            {
                if ((0 == memcmp(CP_NV_TAIL, boot_data->iram_img_data + k * IMG_HEAD_OPS_LEN, IMG_HEAD_OPS_LEN)))
                {
                    boot_data->nv_tail_addr = k * IMG_HEAD_OPS_LEN;
                    boot_data->nv_data_size = boot_data->nv_tail_addr - boot_data->nv_head_addr - IMG_HEAD_OPS_LEN;
                    nvdata = (u32 *) &boot_data->iram_img_data[boot_data->nv_head_addr];
                    break;
                }
            }
        }
        if (!tail_offset)
        {
            skwboot_err("%s,%d,the iram_img not need analysis!!! or Fail!! \n", __func__, __LINE__);
            return -1;
        }
        else
        {
            //get the iram img addr and dram img addr
            dl_addr_data = (unsigned int *)(boot_data->iram_img_data + head_offset + IMG_HEAD_OPS_LEN);
            boot_data->iram_dl_addr = dl_addr_data[0];
            boot_data->dram_dl_addr = dl_addr_data[1];
            head_offset = head_offset + RAM_ADDR_OPS_LEN; //jump the ram addr data;

            skwboot_log("%s line:%d,the tail_offset ---0x%x, the head_offset --0x%x ,iram_addr=0x%x,dram_addr=0x%x, \
					nv_head_addr:0x%x,nv_tail_addr:0x%x,nv_size=%d\n", __func__, __LINE__, tail_offset, head_offset,
                        boot_data->iram_dl_addr, boot_data->dram_dl_addr, boot_data->nv_head_addr, boot_data->nv_tail_addr,
                        boot_data->nv_data_size);
        }
        /*need download the img bin for WIFI or BT service dl_module >0*/
        head_offset = head_offset + IMG_HEAD_OPS_LEN;
        /*get the img head tail offset*/
        boot_data->head_addr = head_offset;
        boot_data->tail_addr = tail_offset;

        skwboot_log("%s line:%d analysis the img module\n", __func__, __LINE__);
        for (i = 0; i * MODULE_INFO_LEN <= (tail_offset - head_offset); i++)
        {
            data = (unsigned int *)(boot_data->iram_img_data + head_offset + i * MODULE_INFO_LEN);
            dl_data_info.dl_addr = data[0];
            dl_data_info.write_addr = data[2];
            dl_data_info.index = 0x000000FF & EndianConv_32(data[1]);
            dl_data_info.data_size = 0x00FFFFFF & data[1];
            if (dl_data_info.index == 1)
            {
                boot_data->bsp_index_count += 1;
            }
            else if (dl_data_info.index == 2)
            {
                boot_data->wifi_index_count += 1;
            }
            else if (dl_data_info.index == 3)
            {
                boot_data->bt_index_count += 1;
            }
            skwboot_log("%s line:%d dl_addr=0x%x, write_addr=0x%x, index=0x%x,data_size=0x%x\n", __func__,
                        __LINE__, dl_data_info.dl_addr, dl_data_info.write_addr, dl_data_info.index, dl_data_info.data_size);
        }
        skwboot_log("%s line:%d bsp_index count:%d, bt_index_count=%d, wifi_index_count=%d \n",
                    __func__, __LINE__, boot_data->bsp_index_count, boot_data->bt_index_count, boot_data->wifi_index_count);
        //get the dl count for the service download module img
        boot_data->wifi_dl_count = boot_data->bsp_index_count + boot_data->wifi_index_count;
        boot_data->all_dl_count = boot_data->bsp_index_count + boot_data->wifi_index_count + boot_data->bt_index_count;
        boot_data->bt_dl_count = boot_data->bsp_index_count + boot_data->bt_index_count;

        if (boot_data->nv_mem_size > 20)  //new nv
        {
            if (boot_data->nv_mem_cmfg_size && (boot_data->nv_mem_cmfg_size <= boot_data->nv_data_size))
            {
                memcpy((boot_data->iram_img_data + boot_data->nv_head_addr + 4), boot_data->nv_mem_cmfg_data, boot_data->nv_mem_cmfg_size);
                //kfree(boot_data->nv_mem_data);
                //boot_data->nv_mem_data = NULL;
            }
        }
        else    //old nv
        {
            if (boot_data->nv_mem_size && (boot_data->nv_mem_size <= boot_data->nv_data_size))
            {
                memcpy((boot_data->iram_img_data + boot_data->nv_head_addr + 4), boot_data->nv_mem_data, boot_data->nv_mem_size);
                kfree(boot_data->nv_mem_data);
                boot_data->nv_mem_data = NULL;
            }
        }
        //print_hex_dump(KERN_ERR, "nvcom ", 0, 16, 1,boot_data->iram_img_data+boot_data->nv_head_addr, boot_data->nv_data_size+8, 1);
    }
    return 0;
}
//debug cp boot for setvalue
int gbl_count_flag = 0;
/****************************************************************
 *Description:download the wifi bt service img,
 *Func:
 *Calls:
 *Call By:
 *Input:the service index
 *Output:download data
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2024-02-07
 * **************************************************************/
static int skw_dloader_module(int service_index)
{
    int i = 0;
    int ret = 0;
    struct img_head_data_t dl_data_info;
    unsigned int *data = NULL;
    int tmp_count = 0;
    int total_count = 0;
#if SKW_CPBOOT_DEBUG//for wifionly debug cp boot sleep no dl img setvalue 1;
    skwboot_log("%s:the debug skw_dloader Enter \n", __func__);
    return 0;
#endif
    //dl count flag setvalue
    if (service_index == SKW_WIFI)
    {
        tmp_count = boot_data->wifi_dl_count;
    }
    else if (service_index == SKW_BT)
    {
        tmp_count = boot_data->bt_dl_count;
    }
    else if (service_index == SKW_ALL)
    {
        tmp_count = boot_data->all_dl_count;
    }
    else
    {
        skwboot_warn("%s No service index ops!!!\n", __func__);
    }
#if 1//debug cp boot for setvalue 1
    if (gbl_count_flag == 0)
    {
        boot_data->dl_done_signal = 0;
        gbl_count_flag = 1;
    }
#endif
    skwboot_log("the dl_count ---dl_bin_counts ===%d\n", tmp_count);
    boot_data->service_ops = SKW_NO_SERVICE;
    if (boot_data->iram_img_data != NULL)
    {
        for (i = 0; i * MODULE_INFO_LEN <= (boot_data->tail_addr - boot_data->head_addr); i++)
        {
            data = (unsigned int *)(boot_data->iram_img_data + boot_data->head_addr + i * MODULE_INFO_LEN);
            dl_data_info.dl_addr = data[0];
            dl_data_info.write_addr = data[2];
            dl_data_info.index = 0x000000FF & EndianConv_32(data[1]);
            dl_data_info.data_size = 0x00FFFFFF & data[1];
            skwboot_log("%s line:%d dl_addr=0x%x, write_addr=0x%x, index=0x%x,data_size=0x%x\n", __func__,
                        __LINE__, dl_data_info.dl_addr, dl_data_info.write_addr, dl_data_info.index, dl_data_info.data_size);
            if (service_index == dl_data_info.index || SKW_BSP == dl_data_info.index || service_index == SKW_ALL)
            {
                total_count += 1;
                boot_data->dl_base_addr = dl_data_info.write_addr;
                boot_data->dl_size = dl_data_info.data_size;
                if (dl_data_info.dl_addr >= boot_data->dram_dl_addr)
                {
                    //iram data dload img
                    boot_data->dl_offset_addr = dl_data_info.dl_addr - boot_data->dram_dl_addr;
                    boot_data->dl_base_img = boot_data->dram_img_data;
                }
                else
                {
                    //dram data dload img
                    boot_data->dl_offset_addr = dl_data_info.dl_addr - boot_data->iram_dl_addr;
                    boot_data->dl_base_img = boot_data->iram_img_data;
                }
                if (tmp_count == total_count)
                {
                    skw_download_signal_ops();
                }
                ret = skw_boot_loader(boot_data);
                if (ret)
                {
                    skwboot_err("%s the load module=%d, fail,ret=%d \n", __func__, dl_data_info.index, ret);
                    break;
                }
            }
            else
            {
                skwboot_err("%s not need to load module_index=%d,ret=%d \n", __func__, dl_data_info.index, ret);
            }
        }

    }
    return ret;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int skw_start_wifi_service(void)
{
    int ret = 0;

    skwboot_log("%s Enter cp_state =%d \n", __func__, cp_exception_sts);
    //mutex_lock(&boot_mutex);
    boot_data->service_ops = SKW_WIFI_START;
    boot_data->dl_module = NONE_BOOT;
    //boot_data->first_dl_flag = 1;
    boot_data->first_boot_flag = 1;
    boot_data->dl_base_img = NULL;
    //download done flag ++
    //skw_download_signal_ops();
    ret = skw_boot_loader(boot_data);
    //mutex_unlock(&boot_mutex);
    if (ret != 0)
    {
        skwboot_err("%s,line:%d boot fail \n", __func__, __LINE__);
        return -1;
    }

    skwboot_log("%s wifi boot sucessfull\n", __func__);
    return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int skw_stop_wifi_service(void)
{
    int ret = 0;
    skwboot_log("%s Enter cp_state =%d \n", __func__, cp_exception_sts);
    mutex_lock(&boot_mutex);
    boot_data->service_ops = SKW_WIFI_STOP;
    boot_data->dl_module = NONE_BOOT;
    boot_data->first_boot_flag = 1;
    boot_data->dl_base_img = NULL;
    boot_data->first_dl_flag = 1;
    //download done flag ++
    //gpio need set high or low power interrupt to cp wakeup
    boot_data->gpio_out = boot_data->chip_gpio;
    if (boot_data->gpio_val)
        boot_data->gpio_val = 0;
    else
        boot_data->gpio_val = 1;
    ret = skw_boot_loader(boot_data);
    mutex_unlock(&boot_mutex);
    if (ret != 0)
    {
        skwboot_warn("dload the img fail \n");
        return -1;
    }
    skwboot_log("seekwave boot stop done:%s\n", __func__);
    return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int skw_start_bt_service(void)
{
    int ret = 0;
    skwboot_log("%s Enter cp_state =%d \n", __func__, cp_exception_sts);
    mutex_lock(&boot_mutex);
    boot_data->service_ops = SKW_BT_START;
    boot_data->first_boot_flag = 1;
    boot_data->dl_base_img = NULL;
    boot_data->dl_module = NONE_BOOT;
    //download done flag ++
    //skw_download_signal_ops();
    ret = skw_boot_loader(boot_data);
    mutex_unlock(&boot_mutex);
    if (ret != 0)
    {
        skwboot_err("%s boot fail \n", __func__);
        return -1;
    }

    skwboot_log("%s line:%d , boot bt sucessfully!\n", __func__, __LINE__);
    return 0;
}

/***************************************************************************
 *Description:
 *Seekwave tech LTD
 *Author:
 *Date:
 *Modify:
 **************************************************************************/
static int skw_stop_bt_service(void)
{
    int ret = 0;
    skwboot_log("%s Enter cp_state =%d \n", __func__, cp_exception_sts);
    mutex_lock(&boot_mutex);
    boot_data->service_ops = SKW_BT_STOP;
    boot_data->first_boot_flag = 1;
    boot_data->dl_base_img = NULL;
    boot_data->first_dl_flag = 1;
    //download done flag ++
    boot_data->dl_module = NONE_BOOT;
    //gpio need set high or low power interrupt to cp wakeup
    boot_data->gpio_out = boot_data->chip_gpio;
    if (boot_data->gpio_val)
        boot_data->gpio_val = 0;
    else
        boot_data->gpio_val = 1;
    ret = skw_boot_loader(boot_data);
    mutex_unlock(&boot_mutex);
    if (ret != 0)
    {
        skwboot_warn("dload the img fail \n");
        return -1;
    }
    skwboot_log("seekwave boot stop done:%s\n", __func__);
    return 0;
}

/****************************************************************
 *Description:double iram dram img first boot cp
 *Func:
 *Calls:
 *Call By:skw_first_boot
 *Input:the file path
 *Output:download data and the data size dl_data image_size
 *Return：0:pass other fail
 *Others:
 *Author：JUNWEI.JIANG
 *Date:2022-02-07
 * **************************************************************/
static int skw_first_boot(struct seekwave_device *boot_data)
{
    int ret = 0;
    //get the img data
#ifdef DEBUG_SKWBOOT_TIME
    ktime_t cur_time, last_time;
    cur_time = ktime_get();
#endif
    //set download the value;
    boot_data->service_ops = SKW_NO_SERVICE;
    boot_data->save_setup_addr = SKW_SDIO_PD_DL_AP2CP_BSP; //160
    boot_data->gpio_out = boot_data->chip_gpio;
    boot_data->gpio_val = 0;
    boot_data->dl_module = 0;
    boot_data->first_dl_flag = 0;
    boot_data->gpio_in  = boot_data->host_gpio;
    boot_data->dma_type_addr = SKW_SDIO_PLD_DMA_TYPE;
    boot_data->slp_disable_addr = SKW_SDIO_CP_SLP_SWITCH;
    boot_data->wifi_start = skw_start_wifi_service;
    boot_data->wifi_stop = skw_stop_wifi_service;
    boot_data->bt_start = skw_start_bt_service;
    boot_data->bt_stop = skw_stop_bt_service;
    boot_data->skw_dloader_module = skw_dloader_module;
    ret = skw_boot_loader(boot_data);
    if (ret < 0)
    {
        skwboot_err("%s firt boot cp fail \n", __func__);
        return -1;
    }
    //download done set the download flag;
    boot_data->first_dl_flag = 1;
    boot_data->first_boot_flag = 1;

    //download done tall cp acount;
    boot_data->dl_done_signal &= 0xFF;
    boot_data->dl_done_signal += 1;
    skwboot_log("%s first boot pass\n", __func__);
#ifdef DEBUG_SKWBOOT_TIME
    last_time = ktime_get();
    skwboot_log("%s,the download time start time %llu and the over time %llu \n",
                __func__, cur_time, last_time);
#endif
    return ret;
}
#ifndef CONFIG_OF
static void seekwave_release(struct device *dev)

{
}
static struct platform_device seekwave_device =
{
    .name = "sv6160lite",
    .dev = {
        .release = seekwave_release,
    }
};
#endif
int seekwave_boot_init(void)
{
    btboot_pdev = NULL;
#ifndef CONFIG_OF
    skwboot_log(" No cfg register %p: %s\n", &seekwave_device, seekwave_device.name);
    platform_device_register(&seekwave_device);
#endif
    platform_driver_register(&seekwave_driver);
    return skw_ucom_init();
}

void seekwave_boot_exit(void)
{
    skw_ucom_exit();
#ifndef CONFIG_OF
    platform_device_unregister(&seekwave_device);
#endif
    platform_driver_unregister(&seekwave_driver);

}

//module_init(seekwave_boot_init);
//module_exit(seekwave_boot_exit);
//MODULE_LICENSE("GPL");
#endif
