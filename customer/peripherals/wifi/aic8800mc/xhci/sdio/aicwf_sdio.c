/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "aicwf_types.h"
#include "aicwf_txrxif.h"
#include "aicwf_sdio.h"
#include "rwnx_defs.h"
#include "rwnx_msg_rx.h"
#include "rwnx_platform.h"
#include "osal_debug.h"
#include "sdio_porting.h"
#ifdef CONFIG_DATA_TXRX
    #include "rwnx_rx.h"
#endif
#include <bf0_hal.h>
#include "drv_sdhci.h"

struct aic_sdio_dev *g_sdio_dev = NULL;

#ifdef CONFIG_PLATFORM_ALLWINNER
    void platform_wifi_power_off(void);
#endif

#ifdef CONFIG_TX_NETIF_FLOWCTRL
    int tx_fc_low_water = AICWF_SDIO_TX_LOW_WATER;
    module_param_named(tx_fc_low_water, tx_fc_low_water, int, 0644);

    int tx_fc_high_water = AICWF_SDIO_TX_HIGH_WATER;
    module_param_named(tx_fc_high_water, tx_fc_high_water, int, 0644);
#endif

int aicwf_sdio_fault_handler(struct aic_sdio_dev *sdiodev)
{
    DBG_SDIO_ERR("%s\n", __func__);
//    host->ops->enable_sdio_irq(host, 0);
    HAL_NVIC_DisableIRQ(SDMMC2_IRQn);
//    aicwf_sdio_release(sdiodev);
    sdiodev->bus_if->state = BUS_DOWN_ST;
    return 0;
}

int aicwf_sdio_readb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 *val)
{
    int ret;
    xhci_sdio_claim_host(sdiodev->func);
    *val = xhci_sdio_readb(sdiodev->func, regaddr, &ret);
    xhci_sdio_release_host(sdiodev->func);
    return ret;
}

int aicwf_sdio_writeb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 val)
{
    int ret;
    xhci_sdio_claim_host(sdiodev->func);
    xhci_sdio_writeb(sdiodev->func, val, regaddr, &ret);
    xhci_sdio_release_host(sdiodev->func);
    return ret;
}

#if 1
#ifdef CONFIG_TX_NETIF_FLOWCTRL
void aicwf_sdio_tx_netif_flowctrl(struct net_device *ndev, bool state)
{
    printk("aicwf_sdio_tx_netif_flowctrl %d\r\n", state);
    if (state)
        netif_tx_stop_all_queues(ndev); //netif_stop_queue(ndev);
    else
        netif_tx_wake_all_queues(ndev); //netif_wake_queue(ndev);
}
#endif

int tx_aggr_counter = 32;
int aicwf_sdio_flow_ctrl_msg(struct aic_sdio_dev *sdiodev)
{
    int ret = -1;
    u8 fc_reg = 0;
    u32 count = 0;

    while (true)
    {
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.flow_ctrl_reg, &fc_reg);
        if (ret)
        {
            aicwf_sdio_fault_handler(sdiodev);
            return -1;
        }

        if (sdiodev->chipid != PRODUCT_ID_AIC8800M40)
        {
            fc_reg = fc_reg & SDIOWIFI_FLOWCTRL_MASK_REG;
        }

        if (fc_reg != 0)
        {
            ret = fc_reg;
            if (ret > tx_aggr_counter)
            {
                ret = tx_aggr_counter;
            }
            return ret;
        }
        else
        {
            if (count >= FLOW_CTRL_RETRY_COUNT)
            {
                ret = -fc_reg;
                DBG_SDIO_ERR("msg fc:%d\n", ret);
                break;
            }
            count++;
            rtos_task_suspend(1);
#if 0
            if (count < 30)
                udelay(200);
            else if (count < 40)
                msleep(2);
            else
                msleep(10);
#endif
        }
    }

    return ret;
}

int aicwf_sdio_flow_ctrl(struct aic_sdio_dev *sdiodev)
{
    int ret = -1;
    u8 fc_reg = 0;
    u32 count = 0;

    while (true)
    {
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.flow_ctrl_reg, &fc_reg);
        if (ret)
        {
            aicwf_sdio_fault_handler(sdiodev);
            return -1;
        }

        if (sdiodev->chipid != PRODUCT_ID_AIC8800M40)
        {
            fc_reg = fc_reg & SDIOWIFI_FLOWCTRL_MASK_REG;
        }

        if (fc_reg > DATA_FLOW_CTRL_THRESH)
        {
            ret = fc_reg;
            if (ret > tx_aggr_counter)
            {
                ret = tx_aggr_counter;
            }
            return ret;
        }
        else
        {
            if (count >= FLOW_CTRL_RETRY_COUNT)
            {
                ret = -fc_reg;
                DBG_SDIO_ERR("data fc:%d\n", ret);
                break;
            }
            count++;
            //printk("C %d\r\n", count);
            rtos_task_suspend(1);
#if 0
            if (count < 30)
                udelay(200);
            else if (count < 40)
                msleep(2);
            else
                msleep(10);
#endif
        }
    }

    return ret;
}
#endif

int aicwf_sdio_send_pkt(struct aic_sdio_dev *sdiodev, u8 *buf, uint count)
{
    int ret = 0;

    xhci_sdio_claim_host(sdiodev->func);
#if CFG_SDIO_BUF_IN_DCACHE
//    rtos_task_critical_enter();
    rtos_dcache_clean(buf, count); // clean dcache before sdio tx data
#endif
    ret = xhci_sdio_writesb(sdiodev->func, sdiodev->sdio_reg.wr_fifo_addr, buf, count);
#if CFG_SDIO_BUF_IN_DCACHE
//    rtos_task_critical_exit();
#endif
    xhci_sdio_release_host(sdiodev->func);

    return ret;
}

int aicwf_sdio_recv_pkt(struct aic_sdio_dev *sdiodev, u32 *buf, u32 size)
{
    int ret;

    if ((!buf) || (!size))
    {
        return -1;//-EINVAL;
    }

    xhci_sdio_claim_host(sdiodev->func);
#if CFG_SDIO_BUF_IN_DCACHE
//    rtos_task_critical_enter();
#endif
    ret = xhci_sdio_readsb(sdiodev->func, buf, sdiodev->sdio_reg.rd_fifo_addr, size);
#if CFG_SDIO_BUF_IN_DCACHE
    rtos_dcache_invalidate(buf, size); // invalidate dcache after sdio rx data
//    rtos_task_critical_exit();
#endif
    xhci_sdio_release_host(sdiodev->func);

    return ret;
}

bool aicwf_another_ptk(uint8_t *data, uint32_t len)
{
    uint16_t aggr_len = 0;
    if (data == NULL || len == 0)
    {
        return false;
    }
    aggr_len = (*data | (*(data + 1) << 8));
    if (aggr_len == 0)
    {
        return false;
    }
    if (aggr_len > len)
    {
        DBG_SDIO_ERR("%s error:%d/%d\n", __func__, aggr_len, len);
        return false;
    }
    return true;
}

#if 0
static int aicwf_sdio_chipmatch(struct aic_sdio_dev *sdio_dev, u16_l vid, u16_l did)
{
    if (vid == SDIO_VENDOR_ID_AIC8800 && did == SDIO_DEVICE_ID_AIC8800)
    {
        sdio_dev->chipid = PRODUCT_ID_AIC8800;
        AICWFDBG(LOGINFO, "%s USE AIC8800\r\n", __func__);
        return 0;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800MC && did == SDIO_DEVICE_ID_AIC8800MC)
    {
        sdio_dev->chipid = PRODUCT_ID_AIC8800MC;
        AICWFDBG(LOGINFO, "%s USE AIC8800MC\r\n", __func__);
        return 0;
    }
    else if (vid == SDIO_VENDOR_ID_AIC8800M40 && did == SDIO_DEVICE_ID_AIC8800M40)
    {
        sdio_dev->chipid = PRODUCT_ID_AIC8800M40;
        AICWFDBG(LOGINFO, "%s USE AIC8800M40\r\n", __func__);
        return 0;
    }
    else
    {
        return -1;
    }
}
#endif

#if CONFIG_NOT_DEFINED
static int aicwf_sdio_suspend(struct device *dev)
{
    int ret = 0;
    struct aicwf_bus *bus_if = dev_get_drvdata(dev);
    struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
    mmc_pm_flag_t sdio_flags;

    DBG_SDIO_INF("%s\n", __func__);
#ifdef CONFIG_VNET_MODE
    if (vnet_dev)
    {
        netif_device_detach(vnet_dev);
    }
#endif
    sdio_flags = sdio_get_host_pm_caps(sdiodev->func);
    if (!(sdio_flags & MMC_PM_KEEP_POWER))
    {
        return -EINVAL;
    }
    ret = sdio_set_host_pm_flags(sdiodev->func, MMC_PM_KEEP_POWER);
    if (ret)
    {
        return ret;
    }

    while (sdiodev->state == SDIO_ACTIVE_ST)
    {
        if (down_interruptible(&sdiodev->tx_priv->txctl_sema))
        {
            continue;
        }
#if defined(CONFIG_SDIO_PWRCTRL)
        aicwf_sdio_pwr_stctl(sdiodev, SDIO_SLEEP_ST);
#endif
        up(&sdiodev->tx_priv->txctl_sema);
        break;
    }

    return 0;
}

static int aicwf_sdio_resume(struct device *dev)
{
    struct aicwf_bus *bus_if = dev_get_drvdata(dev);
    struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
    DBG_SDIO_INF("%s\n", __func__);
#ifdef CONFIG_VNET_MODE
    if (vnet_dev)
    {
        netif_device_attach(vnet_dev);
    }
#endif

#if defined(CONFIG_SDIO_PWRCTRL)
    aicwf_sdio_pwr_stctl(sdiodev, SDIO_ACTIVE_ST);
#endif
    return 0;
}
#endif

#if 0
static const struct sdio_device_id aicwf_sdmmc_ids[] =
{
    {SDIO_DEVICE(SDIO_VENDOR_ID_AIC8800, SDIO_DEVICE_ID_AIC8800)},
    {SDIO_DEVICE(SDIO_VENDOR_ID_AIC8800MC, SDIO_DEVICE_ID_AIC8800MC)},
    {SDIO_DEVICE(SDIO_VENDOR_ID_AIC8800M40, SDIO_DEVICE_ID_AIC8800M40)},
    { },
};

MODULE_DEVICE_TABLE(sdio, aicwf_sdmmc_ids);

static const struct dev_pm_ops aicwf_sdio_pm_ops =
{
    SET_SYSTEM_SLEEP_PM_OPS(aicwf_sdio_suspend, aicwf_sdio_resume)
};

static struct sdio_driver aicwf_sdio_driver =
{
    .probe = aicwf_sdio_probe,
    .remove = aicwf_sdio_remove,
    .name = AICWF_SDIO_NAME,
    .id_table = aicwf_sdmmc_ids,
    .drv = {
        .pm = &aicwf_sdio_pm_ops,
    },
};
#endif

#ifdef CONFIG_NANOPI_M4
    extern int mmc_rescan_try_freq(struct mmc_host *host, unsigned freq);
    extern unsigned  aic_max_freqs;
    extern struct mmc_host *aic_host_drv;
    extern int __mmc_claim_host(struct mmc_host *host, atomic_t *abort);
    extern void mmc_release_host(struct mmc_host *host);
#endif
#ifdef CONFIG_PLATFORM_ALLWINNER
extern void sunxi_mmc_rescan_card(unsigned ids);
extern void sunxi_wlan_set_power(int on);
extern int sunxi_wlan_get_bus_index(void);

int platform_wifi_power_on(void)
{
    int ret = 0;
    int wlan_bus_index = sunxi_wlan_get_bus_index();
    if (wlan_bus_index < 0)
        return wlan_bus_index;

    sunxi_wlan_set_power(1);
    mdelay(1000);
    sunxi_mmc_rescan_card(wlan_bus_index);

    printk("platform_wifi_power_on");

    return ret;
}

void platform_wifi_power_off(void)
{
    int wlan_bus_index = sunxi_wlan_get_bus_index();
    if (wlan_bus_index < 0)
    {
        printk("no wlan_bus_index\n");
        return ;
    }
    printk("power_off\n");
    sunxi_wlan_set_power(0);
    mdelay(100);
    //sunxi_mmc_rescan_card(wlan_bus_index);

    printk("platform_wifi_power_off");
}
#endif

#if defined(CONFIG_SDIO_PWRCTRL)
void aicwf_sdio_register(void)
{
#ifdef CONFIG_PLATFORM_NANOPI
    extern_wifi_set_enable(0);
    mdelay(200);
    extern_wifi_set_enable(1);
    mdelay(200);
    sdio_reinit();
#endif /*CONFIG_PLATFORM_NANOPI*/

#ifdef CONFIG_INGENIC_T31
    int ret = jzmmc_manual_detect(1, 1);
    if (ret)
    {
        printk("manual detect err %d\r\n", ret);
    }
#endif /* CONFIG_INGENIC_T31 */

#ifdef CONFIG_NANOPI_M4
    if (aic_host_drv->card == NULL)
    {
        __mmc_claim_host(aic_host_drv, NULL);
        printk("aic: >>>mmc_rescan_try_freq\n");
        mmc_rescan_try_freq(aic_host_drv, aic_max_freqs);
        mmc_release_host(aic_host_drv);
    }
#endif
#ifdef CONFIG_PLATFORM_ALLWINNER
    platform_wifi_power_on();
#endif
    if (sdio_register_driver(&aicwf_sdio_driver))
    {
        printk("aic>: sdio_register_driver fail\n");
    }
    else
    {
        //may add mmc_rescan here
    }
}

void aicwf_sdio_exit(void)
{
    if (g_rwnx_plat && g_rwnx_plat->enabled)
    {
        rwnx_platform_deinit();
    }

    sdio_unregister_driver(&aicwf_sdio_driver);

#ifdef CONFIG_PLATFORM_NANOPI
    extern_wifi_set_enable(0);
#endif /*CONFIG_PLATFORM_NANOPI*/
    rtos_free(g_rwnx_plat);
    g_rwnx_plat = NULL;
}

int aicwf_sdio_wakeup(struct aic_sdio_dev *sdiodev)
{
    int ret = 0;
    int read_retry;
    int write_retry = 20;
    int wakeup_reg_val;

    if (sdiodev->chipid != PRODUCT_ID_AIC8800M40)
    {
        wakeup_reg_val = 1;
    }
    else
    {
        wakeup_reg_val = 0x11;
    }

    if (sdiodev->state == SDIO_SLEEP_ST)
    {
        down(&sdiodev->pwrctl_wakeup_sema);
        if (sdiodev->state == SDIO_ACTIVE_ST)
        {
            up(&sdiodev->pwrctl_wakeup_sema);
            return 0;
        }

        DBG_SDIO_INF("wakeup\n");
        while (write_retry)
        {
            ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.wakeup_reg, wakeup_reg_val);
            if (ret)
            {
                txrx_err("sdio wakeup fail\n");
                ret = -1;
            }
            else
            {
                read_retry = 10;
                while (read_retry)
                {
                    u8 val;
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.sleep_reg, &val);
                    if (ret == 0 && val & 0x10)
                    {
                        break;
                    }
                    read_retry--;
                    udelay(200);
                }
                if (read_retry != 0)
                    break;
            }

            DBG_SDIO_ERR("write retry:  %d \n", write_retry);
            write_retry--;
            udelay(100);
        }

        sdiodev->state = SDIO_ACTIVE_ST;
        aicwf_sdio_pwrctl_timer(sdiodev, sdiodev->active_duration);
        up(&sdiodev->pwrctl_wakeup_sema);
    }
    return ret;
}

int aicwf_sdio_sleep_allow(struct aic_sdio_dev *sdiodev)
{
    int ret = 0;
    struct aicwf_bus *bus_if = sdiodev->bus_if;

    if (rtos_atomic_read(&sdiodev->tx_priv->tx_pktcnt) > 0)
    {
        DBG_SDIO_INF("Cancel sdio sleep setting\n");
        return 0;
    }

    if (bus_if->state == BUS_DOWN_ST)
    {
        ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.sleep_reg, 0x10);
        if (ret)
        {
            DBG_SDIO_ERR("Write sleep fail!\n");
        }
        aicwf_sdio_pwrctl_timer(sdiodev, 0);
        return ret;
    }

    // sdio_info("sleep: %d, %d\n", sdiodev->state, scanning);
    if (sdiodev->state == SDIO_ACTIVE_ST)
    {
        down(&sdiodev->pwrctl_wakeup_sema);
        DBG_SDIO_INF("sleep\n");
        ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.sleep_reg, 0x10);
        if (ret)
            DBG_SDIO_ERR("Write sleep fail!\n");
        sdiodev->state = SDIO_SLEEP_ST;
        aicwf_sdio_pwrctl_timer(sdiodev, 0);
        up(&sdiodev->pwrctl_wakeup_sema);
    }
    else
    {
        aicwf_sdio_pwrctl_timer(sdiodev, sdiodev->active_duration);
    }

    return ret;
}

int aicwf_sdio_pwr_stctl(struct aic_sdio_dev *sdiodev, uint target)
{
    int ret = 0;

    if (sdiodev->bus_if->state == BUS_DOWN_ST)
    {
        return -1;
    }
    //printk("%s %d\r\n",    __func__, target);

    down(&sdiodev->pwrctl_wakeup_sema);
    if (sdiodev->state == target)
    {
        if (target == SDIO_ACTIVE_ST)
        {
            aicwf_sdio_pwrctl_timer(sdiodev, sdiodev->active_duration);
        }
        up(&sdiodev->pwrctl_wakeup_sema);
        return ret;
    }
    up(&sdiodev->pwrctl_wakeup_sema);

    switch (target)
    {
    case SDIO_ACTIVE_ST:
        aicwf_sdio_wakeup(sdiodev);
        break;
    case SDIO_SLEEP_ST:
        aicwf_sdio_sleep_allow(sdiodev);
        break;
    }

    return ret;
}
#endif

#ifdef CONFIG_DATA_TXRX
int aicwf_sdio_txpkt(struct aic_sdio_dev *sdiodev, uint8_t *pkt, uint16_t pkt_len)
{
    int ret = 0;
    uint32_t len = 0;
    struct aicwf_bus *bus_if = sdiodev->bus_if;

    if (bus_if->state == BUS_DOWN_ST)
    {
        DBG_SDIO_INF("tx bus is down!\r\n");
        return -1;//-EINVAL;
    }

    //len = pkt_len;
    //len = (len + SDIOWIFI_FUNC_BLOCKSIZE - 1) / SDIOWIFI_FUNC_BLOCKSIZE * SDIOWIFI_FUNC_BLOCKSIZE;
    len = (pkt_len + SDIOWIFI_FUNC_BLOCKSIZE - 1) & ~(SDIOWIFI_FUNC_BLOCKSIZE - 1);

    ret = aicwf_sdio_send_pkt(sdiodev, pkt, len);
    if (ret)
    {
        DBG_SDIO_ERR("aicwf_sdio_send_pkt fail%d\r\n", ret);
        aicwf_sdio_fault_handler(sdiodev);
    }
    return ret;
}
#endif

static int aicwf_sdio_intr_get_len_bytemode(struct aic_sdio_dev *sdiodev, u8 *byte_len)
{
    int ret = 0;

    if (!byte_len)
        return -1;//-EBADE;

    if (sdiodev->bus_if->state == BUS_DOWN_ST)
    {
        *byte_len = 0;
    }
    else
    {
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, byte_len);
        sdiodev->rx_priv->data_len = (*byte_len) * 4;
    }

    return ret;
}

static void aicwf_sdio_bus_stop(void *data)
{
    struct aicwf_bus *bus_if = (struct aicwf_bus *)data;
    struct aic_sdio_dev *sdiodev = (struct aic_sdio_dev *)bus_if->bus_priv.xhci;
    int ret;

#if defined(CONFIG_SDIO_PWRCTRL)
    aicwf_sdio_pwrctl_timer(sdiodev, 0);
#endif
    DBG_SDIO_INF("%s\n", __func__);
#if defined(CONFIG_SDIO_PWRCTRL)
    if (sdiodev->pwrctl_tsk)
    {
        complete(&sdiodev->pwrctrl_trgg);
        kthread_stop(sdiodev->pwrctl_tsk);
        sdiodev->pwrctl_tsk = NULL;
    }
#endif

#if 0
    DBG_SDIO_INF("%s:pwrctl stopped\n", __func__);

    bus_if->state = BUS_DOWN_ST;
    ret = down_interruptible(&sdiodev->tx_priv->txctl_sema);
    if (ret)
        DBG_SDIO_ERR("down txctl_sema fail\n");

#if defined(CONFIG_SDIO_PWRCTRL)
    aicwf_sdio_pwr_stctl(sdiodev, SDIO_SLEEP_ST);
#endif
    if (!ret)
        up(&sdiodev->tx_priv->txctl_sema);
    aicwf_frame_queue_flush(&sdiodev->tx_priv->txq);
#endif

    bus_if->state = BUS_DOWN_ST;

#ifdef CONFIG_DATA_TXRX
    aicwf_frame_queue_deinit(&sdiodev->tx_priv->txq);
#endif
    aicwf_tx_deinit(sdiodev->tx_priv);
    aicwf_rx_deinit(sdiodev->rx_priv);
    aicwf_rwnx_sdio_platform_deinit(sdiodev);

//    if (g_rwnx_plat->enabled)
//        rwnx_platform_deinit(sdiodev->rwnx_hw);
    DBG_SDIO_INF("exit %s\n", __func__);
}

#if 0
struct sk_buff *aicwf_sdio_readframes(struct aic_sdio_dev *sdiodev)
{
    int ret = 0;
    u32 size = 0;
    struct sk_buff *skb = NULL;
    struct aicwf_bus *bus_if = dev_get_drvdata(sdiodev->dev);

    if (bus_if->state == BUS_DOWN_ST)
    {
        DBG_SDIO_INF("bus down\n");
        return NULL;
    }
    DBG_SDIO_INF("enter %s\n", __func__);

    size = sdiodev->rx_priv->data_len;
    skb =  __dev_alloc_skb(size, GFP_KERNEL);
    if (!skb)
    {
        return NULL;
    }

    ret = aicwf_sdio_recv_pkt(sdiodev, skb, size);
    if (ret)
    {
        DBG_SDIO_ERR("sdio recv pkt fail, ret=%d\n", ret);
        dev_kfree_skb(skb);
        skb = NULL;
    }

    return skb;
}
#endif

static int aicwf_sdio_tx_msg(struct aic_sdio_dev *sdiodev, uint8_t *buf, uint16_t count)
{
    int err = 0;
    u16 len;
    u8 *payload = buf;
    u16 payload_len = count;
    u8 adjust_str[4] = {0, 0, 0, 0};
    int adjust_len = 0;
    int buffer_cnt = 0;
    u8 retry = 0;

    len = payload_len;
    if ((len % TX_ALIGNMENT) != 0)
    {
        adjust_len = (len + (TX_ALIGNMENT - 1)) & ~(TX_ALIGNMENT - 1); //roundup(len, TX_ALIGNMENT);
        rtos_memcpy(payload + payload_len, adjust_str, (adjust_len - len));
        payload_len += (adjust_len - len);
    }
    len = payload_len;

    //link tail is necessary
    if ((len % SDIOWIFI_FUNC_BLOCKSIZE) != 0)
    {
        rtos_memset(payload + payload_len, 0, TAIL_LEN);
        payload_len += TAIL_LEN;
        len = (payload_len / SDIOWIFI_FUNC_BLOCKSIZE + 1) * SDIOWIFI_FUNC_BLOCKSIZE;
    }
    else
    {
        len = payload_len;
    }
    buffer_cnt = aicwf_sdio_flow_ctrl_msg(sdiodev);
    while ((buffer_cnt <= 0 || (buffer_cnt > 0 && len > (buffer_cnt * BUFFER_SIZE))) && retry < 10)
    {
        retry++;
        buffer_cnt = aicwf_sdio_flow_ctrl_msg(sdiodev);
    }

    rtos_mutex_lock(sdiodev->tx_priv->cmd_txsema, -1);
    do
    {
        if (buffer_cnt > 0 && len <= (buffer_cnt * BUFFER_SIZE))
        {
            err = aicwf_sdio_send_pkt(sdiodev, payload, len);
            if (err)
            {
                DBG_SDIO_ERR("aicwf_sdio_send_pkt fail%d\n", err);
                aicwf_sdio_fault_handler(sdiodev);
                break;
            }
        }
        else
        {
            DBG_SDIO_ERR("tx msg fc retry fail\n");
            err = -1;
            break;
        }

        sdiodev->tx_priv->cmd_txstate = false;
        if (!err)
            sdiodev->tx_priv->cmd_tx_succ = true;
        else
            sdiodev->tx_priv->cmd_tx_succ = false;
    }
    while (0);

    rtos_mutex_unlock(sdiodev->tx_priv->cmd_txsema);

    return err;
}

static void aicwf_sdio_tx_process(struct aic_sdio_dev *sdiodev)
{
    int err = 0;
    if (sdiodev->bus_if->state == BUS_DOWN_ST)
    {
        DBG_SDIO_ERR("Bus is down\r\n");
        return;
    }

#if defined(CONFIG_SDIO_PWRCTRL)
    aicwf_sdio_pwr_stctl(sdiodev, SDIO_ACTIVE_ST);
#endif

    //config
    DBG_SDIO_VRB("send cmd\r\n");
    if (sdiodev->tx_priv->cmd_txstate)
    {
//        if (down_interruptible(&sdiodev->tx_priv->txctl_sema)) {
//            txrx_err("txctl down bus->txctl_sema fail\n");
//            return;
//        }
        if (sdiodev->state != SDIO_ACTIVE_ST)
        {
            DBG_SDIO_ERR("state err\r\n");
            //up(&sdiodev->tx_priv->txctl_sema);
            //DBG_SDIO_ERR("txctl up bus->txctl_sema fail\r\n");
            return;
        }

        err = aicwf_sdio_tx_msg(sdiodev, sdiodev->tx_priv->cmd_buf, sdiodev->tx_priv->cmd_len);
        //up(&sdiodev->tx_priv->txctl_sema);
        //if (waitqueue_active(&sdiodev->tx_priv->cmd_txdone_wait))
        //    wake_up(&sdiodev->tx_priv->cmd_txdone_wait);
    }

#ifdef CONFIG_DATA_TXRX
    //data
    DBG_SDIO_VRB("send data\n");
    //if (down_interruptible(&sdiodev->tx_priv->txctl_sema)) {
    //    DBG_SDIO_ERR("txdata down bus->txctl_sema\n");
    //    return;
    //}

    if (sdiodev->state != SDIO_ACTIVE_ST)
    {
        DBG_SDIO_ERR("sdio state err\n");
        //up(&sdiodev->tx_priv->txctl_sema);
        return;
    }

    if (!aicwf_is_framequeue_empty(&sdiodev->tx_priv->txq))
    {
        sdiodev->tx_priv->fw_avail_bufcnt = aicwf_sdio_flow_ctrl(sdiodev);
    }

    while (!aicwf_is_framequeue_empty(&sdiodev->tx_priv->txq))
    {
        if (sdiodev->bus_if->state == BUS_DOWN_ST)
        {
            break;
        }
        if (sdiodev->tx_priv->fw_avail_bufcnt <= DATA_FLOW_CTRL_THRESH)
        {
            if (sdiodev->tx_priv->cmd_txstate)
                break;
            sdiodev->tx_priv->fw_avail_bufcnt = aicwf_sdio_flow_ctrl(sdiodev);
        }
        else
        {
            if (sdiodev->tx_priv->cmd_txstate)
            {
                aicwf_sdio_send(sdiodev, 1);
                break;
            }
            else
            {
                aicwf_sdio_send(sdiodev, 0);
            }
        }
    }
    //up(&sdiodev->tx_priv->txctl_sema);
#endif
}

#ifdef CONFIG_DATA_TXRX
static int aicwf_sdio_bus_txdata(void *bus_data, u8 *pkt, uint pktlen)
{
    int ret = 0;
    do
    {
        struct aicwf_bus *bus_if = (struct aicwf_bus *)bus_data;
        struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
        DBG_SDIO_VRB("aicwf_sdio_bus_txdata %d\r\n", pktlen);
        struct aicwf_buf_node_s *node;
        uint8_t *buf_tx;
        if ((pkt == NULL) || (pktlen == 0))
        {
            DBG_SDIO_ERR("invalid pkt=%p,len=%d\r\n", pkt, pktlen);
            ret = -1;
            break;
        }
        node = aicwf_bustx_buf_alloc(pktlen);
        if ((node == NULL) || (node->buf == NULL))
        {
            DBG_SDIO_ERR("tx node/buf alloc fail(len=%d),node=%p,buf=%p\r\n", pktlen, node, node->buf);
            if (node)
            {
                aicwf_bustx_buf_free(node);
            }
            ret = -2;
            break;
        }
        buf_tx = node->buf;
        rtos_memcpy(buf_tx, pkt, pktlen);
        aicwf_frame_enqueue(&sdiodev->tx_priv->txq, node);
        rtos_atomic_inc(&sdiodev->tx_priv->tx_pktcnt);
        rtos_semaphore_signal(bus_if->bustx_trgg, false); // to be confirmed
    }
    while (0);
    return ret;
}
#endif

static int aicwf_sdio_bus_txmsg(void *bus_data, u8 *msg, uint msglen)
{
    struct aicwf_bus *bus_if = (struct aicwf_bus *)bus_data;
    struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;

    DBG_SDIO_VRB("aicwf_sdio_bus_txmsg %d\r\n", msglen);

#ifdef CONFIG_NOT_DEFINED
    aicwf_sdio_tx_msg(sdiodev, msg, (uint16_t)msglen);
#else
    rtos_mutex_lock(sdiodev->tx_priv->cmd_txsema, -1);
    sdiodev->tx_priv->cmd_txstate = true;
    sdiodev->tx_priv->cmd_tx_succ = false;
    sdiodev->tx_priv->cmd_buf = msg;
    sdiodev->tx_priv->cmd_len = msglen;
    rtos_mutex_unlock(sdiodev->tx_priv->cmd_txsema);

    if (bus_if->state != BUS_UP_ST)
    {
        DBG_SDIO_ERR("bus has stop\n");
        return -1;
    }

    rtos_semaphore_signal(bus_if->bustx_trgg, 0);
#if 0
    if (sdiodev->tx_priv->cmd_txstate)
    {
        int timeout = msecs_to_jiffies(CMD_TX_TIMEOUT);
        ret = wait_event_interruptible_timeout(sdiodev->tx_priv->cmd_txdone_wait, \
                                               !(sdiodev->tx_priv->cmd_txstate), timeout);
    }

    if (!sdiodev->tx_priv->cmd_txstate && sdiodev->tx_priv->cmd_tx_succ)
    {
        ret = 0;
    }
    else
    {
        DBG_SDIO_ERR("send faild:%d, %d,%x\n", sdiodev->tx_priv->cmd_txstate, sdiodev->tx_priv->cmd_tx_succ, ret);
        ret = -EIO;
    }

    return ret;
#endif
#endif
    return 0;
}

#ifdef CONFIG_DATA_TXRX
int aicwf_sdio_send(struct aic_sdio_dev *sdiodev, u8 txnow)
{
    struct aicwf_tx_priv *tx_priv = sdiodev->tx_priv;
    u32 aggr_len = 0;
#ifdef CONFIG_TX_NETIF_FLOWCTRL
    unsigned long flags;
#endif

    aggr_len = (tx_priv->tail - tx_priv->head);

    if (((tx_priv->aggr_count == 0) && (aggr_len != 0))
            || ((tx_priv->aggr_count != 0) && (aggr_len == 0)))
    {
        if (aggr_len > 0)
            aicwf_sdio_aggrbuf_reset(tx_priv);
        goto done;
    }
    //printk("aicwf_sdio_send %d \r\n", tx_priv->fw_avail_bufcnt);

    if (tx_priv->aggr_count == (tx_priv->fw_avail_bufcnt - 2))
    {
        if (tx_priv->aggr_count > 0)
        {
            tx_priv->fw_avail_bufcnt -= tx_priv->aggr_count;
            aicwf_sdio_aggr_send(sdiodev); //send and check the next pkt;
        }
    }
    else
    {
        struct aicwf_buf_node_s *node;
        int ret_agg;
        //spin_lock_bh(&sdiodev->tx_priv->txqlock);
        node = aicwf_frame_dequeue(&tx_priv->txq);
        if (node == NULL)
        {
            DBG_SDIO_ERR("txq no pkt\n");
            //spin_unlock_bh(&sdiodev->tx_priv->txqlock);
            goto done;
        }
        rtos_atomic_dec(&tx_priv->tx_pktcnt);
        //printk("tc2:%d\n", atomic_read(&sdiodev->tx_priv->tx_pktcnt));
        //spin_unlock_bh(&sdiodev->tx_priv->txqlock);

        ret_agg = aicwf_sdio_aggr(sdiodev, node->buf, node->buf_len);
        aicwf_bustx_buf_free(node);
        if (ret_agg)
        {
            aicwf_sdio_aggrbuf_reset(tx_priv);
            DBG_SDIO_ERR("add aggr pkts failed!\n");
            goto done;
        }

        //when aggr finish or there is cmd to send, just send this aggr pkt to fw
        if ((rtos_atomic_read(&tx_priv->tx_pktcnt) == 0) || txnow || (tx_priv->aggr_count == (tx_priv->fw_avail_bufcnt - 2)))
        {
            tx_priv->fw_avail_bufcnt -= tx_priv->aggr_count;
            aicwf_sdio_aggr_send(sdiodev);
        }
        else
            goto done;
    }

done:
    return 0;
}

uint8_t crc8_ponl_107(uint8_t *p_buffer, uint16_t cal_size)
{
    uint8_t i;
    uint8_t crc = 0;
    if (cal_size == 0)
    {
        return crc;
    }
    while (cal_size--)
    {
        for (i = 0x80; i > 0; i /= 2)
        {
            if (crc & 0x80)
            {
                crc *= 2;
                crc ^= 0x07; //polynomial X8 + X2 + X + 1,(0x107)
            }
            else
            {
                crc *= 2;
            }
            if ((*p_buffer) & i)
            {
                crc ^= 0x07;
            }
        }
        p_buffer++;
    }

    return crc;
}

int aicwf_sdio_aggr(struct aic_sdio_dev *sdiodev, uint8_t *pkt, uint16_t len)
{
    struct aicwf_tx_priv *tx_priv = sdiodev->tx_priv;
    u8 *start_ptr = tx_priv->tail;
    u8 sdio_header[4]; // used by sdio hw
    u8 adjust_str[4] = {0, 0, 0, 0};
    uint16_t align_len = 0;
    uint16_t aggr_len = 0;
    host_data_t payload_hdr = {0};

    // fill sdio_header at last
    tx_priv->tail += sizeof(sdio_header);
    //payload header
    payload_hdr.plen = len;
    rtos_memcpy(tx_priv->tail, (u8 *)(&payload_hdr), HOST_DATA_HDR_LEN);
    tx_priv->tail += HOST_DATA_HDR_LEN;
    //payload
    rtos_memcpy(tx_priv->tail, (u8 *)(pkt), len);
    tx_priv->tail += len;
    // word align
    align_len = (len + TX_ALIGNMENT - 1) & ~(TX_ALIGNMENT - 1); //roundup(len, TX_ALIGNMENT);
    if (align_len > len)
    {
        uint16_t tail_len = align_len - len;
        rtos_memcpy(tx_priv->tail, adjust_str, tail_len);
        tx_priv->tail += tail_len;
    }
    aggr_len = HOST_DATA_HDR_LEN + align_len; //not include sdio_header
    sdio_header[0] = (aggr_len & 0xff);
    sdio_header[1] = ((aggr_len >> 8) & 0xff);
    sdio_header[2] = SDIO_TYPE_DATA_AHB2SDIO; //data
    if (sdiodev->chipid <= PRODUCT_ID_AIC8800DW)
    {
        sdio_header[3] = 0; //reserved
    }
    else
    {
        // crc: contains all pkt execpt sdio_header.
        sdio_header[3] = crc8_ponl_107(&sdio_header[0], 3); // crc8
    }
    // fill sdio_header
    rtos_memcpy(start_ptr, (u8 *)&sdio_header, sizeof(sdio_header));

    tx_priv->aggr_count++;

    return 0;
}

void aicwf_sdio_aggr_send(struct aic_sdio_dev *sdiodev)
{
    struct aicwf_tx_priv *tx_priv = sdiodev->tx_priv;
    uint8_t *tx_buf = tx_priv->aggr_buf;
    int ret = 0;
    int curr_len = 0;

    //link tail is necessary
    curr_len = tx_priv->tail - tx_priv->head;
    if ((curr_len % TXPKT_BLOCKSIZE) != 0)
    {
        rtos_memset(tx_priv->tail, 0, TAIL_LEN);
        tx_priv->tail += TAIL_LEN;
    }

    tx_priv->len = tx_priv->tail - tx_priv->head;
    ret = aicwf_sdio_txpkt(sdiodev, tx_buf, tx_priv->len);
    if (ret < 0)
    {
        DBG_SDIO_ERR("fail to send aggr pkt!\n");
    }

    aicwf_sdio_aggrbuf_reset(tx_priv);
}

void aicwf_sdio_aggrbuf_reset(struct aicwf_tx_priv *tx_priv)
{
    tx_priv->tail = tx_priv->head;
    tx_priv->len = 0;
    tx_priv->aggr_count = 0;
}
#endif

static int aicwf_sdio_bus_start(void *data)
{
    int ret = 0;
    struct aicwf_bus *bus_if = (struct aicwf_bus *)data;
    struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;

#if 1
    xhci_sdio_claim_host(sdiodev->func);
    xhci_sdio_claim_irq(sdiodev->func, aicwf_sdio_hal_irqhandler);
#else
    //since we have func2 we don't register irq handler
    sdio_claim_irq(sdiodev->func, NULL);
    sdiodev->func->irq_handler = (sdio_irq_handler_t *)aicwf_sdio_hal_irqhandler;
#endif

//    if (sdiodev->chipid == PRODUCT_ID_AIC8800M40) {
//        sdio_f0_writeb(sdiodev->func, 0x07, 0x04, &ret);
//        if (ret) {
//            DBG_SDIO_ERR("set func0 int en fail %d\n", ret);
//        }
//    }
    xhci_sdio_release_host(sdiodev->func);

    //enable sdio interrupt
    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.intr_config_reg, 0x07);
    if (ret != 0)
    {
        DBG_SDIO_ERR("intr register failed:%d\n", ret);
    }

    bus_if->state = BUS_UP_ST;
    return ret;
}

int sdio_bustx_thread(void *data)
{
    struct aicwf_bus *bus_if = (struct aicwf_bus *)data;
    struct aic_sdio_dev *sdiodev = bus_if->bus_priv.sdio;

    while (1)
    {
        int ret = rtos_semaphore_wait(bus_if->bustx_trgg, -1);
        if (ret < 0)
        {
            DBG_SDIO_ERR("wait bus txq trigg fail: ret=%d\n", ret);
            continue;
        }
        if (sdiodev->bus_if->state == BUS_DOWN_ST)
        {
            break;
        }
        while ((sdiodev->tx_priv->cmd_txstate == true)
#ifdef CONFIG_DATA_TXRX
                || (rtos_atomic_read(&sdiodev->tx_priv->tx_pktcnt) > 0)
#else
                || 0
#endif
              )
        {
            xhci_sdio_claim_host(sdiodev->func);
            aicwf_sdio_tx_process(sdiodev);
            xhci_sdio_release_host(sdiodev->func);
            if (sdiodev->bus_if->state == BUS_DOWN_ST)
                break;
        }
    }

    return 0;
}

int sdio_busrx_thread(void *data)
{
    //struct aicwf_rx_priv *rx_priv = (struct aicwf_rx_priv *)data;
    //struct aicwf_bus *bus_if = rx_priv->sdiodev->bus_if;
    struct aicwf_bus *bus_if = (struct aicwf_bus *)data;
    struct aic_sdio_dev *sdiodev = (struct aic_sdio_dev *)bus_if->bus_priv.sdio;

    while (1)
    {
        struct aicwf_buf_node_s *node = NULL;
        int ret = rtos_semaphore_wait(bus_if->busrx_trgg, -1); // wait forever
        if (ret < 0)
        {
            DBG_SDIO_ERR("wait bus rxq trigg fail: ret=%d\n", ret);
            continue;
        }
        node = aicwf_frame_dequeue(&sdiodev->rx_priv->rxq);
        if (node)
        {
            uint8_t *buf_raw;
            uint8_t *data = node->buf;
            uint32_t len = node->buf_len;
            if (data == NULL)
            {
                DBG_SDIO_ERR("err: rx data null: node=%p\n", node);
            }
            else
            {
                while (aicwf_another_ptk(data, len))
                {
                    uint16_t pkt_len = (*data | (*(data + 1) << 8));
                    if (pkt_len > len || pkt_len > 4096)
                    {
                        DBG_SDIO_ERR("pkt_len too large: %d, len: %d\n", pkt_len, len);
                        break;
                    }
                    uint16_t aggr_len;
                    if ((data[2] & SDIO_TYPE_CFG) != SDIO_TYPE_CFG)  // type : data
                    {
                        DBG_SDIO_VRB("[task] rx data: %d\n", pkt_len);
#ifdef CONFIG_DATA_TXRX
#ifdef AICWF_RX_REORDER
                        rwnx_rxdataind_aicwf(sdiodev->rwnx_hw, data + 4, sdiodev->rx_priv);
#else
                        rwnx_rxdata_forward(sdiodev->rwnx_hw, data + 4, sdiodev->rx_priv);
#endif
                        aggr_len = 4 + pkt_len;
                        aggr_len = (aggr_len + (RX_ALIGNMENT - 1)) & ~(RX_ALIGNMENT - 1);
#endif
                    }
                    else
                    {
                        uint8_t *msg = data;
                        uint8_t type = *(msg + 2) & 0x7f;
                        if (type == SDIO_TYPE_CFG_CMD_RSP)
                        {
                            struct rwnx_hw *rwnx_hw = sdiodev->rwnx_hw;
                            rwnx_rx_handle_msg(rwnx_hw, (struct e2a_msg *)(msg + 4));
                        }
                        else if (type == SDIO_TYPE_CFG_DATA_CFM)
                        {
                            DBG_SDIO_INF("[task] rx data cfm\n");
                        }
                        else
                        {
                            DBG_SDIO_ERR("unsupported type:%x\n", type);
                        }
                        aggr_len = (pkt_len + 4 + (RX_ALIGNMENT - 1)) & ~(RX_ALIGNMENT - 1);
                    }
                    data += aggr_len;
                    len -= aggr_len;
                    if ((uint32_t)(data - node->buf) > node->buf_len || (int32_t)len < 0)
                    {
                        DBG_SDIO_ERR("%s len error:%d/%d\n", __func__, (uint32_t)(data - node->buf), node->buf_len);
                        break;
                    }
                }
                aicwf_busrx_buf_free(node);
            }
        }
        else
        {
            DBG_SDIO_ERR("rxq triggered but queue is empty\n");
        }
    }

    return 0;
}

#if defined(CONFIG_SDIO_PWRCTRL)
static int aicwf_sdio_pwrctl_thread(void *data)
{
    struct aic_sdio_dev *sdiodev = (struct aic_sdio_dev *) data;

    while (1)
    {
        if (kthread_should_stop())
        {
            DBG_SDIO_ERR("sdio pwrctl thread stop\n");
            break;
        }
        if (!wait_for_completion_interruptible(&sdiodev->pwrctrl_trgg))
        {
            if (sdiodev->bus_if->state == BUS_DOWN_ST)
                break;
            if (sdiodev->state == SDIO_ACTIVE_ST)
            {
                if ((int)(atomic_read(&sdiodev->tx_priv->tx_pktcnt) <= 0) && (sdiodev->tx_priv->cmd_txstate == false) && \
                        atomic_read(&sdiodev->rx_priv->rx_cnt) == 0)
                {
                    // aicwf_sdio_pwr_stctl(sdiodev, SDIO_SLEEP_ST);
                    // for save time
                    aicwf_sdio_sleep_allow(sdiodev);
                }
                else
                    aicwf_sdio_pwrctl_timer(sdiodev, sdiodev->active_duration);
            }
        }
        else
        {
            continue;
        }
    }

    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    static void aicwf_sdio_bus_pwrctl(ulong data)
#else
    static void aicwf_sdio_bus_pwrctl(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    struct aic_sdio_dev *sdiodev = (struct aic_sdio_dev *) data;
#else
    struct aic_sdio_dev *sdiodev = from_timer(sdiodev, t, timer);
#endif

    if (sdiodev->bus_if->state == BUS_DOWN_ST)
    {
        DBG_SDIO_ERR("bus down\n");
        return;
    }

    if (sdiodev->pwrctl_tsk)
    {
        complete(&sdiodev->pwrctrl_trgg);
    }
}
#endif

#if CONFIG_NOT_DEFINED
static void aicwf_sdio_enq_rxpkt(struct aic_sdio_dev *sdiodev, struct sk_buff *pkt)
{
    struct aicwf_rx_priv *rx_priv = sdiodev->rx_priv;
    unsigned long flags = 0;

    spin_lock_irqsave(&rx_priv->rxqlock, flags);
    if (!aicwf_frame_enqueue(rx_priv, &rx_priv->rxq, pkt))
    {
        spin_unlock_irqrestore(&rx_priv->rxqlock, flags);
        aicwf_dev_skb_free(pkt);
        return;
    }
    spin_unlock_irqrestore(&rx_priv->rxqlock, flags);

    atomic_inc(&rx_priv->rx_cnt);
}
#endif

#define SDIO_OTHER_INTERRUPT (0x1ul << 7)

void aicwf_sdio_hal_irqhandler(struct sdio_func *func)
{
    struct aicwf_bus *bus_if;
    struct aic_sdio_dev *sdiodev;
    u8 intstatus = 0;
    u8 byte_len = 0;
    //struct sk_buff *pkt = NULL;
    int ret;

    if (g_sdio_dev == NULL)
    {
        DBG_SDIO_ERR("SDIO already removed\n");
        return;
    }

    sdiodev = g_sdio_dev;

    bus_if = sdiodev->bus_if;
    if (!bus_if || bus_if->state == BUS_DOWN_ST)
    {
        DBG_SDIO_ERR("bus err\n");
        aicwf_sdio_fault_handler(sdiodev);
        return;
    }

    if (sdiodev->chipid != PRODUCT_ID_AIC8800M40)
    {
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
        while (ret || (intstatus & SDIO_OTHER_INTERRUPT))
        {
            DBG_SDIO_ERR("ret=%d, intstatus=%x\r\n", ret, intstatus);
            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
        }
        DBG_SDIO_VRB("enter %s %x\n", __func__, intstatus);

        if (intstatus > 0)
        {
            if (intstatus < 64)
            {
                sdiodev->rx_priv->data_len = intstatus * SDIOWIFI_FUNC_BLOCKSIZE;
                //pkt = aicwf_sdio_readframes(sdiodev);
            }
            else
            {
                aicwf_sdio_intr_get_len_bytemode(sdiodev, &byte_len);//byte_len must<= 128
                DBG_SDIO_INF("byte mode len=%d\r\n", byte_len);
                //pkt = aicwf_sdio_readframes(sdiodev);
            }
            if (sdiodev->rx_priv->data_len)
            {
                do
                {
                    uint8_t *buf_rx;
                    uint32_t data_len = sdiodev->rx_priv->data_len;
                    struct aicwf_buf_node_s *node = aicwf_busrx_buf_alloc(data_len);
                    if ((node == NULL) || (node->buf == NULL))
                    {
                        DBG_SDIO_ERR("node/buf alloc fail(len=%d),node=%p,buf=%p!!!\n", data_len, node, node->buf);
                        if (node)
                        {
                            aicwf_busrx_buf_free(node);
                        }
                        break;
                    }
                    buf_rx = node->buf;
                    ret = aicwf_sdio_recv_pkt(sdiodev, (u32 *)buf_rx, data_len);
                    if (ret)
                    {
                        DBG_SDIO_ERR("recv pkt err %d\n", ret);
                        aicwf_sdio_fault_handler(sdiodev);
                        aicwf_busrx_buf_free(node);
                        break;
                    }
                    node->buf_len = data_len;
                    //DBG_SDIO_INF("enq,%p,%d, node:%p\n",buf_rx, data_len, node);
                    aicwf_frame_enqueue(&sdiodev->rx_priv->rxq, node);
                    rtos_semaphore_signal(bus_if->busrx_trgg, true); // to be confirmed
                }
                while (0);
            }
        }
        else
        {
#ifndef CONFIG_PLATFORM_ALLWINNER
            DBG_SDIO_ERR("Interrupt but no data\n");
#endif
        }

        //if (pkt)
        //    aicwf_sdio_enq_rxpkt(sdiodev, pkt);

        //complete(&bus_if->busrx_trgg);
    }
#if 0
    else
    {
        do
        {
            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.misc_int_status_reg, &intstatus);
            if (!ret)
            {
                break;
            }
            DBG_SDIO_ERR("ret=%d, intstatus=%x\r\n", ret, intstatus);
        }
        while (1);
        if (intstatus & SDIO_OTHER_INTERRUPT)
        {
            u8 int_pending;
            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.sleep_reg, &int_pending);
            if (ret < 0)
            {
                DBG_SDIO_ERR("reg:%d read failed!\n", sdiodev->sdio_reg.sleep_reg);
            }
            int_pending &= ~0x01; // dev to host soft irq
            ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.sleep_reg, int_pending);
            if (ret < 0)
            {
                DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.sleep_reg);
            }
        }

        if (intstatus > 0)
        {
            uint8_t intmaskf2 = intstatus | (0x1UL << 3);
            if (intmaskf2 > 120U)   // func2
            {
                if (intmaskf2 == 127U)   // byte mode
                {
                    //aicwf_sdio_intr_get_len_bytemode(sdiodev, &byte_len, 1);//byte_len must<= 128
                    aicwf_sdio_intr_get_len_bytemode(sdiodev, &byte_len);//byte_len must<= 128
                    sdio_info("byte mode len=%d\r\n", byte_len);
                    //pkt = aicwf_sdio_readframes(sdiodev, 1);
                    pkt = aicwf_sdio_readframes(sdiodev);
                }
                else     // block mode
                {
                    sdiodev->rx_priv->data_len = (intstatus & 0x7U) * SDIOWIFI_FUNC_BLOCKSIZE;
                    //pkt = aicwf_sdio_readframes(sdiodev, 1);
                    pkt = aicwf_sdio_readframes(sdiodev);
                }
            }
            else     // func1
            {
                if (intstatus == 120U)   // byte mode
                {
                    //aicwf_sdio_intr_get_len_bytemode(sdiodev, &byte_len, 0);//byte_len must<= 128
                    aicwf_sdio_intr_get_len_bytemode(sdiodev, &byte_len);//byte_len must<= 128
                    sdio_info("byte mode len=%d\r\n", byte_len);
                    //pkt = aicwf_sdio_readframes(sdiodev, 0);
                    pkt = aicwf_sdio_readframes(sdiodev);
                }
                else     // block mode
                {
                    sdiodev->rx_priv->data_len = (intstatus & 0x7FU) * SDIOWIFI_FUNC_BLOCKSIZE;
                    //pkt = aicwf_sdio_readframes(sdiodev, 0);
                    pkt = aicwf_sdio_readframes(sdiodev);
                }
            }
        }
        else
        {
#ifndef CONFIG_PLATFORM_ALLWINNER
            DBG_SDIO_ERR("Interrupt but no data\n");
#endif
        }

        if (pkt)
            aicwf_sdio_enq_rxpkt(sdiodev, pkt);

#if 0
        if (atomic_read(&sdiodev->rx_priv->rx_cnt) == 1)
        {
            complete(&bus_if->busrx_trgg);
        }
#endif
        complete(&bus_if->busrx_trgg);
    }
#endif
}

#if defined(CONFIG_SDIO_PWRCTRL)
void aicwf_sdio_pwrctl_timer(struct aic_sdio_dev *sdiodev, uint duration)
{
    uint timeout;

    if (sdiodev->bus_if->state == BUS_DOWN_ST && duration)
        return;

    spin_lock_bh(&sdiodev->pwrctl_lock);
    if (!duration)
    {
        if (timer_pending(&sdiodev->timer))
            del_timer_sync(&sdiodev->timer);
    }
    else
    {
        sdiodev->active_duration = duration;
        timeout = msecs_to_jiffies(sdiodev->active_duration);
        mod_timer(&sdiodev->timer, jiffies + timeout);
    }
    spin_unlock_bh(&sdiodev->pwrctl_lock);
}
#endif

static struct aicwf_bus_ops aicwf_sdio_bus_ops =
{
    .stop = aicwf_sdio_bus_stop,
    .start = aicwf_sdio_bus_start,
#ifdef CONFIG_DATA_TXRX
    .txdata = aicwf_sdio_bus_txdata,
#endif
    .txmsg = aicwf_sdio_bus_txmsg,
    .rxtask = sdio_busrx_thread,
    .txtask = sdio_bustx_thread,
};

void aicwf_sdio_reg_init(struct aic_sdio_dev *sdiodev)
{
    DBG_SDIO_INF("\n%s\n", __func__);

    if (sdiodev->chipid == PRODUCT_ID_AIC8800 || sdiodev->chipid == PRODUCT_ID_AIC8800MC)
    {
        sdiodev->sdio_reg.bytemode_len_reg =       SDIOWIFI_BYTEMODE_LEN_REG;
        sdiodev->sdio_reg.intr_config_reg =        SDIOWIFI_INTR_CONFIG_REG;
        sdiodev->sdio_reg.sleep_reg =              SDIOWIFI_SLEEP_REG;
        sdiodev->sdio_reg.wakeup_reg =             SDIOWIFI_WAKEUP_REG;
        sdiodev->sdio_reg.flow_ctrl_reg =          SDIOWIFI_FLOW_CTRL_REG;
        sdiodev->sdio_reg.register_block =         SDIOWIFI_REGISTER_BLOCK;
        sdiodev->sdio_reg.bytemode_enable_reg =    SDIOWIFI_BYTEMODE_ENABLE_REG;
        sdiodev->sdio_reg.block_cnt_reg =          SDIOWIFI_BLOCK_CNT_REG;
        sdiodev->sdio_reg.rd_fifo_addr =           SDIOWIFI_RD_FIFO_ADDR;
        sdiodev->sdio_reg.wr_fifo_addr =           SDIOWIFI_WR_FIFO_ADDR;
    }
    else if (sdiodev->chipid == PRODUCT_ID_AIC8800M40)
    {
        sdiodev->sdio_reg.bytemode_len_reg =       SDIOWIFI_BYTEMODE_LEN_REG_V3;
        sdiodev->sdio_reg.intr_config_reg =        SDIOWIFI_INTR_ENABLE_REG_V3;
        sdiodev->sdio_reg.sleep_reg =              SDIOWIFI_INTR_PENDING_REG_V3;
        sdiodev->sdio_reg.wakeup_reg =             SDIOWIFI_INTR_TO_DEVICE_REG_V3;
        sdiodev->sdio_reg.flow_ctrl_reg =          SDIOWIFI_FLOW_CTRL_Q1_REG_V3;
        sdiodev->sdio_reg.bytemode_enable_reg =    SDIOWIFI_BYTEMODE_ENABLE_REG_V3;
        sdiodev->sdio_reg.misc_int_status_reg =    SDIOWIFI_MISC_INT_STATUS_REG_V3;
        sdiodev->sdio_reg.rd_fifo_addr =           SDIOWIFI_RD_FIFO_ADDR_V3;
        sdiodev->sdio_reg.wr_fifo_addr =           SDIOWIFI_WR_FIFO_ADDR_V3;
    }
}

void aicwf_sdio_release(struct aic_sdio_dev *sdiodev)
{
    int ret;
    DBG_SDIO_INF("%s\n", __func__);

    //disable sdio interrupt
    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.intr_config_reg, 0x0);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.intr_config_reg);
    }
    xhci_sdio_claim_host(sdiodev->func);
    xhci_sdio_release_irq(sdiodev->func);
    xhci_sdio_release_host(sdiodev->func);

    //  rwnx_cmd_mgr_deinit(&sdiodev->cmd_mgr);
    DBG_SDIO_INF("exit %s\n", __func__);
}

int aicwf_sdio_func_init(struct aic_sdio_dev *sdiodev)
{
    //struct mmc_host *host;
    u8 block_bit0 = 0x1;
    u8 byte_mode_disable = 0x1;//1: no byte mode
    int ret = 0;
    uint32_t clock = SDIOWIFI_CLOCK_V2;

    //host = sdiodev->func->card->host;

    xhci_sdio_claim_host(sdiodev->func);
#if 0//SDIO PHASE SETTING
    sdiodev->func->card->quirks |= MMC_QUIRK_LENIENT_FN0;
    sdio_f0_writeb(sdiodev->func, SDIOWIFI_PHASE_V2, 0x13, &ret);
    if (ret < 0)
    {
        DBG_SDIO_ERR("write func0 fail %d\n", ret);
        sdio_release_host(sdiodev->func);
        return ret;
    }
#endif
    ret = xhci_sdio_set_block_size(sdiodev->func, SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret < 0)
    {
        DBG_SDIO_ERR("set blocksize fail %d\n", ret);
        xhci_sdio_release_host(sdiodev->func);
        return ret;
    }
    ret = xhci_sdio_enable_func(sdiodev->func);
    if (ret < 0)
    {
        xhci_sdio_release_host(sdiodev->func);
        DBG_SDIO_ERR("enable func fail %d.\n", ret);
        return ret;
    }

//    host->ios.clock = clock;
//    host->ops->set_ios(host, &host->ios);
//    printk("Set SDIO Clock %d MHz\n",host->ios.clock/1000000);

    xhci_sdio_release_host(sdiodev->func);

    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.register_block, block_bit0);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.register_block);
        return ret;
    }

    //1: no byte mode
    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.bytemode_enable_reg, byte_mode_disable);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
        return ret;
    }

    return ret;
}

int aicwf_sdiov3_func_init(struct aic_sdio_dev *sdiodev)
{
    //u8 val = 0;
    int ret = 0;
    u8 val1 = 0;
    //struct mmc_host *host;
    u8 byte_mode_disable = 0x1;//1: no byte mode

    //host = sdiodev->func->card->host;

    xhci_sdio_claim_host(sdiodev->func);
    //sdiodev->func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

    ret = xhci_sdio_set_block_size(sdiodev->func, SDIOWIFI_FUNC_BLOCKSIZE);
    if (ret < 0)
    {
        DBG_SDIO_ERR("set blocksize fail %d\n", ret);
        xhci_sdio_release_host(sdiodev->func);
        return ret;
    }
    ret = xhci_sdio_enable_func(sdiodev->func);
    if (ret < 0)
    {
        xhci_sdio_release_host(sdiodev->func);
        DBG_SDIO_ERR("enable func fail %d.\n", ret);
        return ret;
    }

//    sdio_f0_writeb(sdiodev->func, 0x7F, 0xF2, &ret);
//    if (ret) {
//        DBG_SDIO_ERR("set fn0 0xF2 fail %d\n", ret);
//        sdio_release_host(sdiodev->func);
//        return ret;
//    }
#if 0
    if (host->ios.timing == MMC_TIMING_UHS_DDR50)
    {
        val = 0x21;//0x1D;//0x5;
    }
    else
    {
        val = 0x01;//0x19;//0x1;
    }
    val |= SDIOCLK_FREE_RUNNING_BIT;
    sdio_f0_writeb(sdiodev->func, val, 0xF0, &ret);
    if (ret)
    {
        DBG_SDIO_ERR("set iopad ctrl fail %d\n", ret);
        sdio_release_host(sdiodev->func);
        return ret;
    }
    sdio_f0_writeb(sdiodev->func, 0x0, 0xF8, &ret);
    if (ret)
    {
        DBG_SDIO_ERR("set iopad delay2 fail %d\n", ret);
        sdio_release_host(sdiodev->func);
        return ret;
    }
    sdio_f0_writeb(sdiodev->func, 0x20, 0xF1, &ret);
    if (ret)
    {
        DBG_SDIO_ERR("set iopad delay1 fail %d\n", ret);
        sdio_release_host(sdiodev->func);
        return ret;
    }
    msleep(1);
#if 1//SDIO CLOCK SETTING
    if (host->ios.timing != MMC_TIMING_UHS_DDR50)
    {
        host->ios.clock = SDIOWIFI_CLOCK_V3;
        host->ops->set_ios(host, &host->ios);
    }
#endif
#endif
    //AICWFDBG(LOGINFO, "Set SDIO Clock %d MHz\n", host->ios.clock/1000000);
    xhci_sdio_release_host(sdiodev->func);

    //1: no byte mode
    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.bytemode_enable_reg, byte_mode_disable);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.bytemode_enable_reg);
        return ret;
    }

    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.wakeup_reg, 0x11);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d write failed!\n", sdiodev->sdio_reg.wakeup_reg);
        return ret;
    }

#if 1
    //mdelay(5);
    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.sleep_reg, &val1);
    if (ret < 0)
    {
        DBG_SDIO_ERR("reg:%d read failed!\n", sdiodev->sdio_reg.sleep_reg);
        return ret;
    }

    if (!(val1 & 0x10))
    {
        DBG_SDIO_ERR("wakeup fail\n");
    }
    else
    {
        DBG_SDIO_INF("sdio ready\n");
    }
#endif
    return ret;
}

void aicwf_sdio_func_deinit(struct aic_sdio_dev *sdiodev)
{
    xhci_sdio_claim_host(sdiodev->func);
    xhci_sdio_disable_func(sdiodev->func);
    xhci_sdio_release_host(sdiodev->func);
}

void *aicwf_sdio_bus_init(struct aic_sdio_dev *sdiodev)
{
    int ret;
    struct aicwf_bus *bus_if;
    struct aicwf_rx_priv *rx_priv;
    struct aicwf_tx_priv *tx_priv;

#if defined(CONFIG_SDIO_PWRCTRL)
    spin_lock_init(&sdiodev->pwrctl_lock);
    sema_init(&sdiodev->pwrctl_wakeup_sema, 1);
#endif

    bus_if = sdiodev->bus_if;
    //bus_if->dev = sdiodev->dev;
    bus_if->ops = &aicwf_sdio_bus_ops;
#if defined(CONFIG_SDIO_PWRCTRL)
    bus_if->state = BUS_DOWN_ST;
    sdiodev->state = SDIO_SLEEP_ST;
    sdiodev->active_duration = SDIOWIFI_PWR_CTRL_INTERVAL;
#else
    sdiodev->state = SDIO_ACTIVE_ST;
#endif

    rx_priv = aicwf_rx_init(sdiodev);
    if (!rx_priv)
    {
        DBG_SDIO_ERR("rx init fail\n");
        goto fail;
    }
    sdiodev->rx_priv = rx_priv;

    tx_priv = aicwf_tx_init(sdiodev);
    if (!tx_priv)
    {
        DBG_SDIO_ERR("tx init fail\n");
        goto fail;
    }
    sdiodev->tx_priv = tx_priv;
#ifdef CONFIG_DATA_TXRX
    aicwf_frame_queue_init(&tx_priv->txq);
    //spin_lock_init(&tx_priv->txqlock);
#endif

#if defined(CONFIG_SDIO_PWRCTRL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    init_timer(&sdiodev->timer);
    sdiodev->timer.data = (ulong) sdiodev;
    sdiodev->timer.function = aicwf_sdio_bus_pwrctl;
#else
    timer_setup(&sdiodev->timer, aicwf_sdio_bus_pwrctl, 0);
#endif
    init_completion(&sdiodev->pwrctrl_trgg);
#ifdef AICWF_SDIO_SUPPORT
    sdiodev->pwrctl_tsk = kthread_run(aicwf_sdio_pwrctl_thread, sdiodev, "aicwf_pwrctl");
#endif
    if (IS_ERR(sdiodev->pwrctl_tsk))
    {
        sdiodev->pwrctl_tsk = NULL;
    }
#endif

    ret = aicwf_bus_init(bus_if);
    if (ret < 0)
    {
        DBG_SDIO_ERR("bus init fail\n");
        goto fail;
    }

    ret = aicwf_sdio_bus_start(bus_if);
    if (ret != 0)
    {
        DBG_SDIO_ERR("bus start fail\n");
        goto fail;
    }

    return sdiodev;

fail:
    aicwf_sdio_release(sdiodev);
    return NULL;
}


int aicwf_rwnx_sdio_platform_init(struct aic_sdio_dev *sdiodev)
{
    struct rwnx_plat *rwnx_plat = NULL;
    void *drvdata;
    int ret = -1;//-ENODEV;

    rwnx_plat = rtos_calloc(1, sizeof(struct rwnx_plat));

    if (!rwnx_plat)
    {
        return -2;//-ENOMEM;
    }

    rwnx_plat->sdiodev = sdiodev;

    ret = rwnx_platform_init(rwnx_plat, &drvdata);

    return ret;
}

int aicwf_rwnx_sdio_platform_deinit(struct aic_sdio_dev *sdiodev)
{
    rwnx_platform_deinit(sdiodev->rwnx_hw);
    rtos_free(g_rwnx_plat);
    g_rwnx_plat = NULL;
    return 0;
}

int aicwf_sdio_probe(struct sdio_func *func)
{
    //struct mmc_host *host;
    struct aic_sdio_dev *sdiodev;
    struct aicwf_bus *bus_if;
    int err = -1;//-ENODEV;

    if (g_sdio_dev)
    {
        DBG_SDIO_ERR("SDIO already probed\n");
        return err;
    }

    if (func->num != 1)
    {
        DBG_SDIO_ERR("%s:%d\n", __func__, func->num);
        return err;
    }

    bus_if = rtos_calloc(1, sizeof(struct aicwf_bus));
    if (!bus_if)
    {
        DBG_SDIO_ERR("alloc bus fail\n");
        return err;
    }

    sdiodev = rtos_calloc(1, sizeof(struct aic_sdio_dev));
    if (!sdiodev)
    {
        rtos_free(bus_if);
        DBG_SDIO_ERR("alloc sdiodev fail\n");
        return -2;//-ENOMEM;
    }

    bus_if->bus_priv.sdio = sdiodev;
    sdiodev->func = func;
    sdiodev->bus_if = bus_if;
    //err = aicwf_sdio_chipmatch(sdiodev, func->vendor, func->device);
    sdiodev->chipid = PRODUCT_ID_AIC8800MC;

    aicwf_sdio_reg_init(sdiodev);

    if (sdiodev->chipid != PRODUCT_ID_AIC8800M40)
    {
        err = aicwf_sdio_func_init(sdiodev);
    }
    else
    {
        err = aicwf_sdiov3_func_init(sdiodev);
    }
    if (err < 0)
    {
        DBG_SDIO_ERR("sdio func init fail\n");
        goto fail1;
    }

    if (aicwf_sdio_bus_init(sdiodev) == NULL)
    {
        DBG_SDIO_ERR("sdio bus init fail\n");
        goto fail2;
    }

    //host->caps |= MMC_CAP_NONREMOVABLE;
    err = aicwf_rwnx_sdio_platform_init(sdiodev);
    if (err < 0)
    {
        DBG_SDIO_ERR("sdio platform init fail\n");
        goto fail3;
    }

    //aicwf_hostif_ready();
    g_sdio_dev = sdiodev;

    DBG_SDIO_INF("%s:%d done\n", __func__, func->num);

    return 0;
fail3:
    aicwf_bus_deinit(sdiodev->bus_if);
fail2:
    aicwf_sdio_func_deinit(sdiodev);
fail1:
    rtos_free(bus_if);
    rtos_free(sdiodev);
    return err;
}

void aicwf_sdio_remove(struct sdio_func *func)
{
    struct aic_sdio_dev *sdiodev;

    DBG_SDIO_INF("%s\n", __func__);

    if (g_sdio_dev == NULL)
    {
        DBG_SDIO_ERR("SDIO already removed\n");
        return;
    }

    sdiodev = g_sdio_dev;

    aicwf_sdio_release(sdiodev);
    aicwf_bus_deinit(sdiodev->bus_if);
    aicwf_sdio_func_deinit(sdiodev);
    rtos_free(sdiodev->bus_if);
    rtos_free(sdiodev);

    g_sdio_dev = NULL;

    DBG_SDIO_INF("%s done\n", __func__);
#ifdef CONFIG_PLATFORM_ALLWINNER
    platform_wifi_power_off();
#endif
}
