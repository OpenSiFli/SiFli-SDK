/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "rwnx_main.h"
#include "sys_al.h"
#include "aic_plat_log.h"

#ifdef CONFIG_SDIO_SUPPORT
#include "sdio_def.h"
#include "sdio_co.h"
#endif /* CONFIG_SDIO_SUPPORT */

#ifdef CONFIG_SDIO_SUPPORT
uint32_t hal_rx_buf_count = SDIO_RX_BUF_COUNT;
#endif /* CONFIG_SDIO_SUPPORT */

#ifdef CONFIG_SDIO_SUPPORT
#ifdef CONFIG_SDIO_BUS_PWRCTRL
int aicwf_hal_bus_pwr_stctl(uint target)
{
    aicwf_sdio_bus_pwr_stctl(target);
}

//__INLINE
void aicwf_hal_rxcnt_decrease(void)
{
    aicwf_sdio_rxcnt_decrease();
}

//__INLINE
void aicwf_hal_txpktcnt_increase(void)
{
    aicwf_sdio_txpktcnt_increase();
}

//__INLINE
void aicfw_hal_txpktcnt_decrease(void)
{
    aicfw_sdio_txpktcnt_decrease();
}
#endif /* CONFIG_SDIO_BUS_PWRCTRL */

#ifdef CONFIG_SDIO_ADMA
void aicwf_hal_adma_desc(void *desc, void (*func)(void *), uint8_t needfree)
{
    aicwf_sdio_adma_desc(desc, (sdio_adma_free)func, needfree);
}
#endif /* CONFIG_SDIO_ADMA */

int aicwf_hal_send_check(void)
{
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    if (sdiodev->fw_avail_bufcnt <= DATA_FLOW_CTRL_THRESH) {
        sdiodev->fw_avail_bufcnt = aicwf_sdio_flow_ctrl();
        if (sdiodev->fw_avail_bufcnt < -DATA_FLOW_CTRL_THRESH) {
            return -2;
        } else
        if (sdiodev->fw_avail_bufcnt <= DATA_FLOW_CTRL_THRESH) {
            return -1;
        }
    }

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    aicwf_sdio_bus_pwr_stctl(HOST_BUS_ACTIVE);
    #endif

    if (aicwf_sdio_send_check() != 0) {
        return -1;
    }

    return 0;
}

int aicwf_hal_send(u8 *pkt_data, u32 pkt_len, bool last)
{
    return aicwf_sdio_send(pkt_data, pkt_len, last);
}

int aicwf_hal_msg_push(uint8_t *buffer, struct lmac_msg *msg, uint16_t len)
{
    int ret = 0, index = 0;

    buffer[0] = (len+4) & 0x00ff;
    buffer[1] = ((len+4) >> 8) &0x0f;
    buffer[2] = HOST_TYPE_CFG_CMD_RSP;
    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
        g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
        buffer[3] = 0x0;
    else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        buffer[3] = crc8_ponl_107(&buffer[0], 3); // crc8
    index += 4;
    //there is a dummy word
    index += 4;

    //make sure little endian
    put_u16(&buffer[index], msg->id);
    index += 2;
    put_u16(&buffer[index], msg->dest_id);
    index += 2;
    put_u16(&buffer[index], msg->src_id);
    index += 2;
    put_u16(&buffer[index], msg->param_len);
    index += 2;
    memcpy(&buffer[index], (u8 *)msg->param, msg->param_len);

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
    aicwf_sdio_cmdstatus_set();
    aicwf_sdio_bus_pwr_stctl(HOST_BUS_ACTIVE);
    #endif

    ret = aicwf_sdio_tx_msg(buffer, len + 8, NULL);

    return ret;
}

bool aicwf_hal_another_ptk(uint8_t *data, uint32_t len)
{
    uint16_t aggr_len = 0;
    if (data == NULL || len == 0) {
        return false;
    }
    aggr_len = (*data | (*(data + 1) << 8));
    if (aggr_len == 0) {
        return false;
    }
    if (aggr_len > len) {
        AIC_LOG_PRINTF("%s error:%d/%d\n", __func__, aggr_len, len);
        return false;
    }
    return true;
}

//__INLINE
uint8_t *aicwf_hal_get_node_pkt_data(void *node)
{
    struct sdio_buf_node_s *local_node = (struct sdio_buf_node_s *)node;
    return local_node->buf;
}

//__INLINE
uint32_t aicwf_hal_get_node_pkt_len(void *node)
{
    struct sdio_buf_node_s *local_node = (struct sdio_buf_node_s *)node;
    return local_node->buf_len;
}

//__INLINE
int aicwf_hal_node_pkt_is_aggr(void)
{
    return 1;
}

void aicwf_hal_buf_free(void *node)
{
    sdio_buf_free((struct sdio_buf_node_s *)node);
}
#endif /* CONFIG_SDIO_SUPPORT */

