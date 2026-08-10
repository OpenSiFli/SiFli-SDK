/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tx_swdesc.h"
#include "dbg_assert.h"
#include "fhost_rx.h"
#include "fhost_tx.h"
#include "rwnx_main.h"
#include "wlan_if.h"
#include "sdio_co.h"
#include "sdio_def.h"
#include "rtos_port.h"
#include "wifi_port.h"
#include "netif_port.h"
#include "sys_al.h"
#include "aic_plat_log.h"
#include "rtos_errno.h"
#include "sdio_al.h"

/* sdio cache define */
#define SYS_CACHE_LINE_SIZE         (PLATFORM_CACHE_LINE_SIZE)
#define WCN_CACHE_ALIGNED(addr)     (((uint32_t)(addr) + (SYS_CACHE_LINE_SIZE - 1)) & (~(SYS_CACHE_LINE_SIZE - 1)))
#define IS_WCN_CACHE_ALIGNED(addr)  !((uint32_t)(addr) & (SYS_CACHE_LINE_SIZE - 1))
#define WCN_CACHE_ALIGN_VARIABLE    __attribute__((aligned(SYS_CACHE_LINE_SIZE)))

/* sdio flowcntrl define */
#define SDIO_FC_UNEXCEPNT_MAX       (20)
#define SDIO_FC_FDELAY_NMS          (6)
#define SDIO_FC_FDELAY_NUS          (30)
#define SDIO_FC_DELAY_MS()          rtos_msleep(1)
#define SDIO_FC_DELAY_US()          rtos_udelay(200)

/* Platform ADMA define */
#define SDIO_HEADER_LEN             (4)
#define SDIO_DATA_FAKE_LEN          (2)
#define SDIO_MGMT_FAKE_LEN          (4)

static struct sdio_buf_list_s sdio_rx_buf_list;
static struct sdio_buf_node_s sdio_rx_buf_node[SDIO_RX_BUF_COUNT];
WCN_CACHE_ALIGN_VARIABLE
static uint8_t sdio_rx_buf_pool[SDIO_RX_BUF_COUNT][SDIO_RX_BUF_SIZE];

void aicwf_sdio_reg_init(struct rwnx_hw *rwnx_hw, struct aic_sdio_dev *sdiodev)
{
    AIC_LOG_PRINTF("%s\n", __func__);

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801 || rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
       rwnx_hw->chipid == PRODUCT_ID_AIC8800DW){
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
    } else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80){
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

uint8_t crc8_ponl_107(uint8_t *p_buffer, uint16_t cal_size)
{
    uint8_t i;
    uint8_t crc = 0;
    if (cal_size==0) {
        return crc;
    }
    while (cal_size--) {
        for (i = 0x80; i > 0; i /= 2) {
            if (crc & 0x80)  {
                crc *= 2;
                crc ^= 0x07; //polynomial X8 + X2 + X + 1,(0x107)
            } else {
                crc *= 2;
            }
            if ((*p_buffer) & i) {
                crc ^= 0x07;
            }
        }
        p_buffer++;
    }
    return crc;
}

void sdio_buf_init(void)
{
    int idx, ret;
    struct sdio_buf_list_s *sdio_list = &sdio_rx_buf_list;

    ret = rtos_mutex_create(&sdio_list->mutex, "SdioBufLMutex");
    if (ret) {
        aic_dbg("sdio rx buf mutex create fail: %d\n", ret);
        AIC_ASSERT_ERR(0);
        return;
    }
    co_list_init(&sdio_list->list);
    if (rtos_semaphore_create(&sdio_list->sdio_rx_node_sema, "SdioBufNodeSema", SDIO_RX_BUF_COUNT, 0)) {
        AIC_ASSERT_ERR(0);
    }
    for (idx = 0; idx < SDIO_RX_BUF_COUNT; idx++) {
        struct sdio_buf_node_s *node = &sdio_rx_buf_node[idx];
        node->buf_raw = &sdio_rx_buf_pool[idx][0];
        node->buf = NULL;
        node->buf_len = 0;
        node->pad_len = 0;
        co_list_push_back(&sdio_list->list, &node->hdr);
        rtos_semaphore_signal(sdio_list->sdio_rx_node_sema, 0);
        DBG_SDIO_VRB("InitN=%p\n", node);
    }
	sdio_list->alloc_fail_cnt = 0;
    AIC_LOG_PRINTF("sdio_rx_node_sema initial count:%d\n", rtos_semaphore_get_count(sdio_list->sdio_rx_node_sema));
}

static struct sdio_buf_node_s *sdio_buf_alloc(uint16_t buf_len)
{
    struct sdio_buf_node_s *node = NULL;
    struct sdio_buf_list_s *sdio_list = &sdio_rx_buf_list;

    int ret = rtos_semaphore_wait(sdio_list->sdio_rx_node_sema, 100);

    if (ret == 0) {
        rtos_mutex_lock(sdio_list->mutex, -1);

        if (co_list_is_empty(&sdio_list->list)) {
            rtos_mutex_unlock(sdio_list->mutex);
            return NULL;
        }

        node = (struct sdio_buf_node_s *)co_list_pop_front(&sdio_list->list);
        if (buf_len > SDIO_RX_BUF_SIZE) {
            uint8_t *buf_raw = rtos_malloc(buf_len + SYS_CACHE_LINE_SIZE);
            if (buf_raw == NULL) {
                AIC_LOG_PRINTF("sdio buf alloc fail(len=%d)!!!\n", buf_len);
                node->buf = NULL;
                node->buf_len = 0;
                node->pad_len = 0;
				sdio_list->alloc_fail_cnt++;
				if(sdio_list->alloc_fail_cnt > 500)  //may 1s
				{
					aic_driver_unexcepted(0x01);
					rtos_mutex_unlock(sdio_list->mutex);
					rtos_task_suspend(-1);
					return NULL;
				}
            } else {
                node->buf = (uint8_t *)WCN_CACHE_ALIGNED(buf_raw);
                node->buf_len = buf_len;
                node->pad_len = node->buf - buf_raw;
				sdio_list->alloc_fail_cnt = 0;
            }
        } else {
            node->buf = node->buf_raw;
            node->buf_len = buf_len;
            node->pad_len = 0;
        }

        rtos_mutex_unlock(sdio_list->mutex);
    }
    DBG_SDIO_VRB("AlocN=%p\n", node);
    return node;
}

void sdio_buf_free(struct sdio_buf_node_s *node)
{
    struct sdio_buf_list_s *sdio_list = &sdio_rx_buf_list;

    DBG_SDIO_VRB("FreeN=%p\n", node);
    rtos_mutex_lock(sdio_list->mutex, -1);

    if (node->buf_len == 0) {
        AIC_LOG_PRINTF("null sdio buf free, buf=%p\n", node->buf);
    } else if (node->buf_len > SDIO_RX_BUF_SIZE) {
        uint8_t *buf_raw = node->buf - node->pad_len;
        rtos_free(buf_raw);
    }

    node->buf = NULL;
    node->buf_len = 0;
    node->pad_len = 0;
    co_list_push_back(&sdio_list->list, &node->hdr);
    rtos_mutex_unlock(sdio_list->mutex);
    rtos_semaphore_signal(sdio_list->sdio_rx_node_sema, 0);
}

static void aicwf_sdio_aggrbuf_reset(struct aic_sdio_dev *sdiodev)
{
    uint8_t i;
    sdiodev->tail = sdiodev->head;

#ifdef CONFIG_SDIO_ADMA
    uint16_t head;
    for(i=0;i<(sdiodev->aggr_segcnt - sdiodev->fill);i++)
    {
        AIC_ASSERT_ERR(ALIGN4_ADJ_LEN((uint32_t)(sdiodev->sg_list[i].buf)) == 0);
        head = *((uint16*)(sdiodev->sg_list[i].buf));
        if(head != 0)
        {
            if(sdiodev->sg_list[i].buf[2] != 0x01)
            {
                struct txdesc *txdesc;
                struct fhost_tx_desc_tag *desc;
                struct hostdesc *host;
                net_buf_tx_t *net_buf;

                desc = (struct fhost_tx_desc_tag *)sdiodev->desc_buf[i];
                txdesc = (struct txdesc *)&desc->txdesc;
                host = (struct hostdesc *)&txdesc->host;
                net_buf = (net_buf_tx_t *)host->packet_addr;

                AIC_LOG_PRINTF("0x%p 0x%p 0x%p",sdiodev->sg_list[i].buf,sdiodev->desc_buf[i],net_buf->data_ptr);
                AIC_ASSERT_ERR(0);
            }
        }
    }

    for(i=sdiodev->aggr_segcnt;i<SDIO_TX_SLIST_MAX;i++)
    {
        sdiodev->sg_list[i].buf = 0;
        sdiodev->sg_list[i].len = 0;
    }

    for(i=0;i<SDIO_TX_SLIST_MAX;i++)
    {
        if((sdiodev->sfree)&&(sdiodev->desc_free[i])&&(sdiodev->desc_buf[i]))
        {
            sdiodev->sfree(sdiodev->desc_buf[i]);
        }
        sdiodev->desc_buf[i] = NULL;
        sdiodev->desc_free[i] = 0;
    }

    sdiodev->aggr_segcnt = 0;
    sdiodev->fill = 0;
#endif /* CONFIG_SDIO_ADMA */

    sdiodev->len = 0;
    sdiodev->aggr_count = 0;
    sdiodev->aggr_end = 0;

	sdiodev->stamp = rtos_now(0);
}

void aicwf_sdio_tx_init(void)
{
    int ret;
    struct aic_sdio_dev *sdiodev = &sdio_dev;
#ifndef	CONFIG_SDIO_ADMA
    sdiodev->aggr_buf = rtos_malloc(MAX_AGGR_TXPKT_LEN + AGGR_TXPKT_ALIGN_SIZE);
    if(!sdiodev->aggr_buf) {
        AIC_LOG_PRINTF("Alloc sdio tx aggr_buf failed!\n");
        return;
    }
    sdiodev->aggr_buf_align = (u8 *)(((uint32_t)sdiodev->aggr_buf + AGGR_TXPKT_ALIGN_SIZE - 1) & ~(AGGR_TXPKT_ALIGN_SIZE - 1));
#else
	memset(sdio_tx_buf_fill,0,512);
	memset(sdio_tx_buf_dummy,0,SDIO_TX_SLIST_MAX*8);
	sdiodev->aggr_buf = NULL;
	sdiodev->sfree = NULL;
	sdiodev->aggr_segcnt = 0;
	sdiodev->fill = 0;
#endif
    sdiodev->fw_avail_bufcnt = 0;
	sdiodev->fc_th = 0;
    sdiodev->head = sdiodev->aggr_buf_align;
    sdiodev->tx_aggr_counter = TX_AGGR_COUNTER;
    aicwf_sdio_aggrbuf_reset(sdiodev);
    AIC_LOG_PRINTF("sdio aggr_buf:%p, aggr_buf_align:%p\n", sdiodev->aggr_buf, sdiodev->aggr_buf_align);
}

void aicwf_sdio_tx_deinit(void)
{
    struct aic_sdio_dev *sdiodev = &sdio_dev;
    if (sdiodev->aggr_buf) {
        rtos_free(sdiodev->aggr_buf);
        sdiodev->aggr_buf = NULL;
        sdiodev->aggr_buf_align = NULL;
    }
    aicwf_sdio_aggrbuf_reset(sdiodev);
}

int aicwf_sdio_send_pkt(u8 *buf, uint count)
{
    int ret = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;
    sdio_claim_host(sdio_function[FUNC_1]);
    ret = sdio_writesb(sdio_function[FUNC_1], sdiodev->sdio_reg.wr_fifo_addr, buf, count);
    sdio_release_host(sdio_function[FUNC_1]);
    return ret;
}

static int aicwf_sdio_aggr(struct aic_sdio_dev *sdiodev, u8 *pkt_data, u32 pkt_len)
{
#ifdef	CONFIG_SDIO_ADMA

    #ifndef CONFIG_SDIO_ADMA_ADJ
    u8 adj_len = ALIGN4_ADJ_LEN((uint32_t)pkt_data);
    #else
    u8 adj_len = 0;
    #endif

    u8 *start_ptr = pkt_data - adj_len;
    u8 *sdio_header = NULL;
    u32 sdio_len = 0;
    u32 curr_len = 0;
    int align_len = 0;

    AIC_ASSERT_ERR((adj_len==0)||(adj_len==2));        // mgmt-pkt or data-pkt
    AIC_ASSERT_ERR((((uint32_t)start_ptr)&3) == 0);    // start_ptr addr word align

    if (adj_len==0) {
        // payload pkt word alignment
        sdio_len = pkt_len + adj_len + SDIO_MGMT_FAKE_LEN;
        if (sdio_len & (TX_ALIGNMENT - 1)) {
            align_len = TX_ALIGNMENT - (sdio_len & (TX_ALIGNMENT - 1));
            #ifdef CONFIG_SDIO_ADMA_ADJ
            memset(pkt_data + sdio_len,0,align_len);
            #endif
            sdio_len += align_len;
        }

        start_ptr = start_ptr - SDIO_HEADER_LEN - SDIO_MGMT_FAKE_LEN; // move start addr.
        sdio_header = start_ptr;

        sdio_header[0] =((sdio_len) & 0xff);
        sdio_header[1] =(((sdio_len) >> 8)&0x0f);
        sdio_header[2] = HOST_TYPE_DATA_SOC2FW;
        if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
            sdio_header[3] = crc8_ponl_107(&sdio_header[0], 3); // crc8
        else
            sdio_header[3] = 0; //reserved

        // fill 4 byte 0x00, need to fasync with data-pkt.
        sdio_header[4] = 0;
        sdio_header[5] = 0;
        sdio_header[6] = 0;
        sdio_header[7] = 0;

        curr_len = sdio_len + SDIO_HEADER_LEN;

        sdiodev->sg_list[sdiodev->aggr_segcnt].buf = start_ptr;
        sdiodev->sg_list[sdiodev->aggr_segcnt].len = curr_len;

        sdiodev->aggr_segcnt++;
        sdiodev->aggr_count++;

        sdiodev->len += curr_len;
    }
    else { // adj_len==2

        // payload pkt word alignment
        sdio_len = pkt_len + adj_len + SDIO_DATA_FAKE_LEN;
        if (sdio_len & (TX_ALIGNMENT - 1)) {
            align_len = TX_ALIGNMENT - (sdio_len & (TX_ALIGNMENT - 1));
            #ifdef CONFIG_SDIO_ADMA_ADJ
            memset(pkt_data + sdio_len,0,align_len);
            #endif
            sdio_len += align_len;
        }

        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][0] = ((sdio_len) & 0xff);
        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][1] = (((sdio_len) >> 8)&0x0f);
        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][2] = HOST_TYPE_DATA_SOC2FW;
        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][3] = 0;
        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][4] = 0;
        sdio_tx_buf_dummy[sdiodev->aggr_segcnt][5] = 0;
        sdiodev->sg_list[sdiodev->aggr_segcnt].buf = sdio_tx_buf_dummy[sdiodev->aggr_segcnt];
        sdiodev->sg_list[sdiodev->aggr_segcnt].len = SDIO_HEADER_LEN + SDIO_DATA_FAKE_LEN;

        sdiodev->aggr_segcnt++;

        sdio_header = start_ptr;
        sdio_header[0] = 0;
        sdio_header[1] = 0;

        curr_len = sdio_len - SDIO_DATA_FAKE_LEN;

        sdiodev->sg_list[sdiodev->aggr_segcnt].buf = start_ptr;
        sdiodev->sg_list[sdiodev->aggr_segcnt].len = curr_len;

        // sdio aggr fake packet is not need to free.
        sdiodev->desc_buf[sdiodev->aggr_segcnt] = sdiodev->desc_buf[sdiodev->aggr_segcnt-1];
        sdiodev->desc_free[sdiodev->aggr_segcnt] = sdiodev->desc_free[sdiodev->aggr_segcnt-1];;
        sdiodev->desc_buf[sdiodev->aggr_segcnt-1] = NULL;
        sdiodev->desc_free[sdiodev->aggr_segcnt-1] = 0;

        sdiodev->aggr_segcnt++;
        sdiodev->aggr_count++;

        sdiodev->len += curr_len + SDIO_HEADER_LEN + SDIO_DATA_FAKE_LEN;
    }

    #if 0
    if (((sdiodev->len + (TXPKT_BLOCKSIZE - 1)) & ~(TXPKT_BLOCKSIZE - 1)) > MAX_AGGR_TXPKT_LEN) {
        printf("over MAX_AGGR_TXPKT_LEN\n");
        sdiodev->aggr_end = 1;
    }
    #endif

    if (sdiodev->aggr_count >= sdiodev->fw_avail_bufcnt - DATA_FLOW_CTRL_THRESH) {
        //AIC_LOG_PRINTF("sdio cnt thrs: avl=%d, cnt=%d\r\n", sdiodev->fw_avail_bufcnt, sdiodev->aggr_count);
        sdiodev->aggr_end = 1;
    }

	if(sdiodev->aggr_segcnt >= (SDIO_TX_SLIST_MAX-2))
	{
		//AIC_LOG_PRINTF("sdio aggr max: avl=%d, seg=%d\r\n", sdiodev->fw_avail_bufcnt, sdiodev->aggr_segcnt);
		sdiodev->aggr_end = 1;
	}

#else /* CONFIG_SDIO_ADMA */
    struct rwnx_txhdr *txhdr = (struct rwnx_txhdr *)pkt_data;
    u8 *start_ptr = sdiodev->tail;
    u8 sdio_header[4];
    u8 adjust_str[4] = {0, 0, 0, 0};
    u32 curr_len = 0;
    int allign_len = 0;

    #if 0
    sdio_header[0] =((pkt_len - sizeof(struct rwnx_txhdr) + sizeof(struct txdesc_api)) & 0xff);
    sdio_header[1] =(((pkt_len - sizeof(struct rwnx_txhdr) + sizeof(struct txdesc_api)) >> 8)&0x0f);
    sdio_header[2] = 0x01; //data
    sdio_header[3] = 0; //reserved

    memcpy(sdiodev->tail, (u8 *)&sdio_header, sizeof(sdio_header));
    sdiodev->tail += sizeof(sdio_header);
    //payload
    memcpy(sdiodev->tail, (u8 *)(long)&txhdr->sw_hdr->desc, sizeof(struct txdesc_api));
    sdiodev->tail += sizeof(struct txdesc_api); //hostdesc
    memcpy(sdiodev->tail, (u8 *)((u8 *)txhdr + txhdr->sw_hdr->headroom), pkt_len-txhdr->sw_hdr->headroom);
    sdiodev->tail += (pkt_len - txhdr->sw_hdr->headroom);
    #else
    sdio_header[0] =((pkt_len) & 0xff);
    sdio_header[1] =(((pkt_len) >> 8)&0x0f);
    sdio_header[2] = 0x01; //data
    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        sdio_header[3] = crc8_ponl_107(&sdio_header[0], 3); // crc8
    else
        sdio_header[3] = 0; //reserved

    memcpy(sdiodev->tail, (u8 *)&sdio_header, sizeof(sdio_header));
    sdiodev->tail += sizeof(sdio_header);
    // hostdesc + payload
    memcpy(sdiodev->tail, (u8 *)pkt_data, pkt_len);
    sdiodev->tail += pkt_len;
    #endif

    //word alignment
    curr_len = sdiodev->tail - sdiodev->head;
    if (curr_len & (TX_ALIGNMENT - 1)) {
        allign_len = TX_ALIGNMENT - (curr_len & (TX_ALIGNMENT - 1));
        memcpy(sdiodev->tail, adjust_str, allign_len);
        sdiodev->tail += allign_len;
    }

    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
        g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        start_ptr[0] = ((sdiodev->tail - start_ptr - 4) & 0xff);
        start_ptr[1] = (((sdiodev->tail - start_ptr - 4)>>8) & 0x0f);
    }

    sdiodev->len = sdiodev->tail - sdiodev->head;
    if (((sdiodev->len + (TXPKT_BLOCKSIZE - 1)) & ~(TXPKT_BLOCKSIZE - 1)) > MAX_AGGR_TXPKT_LEN) {
        AIC_LOG_PRINTF("over MAX_AGGR_TXPKT_LEN\n");
        sdiodev->aggr_end = 1;
    }
    sdiodev->aggr_count++;
    if (sdiodev->aggr_count == sdiodev->fw_avail_bufcnt - DATA_FLOW_CTRL_THRESH) {
        sdiodev->aggr_end = 1;
    }
#endif /* CONFIG_SDIO_ADMA */

    return 0;
}

static void aicwf_sdio_aggr_send(struct aic_sdio_dev *sdiodev)
{
#ifdef	CONFIG_SDIO_ADMA
	u8 *tx_buf = sdiodev->aggr_buf_align;
	int ret = -1;
	int curr_len = 0;

	if(sdiodev->aggr_count == 0)
		return;

    #if 0
    if (sdiodev->aggr_count > 1)
        aic_dbg("sdio ac=%d %d\n",sdiodev->aggr_count,sdiodev->len);
    #endif

	if(sdiodev->len & (TXPKT_BLOCKSIZE-1))
	{
		uint16_t alen = TXPKT_BLOCKSIZE-(sdiodev->len & (TXPKT_BLOCKSIZE-1));

		memset(sdio_tx_buf_fill, 0, TAIL_LEN);
		sdiodev->sg_list[sdiodev->aggr_segcnt].buf = sdio_tx_buf_fill;
		sdiodev->sg_list[sdiodev->aggr_segcnt].len = alen;
		sdiodev->aggr_segcnt += 1;
		sdiodev->fill = 1;
		//AIC_ASSERT_ERR(0);
	}

    #if 0
	sdio_claim_host(sdio_function[FUNC_1]);
	ret = NU_SDIODrv_Write_Memory_sg(sdio_dev.sdio_reg.wr_fifo_addr,(uint8_t*)(sdiodev->sg_list), sdiodev->aggr_segcnt);
	sdio_release_host(sdio_function[FUNC_1]);
    #else
    ret = aicwf_sdio_send_pkt((uint8_t*)(sdiodev->sg_list), sdiodev->aggr_segcnt);
    #endif

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
    aicfw_sdio_txpktcnt_decrease_cnt(sdiodev->aggr_count);
    #endif

	if (ret)
	{
		AIC_LOG_PRINTF("fail to send adma aggr pkt!\n");
	}

#else
    u8 *tx_buf = sdiodev->aggr_buf_align;
    int ret = 0;
    int curr_len = 0;

    #if 0
    if (sdiodev->aggr_count > 1)
        aic_dbg("sdio ac=%d %d\n",sdiodev->aggr_count,sdiodev->len);
    #endif

    //link tail is necessary
    curr_len = sdiodev->tail - sdiodev->head;
    if ((curr_len % TXPKT_BLOCKSIZE) != 0) {
        memset(sdiodev->tail, 0, TAIL_LEN);
        sdiodev->tail += TAIL_LEN;
    }

    sdiodev->len = sdiodev->tail - sdiodev->head;
    curr_len = (sdiodev->len + SDIOWIFI_FUNC_BLOCKSIZE - 1) & ~(SDIOWIFI_FUNC_BLOCKSIZE - 1);
    ret = aicwf_sdio_send_pkt(tx_buf, curr_len);

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
    aicfw_sdio_txpktcnt_decrease_cnt(sdiodev->aggr_count);
    #endif

    if (ret < 0) {
        AIC_LOG_PRINTF("fail to send aggr pkt!\n");
    }
#endif
    aicwf_sdio_aggrbuf_reset(sdiodev);
}

static void sdio_fc_delay(uint32_t cnt)
{
	static u8 delay_ms = 0;
	static uint16_t delay_cnt = 0;
	int16_t intval;
	bool txbuf_full;

	if(cnt == 1)
	{
		intval = rtos_now(0)-sdio_dev.stamp;
		txbuf_full = net_buf_tx_free_list_is_full();
#ifdef	SDIO_FC_NARB
		if((txbuf_full)&&(intval > 0 ))
#else
		if(0)
#endif
		{
			delay_ms = 0;
			delay_cnt = SDIO_FC_FDELAY_NUS;
			AIC_LOG_PRINTF("fc lvl:%d inv:%d ful:%d\r",delay_ms,intval,txbuf_full);
		}
		else
		{
			delay_ms = 1;
			delay_cnt = SDIO_FC_FDELAY_NMS;
		}
	}

	if(cnt <= delay_cnt)
	{
		if(delay_ms)
			SDIO_FC_DELAY_MS();
		else
			SDIO_FC_DELAY_US();
	}
	else if(cnt < 40)
		rtos_msleep(2);
	else
		rtos_msleep(10);
}

int aicwf_sdio_readb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 *val)
{
	int ret;
	sdio_claim_host(sdio_function[FUNC_1]);
	*val = sdio_readb(sdio_function[FUNC_1], regaddr, &ret);
	sdio_release_host(sdio_function[FUNC_1]);
	return ret;
}

int aicwf_sdio_readb_func2(struct aic_sdio_dev *sdiodev, uint regaddr, u8 *val)
{
    int ret;
    sdio_claim_host(sdio_function[FUNC_2]);
    *val = sdio_readb(sdio_function[FUNC_2], regaddr, &ret);
    sdio_release_host(sdio_function[FUNC_2]);
    return ret;
}

int aicwf_sdio_writeb(struct aic_sdio_dev *sdiodev, uint regaddr, u8 val)
{
	int ret;
	sdio_claim_host(sdio_function[FUNC_1]);
	sdio_writeb(sdio_function[FUNC_1], val, regaddr, &ret);
	sdio_release_host(sdio_function[FUNC_1]);
	return ret;
}

int aicwf_sdio_writeb_func2(struct aic_sdio_dev *sdiodev, uint regaddr, u8 val)
{
    int ret;
    sdio_claim_host(sdio_function[FUNC_2]);
    sdio_writeb(sdio_function[FUNC_2], val, regaddr, &ret);
    sdio_release_host(sdio_function[FUNC_2]);
    return ret;
}

int aicwf_sdio_flow_ctrl(void)
{
    int ret = -1;
    u8 fc_reg = 0;
    u32 count = 0;

    while (true) {
        struct aic_sdio_dev *sdiodev = &sdio_dev;
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.flow_ctrl_reg, &fc_reg);
        if (ret) {
            AIC_LOG_PRINTF("%s, reg read failed\n", __func__);

			if(!(wifi_chip_status_get()))
				rtos_task_suspend(-1);

			return (-1 - DATA_FLOW_CTRL_THRESH);
        }
        #ifndef CONFIG_AIC8800D80_SUPPORT
        fc_reg = fc_reg & SDIOWIFI_FLOWCTRL_MASK_REG;
        #endif
        if (fc_reg > DATA_FLOW_CTRL_THRESH) {
            ret = fc_reg;
            if (ret > sdiodev->tx_aggr_counter) {
                ret = sdiodev->tx_aggr_counter;
            }
			sdiodev->fc_th = 0;

			if(sdio_unexcept_test)
			{
				sdio_unexcept_test = 0;
				aic_driver_unexcepted(0x02);
				rtos_task_suspend(-1);
			}
            return ret;
        } else {
            if (count >= FLOW_CTRL_RETRY_COUNT) {
                ret = -fc_reg;
                AIC_LOG_PRINTF("data fc:%d\n", ret);
				//rtos_semaphore_signal(sdio_dev.sdio_rx_sema, false);
				sdiodev->fc_th++;
				if(sdiodev->fc_th >= SDIO_FC_UNEXCEPNT_MAX)
				{
					sdiodev->fc_th = 0;
					aic_driver_unexcepted(0x02);
					rtos_task_suspend(-1);
				}
                break;
            }
            count++;
			sdio_fc_delay(count);
        }
    }

    return ret;
}

static int aicwf_sdio_flow_ctrl_msg(void)
{
    int ret = -1;
    u8 fc_reg = 0;
    u32 count = 0;

    while (true) {
        struct aic_sdio_dev *sdiodev = &sdio_dev;
        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.flow_ctrl_reg, &fc_reg);
        if (ret) {
            AIC_LOG_PRINTF("%s, reg read failed, ret=%d\n", __func__, ret);
			if(!(wifi_chip_status_get()))
				rtos_task_suspend(-1);
            return -1;
        }
        #ifndef CONFIG_AIC8800D80_SUPPORT
        fc_reg = fc_reg & SDIOWIFI_FLOWCTRL_MASK_REG;
        #endif
        if (fc_reg != 0) {
            ret = fc_reg;
            if (ret > sdiodev->tx_aggr_counter) {
                ret = sdiodev->tx_aggr_counter;
            }
            return ret;
        } else {
            if (count >= FLOW_CTRL_RETRY_COUNT) {
                ret = -fc_reg;
                AIC_LOG_PRINTF("msg fc:%d\n", ret);
                break;
            }
            count++;
			if (count < 30)
				rtos_udelay(200);
            else if(count < 40)
                rtos_msleep(2);
            else
                rtos_msleep(10);
        }
    }

    return ret;
}

int aicwf_sdio_send(u8 *pkt_data, u32 pkt_len, bool last)
{
    struct aic_sdio_dev *sdiodev = &sdio_dev;
    aicwf_sdio_aggr(sdiodev, pkt_data, pkt_len);

    if (last || sdiodev->aggr_end) {
        sdiodev->fw_avail_bufcnt -= sdiodev->aggr_count;
        aicwf_sdio_aggr_send(sdiodev);
    }

    return 0;
}

int aicwf_sdio_send_msg(u8 *buf, uint count)
{
    int ret = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    if (!func_flag_tx){
        sdio_claim_host(sdio_function[FUNC_1]);
        ret = sdio_writesb(sdio_function[FUNC_1], sdiodev->sdio_reg.wr_fifo_addr, buf, count);
        sdio_release_host(sdio_function[FUNC_1]);
    } else {
        sdio_claim_host(sdio_function[FUNC_2]);
        ret = sdio_writesb(sdio_function[FUNC_2], sdiodev->sdio_reg.wr_fifo_addr, buf, count);
        sdio_release_host(sdio_function[FUNC_2]);
    }
    return ret;
}

int aicwf_sdio_send_check(void)
{
    struct aic_sdio_dev *sdiodev = &sdio_dev;
    u32 aggr_len = 0;

    //aggr_len = (sdiodev->tail - sdiodev->head);
	aggr_len = sdiodev->len;

	if(((sdiodev->aggr_count == 0) && (aggr_len != 0))
        || ((sdiodev->aggr_count != 0) && (aggr_len == 0))) {
        if (aggr_len > 0)
            aicwf_sdio_aggrbuf_reset(sdiodev);
        AIC_LOG_PRINTF("aggr_count=%d, aggr_len=%d, check fail\n", sdiodev->aggr_count, aggr_len);
        return -1;
    }

    if (sdiodev->aggr_count == (sdiodev->fw_avail_bufcnt - DATA_FLOW_CTRL_THRESH)) {
        if (sdiodev->aggr_count > 0) {
            sdiodev->fw_avail_bufcnt -= sdiodev->aggr_count;
            aic_dbg("cnt equals\n");
            aicwf_sdio_aggr_send(sdiodev); //send and check the next pkt;
            return 1;
        }
    }

    return 0;
}

static int aicwf_sdio_recv_pkt(u8 *buf, u32 size, u8 msg)
{
    int ret = -1;

    if ((!buf) || (!size)) {
        return -EINVAL;;
    }

    struct aic_sdio_dev *sdiodev = &sdio_dev;
    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        sdio_claim_host(sdio_function[FUNC_1]);
        ret = sdio_readsb(sdio_function[FUNC_1], buf, sdiodev->sdio_reg.rd_fifo_addr, size);
        sdio_release_host(sdio_function[FUNC_1]);
    } else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        if (!func_flag_rx) {
            sdio_claim_host(sdio_function[FUNC_1]);
            ret = sdio_readsb(sdio_function[FUNC_1], buf, sdiodev->sdio_reg.rd_fifo_addr, size);
            sdio_release_host(sdio_function[FUNC_1]);
        } else {
            if(!msg) {
                sdio_claim_host(sdio_function[FUNC_1]);
                ret = sdio_readsb(sdio_function[FUNC_1], buf, sdiodev->sdio_reg.rd_fifo_addr, size);
               sdio_release_host(sdio_function[FUNC_1]);
            } else {
                sdio_claim_host(sdio_function[FUNC_2]);
                ret = sdio_readsb(sdio_function[FUNC_2], buf, sdiodev->sdio_reg.rd_fifo_addr, size);
                sdio_release_host(sdio_function[FUNC_2]);
            }
        }
    }

    return ret;
}

int aicwf_sdio_tx_msg(u8 *buf, uint count, u8 *out)
{
    int err = 0;
    u16 len;
    u8 *payload = buf;
    u16 payload_len = (u16)count;
    u8 adjust_str[4] = {0, 0, 0, 0};
    int adjust_len = 0;
    int buffer_cnt = 0;
    u8 retry = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    len = payload_len;
    if ((len % TX_ALIGNMENT) != 0) {
        adjust_len = (len + (TX_ALIGNMENT - 1)) & ~(TX_ALIGNMENT - 1); //roundup(len, TX_ALIGNMENT);
        memcpy(payload+payload_len, adjust_str, (adjust_len - len));
        payload_len += (adjust_len - len);
    }
    len = payload_len;

    //link tail is necessary
    if ((len % SDIOWIFI_FUNC_BLOCKSIZE) != 0) {
        memset(payload+payload_len, 0, TAIL_LEN);
        payload_len += TAIL_LEN;
        len = (payload_len/SDIOWIFI_FUNC_BLOCKSIZE + 1) * SDIOWIFI_FUNC_BLOCKSIZE;
    } else
        len = payload_len;

    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        buffer_cnt = aicwf_sdio_flow_ctrl_msg();
        while ((buffer_cnt <= 0 || (buffer_cnt > 0 && len > (buffer_cnt * BUFFER_SIZE))) && retry < 10) {
            retry++;
            buffer_cnt = aicwf_sdio_flow_ctrl_msg();
        }
    } else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        if(!func_flag_tx)
        {
            buffer_cnt = aicwf_sdio_flow_ctrl_msg();
            while ((buffer_cnt <= 0 || (buffer_cnt > 0 && len > (buffer_cnt * BUFFER_SIZE))) && retry < 10) {
                retry++;
                buffer_cnt = aicwf_sdio_flow_ctrl_msg();
            }
        }
    }

    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        if (buffer_cnt > 0 && len <= (buffer_cnt * BUFFER_SIZE)) {
            //AIC_LOG_PRINTF("aicwf_sdio_send_pkt, len=%d\n", len);
            err = aicwf_sdio_send_pkt(payload, len);
            if (err) {
                AIC_LOG_PRINTF("aicwf_sdio_send_pkt fail%d\n", err);
                goto txmsg_exit;
            }
            if (msgcfm_poll_en && out) {
                u8 intstatus = 0;
                u32 data_len;
                int ret, idx;
                rtos_udelay(100);
                for (idx = 0; idx < 8; idx++) {
                    rtos_udelay(20000);
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    while (ret || (intstatus & SDIO_OTHER_INTERRUPT)) {
                        AIC_LOG_PRINTF("ret=%d, intstatus=%x\n",ret, intstatus);
                        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    }
                    AIC_LOG_PRINTF("[%d] intstatus=%d\n", idx, intstatus);
                    if (intstatus > 0) {
                        if (intstatus < 64) {
                            data_len = intstatus * SDIOWIFI_FUNC_BLOCKSIZE;
                        } else {
                            u8 byte_len = 0;
                            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                            if (ret) {
                                AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                                err = ret;
                                goto txmsg_exit;
                            }
                            AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                            data_len = byte_len * 4; //byte_len must<= 128
                        }
                        if (data_len) {
                            ret = aicwf_sdio_recv_pkt(out, data_len, 0);
                            if (ret) {
                                AIC_LOG_PRINTF("recv pkt err %d\r\n", ret);
                                err = ret;
                                goto txmsg_exit;
                            }
                            AIC_LOG_PRINTF("recv pkt done len=%d\r\n", data_len);
                        }
                        break;
                    }
                }
            }
        } else {
            AIC_LOG_PRINTF("tx msg fc retry fail\n");
            #ifdef CONFIG_SDIO_BUS_PWRCTRL
            //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
            aicwf_sdio_cmdstatus_clr();
            #endif
            return -1;
        }
    }
    else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
        if (((!func_flag_tx) && (buffer_cnt > 0 && len <= (buffer_cnt * BUFFER_SIZE))) || func_flag_tx) {
            //AIC_LOG_PRINTF("aicwf_sdio_send_msg, len=%d \n", len);
            err = aicwf_sdio_send_msg(payload, len);
            if (err) {
                AIC_LOG_PRINTF("aicwf_sdio_send_pkt fail %d\n", err);
                goto txmsg_exit;
            }
            if (msgcfm_poll_en && out) {
                u8 intstatus = 0;
                u32 data_len;
                int ret, idx;
                rtos_udelay(100);
                for (idx = 0; idx < 8; idx++) {
                    ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    while (ret || (intstatus & SDIO_OTHER_INTERRUPT)) {
                        AIC_LOG_PRINTF("ret=%d, intstatus=%x\n",ret, intstatus);
                        ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    }
                    AIC_LOG_PRINTF("[%d] intstatus=%d\n", idx, intstatus);
                    if (intstatus > 0) {
                        if (intstatus < 64) {
                            data_len = intstatus * SDIOWIFI_FUNC_BLOCKSIZE;
                        } else {
                            u8 byte_len = 0;
                            ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                            if (ret < 0) {
                                AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                                err = ret;
                                goto txmsg_exit;
                            }
                            AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                            data_len = byte_len * 4; //byte_len must<= 128
                        }
                        if (data_len) {
                            ret = aicwf_sdio_recv_pkt(out, data_len, 1);
                            if (ret) {
                                AIC_LOG_PRINTF("recv pkt err %d\r\n", ret);
                                err = ret;
                                goto txmsg_exit;
                            }
                            AIC_LOG_PRINTF("recv pkt done len=%d\r\n", data_len);
                        }
                        break;
                    }
                }
            }
        } else {
            AIC_LOG_PRINTF("tx msg fc retry fail\n");
            #ifdef CONFIG_SDIO_BUS_PWRCTRL
            //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
            aicwf_sdio_cmdstatus_clr();
            #endif
            return -1;
        }
    }else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        if (buffer_cnt > 0 && len <= (buffer_cnt * BUFFER_SIZE)) {
            //AIC_LOG_PRINTF("aicwf_sdio_send_pkt, len=%d\n", len);
            err = aicwf_sdio_send_pkt(payload, len);
            if (err) {
                AIC_LOG_PRINTF("aicwf_sdio_send_pkt fail%d\n", err);
                goto txmsg_exit;
            }
            if (msgcfm_poll_en && out) {
                u8 intstatus = 0;
                u32 data_len;
                int ret, idx;
                rtos_udelay(100);
                for (idx = 0; idx < 8; idx++) {
                    do {
                        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.misc_int_status_reg, &intstatus);
                        if (!ret) {
                            break;
                        }
                        AIC_LOG_PRINTF("ret=%d, intstatus=%x\r\n",ret, intstatus);
                    } while (1);
                    if (intstatus & SDIO_OTHER_INTERRUPT) {
                        u8 int_pending;
                        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.sleep_reg, &int_pending);
                        if (ret < 0) {
                            AIC_LOG_PRINTF("reg:%d read failed!\n", sdiodev->sdio_reg.sleep_reg);
                        }
                        int_pending &= ~0x01; // dev to host soft irq
                        ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.sleep_reg, int_pending);
                        if (ret < 0) {
                            AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.sleep_reg);
                        }
                    }
                    AIC_LOG_PRINTF("[%d] intstatus=%d\n", idx, intstatus);
                    if (intstatus > 0) {
                        uint8_t intmaskf2 = intstatus | (0x1UL << 3);
                        u8 byte_len = 0;
                        if (intmaskf2 > 120U) { // func2
                            if (intmaskf2 == 127U) { // byte mode
                                AIC_LOG_PRINTF("%s, intmaskf2=%d\r\n", __func__, intmaskf2);
                                ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                                if (ret < 0) {
                                    AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                                }
                                AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                                data_len = byte_len * 4; //byte_len must<= 128
                            } else { // block mode
                                data_len = (intstatus & 0x7U) * SDIOWIFI_FUNC_BLOCKSIZE;
                              }
                        } else { // func1
                            if (intstatus == 120U) { // byte mode
                                ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                                if (ret < 0) {
                                    AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                                }
                                AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                                data_len = byte_len * 4; //byte_len must<= 128
                                } else { // block mode
                                    data_len = (intstatus & 0x7FU) * SDIOWIFI_FUNC_BLOCKSIZE;
                            }
                        }

                        if (data_len) {
                               ret = aicwf_sdio_recv_pkt(out, data_len, 0);
                            if (ret) {
                                AIC_LOG_PRINTF("recv pkt err %d\r\n", ret);
                                err = ret;
                                goto txmsg_exit;
                            }
                            AIC_LOG_PRINTF("recv pkt done len=%d\r\n", data_len);
                        }
                        break;
                    }
                }
            }
        } else {
            AIC_LOG_PRINTF("tx msg fc retry fail\n");
            #ifdef CONFIG_SDIO_BUS_PWRCTRL
            //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
            aicwf_sdio_cmdstatus_clr();
            #endif
            return -1;
        }
    }

txmsg_exit:
    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
    aicwf_sdio_cmdstatus_clr();
    #endif
    return err;
}

#ifdef CONFIG_SDIO_ADMA
void aicwf_sdio_adma_desc(void *desc,sdio_adma_free func,uint8_t needfree)
{
	struct aic_sdio_dev *sdiodev = &sdio_dev;
	sdiodev->desc_buf[sdiodev->aggr_segcnt] = desc;
	sdiodev->desc_free[sdiodev->aggr_segcnt] = needfree;
	sdiodev->sfree = func;
}
#endif

#ifdef CONFIG_SDIO_BUS_PWRCTRL
int aicwf_sdio_bus_pwrctrl_preinit(void)
{
    int ret = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    sdiodev->rx_cnt = 0;
    sdiodev->tx_pktcnt = 0;
    sdiodev->cmd_txstate = false;
    sdiodev->pwrctl.bus_status = HOST_BUS_ACTIVE;
    sdiodev->pwrctl.init_status = false;
    sdiodev->pwrctl.active_duration = 50;

    rtos_mutex_create(&sdiodev->tx_sema, "tx_sema lock");
    if (sdiodev->tx_sema == NULL)
    {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio tx_sema create fail\n");
        return ret;
    }
    rtos_mutex_create(&sdiodev->rx_sema, "rx_sema lock");
    if (sdiodev->rx_sema == NULL)
    {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio rx_sema create fail\n");
        return ret;
    }
    rtos_mutex_create(&sdiodev->cmdtx_sema, "cmdtx_sema lock");
    if (sdiodev->cmdtx_sema == NULL)
    {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio cmdtx_sema create fail\n");
        return ret;
    }

    rtos_mutex_create(&sdiodev->pwrctl.bus_wakeup_sema, "bus_wakeup_sema");
    if (sdiodev->pwrctl.bus_wakeup_sema == NULL)
    {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio bus wakeup sema create fail\n");
        return ret;
    }
    rtos_mutex_create(&sdiodev->pwrctl.pwrctl_lock, "pwrctl_lock");
    if (sdiodev->pwrctl.pwrctl_lock == NULL)
    {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio bus pwrctl_lock sema create fail\n");
        return ret;
    }
    rtos_semaphore_create(&sdiodev->pwrctl.pwrctl_trgg, "pwrctrl_trgg", 8, 0);
    if (sdiodev->pwrctl.pwrctl_trgg == NULL) {
        ret = AIC_RTOS_CREATE_FAILED;
        AIC_LOG_PRINTF("sdio bus pwrctrl_trgg sema create fail\n");
        return ret;
    }

    return ret;
}

void aicwf_sdio_bus_pwrctl_timer_handler(void)
{
    DBG_SDIO_VRB(">>> timer_hdl\n");
    rtos_semaphore_signal(sdio_dev.pwrctl.pwrctl_trgg, 0);
}

void aicwf_sdio_bus_pwrctl_timer_modify(uint duration)
{
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    if (sdiodev->pwrctl.init_status == false) {
         return ;
    }
    DBG_SDIO_VRB("%s duration=%d\n", __func__, duration);

    rtos_mutex_lock(sdiodev->pwrctl.pwrctl_lock, -1);
    if (!duration) {
        rtos_timer_get_status(sdiodev->pwrctl.pwrctl_timer, &sdiodev->pwrctl.timer_status);
        if (sdiodev->pwrctl.timer_status == AIC_RTOS_TIMER_ACT)
            rtos_timer_stop(sdiodev->pwrctl.pwrctl_timer);

    } else {
        rtos_timer_get_status(sdiodev->pwrctl.pwrctl_timer, &sdiodev->pwrctl.timer_status);

        if (sdiodev->pwrctl.timer_status == AIC_RTOS_TIMER_DACT)
            rtos_timer_start(sdiodev->pwrctl.pwrctl_timer, duration);
        else {
            // restart tne timer.
            rtos_timer_stop(sdiodev->pwrctl.pwrctl_timer);
            rtos_timer_start(sdiodev->pwrctl.pwrctl_timer, duration);
        }
    }
    rtos_mutex_unlock(sdiodev->pwrctl.pwrctl_lock);
}

int aicwf_sdio_bus_sleep_allow(struct aic_sdio_dev *sdiodev);
void aicwf_sdio_bus_pwrctl_timer_task(void *arg)
{
    int ret = 0;
    uint32_t tx_pktcnt = 0, rx_cnt = 0, cmd_txstate = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    while(1) {
        ret = rtos_semaphore_wait(sdiodev->pwrctl.pwrctl_trgg, AIC_RTOS_WAIT_FOREVEVR);
        if (ret < 0) {
            AIC_LOG_PRINTF("wait pwrctrl_trgg fail: ret=%d\n", ret);
            continue;
        } else {
            rtos_mutex_lock(sdiodev->tx_sema, -1);
            tx_pktcnt = sdiodev->tx_pktcnt;
            rtos_mutex_unlock(sdiodev->tx_sema);
            rtos_mutex_lock(sdiodev->rx_sema, -1);
            rx_cnt = sdiodev->rx_cnt;
            rtos_mutex_unlock(sdiodev->rx_sema);
            rtos_mutex_lock(sdiodev->cmdtx_sema, -1);
            cmd_txstate = sdiodev->cmd_txstate;
            rtos_mutex_unlock(sdiodev->cmdtx_sema);
            DBG_SDIO_VRB("tx_pktcnt: %d, rx_cnt: %d, cmd_txstate:%d\n", tx_pktcnt, rx_cnt, cmd_txstate);
            if (sdiodev->pwrctl.bus_status == HOST_BUS_ACTIVE) {
                if((tx_pktcnt == 0) && (sdiodev->cmd_txstate == false) && (rx_cnt == 0)) {
                    // aicwf_sdio_pwr_stctl(sdiodev, HOST_BUS_SLEEP);
                    aicwf_sdio_bus_sleep_allow(sdiodev); // Direct call this func, for save time.
                }
                else
                    aicwf_sdio_bus_pwrctl_timer_modify(sdiodev->pwrctl.active_duration);
            }
        }
    }
}

void aicwf_sdio_bus_pwrctl_timer_task_init(void)
{
    int ret;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    ret = rtos_timer_create("bus_pwrctl", &sdiodev->pwrctl.pwrctl_timer, 30, 0, NULL, aicwf_sdio_bus_pwrctl_timer_handler);
    if (ret) {
        AIC_LOG_PRINTF("sdio pwrctl timer create fail\n");
        return;
    }
    ret = rtos_task_create(aicwf_sdio_bus_pwrctl_timer_task, "aicwf_sdio_bus_pwrctl_timer_task",
                           SDIO_PWRCTL_TASK, sdio_pwrctl_stack_size, NULL, sdio_pwrclt_priority,
                           &sdiodev->pwrctl.pwrctl_task_hdl);
    if (ret < 0) {
        AIC_LOG_PRINTF("aicwf_sdio_bus_pwrctl_timer_task create fail\n");
        return;
    }
    sdiodev->pwrctl.init_status = true;
}

void aicwf_sdio_bus_pwrctl_timer_task_deinit(void)
{
    int ret;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    if (sdiodev->pwrctl.pwrctl_timer) {
        ret = rtos_timer_stop(sdiodev->pwrctl.pwrctl_timer);
        if (ret) {
            AIC_LOG_PRINTF("sdio pwrctl timer stop fail\n");
            return;
        }
        ret = rtos_timer_delete(sdiodev->pwrctl.pwrctl_timer);
        if (ret) {
            AIC_LOG_PRINTF("sdio pwrctl timer delete fail\n");
            return;
        }
        sdiodev->pwrctl.pwrctl_timer = NULL;
    }
    if (sdiodev->pwrctl.pwrctl_task_hdl) {
        rtos_task_delete(sdiodev->pwrctl.pwrctl_task_hdl);
        sdiodev->pwrctl.pwrctl_task_hdl = NULL;
    }
    sdiodev->pwrctl.init_status = false;
}

int aicwf_sdio_bus_wakeup(struct aic_sdio_dev *sdiodev)
{
    int ret = 0;
    int read_retry;
    int write_retry = 20;
    int wakeup_reg_val = 0;

    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 ||
        g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
        g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        wakeup_reg_val = 1;
    } else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        wakeup_reg_val = 0x11;
    }
    DBG_SDIO_VRB("%s bus_status=%d\n", __func__, sdiodev->pwrctl.bus_status);

    if (sdiodev->pwrctl.bus_status == HOST_BUS_SLEEP) {
        rtos_mutex_lock(sdiodev->pwrctl.bus_wakeup_sema, -1);
        if(sdiodev->pwrctl.bus_status == HOST_BUS_ACTIVE) {
            rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
            return 0;
        }
        DBG_SDIO_VRB("wakeup timer\n");
        while(write_retry) {
            ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.wakeup_reg, wakeup_reg_val);
            if (ret) {
                AIC_LOG_PRINTF("sdio wakeup fail, ret=%d\n", ret);
                ret = -1;
            } else {
                read_retry=30;
                while (read_retry) {
                    u8 val = 0;
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.wakeup_reg, &val);
                    if (ret) {
                        AIC_LOG_PRINTF("sdio wakeup read fail\n");
                    } else if ((val & 0x1) == 0) {
                        //AIC_LOG_PRINTF("val: 0x%x\n", val);
                        break;
                    }
                    read_retry--;
                    if (read_retry < 15) {
                        AIC_LOG_PRINTF("retry %d, rd val: 0x%x\n", read_retry, val);
                    }
                    rtos_udelay(200);
                }
                if(read_retry != 0)
                    break;
            }

            AIC_LOG_PRINTF("write retry:  %d \n",write_retry);
            write_retry--;
            rtos_udelay(100);
        }

        if (!ret) {
            sdiodev->pwrctl.bus_status = HOST_BUS_ACTIVE;
            aicwf_sdio_bus_pwrctl_timer_modify(sdiodev->pwrctl.active_duration);
        }
        rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
    }
    return ret;
}

int aicwf_sdio_bus_sleep_allow(struct aic_sdio_dev *sdiodev)
{
    int ret = 0, tx_pktcnt = 0;
    int read_retry;
    DBG_SDIO_VRB("%s bus_status=%d\n", __func__, sdiodev->pwrctl.bus_status);

    rtos_mutex_lock(sdiodev->tx_sema, -1);
    tx_pktcnt = sdiodev->tx_pktcnt;
    rtos_mutex_unlock(sdiodev->tx_sema);
    if(tx_pktcnt > 0) {
        AIC_LOG_PRINTF("Cancel sdio sleep setting\n");
        return 0;
    }

    if ((sdiodev->pwrctl.bus_status == HOST_BUS_ACTIVE)
#ifdef CONFIG_SDIO_BUS_PWRCTRL_DYN
        &&(wlan_ap_get_associated_sta_cnt() == 0)
#endif
    ) {
        rtos_mutex_lock(sdiodev->pwrctl.bus_wakeup_sema, -1);
        if (!(g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 ||
            g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
            g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)) {
            ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.wakeup_reg, 0x2);
            if (ret) {
                rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
                AIC_LOG_PRINTF("Write sleep fail!, ret=%d\n", ret);
                return -1;
            }
        }
        else {
            ret = aicwf_sdio_writeb_func2(sdiodev, sdiodev->sdio_reg.wakeup_reg, 0x2);
            if (ret) {
                rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
                AIC_LOG_PRINTF("Write sleep fail!, ret=%d\n", ret);
                return -1;
            }
            read_retry = 30;
            while (read_retry) {
                u8 val = 0;
                ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.wakeup_reg, &val);
                if (ret) {
                    AIC_LOG_PRINTF("read sleep fail, ret=%d\n", ret);
                } else if ((val & 0x2) == 0) {
                    //AIC_LOG_PRINTF("val: 0x%x\n", val);
                    break;
                }
                read_retry--;
                if (read_retry < 15) {
                    AIC_LOG_PRINTF("retry %d, rd val: 0x%x\n", read_retry, val);
                }
                rtos_udelay(200);
            }
        }
        sdiodev->pwrctl.bus_status = HOST_BUS_SLEEP;
        aicwf_sdio_bus_pwrctl_timer_modify(0);
        rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
        DBG_SDIO_VRB("sdio bus sleep!!\n");
    }
    else {
        aicwf_sdio_bus_pwrctl_timer_modify(sdiodev->pwrctl.active_duration);
    }

    return ret;
}

int aicwf_sdio_bus_pwr_stctl(uint target)
{
    int ret = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    rtos_mutex_lock(sdiodev->pwrctl.bus_wakeup_sema, -1);
    if (sdiodev->pwrctl.bus_status == target) {
        if (target == HOST_BUS_ACTIVE) {
            if (sdiodev->pwrctl.init_status) {
                DBG_SDIO_VRB("%s active\n", __func__);
            }
            aicwf_sdio_bus_pwrctl_timer_modify(sdiodev->pwrctl.active_duration);
        }
        rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);
        return ret;
    }
    rtos_mutex_unlock(sdiodev->pwrctl.bus_wakeup_sema);

    DBG_SDIO_VRB("%s bus_status=%d, target=%d\n", __func__, sdiodev->pwrctl.bus_status, target);
    switch (target) {
    case HOST_BUS_ACTIVE:
        aicwf_sdio_bus_wakeup(sdiodev);
        break;
    case HOST_BUS_SLEEP:
        aicwf_sdio_bus_sleep_allow(sdiodev);
        break;
    }

    return ret;
}

//__INLINE
void aicwf_sdio_rxcnt_increase(void)
{
    rtos_mutex_lock(sdio_dev.rx_sema, -1);
    sdio_dev.rx_cnt++;
    //AIC_LOG_PRINTF("rx_cnt:%d", sdio_dev.rx_cnt);
    rtos_mutex_unlock(sdio_dev.rx_sema);
}

//__INLINE
void aicwf_sdio_rxcnt_decrease(void)
{
    rtos_mutex_lock(sdio_dev.rx_sema, -1);
    sdio_dev.rx_cnt--;
    //AIC_LOG_PRINTF("rx_cnt:%d", sdio_dev.rx_cnt);
    rtos_mutex_unlock(sdio_dev.rx_sema);
}

//__INLINE
void aicwf_sdio_txpktcnt_increase(void)
{
    rtos_mutex_lock(sdio_dev.tx_sema, -1);
    sdio_dev.tx_pktcnt++;
    //AIC_LOG_PRINTF("tx_pktcnt: %d", sdio_dev.tx_pktcnt);
    rtos_mutex_unlock(sdio_dev.tx_sema);
}

//__INLINE
void aicfw_sdio_txpktcnt_decrease(void)
{
    rtos_mutex_lock(sdio_dev.tx_sema, -1);
    sdio_dev.tx_pktcnt--;
    //AIC_LOG_PRINTF("tx_pktcnt: %d", sdio_dev.tx_pktcnt);
    rtos_mutex_unlock(sdio_dev.tx_sema);
}

//__INLINE
void aicfw_sdio_txpktcnt_decrease_cnt(uint32_t cnt)
{
    rtos_mutex_lock(sdio_dev.tx_sema, -1);
    sdio_dev.tx_pktcnt -= cnt;
    //AIC_LOG_PRINTF("tx_pktcnt: %d", sdio_dev.tx_pktcnt);
    rtos_mutex_unlock(sdio_dev.tx_sema);
}

//__INLINE
void aicwf_sdio_cmdstatus_set(void)
{
    rtos_mutex_lock(sdio_dev.cmdtx_sema, -1);
    sdio_dev.cmd_txstate = true;
    //AIC_LOG_PRINTF("cmd_txstate: 1");
    rtos_mutex_unlock(sdio_dev.cmdtx_sema);
}

//__INLINE
void aicwf_sdio_cmdstatus_clr(void)
{
    rtos_mutex_lock(sdio_dev.cmdtx_sema, -1);
    sdio_dev.cmd_txstate = false;
    //AIC_LOG_PRINTF("cmd_txstate: 0");
    rtos_mutex_unlock(sdio_dev.cmdtx_sema);
}
#endif /* CONFIG_SDIO_BUS_PWRCTRL */

void sdio_rx_task(void *argv)
{
    int polling = 0;
    int32_t ret = 0;
    bool  poll = true;
    struct rwnx_hw *rwnx_hw = g_rwnx_hw;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    #if (!CONFIG_RXTASK_INSDIO)
    int tout = AIC_RTOS_WAIT_FOREVEVR;//500;
    #ifdef CONFIG_OOB
    if (sdiodev->oob_enable == true)
        poll = false;
    #endif /* CONFIG_OOB */
    #else  /* CONFIG_RXTASK_INSDIO */
    poll = false;
    #endif /* CONFIG_RXTASK_INSDIO */
    do
    {

        #if !CONFIG_RXTASK_INSDIO
        ret = rtos_semaphore_wait(sdiodev->sdio_rx_sema, tout); // polling to avoid irq missing

        #if (DYNAMIC_PRIORITY)
        if (sdio_datrx_priority >= tcpip_priority) {
            rtos_task_set_priority(sdiodev->sdio_task_hdl, tcpip_priority - 1);
        }
        #endif
        #else
		ret = 0;
        #endif /* CONFIG_RXTASK_INSDIO */

        if(ret == 0)
        {
            polling = 0;
        }
        else if (ret == 1)
        {
            polling = 1;
        }
        else
        {
            AIC_LOG_PRINTF("wait sdio_rx_sema fail: ret=%d\n", ret);
        }

        if (msgcfm_poll_en == 0) { // process sdio rx in task
            u8 intstatus = 0;
            u32 data_len;
            int ret = 0, idx, retry_cnt = 0;
            static uint32_t max_data_len = 0;
            if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
                ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                while (ret || (intstatus & SDIO_OTHER_INTERRUPT)) {
                    AIC_LOG_PRINTF("ret=%d, intstatus=%x\n",ret, intstatus);
                     ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    retry_cnt++;
                    if (retry_cnt >= 8) {
                        break;
                    }
                }
            } else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
                if (func_flag_rx)
                    ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                else
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                while (ret || (intstatus & SDIO_OTHER_INTERRUPT)) {
                    AIC_LOG_PRINTF("ret=%d, intstatus=%x\n",ret, intstatus);
                    if (func_flag_rx)
                        ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    else
                        ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.block_cnt_reg, &intstatus);
                    retry_cnt++;
                    if (retry_cnt >= 8) {
                        break;
                    }
                }
            }
            else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
            {
                do {
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.misc_int_status_reg, &intstatus);
                    if (!ret)
                        break;
                    AIC_LOG_PRINTF("%s, ret=%d, intstatus=%x\r\n", __func__, ret, intstatus);
                } while (1);
                if (intstatus & SDIO_OTHER_INTERRUPT) {
                    u8 int_pending;
                    ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.sleep_reg, &int_pending);
                    if (ret < 0) {
                        AIC_LOG_PRINTF("reg:%d read failed!\n", sdiodev->sdio_reg.sleep_reg);
                    }
                    int_pending &= ~0x01; // dev to host soft irq
                    ret = aicwf_sdio_writeb(sdiodev, sdiodev->sdio_reg.sleep_reg, int_pending);
                    if (ret < 0) {
                        AIC_LOG_PRINTF("reg:%d write failed!\n", sdiodev->sdio_reg.sleep_reg);
                    }
                }
            }

            //AIC_LOG_PRINTF("[task] intstatus=%d, retry_cnt=%d\n", intstatus, retry_cnt);
            if ((intstatus > 0) && (retry_cnt < 8)) {
                if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
                    uint8_t intmaskf2 = intstatus | (0x1UL << 3);
                    u8 byte_len = 0;
                    if (intmaskf2 > 120U) { // func2
                        if (intmaskf2 == 127U) { // byte mode
                            AIC_LOG_PRINTF("%s, intmaskf2=%d\r\n", __func__, intmaskf2);
                            ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                            if (ret < 0) {
                                AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                            }
                            AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                            data_len = byte_len * 4; //byte_len must<= 128
                        } else { // block mode
                            data_len = (intstatus & 0x7U) * SDIOWIFI_FUNC_BLOCKSIZE;
                          }
                    } else { // func1
                        if (intstatus == 120U) { // byte mode
                            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                            if (ret < 0) {
                                AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                            }
                            AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                            data_len = byte_len * 4; //byte_len must<= 128
                        } else { // block mode
                            data_len = (intstatus & 0x7FU) * SDIOWIFI_FUNC_BLOCKSIZE;
                        }
                    }
                } else {
                    if (intstatus < 64) {
                        data_len = intstatus * SDIOWIFI_FUNC_BLOCKSIZE;
                    } else {
                        u8 byte_len = 0;

                        if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
                            ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                        } else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
                            if (func_flag_rx)
                                ret = aicwf_sdio_readb_func2(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                            else
                                ret = aicwf_sdio_readb(sdiodev, sdiodev->sdio_reg.bytemode_len_reg, &byte_len);
                        }
                        if (ret < 0) {
                            AIC_LOG_PRINTF("byte mode len read err %d\r\n", ret);
                        }
                        AIC_LOG_PRINTF("byte mode len=%d\r\n", byte_len);
                        data_len = byte_len * 4; //byte_len must<= 128
                    }
                }
                if (data_len) {
                    if (max_data_len < data_len) {
                        max_data_len = data_len;
                        aic_dbg("max_data_len=%d\n", max_data_len);
                    }

                    retry:
                    do {
                        int ret;
                        uint8_t *buf_rx;
                        struct sdio_buf_node_s *node = sdio_buf_alloc(data_len);

                        if (node == NULL) {
                            AIC_LOG_PRINTF("node is null\n");
                            rtos_msleep(3);
                            goto retry;
                        } else {
                            if (node->buf == NULL) {
                                AIC_LOG_PRINTF("node->buf is null, node=%p, len=%d\n", node, data_len);
                                sdio_buf_free(node);
                                node = NULL;
                                rtos_msleep(3);
                                goto retry;
                            }
                            // It`s ready to recv pkt.
                        }

                        buf_rx = node->buf;
                        ret = aicwf_sdio_recv_pkt(buf_rx, data_len, 1);
                        if (ret) {
                            AIC_LOG_PRINTF("recv pkt err %d\n", ret);
                            sdio_buf_free(node);
                            break;
                        }

                        {
                            #ifdef CONFIG_SDIO_BUS_PWRCTRL
                            //AIC_LOG_PRINTF("%s, %d", __func__, __LINE__);
                            aicwf_sdio_rxcnt_increase();
                            #endif
                            //aic_dbg("enq,%p,%d, node:%p\n",buf_rx, data_len, node);
                            fhost_rxframe_enqueue(node);
                            //printf("enq cnt %d\n", co_list_cnt(&fhost_rx_env.rxq.list));
                            rtos_semaphore_signal(fhost_rx_env.rxq_trigg, 0);
                        }
                    } while (0);
                }
            }
            else if (!polling)
            {
                AIC_LOG_PRINTF("Interrupt but no data, intstatus=%d, retry_cnt=%d\n", intstatus, retry_cnt);
            }

        }
        //else
        {
            sdio_host_enable_isr(TRUE); // re-enable after rx done       Need ???
        }

        #if !CONFIG_RXTASK_INSDIO
        if (sdio_datrx_priority >= tcpip_priority) {
            rtos_task_set_priority(sdiodev->sdio_task_hdl, sdio_datrx_priority);
        }
        #endif

    }while(poll);
}

#define CONFIG_TXMSG_TEST_EN 0

#if (CONFIG_TXMSG_TEST_EN)
#include "aic_utils.h"
void aic_txmsg_test(void)
{
    static u8 buffer[64] = {0,};
    static u8 buffer_rx[512] = {0,};
    int err;
    int len = sizeof(struct lmac_msg) + sizeof(struct dbg_mem_read_req);
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    struct dbg_mem_read_cfm *cfm = &rd_mem_addr_cfm;
    struct lmac_msg *msg;
    struct dbg_mem_read_req *req;
    int index = 0;
    AIC_LOG_PRINTF("%s\n", __func__);
    buffer[0] = (len+4) & 0x00ff;
    buffer[1] = ((len+4) >> 8) &0x0f;
    buffer[2] = 0x11;
    if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8801 || g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
        g_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
        buffer[3] = 0x0;
    else if (g_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        buffer[3] = crc8_ponl_107(&buffer[0], 3); // crc8
    index += 4;
    //there is a dummy word
    index += 4;
    msg = (struct lmac_msg *)&buffer[index];
    msg->id = DBG_MEM_READ_REQ;
    msg->dest_id = TASK_DBG;
    msg->src_id = 100;
    msg->param_len = sizeof(struct dbg_mem_read_req);
    req = (struct dbg_mem_read_req *)&msg->param[0];
    req->memaddr = 0x40500000;
    err = aicwf_sdio_tx_msg(buffer, len + 8, buffer_rx);
    if (!err) {
        AIC_LOG_PRINTF("tx msg done\n");
    }
    hexdump(buffer_rx, 64, 1);
}
#endif

int sdio_host_init(uint16_t chipid)
{
    int32_t ret = 0;
    uint8_t in;
    int err_ret = 0;
    int i =0;
    uint8_t block_bit0 = 0x1;
    uint8_t byte_mode_disable = 0x1;//1: no byte mode
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    sdiodev->chipid = chipid;

    aic_sdio_tx_init();
    sdio_host_enable_isr(0);

    if (chipid != PRODUCT_ID_AIC8800D80) {
        ret = aicwf_sdio_func_init(chipid, sdiodev);
    } else {
        ret = aicwf_sdiov3_func_init(chipid, sdiodev);
    }
    if (ret < 0) {
        AIC_LOG_PRINTF("sdio func init fail\n");
        return FALSE;
    }
    AIC_LOG_PRINTF("sdio func init success\n");

    #if 0
    AIC_LOG_PRINTF("write func0 clk phase %d\n", ret);
    sdio_f0_writeb(CONFIG_SDIO_PAHSE, 0x13, &ret);
    if (ret)
    {
        AIC_LOG_PRINTF("write func0 fhase fail %d\n", ret);
        return ret;
    }
    #endif

    aic_sdio_buf_init();

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    ret = aic_sdio_bus_pwrctrl_preinit();
    if (ret) {
        AIC_LOG_PRINTF("sdio_bus_pwrctrl_preinit fail\n");
    }
    #endif /* CONFIG_SDIO_BUS_PWRCTRL */

    #ifdef CONFIG_OOB
    sdiodev->oob_enable = false;
    sdiodev->sdio_gpio_num = GPIO_PIN_53;
    rtos_semaphore_create(&sdiodev->sdio_oobirq_sema, "sdio_oobirq_sema", 8, 0);
    if (sdiodev->sdio_oobirq_sema == NULL) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio_oobirq_sema create fail\n");
        return ret;
    }
    #endif /* CONFIG_OOB */

    #if !CONFIG_RXTASK_INSDIO
    AIC_LOG_PRINTF("create sdio rx sema\n");
    rtos_semaphore_create(&sdiodev->sdio_rx_sema, "sdio_rx_sema", 8, 0);
    if (sdiodev->sdio_rx_sema == NULL) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio rx sema create fail\n");
        return ret;
    }

    AIC_LOG_PRINTF("create sdio rx task\n");
    ret = rtos_task_create(aic_sdio_rx_task, "sdio_rx_task", SDIO_DATRX_TASK,
                           sdio_datrx_stack_size, NULL, sdio_datrx_priority,
                           &sdiodev->sdio_task_hdl);
    if (ret || (sdiodev->sdio_task_hdl == NULL)) {
        AIC_LOG_PRINTF("sdio task create fail\n");
        return ret;
    }
    #endif /* CONFIG_RXTASK_INSDIO */

    ret = 0;
    AIC_LOG_PRINTF("sdio_host_init:host_int ok\n");

#if (CONFIG_TXMSG_TEST_EN)
    aic_txmsg_test();
#endif

    if (sdio_interrupt_init(sdiodev))
    {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio_host_init: sdio_interrupt_init failed\n");
        return ret;
    }
    //sdio_set_function_irq(sdio_function[FUNC_1]->num);
    sdio_host_enable_isr(1);

    return ret;
}

int sdio_host_deinit(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    struct aic_sdio_dev *sdiodev = &sdio_dev;

    AIC_LOG_PRINTF("sdio_host_deinit\n");
    sdio_host_enable_isr(0);

#if !CONFIG_RXTASK_INSDIO
	RTOS_RES_NULL(sdiodev->sdio_task_hdl);
	RTOS_RES_NULL(sdiodev->sdio_rx_sema);
#endif

#ifdef CONFIG_SDIO_BUS_PWRCTRL
    aicwf_sdio_bus_pwrctl_timer_task_deinit();
    if (sdiodev->tx_sema) {
        rtos_mutex_delete(sdiodev->tx_sema);
        sdiodev->tx_sema = NULL;
    }
    if (sdiodev->rx_sema) {
        rtos_mutex_delete(sdiodev->rx_sema);
        sdiodev->rx_sema = NULL;
    }
    if (sdiodev->cmdtx_sema) {
        rtos_mutex_delete(sdiodev->cmdtx_sema);
        sdiodev->cmdtx_sema = NULL;
    }
    if (sdiodev->pwrctl.bus_wakeup_sema) {
        rtos_mutex_delete(sdiodev->pwrctl.bus_wakeup_sema);
        sdiodev->pwrctl.bus_wakeup_sema = NULL;
    }
    if (sdiodev->pwrctl.pwrctl_trgg) {
        rtos_semaphore_delete(sdiodev->pwrctl.pwrctl_trgg);
        sdiodev->pwrctl.pwrctl_trgg = NULL;
    }
#endif

#ifdef CONFIG_OOB
	RTOS_RES_NULL(sdiodev->sdio_oobirq_hdl);
	RTOS_RES_NULL(sdiodev->sdio_oobirq_sema);
#endif

#ifndef	CONFIG_DRIVER_ORM
    extern void aicwf_sdio_tx_deinit(void);
    aicwf_sdio_tx_deinit();
#endif
    if (sdio_rx_buf_list.mutex) {
        rtos_mutex_delete(sdio_rx_buf_list.mutex);
        sdio_rx_buf_list.mutex = NULL;
    }

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        func_flag_tx = true;
        func_flag_rx = true;
    }

    if (sdio_interrupt_deinit(sdiodev)) {
        ret = EREMOTEIO;
        AIC_LOG_PRINTF("sdio_host_deinit: sdio_interrupt_deinit failed\n");
        return ret;
    }

    return ret;
}

void sdio_host_predeinit(void)
{
    sdio_host_enable_isr(0);
    sdio_set_irq_handler(NULL);
    rtos_msleep(10);
}



