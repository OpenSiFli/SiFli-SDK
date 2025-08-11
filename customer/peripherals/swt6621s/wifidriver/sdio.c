/** @file sdio.c
 *
 *  @brief  This file provides sdio related Generic API
 *
 *  Copyright 2021-2022 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __ZEPHYR__
    #include <wmerrno.h>
    #include <wm_utils.h>
#endif

#include <osa.h>
//#include <mlan_sdio_api.h>


/* Additional WMSDK header files */
#include "mlan_main_defs.h"
#include "mlan_sdio_defs.h"
#include "type_decls.h"
#ifndef __ZEPHYR__
    // #include "fsl_sdmmc_common.h"
    // #include "fsl_sdmmc_host.h"
    // #include "sdmmc_config.h"
#endif
//#include "fsl_common.h"
#include "sdio.h"
#include "firmware_dnld.h"

/*
 * Used to authorize the SDIO interrupt handler to accept the incoming
 * packet from the SDIO interface. If this flag is set a semaphore is
 * signalled.
 */

/* @brief decription about the read/write buffer
 * The size of the read/write buffer should be a multiple of 512, since SDHC/SDXC card uses 512-byte fixed
 * block length and this driver example is enabled with a SDHC/SDXC card.If you are using a SDSC card, you
 * can define the block length by yourself if the card supports partial access.
 * The address of the read/write buffer should align to the specific DMA data buffer address align value if
 * DMA transfer is used, otherwise the buffer address is not important.
 * At the same time buffer address/size should be aligned to the cache line size if cache is supported.
 */

#ifdef __ZEPHYR__
    #define BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE 32
#endif
/*! @brief Data written to the card */
#if CONFIG_SDIO_MULTI_PORT_TX_AGGR
    SDK_ALIGN(uint8_t outbuf[SDIO_MP_AGGR_DEF_PKT_LIMIT * 2 * DATA_BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
#else
    SDK_ALIGN(uint8_t outbuf[DATA_BUFFER_SIZE + DATA_BUFFER_SIZE / 2], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
#endif

/*! @brief Data read from the card */
#if (CONFIG_SDIO_MULTI_PORT_RX_AGGR) && !(FSL_USDHC_ENABLE_SCATTER_GATHER_TRANSFER)
    SDK_ALIGN(uint8_t inbuf[SDIO_MP_AGGR_DEF_PKT_LIMIT * 2 * DATA_BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
#else
    SDK_ALIGN(uint8_t inbuf[2 * DATA_BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
#endif /*CONFIG_SDIO_MULTI_PORT_RX_AGGR*/

t_u32 ioport_g = 0;

/**
 * Function to set mlan ioport.
 * A weak definition of this is added here for compilation of
 * bt ble apps/non wifi apps
 * This funciton is defined in wifi-sdio.c as of this writing
 * for wifi files.
 */
WEAK void set_ioport_inmlan(t_u32 port)
{
    return;
}

unsigned int wifi_get_sdio_ioport()
{
    return ioport_g;
}
uint8_t *wifi_get_sdio_outbuf(uint32_t *outbuf_len)
{
    *outbuf_len = sizeof(outbuf);
    return outbuf;
}

/**
 *  @brief This function reads the CARD_TO_HOST_EVENT_REG and
 *  checks if input bits are set
 *  @param bits     bits to check status against
 *  @return     true if bits are set
 *                      SDIO_POLLING_STATUS_TIMEOUT if bits
 *                      aren't set
 */
bool wlan_card_status(t_u8 bits)
{
    uint32_t resp = 0;
    t_u32 tries;

    for (tries = 0; tries < MAX_POLL_TRIES; tries++)
    {
        if (!(sdio_drv_creg_read(CARD_TO_HOST_EVENT_REG, 1, &resp)))
        {
            return false;
        }
        if ((resp & bits) == bits)
        {
            return true;
        }
        OSA_TimeDelay(1U);
    }
    return false;
}

#define SDIO_BLOCK_SIZE 256U

void calculate_sdio_write_params(t_u32 txlen, t_u32 *tx_blocks, t_u32 *buflen)
{
    *tx_blocks = (txlen + SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE;

    *buflen = SDIO_BLOCK_SIZE;
}

static uint32_t wlan_card_read_scratch_reg(void)
{
    uint32_t val    = 0;
    uint32_t rd_len = 0;

    (void)sdio_drv_creg_read(0x64, 1, &val);
    rd_len = (val & 0xffU);
    (void)sdio_drv_creg_read(0x65, 1, &val);
    rd_len |= ((val & 0xffU) << 8);
    (void)sdio_drv_creg_read(0x66, 1, &val);
    rd_len |= ((val & 0xffU) << 16);
    (void)sdio_drv_creg_read(0x67, 1, &val);
    rd_len |= ((val & 0xffU) << 24);

    return rd_len;
}

static void wlan_sdio_init_ioport(void)
{
    uint32_t resp = 0;
    t_u8 data;

#if defined(SD8978) || defined(SD8987) || defined(SD8997) || defined(SD9097) || defined(SD9098) || defined(SD9177)
    ioport_g = MEM_PORT;

    sdio_io_d("IOPORT : (0x%x)", ioport_g);

    (void)sdio_drv_creg_write(HOST_INT_MASK_REG, 1, 0x0, &resp);

    /* Enable sdio cmd53 new mode */
    (void)sdio_drv_creg_read(CARD_CONFIG_2_1_REG, 1, &resp);
    data = (t_u8)((resp & 0xff) | CMD53_NEW_MODE);
    (void)sdio_drv_creg_write(CARD_CONFIG_2_1_REG, 1, data, &resp);
    (void)sdio_drv_creg_read(CARD_CONFIG_2_1_REG, 1, &resp);

    /* configure cmd port  */
    /* enable reading rx length from the register  */
    (void)sdio_drv_creg_read(CMD_CONFIG_0, 1, &resp);
    data = (t_u8)((resp & 0xff) | CMD_PORT_RD_LEN_EN);
    (void)sdio_drv_creg_write(CMD_CONFIG_0, 1, data, &resp);
    (void)sdio_drv_creg_read(CMD_CONFIG_0, 1, &resp);

    /* enable Dnld/Upld ready auto reset for cmd port
     * after cmd53 is completed */
    (void)sdio_drv_creg_read(CMD_CONFIG_1, 1, &resp);
    data = (t_u8)((resp & 0xff) | CMD_PORT_AUTO_EN);
    (void)sdio_drv_creg_write(CMD_CONFIG_1, 1, data, &resp);
    (void)sdio_drv_creg_read(CMD_CONFIG_1, 1, &resp);
#elif defined(SD8801)
    sdio_drv_creg_write(HOST_INT_MASK_REG, 1, 0x0, &resp);

    /* Read the PORT regs for IOPORT address */
    sdio_drv_creg_read(IO_PORT_0_REG, 1, &resp);
    ioport_g = (resp & 0xff);

    sdio_drv_creg_read(IO_PORT_1_REG, 1, &resp);
    ioport_g |= ((resp & 0xff) << 8);

    sdio_drv_creg_read(IO_PORT_2_REG, 1, &resp);
    ioport_g |= ((resp & 0xff) << 16);

    sdio_io_d("IOPORT : (0x%x)", ioport_g);
#endif

    /* Set Host interrupt reset to read to clear */
    (void)sdio_drv_creg_read(HOST_INT_RSR_REG, 1, &resp);
    data = (t_u8)((resp & 0xff) | HOST_INT_RSR_MASK);
    (void)sdio_drv_creg_write(HOST_INT_RSR_REG, 1, data, &resp);

    /* Dnld/Upld ready set to auto reset */
    (void)sdio_drv_creg_read(CARD_MISC_CFG_REG, 1, &resp);
    data = (t_u8)((resp & 0xff) | AUTO_RE_ENABLE_INT);
    (void)sdio_drv_creg_write(CARD_MISC_CFG_REG, 1, data, &resp);
    set_ioport_inmlan(ioport_g);
}

t_u16 wlan_card_read_f1_base_regs(void)
{
    t_u16 reg;
    uint32_t resp = 0;

    (void)sdio_drv_creg_read(READ_BASE_0_REG, 1, &resp);
    reg = (t_u16)(resp & 0xffU);
    (void)sdio_drv_creg_read(READ_BASE_1_REG, 1, &resp);
    reg |= (t_u16)((resp & 0xffU) << 8);

    return reg;
}

int sdio_init(void)
{
    uint32_t resp = 0;
    /* Initialize SDIO driver */
    int rv = sdio_drv_init(NULL);
    if (rv != WM_SUCCESS)
    {
        sdio_io_e("SDIO driver init failed.");
        return -1;
    }

#if 0
    sdio_drv = sdio_drv_open("MDEV_SDIO");
    if (!sdio_drv)
    {
        sdio_io_e("SDIO driver open failed.");
        return -1;
    }
#endif
    int ret = 0;
    bool wlan_card_stat;
    ret = sdio_drv_creg_read(CARD_TO_HOST_EVENT_REG, 1, &resp);
    if (ret && (resp & (DN_LD_CARD_RDY)) == 0U)
    {
        wlan_card_stat = wlan_card_status(UP_LD_CARD_RDY);
        if (wlan_card_stat != false)
        {
            uint32_t rd_len;
            rd_len = wlan_card_read_scratch_reg();
            if (rd_len > 0U)
            {
                (void)sdio_drv_creg_write(FN1_BLOCK_SIZE_0, 0, 0x8, &resp);
                (void)sdio_drv_creg_write(FN1_BLOCK_SIZE_1, 0, 0x0, &resp);

                uint8_t buf[256];
                ret = sdio_drv_read(0x10000, 1, rd_len, 8, buf, &resp);
                if (!ret)
                {
                    sdio_io_e(
                        "SDIO read failed, "
                        "resp:%x",
                        resp);
                    return -1;
                }
            }
        }
    }
    else if (!ret)
    {
        sdio_io_e("failed to read EVENT_REG");
        return -1;
    }
    else
    {
        /* Do Nothing */
    }
    return 0;
}

int sdio_ioport_init(void)
{
    /* this sets intmask on card and makes interrupts repeatable */
    wlan_sdio_init_ioport();

    return 0;
}

/**
 * Interrupt callback handler registered with the SDIO driver.
 * A weak definition of this is added here for compilation of
 * bt ble apps/non wifi apps
 * This funciton is defined in wifi-sdio.c as of this writing
 * for wifi files.
 */
WEAK void handle_cdint(int error)
{
    return;
}
