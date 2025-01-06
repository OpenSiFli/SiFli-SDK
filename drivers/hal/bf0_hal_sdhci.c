/**
  ******************************************************************************
  * @file   bf0_hal_sdhci.c
  * @author Sifli software development team
  * @brief SD/MMC HCI Controller BSP driver
  This driver is validated by using MSH command 'date'.
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bf0_hal_sdhci.h"
#include "bf0_hal.h"

#ifdef HAL_SDHCI_MODULE_ENABLED

#define HAL_SDHCI_WRITE_LONG(val, reg)          (*((volatile uint32_t *)(reg)) = (val))
#define HAL_SDHCI_WRITE_SHORT(val, reg)         (*((volatile uint16_t *)(reg)) = (val))
#define HAL_SDHCI_WRITE_BYTE(val, reg)          (*((volatile uint8_t *)(reg)) = (val))

#define HAL_SDHCI_READ_LONG(reg)                (*((volatile uint32_t *)(reg)))
#define HAL_SDHCI_READ_SHORT(reg)               (*((volatile uint16_t *)(reg)))
#define HAL_SDHCI_READ_BYTE(reg)                (*((volatile uint8_t *)(reg)))


static inline void hal_sdhci_writel(SDHCI_HandleTypeDef *handle, uint32_t val, int reg)
{
    HAL_SDHCI_WRITE_LONG(val, handle->Instance + reg);
}

static inline void hal_sdhci_writew(SDHCI_HandleTypeDef *handle, uint16_t val, int reg)
{
    HAL_SDHCI_WRITE_SHORT(val, handle->Instance + reg);
}

static inline void hal_sdhci_writeb(SDHCI_HandleTypeDef *handle, uint8_t val, int reg)
{
    HAL_SDHCI_WRITE_BYTE(val, handle->Instance + reg);
}

static inline uint32_t hal_sdhci_readl(SDHCI_HandleTypeDef *handle, int reg)
{
    return HAL_SDHCI_READ_LONG(handle->Instance + reg);
}

static inline uint16_t hal_sdhci_readw(SDHCI_HandleTypeDef *handle, int reg)
{
    return HAL_SDHCI_READ_SHORT(handle->Instance + reg);
}

static inline uint8_t hal_sdhci_readb(SDHCI_HandleTypeDef *handle, int reg)
{
    return HAL_SDHCI_READ_BYTE(handle->Instance + reg);
}

uint16_t hal_sdhci_get_host_version(SDHCI_HandleTypeDef *handle)
{
    return hal_sdhci_readw(handle, SDHCI_HOST_VERSION);
}

uint32_t hal_sdhci_get_host_cap(SDHCI_HandleTypeDef *handle)
{
    return hal_sdhci_readl(handle, SDHCI_CAPABILITIES);
}

static void hal_sdhci_clear_set_irqs(SDHCI_HandleTypeDef *handle, uint32_t clear, uint32_t set)
{
    uint32_t ier;

    ier = hal_sdhci_readl(handle, SDHCI_INT_ENABLE);
    ier &= ~clear;
    ier |= set;
    hal_sdhci_writel(handle, ier, SDHCI_INT_ENABLE);
    hal_sdhci_writel(handle, ier, SDHCI_SIGNAL_ENABLE);
}

void hal_sdhci_clear_mask_irqs(SDHCI_HandleTypeDef *handle, uint32_t set)
{
    uint32_t ier;

    ier = hal_sdhci_readl(handle, SDHCI_INT_ENABLE);
    if (set)
        hal_sdhci_writel(handle, ier, SDHCI_SIGNAL_ENABLE);
    else
        hal_sdhci_writel(handle, 0, SDHCI_SIGNAL_ENABLE);
}


void hal_sdhci_unmask_irqs(SDHCI_HandleTypeDef *handle, uint32_t irqs)
{
    hal_sdhci_clear_set_irqs(handle, 0, irqs);
}

void hal_sdhci_mask_irqs(SDHCI_HandleTypeDef *handle, uint32_t irqs)
{
    hal_sdhci_clear_set_irqs(handle, irqs, 0);
}

uint32_t hal_sdhci_get_int_value(SDHCI_HandleTypeDef *handle)
{
    return hal_sdhci_readl(handle, SDHCI_INT_STATUS);
}

void hal_sdhci_clear_int(SDHCI_HandleTypeDef *handle, uint32_t irq)
{
    hal_sdhci_writel(handle, irq, SDHCI_INT_STATUS);
}

static void hal_sdhci_set_card_detection(SDHCI_HandleTypeDef *handle, bool enable)
{
    uint32_t irqs = SDHCI_INT_CARD_REMOVE | SDHCI_INT_CARD_INSERT;

    if (enable)
        hal_sdhci_unmask_irqs(handle, irqs);
    else
        hal_sdhci_mask_irqs(handle, irqs);
}

void hal_sdhci_enable_card_detection(SDHCI_HandleTypeDef *handle)
{
    hal_sdhci_set_card_detection(handle, true);
}

void hal_sdhci_disable_card_detection(SDHCI_HandleTypeDef *handle)
{
    hal_sdhci_set_card_detection(handle, false);
}

void hal_sdhci_activate_led(SDHCI_HandleTypeDef *handle)
{
    uint8_t ctrl;
    //printk("activate led call\n");
    ctrl = hal_sdhci_readb(handle, SDHCI_HOST_CONTROL);
    ctrl |= SDHCI_CTRL_LED;
    hal_sdhci_writeb(handle, ctrl, SDHCI_HOST_CONTROL);
}

void hal_sdhci_deactivate_led(SDHCI_HandleTypeDef *handle)
{
    uint8_t ctrl;
    //printk("deactivate le call\n");
    ctrl = hal_sdhci_readb(handle, SDHCI_HOST_CONTROL);
    ctrl &= ~SDHCI_CTRL_LED;
    hal_sdhci_writeb(handle, ctrl, SDHCI_HOST_CONTROL);
}

void hal_sdhci_set_timeout(SDHCI_HandleTypeDef *handle, uint8_t timeo)
{
    hal_sdhci_writeb(handle, timeo, SDHCI_TIMEOUT_CONTROL);
}

void hal_sdhic_set_adma_addr(SDHCI_HandleTypeDef *handle, uint32_t addr)
{
    hal_sdhci_writel(handle, addr,    SDHCI_ADMA_ADDRESS);
}

void hal_sdhci_set_dma_addr(SDHCI_HandleTypeDef *handle, uint32_t addr)
{
    hal_sdhci_writel(handle, addr,   SDHCI_DMA_ADDRESS);
}

void hal_sdhci_reset_dma_addr(SDHCI_HandleTypeDef *handle)
{
    uint32_t addr = hal_sdhci_readl(handle, SDHCI_DMA_ADDRESS);
    hal_sdhci_writel(handle, addr,   SDHCI_DMA_ADDRESS);    // ???
}

void hal_sdhci_set_dma_mode(SDHCI_HandleTypeDef *handle, uint8_t flag)
{
    uint8_t ctrl;
    ctrl = hal_sdhci_readb(handle, SDHCI_HOST_CONTROL);
    ctrl &= ~SDHCI_CTRL_DMA_MASK;
    if ((flag & SDHCI_REQ_USE_DMA) && (flag & SDHCI_USE_ADMA))
        ctrl |= SDHCI_CTRL_ADMA32;
    else
        ctrl |= SDHCI_CTRL_SDMA;

    hal_sdhci_writeb(handle, ctrl, SDHCI_HOST_CONTROL);
}

void hal_sdhci_set_transfer_mode(SDHCI_HandleTypeDef *handle, uint16_t mode)
{
    uint16_t value = (uint16_t)mode;
    if (handle->Init.flags & SDHCI_REQ_USE_DMA)
        value |= SDHCI_TRNS_DMA;
    hal_sdhci_writew(handle, value, SDHCI_TRANSFER_MODE);
}

void hal_sdhci_set_boot_mode(SDHCI_HandleTypeDef *handle, uint16_t boot)
{
    uint16_t value = hal_sdhci_readw(handle, SDHCI_TRANSFER_MODE);
    if (boot)
        value |= (1 << 15);
    else
        value &= ~(1 << 15);
    hal_sdhci_writew(handle, value, SDHCI_TRANSFER_MODE);
}


void hal_sdhci_set_blk(SDHCI_HandleTypeDef *handle, uint32_t blk_size, uint32_t blk_cnt)
{
    hal_sdhci_writew(handle, SDHCI_MAKE_BLKSZ(7, blk_size), SDHCI_BLOCK_SIZE);
    hal_sdhci_writew(handle, blk_cnt, SDHCI_BLOCK_COUNT);
}

uint32_t hal_sdhci_get_present_state(SDHCI_HandleTypeDef *handle)
{
    return hal_sdhci_readl(handle, SDHCI_PRESENT_STATE);
}

void hal_sdhci_set_bus_width(SDHCI_HandleTypeDef *handle, uint8_t width)
{
    uint8_t ctrl;

    ctrl = hal_sdhci_readb(handle, SDHCI_HOST_CONTROL);

    ctrl &= ~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);

    if (width == 4)
        ctrl |= SDHCI_CTRL_4BITBUS;
    else if (width == 8)
        ctrl |= SDHCI_CTRL_8BITBUS;

    hal_sdhci_writeb(handle, ctrl, SDHCI_HOST_CONTROL);
}

void hal_sdhci_send_command(SDHCI_HandleTypeDef *handle, SDHCI_CmdArgTypeDef *cmd)
{

    hal_sdhci_writel(handle, cmd->Arg, SDHCI_ARGUMENT);

    hal_sdhci_writew(handle, SDHCI_MAKE_CMD(cmd->CmdIndex, cmd->RespType), SDHCI_COMMAND);
}

uint32_t hal_sdhci_get_response(SDHCI_HandleTypeDef *handle, int res_id)
{
    return hal_sdhci_readl(handle, SDHCI_RESPONSE + res_id * 4);
}
static void hal_sdhci_read_block_pio(SDHCI_HandleTypeDef *handle, uint8_t *data, uint32_t size)
{
    uint32_t blksize, chunk;
    uint32_t scratch;
    uint8_t *buf;

    blksize = size;
    chunk = 0;
    buf = (uint8_t *)data;

    while (blksize)
    {

        if (chunk == 0)
        {
            scratch = hal_sdhci_readl(handle, SDHCI_BUFFER);
            chunk = 4;

        }

        *buf = scratch & 0xFF;
        buf++;
        scratch >>= 8;
        chunk--;
        blksize--;
    }

}

static void hal_sdhci_write_block_pio(SDHCI_HandleTypeDef *handle, uint8_t *data, uint32_t size)
{
    uint32_t blksize,  chunk;
    uint32_t scratch;
    uint8_t *buf;

    blksize = size;
    buf = (uint8_t *)data;
    chunk = 0;
    scratch = 0;

    while (blksize)
    {
        scratch |= (uint32_t) * buf << (chunk * 8);
        buf++;
        chunk++;
        blksize--;

        if ((chunk == 4) || (blksize == 0))
        {
            hal_sdhci_writel(handle, scratch, SDHCI_BUFFER);
            chunk = 0;
            scratch = 0;
        }
    }

}

void hal_sdhci_transfer_pio(SDHCI_HandleTypeDef *handle, uint8_t is_read, uint8_t *data, uint32_t size)
{
    uint32_t mask;

    if (handle->Init.blocks == 0)
        return;

    if (is_read)
        mask = SDHCI_DATA_AVAILABLE;
    else
        mask = SDHCI_SPACE_AVAILABLE;
    /*
     * Some controllers (JMicron JMB38x) mess up the buffer bits
     * for transfers < 4 bytes. As long as it is just one block,
     * we can ignore the bits.
     */
    //mask = ~0;

    while (hal_sdhci_readl(handle, SDHCI_PRESENT_STATE) & mask)
    {
        if (is_read)
            hal_sdhci_read_block_pio(handle, data, size);
        else
            hal_sdhci_write_block_pio(handle, data, size);

        handle->Init.blocks--;
        if (handle->Init.blocks == 0)
            break;
    }

}

static void hal_sdhci_set_adma_desc(uint8_t *desc, uint32_t addr, int len, unsigned char cmd)
{
#if 1
    hal_sdhci_adma_des_line_t *table = (hal_sdhci_adma_des_line_t *)desc;
    table->addr = addr;
    table->len = len & 0xffff; //max 65536, set to 0
    table->reserved = 0;
    table->attr = cmd & 0xff;
#else
    __le32 *dataddr = (__le32 __force *)(desc + 4);
    __le16 *cmdlen = (__le16 __force *)desc;

    /* SDHCI specification says ADMA descriptors should be 4 byte
     * aligned, so using 16 or 32bit operations should be safe. */

    cmdlen[0] = cpu_to_le16(cmd);
    cmdlen[1] = cpu_to_le16(len);

    dataddr[0] = cpu_to_le32(addr);
#endif
}

int hal_sdhci_adma_table_pre(SDHCI_HandleTypeDef *handle,
                             uint8_t *data, uint32_t size)
{
    uint8_t *desc;
    uint32_t addr, buffer;
    uint32_t len, offset, remain;

    desc = handle->Init.adma_desc;
    remain = size;
    offset = 0;
    buffer = (uint32_t)data;

    while (offset < size)
    {
        addr = buffer + offset;
        len = remain > SDHCI_ADMA_MAX_SIZE ? SDHCI_ADMA_MAX_SIZE : remain;

        /* tran, valid */
        hal_sdhci_set_adma_desc(desc, addr, len, 0x21);
        desc += 8;
        offset += len; //(len - offset) > SDHCI_ADMA_MAX_SIZE ? SDHCI_ADMA_MAX_SIZE : (len - offset);
        remain -= len;

        /*
         * If this triggers then we have a calculation bug
         * somewhere. :/
         */
        //ASSERT((desc - handle->adma_desc) < (128 * 2 + 1) * 4);
    }

    if (0) //(SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC)
    {
        /*
         * Mark the last descriptor as the terminating descriptor
         */
        if (desc != handle->Init.adma_desc)
        {
            desc -= 8;
            desc[0] |= 0x2; /* end */
        }
    }
    else
    {
        /*
         * Add a terminating entry.
         */

        /* nop, end, valid */
        hal_sdhci_set_adma_desc(desc, 0, 0, 0x3);
    }

    return 0;
}

void hal_sdhci_set_transfer_irqs(SDHCI_HandleTypeDef *handle)
{
    uint32_t pio_irqs = SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL;
    uint32_t dma_irqs = SDHCI_INT_DMA_END | SDHCI_INT_ADMA_ERROR;

    if (handle->Init.flags & SDHCI_REQ_USE_DMA)
        hal_sdhci_clear_set_irqs(handle, pio_irqs, dma_irqs);
    else
        hal_sdhci_clear_set_irqs(handle, dma_irqs, pio_irqs);
}

#define MMC_VDD_165_195     0x00000080  /* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21       0x00000100  /* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22       0x00000200  /* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23       0x00000400  /* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24       0x00000800  /* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25       0x00001000  /* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26       0x00002000  /* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27       0x00004000  /* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28       0x00008000  /* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29       0x00010000  /* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30       0x00020000  /* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31       0x00040000  /* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32       0x00080000  /* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33       0x00100000  /* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34       0x00200000  /* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35       0x00400000  /* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36       0x00800000  /* VDD voltage 3.5 ~ 3.6 */

void hal_sdhci_set_power(SDHCI_HandleTypeDef *handle, unsigned short power)
{
    uint8_t pwr;
    int odmode = 0;

    if (power == (unsigned short) -1)
        pwr = 0;
    else
    {
        odmode = (power & 0x100) >> 8;
        power &= 0xff;
        switch (1 << power)
        {
        case MMC_VDD_165_195:
            pwr = SDHCI_POWER_180;
            break;
        case MMC_VDD_29_30:
        case MMC_VDD_30_31:
            pwr = SDHCI_POWER_300;
            break;
        case MMC_VDD_32_33:
        case MMC_VDD_33_34:
            pwr = SDHCI_POWER_330;
            break;
        default:
            pwr = SDHCI_POWER_330;
        }
    }

    //if (handle->Init.pwr == pwr && odmode == 0)
    //    return;

    handle->Init.pwr = pwr;

    /*
     * Spec says that we should clear the power reg before setting
     * a new value. Some controllers don't seem to like this though.
     */
    if (pwr == 0)
    {
        hal_sdhci_writeb(handle, 0, SDHCI_POWER_CONTROL);
        return;
    }

    /*
     * At least the Marvell CaFe chip gets confused if we set the voltage
     * and set turn on power at the same time, so set the voltage first.
     */
    // Only PPMODE bit can work, others bit do not work !!!
    //pwr |= SDHCI_POWER_ON;
    if (odmode) // open drain mode, only for emmc identify
        pwr = 0x00;
    else
        pwr = 0x10;

    hal_sdhci_writeb(handle, pwr, SDHCI_POWER_CONTROL);
    // TODO: set to 0x30 to meet FPGA test
    //hal_sdhci_writeb(handle, 0x30, SDHCI_POWER_CONTROL);

    // for EMMC, set open drain before detect
    // for EMMC, set to push pull after detect
    //hal_sdhci_writeb(handle, 0x00, SDHCI_POWER_CONTROL);    // open drain
    //hal_sdhci_writeb(handle, 0x10, SDHCI_POWER_CONTROL);    // push pull

}

void hal_sdhci_set_clk(SDHCI_HandleTypeDef *handle, uint32_t clock, uint32_t max_clk)
{
    int div, div1;
    int version;
    uint16_t clk = 0;
    unsigned long timeout;

    version = hal_sdhci_get_host_version(handle);//sdhci_readw(host, SDHCI_HOST_VERSION);
    version = version & 0xFF;
    //checking the version

    hal_sdhci_writew(handle, 0, SDHCI_CLOCK_CONTROL);
    if (clock == 0)
        return;
    if ((version == 0x00) || (version == 0x01))
    {
        for (div = 1; div < 256; div *= 2)
        {
            if ((max_clk / div) <= clock)
                break;
        }
        div >>= 1;
        clk = div << SDHCI_DIVIDER_SHIFT;
        clk |= SDHCI_CLOCK_INT_EN;
    }
    else
    {
        if (version == 0x02)
        {
            for (div = 1; div <= 1023; div++)
            {
                if ((max_clk / (div * 2)) <= clock)
                    break;
            }
            //div >>= 1;
            //div = div << 0x5;

            if (div >= 0x400)   // max 10 bits
                div = 0x3ff;
            div1 = div >> 0x08;
            //div = div & 0xFF3F;
            div = (div << 8) & 0xFFFF;
            clk = div | (div1 << 6);

            // todo : for fpga, fix to 0x80
            //clk = 0x20 << 8;
            clk |= SDHCI_CLOCK_INT_EN;
        }
    }


    hal_sdhci_writew(handle, clk, SDHCI_CLOCK_CONTROL);
    /* Wait max 20 ms */
    timeout = 20;
    while (!((clk = hal_sdhci_readw(handle, SDHCI_CLOCK_CONTROL))
             & SDHCI_CLOCK_INT_STABLE))
    {
        if (timeout == 0)
        {
            //printk(KERN_ERR "%s: Internal clock never "
            //      "stabilised.\n", mmc_hostname(host->mmc));
            break;
        }
        timeout--;
        HAL_Delay(1);
    }


    clk |= SDHCI_CLOCK_CARD_EN;

    hal_sdhci_writew(handle, clk, SDHCI_CLOCK_CONTROL);

}

void hal_sdhci_set_ddr(SDHCI_HandleTypeDef *handle, uint8_t ddr)
{
    if (ddr == 1)
    {
        hal_sdhci_writew(handle, (1 << 3) | 4, SDHCI_HOST_CONTROL2);
#ifdef SF32LB55X
        hal_sdhci_writeb(handle, 0x1e, SDHCI_CLK_TUNE);
#else
        hal_sdhci_writeb(handle, 0x25, SDHCI_CLK_TUNE);
#endif
    }
    else
    {
        // for 1.8 V high speed
        //hal_sdhci_writew(handle, (1 << 3) | 3, SDHCI_HOST_CONTROL2);
        // for 3.3 or low speed ?
        hal_sdhci_writew(handle, (1 << 3) | 0, SDHCI_HOST_CONTROL2);
        hal_sdhci_writeb(handle, 0x15, SDHCI_CLK_TUNE);
    }
}

void hal_sdhci_reset(SDHCI_HandleTypeDef *handle, uint8_t mask)
{
    unsigned long timeout;
    uint32_t ier;

    ier = hal_sdhci_readl(handle, SDHCI_INT_ENABLE);

    hal_sdhci_writeb(handle, mask, SDHCI_SOFTWARE_RESET);

    /* Wait max 100 ms */
    timeout = 100;

    /* hw clears the bit when it's done */
    while (hal_sdhci_readb(handle, SDHCI_SOFTWARE_RESET) & mask)
    {
        if (timeout == 0)
        {
            return;
        }
        timeout--;
        HAL_Delay(1);
    }

    hal_sdhci_clear_set_irqs(handle, SDHCI_INT_ALL_MASK, ier);

    // enable all inc to meet ctest
    hal_sdhci_writel(handle, 0xf, 0xec);

    hal_sdhci_set_ddr(handle, 0);   // recover to sdr mode for some cases
}


void hal_sdhci_init(SDHCI_HandleTypeDef *handle, int soft)
{
    if (soft)
        hal_sdhci_reset(handle, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
    else
        hal_sdhci_reset(handle, SDHCI_RESET_ALL);

    hal_sdhci_clear_set_irqs(handle, SDHCI_INT_ALL_MASK,
                             SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END_BIT |
                             SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
                             SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
                             SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE | SDHCI_TUNING_EVENT |
                             SDHCI_TUNING_ERROR | eMMC_BOOT_ACK_INT | eMMC_BOOT_DONE_INT | SDHCI_INT_CARD_INT);

}

void hal_sdhci_reinit(SDHCI_HandleTypeDef *handle)
{
    hal_sdhci_init(handle, 0);
    //hal_sdhci_enable_card_detection(handle);
}

int hal_sdhci_command_finish(SDHCI_HandleTypeDef *handle, uint32_t timeout)
{
    uint32_t intmask, cnt;

    cnt = timeout;
    while (cnt > 0)
    {
        intmask = hal_sdhci_get_int_value(handle);

        if ((intmask & SDHCI_INT_ERROR)  || (intmask & SDHCI_INT_CMD_MASK))
        {
            break;
        }
        HAL_Delay(1);
        cnt --;
    }
    //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
    if ((intmask & SDHCI_INT_ERROR) || (cnt <= 0))
        return -1;

    return intmask;
}

int hal_sdhci_data_finish(SDHCI_HandleTypeDef *handle, uint32_t timeout)
{
    uint32_t intmask, cnt;

    cnt = timeout;
    while (cnt > 0)
    {
        intmask = hal_sdhci_get_int_value(handle);

        if ((intmask & SDHCI_INT_ERROR) || (intmask & SDHCI_INT_DATA_MASK))
        {
            break;
        }
        HAL_Delay(1);
        cnt --;
    }
    //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_DATA_MASK);
    if ((intmask & SDHCI_INT_ERROR) || (cnt <= 0))
        return -1;

    return intmask;
}

uint32_t hal_sdhci_read_fifo(SDHCI_HandleTypeDef *handle)
{
    return hal_sdhci_readl(handle, SDHCI_BUFFER);
}

void hal_sdhci_write_fifo(SDHCI_HandleTypeDef *handle, uint32_t value)
{
    hal_sdhci_writel(handle, SDHCI_BUFFER, value);
}

__weak void HAL_SDHCI_MspInit(SDHCI_HandleTypeDef *handle)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(handle);
}
#endif /* HAL_SDHCI_MODULE_ENABLED */
