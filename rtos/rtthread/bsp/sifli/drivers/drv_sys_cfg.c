/**
  ******************************************************************************
  * @file   drv_sys_cfg.c
  * @author Sifli software development team
  * @brief System configure setting in BSP driver
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
#include "board.h"

/** @addtogroup bsp_driver Driver IO
  * @{
  */


#if defined (BSP_USING_SPI_FLASH) && defined(BF0_HCPU)
#include "drv_flash.h"
#include "string.h"
#include "drv_io.h"

#include <stdlib.h>
#define CFG_NAND_FACTORY_MAGIC 0x53450617

#define CFG_SUPPORT_NAND_OTP        (0)

#include "bf0_sys_cfg.h"
#define CFG_IN_FLASH_OTP
#define CFG_OTP_OFFSET          (0)

extern void rt_flash_lock(uint32_t addr);
extern void rt_flash_unlock(uint32_t addr);
extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
extern void rt_flash_switch_dtr(uint32_t addr, uint8_t dtr_en);

#if CFG_SUPPORT_NAND_OTP
    extern void *rt_nand_get_handle(uint32_t addr);

    static uint8_t g_sys_otp_start = 0;
    static uint8_t g_sys_otp_cnt = 0;
#endif

/**
  * @brief Check if it's chip based configure, for saved in CFG_IN_OTP_PAGE.
  * @retval 1 if it is onchip configure, 0 for user configure.
  */
static int is_onchip_cfg(uint8_t id)
{
    if (((id >= FACTORY_CFG_ID_ADC) && (id <= FACTORY_CFG_ID_VBUCK)) || (id == FACTORY_CFG_ID_SIPMODE))
        return 1;

    return 0;
}

/**
  * @brief Check if it's cfg save on flash area, or on otp.
  * @retval 1 if it is on flash configure, 0 for otp configure.
  */
static int is_onflash_cfg(uint8_t id)
{

#if defined(CFG_SUPPORT_NON_OTP)
    return 1;
#endif

    return 0;  //all chip(include SF32LB55X) factory data save in otp
#if 0
#ifndef SF32LB55X
    return 0; // for pro and later version, configure do not save to flash memory
#endif
    if ((id == FACTORY_CFG_ID_MAC) ||
            (id == FACTORY_CFG_ID_SN) ||
            (id == FACTORY_CFG_ID_LOCALNAME) ||
            (id == FACTORY_CFG_ID_THIRDFUNC) ||
            (id == FACTORY_CFG_ID_CTEI) ||
            ((id >= FACTORY_CFG_ID_ALIPAY_PK) && (id <= FACTORY_CFG_ID_BTNAME)) ||
            ((id >= FACTORY_CFG_ID_GOMORE) && (id <= FACTORY_CFG_ID_USERK3))
       )
    {
        return 1;
    }

    return 0;
#endif
}

/**
  * @brief Initialize ATE configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_chip_config_init(void)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int res;
    uint32_t cfg_base = BSP_GetOtpBase();

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    int level = rt_hw_interrupt_disable();
    //rt_flash_lock(cfg_base);
    res = HAL_QSPI_ERASE_OTP(fhandle, CFG_IN_OTP_PAGE << 12);
    rt_hw_interrupt_enable(level);
    //rt_flash_unlock(cfg_base);

    if (res == 0)
        return RT_EOK;
#endif
    return RT_ERROR;
}

/**
  * @brief Initialize factory configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_user_config_init(void)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int res;
    uint32_t cfg_base = BSP_GetOtpBase();

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    int level = rt_hw_interrupt_disable();
    //rt_flash_lock(cfg_base);
    res = HAL_QSPI_ERASE_OTP(fhandle, CFG_USER_OTP_PAGE << 12);
    rt_hw_interrupt_enable(level);
    //rt_flash_unlock(cfg_base);

    if (res == 0)
        return RT_EOK;
#endif
    return RT_ERROR;
}

/**
  * @brief Initialize customer configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_cust_config_init(void)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int res;
    uint32_t cfg_base = BSP_GetOtpBase();

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    int level = rt_hw_interrupt_disable();
    //rt_flash_lock(cfg_base);
    res = HAL_QSPI_ERASE_OTP(fhandle, CFG_CUST_OTP_PAGE << 12);
    rt_hw_interrupt_enable(level);
    //rt_flash_unlock(cfg_base);

    if (res == 0)
        return RT_EOK;
#endif
    return RT_ERROR;
}


/**
  * @brief Initialize flash configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_flash_config_init(void)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int res;
    uint32_t cfg_base = BSP_GetOtpBase();

    res = rt_flash_erase(cfg_base, SYSCFG_FACTORY_SIZE);

    if (res == 0)
        return RT_EOK;
#endif
    return RT_ERROR;
}

/**
  * @brief Read factory configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_flash_config_read(uint8_t id, uint8_t *data, uint8_t size)
{
    int i = 0;
    int len = 0;
    int retry = 0;
    uint32_t fac_cfg_size = 0;
    uint8_t *p;
    int res;
    int level;
    uint8_t *buf = NULL;

    uint32_t cfg_base = BSP_GetOtpBase();
#if !defined(CFG_SUPPORT_NON_OTP)
    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
    {
        if (is_onchip_cfg(id) || is_onflash_cfg(id)) // for ate and flash cfg, do not need check cache
            return 0;

        // if flash not enable, fhandle should be null, it may load to cache at initial for user cfg
        p = (uint8_t *)BSP_Get_UserOTP_Cache();
        if (p != NULL)
        {
            retry = 0;
            fac_cfg_size = 512; //CFG_USER_SIZE;
            goto cfg_read;
        }
        return 0;
    }
    buf = malloc(fhandle->ctable->oob_size * 256);
    if (buf == NULL)
    {
        //rt_kprintf("malloc fail\n");
        return 0;
    }

    if (is_onchip_cfg(id))  // for onchip config, check flash address again
        retry = 1;
    else        // for user configure, check user page + flash address
        retry = 2;

    if (is_onflash_cfg(id))
        retry = 0;

    int to_val = HAL_FLASH_GET_WDT_VALUE(fhandle);
    HAL_FLASH_SET_WDT(fhandle, 0);

    if (retry > 0)  // cfg on otp
    {
        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, CFG_IN_OTP_PAGE << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            free(buf);
            HAL_FLASH_SET_WDT(fhandle, to_val);
            return 0;
        }

        //rt_kprintf("read otp res %d\n", res);
        fac_cfg_size = fhandle->ctable->oob_size * 256 - CFG_OTP_OFFSET;
        p = buf + CFG_OTP_OFFSET;
    }
    else // cfg on flash
    {
        p = (uint8_t *)cfg_base;
        fac_cfg_size = SYSCFG_FACTORY_SIZE;
    }
#else
    FLASH_HandleTypeDef *fhandle  = rt_nand_get_handle(cfg_base);
    if (fhandle == NULL)
    {
        rt_kprintf("rt_flash_config_read addr: 0x%x find handle error\n", cfg_base);
        return 0;
    }

    uint32_t *pBuf;
    int block_size = HAL_NAND_BLOCK_SIZE(fhandle);
    int page_size = HAL_NAND_PAGE_SIZE(fhandle);

    buf = malloc(page_size);
    //rt_kprintf("+-+- 0x%x %d -+-+\n", cfg_base, fhandle->ctable->oob_size);
    if (buf == NULL)
    {
        rt_kprintf("rt_flash_config_read malloc 0x%x fail\n", page_size);
        return 0;
    }

    //level = rt_hw_interrupt_disable();
    //rt_flash_lock(cfg_base);

    res = rt_nand_read_page(cfg_base + block_size * 2, (uint8_t *)buf, page_size, NULL, 0);
    if (res != page_size)
    {
        rt_kprintf("rt_nand_read_page len error: 0x%x vs 0x%x\n", res, page_size);
        free(buf);
        return 0;
    }
    pBuf = (uint32_t *)buf;
    if (pBuf[0] != CFG_NAND_FACTORY_MAGIC)
    {
        rt_kprintf("NAND factory magic error 0x%x vs 0x%x\n", pBuf[0], CFG_NAND_FACTORY_MAGIC);
        free(buf);
        return 0;
    }

    for (int m = block_size / page_size - 1; m > 0; m--)
    {
        res = rt_nand_read_page(cfg_base + block_size * 2 + m * page_size, (uint8_t *)buf, page_size, NULL, 0);
        if (res != page_size || buf[0] != 0xff)
        {
            break;
        }
    }
    //rt_hw_interrupt_enable(level);
    //rt_flash_unlock(cfg_base);
    if (res != page_size)
    {
        rt_kprintf("NAND read len error 0x%x vs 0x%x\n", res, page_size);
        free(buf);
        return 0;
    }

    if (buf[0] == 0xff)
    {
        rt_kprintf("no factory data find\n");
        free(buf);
        return 0;
    }

    //rt_kprintf("read otp res %d\n", res);
    fac_cfg_size = page_size;
    p = buf;
#endif
    //rt_kprintf("start 0x%x, %d\n",i, p[i]);

cfg_read:
    i = 0;
    while (p[i] != FACTORY_CFG_ID_UNINIT)
    {
        len = p[i + 1];
        if ((i + len + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size)   // More than max configuration area?
        {
            //rt_kprintf("over range \n");
            len = 0;
            break;
        }
        if (p[i] == id)                               // Found config
        {
            //rt_kprintf("Find id %d, pos %d\n", id, i);
            break;
        }
        i += (len + SYSCFG_FACTORY_HDR_SIZE);       // Next config
        len = 0;
    }
    if (len)                                        // Found config
    {
        if (len > size)
            len = size;
        memcpy(data, &p[i + SYSCFG_FACTORY_HDR_SIZE], len);
    }
#if !defined(CFG_SUPPORT_NON_OTP)
    else if (retry >= 2) // only for user configure on page 1
    {
        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, CFG_USER_OTP_PAGE << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        rt_flash_unlock(cfg_base);
        if (res > 0)
        {
            fac_cfg_size = fhandle->ctable->oob_size * 256 - CFG_OTP_OFFSET;
            p = buf + CFG_OTP_OFFSET;
            retry--;
            goto cfg_read;
        }
    }

    HAL_FLASH_SET_WDT(fhandle, to_val);
#endif

    if (buf)
        free(buf);

    return len;
}

/**
  * @brief Write factory configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_flash_config_write(uint8_t id, uint8_t *data, uint8_t len)
{
    int i = 0;
    uint8_t temp[2];
    uint32_t fac_cfg_size = 0;
    int level;
    uint32_t cfg_base = BSP_GetOtpBase();
#if !defined(CFG_SUPPORT_NON_OTP)
    if (is_onflash_cfg(id) == 0)
    {
        FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
        if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
        {
            rt_kprintf("configure get flash handle error\n");
            return 0;
        }
        uint8_t *buf = malloc(fhandle->ctable->oob_size * 256);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        int onchip = is_onchip_cfg(id);
        int opage = 0;
        if (onchip)
            opage = CFG_IN_OTP_PAGE;
        else
            opage = CFG_USER_OTP_PAGE;

        int to_val = HAL_FLASH_GET_WDT_VALUE(fhandle);
        HAL_FLASH_SET_WDT(fhandle, 0);
        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        int res = HAL_QSPI_READ_OTP(fhandle, opage << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
            free(buf);
            HAL_FLASH_SET_WDT(fhandle, to_val);
            rt_kprintf("read otp error\n");
            return 0;
        }
#if 0
        for (i = 0; i < fhandle->ctable->oob_size * 256; i++)
        {
            if ((i & 7) == 0)
                rt_kprintf("\n0x%08x: ", i);
            rt_kprintf("0x%02x ", *(buf + i));
        }
#endif
        fac_cfg_size = fhandle->ctable->oob_size * 256 - CFG_OTP_OFFSET;
        uint8_t *p = buf + CFG_OTP_OFFSET;

        int update = 0;
        i = 0;
        //rt_kprintf("start 0x%x, %d\n",i, p[i]);
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                break;
            }
            if (p[i] == id)           // Found config
            {
                //rt_kprintf("Find id %d, len %d\n", id, p[i+1]);
                if (p[i + 1] == len) // size is enough, reuse it, or just for same size？
                {
                    // remain old len to avoid find config error, but may cause get data error?
                    memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
                    update = 1;
                    break;
                }
                else // mark as invalid
                {
                    //p[i] = FACTORY_CFG_ID_INVALID;
                    update = 2;
                    break;
                }
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }

        if (update == 2) // refill cache
        {
            uint8_t *cache = malloc(fhandle->ctable->oob_size * 256);
            if (cache == NULL)
            {
                update = 0;
                len = 0;
                //free(buf);
                //return 0;
            }
            else
            {
                int j = i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE;
                int len = 0;
                while (p[j] != FACTORY_CFG_ID_UNINIT)
                {
                    if (p[j] != FACTORY_CFG_ID_INVALID)
                    {
                        memcpy(cache + len, &p[j], p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);
                        len += p[j + 1] + SYSCFG_FACTORY_HDR_SIZE;
                    }
                    j += (p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
                }
                memset(&p[i], 0xff, fac_cfg_size - i);  // configure be backup, reset to invalid data
                memcpy(&p[i], cache, len);  // fill all configure after this id
                i = i + len; // jump to last
                free(cache);
            }
        }
        //rt_kprintf("i = %d, OTP size %d, len %d, id 0x%x\n", i, fac_cfg_size, len, p[i]);
        if (i < fac_cfg_size &&
                p[i] == FACTORY_CFG_ID_UNINIT &&
                i + SYSCFG_FACTORY_HDR_SIZE + len < fac_cfg_size)
        {
            p[i] = id;
            p[i + 1] = len;
            memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
            update = 1;
            //rt_kprintf("Not find, add new id %d: pos %d: len %d\n", id, i, len);
        }
        if (update)
        {
            rt_flash_lock(cfg_base);
            level = rt_hw_interrupt_disable();
            res = HAL_QSPI_ERASE_OTP(fhandle, opage << 12);
            res = HAL_QSPI_WRITE_OTP(fhandle, opage << 12, p, fhandle->ctable->oob_size * 256);
            rt_hw_interrupt_enable(level);
            rt_flash_unlock(cfg_base);
            if (res == 0)
                len = 0;
#if 0
            rt_kprintf("Write otp res %d\n", res);
            for (i = 0; i < fhandle->ctable->oob_size * 256; i++)
            {
                if ((i & 7) == 0)
                    rt_kprintf("\n0x%08x: ", i);
                rt_kprintf("0x%02x ", *(buf + i));
            }
#endif
        }
        else
            len = 0;

        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        HAL_FLASH_SET_WDT(fhandle, to_val);
        free(buf);
    }
    else
    {
        uint8_t *p = (uint8_t *)cfg_base;
        fac_cfg_size = SYSCFG_FACTORY_SIZE;
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                break;
            }
            if (p[i] == id)                               // Found config, mark as invalid
            {
                temp[0] = FACTORY_CFG_ID_INVALID;
                temp[1] = p[i + 1];
                rt_flash_write(cfg_base + i, temp, 2);  // write 2 byte to avoid dual flash error
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }

        if (i < fac_cfg_size &&
                p[i] == FACTORY_CFG_ID_UNINIT &&
                i + SYSCFG_FACTORY_HDR_SIZE + len < fac_cfg_size)
        {
            temp[0] = id;
            temp[1] = len;
            rt_flash_write(cfg_base + i, temp, SYSCFG_FACTORY_HDR_SIZE);
            rt_flash_write(cfg_base + i + SYSCFG_FACTORY_HDR_SIZE, data, len);
        }
        else
            len = 0;
    }
#else

    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(cfg_base);
    if (fhandle == NULL)
    {
        rt_kprintf("rt_flash_config_read addr: 0x%x find handle error\n", cfg_base);
        return 0;
    }

    int idx = 1;
    int off = 0;
    int find = 0;
    int res = 0;
    uint32_t *pBuf;
    uint8_t *buf;
    int block_size = HAL_NAND_BLOCK_SIZE(fhandle);
    int page_size = HAL_NAND_PAGE_SIZE(fhandle);

    buf = malloc(page_size * 2);
    if (buf == NULL)
    {
        rt_kprintf("rt_flash_config_write malloc 0x%x fail\n", page_size * 2);
        return 0;
    }

    res = rt_nand_read_page(cfg_base + block_size * 2, (uint8_t *)buf, page_size, NULL, 0);
    if (res != page_size)
    {
        rt_kprintf("rt_nand_read_page ret error: 0x%x vs 0x%x\n", res, page_size);
        free(buf);
        return 0;
    }
    pBuf = (uint32_t *)buf;
    if (pBuf[0] != CFG_NAND_FACTORY_MAGIC)
    {
        //rt_kprintf("NAND factory magic error 0x%x vs 0x%x， need erase block\n", pBuf[0], CFG_NAND_FACTORY_MAGIC);
        idx = block_size / page_size - 1;
        memset(buf, 0xff, page_size * 2);
    }
    else
    {
        memset(buf, 0xff, page_size * 2);
        //level = rt_hw_interrupt_disable();
        //rt_flash_lock(cfg_base);
        for (idx = block_size / page_size - 1; idx > 0; idx--)
        {
            res = rt_nand_read_page(cfg_base + block_size * 2 + idx * page_size, (uint8_t *)buf, page_size, NULL, 0);
            if (res != page_size || buf[0] != 0xff)  //find the last block
            {
                break;
            }
        }
        //rt_hw_interrupt_enable(level);
        //rt_flash_unlock(cfg_base);
        if (res != page_size)
        {
            rt_kprintf("rt_nand_read_page ret error: 0x%x vs 0x%x\n", res, page_size);
            free(buf);
            return 0;
        }
    }

    //rt_kprintf("read otp res %d\n", res);
    fac_cfg_size = page_size;
    while (buf[i] != FACTORY_CFG_ID_UNINIT)
    {
        if ((i + buf[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
        {
            break;
        }
        if (buf[i] == id)                               // Found config, mark as invalid
        {
            if (buf[i + 1] == len && memcmp(data, &buf[i + SYSCFG_FACTORY_HDR_SIZE], len) == 0) //same data, no update
            {
                free(buf);
                return len;
            }
            else if (off + len + SYSCFG_FACTORY_HDR_SIZE <= page_size)
            {
                buf[page_size + off] = id;
                off++;
                buf[page_size + off] = len;
                off++;
                memcpy(&buf[page_size + off], data, len);
                off += len;
                find = 1;
            }
            else
            {
                free(buf);
                return 0;
            }
        }
        else
        {
            if (off + buf[i + 1] + SYSCFG_FACTORY_HDR_SIZE <= page_size)
            {
                memcpy(&buf[page_size + off], &buf[i], buf[i + 1] + SYSCFG_FACTORY_HDR_SIZE);
                off += buf[i + 1] + SYSCFG_FACTORY_HDR_SIZE;
            }
            else
            {
                free(buf);
                return 0;
            }
        }
        i += (buf[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
    }

    if (find == 0)   //not find, copy data
    {
        if (off + len + SYSCFG_FACTORY_HDR_SIZE <= page_size)
        {
            buf[page_size + off] = id;
            off++;
            buf[page_size + off] = len;
            off++;
            memcpy(&buf[page_size + off], data, len);
            off += len;
        }
        else
        {
            rt_kprintf("len out of range: 0x%x vs 0x%x\n", off + len + SYSCFG_FACTORY_HDR_SIZE, page_size);
            free(buf);
            return 0;
        }
    }

    //level = rt_hw_interrupt_disable();
    if (idx == block_size / page_size - 1) //the last block, erase, use first block
    {
        rt_kprintf("not init or 128 page over, rt_nand_erase_block\n");
        res = rt_nand_erase_block(cfg_base + block_size * 2);
        if (res != 0)
        {
            free(buf);
            return 0;
        }

        memset(buf, 0xff, page_size);
        pBuf[0] = CFG_NAND_FACTORY_MAGIC;
        res = rt_nand_write_page(cfg_base + block_size * 2, &buf[0], page_size, NULL, 0);
        if (res != page_size)
        {
            rt_kprintf("rt_nand_write_page ret error: 0x%x vs 0x%x\n", res, page_size);
            free(buf);
            return 0;
        }
        idx = 1;
    }
    else if (i != 0 || idx == 0) //use next block
    {
        idx += 1;
    }
    res = rt_nand_write_page(cfg_base + block_size * 2 + idx * page_size, &buf[page_size], page_size, NULL, 0);
    if (res != page_size)
    {
        rt_kprintf("rt_nand_write_page ret error 0x%x vs 0x%x\n", res, page_size);
        free(buf);
        return 0;
    }

    if (buf)
    {
        free(buf);
    }

#endif

    return len;
}


/**
  * @brief Read factory user configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_user_config_read(uint8_t id, uint8_t *data, uint8_t size)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int i = 0;
    int len = 0;
    uint32_t fac_cfg_size = 0;
    uint8_t *p;
    int res;
    int level;
    uint8_t *buf = NULL;

    uint32_t cfg_base = BSP_GetOtpBase();

    if (is_onchip_cfg(id) != 0)
        return 0;

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
    {
        // if flash not enable, fhandle should be null, it may load to cache at initial for user cfg
        p = (uint8_t *)BSP_Get_UserOTP_Cache();
        if (p != NULL)
        {
            fac_cfg_size = 256; //CFG_USER_SIZE;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        buf = malloc(fhandle->ctable->oob_size * 256);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, CFG_USER_OTP_PAGE << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            free(buf);
            return 0;
        }
        //rt_kprintf("read otp res %d\n", res);
        fac_cfg_size = fhandle->ctable->oob_size * 256;
        p = buf;
    }

    i = 0;
    while (p[i] != FACTORY_CFG_ID_UNINIT)
    {
        len = p[i + 1];
        if ((i + len + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size)   // More than max configuration area?
        {
            //rt_kprintf("over range \n");
            len = 0;
            break;
        }
        if (p[i] == id)                               // Found config
        {
            //rt_kprintf("Find id %d, pos %d\n", id, i);
            break;
        }
        i += (len + SYSCFG_FACTORY_HDR_SIZE);       // Next config
        len = 0;
    }
    if (len)                                        // Found config
    {
        if (len > size)
            len = size;
        memcpy(data, &p[i + SYSCFG_FACTORY_HDR_SIZE], len);
    }

    if (buf)
        free(buf);

    return len;
#else
    return rt_flash_config_read(id, data, size);
#endif
}

/**
  * @brief Write factory user configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_user_config_write(uint8_t id, uint8_t *data, uint8_t len)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int i = 0;
    uint32_t fac_cfg_size = 0;
    int level;
    uint32_t cfg_base = BSP_GetOtpBase();

    if (is_onchip_cfg(id) == 0)
    {
        FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
        if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
        {
            rt_kprintf("configure get flash handle error\n");
            return 0;
        }
        uint8_t *buf = malloc(fhandle->ctable->oob_size * 256);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        int opage = CFG_USER_OTP_PAGE;

        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        int res = HAL_QSPI_READ_OTP(fhandle, opage << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
            free(buf);
            rt_kprintf("read otp error\n");
            return 0;
        }

        fac_cfg_size = fhandle->ctable->oob_size * 256;
        uint8_t *p = buf;

        int update = 0;
        i = 0;
        //rt_kprintf("start 0x%x, %d\n",i, p[i]);
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                break;
            }
            if (p[i] == id)           // Found config
            {
                //rt_kprintf("Find id %d, len %d\n", id, p[i+1]);
                if (p[i + 1] == len) // size is enough, reuse it, or just for same size？
                {
                    // remain old len to avoid find config error, but may cause get data error?
                    memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
                    update = 1;
                    break;
                }
                else // mark as invalid
                {
                    //p[i] = FACTORY_CFG_ID_INVALID;
                    update = 2;
                    break;
                }
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }

        if (update == 2) // refill cache
        {
            uint8_t *cache = malloc(fhandle->ctable->oob_size * 256);
            if (cache == NULL)
            {
                update = 0;
                len = 0;
                //free(buf);
                //return 0;
            }
            else
            {
                int j = i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE;
                int len = 0;
                while (p[j] != FACTORY_CFG_ID_UNINIT)
                {
                    if (p[j] != FACTORY_CFG_ID_INVALID)
                    {
                        memcpy(cache + len, &p[j], p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);
                        len += p[j + 1] + SYSCFG_FACTORY_HDR_SIZE;
                    }
                    j += (p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
                }
                memset(&p[i], 0xff, fac_cfg_size - i);  // configure be backup, reset to invalid data
                memcpy(&p[i], cache, len);  // fill all configure after this id
                i = i + len; // jump to last
                free(cache);
            }
        }
        //rt_kprintf("i = %d, OTP size %d, len %d, id 0x%x\n", i, fac_cfg_size, len, p[i]);
        if (i < fac_cfg_size &&
                p[i] == FACTORY_CFG_ID_UNINIT &&
                i + SYSCFG_FACTORY_HDR_SIZE + len < fac_cfg_size)
        {
            p[i] = id;
            p[i + 1] = len;
            memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
            update = 1;
            //rt_kprintf("Not find, add new id %d: pos %d: len %d\n", id, i, len);
        }
        if (update)
        {
            level = rt_hw_interrupt_disable();
            res = HAL_QSPI_ERASE_OTP(fhandle, opage << 12);
            res = HAL_QSPI_WRITE_OTP(fhandle, opage << 12, p, fhandle->ctable->oob_size * 256);
            rt_hw_interrupt_enable(level);
            if (res == 0)
                len = 0;
        }
        else
            len = 0;

        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        free(buf);
    }

    return len;
#else
    return rt_flash_config_write(id, data, len);
#endif
}


/**
  * @brief Read customer configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_cust_config_read(uint8_t id, uint8_t *data, uint8_t size)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int len = 0;
    int i = 0;
    uint32_t fac_cfg_size = 0;
    uint8_t *p;
    int res;
    int level;
    uint8_t *buf = NULL;

    uint32_t cfg_base = BSP_GetOtpBase();

    if (is_onchip_cfg(id) != 0)
        return 0;

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
    {
        // if flash not enable, fhandle should be null, it may load to cache at initial for user cfg
        p = (uint8_t *)BSP_Get_CustOTP_Cache();
        if (p != NULL)
        {
            fac_cfg_size = 256; //CFG_USER_SIZE;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        buf = malloc(fhandle->ctable->oob_size * 256);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, CFG_CUST_OTP_PAGE << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            free(buf);
            return 0;
        }
        //rt_kprintf("read otp res %d\n", res);
        fac_cfg_size = fhandle->ctable->oob_size * 256;
        p = buf;
    }

    i = 0;
    while (p[i] != FACTORY_CFG_ID_UNINIT)
    {
        len = p[i + 1];
        if ((i + len + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size)   // More than max configuration area?
        {
            //rt_kprintf("over range \n");
            len = 0;
            break;
        }
        if (p[i] == id)                               // Found config
        {
            //rt_kprintf("Find id %d, pos %d\n", id, i);
            break;
        }
        i += (len + SYSCFG_FACTORY_HDR_SIZE);       // Next config
        len = 0;
    }
    if (len)                                        // Found config
    {
        if (len > size)
            len = size;
        memcpy(data, &p[i + SYSCFG_FACTORY_HDR_SIZE], len);
    }

    if (buf)
        free(buf);

    return len;
#else
    return rt_flash_config_read(id, data, size);
#endif

}

/**
  * @brief Write customer configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_cust_config_write(uint8_t id, uint8_t *data, uint8_t len)
{
#if !defined(CFG_SUPPORT_NON_OTP)
    int i = 0;
    uint32_t fac_cfg_size = 0;
    int level;
    uint32_t cfg_base = BSP_GetOtpBase();

    if (is_onchip_cfg(id) == 0)
    {
        FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
        if (fhandle == NULL || fhandle->ctable == NULL || fhandle->ctable->oob_size == 0)
        {
            rt_kprintf("configure get flash handle error\n");
            return 0;
        }
        uint8_t *buf = malloc(fhandle->ctable->oob_size * 256);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        int opage = CFG_CUST_OTP_PAGE;

        rt_flash_lock(cfg_base);
        rt_flash_switch_dtr(cfg_base, 0);    // switch to sdr before otp read
        level = rt_hw_interrupt_disable();
        int res = HAL_QSPI_READ_OTP(fhandle, opage << 12, buf, fhandle->ctable->oob_size * 256);
        rt_hw_interrupt_enable(level);
        rt_flash_unlock(cfg_base);
        if (res == 0)
        {
            rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
            free(buf);
            rt_kprintf("read otp error\n");
            return 0;
        }

        fac_cfg_size = fhandle->ctable->oob_size * 256;
        uint8_t *p = buf;

        int update = 0;
        i = 0;
        //rt_kprintf("start 0x%x, %d\n",i, p[i]);
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                break;
            }
            if (p[i] == id)           // Found config
            {
                //rt_kprintf("Find id %d, len %d\n", id, p[i+1]);
                if (p[i + 1] == len) // size is enough, reuse it, or just for same size？
                {
                    // remain old len to avoid find config error, but may cause get data error?
                    memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
                    update = 1;
                    break;
                }
                else // mark as invalid
                {
                    //p[i] = FACTORY_CFG_ID_INVALID;
                    update = 2;
                    break;
                }
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }

        if (update == 2) // refill cache
        {
            uint8_t *cache = malloc(fhandle->ctable->oob_size * 256);
            if (cache == NULL)
            {
                update = 0;
                len = 0;
                //free(buf);
                //return 0;
            }
            else
            {
                int j = i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE;
                int len = 0;
                while (p[j] != FACTORY_CFG_ID_UNINIT)
                {
                    if (p[j] != FACTORY_CFG_ID_INVALID)
                    {
                        memcpy(cache + len, &p[j], p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);
                        len += p[j + 1] + SYSCFG_FACTORY_HDR_SIZE;
                    }
                    j += (p[j + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
                }
                memset(&p[i], 0xff, fac_cfg_size - i);  // configure be backup, reset to invalid data
                memcpy(&p[i], cache, len);  // fill all configure after this id
                i = i + len; // jump to last
                free(cache);
            }
        }
        //rt_kprintf("i = %d, OTP size %d, len %d, id 0x%x\n", i, fac_cfg_size, len, p[i]);
        if (i < fac_cfg_size &&
                p[i] == FACTORY_CFG_ID_UNINIT &&
                i + SYSCFG_FACTORY_HDR_SIZE + len < fac_cfg_size)
        {
            p[i] = id;
            p[i + 1] = len;
            memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
            update = 1;
            //rt_kprintf("Not find, add new id %d: pos %d: len %d\n", id, i, len);
        }
        if (update)
        {
            rt_flash_lock(cfg_base);
            level = rt_hw_interrupt_disable();
            res = HAL_QSPI_ERASE_OTP(fhandle, opage << 12);
            res = HAL_QSPI_WRITE_OTP(fhandle, opage << 12, p, fhandle->ctable->oob_size * 256);
            rt_hw_interrupt_enable(level);
            rt_flash_unlock(cfg_base);
            if (res == 0)
                len = 0;
        }
        else
            len = 0;

        rt_flash_switch_dtr(cfg_base, 1);    // recover dtr if setting
        free(buf);
    }

    return len;
#else
    return rt_flash_config_write(id, data, len);
#endif
}

/**
  * @brief Lock/Unlock config area.
  * @param page page ID to be lock.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_flash_config_lock(uint8_t page)
{
    uint32_t cfg_base = BSP_GetOtpBase();
#if !defined(CFG_SUPPORT_NON_OTP)
#ifdef CFG_IN_FLASH_OTP
    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle == NULL || fhandle->ctable == NULL)
        return RT_ERROR;

    rt_flash_lock(cfg_base);
    int level = rt_hw_interrupt_disable();
    int res = HAL_QSPI_LOCK_OTP(fhandle, (uint32_t)page << 12);
    rt_hw_interrupt_enable(level);
    rt_flash_unlock(cfg_base);
    if (res != 0)
        return RT_ERROR;
#else
    // TODO: enable NOR protect
#endif
#endif
    return RT_EOK;
}

/**
  * @brief Read customer configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_flash_otp_customer_read(uint8_t id, uint8_t *data, uint8_t size)
{
    uint8_t res;
    res = rt_flash_config_read(id, data, size);
    if (res > 0)
        goto end;

    res = rt_user_config_read(id, data, size);
    if (res > 0)
        goto end;

    res = rt_cust_config_read(id, data, size);
end:
    return res;
}

#if CFG_SUPPORT_NAND_OTP
int rt_syscfg_set_otp_page(uint8_t page_start, uint8_t page_cnt)
{
    uint32_t cfg_base = BSP_GetOtpBase();
    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle != NULL)    // it is nor flash, do not need set otp page info now, need update later?
        return 0;

    fhandle = rt_nand_get_handle(cfg_base);
    if (fhandle == NULL)    // not nand? so somthing wrong!
        return -1;

    g_sys_otp_start = page_start;
    g_sys_otp_cnt = page_cnt;

    return 0;
}

int rt_factory_cfg_init(void)
{
    return 0;
}

uint8_t rt_factory_cfg_write(uint8_t id, uint8_t *data, uint8_t len)
{
    int i, res;
    uint32_t fac_cfg_size = 0;
    int level;
    uint32_t cfg_base = BSP_GetOtpBase();

    if (data == NULL || len == 0)
        return 0;

    if (is_onchip_cfg(id) != 0) // onchip cfg do not use this interface, something wrong
        return 0;

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle != NULL)  // nor flash otp, use old interface
    {
        // TODO: use user cfg or customer cfg? add more interface ?
        // support user cfg
        return rt_user_config_write(id, data, len);
    }

    fhandle = rt_nand_get_handle(cfg_base);
    if (fhandle != NULL)
    {
        if (g_sys_otp_cnt == 0)
        {
            rt_kprintf("otp page can not be 0 for nand\n");
            return 0;
        }
        uint8_t *buf = malloc(2048);
        if (buf == NULL)
        {
            //rt_kprintf("malloc fail\n");
            return 0;
        }

        int opage = (int)g_sys_otp_start;

        level = rt_hw_interrupt_disable();  // close irq to avoid someone else access nand flash
        HAL_NAND_SWITCH_OTP(fhandle, 1);    // switch to otp mode

retry:
        if (opage >= g_sys_otp_start + g_sys_otp_cnt) // all otp page parsed
        {
            //HAL_NAND_SWITCH_OTP(fhandle, 0);
            //rt_hw_interrupt_enable(level);
            //free(buf);
            rt_kprintf("Add otp page parsed no spare\n");
            //return 0;
            len = 0;
            goto exit;
        }

        res = HAL_NAND_READ_PAGE(fhandle, opage * SPI_NAND_PAGE_SIZE, buf, 2048);
        if (res == 0)
        {
            //HAL_NAND_SWITCH_OTP(fhandle, 0);
            //rt_hw_interrupt_enable(level);
            //free(buf);
            rt_kprintf("read NAND otp error\n");
            //return 0;
            len = 0;
            goto exit;
        }

        fac_cfg_size = 2048;
        uint8_t *p = buf;

        i = 0;
        //rt_kprintf("start 0x%x, %d\n",i, p[i]);
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                // should not happen, NAND OTP can not over one page
                break;
            }
            if (p[i] == id)           // Found config
            {
                p[i] = FACTORY_CFG_ID_INVALID; //// mark as invalid
                //update = 2;
                //break;
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }
        if (i + SYSCFG_FACTORY_HDR_SIZE + len >= fac_cfg_size)  // this page full, use next
        {
            opage++;
            goto retry;
        }

        if (p[i] == FACTORY_CFG_ID_UNINIT)  // find empty pos, and length enough
        {
            p[i] = id;
            p[i + 1] = len;
            memcpy(&p[i + SYSCFG_FACTORY_HDR_SIZE], data, len);
        }

        res = HAL_NAND_WRITE_PAGE(fhandle, opage * SPI_NAND_PAGE_SIZE, buf, 2048);  // write full page again
        if (res == 0)
            len = 0;

exit:
        HAL_NAND_SWITCH_OTP(fhandle, 0);    // switch to normal mode
        rt_hw_interrupt_enable(level);
        free(buf);
    }

    return len;
}

uint8_t rt_factory_cfg_read(uint8_t id, uint8_t *data, uint8_t len)
{
    int i, res, flag;
    uint32_t fac_cfg_size = 0;
    int level;
    uint32_t cfg_base = BSP_GetOtpBase();

    if (data == NULL || len == 0)
        return 0;

    if (is_onchip_cfg(id) != 0) // onchip cfg do not use this interface, something wrong
        return 0;

    FLASH_HandleTypeDef *fhandle = Addr2Handle(cfg_base);
    if (fhandle != NULL)  // nor flash otp, use old interface
    {
        // TODO: use user cfg or customer cfg? add more interface ?
        // suppose user cfg
        return rt_user_config_write(id, data, len);
    }

    fhandle = rt_nand_get_handle(cfg_base);
    if (fhandle != NULL)
    {
        if (g_sys_otp_cnt == 0)
        {
            rt_kprintf("otp page can not be 0 for nand\n");
            return 0;
        }
        uint8_t *buf = malloc(2048);
        if (buf == NULL)
        {
            rt_kprintf("malloc buf for nand otp read fail\n");
            return 0;
        }

        int opage = (int)g_sys_otp_start;

        level = rt_hw_interrupt_disable();  // close irq to avoid someone else access nand flash
        HAL_NAND_SWITCH_OTP(fhandle, 1);    // switch to otp mode

retry:
        if (opage >= g_sys_otp_start + g_sys_otp_cnt) // all otp page parsed
        {
            //HAL_NAND_SWITCH_OTP(fhandle, 0);
            //rt_hw_interrupt_enable(level);
            //free(buf);
            rt_kprintf("Add otp page parsed no spare\n");
            //return 0;
            len = 0;
            goto exit;
        }

        res = HAL_NAND_READ_PAGE(fhandle, opage * SPI_NAND_PAGE_SIZE, buf, 2048);
        if (res == 0)
        {
            //HAL_NAND_SWITCH_OTP(fhandle, 0);
            //rt_hw_interrupt_enable(level);
            //free(buf);
            rt_kprintf("read NAND otp error\n");
            //return 0;
            len = 0;
            goto exit;
        }

        fac_cfg_size = 2048;
        uint8_t *p = buf;

        i = 0;
        flag = 0;   // data not ready

        if (p[0] == FACTORY_CFG_ID_UNINIT)  // page first byte should be a valid id or 0 for id be covered
        {
            len = 0;
            goto exit;
        }
        //rt_kprintf("start 0x%x, %d\n",i, p[i]);
        while (p[i] != FACTORY_CFG_ID_UNINIT)
        {
            if ((i + p[i + 1] + SYSCFG_FACTORY_HDR_SIZE) >= fac_cfg_size) // More than max configuration area?
            {
                // should not happen, NAND OTP can not over one page
                break;
            }
            if (p[i] == id)           // Found config
            {
                if (len > p[i + 1])
                    len = p[i + 1];
                memcpy(data, &p[i + SYSCFG_FACTORY_HDR_SIZE], len);
                flag = 1;
                break;
            }
            i += (p[i + 1] + SYSCFG_FACTORY_HDR_SIZE);  // Next config
        }
        if ((p[i] == FACTORY_CFG_ID_UNINIT) && (flag == 0))  // not found in this page
        {
            opage++;
            goto retry;
        }

exit:
        HAL_NAND_SWITCH_OTP(fhandle, 0);    // switch to normal mode
        rt_hw_interrupt_enable(level);
        free(buf);
    }

    return len;
}


#endif

int rt_sys_config_init(void)
{
    FACTORY_CFG_CRYSTAL_T xtal_cfg;
    FACTORY_CFG_BATTERY_CALI_T battery_cfg;

    uint8_t res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&xtal_cfg, sizeof(FACTORY_CFG_CRYSTAL_T));
    if ((res > 0) && (xtal_cfg.cbank_sel != 0) && (xtal_cfg.cbank_sel != 0x3ff)) // add xtal invalid data check
    {
        // set crystal configure.
        if (xtal_cfg.cbank_sel != GET_REG_VAL(hwp_pmuc->HXT_CR1, PMUC_HXT_CR1_CBANK_SEL_Msk, PMUC_HXT_CR1_CBANK_SEL_Pos))
        {
            /* TODO: this code block is used when CBANK_SEL is saved in NAND, it should be fixed also just like NOR OTP */
            HAL_PMU_SET_HXT_CBANK(xtal_cfg.cbank_sel);
        }
    }
#if !defined(SOC_SF32LB52X)
    res = rt_flash_config_read(FACTORY_CFG_ID_BATTERY, (uint8_t *)&battery_cfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    if (res > 0)
    {
        BSP_CONFIG_set(FACTORY_CFG_ID_BATTERY, (uint8_t *)&battery_cfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    }
#endif
    return 0;
}
INIT_DEVICE_EXPORT(rt_sys_config_init);


/********************************** SPI FLASH TEST CODE *************************************/

//#define DRV_SYS_CFG_TEST
#ifdef DRV_SYS_CFG_TEST

int is_ate_param_valid(void)
{
    int res;

    FACTORY_CFG_ADC_T cfg;
    FACTORY_CFG_SDMADC_T cfg2;
    FACTORY_CFG_VBK_LDO_T cfg3;
    FACTORY_CFG_CRYSTAL_T cfg4;
    uint8_t mac[8];
    uint8_t sn[32];
    FACTORY_CFG_BATTERY_CALI_T btcfg;
    uint8_t xflag, macflag, snflag, btflag;
    xflag = 0;
    macflag = 0;
    snflag = 0;
    btflag = 0;

    res = rt_flash_config_read(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
    if (res != sizeof(FACTORY_CFG_ADC_T))
    {
        rt_kprintf("Read ADC configure fail res %d\n", res);
        return 0;
    }
    rt_kprintf("Get ADC res %d, vol 300 %d, vol 800 %d\n", res, cfg.vol10 & 0x7fff, cfg.vol25 & 0x7fff);
    if ((cfg.vol10 & 0x8000) && (cfg.vol25 & 0x8000))
        rt_kprintf("ADC X1 MODE\n");
    else
        rt_kprintf("ADC X3 MODE \n");

    res = rt_flash_config_read(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
    if (res != sizeof(FACTORY_CFG_SDMADC_T))
    {
        rt_kprintf("Read SDMADC configure fail res %d\n", res);
        return 0;
    }
    if (cfg2.vol_mv != 1200) // sdmadc voltage fixed to 1200 mv
    {
        rt_kprintf("SDMADC PARAM error, voltage not 1200 : %d\n", cfg2.vol_mv);
        return 0;
    }
    rt_kprintf("Get SDMADC res %d, vol %d\n", res, cfg2.value);


    res = rt_flash_config_read(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
    if (res != sizeof(FACTORY_CFG_VBK_LDO_T))
    {
        rt_kprintf("Read VBUCK configure fail res %d\n", res);
        return 0;
    }
    rt_kprintf("Get VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
               res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);

    res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cfg4, sizeof(FACTORY_CFG_CRYSTAL_T));
    if (res != sizeof(FACTORY_CFG_CRYSTAL_T))
    {
        if (res != 0)
        {
            rt_kprintf("Read XTAL configure fail res %d\n", res);
            return 0;
        }
    }
    else
        xflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_MAC, (uint8_t *)mac, 6);
    if (res != 6)
    {
        if (res != 0)
        {
            rt_kprintf("Read mac configure fail res %d\n", res);
            return 0;
        }
    }
    else
        macflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_SN, (uint8_t *)sn, 32);
    if (res != 32)
    {
        if (res != 0)
        {
            rt_kprintf("Read sn configure fail res %d\n", res);
            return 0;
        }
    }
    else
        snflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_BATTERY, (uint8_t *)&btcfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    if (res != sizeof(FACTORY_CFG_BATTERY_CALI_T))
    {
        if (res != 0)
        {
            rt_kprintf("Read batt configure fail res %d\n", res);
            return 0;
        }
    }
    else
        btflag = 1;

    return 1;
}

int clear_and_overwrite(void)
{
    int res;

    FACTORY_CFG_ADC_T cfg;
    FACTORY_CFG_SDMADC_T cfg2;
    FACTORY_CFG_VBK_LDO_T cfg3;
    FACTORY_CFG_CRYSTAL_T cfg4;
    uint8_t mac[8] = {0};
    uint8_t sn[32] = {0};
    FACTORY_CFG_BATTERY_CALI_T btcfg;
    uint8_t xflag, macflag, snflag, btflag;
    xflag = 0;
    macflag = 0;
    snflag = 0;
    btflag = 0;

    res = rt_flash_config_read(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
    if (res != sizeof(FACTORY_CFG_ADC_T))
    {
        rt_kprintf("Read ADC configure fail res %d, force to default value\n", res);
        cfg.vol10 = 405 | 0x8000;
        cfg.vol25 = 741 | 0x8000;
        //return 0;
    }
    rt_kprintf("Get ADC res %d, vol 300 %d, vol 800 %d\n", res, cfg.vol10 & 0x7fff, cfg.vol25 & 0x7fff);
    if ((cfg.vol10 & 0x8000) && (cfg.vol25 & 0x8000))
        rt_kprintf("ADC X1 MODE\n");
    else
        rt_kprintf("ADC X3 MODE \n");

    res = rt_flash_config_read(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
    if ((res != sizeof(FACTORY_CFG_SDMADC_T)) || (cfg2.vol_mv != 1200))
    {
        rt_kprintf("Read SDMADC configure fail res %d, force to default value\n", res);
        cfg2.value = 1537561;
        cfg2.vol_mv = 1200;
        //return 0;
    }
    rt_kprintf("Get SDMADC res %d, vol %d\n", res, cfg2.value);

    res = rt_flash_config_read(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
    if (res != sizeof(FACTORY_CFG_VBK_LDO_T))
    {
        rt_kprintf("Read VBUCK configure fail res %d, force to default value\n", res);
        cfg3.vbuck1 = (uint8_t)13;
        cfg3.vbuck2 = (uint8_t)6;
        cfg3.hp_ldo = (uint8_t)11;
        cfg3.lp_ldo = (uint8_t)6;
        cfg3.vret = (uint8_t)3;
        //return 0;
    }
    rt_kprintf("Get VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
               res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);

    res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cfg4, sizeof(FACTORY_CFG_CRYSTAL_T));
    if (res != sizeof(FACTORY_CFG_CRYSTAL_T))
    {
        xflag = 0;
    }
    else
        xflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_MAC, (uint8_t *)mac, 6);
    if (res != 6)
    {
        macflag = 0;
    }
    else
        macflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_SN, (uint8_t *)sn, 32);
    if (res != 32)
    {
        snflag = 0;
    }
    else
        snflag = 1;

    res = rt_flash_config_read(FACTORY_CFG_ID_BATTERY, (uint8_t *)&btcfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
    if (res != sizeof(FACTORY_CFG_BATTERY_CALI_T))
    {
        btflag = 0;
    }
    else
        btflag = 1;

    // erase all otp
    FLASH_HandleTypeDef *fhandle = Addr2Handle(SYSCFG_FACTORY_ADDRESS);
    res = HAL_QSPI_ERASE_OTP(fhandle, 1 << 12);

    // write adc, vbuck, sdmadc
    res = rt_flash_config_write(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
    rt_kprintf("Cfg VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
               res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);

    res = rt_flash_config_write(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
    rt_kprintf("Cfg ADC res %d, vol300 %d, vol800 %d\n", res, cfg.vol10 & 0x7fff, cfg.vol25 & 0x7fff);

    res = rt_flash_config_write(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
    rt_kprintf("Cfg SDMADC res %d, vol %d\n", res, cfg2.value);

    // xtal
    if (xflag)
    {
        res = rt_flash_config_write(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cfg4, sizeof(FACTORY_CFG_CRYSTAL_T));
        rt_kprintf("XTAL set %d, res %d\n", cfg4.cbank_sel, res);
    }

    //mac
    if (macflag)
    {
        res = rt_flash_config_write(FACTORY_CFG_ID_MAC, (uint8_t *)mac, 6);
        rt_kprintf("mac set %s, res %d\n", mac, res);
    }

    //sn
    if (snflag)
    {
        res = rt_flash_config_write(FACTORY_CFG_ID_SN, (uint8_t *)sn, 32);
        rt_kprintf("sn set %s, res %d\n", sn, res);
    }

    //bt
    if (btflag)
    {
        res = rt_flash_config_write(FACTORY_CFG_ID_BATTERY, (uint8_t *)&btcfg, sizeof(FACTORY_CFG_BATTERY_CALI_T));
        rt_kprintf("battery set mag %x , %d, %d, res %d\n", btcfg.magic, btcfg.a, btcfg.b, res);
    }

    return 1;
}

void overwrite_ate_cfg(void)
{
    int res;
    FACTORY_CFG_ADC_T cfg;
    FACTORY_CFG_SDMADC_T cfg2;
    FACTORY_CFG_VBK_LDO_T cfg3;

    cfg.vol10 = 405 | 0x8000;
    cfg.vol25 = 741 | 0x8000;
    res = rt_flash_config_write(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
    rt_kprintf("Cfg ADC res %d, vol 300 %d, vol 800 %d\n", res, cfg.vol10 & 0x7fff, cfg.vol25 & 0x7fff);


    cfg2.value = 1537561;
    cfg2.vol_mv = 1200;
    res = rt_flash_config_write(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
    rt_kprintf("Cfg SDMADC res %d, vol %d\n", res, cfg2.value);


    cfg3.vbuck1 = (uint8_t)13;
    cfg3.vbuck2 = (uint8_t)6;
    cfg3.hp_ldo = (uint8_t)10;
    cfg3.lp_ldo = (uint8_t)5;
    cfg3.vret = (uint8_t)6;
    res = rt_flash_config_write(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
    rt_kprintf("Cfg VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
               res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);

}

static uint32_t get_flash_base_by_id(int id)
{
    uint32_t addr = 0;

    if (id == 1)
        addr = FLASH2_BASE_ADDR;
#ifdef FLASH4
    else if (id == 2)
        addr = FLASH3_BASE_ADDR;
#endif
#ifdef FLASH4
    else if (id == 3)
        addr = FLASH4_BASE_ADDR;
#endif
#ifdef FLASH5
    else if (id == 4)
        addr = FLASH5_BASE_ADDR;
#endif
    else if (id == 0)
        addr = QSPI1_MEM_BASE;
    else
        addr = FLASH_BASE_ADDR;
    //addr = QSPI1_MEM_BASE;

    return addr;
}
int cmd_scfg(int argc, char *argv[])
{
    uint32_t value, addr, len;
    int i, id;
    uint8_t *buf;

    if (strcmp(argv[1], "-otpr") == 0)
    {
        id = atoi(argv[2]);
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            rt_kprintf("OTP block only support block 1/2/3\n");
            return 0;
        }

        addr = get_flash_base_by_id(id);
        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        buf = malloc(1024);
        if (buf == NULL || fhandle == NULL)
        {
            rt_kprintf("Malloc buffer fail or get flash handle fail\n");
            return 0;
        }
        int level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, blk << 12, buf, 256);
        rt_hw_interrupt_enable(level);
        rt_kprintf("READ otp addr 0x%x with res %d\n", blk << 12, res);
        if (res > 0)
        {
            int j;
            for (j = 0; j < res; j++)
            {
                rt_kprintf("0x%02x  ", buf[j]);
                if ((j & 0x7) == 0x7)
                    rt_kprintf("\n");
            }
        }
        free(buf);
    }
    else if (strcmp(argv[1], "-otpe") == 0)
    {
        id = atoi(argv[2]);
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            rt_kprintf("OTP block only support block 1/2/3\n");
            return 0;
        }

        addr = get_flash_base_by_id(id);
        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
        {
            rt_kprintf("get flash handle fail\n");
            return 0;
        }
        res = HAL_QSPI_ERASE_OTP(fhandle, blk << 12);
        rt_kprintf("ERASE otp addr 0x%x with res %d\n", blk << 12, res);

    }
    else if (strcmp(argv[1], "-otpl") == 0)
    {
        id = atoi(argv[2]);
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            rt_kprintf("OTP block only support block 1/2/3\n");
            return 0;
        }

        addr = get_flash_base_by_id(id);
        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
        {
            rt_kprintf("get flash handle fail\n");
            return 0;
        }

        res = HAL_QSPI_LOCK_OTP(fhandle, blk << 12);
        rt_kprintf("LOCK otp addr 0x%x with res %d\n", blk << 12, res);

    }
    else if (strcmp(argv[1], "-cfgw") == 0)
    {
        int res;
        id = atoi(argv[2]);
        switch (id)
        {
        case FACTORY_CFG_ID_ADC:
        {
            FACTORY_CFG_ADC_T cfg;
#ifndef SF32LB55X
            cfg.vol10 = atoi(argv[3]) & 0xffff;
            cfg.vol25 = atoi(argv[4]) & 0xffff;
            cfg.low_mv = atoi(argv[5]) & 0xffff;
            cfg.high_mv = atoi(argv[6]) & 0xffff;
#else
            cfg.vol10 = atoi(argv[3]) | 0x8000;
            cfg.vol25 = atoi(argv[4]) | 0x8000;
#endif
            res = rt_flash_config_write(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
            rt_kprintf("Cfg ADC res %d, small reg %d, big reg %d\n", res, cfg.vol10 % 0x7fff, cfg.vol25 & 0x7fff);
            break;
        }
        case FACTORY_CFG_ID_SDMADC:
        {
            FACTORY_CFG_SDMADC_T cfg2;
            cfg2.value = atoi(argv[3]);
            cfg2.vol_mv = 1200;
            res = rt_flash_config_write(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
            rt_kprintf("Cfg SDMADC res %d, vol %d\n", res, cfg2.value);
        }
        break;
        case FACTORY_CFG_ID_VBUCK:
        {
            FACTORY_CFG_VBK_LDO_T cfg3;
            cfg3.vbuck1 = (uint8_t)atoi(argv[3]);
            cfg3.vbuck2 = (uint8_t)atoi(argv[4]);
            cfg3.hp_ldo = (uint8_t)atoi(argv[5]);
            cfg3.lp_ldo = (uint8_t)atoi(argv[6]);
            cfg3.vret = (uint8_t)atoi(argv[7]);
            res = rt_flash_config_write(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
            rt_kprintf("Cfg VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
                       res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);
        }
        break;
        default:
            rt_kprintf("Invalid key %d\n", id);
            rt_kprintf("for ADC input id(4) mv300 mv800, like# spi_flash -cfgw 4 211 985\n");
            rt_kprintf("for SDMADC input id(5) reg1200, like# spi_flash -cfgw 5 123456\n");
            rt_kprintf("for SDMADC input id(6) vbuck1 vbuck2 hp_ldo lp_ldo vert\n");
            return 0;
            //break;

        }

    }
    else if (strcmp(argv[1], "-cfgr") == 0)
    {
        int res;
        id = atoi(argv[2]);
        switch (id)
        {
        case FACTORY_CFG_ID_ADC:
        {
            FACTORY_CFG_ADC_T cfg;
            res = rt_flash_config_read(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, sizeof(FACTORY_CFG_ADC_T));
            rt_kprintf("Get ADC res %d, small %d, big %d\n", res, cfg.vol10 % 0x7fff, cfg.vol25 & 0x7fff);
            break;
        }
        case FACTORY_CFG_ID_SDMADC:
        {
            FACTORY_CFG_SDMADC_T cfg2;
            cfg2.vol_mv = 1200;
            res = rt_flash_config_read(FACTORY_CFG_ID_SDMADC, (uint8_t *)&cfg2, sizeof(FACTORY_CFG_SDMADC_T));
            rt_kprintf("Get SDMADC res %d, vol %d\n", res, cfg2.value);
        }
        break;
        case FACTORY_CFG_ID_VBUCK:
        {
            FACTORY_CFG_VBK_LDO_T cfg3;
            res = rt_flash_config_read(FACTORY_CFG_ID_VBUCK, (uint8_t *)&cfg3, sizeof(FACTORY_CFG_VBK_LDO_T));
            rt_kprintf("Get VBUK res %d, vbuck1 %d, vbuck2 %d, hp_ldo %d, lp_ldo %d, vert %d\n",
                       res, cfg3.vbuck1, cfg3.vbuck2, cfg3.hp_ldo, cfg3.lp_ldo, cfg3.vret);
        }
        break;
        default:
            rt_kprintf("Invalid key %d\n", id);
            return 0;
            //break;

        }

    }
    else if (strcmp(argv[1], "-ateok") == 0)
    {
        int res = is_ate_param_valid();
        rt_kprintf("ATE CFG valid %d\n", res);
    }
    else if (strcmp(argv[1], "-owate") == 0)
    {
        overwrite_ate_cfg();
        rt_kprintf("ATE overwrite done, use -otpr to check, and make sure -otpe before over write!\n");
    }
    else if (strcmp(argv[1], "-ovall") == 0)
    {
        clear_and_overwrite();
        rt_kprintf("ATE overwrite done, use -otpr to check, and make sure -otpe before over write!\n");
    }
    else
    {
        rt_kprintf("Invalid parameters\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_scfg, __cmd_scfg, Test sys_cfg driver);

#endif // DRV_SYS_CFG_TEST

#else
uint8_t rt_flash_config_read(uint8_t id, uint8_t *data, uint8_t size)
{
    return 0;
}

uint8_t rt_flash_config_write(uint8_t id, uint8_t *data, uint8_t len)
{
    return 0;
}

uint8_t rt_cust_config_read(uint8_t id, uint8_t *data, uint8_t size)
{
    return 0;
}

uint8_t rt_cust_config_write(uint8_t id, uint8_t *data, uint8_t len)
{
    return 0;
}


#endif /* BSP_USING_SPI_FLASH */


/// @} bsp_driver


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
