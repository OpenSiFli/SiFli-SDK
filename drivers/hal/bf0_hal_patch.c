/**
  ******************************************************************************
  * @file   bf0_hal_patch.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"
#include "bf0_hal_patch.h"
#include "register.h"

#if defined(HAL_LCPU_PATCH_MODULE)
__USED int HAL_PATCH_install2(struct patch_entry_desc *patch_entries, uint32_t size, int cer)
{
    int r = 0;
    int i;
    uint32_t data;
    __IO uint32_t *p_a = (__IO uint32_t *)PATCH_BASE;

    hwp_patch->CER = 0;
    for (i = 0; i < size ; i++, patch_entries++)
    {
        int pn = i + (cer ? PATCH_AON : 0);
        if (cer == 0 || (cer & (1UL << pn)))
        {
            *(p_a + pn) = (patch_entries->break_addr & PATCH_CH0_ADDR_Msk);
            data = patch_entries->data;
            hwp_patch->CSR = (1UL << pn);
            hwp_patch->CDR = data;
            r |= (1UL << pn);
        }
    }
    hwp_patch->CSR = 0;
    hwp_patch->CER = cer ? cer : r;
    return r;
}

typedef void (hook_install_type)(void);
__WEAK uint32_t *HAL_PATCH_GetEntryAddr(void)
{
    return (uint32_t *)LCPU_PATCH_RECORD_ADDR;
}

__USED int HAL_PATCH_install(void)
{
    uint32_t *p = (uint32_t *)HAL_PATCH_GetEntryAddr();
    int r = 0;

    if ((*p) == PATCH_TAG)
    {
        uint32_t size = *(++p);
        size /= sizeof(struct patch_entry_desc);
        p++;
        r = HAL_PATCH_install2((struct patch_entry_desc *)p, size, 0);
    }
    return r;
}

__USED void HAL_PATCH_Entry(void)
{
    uint32_t *p = (uint32_t *)HAL_PATCH_GetEntryAddr();

    if ((*p) == PATCH_TAG)
    {
        hook_install_type *hook_install = (hook_install_type *)(LCPU_PATCH_START_ADDR + 1);
        hook_install();
    }
}

__USED int HAL_PATCH_save(struct patch_entry_desc *patch_entries, uint32_t size, uint32_t *cer)
{
    int i;
    __IO uint32_t *p_a = (__IO uint32_t *)PATCH_BASE;

    for (i = PATCH_AON; i < MAX_PATCH_ENTRIES && i < (PATCH_AON + size); i++, patch_entries++)
    {
        if ((1UL << i) & hwp_patch->CER)
        {
            patch_entries->break_addr = *(p_a + i);
            patch_entries->data = *(uint32_t *)patch_entries->break_addr;
        }
        else
            break;
    }
    *cer = hwp_patch->CER;
    return i - PATCH_AON;
}

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
