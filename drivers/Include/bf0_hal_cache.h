/**
  ******************************************************************************
  * @file   bf0_hal_cache.h
  * @author Sifli software development team
  * @brief   Header file of CACHE HAL module.
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_CACHE_H
#define __BF0_HAL_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup CACHE
  * @ingroup BF0_HAL_Driver
  * @{
  */


/**
 * @brief  Clear all counters, i.e. reset to 0
 * @retval void
 */
#define HAL_CACHE_RESET()          (hwp_cache->CCR |= CACHE_CCR_CNTCLR_Msk)



/** @defgroup ICACHE_PROF_MODE icache profiling mode
 * @brief ICACHE profiling mode, could be multiple selected
 * @{
 */
#ifdef SF32LB58X

/** Enable MPI1 ICache profiling */
#define HAL_CACHE_ICACHE_MPI1           (CACHE_CCR_IRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_ICACHE_MPI2           (CACHE_CCR_IRANGE_MPI2)
/** Enable MPI3 ICache profiling */
#define HAL_CACHE_ICACHE_MPI3           (CACHE_CCR_IRANGE_MPI3)
/** Enable MPI4 ICache profiling */
#define HAL_CACHE_ICACHE_MPI4           (CACHE_CCR_IRANGE_MPI4)
/** Enable MPI5 ICache profiling */
#define HAL_CACHE_ICACHE_MPI5           (CACHE_CCR_IRANGE_MPI5)
/** Enable all ICache profiling */
#define HAL_CACHE_ICACHE_ALL            (CACHE_CCR_IRANGE_MPI1 | CACHE_CCR_IRANGE_MPI2 \
                                         | CACHE_CCR_IRANGE_MPI3 | CACHE_CCR_IRANGE_MPI4 | CACHE_CCR_IRANGE_MPI5)

#elif defined(SF32LB56X)

/** Enable MPI1 ICache profiling */
#define HAL_CACHE_ICACHE_MPI1           (CACHE_CCR_IRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_ICACHE_MPI2           (CACHE_CCR_IRANGE_MPI2)
/** Enable MPI3 ICache profiling */
#define HAL_CACHE_ICACHE_MPI3           (CACHE_CCR_IRANGE_MPI3)
/** Enable MPI5 ICache profiling */
#define HAL_CACHE_ICACHE_MPI5           (CACHE_CCR_IRANGE_MPI5)
/** Enable all ICache profiling */
#define HAL_CACHE_ICACHE_ALL            (CACHE_CCR_IRANGE_MPI1 | CACHE_CCR_IRANGE_MPI2 \
                                         | CACHE_CCR_IRANGE_MPI3 | CACHE_CCR_IRANGE_MPI5)

#elif defined(SF32LB52X)


/** Enable MPI1 ICache profiling */
#define HAL_CACHE_ICACHE_MPI1           (CACHE_CCR_IRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_ICACHE_MPI2           (CACHE_CCR_IRANGE_MPI2)
/** Enable all ICache profiling */
#define HAL_CACHE_ICACHE_ALL            (CACHE_CCR_IRANGE_MPI1 | CACHE_CCR_IRANGE_MPI2)

#else

/** Enable QSPI1 and QSPI4 ICache profiling */
#define HAL_CACHE_ICACHE_QSPI1_4        (CACHE_CCR_IRANGE_QSPI1_4)
/** Enable QSPI2 ICache profiling */
#define HAL_CACHE_ICACHE_QSPI2          (CACHE_CCR_IRANGE_QSPI2)
/** Enable QSPI3 ICache profiling */
#define HAL_CACHE_ICACHE_QSPI3          (CACHE_CCR_IRANGE_QSPI3)
/** Enable OPSRAM ICache profiling */
#define HAL_CACHE_ICACHE_OPSRAM         (CACHE_CCR_IRANGE_OPSRAM)
/** Enable all ICache profiling */
#define HAL_CACHE_ICACHE_ALL            (HAL_CACHE_ICACHE_QSPI1_4 | HAL_CACHE_ICACHE_QSPI2 \
                                         | HAL_CACHE_ICACHE_QSPI3 | HAL_CACHE_ICACHE_OPSRAM)

#endif

/**
 *
 * @}
 */


/** @defgroup DCACHE_PROF_MODE dcache profiling mode
 * @brief DCACHE profiling mode, could be multiple selected
 * @{
 */
#ifdef SF32LB58X

/** Enable MPI1 ICache profiling */
#define HAL_CACHE_DCACHE_MPI1           (CACHE_CCR_DRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_DCACHE_MPI2           (CACHE_CCR_DRANGE_MPI2)
/** Enable MPI3 ICache profiling */
#define HAL_CACHE_DCACHE_MPI3           (CACHE_CCR_DRANGE_MPI3)
/** Enable MPI4 ICache profiling */
#define HAL_CACHE_DCACHE_MPI4           (CACHE_CCR_DRANGE_MPI4)
/** Enable MPI5 ICache profiling */
#define HAL_CACHE_DCACHE_MPI5           (CACHE_CCR_DRANGE_MPI5)
/** Enable all ICache profiling */
#define HAL_CACHE_DCACHE_ALL            (CACHE_CCR_DRANGE_MPI1 | CACHE_CCR_DRANGE_MPI2 \
                                         | CACHE_CCR_DRANGE_MPI3 | CACHE_CCR_DRANGE_MPI4 | CACHE_CCR_DRANGE_MPI5)

#elif defined(SF32LB56X)

/** Enable MPI1 ICache profiling */
#define HAL_CACHE_DCACHE_MPI1           (CACHE_CCR_DRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_DCACHE_MPI2           (CACHE_CCR_DRANGE_MPI2)
/** Enable MPI3 ICache profiling */
#define HAL_CACHE_DCACHE_MPI3           (CACHE_CCR_DRANGE_MPI3)
/** Enable MPI5 ICache profiling */
#define HAL_CACHE_DCACHE_MPI5           (CACHE_CCR_DRANGE_MPI5)
/** Enable all ICache profiling */
#define HAL_CACHE_DCACHE_ALL            (CACHE_CCR_DRANGE_MPI1 | CACHE_CCR_DRANGE_MPI2 \
                                         | CACHE_CCR_DRANGE_MPI3 | CACHE_CCR_DRANGE_MPI5)


#elif defined(SF32LB52X)

/** Enable MPI1 ICache profiling */
#define HAL_CACHE_DCACHE_MPI1           (CACHE_CCR_DRANGE_MPI1)
/** Enable MPI2 ICache profiling */
#define HAL_CACHE_DCACHE_MPI2           (CACHE_CCR_DRANGE_MPI2)
/** Enable all ICache profiling */
#define HAL_CACHE_DCACHE_ALL            (CACHE_CCR_DRANGE_MPI1 | CACHE_CCR_DRANGE_MPI2)

#else
/** Enable QSPI1 and QSPI4 DCache profiling */
#define HAL_CACHE_DCACHE_QSPI1_4        (CACHE_CCR_DRANGE_QSPI1_4)
/** Enable QSPI2 DCache profiling */
#define HAL_CACHE_DCACHE_QSPI2          (CACHE_CCR_DRANGE_QSPI2)
/** Enable QSPI3 DCache profiling */
#define HAL_CACHE_DCACHE_QSPI3          (CACHE_CCR_DRANGE_QSPI3)
/** Enable OPSRAM DCache profiling */
#define HAL_CACHE_DCACHE_OPSRAM         (CACHE_CCR_DRANGE_OPSRAM)
/** Enable all ICache profiling */
#define HAL_CACHE_DCACHE_ALL            (HAL_CACHE_DCACHE_QSPI1_4 | HAL_CACHE_DCACHE_QSPI2 \
                                         | HAL_CACHE_DCACHE_QSPI3 | HAL_CACHE_DCACHE_OPSRAM)
#endif

/**
 *
 * @}
 */


/**
 * @brief  Enable cache profiling
 *
 * @param[in] imode ICache profiling mode bitmap,
 *                         see @ref ICACHE_PROF_MODE, could select multiple modes,
 *                         such as HAL_CACHE_ICACHE_QSPI1_4|HAL_CACHE_ICACHE_QSPI2 for SF32LB55X
 * @param[in] dmode DCache profiling mode bitmap,
 *                         see @ref DCACHE_PROF_MODE, could select multiple modes,
 *                         such as HAL_CACHE_DCACHE_QSPI1_4|HAL_CACHE_DCACHE_QSPI2 for SF32LB55X
 * @retval void
 */
__STATIC_INLINE void HAL_CACHE_Enable(uint32_t imode, uint32_t dmode)
{
    MODIFY_REG(hwp_cache->CCR, CACHE_CCR_DRANGE_Msk | CACHE_CCR_IRANGE_Msk,
               imode | dmode);
    hwp_cache->CCR |= CACHE_CCR_CNTEN;
    HAL_CACHE_RESET();
}

/**
 * @brief  Disable cache profiling
 * @retval void
 */
__STATIC_INLINE void HAL_CACHE_Disable(void)
{
    hwp_cache->CCR &= ~CACHE_CCR_CNTEN;
}

/**
 * @brief  Get ICACHE and DCACHE miss rate
 * @param[out] irate pointer of output icache miss rate, unit: percentage
 * @param[out] drate pointer of output dcache miss rate, unit: percentage
 * @param[in] reset true: reset all counters, false: do not reset counters
 * @retval void
 */
__STATIC_INLINE void HAL_CACHE_GetMissRate(float *irate, float *drate, bool reset)
{
    if (irate)
    {
        if (hwp_cache->IACR)
        {
            *irate = hwp_cache->IMCR * (float)100 / hwp_cache->IACR;
        }
        else
        {
            *irate = 0;
        }
    }
    if (drate)
    {
        if (hwp_cache->DACR)
        {
            *drate = hwp_cache->DMCR * (float)100 / hwp_cache->DACR;
        }
        else
        {
            *drate = 0;
        }
    }
    if (reset)
    {
        HAL_CACHE_RESET();
    }
}


/**
 * @brief  Get ICACHE miss counter snapshot value when access counter overflows
 * @retval icache miss counter snapshot value
 */
#define HAL_CACHE_GET_ICACHE_SNAPSHOT()   (hwp_cache->ISR)

/**
 * @brief  Get DCACHE miss counter snapshot value when access counter overflows
 * @retval dcache miss counter snapshot value
 */
#define HAL_CACHE_GET_DCACHE_SNAPSHOT()   (hwp_cache->DSR)


#ifdef __cplusplus
}
#endif

///@}   CACHE

#endif /* __BF0_HAL_CACHE_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
