/**
  ******************************************************************************
  * @file   bf0_mbox_common.h
  * @author Sifli software development team
  * @brief Sifli Mailbox interface among cores.
  * @{
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

#ifndef __BF0_MBOX_COMMON_H__
#define __BF0_MBOX_COMMON_H__

#include <rtthread.h>

#if defined(USING_IPC_QUEUE) || defined(_SIFLI_DOXYGEN_)
    #include "ipc_queue.h"
    #include "bf0_hal_rcc.h"

    /**
    ****************************************************************************************
    * @addtogroup mbox_common Mailbox interface
    * @ingroup middleware
    * @brief Sifli Mailbox interface among cores.
    * @{
    ****************************************************************************************
    */


    // Define other cores
    #ifdef SOC_BF0_HCPU
        #define PEER_CORE_1     CORE_ID_LCPU
    #endif


    #ifdef SOC_BF0_LCPU
        #define PEER_CORE_1     CORE_ID_HCPU
    #endif



    #if defined(SOC_BF0_HCPU)
        /** @brief power on LCPU,
        *
        *
        * @return 0: OK, >0: ERROR
        */
        uint8_t lcpu_power_on(void);
        /** @brief power off LCPU,
        *
        *
        * @return 0: OK, >0: ERROR
        */
        uint8_t lcpu_power_off(void);

        /** @brief Disable/Enable RF Calibration when power up LCPU
        *
        * @psram is_disable  1: disable RF calibration, 0: enable RF Calibration
        *
        * @return void
        */
        void lcpu_disable_rf_cal(uint8_t is_disable);
    #endif /* SOC_BF0_HCPU */


    #ifdef SOC_BF0_HCPU
        ipc_queue_handle_t sys_init_hl_ipc_queue(ipc_queue_rx_ind_t rx_ind);
        ipc_queue_handle_t sys_get_hl_ipc_queue(void);
    #endif /* SOC_BF0_HCPU */

    #ifdef SOC_BF0_LCPU
        ipc_queue_handle_t sys_init_lh_ipc_queue(ipc_queue_rx_ind_t rx_ind);
        ipc_queue_handle_t sys_get_lh_ipc_queue(void);
    #endif /* SOC_BF0_LCPU */

#else


#endif  // USING_IPC_QUEUE

void print_sysinfo(char *buf, uint32_t buf_len);


/// @}  mbox_common
/// @}  file

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

