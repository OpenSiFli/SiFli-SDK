/**
  ******************************************************************************
  * @file   context_backup.h
  * @author Sifli software development team
  * @brief Sifli wrappter device interface for ipc_queue
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

#ifndef CONTEXT_BACKUP_H
#define CONTEXT_BACKUP_H
#include <stdint.h>
#include <stdbool.h>

/**
 ****************************************************************************************
* @addtogroup context_backup Context Backup and Restore
* @ingroup middleware
* @brief Context Backup and Restore, context including heap and stack
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /** start address */
    uint32_t start_addr;
    /** region length in byte */
    uint32_t len;
} cb_retained_region_t;

/** backup stack */
#define CB_BACKUP_STACK_MASK         ((uint8_t)1 << 0)
/** backup heap */
#define CB_BACKUP_HEAP_MASK          ((uint8_t)1 << 1)
/** backup static data */
#define CB_BACKUP_STATIC_DATA_MASK   ((uint8_t)1 << 2)
/** backup all */
#define CB_BACKUP_ALL_MASK    (CB_BACKUP_STACK_MASK | CB_BACKUP_HEAP_MASK | CB_BACKUP_STATIC_DATA_MASK)

#define CB_MAX_BACKUP_REGION_NUM  (4)



typedef struct
{
    /** start address of retention memory to backup data */
    uint32_t ret_mem_start_addr;
    /** retention memory size in byte */
    uint32_t ret_mem_size;
    /** indicate which data types need backup
     *
     * such as #CB_BACKUP_STACK_MASK, #CB_BACKUP_HEAP_MASK
     */
    uint8_t backup_mask;
    /** backup region number */
    uint8_t backup_region_num;
    /** backup region list */
    cb_retained_region_t backup_region_list[CB_MAX_BACKUP_REGION_NUM];
} cb_backup_param_t;



rt_err_t cb_init(cb_backup_param_t *param);
rt_err_t cb_deinit(void);
rt_err_t cb_save_context(void);
rt_err_t cb_restore_context(void);
void cb_get_stats(uint32_t *total, uint32_t *min_free);

/// @}  context_backup

#ifdef __cplusplus
}
#endif





/// @} file
#endif /* CONTEXT_BACKUP_H */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
