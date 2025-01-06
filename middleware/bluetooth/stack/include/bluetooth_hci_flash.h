/**
  ******************************************************************************
  * @file   bluetooth_hci_flash.h
  * @author Sifli software development team
  * @brief SIFLI bluetooth hci log written to flash header file.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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

#ifdef HCI_ON_FLASH

    #ifndef __BLUETOOTH_HCI_FLASH_H
        #define __BLUETOOTH_HCI_FLASH_H


        #include "rtthread.h"
        #include "os_adaptor.h"
        #include "board.h"

        #ifndef BT_HCI_PARTION
            #define BT_HCI_PARTION "hci"
        #endif

        #ifndef BT_HCI_PATH
            #define BT_HCI_PATH "hci/"
        #endif

        #ifndef BT_HCI_DFT_SIZE
            #define BT_HCI_DFT_SIZE (512*1024)
        #endif // BT_HCI_DFT_SIZE

        #define BT_HCI_DEFAULT_FLUSH_SIZE (64 * 1024)
        #define BT_HCI_INVALID_FLUSH_SIZE (0xFFFFFFFF)


        int bt_hci_log_path_get(char *path);
        int bt_hci_log_onoff(int flag);
        int bt_hci_log_type_get(void);
        int bt_hci_log_clear(void);



        #if defined(RT_USING_DFS) || defined(USING_FILE_LOGGER)
            uint32_t bt_hci_init(uint32_t flush_size, uint32_t is_start);
            uint32_t bt_hci_write(uint8_t *buffer, uint32_t len);
            uint32_t bt_hci_flush(void);
            uint32_t bt_hci_close(void);
            uint32_t bt_hci_open(void);

        #endif // RT_USING_DFS

    #endif //__BLUETOOTH_HCI_FLASH_H

#endif // HCI_ON_FLASH

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
