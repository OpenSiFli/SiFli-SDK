/**
  ******************************************************************************
  * @file   lcd_io.h
  * @author Sifli software development team
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
#ifndef __DRV_IO_H__
#define __DRV_IO_H__

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief  Initialize the MSP.
 *
 * Called by HAL_PreInit in all board implementation.
 *
 * @retval None
 */
void HAL_MspInit(void);

/**
 * @brief Init IO(pinmux) setting
 */
void BSP_IO_Init(void);

/**
 * @brief Get Flash divider
 */
uint16_t BSP_GetFlash1DIV(void);
uint16_t BSP_GetFlash2DIV(void);
uint16_t BSP_GetFlash3DIV(void);
uint16_t BSP_GetFlash4DIV(void);
uint16_t BSP_GetFlash5DIV(void);

uint16_t BSP_GetFlashExtDiv(void);


/**
 * @brief Set Flash divider
 */
void BSP_SetFlash1DIV(uint16_t div);
void BSP_SetFlash2DIV(uint16_t div);
void BSP_SetFlash3DIV(uint16_t div);
void BSP_SetFlash4DIV(uint16_t div);
void BSP_SetFlash5DIV(uint16_t div);

void BSP_SetFlashExtDiv(uint16_t div);


/**
 * @brief Power up/down board
 */
void BSP_IO_Power_Down(int coreid, bool is_deep_sleep);
void BSP_Power_Up(bool is_deep_sleep);


void BSP_PowerDownCustom(int coreid, bool is_deep_sleep);

void BSP_PowerUpCustom(bool is_deep_sleep);


/**
 * @brief LCD power up/down/reset
 */
void BSP_LCD_Reset(uint8_t high1_low0);
void BSP_LCD_PowerUp(void);
void BSP_LCD_PowerDown(void);

/**
 * @brief Touch power up/down/reset
 */
void BSP_TP_Reset(uint8_t high1_low0);
void BSP_TP_PowerUp(void);
void BSP_TP_PowerDown(void);

/**
 * @brief Flash
 */

void *BSP_Flash_get_handle(uint32_t addr);
void *BSP_Flash_get_handle_by_id(uint8_t id);

int BSP_Flash_read_id(uint32_t addr);

int BSP_Nor_read(uint32_t addr, uint8_t *buf, int size);
int BSP_Nor_erase(uint32_t addr, uint32_t size);
int BSP_Nor_write(uint32_t addr, const uint8_t *buf, uint32_t size);


void BSP_Flash_var_init(void);
int BSP_Flash_hw1_init(void);
int BSP_Flash_hw2_init(void);
int BSP_Flash_hw2_init_with_no_dtr(void);
int BSP_Flash_hw3_init(void);
int BSP_Flash_hw4_init(void);
int BSP_Flash_hw5_init(void);


int BSP_Flash_Init(void);

/**
 * @brief SDIO
 */
void BSP_SD_PowerUp(void);
void BSP_SD_PowerDown(void);

void BSP_GPIO_Set(int pin, int val, int is_porta);

/**
 * @brief PSRAM
 */

#ifdef BSP_USING_PSRAM

    /**
    * @brief  psram controller hardware initial.
    * @retval 0 if success.
    */
    int bsp_psramc_init(void);

    /**
    * @brief Get PSRAM clock frequency.
    * @param addr base address of psram.
    * @return Clock freqency for PSRAM
    */
    uint32_t bsp_psram_get_clk(uint32_t addr);

    /**
    * @brief Update PSRAM refresh rate.
    * @param name name of PSRAM controller.
    * @param value self refresh rate value
    * @return RT_EOK if initial success, otherwise, -RT_ERROR.
    */
    int bsp_psram_update_refresh_rate(char *name, uint32_t value);

    /**
    * @brief PSRAM hardware enter low power.
    * @param name name of PSRAM controller.
    * @return RT_EOK if initial success, otherwise, -RT_ERROR.
    */
    int bsp_psram_enter_low_power(char *name);

    /**
    * @brief PSRAM hardware enter low power.
    * @param name name of PSRAM controller.
    * @return RT_EOK if initial success, otherwise, -RT_ERROR.
    */
    int bsp_psram_deep_power_down(char *name);

    /**
    * @brief PSRAM hardware exit from low power.
    * @param name name of PSRAM controller.
    * @return RT_EOK if initial success, otherwise, -RT_ERROR.
    */
    int bsp_psram_exit_low_power(char *name);

    /**
    * @brief PSRAM set partial array self-refresh.
    * @param name name of PSRAM controller.
    * @param top set top part to self-refresh, else set bottom.
    * @param deno denomenator for refresh, like 2 for 1/2 to refresh, only support 2^n,
    *         when larger than 16, all memory not refresh, when 1 or 0, all meory auto refress by default.
    * @return 0 if success.
    */
    int bsp_psram_set_pasr(char *name, uint8_t top, uint8_t deno);

    /**
    * @brief PSRAM auto calibrate delay.
    * @param name name of PSRAM controller.
    * @param sck sck delay pointer.
    * @param dqs dqs delay pointer
    * @return 0 if success.
    */
    int bsp_psram_auto_calib(char *name, uint8_t *sck, uint8_t *dqs);

    /**
    * @brief Wait psram hardware idle.
    * @return none.
    */
    void bsp_psram_wait_idle(char *name);

#else

    #define bsp_psramc_init() -1
    #define bsp_psram_get_clk() 0
    #define bsp_psram_update_refresh_rate(name,value) -1
    #define bsp_psram_enter_low_power(name) -1
    #define bsp_psram_deep_power_down(name) -1
    #define bsp_psram_exit_low_power(name) -1
    #define bsp_psram_set_pasr(name,top,deno) -1
    #define bsp_psram_auto_calib(name,sck,dqs) -1
    #define bsp_psram_wait_idle(name)

#endif

/**
  * @}
  */

#endif





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
