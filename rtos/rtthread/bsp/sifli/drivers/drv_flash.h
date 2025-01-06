/**
  ******************************************************************************
  * @file   drv_flash.h
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

#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

#include <rtconfig.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_NOR_PAGE_SIZE             (256)
#define SPI_NOR_SECT_SIZE             (4096)
#define SPI_NOR_BLK32_SIZE            (0X8000)
#define SPI_NOR_BLK64_SIZE            (0X10000)


#define FLASH1_ENABLED      (1<<0)
#define FLASH2_ENABLED      (1<<1)
#define FLASH3_ENABLED      (1<<2)
#define FLASH4_ENABLED      (1<<3)
#define FLASH5_ENABLED      (1<<4)

//#define BSP_USING_BBM

/************type define ***********************/

typedef enum
{
    FLASH_CACHE_DISABLE = 0,
    FLASH_CACHE_ENABLE,
    FLASH_CACHE_AUTO,
    FLASH_CACHE_CNT
} FLASH_CACHE_MODE_T;


/**
* @brief  Init drv_flash module variables
*
*  Init drv_flash module variables to make it work before scatter loading
* @return void
*/
#define rt_hw_flash_var_init BSP_Flash_var_init

/* output api */
/* controller ID decide by address */

/**
 * @brief Read nor-flash memory.
 * @param[in] addr: start address for flash memory.
 * @param[out] buf: output data buffer, should not be null.
 * @param[in] size: read memory size, in bytes.
 * @return read size, 0 if fail.
 */
int rt_flash_read(uint32_t addr, uint8_t *buf, int size);

/**
 * @brief Write nor-flash memory.
 * @param[in] addr: start address for flash memory.
 * @param[in] buf: input data buffer, should not be null.
 * @param[in] size: write memory size, in bytes.
 * @return write size, 0 if fail.
 */
int rt_flash_write(uint32_t addr, const uint8_t *buf, int size);

/**
 * @brief Erase flash with block or secor based .
 * @note For NOR flash, it's erase based unit is sector, so address and size should be sector aligned.
 *       For NAND flash, it's erase base unit is block, so address and size should be block aligned.
 * @param[in] addr: phy start address to erase.
 * @param[in] size: erase memory size, in bytes, but should sector or block aligned.
 * @return RT_EOK if success.
 */
int rt_flash_erase(uint32_t addr, int size);

//#ifdef SOC_BF_Z0

/**
 * @brief Lock flash to avoid read/write/erase conflict
 * @param[in] addr flash address
 * @return void
 */
void rt_flash_lock(uint32_t addr);

/**
 * @brief Unlock flash
 * @param[in] addr flash address
 * @return void
 */
void rt_flash_unlock(uint32_t addr);
//#endif /* SOC_BF_Z0 */


/**
 * @brief get current flash chip index.
 * @param[in] addr: flash start address for this chip.
 * @return manufactory id.
 */
int rt_flash_read_id(uint32_t addr);

/**
 * @brief Get flash erase address and size alignment request.
 * @note The alignment depend on flash type, dual/single flash mode.
 * @param[in] addr: flash start address for this chip.
 * @return Erase basic unit.
 */
int rt_flash_get_erase_alignment(uint32_t addr);

/**
* @brief  enable NOR flash block protect for some address.
* @param[in]  addr, protect start address.
* @param[in]  size, protect memory size.
* @retval 0 if success.
*/
int rt_flash_mem_protect(uint32_t addr, uint32_t size);

/**
* @brief  Remove NOR flash block protect.
* @param[in]  addr, protect start address.
* @retval 0 if success.
*/
int rt_flash_remove_protect(uint32_t addr);

/**
* @brief  Just clear protect bits this time, it will recover after reboot.
* @param[in]  addr, protect start address.
* @retval 0 if success.
*/
int rt_flash_disable_protect(uint32_t addr);

/**
* @brief  Set flash cache mode.
* @param[in]  mode, type FLASH_CACHE_MODE_T .
* @retval 0 if success.
*/
int rt_flash_set_cache(uint32_t addr, FLASH_CACHE_MODE_T mode);

/**
* @brief  Init nor flash.
* @retval for Z0 return 0 if success,
*         for A0, each bit decide which flash valid as FLASH1_ENABLED/FLASH2_ENABLED/FLASH3_ENABLED/FLASH4_ENABLED
*/
int rt_hw_flash_init(void);

/**
* @brief  Deinit nor flash.
* @retval void
*/
int rt_hw_flash_deinit(void);

/**
* @brief  Preinit nor flash1 in high speed mode
* @retval void
*/
void rt_hw_flash1_preinit(void);

/**
* @brief  Access memory larger than 16MB when enable high, and access less than 16MB when disable high
* @param[in]  addr, flash base address .
* @param[in]  high, when en = 1, memory access 16MB ~ 32MB, when en = 0, access 0 ~ 16MB .
* @retval 0 if success
*/
int rt_flash_switch_mem(uint32_t addr, uint8_t high);

void rt_flash_set_alias(uint32_t addr, uint32_t start, uint32_t len, uint32_t offset);

void rt_flash_set_ctr_nonce(uint32_t addr, uint32_t start, uint32_t end, uint8_t *nonce);

void rt_flash_enable_aes(uint32_t addr, uint8_t aes256);

void rt_flash_disable_aes(uint32_t addr);

uint32_t rt_flash_get_clk(uint32_t addr);

int rt_flash_get_total_size(uint32_t addr);


/**
* @brief  Set flash to deep power down or wake up, it should not call when XIP.
* @param addr flash base address.
* @param pd   1 means deep power down, 0 means wake up.
*/
void rt_flash_power_down(uint32_t addr, int pd);

/**
* @brief  Get dual flash mode.
* @retval dual flash mode.
*/
uint8_t flash_get_dual_mode(uint8_t id);

/**
* @brief  Get flash enabled or not.
* @retval flash enable status.
*/
uint8_t flash_is_enabled(uint8_t id);

/**
* @brief  Get flash HAL handle with address.
* @param[in]  addr, memory address .
* @retval flash handle, NULL if fail.
*/
void *rt_flash_get_handle_by_addr(uint32_t addr);

/**
* @brief  Get flash pass id.
* @param[in]  addr, flash base address to decide with flash.
* @retval 0 if fail.
*/
int rt_flash_get_pass_id(uint32_t addr);

/**
* @brief  Get flash uid.
* @param[in]  addr, flash base address to decide with flash.
* @param[out] uid,  data buffer to save uid, can not be NULL
* @param[in]  length, uid length , can not larger than 16.
* @retval Read uid length, 0 if fail.
*/
int rt_flash_get_uid(uint32_t addr, uint8_t *uid, uint32_t length);


#ifdef BSP_USING_SPI_NAND

/**
 * @brief Get nand flash mpi handler.
 * @param[in] addr: start address for nand flash memory.
 * @return handler or NULL.
 */
void *rt_nand_get_handle(uint32_t addr);

/**
 * @brief Read nand-flash memory.
 * @param[in] addr: start address for flash memory.
 * @param[out] buf: output data buffer, should not be null.
 * @param[in] size: read memory size, in bytes.
 * @return read size, 0 if fail.
 */
int rt_nand_read(uint32_t addr, uint8_t *buf, int size);

/**
 * @brief Write nand-flash memory.
  * @note For NAND flash, it's write base unit is page, so address and size should be page aligned.
 * @param[in] addr: start address for flash memory, should be page aligned.
 * @param[in] buf: input data buffer, should not be null.
 * @param[in] size: write memory size, in bytes, it should be page aligned.
 * @return write size, 0 if fail.
 */
int rt_nand_write(uint32_t addr, const uint8_t *buf, int size);

/**
 * @brief Erase nand flash with block aligned .
 * @note For NAND flash, it's erase base unit is block, so address and size should be block aligned.
 * @param[in] addr: phy start address to erase.
 * @param[in] size: erase memory size, in bytes, but should be block aligned.
 * @return RT_EOK if success.
 */
int rt_nand_erase(uint32_t addr, int size);

/**
* @brief  Read nand flash data.
* @param[in]  addr, flash address need to read.
* @param[out]  data, output data buffer.
* @param[in]  size, data size to read.
* @param[out]  spare, buffer to save oob data.
* @param[in]  spare_len, oob data size to read.
* @retval read data size.
*/
int rt_nand_read_page(uint32_t addr, uint8_t *data, int size, uint8_t *spare, int spare_len);

/**
* @brief  Write nand flash data.
* @param[in]  addr, flash address need to write.
* @param[in]  buf, input data buffer.
* @param[in]  size, data size to read.
* @param[in]  spare, buffer to save oob data.
* @param[in]  spare_len, oob data size to read.
* @retval read data size.
*/
int rt_nand_write_page(uint32_t addr, const uint8_t *buf, int size, const uint8_t *spare, int spare_len);

/**
* @brief  Erase nand flash block.
* @param[in]  addr, flash address need to erase.
* @retval 0 if success.
*/
int rt_nand_erase_block(uint32_t addr);

/**
* @brief  Read nand chip manufacture ID.
* @param[in]  addr, nand valid address.
* @retval NAND MID if success, 0 if fail.
*/
int rt_nand_read_id(uint32_t addr);

/**
* @brief  Get nand full valid size (byte).
* @param[in]  addr, nand valid address.
* @retval NAND size if success, 0 if fail.
*/
int rt_nand_get_total_size(uint32_t addr);

/**
 * @brief Get nand controller id.
 * @return nand controller index if success, -1 if fail.
 */
int rt_flash_get_nand_index(void);

/**
 * @brief Initialize nand.
 * @return none-zero if success, 0 if fail.
 */
int rt_hw_nand_init(void);

/**
 * @brief De-Initialize nand.
 * @return 0.
 */
int rt_hw_nand_deinit(void);

int rt_nand_init(void);

#else
#define rt_nand_get_handle(addr) NULL
#define rt_nand_read_rom(handle,addr,buf,size) 0
#define rt_nand_read(addr,buf,size) 0
#define rt_nand_write(addr,buf,size) 0
#define rt_nand_erase(addr,size) 0
#define rt_nand_read_page(addr,data,size,spare,spare_len) 0
#define rt_nand_write_page(addr,buf,size,spare,spare_len) 0
#define rt_nand_erase_block(addr) 0
#define rt_nand_get_total_size(addr) 0
#define rt_nand_init() 0
#define rt_hw_nand_init() 0
#define rt_hw_nand_deinit() 0
#endif

/**
  * @brief Initialize ATE configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_chip_config_init(void);

/**
  * @brief Initialize factory configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_user_config_init(void);

/**
  * @brief Initialize factory configuration area.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_flash_config_init(void);

/**
  * @brief Read factory configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param len Max length of data .
  * @retval length of data read.
  */
uint8_t rt_flash_config_read(uint8_t id, uint8_t *data, uint8_t size);

/**
  * @brief Write factory configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_flash_config_write(uint8_t id, uint8_t *data, uint8_t len);

/**
  * @brief Read factory user configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_user_config_read(uint8_t id, uint8_t *data, uint8_t size);

/**
  * @brief Write factory user configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_user_config_write(uint8_t id, uint8_t *data, uint8_t len);

/**
  * @brief Read customer configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be read.
  * @param size Max length of data .
  * @retval length of data read.
  */
uint8_t rt_cust_config_read(uint8_t id, uint8_t *data, uint8_t size);

/**
  * @brief Write customer configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data written.
  */
uint8_t rt_cust_config_write(uint8_t id, uint8_t *data, uint8_t len);

/**
  * @brief read  configuraiton.
  * @param id  Factory config ID.
  * @param data  config data to be written.
  * @param len length of data .
  * @retval length of data read.
  */
uint8_t rt_flash_otp_customer_read(uint8_t id, uint8_t *data, uint8_t size);

/**
  * @brief Lock config area.
  * @param page  page ID to be lock.
  * @retval RT_EOK if successful, otherwise RT_ERROR.
  */
rt_err_t rt_flash_config_lock(uint8_t page);

/**
* @brief  Get flash latest action error code.
* @param[in]  addr, flash base address .
* @retval latest flash status.
*/
int rt_flash_get_last_status(uint32_t addr);


#ifdef RT_USING_MTD_NOR

/**
 * @brief Register MTD-NOR device with default sector size.
 * @param[in] flash_base: base address for flash controller.
 * @param[in] offset: device start address to access.
 * @param[in] size: device access total size, in bytes.
 * @param[in] name: register device name.
 * @return none.
 */
void register_nor_device(uint32_t flash_base, uint32_t offset, uint32_t size, char *name);
#else
#define register_nor_device(flash_base,offset,size,name)
#endif


#ifdef RT_USING_MTD_NAND
/**
 * @brief Register MTD-NAND device.
 * @param[in] flash_base: base address for flash controller.
 * @param[in] offset: device start address to access.
 * @param[in] size: device access total size, in bytes.
 * @param[in] name: register device name.
 * @return none.
 */
void register_nand_device(uint32_t flash_base, uint32_t offset, uint32_t size, char *name);
#else
#define register_nand_device(flash_base,offset,size,name)
#endif

/**
 * @brief Register MTD device.
 * @param[in] address: start address for MTD device.
 * @param[in] size: device access total size, in bytes.
 * @param[in] name: register device name.
 * @return none.
 */
void register_mtd_device(uint32_t address, uint32_t size, char *name);


#ifdef __cplusplus
}
#endif

#endif  /* __DRV_FLASH_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
