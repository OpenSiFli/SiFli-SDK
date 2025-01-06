/**
  ******************************************************************************
  * @file   dfu_uart.c
  * @author Sifli software development team
  * @brief uart dfu protocol.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <rthw.h>

#include "bf0_hal_patch.h"
#include "bf0_mbox_common.h"

#include "drv_flash.h"
#include "dfu_uart.h"

#ifdef BSP_USING_DFU_UART
#define LOG_TAG "dfu_uart"
#include "log.h"

static uint32_t crc32mpeg2_table[] = {
        0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
        0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
        0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
        0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
        0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
        0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
        0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
        0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
        0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
        0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
        0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
        0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
        0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
        0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
        0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
        0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
        0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
        0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
        0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
        0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
        0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
        0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
        0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
        0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
        0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
        0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
        0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
        0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
        0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
        0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
        0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
        0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
        0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
        0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
        0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
        0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
        0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
        0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
        0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
        0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
        0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
        0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
        0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

static dfu_uart_env_t g_dfu_uart_env;
static void dfu_uart_send(uint8_t *data, uint16_t len);
int dfu_uart_forward_init(void);

static void run_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}

static void clear_interrupt_setting(void)
{
    uint32_t i;
    for (i = 0; i < 16; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        __ISB();
    }
}

static void dfu_bootjump(void)
{

#ifdef RT_USING_WDT
    // TODO: Deinit watch log should be implmented in user bin
    extern void rt_hw_watchdog_deinit(void);
    rt_hw_watchdog_deinit();
#endif // RT_USING_WDT

    //HAL_sw_breakpoint();
    uint32_t i;

    register rt_base_t ret;
    ret = rt_hw_interrupt_disable();
    clear_interrupt_setting();
    rt_hw_interrupt_enable(ret);

    for (i = 0; i < 8; i++)
        NVIC->ICER[0] = 0xFFFFFFFF;
    for (i = 0; i < 8; i++)
        NVIC->ICPR[0] = 0xFFFFFFFF;
    SysTick->CTRL = 0;
    SCB->ICSR |= SCB_ICSR_PENDNMICLR_Msk;
    SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTACT_Msk | SCB_SHCSR_BUSFAULTACT_Msk | SCB_SHCSR_MEMFAULTACT_Msk);

    if (CONTROL_SPSEL_Msk & __get_CONTROL())
    {
        __set_MSP(__get_PSP());
        __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
    }



    SCB->VTOR = (uint32_t)HCPU2_CODE_START_ADDR;
#ifdef PSRAM_XIP
    memcpy((void *)PSRAM_BASE, (void *)HCPU2_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE);
    run_img((uint8_t *)PSRAM_BASE);
#else
    run_img((uint8_t *)HCPU2_CODE_START_ADDR);
#endif

}

static dfu_uart_env_t *dfu_uart_get_env(void)
{
    return &g_dfu_uart_env;
}

void dfu_running_img_init()
{
    dfu_uart_env_t *env = dfu_uart_get_env();
    LOG_I("dfu_running_img_init");
    dfu_running_img_info info;
    uint32_t addr = DFU_RUNNING_IMG_STATE_ADDR;
    rt_flash_read(addr, (uint8_t *)&info, sizeof(dfu_running_img_info));

    if (info.magic != SEC_CONFIG_MAGIC)
    {
        LOG_I("dfu_ota_bootloader_ram_run_flash_init not ready");
        // first use not ready
        info.magic = SEC_CONFIG_MAGIC;
        info.running_img = DFU_RUNNING_ON_HCPU;

        rt_flash_erase(addr, DFU_RUNNING_IMG_INFO_MAX_SIZE);
        rt_flash_write(addr, (uint8_t *)&info, sizeof(dfu_running_img_info));
    }

    LOG_I("dfu_running_img_init %d", info.running_img);

    if (info.running_img == DFU_RUNNING_ON_HCPU)
    {
        env->mode = DFU_RUNNING_ON_HCPU;
    } else {
        env->mode = DFU_RUNNING_ON_OTA;
    }
}

static void dfu_running_img_set(uint8_t target)
{
    dfu_running_img_info info;
    uint32_t addr = DFU_RUNNING_IMG_STATE_ADDR;
    rt_flash_read(addr, (uint8_t *)&info, sizeof(dfu_running_img_info));

    info.magic = SEC_CONFIG_MAGIC;
    info.running_img = target;

    rt_flash_erase(addr, DFU_RUNNING_IMG_INFO_MAX_SIZE);
    rt_flash_write(addr, (uint8_t *)&info, sizeof(dfu_running_img_info));
}

void dfu_uart_dfu_mode_set()
{
    dfu_running_img_set(DFU_RUNNING_ON_OTA);
    drv_reboot();
}

void dfu_uart_reset_handler()
{
    dfu_uart_env_t *env = dfu_uart_get_env();
    LOG_I("dfu_uart_reset_handler %d", env->mode);
    if (env->mode == DFU_RUNNING_ON_HCPU)
    {
        dfu_bootjump();
    }
    dfu_uart_forward_init();
}

static rt_err_t dfu_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    dfu_uart_env_t *env = dfu_uart_get_env();
    //LOG_I("dfu_uart_rx_ind %d", size);
    rt_mb_send(env->to_mb, size);
    return 0;
}

static int dfu_uart_init(void)
{
    LOG_I("dfu_uart_init");

    dfu_uart_env_t *env = dfu_uart_get_env();
    memset((uint8_t *)env, 0, sizeof(dfu_uart_env_t));

    //rt_thread_mdelay(5000);
    dfu_running_img_init();
    return 0;
}
INIT_APP_EXPORT(dfu_uart_init);

static void dfu_uart_device_init(void)
{
    rt_err_t result;
    rt_uint16_t oflag;
    dfu_uart_env_t *env = dfu_uart_get_env();

    env->device = rt_device_find(UART_FORWARD_PORT);
    if (env->device)
    {
        //oflag = RT_DEVICE_OFLAG_RDWR;
        oflag = RT_DEVICE_OFLAG_RDWR;

        if (env->device->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }
#if 0
        if (env->uart_port->flag & RT_DEVICE_FLAG_DMA_TX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_TX;
        }
#endif
        result = rt_device_open(env->device, oflag);
        RT_ASSERT(RT_EOK == result);

        rt_device_set_rx_indicate(env->device, dfu_uart_rx_ind);
    }
}

static void dfu_uart_image_start_rsp(dfu_uart_env_t *env, uint16_t result)
{
    LOG_I("dfu_uart_image_start_rsp %d", result);
    dfu_image_start_rsp_t rsp;
    rsp.command = DFU_UART_IMAGE_START_RSP;
    rsp.length = 2;
    rsp.result = result;

    uint8_t *rsp_data;
    uint16_t rsp_data_len = 6;
    rsp_data = malloc(rsp_data_len);
    memcpy(rsp_data, (uint8_t *)&rsp, rsp_data_len);
    dfu_uart_send(rsp_data, rsp_data_len);
    free(rsp_data);
}

static void dfu_uart_image_start_handler(dfu_uart_env_t *env, uint8_t *data, uint16_t data_len)
{
    LOG_I("dfu_uart_image_start_handler");
    uint16_t result = DFU_UART_ERROR_NO_ERROR;
    if (env->state != DFU_UART_STATE_NONE && env->state != DFU_UART_STATE_DOWNLOAD_END)
    {
        //LOG_I("dfu_uart_image_start_handler error state %d", env->state);
        //return;
    }

    if (data_len != 16)
    {
        LOG_I("SIZE ERROR");
        return;
    }

    uint8_t *t_data;
    t_data = malloc(data_len);
    memcpy(t_data, data, data_len);

    dfu_image_start_req_t *req = (dfu_image_start_req_t *)t_data;

    LOG_I("image len %d", req->image_length);
    LOG_I("ADDR 0x%x, crc 0x%x, data size %d", req->addr, req->crc, req->data_size);

    uint32_t erase_size = req->image_length;
    uint32_t align_size;
#ifdef SF32LB55X
    align_size = 0x2000;
#else
    align_size = 0x1000;
#endif

    if (erase_size % align_size != 0)
    {
        erase_size = (erase_size + align_size) / align_size * align_size;
    }
    LOG_I("erase size %d", erase_size);

    int ret = rt_flash_erase(req->addr, erase_size);

    if (ret != 0)
    {
        LOG_I("flash erase error with %d", ret);
        result = DFU_UART_ERROR_FLASH_ERASE_ERROR;
    } else {
        result = DFU_UART_ERROR_NO_ERROR;
    }

    if (result == 0)
    {
        env->image_data_index = 0;
        env->image_total_length = req->image_length;
        env->base_addr = req->addr;
        env->remote_crc = req->crc;
        env->single_packet_size = req->data_size * 1024;
    }

    free(t_data);
    dfu_uart_image_start_rsp(env, result);
}

static void dfu_uart_image_data_cfm(dfu_uart_env_t *env, uint16_t result, uint16_t index)
{
    LOG_I("dfu_uart_image_data_cfm %d", result);

    dfu_image_data_cfm_t cfm_data;
    cfm_data.command = DFU_UART_IMAGE_DATA_CFM;
    cfm_data.length = 4;
    cfm_data.result = DFU_UART_ERROR_NO_ERROR;
    cfm_data.current_index = index;

    uint8_t *rsp_data;
    uint16_t rsp_data_len = sizeof(dfu_image_data_cfm_t);
    rsp_data = malloc(rsp_data_len);
    memcpy(rsp_data, (uint8_t *)&cfm_data, rsp_data_len);
    dfu_uart_send(rsp_data, rsp_data_len);
    free(rsp_data);
}

static void dfu_uart_image_data_handler(dfu_uart_env_t *env, uint8_t *data, uint16_t data_len)
{
    uint8_t result = DFU_UART_ERROR_NO_ERROR;
    dfu_image_data_t *img_data = (dfu_image_data_t *)data;
    LOG_I("dfu_uart_image_data_handler data len %d, index %d", data_len - 2, img_data->index);

    if (img_data->index != env->image_data_index + 1)
    {
        // TODO
        LOG_E("index error expect %d, receive %d", env->image_data_index + 1, img_data->index);
        result = DFU_UART_ERROR_INDEX_ERROR;
        dfu_uart_image_data_cfm(env, result, env->image_data_index);
        return;
    }

    if (env->single_packet_size == data_len - 2)
    {
        // len check ok
    } else {
        if (env->image_current_length + data_len - 2 == env->image_total_length)
        {
            // last packet
        } else {
            result = DFU_UART_ERROR_PACKET_LENGTH_ERROR;
            dfu_uart_image_data_cfm(env, result, 0);
            return;
        }
    }

    LOG_I("write addr 0x%x, len %d", env->base_addr + env->image_current_length, data_len - 2);
    rt_flash_write(env->base_addr + env->image_current_length, img_data->packet, data_len - 2);

    env->image_current_length += data_len - 2;
    env->image_data_index++;

    dfu_uart_image_data_cfm(env, result, 0);
}

static uint32_t crc32mpeg2_get_value(uint8_t *inData, uint16_t len, uint32_t lastCrc) {
    uint16_t i;
    uint32_t crc = lastCrc;

    for (i = 0; i < len; i++)
        crc = (int) ((crc << 8) ^ crc32mpeg2_table[(int) (((crc >> 24) ^ inData[i]) & 0xFF)]);
    return crc;
}

static uint8_t dfu_uart_crc_verification(dfu_uart_env_t *env)
{
    uint8_t ret;
    uint8_t *dfu_temp;
    dfu_temp = malloc(DFU_UART_MAX_BLOCK_SIZE);
    RT_ASSERT(dfu_temp);

    uint32_t offset = 0;
    uint32_t size;
    uint32_t current_crc = 0xFFFFFFFF;

    while (offset < env->image_total_length)
    {
        if (offset + DFU_UART_MAX_BLOCK_SIZE <= env->image_total_length)
        {
            size = DFU_UART_MAX_BLOCK_SIZE;
        }
        else
        {
            size = env->image_total_length - offset;
        }

        rt_flash_read(env->base_addr + offset, dfu_temp, size);
        current_crc = crc32mpeg2_get_value(dfu_temp, size, current_crc);

        offset += size;
    }
    free(dfu_temp);

    LOG_I("dfu_uart_crc_verification 0x%x", current_crc);
    if (env->remote_crc == current_crc)
    {
        ret = DFU_UART_ERROR_NO_ERROR;
    }
    else
    {
        LOG_I("crc error with 0x%x", env->remote_crc);
        ret = DFU_UART_ERROR_CRC_ERROR;
    }
    return ret;
}

static void dfu_uart_image_end_rsp(dfu_uart_env_t *env, uint16_t result)
{
    LOG_I("dfu_uart_image_end_rsp %d", result);
    dfu_image_end_rsp_t rsp;
    rsp.command = DFU_UART_IMAGE_END_RSP;
    rsp.length = 2;
    rsp.result = result;

    uint8_t *rsp_data;
    uint16_t rsp_data_len = sizeof(dfu_image_end_rsp_t);
    rsp_data = malloc(rsp_data_len);
    memcpy(rsp_data, (uint8_t *)&rsp, rsp_data_len);
    dfu_uart_send(rsp_data, rsp_data_len);
    free(rsp_data);
}

static void dfu_uart_image_end_handler(dfu_uart_env_t *env, uint8_t *data, uint16_t data_len)
{
    uint8_t status = DFU_UART_ERROR_NO_ERROR;
    LOG_I("dfu_uart_image_end_handler %d", env->image_current_length);
    if (env->image_current_length != env->image_total_length)
    {
        LOG_E("end file len error");
        status = DFU_UART_ERROR_FILE_LENGTH_ERROR;
    }

    if (status == DFU_UART_ERROR_NO_ERROR)
    {
        status = dfu_uart_crc_verification(env);
    }

    dfu_uart_image_end_rsp(env, status);
}

static void dfu_uart_start_rsp(dfu_uart_env_t *env, uint16_t result)
{
    LOG_I("dfu_uart_start_rsp %d", result);
    env->state = DFU_UART_STATE_NONE;
    dfu_start_rsp_t rsp;
    rsp.command = DFU_UART_START_RSP;
    rsp.length = 4;
    rsp.result = result;
    rsp.version = DFU_UART_VERSION;

    uint8_t *rsp_data;
    uint16_t rsp_data_len = sizeof(dfu_start_rsp_t);
    rsp_data = malloc(rsp_data_len);
    memcpy(rsp_data, (uint8_t *)&rsp, rsp_data_len);
    dfu_uart_send(rsp_data, rsp_data_len);
    free(rsp_data);
}

static void dfu_uart_start_handler(dfu_uart_env_t *env, uint8_t *data, uint16_t data_len)
{
    uint8_t result = DFU_UART_ERROR_NO_ERROR;
    dfu_uart_start_rsp(env, result);
}

static void dfu_uart_end_rsp(dfu_uart_env_t *env, uint16_t result)
{
    LOG_I("dfu_uart_end_rsp %d", result);
    dfu_end_rsp_t rsp;
    rsp.command = DFU_UART_END_RSP;
    rsp.length = 2;
    rsp.result = result;

    uint8_t *rsp_data;
    uint16_t rsp_data_len = sizeof(dfu_end_rsp_t);
    rsp_data = malloc(rsp_data_len);
    memcpy(rsp_data, (uint8_t *)&rsp, rsp_data_len);
    dfu_uart_send(rsp_data, rsp_data_len);
    free(rsp_data);
}

static void dfu_uart_end_handler(dfu_uart_env_t *env, uint8_t *data, uint16_t data_len)
{
    uint16_t result = DFU_UART_ERROR_NO_ERROR;

    dfu_uart_end_rsp(env, result);

    if (result == DFU_UART_ERROR_NO_ERROR)
    {
        dfu_running_img_set(DFU_RUNNING_ON_HCPU);
        drv_reboot();
    }
}


static void dfu_uart_command_process(dfu_uart_env_t *env, uint16_t command, uint8_t *data, rt_uint32_t size)
{
    LOG_I("dfu_uart_command_process %d", command);
    switch (command)
    {
        case DFU_UART_START_REQ:
        {
            dfu_uart_start_handler(env, data, size);
            break;
        }
        case DFU_UART_IMAGE_START_REQ:
        {
            dfu_uart_image_start_handler(env, data, size);
            break;
        }
        case DFU_UART_IMAGE_DATA_IND:
        {
            dfu_uart_image_data_handler(env, data, size);
            break;
        }
        case DFU_UART_IMAGE_END_REQ:
        {
            dfu_uart_image_end_handler(env, data, size);
            break;
        }
        case DFU_UART_END_REQ:
        {
            dfu_uart_end_handler(env, data, size);
            break;
        }
    }
}


static void dfu_uart_protocol_handler(dfu_uart_env_t *env, uint8_t *data, rt_uint32_t size)
{
    if (env->is_assemable == 0)
    {
        if (size < 6)
        {
            LOG_I("error msg len %d", size);
            return;
        }
        uint32_t header_front;
        uint16_t header_rear;
        uint16_t p = 0;

        memcpy(&header_front, data, SFUART_HEADER_FRONT_LEN);
        p = SFUART_HEADER_FRONT_LEN;

        memcpy(&header_rear, data + p, SFUART_HEADER_REAR_LEN);
        p += SFUART_HEADER_REAR_LEN;

        //LOG_I("header_front 0x%x, rear 0x%x", header_front,header_rear);

        if (header_front != SFUART_HEADER_FRONT)
        {
            LOG_I("HEADER ERROR F!");
            return;
        }

        if (header_rear != SFUART_HEADER_REAR)
        {
            LOG_I("HEADER ERROR R!");
            return;
        }

        if (size < DFU_UART_HEADER_EX_LEN)
        {
            // TODO: assemble
            LOG_I("error msg len2 %d", size);
            return;
        }

        uint16_t command;
        memcpy(&command, data + p, 2);
        p += 2;

        uint16_t data_len;
        memcpy(&data_len, data + p, 2);
        p += 2;

        if (data_len != size - DFU_UART_HEADER_EX_LEN)
        {
            // TODO: assemble
            LOG_I("current size %d, will assemble", size);

            env->is_assemable = 1;
            env->target_length = data_len + DFU_UART_HEADER_EX_LEN;
            LOG_I("ASSEMABLE %d", env->target_length);
            env->assemable_data = malloc(env->target_length);
            RT_ASSERT(env->assemable_data);
            memcpy(env->assemable_data, data, size);
            env->assemable_length = size;
            return;
        } else {
            dfu_uart_command_process(env, command, data + p, data_len);
        }
    } else {
        if (env->assemable_length + size > env->target_length)
        {
            LOG_I("assemable over %d, %d", env->assemable_length + size, env->target_length);
            RT_ASSERT(0);
        }

        // ("current offset %d, size %d", env->assemable_length, size);
        memcpy(env->assemable_data + env->assemable_length, data, size);
        env->assemable_length += size;

        // LOG_I("after offset %d", env->assemable_length);

        if (env->assemable_length == env->target_length)
        {
            LOG_I("assemable end %d", env->target_length);
            env->is_assemable = 0;

            uint16_t command;
            memcpy(&command, env->assemable_data + 6, 2);

            uint16_t data_len;
            memcpy(&data_len, env->assemable_data + 8, 2);

            dfu_uart_command_process(env, command, env->assemable_data + 10, data_len);
            free(env->assemable_data);
        }
    }
}

void dfu_forward_to_mb_entry(void *param)
{
    dfu_uart_env_t *env = dfu_uart_get_env();
    env->to_mb = rt_mb_create("fwd_mb", 64, RT_IPC_FLAG_FIFO);
    dfu_uart_device_init();

    // always send start rsp in power on in ota
    dfu_uart_start_rsp(env, 0);

    rt_uint32_t size;
    uint8_t *ptr;
    int written, offset = 0;
    rt_size_t read_len;
    while (1)
    {
        while (1)
        {
            rt_mb_recv(env->to_mb, &size, RT_WAITING_FOREVER);
            //rt_kprintf("(TB)read size %d, mb ptr %x\r\n", size, env->ipc_port);
            LOG_I("(TB)read size %d", size);
            if (!size)
                continue;
            ptr = malloc(size);
            RT_ASSERT(ptr);

            // Read from uart
            read_len = rt_device_read(env->device, 0, ptr, size);
            if (read_len < size)
                size = read_len;

            if (read_len == 0)
            {
                break;
            }

            dfu_uart_protocol_handler(env, ptr, size);
            //LOG_HEX("DFU_UART", 16, ptr, size);

            free(ptr);
        }
        rt_thread_mdelay(10);
    }
}

int dfu_uart_forward_init(void)
{
    rt_thread_t tid;
    // Forward data to MB
    tid = rt_thread_create("fwd_mb", dfu_forward_to_mb_entry, NULL, 2048, 15, 10);
    rt_thread_startup(tid);
    // Forward data to uart
    // tid = rt_thread_create("fwd_uart", dfu_forward_to_uart_entry, NULL, 4096, 15, 10);
    // rt_thread_startup(tid);
    return 0;
}

void dfu_uart_send(uint8_t *data, uint16_t len)
{
    //LOG_I("dfu_uart_send");
    dfu_uart_env_t *env = dfu_uart_get_env();
    if (env->device)
    {
        uint8_t *send_data;
        uint16_t send_len = SFUART_HEADER_LEN + len;

        send_data = malloc(send_len);
        //LOG_I("dfu_uart_send %d", send_len);
        RT_ASSERT(send_data);

        uint32_t header_front = SFUART_HEADER_FRONT;
        uint16_t header_rear = SFUART_HEADER_REAR;

        memcpy(send_data, &header_front, SFUART_HEADER_FRONT_LEN);
        memcpy(send_data + SFUART_HEADER_FRONT_LEN, &header_rear, SFUART_HEADER_REAR);
        memcpy(send_data + SFUART_HEADER_LEN, data, len);

        // LOG_HEX("UART SEND", 16, send_data, send_len);

        rt_size_t written = rt_device_write(env->device, 0, send_data, send_len);
        RT_ASSERT(send_len == written);

        free(send_data);
    } else {
        LOG_I("not ready");
    }
}

#endif


