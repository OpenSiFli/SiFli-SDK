/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "register.h"
#include "dfu.h"
#include "drv_flash.h"
#include "drv_io.h"
#include "drv_psram.h"

// Target is retention memory 0x20000, 0x20000000 is temp before retention memory ready
#define HCPU_RAM_RETENTION_ADDR 0x20000000
#define REG_LOCK_PASSWORD       "66776677"

#define BOOT_INVALID_ADDR 0xFFFFFFFF

#define MPI1_POWER_PIN  (33)
#define MPI2_POWER_PIN  (26)
#define MPI3_POWER_PIN  (43)

typedef enum
{
    BOOT_FLASH1,
    BOOT_FLASH3,
    BOOT_FLASH4
} boot_flash_t;

typedef void (*ram_hook_handler)(void);

typedef int (*flash_enable_func)();

extern int BSP_Flash_hw1_init();
extern int BSP_Flash_hw3_init();
extern int BSP_Flash_hw4_init();
extern void flash_set_dual_mode(uint8_t id, uint8_t dual);
extern int rt_nand_init();

__STATIC_INLINE void dwtIpInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtIpDeinit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtReset(void)
{
    DWT->CYCCNT = 0; /* Clear DWT cycle counter */
}

__STATIC_INLINE uint32_t dwtGetCycles(void)
{
    return DWT->CYCCNT;
}


static uint32_t boot_get_flash_start_addr(boot_flash_t flash)
{
    uint32_t addr = BOOT_INVALID_ADDR;
    switch (flash)
    {
    case BOOT_FLASH1:
    {
        addr = QSPI1_MEM_BASE;
        break;
    }
    case BOOT_FLASH3:
    {
        addr = QSPI3_MEM_BASE;
        break;
    }
    case BOOT_FLASH4:
    {
        addr = QSPI4_MEM_BASE;
        break;
    }
    default:
        break;
    }
    return addr;
}

void boot_mpu_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}


void boot_ram(void)
{

    if (__HAL_SYSCFG_GET_BOOT_MODE() < 1)
    {
        volatile ram_hook_handler hook = (volatile ram_hook_handler)hwp_hpsys_aon->RESERVE0;
        if (hook)
            hook();
    }
}


void boot_image(int coreid, int force)
{
    struct image_header_enc *hdr;
    hdr = g_sec_config->running_imgs[coreid];
    if (hdr != (struct image_header_enc *)FLASH_UNINIT_32)
    {
        int flashid = hdr - &(g_sec_config->imgs[0]) + DFU_FLASH_IMG_LCPU;
        dfu_boot_img_in_flash(flashid);
    }
}

// Return:
// -1 means init failed
// 0 means init successful but magic number not matched,
// 1 means single flash,
// 2 means dual flash


static int boot_flash_init(boot_flash_t flash)
{
    int ret = -1;
    int flash_en;
    uint32_t flash_addr = boot_get_flash_start_addr(flash);
    // Only need check flash1 and flash2
    if (flash_addr == BOOT_INVALID_ADDR)
        return ret;
    uint8_t id = flash == BOOT_FLASH1 ? 0 : 1;
    flash_enable_func init_func;

    if (flash == BOOT_FLASH1)
        init_func = BSP_Flash_hw1_init;
    else if (flash == BOOT_FLASH3)
        init_func = BSP_Flash_hw3_init;
    else if (flash == BOOT_FLASH4)
        init_func = BSP_Flash_hw4_init;
    else
        HAL_ASSERT(0);

    // Enable flash with single mode
    //flash_set_dual_mode(id, 0);
    flash_en = rt_nand_init();//rt_hw_nand_init(); //init_func(0);

    // Enable failed
    if (!flash_en)
    {
        return ret;
    }
    else
        ret = 0; // flash existed

    do
    {
        ret = 1;
    }
    while (0);

    return ret;
}


uint32_t boot_enable_flash(void)
{
    uint32_t addr = BOOT_INVALID_ADDR;
    int ret;
#if  (defined (BSP_USING_SPI_FLASH))
    rt_hw_flash_var_init();
    do
    {
        int flash_en;
        /* init divider for bootloader */
        BSP_SetFlash1DIV(1);
        BSP_SetFlash3DIV(6);
        BSP_SetFlash4DIV(6);

#ifdef BSP_ENABLE_MPI3
        //HAL_PIN_SetFlash3();
        HAL_PIN_Set(PAD_PA46, MPI3_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA44, MPI3_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA50, MPI3_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA48, MPI3_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA47, MPI3_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA45, MPI3_DIO3, PIN_PULLUP, 1);

        if (boot_flash_init(BOOT_FLASH3) >= 0)
            addr = boot_get_flash_start_addr(BOOT_FLASH3);
#endif
#ifdef BSP_ENABLE_MPI4
        HAL_PIN_SetFlash4();
        HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA30, MPI4_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA40, MPI4_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA37, MPI4_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA36, MPI4_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA38, MPI4_DIO3, PIN_PULLUP, 1);
        if (boot_flash_init(BOOT_FLASH4) >= 0)
            addr = boot_get_flash_start_addr(BOOT_FLASH4);
#endif

    }
    while (0);
#endif /* (defined (BSP_USING_FLASH)||defined (BSP_USING_SPI_FLASH))  */
    return addr;
}

static void config_psram_pinmux(void)
{
    // PSRAM1
    HAL_PIN_Set(PAD_SA00, MPI1_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA01, MPI1_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA02, MPI1_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA03, MPI1_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA04, MPI1_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA05, MPI1_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA06, MPI1_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA07, MPI1_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA08, MPI1_DQS0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA09, MPI1_CLK,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA10, MPI1_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SA11, MPI1_DIO8, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA12, MPI1_DIO9, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA13, MPI1_DIO10, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA14, MPI1_DIO11, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA15, MPI1_DIO12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA16, MPI1_DIO13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA17, MPI1_DIO14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA18, MPI1_DIO15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SA19, MPI1_DQS1, PIN_PULLDOWN, 1);

    // PSRAM2
    HAL_PIN_Set(PAD_SB00, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB01, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB02, MPI2_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB03, MPI2_DIO3, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB04, MPI2_DIO4, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB05, MPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB06, MPI2_DIO6, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB07, MPI2_DIO7, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB08, MPI2_DQS0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB09, MPI2_CLK,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB10, MPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_SB11, MPI2_DIO8, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB12, MPI2_DIO9, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB13, MPI2_DIO10, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB14, MPI2_DIO11, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB15, MPI2_DIO12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB16, MPI2_DIO13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB17, MPI2_DIO14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB18, MPI2_DIO15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_SB19, MPI2_DQS1, PIN_PULLDOWN, 1);


}
void boot_images()
{
    int i;
    int flash_en;
    uint32_t start_addr;

#ifdef CODE_IN_RAM
    boot_mpu_img((uint8_t *)(HPSYS_RAM0_BASE + 0x20000));
#endif

    /* set MPI1 clock divider for PSRAM */
    BSP_SetFlash1DIV(1);
    BSP_SetFlash2DIV(1);

    HAL_HPAON_EnableXT48();

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

    HAL_PMU_EnableDLL(1);

#if defined (BSP_USING_PSRAM)
    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (2 * 24000))
    {
    }

    HAL_RCC_HCPU_EnableDLL1(240000000);
    /* set hdiv again to make HCLK run on 240MHz actually
     * Although the default hdiv registe value is 1, hardware would use 2 as the default hdiv.
     * After write the hdiv register, hardware would use the updated value.
     */
    HAL_RCC_HCPU_SetDiv(1, 2, 5);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);

    __HAL_SYSCFG_HPBG_EN();
    __HAL_SYSCFG_HPBG_VDDPSW_EN();

    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (20 * 2400))
    {
    }

    HAL_RCC_HCPU_EnableDLL2(336000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
    config_psram_pinmux();
    rt_psram_init();
#endif
    HAL_Delay_us(0);

    start_addr = boot_enable_flash();
    // Enable failed just back to bootloader loop.
    if (start_addr == BOOT_INVALID_ADDR)
        return;

    {
        extern void mbedtls_aes_lib_init(void);
        mbedtls_aes_lib_init();
    }

    rt_flash_disable_aes(start_addr);
    /* init AES_ACC as normal mode */
    __HAL_SYSCFG_CLEAR_SECURITY();

    g_sec_config = (struct sec_configuration *)FLASH_BASE_ADDR;

    if (g_sec_config->magic == SEC_CONFIG_MAGIC)
    {
        if (g_sec_config->running_imgs[CORE_BOOT] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = g_sec_config->running_imgs[CORE_BOOT] - &(g_sec_config->imgs[0]) + DFU_FLASH_IMG_LCPU;
            dfu_boot_img_in_flash(flash_id);
        }

        if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT1)].length != FLASH_UNINIT_32)
        {
            dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT1);
        }

        boot_image(CORE_HCPU, 0);
    }
}


static void boot_flash_power_on(void)
{
#define V58_PIN   (58U)
    uint8_t pin = V58_PIN;

#if 0
    //GPIO_InitTypeDef gpio_config;
    //GPIO_A39
    gpio_config.Pin = 39;
    gpio_config.Mode = GPIO_MODE_OUTPUT;
    gpio_config.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(hwp_gpio1, &gpio_config);
    HAL_GPIO_WritePin(hwp_gpio1, 39, GPIO_PIN_SET);
#endif


    /* enable V43 */
    pin = 43 - 32;
    (hwp_gpio1 + 1)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 1)->DOR |= (1UL << pin);

    /* enable V33 */
    pin = 33 - 32;
    (hwp_gpio1 + 1)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 1)->DOR |= (1UL << pin);

    /* enable V26 */
    pin = 26;
    (hwp_gpio1 + 0)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 0)->DOR |= (1UL << pin);

#ifdef SOC_BF0_HCPU
    // Enable PADA
    HAL_HPAON_ENABLE_PAD();
#endif /* SOC_BF0_HCPU */


    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (2 * 24000))
    {
    }

    dwtIpDeinit();

#if defined(SOC_SF32LB58X) && defined(FPGA)
    HAL_QSPI_SET_RXDELAY(0, 0, 2);
    HAL_QSPI_SET_RXDELAY(1, 0, 2);
#endif
}


static void boot_efuse_init(void)
{

    HAL_EFUSE_Init();
    sifli_hw_efuse_read_all();
    /* init hook to NULL as bss is initialized yet */
    g_dfu_efuse_read_hook = NULL;
}

void hw_preinit0(void)
{
    // Enable cache due to boot flow won't enable cache.
    SCB_EnableICache();
    SCB_EnableDCache();
    // 1. Read efuse first to take efuse effect.
    boot_efuse_init();

    // 2. If ram hook existed, just jump to ram.
    boot_ram();

    // 3. Power on flash and wait it's stable.
    boot_flash_power_on();
}



int regop_lock_check(char *passwd, uint32_t len)
{
    int ret = -1;
    if (strncmp((const char *)passwd, REG_LOCK_PASSWORD, sizeof(REG_LOCK_PASSWORD)) == 0)
        ret = 0;
    return ret;
}


int main(void)
{
    int count = 1;

    //g_sec_config = (struct sec_configuration *)boot_enable_flash();
#if 0
    rt_flash_set_alias(FLASH_START, FLASH_START, 0, 0);
    g_sec_config = (struct sec_configuration *)FLASH_START;
    if (g_sec_config->magic != SEC_CONFIG_MAGIC)
    {
        // TODO: patch not installed
        uint32_t magic = SEC_CONFIG_MAGIC;
        sec_flash_erase(DFU_FLASH_SEC_CONFIG, 0, sizeof(*g_sec_config));
        sec_flash_write(DFU_FLASH_SEC_CONFIG, 0, (uint8_t *)&magic, 4);
    }
#endif
    dfu_flash_init();

    while (count++)
    {
        rt_thread_mdelay(1000000);
    }

    return RT_EOK;
}

