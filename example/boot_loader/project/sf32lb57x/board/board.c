/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include "boot_flash.h"

// #define BOOT_DEVICE_FORCED

board_boot_device_type_t board_boot_device;

typedef struct
{
    uint8_t pkgid;
} board_info_t;

#ifdef CFG_BOOTROM
    static board_info_t board_info;
#endif /* CFG_BOOTROM */

void SystemClock_Config(void)
{
}

void board_gpio_set(pin_pad pad, int val, int is_porta)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    uint32_t pin;

    HAL_ASSERT((pad >= PAD_PA00) && (pad < PIN_PAD_MAX_H));
    pin = pad - PAD_PA00;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio1, &GPIO_InitStruct);

    HAL_GPIO_WritePin(hwp_gpio1, pin, (GPIO_PinState)val);
}

// Do not use HAL_PIN_Get in bootloader ROM
pin_function HAL_PIN_Idx2Func(int pad, int idx,  int hcpu)
{
    return 0;
}

void board_pinmux_mpi1_type1(void)
{
    uint32_t i;

    HAL_PIN_CompileTimeSet(PAD_SA10, MPI1_FLASH_CLK,  PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SA00, MPI1_FLASH_CS,   PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SA12, MPI1_FLASH_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SA02, MPI1_FLASH_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SA04, MPI1_FLASH_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_SA11, MPI1_FLASH_DIO3, PIN_PULLUP, 1);

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SA12 - PAD_SA00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SA00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SA00 + i, 1, 0);
    }
}

void board_pinmux_mpi1_type2(void)
{
    uint32_t i;

    HAL_PIN_CompileTimeSet(PAD_SA10, MPI1_FLASH_CLK,  PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SA03, MPI1_FLASH_CS,   PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SA12, MPI1_FLASH_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SA02, MPI1_FLASH_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SA01, MPI1_FLASH_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_SA09, MPI1_FLASH_DIO3, PIN_PULLUP, 1);

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SA12 - PAD_SA00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SA00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SA00 + i, 1, 0);
    }
}

void board_pinmux_mpi2(void)
{
    uint32_t i;

    HAL_PIN_CompileTimeSet(PAD_SB12, MPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SB06, MPI2_CS,  PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SB10, MPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SB05, MPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SB04, MPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_SB11, MPI2_DIO3, PIN_PULLUP, 1);

    /* DS1=0, DS0=1 (8mA drive), MPI3 may use SB at the same time so the configuration also work for MPI3 */
    for (i = 0; i < (PAD_SB12 - PAD_SB00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SB00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SB00 + i, 1, 0);
    }
}

void board_pinmux_mpi3(void)
{
    uint32_t i;

    HAL_PIN_CompileTimeSet(PAD_PA16, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_PA12, MPI3_CS,  PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_PA15, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_PA13, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_PA14, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_PA17, MPI3_DIO3, PIN_PULLUP, 1);
    HAL_PMU_ClearPadRetention(PAD_PA16);
    HAL_PMU_ClearPadRetention(PAD_PA12);
    HAL_PMU_ClearPadRetention(PAD_PA15);
    HAL_PMU_ClearPadRetention(PAD_PA13);
    HAL_PMU_ClearPadRetention(PAD_PA14);
    HAL_PMU_ClearPadRetention(PAD_PA17);

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_PA17 - PAD_PA12 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_PA12 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_PA12 + i, 1, 0);
    }
}

void board_pinmux_sd(void)
{
    HAL_PIN_CompileTimeSet(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
    HAL_PMU_ClearPadRetention(PAD_PA15);
    HAL_Delay_us(20);   // add a delay before clock setting to avoid wrong cmd happen

    HAL_PIN_CompileTimeSet(PAD_PA14, SD1_CLK,  PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);

    HAL_PMU_ClearPadRetention(PAD_PA14);
    HAL_PMU_ClearPadRetention(PAD_PA16);
    HAL_PMU_ClearPadRetention(PAD_PA17);
    HAL_PMU_ClearPadRetention(PAD_PA12);
    HAL_PMU_ClearPadRetention(PAD_PA13);
}

static uint8_t board_read_pkgid(void)
{
    uint8_t pkgid;
    int32_t r;
    const uint32_t *bank0_data;

    bank0_data = sifli_hw_efuse_get_bank0_data();
    r = HAL_EFUSE_Extract(bank0_data, EFUSE_PKGID_OFFSET, &pkgid, EFUSE_PKGID_SIZE);
    HAL_ASSERT(EFUSE_PKGID_SIZE == r);

    return pkgid;
}

#ifdef CFG_BOOTROM
static uint8_t board_read_ssen(void)
{
    uint8_t ssen;
    int32_t r;
    const uint32_t *bank0_data;

    bank0_data = sifli_hw_efuse_get_bank0_data();
    r = HAL_EFUSE_Extract(bank0_data, EFUSE_SSEN_OFFSET, &ssen, EFUSE_SSEN_SIZE);
    HAL_ASSERT(EFUSE_SSEN_SIZE == r);

    return ssen;
}

static uint8_t board_get_soc_boot_device(void)
{
    return GET_REG_VAL2(board_info.pkgid, PKGID_BOOT_DEVICE);
}

static uint8_t board_get_bootstrap_type(void)
{
    uint8_t bootstrap_type;
    bootstrap_type = (uint8_t)HAL_GPIO_ReadPin(hwp_gpio1, BOARD_BOOTSTRAP_PIN_BIT0);
    bootstrap_type |= ((uint8_t)HAL_GPIO_ReadPin(hwp_gpio1, BOARD_BOOTSTRAP_PIN_BIT1) << 1);

    return bootstrap_type;
}
#endif /* CFG_BOOTROM */

board_boot_device_type_t board_boot_from(void)
{
#ifdef CFG_BOOTROM
    uint8_t boot_device;
    uint8_t bootstrap_type;
    uint32_t tmp;
#endif /* CFG_BOOTROM */
    board_boot_device_type_t r;

#ifdef CFG_BOOTROM
    boot_device = board_get_soc_boot_device();
#ifdef BOOT_DEVICE_FORCED
    boot_device = PKGID_BOOT_DEVICE_EXT;
#endif /* BOOT_DEVICE_FORCED */

    if (boot_device == PKGID_BOOT_DEVICE_MPI2)
    {
        r = BOARD_BOOT_DEVICE_MPI2;
    }
    else if (boot_device == PKGID_BOOT_DEVICE_MPI1_TYPE1)
    {
        r = BOARD_BOOT_DEVICE_MPI1_TYPE1;
    }
    else if (boot_device == PKGID_BOOT_DEVICE_MPI1_TYPE2)
    {
        r = BOARD_BOOT_DEVICE_MPI1_TYPE2;
    }
    else
    {
        bootstrap_type = board_get_bootstrap_type();

        if (BOARD_BOOTSTRAP_FROM_EXT_NOR == bootstrap_type)
        {
            r = BOARD_BOOT_DEVICE_MPI3_NOR;
        }
        else if (BOARD_BOOTSTRAP_FROM_EXT_NAND == bootstrap_type)
        {
            r = BOARD_BOOT_DEVICE_MPI3_NAND;
        }
        else if (BOARD_BOOTSTRAP_FROM_EXT_SD == bootstrap_type)
        {
            r = BOARD_BOOT_DEVICE_SD;
        }
        else if (BOARD_BOOTSTRAP_FROM_EXT_EMMC == bootstrap_type)
        {
            r = BOARD_BOOT_DEVICE_EMMC;
        }
        tmp = HAL_Get_backup(RTC_BACKUP_BOOTOPT);
        MODIFY_REG(tmp, RTC_BACKUP_BOOTOPT_SRC_Msk, MAKE_REG_VAL2(r, RTC_BACKUP_BOOTOPT_SRC));
        HAL_Set_backup(RTC_BACKUP_BOOTOPT, tmp);
    }
#else
    r = GET_REG_VAL2(HAL_Get_backup(RTC_BACKUP_BOOTOPT), RTC_BACKUP_BOOTOPT_SRC);
#endif


    return r;
}

#ifdef CFG_BOOTROM
void board_ldo_en_ss_init(void)
{
    uint8_t ssen;
    uint32_t val;

    ssen = board_read_ssen();

    val = MAKE_REG_VAL2(GET_REG_VAL2(ssen, SSEN_LDO18), PMUC_PERI_LDO_LDO18_EN_SS);
    MODIFY_REG(hwp_pmuc->PERI_LDO, PMUC_PERI_LDO_LDO18_EN_SS_Msk, val);

    val = MAKE_REG_VAL2(GET_REG_VAL2(ssen, SSEN_VDD33_LDO2), PMUC_PERI_LDO_VDD33_LDO2_EN_SS);
    MODIFY_REG(hwp_pmuc->PERI_LDO, PMUC_PERI_LDO_VDD33_LDO2_EN_SS_Msk, val);
}
#endif /* CFG_BOOTROM */

void board_init(void)
{
#ifdef CFG_BOOTROM
    uint32_t delay;
    uint32_t boot_opt;

    board_info.pkgid = board_read_pkgid();
    if (0 == GET_REG_VAL2(board_info.pkgid, PKGID_LDO18_EN))
    {
        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, true, true);
    }
    if (0 == GET_REG_VAL2(board_info.pkgid, PKGID_LDO33_EN))
    {
        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, true, true);
    }

    boot_opt = HAL_Get_backup(RTC_BACKUP_BOOTOPT);
    delay = GET_REG_VAL2(boot_opt, BOOT_PD_Delay);
    if (delay)
    {
        board_gpio_set(MPI_POWER_PAD, 0, 1);
        HAL_PIN_CompileTimeSet(MPI_POWER_PAD, MPI_POWER_PAD_FUNC, PIN_NOPULL, 1);
        HAL_Delay_us(delay * 1000);
    }

    delay = GET_REG_VAL2(boot_opt, BOOT_PU_Delay);
    if (delay)
    {
        /* update DOR before enable DOER to avoid output 0 when enabling output */
        HAL_GPIO_WritePin(hwp_gpio1, MPI_POWER_PAD - PAD_PA00, 1);
        board_gpio_set(MPI_POWER_PAD, 1, 1);
        HAL_PIN_CompileTimeSet(MPI_POWER_PAD, MPI_POWER_PAD_FUNC, PIN_NOPULL, 1);
        HAL_Delay_us(delay * 1000);
    }
#endif /* CFG_BOOTROM */
}

