/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "bf0_hal_pinmux.h"
#include "bf0_pin_const.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup PIMNUX PINMUX
  * @brief PINMUX HAL module driver
  * @{
  */

#if defined(HAL_PINMUX_MODULE_ENABLED)

void HAL_PIN_SetSipFlash1(PIN_MpiPinmapMode pinmap_mode)
{
    uint32_t i;

    if (PIN_MPI_PINMAP_MODE_4 == pinmap_mode)
    {
        HAL_PIN_CompileTimeSet(PAD_SA10, MPI1_FLASH_CLK,  PIN_NOPULL, 1);
        HAL_PIN_CompileTimeSet(PAD_SA00, MPI1_FLASH_CS,   PIN_NOPULL, 1);
        HAL_PIN_CompileTimeSet(PAD_SA12, MPI1_FLASH_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_CompileTimeSet(PAD_SA02, MPI1_FLASH_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_CompileTimeSet(PAD_SA04, MPI1_FLASH_DIO2, PIN_PULLUP, 1);
        HAL_PIN_CompileTimeSet(PAD_SA11, MPI1_FLASH_DIO3, PIN_PULLUP, 1);

    }
    else if (PIN_MPI_PINMAP_MODE_5 == pinmap_mode)
    {
        HAL_PIN_CompileTimeSet(PAD_SA10, MPI1_FLASH_CLK,  PIN_NOPULL, 1);
        HAL_PIN_CompileTimeSet(PAD_SA03, MPI1_FLASH_CS,   PIN_NOPULL, 1);
        HAL_PIN_CompileTimeSet(PAD_SA12, MPI1_FLASH_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_CompileTimeSet(PAD_SA02, MPI1_FLASH_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_CompileTimeSet(PAD_SA01, MPI1_FLASH_DIO2, PIN_PULLUP, 1);
        HAL_PIN_CompileTimeSet(PAD_SA09, MPI1_FLASH_DIO3, PIN_PULLUP, 1);
    }
    else
    {
        HAL_ASSERT(0);
    }


    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SA12 - PAD_SA00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SA00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SA00 + i, 1, 0);
    }
}


void HAL_PIN_SetSipFlash2(PIN_MpiPinmapMode pinmap_mode)
{
    uint32_t i;

    /* not used */
    (void)pinmap_mode;

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

void HAL_PIN_SetSipFlash3(PIN_MpiPinmapMode pinmap_mode)
{
    uint32_t i;

    (void)pinmap_mode;

    HAL_PIN_CompileTimeSet(PAD_SB09, MPI3_CLK, PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SB03, MPI3_CS, PIN_NOPULL, 1);
    HAL_PIN_CompileTimeSet(PAD_SB07, MPI3_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SB02, MPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_CompileTimeSet(PAD_SB01, MPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_CompileTimeSet(PAD_SB08, MPI3_DIO3, PIN_PULLUP, 1);

    /* DS1=0, DS0=1 (8mA drive), MPI3 may use SB at the same time so the configuration also work for MPI3 */
    for (i = 0; i < (PAD_SB12 - PAD_SB00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SB00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SB00 + i, 1, 0);
    }
}

void HAL_PIN_SetSipPsram1(PIN_MpiPinmapMode pinmap_mode)
{
    uint32_t i;

    if (PIN_MPI_PINMAP_MODE_1 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SA01, MPI1_PSRAM_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_PSRAM_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_PSRAM_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_PSRAM_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_PSRAM_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_PSRAM_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_PSRAM_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_PSRAM_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA00, MPI1_PSRAM_DM, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_PSRAM_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_PSRAM_CLKB, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_PSRAM_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_PSRAM_DQS, PIN_PULLDOWN, 1);
    }
    else if (PIN_MPI_PINMAP_MODE_2 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SA01, MPI1_PSRAM_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_PSRAM_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_PSRAM_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_PSRAM_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_PSRAM_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_PSRAM_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_PSRAM_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_PSRAM_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_PSRAM_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_PSRAM_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_PSRAM_DQSDM, PIN_PULLDOWN, 1);
    }
    else if (PIN_MPI_PINMAP_MODE_3 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SA01, MPI1_PSRAM_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_PSRAM_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_PSRAM_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_PSRAM_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_PSRAM_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_PSRAM_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_PSRAM_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_PSRAM_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_PSRAM_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_PSRAM_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_PSRAM_DQSDM, PIN_PULLDOWN, 1);
    }
    else
    {
        HAL_ASSERT(0);
    }

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SA12 - PAD_SA00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SA00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SA00 + i, 1, 0);
    }
}


void HAL_PIN_SetSipPsram2(PIN_MpiPinmapMode pinmap_mode)
{
    uint32_t i;

    if (PIN_MPI_PINMAP_MODE_1 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB00, MPI2_DM, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_CLKB, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DQS, PIN_PULLDOWN, 1);
    }
    else if (PIN_MPI_PINMAP_MODE_2 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DQSDM, PIN_PULLDOWN, 1);
    }
    else if (PIN_MPI_PINMAP_MODE_3 == pinmap_mode)
    {
        HAL_PIN_Set(PAD_SB01, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_CS,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_DQSDM, PIN_PULLDOWN, 1);
    }
    else
    {
        HAL_ASSERT(0);
    }

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SB12 - PAD_SB00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SB00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SB00 + i, 1, 0);
    }
}

#endif /* HAL_PINMUX_MODULE_ENABLED */

/**
  * @}
  */


/**
  * @}
  */