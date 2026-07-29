/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"

#ifdef SF32LB57X
#include "board.h"
#include "register.h"

#define BSP_PKGID_PSRAM_TYPE_NUM  (GET_REG_VAL2(PKGID_MPI1_PSRAM_TYPE_Msk, PKGID_MPI1_PSRAM_TYPE) + 1)

enum
{
#ifdef BSP_USING_PSRAM1
    BSP_PSRAM1_INDEX,
#endif /* BSP_USING_PSRAM1 */
#ifdef BSP_USING_PSRAM2
    BSP_PSRAM2_INDEX,
#endif /* BSP_USING_PSRAM2 */
    BSP_PSRAM_NUM
};

typedef struct
{
    /** whether type is valid, true: valid, false: invalid */
    bool  valid;
    /** PSRAM type from EFUSE_PKGID */
    uint8_t type;
} bsp_psram_info_t;

typedef struct
{
    /** mpi mode, SPI_FLASH_MODE_E */
    uint8_t mpi_mode: 4;
    /* pinmap mode, range: 1~3 */
    uint8_t pinmap_mode: 4;
} bsp_psram_type_t;


/* PKGID PSRAM TYPE -> MPI Mode and pinmap mapping table */
#ifdef BSP_USING_PSRAM1
HAL_RAM_RET_RODATA_SECT(bsp_psram_mpi1_type_mapping_tbl, const static bsp_psram_type_t bsp_psram_mpi1_type_mapping_tbl[BSP_PKGID_PSRAM_TYPE_NUM]) =
{
    {BSP_PSRAM1_PKG_TYPE0_MPI_MODE, BSP_PSRAM1_PKG_TYPE0_PINMAP_MODE},
    {BSP_PSRAM1_PKG_TYPE1_MPI_MODE, BSP_PSRAM1_PKG_TYPE1_PINMAP_MODE},
    {BSP_PSRAM1_PKG_TYPE2_MPI_MODE, BSP_PSRAM1_PKG_TYPE2_PINMAP_MODE},
    {BSP_PSRAM1_PKG_TYPE3_MPI_MODE, BSP_PSRAM1_PKG_TYPE3_PINMAP_MODE},
};
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
HAL_RAM_RET_RODATA_SECT(bsp_psram_mpi2_type_mapping_tbl, const static bsp_psram_type_t bsp_psram_mpi2_type_mapping_tbl[BSP_PKGID_PSRAM_TYPE_NUM]) =
{
    {BSP_PSRAM2_PKG_TYPE0_MPI_MODE, BSP_PSRAM2_PKG_TYPE0_PINMAP_MODE},
    {BSP_PSRAM2_PKG_TYPE1_MPI_MODE, BSP_PSRAM2_PKG_TYPE1_PINMAP_MODE},
    {BSP_PSRAM2_PKG_TYPE2_MPI_MODE, BSP_PSRAM2_PKG_TYPE2_PINMAP_MODE},
    {BSP_PSRAM2_PKG_TYPE3_MPI_MODE, BSP_PSRAM2_PKG_TYPE3_PINMAP_MODE},
};
#endif /* BSP_USING_PSRAM2 */


HAL_RETM_BSS_SECT(bsp_psram_info, static bsp_psram_info_t bsp_psram_info[BSP_PSRAM_NUM]);

HAL_RAM_RET_CODE_SECT(bsp_psram_init_info, int32_t bsp_psram_init_info(void))
{
    uint8_t pkgid;
    int32_t r;
    bsp_psram_info_t *info;

    if (sizeof(pkgid) < (EFUSE_PKGID_SIZE >> 3))
    {
        return -1;
    }
    r = HAL_EFUSE_Read2(EFUSE_PKGID_OFFSET, &pkgid, EFUSE_PKGID_SIZE);
    if (r < EFUSE_PKGID_SIZE)
    {
        return -2;
    }
#ifdef BSP_USING_PSRAM1
    info = &bsp_psram_info[BSP_PSRAM1_INDEX];
    info->type = GET_REG_VAL2(pkgid, PKGID_MPI1_PSRAM_TYPE);
    info->valid = true;
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
    info = &bsp_psram_info[BSP_PSRAM2_INDEX];
    info->type = GET_REG_VAL2(pkgid, PKGID_MPI2_PSRAM_TYPE);
    info->valid = true;
#endif /* BSP_USING_PSRAM2 */
    return 0;
}

HAL_RAM_RET_CODE_SECT(bsp_psram_get_mpi_mode, uint8_t bsp_psram_get_mpi_mode(uint32_t mpi_id))
{
    uint8_t mpi_mode = SPI_MODE_NOR;
    bsp_psram_info_t *info = NULL;

#ifdef BSP_USING_PSRAM1
    if (1 == mpi_id)
    {
        info = &bsp_psram_info[BSP_PSRAM1_INDEX];
        if (!info->valid)
        {
            if (0 != bsp_psram_init_info())
            {
                goto __EXIT;
            }
        }
        mpi_mode = bsp_psram_mpi1_type_mapping_tbl[info->type].mpi_mode;
    }
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
    if (2 == mpi_id)
    {
        info = &bsp_psram_info[BSP_PSRAM2_INDEX];
        if (!info->valid)
        {
            if (0 != bsp_psram_init_info())
            {
                goto __EXIT;
            }
        }
        mpi_mode = bsp_psram_mpi2_type_mapping_tbl[info->type].mpi_mode;
    }
#endif  /* BSP_USING_PSRAM2 */


__EXIT:
    return  mpi_mode;
}


#ifdef BSP_USING_PSRAM1
HAL_RAM_RET_CODE_SECT(bsp_psram1_pinmux_init, int32_t bsp_psram1_pinmux_init(void))
{
    int32_t ret = 0;
    bsp_psram_info_t *info;
    uint8_t pinmap_mode;
    uint32_t i;

    info = &bsp_psram_info[BSP_PSRAM1_INDEX];
    if (!info->valid)
    {
        if (0 != bsp_psram_init_info())
        {
            ret = -2;
            goto __EXIT;
        }
    }
    pinmap_mode = bsp_psram_mpi1_type_mapping_tbl[info->type].pinmap_mode;
    if (1 == pinmap_mode)
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
    else if (2 == pinmap_mode)
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
    else if (3 == pinmap_mode)
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
        ret  = -1;
    }
    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SA12 - PAD_SA00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SA00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SA00 + i, 1, 0);
    }


__EXIT:
    return ret;
}
#endif /* BSP_USING_PSRAM1 */

#ifdef BSP_USING_PSRAM2
HAL_RAM_RET_CODE_SECT(bsp_psram2_pinmux_init, int32_t bsp_psram2_pinmux_init(void))
{
    int32_t ret = 0;
    bsp_psram_info_t *info;
    uint8_t pinmap_mode;
    uint32_t i;

    info = &bsp_psram_info[BSP_PSRAM2_INDEX];
    if (!info->valid)
    {
        if (0 != bsp_psram_init_info())
        {
            ret = -2;
            goto __EXIT;
        }
    }
    pinmap_mode = bsp_psram_mpi2_type_mapping_tbl[info->type].pinmap_mode;
    if (1 == pinmap_mode)
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
    else if (2 == pinmap_mode)
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
    else if (3 == pinmap_mode)
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
        ret  = -1;
    }

    /* DS1=0, DS0=1 (8mA drive) */
    for (i = 0; i < (PAD_SB12 - PAD_SB00 + 1); i++)
    {
        HAL_PIN_Set_DS0(PAD_SB00 + i, 1, 1);
        HAL_PIN_Set_DS1(PAD_SB00 + i, 1, 0);
    }

__EXIT:
    return ret;
}
#endif /* BSP_USING_PSRAM2 */

#endif /* SF32LB57X */
