/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_EPDV2_H
#define LVSF_EPDV2_H

/*
 * Keep the merged EPDV2 extensions isolated from the default SDK behavior.
 * The macro can still be overridden explicitly by the build system.
 */
#if !defined(EPDV2)
    #if defined(BSP_USING_BOARD_SF32_OED_EPD_V11) \
        || defined(BSP_USING_BOARD_SF32_OED_EPD_V12) \
        || defined(BSP_USING_BOARD_SF32_OED_EPD_V12_SPI)
        #define EPDV2 1
    #else
        #define EPDV2 0
    #endif
#endif

#endif /* LVSF_EPDV2_H */
