/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#ifdef USING_PARTITION_TABLE
    #include "ptab.h"
#endif /* USING_PARTITION_TABLE */

#ifndef USING_PTAB_V3

    /* Match the legacy CoreMark 57x HCPU windowing on v1/v2 PTAB boards. */
    #undef LPSYS_RAM_SIZE
    #define LPSYS_RAM_SIZE (32 * 1024)

    #undef HCPU_RAM_DATA_START_ADDR
    #define HCPU_RAM_DATA_START_ADDR (HPSYS_RAM0_BASE)

    #undef HCPU_RAM_DATA_SIZE
    #define HCPU_RAM_DATA_SIZE (HPSYS_RAM0_SIZE)
#endif /* USING_PTAB_V3 */

#endif  /* __MEM_MAP__ */
