/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MEM_SECTION_H
#define MEM_SECTION_H
#include "rtthread.h"
#include "section.h"

/**
 ****************************************************************************************
* @addtogroup mem_pool_mng Memory Pool Manager
* @ingroup middleware
* @brief Memory Pool Manager
* @{
****************************************************************************************
*/

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************
 *
 *  ITCM non-retained section
 *
 ********************************************************************/
/** ITCM non-retained code section */
#define ITCM_NON_RET_CODE_SEC(section_name)        SECTION(STRINGIFY(.itcm_non_ret_text_##section_name))

/** ITCM non-retained rodata section */
#define ITCM_NON_RET_RODATA_SEC(section_name)      SECTION(STRINGIFY(.itcm_non_ret_rodata_##section_name))

/** ITCM non-retained bss section begin*/
#define ITCM_NON_RET_BSS_SECT_BEGIN(section_name)  SECTION_ZIDATA_BEGIN(.itcm_non_ret_bss_##section_name)
/** ITCM non-retained bss section end*/
#define ITCM_NON_RET_BSS_SECT_END                  SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define ITCM_NON_RET_BSS_SECT(section_name)        //SECTION(STRINGIFY(.bss.itcm_non_ret_bss_##section_name))
#else
#define ITCM_NON_RET_BSS_SECT(section_name)        SECTION(STRINGIFY(.bss.itcm_non_ret_bss_##section_name))
#endif

/** ITCM non-retained rwdata section */
#define ITCM_NON_RET_DATA_SECT(section_name)       SECTION(STRINGIFY(.itcm_non_ret_data_##section_name))

/********************************************************************
 *
 *  Retention Memory section
 *
 ********************************************************************/
/** code section in retention memory */
#if  defined(_MSC_VER)
#define RETM_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define RETM_CODE_SECT(section_name, func)        func SECTION(STRINGIFY(.retm_text_##section_name))
#else
#define RETM_CODE_SECT(section_name, func)        SECTION(STRINGIFY(.retm_text_##section_name)) func
#endif /* _MSC_VER */

/** rodata section in retention memory */
#define RETM_RODATA_SECT(section_name)      SECTION(STRINGIFY(.retm_rodata_##section_name))

/** retention memory bss section begin*/
#define RETM_BSS_SECT_BEGIN(section_name)  SECTION_ZIDATA_BEGIN(.bss.retm_bss_##section_name)
/** retention memory bss section end*/
#define RETM_BSS_SECT_END                  SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define RETM_BSS_SECT(section_name)        //SECTION(STRINGIFY(.bss.retm_bss_##section_name))
#else
#define RETM_BSS_SECT(section_name)        SECTION(STRINGIFY(.bss.retm_bss_##section_name))
#endif

/** rwdata section in retention memory */
#define RETM_DATA_SECT(section_name)       SECTION(STRINGIFY(.retm_data_##section_name))

/********************************************************************
 *
 *  L1 non-retained section
 *
 ********************************************************************/
/** L1 non-retained code section */
#define L1_NON_RET_CODE_SECT(section_name)         SECTION(STRINGIFY(.l1_non_ret_text_##section_name))

/** L1 non-retained rodata section */
#define L1_NON_RET_RODATA_SECT(section_name)       SECTION(STRINGIFY(.l1_non_ret_rodata_##section_name))

/** L1 non-retained bss section begin*/
#define L1_NON_RET_BSS_SECT_BEGIN(section_name)    SECTION_ZIDATA_BEGIN(.l1_non_ret_bss_##section_name)
/** L1 non-retained bss section end*/
#define L1_NON_RET_BSS_SECT_END                    SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define L1_NON_RET_BSS_SECT(section_name, var)     var     //SECTION(STRINGIFY(.bss.l1_non_ret_bss_##section_name))
#elif  defined(_MSC_VER)
#define L1_NON_RET_BSS_SECT(section_name, var)     var
#else
#define L1_NON_RET_BSS_SECT(section_name, var)     var SECTION(STRINGIFY(.bss.l1_non_ret_bss_##section_name))
#endif

/** L1 non-retained rwdata section */
#define L1_NON_RET_DATA_SECT(section_name)         SECTION(STRINGIFY(.l1_non_ret_data_##section_name))

/********************************************************************
 *
 *  L1 retained section
 *
 ********************************************************************/

/** L1 retained code section */
#if  defined(_MSC_VER)
#define L1_RET_CODE_SECT(section_name, func)        func
#elif defined(__IAR_SYSTEMS_ICC__)
#define L1_RET_CODE_SECT(section_name, func)        func SECTION(STRINGIFY(.l1_ret_text_##section_name))
#else
#define L1_RET_CODE_SECT(section_name, func)        SECTION(STRINGIFY(.l1_ret_text_##section_name)) func
#endif /* _MSC_VER */

/** L1 retained rodata section */
#define L1_RET_RODATA_SECT(section_name)           SECTION(STRINGIFY(.l1_ret_rodata_##section_name))

/** L1 retained bss section begin */
#define L1_RET_BSS_SECT_BEGIN(section_name)        SECTION_ZIDATA_BEGIN(.l1_ret_bss_##section_name)
/** L1 retained bss section end */
#define L1_RET_BSS_SECT_END                        SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define L1_RET_BSS_SECT(section_name, var)              var //SECTION(STRINGIFY(.bss.l1_ret_bss_##section_name))
#elif  defined(_MSC_VER)
#define L1_RET_BSS_SECT(section_name, var)              var
#else
#define L1_RET_BSS_SECT(section_name, var)              var SECTION(STRINGIFY(.bss.l1_ret_bss_##section_name))
#endif

/** L1 retained rwdata section */
#define L1_RET_DATA_SECT(section_name)             SECTION(STRINGIFY(.l1_ret_data_##section_name))

/********************************************************************
 *
 *  L2 non-cachable non-retained section
 *
 ********************************************************************/

/** L2 non-retained bss section begin */
#define L2_NON_RET_BSS_SECT_BEGIN(section_name)    SECTION_ZIDATA_BEGIN(.l2_non_ret_bss_##section_name)
/** L2 non-retained bss section end */
#define L2_NON_RET_BSS_SECT_END                    SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define L2_NON_RET_BSS_SECT(section_name, var)      var     //SECTION(STRINGIFY(.bss.l2_non_ret_bss_##section_name))
#elif defined(_MSC_VER)
#define L2_NON_RET_BSS_SECT(section_name, var)      var
#else
#define L2_NON_RET_BSS_SECT(section_name, var )     var SECTION(STRINGIFY(.bss.l2_non_ret_bss_##section_name))
#endif

/** L2 non-retained rwdata section */
#define L2_NON_RET_DATA_SECT(section_name)         SECTION(STRINGIFY(.l2_non_ret_data_##section_name))

/********************************************************************
 *
 *  L2 non-cachable retained section
 *
 ********************************************************************/

/** L2 retained bss section begin */
#define L2_RET_BSS_SECT_BEGIN(section_name)        SECTION_ZIDATA_BEGIN(.l2_ret_bss_##section_name)
/** L2 retained bss section end */
#define L2_RET_BSS_SECT_END                        SECTION_ZIDATA_END
#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define L2_RET_BSS_SECT(section_name)              //SECTION(STRINGIFY(.bss.l2_ret_bss_##section_name))
#elif defined(_MSC_VER)
#define L2_RET_BSS_SECT(section_name)              //SECTION(STRINGIFY(.bss.l2_ret_bss_##section_name))
#else
#define L2_RET_BSS_SECT(section_name)              SECTION(STRINGIFY(.bss.l2_ret_bss_##section_name))
#endif

/** L2 retained rwdata section */
#define L2_RET_DATA_SECT(section_name)             SECTION(STRINGIFY(.l2_ret_data_##section_name))

/********************************************************************
 *
 *  L2 cachable non-retained section
 *
 ********************************************************************/

/** L2 cachable non-retained bss section begin*/
#define L2_CACHE_NON_RET_BSS_SECT_BEGIN(section_name)    SECTION_ZIDATA_BEGIN(.l2_cache_non_ret_bss_##section_name)
/** L2 cachable non-retained bss section */
#define L2_CACHE_NON_RET_BSS_SECT_END                    SECTION_ZIDATA_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
#define L2_CACHE_NON_RET_BSS_SECT(section_name)         //SECTION(STRINGIFY(.bss.l2_cache_non_ret_bss_##section_name))
#else
#define L2_CACHE_NON_RET_BSS_SECT(section_name)         SECTION(STRINGIFY(.bss.l2_cache_non_ret_bss_##section_name))
#endif

/** L2 cachable non-retained rwdata section */
#define L2_CACHE_NON_RET_DATA_SECT(section_name)         SECTION(STRINGIFY(.l2_cache_non_ret_data_##section_name))

/********************************************************************
 *
 *  L2 cachable retained section
 *
 ********************************************************************/

/** L2 cachable retained bss section begin*/
#define L2_CACHE_RET_BSS_SECT_BEGIN(section_name)        SECTION_ZIDATA_BEGIN(.l2_cache_ret_bss_##section_name)
/** L2 cachable retained bss section end*/
#define L2_CACHE_RET_BSS_SECT_END                        SECTION_ZIDATA_END
#if defined(__CC_ARM) || defined(__CLANG_ARM)
/* ARMCC use ITCM_NON_RET_BSS_SECT_BEGIN for backward compatibility, such that no need to update link file */
#define L2_CACHE_RET_BSS_SECT(section_name)              //SECTION(STRINGIFY(.bss.l2_cache_ret_bss_##section_name))
#elif defined(_MSC_VER)
#define L2_CACHE_RET_BSS_SECT(section_name)              //SECTION(STRINGIFY(.bss.l2_cache_ret_bss_##section_name))
#else
#define L2_CACHE_RET_BSS_SECT(section_name)              SECTION(STRINGIFY(.bss.l2_cache_ret_bss_##section_name))
#endif

/** L2 cachable retained rwdata section */
#define L2_CACHE_RET_DATA_SECT(section_name)             SECTION(STRINGIFY(.l2_cache_ret_data_##section_name))

#ifdef __cplusplus
}
#endif
/// @} mem_pool_mng

/// @} file
#endif

