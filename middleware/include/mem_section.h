/**
  ******************************************************************************
  * @file   mem_section.h
  * @author Sifli software development team
  * @brief memory section define macro
  * @{
  ******************************************************************************
*/
/*
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
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
