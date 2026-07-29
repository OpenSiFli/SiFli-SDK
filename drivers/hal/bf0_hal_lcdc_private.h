/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BF0_HAL_LCDC_PRIVATE_H__
#define __BF0_HAL_LCDC_PRIVATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAL_LCD_MODULE_ENABLED)
extern void rt_kprintf(const char *fmt, ...);

#define LCDC_LOG(...)   //do{rt_kprintf(__VA_ARGS__);rt_kprintf("\r\n");}while(0)
#define LCDC_LOG_D(...)  LCDC_LOG(__VA_ARGS__)
#define LCDC_LOG_I(...)  LCDC_LOG(__VA_ARGS__)
#define LCDC_LOG_E(...)  LCDC_LOG(__VA_ARGS__)
#define LCDC_PRINT_AREA(s,area) LCDC_LOG("%s: x0y0=%d,%d  x1y1=%d,%d  \n",s,(area)->x0,(area)->y0,(area)->x1,(area)->y1)


#if defined(USE_FULL_ASSERT)||defined(_SIFLI_DOXYGEN_) || defined(USE_LOOP_ASSERT)
#define HAL_LCDC_ASSERT  HAL_ASSERT
#else
#define HAL_LCDC_ASSERT(expr) if ((expr)==0) while (1)
#endif /* USE_FULL_ASSERT */

#define LCDC_TIMEOUT_SECONDS  1


extern uint8_t HAL_LCDC_GetPixelSize(HAL_LCDC_PixelFormat color_format);


#endif /* HAL_LCD_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif


#endif /* __BF0_HAL_LCDC_PRIVATE_H__ */