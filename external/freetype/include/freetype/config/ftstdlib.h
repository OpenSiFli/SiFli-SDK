/****************************************************************************
 *
 * ftstdlib.h
 *
 *   ANSI-specific library and header configuration file (specification
 *   only).
 *
 * Copyright (C) 2002-2020 by
 * David Turner, Robert Wilhelm, and Werner Lemberg.
 *
 * This file is part of the FreeType project, and may only be used,
 * modified, and distributed under the terms of the FreeType project
 * license, LICENSE.TXT.  By continuing to use, modify, or distribute
 * this file you indicate that you have read the license and
 * understand and accept it fully.
 *
 */


/**************************************************************************
 *
 * This file is used to group all `#includes` to the ANSI~C library that
 * FreeType normally requires.  It also defines macros to rename the
 * standard functions within the FreeType source code.
 *
 * Load a file which defines `FTSTDLIB_H_` before this one to override it.
 *
 */


#ifndef FTSTDLIB_H_
#define FTSTDLIB_H_


#include <stddef.h>

#define ft_ptrdiff_t  ptrdiff_t


/**************************************************************************
 *
 *                          integer limits
 *
 * `UINT_MAX` and `ULONG_MAX` are used to automatically compute the size of
 * `int` and `long` in bytes at compile-time.  So far, this works for all
 * platforms the library has been tested on.
 *
 * Note that on the extremely rare platforms that do not provide integer
 * types that are _exactly_ 16 and 32~bits wide (e.g., some old Crays where
 * `int` is 36~bits), we do not make any guarantee about the correct
 * behaviour of FreeType~2 with all fonts.
 *
 * In these cases, `ftconfig.h` will refuse to compile anyway with a
 * message like 'couldn't find 32-bit type' or something similar.
 *
 */


#include <limits.h>

#define FT_CHAR_BIT    CHAR_BIT
#define FT_USHORT_MAX  USHRT_MAX
#define FT_INT_MAX     INT_MAX
#define FT_INT_MIN     INT_MIN
#define FT_UINT_MAX    UINT_MAX
#define FT_LONG_MIN    LONG_MIN
#define FT_LONG_MAX    LONG_MAX
#define FT_ULONG_MAX   ULONG_MAX


/**************************************************************************
 *
 *                character and string processing
 *
 */


#include <string.h>

#define ft_memchr   memchr
#define ft_memcmp   memcmp
#define ft_memcpy   memcpy
#define ft_memmove  memmove
#define ft_memset   memset
#define ft_strcat   strcat
#define ft_strcmp   strcmp
#define ft_strcpy   strcpy
#define ft_strlen   strlen
#define ft_strncmp  strncmp
#define ft_strncpy  strncpy
#define ft_strrchr  strrchr
#define ft_strstr   strstr


/**************************************************************************
 *
 *                          file handling
 *
 */


#include <stdio.h>

#include "rtconfig.h"

#if defined (RT_USING_DFS)
#include <dfs_posix.h>
#define FT_FILE     struct dfs_fd
extern struct dfs_fd* ft_fopen(const char* name, const char* mode);
extern long ft_ftell(struct dfs_fd* f);
extern int ft_fseek(struct dfs_fd* f, long offset, int whence);
extern size_t ft_fread(void* ptr, size_t size, size_t nitems, struct dfs_fd* f);
//extern size_t lv_font_write(const void* ptr, size_t size, size_t nitems,struct dfs_fd* f);
extern int ft_fclose(struct dfs_fd* f);
#endif

#define ft_sprintf  sprintf


/**************************************************************************
 *
 *                            sorting
 *
 */


#include <stdlib.h>

#define ft_qsort  qsort


/**************************************************************************
 *
 *                       memory allocation
 *
 */

#if 0
#if defined(FREETYPE_CACHE_IN_SRAM) || defined(BSP_USING_PC_SIMULATOR)
    #define ft_scalloc   calloc
    #define ft_sfree     free
    #define ft_smalloc   malloc
    #define ft_srealloc  realloc
#else
    extern struct rt_memheap app_ft_memheap;
    #define ft_scalloc(count, size)   rt_memheap_calloc(&app_ft_memheap, count, size)
    #define ft_sfree(p)     rt_memheap_free(p)
    #define ft_smalloc(n)    rt_memheap_alloc(&app_ft_memheap, n)
    #define ft_srealloc(p, newsize)  rt_memheap_realloc(&app_ft_memheap, p, newsize)
#endif
#else
extern void * ft_smalloc(size_t nbytes);
extern void ft_sfree(void *ptr);
extern void *ft_srealloc(void *ptr, size_t nbytes);
extern void *ft_scalloc(size_t count, size_t size);

#endif

/**************************************************************************
 *
 *                         miscellaneous
 *
 */


#define ft_strtol  strtol
#define ft_getenv  getenv


/**************************************************************************
 *
 *                        execution control
 *
 */


#include <setjmp.h>

#define ft_jmp_buf     jmp_buf  /* note: this cannot be a typedef since  */
/*       `jmp_buf` is defined as a macro */
/*       on certain platforms            */

#define ft_longjmp     longjmp
#define ft_setjmp( b ) setjmp( *(ft_jmp_buf*) &(b) ) /* same thing here */


/* The following is only used for debugging purposes, i.e., if   */
/* `FT_DEBUG_LEVEL_ERROR` or `FT_DEBUG_LEVEL_TRACE` are defined. */

#include <stdarg.h>


#endif /* FTSTDLIB_H_ */


/* END */
