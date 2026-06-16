#ifndef _HAD_ZIPCONF_H
#define _HAD_ZIPCONF_H

/*
   zipconf.h -- platform specific include file

   This file was generated automatically by CMake
   based on ../cmake-zipconf.h.in.
 */

#include "dfs_posix.h"

#ifndef WIN32 /*When compiling in Visual C++, it conflicts with corecrt.h.*/
#undef chmod
#define chmod(x,y) 
#endif
extern long sys_ftell(struct dfs_fd *f);
extern struct dfs_fd *sys_fdopen(int fildes, const char * mode);
extern int sys_fileno(struct dfs_fd *d);
extern struct dfs_fd *sys_fopen(const char *name, const char *mode);
extern size_t sys_fread(void *ptr, size_t size, size_t nitems, struct dfs_fd *f);
extern size_t sys_fwrite(const void *ptr, size_t size, size_t nitems, struct dfs_fd *f);
extern int sys_fclose(struct dfs_fd *f);

#define ZIP_FILE struct dfs_fd
#define libzip_fopen sys_fopen
#define libzip_fwrite sys_fwrite
#define libzip_fread sys_fread
#define libzip_fclose sys_fclose
#define libzip_fileno sys_fileno
#define libzip_fdopen sys_fdopen
#define libzip_ferror(x) NULL
#define libzip_clearerr(x) NULL

#define LIBZIP_VERSION "1.11.3"
#define LIBZIP_VERSION_MAJOR 1
#define LIBZIP_VERSION_MINOR 11
#define LIBZIP_VERSION_MICRO 3

/* #undef ZIP_STATIC */

#if !defined(__STDC_FORMAT_MACROS)
#define __STDC_FORMAT_MACROS 1
#endif
#include <inttypes.h>

typedef int8_t zip_int8_t;
typedef uint8_t zip_uint8_t;
typedef int16_t zip_int16_t;
typedef uint16_t zip_uint16_t;
typedef int32_t zip_int32_t;
typedef uint32_t zip_uint32_t;
typedef int64_t zip_int64_t;
typedef uint64_t zip_uint64_t;

#define ZIP_INT8_MIN	 (-ZIP_INT8_MAX-1)
#define ZIP_INT8_MAX	 0x7f
#define ZIP_UINT8_MAX	 0xff

#define ZIP_INT16_MIN	 (-ZIP_INT16_MAX-1)
#define ZIP_INT16_MAX	 0x7fff
#define ZIP_UINT16_MAX	 0xffff

#define ZIP_INT32_MIN	 (-ZIP_INT32_MAX-1L)
#define ZIP_INT32_MAX	 0x7fffffffL
#define ZIP_UINT32_MAX	 0xffffffffLU

#define ZIP_INT64_MIN	 (-ZIP_INT64_MAX-1LL)
#define ZIP_INT64_MAX	 0x7fffffffffffffffLL
#define ZIP_UINT64_MAX	 0xffffffffffffffffULL

#endif /* zipconf.h */
