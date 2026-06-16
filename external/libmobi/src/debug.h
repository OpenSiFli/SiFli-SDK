/** @file debug.h
 *
 * Copyright (c) 2014 Bartek Fabiszewski
 * http://www.fabiszewski.net
 *
 * This file is part of libmobi.
 * Licensed under LGPL, either version 3, or any later.
 * See <http://www.gnu.org/licenses/>
 */

#ifndef libmobi_debug_h
#define libmobi_debug_h

#include "config.h"
#include "mobi.h"

#ifndef MOBI_DEBUG
#define MOBI_DEBUG 0 /**< Turn on debugging, set this on by running "configure --enable-debug" */
#endif

#if MOBI_DEBUG_ALLOC
/**
 @defgroup mobi_debug Debug wrappers for memory allocation functions
 
 Set this on by running "configure --enable-debug-alloc"
 @{
 */
#define free(x) debug_free(x, __FILE__, __LINE__)
#define malloc(x) debug_malloc(x, __FILE__, __LINE__)
#define realloc(x, y) debug_realloc(x, y, __FILE__, __LINE__)
#define calloc(x, y) debug_calloc(x, y, __FILE__, __LINE__)
/** @} */
#endif

void debug_free(void *ptr, const char *file, const int line);
void *debug_malloc(const size_t size, const char *file, const int line);
void *debug_realloc(void *ptr, const size_t size, const char *file, const int line);
void *debug_calloc(const size_t num, const size_t size, const char *file, const int line);
void print_indx(const MOBIIndx *indx);
void print_indx_infl_old(const MOBIIndx *indx);
void print_indx_orth_old(const MOBIIndx *indx);

#if defined(RT_USING_DFS) && !MOBI_DEBUG_ALLOC
#include "app_mem.h"

static inline void *libmobi_anim_malloc(size_t size) {
    return size ? app_anim_alloc(size) : NULL;
}

static inline void *libmobi_anim_calloc(size_t count, size_t size) {
    if (!count || !size) {
        return NULL;
    }
    return app_anim_calloc(count, size);
}

static inline void *libmobi_anim_realloc(void *ptr, size_t nbytes) {
    if (!nbytes) {
        if (ptr) {
            app_anim_free(ptr);
        }
        return NULL;
    }
    return app_anim_realloc(ptr, nbytes);
}

static inline void libmobi_anim_free(void *ptr) {
    if (ptr) {
        app_anim_free(ptr);
    }
}

static inline char *libmobi_anim_strdup(const char *s) {
    return s ? app_anim_strdup(s) : NULL;
}

#define malloc(x) libmobi_anim_malloc(x)
#define calloc(x, y) libmobi_anim_calloc(x, y)
#define realloc(x, y) libmobi_anim_realloc(x, y)
#define free(x) libmobi_anim_free(x)
#ifdef strdup
#undef strdup
#endif
#define strdup(x) libmobi_anim_strdup(x)
#endif

/**
 @brief Macro for printing debug info to stderr. Wrapper for fprintf
 @param[in] fmt Format
 @param[in] ... Additional arguments
 */
#if (MOBI_DEBUG)
#define debug_print(fmt, ...) { \
    fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, \
    __LINE__, __func__, __VA_ARGS__); \
}
#else
#define debug_print(fmt, ...)
#endif

#endif
