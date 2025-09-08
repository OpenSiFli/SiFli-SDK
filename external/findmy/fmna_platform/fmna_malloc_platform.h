/*
*      Copyright (C) 2020 Apple Inc. All Rights Reserved.
*
*      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
*      which is contained in the License.txt file distributed with the Find My Network ADK,
*      and only to those who accept that license.
*/

#ifndef fmna_malloc_platform_h
#define fmna_malloc_platform_h

#define USE_FMNA_DEBUG_MALLOC

#ifdef USE_FMNA_DEBUG_MALLOC
    extern void fmna_free(void *ptr);
    extern void *fmna_malloc(size_t size);
    extern void *fmna_realloc(void *ptr, size_t size);
    extern void fmna_malloc_dump(void);

#else
    #define fmna_free(ptr) free(ptr)
    #define fmna_malloc(size) malloc(size)
    #define fmna_realloc(ptr, size) realloc(ptr, size)
#endif

#endif /* fmna_malloc_platform_h */
