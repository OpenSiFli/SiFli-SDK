// Copyright 2014 PDFium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Original code copyright 2014 Foxit Software Inc. http://www.foxitsoftware.com

#include "rtconfig.h"
#include "../../include/fxcrt/fx_basic.h"

#if defined(SOLUTION)
#include <rtthread.h>
extern "C" {
void       *app_anim_alloc(size_t size);
void        app_anim_free(void *ptr);
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

static volatile int    g_fxmem_oom = 0;
static volatile int    g_fxmem_oom_count = 0;
static volatile size_t g_fxmem_oom_maxsize = 0;

typedef struct FXMEM_BlockHeader {
    unsigned int magic;
    size_t size;
#if defined(SOLUTION)
    struct FXMEM_BlockHeader *prev;
    struct FXMEM_BlockHeader *next;
    const char *file;
    int line;
    void *caller;
#endif
} FXMEM_BlockHeader;

#define FXMEM_BLOCK_MAGIC      0x5044464du
#define FXMEM_BLOCK_FREED      0x46524545u

#if defined(_MSC_VER)
#define FXMEM_CALLER()         NULL
#else
#define FXMEM_CALLER()         __builtin_return_address(0)
#endif

#if defined(SOLUTION)
static FXMEM_BlockHeader *g_fxmem_blocks = NULL;
static volatile int       g_fxmem_block_count = 0;
static volatile size_t    g_fxmem_block_bytes = 0;

static void FXMEM_LinkBlock(FXMEM_BlockHeader *header, const char *file, int line, void *caller)
{
    header->file = file;
    header->line = line;
    header->caller = caller;
    header->prev = NULL;
    header->next = g_fxmem_blocks;
    if (g_fxmem_blocks) {
        g_fxmem_blocks->prev = header;
    }
    g_fxmem_blocks = header;
    g_fxmem_block_count++;
    g_fxmem_block_bytes += header->size;
}

static void FXMEM_UnlinkBlock(FXMEM_BlockHeader *header)
{
    if (header->prev) {
        header->prev->next = header->next;
    } else if (g_fxmem_blocks == header) {
        g_fxmem_blocks = header->next;
    }
    if (header->next) {
        header->next->prev = header->prev;
    }
    if (g_fxmem_block_count > 0) {
        g_fxmem_block_count--;
    }
    if (g_fxmem_block_bytes >= header->size) {
        g_fxmem_block_bytes -= header->size;
    } else {
        g_fxmem_block_bytes = 0;
    }
    header->prev = NULL;
    header->next = NULL;
}
#endif

static void FXMEM_RecordOOM(const char *func, size_t size, const char *reason, void *caller, const char *file, int line)
{
    g_fxmem_oom = 1;
    g_fxmem_oom_count++;
    if (size > g_fxmem_oom_maxsize) {
        g_fxmem_oom_maxsize = size;
    }
#if defined(SOLUTION)
    rt_kprintf("pdfium_oom: %s size=%u count=%d max=%u reason=%s caller=0x%08x loc=%s:%d\n",
               func ? func : "?", (unsigned int)size,
               g_fxmem_oom_count, (unsigned int)g_fxmem_oom_maxsize,
               reason ? reason : "?", (unsigned int)(uintptr_t)caller,
               file ? file : "?", line);
#endif
}

static void* FXMEM_DefaultAllocWithCaller(size_t byte_size, int flags, void *caller, const char *file, int line)
{
    FXMEM_BlockHeader *header;
    size_t real_size;
    void *p = NULL;

    if (byte_size == 0)
        return NULL;

    if (byte_size > (~(size_t)0) - sizeof(FXMEM_BlockHeader)) {
        FXMEM_RecordOOM("FXMEM_DefaultAlloc", byte_size, "overflow", caller, file, line);
        return NULL;
    }

    real_size = byte_size + sizeof(FXMEM_BlockHeader);
#if defined(SOLUTION)
    p = app_anim_alloc(real_size);
#else
    p = malloc(real_size);
#endif
    if (!p) {
        FXMEM_RecordOOM("FXMEM_DefaultAlloc", byte_size, "alloc_fail", caller, file, line);
        return NULL;
    }

    header = (FXMEM_BlockHeader *)p;
    header->magic = FXMEM_BLOCK_MAGIC;
    header->size = byte_size;
#if defined(SOLUTION)
    FXMEM_LinkBlock(header, file, line, caller);
#endif
    return (void *)(header + 1);
}

void* FXMEM_DefaultAlloc(size_t byte_size, int flags)
{
    return FXMEM_DefaultAllocWithCaller(byte_size, flags, FXMEM_CALLER(), NULL, 0);
}

void* FXMEM_DefaultAllocDebug(size_t byte_size, int flags, const char *file, int line)
{
    return FXMEM_DefaultAllocWithCaller(byte_size, flags, FXMEM_CALLER(), file, line);
}

void* FXMEM_DefaultCalloc(size_t count, size_t byte_size, int flags)
{
    size_t total;
    void *p;

    if (count == 0 || byte_size == 0)
        return NULL;

    if (count > (~(size_t)0) / byte_size) {
        FXMEM_RecordOOM(__func__, byte_size, "overflow", FXMEM_CALLER(), NULL, 0);
        return NULL;
    }

    total = count * byte_size;
    p = FXMEM_DefaultAllocWithCaller(total, flags, FXMEM_CALLER(), NULL, 0);
    if (!p)
        return NULL;
    FXSYS_memset(p, 0, total);
    return p;
}

void* FXMEM_DefaultCallocDebug(size_t count, size_t byte_size, int flags, const char *file, int line)
{
    size_t total;
    void *p;

    if (count == 0 || byte_size == 0)
        return NULL;

    if (count > (~(size_t)0) / byte_size) {
        FXMEM_RecordOOM("FXMEM_DefaultCalloc", byte_size, "overflow", FXMEM_CALLER(), file, line);
        return NULL;
    }

    total = count * byte_size;
    p = FXMEM_DefaultAllocWithCaller(total, flags, FXMEM_CALLER(), file, line);
    if (!p)
        return NULL;
    FXSYS_memset(p, 0, total);
    return p;
}

void* FXMEM_DefaultRealloc(void* pointer, size_t new_size, int flags)
{
    FXMEM_BlockHeader *old_header;
    size_t old_size;
    void *new_ptr;

    if (!pointer)
        return FXMEM_DefaultAllocWithCaller(new_size, flags, FXMEM_CALLER(), NULL, 0);
    if (new_size == 0) {
        FXMEM_DefaultFree(pointer, flags);
        return NULL;
    }

    old_header = ((FXMEM_BlockHeader *)pointer) - 1;
    if (old_header->magic != FXMEM_BLOCK_MAGIC) {
        FXMEM_RecordOOM(__func__, new_size, "realloc_badptr", FXMEM_CALLER(), NULL, 0);
        return NULL;
    }
    old_size = old_header->size;
    if (new_size <= old_size) {
        old_header->size = new_size;
        return pointer;
    }

    new_ptr = FXMEM_DefaultAllocWithCaller(new_size, flags, FXMEM_CALLER(), NULL, 0);
    if (!new_ptr) {
        FXMEM_RecordOOM(__func__, new_size, "realloc_grow_fail", FXMEM_CALLER(), NULL, 0);
        return NULL;
    }
    FXSYS_memcpy(new_ptr, pointer, old_size);
    FXMEM_DefaultFree(pointer, flags);
    return new_ptr;
}

void* FXMEM_DefaultReallocDebug(void* pointer, size_t new_size, int flags, const char *file, int line)
{
    FXMEM_BlockHeader *old_header;
    size_t old_size;
    void *new_ptr;

    if (!pointer)
        return FXMEM_DefaultAllocWithCaller(new_size, flags, FXMEM_CALLER(), file, line);
    if (new_size == 0) {
        FXMEM_DefaultFree(pointer, flags);
        return NULL;
    }

    old_header = ((FXMEM_BlockHeader *)pointer) - 1;
    if (old_header->magic != FXMEM_BLOCK_MAGIC) {
        FXMEM_RecordOOM("FXMEM_DefaultRealloc", new_size, "realloc_badptr", FXMEM_CALLER(), file, line);
        return NULL;
    }
    old_size = old_header->size;
    if (new_size <= old_size) {
        old_header->size = new_size;
        return pointer;
    }

    new_ptr = FXMEM_DefaultAllocWithCaller(new_size, flags, FXMEM_CALLER(), file, line);
    if (!new_ptr) {
        FXMEM_RecordOOM("FXMEM_DefaultRealloc", new_size, "realloc_grow_fail", FXMEM_CALLER(), file, line);
        return NULL;
    }
    FXSYS_memcpy(new_ptr, pointer, old_size);
    FXMEM_DefaultFree(pointer, flags);
    return new_ptr;
}

void FXMEM_DefaultFree(void* pointer, int flags)
{
    FXMEM_BlockHeader *header;

    if (!pointer)
        return;

    header = ((FXMEM_BlockHeader *)pointer) - 1;
    if (header->magic != FXMEM_BLOCK_MAGIC) {
#if defined(SOLUTION)
        rt_kprintf("pdfium_free_badptr: ptr=0x%08x header=0x%08x magic=0x%08x caller=0x%08x\n",
                   (unsigned int)(uintptr_t)pointer,
                   (unsigned int)(uintptr_t)header,
                   (unsigned int)header->magic,
                   (unsigned int)(uintptr_t)FXMEM_CALLER());
#endif
        return;
    }
    #if defined(SOLUTION)
    FXMEM_UnlinkBlock(header);
    #endif
    header->magic = FXMEM_BLOCK_FREED;
#if defined(SOLUTION)
    app_anim_free(header);
#else
    free(header);
#endif
}

void FXMEM_ClearOOM(void)
{
    g_fxmem_oom = 0;
    g_fxmem_oom_count = 0;
    g_fxmem_oom_maxsize = 0;
}

int FXMEM_HasOOM(void)
{
    return g_fxmem_oom;
}

void FXMEM_GetOOMInfo(int *count, size_t *maxsize)
{
    if (count)
        *count = g_fxmem_oom_count;
    if (maxsize)
        *maxsize = g_fxmem_oom_maxsize;
}

void FXMEM_DumpLeaks(void)
{
#if defined(SOLUTION)
    FXMEM_BlockHeader *p;
    int index = 0;

    if (!g_fxmem_blocks) {
        rt_kprintf("pdfium_mem: no leaks outstanding=0 bytes=0\n");
        return;
    }

    rt_kprintf("pdfium_mem: leaks outstanding=%d bytes=%u\n",
               g_fxmem_block_count, (unsigned int)g_fxmem_block_bytes);
    for (p = g_fxmem_blocks; p; p = p->next) {
        rt_kprintf("pdfium_mem_leak[%d]: ptr=0x%08x size=%u caller=0x%08x loc=%s:%d\n",
                   index,
                   (unsigned int)(uintptr_t)(p + 1),
                   (unsigned int)p->size,
                   (unsigned int)(uintptr_t)p->caller,
                   p->file ? p->file : "?",
                   p->line);
        index++;
    }
#endif
}

#ifdef __cplusplus
}

const FX_Nothrow_t FX_Nothrow = FX_Nothrow_t();
#endif

CFX_GrowOnlyPool::CFX_GrowOnlyPool(size_t trunk_size)
{
    m_TrunkSize = trunk_size;
    m_pFirstTrunk = NULL;
}

CFX_GrowOnlyPool::~CFX_GrowOnlyPool()
{
    FreeAll();
}

struct _FX_GrowOnlyTrunk {
    size_t              m_Size;
    size_t              m_Allocated;
    _FX_GrowOnlyTrunk*  m_pNext;
};

void CFX_GrowOnlyPool::FreeAll()
{
    _FX_GrowOnlyTrunk* pTrunk = (_FX_GrowOnlyTrunk*)m_pFirstTrunk;
    while (pTrunk) {
        _FX_GrowOnlyTrunk* pNext = pTrunk->m_pNext;
        FX_Free(pTrunk);
        pTrunk = pNext;
    }
    m_pFirstTrunk = NULL;
}

void* CFX_GrowOnlyPool::Alloc(size_t size)
{
    size = (size + 3) / 4 * 4;
    _FX_GrowOnlyTrunk* pTrunk = (_FX_GrowOnlyTrunk*)m_pFirstTrunk;
    while (pTrunk) {
        if (pTrunk->m_Size - pTrunk->m_Allocated >= size) {
            void* p = (FX_LPBYTE)(pTrunk + 1) + pTrunk->m_Allocated;
            pTrunk->m_Allocated += size;
            return p;
        }
        pTrunk = pTrunk->m_pNext;
    }
    size_t alloc_size = size > m_TrunkSize ? size : m_TrunkSize;
    pTrunk = (_FX_GrowOnlyTrunk*)FX_Alloc(FX_BYTE, sizeof(_FX_GrowOnlyTrunk) + alloc_size);
    if (!pTrunk) {
        FXMEM_RecordOOM(__func__, sizeof(_FX_GrowOnlyTrunk) + alloc_size, "grow_pool_alloc_fail", FXMEM_CALLER(), __FILE__, __LINE__);
        return NULL;
    }
    pTrunk->m_Size = alloc_size;
    pTrunk->m_Allocated = size;
    pTrunk->m_pNext = (_FX_GrowOnlyTrunk*)m_pFirstTrunk;
    m_pFirstTrunk = pTrunk;
    return pTrunk + 1;
}
