
#include <rtthread.h>
#ifdef PKG_USING_CJSON
#include "cJSON.h"
#endif
#include "mp3_mem.h"

#ifdef PKG_USING_CJSON
cJSON_Hooks mp3_mem_hook =
{
    .malloc_fn = mp3_mem_malloc,
    .free_fn = mp3_mem_free,
};
#endif

L2_CACHE_RET_BSS_SECT_BEGIN(mp3_psram_ret_cache)
    ALIGN(4) static uint8_t mp3_psram_heap[0x200000] SECTION(STRINGIFY(.bss.l2_cache_ret_bss_mp3_psram_ret_cache));
L2_CACHE_RET_BSS_SECT_END

static struct rt_memheap mp3_memheap;
static struct rt_memheap *p_mp3_memheap;

static int mp3_memheap_init(void)
{
    rt_memheap_init(&mp3_memheap, "mp3_memheap", (void *)mp3_psram_heap, sizeof(mp3_psram_heap));
    p_mp3_memheap = &mp3_memheap;
#ifdef PKG_USING_CJSON
    cJSON_InitHooks(&mp3_mem_hook);
#endif
    return 0;
}
INIT_PREV_EXPORT(mp3_memheap_init);

void * mp3_mem_malloc(size_t size)
{
    void * ptr = NULL;
    ptr = rt_memheap_alloc(p_mp3_memheap, size);
    return ptr;
}

void mp3_mem_free(void * ptr)
{
    rt_memheap_free(ptr);
}

void * mp3_mem_calloc(uint32_t count, uint32_t size)
{
    void * ptr = NULL;
    ptr = rt_memheap_calloc(p_mp3_memheap, count, size);
    return ptr;
}
