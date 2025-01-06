#include <rtthread.h>
#include <rtdevice.h>
#include "bf0_hal.h"
#include "media_dec.h"
#include "mem_section.h"

typedef struct _ffmpeg_mem_header
{
    uint32_t magic;
    uint32_t offset;//Offset between 'ffmpeg_alloc' returned value and 'app_anim_mem_(re)alloc' returned value
    uint32_t size;
} ffmpeg_mem_header;

//Assumed that > 64K memory area used by EPIC
#define ALIGN64_SIZE_THRESHOLD 65536
#define FFMPEG_MEM_HEADER sizeof(ffmpeg_mem_header)
#define FFMPEG_MEM_MAGIC  0xFF3E63E3
#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

L2_RET_BSS_SECT_BEGIN(ffmpeg_heap2)
ALIGN(4) uint8_t ffmpeg_heap[MEDIA_CACHE_SIZE];
L2_RET_BSS_SECT_END


struct rt_memheap ffmpeg_memheap;

void ffmpeg_heap_init()
{
    rt_memheap_init(&ffmpeg_memheap, "ffmpeg_memheap", (void *)ffmpeg_heap, MEDIA_CACHE_SIZE);
}

void *ffmpeg_alloc(size_t nbytes)
{
    uint8_t *p;
    ffmpeg_mem_header *header_p;

    if (nbytes > ALIGN64_SIZE_THRESHOLD)
    {
        size_t header_size = 63 + FFMPEG_MEM_HEADER;
        p = rt_memheap_alloc(&ffmpeg_memheap, nbytes + header_size);
        RT_ASSERT(p);
        if (!p) return NULL;

        header_p = (ffmpeg_mem_header *)(RT_ALIGN_DOWN((uint32_t)(p + header_size), 64) - FFMPEG_MEM_HEADER);

        RT_ASSERT(((uint32_t)header_p) >= ((uint32_t)p));
    }
    else
    {
        p = rt_memheap_alloc(&ffmpeg_memheap, nbytes + FFMPEG_MEM_HEADER);
        RT_ASSERT(p);
        if (!p) return NULL;
        header_p = (ffmpeg_mem_header *) p;
    }


    header_p->magic = FFMPEG_MEM_MAGIC;
    header_p->offset = ((uint32_t)header_p) + sizeof(ffmpeg_mem_header) - ((uint32_t)p);
    header_p->size = nbytes;

    return (uint8_t *)(header_p + 1);
}

void ffmpeg_free(void *p)
{
    if (!p) return;

    ffmpeg_mem_header *header_p = ((ffmpeg_mem_header *)p) - 1;

    RT_ASSERT(FFMPEG_MEM_MAGIC == header_p->magic);
    rt_memheap_free(((uint8_t *)p) - header_p->offset);
}

void *ffmpeg_realloc(void *p, size_t new_size)
{
    if (!p) return ffmpeg_alloc(new_size);
    if (!new_size)
    {
        ffmpeg_free(p);
        return NULL;
    }
    uint8_t *new_p = ffmpeg_alloc(new_size);

    if (new_p)
    {
        ffmpeg_mem_header *header_p = ((ffmpeg_mem_header *)p) - 1;
        RT_ASSERT(FFMPEG_MEM_MAGIC == header_p->magic);
        memcpy(new_p, p, MIN(new_size, header_p->size));
        ffmpeg_free(p);
    }

    return new_p;
}



