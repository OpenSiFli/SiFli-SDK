
#ifndef __MP3_MEM_H__
#define __MP3_MEM_H__

#include "mem_section.h"

void * mp3_mem_malloc(size_t size);
void mp3_mem_free(void * ptr);
void * mp3_mem_calloc(uint32_t count, uint32_t size);

#endif
