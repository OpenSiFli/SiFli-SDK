#include "sm_internal.h"

void sm_packet_free(void *p)
{
    ffmpeg_free(p);
}
void *sm_packet_malloc(size_t size)
{
    return ffmpeg_alloc(size);
}