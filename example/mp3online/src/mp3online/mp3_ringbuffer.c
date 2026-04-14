
#include <rtdevice.h>
#include "mp3_ringbuffer.h"

struct rt_ringbuffer g_mp3_ring_buffer_handle;
/* ringbuff for stream download and play */
uint8_t g_mp3_ring_buffer[MP3_RING_BUFFER_SIZE] = {};

void mp3_ring_buffer_init(void)
{
    rt_ringbuffer_init(&g_mp3_ring_buffer_handle, g_mp3_ring_buffer, MP3_RING_BUFFER_SIZE);
}

void mp3_ring_buffer_reset(void)
{
    rt_ringbuffer_reset(&g_mp3_ring_buffer_handle);
}

rt_size_t mp3_ring_buffer_put(uint8_t *data, int len)
{
    return rt_ringbuffer_put_force(&g_mp3_ring_buffer_handle, data, len);
}

rt_size_t mp3_ring_buffer_get(uint8_t *data, int len)
{
    return rt_ringbuffer_get(&g_mp3_ring_buffer_handle, data, len);
}

rt_size_t mp3_ring_buffer_data_len(void)
{
    return rt_ringbuffer_data_len(&g_mp3_ring_buffer_handle);
}

rt_size_t mp3_ring_buffer_space_len(void)
{
    return rt_ringbuffer_space_len(&g_mp3_ring_buffer_handle);
}

#if 0
struct rt_ringbuffer *mp3_ring_buffer_get_handle(void)
{
    return &g_mp3_ring_buffer_handle;
}
#endif
