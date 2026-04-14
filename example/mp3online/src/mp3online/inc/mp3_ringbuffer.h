
#ifndef MP3_RING_BUFFER_H
#define MP3_RING_BUFFER_H

#include <stdint.h>
#include <rtthread.h>

#define MP3_RING_BUFFER_SIZE (1024*30) // 30KB ring buffer for mp3 stream play

void mp3_ring_buffer_init(void);
void mp3_ring_buffer_reset(void);
rt_size_t mp3_ring_buffer_put(uint8_t *data, int len);
rt_size_t mp3_ring_buffer_get(uint8_t *data, int len);
rt_size_t mp3_ring_buffer_data_len(void);
rt_size_t mp3_ring_buffer_space_len(void);
//struct rt_ringbuffer *mp3_ring_buffer_get_handle(void);

#endif
