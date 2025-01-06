#ifndef _MEDIA_QUEUE_H_
#define _MEDIA_QUEUE_H_

#include <rtthread.h>
#include <stdlib.h>
#include <stdint.h>
typedef struct
{
    rt_list_t node;
    uint32_t seq_num;
    uint32_t data_type;
    uint32_t data_len;
    uint32_t data_offset;
    uint8_t  data[0];
} media_packet_t;

typedef void (*media_free)(void *p);

typedef struct
{
    rt_list_t root;
    rt_mutex_t mutex;
    rt_event_t event;
    media_free free;
    uint32_t  debug_last_v;
    uint32_t  debug_last_a;
    uint32_t  debug_v;
    uint32_t  debug_a;
    uint64_t  bytes_download;
    uint64_t  bytes_used;
} media_queue_t;

media_queue_t *media_queue_open(media_free func);
uint64_t media_bytes_in_queue(media_queue_t *q);
void media_queue_close(media_queue_t *q);
void media_queue_set(media_queue_t *q);
void media_queue_wait(media_queue_t *q);
void media_queue_add_tail(media_queue_t *q, media_packet_t *p);
void media_queue_clean(media_queue_t *q);

static inline void _lock(media_queue_t *q)
{
    if (q && q->mutex)
    {
        rt_mutex_take(q->mutex, RT_WAITING_FOREVER);
    }
}
static inline void _unlock(media_queue_t *q)
{
    if (q && q->mutex)
    {
        rt_mutex_release(q->mutex);
    }
}

#define media_queue_add_readed(q, used)   q->bytes_used += used


#endif
