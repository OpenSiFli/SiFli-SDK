#include "media_queue.h"

static inline void lock(rt_mutex_t mutex)
{
    rt_mutex_take(mutex, RT_WAITING_FOREVER);
}

static inline void unlock(rt_mutex_t mutex)
{
    rt_mutex_release(mutex);
}

media_queue_t *media_queue_open(media_free func)
{
    media_queue_t *q = (media_queue_t *)calloc(sizeof(media_queue_t), 1);
    RT_ASSERT(q && func);

    q->bytes_download = 0;
    q->bytes_used = 0;
    rt_list_init(&q->root);
    q->mutex = rt_mutex_create("smedia", RT_IPC_FLAG_FIFO);
    RT_ASSERT(q->mutex);
    q->event = rt_event_create("qmedia", RT_IPC_FLAG_FIFO);
    RT_ASSERT(q->event);
    q->free = func;

    return q;
}

void media_queue_close(media_queue_t *q)
{
    media_packet_t *p;
    if (!q)
    {
        return;
    }

    lock(q->mutex);

    while (!rt_list_isempty(&q->root))
    {
        p = rt_list_first_entry(&q->root, media_packet_t, node);
        rt_list_remove(&p->node);
        q->free(p);
    }
    rt_event_delete(q->event);
    unlock(q->mutex);
    rt_mutex_delete(q->mutex);
    free(q);
}

uint64_t media_bytes_in_queue(media_queue_t *q)
{
    RT_ASSERT(q);
    return (q->bytes_download - q->bytes_used);
}

void media_queue_add_tail(media_queue_t *q, media_packet_t *p)
{
    RT_ASSERT(q && p);

    lock(q->mutex);

    rt_list_insert_before(&q->root, &p->node);

    q->bytes_download += p->data_len;
    if (p->data_type == 'v')
    {
        q->debug_v++;
    }
    else
    {
        q->debug_a++;
    }
    unlock(q->mutex);
    media_queue_set(q);
}


void media_queue_clean(media_queue_t *q)
{
    media_packet_t *p;
    RT_ASSERT(q);
    lock(q->mutex);

    while (!rt_list_isempty(&q->root))
    {
        p = rt_list_first_entry(&q->root, media_packet_t, node);
        rt_list_remove(&p->node);
        q->free(p);
    }

    q->bytes_download = 0;
    q->bytes_used = 0;

    unlock(q->mutex);
}

void media_queue_set(media_queue_t *q)
{
    RT_ASSERT(q);
    rt_event_send(q->event, 1);
}

void media_queue_wait(media_queue_t *q)
{
    RT_ASSERT(q);
    rt_event_recv(q->event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
}


