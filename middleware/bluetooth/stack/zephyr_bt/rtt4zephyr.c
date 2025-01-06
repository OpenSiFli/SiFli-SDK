#include "rtt4zephyr.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/ias.h>
#include <../host/conn_internal.h>
#include <../host/l2cap_internal.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/bluetooth/mesh.h>
#include <mesh/subnet.h>
#include <mesh/mesh.h>
#include "bf0_sibles.h"


k_timepoint_t sys_timepoint_calc(k_timeout_t timeout)
{
    k_timepoint_t r;

    r.tick = rt_tick_get() + timeout.ticks;
    return r;
}

k_timeout_t sys_timepoint_timeout(k_timepoint_t timepoint)
{
    k_timeout_t r;

    r.ticks = timepoint.tick - rt_tick_get();
    return r;
}

bool device_is_ready(const struct device *dev)
{
    return 1;
}


/**********************************************************************************/

#if 0
static RNG_HandleTypeDef rng_hdl;
int bt_rand(void *buf, size_t len)
{
    uint8_t *buf2 = (uint8_t *)buf;

    rng_hdl.Instance = hwp_trng;
    HAL_RNG_Init(&rng_hdl);
    while (len)
    {
        uint32_t val;
        HAL_RNG_Generate(&rng_hdl, &val, 0);
        if (len > sizeof(uint32_t))
        {
            memcpy(buf2, &val, sizeof(uint32_t));
            buf2 += sizeof(uint32_t);
            len -= sizeof(uint32_t);
        }
        else
        {
            memcpy(buf2, &val, len);
            break;
        }
    }
    return 0;
}

int bt_encrypt_le(const uint8_t key[16], const uint8_t plaintext[16],
                  uint8_t enc_data[16])
{
    HAL_AES_init((uint32_t *)key, 16, NULL, AES_MODE_ECB);
    HAL_AES_run(1, (uint8_t *)plaintext, (uint8_t *)enc_data, 16);
    return 0;
}
#endif

rt_tick_t k_work_delayable_remaining_get(const struct rt_delayed_work *dwork)
{
    rt_tick_t t = 0;

    if (dwork->timer.timeout_tick > rt_tick_get())
        t = dwork->timer.timeout_tick - rt_tick_get();

    return t;
}

int k_work_reschedule2(struct rt_delayed_work *dwork, rt_tick_t delay)
{
    k_timeout_t t;

    if (dwork->work.list.next == NULL && dwork->work.list.prev == NULL)
        rt_work_init(&(dwork->work), dwork->work.work_func, dwork->work.work_data);

    t.ticks = delay;
    k_work_cancel_delayable_sync(dwork, NULL);
    return k_work_schedule(dwork, t);
}

int k_mutex_init(struct rt_mutex *mutex)
{
    static int idx = 0;
    int r = RT_EOK;
    char name[8];

    if (mutex->parent.parent.type == 0)
    {
        rt_sprintf(name, "k_mux%d", idx++);
        r = rt_mutex_init(mutex, name, RT_IPC_FLAG_FIFO);
    }
    else
        mutex->value = 1;
    return r;
}


int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
    static int idx = 0;
    char name[8];

    rt_sprintf(name, "k_sem%d", idx++);
    rt_sem_init(sem, name, initial_count, RT_IPC_FLAG_FIFO);
    return 0;
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
    if (sem->parent.parent.type == 0)
        k_sem_init(sem, sem->value, 0);
    return rt_sem_take(sem, timeout.ticks);
}

void k_sem_give(struct k_sem *sem)
{
    if (sem->parent.parent.type == 0)
        k_sem_init(sem, sem->value, 0);
    rt_sem_release(sem);
}


/*********************work*********************************************/

/*Fix me: implement need review.*/
bool k_work_flush(struct k_work *work,
                  struct k_work_sync *sync)
{
    bool need_flush = (work->flags & RT_WORK_STATE_PENDING);

    /* If necessary wait until the flusher item completes */
    while (need_flush)
    {
        rt_thread_delay(20);
        need_flush = (work->flags & RT_WORK_STATE_PENDING);
    }
    return need_flush;
}

/***********************slab memory*********************************/
int k_mem_slab_alloc(struct rt_mempool *slab, void **mem,
                     k_timeout_t timeout)
{

    if (slab->size == 0)
    {
        char *name = (char *)slab->start_address;
        uint32_t size = (slab->block_size + sizeof(rt_uint8_t *)) * slab->block_total_count;
        uint8_t *m = rt_malloc(size);
        rt_mp_init(slab, name, m, size, slab->block_size);
    }
    *mem = rt_mp_alloc(slab, timeout.ticks);
    if (*mem == NULL)
        return -ENOMEM;
    else
        return RT_EOK;
}

__USED struct net_buf_pool _net_buf_pool_list_start SPACE1("._net_buf_pool.static.00_start");
__USED struct net_buf_pool _net_buf_pool_list_end SPACE1("._net_buf_pool.static.zz_end");

#ifdef BT_CONN
    __USED const struct bt_conn_cb _bt_conn_cb_list_start SPACE1("._bt_conn_cb.static.00_start");
    __USED const struct bt_conn_cb _bt_conn_cb_list_end SPACE1("._bt_conn_cb.static.zz_end");
    __USED const struct bt_l2cap_fixed_chan _bt_l2cap_fixed_chan_list_start SPACE1("._bt_l2cap_fixed_chan.static.00_start");
    __USED const struct bt_l2cap_fixed_chan _bt_l2cap_fixed_chan_list_end  SPACE1("._bt_l2cap_fixed_chan.static.zz_end");
    __USED const struct bt_gatt_service_static _bt_gatt_service_static_list_start  SPACE1("._bt_gatt_service_static.static.00_start");
    __USED const struct bt_gatt_service_static _bt_gatt_service_static_list_end  SPACE1("._bt_gatt_service_static.static.zz_end");
#endif

#ifdef BT_IAS
    __USED const struct bt_ias_cb _bt_ias_cb_list_start  SPACE1("._bt_ias_cb.static.00_start");
    __USED const struct bt_ias_cb _bt_ias_cb_list_end  SPACE1("._bt_ias_cb.static.zz_end");
#endif

#ifdef BT_MESH
    __USED const struct bt_mesh_subnet_cb _bt_mesh_subnet_cb_list_start SPACE1("._bt_conn_cb.static.00_start");
    __USED const struct bt_mesh_subnet_cb  _bt_mesh_subnet_cb_list_end SPACE1("._bt_conn_cb.static.zz_end");
    __USED const struct bt_mesh_app_key_cb  _bt_mesh_app_key_cb_list_start SPACE1("._bt_mesh_app_key_cb.static.00_start");
    __USED const struct bt_mesh_app_key_cb  _bt_mesh_app_key_cb_list_end SPACE1("._bt_mesh_app_key_cb.static.zz_end");
    __USED const struct bt_mesh_hb_cb    _bt_mesh_hb_cb_list_start SPACE1("._bt_mesh_hb_cb.static.00_start");
    __USED const struct bt_mesh_hb_cb    _bt_mesh_hb_cb_list_end SPACE1("._bt_mesh_hb_cb.static.zz_end");
    __USED const struct bt_mesh_friend_cb    _bt_mesh_friend_cb_list_start SPACE1("._bt_mesh_friend_cb.static.00_start");
    __USED const struct bt_mesh_friend_cb    _bt_mesh_friend_cb_list_end SPACE1("._bt_mesh_friend_cb.static.zz_end");
    __USED const struct bt_mesh_proxy_cb    _bt_mesh_proxy_cb_list_start SPACE1("._bt_mesh_proxy_cb.static.00_start");
    __USED const struct bt_mesh_proxy_cb    _bt_mesh_proxy_cb_list_end SPACE1("._bt_mesh_proxy_cb.static.zz_end");
#endif

/********************FIFO ***********************************************/
void k_fifo_put(struct k_fifo *fifo, void *data)
{
    struct rt_slist_node *node = (struct rt_slist_node *)data;

    rt_enter_critical();
    if (node && fifo)
    {
        node->next = NULL;
        if (fifo->tail == NULL)
            fifo->tail = node;
        else
        {
            fifo->tail->next = node;
            fifo->tail = node;
        }
        if (fifo->head == NULL)
            fifo->head = node;
    }
    rt_exit_critical();
}

void *k_fifo_get(struct k_fifo *fifo, k_timeout_t timeout)
{
    struct rt_slist_node *r = NULL;

    rt_enter_critical();
    if (fifo && fifo->head)
    {
        r = fifo->head;
        fifo->head = r->next;
        if (r->next == NULL && fifo->tail)
            fifo->tail = NULL;
    }
    rt_exit_critical();
    return (void *)r;
}


/********************LIFO ***********************************************/
void k_lifo_put(struct k_lifo *lifo, void *data)
{
    struct rt_slist_node *node = (struct rt_slist_node *)data;

    if (node && lifo)
    {
        if (lifo->head == NULL)
        {
            node->next = NULL;
            lifo->head = node;
        }
        else
        {
            node->next = lifo->head;
            lifo->head = node;
        }
    }
}


void *k_lifo_get(struct k_lifo *lifo, k_timeout_t timeout)
{
    struct rt_slist_node *r = NULL;

    if (lifo && lifo->head)
    {
        r = lifo->head;
        lifo->head = r->next;
    }
    return (void *)r;
}

int k_work_submit(struct k_work *work)
{
    if (work->list.next == NULL && work->list.prev == NULL)
        rt_work_init(work, work->work_func, work->work_data);
    return rt_work_submit(work, 0);
}

/***********************Not implemented*********************************/

int bt_hci_ecc_send(struct net_buf *buf)
{
    return 0;
}

void bt_hci_ecc_supported_commands(uint8_t *supported_commands)
{
    return;
}



void k_work_queue_start(struct k_work_q *queue,
                        void *stack,
                        rt_size_t stack_size,
                        int prio,
                        const struct k_work_queue_config *cfg)
{
    if (cfg)
        rt_workqueue_start(queue, (cfg)->name, stack, stack_size, prio);
    else
        rt_workqueue_start(queue, "unknown", stack, stack_size, prio);
}

k_tid_t k_thread_create(struct k_thread *new_thread,
                        uint32_t *stack,
                        rt_size_t stack_size,
                        k_thread_entry_t entry,
                        void *p1, void *p2, void *p3,
                        int prio, uint32_t options, k_timeout_t delay)

{
    k_tid_t r = NULL;
    if (RT_EOK == rt_thread_init(new_thread, "thread", (void(*)(void *))entry, p1, stack, stack_size, prio, 20))
        r = new_thread;
    return r;
}

int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
    if (work->list.next == NULL && work->list.prev == NULL)
        rt_work_init(work, work->work_func, work->work_data);
    return rt_workqueue_submit_work(queue, work, 0);
}


int k_poll_signal_raise(struct k_poll_signal *sig, int result)
{
    if (sig->waiting_list.prev == NULL && sig->waiting_list.next == NULL)
        rt_wqueue_init(sig);
    rt_wqueue_wakeup(sig, &result);
    return 0;
}

__syscall int k_poll(struct k_poll_event *events, int num_events,
                     k_timeout_t timeout)
{
    // TODO

    return 0;
}

void k_fifo_cancel_wait(struct k_fifo *queue)
{
    // TODO
}

int k_work_cancel_delayable(struct k_work_delayable *dwork)
{
    struct rt_work *work = (struct rt_work *)dwork;

    if (work->flags)
        return rt_work_cancel(work);
    else
        return 0;

}

void k_thread_start(rt_thread_t thread)
{
    if (thread->type == 0)
    {
        rt_thread_init(thread, thread->name, thread->entry,
                       thread->parameter, thread->stack_addr, thread->stack_size, thread->init_priority, 20);
    }
    if ((thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_INIT)
        rt_thread_startup(thread);
}


/**
 * determine the number of characters before the first separator
 *
 * @param[in] name in string format
 * @param[out] next pointer to remaining of name (excluding separator)
 *
 * @return index of the first separator, in case no separator was found this
 * is the size of name
 *
 */
int settings_name_next(const char *name, const char **next)
{
    int rc = 0;

    if (next)
    {
        *next = NULL;
    }

    if (!name)
    {
        return 0;
    }

    /* name might come from flash directly, in flash the name would end
     * with '=' or '\0' depending how storage is done. Flash reading is
     * limited to what can be read
     */
    while ((*name != '\0') && (*name != SETTINGS_NAME_END) &&
            (*name != SETTINGS_NAME_SEPARATOR))
    {
        rc++;
        name++;
    }

    if (*name == SETTINGS_NAME_SEPARATOR)
    {
        if (next)
        {
            *next = name + 1;
        }
        return rc;
    }

    return rc;
}

/*
 * Append a single value to persisted config. Don't store duplicate value.
 */
int settings_save_one(const char *name, const void *value, size_t val_len)
{
    // TODO
    return 0;
}

/**
 * Delete a single serialized in persisted storage.
 *
 * Deleting an existing key-value pair in the settings mean
 * to set its value to NULL.
 *
 * @param name Name/key of the settings item.
 *
 * @return 0 on success, non-zero on failure.
 */
int settings_delete(const char *name)
{
    // TODO
    return 0;
}


void sibles_set_trc_cfg(sibles_trc_cfg_t cfg_mode, uint32_t mask_ext)
{
    // Dummy for compile
}



#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

