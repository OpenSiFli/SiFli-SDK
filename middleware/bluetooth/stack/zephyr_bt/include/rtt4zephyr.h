#ifndef RTT4ZEPHYR_H
#define RTT4ZEPHYR_H

#include "rtthread.h"
#include "rthw.h"
#include "zbt_rtt.h"
#include <ipc/dataqueue.h>
#include <ipc/workqueue.h>
#include <ipc/waitqueue.h>

#include "zephyr/autoconf.h"
#include "bf0_hal_def.h"


extern struct bt_dev bt_dev;
struct arch_esf
{
    uint32_t dummy;
};
#define device  rt_device

bool device_is_ready(const struct device *dev);
#define K_PRIO_COOP(x) x


#ifndef POPCOUNT
    #define POPCOUNT(x) __builtin_popcount(x)
#endif

#define LOG_MODULE_DECLARE(a,b)

//------------------------After this is implmemented.-------------------------------

#ifndef BIT
    #define BIT(n)  (1UL << (n))
#endif

enum
{
    /**
     * @cond INTERNAL_HIDDEN
     */

    /* The atomic API is used for all work and queue flags fields to
     * enforce sequential consistency in SMP environments.
     */

    /* Bits that represent the work item states.  At least nine of the
     * combinations are distinct valid stable states.
     */
    K_WORK_RUNNING_BIT = 0,
    K_WORK_CANCELING_BIT = 1,
    K_WORK_QUEUED_BIT = 2,
    K_WORK_DELAYED_BIT = 3,
    K_WORK_FLUSHING_BIT = 4,

    K_WORK_MASK = BIT(K_WORK_DELAYED_BIT) | BIT(K_WORK_QUEUED_BIT)
                  | BIT(K_WORK_RUNNING_BIT) | BIT(K_WORK_CANCELING_BIT) | BIT(K_WORK_FLUSHING_BIT),

    /* Static work flags */
    K_WORK_DELAYABLE_BIT = 8,
    K_WORK_DELAYABLE = BIT(K_WORK_DELAYABLE_BIT),

    /* Dynamic work queue flags */
    K_WORK_QUEUE_STARTED_BIT = 0,
    K_WORK_QUEUE_STARTED = BIT(K_WORK_QUEUE_STARTED_BIT),
    K_WORK_QUEUE_BUSY_BIT = 1,
    K_WORK_QUEUE_BUSY = BIT(K_WORK_QUEUE_BUSY_BIT),
    K_WORK_QUEUE_DRAIN_BIT = 2,
    K_WORK_QUEUE_DRAIN = BIT(K_WORK_QUEUE_DRAIN_BIT),
    K_WORK_QUEUE_PLUGGED_BIT = 3,
    K_WORK_QUEUE_PLUGGED = BIT(K_WORK_QUEUE_PLUGGED_BIT),

    /* Static work queue flags */
    K_WORK_QUEUE_NO_YIELD_BIT = 8,
    K_WORK_QUEUE_NO_YIELD = BIT(K_WORK_QUEUE_NO_YIELD_BIT),

    /**
     * INTERNAL_HIDDEN @endcond
     */
    /* Transient work flags */

    /** @brief Flag indicating a work item that is running under a work
     * queue thread.
     *
     * Accessed via k_work_busy_get().  May co-occur with other flags.
     */
    K_WORK_RUNNING = BIT(K_WORK_RUNNING_BIT),

    /** @brief Flag indicating a work item that is being canceled.
     *
     * Accessed via k_work_busy_get().  May co-occur with other flags.
     */
    K_WORK_CANCELING = BIT(K_WORK_CANCELING_BIT),

    /** @brief Flag indicating a work item that has been submitted to a
     * queue but has not started running.
     *
     * Accessed via k_work_busy_get().  May co-occur with other flags.
     */
    K_WORK_QUEUED = BIT(K_WORK_QUEUED_BIT),

    /** @brief Flag indicating a delayed work item that is scheduled for
     * submission to a queue.
     *
     * Accessed via k_work_busy_get().  May co-occur with other flags.
     */
    K_WORK_DELAYED = BIT(K_WORK_DELAYED_BIT),

    /** @brief Flag indicating a synced work item that is being flushed.
     *
     * Accessed via k_work_busy_get().  May co-occur with other flags.
     */
    K_WORK_FLUSHING = BIT(K_WORK_FLUSHING_BIT),
};

#undef IS_ALIGNED

#define BUILD_ASSERT(...)
#define ALWAYS_INLINE __INLINE
#define __noasan
#define __fallthrough
#define __noinit
#define __maybe_unused
#define __aligned(x) __ALIGNED(x)
#define __syscall
#define __subsystem
#define unlikely(x) x

#define k_oops()
#define CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 48000000
#define Z_DECL_ALIGN(type) type

#define irq_lock() rt_hw_interrupt_disable()
#define irq_unlock(key) rt_hw_interrupt_enable(key)

#define INT_MAX (0x7FFFFFFF)

/**
 * @brief Register an initialization function.
 *
 * The function will be called during system initialization according to the
 * given level and priority.
 *
 * @param init_fn Initialization function.
 * @param level Initialization level. Allowed tokens: `EARLY`, `PRE_KERNEL_1`,
 * `PRE_KERNEL_2`, `POST_KERNEL`, `APPLICATION` and `SMP` if
 * @kconfig{CONFIG_SMP} is enabled.
 * @param prio Initialization priority within @p _level. Note that it must be a
 * decimal integer literal without leading zeroes or sign (e.g. `32`), or an
 * equivalent symbolic name (e.g. `#define MY_INIT_PRIO 32`); symbolic
 * expressions are **not** permitted (e.g.
 * `CONFIG_KERNEL_INIT_PRIORITY_DEFAULT + 5`).
 */
#define SYS_INIT(init_fn, level, prio) INIT_ENV_EXPORT(init_fn)

#define k_sys_work_q_get rt_workqueue_sysq

#define CONFIG_SYS_CLOCK_EXISTS 1
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC RT_TICK_PER_SECOND

/**
 * @brief Generate null timeout delay.
 *
 * This macro generates a timeout delay that instructs a kernel API
 * not to wait if the requested operation cannot be performed immediately.
 *
 * @return Timeout delay value.
 */
#define K_NO_WAIT Z_TIMEOUT_NO_WAIT

/**
 * @brief Generate timeout delay from microseconds.
 *
 * This macro generates a timeout delay that instructs a kernel API
 * to wait up to @a t microseconds to perform the requested operation.
 * Note that timer precision is limited to the tick rate, not the
 * requested value.
 *
 * @param t Duration in microseconds.
 *
 * @return Timeout delay value.
 */
#define K_USEC(t)     Z_TIMEOUT_US(t)

/**
 * @brief Generate timeout delay from milliseconds.
 *
 * This macro generates a timeout delay that instructs a kernel API
 * to wait up to @a ms milliseconds to perform the requested operation.
 *
 * @param ms Duration in milliseconds.
 *
 * @return Timeout delay value.
 */
#define K_MSEC(ms)     Z_TIMEOUT_MS(ms)

/**
 * @brief Generate timeout delay from seconds.
 *
 * This macro generates a timeout delay that instructs a kernel API
 * to wait up to @a s seconds to perform the requested operation.
 *
 * @param s Duration in seconds.
 *
 * @return Timeout delay value.
 */
#define K_SECONDS(s)   K_MSEC((s) * MSEC_PER_SEC)

/**
 * @brief Generate infinite timeout delay.
 *
 * This macro generates a timeout delay that instructs a kernel API
 * to wait as long as necessary to perform the requested operation.
 *
 * @return Timeout delay value.
 */
#define K_FOREVER Z_FOREVER

#define __deprecated
#define __must_check

#define ARG_UNUSED(arg)

#define inline __INLINE
#define printk rt_kprintf
#define snprintk rt_snprintf
#define __printf_like(...)
#include "zephyr/sys/__assert.h"
#include "zephyr/sys/atomic.h"
#include "zephyr/sys_clock.h"
#define __used __USED

/* Unaligned access */
#define UNALIGNED_GET(g)                        \
__extension__ ({                            \
    struct  __attribute__((__packed__)) {               \
        __typeof__(*(g)) __v;                   \
    } *__g = (__typeof__(__g)) (g);                 \
    __g->__v;                           \
})

/**
 *
 * @brief find most significant bit set in a 32-bit word
 *
 * This routine finds the first bit set starting from the most significant bit
 * in the argument passed in and returns the index of that bit.  Bits are
 * numbered starting at 1 from the least significant bit.  A return value of
 * zero indicates that the value passed is zero.
 *
 * @return most significant bit set, 0 if @a op is 0
 */

static ALWAYS_INLINE unsigned int find_msb_set(uint32_t op)
{
    if (op == 0)
    {
        return 0;
    }

    return 32 - __builtin_clz(op);
}

static ALWAYS_INLINE unsigned int find_lsb_set(uint32_t op)
{
#ifdef CONFIG_TOOLCHAIN_HAS_BUILTIN_FFS
    return __builtin_ffs(op);
#else
    /*
     * Toolchain does not have __builtin_ffs(). Leverage find_lsb_set()
     * by first clearing all but the lowest set bit.
     */

    op = op ^ (op & (op - 1));

    return find_msb_set(op);
#endif /* CONFIG_TOOLCHAIN_HAS_BUILTIN_FFS */
}


#define k_uptime_get_32() rt_tick_get()
#define CODE_UNREACHABLE
#define K_HOURS(x) (x*3600*1000)

//*********** k_sem -> rt_sem ***********************
#define k_sem rt_semaphore
#define K_SEM_DEFINE(sem,a,b) struct rt_semaphore sem = {.value=a}
#define k_sem_count_get(a) (a)->value
#define k_sem_reset(a) (a)->value=0
int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit);
void k_sem_give(struct k_sem *sem);
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);


//*********** k_heap -> rt_memheap ***********************
#define k_heap rt_memheap
#define k_heap_alloc(h,bytes,t) rt_memheap_alloc(h,bytes)
#define k_heap_free(h,mem) rt_memheap_free(mem)

//*********** k_thread -> rt_thread ***********************

#define k_tid_t rt_thread_t
#define k_thread_abort rt_thread_delete
#define k_yield() rt_thread_suspend(rt_thread_self())
typedef void (*k_thread_entry_t)(void *p1, void *p2, void *p3);
#define Z_THREAD_INITIALIZER(thread, stack, _stack_size,           \
                _entry, p1, p2, p3,                   \
                prio, options, delay, tname)         \
    {                                                        \
    .stack_addr = (stack),                   \
    .stack_size = (_stack_size),                         \
    .entry = (void *)_entry,             \
    .parameter = (void *)p1,                                   \
    .init_priority = (prio),                                     \
    .flags = (options),                               \
    .name = STRINGIFY(tname),                           \
    }

#define K_THREAD_STACK_DEFINE(stack,size) uint32_t stack[size>>2]
#define K_THREAD_DEFINE(name, stack_size,                                \
            entry, p1, p2, p3,                               \
            prio, options, delay)  \
            uint32_t _k_thread_stack_##name[stack_size>>2]; \
            struct rt_thread _k_thread_##name = \
                Z_THREAD_INITIALIZER(_k_thread_##name,      \
                     _k_thread_stack_##name, stack_size,\
                     entry, p1, p2, p3, prio, options,  \
                     delay, name);\
            rt_thread_t name = &_k_thread_##name;

#define K_PRIO_PREEMPT(pri) pri
#define K_KERNEL_STACK_SIZEOF(sym) sizeof(sym)
#define K_KERNEL_STACK_DEFINE(sym, size) uint8_t sym[size]
#define k_current_get() rt_current_thread
void k_thread_start(k_tid_t thread);
#define k_sleep(s) rt_thread_mdelay(s.ticks*1000)
#define k_thread rt_thread
k_tid_t k_thread_create(struct k_thread *new_thread,
                        uint32_t *stack,
                        rt_size_t stack_size,
                        k_thread_entry_t entry,
                        void *p1, void *p2, void *p3,
                        int prio, uint32_t options, k_timeout_t delay);
#define k_thread_name_set(th,name2) {\
    rt_strncpy(((rt_object_t)th)->name,name2,RT_NAME_MAX); \
    ((rt_object_t)th)->name[RT_NAME_MAX-1]='\0'; \
}

extern struct rt_thread *rt_current_thread;

//*********** k_mutex -> k_mutex ***********************
#define k_mutex rt_mutex
#define k_mutex_lock(a,timeout) rt_mutex_take(a,timeout.ticks)
#define k_mutex_unlock(a) rt_mutex_release(a)

extern int k_mutex_init(struct rt_mutex *mutex);

/*********** k_work -> rt_work *********************************/
#define k_work rt_work
#define k_work_delayable rt_delayed_work
#define k_work_q rt_workqueue

struct k_work_sync
{
    uint32_t dummy;
};

#define Z_WORK_INITIALIZER(work_handler) { \
    .work_func = (rt_workqueue_handler_t)(work_handler), \
}

typedef void (*k_work_handler_t)(struct rt_work *work);
typedef void (*rt_workqueue_handler_t)(struct rt_work *work, void *work_data);

#define k_work_delayable_from_work(m)  (struct k_work_delayable *)m
#define k_work_init_delayable(dwork,handler) rt_delayed_work_init(dwork,(rt_workqueue_handler_t)handler,NULL)
int k_work_cancel_delayable(struct k_work_delayable *dwork);

#define K_WORK_DEFINE(work, work_handler) \
    struct k_work work = Z_WORK_INITIALIZER(work_handler)
#define Z_WORK_DELAYABLE_INITIALIZER(work_handler) { \
        .work = { \
            .work_func =  (rt_workqueue_handler_t)(work_handler), \
            .flags = 0, \
        }, \
    }
#define K_WORK_DELAYABLE_DEFINE(work, work_handler) \
    struct k_work_delayable work \
      = Z_WORK_DELAYABLE_INITIALIZER(work_handler)
int k_work_submit(struct k_work *work);
#define  k_work_schedule_for_queue(queue,dwork,delay) rt_workqueue_submit_work(queue,(struct rt_work *)dwork,delay.ticks)

#define k_work_cancel_delayable_sync(dwork,sync) rt_workqueue_cancel_work_sync(rt_workqueue_sysq(),(struct rt_work *)dwork)
#define k_work_delayable_is_pending(dwork) ((dwork)->work.flags&RT_WORK_STATE_PENDING)
#define k_work_schedule(dwork,delay) rt_work_submit((struct rt_work*)dwork,delay.ticks)
#define k_work_init(work,handler) rt_work_init(work, (void(*)(struct rt_work*, void*))handler, NULL)
#define k_work_delayable_busy_get(dwork) ((dwork)->work.flags&RT_WORK_STATE_RUNNING)
#define k_work_reschedule(dwork,delay) k_work_reschedule2(dwork,delay.ticks)
rt_tick_t k_work_delayable_remaining_get(const struct rt_delayed_work *dwork);
int k_work_reschedule2(struct k_work_delayable *dwork, rt_tick_t delay);
bool k_work_flush(struct k_work *work, struct k_work_sync *sync);
#define k_work_busy_get(work) (work)->flags


/*********** k_work_queue -> rt_workqueue *********************************/
/** @brief A structure holding optional configuration items for a work
 * queue.
 *
 * This structure, and values it references, are not retained by
 * k_work_queue_start().
 */
struct k_work_queue_config
{
    /** The name to be given to the work queue thread.
     *
     * If left null the thread will not have a name.
     */
    const char *name;

    /** Control whether the work queue thread should yield between
     * items.
     *
     * Yielding between items helps guarantee the work queue
     * thread does not starve other threads, including cooperative
     * ones released by a work item.  This is the default behavior.
     *
     * Set this to @c true to prevent the work queue thread from
     * yielding between items.  This may be appropriate when a
     * sequence of items should complete without yielding
     * control.
     */
    bool no_yield;

    /** Control whether the work queue thread should be marked as
     * essential thread.
     */
    bool essential;
};

#define k_work_queue_thread_get(q) (q)->work_thread
#define k_work_queue_init rt_workqueue_init
#define k_work_reschedule_for_queue(queue,dwork,delay) rt_workqueue_submit_work(queue,(struct rt_work *)dwork,delay.ticks)
int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work);
void k_work_queue_start(struct k_work_q *queue,
                        void *stack,
                        rt_size_t stack_size,
                        int prio,
                        const struct k_work_queue_config *cfg);

/*********** k_spin_lock -> rt_hw_spin_lock *********************************/
struct k_spinlock
{
    rt_ubase_t key;
};
#define k_spinlock_key_t rt_base_t
#define k_spin_lock(l) rt_hw_spin_lock(&((l)->key))
#define k_spin_unlock(l,key) rt_hw_spin_unlock(&key)

/****************** sched_lock-> critical**********************/
#define k_sched_lock  rt_enter_critical
#define k_sched_unlock rt_exit_critical

/*********** BT MACROS *********************************/
#undef BT_PAC_SNK
#undef BT_PAC_SRC

/*********** fifo -> rt_data_queue ********************/

struct k_fifo
{
    struct rt_slist_node *head;
    struct rt_slist_node *tail;
};
#define K_FIFO_DEFINE(fifo) struct k_fifo fifo
#define k_fifo_init(fifo) memset(fifo, 0,sizeof(struct k_fifo))
#define k_fifo_is_empty(fifo) (((fifo)->head)==NULL)
#define k_fifo_peek_head(fifo) (void*)((fifo)->head)
void *k_fifo_get(struct k_fifo *fifo, k_timeout_t timeout);
void k_fifo_put(struct k_fifo *fifo, void *data);
void k_fifo_cancel_wait(struct k_fifo *queue);

#define _queue head


/*********** Lifo -> rt_data_queue ********************/

#define Z_LIFO_INITIALIZER(obj) NULL
struct k_lifo
{
    struct rt_slist_node   *head;
    struct rt_slist_node   *tail;
};
void k_lifo_put(struct k_lifo *lifo, void *data);
void *k_lifo_get(struct k_lifo *lifo, k_timeout_t timeout);

#define k_queue_prepend(queue,data) k_lifo_put((struct k_lifo *)queue,data)

/*********** Lifo -> rt_data_queue ********************/
#define k_poll_signal rt_wqueue
int k_poll_signal_raise(struct k_poll_signal *sig, int result);
#define K_POLL_SIGNAL_INITIALIZER(obj) {}

/*********** mem_slab -> rt_mempool ********************/
#define k_mem_slab rt_mempool
#define K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align) \
    struct rt_mempool name = { \
        .block_size=slab_block_size, \
        .block_total_count=slab_num_blocks,\
        .start_address=Z_STRINGIFY(name),\
    }

#define K_MEM_SLAB_DEFINE_STATIC(name, slab_block_size, slab_num_blocks, slab_align) \
        static struct rt_mempool name = { \
            .block_size=slab_block_size, \
            .block_total_count=slab_num_blocks,\
            .start_address=Z_STRINGIFY(name),\
        }

int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem,
                     k_timeout_t timeout);
#define k_mem_slab_free(slab,mem) rt_mp_free(mem)
#define k_mem_slab_num_free_get(mp) (mp)->block_free_count


/***********************************************************/


#define k_thread_stack_t uint32_t
#define K_THREAD_STACK_SIZEOF(x) sizeof(x)

/**
 * @brief Get system uptime.
 *
 * This routine returns the elapsed time since the system booted,
 * in milliseconds.
 *
 * @note
 *    While this function returns time in milliseconds, it does
 *    not mean it has millisecond resolution. The actual resolution depends on
 *    @kconfig{CONFIG_SYS_CLOCK_TICKS_PER_SEC} config option.
 *
 * @return Current uptime in milliseconds.
 */
static inline int64_t k_uptime_get(void)
{
    return (int64_t)rt_tick_get();
}

/**
 * @brief Get elapsed time.
 *
 * This routine computes the elapsed time between the current system uptime
 * and an earlier reference time, in milliseconds.
 *
 * @param reftime Pointer to a reference time, which is updated to the current
 *                uptime upon return.
 *
 * @return Elapsed time.
 */
static inline int64_t k_uptime_delta(int64_t *reftime)
{
    int64_t uptime, delta;

    uptime = k_uptime_get();
    delta = uptime - *reftime;
    *reftime = uptime;

    return delta;
}


/**********************************************************************************/

/* private - types bit positions */
enum _poll_types_bits
{
    /* can be used to ignore an event */
    _POLL_TYPE_IGNORE,

    /* to be signaled by k_poll_signal_raise() */
    _POLL_TYPE_SIGNAL,

    /* semaphore availability */
    _POLL_TYPE_SEM_AVAILABLE,

    /* queue/FIFO/LIFO data availability */
    _POLL_TYPE_DATA_AVAILABLE,

    /* msgq data availability */
    _POLL_TYPE_MSGQ_DATA_AVAILABLE,

    /* pipe data availability */
    _POLL_TYPE_PIPE_DATA_AVAILABLE,

    _POLL_NUM_TYPES
};

#define Z_POLL_TYPE_BIT(type) (1U << ((type) - 1U))

/* private - states bit positions */
enum _poll_states_bits
{
    /* default state when creating event */
    _POLL_STATE_NOT_READY,

    /* signaled by k_poll_signal_raise() */
    _POLL_STATE_SIGNALED,

    /* semaphore is available */
    _POLL_STATE_SEM_AVAILABLE,

    /* data is available to read on queue/FIFO/LIFO */
    _POLL_STATE_DATA_AVAILABLE,

    /* queue/FIFO/LIFO wait was cancelled */
    _POLL_STATE_CANCELLED,

    /* data is available to read on a message queue */
    _POLL_STATE_MSGQ_DATA_AVAILABLE,

    /* data is available to read from a pipe */
    _POLL_STATE_PIPE_DATA_AVAILABLE,

    _POLL_NUM_STATES
};

#define Z_POLL_STATE_BIT(state) (1U << ((state) - 1U))

#define _POLL_EVENT_NUM_UNUSED_BITS \
    (32 - (0 \
           + 8 /* tag */ \
           + _POLL_NUM_TYPES \
           + _POLL_NUM_STATES \
           + 1 /* modes */ \
          ))


/* Public polling API */

/* public - values for k_poll_event.type bitfield */
#define K_POLL_TYPE_IGNORE 0
#define K_POLL_TYPE_SIGNAL Z_POLL_TYPE_BIT(_POLL_TYPE_SIGNAL)
#define K_POLL_TYPE_SEM_AVAILABLE Z_POLL_TYPE_BIT(_POLL_TYPE_SEM_AVAILABLE)
#define K_POLL_TYPE_DATA_AVAILABLE Z_POLL_TYPE_BIT(_POLL_TYPE_DATA_AVAILABLE)
#define K_POLL_TYPE_FIFO_DATA_AVAILABLE K_POLL_TYPE_DATA_AVAILABLE
#define K_POLL_TYPE_MSGQ_DATA_AVAILABLE Z_POLL_TYPE_BIT(_POLL_TYPE_MSGQ_DATA_AVAILABLE)
#define K_POLL_TYPE_PIPE_DATA_AVAILABLE Z_POLL_TYPE_BIT(_POLL_TYPE_PIPE_DATA_AVAILABLE)

/* public - polling modes */
enum k_poll_modes
{
    /* polling thread does not take ownership of objects when available */
    K_POLL_MODE_NOTIFY_ONLY = 0,

    K_POLL_NUM_MODES
};

/* public - values for k_poll_event.state bitfield */
#define K_POLL_STATE_NOT_READY 0
#define K_POLL_STATE_SIGNALED Z_POLL_STATE_BIT(_POLL_STATE_SIGNALED)
#define K_POLL_STATE_SEM_AVAILABLE Z_POLL_STATE_BIT(_POLL_STATE_SEM_AVAILABLE)
#define K_POLL_STATE_DATA_AVAILABLE Z_POLL_STATE_BIT(_POLL_STATE_DATA_AVAILABLE)
#define K_POLL_STATE_FIFO_DATA_AVAILABLE K_POLL_STATE_DATA_AVAILABLE
#define K_POLL_STATE_MSGQ_DATA_AVAILABLE Z_POLL_STATE_BIT(_POLL_STATE_MSGQ_DATA_AVAILABLE)
#define K_POLL_STATE_PIPE_DATA_AVAILABLE Z_POLL_STATE_BIT(_POLL_STATE_PIPE_DATA_AVAILABLE)
#define K_POLL_STATE_CANCELLED Z_POLL_STATE_BIT(_POLL_STATE_CANCELLED)


struct z_poller
{
    uint32_t reserved;
};

/**
 * @brief Poll Event
 *
 */
struct k_poll_event
{
    /** PRIVATE - DO NOT TOUCH */
    sys_dnode_t _node;

    /** PRIVATE - DO NOT TOUCH */
    struct z_poller *poller;

    /** optional user-specified tag, opaque, untouched by the API */
    uint32_t tag: 8;

    /** bitfield of event types (bitwise-ORed K_POLL_TYPE_xxx values) */
uint32_t type:
    _POLL_NUM_TYPES;

    /** bitfield of event states (bitwise-ORed K_POLL_STATE_xxx values) */
uint32_t state:
    _POLL_NUM_STATES;

    /** mode of operation, from enum k_poll_modes */
    uint32_t mode: 1;

    /** unused bits in 32-bit word */
uint32_t unused:
    _POLL_EVENT_NUM_UNUSED_BITS;

    /** per-type data */
    union
    {
        /* The typed_* fields below are used by K_POLL_EVENT_*INITIALIZER() macros to ensure
         * type safety of polled objects.
         */
        void *obj, *typed_K_POLL_TYPE_IGNORE;
        struct k_poll_signal *signal, *typed_K_POLL_TYPE_SIGNAL;
        struct k_sem *sem, *typed_K_POLL_TYPE_SEM_AVAILABLE;
        struct k_fifo *fifo, *typed_K_POLL_TYPE_FIFO_DATA_AVAILABLE;
    };
};

#define K_POLL_EVENT_INITIALIZER(_event_type, _event_mode, _event_obj) \
    { \
    .poller = NULL, \
    .type = _event_type, \
    .state = K_POLL_STATE_NOT_READY, \
    .mode = _event_mode, \
    .unused = 0, \
    { \
        .typed_##_event_type = _event_obj, \
    }, \
    }

#define K_POLL_EVENT_STATIC_INITIALIZER(_event_type, _event_mode, _event_obj, \
                    event_tag) \
    { \
    .tag = event_tag, \
    .type = _event_type, \
    .state = K_POLL_STATE_NOT_READY, \
    .mode = _event_mode, \
    .unused = 0, \
    { \
        .typed_##_event_type = _event_obj, \
    }, \
    }

/**
 * @brief Wait for one or many of multiple poll events to occur
 *
 * This routine allows a thread to wait concurrently for one or many of
 * multiple poll events to have occurred. Such events can be a kernel object
 * being available, like a semaphore, or a poll signal event.
 *
 * When an event notifies that a kernel object is available, the kernel object
 * is not "given" to the thread calling k_poll(): it merely signals the fact
 * that the object was available when the k_poll() call was in effect. Also,
 * all threads trying to acquire an object the regular way, i.e. by pending on
 * the object, have precedence over the thread polling on the object. This
 * means that the polling thread will never get the poll event on an object
 * until the object becomes available and its pend queue is empty. For this
 * reason, the k_poll() call is more effective when the objects being polled
 * only have one thread, the polling thread, trying to acquire them.
 *
 * When k_poll() returns 0, the caller should loop on all the events that were
 * passed to k_poll() and check the state field for the values that were
 * expected and take the associated actions.
 *
 * Before being reused for another call to k_poll(), the user has to reset the
 * state field to K_POLL_STATE_NOT_READY.
 *
 * When called from user mode, a temporary memory allocation is required from
 * the caller's resource pool.
 *
 * @param events An array of events to be polled for.
 * @param num_events The number of events in the array.
 * @param timeout Waiting period for an event to be ready,
 *                or one of the special values K_NO_WAIT and K_FOREVER.
 *
 * @retval 0 One or more events are ready.
 * @retval -EAGAIN Waiting period timed out.
 * @retval -EINTR Polling has been interrupted, e.g. with
 *         k_queue_cancel_wait(). All output events are still set and valid,
 *         cancelled event(s) will be set to K_POLL_STATE_CANCELLED. In other
 *         words, -EINTR status means that at least one of output events is
 *         K_POLL_STATE_CANCELLED.
 * @retval -ENOMEM Thread resource pool insufficient memory (user mode only)
 * @retval -EINVAL Bad parameters (user mode only)
 */

__syscall int k_poll(struct k_poll_event *events, int num_events,
                     k_timeout_t timeout);

#endif

