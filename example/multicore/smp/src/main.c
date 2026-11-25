/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <rthw.h>
#include <rtsmp.h>

#ifdef RT_USING_SMP
    #define EXAMPLE_HANDOFFS 128u
    #define EXAMPLE_MIGRATIONS 32u
    #define EXAMPLE_WORDS 64u
    #define EXAMPLE_WAIT_TICKS (5 * RT_TICK_PER_SECOND)

    #if defined(BSP_USING_PSRAM) && defined(RT_SMP_SHARED_XIP) && \
        (defined(RT_SMP_PSRAM_NONCACHE) || defined(RT_SMP_PSRAM_WT_SYNC))
        #define EXAMPLE_HAS_SHARED_PSRAM
/* Existing SDK PSRAM section; no application-specific linker rewrite. */
static rt_uint8_t s_psram_pool[16384]
    __attribute__((section(".bss.l2_non_ret_data_smp"), aligned(32)));
static struct rt_memheap s_psram_heap;
    #endif

/* Explicitly shared constants: a function attribute cannot move its callees
 * or referenced strings/constants out of the HCPU-only Flash window. */
static const rt_uint32_t s_seeds[4] RT_SMP_RODATA =
{
    0x19283746u, 0xabcdef01u, 0x5700a3c1u, 0xf1e2d3c4u
};
static struct rt_semaphore s_turn[2], s_produced[2], s_done, s_reaped;
static struct rt_mutex s_mutex;
static volatile rt_uint32_t s_payload[EXAMPLE_WORDS];
static volatile rt_uint32_t *s_blocks[2];
static volatile rt_uint32_t s_errors[2], s_visits[2], s_moves[2];
static volatile rt_uint32_t s_counter, s_cleaned;
static volatile float s_fp_step = 0.25f;
static rt_uint32_t s_round;
static rt_bool_t s_use_psram;
#ifdef RT_USING_HOOK
static volatile rt_uint32_t s_hook_errors;

/* Existing scheduler hooks receive two valid threads, including on CPU1. */
static void RT_SMP_TEXT _schedule_hook(rt_thread_t from, rt_thread_t to)
{
    if (from == RT_NULL || to == RT_NULL)
    {
        s_hook_errors++;
    }
}
#endif

#ifdef RT_USING_MEMPOOL
static struct rt_mempool s_pool;
static rt_uint8_t s_pool_storage[2048] __attribute__((aligned(RT_ALIGN_SIZE)));
static rt_bool_t s_use_pool;
#endif

static rt_uint32_t RT_SMP_TEXT _pattern(rt_uint32_t sequence, unsigned word)
{
    return s_seeds[word & 3u] ^ sequence * 2654435761u ^ word * 1664525u;
}

/* Cleanup runs on the HCPU reaper. It precedes the final TCB/stack free. */
static void _cleanup(rt_thread_t thread)
{
    unsigned id = thread->user_data;
    if (thread->oncpu != RT_CPU_DETACHED)
    {
        s_errors[id]++;
    }
    s_cleaned++;
    rt_sem_release(&s_reaped);
}

static void RT_SMP_TEXT _worker(void *parameter)
{
    unsigned id = (rt_ubase_t)parameter;
    rt_thread_t self = rt_thread_self();
    volatile rt_uint32_t cookie[EXAMPLE_WORDS];
    float fp = 0.0f;

    if (rt_hw_cpu_id() != id)
    {
        s_errors[id]++;
    }
    for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
    {
        cookie[word] = _pattern(s_round * 2u + id, word);
    }

    for (unsigned iteration = 0; iteration < EXAMPLE_HANDOFFS; iteration++)
    {
        if (rt_sem_take(&s_turn[id], EXAMPLE_WAIT_TICKS) != RT_EOK)
        {
            s_errors[id]++;
            break;
        }
        rt_uint32_t sequence = iteration * 2u + id;
        for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
        {
            if (s_payload[word] != _pattern(sequence, word))
            {
                s_errors[id]++;
            }
            s_payload[word] = _pattern(sequence + 1u, word);
        }
        rt_sem_release(&s_turn[id ^ 1u]);

        if (rt_mutex_take(&s_mutex, EXAMPLE_WAIT_TICKS) == RT_EOK)
        {
            s_counter++;
            rt_mutex_release(&s_mutex);
        }
        else
        {
            s_errors[id]++;
        }
    }

    for (unsigned iteration = 0; iteration < EXAMPLE_MIGRATIONS; iteration++)
    {
        unsigned target = rt_hw_cpu_id() ^ 1u;
        fp += s_fp_step;
        if (rt_thread_control(self, RT_THREAD_CTRL_BIND_CPU,
                              (void *)(rt_ubase_t)target) != RT_EOK ||
            rt_hw_cpu_id() != target || rt_thread_self() != self)
        {
            s_errors[id]++;
        }
        s_visits[id] |= 1u << rt_hw_cpu_id();
        s_moves[id]++;
        for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
        {
            if (cookie[word] != _pattern(s_round * 2u + id, word))
            {
                s_errors[id]++;
            }
        }
        if (fp != (iteration + 1u) * 0.25f)
        {
            s_errors[id]++;
        }
    }

    rt_thread_control(self, RT_THREAD_CTRL_BIND_CPU, (void *)(rt_ubase_t)id);
    volatile rt_uint32_t *block;
    #ifdef RT_USING_MEMPOOL
    if (s_use_pool)
    {
        block = rt_mp_alloc(&s_pool, EXAMPLE_WAIT_TICKS);
    }
    else
    #endif
    #ifdef EXAMPLE_HAS_SHARED_PSRAM
    if (s_use_psram)
    {
        block = rt_memheap_alloc(&s_psram_heap, sizeof(s_payload));
    }
    else
    #endif
    {
        block = rt_malloc(sizeof(s_payload));
    }
    s_blocks[id] = block;
    if (block)
    {
        for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
        {
            block[word] = _pattern(id, word);
        }
    }
    else
    {
        s_errors[id]++;
    }
    rt_sem_release(&s_produced[id]);
    if (rt_sem_take(&s_produced[id ^ 1u], EXAMPLE_WAIT_TICKS) == RT_EOK)
    {
        block = s_blocks[id ^ 1u];
        if (block)
        {
            for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
            {
                if (block[word] != _pattern(id ^ 1u, word))
                {
                    s_errors[id]++;
                }
            }
            #ifdef RT_USING_MEMPOOL
            if (s_use_pool)
            {
                rt_mp_free((void *)block);
            }
            else
            #endif
            #ifdef EXAMPLE_HAS_SHARED_PSRAM
            if (s_use_psram)
            {
                rt_memheap_free((void *)block);
            }
            else
            #endif
            {
                rt_free((void *)block);
            }
        }
        else
        {
            s_errors[id]++;
        }
    }
    else
    {
        s_errors[id]++;
    }
    rt_sem_release(&s_done);
    /* Returning exits the thread; the reaper later releases its allocation. */
}

int main(void)
{
    rt_uint32_t heap_before, heap_after;
    rt_err_t result;
    rt_kprintf("SMP_BEGIN\n");
    for (unsigned id = 0; id < 2; id++)
    {
        rt_sem_init(&s_turn[id], "turn", 0, RT_IPC_FLAG_FIFO);
        rt_sem_init(&s_produced[id], "prod", 0, RT_IPC_FLAG_FIFO);
    }
    rt_sem_init(&s_done, "done", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&s_reaped, "reaped", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&s_mutex, "count", RT_IPC_FLAG_FIFO);
    #ifdef RT_USING_MEMPOOL
    result = rt_mp_init(&s_pool, "smppool", s_pool_storage,
                        sizeof(s_pool_storage), sizeof(s_payload));
    if (result != RT_EOK)
    {
        rt_kprintf("SMP_FAIL pool_init=%d\n", result);
        return 0;
    }
    #endif
    #ifdef EXAMPLE_HAS_SHARED_PSRAM
    result = rt_memheap_init(&s_psram_heap, "smpps", s_psram_pool, sizeof(s_psram_pool));
    if (result != RT_EOK)
    {
        rt_kprintf("SMP_FAIL psram_init=%d\n", result);
        return 0;
    }
    rt_size_t psram_before = s_psram_heap.available_size;
    #endif
    #ifdef RT_USING_HOOK
    rt_scheduler_sethook(_schedule_hook);
    #endif
    result = rt_smp_start_secondary();
    if (result != RT_EOK)
    {
        rt_kprintf("SMP_FAIL secondary_start=%d\n", result);
        return 0;
    }
    rt_memory_info(RT_NULL, &heap_before, RT_NULL);
    rt_tick_t start = rt_tick_get();

    for (s_round = 0; s_round < EXAMPLE_SMP_ROUNDS; s_round++)
    {
        for (unsigned word = 0; word < EXAMPLE_WORDS; word++)
        {
            s_payload[word] = _pattern(0, word);
        }
        s_use_psram = (s_round & 1u) != 0;
        #ifdef RT_USING_MEMPOOL
        s_use_pool = !s_use_psram && (s_round & 2u) != 0;
        #endif
        /* The last handoff leaves a token for CPU0; reset it between rounds. */
        rt_sem_control(&s_turn[0], RT_IPC_CMD_RESET, RT_NULL);
        rt_sem_release(&s_turn[0]);
        for (unsigned id = 0; id < 2; id++)
        {
            rt_thread_t thread = rt_thread_create(id ? "smpa" : "smph", _worker,
                                                 (void *)(rt_ubase_t)id, 2048,
                                                 RT_THREAD_PRIORITY_MAX / 2, 2);
            if (!thread)
            {
                rt_kprintf("SMP_FAIL thread_create\n");
                return 0;
            }
            if (thread->bind_cpu != 0)
            {
                s_errors[id]++;
            }
            thread->cleanup = _cleanup;
            thread->user_data = id;
            rt_thread_control(thread, RT_THREAD_CTRL_BIND_CPU, (void *)(rt_ubase_t)id);
            rt_thread_startup(thread);
        }
        for (unsigned id = 0; id < 2; id++)
        {
            if (rt_sem_take(&s_done, EXAMPLE_WAIT_TICKS) != RT_EOK ||
                rt_sem_take(&s_reaped, EXAMPLE_WAIT_TICKS) != RT_EOK)
            {
                rt_kprintf("SMP_FAIL completion_timeout\n");
                return 0;
            }
        }
        /* A cleanup callback runs before free. Wait on memory accounting,
         * never inspect the TCB or stack through a pointer after its exit. */
        rt_tick_t deadline = rt_tick_get();
        do
        {
            rt_thread_mdelay(1);
            rt_memory_info(RT_NULL, &heap_after, RT_NULL);
        }
        while (heap_after != heap_before && rt_tick_get() - deadline < EXAMPLE_WAIT_TICKS);
        if (s_errors[0] || s_errors[1] || s_cleaned != (s_round + 1u) * 2u ||
            heap_before != heap_after)
        {
            rt_kprintf("SMP_FAIL round=%u errors=%u cleaned=%u heap_before=%u heap_after=%u\n",
                       s_round, s_errors[0] + s_errors[1], s_cleaned, heap_before, heap_after);
            return 0;
        }
    }
    #ifdef RT_USING_MEMPOOL
    if (s_pool.block_free_count != s_pool.block_total_count)
    {
        rt_kprintf("SMP_FAIL pool_leak\n");
        return 0;
    }
    #endif
    #ifdef EXAMPLE_HAS_SHARED_PSRAM
    if (s_psram_heap.available_size != psram_before)
    {
        rt_kprintf("SMP_FAIL psram_leak\n");
        return 0;
    }
    #endif
    #ifdef RT_USING_HOOK
    if (s_hook_errors)
    {
        rt_kprintf("SMP_FAIL scheduler_hook\n");
        return 0;
    }
    #endif
    if (s_counter != EXAMPLE_SMP_ROUNDS * EXAMPLE_HANDOFFS * 2u ||
        s_visits[0] != 3 || s_visits[1] != 3 ||
        s_moves[0] + s_moves[1] != EXAMPLE_SMP_ROUNDS * EXAMPLE_MIGRATIONS * 2u)
    {
        rt_kprintf("SMP_FAIL counters\n");
        return 0;
    }
    rt_kprintf("SMP_RESULT rounds=%u threads=%u migrations=%u errors=0 heap_restored=1 cpus=3 ticks=%u\n",
               s_round, s_cleaned, s_moves[0] + s_moves[1], rt_tick_get() - start);
    rt_kprintf("SMP_PASS\n");
    return 0;
}
#else
int main(void)
{
    rt_kprintf("SMP_DISABLED\n");
    return 0;
}
#endif
