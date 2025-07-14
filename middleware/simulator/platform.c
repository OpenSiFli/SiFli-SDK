#include <rtthread.h>
#include "board.h"
#ifdef RWBT_ENABLE
    #include "rwip.h"
#endif

#include <shell.h>
#ifdef RT_USING_DFS
    #include "dfs_fs.h"
#endif

static int platform_init_done = 0;
int platform_init(void)
{
    rt_components_init();
    return 0;
}

int platform_post_init(void)
{
    platform_init_done = 1;
    return 0;
}

int wait_platform_init_done(void)
{
    while (!platform_init_done) rt_thread_delay(1);
    return 0;
}


RT_WEAK void *cxx_mem_allocate(size_t size)
{
    return rt_malloc(size);
}
RT_WEAK void cxx_mem_free(void *ptr)
{
    rt_free(ptr);
}


