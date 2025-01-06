#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "drv_flash.h"
#include "fal.h"
#include <flashdb.h>

/* Common functions for RT-Thread based platform -----------------------------------------------*/

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif

#define FS_ROOT "root"

/**
 * @brief Mount fs.
 */
int mnt_init(void)
{
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, FS_ROOT);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)//Format file system
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
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

/* User code start from here --------------------------------------------------------*/
#ifdef FDB_USING_KVDB

    #define SIFLI_KVDB_NAME "kvdb_tst"
    #define SIFLI_KVDB_PATH "kvdb_tst"
    #define SIFLI_KVDB_MAX (0x4000)

    static struct fdb_kvdb g_kvdb_db;

    #if (FDB_KV_CACHE_TABLE_SIZE == 1)
        static uint32_t g_ble_db_cache[256];
    #endif /* (FDB_KV_CACHE_TABLE_SIZE == 1) */

    static fdb_kvdb_t p_kvdb_db = &g_kvdb_db;

    static struct rt_mutex g_kvdb_db_mutex;

#endif

#ifdef FDB_USING_TSDB

    #define SIFLI_TSDB_NAME "tsdb_tst"
    #define SIFLI_TSDB_PATH "tsdb_tst"
    #define SIFLI_TSDB_MAX (0x4000)
    #define SIFLI_TSDB_MAX_DATA_LEN (256)

    static struct fdb_tsdb g_tsdb_db;

    static fdb_tsdb_t p_tsdb_db = &g_tsdb_db;

    static struct rt_mutex g_tsdb_db_mutex;

#endif


#ifdef FDB_USING_KVDB
/**
 * @brief lock interface for kvdb.
 */
static void lock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_take(&g_kvdb_db_mutex, RT_WAITING_FOREVER);
    //rt_kprintf("lock\n");
}

/**
 * @brief unlock interface for kvdb.
 */
static void unlock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_release(&g_kvdb_db_mutex);
    //rt_kprintf("unlock\n");
}

/**
 * @brief KVDB intialization.
 */
static int example_kvdb_init(void)
{
    int ret = -1;
    fdb_err_t err;

    /* mutex initialization. */
    rt_mutex_init(&g_kvdb_db_mutex, "kvdb_mtx", RT_IPC_FLAG_FIFO);

    memset(p_kvdb_db, 0, sizeof(*p_kvdb_db));

    /* set lock/unlock interface. */
    fdb_kvdb_control(p_kvdb_db, FDB_KVDB_CTRL_SET_LOCK, lock);
    fdb_kvdb_control(p_kvdb_db, FDB_KVDB_CTRL_SET_UNLOCK, unlock);

    do
    {
        const char *path = SIFLI_KVDB_PATH;
#ifdef FDB_USING_FILE_MODE
        int sec_size = PKG_FLASHDB_ERASE_GRAN;
        int max_size = SIFLI_KVDB_MAX;
        bool file_mode = true;
        rt_kprintf("kvdb init: sector_size %d size %d\n", sec_size, max_size);
        fdb_kvdb_control(p_kvdb_db, FDB_KVDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
        fdb_kvdb_control(p_kvdb_db, FDB_KVDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
        fdb_kvdb_control(p_kvdb_db, FDB_KVDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
        if (0 != access(path, 0) && 0 != mkdir(path, 0))
        {
            rt_kprintf("create kvdb %s fail\n", SIFLI_KVDB_NAME);
            break;
        }

#endif /* FDB_USING_FILE_MODE */
        err = fdb_kvdb_init(p_kvdb_db, SIFLI_KVDB_NAME, path, NULL, NULL);
        if (err != FDB_NO_ERR)
        {
            rt_kprintf("kvdb init failed !!!\n");
            break;
        }

        ret = 0;
    }
    while (0);

    rt_kprintf("kvdb init sucess !!!\n");
    return ret;
}

/**
 * @brief kvdb commands.
 */
int flashdb_kvdb_test(int argc, char *argv[])
{
    struct fdb_blob blob;
    int int_value = 0;

    if (0 == rt_strncmp("get", argv[1], rt_strlen("get")))
    {
        if (0 == rt_strncmp("int", argv[3], rt_strlen("int")))
        {
            rt_kprintf("[%s] int\n", argv[2]);
            /* GET the KV value */
            fdb_kv_get_blob(p_kvdb_db, argv[2], fdb_blob_make(&blob, &int_value, sizeof(int_value)));
            /* the blob.saved.len is more than 0 when get the value successful */
            if (blob.saved.len > 0)
            {
                rt_kprintf("get the %s value is %d \n", argv[2], int_value);
            }
            else
            {
                rt_kprintf("get the %s failed\n", argv[2]);
            }
        }

        if (0 == rt_strncmp("str", argv[3], rt_strlen("str")))
        {
            rt_kprintf("[%s] str\n", argv[2]);
            char *return_value;

            /* Get the KV value.
            * NOTE: The return value saved in fdb_kv_get's buffer. Please copy away as soon as possible.
            */
            return_value = fdb_kv_get(p_kvdb_db, argv[2]);
            /* the return value is NULL when get the value failed */
            if (return_value != NULL)
            {
                rt_kprintf("get the %s value is %s \n", argv[2], return_value);
            }
            else
            {
                rt_kprintf("get the %s failed\n", argv[2]);
            }
        }
    }

    if (0 == rt_strncmp("set", argv[1], rt_strlen("set")))
    {
        if (0 == rt_strncmp("int", argv[3], rt_strlen("int")))
        {
            /* CHANGE the KV value */
            int_value = atoi(argv[4]);
            /* change the KV's value */
            fdb_kv_set_blob(p_kvdb_db, argv[2], fdb_blob_make(&blob, &int_value, sizeof(int_value)));
            rt_kprintf("set the %s value to %d\n", argv[2], int_value);
        }

        if (0 == rt_strncmp("str", argv[3], rt_strlen("str")))
        {
            /* CHANGE the KV value */
            fdb_kv_set(p_kvdb_db, argv[2], argv[4]);
            rt_kprintf("set %s value to %s\n", argv[2], argv[4]);
        }
    }

    if (0 == rt_strncmp("del", argv[1], rt_strlen("del")))
    {
        {
            /* DELETE the KV by name */
            fdb_kv_del(p_kvdb_db, argv[2]);
            rt_kprintf("delete the %s finish\n", argv[2]);
        }
    }

    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(flashdb_kvdb_test, kvdb, test kvdb)

#endif

#ifdef FDB_USING_TSDB
/**
 * @brief lock interface for tsdb.
 */
static void tsdb_lock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_take(&g_tsdb_db_mutex, RT_WAITING_FOREVER);
    //rt_kprintf("lock\n");
}

/**
 * @brief unlock interface for tsdb.
 */
static void tsdb_unlock(fdb_db_t db)
{
    rt_err_t err = rt_mutex_release(&g_tsdb_db_mutex);
    //rt_kprintf("unlock\n");
}

/**
 * @brief gettime interface for tsdb.
 */
static fdb_time_t get_time(void)
{
    time_t t = 0;

    time(&t);
    return t;
}

/**
 * @brief TSDB initialization.
 */
static int example_tsdb_init(void)
{
    fdb_err_t result;

    /* set lock/unlock interface. */
    fdb_kvdb_control(p_kvdb_db, FDB_TSDB_CTRL_SET_LOCK, lock);
    fdb_kvdb_control(p_kvdb_db, FDB_TSDB_CTRL_SET_UNLOCK, unlock);

    const char *path = SIFLI_TSDB_PATH;
#ifdef FDB_USING_FILE_MODE
    int sec_size = PKG_FLASHDB_ERASE_GRAN;
    int max_size = SIFLI_TSDB_MAX;
    bool file_mode = true;
    rt_kprintf("TSDB init: sector_size %d size %d\n", sec_size, max_size);
    fdb_tsdb_control(p_tsdb_db, FDB_TSDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
    fdb_tsdb_control(p_tsdb_db, FDB_TSDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
    fdb_tsdb_control(p_tsdb_db, FDB_TSDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
    if (0 != access(path, 0) && 0 != mkdir(path, 0))
    {
        rt_kprintf("create tsdb %s fail\n", SIFLI_TSDB_NAME);
        return -1;
    }

#endif /* FDB_USING_FILE_MODE */

    result = fdb_tsdb_init(p_tsdb_db, path, SIFLI_TSDB_NAME, get_time, SIFLI_TSDB_MAX_DATA_LEN, NULL);
    RT_ASSERT(FDB_NO_ERR == result);

    rt_kprintf("tsdb init sucess !!!\n");
    return 0;
}

/**
 * @brief tsdb query callback.
 */
static bool query_cb(fdb_tsl_t tsl, void *arg)
{
    struct fdb_blob blob;
    int value;
    struct tm *time_info;
    fdb_tsdb_t db = arg;

    fdb_blob_read((fdb_db_t) db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, &value, sizeof(int))));
    time_info = localtime(&tsl->time);
    rt_kprintf("[query_cb] queried a TSL: value: %d time: %d %s", value, tsl->time, asctime(time_info));

    return false;
}

/**
 * @brief tsdb query callback (by time).
 */
static bool query_by_time_cb(fdb_tsl_t tsl, void *arg)
{
    struct fdb_blob blob;
    int value;
    struct tm *time_info;
    fdb_tsdb_t db = arg;

    fdb_blob_read((fdb_db_t) db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, &value, sizeof(int))));
    time_info = localtime(&tsl->time);
    rt_kprintf("[query_by_time_cb] queried a TSL: value: %d time: %d %s", value, tsl->time, asctime(time_info));

    return false;
}

/**
 * @brief tsdb commands.
 */
int flashdb_tsdb_test(int argc, char *argv[])
{
    struct fdb_blob blob;
    int int_value = 0;

    if (0 == rt_strncmp("append", argv[1], rt_strlen("append")))
    {
        /* APPEND new TSL (time series log) */
        int value;
        size_t count;

        /* append new log to TSDB */
        value = atoi(argv[2]);
        fdb_tsl_append(p_tsdb_db, fdb_blob_make(&blob, &value, sizeof(int)));
        rt_kprintf("append tsdb item : value = %d\n", value);
        /* query all FDB_TSL_WRITE status TSL's count in TSDB by time */
        count = fdb_tsl_query_count(p_tsdb_db, 0, 0x7FFFFFFF, FDB_TSL_WRITE);
        rt_kprintf("tsdb count is: %d\n", count);
    }

    if (0 == rt_strncmp("query_all", argv[1], rt_strlen("query_all")))
    {
        /* QUERY the TSDB */
        /* query all TSL in TSDB by iterator */
        rt_kprintf("query all:\n");
        fdb_tsl_iter(p_tsdb_db, query_cb, p_tsdb_db);
    }

    if (0 == rt_strncmp("query_by_time", argv[1], rt_strlen("query_by_time")))
    {
        /* QUERY the TSDB by time */
        time_t from_time = atoi(argv[2]);
        time_t to_time = atoi(argv[3]);
        size_t count;

        rt_kprintf("query by time:\n");
        rt_kprintf("from time:%d %s", from_time, asctime(localtime(&from_time)));
        rt_kprintf("to time:%d %s", to_time, asctime(localtime(&to_time)));
        /* query all TSL in TSDB by time */
        fdb_tsl_iter_by_time(p_tsdb_db, from_time, to_time, query_by_time_cb, p_tsdb_db);
        /* query all FDB_TSL_WRITE status TSL's count in TSDB by time */
        count = fdb_tsl_query_count(p_tsdb_db, from_time, to_time, FDB_TSL_WRITE);
        rt_kprintf("query count is: %d\n", count);
    }

    if (0 == rt_strncmp("clear", argv[1], rt_strlen("clear")))
    {
        fdb_tsl_clean(p_tsdb_db);
        rt_kprintf("clear tsdb.\n");
    }

    return RT_EOK;
}

MSH_CMD_EXPORT_ALIAS(flashdb_tsdb_test, tsdb, test tsdb)
#endif

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_kprintf("\n[FLASHDB]Flashdb Example.\n");

#ifdef FDB_USING_KVDB
    /* KVDB initialization */
    int err = example_kvdb_init();
    if (0 != err)
    {
        goto __END;
    }
#endif

#ifdef FDB_USING_TSDB
    /* TSDB initialization */
    err = example_tsdb_init();
    if (0 != err)
    {
        goto __END;
    }
#endif

    /* ls files in root. */
    extern void ls(const char *name);
    ls("/");

__END:
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}

