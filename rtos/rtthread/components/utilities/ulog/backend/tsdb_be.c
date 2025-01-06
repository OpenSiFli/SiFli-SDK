#include <rthw.h>
#include <ulog.h>
#include <flashdb.h>
#include <string.h>
#include <stdlib.h>

#ifdef ULOG_BACKEND_USING_TSDB

#if defined(ULOG_ASYNC_OUTPUT_BY_THREAD) && ULOG_ASYNC_OUTPUT_THREAD_STACK < 384
    #error "The thread stack size must more than 384 when using async output by thread (ULOG_ASYNC_OUTPUT_BY_THREAD)"
#endif

int ulog_tsdb_init(void);
void ulog_tsdb_append(const char *str, size_t len);

static struct ulog_backend tsdb;

void ulog_tsdb_backend_output(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw,
                              const char *log, size_t len)
{

    ulog_tsdb_append(log, len);
}

void ulog_tsdb_en(int enable)
{
    if (enable == 0)
    {
        ulog_backend_unregister(&tsdb);
    }
    else
    {
        ulog_backend_register(&tsdb, "tsdb", RT_TRUE);
    }
}


int ulog_tsdb_backend_init(void)
{

    ulog_init();
    tsdb.output = ulog_tsdb_backend_output;
    ulog_tsdb_init();
    ulog_backend_register(&tsdb, "tsdb", RT_TRUE);

    return 0;
}
INIT_ENV_EXPORT(ulog_tsdb_backend_init);


#ifdef FDB_USING_TSDB

#define FDB_LOG_TAG "[tsdb log]"
#define FDB_MAX_TSL_LEN 256

static bool query_cb(fdb_tsl_t tsl, void *arg);
struct fdb_tsdb _global_tsdb =
{
    0
};
struct fdb_kvdb _global_kvdb =
{
    0
};

static fdb_time_t get_hw_rtc_time(void)
{

    time_t t = 0;
    time(&t);
    return t;
}

int ulog_tsdb_init()
{

    fdb_err_t result;
    result = fdb_tsdb_init(&_global_tsdb, "log", "ts_area", get_hw_rtc_time, FDB_MAX_TSL_LEN, NULL);
    return result;
}

void ulog_tsdb_append(const char *str, size_t len)
{

    struct fdb_blob blob;
    fdb_tsl_append(&_global_tsdb, fdb_blob_make(&blob, str, len));
}

void ulog_add_tsl(int argc, char **argv)
{

    struct fdb_blob blob;
    if (argc >= 2)
    {
        fdb_tsl_append(&_global_tsdb, fdb_blob_make(&blob, argv[1], sizeof(argv[1])));
    }
}
MSH_CMD_EXPORT(ulog_add_tsl, add tsl);

static void ulog_tsdb_clr(void)
{
    fdb_tsl_clean(&_global_tsdb);
}
MSH_CMD_EXPORT(ulog_tsdb_clr, clean tsdb);

void ulog_tsdb_query(void)
{
    ulog_pause(1);
    fdb_tsl_iter(&_global_tsdb, query_cb, &_global_tsdb);
    ulog_pause(0);
}
MSH_CMD_EXPORT(ulog_tsdb_query, get all ulog from tsdb);

void ulog_tsdb_query2flash(fdb_tsl_cb func)
{
    ulog_pause(1);
    fdb_tsl_iter(&_global_tsdb, func, &_global_tsdb);
    ulog_pause(0);
}

int ulog_tsdb_query_by_time(int argc, char **argv)
{
    ulog_pause(1);
    if (argc >= 10)
    {

        struct tm tm_from =
        {
            .tm_year = atoi((const char *)argv[1]) - 1900,
            .tm_mon  = atoi((const char *)argv[2]) - 1,
            .tm_mday = atoi((const char *)argv[3]),
            .tm_hour = atoi((const char *)argv[4]),
            .tm_min  = atoi((const char *)argv[5]),
            .tm_sec  = 0
        };
        struct tm tm_to   =
        {
            .tm_year = atoi((const char *)argv[1]) - 1900,
            .tm_mon  = atoi((const char *)argv[6]) - 1,
            .tm_mday = atoi((const char *)argv[7]),
            .tm_hour = atoi((const char *)argv[8]),
            .tm_min  = atoi((const char *)argv[9]),
            .tm_sec  = 0
        };
        //转换时间格式
        time_t from_time = mktime(&tm_from), to_time = mktime(&tm_to);
        rt_kprintf("from time = %ld,  to time = %ld\n", (long)from_time, (long)to_time);
        fdb_tsl_iter_by_time(&_global_tsdb, from_time, to_time, query_cb, &_global_tsdb);
    }
    else
    {

        rt_kprintf("Please input var:'time section [from_ymdh to_ymdh]'\n");
        rt_kprintf("sample : ulog_tsdb_query_by_time 2018 8 8 8 8 9 9 9 9\n");
        rt_kprintf("mean : query from 2018-08-08-8:00 to 2018-09-09-9:00\n");
    }
    ulog_pause(0);
    return 0;
}
MSH_CMD_EXPORT(ulog_tsdb_query_by_time, tsdb search by time Please input var: time section [from_ymdh to_ymdh]);

static bool query_cb(fdb_tsl_t tsl, void *arg)
{

    struct fdb_blob blob;
    char str[FDB_MAX_TSL_LEN];
    fdb_tsdb_t db = arg;
    fdb_blob_read((fdb_db_t) db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, str, FDB_MAX_TSL_LEN)));
    if (tsl->log_len >= FDB_MAX_TSL_LEN)
        str[FDB_MAX_TSL_LEN - 1] = '\0';
    else
        str[tsl->log_len] = '\0';
    rt_kprintf("%s", str);
    return false;
}
#endif /* FDB_USING_TSDB */
#endif /* ULOG_BACKEND_USING_TSDB */
