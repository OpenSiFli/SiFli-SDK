
#include "share_prefs.h"
#include "stdio.h"

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
    #define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifdef PKG_USING_FLASHDB
#include "flashdb.h"


typedef struct
{
    share_prefs_t prefs;
    struct fdb_kvdb db;
} flshdb_share_prefs_t;

share_prefs_t *share_prefs_open(const char *prefs_name, uint32_t mode)
{
    uint32_t name_len;
    flshdb_share_prefs_t *p_flshdb_prefs;
    share_prefs_t *p_prefs;
    fdb_kvdb_t p_db;

    if (NULL == prefs_name) return NULL;

    p_flshdb_prefs = rt_malloc(sizeof(flshdb_share_prefs_t));
    if (NULL == p_flshdb_prefs) return NULL;
    p_prefs = &p_flshdb_prefs->prefs;
    p_db    = &p_flshdb_prefs->db;

    name_len = strlen(prefs_name);
    name_len = MAX((SHARE_PREFS_MAX_NAME_LEN - 1), name_len);

    memcpy(p_prefs->prfs_name, prefs_name, name_len);
    p_prefs->prfs_name[name_len] = '\0';
    p_prefs->mode = mode;

    {
        struct fdb_default_kv default_kv;
        fdb_err_t err;

        default_kv.kvs = NULL;
        default_kv.num = 0;

        memset(p_db, 0, sizeof(struct fdb_kvdb));
#ifdef FDB_USING_FILE_MODE
#define SHARE_PREF_LEN  (16*1024)
        int sec_size = PKG_FLASHDB_ERASE_GRAN;
        int max_size = SHARE_PREF_LEN;
        int file_mode = true;
        rt_kprintf("share_prefs_open: sector_size %d size %d\n", sec_size, max_size);
        fdb_kvdb_control(p_db, FDB_KVDB_CTRL_SET_SEC_SIZE, (void *)&sec_size);
        fdb_kvdb_control(p_db, FDB_KVDB_CTRL_SET_MAX_SIZE, (void *)&max_size);
        fdb_kvdb_control(p_db, FDB_KVDB_CTRL_SET_FILE_MODE, (void *)&file_mode);
#endif
        err = fdb_kvdb_init(p_db, "share_pref", "prefdb", &default_kv, NULL);
        if (err != FDB_NO_ERR)
        {
            rt_free(p_flshdb_prefs);
            return NULL;
        }
    }


    return p_prefs;
}
rt_err_t share_prefs_close(share_prefs_t *prfs)
{
    flshdb_share_prefs_t *p_flshdb_prefs = (flshdb_share_prefs_t *) prfs;

    if (p_flshdb_prefs != NULL)
        rt_free(p_flshdb_prefs);

    return RT_EOK;
}

rt_err_t share_prefs_clear(share_prefs_t *prfs)
{
    return RT_ENOSYS; //NOT support yet
}

//combine prfs_nm and key to form kvdb_key
static char *_init_kvdb_key(const char *prfs_nm, const char *key)
{
    char *kvdb_key;
    uint32_t kvdb_key_len;

    //combine prfs_nm and key to form kvdb_key
    kvdb_key_len = strlen(prfs_nm) + 1 + strlen(key);
    kvdb_key = rt_malloc(kvdb_key_len + 1);
    if (NULL == kvdb_key) return NULL;
    sprintf(kvdb_key, "%s.%s", prfs_nm, key);
    kvdb_key[kvdb_key_len] = '\0';

    return kvdb_key;
}

rt_inline void _deinit_kvdb_key(const char *kvdb_key)
{
    rt_free((void *)kvdb_key);
}


static size_t _get_block(share_prefs_t *prfs, const char *key, void *buf, size_t buf_len, size_t *saved_value_len)
{
    flshdb_share_prefs_t *p_flshdb_prefs = (flshdb_share_prefs_t *) prfs;
    const char *prfs_nm = prfs->prfs_name;
    char *kvdb_key;
    size_t ret_v;

    kvdb_key = _init_kvdb_key(prfs_nm, key);
    if (NULL == kvdb_key)
    {
        return -1;
    }
    else
    {
        struct fdb_blob blob;
        ret_v = fdb_kv_get_blob(&p_flshdb_prefs->db, kvdb_key, fdb_blob_make(&blob, buf, buf_len));
        *saved_value_len = blob.saved.len;
        _deinit_kvdb_key(kvdb_key);
        return ret_v;
    }
}

static rt_err_t _set_block(share_prefs_t *prfs, const char *key, const void *value, int32_t value_len)
{
    flshdb_share_prefs_t *p_flshdb_prefs = (flshdb_share_prefs_t *) prfs;
    const char *prfs_nm = prfs->prfs_name;

    char *kvdb_key;
    rt_err_t ret_v;

    kvdb_key = _init_kvdb_key(prfs_nm, key);
    if (NULL == kvdb_key)
    {
        return RT_ENOMEM;
    }
    else
    {

        struct fdb_blob  blob;
        fdb_err_t err;
        err = fdb_kv_set_blob(&p_flshdb_prefs->db, kvdb_key, fdb_blob_make(&blob, value, value_len));
        ret_v = (err == FDB_NO_ERR) ? RT_EOK : err;
        _deinit_kvdb_key(kvdb_key);
        return ret_v;
    }
}


//anytype
rt_err_t share_prefs_remove(share_prefs_t *prfs, const char *key)
{
    flshdb_share_prefs_t *p_flshdb_prefs = (flshdb_share_prefs_t *) prfs;

    char *kvdb_key;
    rt_err_t ret_v;

    kvdb_key = _init_kvdb_key(prfs->prfs_name, key);
    if (NULL == kvdb_key)
    {
        return RT_ENOMEM;
    }
    else
    {
        fdb_err_t err;

        err = fdb_kv_del(&p_flshdb_prefs->db, kvdb_key);
        ret_v = (err == FDB_NO_ERR) ? RT_EOK : err;
        _deinit_kvdb_key(kvdb_key);
        return ret_v;
    }
}



//int
int32_t share_prefs_get_int(share_prefs_t *prfs, const char *key, int32_t default_v)
{
    int32_t ret_v;
    size_t sv_len, ret_len;

    ret_len = _get_block(prfs, key, &ret_v, sizeof(ret_v), &sv_len);

    if ((ret_len < sizeof(ret_v)) || (ret_len != sv_len))
        return default_v;
    else
        return ret_v;
}
rt_err_t share_prefs_set_int(share_prefs_t *prfs, const char *key, int32_t value)
{

    return share_prefs_set_block(prfs, key, &value, sizeof(value));
}

//string
int32_t share_prefs_get_string(share_prefs_t *prfs, const char *key, char *buf, int32_t buf_len)
{
    return share_prefs_get_block(prfs, key, buf, buf_len);
}
rt_err_t share_prefs_set_string(share_prefs_t *prfs, const char *key, const char *buf)
{
    return share_prefs_set_block(prfs, key, buf, strlen(buf));
}

//block
int32_t share_prefs_get_block(share_prefs_t *prfs, const char *key, void *buf, int32_t buf_len)
{
    size_t sv_len, ret_len;

    ret_len = _get_block(prfs, key, buf, buf_len, &sv_len);

    if (buf_len < (int32_t)sv_len)
        return (buf_len - sv_len); //buf is too small, return a negative len
    else
        return ret_len;
}
rt_err_t share_prefs_set_block(share_prefs_t *prfs, const char *key, const void *buf, int32_t buf_len)
{
    return _set_block(prfs, key, buf, buf_len);
}
#endif

