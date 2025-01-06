
#include "share_prefs.h"
#include "stdio.h"

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
    #define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifdef PKG_EASYFLASH_ENV
#include "easyflash.h"

share_prefs_t *share_prefs_open(const char *prefs_name, uint32_t mode)
{
    uint32_t name_len;
    share_prefs_t *p_prefs;

    if (NULL == prefs_name) return NULL;

    p_prefs = rt_malloc(sizeof(share_prefs_t));
    if (NULL == p_prefs) return NULL;

    name_len = strlen(prefs_name);
    name_len = MAX((SHARE_PREFS_MAX_NAME_LEN - 1), name_len);

    memcpy(p_prefs->prfs_name, prefs_name, name_len);
    p_prefs->prfs_name[name_len] = '\0';
    p_prefs->mode = mode;

    return p_prefs;
}
rt_err_t share_prefs_close(share_prefs_t *prfs)
{
    if (prfs != NULL)
        rt_free(prfs);

    return RT_EOK;
}

rt_err_t share_prefs_clear(share_prefs_t *prfs)
{
    return RT_ENOSYS; //NOT support yet
}

//combine prfs_nm and key to form ez_key
static char *_init_ez_key(const char *prfs_nm, const char *key)
{
    char *ez_key;
    uint32_t ez_key_len;

    //combine prfs_nm and key to form ez_key
    ez_key_len = strlen(prfs_nm) + 1 + strlen(key);
    ez_key = rt_malloc(ez_key_len + 1);
    if (NULL == ez_key) return NULL;
    sprintf(ez_key, "%s.%s", prfs_nm, key);
    ez_key[ez_key_len] = '\0';

    return ez_key;
}

rt_inline void _deinit_ez_key(const char *ez_key)
{
    rt_free((void *)ez_key);
}


static size_t _get_block(const char *prfs_nm, const char *key, void *buf, size_t buf_len, size_t *saved_value_len)
{
    char *ez_key;
    size_t ret_v;

    ez_key = _init_ez_key(prfs_nm, key);
    if (NULL == ez_key)
    {
        return -1;
    }
    else
    {
        ret_v = ef_get_env_blob(ez_key, buf, buf_len, saved_value_len);
        _deinit_ez_key(ez_key);
        return ret_v;
    }
}

static EfErrCode _set_block(const char *prfs_nm, const char *key, const void *value, int32_t value_len)
{
    char *ez_key;
    EfErrCode ret_v;

    ez_key = _init_ez_key(prfs_nm, key);
    if (NULL == ez_key)
    {
        return EF_ENV_INIT_FAILED;
    }
    else
    {
        ret_v = ef_set_env_blob(ez_key, value, value_len);
        _deinit_ez_key(ez_key);
        return ret_v;
    }
}


//anytype
rt_err_t share_prefs_remove(share_prefs_t *prfs, const char *key)
{
    char *ez_key;
    rt_err_t ret_v;

    ez_key = _init_ez_key(prfs->prfs_name, key);
    if (NULL == ez_key)
    {
        return RT_ENOMEM;
    }
    else
    {
        EfErrCode err;
        err = ef_del_env(ez_key);
        _deinit_ez_key(ez_key);
        ret_v = (err == EF_NO_ERR) ? RT_EOK : err;
        return ret_v;
    }
}



//int
int32_t share_prefs_get_int(share_prefs_t *prfs, const char *key, int32_t default_v)
{
    int32_t ret_v;
    size_t sv_len, ret_len;

    ret_len = _get_block(prfs->prfs_name, key, &ret_v, sizeof(ret_v), &sv_len);

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

    ret_len = _get_block(prfs->prfs_name, key, buf, buf_len, &sv_len);

    if (buf_len < sv_len)
        return (buf_len - sv_len); //buf is too small, return a negative len
    else
        return ret_len;
}
rt_err_t share_prefs_set_block(share_prefs_t *prfs, const char *key, const void *buf, int32_t buf_len)
{
    EfErrCode ef_err;

    ef_err = _set_block(prfs->prfs_name, key, buf, buf_len);

    return (EF_NO_ERR == ef_err) ? RT_EOK : ef_err;

}
#endif

