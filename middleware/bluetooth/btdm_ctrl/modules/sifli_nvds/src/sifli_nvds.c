/**
 ****************************************************************************************
 *
 * @file sifli_nvds.c
 *
 * @brief SiFli NVDS Module Entry point
 *
 * Copyright (C) SiFli 2019-2022
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup NVDS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
*/


#include "rwip_config.h"
#ifdef NVDS_SUPPORT

#include <string.h>
#include "sifli_nvds.h"
#include "sifli_nvds_internal.h"
#include "ke_msg.h"
#include "co_utils.h"
#include "rwip.h"
#include "app.h"


#define NVDS_GET_CURRENT_SIZE() (((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff))->used_mem)
#define NVDS_SET_CURRENT_SIZE(size) ((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff))->used_mem = size
#define NVDS_ADD_CURRENT_SIZE(size) ((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff))->used_mem += size
#define NVDS_GET_VALUE_PTR(offset) ((uint8_t *)((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff)+1) + (offset))
#define NVDS_WRITTING_PROTECT_ENABLE() ((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff))->writting = 1
#define NVDS_WRITTING_PROTECT_DISABLE() ((sifli_nvds_mem_init_t *)(g_sifli_nvds_env.stack_buff))->writting = 0

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
*/


static uint8_t sifli_nvds_type_tag_check(sifli_nvds_type_t type, uint8_t tag);
static uint8_t sifli_nvds_add_tag(sifli_nvds_type_t type, uint8_t tag, uint8_t len, uint8_t *buf);

static uint8_t sifli_nvds_modify_tag(sifli_nvds_type_t type,
                                     sifli_nvds_tag_value_t *val,
                                     uint8_t len, uint8_t *buf);

static sifli_nvds_tag_value_t *sifli_nvds_walk_tag(sifli_nvds_type_t type, uint8_t tag);

static uint8_t sifli_nvds_del_item(sifli_nvds_type_t type, sifli_nvds_tag_value_t *val);

static uint8_t _sifli_nvds_get(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf);

static uint8_t _sifli_nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);

static uint8_t _sifli_nvds_lock(uint8_t tag);

static uint8_t _sifli_nvds_del(uint8_t tag);




static sifli_nvds_env_t g_sifli_nvds_env;


sifli_nvds_init_handler _sifli_nvds_init_handler = _sifli_nvds_init;
sifli_nvds_update_handler _sifli_nvds_update_handler = _sifli_nvds_update;
sifli_nvds_get_handler _sifli_nvds_get_handler = _sifli_nvds_get;
sifli_nvds_put_handler _sifli_nvds_put_handler = _sifli_nvds_put;
sifli_nvds_lock_handler _sifli_nvds_lock_handler = _sifli_nvds_lock;
sifli_nvds_del_handler _sifli_nvds_del_handler = _sifli_nvds_del;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

static void sifli_nvds_wakeup_lcpu(void)
{
#if defined(BF0_HCPU)
    bluetooth_wakeup_lcpu();
#endif
}

static void sifli_nvds_release_lcpu(void)
{
#if defined(BF0_HCPU)
    bluetooth_release_lcpu();
#endif
}


static uint8_t sifli_nvds_type_tag_check(sifli_nvds_type_t type, uint8_t tag)
{
    uint8_t check_ret = 0;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        if (tag >= PARAM_ID_BD_ADDRESS && tag < PARAM_ID_APP_SPECIFIC_FIRST)
        {
            check_ret = 1;
        }
        break;
    }
    default:
        break;
    }
    return check_ret;

}


static uint8_t sifli_nvds_mem_corruption_check(sifli_nvds_mem_init_t *buffer)
{
    uint16_t cur_len = buffer->used_mem;
    sifli_nvds_tag_value_t *val = (sifli_nvds_tag_value_t *)(buffer + 1);
    sifli_nvds_tag_value_t *cur;
    uint8_t ret = 0;
    uint16_t tag_len;
    while (cur_len)
    {

        if (sifli_nvds_type_tag_check(SIFLI_NVDS_TYPE_STACK, val->tag) == 0)
        {
            ret = 0x1;
            break;
        }

        tag_len = sizeof(sifli_nvds_tag_value_t) + (uint16_t)val->len;

        if (cur_len < tag_len || tag_len == 0)
        {
            ret = 0x2;
            break;
        }
        val = (sifli_nvds_tag_value_t *)((uint8_t *)val + tag_len);
        cur_len -= tag_len;
    };
    return ret;
}

//
static uint8_t sifli_nvds_buffer_init(sifli_nvds_env_t *env, uint8_t is_force_init)
{
    uint8_t reinit = is_force_init;
    // Should be 4-byte aligned
    if (((uint32_t)env->stack_buff & 0x3) != 0)
        return NVDS_CORRUPT;

    sifli_nvds_mem_init_t *init_buffer = (sifli_nvds_mem_init_t *)env->stack_buff;

    if (!reinit)
    {
        if (init_buffer->writting || init_buffer->used_mem > env->stack_total_len || init_buffer->pattern != NVDS_PATTERN)
            reinit = 1;
        else if (init_buffer->pattern == NVDS_PATTERN && init_buffer->used_mem != 0)
            if (sifli_nvds_mem_corruption_check(init_buffer) != 0)
                reinit = 1;
    }

    if (reinit)
    {
        memset(env->stack_buff, 0, env->stack_total_len);
        init_buffer->pattern = NVDS_PATTERN;
    }
    return NVDS_OK;
}


static uint8_t sifli_nvds_add_tag(sifli_nvds_type_t type, uint8_t tag, uint8_t len, uint8_t *buf)
{
    uint16_t curr_len, max_len;
    sifli_nvds_tag_value_t *val = NULL;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        curr_len = NVDS_GET_CURRENT_SIZE();
        val = (sifli_nvds_tag_value_t *)NVDS_GET_VALUE_PTR(curr_len);
        max_len = g_sifli_nvds_env.stack_total_len;
        break;
    }
    default:
        return NVDS_FAIL;
    }

    if (curr_len + len + sizeof(sifli_nvds_tag_value_t) > max_len)
    {
        return NVDS_NO_SPACE_AVAILABLE;
    }

    NVDS_WRITTING_PROTECT_ENABLE();

    val->tag = tag;
    val->len = len;
    memcpy(val->value, buf, len);

    NVDS_ADD_CURRENT_SIZE(len + sizeof(sifli_nvds_tag_value_t));

    NVDS_WRITTING_PROTECT_DISABLE();

    return NVDS_OK;
}

static uint8_t sifli_nvds_modify_tag(sifli_nvds_type_t type,
                                     sifli_nvds_tag_value_t *val,
                                     uint8_t len, uint8_t *buf)
{
    uint16_t curr_len, max_len;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        curr_len = NVDS_GET_CURRENT_SIZE();
        max_len = g_sifli_nvds_env.stack_total_len;
        break;
    }
    default:
        return NVDS_FAIL;
    }

    if (val->len == len)
    {
        NVDS_WRITTING_PROTECT_ENABLE();

        //No need modify the posistion
        memcpy(val->value, buf, len);

        NVDS_WRITTING_PROTECT_DISABLE();
    }
    else
    {
        if (len > val->len)
            if ((curr_len + len - val->len) > max_len)
                return NVDS_NO_SPACE_AVAILABLE;

        uint8_t tag = val->tag;
        sifli_nvds_del(tag);
        sifli_nvds_add_tag(type, tag, len, buf);
    }

    return NVDS_OK;

}


static sifli_nvds_tag_value_t *sifli_nvds_walk_tag(sifli_nvds_type_t type, uint8_t tag)
{
    uint16_t len, val_len;
    sifli_nvds_tag_value_t *val = NULL, *val1 = NULL;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        len = NVDS_GET_CURRENT_SIZE();
        val = (sifli_nvds_tag_value_t *)NVDS_GET_VALUE_PTR(0);
        break;
    }
    default:
        return val;
    }

    while (len)
    {
        if (val->tag == tag)
        {
            return val;
        }
        val_len = sizeof(sifli_nvds_tag_value_t) + val->len;
        len  -= val_len;
        val = (sifli_nvds_tag_value_t *)((uint8_t *)val + val_len);
    }
    return NULL;
}

static uint8_t sifli_nvds_del_item(sifli_nvds_type_t type, sifli_nvds_tag_value_t *val)
{
    uint16_t cur_len, max_len, left_len;
    uint8_t val_len = val->len;
    sifli_nvds_tag_value_t *begin_val;
    switch (type)
    {
    case SIFLI_NVDS_TYPE_STACK:
    {
        cur_len = NVDS_GET_CURRENT_SIZE();
        begin_val = (sifli_nvds_tag_value_t *)NVDS_GET_VALUE_PTR(0);
        max_len = g_sifli_nvds_env.stack_total_len;
        break;
    }
    default:
        return NVDS_TAG_NOT_DEFINED;
    }

    NVDS_WRITTING_PROTECT_ENABLE();

    sifli_nvds_tag_value_t *next_val = (sifli_nvds_tag_value_t *)((uint8_t *)val + sizeof(sifli_nvds_tag_value_t) + val->len);

    left_len = (uint8_t *)next_val - (uint8_t *)begin_val;
    if (left_len <= cur_len)
    {
        // Overwrite delete part
        left_len = cur_len - left_len;
        if (left_len != 0)
        {
            memcpy(val, next_val, left_len);
        }
    }


    cur_len -= val_len + sizeof(sifli_nvds_tag_value_t);
    memset((uint8_t *)begin_val + cur_len, 0, max_len - cur_len);

    NVDS_SET_CURRENT_SIZE(cur_len);

    NVDS_WRITTING_PROTECT_DISABLE();


    return NVDS_OK;
}



int32_t _sifli_nvds_update(uint8_t *buffer, uint16_t buf_len)
{
    if (g_sifli_nvds_env.state != SIFLI_NVDS_READY)
        return NVDS_FAIL;

    if (buf_len > (g_sifli_nvds_env.stack_total_len - sizeof(sifli_nvds_mem_init_t)))
        return NVDS_FAIL;

    sifli_nvds_wakeup_lcpu();

    // Just overwrite all memory
    memset((void *)g_sifli_nvds_env.stack_buff, 0, g_sifli_nvds_env.stack_total_len);
    uint8_t ret = sifli_nvds_buffer_init(&g_sifli_nvds_env, 0);
    if (ret == NVDS_OK)
    {
        sifli_nvds_mem_init_t *mem_ctrl = (sifli_nvds_mem_init_t *)g_sifli_nvds_env.stack_buff;
        mem_ctrl->writting = 1;
        memcpy((uint8_t *)(mem_ctrl + 1), buffer, buf_len);
        mem_ctrl->used_mem = buf_len;
        mem_ctrl->writting = 0;
    }

    sifli_nvds_release_lcpu();
    return ret;
}



/* NVDS only for controller to maintain RAM data.
*  Host shal update all related infomations and controller should send
*  modified info to host via HCI.
*/
int32_t _sifli_nvds_init(uint8_t *buffer, uint16_t buf_len)
{
    g_sifli_nvds_env.stack_buff = buffer;
    g_sifli_nvds_env.stack_total_len = buf_len;

    sifli_nvds_wakeup_lcpu();

    uint8_t ret = sifli_nvds_buffer_init(&g_sifli_nvds_env, 0);
    if (ret == NVDS_OK)
        g_sifli_nvds_env.state = SIFLI_NVDS_READY;

    sifli_nvds_release_lcpu();
    return ret;
}


extern size_t sifli_nvds_flash_read(const char *key, void *value_buf, size_t buf_len);
extern uint8_t sifli_nvds_flash_write(const char *key, const void *value_buf, size_t buf_len);

static uint8_t *sifli_app_nvds_read_int(uint16_t *len)
{
    uint8_t *ptr = NULL;
    size_t read_len = 0;

    ptr = rt_malloc(SIFLI_NVDS_KEY_LEN_APP);

    if (ptr)
    {
        memset(ptr, 0, SIFLI_NVDS_KEY_LEN_APP);
        read_len = sifli_nvds_flash_read(SIFLI_NVDS_KEY_APP, ptr, SIFLI_NVDS_KEY_LEN_APP);
        // Just assume SIFLI_NVDS_KEY_LEN_APP is the maximum len
        if (read_len > SIFLI_NVDS_KEY_LEN_APP)
        {
            rt_kprintf("app nvds readlen err\n");
            read_len = SIFLI_NVDS_KEY_LEN_APP;
        }
    }
    *len = (uint16_t)read_len;

    return ptr;
}


static sifli_nvds_tag_value_t *sifli_nvds_get_tag_value_via_buffer(uint8_t *buffer, uint16_t buffer_len, uint8_t tag)
{
    uint16_t len = buffer_len, val_len;
    sifli_nvds_tag_value_t *val = NULL, *val1 = NULL;

    val = (sifli_nvds_tag_value_t *)buffer;
    while (len && val->len)
    {
        if (val->tag == tag)
        {
            return val;
        }
        val_len = sizeof(sifli_nvds_tag_value_t) + val->len;
        if (len >= val_len)
            len  -= val_len;
        else
            break;
        val = (sifli_nvds_tag_value_t *)((uint8_t *)val + val_len);
    }
    return NULL;
}


static uint8_t sifli_app_nvds_modify_tag(uint8_t *buffer_pool, uint16_t *buffer_pool_len, uint16_t max_len,
        sifli_nvds_tag_value_t *old_tag, sifli_nvds_tag_value_t *new_tag)
{
    uint8_t is_modified = 0;
    if (old_tag->len != new_tag->len)
    {
        /* 1. Check whether could added new tag. */
        if ((new_tag->len > old_tag->len) &&
                (new_tag->len - old_tag->len + *buffer_pool_len > max_len))
            return is_modified;
        /* 2. Remove old_tag. */
        uint16_t old_tag_len = old_tag->len + sizeof(sifli_nvds_tag_value_t);
        sifli_nvds_tag_value_t *next_tag = (sifli_nvds_tag_value_t *)((uint8_t *)old_tag + old_tag->len);
        if (next_tag->len != 0)
            memcpy((uint8_t *)old_tag, (uint8_t *)next_tag, *buffer_pool_len - old_tag_len);
        else
            memset((uint8_t *)old_tag, 0, old_tag_len);
        /* 3. Add new tag. */
        uint16_t new_tag_len = new_tag->len + sizeof(sifli_nvds_tag_value_t);
        memcpy((uint8_t *)buffer_pool + *buffer_pool_len - old_tag_len, new_tag, new_tag_len);
        *buffer_pool_len = *buffer_pool_len - old_tag_len + new_tag_len;
        is_modified = 1;
    }
    else if (memcmp(old_tag->value, new_tag->value, old_tag->len) != 0)
    {
        memcpy(old_tag->value, new_tag->value, old_tag->len);
        is_modified = 1;
    }
    return is_modified;
}


static uint8_t sifli_app_nvds_add_tag(uint8_t *buffer_pool, uint16_t *buffer_pool_len, uint16_t max_len,
                                      sifli_nvds_tag_value_t *new_tag)
{
    uint8_t is_added = 0;
    uint16_t new_tag_len = sizeof(sifli_nvds_tag_value_t) + new_tag->len;
    if (*buffer_pool_len + new_tag_len > max_len)
        return is_added;
    memcpy((uint8_t *)buffer_pool + *buffer_pool_len, new_tag, new_tag_len);
    is_added = 1;
    *buffer_pool_len = *buffer_pool_len + new_tag_len;

    return is_added;
}


static uint8_t _sifli_nvds_get(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf)
{
    sifli_nvds_tag_value_t *val = NULL;
    uint8_t ret = NVDS_TAG_NOT_DEFINED;
    sifli_nvds_type_t type = 0;

    if (tag == PARAM_ID_LE_PUBLIC_KEY_P256)
    {
        uint8_t *buf_ptr;
        uint16_t len;
        buf_ptr = sifli_app_nvds_read_int(&len);
        if (buf_ptr)
        {
            sifli_nvds_tag_value_t *search_tag = sifli_nvds_get_tag_value_via_buffer(buf_ptr, len, PARAM_ID_LE_PUBLIC_KEY_P256);
            if (search_tag)
            {
                *lengthPtr = search_tag->len;
                memcpy(buf, search_tag->value, search_tag->len);
                ret = NVDS_OK;
            }
            else
            {
                ret = NVDS_FAIL;
            }
            rt_kprintf("le p256 nvds get %d\n", ret);
            rt_free(buf_ptr);
        }
        else
        {
            rt_kprintf("app nvds read fail\n");
            ret = NVDS_FAIL;
        }

        return ret;
    }
    else if (tag == NVDS_TAG_BD_ADDRESS)
    {
        uint8_t *buf_ptr;
        uint16_t len;
        buf_ptr = sifli_app_nvds_read_int(&len);
        if (buf_ptr)
        {
            sifli_nvds_tag_value_t *search_tag = sifli_nvds_get_tag_value_via_buffer(buf_ptr, len, NVDS_TAG_BD_ADDRESS);
            if (search_tag)
            {
                *lengthPtr = search_tag->len;
                memcpy(buf, search_tag->value, search_tag->len);
                ret = NVDS_OK;
            }
            else
            {
                ret = NVDS_FAIL;
            }
            rt_kprintf("app addr nvds get %d\n", ret);
            rt_free(buf_ptr);
        }
        else
        {
            rt_kprintf("app nvds read addr fail\n");
            ret = NVDS_FAIL;
        }

        return ret;
    }

    sifli_nvds_wakeup_lcpu();

    do
    {
        if (g_sifli_nvds_env.state != SIFLI_NVDS_READY)
        {
            ret = NVDS_FAIL;
            break;
        }
        if (sifli_nvds_type_tag_check(SIFLI_NVDS_TYPE_STACK, tag))
        {
            type = SIFLI_NVDS_TYPE_STACK;
        }
        else
            break;

        if ((val = sifli_nvds_walk_tag(type, tag)) != NULL)
        {
            *lengthPtr = val->len;
            memcpy(buf, val->value, val->len);
            ret = NVDS_OK;
        }
    }
    while (0);

    sifli_nvds_release_lcpu();

    return ret;
}



static uint8_t _sifli_nvds_del(uint8_t tag)
{
    sifli_nvds_tag_value_t *val = NULL;
    uint8_t ret = NVDS_TAG_NOT_DEFINED;
    sifli_nvds_type_t type = 0;

    sifli_nvds_wakeup_lcpu();

    do
    {
        if (g_sifli_nvds_env.state != SIFLI_NVDS_READY)
        {
            ret = NVDS_FAIL;
            break;
        }

        if (sifli_nvds_type_tag_check(SIFLI_NVDS_TYPE_STACK, tag))
        {
            type = SIFLI_NVDS_TYPE_STACK;
        }
        else
            break;

        if ((val = sifli_nvds_walk_tag(type, tag)) != NULL)
        {
            ret = sifli_nvds_del_item(type, val);
        }

    }
    while (0);

    sifli_nvds_release_lcpu();

    return ret;

}

static uint8_t _sifli_nvds_lock(uint8_t tag)
{
    return NVDS_FAIL;
}


static uint8_t _sifli_nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
    sifli_nvds_tag_value_t *val = NULL;
    uint8_t ret = NVDS_TAG_NOT_DEFINED;
    sifli_nvds_type_t type = 0;

    if (tag == PARAM_ID_LE_PUBLIC_KEY_P256)
    {
        uint8_t *nv_buf;
        uint16_t len;
        nv_buf = sifli_app_nvds_read_int(&len);
        if (nv_buf)
        {
            sifli_nvds_tag_value_t *search_tag;
            sifli_nvds_tag_value_t *write_tag = rt_malloc(sizeof(sifli_nvds_tag_value_t) + PARAM_LEN_PUBLIC_KEY_P256);
            write_tag->tag = PARAM_ID_LE_PUBLIC_KEY_P256;
            write_tag->len = PARAM_LEN_PUBLIC_KEY_P256;
            memcpy(write_tag->value, buf, length);


            search_tag = sifli_nvds_get_tag_value_via_buffer(nv_buf, len, write_tag->tag);
            if (search_tag)
            {
                /* Only update the tag for SIFLI_NVDS_UPDATE_ALWAYS. */
                {
                    sifli_app_nvds_modify_tag(nv_buf, &len, SIFLI_NVDS_KEY_LEN_APP, search_tag, write_tag);
                }
                ret = NVDS_OK;
            }
            else
            {
                sifli_app_nvds_add_tag(nv_buf, &len, SIFLI_NVDS_KEY_LEN_APP, write_tag);
            }
            //ret = sifli_nvds_write(SIFLI_NVDS_TYPE_APP, len, buf);
            ret = sifli_nvds_flash_write(SIFLI_NVDS_KEY_APP, nv_buf, len);
            if (ret != NVDS_OK)
                rt_kprintf("app nvds write failed\n");

            rt_free(write_tag);
            rt_free(nv_buf);
        }
        else
        {
            rt_kprintf("app nvds read fail");
            ret = NVDS_FAIL;
        }
        return ret;
    }

    sifli_nvds_wakeup_lcpu();

    do
    {
        if (g_sifli_nvds_env.state != SIFLI_NVDS_READY)
        {
            ret = NVDS_FAIL;
            break;
        }

        if (sifli_nvds_type_tag_check(SIFLI_NVDS_TYPE_STACK, tag))
        {
            type = SIFLI_NVDS_TYPE_STACK;

        }
        else
            break;

        if ((val = sifli_nvds_walk_tag(type, tag)) == NULL)
        {
            ret = sifli_nvds_add_tag(type, tag, length, buf);
        }
        else
        {
            ret = sifli_nvds_modify_tag(type, val, length, buf);
        }
    }
    while (0);

    sifli_nvds_release_lcpu();

    return ret;

}


uint8_t sifli_nvds_get(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf)
{
    return _sifli_nvds_get_handler(tag, lengthPtr, buf);
}


uint8_t sifli_nvds_del(uint8_t tag)
{
    return _sifli_nvds_del_handler(tag);
}


uint8_t sifli_nvds_lock(uint8_t tag)
{
    return _sifli_nvds_lock_handler(tag);
}


uint8_t sifli_nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
    return _sifli_nvds_put_handler(tag, length, buf);
}

#endif

