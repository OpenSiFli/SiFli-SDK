#include "share_prefs.h"

#ifdef FINSH_USING_MSH
#include <finsh.h>
#if defined(PKG_EASYFLASH_ENV) || defined(PKG_USING_FLASHDB)


static share_prefs_t *pref;
static rt_err_t share_prefs(int argc, char **argv)
{
    char cmd_buf[32];
    rt_err_t res = RT_EOK;

    if (argc < 2)
    {
        rt_kprintf("share_prefs useage:\n");
        rt_kprintf("\t-o <pref name>\n");
        rt_kprintf("\t-c \n");
        rt_kprintf("\t-d <key>\n");
        rt_kprintf("\t-wb <key> <bool value>\n");
        rt_kprintf("\t-rb <key>\n");
        rt_kprintf("\t-wi <key> <integer value>\n");
        rt_kprintf("\t-ri <key>\n");
        rt_kprintf("\t-ws <key> <string value>\n");
        rt_kprintf("\t-rs <key>\n");
        return -RT_ERROR;
    }

    if (strcmp(argv[1], "-o") == 0)
    {
        pref = share_prefs_open(argv[2], SHAREPREFS_MODE_PRIVATE);
    }
    else if (strcmp(argv[1], "-c") == 0)
    {
        res = share_prefs_close(pref);
        pref = NULL;
    }
    else if (strcmp(argv[1], "-d") == 0)
    {
        res = share_prefs_remove(pref, argv[2]);
    }
    else if (strcmp(argv[1], "-rs") == 0)
    {
        int32_t rd_len;

        rd_len = share_prefs_get_string(pref, argv[2], cmd_buf, sizeof(cmd_buf) - 1);

        if (rd_len < 0)
        {
            rt_kprintf("rs error %d\n", rd_len);
            res = RT_ERROR;
        }
        else if (0 == rd_len)
        {
            rt_kprintf("NOT found\n");
        }
        else
        {
            cmd_buf[rd_len] = '\0';
            rt_kprintf("%s\n", cmd_buf);
        }
    }
    else if (strcmp(argv[1], "-ws") == 0)
    {
        res = share_prefs_set_string(pref, argv[2], argv[3]);
    }
    else if (strcmp(argv[1], "-ri") == 0)
    {
        int32_t v = share_prefs_get_int(pref, argv[2], 12345678);
        rt_kprintf("%d\n", v);
    }
    else if (strcmp(argv[1], "-wi") == 0)
    {
        int32_t v = atoi(argv[3]);
        res = share_prefs_set_int(pref, argv[2], v);
    }

    if (res != RT_EOK)
        rt_kprintf("ERROR %d\n", res);
    else
        rt_kprintf("OK\n");
    return res;
}
FINSH_FUNCTION_EXPORT(share_prefs, share preferences);
MSH_CMD_EXPORT(share_prefs, share preferences);
#endif
#endif /*PKG_USING_FLASHDB || PKG_EASYFLASH_ENV*/

