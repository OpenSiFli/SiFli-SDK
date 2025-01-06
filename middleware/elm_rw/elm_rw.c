#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "bf0_hal.h"
#include "drv_flash.h"
#include <drv_log.h>
#include <dfs_fs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

#define MAX_BLOCK_LEN   (8*1024)
//uint8_t elm_data[MAX_BLOCK_LEN];
static int elm_fptr = -1;
static uint8_t *p_buf = RT_NULL;

#define CLOSE_ELM_FILE \
        if(elm_fptr != -1) \
        { \
            close(elm_fptr); \
            elm_fptr = -1; \
        }

#define FREE_ELM_BUF \
        if(p_buf != RT_NULL) \
        { \
            rt_free(p_buf); \
            p_buf = RT_NULL; \
        }

unsigned int getCrc(unsigned char *pSrc, int len, unsigned int crc)
{
    //unsigned int crc = 0xffffffff;

    for (int m = 0; m < len; m++)
    {
        crc ^= ((unsigned int)pSrc[m]) << 24;

        for (int n = 0; n < 8; n++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ 0x04c11db7;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static int elm_trans_in(char *file_path, char *file_size, char *crc_str)
{
    uint32_t len, off, size, crc1, crc2, start, end;
    int delta;
    int res;
    int cnt;
#if RT_USING_CONSOLE
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
#else
    rt_device_t pDev = rt_device_find("uart1");
#endif

    size = strtoul(file_size, 0, 16);
    crc1 = strtoul(crc_str, 0, 16);
    cnt = (size + MAX_BLOCK_LEN - 1) / MAX_BLOCK_LEN;
    crc2 = 0xffffffff;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    elm_fptr =  open(file_path, O_RDWR | O_TRUNC | O_CREAT | O_BINARY, 0);
    if (elm_fptr < 0)
    {
        rt_kprintf("open file err: %s\n", file_path);
        rt_kprintf("elm_trans_in FAIL\n");
        return RT_ERROR;
    }

    if (0 != ioctl(elm_fptr, F_RESERVE_CONT_SPACE, size))
    {
        CLOSE_ELM_FILE

        rt_kprintf("set continue space error:%d\n", size);
        rt_kprintf("elm_trans_in FAIL\n");

        return RT_ERROR;
    }

    p_buf = (uint8_t *)rt_malloc(MAX_BLOCK_LEN);
    if (p_buf == RT_NULL)
    {
        CLOSE_ELM_FILE

        rt_kprintf("p_buf malloc err: %d\n", MAX_BLOCK_LEN);
        rt_kprintf("elm_trans_in FAIL\n");

        return RT_ERROR;
    }

    for (int m = 0; m < cnt; m++)
    {
        //rt_device_write(pDev, 0, "elm_trans_in_waitrx\n", strlen("elm_trans_in_waitrx\n"));
        rt_kprintf("elm_trans_in_waitrx\n");
        off = 0;
        len = MAX_BLOCK_LEN;
        if ((m == cnt - 1) && (size % MAX_BLOCK_LEN))
        {
            len = size % MAX_BLOCK_LEN;
        }

        start = rt_tick_get();
        while (1)
        {
            delta = rt_device_read(pDev, off, &p_buf[off], len - off);
            off += delta;
            if (off >= len)
            {
                crc2 = getCrc(p_buf, len, crc2);
                res = write(elm_fptr, p_buf, len);
                if (res != len)
                {
                    CLOSE_ELM_FILE
                    FREE_ELM_BUF

                    rt_kprintf("write file err: %d != %d\n", res, len);
                    rt_kprintf("elm_trans_in FAIL\n");

                    return RT_ERROR;
                }
                break;
            }
            end = rt_tick_get();
            if (end - start > 2000)
            {
                CLOSE_ELM_FILE
                FREE_ELM_BUF

                rt_kprintf("rx data outof timer 2s\n");
                rt_kprintf("elm_trans_in FAIL\n");
                return RT_ERROR;
            }
        }
    }

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    if (crc2 == crc1)
    {
        rt_kprintf("crc2(0x%08x) == crc1(0x%08x)\n", crc2, crc1);
        rt_kprintf("elm_trans_in OK\n");
    }
    else
    {
        rt_kprintf("crc2(0x%08x) != crc1(0x%08x)\n", crc2, crc1);
        rt_kprintf("elm_trans_in FAIL\n");
    }

    return RT_EOK;
}

static int elm_trans_info(char *file_path)
{
    int res;
    struct stat file_stat;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    res = stat(file_path, &file_stat);
    if (0 != res)
    {
        rt_kprintf("stat error\n");
        rt_kprintf("elm_trans_info FAIL\n");
        return RT_ERROR;
    }
    rt_kprintf("elm_trans_len 0x%08x\n", file_stat.st_size);
    rt_kprintf("elm_trans_info OK\n");

    return 0;
}

static int elm_trans_out(char *file_path, char *file_size)
{
    int delta, res, cnt;
    int flag = 0;
    int match = 0;
    uint32_t len, off, size, crc;
    int cmdlen = strlen("elm_trans_out_waitrx");

#if RT_USING_CONSOLE
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
#else
    rt_device_t pDev = rt_device_find("uart1");
#endif

    size = strtoul(file_size, 0, 16);
    cnt = (size + MAX_BLOCK_LEN - 1) / MAX_BLOCK_LEN;
    crc = 0xffffffff;

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    elm_fptr = open(file_path, O_RDWR | O_BINARY, 0);
    if (elm_fptr < 0)
    {
        rt_kprintf("open error\n");
        rt_kprintf("elm_trans_out FAIL\n");
        return RT_ERROR;
    }

    p_buf = (uint8_t *)rt_malloc(MAX_BLOCK_LEN);
    if (p_buf == RT_NULL)
    {
        CLOSE_ELM_FILE

        rt_kprintf("p_buf malloc err: %d\n", MAX_BLOCK_LEN);
        rt_kprintf("elm_trans_out FAIL\n");

        return RT_ERROR;
    }

    rt_kprintf("elm_trans_out START\n");

    if (pDev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        pDev->open_flag &= ~RT_DEVICE_FLAG_STREAM;
        flag = 1;
    }

    off = 0;
    while (1)
    {
        delta = rt_device_read(pDev, off, &p_buf[off], cmdlen - off);
        off += delta;
        if (off >= cmdlen)
        {
            for (int m = 0; m < cmdlen / 2; m++)
            {
                if (memcmp(&p_buf[m], "elm_trans_out_waitrx", cmdlen - m) == 0)
                {
                    if (m != 0)
                    {
                        rt_device_read(pDev, off, &p_buf[off], m);
                    }
                    break;
                }
            }
            break;
            /*
            if(memcmp(p_buf, "elm_trans_out_waitrx", cmdlen)==0)
            {
                break;
            }
            else
            {
                rt_kprintf("HEX: %02X %02X %02X %02X %02X %02X %02X %02X\n", p_buf[0],p_buf[1],p_buf[2],p_buf[3],p_buf[4],p_buf[5],p_buf[6],p_buf[7]);
                CLOSE_ELM_FILE
                FREE_ELM_BUF
                rt_kprintf("not receive elm_trans_out_waitrx\n");
                rt_kprintf("elm_trans_out FAIL\n");
                return RT_ERROR;
            }*/
        }
    }

    for (int m = 0; m < cnt; m++)
    {

        len = MAX_BLOCK_LEN;
        off = 0;
        if ((m == cnt - 1) && (size % MAX_BLOCK_LEN))
        {
            len = size % MAX_BLOCK_LEN;
        }
#if 1
        res = read(elm_fptr, &p_buf[0], len);
        crc = getCrc(p_buf, len, crc);
        if (res != len)
        {
            CLOSE_ELM_FILE
            FREE_ELM_BUF
            rt_kprintf("read err: res(%d) != len(%d)\n", res, len);
            rt_kprintf("elm_trans_out FAIL\n");
            return RT_ERROR;
        }
        rt_device_write(pDev, 0, &p_buf[0], res);
#else
        while (1)
        {
            delta = rt_device_read(pDev, 0, &p_buf[off], cmdlen - off);
            off += delta;
            if (off >= cmdlen)
            {
                if (memcmp(p_buf, "elm_trans_out_waitrx", cmdlen) == 0)
                {
                    res = read(elm_fptr, &p_buf[0], len);
                    crc = getCrc(p_buf, len, crc);
                    if (res != len)
                    {
                        CLOSE_ELM_FILE
                        FREE_ELM_BUF
                        rt_kprintf("read err: res(%d) != len(%d)\n", res, len);
                        rt_kprintf("elm_trans_out FAIL\n");
                        return RT_ERROR;
                    }
                    rt_device_write(pDev, 0, &p_buf[0], res);
                    break;
                }
                else
                {
                    CLOSE_ELM_FILE
                    FREE_ELM_BUF
                    rt_kprintf("not receive elm_trans_out_waitrx\n");
                    rt_kprintf("elm_trans_out FAIL\n");
                    return RT_ERROR;
                }
            }
        }
#endif
    }

    if (flag == 1)
    {
        pDev->open_flag |= RT_DEVICE_FLAG_STREAM;
    }

    CLOSE_ELM_FILE
    FREE_ELM_BUF

    rt_kprintf("elm_trans_crc 0x%08x\n", crc);
    rt_kprintf("elm_trans_out OK\n");

    return 0;
}

static int elm_trans_test(int argc, char **argv)
{
    int res;
    struct stat file_stat;

    if (argc < 2)
    {
        rt_kprintf("para num %d < 2\n", argc);
        rt_kprintf("eg1: elm_trans_test 0 (check if support new trans api)\n");
        rt_kprintf("eg1: elm_trans_test 1 /test.bin (get test.bin file size)\n");
        rt_kprintf("eg1: elm_trans_test 2 /test.bin 0x20000 (trans test.bin file out)\n");
        rt_kprintf("eg1: elm_trans_test 3 /test.bin 0x20000 0x12345678(trans test.bin file in)\n");
        rt_kprintf("elm_trans_test FAIL\n");
        return RT_ERROR;
    }

    if (argv[1][0] == '0')
    {
        rt_kprintf("elm_trans_test NEW\n");
    }
    else if (argv[1][0] == '1')
    {
        if (argc != 3)
        {
            rt_kprintf("para num %d != 3\n", argc);
            rt_kprintf("elm_trans_info FAIL\n");
            return RT_ERROR;
        }
        return  elm_trans_info(argv[2]);
    }
    else if (argv[1][0] == '2')
    {
        if (argc != 4)
        {
            rt_kprintf("para num %d != 4\n", argc);
            rt_kprintf("elm_trans_out FAIL\n");
            return RT_ERROR;
        }
        return  elm_trans_out(argv[2], argv[3]);
    }
    else if (argv[1][0] == '3')
    {
        if (argc != 5)
        {
            rt_kprintf("para num %d != 5\n", argc);
            rt_kprintf("elm_trans_in FAIL\n");
            return RT_ERROR;
        }
        return  elm_trans_in(argv[2], argv[3], argv[4]);
    }

    return 0;
}
MSH_CMD_EXPORT(elm_trans_test, elm_trans_test 0 / 1 / 2 para...);


