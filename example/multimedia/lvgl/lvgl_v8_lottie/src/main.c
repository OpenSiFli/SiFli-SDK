/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lvsf_rlottie.h"
#include "dfs_posix.h"

#define LOTTIE_WIDTH    200
#define LOTTIE_HEIGHT   200



/* User code start from here --------------------------------------------------------*/

void lottie_setup(lv_obj_t *parent, const char *fname);

#if !defined(BSP_USING_PC_SIMULATOR) && defined(RT_USING_DFS)

#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#else
    #define FS_ROOT "root"
#endif

extern void register_mtd_device(uint32_t address, uint32_t size, char *name);
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
        if (dfs_mkfs("elm", FS_ROOT) == 0)
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
#endif

const char *get_file_ext(const char *name)
{
    const char *dot = NULL;

    if (name)
    {
        dot = &name[strlen(name) - 1];
        while (*dot != '.' && dot > name && *dot != '\\' && *dot != '/')
            dot--;
        if (*dot == '.')
            dot = dot + 1;
        else
            dot = NULL;
    }
    return dot;
}

static char *next_lottie_file()
{
    DIR *dir;
    struct dirent *ent;
    static char lottie_file[128];
    static int total_cnt;
    static int next_cnt;
    int i = 0;


    dir = opendir("/");
    if (dir == NULL)
        rt_kprintf("None lottie file found\n");
    else
    {
        if (total_cnt == 0)
        {
            while ((ent = readdir(dir)) != NULL)
            {
                if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
                    continue;
                if (strcmp(get_file_ext(ent->d_name), "json") == 0)
                    total_cnt++;
            }
            rt_kprintf("Total_cnt:%d\n", total_cnt);
            closedir(dir);
            dir = opendir("/");
        }
        while ((ent = readdir(dir)) != NULL && total_cnt)
        {
            if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
                continue;
            if (strcmp(get_file_ext(ent->d_name), "json") != 0)
            {
                continue;
            }
            if (i == next_cnt)
            {
                strcpy(lottie_file, ent->d_name);
                next_cnt++;
                next_cnt %= total_cnt;
                break;
            }
            i++;
        }
        closedir(dir);
    }
    rt_kprintf("%s\n", lottie_file);
    return &(lottie_file[0]);
}

static void event_cb(lv_event_t *e)
{
    //LV_LOG_USER("Clicked");

    lv_obj_t *lottie = lv_event_get_target(e);

    lv_rlottie_play(lottie, 0);
    lv_obj_del(lottie);
    lottie_setup(lv_scr_act(), NULL);
}

void lottie_setup(lv_obj_t *parent, const char *fname)
{
    lv_obj_t *lottie = lv_rlottie_create(parent);
    lv_obj_set_size(lottie, LOTTIE_WIDTH, LOTTIE_HEIGHT);
    lv_obj_center(lottie);
    if (fname)
        lv_rlottie_file(lottie, fname);
    else
        lv_rlottie_file(lottie, next_lottie_file());
    lv_rlottie_play(lottie, 1);
    lv_obj_add_event_cb(lottie, event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(lottie, LV_OBJ_FLAG_CLICKABLE);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();
    lottie_setup(lv_scr_act(), NULL);

    while (1)
    {
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
