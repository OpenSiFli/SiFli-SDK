/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"

#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "drv_flash.h"
#include "mem_section.h"

typedef enum
{
    IMG_SRC_INVALID = 0,
    IMG_SRC_DSC,
    IMG_SRC_FILE,
} img_src_type_t;

typedef struct img_src_info
{
    img_src_type_t type;
    union
    {
        lv_img_dsc_t *img_dsc;
        char *file_path;
    } src;
    int use_psram;  // 0: sram, 1: psram
    int updated;
} img_src_info_t;

L2_RET_BSS_SECT_BEGIN(psram_heap_pool)
static uint8_t psram_heap_pool[4096 * 1024] L2_RET_BSS_SECT(psram_heap_pool);
L2_RET_BSS_SECT_END
static struct rt_memheap psram_memheap;
static img_src_info_t g_img_src_info;

/**
 * @brief Initialize PSRAM heap memory pool
 *
 * @return int 0 on success
 */
int psram_heap_init(void)
{
    rt_memheap_init(&psram_memheap, "psram_heap", (void *)psram_heap_pool,
                    sizeof(psram_heap_pool));
    return 0;
}

/**
 * @brief Allocate memory from PSRAM heap
 *
 * @param size Size of memory to allocate in bytes
 * @return void* Pointer to allocated memory, NULL if allocation fails
 */
void *psram_heap_malloc(uint32_t size)
{
    return rt_memheap_alloc(&psram_memheap, size);
}

/**
 * @brief Free memory allocated from PSRAM heap
 *
 * @param p Pointer to memory to free
 */
void psram_heap_free(void *p)
{
    rt_memheap_free(p);
}

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

/**
 * @brief Initialize SD card and mount file system
 *
 * Finds the SD card device, creates mount point, and mounts the file system to /sdcard
 */
void sdcard_init(void)
{
    rt_device_t msd = rt_device_find("sd0");
    if (msd == NULL)
    {
        rt_kprintf("sd card not found\n");
        return;
    }
    mkdir("/sdcard", 0777);
    if (dfs_mount("sd0", "/sdcard", "elm", 0, 0) != 0) // fs exist
    {
        rt_kprintf("mount fs on tf card to /sdcard fail\n");
        rt_kprintf("sd card might not be formatted or is corrupted.\n");
        return;
    }

    rt_kprintf("mount fs on tf card to /sdcard success\n");
}

/**
 * @brief Load ezip image file into memory
 *
 * @param img_dsc Pointer to LVGL image descriptor structure
 * @param ezip_path Path to the ezip file
 * @param use_psram 1 to allocate memory from PSRAM, 0 to allocate from SRAM
 * @return int 0 on success, -1 on failure
 */
int load_ezip(lv_img_dsc_t *img_dsc, const char *ezip_path, int use_psram)
{
    if (img_dsc == NULL || ezip_path == NULL)
    {
        return -1;
    }
    FILE *fp = fopen(ezip_path, "rb");
    if (fp == NULL)
    {
        rt_kprintf("Failed to open ezip file: %s\n", ezip_path);
        return -1;
    }

    fread(&img_dsc->header, sizeof(lv_img_header_t), 1, fp);
    fseek(fp, 0, SEEK_END);
    size_t file_size = ftell(fp);
    fseek(fp, sizeof(lv_img_header_t), SEEK_SET);
    img_dsc->data_size = file_size - sizeof(lv_img_header_t);

    if (use_psram)
    {
        img_dsc->data = (const uint8_t *)psram_heap_malloc(img_dsc->data_size);
    }
    else
    {
        img_dsc->data = (const uint8_t *)malloc(img_dsc->data_size);
    }

    if (img_dsc->data == NULL)
    {
        rt_kprintf("Failed to allocate memory for ezip file%s\n", use_psram ? " in psram" : "");
        fclose(fp);
        return -1;
    }

    size_t read_size = fread((void *)img_dsc->data, 1, img_dsc->data_size, fp);
    if (read_size != img_dsc->data_size)
    {
        rt_kprintf("Failed to read ezip file: %s\n", ezip_path);
        if (use_psram)
        {
            psram_heap_free((void *)img_dsc->data);
        }
        else
        {
            free((void *)img_dsc->data);
        }
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

/**
 * @brief Unload ezip image and free allocated memory
 *
 * @param img_dsc Pointer to LVGL image descriptor structure
 * @param use_psram 1 if memory was allocated from PSRAM, 0 if from SRAM
 */
void unload_ezip(lv_img_dsc_t *img_dsc, int use_psram)
{
    if (img_dsc == NULL || img_dsc->data == NULL)
    {
        return;
    }

    if (use_psram)
    {
        psram_heap_free((void *)img_dsc->data);
    }
    else
    {
        free((void *)img_dsc->data);
    }

    img_dsc->data = NULL;
    img_dsc->data_size = 0;
}

/**
 * @brief Set image source via console command
 *
 * This function allows setting the image source through finsh command line.
 * It supports loading images either as file references or as memory descriptors,
 * and allows choosing between SRAM and PSRAM for memory allocation.
 *
 * @param argc Argument count
 * @param argv Argument vector
 *             argv[1]: Image file path
 *             argv[2]: Load type ("file" or "dsc")
 *             argv[3]: Memory type ("sram" or "psram")
 * @return int 0 on success, -1 on failure
 */
static int set_image(int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        rt_kprintf("Usage: set_image [path] [file/dsc] [sram/psram]\n");
        rt_kprintf("  path: image file path\n");
        rt_kprintf("  file/dsc: load as file or descriptor\n");
        rt_kprintf("  sram/psram: (optional) load to sram or psram, default is sram\n");
        rt_kprintf("  Note: sram/psram parameter is only used when load type is 'dsc'\n");
        return -1;
    }

    const char *path = argv[1];
    const char *load_type = argv[2];
    const char *mem_type = (argc == 4) ? argv[3] : "sram";

    // Clean up old image resources
    if (g_img_src_info.type == IMG_SRC_DSC && g_img_src_info.src.img_dsc != NULL)
    {
        unload_ezip(g_img_src_info.src.img_dsc, g_img_src_info.use_psram);
        free(g_img_src_info.src.img_dsc);
        g_img_src_info.src.img_dsc = NULL;
    }
    else if (g_img_src_info.type == IMG_SRC_FILE && g_img_src_info.src.file_path != NULL)
    {
        free(g_img_src_info.src.file_path);
        g_img_src_info.src.file_path = NULL;
    }

    // Determine memory type
    int use_psram = 0;
    if (strcmp(mem_type, "psram") == 0)
    {
        use_psram = 1;
    }
    else if (strcmp(mem_type, "sram") == 0)
    {
        use_psram = 0;
    }
    else
    {
        rt_kprintf("Invalid memory type: %s, use 'sram' or 'psram'\n", mem_type);
        return -1;
    }

    // Determine load type
    if (strcmp(load_type, "file") == 0)
    {
        // Load as file reference
        g_img_src_info.type = IMG_SRC_FILE;
        g_img_src_info.src.file_path = (char *)malloc(strlen(path) + 1);
        if (g_img_src_info.src.file_path == NULL)
        {
            rt_kprintf("Failed to allocate memory for file path\n");
            return -1;
        }
        strcpy(g_img_src_info.src.file_path, path);
        g_img_src_info.use_psram = use_psram;
        g_img_src_info.updated = 1;
        rt_kprintf("Set image source to file: %s\n", path);
    }
    else if (strcmp(load_type, "dsc") == 0)
    {
        // Load as descriptor into memory
        lv_img_dsc_t *img_dsc = (lv_img_dsc_t *)malloc(sizeof(lv_img_dsc_t));
        if (img_dsc == NULL)
        {
            rt_kprintf("Failed to allocate memory for image descriptor\n");
            return -1;
        }

        if (load_ezip(img_dsc, path, use_psram) != 0)
        {
            rt_kprintf("Failed to load ezip image from: %s\n", path);
            free(img_dsc);
            return -1;
        }

        g_img_src_info.type = IMG_SRC_DSC;
        g_img_src_info.src.img_dsc = img_dsc;
        g_img_src_info.use_psram = use_psram;
        g_img_src_info.updated = 1;
        rt_kprintf("Set image source to descriptor: %s (loaded to %s)\n", path, use_psram ? "psram" : "sram");
    }
    else
    {
        rt_kprintf("Invalid load type: %s, use 'file' or 'dsc'\n", load_type);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(set_image, Set image source: set_image [path] [file / dsc] [sram / psram]);

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("dynamic ezip loading example.\n");
    /* init sd card and mount fs */
    sdcard_init();
    /* init psram heap */
    psram_heap_init();
    /* init littlevGL */
    rt_err_t ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        rt_kprintf("littlevgl2rtt_init failed! error code: %d\n", ret);
        return ret;
    }
    lv_ex_data_pool_init();

    /* Create an image object */
    lv_obj_t *img = lv_img_create(lv_scr_act());
    /* Set initial image source for LVGL V8 */
    lv_img_set_src(img, "example.ezip");
    /* Align the image to the center of the screen */
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    /* Infinite loop */
    while (1)
    {
        /* Update image source if changed */
        if (g_img_src_info.updated)
        {
            if (g_img_src_info.type == IMG_SRC_DSC)
            {
                lv_img_set_src(img, g_img_src_info.src.img_dsc);
            }
            else if (g_img_src_info.type == IMG_SRC_FILE)
            {
                lv_img_set_src(img, g_img_src_info.src.file_path);
            }
            g_img_src_info.updated = 0;
        }
        /* Handle LittlevGL tasks */
        uint32_t ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return 0;
}

