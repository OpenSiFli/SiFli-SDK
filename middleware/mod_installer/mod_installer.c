/**
  ******************************************************************************
  * @file   mod_installer.c
  * @author Sifli software development team
  * @brief Sibles source of wrapper device for ipc queue
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include "mod_installer.h"
#include "dlmodule.h"
#include "dlfcn.h"
#include "dfs_posix.h"
#include "board.h"

#define SYS_INFO_FILE  "sys_info.cfg"
/** file is saved in contiguous space  */
#if defined(SOLUTION_RES_USING_NAND)
    #define CONT_FILE_SPACE   0
#else
    #define CONT_FILE_SPACE   1
#endif
const char *const res_mod_name_suffix = "_res";

static struct rt_dlmodule *res_module;

static rt_uint32_t find_resource_symbol(const char *sym_str)
{
    rt_uint32_t addr = 0;

    if (res_module)
    {
        addr = (rt_uint32_t)dlsym(res_module, sym_str);
    }
    return addr;
}


static const char *make_app_res_mod_name(const char *app_name)
{
    char *res_mod_name = NULL;
    size_t res_mod_name_len;

    RT_ASSERT(app_name);

    res_mod_name_len = strlen(app_name);
    res_mod_name_len += strlen(res_mod_name_suffix);
    res_mod_name_len += 1; /* null-terminator */

    res_mod_name = rt_malloc(res_mod_name_len);

    if (!res_mod_name)
    {
        return res_mod_name;
    }
    res_mod_name[0] = 0;
    strcat(res_mod_name, app_name);
    strcat(res_mod_name, res_mod_name_suffix);

    return res_mod_name;
}

rt_err_t app_install(app_installer_info_t *info)
{
    const char *res_mod_name = NULL;
    rt_err_t res;

    if (info->res_package_path)
    {
        res_mod_name = make_app_res_mod_name(info->app_name);
        if (!res_mod_name)
        {
            res = -1;
            goto __EXIT;
        }
        /* install resource module first */
        res = mod_install(res_mod_name, info->res_package_path, info->res_install_path);
        if (RT_EOK != res)
        {
            rt_kprintf("fail to install resource\n");
            goto __EXIT;
        }

        res_module = dlrun(res_mod_name, info->res_install_path);
        RT_ASSERT(res_module);
        RT_ASSERT(RT_EOK == dlmodule_register_ex_symbol_resolver(find_resource_symbol));
    }

    res = mod_install(info->app_name, info->pgm_package_path, info->pgm_install_path);

    if (res_module)
    {
        dlmodule_unregister_ex_symbol_resolver();
        dlclose(res_module);
        res_module = NULL;
    }

    if (RT_EOK != res)
    {
        /* delete temp files if install fails */
        app_uninstall(info->app_name, info->res_install_path, info->pgm_install_path);
    }

__EXIT:
    if (res_mod_name)
    {
        rt_free((void *)res_mod_name);
    }

    return res;
}

int32_t app_uninstall(const char *app_name, const char *res_install_path, const char *pgm_install_path)
{
    const char *res_mod_name = NULL;

    int32_t res = 0;

    if (res_install_path)
    {
        res_mod_name = make_app_res_mod_name(app_name);
        if (!res_mod_name)
        {
            res = -1;
            goto __EXIT;
        }

        /* uninstall resource module first */
        res = dlmodule_uninstall(res_mod_name, res_install_path);
        if (0 != res)
        {
            rt_kprintf("fail to uninstall resource\n");
        }
    }

    /* uninstall app */
    res += dlmodule_uninstall(app_name, pgm_install_path);

    if (0 != res)
    {
        rt_kprintf("fail to uninstall app\n");
        goto __EXIT;
    }
__EXIT:
    if (res_mod_name)
    {
        rt_free((void *)res_mod_name);
    }

    return res;
}

rt_err_t mod_install(const char *mod_name, const char *package_path,
                     const char *install_path)
{
    int fid;
    off_t length;
    int res;
    uint8_t *module_ptr;

    fid = -1;
    module_ptr = NULL;
    if (!mod_name)
    {
        res = -RT_EINVAL;
        goto __EXIT;
    }

    if (!package_path)
    {
        package_path = "/installer";
    }

    if (!install_path)
    {
        install_path = "/app";
    }

    fid = open(package_path, O_RDONLY);

    if (fid < 0)
    {
        rt_kprintf("fail to open %s\n", package_path);
        res = -RT_EIO;
        goto __EXIT;
    }

#if CONT_FILE_SPACE
    if (0 != ioctl(fid, F_GET_PHY_ADDR, &module_ptr))
    {
        module_ptr = NULL;
        res = -RT_EIO;
        goto __EXIT;
    }

#else
    length = lseek(fid, 0, SEEK_END);
    lseek(fid, 0, SEEK_SET);

    if (length == 0)
    {
        rt_kprintf("file is empty\n");
        res = -RT_EEMPTY;
        goto __EXIT;
    }

    module_ptr = (uint8_t *)rt_malloc(length);
    if (!module_ptr)
    {
        res = -RT_EEMPTY;
        goto __EXIT;
    }

    if (read(fid, module_ptr, length) != length)
    {
        rt_kprintf("file read len error\n");
        res = -RT_EIO;
        goto __EXIT;
    }
#endif

    res = dlmodule_install(mod_name, module_ptr, install_path);

__EXIT:
#if !CONT_FILE_SPACE
    if (module_ptr)
    {
        rt_free(module_ptr);
    }
#endif

    if (fid >= 0)
    {
        close(fid);
    }

    return res;
}


int32_t app_auto_install(void)
{
    DIR *dir;
    struct dirent *dir_entry;
    struct stat *ent_stat;
    char *full_path;
    const char *path = "ex/installer";
    uint32_t name_len;
    uint32_t full_path_len;
    int32_t ret = 0;
    app_installer_info_t info;
    char *app_name;
    char *res_package_name;
    int32_t install_res;
    int res_fd;

    dir = opendir(path);
    if (!dir)
    {
        ret = -1;
        goto __EXIT;

    }

    info.res_install_path = "ex/resource";
    info.pgm_install_path = "watchface";

    ent_stat = rt_malloc(sizeof(*ent_stat));
    RT_ASSERT(stat);
    do
    {
        dir_entry = readdir(dir);
        if (!dir_entry)
        {
            break;
        }

        memset(ent_stat, 0, sizeof(*ent_stat));

        /* build full path for each file */
        full_path = dfs_normalize_path(path, dir_entry->d_name);
        if (full_path == NULL)
        {
            break;
        }
        full_path_len = strlen(full_path);
        if (stat(full_path, ent_stat) == 0)
        {
            if (!S_ISDIR(ent_stat->st_mode))
            {
                name_len = strlen(dir_entry->d_name);
                if ((name_len > 3)
                        && ('o' == dir_entry->d_name[name_len - 1])
                        && ('s' == dir_entry->d_name[name_len - 2])
                        && ('.' == dir_entry->d_name[name_len - 3])) /* ending with .m */
                {
                    /* check whether _res.so */
                    if ((name_len > 7)
                            && ('s' == dir_entry->d_name[name_len - 4])
                            && ('e' == dir_entry->d_name[name_len - 5])
                            && ('r' == dir_entry->d_name[name_len - 6])
                            && ('_' == dir_entry->d_name[name_len - 7]))
                    {
                        /* resource installer, skip */
                    }
                    else
                    {
                        /* add null terminator */
                        app_name = rt_malloc(name_len - 3 + 1);
                        RT_ASSERT(app_name);
                        memcpy(app_name, dir_entry->d_name, name_len - 3);
                        app_name[name_len - 3] = 0;

                        /* add _res and null terminator */
                        res_package_name = rt_malloc(full_path_len + sizeof(res_mod_name_suffix) + 1);
                        RT_ASSERT(res_package_name);
                        memcpy(res_package_name, full_path, full_path_len - 3);
                        res_package_name[full_path_len - 3] = 0;
                        strcat(res_package_name, res_mod_name_suffix);
                        strncat(res_package_name, ".so", 3);

                        info.app_name = app_name;
                        res_fd = open(res_package_name, O_RDONLY);
                        if (res_fd >= 0)
                        {
                            close(res_fd);
                            info.res_package_path = res_package_name;
                        }
                        else
                        {
                            info.res_package_path = NULL;
                        }
                        info.pgm_package_path = full_path;
                        app_uninstall(app_name, info.res_install_path, info.pgm_install_path);
                        install_res = app_install(&info);
                        if (0 != install_res)
                        {
                            rt_kprintf("Fail to install %s: %d\n", app_name, install_res);
                        }
                        else
                        {
                            rt_kprintf("Succ to install %s\n", app_name, install_res);
                        }
                        rt_free(app_name);
                        rt_free(res_package_name);
                        ret += install_res;
                    }
                }
            }
        }
        rt_free(full_path);
    }
    while (true);

    rt_free(ent_stat);
    closedir(dir);

__EXIT:
    return ret;
}


static int mod(int argc, char **argv)
{
    const char *package_path = NULL;
    const char *install_path = NULL;
    const char *mod_name;
    int res;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }

    if (0 == strcmp(argv[1], "install"))
    {
        if (argc < 3)
        {
            rt_kprintf("wrong argument\n");
            return -1;
        }
        mod_name = argv[2];
        if (argc >= 4)
        {
            package_path = argv[3];
        }
        if (argc >= 5)
        {
            install_path = argv[4];
        }

        res = mod_install(mod_name, package_path, install_path);
        rt_kprintf("install %s:%d\n", mod_name, res);
    }
    else if (0 == strcmp(argv[1], "run"))
    {
        if (argc < 4)
        {
            rt_kprintf("wrong argument\n");
            return -1;
        }
        mod_name = argv[2];
        install_path = argv[3];
        dlrun(mod_name, install_path);
    }
    else
    {
        rt_kprintf("Unknown command %s\n", argv[1]);
    }
    return 0;
}
MSH_CMD_EXPORT(mod, Install Module);

static int app(int argc, char **argv)
{
    const char *package_path = NULL;
    const char *install_path = NULL;
    const char *mod_name;
    int res;
    app_installer_info_t info;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }

    if (0 == strcmp(argv[1], "install"))
    {
        info.app_name = argv[2];
        if (argc < 7)
        {
            rt_kprintf("wrong argument\n");
            return -1;
        }
        info.res_package_path = argv[3];
        info.res_install_path = argv[4];
        info.pgm_package_path = argv[5];
        info.pgm_install_path = argv[6];
        /* uninstall first */
        app_uninstall(info.app_name, info.res_install_path, info.pgm_install_path);
        res = app_install(&info);
        rt_kprintf("install app %s:%d\n", info.app_name, res);
        if (0 != res)
        {
            rt_kprintf("Fail to install app: %s\n", info.app_name);
        }
    }
    else if (0 == strcmp(argv[1], "uninstall"))
    {
        if (argc < 5)
        {
            rt_kprintf("wrong argument\n");
            return -1;
        }
        info.app_name = argv[2];
        info.res_install_path = argv[3];
        info.pgm_install_path = argv[4];
        res = app_uninstall(info.app_name, info.res_install_path, info.pgm_install_path);
        rt_kprintf("uninstall app %s:%d\n", info.app_name, res);
    }
    else
    {
        rt_kprintf("Unknown command %s\n", argv[1]);
    }
    return 0;
}
MSH_CMD_EXPORT(app, Install Application);


int32_t mod_installer_init(uint32_t sys_prog_start_addr, uint32_t sys_prog_end_addr)
{
    int fd;
    int32_t ret = 0;
    uint32_t sys_prog_size;
    uint32_t old_size;
    bool reinstall_needed;
    char buf[16];
    int len;
    size_t str_len;
    CRC_HandleTypeDef crc_handle;
    uint32_t new_crc_val;
    uint32_t old_crc_val;

    sys_prog_size = sys_prog_end_addr - sys_prog_start_addr + 1;
    if (0 == sys_prog_size)
    {

        ret = -1;
        goto __EXIT;
    }

    crc_handle.Instance = hwp_crc;
    if (HAL_CRC_Init(&crc_handle) != HAL_OK)
    {
        ret = -1;
        goto __EXIT;
    }
    HAL_CRC_Setmode(&crc_handle, CRC_32);
    new_crc_val = HAL_CRC_Calculate(&crc_handle, (uint8_t *)sys_prog_start_addr, sys_prog_size);

    reinstall_needed = true;
    /* read old crc value */
    fd = open(SYS_INFO_FILE, O_RDONLY, 0);
    if (fd >= 0)
    {
        len = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        buf[len] = 0;
        if (len > 0)
        {
            old_crc_val = atoh(buf);
            if (old_crc_val == new_crc_val)
            {
                reinstall_needed = false;
            }
        }
    }

    if (reinstall_needed)
    {
        /* write new size */
        fd = open(SYS_INFO_FILE, O_WRONLY | O_CREAT, 0);
        if (fd < 0)
        {
            ret = -1;
            goto __EXIT;
        }
        rt_snprintf(buf, sizeof(buf), "%x", new_crc_val);
        str_len = strlen(buf);
        len = write(fd, buf, strlen(buf));
        RT_ASSERT(len == str_len);
        close(fd);
        rt_kprintf("Installing app...\n");
        app_auto_install();
    }

__EXIT:

    return ret;
}


