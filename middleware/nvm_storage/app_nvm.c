/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <app_nvm.h>
#include <dfs_posix.h>

#define APP_NVM_ROOT "/app/nvm"

static const char *app_nvm_path(const char *key_name)
{
    static char path[128];

    rt_snprintf(path, sizeof(path), APP_NVM_ROOT "/%s.bin", key_name ? key_name : "app");
    return path;
}

size_t app_nvm_read(const char *key_name, const void *data, size_t length)
{
    int fd;
    ssize_t size;

    if (!data || !length)
        return 0;

    fd = open(app_nvm_path(key_name), O_RDONLY, 0);
    if (fd < 0)
        return 0;

    size = read(fd, (void *)data, length);
    close(fd);

    return size > 0 ? (size_t)size : 0;
}

rt_err_t app_nvm_write(const char *key_name, const void *data, size_t length)
{
    int fd;
    ssize_t size;

    if (!data || !length)
        return -RT_ERROR;

    mkdir(APP_NVM_ROOT, 0);

    fd = open(app_nvm_path(key_name), O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
        return -RT_ERROR;

    size = write(fd, data, length);
    close(fd);

    return size == (ssize_t)length ? RT_EOK : -RT_ERROR;
}
