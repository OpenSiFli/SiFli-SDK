/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 ******************************************************************************
 * @file   app_lang.c
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "rtthread.h"
#include "app_lang.h"
#include "app_mem.h"
#include "log.h"
#include "lv_ext_resource_manager.h"
#include "llt_mem.h"

#if defined(RT_USING_DFS)
    #include "dfs.h"
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #ifdef WIN32
        #define open(filename, flag) rt_open(filename, flag, 0x644)
        #define close(handle) rt_close(handle)
        #define read(fd, buf, len) rt_read(fd, buf, len)
        #define lseek rt_lseek
    #endif
#endif

static bool fat_lang_using_mem = true;

#if defined(LV_USING_EXT_RESOURCE_MANAGER) && defined(RT_USING_DFS)

static ex_lang_t lang_list = { 0 };

#define EX_LANG_FILE            "ex_lang.bin"
#define EX_LANG_TRANS_LIST_FILE "ex_lang_trans_list.lst"

//return value: content length
static int read_line_with_content(int fd, char *buf, uint32_t max_len)
{
    RT_ASSERT(max_len > 1);
    uint32_t i = 0;
    uint32_t off = lseek(fd, 0, SEEK_CUR);
    buf[0] = 0;
    while (i < max_len)
    {
        if (0 == read(fd, &buf[i], 64) || strlen(&buf[i]) < 64) break;
        i += 64;
        buf[i] = 0;
    }
    buf[max_len - 1] = 0;
    uint32_t len = strlen(buf);
    lseek(fd, off + len + 1, SEEK_SET);
    return len;
}

//return value: 0: file end; 1: normal
static int read_line_no_content(uint8_t *buf, uint32_t *pos, uint32_t total_len)
{
    uint32_t cur_pos = *pos;
    while (cur_pos < total_len && buf[cur_pos]) cur_pos++;
    int ret = (cur_pos == *pos) ? 0 : 1;
    //rt_kprintf("%s: pos %x str %s end %x ret %d\n", __func__, *pos, &buf[*pos], cur_pos, ret);
    *pos = cur_pos + 1;
    return ret;
}

static int lang_decode_header(const char *lang_file, ex_lang_node_t *node)
{
    int ret = -1, fd = -1;
    char *buf = NULL;

    memset(node, 0x00, sizeof(ex_lang_node_t));

    while (1)
    {
        uint32_t num = 0;
        fd = open(lang_file, O_RDONLY);
        if (fd < 0) break;

        buf = app_malloc(129);
        //number
        if (0 == read(fd, &num, 4)) break;
        //version
        int ret_len = read_line_with_content(fd, buf, 128);
        if (0 == ret_len) break;
        //pgm_name
        ret_len = read_line_with_content(fd, buf, 128);
        if (0 == ret_len) break;
        node->pgm_name = (const char *)app_malloc(ret_len + 1);
        strcpy((void *)node->pgm_name, buf);
        //locale
        ret_len = read_line_with_content(fd, buf, 128);
        if (0 == ret_len) break;
        node->locale = (const char *)app_malloc(ret_len + 1);
        strcpy((void *)node->locale, buf);

        ret = 0;
        break;
    }

    if (buf) app_free(buf);
    if (fd >= 0) close(fd);

    if (-1 == ret)
    {
        if (node->pgm_name) app_free((void *)node->pgm_name);
        if (node->locale) app_free((void *) node->locale);
    }

    return ret;
}

static void *lang_key_list_mem;

static lv_i18n_lang_t *lang_decode_trans(const char *lang_path, ex_lang_node_t *node)
{
    int             ret = -1;
    lv_i18n_lang_t *lang_pack = NULL;
    uint32_t        pos = 0, num = 0, addr_version = 0, addr_key = 0, addr_locale = 0, i = 0, length = 0;
    uint8_t        *addr;

    struct dfs_fd src_fd;
    if (dfs_file_open(&src_fd, lang_path, O_RDONLY | O_BINARY) < 0) goto end;
    if (src_fd.size <= 4) goto end;
    if (lang_key_list_mem)
    {
        app_cache_free(lang_key_list_mem);
        lang_key_list_mem = 0;
    }
    addr = (uint8_t *) app_cache_alloc(src_fd.size, CACHE_PSRAM);
    if (!addr) goto end;
    if (0 >= dfs_file_read(&src_fd, (void *) addr, src_fd.size)) goto end;

    num = *((uint32_t *)addr);
    pos += 4;
    addr_version = (uint32_t) addr + pos;
    if (0 == read_line_no_content(addr, &pos, src_fd.size)) goto end;
    addr_key = (uint32_t) addr + pos;
    if (0 == read_line_no_content(addr, &pos, src_fd.size)) goto end;
    addr_locale = (uint32_t) addr + pos;
    if (0 == read_line_no_content(addr, &pos, src_fd.size))goto end;
    length = sizeof(lv_i18n_lang_t) + sizeof(lv_i18n_phrase_t) * num;
    lang_pack = app_malloc(length);
    if (!lang_pack) goto end;
    lang_pack->locale = (const char *) addr_locale;
    lang_pack->translation = (uint8_t *) lang_pack + sizeof(lv_i18n_lang_t);
    lv_i18n_phrase_t *phrase = (lv_i18n_phrase_t *) lang_pack->translation;

    int ret_len = 1;
    while (i < num && ret_len)
    {
        //uint32_t key_addr = (uint32_t ) addr + pos;
        //if (0 == read_line_no_content(addr, &pos, src_fd.size)) break;
        uint32_t singular_addr = (uint32_t) addr + pos;
        //rt_kprintf("%s: i %4d pos %8d %s\n", __func__, i, pos, (const char *) singular_addr);
        ret_len = read_line_no_content(addr, &pos, src_fd.size);
        //phrase->key = (const char *) key_addr;
        phrase->singular = (const char *) singular_addr;
        if (0 == strlen((const char *) singular_addr))
        {
            LOG_E("%s: error happened in file (%s)!!!! off %x str (%s) ret_len %d errno %d ", __func__, lang_path, pos, (const char *)singular_addr, ret_len, rt_get_errno());
            break;
        }
        phrase++;
        i++;
    }

    if (i < num) goto end;
    {
        ret = 0;
    }

    node->pgm_name = (const char *) addr_key;
    node->locale = (const char *) addr_locale;
    LOG_I("%s: file %s num %d version %s pgm_name %s locale %s %p mem %d",
          __func__, lang_path, num, (char *)addr_version, (char *)node->pgm_name, node->locale, lang_pack, fat_lang_using_mem);

end:
    if (src_fd.path)
        dfs_file_close(&src_fd);
    if (-1 == ret)
    {
        if (lang_pack) rt_free(lang_pack);
        lang_pack = NULL;
    }
    else
    {
        lang_key_list_mem = (void *) addr;
    }

    return lang_pack;
}

/**
 * @brief  Load multi-language list from flash, which dir is "path".
           This is only used in file_system.
 * @param  path The directory of multi-language package bin.
 * @retval lang_list Pointer of multi-language list.
 */
ex_lang_t *app_lang_load_pack_list(const char *path)
{
    uint32_t i = 0;

    DIR *dir = opendir(path);
    if (!dir) goto end;

    if (lang_list.node_list)
    {
        while (i < lang_list.list_num)
        {
            app_free((void *) lang_list.node_list[i].locale);
            app_free((void *) lang_list.node_list[i].pgm_name);
            i++;
        }
        LLT_MEM_FREE(lang_list.node_list);
        lang_list.node_list = NULL;
    }

    i = 0;

    do
    {
        struct dirent *dir_entry = readdir(dir);
        if (!dir_entry) break;
        char *full_path = dfs_normalize_path(path, dir_entry->d_name);
        if (!full_path) break;
        size_t name_len = strlen(dir_entry->d_name);
        if ((name_len > 4)
                && ('n' == dir_entry->d_name[name_len - 1])
                && ('i' == dir_entry->d_name[name_len - 2])
                && ('b' == dir_entry->d_name[name_len - 3])
                && ('.' == dir_entry->d_name[name_len - 4])) /* ending with .dsc */
        {
            ex_lang_node_t lang_node = {0};
            if (-1 == lang_decode_header(full_path, &lang_node))
            {
                dfs_file_unlink(full_path);
                LOG_E("%s: Fail to lang_decode_header %s", __func__, full_path);
            }
            else
            {
                LOG_I("%s: Succ to lang_decode_header %s %s %s", __func__, full_path, lang_node.pgm_name, lang_node.locale);
                lang_list.node_list = LLT_MEM_REALLOC(lang_list.node_list, (i + 1) * sizeof(ex_lang_node_t));
                lang_list.node_list[i++] = lang_node;
            }
        }
        else
        {
            dfs_file_unlink(full_path);
            LOG_E("%s_1: Fail to lang_decode_header %s", __func__, full_path);
        }
        app_free(full_path);
    }
    while (true);

    lang_list.list_num = i;

end:
    if (dir) closedir(dir);
    LOG_I("%s: path %s dir %p num %d", __func__, path, dir, lang_list.list_num);
    return &lang_list;
}

/**
 * @brief  Traverse through all multilingual lists one by one,
           and return the names of each multi-language
 * @param  i Index, increase by 1 each time.
 * @retval name The name of each multi-language.
 */
const char *app_lang_pack_iterator(uint32_t *i)
{
    uint32_t index = *i;
    if (!lang_list.node_list || index >= lang_list.list_num) return NULL;
    (*i)++;
    return lang_list.node_list[index].locale;
}

/**
 * @brief  Install a language to memory, which file_name is "path/locale".
           This funciton only used to set current language.
 * @param  path The directory of multi-language package bin.
 * @param  The names of multi-language, which will be installed.
 * @retval install_result RT_EOK: normal; RT_ERR: abnormal.
 */
rt_err_t app_lang_install_pack(const char *path, const char *locale)
{
    rt_err_t ret = RT_EOK;
    int i = 0;
    ex_lang_node_t lang_node = { 0 };

    if (!lang_list.node_list || 0 == lang_list.list_num) return -1;
    while (lang_list.node_list[i].locale && i < lang_list.list_num && 0 != strncmp(lang_list.node_list[i].locale, locale, strlen(locale))) i++;
    LOG_I("%s: pgm_name %s", __func__, lang_list.pgm_name ? lang_list.pgm_name : "null");
//    if (i < lang_list.list_num && 0 == strcmp(lang_list.pgm_name, lang_list.node_list[i].pgm_name)) return 0;
    if (lang_list.pgm_name)
    {
        lv_ext_del_lang_pack(NULL, lang_list.pgm_name);
        if (fat_lang_using_mem) app_free(lang_list.lang_pack);
        if (lang_list.pgm_name) app_free((void *)lang_list.pgm_name);
        lang_list.lang_pack = NULL;
        lang_list.pgm_name = NULL;
    }
    if (i >= lang_list.list_num) return -1;
    LOG_I("%s: new_pgm_name %s", __func__, lang_list.node_list[i].pgm_name);
    char *full_path = app_calloc(1, 256);
    RT_ASSERT(full_path);
    snprintf(full_path, 255, "%s/%s.bin", path, lang_list.node_list[i].pgm_name);
    static lv_i18n_lang_t *lang_pack[2] = { 0 };
    lang_pack[0] = lang_decode_trans(full_path, &lang_node);
    if (NULL == lang_pack[0] || NULL == lv_ext_add_lang_pack(NULL, (const lv_i18n_lang_pack_t *)lang_pack, lang_node.pgm_name))
    {
        LOG_E("%s: Fail to register lang_pack %s pgm_name %s", __func__, full_path, lang_node.pgm_name);
        if (fat_lang_using_mem && lang_pack[0]) app_free(lang_pack[0]);
        ret = RT_ERROR;
    }
    else
    {
        lang_list.pgm_name = app_malloc(strlen(lang_node.pgm_name) + 1);
        strcpy((char *)lang_list.pgm_name, lang_node.pgm_name);
        lang_list.lang_pack = lang_pack[0];
        LOG_I("%s: Succ to register lang_pack %s pgm_name %s", __func__, full_path, lang_node.pgm_name);
    }
    app_free(full_path);
    return ret;
}

/**
 * @brief  Delete a language from flash, which file_name is "path/locale" .
 * @param  path The directory of multi-language package bin.
 * @param  lang_name The directory of multi-language package bin.
 * @retval delete_result RT_EOK: normal; RT_ERR: abnormal.
 */
rt_err_t app_lang_del_one_pack(const char *path, const char *lang_pack_name)
{
    char *fullpath;
    if (!lang_list.node_list || 0 == lang_list.list_num) return RT_ERROR;
    int i = 0;
    while (i < lang_list.list_num && 0 != strncmp(lang_list.node_list[i].pgm_name, lang_pack_name, strlen(lang_list.node_list[i].pgm_name))) i++;
    if (i == lang_list.list_num) goto end;
    lang_list.list_num--;
    while (i < lang_list.list_num)
    {
        lang_list.node_list[i] = lang_list.node_list[i + 1];
        i++;
    }
end:
    fullpath = dfs_normalize_path(path, lang_pack_name);
    if (!fullpath) return RT_ERROR;
    dfs_file_unlink(fullpath);
    rt_free(fullpath);
    return RT_EOK;
}


#endif

void app_fat_set_para(bool nor_lang_using_mem)
{
    fat_lang_using_mem = nor_lang_using_mem;
}


