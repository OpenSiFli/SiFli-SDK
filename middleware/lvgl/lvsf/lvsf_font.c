/**
  ******************************************************************************
  * @file   lvsf_ft.c
  * @author Sifli software development team
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

#include "lv_ext_resource_manager.h"
#ifdef LV_USING_FREETYPE_ENGINE
    #include "lvsf_ft_reg.h"
#endif
#if defined (RT_USING_DFS)
    #include <dfs_posix.h>
    #ifdef WIN32
        #define open(filename,flag)  rt_open(filename,flag,0x644)
        #define close(handle) rt_close(handle)
        #define read(fd,buf,len) rt_read(fd,buf,len)
        #define write rt_write
        #define lseek rt_lseek
    #endif
#endif

void lv_ext_set_local_font_bitmap(lv_obj_t *obj, lv_color_t color, lv_font_t *font)
{
    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void lv_ext_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color)
{
    lv_font_t *font = (lv_font_t *)LV_EXT_FONT_GET(size);

    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
void lv_ext_lable_set_fixed_font(lv_obj_t *obj, uint16_t size, lv_color_t color, const char *font_name)
{
    LV_ASSERT_OBJ(obj, "lv_label");
#ifdef LV_USING_FREETYPE_ENGINE
    lv_label_ext_t *ext = lv_obj_get_ext_attr(obj);
    ext->font = lvsf_get_font_by_name((char *)font_name, size);
    rt_kprintf("lv_ext_lable_set_fixed_font: ext->font %p\n", ext->font);
    if (ext->font)
    {
        ext->font_fixed = true;
        lv_ext_set_local_text_font(obj, ext->font, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
#endif
        lv_ext_set_local_font(obj, size, color);
}
#endif

void lv_obj_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color)
{
    lv_ext_set_local_font(obj, size, color);
}


#ifndef LV_USING_FREETYPE_ENGINE

void lv_ext_font_reset(void)
{
}

uint16_t lv_font_get_ext_size(lv_font_t *font, uint32_t u_letter, lv_font_t **new_font)
{
    return 0;
}

#else

void lv_ext_set_font_local_by_name(lv_obj_t *obj, uint16_t size, lv_color_t color, char *fontname)
{
    lv_font_t *font = lvsf_get_font_by_name(fontname, size);
    if (!font) return;

    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

extern lv_font_t *lvsf_get_font_by_lib(lv_font_freetype_lib_dsc_t *font_lib, uint16_t size);

void lv_ext_set_font_local_by_lib(lv_obj_t *obj, uint16_t size, lv_color_t color, lv_font_freetype_lib_dsc_t *font_lib)
{
    lv_font_t *font = lvsf_get_font_by_lib(font_lib, size);

    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}


void lv_ext_set_font_line_height(uint16_t size, int line_height)
{
    lv_font_t *font = (lv_font_t *)LV_EXT_FONT_GET(size);

    font->line_height = line_height;
}

void lv_ext_font_reset(void)
{
#if defined (LV_USING_FREETYPE_ENGINE)
    //lv_freetype_clean_cache(FT_CACHE_QUAD_CLEAN);
#endif
}


void lv_freetype_set_font_size(lv_font_t *font, uint16_t size)
{
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;

    dsc->font_size = size;
    font->line_height = size  + size * 1 / 3 + 4;//* 3 / 2;
    font->base_line = size * 1 / 3;//* 3 / 2;
}


#if USE_CACHE_MANGER
uint32_t ft_get_cache_size(void)
{
    uint32_t max_weight = FT_CACHE_SIZE < 60 * 1024 ? 60 * 1024 : FT_CACHE_SIZE;
    return max_weight * 75 / 100;
}

static void ft_clean_cache_cb(void)
{
    // return;

#if defined (FREETYPE_CACHE_IN_SRAM_STANDALONE) || defined (FREETYPE_CACHE_IN_PSRAM)
    extern uint32_t app_mem_get_ft_cache_avail_size();
    uint32_t alloc_size = app_mem_get_ft_cache_avail_size();

    if (alloc_size >= ft_get_cache_size())
    {
        rt_kprintf("lv_freetype_clean_cache %d,%d\n", alloc_size, FT_CACHE_SIZE);
        lv_freetype_clean_cache(FT_CACHE_WHOLE_CLEAN);
    }
#endif

}

#include <freetype/internal/ftmemory.h>
#include <freetype/internal/ftobjs.h>

#if COMPATIBLE_WITH_SIFLI_EPIC_Ax
/*Byte align for every row is required if using GPU*/
static FT_Error ft_convert_bitmap_2bpp_cb(FTC_SBit sbit,             FT_Bitmap  *bitmap, FT_Memory   memory)
{
    FT_Error  error;
    FT_Int    pitch = bitmap->pitch;
    FT_ULong  size;
    FT_ULong  line_bytes;

    //rt_kprintf("ft_clean: ft_convert_bitmap_2bpp_c %d\n", FT_BPP);

    if (pitch < 0)
        pitch = -pitch;


#if FT_BPP == 4
    line_bytes = RT_ALIGN(pitch, 2) >> 1;
#elif FT_BPP == 2
    line_bytes = RT_ALIGN(pitch, 4) >> 2;
#endif
    size = line_bytes * bitmap->rows;
    //rt_kprintf("ft_convert_bitmap_2bpp_c %d,%d,%d\n", bitmap->pitch, bitmap->rows, line_bytes);
    if (!FT_QALLOC(sbit->buffer, size))
    {
        uint8_t *dst = sbit->buffer;
        uint8_t *src = bitmap->buffer;
        FT_Int  pitch_idx = 0;


#if FT_BPP == 4
        for (FT_ULong i = 0; i < size; i++)
        {
            if (pitch_idx + 2 < pitch)
            {
                dst[i] = ((src[pitch_idx] & 0xf0) >> 4) | (src[pitch_idx + 1] & 0xf0);
                pitch_idx += 2;
            }
            else if (pitch_idx + 2 == pitch)
            {
                dst[i] = ((src[pitch_idx] & 0xf0) >> 4) | (src[pitch_idx + 1] & 0xf0);
                src += pitch;//Next row
                pitch_idx = 0;
            }
            else
            {
                dst[i] = ((src[pitch_idx] & 0xf0) >> 4);
                src += pitch;//Next row
                pitch_idx = 0;
            }
        }
#elif FT_BPP == 2
        for (FT_ULong i = 0; i < size; i++)
        {
            if (pitch_idx + 4 < pitch)
            {
                dst[i] = ((src[pitch_idx + 0] & 0xC0) >> 6)
                         | ((src[pitch_idx + 1] & 0xC0) >> 4)
                         | ((src[pitch_idx + 2] & 0xC0) >> 2)
                         | ((src[pitch_idx + 3] & 0xC0) >> 0);

                pitch_idx += 4;
            }
            else
            {
                dst[i] = (src[pitch_idx] & 0xC0) >> 6;
                if (pitch_idx + 1 < pitch) dst[i] |= (src[pitch_idx + 1] & 0xC0) >> 4;
                if (pitch_idx + 2 < pitch) dst[i] |= (src[pitch_idx + 2] & 0xC0) >> 2;
                if (pitch_idx + 3 < pitch) dst[i] |= (src[pitch_idx + 3] & 0xC0) >> 0;

                src += pitch;//Next row
                pitch_idx = 0;
            }
        }
#endif
    }

    //print_letter_map(96,bitmap->buffer,bitmap->pitch,bitmap->rows,8);
    //print_letter_map(96,sbit->buffer,  bitmap->pitch,bitmap->rows,FT_BPP);

    FT_FREE(bitmap->buffer);

    return error;
}

#else

#define FT_USING_2BPP_INTERNAL 1

static FT_Error ft_convert_bitmap_2bpp_cb(FTC_SBit sbit,             FT_Bitmap  *bitmap, FT_Memory   memory)
{
#if FT_USING_2BPP_INTERNAL
    sbit->buffer = bitmap->buffer;
    bitmap->buffer = NULL;
    return 0;
#else
    FT_Error  error;
    FT_Int    pitch = bitmap->pitch;
    FT_ULong  size;

    //rt_kprintf("ft_clean: ft_convert_bitmap_2bpp_c %d\n", FT_BPP);

    if (pitch < 0)
        pitch = -pitch;

    size = (FT_ULong)pitch * bitmap->rows;

#if FT_BPP == 4
    size = (size >> 1) + 1;
#elif FT_BPP == 2
    size = (size >> 2) + 1;
#endif

    if (!FT_QALLOC(sbit->buffer, size))
    {
        uint8_t *dst = sbit->buffer;
        uint8_t *src = bitmap->buffer;

#if FT_BPP == 4
        for (FT_ULong i = 0; i < size; i++)
        {
            *dst++ = ((*src) & 0xf0) | (((*(src + 1)) & 0xf0) >> 4);
            src += 2;
        }
#elif FT_BPP == 2
        for (FT_ULong i = 0; i < size; i++)
        {
            *dst++ = ((*src) & 0xC0)
                     | (((*(src + 1)) & 0xC0) >> 2)
                     | (((*(src + 2)) & 0xC0) >> 4)
                     | (((*(src + 3)) & 0xC0) >> 6);
            src += 4;
        }
#endif
    }

    FT_FREE(bitmap->buffer);

    return error;
#endif
}
#endif /* LV_USE_GPU */

#endif


#if defined (RT_USING_DFS)
#include "dfs.h"
#include "dfs_posix.h"

static inline int ft_get_fd(struct dfs_fd *d)
{
    int index = 0;
    int fd = -1;
    struct dfs_fdtable *fd_table;
    if (RT_NULL == d)
    {
        return -1;
    };
    dfs_lock();
    fd_table = dfs_fdtable_get();
    if (RT_NULL == fd_table)
    {
        dfs_unlock();
        return -1;
    }

    for (index = 0; index < (int)fd_table->maxfd; index ++)
    {
        if (d == fd_table->fds[index])
        {
            fd = index + DFS_FD_OFFSET;
            break;
        }

    }
    dfs_unlock();
    return fd;
}


struct dfs_fd *ft_fopen(const char *name, const char *mode)
{
    int fd = open(name, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        rt_kprintf("ft_fopen name:%s err:%d\n", name, rt_get_errno());
        return RT_NULL;
    }

    struct dfs_fd *d = fd_get(fd);
    fd_put(d);
    return d;
}

long ft_ftell(struct dfs_fd *f)
{
    if (RT_NULL == f)
    {
        rt_kprintf("ft_ftell err\n");
        return -1;
    }
    return f->size;
}

int ft_fseek(struct dfs_fd *f, long offset, int whence)
{
    int fd = ft_get_fd(f);
    if (-1 != lseek(fd, offset, whence))
    {
        return 0;
    }
    rt_kprintf("ft_fseek %s filelen %d err fd:%d offset:%d whence:%d err:%d\n", f->path, ft_ftell(f), fd, offset, whence, rt_get_errno());
    return -1;
}

size_t ft_fread(void *ptr, size_t size, size_t nitems, struct dfs_fd *f)
{
    int read_size = 0;
    size_t total_size = 0;
    int fd = ft_get_fd(f);
    if (0 == size || 0 == nitems)
    {
        return 0;
    }

    read_size = read(fd, ptr, size * nitems);
    if (read_size <= 0)
    {
        rt_kprintf("ft_fread %s filelen %d err read:%d fd:%d src_size:%d err:%d\n", f->path, ft_ftell(f), read_size, fd, size * nitems, rt_get_errno());
        return 0;
    }
    total_size += read_size;
    return total_size;
}

size_t ft_fwrite(const void *ptr, size_t size, size_t nitems, struct dfs_fd *f)
{
    int write_size = 0;
    size_t total_size = 0;
    int fd = ft_get_fd(f);

    if (0 == size || 0 == nitems)
    {
        return 0;
    }

    write_size = write(fd, ptr, size * nitems);
    if (write_size <= 0)
    {
        rt_kprintf("ft_fwrite err size:%d\n", size * nitems);
        return 0;
    }
    total_size += write_size;
    return total_size;
}

int ft_fclose(struct dfs_fd *f)
{
    int fd = ft_get_fd(f);
    return close(fd);
}
#endif

#if !defined(PKG_SCHRIFT)

#define FT_RENDER_SIZE 0x2000

#ifndef  FT_RENDER_USE_DYNAMIC_ALLOC
#include "mem_section.h"

L1_NON_RET_BSS_SECT_BEGIN(ft_render_pool)
ALIGN(RT_ALIGN_SIZE) static uint8_t render[FT_RENDER_SIZE];
L1_NON_RET_BSS_SECT_END

static void *ft_render_pool_apply_mem(uint8_t *mem_type, uint32_t *max_pool)
{
    *max_pool = FT_RENDER_SIZE;
    return (void *) render;
}

static void ft_render_pool_rel_mem(void *ptr, uint8_t mem_type)
{
    ;
}
#else

static void *ft_render_pool_apply_mem(uint8_t *mem_type, uint32_t *max_pool)
{
    void *ptr = (long *)ft_smalloc(FT_RENDER_SIZE);
    *mem_type = 0;

    if (!ptr)
    {
        ptr = rt_malloc(FT_RENDER_SIZE);
        *mem_type = 1;
    }

    *max_pool = FT_RENDER_SIZE;

    RT_ASSERT(ptr);

    //rt_kprintf("ft_render_pool_apply_mem: type %d size %d ptr %p\n", *mem_type, FT_RENDER_SIZE, ptr);

    return ptr;
}

static void ft_render_pool_rel_mem(void *ptr, uint8_t mem_type)
{
    //rt_kprintf("ft_render_pool_rel_mem: type %d ptr %p\n", mem_type, ptr);

    if (0 == mem_type)
        ft_sfree(ptr);
    else if (1 == mem_type)
        rt_free(ptr);
}
#endif

typedef void *(* ft_render_pool_mem_apply_func)(uint8_t *mem_type, uint32_t *max_pool);
typedef void (* ft_render_pool_mem_rel_func)(void *ptr, uint8_t mem_type);
void ft_render_pool_apply_mem_register(ft_render_pool_mem_apply_func apply_func, ft_render_pool_mem_rel_func rel_func);

typedef void (* ft_clean_func)(void);
extern void ft_cache_clean_register(ft_clean_func func);

typedef FT_Error(*ft_bitmap_to_bpp_func)(FTC_SBit,           FT_Bitmap *, FT_Memory);
extern void ft_bitmap_to_bpp_register(ft_bitmap_to_bpp_func func, int bpp);


int ft_callback_reg(void)
{
    ft_render_pool_apply_mem_register(ft_render_pool_apply_mem, ft_render_pool_rel_mem);
    ft_cache_clean_register(ft_clean_cache_cb);
#if FT_USING_2BPP_INTERNAL
    ft_bitmap_to_bpp_register(ft_convert_bitmap_2bpp_cb, FT_BPP);
#else
    ft_bitmap_to_bpp_register(ft_convert_bitmap_2bpp_cb, 8);
#endif

    return 0;
}

INIT_PREV_EXPORT(ft_callback_reg);
#endif

//For the SDK, this is an example of registering fonts.
//For the Solution, the registering fonts will implemented in butterfli.exe tool.
#ifndef SOLUTION_WATCH
//LVSF_FREETYPE_FONT_REGISTER(tiny55_full);
//LVSF_FREETYPE_FONT_REGISTER(hindi);
//LVSF_FREETYPE_FONT_REGISTER(arab);
#if defined (FREETYPE_TINY_FONT_FULL)
    LVSF_FREETYPE_FONT_REGISTER(tiny55_full);
#elif defined (FREETYPE_TINY_FONT_LITE)
    LVSF_FREETYPE_FONT_REGISTER(tiny55_lite);
#else //FREETYPE_NORMAL_FONT
    LVSF_FREETYPE_FONT_REGISTER(SourceHanSansCN_Normal);
#endif


#ifdef FREETYPE_FONT_IN_FILE_SYSTEM
lv_font_freetype_lib_dsc_t SourceHanSansCN_Normal_lib = { 0, "/ex/fonts/SourceHanSansCN-Bold.ttf" };
#endif

#endif
#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
