/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_ext_resource_manager.h"
#include "lvsf_font.h"
#ifdef LV_USING_FREETYPE_ENGINE
    #include "lvsf_ft_reg.h"
    #include "lv_freetype.h"
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

extern const lv_obj_class_t lv_label_class;

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

void lv_ext_lable_set_fixed_font(lv_obj_t *obj, uint16_t size, lv_color_t color, const char *font_name)
{
#ifdef LV_USING_FREETYPE_ENGINE
    lv_font_t *font = lvsf_get_font_by_name((char *)font_name, size);

    if (font)
    {
        lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
        return;
    }
#endif

    lv_ext_set_local_font(obj, size, color);
}

void lv_ext_label_set_indicated_font(lv_obj_t *obj, uint16_t size, lv_color_t color, const char *font_name)
{
    LV_ASSERT_OBJ(obj, &lv_label_class);
#ifdef LV_USING_FREETYPE_ENGINE
    lv_font_t *font = lvsf_get_font_by_name((char *)font_name, size);
    rt_kprintf("%s: size %d font %p font_name %s\n", __func__, size, font, font_name);
    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
#endif
}


void lv_ext_label_set_font_image(lv_obj_t *obj, uint16_t size, lv_color_t color, const char *font_name, uint8_t font_interval)
{
    LV_ASSERT_OBJ(obj, &lv_label_class);
#if 0//def LV_USING_FREETYPE_ENGINE
    lv_label_ext_t *ext = lv_obj_get_ext_attr(obj);
    ext->font_image = font_name;
    ext->font_interval = font_interval;
#endif
}

void lv_obj_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color)
{
    lv_ext_set_local_font(obj, size, color);
}

static void reload_font(lv_obj_t *obj, const char *unload_font)
{
#ifdef LV_USING_FREETYPE_ENGINE
    const lv_font_t *font   = lv_obj_get_style_text_font(obj, LV_PART_MAIN);
    if (font && font->font_name && 0 == strcmp(font->font_name, unload_font))
    {
        lv_font_t *fallback_font = (lv_font_t *)LV_EXT_FONT_GET(FONT_NORMAL);
        lv_ext_set_local_text_font(obj, fallback_font, LV_PART_MAIN | LV_STATE_DEFAULT);
    }
#endif
}

void lv_ext_reload_font(lv_obj_t *top_obj, const char *unload_font)
{
    if (top_obj == NULL) return; /* Shouldn't happen */

    uint32_t child_cnt = lv_obj_get_child_cnt(top_obj);
    for (uint32_t i = 0; i < child_cnt; i++)
    {
        lv_obj_t *child = top_obj->spec_attr->children[i];
        lv_ext_reload_font(child, unload_font);
        if (lv_obj_check_type(child, &lv_label_class))
        {
            reload_font(child, unload_font);
        }
    }
}

#ifdef PSRAM_CACHE_WB
extern int mpu_dcache_clean(void *data, uint32_t size);
/**
 * @brief Clean DCache for font related buffer data.
 * @param data Buffer address.
 * @param size Buffer size in bytes.
 * @retval int Cache clean result.
 */
int lv_font_dcache_clean(void *data, uint32_t size)
{
    return mpu_dcache_clean(data, size);
}
#else
/**
 * @brief Provide a dummy DCache clean operation on unsupported platforms.
 * @param data Buffer address.
 * @param size Buffer size in bytes.
 * @retval int Always returns 0.
 */
int lv_font_dcache_clean(void *data, uint32_t size)
{
    return 0;
}
#endif


#ifndef LV_USING_FREETYPE_ENGINE
int lv_ext_get_font_full_name(char *font_name, char *full_name, uint16_t full_name_len)
{
    return -1;
}

void lv_ext_font_reset(void)
{
}

/**
 * @brief Get extended glyph size when FreeType is disabled.
 * @param font Source font.
 * @param u_letter Unicode letter.
 * @param new_font Output font pointer.
 * @retval uint16_t Always returns 0.
 */
uint16_t lv_font_get_ext_size(lv_font_t *font, uint32_t u_letter, lv_font_t **new_font)
{
    return 0;
}

/**
 * @brief Estimate the font size from a font object.
 * @param font Font object.
 * @retval uint16_t Estimated font size.
 */
uint16_t lvsf_get_size_from_font(lv_font_t *font)
{
    return font->line_height - font->base_line;
}

/**
 * @brief Get a font by name when FreeType is disabled.
 * @param font_name Font name.
 * @param size Font size.
 * @retval lv_font_t * Always returns NULL.
 */
lv_font_t *lvsf_get_font_by_name(char *font_name, int size)
{
    return NULL;
}

#else

#define be16_to_host(be) ((uint16_t) ((be >> 8) | (be << 8)))
#define be32_to_host(be) (((be >> 24) & 0xFF) | ((be >> 8) & 0xFF00) | ((be << 8) & 0xFF0000) | ((be << 24)))

#ifdef RT_USING_DFS
    #define file_read(x, len) dfs_file_read(&fd, x, len)
    #define file_seek(x, mod) dfs_file_lseek(&fd, mod ? fd.pos + x : x)
    #define file_pos          fd.pos
#else
    #define file_read(x, len)
    #define file_seek(x, mod)
    #define file_pos          0
#endif

#define ttf_read(x, len)                                                        \
{                                                                               \
    if (is_file)                                                                \
    {                                                                           \
        file_read(x, len);                                                      \
    }                                                                           \
    else if (0 == is_builtin_res)                                               \
    {                                                                           \
        memcpy((uint8_t *) x, p_ttf + pos, len);                                \
        pos += len;                                                             \
    }                                                                           \
    else                                                                        \
    {                                                                           \
        lv_img_decode_flash_read((uint32_t)(p_ttf + pos), (uint8_t *) x, len);  \
        pos += len;                                                             \
    }                                                                           \
}

#define ttf_lseek(_pos, mod)                                                    \
{                                                                               \
    if (is_file)                                                                \
    {                                                                           \
        file_seek(_pos, mod);                                                   \
    }                                                                           \
    else                                                                        \
    {                                                                           \
        pos = mod ? pos + _pos : _pos;                                          \
    }                                                                           \
}

#define ttf_cur_pos  (is_file ? file_pos : pos)

static int ttf_get_name(const char *ttf, uint8_t is_file, char *en, uint16_t en_len, char *cn, uint16_t cn_len)
{
    uint8_t is_builtin_res = 0;
    uint32_t pos = 0;
    uint8_t *p_ttf = (uint8_t *) ttf;

    memset(cn, 0x00, cn_len);
    memset(en, 0x00, en_len);

#ifdef RT_USING_DFS
    struct dfs_fd fd = {0};
    if (is_file)
    {
        if (dfs_file_open(&fd, ttf, O_RDONLY | O_BINARY))
        {
            rt_kprintf("Failed to open file");
            return -1;
        }
    }
#else
    is_file = 0;
#endif

#ifndef BSP_USING_PC_SIMULATOR
#ifndef LV_IS_IMG_ON_FLASH
#define LV_IS_IMG_ON_FLASH(ttf) 0
#endif
    extern uint32_t lv_img_decode_flash_read(uint32_t addr, uint8_t *buf, int size);
    if (!is_file && LV_IS_IMG_ON_FLASH(ttf))
    {
        is_builtin_res = 1;
    }
#endif

    /* read Offset Table */
    uint32_t        sfntVersion;
    ttf_read(&sfntVersion, 4);
    sfntVersion         = be32_to_host(sfntVersion);

    uint16_t        numTables;
    ttf_read(&numTables, 2);
    numTables            = be16_to_host(numTables);

    /* skip scope (6bytes) */
    ttf_lseek(6, 1);

    /* search'name' table */
    uint32_t        nameTableOffset = 0, nameTableLength = 0;
    int             found = 0;

    for (int i = 0; i < numTables; ++i)
    {
        char            tag[4];
        ttf_read(&tag, 4);
        if (memcmp(tag, "name", 4) == 0)
        {
            uint32_t        checksum, offset, length;

            ttf_read(&checksum, 4);
            ttf_read(&offset, 4);
            ttf_read(&length, 4);
            nameTableOffset     = be32_to_host(offset);
            nameTableLength     = be32_to_host(length);
            found                = 1;
            break;
        }
        else
        {
            /* skip non-name-table */
            ttf_lseek(12, 1);
        }
    }

    if (!found)
    {
        dfs_file_close(&fd);
        printf("'name' table not found.\n");
        return -1;
    }

    /* goto name-table */
    ttf_lseek(nameTableOffset, 0);

    /* read name header */
    uint16_t        format, count, storageOffset;

    ttf_read(&format, 2);
    ttf_read(&count, 2);
    ttf_read(&storageOffset, 2);
    format               = be16_to_host(format);
    count                = be16_to_host(count);
    storageOffset        = be16_to_host(storageOffset);

    for (int i = 0; i < count; ++i)
    {
        uint16_t        platformID, encodingID, languageID, nameID, length, offset;

        ttf_read(&platformID, 2);
        ttf_read(&encodingID, 2);
        ttf_read(&languageID, 2);
        ttf_read(&nameID, 2);
        ttf_read(&length, 2);
        ttf_read(&offset, 2);
        platformID            = be16_to_host(platformID);
        encodingID            = be16_to_host(encodingID);
        languageID            = be16_to_host(languageID);
        nameID                = be16_to_host(nameID);
        length                = be16_to_host(length);
        offset                = be16_to_host(offset);

        if (nameID == 4) /* full name */
        {
            long    stringPos = nameTableOffset + storageOffset + offset;
            long    currentPos = ttf_cur_pos;

            ttf_lseek(stringPos, 0);

            uint8_t    *buffer = (uint8_t *) lv_mem_alloc(length + 1);
            RT_ASSERT(buffer);

            ttf_read(buffer, length);

            if (platformID == 3 && encodingID == 1) // Windows Unicode
            {
                int         chars = length / 2;
                uint16_t    *wstr = (uint16_t *) lv_mem_alloc((chars + 1) * sizeof(uint16_t));
                RT_ASSERT(wstr);

                for (int j = 0; j < chars; ++j)
                {
                    wstr[j] = (buffer[j * 2] << 8) + buffer[j * 2 + 1];
                }

                wstr[chars]         = L'\0';

                if (0x804 == languageID || 0x404 == languageID || 0x409 == languageID)
                {

                    unsigned char *utf16le_bytes = (unsigned char *) wstr;
                    unsigned short  utf16_code_point;
                    char           *utf8_str = lv_mem_alloc(2 * length + 1);
                    RT_ASSERT(utf8_str);
                    memset(utf8_str, 0x00, 2 * length + 1);
                    int             i, j = 0;

                    for (i = 0; i < chars * 2; i += 2)
                    {
                        utf16_code_point    = (utf16le_bytes[i] | (utf16le_bytes[i + 1] << 8));

                        if (utf16_code_point < 0x80)
                        {
                            utf8_str[j++]        = (char) utf16_code_point;
                        }
                        else if (utf16_code_point < 0x800)
                        {
                            utf8_str[j++]        = (char)(0xC0 | (utf16_code_point >> 6));
                            utf8_str[j++]        = (char)(0x80 | (utf16_code_point & 0x3F));
                        }
                        else
                        {
                            utf8_str[j++]        = (char)(0xE0 | (utf16_code_point >> 12));
                            utf8_str[j++]        = (char)(0x80 | ((utf16_code_point >> 6) & 0x3F));
                            utf8_str[j++]        = (char)(0x80 | (utf16_code_point & 0x3F));
                        }
                    }

                    utf8_str[j]         = '\0';

                    if (0x804 == languageID || 0x404 == languageID)
                    {
                        memset(cn, 0x00, cn_len);
                        memcpy(cn, utf8_str, j + 1 > cn_len ? cn_len - 1 : j + 1);
                    }
                    else if (0x409 == languageID)
                    {
                        memset(en, 0x00, en_len);
                        memcpy(en, utf8_str, j + 1 > en_len ? en_len - 1 : j + 1);
                    }
                    lv_mem_free(utf8_str);
                }

                // wprintf(L"Font Full Name: %ls\n", wstr);
                lv_mem_free(wstr);
            }
            else if (platformID == 1 && encodingID == 0)
            {
                // Mac Roman
                buffer[length]        = '\0';
                //printf("Font Full Name: %s\n", buffer);
            }
            else
            {
                printf("Unsupported encoding platform=%d, encoding=%d\n", platformID, encodingID);
            }

            lv_mem_free(buffer);
            ttf_lseek(currentPos, 0);
        }
    }

#ifdef RT_USING_DFS
    if (fd.path)
        dfs_file_close(&fd);
#endif
    return 0;
}

int lv_ext_get_font_full_name(char *font_name, char *full_name, uint16_t full_name_len)
{
#if 0
    lv_font_t *font = lvsf_get_font_by_name(font_name, FONT_NORMAL);
    if (!font) return NULL;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;
    if (dsc->face->family_name)
    {
        rt_kprintf("%s full name: %s\n", font_name, dsc->face->family_name);
        //return dsc->face->family_name;
    }
    else
    {
        rt_kprintf("%s full name failed!!!\n", font_name);
        return NULL;
    }
    if (dsc->face->style_name)
    {
        rt_kprintf("%s full name: %s\n", font_name, dsc->face->style_name);
        //return dsc->face->style_name;
    }
    else
    {
        rt_kprintf("%s full name failed!!!\n", font_name);
        return NULL;
    }
#else
    lv_font_t *font = lvsf_get_font_by_name(font_name, FONT_NORMAL);
    if (!font) return -1;

    if (font->font_name && 0 < strlen(font->font_name))
    {
        strncpy(full_name, font->font_name, full_name_len - 1);
        full_name[full_name_len - 1] = '\0';
    }
    else
    {
        lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;
        /* dsc->face is released after metrics setup when the FTC cache
         * manager is active; family name is unavailable then. */
        if (!dsc || !dsc->face || !dsc->face->family_name)  return -1;
        memset(full_name, 0x00, full_name_len);
        strncpy(full_name, dsc->face->family_name, full_name_len - 1);
    }

    rt_kprintf("%s: ttf %s full_name %s\n", __func__, font_name, full_name);

    return 0;

#endif
}

void lv_ext_set_font_local_by_name(lv_obj_t *obj, uint16_t size, lv_color_t color, char *fontname)
{
    lv_font_t *font = lvsf_get_font_by_name(fontname, size);
    if (!font) return;

    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

extern lv_font_t *lvsf_get_font_by_lib(const lv_font_freetype_lib_dsc_t *font_lib, uint16_t size);

/**
 * @brief Set a local font and color for an object by font library.
 * @param obj Target object.
 * @param size Font size.
 * @param color Text color.
 * @param font_lib Font library descriptor.
 * @retval None
 */
void lv_ext_set_font_local_by_lib(lv_obj_t *obj, uint16_t size, lv_color_t color, lv_font_freetype_lib_dsc_t *font_lib)
{
    lv_font_t *font = lvsf_get_font_by_lib(font_lib, size);

    lv_ext_set_local_text_font(obj, font, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_ext_set_local_text_color(obj, color, LV_PART_MAIN | LV_STATE_DEFAULT);
}


/**
 * @brief Set the line height of the font with the specified size.
 * @param size Font size.
 * @param line_height New line height.
 * @retval None
 */
void lv_ext_set_font_line_height(uint16_t size, int line_height)
{
    lv_font_t *font = (lv_font_t *)LV_EXT_FONT_GET(size);

    font->line_height = line_height;
}

/**
 * @brief Reset the font module state.
 * @retval None
 */
void lv_ext_font_reset(void)
{
#if defined (LV_USING_FREETYPE_ENGINE)
    //lv_freetype_clean_cache(FT_CACHE_QUAD_CLEAN);
#endif
}

uint16_t lvsf_get_size_from_font(lv_font_t *font)
{

    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;
    return dsc->font_size;
}

void lv_freetype_set_font_size(lv_font_t *font, uint16_t size)
{
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;
#if !defined (PKG_SCHRIFT)
    if (dsc->face)
    {
        FT_Set_Pixel_Sizes(dsc->face, 0, size);
        font->line_height = (dsc->face->size->metrics.height >> 6);
        font->base_line = -(dsc->face->size->metrics.descender >> 6) + 4;  /*Base line measured from the top of line_height*/
    }
#if USE_CACHE_MANGER
    else if (dsc->face_source)
    {
        /* dsc->face is released after init under the FTC, so take the metrics
         * of the new size from the cache to keep them in sync with it. */
        FT_Size face_size = NULL;
        struct FTC_ScalerRec_ scaler;

        memset(&scaler, 0, sizeof(scaler));
        scaler.face_id = (FTC_FaceID)dsc->face_source;
        scaler.width = size;
        scaler.height = size;
        scaler.pixel = 1;
        if (lv_freetype_lookup_size(&scaler, &face_size) == 0 && face_size)
        {
            font->line_height = (face_size->metrics.height >> 6);
            font->base_line = -(face_size->metrics.descender >> 6) + 4;
        }
    }
#endif
#endif
    dsc->font_size = size;
}


#if FT_CACHE_SIZE > 0
/**
 * @brief Get the maximum weight allowed for the FreeType cache.
 * @retval uint32_t Maximum cache weight.
 */
uint32_t ft_get_cache_size(void)
{
    uint32_t max_weight = FT_CACHE_SIZE < 60 * 1024 ? 60 * 1024 : FT_CACHE_SIZE * 80 / 100;
    return max_weight;
}

static void ft_clean_cache_cb(void)
{
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

#if defined (LV_USING_FREETYPE_ENGINE)
#include <freetype/internal/ftmemory.h>
#include <freetype/internal/ftobjs.h>

#if (COMPATIBLE_WITH_SIFLI_EPIC_Ax)
/*Byte align for every row is required if using GPU*/
static FT_Error ft_convert_bitmap_2bpp_cb(FTC_SBit sbit, FT_Bitmap  *bitmap, FT_Memory   memory)
{
    FT_Error  error;
    FT_Int    pitch = bitmap->pitch;
    FT_ULong  size;
    FT_ULong  line_bytes;
    static uint8_t logged;

    if (!logged)
    {
        logged = 1;
        rt_kprintf("%s: width %d rows %d pitch %d target_bpp %d\n",
                   __func__, bitmap->width, bitmap->rows, bitmap->pitch, FT_BPP);
    }

    if (pitch < 0)
        pitch = -pitch;

#if FT_BPP == 4
    line_bytes = RT_ALIGN(pitch, 2) >> 1;
#elif FT_BPP == 2
    line_bytes = RT_ALIGN(pitch, 4) >> 2;
#endif
    size = line_bytes * bitmap->rows;
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

    FT_FREE(bitmap->buffer);
    return error;
}

#else

#define FT_USING_2BPP_INTERNAL 0

static FT_Error ft_convert_bitmap_2bpp_cb(FTC_SBit sbit, FT_Bitmap  *bitmap, FT_Memory   memory)
{
    static uint8_t logged;

    if (!logged)
    {
        logged = 1;
        rt_kprintf("%s: width %d rows %d pitch %d target_bpp %d\n",
                   __func__, bitmap->width, bitmap->rows, bitmap->pitch, FT_BPP);
    }

#if FT_USING_2BPP_INTERNAL || FT_BPP == 8
    sbit->buffer = bitmap->buffer;
    bitmap->buffer = NULL;
    return 0;
#else
    FT_Error  error = 0;
    FT_Int    pitch = bitmap->pitch;
    FT_ULong  size;

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

#endif

#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    #include "drv_flash.h"
    #include "mem_map.h"
    #include "flash_map.h"
#elif defined (RT_USING_DFS)
    #include "dfs.h"
    #include "dfs_posix.h"
#endif

/**
 * @brief Check whether a font library descriptor points to flash resources.
 * @param font_lib Font library descriptor.
 * @retval int Non-zero if the font is stored in flash, otherwise 0.
 */
int lv_font_lib_is_in_flash(void *font_lib)
{
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    return LV_IS_FONT_ON_FLASH(font_lib);
#else
    return 0;
#endif
}

#if defined (RT_USING_DFS) || defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)

static uint32_t lv_font_decode_flash_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    if (LV_IS_FONT_ON_FLASH(addr))
    {
#if defined (FONT_USING_FLASH_B)
        rt_flash_read(addr, buf, size);
#else
#ifdef SOLUTION_RES_BUILT_IN_ON_EMMC
        rt_sdio_read(addr, buf, size);
#else
        rt_nand_read(addr, buf, size);
#endif
#endif
    }
    else
#endif
    {
        lv_memcpy(buf, (const void *) addr, size);
    }

    return size;
}

struct dfs_fd *ft_fopen(const char *name, const char *mode)
{
    //rt_kprintf("%s: name %s\n", __func__, name);
#if defined (RT_USING_DFS)
    if (name[0] >= 0x20 && name[0] <= 0x7F)
    {
        struct dfs_fd *fd = lv_mem_alloc(sizeof(struct dfs_fd));
        RT_ASSERT(fd);
        int ret = dfs_file_open(fd, name, O_RDONLY | O_BINARY);
        //rt_kprintf("%s: fd %p name %s\n", __func__, fd, name);
        if (0 != ret)
        {
            lv_mem_free(fd);
            fd = NULL;
        }
        else
        {
            fd->magic = DFS_FD_MAGIC;
            dfs_file_enable_fast_seek(fd, 1);
        }
        return fd;
    }
#endif
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    {
        lv_freetype_font_builtin_t *fd = lv_mem_alloc(sizeof(lv_freetype_font_builtin_t));
        RT_ASSERT(fd);
        *fd = *((lv_freetype_font_builtin_t *) name);
        fd->pos = 0;
        return (struct dfs_fd *)fd;
    }
#endif
    return NULL;
}

long ft_ftell(struct dfs_fd *f)
{
    RT_ASSERT(f);
#if defined (RT_USING_DFS)
    if (f->magic)
    {
        return f->pos;
    }
#endif
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    lv_freetype_font_builtin_t *fd = (lv_freetype_font_builtin_t *) f;
    return fd->pos;
#endif
    return -1;
}

int ft_fseek(struct dfs_fd *f, long offset, int whence)
{
    RT_ASSERT(f);
#if defined (RT_USING_DFS)
    if (f->magic)
    {
        switch (whence)
        {
        case SEEK_SET:
            break;
        case SEEK_CUR:
            offset += f->pos;
            break;
        case SEEK_END:
            offset += f->size;
            break;
        default:
            return -1;
        }
        //rt_kprintf("%s_1: f %p offset %d whence %d\n", __func__, f, offset, whence);
        return dfs_file_lseek(f, offset);
    }
#endif
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    lv_freetype_font_builtin_t *fd = (lv_freetype_font_builtin_t *) f;
    switch (whence)
    {
    case SEEK_SET:
        fd->pos = offset;
        break;
    case SEEK_CUR:
        fd->pos += offset;
        break;
    case SEEK_END:
        fd->pos = offset + fd->lib_size;
        break;
    default:
        return -1;
    }
    return fd->pos;
#endif
    return -1;
}

size_t ft_fread(void *ptr, size_t size, size_t nitems, struct dfs_fd *f)
{
    RT_ASSERT(f);
#if defined (RT_USING_DFS)
    if (f->magic)
    {
        return dfs_file_read(f, ptr, size * nitems);
    }
#endif
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    lv_freetype_font_builtin_t *fd = (lv_freetype_font_builtin_t *) f;
    int ret = lv_font_decode_flash_read((uint32_t)(fd->lib_addr + fd->pos), ptr, size * nitems);
    //memcpy(ptr, (void *) (fd->lib_addr + fd->pos), size * nitems);
    fd->pos += size * nitems;
    return ret;
#endif
    return 0;
}

size_t ft_fwrite(const void *ptr, size_t size, size_t nitems, struct dfs_fd *f)
{
    RT_ASSERT(f);
#if defined (RT_USING_DFS)
    if (f->magic)
    {
        return dfs_file_write(f, ptr, size * nitems);
    }
#endif
#if defined (SOLUTION_RES_BUILT_IN) || defined (FONT_USING_FLASH_B)
    lv_freetype_font_builtin_t *fd = (lv_freetype_font_builtin_t *) f;
    fd->pos += size * nitems;
    return 0;
#endif
    return 0;
}

int ft_fclose(struct dfs_fd *f)
{
    RT_ASSERT(f);
    rt_kprintf("%s: f %p\n", __func__, f);
#if defined (RT_USING_DFS)
    if (f->magic)
    {
        int ret = dfs_file_close(f);
    }
#endif
    lv_mem_free(f);
    return 0;
}
#endif

const char *ft_get_font_path(void)
{
#ifdef SOLUTION
#include "flash_map.h"
    return RES_FONT_PATH;
#else
    return NULL;
#endif
}

#if !defined(PKG_SCHRIFT)

#define FT_RENDER_SIZE 0x1600
//#define FT_RENDER_USE_DYNAMIC_ALLOC

#ifndef  FT_RENDER_USE_DYNAMIC_ALLOC
#include "mem_section.h"

#ifdef BSP_USING_PSRAM
    //L2_NON_RET_BSS_SECT_BEGIN(ft_render_pool)
    //ALIGN(RT_ALIGN_SIZE) static uint8_t render[FT_RENDER_SIZE];
    //L2_NON_RET_BSS_SECT_END
    L2_CACHE_NON_RET_BSS_SECT_BEGIN(ft_render_pool)
    ALIGN(RT_ALIGN_SIZE) static uint8_t render[FT_RENDER_SIZE] L2_CACHE_NON_RET_BSS_SECT(ft_render_pool);
    L2_CACHE_NON_RET_BSS_SECT_END
#else
    L1_NON_RET_BSS_SECT_BEGIN(ft_render_pool)
    ALIGN(RT_ALIGN_SIZE) static uint8_t render[FT_RENDER_SIZE] L1_NON_RET_BSS_SECT(ft_render_pool);
    L1_NON_RET_BSS_SECT_END
#endif

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
#endif


int ft_callback_reg(void)
{
#ifdef SOLUTION_RES_USING_NAND
#define FREETYPE_CACHE_SIZE     512
#else
#define FREETYPE_CACHE_SIZE     256
#endif
    lv_freetype_set_parameter(FT_BPP, 512, EXTERN_CACHE_AGINE);
#if !defined(PKG_SCHRIFT)
    ft_render_pool_apply_mem_register(ft_render_pool_apply_mem, ft_render_pool_rel_mem);
    ft_cache_clean_register(ft_clean_cache_cb);
#endif
#if FT_USING_2BPP_INTERNAL
    ft_bitmap_to_bpp_register(ft_convert_bitmap_2bpp_cb, FT_BPP);
    rt_kprintf("%s: bitmap_to_bpp registered bpp=%d mode=internal\n", __func__, FT_BPP);
#else
    ft_bitmap_to_bpp_register(ft_convert_bitmap_2bpp_cb, 8);
    rt_kprintf("%s: bitmap_to_bpp registered bpp=%d mode=compat8\n", __func__, 8);
#endif
#ifdef USING_FONT_UNKOWN_CUSTOM_DISPLAY
    /**
    * Usage of lv_freetype_set_unkown_font_mode():
    * 1. Use the native display mode for UNKNOWN characters (usually a square box). Two setting methods:
    *    1) Do not call the function lv_freetype_set_unkown_font_mode
    *    2) lv_freetype_set_unkown_font_mode(0, UINT32_MAX);
    * 2. Custom display mode for customers:
    *    1) Ignore the unknown character directly without occupying space: lv_freetype_set_unkown_font_mode(1, UINT32_MAX);
    *    2) Replace with a specific character: lv_freetype_set_unkown_font_mode(0, replace_unicode);
    */
    lv_freetype_set_unkown_font_mode(0, 0xFF1F); /* '?' U+3F (0x3F) ,  "？" U+FF1F (0xFF1F)*/
#endif
    return 0;
}

INIT_PREV_EXPORT(ft_callback_reg);

//For the SDK, this is an example of registering fonts.
//For the Solution, the registering fonts will implemented in butterfli.exe tool.
#if !defined(SOLUTION) && !defined(FREETYPE_FONT_NAME)
//LVSF_FREETYPE_FONT_REGISTER(tiny55_full);
//LVSF_FREETYPE_FONT_REGISTER(hindi);
//LVSF_FREETYPE_FONT_REGISTER(arab);
#if defined (FREETYPE_TINY_FONT_FULL)
    LVSF_FREETYPE_FONT_REGISTER(tiny55_full);
#elif defined (FREETYPE_TINY_FONT_LITE)
    LVSF_FREETYPE_FONT_REGISTER(tiny55_lite);
#else //FREETYPE_NORMAL_FONT
    LVSF_FREETYPE_FONT_REGISTER(ReaderSourceHanSansCN_Normal);
#endif

const lv_font_freetype_lib_dsc_t ReaderSourceHanSansCN_Normal_lib = { 0, "/ex/fonts/ReaderSourceHanSansCN-Normal.ttf" };

#endif

#endif
