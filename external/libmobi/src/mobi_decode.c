#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include "lvgl.h"
#include "app_mem.h"
#include "mobi.h"

#ifdef LVSF_USING_SJPG
    #include "lvsf_sjpg.h"
#endif
#if LV_USE_PNG
    #include "lv_png.h"
#endif

#define MOBI_DECODE_IMG_INFO_HASH_ENABLE
#define MOBI_DECODE_TAG_IMG      0x4d494d47  /* "MIMG" */
#define MOBI_DECODE_TAG_COVER    0x4d434f56  /* "MCOV" */
#define MOBI_DECODE_SRC_COVER    "cover"
#define MOBI_DECODE_SRC_UID      "uid:%u"

#ifdef MOBI_DECODE_IMG_INFO_HASH_ENABLE
#include "hash_cache.h"

#define MOBI_HASH_TABLE_NUMBER   32
#define MAX_MOBI_CACHE_SIZE      1024

typedef struct
{
    char                        *book_path;
    uint32_t                     uid;
    lv_img_header_t              header;
    uint16_t                     book_path_len;
    uint16_t                     decode_err : 1;
    uint16_t                     is_cover   : 1;
    uint16_t                     path_map   : 1;
} mobi_hash_key_t;

static hash_cache_t mobi_cache;
static bool mobi_hash_inited = false;

static int mobi_hash_match(void *key1, void *key2)
{
    mobi_hash_key_t *_key2 = key2, *_key1 = key1;

    if (!_key1->path_map) _key1->book_path = (char *)(_key1 + 1);
    return (_key1->uid == _key2->uid &&
            _key1->is_cover == _key2->is_cover &&
            _key1->book_path && _key2->book_path &&
            0 == strcmp(_key1->book_path, _key2->book_path));
}

static void mobi_hash_save(void *key1, void *key2)
{
    memcpy(key1, key2, sizeof(mobi_hash_key_t));
}

static uint32_t mobi_hash_map(void *key)
{
    mobi_hash_key_t *key1 = key;
    const char *book_path = key1->path_map ? key1->book_path : (char *)(key1 + 1);
    uint32_t len = book_path ? strlen(book_path) : 0;
    uint32_t hash = key1->uid + (key1->is_cover ? 17 : 0);

    if (len > 0) hash += book_path[len - 1];
    if (len > 5) hash += ((len & 0x01) ? (book_path[len - 5] << 1) : book_path[len - 5]);
    return hash & (MOBI_HASH_TABLE_NUMBER - 1);
}
#endif

static bool mobi_decode_part_is_jpg(MOBIPart *part)
{
    const uint8_t *data;

    if (!part || !part->data || part->size < 3) return false;
    data = (const uint8_t *)part->data;
    return part->type == T_JPG || (data[0] == 0xff && data[1] == 0xd8 && data[2] == 0xff);
}

static bool mobi_decode_part_is_png(MOBIPart *part)
{
    const uint8_t *data;
    static const uint8_t png_magic[] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};

    if (!part || !part->data || part->size < sizeof(png_magic)) return false;
    data = (const uint8_t *)part->data;
    return part->type == T_PNG || memcmp(data, png_magic, sizeof(png_magic)) == 0;
}

void mobi_decode_free(void *ptr)
{
    if (ptr) app_anim_free(ptr);
}

static char *mobi_decode_make_src(const char *book_path, const char *src, uint32_t tag)
{
    uint32_t align_4;
    uint32_t i;
    char *new_str;

    if (!book_path || !book_path[0] || !src || !src[0]) return NULL;
    align_4 = ((strlen(src) + 1 + 3) >> 2) << 2;
    new_str = (char *)app_anim_calloc(1, align_4 + 4 + 8 + strlen(book_path) + 2);
    if (!new_str) return NULL;

    strcpy(new_str, src);
    i = strlen(new_str);
    while (i < align_4) new_str[i++] = 1;
    *((uint32_t *)(new_str + align_4)) = tag;
    sprintf(new_str + align_4 + 4, "%08x", align_4);
    sprintf(new_str + align_4 + 4 + 8, "%s", book_path);
    return new_str;
}

char *mobi_decode_set_img_src(const char *book_path, uint32_t uid)
{
    char src[24];

    if (!book_path || !book_path[0]) return NULL;
    sprintf(src, MOBI_DECODE_SRC_UID, uid);
    return mobi_decode_make_src(book_path, src, MOBI_DECODE_TAG_IMG);
}

char *mobi_decode_set_cover_src(const char *book_path, const char *cover_name)
{
    const char *src = (cover_name && cover_name[0]) ? cover_name : MOBI_DECODE_SRC_COVER;

    return mobi_decode_make_src(book_path, src, MOBI_DECODE_TAG_COVER);
}

static int mobi_decode_get_src(const char *src, char **book_path, uint32_t *uid, bool *is_cover)
{
    int i = 0;
    uint32_t align_4;
    uint8_t *p;
    uint32_t d;
    uint32_t tag;

    if (!src || !book_path || !uid || !is_cover) return -1;
    while (src[i] && 1 != src[i]) i++;
    if (0 == src[i]) return -1;

    *book_path = NULL;
    *uid = 0;
    *is_cover = false;

    align_4 = ((i + 1 + 3) >> 2) << 2;
    p = (uint8_t *)src + align_4;
    sscanf((const char *)(p + 4), "%08x", &d);
    if (align_4 != d) return -1;

    tag = *((uint32_t *)p);
    if (MOBI_DECODE_TAG_IMG == tag)
    {
        if (sscanf(src, "uid:%u", uid) != 1) return -1;
        *book_path = (char *)(p + 12);
    }
    else if (MOBI_DECODE_TAG_COVER == tag)
    {
        *is_cover = true;
        *book_path = (char *)(p + 12);
    }
    else
    {
        return -1;
    }

    return i;
}

static MOBIPart *mobi_decode_get_cover_part(MOBIData *mobi, MOBIRawml *rawml, uint32_t *uid_out)
{
    MOBIExthHeader *exth;
    uint32_t cover_uid;
    MOBIPart *part;

    if (!mobi || !rawml) return NULL;
    exth = mobi_get_exthrecord_by_tag(mobi, EXTH_COVEROFFSET); 
    if (!exth || !exth->data || !exth->size) return NULL;

    cover_uid = mobi_decode_exthvalue((const unsigned char *)exth->data, exth->size);
    part = mobi_get_resource_by_uid(rawml, cover_uid);
    if (!part && cover_uid > 0) part = mobi_get_resource_by_uid(rawml, cover_uid - 1);
    if (!part) part = mobi_get_resource_by_uid(rawml, cover_uid + 1);
    if (!part || !part->data || !part->size) return NULL;

    if (uid_out) *uid_out = (uint32_t)part->uid;
    return part;
}

static MOBIPart *mobi_decode_load_part(const char *book_path, uint32_t uid, bool is_cover, MOBIData **mobi_out, MOBIRawml **rawml_out)
{
    MOBIData *mobi;
    MOBIRawml *rawml;
    MOBI_RET ret;
    MOBIPart *part;

    if (!book_path || !book_path[0] || !mobi_out || !rawml_out) return NULL;
    *mobi_out = NULL;
    *rawml_out = NULL;

    mobi = mobi_init();
    if (!mobi) return NULL;

    ret = mobi_load_filename(mobi, book_path);
    if (ret != MOBI_SUCCESS)
    {
        rt_kprintf("mobi_decode: load failed ret=%d\n", ret);
        mobi_free(mobi);
        return NULL;
    }

    rawml = mobi_init_rawml(mobi);
    if (!rawml)
    {
        mobi_free(mobi);
        return NULL;
    }

    ret = mobi_parse_rawml_opt(rawml, mobi, false, false, true);
    if (ret != MOBI_SUCCESS)
    {
        rt_kprintf("mobi_decode: parse failed ret=%d\n", ret);
        mobi_free_rawml(rawml);
        mobi_free(mobi);
        return NULL;
    }

    part = is_cover ? mobi_decode_get_cover_part(mobi, rawml, NULL) : mobi_get_resource_by_uid(rawml, uid);
    if (!part || !part->data || !part->size)
    {
        mobi_free_rawml(rawml);
        mobi_free(mobi);
        return NULL;
    }

    *mobi_out = mobi;
    *rawml_out = rawml;
    return part;
}

lv_res_t mobi_decode_get_img_info(const char *book_path, uint32_t uid, bool is_cover, lv_img_header_t *header)
{
    MOBIData *mobi = NULL;
    MOBIRawml *rawml = NULL;
    MOBIPart *part;
    lv_img_dsc_t img_dsc;
    lv_img_decoder_dsc_t dec_dsc;
    lv_res_t ret = LV_RES_INV;

    if (!book_path || !book_path[0] || !header) return LV_RES_INV;

#ifdef MOBI_DECODE_IMG_INFO_HASH_ENABLE
    mobi_hash_key_t key = {0};
    hash_node_t *node;

    key.book_path = (char *)book_path;
    key.uid = uid;
    key.is_cover = is_cover ? 1 : 0;
    key.path_map = 1;

    if (mobi_hash_inited)
    {
        node = hash_cache_get(&mobi_cache, &key);
        if (node)
        {
            mobi_hash_key_t *p_key = (mobi_hash_key_t *)(node + 1);
            if (p_key->decode_err) return LV_RES_INV;
            *header = p_key->header;
            return LV_RES_OK;
        }
    }
#endif

    part = mobi_decode_load_part(book_path, uid, is_cover, &mobi, &rawml);
    if (!part)
    {
        goto end;
    }

    memset(&img_dsc, 0, sizeof(img_dsc));
    img_dsc.header.cf = mobi_decode_part_is_jpg(part) ? LV_IMG_CF_JPG : LV_IMG_CF_RAW;
    img_dsc.data = (const uint8_t *)part->data;
    img_dsc.data_size = part->size;


    if (mobi_decode_part_is_jpg(part))
    {
#ifdef LVSF_USING_SJPG
        memset(&dec_dsc, 0, sizeof(dec_dsc));
        dec_dsc.src = &img_dsc;
        dec_dsc.src_type = LV_IMG_SRC_VARIABLE;
        ret = lvsf_sjpg_decoder_open(NULL, &dec_dsc);
        if (ret == LV_RES_OK)
        {
            *header = dec_dsc.header;
            lvsf_sjpg_decoder_close(NULL, &dec_dsc);
        }
#endif
    }
    else if (mobi_decode_part_is_png(part))
    {
#if LV_USE_PNG
        ret = lvsf_png_decoder_info(&img_dsc, header);
#endif
    }


end:
#ifdef MOBI_DECODE_IMG_INFO_HASH_ENABLE
    if (mobi_hash_inited)
    {
        uint16_t path_len = (strlen(book_path) + 1 + 3) >> 2 << 2;
        node = hash_cache_alloc(&mobi_cache, &key, sizeof(mobi_hash_key_t) + path_len);
        if (node)
        {
            mobi_hash_key_t *p_key = (mobi_hash_key_t *)(node + 1);
            strcpy((char *)(p_key + 1), book_path);
            p_key->book_path_len = path_len;
            p_key->decode_err = (LV_RES_OK == ret) ? 0 : 1;
            p_key->header = *header;
            p_key->path_map = 0;
            hash_cache_set(&mobi_cache, node, 0);
        }
    }
#endif

    if (rawml) mobi_free_rawml(rawml);
    if (mobi) mobi_free(mobi);
    return ret;
}

static lv_res_t mobi_decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header)
{
    char *book_path = NULL;
    uint32_t uid = 0;
    bool is_cover = false;

    (void)decoder;
    if (mobi_decode_get_src((const char *)src, &book_path, &uid, &is_cover) < 0) return LV_RES_INV;
    return mobi_decode_get_img_info(book_path, uid, is_cover, header);
}

static lv_res_t mobi_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    char *book_path = NULL;
    uint32_t uid = 0;
    bool is_cover = false;
    MOBIData *mobi = NULL;
    MOBIRawml *rawml = NULL;
    MOBIPart *part;
    lv_img_dsc_t img_dsc;
    lv_res_t ret = LV_RES_INV;
    const void *save_src;
    lv_img_src_t save_src_type;
    bool is_jpg;
    bool is_png;

    if (!dsc || mobi_decode_get_src((const char *)dsc->src, &book_path, &uid, &is_cover) < 0) return LV_RES_INV;
    part = mobi_decode_load_part(book_path, uid, is_cover, &mobi, &rawml);
    if (!part)
    {
        return LV_RES_INV;
    }

    is_jpg = mobi_decode_part_is_jpg(part);
    is_png = mobi_decode_part_is_png(part);
    if (!is_jpg && !is_png) goto end;

    save_src = dsc->src;
    save_src_type = dsc->src_type;

    memset(&img_dsc, 0, sizeof(img_dsc));
    img_dsc.header.cf = is_jpg ? LV_IMG_CF_JPG : LV_IMG_CF_RAW;
    img_dsc.data = (const uint8_t *)part->data;
    img_dsc.data_size = part->size;
    dsc->src = &img_dsc;
    dsc->src_type = LV_IMG_SRC_VARIABLE;

    if (is_jpg)
    {
#ifdef LVSF_USING_SJPG
        ret = lvsf_sjpg_decoder_open(decoder, dsc);
#endif
    }
    else
    {
#if LV_USE_PNG
        ret = lvsf_png_decoder_open(decoder, dsc);
#endif
    }


    dsc->src = save_src;
    dsc->src_type = save_src_type;

end:
    mobi_free_rawml(rawml);
    mobi_free(mobi);
    return ret;
}

static lv_res_t mobi_decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x, lv_coord_t y,
                                       lv_coord_t len, uint8_t *buf)
{
    return LV_RES_INV;
}

static void mobi_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    lv_img_src_t save_src_type;

    if (!dsc) return;
    save_src_type = dsc->src_type;
    dsc->src_type = LV_IMG_SRC_VARIABLE;

    if (dsc->user_data)
    {
#ifdef LVSF_USING_SJPG
        lvsf_sjpg_decoder_close(decoder, dsc);
#endif
    }
    else if (dsc->img_data)
    {
#if LV_USE_PNG
        lvsf_png_decoder_close(decoder, dsc);
#endif
    }

    dsc->src_type = save_src_type;
}

int mobi_decoder_img_init(void)
{
    lv_img_decoder_t *dec;

    rt_kprintf("%s\n", __func__);
#ifdef MOBI_DECODE_IMG_INFO_HASH_ENABLE
    if (!mobi_hash_inited)
    {
        hash_cache_reg(&mobi_cache, rt_malloc, rt_free, mobi_hash_match, mobi_hash_save, mobi_hash_map,
                       NULL, NULL, MOBI_HASH_TABLE_NUMBER, MAX_MOBI_CACHE_SIZE);
        mobi_hash_inited = true;
    }
#endif
    dec = lv_img_decoder_create();
    if (!dec) return -1;
    lv_img_decoder_set_info_cb(dec, mobi_decoder_info);
    lv_img_decoder_set_open_cb(dec, mobi_decoder_open);
    lv_img_decoder_set_close_cb(dec, mobi_decoder_close);
    lv_img_decoder_set_read_line_cb(dec, mobi_decoder_read_line);
    return 0;
}
