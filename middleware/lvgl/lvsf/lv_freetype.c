/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
*      INCLUDES
*********************/
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
*  STATIC VARIABLES
**********************/

#if defined (LV_USING_FREETYPE_ENGINE) && !defined(PKG_SCHRIFT) && !defined(USING_VGLITE)

#include "lv_freetype.h"
#include "lvsf_ft_reg.h"
#include "lvsf_font.h"
#include <string.h>

FT_Library library;
static uint16_t g_bpp = FT_BPP;
static uint16_t g_cache_max_font_size = FONT_SUBTITLE;
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    static bool     g_extern_cache = false;
#endif

#if USE_CACHE_MANGER
static FTC_Manager cache_manager;
static FTC_CMapCache cmap_cache;
static FTC_ImageCache image_cache;

/*
 * Rotating scratch buffers holding the glyph bitmaps handed to the renderer.
 *
 * The bitmap of FTC_ImageCache_Lookup belongs to the cache and may be evicted
 * by any later lookup, while draw_letter blits it through the EPIC GPU
 * asynchronously, so the pointer handed out has to stay valid - and hold the
 * box_w bytes/row layout LVGL expects for A8 - after the callback returns.
 *
 * A slot that was handed out is only recycled once
 * draw_ctx->wait_for_finish() reports the GPU idle.
 */
#define GLYPH_SNAPSHOT_SLOTS 4

typedef struct
{
    uint8_t  *buf;
    uint32_t  buf_size;
    bool      handed_out; /* returned to the renderer since the last GPU sync */
} glyph_snapshot_t;

static glyph_snapshot_t glyph_snapshots[GLYPH_SNAPSHOT_SLOTS];
static uint8_t glyph_snapshot_next;

/* FTC has no locking of its own, so a flush requested from another thread
 * must run on the rendering thread, between two lookups. */
static volatile uint8_t g_ftc_flush_pending;

#ifdef DRV_EPIC_NEW_API
extern void *ft_smalloc(size_t nbytes);
extern void ft_sfree(void *ptr);
#else
static void glyph_snapshot_sync_gpu(void)
{
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    int i;

    /* Text can also be rendered outside a display refresh (e.g. canvas);
     * the GPU queue is global, so any display's wait hook serves. */
    if (!disp) disp = lv_disp_get_default();

    if (disp && disp->driver && disp->driver->draw_ctx &&
            disp->driver->draw_ctx->wait_for_finish)
    {
        disp->driver->draw_ctx->wait_for_finish(disp->driver->draw_ctx);
    }

    for (i = 0; i < GLYPH_SNAPSHOT_SLOTS; i++)
    {
        glyph_snapshots[i].handed_out = false;
    }
}
#endif

static const uint8_t *glyph_snapshot_store(const FT_Bitmap *bitmap)
{
    uint32_t need = (uint32_t)bitmap->width * bitmap->rows;
    glyph_snapshot_t *s = &glyph_snapshots[glyph_snapshot_next];
    uint8_t *dst_buf;

    if (need == 0) return NULL;

#ifdef DRV_EPIC_NEW_API
    /*
     * The render list holds the glyph pointer until the frame is blitted, so
     * a buffer cannot be reused within a frame. Give every glyph its own ft
     * heap block: ft_sfree defers the free while the pipeline still
     * references it (ref-counted through MEM_ASYN_FONT).
     */
    dst_buf = ft_smalloc(need);
    if (!dst_buf) return NULL;
    if (s->buf) ft_sfree(s->buf);
    s->buf = dst_buf;
    s->buf_size = need;
#else
    if (s->handed_out)
    {
        glyph_snapshot_sync_gpu();
    }

    if (s->buf_size < need)
    {
        uint8_t *nbuf = rt_realloc(s->buf, need);
        if (!nbuf) return NULL;
        s->buf = nbuf;
        s->buf_size = need;
    }
    dst_buf = s->buf;
#endif

    /* FT gray bitmaps may have pitch != width, and a negative pitch for a
     * bottom-up layout; LVGL A8 rows are exactly box_w bytes. */
    {
        const uint8_t *src = bitmap->buffer;
        int pitch = bitmap->pitch;
        uint8_t *dst = dst_buf;
        unsigned int row;

        if (pitch < 0)
        {
            src += (unsigned int)(-pitch) * (bitmap->rows - 1);
        }

        for (row = 0; row < bitmap->rows; row++)
        {
            memcpy(dst, src, bitmap->width);
            dst += bitmap->width;
            src += pitch;
        }
    }

#ifndef DRV_EPIC_NEW_API
    s->handed_out = true;
#endif
    glyph_snapshot_next = (glyph_snapshot_next + 1) % GLYPH_SNAPSHOT_SLOTS;
    return dst_buf;
}

/* Unicode whitespace codepoints that legitimately render to an empty bitmap
 * while still advancing the pen. */
static bool freetype_letter_is_whitespace(uint32_t letter)
{
    return letter == 0x20 || letter == 0xA0 || letter == 0x1680 ||
           (letter >= 0x2000 && letter <= 0x200D) ||
           letter == 0x202F || letter == 0x205F ||
           letter == 0x3000 || letter == 0xFEFF;
}

static uint32_t freetype_cache_size = 0;

extern void FTC_Manager_Cache_Free(FTC_Manager  manager, unsigned int max_weight);

/**********************
 *      MACROS
 **********************/

/**********************
 *   STATIC FUNCTIONS
 **********************/
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
typedef struct
{
    uint32_t unicode_letter; //max unicode is 0x10FFFF
    uint16_t font_size;
    uint16_t padding;
} sft_hash_key_t;

typedef struct sft_lru_link
{
    struct sft_lru_link *prev;
    struct sft_lru_link *next;
} sft_lru_link_t;

typedef struct sft_hash_node_tag
{
    sft_lru_link_t              lru_link;   //LRU double link
    sft_hash_key_t              key;
    struct sft_hash_node_tag    *next;      //hash conflict list,has same hash map,  use single link to save memory
    uint16_t                    adv_w;
    uint16_t                    box_h;
    uint16_t                    box_w;
    int8_t                      ofs_x;
    int8_t                      ofs_y;
    //uint32_t                    value[0]; //MSVC not support 0 length array, value is attached after sft_hash_node_t
} sft_hash_node_t;

//SFT_HASH_TABLE_NUMBER must be 2^n
#define SFT_HASH_TABLE_NUMBER       256     //big enough to reduce hash confict, one launguage sways has continue unicode
#define MAX_UNICODE_CACHED_NUMBER   1024    //how many unicode char to cache

typedef struct
{
    int                 cached_number;
    int                 cached_number_limit;
    sft_lru_link_t      lru_root;
    sft_hash_node_t    *hash_table[SFT_HASH_TABLE_NUMBER]; //only save first element pointer in hash_table to save memory
} sft_cache_t;

#define  IS_KEY_MATCH(key1, key2)  ((key1.unicode_letter == key2.unicode_letter) && (key1.font_size == key2.font_size))
#define  SFT_HASH_MAP(key) (key.unicode_letter & (SFT_HASH_TABLE_NUMBER - 1))

static sft_cache_t *g_cache_p;

/*
....hashmap is index of hash array.to save memory, hash array only save element pointer.
    if some elements have same hashmap, first element save
    it's pointer in array[hashmap], other save in the list apend at array[hashmap]->next
*/
static sft_hash_node_t *sft_cache_get(sft_cache_t *cache, sft_hash_key_t *p_key)
{
    sft_hash_key_t k = *p_key;
    sft_hash_node_t *node = cache->hash_table[SFT_HASH_MAP(k)];

    //if has first in hashmap array
    if (node == NULL)
    {
        return NULL;
    }
    //check which match in first or it's conflict list
    if (IS_KEY_MATCH(node->key, k))
    {
        return node;
    }

    while (node->next)
    {
        if (IS_KEY_MATCH(node->key, k))
        {
            return node;
        }
        node = node->next;
    }

    return NULL;
}

static void sft_cache_delete_one(sft_cache_t *cache)
{
    if (cache->lru_root.prev != &cache->lru_root)
    {
        sft_hash_node_t *first_node;
        sft_hash_node_t *del_node;
        sft_hash_node_t *cur_node;
        RT_ASSERT(cache->cached_number > 0);
        //delete from LRU
        del_node = rt_container_of(cache->lru_root.prev, sft_hash_node_t, lru_link);
        cache->lru_root.prev = cache->lru_root.prev->prev;
        cache->lru_root.prev->next = &cache->lru_root;
        //delete from cache array or conflict list
        int hash_index = SFT_HASH_MAP(del_node->key);
        first_node = cache->hash_table[hash_index];

        RT_ASSERT(first_node);
        cur_node = first_node;
        if (cur_node == del_node)
        {
            cache->hash_table[hash_index] = cur_node->next;
        }
        else
        {
            while (1)
            {
                cur_node = first_node->next;
                if (cur_node == del_node)
                {
                    first_node->next = cur_node->next;
                    break;
                }
                first_node = first_node->next;
                RT_ASSERT(first_node);
            }
        }
        RT_ASSERT(IS_KEY_MATCH(del_node->key, cur_node->key));
        cache->cached_number--;
        ft_sfree(del_node);
    }
    else
    {
        RT_ASSERT(cache->cached_number == 0);
    }
}

static void sft_cache_delete_old(sft_cache_t *cache, int num)
{
    int j = num <= cache->cached_number ? num :  cache->cached_number;
    for (int i = 0; i < j; i++)
    {
        sft_cache_delete_one(cache);
    }
}

static void sft_cache_delete_all(sft_cache_t *cache)
{
#if 0
    //not good performance, only for testing
    int j = cache->cached_number;
    for (int i = 0; i < j; i++)
    {
        sft_cache_delete_one(cache);
    }
#else
    for (int i = 0; i < SFT_HASH_TABLE_NUMBER; i++)
    {
        sft_hash_node_t *first_node = cache->hash_table[i];

        if (first_node)
        {
            sft_hash_node_t *del = first_node;
            first_node = first_node->next;
            ft_sfree(del);
            cache->cached_number--;
            while (first_node)
            {
                del = first_node;
                first_node = first_node->next;
                ft_sfree(del);
                cache->cached_number--;
            }
        }
    }
    memset(cache->hash_table, 0, sizeof(cache->hash_table));
    cache->lru_root.next = &cache->lru_root;
    cache->lru_root.prev =  &cache->lru_root;

#endif

    RT_ASSERT(cache->cached_number == 0);
}

static sft_hash_node_t *sft_cache_alloc(sft_cache_t *cache, sft_hash_key_t *p_key, uint32_t value_size)
{
    value_size = sizeof(sft_hash_node_t) + value_size;

    sft_hash_node_t *new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    if (new_node)
    {
        goto got_it;
    }
    //rt_kprintf("--sft deltete cache old\n");
    sft_cache_delete_old(cache, cache->cached_number >> 4);
    new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    if (new_node)
    {
        goto got_it;
    }
    sft_cache_delete_old(cache, cache->cached_number >> 3);
    new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    if (new_node)
    {
        goto got_it;
    }
    sft_cache_delete_old(cache, cache->cached_number >> 2);
    new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    if (new_node)
    {
        goto got_it;
    }
    sft_cache_delete_old(cache, cache->cached_number >> 1);
    new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    if (new_node)
    {
        goto got_it;
    }
    sft_cache_delete_all(cache);
    new_node = (sft_hash_node_t *)ft_smalloc(value_size);
    RT_ASSERT(new_node);

got_it:
    memset(new_node, 0, sizeof(*new_node));
    new_node->key = *p_key;
    return new_node;
}

static void sft_cache_free(sft_hash_node_t *p)
{
    ft_sfree((void *)p);
}

static void sft_cache_set(sft_cache_t *cache, sft_hash_node_t *new_node)
{
    //add to LRU list head
    new_node->lru_link.prev = &cache->lru_root;
    new_node->lru_link.next = cache->lru_root.next;
    cache->lru_root.next->prev = &new_node->lru_link;
    cache->lru_root.next = &new_node->lru_link;
    int hash_index = SFT_HASH_MAP(new_node->key);
    sft_hash_node_t *first = cache->hash_table[hash_index];
    if (first == NULL)
    {
        //first element for this key
        cache->hash_table[hash_index] = new_node;
    }
    else
    {
        // conflict hash map, insert to list head
        new_node->next = first->next;
        first->next = new_node;
    }
    cache->cached_number++;
}


static sft_cache_t *sft_cache_init(int limit)
{

    sft_cache_t *p = (sft_cache_t *)ft_smalloc(sizeof(*p));
    RT_ASSERT(p);
    memset(p, 0, sizeof(*p));
    p->cached_number_limit = limit;
    p->lru_root.next = &p->lru_root;
    p->lru_root.prev =  &p->lru_root;
    return p;
}

static void sft_cache_deinit(sft_cache_t *p)
{
    if (p)
    {
        sft_cache_delete_all(p);
        ft_sfree(p);
    }
}


static void sft_cache_dump_all()
{
    sft_cache_t *cache = g_cache_p;
    if (cache)
        return;
    int dumped = 0;
    for (int i = 0; i < SFT_HASH_TABLE_NUMBER; i++)
    {
        sft_hash_node_t *first_node = cache->hash_table[i];
        while (first_node)
        {
            dumped++;
            rt_kprintf("one cache: unicode=0x%x, font_size=%d, data=%p, data_size=%d\n",
                       first_node->key.unicode_letter,
                       first_node->key.font_size,
                       &first_node[1],
                       (first_node->box_w * first_node->box_h * g_bpp + 7) / 8);

            first_node = first_node->next;
        }
    }

    RT_ASSERT(cache->cached_number == dumped);
}
#endif

/*
 * The FTC face_id. One FTC face is a fully parsed, resident copy of the font
 * file plus, for file fonts, one open fd - so faces are keyed per font file
 * and refcounted, while the FTC sizes MRU manages the FT_Size of each size.
 */
typedef struct freetype_face_source
{
    struct freetype_face_source *next;
    const char *mem_addr;   /* memory fonts: key = base address */
    int         mem_size;
    char       *path;       /* file fonts: key = path (owned copy) */
    uint16_t    ref_count;
} freetype_face_source_t;

static freetype_face_source_t *g_face_sources;

static freetype_face_source_t *freetype_face_source_acquire(const char *font_lib_addr, int font_lib_size)
{
    freetype_face_source_t *src;

    if (!font_lib_addr) return NULL;

    for (src = g_face_sources; src; src = src->next)
    {
        if (font_lib_size > 0)
        {
            if (src->mem_addr == font_lib_addr && src->mem_size == font_lib_size) break;
        }
        else if (src->path && strcmp(src->path, font_lib_addr) == 0)
        {
            break;
        }
    }

    if (!src)
    {
        src = rt_calloc(1, sizeof(*src));
        if (!src) return NULL;
        if (font_lib_size > 0)
        {
            src->mem_addr = font_lib_addr;
            src->mem_size = font_lib_size;
        }
        else
        {
            size_t len = strlen(font_lib_addr) + 1;
            src->path = rt_malloc(len);
            if (!src->path)
            {
                rt_free(src);
                return NULL;
            }
            memcpy(src->path, font_lib_addr, len);
        }
        src->next = g_face_sources;
        g_face_sources = src;
    }

    src->ref_count++;
    return src;
}

static void freetype_face_source_release(freetype_face_source_t *src)
{
    freetype_face_source_t **pp;

    if (!src) return;
    if (--src->ref_count > 0) return;

    if (cache_manager)
    {
        FTC_Manager_RemoveFaceID(cache_manager, (FTC_FaceID)src);
    }

    for (pp = &g_face_sources; *pp; pp = &(*pp)->next)
    {
        if (*pp == src)
        {
            *pp = src->next;
            break;
        }
    }
    if (src->path) rt_free(src->path);
    rt_free(src);
}

static FT_Error  font_Face_Requester(FTC_FaceID  face_id,
                                     FT_Library  request_library,
                                     FT_Pointer  req_data,
                                     FT_Face    *aface)
{
    freetype_face_source_t *src = (freetype_face_source_t *)face_id;
    FT_Error error;
    FT_Library ft_library = request_library ? request_library : library;

    (void)req_data;

    if (!src || !aface) return FT_Err_Invalid_Argument;

    if (src->mem_size > 0)
    {
        error = FT_New_Memory_Face(ft_library, (const FT_Byte *)src->mem_addr, src->mem_size, 0, aface);
    }
    else if (src->path)
    {
        error = FT_New_Face(ft_library, src->path, 0, aface);
    }
    else
    {
        return FT_Err_Invalid_Argument;
    }
    if (error)
    {
        return error;
    }

    return FT_Err_Ok;
}

/*
 * Every size lookup goes through here, and every one of them opens the face
 * first. That order is what keeps FTC out of a state it cannot survive.
 *
 * When the size cache is full, FTC recycles its least recently used size node
 * (ftc_size_node_reset): it frees the node's FT_Size, hands the node the new
 * face id, and only then opens the face for it. The node stays in the size
 * list the whole time (ftcmru.c uses FTC_MruNode_Up, not a remove), so it is
 * still reachable while it holds a freed FT_Size under the new face id. If
 * opening that face fails, FreeType discards every size node belonging to the
 * face id it failed on - this one included - and frees the FT_Size a second
 * time.
 *
 * Opening the face here first means the lookup FTC performs inside that window
 * can only hit the face cache, never re-open, so it cannot fail there. A face
 * that will not open is reported below instead, before any size node is
 * touched, which also turns a font file that has gone missing into a clean
 * failure rather than a fault.
 */
static FT_Error freetype_lookup_size(FTC_Scaler scaler, FT_Size *asize)
{
    FT_Face face;
    FT_Error error;

    if (!cache_manager || !scaler || !scaler->face_id) return FT_Err_Invalid_Argument;

    error = FTC_Manager_LookupFace(cache_manager, scaler->face_id, &face);
    if (error) return error;

    return FTC_Manager_LookupSize(cache_manager, scaler, asize);
}

FT_Error lv_freetype_lookup_size(FTC_Scaler scaler, FT_Size *asize)
{
    return freetype_lookup_size(scaler, asize);
}

/* The returned glyph belongs to the FTC and is only valid until the next
 * lookup - copy out anything that must survive. */
static FT_BitmapGlyph freetype_lookup_glyph(lv_freetype_font_fmt_dsc_t *dsc, uint32_t unicode_letter)
{
    FT_UInt glyph_index;
    FT_UInt charmap_index;
    FT_Error error;
    uint32_t max_box;
    FT_BitmapGlyph glyph_bitmap;
    FT_Face face;
    FT_Size face_size = NULL;
    struct FTC_ScalerRec_ scaler;
    FTC_ImageTypeRec desc_type;
    FTC_FaceID face_id = (FTC_FaceID)dsc->face_source;
    FT_Glyph image_glyph = NULL;

    if (!face_id) return NULL;

    memset(&scaler, 0, sizeof(scaler));
    scaler.face_id = face_id;
    scaler.width = dsc->font_size;
    scaler.height = dsc->font_size;
    scaler.pixel = 1;
    error = freetype_lookup_size(&scaler, &face_size);
    if (error || !face_size || !face_size->face || !face_size->face->charmap)
    {
        return NULL;
    }

    face = face_size->face;
    desc_type.face_id = face_id;
    /* FT_LOAD_NO_BITMAP: embedded bitmap strikes are typically 1bpp MONO
     * (pitch = width/8); everything downstream assumes 8bpp gray rows. */
    desc_type.flags = FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL | FT_LOAD_NO_BITMAP;
    desc_type.height = dsc->font_size;
    desc_type.width = dsc->font_size;

    charmap_index = FT_Get_Charmap_Index(face->charmap);
    glyph_index = FTC_CMapCache_Lookup(cmap_cache, face_id, charmap_index, unicode_letter);
    if (0 == glyph_index) return NULL;
    error = FTC_ImageCache_Lookup(image_cache, &desc_type, glyph_index, &image_glyph, NULL);
    if (error || !image_glyph)
    {
        return NULL;
    }
    if (image_glyph->format != FT_GLYPH_FORMAT_BITMAP)
    {
        return NULL;
    }

    glyph_bitmap = (FT_BitmapGlyph)image_glyph;
    max_box = (uint32_t)dsc->font_size * 4;
    if (((glyph_bitmap->bitmap.width || glyph_bitmap->bitmap.rows) && !glyph_bitmap->bitmap.buffer) ||
            (glyph_bitmap->bitmap.width && !glyph_bitmap->bitmap.rows) ||
            (!glyph_bitmap->bitmap.width && glyph_bitmap->bitmap.rows) ||
            glyph_bitmap->bitmap.width > max_box || glyph_bitmap->bitmap.rows > max_box)
    {
        return NULL;
    }
    if (glyph_bitmap->bitmap.width && glyph_bitmap->bitmap.pixel_mode != FT_PIXEL_MODE_GRAY)
    {
        return NULL;
    }

    return glyph_bitmap;
}

static bool get_glyph_dsc_cache_cb(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next)
{
    if (unicode_letter < 0x20)
    {
        dsc_out->adv_w = 0;
        dsc_out->box_h = 0;
        dsc_out->box_w = 0;
        dsc_out->ofs_x = 0;
        dsc_out->ofs_y = 0;
        dsc_out->bpp = 0;
        dsc_out->is_placeholder = false;
        return true;
    }

    FT_BitmapGlyph glyph_bitmap;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);

    (void)unicode_letter_next;

    if (!dsc) return false;

    /* Deferred reset_ft, see g_ftc_flush_pending. FTC_Manager_Reset also
     * drops the pinned faces, so a font file replaced on disk is picked up. */
    if (g_ftc_flush_pending)
    {
        g_ftc_flush_pending = 0;
        FTC_Manager_Reset(cache_manager);
    }

#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    sft_hash_key_t key;
    if (g_extern_cache && dsc->font_size <= g_cache_max_font_size && unicode_letter > 0xff)
    {
        key.unicode_letter = unicode_letter;
        key.font_size = dsc->font_size;
        sft_hash_node_t *node = sft_cache_get(g_cache_p, &key);
        //rt_kprintf("sft glyph cache u %x size %d %p\n", unicode_letter, dsc->font_size, node);
        if (node)
        {
            // rt_kprintf("sft get u=%04x s=%d %p\n", key.unicode_letter, key.font_size, (uint8_t*)(&node[1]));
            dsc->buf = (uint8_t *)(&node[1]);
            dsc_out->adv_w = node->adv_w;
            dsc_out->box_h = node->box_h;
            dsc_out->box_w = node->box_w;
            dsc_out->ofs_x = node->ofs_x;
            dsc_out->ofs_y = node->ofs_y;
            dsc_out->bpp = FT_BPP;
            //rt_kprintf("sft glyph cache hit!!! u %x size %d\n", unicode_letter, dsc->font_size);
            return true;
        }
        else
        {
            //rt_kprintf("sft new u=%04x s=%d\n", key.unicode_letter, key.font_size);
        }
    }
#endif

    glyph_bitmap = freetype_lookup_glyph(dsc, unicode_letter);
    if (!glyph_bitmap)
    {
        return false;
    }

    dsc_out->adv_w = (glyph_bitmap->root.advance.x >> 16);
    dsc_out->box_h = glyph_bitmap->bitmap.rows;         /*Height of the bitmap in [px]*/
    dsc_out->box_w = glyph_bitmap->bitmap.width;        /*Width of the bitmap in [px]*/
    dsc_out->ofs_x = glyph_bitmap->left;                /*X offset of the bitmap in [pf]*/
    dsc_out->ofs_y = glyph_bitmap->top - glyph_bitmap->bitmap.rows;         /*Y offset of the bitmap measured from the as line*/
    dsc_out->bpp = 8;         /*Bit per pixel: 1/2/4/8*/
    dsc_out->is_placeholder = false;

    /* Whitespace renders to nothing legitimately; any other empty glyph has
     * its ink elsewhere (an embedded bitmap, say), so report it as missing
     * and let a fallback font provide it. */
    if (!dsc_out->box_w && !dsc_out->box_h && !freetype_letter_is_whitespace(unicode_letter))
    {
        return false;
    }

    return true;                /*true: glyph found; false: glyph was not found*/
}

/* Get the bitmap of `unicode_letter` from `font`. */
#ifdef DISABLE_LVGL_V9
    static const uint8_t *get_glyph_bitmap_cache_cb(const lv_font_t *font, uint32_t unicode_letter)
#else
    static const uint8_t *get_glyph_bitmap_cache_cb(const struct _lv_font_t *font, lv_font_glyph_dsc_t *desc, uint32_t unicode_letter, uint8_t *param)
#endif
{
    FT_BitmapGlyph glyph_bitmap;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);

#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (g_extern_cache)
    {
        if (dsc->font_size <= g_cache_max_font_size && unicode_letter > 0xff) return dsc->buf;
    }
#endif

    if (!dsc) return NULL;

    /* Cache-hot re-lookup: the matching get_glyph_dsc ran moments ago. */
    glyph_bitmap = freetype_lookup_glyph(dsc, unicode_letter);
    if (!glyph_bitmap)
    {
        return NULL;
    }

    return glyph_snapshot_store(&glyph_bitmap->bitmap);
}
#else
static bool get_glyph_dsc_cb(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next)
{
    if (unicode_letter < 0x20)
    {
        dsc_out->adv_w = 0;
        dsc_out->box_h = 0;
        dsc_out->box_w = 0;
        dsc_out->ofs_x = 0;
        dsc_out->ofs_y = 0;
        dsc_out->bpp = 0;
        dsc_out->is_placeholder = false;
        return true;
    }

    int error;
    FT_Face face;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);
    face = dsc->face;

    FT_UInt glyph_index = FT_Get_Char_Index(face, unicode_letter);

    FT_Set_Pixel_Sizes(face, 0, dsc->font_size);
    error = FT_Load_Glyph(
                face,          /* handle to face object */
                glyph_index,   /* glyph index           */
                FT_LOAD_DEFAULT);   /* load flags, see below */ //FT_LOAD_MONOCHROME|FT_LOAD_NO_AUTOHINTING
    if (error)
    {
        rt_kprintf("Error in FT_Load_Glyph: %d\n", error);
        glyph_index = 0;
        goto no_face_render;
    }
    error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);

no_face_render:
    dsc_out->adv_w = (face->glyph->metrics.horiAdvance >> 6);
    dsc_out->box_h = face->glyph->bitmap.rows;         /*Height of the bitmap in [px]*/
    dsc_out->box_w = face->glyph->bitmap.width;         /*Width of the bitmap in [px]*/
    dsc_out->ofs_x = face->glyph->bitmap_left;         /*X offset of the bitmap in [pf]*/
    dsc_out->ofs_y = face->glyph->bitmap_top - face->glyph->bitmap.rows;         /*Y offset of the bitmap measured from the as line*/
    dsc_out->bpp = FT_BPP;         /*Bit per pixel: 1/2/4/8*/
    dsc_out->is_placeholder = false;

    if (0 == glyph_index || error) return false;


    return true;
}

/* Get the bitmap of `unicode_letter` from `font`. */
static const uint8_t *get_glyph_bitmap_cb(const lv_font_t *font, uint32_t unicode_letter)
{
    //FT_Face face;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);
#if 0
    face = dsc->face;
    return (const uint8_t *)(face->glyph->bitmap.buffer);
#else
    return (const uint8_t *)(dsc->buf);
#endif
}
#endif //USE_CACHE_MANGER

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
* init freetype library
* @param max_faces Maximum number of opened @FT_Face objects managed by this cache
* @param max_sizes Maximum number of opened @FT_Size objects managed by this cache
* @return FT_Error
* example: if you have two faces,max_faces should >= 2
*/
int lv_freetype_init(uint16_t max_faces, uint16_t max_sizes, uint32_t max_cache_size)
{
    FT_Error error;
    error = FT_Init_FreeType(&library);
    if (error)
    {
        rt_kprintf("Error in FT_Init_FreeType: %d\n", error);
        return error;
    }
#if USE_CACHE_MANGER
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (g_extern_cache)
    {
        uint32_t sec_cache_size = MAX_UNICODE_CACHED_NUMBER * (32 + sizeof(sft_hash_node_t) + ((g_cache_max_font_size * g_cache_max_font_size * g_bpp  + 7) >> 3));
        sec_cache_size += (SFT_HASH_TABLE_NUMBER * sizeof(sft_hash_node_t)) + sizeof(sft_cache_t);
        if (sec_cache_size < max_cache_size)
        {
            max_cache_size = max_cache_size - sec_cache_size;// * 120 / 100;
        }

        rt_kprintf("lv_freetype_init: extern_cache exist %d sec %d\n", max_cache_size, sec_cache_size);
    }
#endif

    //cache
    error = FTC_Manager_New(library, max_faces, max_sizes, max_cache_size, font_Face_Requester, NULL, &cache_manager);
    if (error)
    {
        rt_kprintf("Failed to open cache manager\n");
        return error;
    }

    error = FTC_CMapCache_New(cache_manager, &cmap_cache);
    if (error)
    {
        rt_kprintf("Failed to open Cmap Cache\n");
        return error;
    }
    error = FTC_ImageCache_New(cache_manager, &image_cache);
    if (error)
    {
        rt_kprintf("Failed to open image cache\n");
        return error;
    }
#endif

    return FT_Err_Ok;
}


/**
* init lv_font_t struct
* @param font pointer to a font
* @param font_path the font path
* @param font_size the height of font
* @return FT_Error
*/
int lv_freetype_font_init(lv_font_t *font, const char *font_lib_addr, int font_lib_size, uint16_t font_size, const char *font_name)
{

#ifndef LV_USE_USER_DATA
#error "lv_freetype : user_data is required.Enable it lv_conf.h(LV_USE_USER_DATA 1)"
#endif

    FT_Error error;

    lv_freetype_font_fmt_dsc_t *dsc = rt_malloc(sizeof(lv_freetype_font_fmt_dsc_t));
    //LV_ASSERT_MEM(dsc);
    if (dsc == NULL) return FT_Err_Out_Of_Memory;

    dsc->font_size = font_size;
    dsc->font_lib_addr = font_lib_addr;
    dsc->font_lib_size = font_lib_size;
    dsc->buf = NULL;
    dsc->face = NULL;
    dsc->face_source = NULL;

#if USE_CACHE_MANGER
    /* Rendering goes through the FTC anyway, and a private FT_New_Face here
     * would cost another fd and a full parse on top of the cached faces. */
    if (cache_manager)
    {
        struct FTC_ScalerRec_ scaler;
        FT_Size face_size = NULL;

        dsc->face_source = freetype_face_source_acquire(font_lib_addr, font_lib_size);
        if (!dsc->face_source)
        {
            rt_free(dsc);
            return FT_Err_Out_Of_Memory;
        }

        memset(&scaler, 0, sizeof(scaler));
        scaler.face_id = (FTC_FaceID)dsc->face_source;
        scaler.width = font_size;
        scaler.height = font_size;
        scaler.pixel = 1;
        error = freetype_lookup_size(&scaler, &face_size);
        if (error || !face_size)
        {
            rt_kprintf("Error in freetype_lookup_size: %d\n", error);
            if (error == FT_Err_Unknown_File_Format)
            {
                /* ftmodule.h registers the TrueType driver only. */
                rt_kprintf("  unsupported outline format: convert CFF/OTF fonts to TrueType\n");
            }
            freetype_face_source_release(dsc->face_source);
            rt_free(dsc);
            return error ? error : FT_Err_Cannot_Open_Resource;
        }

        font->line_height = (face_size->metrics.height >> 6);
        font->base_line = -(face_size->metrics.descender >> 6) + 4;  /*Base line measured from the top of line_height*/
    }
    else
#endif
    {
        //font_lib_size > 0, for font_lib data
        if (font_lib_size > 0)
        {
            error = FT_New_Memory_Face(library, (const FT_Byte *)font_lib_addr, font_lib_size, 0, &dsc->face);
        }
        else //font file of file-system
        {
            error = FT_New_Face(library, (font_lib_addr), 0, &dsc->face);
        }

        if (error)
        {
            rt_kprintf("Error in FT_New_Face: %d\n", error);
            rt_free(dsc);
            return error;
        }

        error = FT_Set_Pixel_Sizes(dsc->face, 0, font_size);
        if (error)
        {
            rt_kprintf("Error in FT_Set_Char_Size: %d\n", error);
            FT_Done_Face(dsc->face);
            rt_free(dsc);
            return error;
        }

        font->line_height = (dsc->face->size->metrics.height >> 6);
        font->base_line = -(dsc->face->size->metrics.descender >> 6) + 4;  /*Base line measured from the top of line_height*/
    }

#if USE_CACHE_MANGER
    font->get_glyph_dsc = get_glyph_dsc_cache_cb;        /*Set a callback to get info about gylphs*/
    font->get_glyph_bitmap = get_glyph_bitmap_cache_cb;  /*Set a callback to get bitmap of a glyp*/
#else
    font->get_glyph_dsc = get_glyph_dsc_cb;        /*Set a callback to get info about gylphs*/
    font->get_glyph_bitmap = get_glyph_bitmap_cb;  /*Set a callback to get bitmap of a glyp*/
#endif

    font->user_data = dsc;
    font->subpx = LV_FONT_SUBPX_NONE;

    font->font_lib_size = font_lib_size;
    font->font_lib_data = font_lib_addr;
    font->font_name = font_name;
    font->fallback = NULL;
    return FT_Err_Ok;
}

void lv_freetype_font_deinit(lv_font_t *font)
{
    lv_freetype_font_fmt_dsc_t *dsc;

    if (!font) return;

    dsc = (lv_freetype_font_fmt_dsc_t *)font->user_data;
    if (!dsc) return;

#if USE_CACHE_MANGER
    freetype_face_source_release((freetype_face_source_t *)dsc->face_source);
    dsc->face_source = NULL;
#endif

    if (dsc->face)
    {
        FT_Done_Face(dsc->face);
        dsc->face = NULL;
    }

    rt_free(dsc);
    font->user_data = NULL;
}

extern void lvsf_font_inital(uint32_t cache_size, bool init);
extern uint32_t ft_get_cache_size(void);
int lvsf_font_deinit(void);


void lv_freetype_open_font(bool init)
{
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (g_extern_cache)
    {
        //must called before lvsf_font_inital()-->lv_freetype_font_init()
        g_cache_p = sft_cache_init(MAX_UNICODE_CACHED_NUMBER);
        RT_ASSERT(g_cache_p);
    }
#endif
    lvsf_font_inital(ft_get_cache_size(), init);
}

void lv_freetype_close_font(void)
{
    rt_kprintf("lv_freetype_close_font\n");

    /* Font objects that are still displayed keep their faces in the cache;
     * tearing the engine down under them would leave them unusable. */
    if (lvsf_font_deinit() != 0)
    {
        return;
    }

#if USE_CACHE_MANGER
    if (cache_manager) FTC_Manager_Done(cache_manager);
    cache_manager = NULL;
#endif

    if (library) FT_Done_FreeType(library);
    library = NULL;

#if 0
    if (g_extern_cache)
    {
        if (g_cache_p) sft_cache_deinit(g_cache_p);
        g_cache_p = NULL;
    }
#endif
}

void lv_freetype_clean_cache(uint8_t clean_type)
{
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (FT_CACHE_QUAD_CLEAN ==  clean_type)
    {
        sft_cache_delete_old(g_cache_p, g_cache_p->cached_number >> 2);
    }
    else if (FT_CACHE_HALF_CLEAN ==  clean_type)
    {
        sft_cache_delete_old(g_cache_p, g_cache_p->cached_number >> 1);
    }
    else //FT_CACHE_WHOLE_CLEAN
    {
        sft_cache_delete_all(g_cache_p);
    }
#endif

    //rt_kprintf("ft_clean: %d\n", clean_type);
#if USE_CACHE_MANGER
    //extern void list_mem();
    //list_mem();
    if (FT_CACHE_QUAD_CLEAN ==  clean_type)
    {
        FTC_Manager_Cache_Free(cache_manager, ft_get_cache_size() >> 2);
    }
    else if (FT_CACHE_HALF_CLEAN ==  clean_type)
    {
        FTC_Manager_Cache_Free(cache_manager, ft_get_cache_size() >> 1);
    }
    else //FT_CACHE_WHOLE_CLEAN
    {
        FTC_Manager_Cache_Free(cache_manager, 0);
    }

#endif //USE_CACHE_MANGER
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int lv_freetype_test(void)
{
    /*
     * Only schedule a cache flush: font objects created through the manager
     * may still be referenced by live LVGL styles, and this command runs on
     * the finsh shell thread while FTC has no locking against the rendering
     * thread's lookups - get_glyph_dsc_cache_cb performs the flush before
     * the next lookup.
     */
#if USE_CACHE_MANGER
    if (!cache_manager) return 0;
    g_ftc_flush_pending = 1;
    rt_kprintf("reset_ft: freetype cache flush scheduled (applies at next glyph render)\n");
#endif
    return 0;
}
MSH_CMD_EXPORT_ALIAS(lv_freetype_test, reset_ft, reset_ft: flush freetype glyph cache);

#endif
#endif

void lv_freetype_set_parameter(uint16_t bpp, uint16_t cache_max_font_size, lv_freetype_extern_cache_t extern_cache)
{
    g_bpp = bpp;
    g_cache_max_font_size = cache_max_font_size;
#if 0
    g_extern_cache = extern_cache;
#endif
    rt_kprintf("lv_freetype_set_parameter: bpp %d max_fsize %d extern %d\n", bpp, cache_max_font_size, extern_cache);
}
