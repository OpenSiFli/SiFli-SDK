/**
 * @file lv_freetype.c
 *
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

FT_Library library;
static uint16_t g_bpp = FT_BPP;
static uint16_t g_cache_max_font_size = FONT_SUBTITLE;
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    static bool     g_extern_cache = false;
#endif

#if USE_CACHE_MANGER
static FTC_Manager cache_manager;
static FTC_CMapCache cmap_cache;
/*static FTC_ImageCache image_cache;*/
static FTC_SBitCache sbit_cache;
static FTC_SBit sbit;

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

static FT_Error  font_Face_Requester(FTC_FaceID  face_id,
                                     FT_Library  library,
                                     FT_Pointer  req_data,
                                     FT_Face    *aface)
{
    *aface = face_id;

    return FT_Err_Ok;
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
        return true;
    }

    FT_UInt glyph_index;
    FT_UInt charmap_index;
    FT_Face face;
    lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);
    face = dsc->face;

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

    FTC_ImageTypeRec desc_sbit_type;

    desc_sbit_type.face_id = (FTC_FaceID)face;
    desc_sbit_type.flags = FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL;
    desc_sbit_type.height = dsc->font_size;
    desc_sbit_type.width = 0;

    /*FTC_Manager_LookupFace(cache_manager, face, &get_face);*/
    charmap_index = FT_Get_Charmap_Index(face->charmap);
    glyph_index = FTC_CMapCache_Lookup(cmap_cache, face, charmap_index, unicode_letter);
    if (0 == glyph_index) return false;
    FTC_SBitCache_Lookup(sbit_cache, &desc_sbit_type, glyph_index, &sbit, NULL);

    dsc_out->adv_w = sbit->xadvance;
    dsc_out->box_h = sbit->height;          /*Height of the bitmap in [px]*/
    dsc_out->box_w = sbit->width;           /*Width of the bitmap in [px]*/
    dsc_out->ofs_x = sbit->left;            /*X offset of the bitmap in [pf]*/
    dsc_out->ofs_y = sbit->top - sbit->height;         /*Y offset of the bitmap measured from the as line*/
#if 1
    dsc_out->bpp = FT_BPP;         /*Bit per pixel: 1/2/4/8*/
#else
    dsc_out->bpp = 8;         /*Bit per pixel: 1/2/4/8*/
#endif

    //if((dsc_out->box_h == 0) && (dsc_out->box_w == 0)) return false;

#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (g_extern_cache && dsc->font_size <= g_cache_max_font_size && unicode_letter > 0xff)
    {
        if (sbit->buffer)
        {
            int size = (sbit->height * sbit->width * FT_BPP + 7) >> 3;
            sft_hash_node_t *node1 = sft_cache_alloc(g_cache_p, &key, size);
            RT_ASSERT(node1);
            //rt_kprintf("sft set u=%04x s=%d %p\n", key.unicode_letter, key.font_size, (uint8_t *)&node1[1]);
            node1->adv_w = dsc_out->adv_w;
            node1->box_h = dsc_out->box_h;
            node1->box_w = dsc_out->box_w;
            node1->ofs_x = dsc_out->ofs_x;
            node1->ofs_y = dsc_out->ofs_y;
            memcpy((uint8_t *)&node1[1], sbit->buffer, size);
            sft_cache_set(g_cache_p, node1);
            dsc->buf = (uint8_t *)&node1[1];
        }
        else
        {
            dsc->buf = NULL;
        }
    }
#endif

    return true;                /*true: glyph found; false: glyph was not found*/
}

/* Get the bitmap of `unicode_letter` from `font`. */
#ifdef DISABLE_LVGL_V9
    static const uint8_t *get_glyph_bitmap_cache_cb(const lv_font_t *font, uint32_t unicode_letter)
#else
    static const uint8_t *get_glyph_bitmap_cache_cb(const struct _lv_font_t *font, lv_font_glyph_dsc_t *desc, uint32_t unicode_letter, uint8_t *param)
#endif
{
#if 0//def FREETYPE_EXTERN_CACHE_AGAIN
    if (g_extern_cache)
    {
        lv_freetype_font_fmt_dsc_t *dsc = (lv_freetype_font_fmt_dsc_t *)(font->user_data);
        if (dsc->font_size <= g_cache_max_font_size && unicode_letter > 0xff) return dsc->buf;
    }
#endif

    return (const uint8_t *)sbit->buffer;
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
* @return FT_Error
* example: if you have two faces,max_faces should >= 2
*/
int lv_freetype_init(uint8_t max_faces, uint32_t max_cache_size)
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
    error = FTC_Manager_New(library, max_faces, 1, max_cache_size, font_Face_Requester, NULL, &cache_manager);
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
    /*
    error = FTC_ImageCache_New(cache_manager, &image_cache);
    if(error)
    {
        printf("Failed to open image cache\n");
        return error;
    }
    */
    error = FTC_SBitCache_New(cache_manager, &sbit_cache);
    if (error)
    {
        rt_kprintf("Failed to open sbit cache\n");
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
        return error;
    }



    error = FT_Set_Pixel_Sizes(dsc->face, 0, font_size);
    if (error)
    {
        rt_kprintf("Error in FT_Set_Char_Size: %d\n", error);
        return error;
    }

#if USE_CACHE_MANGER
    font->get_glyph_dsc = get_glyph_dsc_cache_cb;        /*Set a callback to get info about gylphs*/
    font->get_glyph_bitmap = get_glyph_bitmap_cache_cb;  /*Set a callback to get bitmap of a glyp*/
#else
    font->get_glyph_dsc = get_glyph_dsc_cb;        /*Set a callback to get info about gylphs*/
    font->get_glyph_bitmap = get_glyph_bitmap_cb;  /*Set a callback to get bitmap of a glyp*/
#endif

    font->user_data = dsc;
    font->line_height = (dsc->face->size->metrics.height >> 6);
    font->base_line = -(dsc->face->size->metrics.descender >> 6) + 4;  /*Base line measured from the top of line_height*/
    font->subpx = LV_FONT_SUBPX_NONE;

    font->font_lib_size = font_lib_size;
    font->font_lib_data = font_lib_addr;
    font->font_name = font_name;
    font->fallback = NULL;
    return FT_Err_Ok;
}

extern void lvsf_font_inital(uint32_t cache_size, bool init);
extern uint32_t ft_get_cache_size(void);
void lvsf_font_deinit(void);


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
    lvsf_font_deinit();

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


FTC_Manager lv_freetype_get_cache_manager(void)
{
#if USE_CACHE_MANGER
    return cache_manager;
#endif
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int lv_freetype_test(void)
{
    lv_freetype_close_font();
    lv_freetype_open_font(true);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(lv_freetype_test, reset_ft, reset_ft: close and re - open freetype test);

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

