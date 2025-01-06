/**
 * @file lvsf_page.c
 *
 *
 * an header consist of: back_button + title + indicate icons + time clock
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvsf.h"

#if LVSF_USE_IDXIMG!=0

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfidximg_class


/**********************
 *      TYPEDEFS
 **********************/
/*struct of idximg*/
typedef struct
{
    lv_img_t img; /*Ext. of ancestor*/
    /*Ext. of ancestor*/
    lv_img_dsc_t **dsc_array;
    uint16_t size;
    char prefix[LVSF_IDXIMG_PREFIX_LEN];            /*<! Prefix for the image file path*/
} lv_idximg_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfidximg_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfidximg_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);

/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfidximg_class =
{
    .constructor_cb = lv_lvsfidximg_constructor,
    .destructor_cb = lv_lvsfidximg_destructor,
    .instance_size = sizeof(lv_idximg_t),
    .base_class = &lv_img_class
};

/**********************
 *      MACROS
 **********************/


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfidximg_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
}

static void lv_lvsfidximg_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Create Indexed image objects
 * @param par pointer to an object, it will be the parent of the new weather
 * @param copy pointer to a weather object, if not NULL then the new object will be copied from it
 * @return pointer to the created weather
 */
lv_obj_t *lv_idximg_create(lv_obj_t *par)
{
    LV_LOG_TRACE("idximg create started");

    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, par);
    lv_obj_class_init_obj(obj);

    return obj;

}

void lv_idximg_src_array(lv_obj_t *idximg, const lv_img_dsc_t **dsc_array, uint16_t size)
{
    RT_ASSERT(*dsc_array);
    lv_idximg_t *ext = (lv_idximg_t *)idximg;
    ext->dsc_array = (lv_img_dsc_t **)dsc_array;
    ext->size = size;
}

void lv_idximg_prefix(lv_obj_t *idximg, const char *prefix)
{
    RT_ASSERT(strlen(prefix) <= LVSF_IDXIMG_PREFIX_LEN);
    lv_idximg_t *ext = (lv_idximg_t *)idximg;
    strcpy(ext->prefix, prefix);
}

void lv_idximg_select(lv_obj_t *idximg, uint16_t index)
{
    RT_ASSERT(index < 100);
    lv_idximg_t *ext = (lv_idximg_t *)idximg;

    if (ext->size)
    {
        RT_ASSERT(index < ext->size);
        lv_img_set_src(idximg, ext->dsc_array[index]);
    }
    else
    {
        static char img_path[LVSF_IDXIMG_PREFIX_LEN + 6];
        rt_sprintf(img_path, "%s%02d.bin", ext->prefix, index);
        lv_img_set_src(idximg, img_path);
    }
}


#endif
