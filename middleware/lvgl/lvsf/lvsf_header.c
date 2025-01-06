/**
 * @file lvsf_header.c
 *
 *
 * an header consist of: back_button + title + indicate icons + time clock
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"
#include "time.h"
#if LVSF_USE_HEADER != 0
/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfheader_class

#define LVSF_HEADER_HEIGHT_DEF (LV_DPI_DEF/4)

/**********************
 *      TYPEDEFS
 **********************/

/*Data of header*/
typedef struct
{
    /*Ext. of ancestor*/
    /*New data for this type */
    lv_obj_t base;
    lv_obj_t *back_btn;                 /*Pointer to the back button of the header*/
    lv_obj_t *title;                  /*Pointer to the title label of the header*/
    lv_obj_t *icons;                  /*Pointer to the container of icons of the header*/
    lv_obj_t *time;                  /*Pointer to the time label of the header*/

    lv_lvsfheader_item_t visible;  /*Visible items on header*/
} lv_lvsfheader_ext_t;

/** Header styles. */
enum
{
    LVSF_HEADER_STYLE_BG, /**< Header object background style. */
    LVSF_HEADER_STYLE_TITLE, /**< Header title style. */
    LVSF_HEADER_STYLE_ICONS, /**< Header icons style. */
    LVSF_HEADER_STYLE_TIME, /**< Header time style. */
    LVSF_HEADER_STYLE_BTN_REL, /**< Same meaning as ordinary button styles. */
    LVSF_HEADER_STYLE_BTN_PR,
};
typedef uint8_t lv_lvsfheader_style_t;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfheader_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfheader_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfheader_realign(lv_obj_t *hdr);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfheader_class =
{
    .constructor_cb = lv_lvsfheader_constructor,
    .destructor_cb = lv_lvsfheader_destructor,
    .instance_size = sizeof(lv_lvsfheader_ext_t),
    .base_class = &lv_obj_class
};


/**********************
 *      MACROS
 **********************/
/**********************
 *   STATIC FUNCTIONS
 **********************/
static void get_time_string(char *str)
{
    struct tm *time_info;

#ifdef WIN32
    __time32_t raw_time;
    _time32(&raw_time);
    time_info = _localtime32(&raw_time);
#else
    time_t raw_time;
    time(&raw_time);
    time_info = localtime(&raw_time);
#endif

    lv_snprintf(str, 5, "%02d:%02d", time_info->tm_hour, time_info->tm_min);
}


static void lv_lvsfheader_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_lvsfheader_ext_t *new_header = (lv_lvsfheader_ext_t *)obj;

    /*Create a back button for the header*/
    new_header->back_btn = lv_btn_create(obj);
    lv_obj_t *back_btn_img = lv_label_create(new_header->back_btn);
    lv_label_set_text(back_btn_img, " < ");
    lv_obj_add_flag(new_header->back_btn, LV_OBJ_FLAG_CLICKABLE);

    /*Create a title on the header*/
    new_header->title = lv_label_create(obj);
    lv_label_set_text(new_header->title, "My title");


    /*Create a icons container on the header*/
    new_header->icons = lv_img_create(obj);
    lv_obj_set_style_text_font(new_header->icons, LV_FONT_DEFAULT, LV_STATE_DEFAULT | LV_PART_MAIN);
    lv_img_set_src(new_header->icons, LV_SYMBOL_BLUETOOTH);

    /*Create a time label on the header*/
    new_header->time = lv_label_create(obj);
    char time_str[8];
    memset(time_str, 0, sizeof(time_str));
    get_time_string(time_str);
    lv_label_set_text(new_header->time, time_str);

    new_header->visible       = 0;
}

static void lv_lvsfheader_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

}


/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Create a header objects
 * @param par pointer to an object, it will be the parent of the new header
 * @return pointer to the created header
 */
lv_obj_t *lv_lvsfheader_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, lv_obj_get_content_width(lv_obj_get_parent(obj)),
                    LVSF_HEADER_HEIGHT_DEF);
    lv_obj_set_pos(obj, 0, 0);
    return obj;
}

/*=====================
 * Setter functions
 *====================*/

/**
 * Set a an event handler function for back button.
 * @param win pointer to a header object
 * @param cb the new event function
 */
void lv_lvsfheader_back_event_cb(lv_obj_t *hdr, lv_event_cb_t cb)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfheader_ext_t *ext = (lv_lvsfheader_ext_t *)hdr;

    lv_obj_add_event_cb(ext->back_btn, cb, LV_EVENT_ALL, NULL);
}

/**
 * Set the title of a header
 * @param win pointer to a header object
 * @param title string of the new title
 */
void lv_lvsfheader_set_title(lv_obj_t *hdr, const char *title)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfheader_ext_t *ext = (lv_lvsfheader_ext_t *)hdr;

    lv_label_set_text(ext->title, title);
}

/**
 * configurate visible item
 * @param head pointer to a header object
 * @param visible item mask
 */
void lv_lvsfheader_set_visible_item(lv_obj_t *hdr, lv_lvsfheader_item_t flag)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfheader_ext_t *ext = (lv_lvsfheader_ext_t *)hdr;

    if (ext->visible != flag)
    {
        ext->visible = flag;

        if (0 == (flag & LVSF_HEADER_BACK_BTN))
            lv_obj_add_flag(ext->back_btn, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(ext->back_btn, LV_OBJ_FLAG_HIDDEN);

        if (0 == (flag & LVSF_HEADER_TITLE))
            lv_obj_add_flag(ext->title, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(ext->title, LV_OBJ_FLAG_HIDDEN);

        if (0 == (flag & LVSF_HEADER_ICONS))
            lv_obj_add_flag(ext->icons, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(ext->icons, LV_OBJ_FLAG_HIDDEN);

        if (0 == (flag & LVSF_HEADER_TIME))
            lv_obj_add_flag(ext->time, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(ext->time, LV_OBJ_FLAG_HIDDEN);

    }
    lv_lvsfheader_realign(hdr);
}

/*=====================
 * Getter functions
 *====================*/

/**
 * Get the title of a header
 * @param win pointer to a header object
 * @return title string of the header
 */
const char *lv_lvsfheader_get_title(const lv_obj_t *hdr)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfheader_ext_t *ext = (lv_lvsfheader_ext_t *)hdr;
    return lv_label_get_text(ext->title);
}

/*=====================
 * Other functions
 *====================*/


/**
 * Realign the building elements of a header
 * @param win pointer to header objectker
 */
static void lv_lvsfheader_realign(lv_obj_t *hdr)
{
    lv_lvsfheader_ext_t *ext = (lv_lvsfheader_ext_t *)hdr;

    lv_coord_t left = lv_obj_get_style_pad_left(hdr, LV_PART_MAIN);
    lv_coord_t right = lv_obj_get_style_pad_right(hdr, LV_PART_MAIN);
    lv_coord_t inner = lv_obj_get_style_pad_top(hdr, LV_PART_MAIN);

    if (ext->back_btn == NULL && ext->title == NULL && ext->icons == NULL && ext->time == NULL) return;

    if (ext->visible & LVSF_HEADER_BACK_BTN)
        lv_obj_align(ext->back_btn, LV_ALIGN_LEFT_MID, left, 0);
    if (ext->visible & LVSF_HEADER_TIME)
        lv_obj_align(ext->time, LV_ALIGN_RIGHT_MID, -right, 0);

    if (ext->visible & LVSF_HEADER_TITLE)
        lv_obj_align(ext->title, LV_ALIGN_CENTER, left, 0);

    if (ext->visible & LVSF_HEADER_ICONS)
        lv_obj_align_to(ext->icons, ext->time, LV_ALIGN_OUT_LEFT_MID, inner, 0);
}

#endif

