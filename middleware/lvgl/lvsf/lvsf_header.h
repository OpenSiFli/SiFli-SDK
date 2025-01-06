#ifndef LVSF_HEADER_H
#define LVSF_HEADER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_HEADER != 0

/*Testing of dependencies*/
#if LV_USE_BTN == 0
#error "lvsf_header: lv_btn is required. Enable it in lv_conf.h (LV_USE_BTN  1) "
#endif

#if LV_USE_LABEL == 0
#error "lvsf_header: lv_label is required. Enable it in lv_conf.h (LV_USE_LABEL  1) "
#endif

#if LV_USE_IMG == 0
#error "lvsf_header: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
/** Header items. */
enum
{
    LVSF_HEADER_BACK_BTN = 0x01, /**< Header's back button. */
    LVSF_HEADER_TITLE    = 0x02, /**< Header's title label. */
    LVSF_HEADER_ICONS    = 0x04, /**< Header's icons. */
    LVSF_HEADER_TIME     = 0x08, /**< Header's time label. */

    LVSF_HEADER_ROOT  = (LVSF_HEADER_TITLE | LVSF_HEADER_ICONS | LVSF_HEADER_TIME),
    LVSF_HEADER_BRANCH  = (LVSF_HEADER_BACK_BTN | LVSF_HEADER_TITLE | LVSF_HEADER_ICONS | LVSF_HEADER_TIME),
    LVSF_HEADER_DEFAULT  = LVSF_HEADER_BRANCH,
};
typedef uint8_t lv_lvsfheader_item_t;

extern const lv_obj_class_t lv_lvsfheader_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a header objects
 * @param par pointer to an object, it will be the parent of the new header
 * @return pointer to the created header
 */
lv_obj_t *lv_lvsfheader_create(lv_obj_t *par);


/**
 * Set a an event handler function for back button.
 * @param win pointer to a header object
 * @param cb the new event function
 */
void lv_lvsfheader_back_event_cb(lv_obj_t *hdr, lv_event_cb_t cb);

/**
 * Set the title of a header
 * @param head pointer to a header object
 * @param title string of the new title
 */
void lv_lvsfheader_set_title(lv_obj_t *hdr, const char *title);

/**
 * configurate visible item
 * @param head pointer to a header object
 * @param visible item mask
 */
void lv_lvsfheader_set_visible_item(lv_obj_t *hdr, lv_lvsfheader_item_t flag);


/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_HEADER*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_HEADER_H*/

