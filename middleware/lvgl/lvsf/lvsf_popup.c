/**
 * @file lvsf_popup.c
 *
 *
 * an popup consist of: full screen gray background + app_icon + lable + 2 button
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"

#if LVSF_USE_POPUP != 0

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_lvsfpopup_class

#define LVSF_POPUP_WIDTH_DEF            (LV_HOR_RES * 4 / 5)
#define LVSF_POPUP_TITLE_ICON_WIDTH_DEF (LVSF_POPUP_WIDTH_DEF / 4)
#define LVSF_POPUP_TITLE_WIDTH_DEF      (LVSF_POPUP_WIDTH_DEF - LVSF_POPUP_TITLE_ICON_WIDTH_DEF)
#define LVSF_POPUP_BUTTON_WIDTH_DEF     (LVSF_POPUP_WIDTH_DEF / 2)


#define LVSF_POPUP_HEIGHT_DEF          (LV_VER_RES * 3 / 5)
#define LVSF_POPUP_TITLE_HEIGHT_DEF    (LVSF_POPUP_HEIGHT_DEF / 4)
#define LVSF_POPUP_CONTENT_HEIGHT_DEF  (LVSF_POPUP_HEIGHT_DEF / 2)
#define LVSF_POPUP_BUTTON_HEIGHT_DEF   (LVSF_POPUP_HEIGHT_DEF / 4)

/**********************
 *      TYPEDEFS
 **********************/


/*Data of popup*/
typedef struct
{
    /*Ext. of ancestor*/
    lv_obj_t obj_ext;
    /*New data for this type */
    lv_obj_t *title;                  /*Pointer to the title label of the popup*/
    lv_obj_t *title_icon;             /*Pointer to the title icons of the popup*/
    lv_obj_t *content;                /*Pointer to the content label of the popup*/
    lv_obj_t *confirm_btn;            /*Pointer to the confirm button of the popup*/
    lv_obj_t *confirm_btn_txt;        /*Pointer to the confirm button label of the popup*/
    lv_obj_t *confirm_btn_img;        /*Pointer to the confirm button image of the popup*/
    lv_obj_t *cancel_btn;            /*Pointer to the cancel button of the popup*/
    lv_obj_t *cancel_btn_txt;        /*Pointer to the cancel button label of the popup*/
    lv_obj_t *cancel_btn_img;        /*Pointer to the cancel button image of the popup*/
} lv_lvsfpopup_ext_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_lvsfpopup_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfpopup_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_lvsfpopup_realign(lv_obj_t *hdr);


/**********************
 *  STATIC VARIABLES
 **********************/

const lv_obj_class_t lv_lvsfpopup_class =
{
    .constructor_cb = lv_lvsfpopup_constructor,
    .destructor_cb = lv_lvsfpopup_destructor,
    .instance_size = sizeof(lv_lvsfpopup_ext_t),
    .base_class = &lv_obj_class
};


/**********************
 *      MACROS
 **********************/
/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_lvsfpopup_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

    lv_obj_refresh_style(obj, 0, LV_STYLE_PROP_ALL);
}

static void lv_lvsfpopup_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);

}


static void confirm_btn_event_callback(lv_event_t *event)
{
    if (LV_EVENT_CLICKED == event->code)
    {
        lv_obj_t *obj = (lv_obj_t *)lv_event_get_user_data(event);
        lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)obj;

#ifdef DISABLE_LVGL_V9
        lv_event_send(obj, LV_EVENT_READY, NULL);
#else
        lv_obj_send_event(obj, LV_EVENT_READY, NULL);
#endif
        lv_obj_del(obj);
    }
}

static void cancel_btn_event_callback(lv_event_t *event)
{

    if (LV_EVENT_CLICKED == event->code)
    {
        lv_obj_t *obj = lv_event_get_user_data(event);

#ifdef DISABLE_LVGL_V9
        lv_event_send(obj, LV_EVENT_CANCEL, NULL);
#else
        lv_obj_send_event(obj, LV_EVENT_CANCEL, NULL);
#endif
        lv_obj_del(obj);
    }

}


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Create a popup objects
 * @param par pointer to an object, it will be the parent of the new popup
 * @return pointer to the created popup
 */
lv_obj_t *lv_lvsfpopup_create(lv_obj_t *parent)
{
    LV_LOG_TRACE("popup create started");

    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)obj;

    lv_obj_class_init_obj(obj);
    lv_obj_set_size(obj, LVSF_POPUP_WIDTH_DEF, LVSF_POPUP_HEIGHT_DEF);
    lv_obj_set_style_bg_opa(obj, LV_OPA_40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_gap(obj, LV_HOR_RES / 30, LV_PART_MAIN | LV_STATE_DEFAULT);

    /*Create a title on the popup*/
    ext->title = lv_label_create(obj);
    lv_label_set_text(ext->title, "No title");
    lv_obj_set_size(ext->title, LVSF_POPUP_TITLE_WIDTH_DEF, LVSF_POPUP_TITLE_HEIGHT_DEF);

    ext->content = lv_label_create(obj);
    lv_label_set_text(ext->content, "No content");
    lv_obj_set_size(ext->content, LVSF_POPUP_WIDTH_DEF, LVSF_POPUP_CONTENT_HEIGHT_DEF);

    /*Create a confirm button for the popup*/
    ext->confirm_btn = lv_btn_create(obj);
    lv_obj_set_size(ext->confirm_btn, LVSF_POPUP_BUTTON_WIDTH_DEF, LVSF_POPUP_BUTTON_HEIGHT_DEF);
    lv_obj_add_event_cb(ext->confirm_btn, confirm_btn_event_callback, LV_EVENT_ALL, (void *)obj);

    /*Create a cancel button for the popup*/
    ext->cancel_btn = lv_btn_create(obj);
    lv_obj_set_size(ext->cancel_btn, LVSF_POPUP_BUTTON_WIDTH_DEF, LVSF_POPUP_BUTTON_HEIGHT_DEF);
    lv_obj_add_event_cb(ext->cancel_btn, cancel_btn_event_callback, LV_EVENT_ALL, (void *)obj);
    return obj;
}

/*=====================
 * Setter functions
 *====================*/

/**
 * Set the title of a popup
 * @param win pointer to a popup object
 * @param title string of the new title
 */
void lv_lvsfpopup_set_title(lv_obj_t *hdr, const char *title)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;

    if (ext->title == NULL)
        ext->title = lv_label_create(hdr);
    lv_label_set_text(ext->title, title);
    lv_lvsfpopup_realign(hdr);
}

void lv_lvsfpopup_set_icon(lv_obj_t *hdr, const void *img)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);
    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;

    ext->title_icon = lv_img_create(hdr);
    lv_img_set_src(ext->title_icon, img);
    {
        lv_img_t *img_ext;
        uint16_t zoom;
        int32_t img_r, area_r;


        img_ext = (lv_img_t *)ext->title_icon;


        img_r = LV_MAX(img_ext->w, img_ext->h);
        area_r = LV_MIN(LVSF_POPUP_TITLE_ICON_WIDTH_DEF, LVSF_POPUP_TITLE_HEIGHT_DEF);

        zoom = area_r * 256 / img_r;
        lv_img_set_zoom(ext->title_icon, zoom);
        lv_obj_update_layout(ext->title_icon);
    }
    lv_lvsfpopup_realign(hdr);
}



void lv_lvsfpopup_set_content(lv_obj_t *hdr, const char *content)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;

    if (ext->content == NULL)
        ext->content = lv_label_create(hdr);
    lv_label_set_text(ext->content, content);
    lv_obj_set_size(ext->content, LVSF_POPUP_WIDTH_DEF, LVSF_POPUP_CONTENT_HEIGHT_DEF);
    lv_lvsfpopup_realign(hdr);
}

void lv_lvsfpopup_set_confirm_btn_txt(lv_obj_t *hdr, const char *txt)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;
    if (ext->confirm_btn_txt == NULL)
        ext->confirm_btn_txt = lv_label_create(ext->confirm_btn);
    lv_label_set_text(ext->confirm_btn_txt, txt);

    lv_lvsfpopup_realign(hdr);
}

void lv_lvsfpopup_set_cancel_btn_txt(lv_obj_t *hdr, const char *txt)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;
    if (ext->cancel_btn_txt == NULL)
        ext->cancel_btn_txt = lv_label_create(ext->cancel_btn);
    lv_label_set_text(ext->cancel_btn_txt, txt);

    lv_lvsfpopup_realign(hdr);
}


/*=====================
 * Getter functions
 *====================*/

/**
 * Get the title of a popup
 * @param win pointer to a popup object
 * @return title string of the popup
 */
const char *lv_lvsfpopup_get_title(const lv_obj_t *hdr)
{
    LV_ASSERT_OBJ(hdr, MY_CLASS);

    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;
    return lv_label_get_text(ext->title);
}

/*=====================
 * Other functions
 *====================*/



/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Realign the building elements of a popup
 * @param win pointer to popup objectker
 */
static void lv_lvsfpopup_realign(lv_obj_t *hdr)
{
    lv_lvsfpopup_ext_t *ext = (lv_lvsfpopup_ext_t *)hdr;

    lv_coord_t left = lv_obj_get_style_pad_left(hdr, LV_PART_MAIN);
    lv_coord_t right = lv_obj_get_style_pad_right(hdr, LV_PART_MAIN);
    lv_coord_t top = lv_obj_get_style_pad_top(hdr, LV_PART_MAIN);
    lv_coord_t bottom = lv_obj_get_style_pad_bottom(hdr, LV_PART_MAIN);

    if (ext->title_icon)
    {
        lv_obj_align(ext->title_icon, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_align_to(ext->title, ext->title_icon, LV_ALIGN_OUT_RIGHT_MID, left, 0);
        lv_obj_align_to(ext->content, ext->title_icon, LV_ALIGN_OUT_BOTTOM_LEFT, 0, left);
    }
    else
    {
        lv_obj_align(ext->title, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_align_to(ext->content, ext->title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, left);
    }

    lv_obj_align(ext->confirm_btn,  LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_align(ext->cancel_btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
}

#endif

