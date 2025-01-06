#ifndef LVSF_POPUP_H
#define LVSF_POPUP_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_POPUP != 0

/*Testing of dependencies*/
#if LV_USE_BTN == 0
#error "lv_lvsfpopup: lv_btn is required. Enable it in lv_conf.h (LV_USE_BTN  1) "
#endif

#if LV_USE_LABEL == 0
#error "lv_lvsfpopup: lv_label is required. Enable it in lv_conf.h (LV_USE_LABEL  1) "
#endif

#if LV_USE_IMG == 0
#error "lv_lvsfpopup: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a popup objects
 * @param par pointer to an object, it will be the parent of the new popup
 * @return pointer to the created popup
 */
lv_obj_t *lv_lvsfpopup_create(lv_obj_t *par);


/**
 * Set the title of a popup
 * @param head pointer to a popup object
 * @param title string of the new title
 */
void lv_lvsfpopup_set_title(lv_obj_t *hdr, const char *title);


/**
 * @brief Set the title icon of the popup
 * @param hdr - pointer to a popup object
 * @param img - pointer to image source
 */
void lv_lvsfpopup_set_icon(lv_obj_t *hdr, const void *img);


/**
 * @brief Set the content of a popup
 * @param hdr - pointer to a popup object
 * @param content - popup content
 */
void lv_lvsfpopup_set_content(lv_obj_t *hdr, const char *content);


/**
 * @brief Set title txt of the confirm button
 * @param hdr - pointer to a popup object
 * @param txt - button title
 */
void lv_lvsfpopup_set_confirm_btn_txt(lv_obj_t *hdr, const char *txt);



/**
 * @brief Set title txt of the cancel button
 * @param hdr - pointer to a popup object
 * @param txt - button title
 */
void lv_lvsfpopup_set_cancel_btn_txt(lv_obj_t *hdr, const char *txt);








/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_POPUP*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_POPUP_H*/

