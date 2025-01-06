#ifndef LVSF_BARCODE_H
#define LVSF_BARCODE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_BARCODE != 0

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
 * Create a barcode objects
 * @param par pointer to an object, it will be the parent of the new barcode
 * @return pointer to the created barcode
 */
lv_obj_t *lv_lvsfbarcode_create(lv_obj_t *parent);


/**
 * @brief Set text for barcode
 * @param text - qrcode text
 */
void lv_lvsfbarcode_set_text(lv_obj_t *qrcode, char *text);


/**********************
 *      MACROS
 **********************/

#endif /*LVSF_USE_BARCODE*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_BARCODE_H*/

