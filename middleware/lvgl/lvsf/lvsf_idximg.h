#ifndef _LVSF_IDXIMG_H_
#define _LVSF_IDXIMG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USE_IDXIMG!=0

#if LV_USE_IMG == 0
#error "lvsf_icon: lv_img is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
#define LVSF_IDXIMG_PREFIX_LEN  32

/**********************
 * GLOBAL PROTOTYPES
 **********************/
lv_obj_t *lv_idximg_create(lv_obj_t *par);
void lv_idximg_bind_src_array(lv_obj_t *idximg, const lv_img_dsc_t **dsc_array, uint16_t size);
void lv_idximg_select(lv_obj_t *idximg, uint16_t index);
void lv_idximg_prefix(lv_obj_t *idximg, const char *prefix);

/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USING_IDXIMG*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LVSF_IDXIMG_H_*/

