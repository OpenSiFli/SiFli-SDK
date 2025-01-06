/**
 * @file lvsf_analogclk.h
 *
 */

#ifndef LVSF_ANALOGCLK_H
#define LVSF_ANALOGCLK_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

lv_obj_t *lv_analogclk_create(lv_obj_t *parent);

int lv_analogclk_img(lv_obj_t *aclk, const char *bg, const char *hour, const char *min, const char *second);

void lv_analogclk_refr_inteval(lv_obj_t *aclk, uint16_t ms);

void lv_analogclk_pos_off(lv_obj_t *aclk, uint8_t hoff, uint8_t moff, uint8_t soff);

int lv_analogclk_set_bg(lv_obj_t *aclk, const char *bg);
int lv_analogclk_set_hour(lv_obj_t *aclk, const char *hour);
int lv_analogclk_set_min(lv_obj_t *aclk, const char *min);
int lv_analogclk_set_second(lv_obj_t *aclk, const char *second);

void lv_analogclk_set_hoff(lv_obj_t *aclk, uint8_t hoff);
void lv_analogclk_set_moff(lv_obj_t *aclk, uint8_t moff);
void lv_analogclk_set_soff(lv_obj_t *aclk, uint8_t soff);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_ANALOGCLK_H*/


