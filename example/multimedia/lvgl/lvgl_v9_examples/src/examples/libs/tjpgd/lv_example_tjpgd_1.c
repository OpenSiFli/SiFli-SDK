#include "../../lv_examples.h"

#if LV_USE_TJPGD &&  LV_USE_FS_MEMFS && LV_BUILD_EXAMPLES
// Assume the image is in memory (e.g. embedded in the firmware)
// Type:LV_IMAGE_SRC_VARIABLE
LV_IMG_DECLARE(img_logo)//Change to different image data
void lv_example_tjpgd_2(void)
{
    lv_obj_t *wp;

    wp = lv_image_create(lv_screen_active());
    lv_image_set_src(wp, &img_logo);

    lv_obj_center(wp);
}
/*LV_USE_FS_MEMFS*/

#elif LV_USE_TJPGD && LV_BUILD_EXAMPLES
/**
 * Load a JPG image, from file system and display it on screen
 * Type:LV_IMAGE_SRC_FILE
 */
void lv_example_tjpgd_1(void)
{
    lv_obj_t * wp;

    wp = lv_image_create(lv_screen_active());
    /* Assuming a File system is attached to letter 'A'
     * E.g. set LV_USE_FS_STDIO 'A' in lv_conf.h */
    lv_image_set_src(wp, "A:flower.jpg");
    lv_obj_center(wp);
}

#endif

