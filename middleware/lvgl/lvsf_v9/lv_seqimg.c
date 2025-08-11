/**
 * @file lv_seqimg.c
 *
 *
 */
#include "rtthread.h"
#include "lv_seqimg.h"
#include "lv_timer_private.h"
#include "lv_image_private.h"
#include "lv_obj_class_private.h"


#define MY_CLASS &lv_seqimg_class


typedef struct
{
    lv_image_t img;                   // Basic Image Control
    lv_image_dsc_t **dsc_array;       // Array of images
    uint16_t size;                  // Array Size
    lv_timer_t *timer;              // Playback timer
    uint16_t curr_frame;            // Current frame index
    uint32_t period;                // Playback interval (ms)
    bool playing;                   // Is it currently playing?
} lv_seqimg_t;


static void lv_seqimg_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_seqimg_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);


const lv_obj_class_t lv_seqimg_class =
{
    .constructor_cb = lv_seqimg_constructor,
    .destructor_cb = lv_seqimg_destructor,
    .instance_size = sizeof(lv_seqimg_t),
    .base_class = &lv_img_class,

};
/*
Timer callback, frame data switching
*/
static void seqimg_timer_cb(lv_timer_t *timer)
{
    lv_obj_t *obj = (lv_obj_t *)timer->user_data;
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;

    ext->curr_frame = (ext->curr_frame + 1) % ext->size;//Each time an index update is triggered
    if (ext->dsc_array)
    {
        lv_img_set_src(obj, ext->dsc_array[ext->curr_frame]);
    }

}
/*
initializer
*/
static void lv_seqimg_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;
    ext->dsc_array = NULL;
    ext->size = 0;
    ext->timer = NULL;
    ext->curr_frame = 0;
    ext->period = 100;
    ext->playing = false;
}

static void lv_seqimg_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;
    if (ext->timer)
    {
        lv_timer_del(ext->timer);
        ext->timer = NULL;
    }
}


/*
Create a sequence frame image control
*/
lv_obj_t *lv_seqimg_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(&lv_seqimg_class, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

/*
Set up an array of pictures
dsc_array：A new array of frame images (each element is a lv_img_dsc_t description)
size：The number of frames

After the replacement, the frame index is reset to 0 and the first frame is displayed.

*/
void lv_seqimg_src_array(lv_obj_t *obj, const lv_img_dsc_t **dsc_array, uint16_t size)
{
    LV_ASSERT_NULL(obj);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;
    ext->dsc_array = (lv_img_dsc_t **)dsc_array;
    ext->size = size;
    ext->curr_frame = 0;
    if (dsc_array && size > 0)
    {
        lv_img_set_src(obj, dsc_array[0]);
    }
}

/*
Select frame playback
*/
void lv_seqimg_select(lv_obj_t *obj, uint16_t index)
{
    LV_ASSERT_NULL(obj);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;

    if (index >= ext->size)
    {
        return;
    }

    ext->curr_frame = index;
    if (ext->dsc_array)
    {
        lv_img_set_src(obj, ext->dsc_array[index]);
    }

}
/*
Start playing
*/
void lv_seqimg_play(lv_obj_t *obj)
{
    LV_ASSERT_NULL(obj);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;

    if (!ext->playing && ext->size > 0)
    {
        if (!ext->timer)
        {
            ext->timer = lv_timer_create(seqimg_timer_cb, ext->period, obj);//Create a timer binding event to trigger playback
        }
        else
        {
            lv_timer_resume(ext->timer);
        }
        ext->playing = true;
    }
    else
    {
        if (ext->size == 0)
        {
            LV_LOG_WARN("seqimg: NULL\n");
        }
        else if (ext->playing)
        {
            LV_LOG_WARN("seqimg: drawing\n");
        }

    }
}
/*
Trigger the timer to pause the playback
*/
void lv_seqimg_pause(lv_obj_t *obj)
{
    LV_ASSERT_NULL(obj);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;

    if (ext->playing)
    {
        lv_timer_pause(ext->timer);
        ext->playing = false;
    }
}
/*
Set the playback interval (set the time interval of the timer)
*/
void lv_seqimg_set_period(lv_obj_t *obj, uint32_t period)
{
    LV_ASSERT_NULL(obj);
    lv_seqimg_t *ext = (lv_seqimg_t *)obj;
    ext->period = period;
    if (ext->timer)
    {
        lv_timer_set_period(ext->timer, period);
    }
}


