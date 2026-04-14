/**
* @file lv_gesture.c
* This gesture framework can support gesture operations at any location, 
* depending on the size of the area set.
*
*/

/*********************
*      INCLUDES
*********************/
#include "lvgl.h"
#include "lv_indev.h"
#include "lv_gesture.h"

/*********************
*      DEFINES
*********************/

#define GESTURE_MIN_DIS 40
/**
 * Determine if the gesture is in the valid areas.
 */
#define IS_IN_LEFT_AREA  (gesture.press_point.x < gesture.boundary_line)
#define IS_IN_MOVE_AREA(point) (HOR_MOVE_DIS(point)* 0.577f > VER_MOVE_DIS(point)  && (gesture.press_point.x < point.x))
/**
 * The horizontal distance between the current point and the initial position
 */
#define HOR_MOVE_DIS(point) LV_ABS(gesture.press_point.x - point.x)

  /**
* The vertical distance between the current point and the initial position
*/
#define VER_MOVE_DIS(point) LV_ABS(gesture.press_point.y - point.y)

 /**********************
*      TYPEDEFS
**********************/

/**********************
*  STATIC PROTOTYPES
**********************/
static bool lv_gesture_is_ignore(lv_obj_t *obj);

/**********************
*  STATIC VARIABLES
**********************/

enum
{
	LV_GESTURE_INDEV_INIT = 0,
	LV_GESTURE_INDEV_PRESSING,
	LV_GESTURE_INDEV_ANIM,
	LV_GESTURE_INDEV_WAIT,
};

typedef uint8_t lv_gesture_state_t;

/**
 * Ignore the gesture node structure.
 */
typedef struct
{
    lv_obj_t                    *obj;   /* Ignore the object with gestures. */
    lv_ll_t                     list;   /* Linked list. */

} lv_gesture_ignore_t;

/**
 * Gesture structure
 */
typedef struct
{
	lv_gesture_start_cb_t		start_cb;
    lv_gesture_event_cb_t       event_cb;
    lv_point_t                  press_point;
    lv_gesture_state_t          cur_state;
    lv_gesture_state_t          tar_state;
    lv_gesture_sche_state_t     sche_state;
    lv_gesture_state_t          indev_state;
    lv_ll_t                     ignore_list;
    lv_coord_t                  boundary_line;
    bool                        goback_en;  
} lv_gesture_t;

static lv_gesture_t gesture = { 0 };

/**********************
*      MACROS
**********************/

/**********************
*   GLOBAL FUNCTIONS
**********************/

void lv_gesture_set_sche_state(bool sche_idle)
{
	gesture.sche_state = sche_idle ? LV_GESTURE_SCHE_IDLE : LV_GESTURE_SCHE_SUSPEND;
    
	/** If it's not idle, do nothing **/
	if(LV_GESTURE_SCHE_IDLE != gesture.sche_state)
		return;

	/** If the target state is not none, change current state to target **/
	if(LV_GESTURE_NONE != gesture.tar_state)
	{
		gesture.cur_state = gesture.tar_state;
		gesture.tar_state = LV_GESTURE_NONE;
	}
}

lv_gesture_sche_state_t lv_gesture_get_sche_state(void)
{
	return gesture.sche_state;
}

typedef struct
{
    lv_point_t press;
    lv_point_t dis_p;
    lv_dir_t dir;
    uint32_t start_tick;
    lv_gesture_state_t state;
}dir_detect_t;

static dir_detect_t dir_detect;
#define dir_dd 225


void lv_dir_detect_proc(const lv_indev_t* indev, lv_indev_state_t state)
{
    switch (state)
    {
    case LV_INDEV_STATE_PR:
    {
        if (LV_GESTURE_INDEV_INIT == dir_detect.state)
        {
            dir_detect.dir = LV_DIR_NONE;
            dir_detect.start_tick = lv_tick_get();
            dir_detect.state = LV_GESTURE_INDEV_PRESSING;
            lv_indev_get_point(indev, &dir_detect.press);
            lv_indev_get_point(indev, &dir_detect.dis_p);
        }
        else if (LV_GESTURE_INDEV_PRESSING == dir_detect.state)
        {
            lv_point_t p;
            lv_indev_get_point(indev, &p);

            int32_t dx = p.x - dir_detect.press.x;
            int32_t dy = p.y - dir_detect.press.y;
            int32_t ddx = dx * dx;
            int32_t ddy = dy * dy;
            if (LV_DIR_NONE == dir_detect.dir)
            {
                if (ddx > ddy && ddx > dir_dd)
                    dir_detect.dir = dx > 0 ? LV_DIR_RIGHT : LV_DIR_LEFT;
                else if(ddx < ddy && ddy > dir_dd)
                    dir_detect.dir = dy > 0 ? LV_DIR_BOTTOM : LV_DIR_TOP;
            }
            else
            {
                lv_dir_t dir = dir_detect.dir;
                lv_point_t d = dir_detect.dis_p;
                if ((LV_DIR_LEFT == dir && (d.x > p.x)) ||
                    (LV_DIR_RIGHT == dir && (d.x < p.x)) ||
                    (LV_DIR_BOTTOM == dir && (d.y < p.y)) ||
                    (LV_DIR_TOP == dir && (d.y > p.y)))
                {
                    dir_detect.dis_p = p;
                }
                d = dir_detect.dis_p;

                //tan(30) = 0.57735;
                //0.577.3^2 = 0.3333
                int32_t ddx_t = ddx * 0.3333f;
                int32_t ddy_t = ddy * 0.3333f;
                
                if ((LV_DIR_LEFT == dir || LV_DIR_RIGHT == dir) && ((ddx < ddy_t) || LV_ABS(d.x - p.x) > 5) ||
                    (LV_DIR_BOTTOM == dir || LV_DIR_TOP == dir) && ((ddy < ddx_t) || LV_ABS(d.y - p.y) > 5))
                {
                    dir_detect.dir = LV_DIR_NONE;
                    dir_detect.state = LV_GESTURE_INDEV_WAIT;
                }
            }
        }
        break;
    }
    case LV_INDEV_STATE_REL:
    {
        if (LV_GESTURE_INDEV_PRESSING == dir_detect.state && LV_DIR_NONE != dir_detect.dir)
        {
            lv_obj_t *obj = indev->proc.types.pointer.act_obj;
            if (obj)
            {
                gesture_para_t para;
                para.time = lv_tick_elaps(dir_detect.start_tick);
                lv_coord_t dx = dir_detect.dis_p.x - dir_detect.press.x;
                lv_coord_t dy = dir_detect.dis_p.y - dir_detect.press.y;
                para.dis = LV_MAX(LV_ABS(dx), LV_ABS(dy));
                para.dir = dir_detect.dir;
                lv_event_send(obj, LV_EVENT_GESTURE_DIR, &para);
            } 
        }
        dir_detect.state = LV_GESTURE_INDEV_INIT;
        break;
    }
    default:
        break;
    }
}


int lv_gesture_proc(const lv_indev_t * indev, lv_indev_state_t state)
{
    int err = RT_ERROR;
    
	lv_point_t act_point;
	lv_indev_get_point(indev, &act_point);

    //gesture detection
    lv_dir_detect_proc(indev, state);

	lv_obj_t *act_obj = indev->proc.types.pointer.act_obj;

    /* If the object currently being touched has already been set to ignore the gesture animation, then return directly. */
	if (act_obj && lv_gesture_is_ignore(act_obj))
	{
        return err;
	}

	if((LV_GESTURE_ENABLE != gesture.cur_state) || !gesture.event_cb) return err;
	switch(state)
	{
		case LV_INDEV_STATE_PR:
		{
        /**Determine whether to start drage begin only when switch from the release state to the pressed state for the first time **/
        if (LV_GESTURE_INDEV_INIT == gesture.indev_state && (gesture.sche_state == LV_GESTURE_SCHE_IDLE))
        {
            gesture.indev_state = LV_GESTURE_INDEV_PRESSING;
            gesture.press_point = act_point;
        }
        else if (LV_GESTURE_INDEV_PRESSING == gesture.indev_state)
        {
			if (gesture.start_cb && !gesture.start_cb(&act_point))
			{
				gesture.indev_state = LV_GESTURE_INDEV_WAIT;
				return RT_ERROR;
			}

            if (HOR_MOVE_DIS(act_point) < GESTURE_MIN_DIS)
                return RT_ERROR;

            if (IS_IN_MOVE_AREA(act_point))
            {
                if (IS_IN_LEFT_AREA)
                    gesture.event_cb(LV_GESTURE_ANIM_START, act_point.x);
                gesture.indev_state = LV_GESTURE_INDEV_ANIM;
                /**Prevents the next page from switching the gesture switch state,
                temporarily switching the scheduling state to suspend**/
                lv_gesture_set_sche_state(false);
            }
            else
            {
                gesture.indev_state = LV_GESTURE_INDEV_WAIT;
            }
        }
        else if (LV_GESTURE_INDEV_ANIM == gesture.indev_state)
        {
            err = RT_EOK;
            if (IS_IN_LEFT_AREA)
                gesture.event_cb(LV_GESTURE_ANIM_PRESSING, act_point.x);
            else if (!gesture.goback_en || !IS_IN_MOVE_AREA(act_point))
            {
                gesture.indev_state = LV_GESTURE_INDEV_WAIT;
                err = RT_ERROR;
            }
        }
        break;
    }
    case LV_INDEV_STATE_REL:
    {
        if (LV_GESTURE_INDEV_ANIM == gesture.indev_state)
        {
            int32_t pro = lv_map(HOR_MOVE_DIS(indev->proc.types.pointer.last_point), 0, LV_HOR_RES - gesture.press_point.x, 0, LV_HOR_RES);
            gesture.event_cb(IS_IN_LEFT_AREA ? LV_GESTURE_ANIM_END : LV_GESTURE_GOBACK, pro);
        }
        gesture.indev_state = LV_GESTURE_INDEV_INIT;
        break;
    }
		default:
			break;
	}
	return err;
}


int lv_gesture_is_valid()
{
    return (LV_GESTURE_ENABLE == gesture.cur_state && (gesture.indev_state == LV_GESTURE_INDEV_ANIM));
}

void lv_gesture_enable(void)
{
	if(LV_GESTURE_SCHE_IDLE == gesture.sche_state)
	{
		gesture.cur_state = LV_GESTURE_ENABLE;
		gesture.tar_state = LV_GESTURE_NONE;
	}
	else
	{
		gesture.tar_state = LV_GESTURE_ENABLE;
	}
}

void lv_gesture_disable(void)
{
	if(LV_GESTURE_SCHE_IDLE == gesture.sche_state)
	{
		gesture.cur_state = LV_GESTURE_DISABLE;
		gesture.tar_state = LV_GESTURE_NONE;
	}
	else
	{
		gesture.tar_state = LV_GESTURE_DISABLE;
	}
}

void lv_gesture_init(lv_gesture_event_cb_t event_cb)
{
    gesture.indev_state = LV_GESTURE_INDEV_INIT;

    gesture.cur_state = LV_GESTURE_ENABLE;
    gesture.tar_state = LV_GESTURE_NONE;

    gesture.sche_state = LV_GESTURE_SCHE_IDLE;
    gesture.boundary_line = 0;
    gesture.goback_en = false;
    gesture.event_cb = event_cb;
	gesture.start_cb = NULL;
    _lv_ll_init(&gesture.ignore_list, sizeof(lv_gesture_ignore_t));
}


lv_coord_t lv_gesture_set_area_line(lv_coord_t left_area)
{
	lv_coord_t pre_line = gesture.boundary_line;
	gesture.boundary_line = left_area;
	return pre_line;
}

lv_coord_t lv_gesture_get_area_line()
{
	return gesture.boundary_line;
}

void lv_gesture_set_goback(uint8_t goback_en)
{
	gesture.goback_en = goback_en;
}

uint8_t lv_gesture_get_goback()
{
	return gesture.goback_en;
}

void lv_gesture_set_start_cb(lv_gesture_start_cb_t event_cb)
{
	gesture.start_cb = event_cb;
}

/**
 * Determine whether the object does not respond to gestures.
 */
static bool lv_gesture_is_ignore(lv_obj_t *obj)
{
    lv_gesture_ignore_t *ignore_node = NULL;
    _LV_LL_READ(&gesture.ignore_list, ignore_node)
    {
        lv_obj_t *par = obj;
        lv_obj_t *act_par;
        do {
            if(par == ignore_node->obj)
                return true;
            act_par = par;
            par = lv_obj_get_parent(act_par);
        } while(par != NULL);
    }
    
    return false;
}

/**
 * Determine whether the ignore node is in the gesture ignore linked list.
 */
static bool lv_gesture_ignore_is_valid(lv_gesture_ignore_t *ignore)
{
    lv_gesture_ignore_t *ignore_node = NULL;
    _LV_LL_READ(&gesture.ignore_list, ignore_node)
    {
        if(ignore == ignore_node)
            return true;
    }
    
    return false;
}

/**
 * Delete the event callback of the object that ignores gestures. 
 * In this callback, remove the corresponding node from the gesture linked list to prevent memory leaks.
 */
static void lv_gesture_ignore_cb(lv_event_t *e)
{
    lv_gesture_ignore_t *ignore_node = lv_event_get_user_data(e);
    
    /* Check whether the node is valid to prevent double freeing 
     * after the deletion event is sent by the child of the object. */
    if(ignore_node && lv_gesture_ignore_is_valid(ignore_node)) 
    {
        _lv_ll_remove(&gesture.ignore_list, (void *)ignore_node);
        lv_mem_free((void *)ignore_node);
    }
}

/**
 * Set whether an object ignores gestures.
 */
void lv_gesture_set_ignore(lv_obj_t *obj, bool en)
{
    if(!obj) return;

    lv_gesture_ignore_t *ignore_node = NULL;

    if(en)
    {
        /* Add a new node and register a deletion event callback for the object 
         * so that the corresponding node information can be removed when the object is destroyed. */
        ignore_node = _lv_ll_ins_tail(&gesture.ignore_list);
        ignore_node->obj = obj;
        lv_obj_add_event_cb(obj, lv_gesture_ignore_cb, LV_EVENT_DELETE, ignore_node);
    }
    else
    {
        /* Destroy the object node. */
        
        ignore_node = _lv_ll_get_head(&gesture.ignore_list);
        while (ignore_node)
        {
            _LV_LL_READ(&gesture.ignore_list, ignore_node)
            {
                if (obj != ignore_node->obj) continue;

                lv_obj_remove_event_cb(obj, lv_gesture_ignore_cb);
                _lv_ll_remove(&gesture.ignore_list, (void*)ignore_node);
                lv_mem_free((void*)ignore_node);
                break;
            }
        }
    }
}