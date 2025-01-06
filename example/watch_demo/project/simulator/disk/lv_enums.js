// LV_ALIGN_XXX
//export const ALIGN_IN_TOP_LEFT = 1;
export const ALIGN_IN_TOP_MID = 2;
export const ALIGN_IN_TOP_RIGHT = 3;
//export const ALIGN_IN_BOTTOM_LEFT = 4;
export const ALIGN_IN_BOTTOM_MID = 5;
//export const ALIGN_IN_BOTTOM_RIGHT = 6;
//export const ALIGN_IN_LEFT_MID = 7;
//export const ALIGN_IN_RIGHT_MID = 8;
export const ALIGN_CENTER = 9;
//export const ALIGN_OUT_TOP_LEFT = 10;
//export const ALIGN_OUT_TOP_MID = 11;
//export const ALIGN_OUT_TOP_RIGHT = 12;
//export const ALIGN_OUT_BOTTOM_LEFT = 13;
//export const ALIGN_OUT_BOTTOM_MID = 14;
//export const ALIGN_OUT_BOTTOM_RIGHT = 15;
//export const ALIGN_OUT_LEFT_TOP = 16;
export const ALIGN_OUT_LEFT_MID = 17;
//export const ALIGN_OUT_LEFT_BOTTOM = 18;
//export const ALIGN_OUT_RIGHT_TOP = 19;
export const ALIGN_OUT_RIGHT_MID = 20;
//export const ALIGN_OUT_RIGHT_BOTTOM = 21;

// LV_EVENT_XXX
//export const EVENT_PRESSED=1;           
//export const EVENT_PRESSING=2;
//export const EVENT_PRESS_LOST=3;
//export const EVENT_SHORT_CLICKED=4;
//export const EVENT_LONG_PRESSED=5;
//export const EVENT_LONG_PRESSED_REPEAT=6;
export const EVENT_CLICKED=7;
//export const EVENT_RELEASED=8;
//export const EVENT_SCROLL_BEGIN=9;
//export const EVENT_SCROLL_END=10;
//export const EVENT_SCROLL=11;
//export const EVENT_GESTURE=12;
//export const EVENT_KEY=13;
//export const EVENT_FOCUSED=14;
//export const EVENT_DEFOCUSED=15;
//export const EVENT_LEAVE=16;
//export const EVENT_HIT_TEST=17;

//FONT_XXX
export const FONT_SMALL = 16;   // > 360: 16; else: 12
export const FONT_NORMAL = 20;      // > 360: 20; else: 16
export const FONT_SUBTITLE = 24;    // > 360: 24; else: 20
export const FONT_TITLE = 28;       // > 360: 28; else: 24
export const FONT_BIGL = 36;        // > 360: 36; else: 28
export const FONT_HUGE = 56;        // > 360: 56; else: 36
export const FONT_SUPER = 72;       // > 360: 72; else: 56

// LV_COLOR_XXX, color const is defined in RGB (24bit)
export const LV_COLOR_WHITE = 16777215;
export const LV_COLOR_BLACK = 0;
// GUI Data
//export const DATA_NULL = 0;
export const DATA_BATTERY=1;
export const DATA_STEP=2;
export const DATA_HR=3;
//export const DATA_SPO2=4;
//export const DATA_BP=5;
//export const DATA_TEMP=6; /*<!reserved*/
//export const DATA_STRESS=7; /*<!reserved*/
//export const DATA_SETTING=8;
export const DATA_WEATHER_INFO=9;
//export const DATA_BLE_CONNECT=10;
//export const DATA_BLE_ADV=11;
//export const DATA_WIFI_CONNECT=12;
//export const DATA_WIFI_ENABLE=13;
//export const DATA_AIR_MODE_ENABLE=14;
//export const DATA_LANG_CHANGE=15;

//lv_img_cfg_t;
export const CF_RAW=1;
export const CF_RAW_ALPHA=2;    
export const CF_RAW_CHROMA_KEYED=3; 
export const CF_TRUE_COLOR=4;              
export const CF_TRUE_COLOR_ALPHA=5;       
export const CF_TRUE_COLOR_CHROMA_KEYED=6;

// LV_OBJ_FLAG_XXX
export const LV_OBJ_FLAG_HIDDEN          = 0x1;  /**< Make the object hidden. (Like it wasn't there at all)*/
export const LV_OBJ_FLAG_CLICKABLE       = 0x2;  /**< Make the object clickable by the input devices*/
//export const LV_OBJ_FLAG_CLICK_FOCUSABLE = (1L << 2),  /**< Add focused state to the object when clicked*/
//export const LV_OBJ_FLAG_CHECKABLE       = (1L << 3),  /**< Toggle checked state when the object is clicked*/
//export const LV_OBJ_FLAG_SCROLLABLE      = (1L << 4),  /**< Make the object scrollable*/
//export const LV_OBJ_FLAG_SCROLL_ELASTIC  = (1L << 5),  /**< Allow scrolling inside but with slower speed*/
//export const LV_OBJ_FLAG_SCROLL_MOMENTUM = (1L << 6),  /**< Make the object scroll further when "thrown"*/
//export const LV_OBJ_FLAG_SCROLL_ONE      = (1L << 7),  /**< Allow scrolling only one snappable children*/
//export const LV_OBJ_FLAG_SCROLL_CHAIN_HOR = (1L << 8), /**< Allow propagating the horizontal scroll to a parent*/
//export const LV_OBJ_FLAG_SCROLL_CHAIN_VER = (1L << 9), /**< Allow propagating the vertical scroll to a parent*/
//export const LV_OBJ_FLAG_SCROLL_CHAIN     = (LV_OBJ_FLAG_SCROLL_CHAIN_HOR | LV_OBJ_FLAG_SCROLL_CHAIN_VER),
//export const LV_OBJ_FLAG_SCROLL_ON_FOCUS = (1L << 10),  /**< Automatically scroll object to make it visible when focused*/
//export const LV_OBJ_FLAG_SCROLL_WITH_ARROW  = (1L << 11), /**< Allow scrolling the focused object with arrow keys*/
//export const LV_OBJ_FLAG_SNAPPABLE       = (1L << 12), /**< If scroll snap is enabled on the parent it can snap to this object*/
//export const LV_OBJ_FLAG_PRESS_LOCK      = (1L << 13), /**< Keep the object pressed even if the press slid from the object*/
//export const LV_OBJ_FLAG_EVENT_BUBBLE    = (1L << 14), /**< Propagate the events to the parent too*/
//export const LV_OBJ_FLAG_GESTURE_BUBBLE  = (1L << 15), /**< Propagate the gestures to the parent*/
//export const LV_OBJ_FLAG_ADV_HITTEST     = (1L << 16), /**< Allow performing more accurate hit (click) test. E.g. consider rounded corners.*/
//export const LV_OBJ_FLAG_IGNORE_LAYOUT   = (1L << 17), /**< Make the object position-able by the layouts*/
//export const LV_OBJ_FLAG_FLOATING        = (1L << 18), /**< Do not scroll the object when the parent scrolls and ignore layout*/
//export const LV_OBJ_FLAG_OVERFLOW_VISIBLE = (1L << 19), /**< Do not clip the children's content to the parent's boundary*/
//export const LV_OBJ_FLAG_LAYOUT_1        = (1L << 23), /**< Custom flag, free to use by layouts*/
//export const LV_OBJ_FLAG_LAYOUT_2        = (1L << 24), /**< Custom flag, free to use by layouts*/
//export const LV_OBJ_FLAG_WIDGET_1        = (1L << 25), /**< Custom flag, free to use by widget*/
//export const LV_OBJ_FLAG_WIDGET_2        = (1L << 26), /**< Custom flag, free to use by widget*/
//export const LV_OBJ_FLAG_USER_1          = (1L << 27), /**< Custom flag, free to use by user*/
//export const LV_OBJ_FLAG_USER_2          = (1L << 28), /**< Custom flag, free to use by user*/
//export const LV_OBJ_FLAG_USER_3          = (1L << 29), /**< Custom flag, free to use by user*/
//export const LV_OBJ_FLAG_USER_4          = (1L << 30), /**< Custom flag, free to use by user*/

//lvsf_com_type
export const COMP_CIRCLE=0;
export const COMP_MOD_SMALL=1;
export const COMP_MOD_LARGE=2;
export const COMP_EXTRA_LARGE=3;
export const COMP_UTILITY_SMALL=4;
export const COMP_CIRCULAR=5;
export const COMP_CORNER=6;
//analogclk child type
export const ANALOGCLK_BG=0;
export const ANALOGCLK_HOUR=1;
export const ANALOGCLK_MIN=2;
export const ANALOGCLG_SEC=3;
//setting subpage idx
export const USER_NAME=1;
export const NOTIFY=2;
export const GENERAL=3;
export const DND_MODE=4;
export const AIRPLANE_MODE=5;
export const WIFI=6;
export const BLUETOOTH=7;
export const DISPLAY_BRIGHTNESS=8;
export const APP_VIEW=9;
export const APP_ABOUT=10;
export const APP_POWEROFF=11;
export const APP_STOPWATCH=12;
export const SYS_LANGUAGE=13;
export const LOCK_SCREEN=14;
export const SOUNDS_TOUCH=15;
export const ELECTRIC=16;
export const FLASHLIGHT=17;
export const POWER_DOWN=18;
export const PASSCODE=19;
export const DRINK_WATER=20;
export const SEDENTAR=21;
