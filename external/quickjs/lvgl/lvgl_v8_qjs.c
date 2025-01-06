#include <rtthread.h>
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "dfs_file.h"
#include "quickjs.h"
#include "cutils.h"
#include "gui_app_data.h"

#define DBG_TAG           "QJS.LV"
//#define DBG_LVL           DBG_LOG
#define DBG_LVL           DBG_INFO
#include "rtdbg.h"

/*unvarnished transmission message structure*/
typedef struct
{
    uint16_t idx;           /*subscribed index, align to service_if_type_t */
    uint16_t data_len;      /*message lenght*/
    void *data;             /*message pointer*/

} lv_obj_datasubs_t;

typedef void (*lv_obj_datasubs_cb_t)(struct _lv_obj_t *obj, lv_obj_datasubs_t *data);
#if SOLUTION_WATCH
extern int lv_obj_data_subscribe(lv_obj_t *obj, uint16_t idx, lv_obj_datasubs_cb_t cb);
extern int lv_obj_data_unsubscribe(lv_obj_t *obj, uint16_t idx);
#else
int lv_obj_data_subscribe(lv_obj_t *obj, uint16_t idx, lv_obj_datasubs_cb_t cb)
{
    return RT_EOK;
}

int lv_obj_data_unsubscribe(lv_obj_t *obj, uint16_t idx)
{
    return RT_EOK;
}
#endif


#define MAX_PARAMS 	8
typedef struct {
    const char * name;
	uint8_t param_type[MAX_PARAMS];
	uint8_t return_type;
} JS_lv_protos;

enum {
	LVTYPE_none=0,
	LVTYPE_int,
	LVTYPE_bool,
	LVTYPE_func,
	LVTYPE_object,
	LVTYPE_color,
	LVTYPE_string,
};


static JSValue js_lv_obj_method(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic);
static JSValue js_lv_func(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic);

static JSValue js_lv_ext_func(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic);

static JSClassID js_lvobj_class_id;
static script_data_callback js_data_cbk;

typedef struct {
    lv_obj_t * lvobj;
    JSContext *ctx;
    JSValueConst this_obj;
    JSValue callback;
    JSValue data_cbk;
    uint32_t data_idx;
} JSlvobj;

void lv_data_callback(void * ctx, uint16_t len, void * data)
{
    JSValue val, val2;
    JSValue jret;
	JSlvobj * s=(JSlvobj *)ctx;

    switch (s->data_idx) {
		case GUI_DATA_RT_BATTERY_INFO:
        {
            
            battery_info_t * bat_info=(battery_info_t*)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewInt32(s->ctx, (int32_t)bat_info->level);
            JS_SetPropertyStr( s->ctx, val, "level", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)bat_info->charger_status);
            JS_SetPropertyStr( s->ctx, val, "charger_status", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }        
        case GUI_DATA_RT_STEP_INFO:
        {
            step_info_t * step=(step_info_t *)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewInt32(s->ctx, (int32_t)step->step);
            JS_SetPropertyStr( s->ctx, val, "step", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)step->calories);
            JS_SetPropertyStr( s->ctx, val, "calories", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)step->distance);
            JS_SetPropertyStr( s->ctx, val, "distance", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }
        case GUI_DATA_RT_HR_INFO:
        {
            hr_info_t * heart_rate=(hr_info_t *)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewInt32(s->ctx, (int32_t)heart_rate->hr);
            JS_SetPropertyStr( s->ctx, val, "hr", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)heart_rate->rhr);
            JS_SetPropertyStr( s->ctx, val, "rhr", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)heart_rate->timestamp);
            JS_SetPropertyStr( s->ctx, val, "timestamp", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }
        case GUI_DATA_RT_SPO2_INFO:
        case GUI_DATA_DEV_BLE_CONNECT:
        case GUI_DATA_DEV_BLE_ADV:
        case GUI_DATA_DEV_WIFI_CONNECT:
        case GUI_DATA_DEV_WIFI_ENABLE:
        case GUI_DATA_DEV_AIR_MODE_ENABLE:
            val=JS_NewInt32(s->ctx, (int32_t)*((uint8_t*)data));
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            break;
        case GUI_DATA_RT_BP_INFO:
        {
            bp_info_t * blood_pressure=(bp_info_t *)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewInt32(s->ctx, (int32_t)blood_pressure->bp_h);
            JS_SetPropertyStr( s->ctx, val, "bp_h", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)blood_pressure->bp_l);
            JS_SetPropertyStr( s->ctx, val, "bp_l", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }
        case GUI_DATA_RT_TEMP_INFO:
        case GUI_DATA_RT_STRESS_INFO:
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 0, NULL);
            break;   

        case GUI_DATA_SETTING_SYS_USER:
        {
            setting_sys_user_t * setting=(setting_sys_user_t *)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewString(s->ctx, (const char *)setting->sys_language);
            JS_SetPropertyStr( s->ctx, val, "sys_language", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)setting->sys_hour_mode);
            JS_SetPropertyStr( s->ctx, val, "sys_hour_mode", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)setting->sys_display_time);
            JS_SetPropertyStr( s->ctx, val, "sys_display_time", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)setting->sys_pair_req);
            JS_SetPropertyStr( s->ctx, val, "sys_pair_req", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }
 
        case GUI_DATA_WEATHER_INFO:
        {
            setting_weather_t * weather=(setting_weather_t *)data;
            val=JS_NewObject(s->ctx);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->l_temp);
            JS_SetPropertyStr( s->ctx, val, "l_temp", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->h_temp);
            JS_SetPropertyStr( s->ctx, val, "h_temp", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->state);
            JS_SetPropertyStr( s->ctx, val, "state", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->temperature);
            JS_SetPropertyStr( s->ctx, val, "temperature", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->aqi);
            JS_SetPropertyStr( s->ctx, val, "aqi", val2);
            val2=JS_NewInt32(s->ctx, (int32_t)weather->hour);
            JS_SetPropertyStr( s->ctx, val, "hour", val2);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        }            
        case GUI_DATA_LANG_CHANGE:
            val=JS_NewString(s->ctx, (const char *)data);
            jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
            JS_FreeValue(s->ctx, val);
            break;
        default:
            if (len==0)
                jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 0, NULL);
            else {
                val=JS_NewArrayBufferCopy(s->ctx, (const uint8_t *)data,len);
                jret = JS_Call(s->ctx, s->data_cbk, s->this_obj, 1, &val);
                JS_FreeValue(s->ctx, val);
            }
            break;            
     }
}

void lv_event_callback_proxy(lv_event_t * event)
{
	lv_obj_t * obj = lv_event_get_target(event);
    JSlvobj *s = (JSlvobj *)lv_obj_get_user_data(obj);
    JSValue jret;
    uint16_t * data= (uint16_t *)lv_event_get_param(event);

    if (s==NULL) {  
        // Already deleted by js object destructor
        return;
    }
    if (event->code==LV_EVENT_REFRESH && data && (*data)==s->data_idx) {
        void * param=(void *)*(uint32_t*)(data+2);
        if (js_data_cbk)
			(*js_data_cbk)((void *)s,*(data+1),param);
		else
			lv_data_callback((void *)s,*(data+1),param);
    }
    else {
        if (event->code ==LV_EVENT_DELETE) {
            if (s->data_idx) {
       			JS_FreeValue(s->ctx, s->data_cbk);
                lv_obj_data_unsubscribe(s->lvobj, s->data_idx);
                s->data_idx=0;
            }
		    if (s->callback) {
   			    JS_FreeValue(s->ctx, s->callback);
            }
            s->lvobj=NULL;
        }
        else if (s->ctx && s->this_obj && s->callback){
			JSValue jevent=JS_NewInt32(s->ctx, event->code);
        	jret= JS_Call(s->ctx, s->callback, s->this_obj, 1, &jevent);
        }
		else {
			LOG_D("ctx=%p, this_obj=%lx, callback=%lx\n", s->ctx, s->this_obj, s->callback);
		}
    }

#if 0
    {
        int r;
        JS_ToInt32(s->ctx, &r, jret);
    }
#endif
}


static void lv_data_bind(JSlvobj * s)
{
    lv_obj_set_user_data(s->lvobj,s);
    lv_obj_add_event_cb(s->lvobj, lv_event_callback_proxy,LV_EVENT_ALL,NULL);
    lv_obj_data_subscribe(s->lvobj, s->data_idx, NULL);
}

lv_color_t lv_color_from_int(int full)
{
    uint8_t r, g, b;

    b=(uint8_t)(full&0xff);
    full>>=8;
    g=(uint8_t)(full&0xff);
    full>>=8;
    r=(uint8_t)(full&0xff);
    return lv_color_make(r,g,b);
}
/*************************************** To Be generated*******************************************************/
#include "lv_qjs_generated.c"
/*********************************************End of generation ********************************/

/***************************************** Common codes ****************************************/

static JSValue js_lv_process(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic, const JS_lv_protos * method, uint8_t is_obj)
{
    JSValueConst param[MAX_PARAMS];
    JSValue r;
     
    memset(param,0,sizeof(param));
	for (int i=0;i<MAX_PARAMS;i++) {
		if (method->param_type[i]==LVTYPE_none)
            break;
        switch (method->param_type[i]) {
            case LVTYPE_bool:
                param[i]=(JSValueConst)JS_ToBool(ctx,  argv[i]);
                break;
            case LVTYPE_int:
            case LVTYPE_color:
                JS_ToInt32(ctx, (int32_t*)&param[i], argv[i]);
                break;
            case LVTYPE_object:
                param[i]= argv[i];
                break;
            case LVTYPE_func:
                param[i]= argv[i];
                break;
            case LVTYPE_string:
                param[i]=(JSValueConst)JS_ToCString(ctx, argv[i]);
                break;                
        }
	};
    LOG_D("%s(%d,%d), %d, %d, %d, %d\n", method->name, magic, is_obj, (int)param[0], (int)param[1], (int)param[2],(int)param[3]);

    switch (is_obj) {
    case 1:
        r = lv_obj_call_method(ctx, this_val, magic, param);
        break;
    case 2:
        r = lv_ext_call_func(ctx, magic, param);
        break;
    default:
        r = lv_call_func(ctx, magic, param);
    }

	for (int i=0;i<MAX_PARAMS;i++) {
		if (method->param_type[i]==LVTYPE_none)
            break;
        if (method->param_type[i]==LVTYPE_string)
            JS_FreeCString(ctx, (const char *)param[i]); 
    }
    return r;
}

/**********************************************obj *********************************************/
/* lv_obj Class */

static void js_lv_obj_finalizer(JSRuntime *rt, JSValue val)
{
    JSlvobj *s = JS_GetOpaque(val, js_lvobj_class_id);
    /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

    if (s->lvobj) {
        if (s->data_idx) {
            JS_FreeValue(s->ctx, s->data_cbk);
            lv_obj_data_unsubscribe(s->lvobj, s->data_idx);
            s->data_idx=0;
        }
        if (s->callback) {
            JS_FreeValue(s->ctx, s->callback);
        }    
        lv_obj_set_user_data(s->lvobj, NULL);
    }
    js_free_rt(rt, s);
    LOG_D("lvobj desctructor\n");

}

static JSValue js_lv_obj_ctor(JSContext *ctx,
                             JSValueConst new_target,
                             int argc, JSValueConst *argv)
{
    JSlvobj *s;
    JSValue obj = JS_UNDEFINED;
    JSValue proto;
    
    s = js_mallocz(ctx, sizeof(*s));
    if (!s)
        return JS_EXCEPTION;


    /* using new_target to get the prototype is necessary when the
       class is extended. */
    {
        proto = JS_GetPropertyStr(ctx, new_target, "prototype");
        if (JS_IsException(proto))
            goto fail;
        obj = JS_NewObjectProtoClass(ctx, proto, js_lvobj_class_id);
        JS_FreeValue(ctx, proto);
    }

    // TODO: check parent
    JS_SetOpaque(obj, s);
    s->ctx = ctx;
    return obj;
fail:
    js_free(ctx, s);
    JS_FreeValue(ctx, obj);
    return JS_EXCEPTION;
}

static JSValue js_lv_obj_method(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic)
{
    return js_lv_process(ctx, this_val, argc, argv, magic, &(lv_obj_protos[magic - 1]), 1);
}

static JSClassDef js_lv_obj_class = {
    "lvobj",
    .finalizer = js_lv_obj_finalizer,
}; 


/***************************** Functions *****************************************************/

static JSValue js_lv_func(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic)
{
    return js_lv_process(ctx, this_val, argc, argv, magic, &(lv_funcs_proto[magic - 1]),0);
}


static JSValue js_lv_ext_func(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic)
{
    return js_lv_process(ctx, this_val, argc, argv, magic, &(lv_ext_funcs_proto[magic -1 ]),2);
}

/********************************************init*********************************************/

static int js_init_lvgl(JSContext *ctx, JSModuleDef *m)
{
    JSValue proto, class;
    
    /* create the lvobj class */
    JS_NewClassID(&js_lvobj_class_id);
    JS_NewClass(JS_GetRuntime(ctx), js_lvobj_class_id, &js_lv_obj_class);

    proto = JS_NewObject(ctx);
    JS_SetPropertyFunctionList(ctx, proto, js_lv_obj_methods, countof(js_lv_obj_methods));
    
    class = JS_NewCFunction2(ctx, js_lv_obj_ctor, "obj", 1, JS_CFUNC_constructor, 0);
    /* set proto.constructor and ctor.prototype */
    JS_SetConstructor(ctx, class, proto);
    JS_SetClassProto(ctx, js_lvobj_class_id, proto);
                      
    JS_SetModuleExport(ctx, m, "obj", class);

    JS_SetModuleExportList(ctx, m, js_lv_funcs, countof(js_lv_funcs));
    
    return 0;
}

JSModuleDef *js_add_lvgl(JSContext *ctx, const char *module_name)
{
    JSModuleDef *m;

    m = JS_NewCModule(ctx, module_name, js_init_lvgl);
    if (!m)
        return NULL;
    JS_AddModuleExport(ctx, m, "obj");
    JS_AddModuleExportList(ctx, m, js_lv_funcs, countof(js_lv_funcs));

    return m;
}

static int js_init_lvgl_ext(JSContext *ctx, JSModuleDef *m)
{
    
    JS_SetModuleExportList(ctx, m, js_lv_ext_funcs, countof(js_lv_ext_funcs));
    
    return 0;
}

JSModuleDef *js_add_lvgl_ext(JSContext *ctx, const char *module_name)
{
    JSModuleDef *m;

    m = JS_NewCModule(ctx, module_name, js_init_lvgl_ext);
    if (!m)
        return NULL;
    JS_AddModuleExportList(ctx, m, js_lv_ext_funcs, countof(js_lv_ext_funcs));

    return m;
}

void js_reg_datacbk(script_data_callback cbk)
{
	js_data_cbk=cbk;
}

