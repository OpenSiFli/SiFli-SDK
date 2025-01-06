#include <rtthread.h>
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "dfs_file.h"
#include "quickjs.h"
#include "cutils.h"
#include "gui_app_fwk.h"

#define DBG_TAG           "QJS.APP"
//#define DBG_LVL           DBG_LOG
#define DBG_LVL           DBG_INFO
#include "rtdbg.h"


typedef struct {
    JSValueConst this_obj;
    JSValue callback;
    lv_obj_t * lvobj;
    lv_timer_t * lvtask;
    JSContext *ctx;
    int watch_app;
} JSlvapp;

static JSValue js_lv_app_method(JSContext* ctx, JSValueConst this_val,
    int argc, JSValueConst* argv, int magic);


enum {
	LVAPP_start=1,
	LVAPP_stop,
	LVAPP_resume,
	LVAPP_pause,
	LVAPP_root,
	LVAPP_task,
	LVAPP_memory,
}JS_lv_app_methods_enum;

static JSClassID js_lv_app_class_id;


const JSCFunctionListEntry js_lv_app_methods[]={
	JS_CFUNC_MAGIC_DEF("start", 0, js_lv_app_method,LVAPP_start),
	JS_CFUNC_MAGIC_DEF("stop", 0, js_lv_app_method,LVAPP_stop),
	JS_CFUNC_MAGIC_DEF("resume", 0, js_lv_app_method,LVAPP_resume),
	JS_CFUNC_MAGIC_DEF("pause", 0, js_lv_app_method,LVAPP_pause),
	JS_CFUNC_MAGIC_DEF("root", 0, js_lv_app_method,LVAPP_root),
	JS_CFUNC_MAGIC_DEF("task", 1, js_lv_app_method,LVAPP_task),
	JS_CFUNC_MAGIC_DEF("memory", 0, js_lv_app_method,LVAPP_memory),
};


static void lv_app_task_proxy(struct _lv_timer_t * task)
{
    JSlvapp *s = (JSlvapp*)task->user_data;
    JSValue jret = JS_Call(s->ctx, s->callback, s->this_obj, 0, NULL);
    int r;

    JS_ToInt32(s->ctx, &r, jret);
}

static void js_lv_app_finalizer(JSRuntime *rt, JSValue val)
{
    JSlvapp *s = JS_GetOpaque(val, js_lv_app_class_id);
    /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
    if (s->callback!=JS_UNDEFINED)
        JS_FreeValue(s->ctx, s->callback);
    if (s->lvtask)
        lv_timer_del(s->lvtask);
    if (s->lvobj && s->watch_app==0)
        lv_obj_del(s->lvobj);
    js_free_rt(rt, s);
    LOG_D("lvapp desctructor\n");
}

static JSClassDef js_lv_app_class = {
    "lvapp",
    .finalizer = js_lv_app_finalizer,
};

static JSValue js_lv_app_method(JSContext *ctx, JSValueConst this_val,
                             int argc, JSValueConst *argv, int magic)
{
	JSlvapp *s = JS_GetOpaque2(ctx, this_val, js_lv_app_class_id);
    JSValue r=JS_UNDEFINED;

    LOG_D("APP base %s(%d)\n",js_lv_app_methods[magic - 1].name,magic );
    switch (magic) {
		case LVAPP_start:
		case LVAPP_stop:
		case LVAPP_resume:
		case LVAPP_pause:
			break;
		case LVAPP_root:
		{
		    r=JS_NewInt32(ctx,(int)s->lvobj);
		    LOG_D("APP Get Root:%p\n", s->lvobj);
			break;
		}
		case LVAPP_task:
		{

            int interval;

			if (s->lvtask) {
                LOG_D("APP Task delete\n");
                lv_timer_del(s->lvtask);
                s->lvtask=NULL;
				JS_FreeValue(ctx, s->callback);
                s->callback=JS_UNDEFINED;
            }
            if (argc>1) {
                LOG_D("APP Task create\n");
			    s->callback=JS_DupValue(ctx, (JSValueConst)argv[0]);
			    s->ctx=ctx;
			    s->this_obj=this_val;
                JS_ToInt32(ctx, (int32_t*)&interval, argv[1]);
			    s->lvtask=lv_timer_create(lv_app_task_proxy, interval,s);
            }

		    break;
		}
        case LVAPP_memory:
        {
            JSMemoryUsage stats;
            JS_ComputeMemoryUsage(JS_GetRuntime(ctx), &stats);
            JS_DumpMemoryUsage(stdout, &stats, JS_GetRuntime(ctx));
        }
		default:
		{
			LOG_E("APP base undefined method\n");
			break;
		}
    }
    return r;
}

static JSValue js_lv_app_ctor(JSContext *ctx,
                             JSValueConst new_target,
                             int argc, JSValueConst *argv)
{
    JSlvapp *s;
    JSValue obj = JS_UNDEFINED;
    JSValue proto;
    int watch_app=0;

    s = js_mallocz(ctx, sizeof(*s));
    if (!s)
        return JS_EXCEPTION;


    /* using new_target to get the prototype is necessary when the
       class is extended. */
    {
        proto = JS_GetPropertyStr(ctx, new_target, "prototype");
        if (JS_IsException(proto))
            goto fail;
        obj = JS_NewObjectProtoClass(ctx, proto, js_lv_app_class_id);
        JS_FreeValue(ctx, proto);
    }

    if (argc>0)
        JS_ToInt32(ctx, (int32_t*)&s->watch_app, argv[0]);
    JS_SetOpaque(obj, s);
    s->ctx = ctx;
    s->callback=JS_NULL;

    if (s->watch_app==0) {  
        // Normal APP use current screen as parent
        s->lvobj=lv_obj_create(lv_scr_act());
    }
    else {
        // Watch APP use system assigned parent.
        s->lvobj=gui_app_get_clock_parent();
    }
    lv_obj_set_size(s->lvobj, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    LOG_D("LV APP base create lv obj:%p, watch:%d\n", s->lvobj, s->watch_app);
    return obj;
fail:
    js_free(ctx, s);
    JS_FreeValue(ctx, obj);
    return JS_EXCEPTION;
}


/********************************************init*********************************************/

static int js_init_lv_app(JSContext *ctx, JSModuleDef *m)
{
    JSValue proto, class;

    /* create the lvobj class */
    JS_NewClassID(&js_lv_app_class_id);
    JS_NewClass(JS_GetRuntime(ctx), js_lv_app_class_id, &js_lv_app_class);

    proto = JS_NewObject(ctx);
    JS_SetPropertyFunctionList(ctx, proto, js_lv_app_methods, countof(js_lv_app_methods));

    class = JS_NewCFunction2(ctx, js_lv_app_ctor, "app", 0, JS_CFUNC_constructor, 0);
    /* set proto.constructor and ctor.prototype */
    JS_SetConstructor(ctx, class, proto);
    JS_SetClassProto(ctx, js_lv_app_class_id, proto);

    JS_SetModuleExport(ctx, m, "app", class);

    return 0;
}

JSModuleDef *js_add_app(JSContext *ctx, const char *module_name)
{
    JSModuleDef *m;

    m = JS_NewCModule(ctx, module_name, js_init_lv_app);
    if (!m)
        return NULL;
    JS_AddModuleExport(ctx, m, "app");
    return m;
}

/*******************************Custom Interface***************************************/
/**
 * Glue the object to the page. After it the page can be moved (dragged) with this object too.
 * @param obj pointer to an object on a page
 * @param glue true: enable glue, false: disable glue
 */
void lv_obj_set_page_glue(lv_obj_t* obj, bool glue)
{
    //lv_obj_set_drag_parent(obj, glue);
    //lv_obj_set_drag(obj, glue);
}

lv_coord_t lv_get_ver_max()
{
    return lv_disp_get_ver_res(NULL);
}

lv_coord_t lv_get_hor_max()
{
    return lv_disp_get_hor_res(NULL);
}


int lv_font_height(int font_size)
{
    return LV_EXT_FONT_GET(font_size)->line_height;
}

int lv_font_width(int font_size, char c)
{
    char text[2];
    if (c == '\0')
        c = 'A';
    text[0] = c;
    text[1] = '\0';
    return lv_font_get_glyph_width(LV_EXT_FONT_GET(font_size), *text, *(text + 1));
}

