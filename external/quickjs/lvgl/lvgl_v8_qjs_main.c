#include <rtthread.h>
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "dfs_file.h"
#include "quickjs.h"
#include "cutils.h"
#include "lvgl_qjs.h"

#define DBG_TAG           "QJS"
#define DBG_LVL           DBG_LOG
//#include "rtdbg.h"

#define LV_QJS_TASK_INTERVAL 30


static JSRuntime *qjs_rt;
static  JSContext *qjs_ctx;
static JSContext * qjs_lv_init(JSRuntime *rt);


/************************* QuickJS application support *******************************************/

static const char qjs_app_prefix[]="JA_";
static char cbk_func[64];
    
static void qjs_app_init(void *param)
{
	char* name = (char*)param;
    if (qjs_ctx==NULL) {
		if (qjs_rt==NULL)
			qjs_rt = quickjs_rt();
        qjs_ctx=qjs_lv_init(qjs_rt);
   	}
	rt_sprintf(cbk_func, "globalThis.%s;", name + sizeof(qjs_app_prefix) - 1);
	JSValue loaded= JS_Eval(qjs_ctx, cbk_func, strlen(cbk_func),"<input>",0);
    if (JS_VALUE_GET_PTR(loaded)) {
        rt_kprintf("----loaded\n");
    }
    else {
		strcpy(cbk_func, "/");
		strcat(cbk_func, (const char*)param);
        strcat(cbk_func, "/main.js");
        rt_kprintf("Quick js run %s\n", cbk_func);
        eval_file(qjs_ctx,(const char *)cbk_func, -1);
    }
    JS_FreeValue(qjs_ctx, loaded);
}

void qjs_msg_handler(gui_app_msg_type_t msg, void *param)
{
    
    rt_kprintf("Message %d in Context %p\n", msg,qjs_ctx);
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        qjs_app_init(param);
		rt_sprintf(cbk_func, "%s=new %s();\n", (const char*)param, (const char*)param+sizeof(qjs_app_prefix)-1);
		rt_kprintf(cbk_func);
		eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
        rt_sprintf(cbk_func, "%s.start();\n", (const char*)param);
		rt_kprintf(cbk_func);
		eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
		break;

    case GUI_APP_MSG_ONRESUME:
        rt_sprintf(cbk_func, "%s.resume();\n", (const char*)param);
		rt_kprintf(cbk_func);
		eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
		break;
	case GUI_APP_MSG_ONPAUSE:
		rt_sprintf(cbk_func, "%s.pause();\n", (const char*)param);
		rt_kprintf(cbk_func);
		eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
		break;
	case GUI_APP_MSG_ONSTOP:
		rt_sprintf(cbk_func, "%s.stop();\ndelete %s;\n", (const char*)param,(const char*)param);
		rt_kprintf(cbk_func);
		eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);    
		JS_RunGC(qjs_rt);
#if 1
		{
			extern void list_memheap(void);
			list_memheap();
		}
#endif
        break;
	default:
		rt_kprintf("QJS:Unknown message:%d\n", msg);
        break;
    }
}

static int qjs_app_main(intent_t i)
{
    /* Regist root page message handler */
    gui_app_regist_msg_handler((const char *)i, qjs_msg_handler);
    return 0;
}

#include "lv_ext_resource_manager.h"

const builtin_app_desc_t *gui_qjs_app_list_get_next(const builtin_app_desc_t *ptr_app)
{
	static struct dfs_fd fd;
	static struct dirent dirent;
	static builtin_app_desc_t desc;
	static char name[80];

	int length;
	builtin_app_desc_t * r=NULL;
    /* list directory */
    if ((int)ptr_app==-1) {
        dfs_file_close(&fd);
        return r;
    }
	if (ptr_app==NULL) {        
		if (dfs_file_open(&fd, "/", O_DIRECTORY) != 0)
			return r;
    }
    
    memset(&dirent, 0, sizeof(struct dirent));
    length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
	while (length>0&&strncmp(dirent.d_name,qjs_app_prefix, strlen(qjs_app_prefix)))
		length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));		
    if (length > 0) {		
		r = &desc;
		r->entry=qjs_app_main;
		r->icon= name;
		strcpy((char*)r->icon, "/");
		strcat((char*)r->icon,dirent.d_name);
		strcat((char*)r->icon,"/thumb.bin");
		strcpy(r->id,dirent.d_name);
#ifdef _MSC_VER		
		r->magic_flag=MSC_APP_STRUCT_MAGIC_HEAD;
#endif
		rt_kprintf("name: %s\n", dirent.d_name);
    }
	else {
		dfs_file_close(&fd);
	}
    return (const builtin_app_desc_t *)r;
}

/************************* QuickJS watch face support *******************************************/

const char qjs_watch_prefix[]="JW_";

#ifdef SOLUTION_WATCH
#include "app_clock_comm.h"
#else
#include "app_clock_main.h" 
#endif

rt_int32_t jw_init(lv_obj_t * parent)
{
    char * name=app_clock_change_context();
    qjs_app_init((void*)name);
    rt_sprintf(cbk_func, "%s=new %s(%d);\n", name, name+sizeof(qjs_watch_prefix)-1, (int)parent);
    eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
    rt_sprintf(cbk_func, "%s.start();\n", name);
    eval_buf(qjs_ctx, cbk_func, strlen(cbk_func), "<input>", 0);
	return RT_EOK;
}

rt_int32_t jw_pause(void)
{
	qjs_msg_handler(GUI_APP_MSG_ONPAUSE, (void*)app_clock_change_context());
	return RT_EOK;
}

rt_int32_t jw_resume(void)
{
	qjs_msg_handler(GUI_APP_MSG_ONRESUME, (void*)app_clock_change_context());
	return RT_EOK;
}
rt_int32_t jw_deinit(void)
{
	qjs_msg_handler(GUI_APP_MSG_ONSTOP, (void*)app_clock_change_context());
	return RT_EOK;
}

static app_clock_ops_t qjs_watch = {
	.init=jw_init,
	.pause=jw_pause,
	.resume=jw_resume,
	.deinit=jw_deinit,
};

void gui_qjs_watch_face_register()
{
	static struct dfs_fd fd;
	static struct dirent dirent;

	int length;
	if (dfs_file_open(&fd, "/", O_DIRECTORY) != 0)
		return ;
    memset(&dirent, 0, sizeof(struct dirent));
    length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
	while (length>0) {
		if (strncmp(dirent.d_name,qjs_watch_prefix, strlen(qjs_watch_prefix))==0)
		{
#ifdef SOLUTION_WATCH		
		    // name will be freed when watch face was deleted. see app_tileview_clock_list_del
			char * name=rt_malloc(strlen(dirent.d_name)+strlen("/thumb.bin")+2);
			
			strcpy(name, "/");
			strcat(name,dirent.d_name);
			strcat(name,"/thumb.bin");
			app_clock_register(dirent.d_name,
				NULL,
				&qjs_watch,
				(const void *)name				
				);
#else
			app_clock_register(dirent.d_name, &qjs_watch);
#endif
		}
		length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));		
    }
	dfs_file_close(&fd);
}

/************************* Quick JS main entry********* *******************************************/

static JSContext * qjs_lv_init(JSRuntime *rt)
{
    JSContext * r = quickjs_ctx(rt);

	js_add_lvgl(r, "lv");
    js_add_lvgl_ext(r, "lvext");
    js_add_app(r, "lvapp");
#ifdef 	QUICKJS_PSRAM_SIZE
	JS_SetGCThreshold(rt,QUICKJS_PSRAM_SIZE*3/4);
#else
	JS_SetGCThreshold(rt,256*1024);
#endif
	return r;
}




