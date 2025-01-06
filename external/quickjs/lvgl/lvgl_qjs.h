#ifndef LVGL_QJS_H
#define LVGL_QJS_H

// Exported from original QuickJS
extern int eval_file(JSContext *ctx, const char *filename, int module);
extern int eval_buf(JSContext *ctx, const void *buf, int buf_len, const char *filename, int eval_flags);
extern JSRuntime *quickjs_rt();
extern JSContext *quickjs_ctx(JSRuntime *rt);

// Export interface for LVGL QJS.
extern JSModuleDef *js_add_lvgl(JSContext *ctx, const char *module_name);    
extern JSModuleDef *js_add_lvgl_ext(JSContext *ctx, const char *module_name);    
extern JSModuleDef *js_add_app(JSContext *ctx, const char *module_name);    

#endif
