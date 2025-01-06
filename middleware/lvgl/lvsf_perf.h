#ifndef LVSF_PERF_H
#define LVSF_PERF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#ifdef PKG_USING_SYSTEMVIEW


void lv_debug_task_create(const lv_timer_t *t);
void lv_debug_task_terminate(const lv_timer_t *t);
void lv_debug_task_start_exec(const lv_timer_t *t);
void lv_debug_task_stop_exec(const lv_timer_t *t);
void lv_debug_mark_start(uint32_t id, const char *desc);
void lv_debug_mark_stop(uint32_t id);
void lv_debug_gpu_start(uint8_t type, int16_t angle, uint32_t scale, const lv_area_t *output_coords);
void lv_debug_gpu_stop(void);
void lv_debug_lcd_flush_start(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void lv_debug_lcd_flush_stop(void);

extern void lv_debug_obj_start_draw(const lv_obj_t *obj, const lv_area_t *mask);
extern void lv_debug_obj_stop_draw(const lv_obj_t *obj, const lv_area_t *mask);
extern void lv_debug_vdb_start_flush(const lv_color_t *color_p, const lv_area_t *area);
extern void lv_debug_vdb_stop_flush(const lv_color_t *color_p, const lv_area_t *area);


#define LV_DEBUG_TASK_CREATE(task)     lv_debug_task_create(task)
#define LV_DEBUG_TASK_TERMINATE(task)  lv_debug_task_terminate(task)
#define LV_DEBUG_TASK_START_EXEC(task) lv_debug_task_start_exec(task)
#define LV_DEBUG_TASK_STOP_EXEC(task)  lv_debug_task_stop_exec(task)

#define LV_DEBUG_MARK_START(id,desc)   lv_debug_mark_start(id,desc)
#define LV_DEBUG_MARK_STOP(id)         lv_debug_mark_stop(id)

#define LV_DEBUG_GPU_START(type, p1, p2, output_coords) lv_debug_gpu_start((type), (int16_t)(p1), (uint32_t)(p2), (output_coords))
#define LV_DEBUG_GPU_STOP()  lv_debug_gpu_stop()

#define LV_DEBUG_LCD_FLUSH_START(x1,y1,x2,y2) lv_debug_lcd_flush_start((x1),(y1),(x2),(y2))
#define LV_DEBUG_LCD_FLUSH_STOP()  lv_debug_lcd_flush_stop()


#define LV_DEBUG_OBJ_START_DRAW(obj,mask) lv_debug_obj_start_draw(obj,mask)
#define LV_DEBUG_OBJ_STOP_DRAW(obj,mask)  lv_debug_obj_stop_draw(obj,mask)

#define LV_DEBUG_VDB_START_FLUSH(pixels,area)  lv_debug_vdb_start_flush(pixels,area)
#define LV_DEBUG_VDB_STOP_FLUSH(pixels,area)   lv_debug_vdb_stop_flush(pixels,area)

#elif defined(USING_PROFILER)

void lv_debug_task_create(lv_timer_t *t);
void lv_debug_task_terminate(lv_timer_t *t);
void lv_debug_task_start_exec(lv_timer_t *t);
void lv_debug_task_stop_exec(lv_timer_t *t);
void lv_debug_gpu_start(uint8_t type, int16_t angle, uint32_t scale, const lv_area_t *output_coords);
void lv_debug_gpu_stop(void);
void lv_debug_lcd_flush_start(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
void lv_debug_lcd_flush_stop(void);

extern void lv_debug_obj_start_draw(lv_obj_t *obj, const lv_area_t *mask);
extern void lv_debug_obj_stop_draw(lv_obj_t *obj, const lv_area_t *mask);
extern void lv_debug_vdb_start_flush(lv_color_t *color_p, const lv_area_t *area);
extern void lv_debug_vdb_stop_flush(lv_color_t *color_p, const lv_area_t *area);



#define LV_DEBUG_TASK_CREATE(task)     lv_debug_task_create(task)
#define LV_DEBUG_TASK_TERMINATE(task)  lv_debug_task_terminate(task)
#define LV_DEBUG_TASK_START_EXEC(task) lv_debug_task_start_exec(task)
#define LV_DEBUG_TASK_STOP_EXEC(task)  lv_debug_task_stop_exec(task)

#define LV_DEBUG_GPU_START(type, p1, p2, output_coords) lv_debug_gpu_start((type), (int16_t)(p1), (uint32_t)(p2), (output_coords))
#define LV_DEBUG_GPU_STOP()  lv_debug_gpu_stop()

#define LV_DEBUG_LCD_FLUSH_START(x1,y1,x2,y2) lv_debug_lcd_flush_start((x1),(y1),(x2),(y2))
#define LV_DEBUG_LCD_FLUSH_STOP()  lv_debug_lcd_flush_stop()


#define LV_DEBUG_MARK_START(id,desc)
#define LV_DEBUG_MARK_STOP(id)

#define LV_DEBUG_OBJ_START_DRAW(obj,mask) lv_debug_obj_start_draw(obj,mask)
#define LV_DEBUG_OBJ_STOP_DRAW(obj,mask)  lv_debug_obj_stop_draw(obj,mask)

#define LV_DEBUG_VDB_START_FLUSH(pixels,area)  lv_debug_vdb_start_flush(pixels,area)
#define LV_DEBUG_VDB_STOP_FLUSH(pixels,area)   lv_debug_vdb_stop_flush(pixels,area)

#else

#define LV_DEBUG_TASK_CREATE(task)
#define LV_DEBUG_TASK_TERMINATE(task)
#define LV_DEBUG_TASK_START_EXEC(task)
#define LV_DEBUG_TASK_STOP_EXEC(task)

#define LV_DEBUG_GPU_START(type, p1, p2, output_coords)
#define LV_DEBUG_GPU_STOP()

#define LV_DEBUG_MARK_START(id,desc)
#define LV_DEBUG_MARK_STOP(id)

#define LV_DEBUG_LCD_FLUSH_START(x1,y1,x2,y2)
#define LV_DEBUG_LCD_FLUSH_STOP()


#define LV_DEBUG_OBJ_START_DRAW(obj,mask)
#define LV_DEBUG_OBJ_STOP_DRAW(obj,mask)

#define LV_DEBUG_VDB_START_FLUSH(pixels,area)
#define LV_DEBUG_VDB_STOP_FLUSH(pixels,area)

#endif



#define PRINT_AREA(s,area) //rt_kprintf("%s \t x1y1=%d,%d  x2y2=%d,%d  \n",s,(area)->x1,(area)->y1,(area)->x2,(area)->y2)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_PERF_H*/

