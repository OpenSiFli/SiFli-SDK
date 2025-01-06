#include "data_prov_int.h"
#include "lvgl.h"
#include "data_service_subscriber.h"

static rt_mq_t g_ui_ds_queue;

#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    static void ui_datac_task(lv_task_t *param)
#else
    static void ui_datac_task(lv_timer_t *param)
#endif /* DISABLE_LVGL_V8 */
{
    data_service_mq_t msg;

    while (rt_mq_recv(g_ui_ds_queue, &msg, sizeof(msg), RT_WAITING_NO) == RT_EOK)
    {
        datac_delayed_usr_cbk(&msg);
    }
}

void ui_datac_init(void)
{
    g_ui_ds_queue = rt_mq_create("uisrv", sizeof(data_service_mq_t), 30, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_ui_ds_queue);
#if defined(DISABLE_LVGL_V8)&&defined(DISABLE_LVGL_V9)
    lv_task_create(ui_datac_task, 15, LV_TASK_PRIO_MID, (void *)0);
#else
    lv_timer_create(ui_datac_task, 15, (void *)0);
#endif /* DISABLE_LVGL_V8 */
}


/*----------------------------APIS-----------------------------------*/
void ui_datac_subscribe(datac_handle_t handle, char *name, data_callback_t cbk, uint32_t user_data)
{
    datac_subscribe_ex(handle, name, cbk, user_data, g_ui_ds_queue);
}



