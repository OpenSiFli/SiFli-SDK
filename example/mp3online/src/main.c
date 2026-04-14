#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "littlevgl2rtt.h"
#include "lv_ex_data.h"
#include "lv_demos.h"
#include "lv_demo_music_main.h"
extern int bt_pan_thread_create(void);
extern rt_err_t mp3_comm_init(void);
rt_mq_t ui_msg_queue = RT_NULL;
extern bool g_pan_connected;
/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* BT PAN thread */
    bt_pan_thread_create();

    /* MP3 thread */
    mp3_comm_init();

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();
    // Create UI message queue
    ui_msg_queue = rt_mq_create("ui_msg", sizeof(ui_msg_t), 20, RT_IPC_FLAG_FIFO);
    if(ui_msg_queue == RT_NULL)
    {
        rt_kprintf("Failed to create UI message queue\n");
        return -1;
    }

    lv_demo_music();

    while (1)
    {
        ui_msg_t msg;

        while(rt_mq_recv(ui_msg_queue, &msg, sizeof(ui_msg_t), 0) == RT_EOK) 
        {
            switch (msg.type)
            {
                case UI_MSG_PLAY_PAUSE:
                    rt_kprintf("PLAY_PAUSE_clik_main\n");
                    extern bool playing;
                    if (playing) {
                        _lv_demo_music_pause();
                    } else {
                        _lv_demo_music_resume();
                    }
                    break;
                    
                case UI_MSG_PREV:
                    rt_kprintf("is_prev_click_main\n");
                    _lv_demo_music_album_next(false);
                    break;
                    
                case UI_MSG_NEXT:
                    rt_kprintf("is_next_click_main\n");
                    _lv_demo_music_album_next(true);
                    break;
                    
                case UI_MSG_UPDATE_ALBUM:
                    rt_kprintf("is_update_album_main\n");
                    // Handle updates for the album and other related messages
                    break;
            }
        }
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}
