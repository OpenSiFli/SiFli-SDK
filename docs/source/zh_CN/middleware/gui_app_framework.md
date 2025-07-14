# 图形应用框架

## 名词解释
App       -  指一个应用程序的GUI部分，即MVC模型中的View部分，它是page的集合。支持内置App以及动态安装App。
Page      -  指一个显示页面，允许用户在上面创建控件、注册输入设备处理、链接数据服务并显示，（注意这不是lv_page）。每个page拥有独立的lv_screen，状态处理回调函数，以及转场动画
lv_screen -  littleVGL上的虚拟屏幕，一个物理LCD上同一时间只能显示一个lv_screen的内容，输入设备只能操作当前显示的lv_screen



## 功能
- 提供App之间、App内部的显示界面的调度以及转场动画
- 提供App打开顺序、App内部Page打开顺序的记录
- 支持后台刷新App/page

## 限制
- 一个App至少包含一个page
- 一个page可以从属任意App
- 同时只能有一个page处于active状态
- 同时只能有一个App处于avtive状态


## Page间的调度
AppA、AppB各有3个page，分别表示为A1~A3,B1~B3
下图显示了各种调度情况下各Page的最终状态(注意此处忽略了过渡态)

![Figure 1: Page间的调度](../../assets/app_fsm.png)

	
## Page状态机和各状态的实现规范
-	entryfunction() - 启动page函数, 如果不需要参数，此步骤可以并入on_start 
    -	分配内存
    -	接收处理参数

-	on_start()
    -	创建/布局当前page内的lv控件
    -	订阅service
    -	向service request 数据

-	on_resume()
    -	运行app内部的lv_task/timer

-	on_pause()
    -	停止（或删除）app内部lv_task

-	on_stop()
    -	退订service
    -	释放内存
    -	创建的lv控件会自动删除，可以不用主动删除

![Figure 2: Page内部状态机](../../assets/app_page_fsm.png)

图片示例中其他处理函数说明：
-	xxx_service_callback()
Page内订阅服务的处理函数  @see data_service

-	Page’s lv_task()
Page内可选的lv_task处理函数

-	lv_obj_callback()
littleVGL object的事件处理函数
	

## Application example
  
```c

typedef struct
{
    lv_obj_t *title_text;
    lv_obj_t *img_arrow;

    lv_img_dsc_t *p_img_arrow_dsc;

    datac_handle_t srv_handle;
    uint32_t last_degree;
} app_compass_t;


static app_compass_t *p_compass = NULL;

static void compass_request_data(void)
{
    data_msg_t msg;
    data_service_init_msg(&msg, MSG_SRV_COMPASS_CUR_VAL_GET_REQ, 0);
    datac_send_msg(p_compass->srv_handle, &msg);
}

static int compass_data_callback(data_callback_arg_t *arg)
{
    switch (arg->msg_id)
    {
        case MSG_SERVICE_SUBSCRIBE_RSP:
        {
            data_rsp_t *rsp;
            rsp = (data_rsp_t *)arg->data;
            RT_ASSERT(rsp);
            /* Subscribe data error*/
            if (rsp->result < 0)
            {
                p_compass->srv_handle = DATA_CONN_INVALID_ID;
            }
            else
            {
                /* Request compass degree*/
                compass_request_data();
            }

            break;
        }

        case MSG_SRV_COMPASS_CUR_VAL_GET_RSP:
        {
            compass_data_t *data = (compass_data_t *)arg->data;

            if(data)
            {
                /* Valid data*/
                if(data->accuracy > 0)
                {
                    uint32_t degree = (3600 - data->azimuth * 10);

                    /* Need update UI*/
                    if(degree != p_compass->last_degree)
                    {
                        char text_buf[20];

                        /* Update compass arrow angle*/
                        lv_img_set_angle(p_compass->img_arrow, degree);

                        /* Update compass angle label text*/
                        sprintf(text_buf, "%d d", degree);
                        lv_label_set_text(p_compass->title_text, text_buf);

                        p_compass->last_degree = degree;
                    }
                }
            }

            /* Request compass degree again*/
            compass_request_data();
        }
        break;

        default:
            break;
    }
    return 0;
}


static void on_start(void)
{
    if (NULL != p_compass)
        rt_free(p_compass);

    /* Alloc app memory*/
    p_compass = rt_malloc(sizeof(app_compass_t));
    memset(p_compass, 0, sizeof(app_compass_t));

    /* Create UI widgets*/
    {
        lv_obj_t *obj;
    
        obj = lv_img_create(lv_scr_act(), NULL);
        lv_img_set_src(obj, LV_EXT_IMG_GET(compass_bg));
        lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);
    
        obj = lv_img_create(lv_scr_act(), NULL);
        p_compass->img_arrow = obj;
    
        obj = lv_label_create(lv_scr_act(), NULL);
        lv_label_set_text(obj, "230 d");
        lv_obj_align(obj, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
        p_compass->title_text = obj;
    }

    /* Subscribe service data*/
    p_compass->srv_handle = ui_datac_subscribe("COMP", 
                                                compass_data_callback, 0);
}

static void on_pause(void)
{
    /*Free SRAM image to save heap memory*/
    lv_img_set_src(p_compass->img_arrow, NULL);

    if(NULL != p_compass->p_img_arrow_dsc)
    {
        lv_img_buf_free(p_compass->p_img_arrow_dsc);
        p_compass->p_img_arrow_dsc = NULL;
    }
}

static void on_resume(void)
{
    const lv_img_dsc_t *org_img;

    org_img = LV_EXT_IMG_GET(compass_arrow);


    /*Copy image to SRAM to speed up*/
    p_compass->p_img_arrow_dsc = lv_img_buf_alloc(org_img->header.w,
                                                org_img->header.h, 
                                                org_img->header.cf);
                                                
    memcpy((void *)p_compass->p_img_arrow_dsc->data, org_img->data, org_img->data_size);

    /* Update image's source and coordinates.*/
    lv_img_set_src(p_compass->img_arrow, p_compass->p_img_arrow_dsc);
    lv_obj_align(p_compass->img_arrow, NULL, LV_ALIGN_CENTER, 0, 0);
}

static void on_stop(void)
{
    /*unsubscribe service before exit app*/
    if (DATA_CONN_INVALID_ID != p_compass->srv_handle)
    {
        datac_unsubscribe(p_compass->srv_handle);
        p_compass->srv_handle = DATA_CONN_INVALID_ID;
    }

    if (NULL != p_compass)
    {
        rt_free(p_compass);
        p_compass = NULL;
    }
}

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}


static int app_main(int argc, char *argv[])
{
    /* Regist root page message handler */
    gui_app_regist_msg_handler(argv[0], msg_handler);

    return 0;
}


/* Regist compass app */
BUILTIN_APP_EXPORT("compass", LV_EXT_IMG_GET(img_compass), "compass", app_main);

```

## API参考
[](middleware-gui)
