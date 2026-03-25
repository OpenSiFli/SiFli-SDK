# Graphical Application Framework

## Terminology
App: Refers to the GUI component of an application, corresponding to the View in
the MVC model. It consists of a collection of pages and supports both built-in
and dynamically installed applications.\
\
Page: Refers to a single display page where users can create widgets, register
input device handlers, link data services, and display information (note: this
is distinct from `lv_page`). Each page possesses an independent `lv_screen`,
state handling callbacks, and transition animations.\
\
lv_screen: A virtual screen in LVGL. Only one `lv_screen` can be displayed on a
physical LCD at a time, and input devices can only interact with the currently
active `lv_screen`.



## Features
- Provides scheduling and transition animations for display interfaces between
  and within Apps.
- Keeps track of the order in which Apps are opened and the order in which Pages
  are opened within an App.
- Supports background refreshing of Apps and Pages.

## Restrictions
- An App must contain at least one page.
- A page can belong to any App.
- Only one page can be active at a time.
- Only one App can be active at a time.

## Scheduling Between Pages
For AppA and AppB, each having three pages, labeled A1~A3 and B1~B3
respectively, the diagram below shows the final state of each page in various
scheduling scenarios (Note: the transition states are ignored here).

![Figure 1: Scheduling Between Pages](../../assets/app_fsm.png)


## Page State Machine and State Implementation Guidelines
-   **entryfunction()**: The function that starts the page. If no parameters are
    needed, this step can be merged with `on_start`.
    -   Allocate memory
    -   Receive processing parameters

-   **on_start()**:
    -   Create and layout `lv` controls within the current page.
    -   Subscribe to services.
    -   Request data from services.

-   **on_resume()**:
    -   Run internal `lv_task` or timers within the App.

-   **on_pause()**:
    -   Stop or delete internal `lv_task` in the App.

-   **on_stop()**:
    -   Unsubscribe from services.
    -   Release memory.
    -   Created `lv` controls will be automatically deleted, no need for manual
        deletion.

![Figure 2: Page Internal State Machine](../../assets/app_page_fsm.png)

Description of other handler functions in the illustrated example:
-   xxx_service_callback(): A handler function for subscribing to services
    within a Page. @see data_service

-   Page’s lv_task(): An optional `lv_task` handler function within the Page.

-   lv_obj_callback(): An event handler function for LVGL objects.

## Application Example

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
            data_subscribe_rsp_t *rsp;
            rsp = (data_subscribe_rsp_t *)arg->data;
            RT_ASSERT(rsp);
            /* Subscribe data error */
            if (rsp->result < 0)
            {
                p_compass->srv_handle = DATA_CLIENT_INVALID_HANDLE;
            }
            else
            {
                /* Request compass degree */
                compass_request_data();
            }

            break;
        }

        case MSG_SRV_COMPASS_CUR_VAL_GET_RSP:
        {
            compass_data_t *data = (compass_data_t *)arg->data;

            if(data)
            {
                /* Valid data */
                if(data->accuracy > 0)
                {
                    uint32_t degree = (3600 - data->azimuth * 10);

                    /* Need to update UI */
                    if(degree != p_compass->last_degree)
                    {
                        char text_buf[20];

                        /* Update compass arrow angle */
                        lv_img_set_angle(p_compass->img_arrow, degree);

                        /* Update compass angle label text */
                        sprintf(text_buf, "%d d", degree);
                        lv_label_set_text(p_compass->title_text, text_buf);

                        p_compass->last_degree = degree;
                    }
                }
            }

            /* Request compass degree again */
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

    /* Alloc app memory */
    p_compass = rt_malloc(sizeof(app_compass_t));
    memset(p_compass, 0, sizeof(app_compass_t));

    /* Create UI widgets */
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

    /* Subscribe to service data */
    p_compass->srv_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != p_compass->srv_handle);

    ui_datac_subscribe(p_compass->srv_handle, "COMP", 
                      compass_data_callback, 0);
}

static void on_pause(void)
{
    /* Free SRAM image to save heap memory */
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

    /* Copy image to SRAM to speed up */
    p_compass->p_img_arrow_dsc = lv_img_buf_alloc(org_img->header.w,
                                                org_img->header.h, 
                                                org_img->header.cf);

    memcpy((void *)p_compass->p_img_arrow_dsc->data, org_img->data, org_img->data_size);

    /* Update image's source and coordinates */
    lv_img_set_src(p_compass->img_arrow, p_compass->p_img_arrow_dsc);
    lv_obj_align(p_compass->img_arrow, NULL, LV_ALIGN_CENTER, 0, 0);
}

static void on_stop(void)
{
    /* Unsubscribe from service before exiting app */
    if (DATA_CLIENT_INVALID_HANDLE != p_compass->srv_handle)
    {
        datac_unsubscribe(p_compass->srv_handle);
        datac_close(p_compass->srv_handle);
        p_compass->srv_handle = DATA_CLIENT_INVALID_HANDLE;
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
    /* Register root page message handler */
    gui_app_regist_msg_handler(argv[0], msg_handler);

    return 0;
}

/* Register compass app */
BUILTIN_APP_EXPORT("compass", LV_EXT_IMG_GET(img_compass), "compass", app_main);
```

## API Reference
[Middleware GUI](middleware-gui)
