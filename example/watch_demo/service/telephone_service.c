#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include "telephone_service.h"
#define DBG_TAG           "DS.TEL"
#define DBG_LVL           DBG_LOG
#include "rtdbg.h"

#define CON_UART_NAME                 "uart2"
#define DATA_CMD_END                     '\r'
#define ONE_DATA_MAXLEN                  64

#define AT_CMD_ANSWER_CALL "AT+BTACCEPTCALL\r"
#define AT_CMD_REJECT_CALL      "AT+BTREJECTCALL\r"
#define AT_CMD_END_CALL      "AT+BTTERMINALCALL\r"
#define AT_CMD_OUTGOINGCALL        "AT+BTCALL=5,%s\r"

static datas_handle_t this_service = NULL;
static struct rt_semaphore tx_sem;
static rt_device_t serial;
static rt_timer_t rsp_timer;
static char uart_rx_data[ONE_DATA_MAXLEN];
static uint8_t uart_rx_i = 0;

static void rsp_timeout_ind(void *param)
{
    rt_err_t result = -RT_ERROR;

}


static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        datas_ind_size(this_service, size);
    }
    return RT_EOK;
}


/*
    replace cotinued control mark&space mark with a space
*/
static void data_convert(char *data, uint16_t len)
{
    uint16_t i, j;
    uint8_t prev_char_is_space = 1;

    for (i = 0, j = 0; i < len; i++)
    {
        switch (data[i])
        {
        case ' ':
        case '\t':
        case '\r':
        case '\n':
        case ',':
        case '\"':
        {
            if (prev_char_is_space)
            {
                ;
            }
            else
            {
                data[j++] = ' ';
                prev_char_is_space = 1;
            }
        }
        break;

        default:
        {
            data[j++] = data[i];
            prev_char_is_space = 0;
        }
        break;
        }
    }

    while (j < len) data[j++] = '\0';

}

static void at_cmd_parsing(char *data, uint16_t len)
{
    char *ptr;

    data_convert(data, len);

    if (rt_strlen(data) < 1) return;

    LOG_D("recv at_cmd:[%s]", data);

    if ((ptr = rt_strstr(data, "+CIEV:")) != NULL)
    {
        int a, b;

        a = -1;
        b = -1;

        sscanf(ptr, "+CIEV: %d %d", &a, &b);


        LOG_D("update call status[%d]", b);

        datas_push_msg_to_client(this_service, TELEPHONE_MSG_CALL_STATUS_IND, sizeof(b), (uint8_t *) &b);
    }

    if ((ptr = rt_strstr(data, "+CLIP:")) != NULL)
    {
        char a[32];
        int b;

        memset(a, 0, sizeof(a));
        b = -1;

        sscanf(ptr, "+CLIP: %s %d", a, &b);

        LOG_D("incoming call: [%s]", a);

        datas_push_msg_to_client(this_service, TELEPHONE_MSG_CALL_INCOMMING, sizeof(a), (uint8_t *)&a[0]);
    }

    if (((ptr = rt_strstr(data, "OK")) != NULL) || ((ptr = rt_strstr(data, "ERROR")) != NULL))
    {
        LOG_D("recv at cmd rsp. release tx_sem");
        rt_sem_release(&tx_sem);
    }

    if ((ptr = rt_strstr(data, "AST_POWERON")) != NULL)
    {
        LOG_D("AST_POWERON");
    }

}



static rt_err_t send_at_cmd(const char *req_str, uint16_t len)
{
    rt_err_t err = rt_sem_take(&tx_sem, rt_tick_from_millisecond(100));
    if (RT_EOK != err)
    {
        LOG_D("wait request responese TIMEOUT. RESET tx_sem");
        rt_sem_control(&tx_sem, RT_IPC_CMD_RESET, RT_NULL);
    }

    //try to send anyway
    err = (len == rt_device_write(serial, 0, req_str, len)) ? RT_EOK : RT_EIO;

    LOG_D("send at_cmd[%s]", req_str);

    if (RT_EOK != err)
        LOG_D(" err=%d", err);
    else
        LOG_D(" ");

    return err;
}

static int32_t msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }
    case MSG_SERVICE_TX_REQ:
    {
        //send_request
        break;
    }
    case MSG_SERVICE_RX_REQ:
    {
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));

        RT_ASSERT(data_ind);

        if (data_ind->len > 0)
        {
            char ch;

            while (rt_device_read(serial, 0, &ch, 1) > 0)
            {
                if (ch == DATA_CMD_END)
                {
                    uart_rx_data[uart_rx_i] = '\0';
                    at_cmd_parsing(uart_rx_data, (uint16_t) uart_rx_i);
                    uart_rx_i = 0;
                    continue;
                }
                uart_rx_i = (uart_rx_i >= ONE_DATA_MAXLEN - 1) ? ONE_DATA_MAXLEN - 1 : uart_rx_i;
                uart_rx_data[uart_rx_i++] = ch;
            }
        }
        break;
    }

    case TELEPHONE_MSG_CALL_OUTGOING:
    {
        rt_err_t err;
        char *tel_num = (char *)(data_service_get_msg_body(msg));
        char at_cmd[64];

        memset(at_cmd, 0, sizeof(at_cmd));
        //strcpy(at_cmd,"AT+BTCALL=5,");
        //strcat(at_cmd,tel_num);
        rt_sprintf(at_cmd, AT_CMD_OUTGOINGCALL, tel_num);
        err = send_at_cmd(at_cmd, strlen(at_cmd));

        datas_send_response(this_service, msg, err);
    }
    break;

    case TELEPHONE_MSG_ANSWER_CALL:
    {
        rt_err_t err;
        err = send_at_cmd(AT_CMD_ANSWER_CALL, strlen(AT_CMD_ANSWER_CALL));
        datas_send_response(this_service, msg, err);
    }
    break;



    case TELEPHONE_MSG_END_CALL:
    {
        rt_err_t err;
        err = send_at_cmd(AT_CMD_END_CALL, strlen(AT_CMD_END_CALL));
        datas_send_response(this_service, msg, err);
    }
    break;



    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t telephone_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = NULL,
    .msg_handler = msg_handler,
};

static int telephone_service_register(void)
{
    rt_err_t ret = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;


    serial = rt_device_find(CON_UART_NAME);
    if (!serial)
    {
        LOG_D("find %s failed!\n", CON_UART_NAME);
        return RT_ERROR;
    }

    rt_sem_init(&tx_sem, "teltx", 1, RT_IPC_FLAG_FIFO);

    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);

    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_set_rx_indicate(serial, uart_rx_ind);

    rsp_timer = rt_timer_create("telreq", rsp_timeout_ind, 0, rt_tick_from_millisecond(1000),  RT_TIMER_FLAG_SOFT_TIMER);
    this_service = datas_register("telephone", &telephone_service_cb);


    return ret;
}

INIT_COMPONENT_EXPORT(telephone_service_register);
