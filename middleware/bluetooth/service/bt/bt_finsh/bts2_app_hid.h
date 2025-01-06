#ifndef _BTS2_APP_HID_H_
#define _BTS2_APP_HID_H_



#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_PC_SIMULATOR
#define __packed
#endif

#include "bf0_hal_def.h"

#ifdef  CFG_HID

typedef struct
{
    U8 header;
    U8 report_id;
    U8 buttons;
    S16 dx: 12;
    S16 dy: 12;
    S8 wheel;
    U8 ac_pan;
} __packed hid_msg_mouse_t;


typedef struct
{
    U8 header;
    U8 report_id;
    U8 consumer;
} __packed hid_msg_consumer_t;

typedef struct
{
    U8 report_id;
    U8 data[9];
} hid_msg_touch_t;


#define HID_FRAME_DATA_MAX_LEN 11 /* DM1 is recommended packet */
struct hid_report_data_t
{
    U8 report_id;
    U8 data[HID_FRAME_DATA_MAX_LEN];
    U8 data_len;
};

struct hid_frame_t
{
    U8 header;
    U8 data[HID_FRAME_DATA_MAX_LEN];
    U8 data_len;
};

extern void bt_hid_msg_handler(bts2_app_stru *bts2_app_data);
void bt_hid_init(bts2_app_stru *bts2_app_data);
void bt_hid_open(void);
void bt_hid_cmpose_hid_descriptor(void);
void bt_hid_add_descriptor(const U8 *data, U8 len);
void bt_hid_close(void);
void bt_hid_conn_2_dev(BTS2S_BD_ADDR *bd);
void bt_hid_disc_2_dev(BTS2S_BD_ADDR *bd_addr);

void bt_hid_mouse_reset(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_reset_at_middle(bts2_app_stru *bts2_app_data);
void bt_hid_timeout_handler_reset_at_middle_ios(void *parameter);
void bt_hid_timeout_handler_reset_at_middle_ios1(void *parameter);
static const U16 bt_hid_mouse_get_drag_speed(void);
void bt_hid_mouse_move(bts2_app_stru *bts2_app_data, S16 dx, S16 dy);
void bt_hid_mouse_move_without_reset(bts2_app_stru *bts2_app_data, S16 dx, S16 dy);
void bt_hid_mouse_left_click(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_right_click(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_middle_button_click(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_left_double_click(bts2_app_stru *bts2_app_data);
void bt_hid_timeout_handler(void *parameter);
void bt_hid_mouse_right_double_click(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_drag_page(bts2_app_stru *bts2_app_data, U8 buttons, S16 dx, S16 dy, S8 wheel_offset);
// void bt_hid_reset_num_count_drag_down(void);
void bt_hid_mouse_drag_page_up(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_drag_page_down(bts2_app_stru *bts2_app_data);
void bt_hid_timeout_handler_drag_up(void *parameter);
void bt_hid_timeout_handler_drag_down(void *parameter);
void bt_hid_timeout_handler_reset_at_middle1(void *parameter);
void bt_hid_timeout_handler_reset_at_middle2(void *parameter);
void bt_hid_timeout_handler_reset_report(void *parameter);
BOOL bt_hid_check_is_ios_device(void);
void bt_hid_set_ios_device(U8 is_ios);
void bt_hid_consumer_report_reset(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_power_onoff(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_play_status(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_next_track(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_back_track(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_volume_down(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_volume_up(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_forward(bts2_app_stru *bts2_app_data);
void bt_hid_consumer_report_go_back(bts2_app_stru *bts2_app_data);


void bt_hid_receive_contro_handle(bts2_app_stru *bts2_app_data);
void hid_send_handshake(bts2_app_stru *bts2_app_data, hid_handshake_param_type_enum_t code);
void hid_receive_get_report_handle(bts2_app_stru *bts2_app_data);
void hid_receive_set_report_handle(bts2_app_stru *bts2_app_data);
void hid_receive_get_protocol_handle(bts2_app_stru *bts2_app_data);
void hid_receive_set_protocol_handle(bts2_app_stru *bts2_app_data);
void hid_receive_interrupt_report(bts2_app_stru *bts2_app_data);
void hid_send_response_report(bts2_app_stru *bts2_app_data, hid_report_type_enum_t report_type, struct hid_report_data_t *report_data, BOOL has_report_id);


void bt_hid_mouse_test1(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test2(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test3(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test4(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test5(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test6(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test7(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test8(bts2_app_stru *bts2_app_data);
void bt_hid_mouse_test9(bts2_app_stru *bts2_app_data);



#endif

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/