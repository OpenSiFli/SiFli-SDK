
#ifndef __APP_CMD_SEND_H__
#define __APP_CMD_SEND_H__

void comm_send_ext_packet(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen), uint8_t data_array[], uint8_t data_len);

void comm_send_app_cmd(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen), uint8_t dev_state, uint8_t data_array[], uint8_t data_len);

void comm_send_app_cmd_auto_led_fail(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen));
void comm_send_app_cmd_reset(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen));
void comm_send_app_cmd_battery_low(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen));

void comm_send_app_cmd_hb_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                   uint8_t hb_aval, uint8_t wear_sval, uint8_t wear_qval, uint8_t sc_bval,
                                   uint8_t lvl_lval, uint8_t func_rval, uint16_t rri_rval);

void comm_send_app_cmd_hrv_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                    uint16_t rri1_rval, uint16_t rri2_rval, uint16_t rri3_rval, uint16_t rri4_rval,
                                    uint8_t lvl_lval, uint8_t cnt_cval);

void comm_send_app_cmd_spo2_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                     uint8_t spo2_rval,  uint8_t spo2_reliable_rval, uint8_t hb_aval, uint8_t hb_reliable_rval,
                                     uint16_t rri1_rval, uint16_t rri2_rval, uint16_t rri3_rval, uint16_t rri4_rval,
                                     uint8_t lvl_lval, uint8_t cnt_cval, uint16_t spo2R_rval);

uint16_t comm_packaging_rawdata_packet(uint8_t *packet_buff, uint16_t comm_max_len,
                                       uint8_t algo_type, uint16_t total_len, uint8_t packet_index, uint8_t algo_result_len,
                                       uint8_t data_array[], uint16_t data_len);
#endif // __APP_CMD_SEND_H__

