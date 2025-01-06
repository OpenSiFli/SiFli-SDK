
1.BLE update adv data
sibles_advertising_update_adv_and_scan_rsp_data(g_app_advertising_context,adv_data,scan_data)

2.update connection parameter
ble_app_update_conn_param

3.update BT name
static const char *local_name = "sifli_music";
bt_interface_set_local_name(strlen(local_name), (void *)local_name);


4.read BT name
bt_interface_rd_local_name();


5.modify bt addr
sibles_change_bd_addr(SIBLES_CH_BD_TYPE_BT, SIBLES_CH_BD_METHOD_CUSTOMIZE, &addr);
// for example:
// case BT_CONTROL_CHANGE_BD_ADDR:
// {
//     if (RT_NULL == args)
//     {
//         bd_addr_t addr = {0};
//         ble_get_public_address(&addr);
//         sibles_change_bd_addr(SIBLES_CH_BD_TYPE_BT, SIBLES_CH_BD_METHOD_CUSTOMIZE, &addr);
//     }
//     else
//     {
//         rt_memcpy(&g_custom_bt_addr, args, sizeof(bd_addr_t));
//         sibles_change_bd_addr(SIBLES_CH_BD_TYPE_BT, SIBLES_CH_BD_METHOD_CUSTOMIZE, args);
//     }
// }


6.read bt addr
void bt_interface_rd_local_bd_addr(void)