# Serial Transfer Service

The serial transfer service allows users to simply and quickly transfer serial data via BLE.

The service UUID is 7369666c-695f-7364-0000-0000000000000. It has two characteristics:
	- Serial Transfer Configuration: UUID 7369666c-695f-7364-0001-0000000000000 (notify required).
	- Serial Transfer Data: UUID 7369666c-695f-7364-0002-0000000000000 (readable, writable, and notifiable)

The configuration characteristic has not been used yet.

The data characteristic is used to transfer serial data. Its data format is:
![](../../assets/ble_serail_data.png)
	- CateID: categoryID for different users.
	- Flag: for fragmentation.
		- 0x00: Complete packet.
		- 0x01: First packet.
		- 0x02: Continue packet.
		- 0x03: Last packet.
	- Length: Packet length, only available when flag equals 0x00.
	- Data: serial data.
	
## Implementing Serial Data Transfer

Users only need to negotiate and confirm the categoryID with the client and ensure that this ID is different from other users in the device. Then data can be transferred.

For serial transfer API details, please refer to @ref Serial_tran.

Here is sample code:

```c

// Make sure the categoryID is different from other IDs in local.
#define APP_BLE_SERIAL_ID 0x1F

// Handle data from client.
void ble_app_serial_callback(uint8_t conn_idx, ble_serial_tran_data_t *data)
{
	int ret;
	ble_serial_tran_data_t send_data;
	uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
	// The cateID should be the same as registered ID APP_BLE_SERIAL_ID.
    LOG_I("cateID(%d), data(%d) %s\r\n", data->cateID, data->len, data->data);
	// Handle data from peer device start.
	
	// Handle data from peer device end.
	
	// Prepare send data to client
	send_data.cateID = APP_BLE_SERIAL_ID;
	send_data.len = sizeof(test_data);
	send_data.data = test_data;
	ret = ble_serial_tran_send_data(conn_idx, &send_data);
	LOG_I("result %d",ret);
}

// Register user defined categoryID and associated callback.
BLE_SERIAL_TRAN_EXPORT(APP_BLE_SERIAL_ID, ble_app_serial_callback);


```

*/