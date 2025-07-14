# GAP

通用访问配置文件 (GAP) 定义了设备发现、链接管理和相关安全程序。 在 BLE 中，为不同的场景定义了几种模式和程序 ：
- 广播模式和观察程序。 该模式和流程适用于无连接通信场景，例如信标 。
	- 处于广播模式的设备称为广播器。 它将发送带有指定数据的不可连接的广播。
	- 设备实现的观察程序称为观察者。 它将扫描不可连接的广播以获取数据。
- 发现模式和程序，连接模式和程序。 两组模式和程序是针对不同场景下的链路建立，如定向连接、正常连接。
	- 接受链路建立的设备称为外设。 它将发送可连接或定向可连接广播。
	- 发起链路建立的设备称为中央设备。 它将扫描广播并通过广播连接到指定的对等地址。
- 配对模式和程序。 模式和程序是将链路设置为不同的安全级别。

GAP 提供外围 API 来管理广播、连接和安全。

GAP API 详情请参考 @ref GAP .

## 低功耗蓝牙地址类型

蓝牙 LE 有 4 种地址类型：
	1. 公共设备地址，由SIG分配，是唯一地址。
	1. 静态随机设备地址是随机生成的，在电源循环期间不应更改地址。 最高 2 位应为 0x11。
	1. 私有设备地址，用于保护设备地址，并且可以在几分钟内更改。 使用私有设备地址的设备必须具有身份地址，该地址可以是公共地址或静态随机地址。
		1. 可解析的私有设备地址，由本地身份解析密钥（IRK）生成。 它可以由具有 IRK 的对等设备解决。 IRK 将在粘合过程中交换。 最高 2 位应为 0x01
		1. 不可解析的私有设备地址，是随机生成的。 无法解决。 最高 2 位应为 0x00。

用户可以在不同的场景中使用这 4 种地址类型。 
- 一般情况下，应使用公共地址或静态随机地址。 
- 在安全敏感的可连接情况下，应使用可解析的私有设备地址。 
- 在安全敏感的不可连接的情况下，不可解析的私有地址应该是 usd。
		

## 广播

蓝牙 LE 有 40 个物理信道（索引从 0~39）从 2.4GHz - 2.4835Hz，其中 37、38 和 39 信道分配给广播。 在 37、38 或 39 频道上依次发送的广播称为广播事件。
相邻广播事件的两个开始之间的距离称为广播间隔。 通道可以通过广播参数channel_map配置为37、38、39中的任意通道。

![](../../../assets/advertisement_channel.png)

如果间隔越小，则广播越密集，中心设备可以更容易地扫描该广播或更快地连接，而功耗更高。
相比之下，间隔越大，功耗越低，但扫描或连接会更困难。

特殊情况是高负载循环定向连接广播。 定向广播是发送指定的对等地址设备，使其携带对等地址而不是广播数据或扫描 rsp 数据。
高占空比期望对端设备以非常高的频率快速发起链路建立和广播。 在这种情况下，不使用广播间隔。 由于频率高，广播时长不能超过1.28s。

![](../../../assets/directed_adv.png)

## 连接参数

在控制器中，发起链接建立的设备称为主设备（master），接受链接建立的设备称为从设备（slave）。 链接建立后，连接事件用于交换数据包。
在连接事件期间，主从交替发送数据接收数据。 主机先发送数据并接收数据，而从机接收数据并响应数据。 在一次正常的连接事件中，主设备和从设备应至少发送一个数据包。

影响连接行为的参数有很多：
- 连接事件的长度称为 CE 长度。 参数 max_ce_len 和 min_ce_len 确定长度。
- 两个连续连接事件之间的距离称为连接间隔。
- 从设备延迟允许从设备在某些连接事件中不响应主设备。 例如，如果从设备延迟为 2，则从设备可以每两个连接事件响应主设备。
- 两次接收数据之间的最大间隔称为监督超时。 如果在这个时间间隔内没有收到数据，说明连接异常。 导致超时的原因有很多，例如一台设备超出范围，一台设备突然关机或堆栈损坏。

![](../../../assets/connection_parameter.png) 连接参数

![](../../../assets/slave_latency.png) 从延迟

## 吞吐量和低功耗

吞吐量可以简单地通过 Dsize（在一个连接事件中传输数据大小）* Dcount（每秒连接事件计数，1/connection_interval）计算。 Dsize 和 Dcount 越大，吞吐量越高。
Dcount 仅由连接间隔决定，间隔越小计数越大。 最小连接间隔为 7.5 毫秒，通常为高吞吐量的间隔为 15 - 48 毫秒。
Dsize 比较复杂，用户可以采用以下方法进行改进：
	- 确保主设备和从设备都支持数据长度扩展（DLE）功能，这是蓝牙核心协议 4.2 之后的一个选项功能。 此功能允许最大数据包大小从 27 字节到 251 字节。 每个数据包都有一定的头部，当Dsize增大时，数据包的数量会减少，头部的开销也会减少。
	![](../../../assets/dle.png)
	
	- 确保主设备和从设备都支持蓝牙核心协议 5.0 支持的物理 2M 功能。 该特性允许物理速率从 1M 到 2M。
	
	- 准备了足够的数据进行传输。 从Connection参数的说明来看，主设备和从设备交替发送和接收数据。 如果没有更多数据要发送，则连接事件结束。 数据需要一些时间准备，
	如果控制器没有数据要依次传输，则数据只能在下一个连接事件中传输。 例如，如果主机发送一个数据包，需要从机响应，则可以发送下一个数据包，两个连接事件只能发送一个数据包。
	但是如果主机发送两个数据包，然后需要从机响应，则两个连接事件可以发送两个数据包。
	![](../../../assets/slave_respond.png)
	
如果不需要传输数据，设备可以改变参数进入低功耗传输模式。 有2种方法：
	1. 使用更大的连接间隔，例如将连接间隔从 30 更改为 500 毫秒。 这种方法可以降低主从传输功率。
	但是当切换回高吞吐量间隔时会有一些延迟。 通常，连接间隔切换至少需要 6 个连接事件。 如果从 500 毫秒切换到 30 毫秒，前 3 秒 (500 * 6) 仍然使用 500 毫秒间隔。
	2. 启用从机延迟，从机可以跳过几个连接事件来响应。 它可以解决延迟问题，但主机无法节省传输功率。 比如设置间隔30毫秒，从设备延迟20。
	然后 主设备 每 30 毫秒传输一次，而 从设备 每 600 毫秒传输一次。 当从设备想要发送数据时，它只能每30毫秒响应一次。

## SMP

安全管理器配置文件 (SMP) 定义了生成和分发安全密钥并加密与这些密钥的链接的协议和行为。 相关 API 与 GAP 集成。 SMP 中有四个功能。
	- 配对：创建共享密钥。
	- 绑定：存储配对创建并用于后续连接的密钥。 绑定后，两个设备成为可信设备对。
	- 身份验证：验证两台设备是否具有相同的密钥。
	- 加密：加密链接以确保安全通信。

如图 SMP 流程所示，主设备可以发起配对请求来触发配对流程，而从设备可以发起安全请求来请求主设备发起配对请求。 

![](../../../assets/smp_flow.png) SMP 流程

在 SIFLI BLE SDK 中，对于从设备角色，大部分过程都封装在堆栈中，用户只需要处理 #BLE_GAP_BOND_IND 和 #BLE_GAP_ENCRYPT_IND 。

默认的鉴权方式是just works，如果要修改配对的鉴权方式为passkey entry或者numeric comparison
需要调用如下函数，设置SDK的IO capabilities
```c
uint8_t connection_manager_set_bond_cnf_iocap(uint8_t iocap)
```
同时调用设置配对确认的函数为BOND_PENDING
```c
uint8_t connection_manager_set_bond_ack(uint8_t state)
```
当BOND_PENDING被设置后，用户需要处理#CONNECTION_MANAGER_BOND_AUTH_INFOR消息来回复对应的配对事件
```c
case CONNECTION_MANAGER_BOND_AUTH_INFOR:
{
	connection_manager_bond_ack_infor_t *info = (connection_manager_bond_ack_infor_t *)data;
	if (info->request == GAPC_PAIRING_REQ)
	{
		// 对端发起配对的事件， 可以做弹窗或者其他处理，最后要调用下面的ack_reply
		connection_manager_bond_ack_reply(info->conn_idx, GAPC_PAIRING_REQ, true);
	}
	else if (info->request == GAPC_TK_EXCH)
	{
		uint32_t pin_code = info->confirm_data;
		// TODO: 显示passkey
		// 回复对端
		connection_manager_bond_ack_reply(info->conn_idx, GAPC_TK_EXCH, true);
	}
	else if (info->request == GAPC_NC_EXCH)
	{
		uintr32_t nc_number = confirm_data;
		// TODO: 显示nc number
		// 回复对端
		connection_manager_bond_ack_reply(info->conn_idx, GAPC_NC_EXCH, true);
	}
	break;
}
```

## 消息流

- 广播程序。

![](../../../assets/advertising.png)

- 连接程序。

![](../../../assets/connection.png)

- 连接参数更新程序。

![](../../../assets/conn_update.png)

- 断开程序。

![](../../../assets/disconnection.png)

## 例子

- 广播

要启用广播，用户需要 
	1. 创建具有指定参数的广播。
	1. 设置广播数据并扫描 rsp 数据。
	1. 开始有持续时间的广播。

```c

static uint8_t adv_idx;

/** advertising data format. Flag is refer to @ble_gap_adv_type.
	   
 * ------------------------------------------------------------------
 *  | Length(1B) | Flag(1B) |  Value  |  Value  |   ...   |   Value  | 
 *	------------------------------------------------------------------
*/

uint8_t adv_data[] = {0x07, 0x09, 0x53, 0x69, 0x66, 0x6c, 0x69, 0x00} // full name: sifli
uint8_t scan_rsp[] = {0x07, 0xff, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04} // Manufactrer specfied data, company id : 0x01, additianl data: 0x01, 0x02, 0x03, 0x04

int app_advertising_evt_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
	uint8_t ret;
    switch (event_id)
    {
    case BLE_GAP_ADV_CREATED_IND:
    {
		// If create advertising successfully, user will received BLE_GAP_ADV_CREATED_IND.
        ble_gap_adv_created_ind_t *ind = (ble_gap_adv_created_ind_t *)data;

		// Store adv index for following APIs usage.
        adv_idx = ind->actv_idx;
        ble_gap_adv_start_t adv_start;

		// Set advertising data.
        ret = ble_gap_set_adv_data(adv_data);
        LOG("Set adv data result %d", ret);
		
		// Set scan rsp data.
		ble_gap_set_scan_rsp_data(scan_rsp);
        LOG("Set scan rsp result %d", ret);
        
		adv_start.actv_idx = ind->actv_idx;

        adv_start.duration = 0;
        adv_start.max_adv_evt = 0;
		
        ret = ble_gap_start_advertising(&adv_start);
        LOG("start adv result %d", ret);
        break;
    }
	case BLE_GAP_SET_ADV_DATA_CNF:
	{
		ble_gap_set_adv_data_cnf_t *cnf = (ble_gap_set_adv_data_cnf_t *)data;
		LOG("Set adv cnf result %d", cnf->status);
		break;
	}
	case BLE_GAP_SET_SCAN_RSP_DATA_CNF:
	{
		ble_gap_set_scan_rsp_data_cnf_t *cnf = (ble_gap_set_scan_rsp_data_cnf_t *)data;
		LOG("Set scan rsp cnf result %d", cnf->status);
		break;
	}
    case BLE_GAP_START_ADV_CNF:
    {
        ble_gap_start_adv_cnf_t *cnf = (ble_gap_start_adv_cnf_t*)data;
		LOG("Start adv cnf result %d", cnf->result);
        break;

    }
    case BLE_GAP_ADV_STOPPED_IND:
    {
		// duration timeout will notify with reason #GAP_ERR_TIMEOUT
        ble_gap_adv_stopped_ind_t *ind = (ble_gap_adv_stopped_ind_t *)data;
		LOG("ADV stopped reason %d", ind->reason);
        break;
    }
    case BLE_GAP_STOP_ADV_CNF:
    {
        ble_gap_stop_adv_cnf_t *cnf = (ble_gap_stop_adv_cnf_t *)data;
		LOG("ADV stop adv cnf result %d", cnf->status);
        break;
    }
    default:
        break;
    }
    return 0;

}

// Create advertising
void app_start_advertising(void *para)
{
    // Prepare advertising parameter
    ble_gap_adv_parameter_t adv_para;
	uint8_t ret;

	// Set parameter via user's scenario.
	adv_para.own_addr_type = GAPM_STATIC_ADDR;
    adv_para.type = GAPM_ADV_TYPE_LEGACY;
    adv_para.disc_mode = GAPM_ADV_MODE_NON_DISC;
    adv_para.max_tx_pwr = 0;
    adv_para.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
	adv_para.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
    adv_para.prim_cfg.adv_intv_max = 0x40;
    adv_para.prim_cfg.adv_intv_min = 0x20;
	adv_para.prim_cfg.chnl_map = 0x07;
	adv_para.prim_cfg.phy = GAP_PHY_TYPE_LE_1M;
	
	// Create advertising.
	ret = ble_gap_create_advertising(&para);
	LOG("Create ADV result %d", ret);
}

// Stop advertising
void app_stop_advertising(void *para)
{
	ble_gap_adv_stop_t stop_req;
	uint8_t ret;
	
	stop_req.actv_idx = adv_idx;
	ble_gap_stop_advertising(&stop_req)
	LOG("Stop ADV result %d", ret);
}

BLE_EVENT_REGISTER(sibles_advertising_evt_handler, NULL);

```


- 连接

充当外围角色，用户只需启用广播并等待中心设备连接。

除了数据传输之外，以下子程序在 GAP 中也很重要：
	1. 连接参数更新程序。 外围设备可能需要中央设备修改一些参数，例如连接间隔、监督超时，以平衡高吞吐量和低功耗。
	2. 断开程序。 断开与中央设备的连接。

```c

static uint8_t conn_idx;

int app_connection_evt_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
		// Store the conn idx for following usage.
        conn_idx = ind->conn_idx;
		LOG("conn interval:%d, conn timeout:%d", ind->con_interval, ind->sup_to);
        break;
    }
	case BLE_GAP_UPDATE_CONN_PARAM_IND:
	{
		// Get the adjust parameters.
		ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
		if (ind->conn_idx == conn_idx) // Make sure the conn_idx is the same device.
			LOG("new conn interval :%d, conn timeout :%d", ind->con_interval, ind->sup_to);
		break;
	}
	case BLE_GAP_UPDATE_CONN_PARAM_CNF:
	{
		// Check whether update successfully.
		BLE_GAP_UPDATE_CONN_PARAM_CNF *cnf = (ble_gap_update_conn_param_cnf_t *)data;
		if (ind->conn_idx == conn_idx) // Make sure the conn_idx is the same device.
			LOG("conn update reason %d", cnf->reason);
		break;
	}
	case BLE_GAP_DISCONNECTED_IND:
	{
		// Received this event indicates connection is disconnected and can check the disconnection reason.
		ble_gap_disconnected_ind_t *ind = (ble_gap_disconnected_ind_t *)data;
		if (ind->conn_idx == conn_idx) // Make sure the conn_idx is the same device.
			LOG("disonncet reason %d", ind->reason);
		break;
	}
    default:
        break;
    }
}

void app_update_connection_parameter(void *para)
{
	ble_gap_update_conn_param_t conn_para;
	uint8_t ret;
	
	// Require new paramters.
	conn_para.conn_idx = conn_idx;
	conn_para.intv_min = 0x20;
	conn_para.intv_max = 0x30;
	conn_para.latency = 0x0;
	conn_para.time_out = 500;
	conn_para.ce_len_min = 20;
	conn_para.ce_max_len = 100;
	
	ret = ble_gap_update_conn_param(&conn_para);
	LOG("Update conn parameter ret %d", ret);
}

void app_disconnect(void *para)
{
    ble_gap_disconnect_t disc_cmd;
	
	disc_cmd.conn_idx = conn_idx;
	disc_cmd.reason = 0x13;
	
	ret = ble_gap_disconnect(&disc_cmd);
	LOG("Disconnect ret %d", ret);
	
}

BLE_EVENT_REGISTER(app_connection_evt_handler, NULL);

```
