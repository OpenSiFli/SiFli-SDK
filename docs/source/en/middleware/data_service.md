# Data Service

Applications need to use data from various sensors or data transmitted via BLE,
and this data may come from different hardware interfaces or cores. For example,
sensors connected to the LCPU may provide data, and the graphical interface
running on the HCPU may need to display the data collected by the sensor. To
simplify dual-core designs and to make the application unaware of the data
sources, regardless of which core generates the data or from which device, data
consumers can obtain and process the data in the same way. The SDK provides a
data service framework that allows applications to use the data without knowing
the details of data generation.

The main features include:
- Applications can subscribe to data services, where the application is called
  the subscriber or client, acting as the consumer of the data.
- Sensor or BLE data services act as the provider, implementing a specific data
  service and obtaining data from I2C, SPI, or different remote BLE devices.
  They are the producers of the data.
- The framework supports data service providers and subscribers running on
  different CPU cores. A data service can be implemented on one core and moved
  to another core without affecting the subscriber application.
- Provides data service simulation features to allow independent development of
  applications and data services.
- Thread-safe API interfaces, allowing the service to be used in multi-threaded
  environments.

The SIFLI SDK includes test tools for testing data service providers or
subscribers. Users can simulate sensor services, such as heart rate sensors,
compass, motion sensors, etc. These functions are integrated with the SDK
example – the watch emulator, which helps users develop watch applications that
require sensor support. For a detailed introduction to the data service API,
please refer to [data_service](middleware-data_service).

Data is transferred between cores/threads via messages, which are divided into
request (XXX_REQ) and response (XXX_RSP) types. A request message is sent from
the client to the service, while a response message is sent from the service to
the client. The message ID is 16 bits long, with the lower 15 bits representing
the message number, and the highest bit (bit 15) indicating the message type: 0
for request messages and 1 for response messages. Request and response messages
typically appear in pairs, with the lower 15 bits of their IDs being the same,
and only the highest bit differing. For example, the following defines two
messages, `SUBSCRIBE_REQ` and `SUBSCRIBE_RSP`:
```c
MSG_SERVICE_SUBSCRIBE_REQ = 0x0,
MSG_SERVICE_SUBSCRIBE_RSP = RSP_MSG_TYPE | MSG_SERVICE_SUBSCRIBE_REQ,
```
Sometimes, a service may actively send messages to the client without a
corresponding request message. In such cases, independent response messages are
defined, generally ending with `IND`, but they are also marked as response
messages. For example:
```c
MSG_SERVICE_DATA_NTF_IND  = RSP_MSG_TYPE | 0x09,
```

The framework reserves 48 message IDs (0x0 ~ 0x2F) for internal use. Users can
define their own messages starting from `MSG_SERVICE_CUSTOM_ID_BEGIN` to avoid
conflicts with the reserved messages. For example:
```c
typedef enum
{
    BLE_DFU_SEND_DATA_REQ = MSG_SERVICE_CUSTOM_ID_BEGIN,
    /* Response. */
    BLE_DFU_SEND_DATA_RSP = BLE_DFU_SEND_DATA_REQ | RSP_MSG_TYPE,
} ble_dfu_service_message_t;
```



## Data Service Configuration
Run `menuconfig`, and under `SiFli Middleware`, select `Enable Data Service`. If
cross-core services are required, you must also select the `Enable IPC Queue
Library` in the Middleware. Once configured, the following two macros will
appear in the project's `rtconfig.h`:
```c
#define BSP_USING_DATA_SVC 1
#define USING_IPC_QUEUE 1
```

## System Resources Used
If IPC Queue is enabled, the data service framework uses IPC Queue 1 as the
communication channel between the two cores. The buffer address for sending from
HCPU to LCPU is `HCPU2LCPU_MB_CH2_BUF_START_ADDR`, with a size of
`HCPU2LCPU_MB_CH2_BUF_SIZE`. The buffer address for sending from LCPU to HCPU is
`LCPU2HCPU_MB_CH2_BUF_START_ADDR`, with a size of `LCPU2HCPU_MB_CH2_BUF_SIZE`.
The default size for these buffers is 512 bytes. If you need to change the
buffer size, refer to (_/app_note/memory_usage.md_).

## Registering Data Services
The data service component is an open framework, and the implemented data
services must be registered with the framework before they can be used by
subscribers. The following example uses the function `#datas_register` to
register a service named `test`. Note that service registration must be done
during the `INIT_COMPONENT` stage to ensure all services are ready when the data
service framework is started during the `INIT_ENV` phase.

```c
#include "data_service_provider.h"

bool test_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)		
{
    return TRUE;								// Do not filter any data, push all data to subscriber.
}

static void timer_callback()
{
    datas_ind_size(service, 16);				// Indicate to service that 16 bytes of data is generated
}

static int32_t test_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
        case MSG_SERVICE_SUBSCRIBE_REQ:
            // Start to generate data, for example, start a periodical timer to read from sensor. Timeout will call timer_callback.
            break;
        case MSG_SERVICE_UNSUBSCRIBE_REQ:
            // Stop generating data, for example, stop the periodical timer created earlier.
            break;
        case MSG_SERVICE_CONFIG_REQ:
        case MSG_SERVICE_TX_REQ:
        case MSG_SERVICE_RX_REQ:
            break;
        case MSG_SERVICE_DATA_RDY_IND:
        {
            data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
            uint8_t *data;

            // Fetch data, the code is not in this sample. In this example, data_ind->len is 16 bytes.
            ...
            datas_push_data_to_client(service, data_ind->len, data);
            break;
        }
        default:
            ASSERT(0);			// Unknown message
    }
    return 0;
}

static data_service_config_t test_service=
{
    .max_client_num = 5,						// Support up to 5 client connections
    .queue = NULL,								// Use framework thread for message handling
    .data_filter = test_service_filter,			// Message filter
    .msg_handler = test_service_msg_handler,	// Message handler
};

static int test_service_register(void)
{
    /* Register test service, service name is "test" */
    datas_handle_t service_handle = datas_register("test", &test_service);
    RT_ASSERT(service_handle);

    return 0;
}
INIT_COMPONENT_EXPORT(test_service_register);
```

## Using Data Services
Applications use the service name to subscribe to services. The following
example shows how to subscribe to a service named `test` and send a request
message. Note that you can only send a message to the service after successfully
subscribing. Otherwise, it will return a failure.

```c
#include "data_service_subscriber.h"

int datasvc_clnt_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_TEST_DATA_REQ == arg->msg_id)
    {
        uint8_t *data = arg->data;
        // Handle arg->data, length is indicated by arg->data_len
        LOG("Command result %d
", rsp->result);
    }
    else if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        LOG("Get data length %d
", arg->data_len);
    }
    else
    {
        LOG("Get message %d
", arg->msg_id);
    }
}

static datac_handle_t hdl;

int main(void)
{
    // Allocate subscriber (i.e., client) handle
    hdl = datac_open();
    // Check handle
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != hdl);

    // Subscribe to a service named "test"
    hdl = datac_subscribe(hdl, "test", datasvc_clnt_callback, NULL);
}

int send_req(void)
{
    data_msg_t msg;
    uint8_t *body;
    rt_err_t err;

    body = data_service_init_msg(&msg, MSG_SERVICE_TEST_DATA_REQ, 0);
    err = datac_send_msg(client_handle, &msg);
    RT_ASSERT(RT_EOK == err);
}

int unsubscribe(void)
{
    // Unsubscribe from data service. hdl could be used to subscribe again
    datac_unsubscribe(hdl);

    // Free subscriber handle, the handle cannot be used anymore
    datac_close(hdl);
    hdl = DATA_CLIENT_INVALID_HANDLE;
}
```

## Call Sequence

Users should follow the following call sequence to use data services. In the
SDK, the data service framework is started during the `INIT_ENV` phase, so it
requires the data service to be registered in the `INIT_COMPONENT` phase.
Subscribers can only subscribe to services in the `INIT_APP` phase or later.

1. `datas_register`: Called during the `INIT_COMPONENT` phase of RT-Thread to
   register the data service with the framework.
2. `datas_start`: Automatically executed during the `INIT_ENV` phase (no user
   code needed), starts the data service framework and accepts requests from
   subscribers. The HCPU will start the LCPU at this point.
3. `datac_open`: Allocates a port for the subscriber.
4. `datac_subscribe`: The subscriber initiates a connection and subscribes to
   the data service.

## Using Data Services in GUI

GUI systems like LVGL run in single-threaded mode and have their own LVGL task.
To update data in the GUI thread, the data service defines specific interfaces
that can be called from the GUI thread/task.

### Working Principle

Implementation mechanism of GUI data service:
1. Register a message queue through `datac_subscribe_ex`
2. All messages from the data service first enter this queue instead of calling
   back directly
3. Start an LVGL timer (lv_timer) to periodically check this queue
4. Retrieve messages from the queue and trigger callbacks in the LVGL task
   context

This design meets the requirement that LVGL can only execute in a single thread,
ensuring all GUI update operations are completed in the LVGL task, avoiding
thread safety issues.

### Usage Example

The following demonstrates the core steps for using data services in a GUI
application:

```c
#include "ui_datasrv_subscriber.h"

static datac_handle_t sensor_handle;

/* Data service callback function - executes in LVGL task */
static int sensor_data_callback(data_callback_arg_t *arg)
{
    switch (arg->msg_id)
    {
        case MSG_SERVICE_SUBSCRIBE_RSP:
        {
            data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
            if (rsp->result == 0)
            {
                /* Subscription successful, can start requesting data */
            }
            break;
        }

        case MSG_SERVICE_DATA_NTF_IND:
        {
            /* Received data notification, safely update LVGL widgets */
            sensor_data_t *data = (sensor_data_t *)arg->data;

            // Update UI directly, no additional synchronization needed
            lv_label_set_text_fmt(label, "Value: %d", data->value);
            break;
        }
    }
    return 0;
}

/* When application starts */
void app_start(void)
{
    /* 1. Initialize GUI data service module (only once for the entire app) */
    ui_datac_init();

    /* 2. Allocate client handle */
    sensor_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != sensor_handle);

    /* 3. Subscribe to service - callback will execute in LVGL task */
    ui_datac_subscribe(sensor_handle, "SENSOR", 
                      sensor_data_callback, 0);
}

/* When application stops */
void app_stop(void)
{
    if (DATA_CLIENT_INVALID_HANDLE != sensor_handle)
    {
        datac_unsubscribe(sensor_handle);
        datac_close(sensor_handle);
        sensor_handle = DATA_CLIENT_INVALID_HANDLE;
    }
}
```

**For a complete GUI application example**, please refer to the compass
application in the [Graphical Application Framework](gui_app_framework.md#应用示例)
section, which demonstrates the full application lifecycle and data service
integration.



### Key Interface Description

- **`ui_datac_init()`**: Initialize the GUI data service module, create message
  queue and start LVGL timer (15ms period). Should be called once during GUI
  initialization.

- **`ui_datac_subscribe(handle, name, callback, user_data)`**: Subscribe to data
  service in GUI thread context. Unlike regular `datac_subscribe`, it ensures
  callbacks execute in the LVGL task through a message queue.

- **Callback Execution Environment**: All callback functions registered through
  `ui_datac_subscribe` execute in the LVGL task, allowing safe calls to any LVGL
  API for interface updates.

### Differences from Regular Subscription

| Feature                   | datac_subscribe              | ui_datac_subscribe              |
| ------------------------- | ---------------------------- | ------------------------------- |
| Callback Execution Thread | Data service task            | LVGL task                       |
| Thread Safety             | User handles synchronization | Framework automatically ensures |
| Application Scenario      | Non-GUI applications         | GUI applications                |
| Can Update UI             | No, requires additional sync | Yes, direct updates             |


## API Reference
[Middleware Data Service](middleware-data_service)
