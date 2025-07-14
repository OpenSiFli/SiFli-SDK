# ANCS Client

## ANCS Introduction
Apple Notification Center Service (ANCS) is a GATT service in iOS devices. This service can notify accessories of notifications in iOS devices, called Notification Provider (NP).
The local device acts as an accessory, called Notification Consumer (NC).

NP has 3 characteristics:

- 1. Notification Source

    After NC enables CCCD, the notification source characteristic will send notifications with categoryID and count.
    	![Notification Data Format](../../assets/formatOfGATTNotifications_2x.png)
    Event ID notifies NC whether a notification is added, modified, or removed.

    - EventFlags notify the specificity of the notification. NC can decide whether to notify the user or just filter.
    - CategoryID notifies the category of notification, such as incoming call, news, or message.
    - CategoryCount notifies the count of notifications present in the notification center.
    - NotificationUID is a 32-bit ID used to identify the category.


- 2. Control Point and Data Source

  NC can write commands to the control point to interact with NP to get details or perform actions. There are 3 commands:

  - Get Notification Attributes. This command allows NC to retrieve mode details of notifications from Notification.![Get Notification Attributes Command Format](../../assets/formatOfAGetNotificationAttributesCommand_2x.png)

  - - CommandID should be set to 0
       - NotificationUID from the notification source notification.
       - AttributeIDs are the notification attributes that NC wants to get more details about.

       
       ![Response Format for Get Notification Attributes Command](../../assets/formatOfAResponseToAGetNotificationAttributesCommand_2x.png) 

       - CommandID, Notification UID, Attribute_ID are all the same as the Get Notification Attributes command.
       - Attribute length and attribute are the length and data associated with the attribute ID. The data includes details.

       - Get App Attributes. This command allows NC to retrieve details of installed apps.
         ![Get App Attributes Command Format](../../assets/formatOfAGetAppAttributesCommand_2x.png) 

       - CommandID should be set to 1

       - APP Identifier is the string ID obtained from the notification APP_ID attribute. It represents the APP ID registered in iOS.

       - APP AttributeIDs are the APP attributes that NC wants to get more details about.
         ![Response Format for Get App Attributes Command](../../assets/formatOfAResponseToAGetAppAttributesCommand_2x.png) 

       - CommandID, APP Identifier are the same as the Get APP Attributes command.

       - Attribute length and attribute are the length and data associated with the attribute ID. The data includes details.

         

- 3. Perform Notification Action.
     This command allows NC to perform a predetermined action on a specific iOS notification. There are two actions:

     - 1. Positive action

     - 2. Negative action

     ​    iOS will perform behavior based on the action, but UI behavior depends on iOS and the notification. For example, if the notification is an incoming call, the positive action might answer it, while the negative action might reject it. But the behavior may change in other notifications.


## Implementing ANCS NC

Sibles ANCS provides APIs to search for ANCS in iOS devices and configure notification attributes. Users only need to do the following:
	1. Configure expected notification and APP attributes and associated lengths.
	2. After receiving APP_ID attribute information, use the relevant API to get APP attributes.
	3. Use the perform action API to execute actions for specified attributeID.

For ANCS API details, please refer to @ref ANCS.

Here is sample code:

```c

int app_ancs_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{

    switch (event_id)
    {
	// ANCS only allows paired devices to access its service.
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        rt_kprintf("encrypt ind %d", ind->conn_idx);
		// configure attributes and their lengths.
		ble_ancs_attr_enable(BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID, 1, 0);
		ble_ancs_attr_enable(BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE, 1, 50);
		ble_ancs_attr_enable(BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE, 1, 100);
		// enable ANCS
        ble_ancs_enable(ind->conn_idx);
        break;
    }
    case BLE_ANCS_NOTIFICATION_IND:
    {
		// Go through all attributes data.
        ble_ancs_noti_attr_t *notify = (ble_ancs_noti_attr_t *)data;
        rt_kprintf("received_notify %d, attr_count %d\r\n", notify->evt_id, notify->attr_count);
        if (notify->evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED &&
                !(notify->evt_flag & BLE_ANCS_EVENT_FLAG_PRE_EXISTING)) // No need to show pre-existing notifications
        {
            ble_ancs_attr_value_t *value = notify->value;
            ble_ancs_dev_app_id_name_t *app_id_name;
            uint32_t i;
            for (i = 0; i < notify->attr_count; i++)
            {
                if (value->len == 0)
                {
                    value = (ble_ancs_attr_value_t *)((uint8_t *)value + sizeof(ble_ancs_attr_value_t));
                    continue;
                }
                if (value->attr_id == BLE_ANCS_NOTIFICATION_ATTR_ID_TITLE)
                {
					// Handle title content.
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_APP_ID)
                {
					// Get APP attribute
                    ble_ancs_get_app_attr(app_id, value->len + 1);
 
                }
                else if (value->attr_id  == BLE_ANCS_NOTIFICATION_ATTR_ID_MESSAGE)
                {
					// Handle message content.
                }
                value = (ble_ancs_attr_value_t *)((uint8_t *)value + sizeof(ble_ancs_attr_value_t) + value->len);
            }
            break;
        }
        break;
    }
    case BLE_ANCS_GET_APP_ATTR_RSP:
    {
		// Handle APP display name
        break;
    }
    default:
        break;

    }

    return 0;
}

// Register event to listen to ANCS events.
BLE_EVENT_REGISTER(app_ancs_event_handler, NULL);

```