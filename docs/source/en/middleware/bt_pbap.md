
# PBAP (Phone Book Access Profile)

PBAP stands for Phone Book Access Profile, an upper-layer protocol based on OBEX. This protocol allows synchronization of contacts, call logs, and other information from devices with phone book functionality, such as mobile phones.

PBAP defines two roles:

- **PSE (Phone Book Server Equipment)**: The device that holds the phone book source data, acting as the server, such as a mobile phone.
- **PCE (Phone Book Client Equipment)**: The device that requests phone book information from the PSE, acting as the client, such as a car infotainment system.

The PSE can be a mobile phone or a device with a SIM card. The stored contact information may reside on the phone or the SIM card. The PCE must specify the path from which to retrieve the data during synchronization, i.e., whether to sync from the phone or the SIM card.

The following objects are supported in PBAP:
- Phone Book object (pb): `pb.vcf`
- Incoming Calls History object (ich): `ich.vcf`
- Outgoing Calls History object (och): `och.vcf`
- Missed Calls History object (mch): `mch.vcf`
- Combined Calls History object (cch): `cch.vcf`
- Speed-Dial object (spd): `spd.vcf`
- Favorite Contacts object (fav): `fav.vcf`

**Contact paths:**
- Phone path: `telecom/xxx.vcf`
- SIM card path: `SIM1/telecom/xxx.vcf`

This document mainly describes the basic functionality support for PBAP in the PCE based on the Sifli SDK. The relevant files are:
- `bts2_app_interface`
- `bts2_app_pbap_c`

## PBAP Initialization

- The PBAP initialization function: `bt_pbap_clt_init`, which initializes PBAP-related states and flags.

```c
void bt_pbap_clt_init(bts2_app_stru *bts2_app_data)
{
    local_inst = (bts2s_pbap_clt_inst_data *)bmalloc(sizeof(bts2s_pbap_clt_inst_data));
    // Must allocate successful
    BT_ASSERT(local_inst);
    local_inst->pbap_clt_st = BT_PBAPC_IDLE_ST;
    local_inst->is_valid_vcard = FALSE;
    local_inst->elem_index = BT_PBAP_ELEM_VCARD_IDLE;
    local_inst->pbab_vcard = NULL;
    local_inst->mfs = pbap_clt_get_max_mtu();
    local_inst->rmt_supp_repos = 0;
    local_inst->curr_cmd = BT_PBAP_CLT_IDLE;

    local_inst->curr_repos = PBAP_LOCAL;
    local_inst->curr_phonebook = PBAP_PB;

    local_inst->target_repos = PBAP_UNKNOWN_REPO;
    local_inst->target_phonebook = PBAP_UNKNOWN_PHONEBOOK;

    local_inst->cur_file_hdl = NULL;
}
```

### PBAP Function for Getting Contact Names

Since there are many phone books on the mobile phone and syncing all the data to a watch would result in too much data, this feature retrieves contact names by querying the phone using a number during a call. Note: This feature requires permission from the phone during pairing.

- **PBAP connection device interfaces:**
    - `bts2_app_interface` connection interface: `bt_interface_conn_ext`
    - `bts2_app_pbap_c` connection interface: `bt_pbap_clt_conn_to_srv`

- **PBAP disconnect device interfaces:**
    - `bts2_app_interface` disconnect interface: `bt_interface_disc_ext`
    - `bts2_app_pbap_c` disconnect interface: `bt_pbap_clt_disc_to_srv`

- **PBAP select and set contact repository (by default, it is set to phone storage):**
    - `bts2_app_pbap_c` interface for setting contact repository: `bt_pbap_client_set_pb`

- **PBAP retrieve contact name by number (this is called when there is a call):**
    - `bts2_app_pbap_c` interface to get contact name by number: `bt_pbap_client_get_name_by_number`
    - Contact name event: `BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND`
    - Event indicating the end of contact name retrieval: `BT_NOTIFY_PBAP_VCARD_LIST_CMPL`

```c
// Register notify event handle function start
// step1: Successfully establish PBAP connection via interface
// step2: Set contact path via bt_pbap_client_set_pb (Phone path: telecom/xxx.vcf / SIM card path: SIM1/telecom/xxx.vcf)
// step3: Call bt_pbap_client_get_name_by_number to retrieve the contact name by passing the phone number
int bt_sifli_notify_pbap_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // PBAP connection successful
    case BT_NOTIFY_PBAP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // PBAP disconnected successfully
    case BT_NOTIFY_PBAP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // Contact handle value (1.vcf) + contact name information
    case BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND:
    {
        pbap_vcard_listing_item_t *list_item = (pbap_vcard_listing_item_t *)data;
        break;
    }
    // Contact name retrieval complete
    case BT_NOTIFY_PBAP_VCARD_LIST_CMPL:
    {
        break;
    }
    default:
        return -1;
    }
    return 0;
}
static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // Bluetooth function successfully started and can be used normally
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }
    // Bluetooth turned off successfully
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        break;
    }
    // ACL connection successful
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;
        // Device ACL connected
        break;
    }
    // ACL disconnected successfully
    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
    {
        bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
        // Device ACL disconnected
        break;
    }
    default:
        return -1;
    }
    return 0;
}

static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_COMMON:
    {
        ret = bt_sifli_notify_common_event_hdl(event_id, data, data_len);
    }
    break;

    case BT_NOTIFY_PBAP:
    {
        bt_sifli_notify_pbap_event_hdl(event_id, data, data_len);
    }
    break;

    default:
        break;
    }

    return 0;
}

int app_bt_notify_init(void)
{
    bt_interface_register_bt_event_notify_callback(bt_notify_handle);
    return 0;
}

INIT_ENV_EXPORT(app_bt_notify_init);
// Register notify event handle function end
```

This document provides an overview of PBAP functionality, implementation details, and the necessary SDK files.
