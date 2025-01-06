#ifndef __BT_DEVICE_H__
#define __BT_DEVICE_H__
#include <rtthread.h>
#include <stdint.h>
#ifdef RT_USING_AT
    //#ifndef ENABLE_SOLUTON_BT_INTERFACE
    #include <at.h>
    //#endif
#endif

#include "state_machine.h"

#ifndef CFG_MAX_BT_BOND_NUM
    #define BT_MAX_ACL_NUM  (1)
#else
    #define BT_MAX_ACL_NUM CFG_MAX_BT_BOND_NUM
#endif
#define BT_INVALID_CONN_INDEX (0xFF)

#define BT_MAX_EVENT_NOTIFY_CB_NUM (8)
#define BT_MAX_MAC_LEN (6)
#define BT_DEVICE_FLAG_OPEN (0x010)
#define BT_MAX_NAME_LEN (60)
#define BT_MAX_CALL_NUM (2)
#define BT_INVALID_CALL_IDX (0xFF)
#define BT_MAX_PHONE_NUMBER_LEN (20)

#define CALL_CLCC_START           (0xAA)
#define CALL_CLCC_IN_PROGRESS     (0x55)
#define CALL_CLCC_COMPLETE        (0xFF)

enum
{
    BT_COMMON_TYPE_ID      = 0x40,
    BT_HF_TYPE_ID          = 0x41,
    BT_AG_TYPE_ID          = 0x42,
    BT_A2DP_TYPE_ID        = 0x43,
    BT_AVRCP_TYPE_ID       = 0x44,
    BT_HID_TYPE_ID         = 0x45,
    BT_SPP_TYPE_ID         = 0x46,
    BT_GATT_TYPE_ID        = 0x47,
    BT_PBAP_TYPE_ID        = 0x48,
};

typedef enum
{
    BT_CONTROL_SEARCH_EQUIPMENT = BT_COMMON_TYPE_ID << 8,   /**< control BT device to search other BT device */
    BT_CONTROL_CANCEL_SEARCH,               /**< control BT device to cancel search */
    BT_CONTROL_CONNECT_DEVICE,              /**< control BT device to connect other BT device */
    BT_CONTROL_CONNECT_DEVICE_EX,           /**< control BT device to connect other BT device extend*/
    BT_CONTROL_DISCONNECT,                  /**< control BT device to disconnect all device*/
    BT_CONTROL_DISCONNECT_EX,               /**< control BT device to disconnect profile extend*/
    BT_CONTROL_DISCONNECT_PROFILE,          /**< control BT device to disconnect by conn idx profile*/
    BT_CONTROL_DISCONNECT_BY_CONNIDX,       /**< control BT device to disconnect by conn idx*/
    BT_CONTROL_QUERY_STATE,                 /**< control BT device to query state blocking*/
    BT_CONTROL_QUERY_STATE_NONBLOCK,        /**< control BT device to query state nonblocking*/
    BT_CONTROL_QUERY_STATE_EX,              /**< control BT device to query state extend*/
    BT_CONTROL_SET_LOCAL_NAME,
    BT_CONTROL_RD_LOCAL_NAME,
    BT_CONTROL_RD_LOCAL_RSSI,
    BT_CONTROL_SWITCH_OFF,                  /**< control BT device switch to undiscoverable and unconected state  */
    BT_CONTROL_SWITCH_ON,                   /**< control BT device switch to discoverable and conected state  */
    BT_CONTROL_SET_SCAN,                    /**< set scan. */
    BT_CONTROL_GET_SCAN,                    /**< get scan. */
    BT_CONTROL_EXIT_SNIFF,
    BT_CONTROL_SWITCH_TO_SOURCE,            /**< control BT device switch to source mode  */
    BT_CONTROL_SWITCH_TO_SINK,              /**< control BT device switch to sink mode  */
    BT_CONTROL_GET_BT_MAC,                  /**< control BT device to get mac */
    BT_CONTROL_GET_VOLUME,                  /**< control BT device to get current volume config */
    BT_CONTROL_SET_VOLUME,                  /**< control BT device to set volume config */
    BT_CONTROL_GET_PEER_DEVICEINFO,         /**< control to get connected device's info,use in BT_STATE_CONNECTED*/
    /*local media*/
    BT_CONTROL_LOCAL_PLAY_NEXT,             /**< control play local next song */
    BT_CONTROL_LOCAL_PLAY,                  /**< control play local current song */
    BT_CONTROL_LOCAL_PLAY_SUSPEND,          /**< control suspend play local song */
    BT_CONTROL_LOCAL_PLAY_PREVIOUS,         /**< control play local previous song */
    BT_CONTROL_GET_LOCAL_SONG_DETAILS,      /**< control get local song detail infomation */
    BT_CONTROL_GET_LOCAL_SONG_BRIEF,        /**< control get local song brief infomation */
    BT_CONTROL_GET_LOCAL_SONG_TOTAL_NUM,    /**< control get local song's total num */
    BT_CONTROL_SET_LOCAL_SPK_VOL,           /**< control set local speaker vol */
#ifdef BT_USING_MIC_MUTE
    BT_CONTROL_SET_MIC_MUTE,                /**< control set mic mute */
    BT_CONTROL_GET_MIC_MUTE,                /**< control get mic mute */
#endif
#ifdef BT_USING_LOCAL_MEDIA_EX
    BT_CONTROL_LOCAL_DEL_MUSIC,             /**< control del local music */
    BT_CONTROL_LOCAL_SET_PLAY_MODE,         /**< control set local play mode */
    BT_CONTROL_LOCAL_PLAY_ASSIGNED_MUSIC,   /**< control play local assigned music */
    BT_CONTROL_LOCAL_GET_MUSIC_LIST,        /**< control get local music list info */
    BT_CONTROL_LOCAL_GET_MUSIC_ID,          /**< control get current music id */
#endif
    BT_CONTROL_REGISTER_NOTIFY,             /**< register bt event notify func */
    BT_CONTROL_UNREGISTER_NOTIFY,           /**< unresgiter bt event notify func */
    BT_CONTROL_CLOSE_DEVICE,                /**< close bt device */
    BT_CONTROL_OPEN_DEVICE,                 /**< open bt device */
#ifdef BT_USING_USB
    BT_CONTROL_ENTER_USB_MODE,              /**< control bt device enter usb mode */
    BT_CONTROL_EXIT_USB_MODE,               /**< control bt device exit  usb mode */
#endif
#ifdef BT_USING_DEVICE_TYPE
    BT_CONTROL_SET_DEVICE_TYPE,             /**< control bt source and sink mode> */
#endif
    BT_BQB_TEST_MODE,
    BT_CONTROL_GET_RMT_VERSION,             /**< control get remote device version */
#ifdef BT_USING_PAIRING_CONFIRMATION
    BT_CONTROL_IO_CAPABILITY_RES,
    BT_CONTROL_USER_CONFIRM_RES,
#endif
    BT_CONTROL_DEVICE_INIT,
    BT_CONTROL_DEVICE_DEINIT,
    BT_CONTROL_AUDIO_TRANSFER_EX,
    BT_CONTROL_CHANGE_BD_ADDR,              /**< control change bt bd addr */
    BT_CONTROL_CANCEL_PAGE,                 /**< control cancel page */
    BT_CONTROL_CANCEL_PAGE_BY_ADDR,         /**< control cancel page by addr */
    BT_CONTROL_LOCAL_PLAY_PROMPT_TONE,      /**< control play local prompt tone */
    BT_CONTROL_PLAY_POWER_ON_RING,          /**< control play power on ring */
    BT_CONTROL_PLAY_POWER_OFF_RING,         /**< control play power off ring */
    BT_CONTROL_SLEEP,                       /**< control bt sleep */
    BT_CONTROL_GET_RMT_NAME,                /**< get remote device name */
} bt_common_cmd_t;

typedef enum
{
    BT_EVENT_CALL_lINK_ESTABLISHED = BT_COMMON_TYPE_ID << 8 | 0x0080,     /**< it means speech pathway established succssefully*/
    BT_EVENT_CALL_LINK_DOWN,            /**< it means call disconencted*/
    BT_EVENT_INQ,                       /**< it means bluetooth has been searched*/
    BT_EVENT_INQ_FINISHED,
    BT_EVENT_CONNECT_COMPLETE,          /**< it means BT device connect success*/
    BT_EVENT_PROFILE_DISCONNECT,        /**< it means BT device profile disconnect*/
    BT_EVENT_DISCONNECT,                /**< it means BT device disconnect*/
    BT_EVENT_KEY_MISSING,               /**< it means remote BT device cancel pair*/
    BT_EVENT_ENCRYPTION,
    BT_EVENT_ACCESS_MODE_CHANGE,        /**< it means access mode hasbeen change*/
    BT_EVENT_VOL_CHANGED,               /**< bt volume changed */
#ifdef BT_USING_LOCAL_MEDIA_EX
    BT_EVENT_LOCAL_MUSIC_LIST_INFO,     /**< music list info report*/
#endif
    BT_EVENT_STATE,                     /**< bt state event */
    BT_EVENT_CONTROL_RET,               /**< bt control result event */
    BT_EVENT_POWER_OFF,                 /**< bt power off event  */
    BT_EVENT_BT_STACK_READY,            /**< it means bt stack is ready,solution can communicate with bt stack. */
    BT_EVENT_BT_ADDR_IND,               /**< indicate bt address. */
    BT_EVENT_CLOSE_COMPLETE,            /**< indicate bt is closed. */
    BT_EVENT_RD_LOCAL_NAME,
    BT_EVENT_RD_LOCAL_RSSI,
    BT_EVENT_ACL_OPENED_IND,
    BT_EVENT_PAIR_IND,                  /**< indicate pair result */
    BT_EVENT_CHANGE_BD_ADDR,
    BT_EVENT_CANCEL_PAGE_IND,
    BT_EVENT_TRANS_AUDIO_IND,
    BT_EVENT_RMT_VERSION_IND,
#ifdef BT_USING_PAIRING_CONFIRMATION
    BT_EVENT_IO_CAPABILITY_IND,
    BT_EVENT_USER_CONFIRM_IND,
#endif
    BT_EVENT_KEY_OVERLAID,
    BT_EVENT_RMT_NAME,
} bt_common_event_t;

#ifdef BT_USING_HF
typedef enum
{
    BT_CONTROL_PHONE_CONNECT = BT_HF_TYPE_ID << 8,               /**< control BT device to answer the phone */
    BT_CONTROL_PHONE_HANDUP,                /**< control BT device to handup the phone */
    BT_CONTROL_DIAL_BACK,                   /**< control BT device back to dial the phone */
    BT_CONTROL_MAKE_CALL,                   /**< control BT device make a phone call */
    BT_CONTROL_SET_INBAND_RING,             /**< control BT device enable/disable inband ring for hf role,default is support inband ring */
    BT_CONTROL_GET_PHONE_NUMBER,            /**< control BT device to get the mobile phone num connected to watch */
    BT_CONTROL_GET_REMOTE_PHONE_NUMER,      /**< control BT device to get the remote mobile phone num  */
    BT_CONTROL_SET_WBS_STATUS,              /**< control sco codec type */
    BT_CONTROL_GET_REMOTE_CALL_STATUS,      /**< control get remote call status */
    BT_CONTROL_3WAY_HOLD,                   /**< control 3 WAY CALL,send AT+HOLD */
    BT_CONTROL_3WAY_BTRH,                   /**< control 3 WAY CALL,send AT+BTRH */
    BT_CONTROL_3WAY_CCWA,                   /**< control 3 WAY CALL,send AT+CCWA,enable call waitint notification */
    BT_CONTROL_UPDATE_BATT_BY_HFP,          /**< control update batt by at cmd */
    BT_CONTROL_AUDIO_TRANSFER,
#ifdef BT_USING_DTMF
    BT_CONTROL_DTMF_DIAL,                   /**< control dtmf dial */
#endif
#ifdef BT_USING_SIRI
    BT_CONTROL_SIRI_ON,                     /**< control bt open siri */
    BT_CONTROL_SIRI_OFF,                    /**< control bt close siri */
    BT_CONTROL_GET_SIRI_CAPABILITY,         /**< control get siri capability */
#endif
} bt_hf_cmd_t;

typedef enum
{
    BT_EVENT_LOCAL_CALL_NUMBER = BT_HF_TYPE_ID << 8 | 0x0080,         /**< it means local call number notifycation event*/
    BT_EVENT_CALL_STATUS_IND,           /**< it means call status ,when dont support 3 way calls*/
    BT_EVENT_DIAL_COMPLETE,
    BT_EVENT_CIND_IND,
    BT_EVENT_CLCC_IND,
    BT_EVENT_CLCC_COMPLETE,
#ifdef BT_USING_SIRI
    BT_EVENT_SIRI_STATE_NOTIFY,         /**< bt siri event  */
    BT_EVENT_SIRI_ON_COMPLETE,          /**< bt siri event:indicate hf active siri complete */
    BT_EVENT_SIRI_OFF_COMPLETE,         /**< bt siri event:indicate hf deactive siri complete */
    BT_EVENT_SIRI_CAPABILITY_NOTIFY,    /**< bt siri event:indicate if ag support siri or not */
#endif
    BT_EVENT_VGS_IND,
    BT_EVENT_DTMF_IND,
    BT_EVENT_AT_CMD_CFM_STATUS,
} bt_hf_event_t;
#endif

#ifdef BT_USING_AG
typedef enum
{
    BT_CONTROL_AG_PHONE_CALL_STATUS_CHANGED = BT_AG_TYPE_ID << 8,
    BT_CONTROL_AG_LOCAL_PHONE_INFO_RES,
    BT_CONTROL_AG_REMOTE_CALL_INFO_RES,
    BT_CONTROL_AG_ALL_INDICATOR_INFO_RES,
    BT_CONTROL_AG_INDICATOR_CHANGED_RES,
    BT_CONTROL_AG_SPK_VOL_CHANGE_REQ,
    BT_CONTROL_AG_MIC_VOL_CHANGE_REQ,
    BT_CONTROL_AG_CMD_RESULT_RES,
} bt_ag_cmd_t;

typedef enum
{
    BT_EVENT_AG_ANSWER_CALL_REQ = BT_AG_TYPE_ID << 8 | 0x0080,
    BT_EVENT_AG_HUNGUP_CALL_REQ,
    BT_EVENT_MAKE_CALL_REQ,
    BT_EVENT_DTMF_KEY_REQ,
    BT_EVENT_GET_LOCAL_PHONE_INFO_REQ,
    BT_EVENT_GET_INDICATOR_STATUS_REQ,
    BT_EVENT_GET_ALL_REMOTE_CALL_INFO_REQ,
} bt_ag_event_t;
#endif

#ifdef BT_USING_A2DP
typedef enum
{
    BT_CONTROL_OPEN_AVSINK = BT_A2DP_TYPE_ID << 8,
    BT_CONTROL_CLOSE_AVSINK,
    BT_CONTROL_UNREGISTER_AVSINK_SDP,
    BT_CONTROL_REGISTER_AVSINK_SDP,
    BT_CONTROL_SET_A2DP_BQB_TEST,
    BT_CONTROL_SET_A2DP_SRC_AUDIO_DEVICE,
    BT_CONTROL_RELEASE_A2DP,
} bt_a2dp_cmd_t;

typedef enum
{
    BT_EVENT_A2DP_START_IND = BT_A2DP_TYPE_ID << 8 | 0x0080,
    BT_EVENT_AVSNK_OPEN_COMPLETE,
    BT_EVENT_AVSNK_CLOSE_COMPLETE,
} bt_a2dp_event_t;
#endif

#ifdef BT_USING_AVRCP
typedef enum
{
    BT_CONTROL_OPEN_AVRCP = BT_AVRCP_TYPE_ID << 8,
    BT_CONTROL_CLOSE_AVRCP,
    BT_CONTROL_PHONE_PLAY_NEXT,             /**< control play phone next song */
    BT_CONTROL_PHONE_PLAY,                  /**< control play phone current song */
    BT_CONTROL_PHONE_PLAY_SUSPEND,          /**< control suspend play phone song */
    BT_CONTROL_PHONE_PLAY_STOP,
    BT_CONTROL_AVRCP_VOLUME_UP,
    BT_CONTROL_AVRCP_VOLUME_DOWN,
    BT_CONTROL_PHONE_PLAY_PREVIOUS,         /**< control play phone previous song */
    /*eraphone media*/
    BT_CONTROL_EARPHONE_PLAY_NEXT,          /**< control play earphone next song */
    BT_CONTROL_EARPHONE_PLAY,               /**< control play earphone current song */
    BT_CONTROL_EARPHONE_PLAY_SUSPEND,       /**< control suspend play earphone song */
    BT_CONTROL_EARPHONE_PLAY_PREVIOUS,      /**< control play earphone previous song */
} bt_avrcp_cmd_t;

typedef enum
{
    BT_EVENT_AVRCP_OPEN_COMPLETE = BT_AVRCP_TYPE_ID << 8 | 0x0080,
    BT_EVENT_AVRCP_CLOSE_COMPLETE,
    BT_EVENT_MUSIC_TRACK_CHANGED,           /**< it means music track changed. */
    BT_EVENT_MP3_DETAIL_INFO,               /**< mp3 detail info event report */
    BT_EVENT_SONG_PLAY_PROGRESS,            /**< song playing progress */
    BT_EVENT_MUSIC_PLAY_STATUS_CHANGED,     /**< it means music play status changed. */
    BT_EVENT_AVRCP_VOLUME_CHANGE_RIGISTER,
} bt_avrcp_event_t;
#endif

#ifdef BT_USING_HID
typedef enum
{
    BT_CONTROL_OPEN_HID = BT_HID_TYPE_ID << 8,
    BT_CONTROL_CLOSE_HID,
    BT_CONTROL_SET_HID_DEVICE,
    BT_CONTROL_PHONE_DRAG_UP,               /**< control phone drag up once*/
    BT_CONTROL_PHONE_DRAG_DOWN,             /**< control phone drag down once*/
    BT_CONTROL_PHONE_ONCE_CLICK,            /**< control phone click once*/
    BT_CONTROL_PHONE_DOUBLE_CLICK,          /**< control phone double click*/
    BT_CONTROL_PHONE_TAKE_PICTURE,          /**< control phone take a picture*/
    BT_CONTROL_PHONE_VOLUME_UP,             /**< control phone volume up*/
    BT_CONTROL_PHONE_VOLUME_DOWN,           /**< control phone volume down*/
} bt_hid_cmd_t;

typedef enum
{
    BT_EVENT_HID_OPEN_COMPLETE = BT_HID_TYPE_ID << 8 | 0x0080,
    BT_EVENT_HID_CLOSE_COMPLETE,
} bt_hid_event_t;
#endif

#ifdef BT_USING_SPP
typedef enum
{
    BT_CONTROL_SEND_SPP_DATA = BT_SPP_TYPE_ID << 8,
    BT_CONTROL_SEND_SPP_RSP,
    BT_CONTROL_SEND_SPP_DISC_REQ,
} bt_spp_cmd_t;

typedef enum
{
    BT_EVENT_SPP_CONN_IND = BT_SPP_TYPE_ID << 8 | 0x0080,
    BT_EVENT_SPP_DATA_IND,
    BT_EVENT_SPP_DATA_CFM,
    BT_EVENT_SPP_DISCONN_IND,
} bt_spp_event_t;
#endif

#ifdef BT_USING_GATT
typedef enum
{
    BT_CONTROL_BT_GATT_SDP_REG_REQ = BT_GATT_TYPE_ID << 8,
    BT_CONTROL_BT_GATT_SDP_UNREG_REQ,
    BT_CONTROL_BT_GATT_MTU_CHANGE_REQ,
} bt_gatt_cmd_t;

typedef enum
{
    BT_EVENT_BT_GATT_REG_RES = BT_GATT_TYPE_ID << 8 | 0x0080,
    BT_EVENT_BT_GATT_UNREG_RES,
    BT_EVENT_BT_GATT_MTU_RES,
} bt_gatt_event_t;
#endif

#ifdef BT_USING_PBAP
typedef enum
{
    BT_CONTROL_PBAP_PULL_PB = BT_PBAP_TYPE_ID << 8,
    BT_CONTROL_PBAP_SET_PB,
    BT_CONTROL_PBAP_PULL_VCARD_LIST,
    BT_CONTROL_PBAP_PULL_VCARD_ENTRY,
    BT_CONTROL_PBAP_GET_NAME_BY_NUMBER,
    BT_CONTROL_PBAP_AUTH_RSP,
} bt_pbap_cmd_t;

typedef enum
{
    BT_EVENT_VCARD_LIST_ITEM_NOTIFY = BT_PBAP_TYPE_ID << 8 | 0x0080,
    BT_EVENT_VCARD_LIST_CMP,
} bt_pbap_event_t;
#endif

typedef enum
{
    BT_STATE_POWER_OFF = 0,
    BT_STATE_POWER_ON,
    BT_STATE_PAIR,
    BT_STATE_SEARCH,
    BT_STATE_OTHER,
    BT_STATE_MAX,
} bt_state_t;    /*device state*/

typedef enum
{
    BT_STATE_CONNECT_IDLE,
    BT_STATE_CONNECTING,
    BT_STATE_CONNECTED,
    BT_STATE_DISCONNECTING,
    BT_STATE_CONNECT_ERROR,
} bt_connect_state_t;

typedef enum
{
    BT_STATE_ACL_IDLE,
    BT_STATE_ACL_CONNECTED,
    BT_STATE_ACL_DISCONNECTING,
    BT_STATE_ACL_ERROR,
} bt_acl_state_t;

typedef enum
{
    BT_CALL_IDLE = 0x00,
    BT_CALL_ONHOLD,
    BT_CALL_INCOMING,
    BT_CALL_ACTIVE,
    BT_CALL_OUTGOING_DAILING,
    BT_CALL_OUTGOING_ALERTING,
    BT_CALL_WAITING,       /*incomming status,@note  The three-party call answering interface is required */
    BT_CALL_ERROR = 0XFF,
} bt_call_state_t;


typedef enum
{
    BT_CIND_SERVICE_TYPE = 0x01,    //(0,1)
    BT_CIND_CALL_TYPE,              //(0,1)
    BT_CIND_CALLSETUP_TYPE,         //(0,3)
    BT_CIND_BATT_TYPE,              //(0,5)
    BT_CIND_SIGNAL_TYPE,            //(0,5)
    BT_CIND_ROAM_TYPE,              //(0,1)
    BT_CIND_CALLHELD_TYPE,          //(0,2)
} bt_cind_type_t;

typedef struct
{
    bt_cind_type_t type;
    uint8_t val;
} bt_cind_ind_t;

typedef struct
{
    uint8_t cmd_id;                   /**at cmd id */
    uint8_t res;                     /**result*/
} bt_at_cmd_cfm_t;

typedef enum
{
    BT_3WAY_REL_HOLDCALLS = 0,//release all hold calls or sets User Determined User Busy for a waiting call.
    BT_3WAY_REL_ACTCALLS,//default no index:relase all active calls and accept the other call. with idex:release call with specifiled index
    BT_3WAY_HOL_ACTCALLS_ACP_OTHER,//defaultno index:place all active calls on hold and accept the other.with index:place all calls on hold except the call indicated by idx.
    BT_3WAY_ADD_HOL_TO_CONV,//add a hold call to the conversation
    BT_3WAY_EXPLICIT_CALL_TRANSFER,//TODO????
    BT_3WAY_MAX,
} bt_3way_coded_t;

typedef struct
{
    bool inquiry_scan;          //true enable inquiry scan, false disable inquiry scan.
    bool page_scan;             //true enable page scan, false disable page scan.
} bt_scan_con_t;

typedef enum
{
    BT_DISC_CONNECT_TIMOUT = 0,  /**< disconnect for timout */
    BT_DISC_USER_BREAK,          /**< user active disconnect */
    BT_DISC_LOCAL_BREAK,         /**< itself active disconnect */
    BT_DISC_OTHER,               /**< unkown reason */
    BT_DISC_MAX
} bt_disconnect_reason_t;

typedef enum
{
    BT_ACCESS_MODE_GENERAL = 0,     /**< discoverable and conectable state */
    BT_ACCESS_MODE_CONNECT_ABLE,    /**< conectable state */
    BT_ACCESS_MODE_INACCESSIABLE,   /**< undiscoverable and unconected able state */
    BT_ACCESS_MODE_MAX
} bt_access_mode_t;


typedef enum
{
    BT_DIR_CURRENT = 0,
    BT_DIR_NEXT,
    BT_DIR_PREVIOUS,
    BT_DIR_MAX
} bt_direction_t;

typedef enum
{
    BT_VOLUME_MEDIA = 0,
    BT_VOLUME_CALL,
    BT_VOLUME_PROMPT,
    BT_VOLUME_MODE_MAX
} bt_volume_mode_t;

typedef enum
{
    BT_DEVICE_TYPE_COMPUTER = 0,
    BT_DEVICE_TYPE_PHONE,
    BT_DEVICE_TYPE_EARPHONE,
    BT_DEVICE_TYPE_OTHER,
    BT_DEVICE_TYPE_MAX
} bt_device_type_t;

#ifdef BT_USING_DTMF
typedef enum
{
    BT_DTMF_KEY_0,          /**< 0 */
    BT_DTMF_KEY_1,          /**< 1 */
    BT_DTMF_KEY_2,          /**< 2 */
    BT_DTMF_KEY_3,          /**< 3 */
    BT_DTMF_KEY_4,          /**< 4 */
    BT_DTMF_KEY_5,          /**< 5 */
    BT_DTMF_KEY_6,          /**< 6 */
    BT_DTMF_KEY_7,          /**< 7 */
    BT_DTMF_KEY_8,          /**< 8 */
    BT_DTMF_KEY_9,          /**< 9 */
    BT_DTMF_KEY_STAR,       /**< * */
    BT_DTMF_KEY_HASH,       /**< # */
    BT_DTMF_KEY_MAX
} bt_dtmf_key_t;
#endif


#ifdef BT_USING_MIC_MUTE
typedef enum
{
    BT_MIC_MUTE_DISABLE,    /**< mic unmute */
    BT_MIC_MUTE_ENABLE,     /**< mic mute */
    BT_MIC_MUTE_MAX
} bt_mic_mute_t;
#endif

#ifdef BT_USING_LOCAL_MEDIA_EX
typedef enum
{
    BT_MUSIC_PLAY_SEQUENCE_MODE,
    BT_MUSIC_PLAY_RANDOM_MODE,
    BT_MUSIC_PLAY_SINGLE_MODE,
    BT_MUSIC_PLAY_MODE_MAX
} bt_music_play_mode_t;
#endif

typedef enum
{
    BT_EOK = 0,
    /* general error code */
    BT_ERROR_INPARAM            = 0x10000001,  /**< input param error */
    BT_ERROR_UNSUPPORTED        = 0x10000002,  /**< unsupported function */
    BT_ERROR_TIMEOUT            = 0x10000003,  /**< error timout */
    BT_ERROR_DISCONNECTED       = 0x10000004,  /**< the bt device is disconnected */
    BT_ERROR_STATE              = 0x10000005,  /**< current state  unsupported this function */
    BT_ERROR_PARSING            = 0x10000006,  /**< parsing at response error */
    BT_ERROR_POWER_OFF          = 0x10000007,  /**< current bt device has been power off */
    BT_ERROR_NOTIFY_CB_FULL     = 0x10000008,  /**< register notify cb is more than BT_MAX_EVENT_NOTIFY_CB_NUM */
    BT_ERROR_DEVICE_EXCEPTION   = 0x10000009,  /**< current bt device has happend exception */
    BT_ERROR_RESP_FAIL          = 0x10000010,  /**< at cmd response fail */
    BT_ERROR_AVRCP_NO_REG       = 0x10000011,  /**<set absolute volume, but remote device dont register the event */
    BT_ERROR_IN_PROGRESS        = 0x10000012,  /**<for non-blocking processing, wait until the corresponding event is reported */
    BT_ERROR_OUT_OF_MEMORY      = 0x10000013,  /**<System heap is not enough */
} bt_err_t;

typedef enum
{
    BT_PROFILE_HFP = 0,
    BT_PROFILE_AVRCP,
    BT_PROFILE_A2DP,
    BT_PROFILE_PAN,
    BT_PROFILE_HID,
    BT_PROFILE_AG,
    BT_PROFILE_SPP,
    BT_PROFILE_BT_GATT,
    BT_PROFILE_PBAP,
    BT_PROFILE_MAX
} bt_profile_t;


typedef enum
{
    BT_STATE_MEDIA_IDLE = 0,
    BT_STATE_MEDIA_PAUSE,
    BT_STATE_MEDIA_PLAY,
    BT_STATE_MEDIA_ERROR
} bt_media_state_t;

typedef struct
{
    uint8_t status;
    uint8_t conn_idx;
} bt_media_play_status_t;

typedef struct
{
    int size;
    uint8_t number[BT_MAX_PHONE_NUMBER_LEN];
} phone_number_t;

typedef struct
{
    bt_3way_coded_t cmdCode;
    int index;//invalid_value:0xff
} bt_3way_hold_t;

typedef enum
{
    BT_PUT_INCOM_CALL_HOLD = 0,
    BT_ACPT_HELD_INCOME_CALL,
    BT_REJ_HELD_INCOME_CALL,
    BT_GET_HOLD_STATUS,
    BT_3WAY_INCOME_MAX
} bt_3way_incom_t;

typedef enum
{
    BT_ROLE_MASTER = 0x55,
    BT_ROLE_SLAVE = 0xAA,
} bt_role_t;

/* PhoneBook repositories */
typedef enum
{
    BT_PBAP_CURRENT,
    BT_PBAP_LOCAL,
    BT_PBAP_SIM1,
    BT_PBAP_UNKNOWN_REPO
} pbap_phone_repository_t;

/* Phone Book Objects */
typedef enum
{
    BT_PBAP_TELECOM,
    BT_PBAP_PB,
    BT_PBAP_ICH,
    BT_PBAP_OCH,
    BT_PBAP_MCH,
    BT_PBAP_CCH,
    BT_PBAP_UNKNOWN_PHONEBOOK
} pbap_phone_book_t;

#define PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE    42
#define PBAP_MAX_VCARD_CONTACT_NAME_SIZE    80
typedef struct
{
    uint8_t vcard_handle_len;
    uint8_t name_len;
    char vcard_handle[PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE + 1];
    char vcard_name[PBAP_MAX_VCARD_CONTACT_NAME_SIZE + 1];
} pbap_vcard_listing_item_t;

typedef struct
{
    uint8_t name_len;
    char vcard_name[PBAP_MAX_VCARD_CONTACT_NAME_SIZE + 1];
} pbap_vcard_list_t;

typedef struct
{
    pbap_phone_repository_t repos;
    pbap_phone_book_t phone_book;
    uint8_t max_size;
} bt_pbap_pb_info;

typedef struct
{
    pbap_phone_repository_t repos;
    pbap_phone_book_t phone_book;
} bt_pbap_pb_set_t;

typedef struct
{
    uint8_t password_len;
    uint8_t password[];
} bt_pbap_auth_info;

typedef struct
{
    uint8_t call_num;
    uint8_t active_idx;//the index of call to be handled currently
    uint8_t ring_type;  /* 1:inband ring 0:local ring */
    bt_call_state_t active_state;
    uint8_t dir[BT_MAX_CALL_NUM];        /*0:outgoing 1:incoming*/
    phone_number_t phone_number[BT_MAX_CALL_NUM];
#ifdef BT_USING_PBAP
    pbap_vcard_list_t contacts[BT_MAX_CALL_NUM];
#endif
} bt_call_info_t;


typedef struct
{
    uint16_t phone_number_type;
    uint8_t  idx;      /* Current call index, counting from 1 */
    uint8_t  dir;      /* Telephone direction 0:outgoing 1:incoming */
    uint8_t  st;       /* clcc status
                        0: Active
                        1: held
                        2: Dialing(outgoing calls only)
                        3: Alerting(outgoing calls only)
                        4: Incoming (incoming calls only)
                        5: Waiting (incoming calls only)
                        6: Call held by Response and Hold */
    uint8_t  mode;     /*Telephone mode£¬0 (Voice), 1 (Data), 2 (FAX) */
    uint8_t  mpty;     /* Multiparty call identification 0: not multiparty call 1:multiparty call */
    uint8_t  number_size;
    uint8_t  *number;
} bt_clcc_ind_t;


typedef struct
{
    int size;
    const char *name;
} set_name_t;

typedef struct
{
    char addr[BT_MAX_MAC_LEN];
} bt_mac_t;


typedef struct
{
    uint8_t *data;
    uint16_t len;
    uint8_t srv_chl;
    bt_mac_t mac_addr;
} spp_data_t;

typedef struct
{
    uint8_t srv_chl;
    bt_mac_t mac_addr;
} spp_common_t;

typedef struct
{
    uint16_t event;
    void *args;
} bt_notify_t;

typedef struct
{
    uint8_t conn_idx;
    bt_profile_t profile;
    bt_mac_t mac;         /**< the bt peer mac*/
} bt_connect_info_t;

typedef struct
{
    uint8_t inband_ring; /* 1:hf support inband ring tone; 0:hf dont support inband ring tone */
} bt_config_t;

typedef struct
{
    uint8_t stack_ready;
    uint8_t sco_link;   /* 0:remote device 1:local device */
    uint8_t siri_status; /* 0:off 1:on */
    uint8_t clcc_process_status;

    struct state_machine media_fsm[BT_MAX_ACL_NUM];
    struct state_machine device_fsm;
    struct state_machine acl_fsm[BT_MAX_ACL_NUM];
    struct state_machine connect_fsm[BT_MAX_ACL_NUM][BT_PROFILE_MAX];
    struct state_machine call_fsm[BT_MAX_CALL_NUM];
} bt_fsm_t;


typedef struct
{
    uint8_t conn_idx;
    bt_disconnect_reason_t reason;
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    bt_profile_t profile;
} bt_disconnect_info_t;

typedef struct
{
    uint8_t conn_idx;
    bt_disconnect_reason_t reason;
    bt_mac_t peer_addr;         /**< the bt peer mac*/
} bt_acl_disconnect_info_t;

typedef struct
{
    uint8_t conn_idx;
    bt_mac_t peer_addr;         /**< the bt peer mac*/
} bt_key_missing_info_t;

typedef struct
{
    uint8_t conn_idx;
    bt_mac_t peer_addr;         /**< the bt peer mac*/
} bt_encryption_info_t;

typedef struct
{
    bt_mac_t mac_addr;          /**< the bt device mac*/
    int rssi;                   /**< the bt device signal strength unit:dbm */
    uint32_t name_size;         /**< the bt device name size */
    char *bt_name;        /**< the bt device name,utf8 */
} bt_serached_device_info_t;

typedef struct
{
    uint8_t conn_idx;
    bt_mac_t mac_addr;          /**< the bt device mac*/
    uint8_t res;                /**the status code,0:success,other:error code */
    uint8_t incoming;           /**< 1:incoming;0:outgoing */
    uint32_t dev_cls;           /**< 0x001f00 audio box */
} bt_acl_opened_t;

typedef struct
{
    uint8_t conn_idx;
    bt_mac_t mac_addr;          /**< the bt device mac*/
    uint8_t res;                /**the status code,0:success,other:error code */
} bt_pair_ind_t;

typedef struct
{
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    uint32_t type;              /**< 0:connect 1:discocnnet*/
} bt_hfp_audio_switch_t;

typedef struct
{
    bt_mac_t            addr;
    uint8_t             res;
    char                name[61];
} bt_rmt_name_t;



#pragma pack(push,1)
typedef struct
{
    char frame_id[4];
    uint32_t size;
    uint16_t flag;
} bt_mp3_tag_header_t;

typedef struct
{
    uint8_t encode;
    // char bom[2];
    const char *data;
} bt_mp3_tag_vaule_t;


typedef struct
{
    char header[5];  /**< "+MP3:" */
    char addr[4];
    uint16_t tag_data_size;
    uint32_t song_total_size;
    uint32_t duration; /**< unit:ms */
} bt_mp3_header_t;

typedef struct
{
    char header[6];  /**< "+NAME:" */
    char addr[4];
    uint16_t file_name_size;
} bt_mp3_brief_header_t;
#pragma pack(pop)


#define BT_MAX_SONG_NAME_LEN    (128)
#define BT_MAX_SINGER_NAME_LEN  (128)
#define BT_MAX_ALBUM_INFO_LEN   (128)
#define BT_MAX_PLAY_TIME_LEN    (8)


typedef struct
{
    uint32_t size;
    uint8_t song_name[BT_MAX_SONG_NAME_LEN];
} mp3_song_name_t;

typedef struct
{
    uint32_t size;
    uint8_t singer_name[BT_MAX_SINGER_NAME_LEN];
} mp3_singer_name_t;


typedef struct
{
    uint32_t size;
    uint8_t album_name[BT_MAX_ALBUM_INFO_LEN];
} mp3_album_info_t;

typedef struct
{
    uint32_t size;
    uint8_t play_time[BT_MAX_PLAY_TIME_LEN];//ascii code  ,unit:ms
} mp3_play_time_t;




typedef struct
{
    uint32_t  song_total_size;          /**< the song's total length */
    mp3_play_time_t duration;                  /**< the song's total duration */
    mp3_song_name_t song_name;          /**< the song's name */
    mp3_singer_name_t singer_name;      /**< the song's singer name */
    mp3_album_info_t album_info;        /**< the song's album name */
    uint16_t          character_set_id;  //UTF-8 0x006A; other??
} bt_mp3_detail_info_t;


typedef struct
{
    mp3_song_name_t song_name;      /**< the song's name */
} bt_mp3_brief_info_t;


typedef struct
{
    bt_direction_t dir;
    void *args;
} bt_mp3_info_t;

#ifdef BT_USING_LOCAL_MEDIA_EX

typedef struct
{
    uint32_t offset;            /**< Music list information read offset, */
    uint32_t num;               /**< Number of music list reads */
} bt_music_list_pageinfo_t;


typedef struct
{
    uint32_t music_id;
    mp3_song_name_t song_name;
    mp3_singer_name_t singer_name;
} bt_music_info_t;

typedef struct
{
    uint32_t num;                /**< number of get music info*/
    bt_music_info_t *info;
} bt_music_list_info_t;
#endif

typedef struct
{
    uint8_t media_volume;   /**< the media volume 0-15 */
    uint8_t call_volume;   /**< the call volume 0-15 */
    uint8_t prompt_volume;
} bt_volume_t;

typedef enum
{
    BT_VOLUME_NO_SAVE = 0,
    BT_VOLUME_SAVE,
} bt_volume_save;

typedef struct
{
    bt_volume_t volume;
    bt_volume_mode_t mode;
    bt_volume_save save;
} bt_volume_set_t;

typedef struct
{
    uint8_t *payload;
    uint16_t payload_len;
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    uint8_t srv_chl;
    uint8_t *uuid;
    uint8_t uuid_len;
} bt_spp_data_t;

typedef struct
{
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    uint8_t srv_chl;
    uint8_t *uuid;
    uint8_t uuid_len;
} bt_spp_data_cfm_t;

typedef struct
{
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    uint8_t srv_chl;
    uint8_t *uuid;
    uint8_t uuid_len;
    uint16_t mtu_size;
} bt_spp_conn_ind_t;

typedef struct
{
    bt_mac_t peer_addr;         /**< the bt peer mac*/
    uint8_t srv_chl;
    uint8_t *uuid;
    uint8_t uuid_len;
} bt_spp_disconn_ind_t;

typedef struct
{
    uint16_t phone_len;
    uint8_t  phone_num[20];
    uint8_t phone_type;
} bt_call_wait_t;




typedef struct
{
    bt_mac_t mac;
    bt_device_type_t type;
} bt_peer_deviceinfo_t;

typedef struct
{
    uint8_t    res;
    uint8_t    lmp_version;
    uint16_t   manufacturer_name;
    uint16_t   lmp_subversion;
} bt_rmt_version_t;

#pragma pack(push,1)
typedef struct
{
    uint16_t cmd;
    bt_err_t err;
    uint16_t data_len;
    void *data;
} bt_control_result_t;
#pragma pack(pop)

#ifdef BT_USING_PAIRING_CONFIRMATION
typedef struct
{
    bt_mac_t mac;
    uint32_t num_val;
} bt_pair_confirm_t;

// IO Capability
typedef enum
{
    CAPABILITY_DISPLAY_ONLY,         // Display Only
    CAPABILITY_DISPLAY_YES_NO,       // Display Yes/No
    CAPABILITY_KEYBOARD_ONLY,        // Keyboard Only
    CAPABILITY_NO_INPUT_NO_OUTPUT,   // No Input/Output
    CAPABILITY_REJECT_REQ            // Use this to reject the IO capability request
} bts2e_io_capability;

typedef struct
{
    bt_mac_t mac;
    bts2e_io_capability io_capability;
    uint8_t mitm;
    uint8_t bonding;
} bt_io_capability_rsp_t;

typedef struct
{
    bt_mac_t mac;
    uint8_t confirm;        //TRUE or FALSE.
} bt_user_confirm_rsp_t;
#endif

#ifdef BT_USING_AG
//#define PHONE_NUM_LEN 23
typedef struct
{
    char phone_number[23];
    uint8_t type;
} hfp_phone_num_t;

typedef struct
{
    uint8_t call_idx;
    uint8_t call_dir;
    uint8_t call_status;
    uint8_t call_mode;
    uint8_t call_mtpty;
    hfp_phone_num_t phone_info;
} hfp_phone_calls_info_t;

typedef struct
{
    uint8_t service_status;
    uint8_t call;
    uint8_t callsetup;
    uint8_t batt_level;
    uint8_t signal;
    uint8_t roam_status;
    uint8_t callheld;
} hfp_cind_state_t;

typedef struct
{
    uint16_t type;
    uint8_t num_active;
    uint8_t num_held;
    uint8_t callsetup_state;
    uint8_t phone_type;
    uint8_t phone_len;
    uint8_t phone_number[1];
} hfp_call_info_t;

typedef struct
{
    uint8_t ind_type;
    uint8_t ind_val;
} hfp_ind_info_t;

typedef struct
{
    uint8_t num_call;
    hfp_phone_calls_info_t *calls;
} hfp_remote_call_info_t;

#endif


typedef void (*bt_notify_cb)(bt_notify_t *param);


typedef struct
{
    uint32_t size;
    bt_notify_cb cb[BT_MAX_EVENT_NOTIFY_CB_NUM];
} bt_notify_cb_array_t;

typedef struct rt_bt_device
{
    struct rt_device   parent;
    bt_notify_cb_array_t cb_arry;
    rt_mutex_t handle_lock;
    rt_mutex_t control_lock;
    rt_mutex_t   call_sem;

    bt_role_t role;
    bt_fsm_t fsm;
    bt_call_info_t call_info;
    bt_config_t config;

    const struct rt_bt_ops *ops;
} rt_bt_device_t;

typedef bt_err_t (*bt_control_cb)(struct rt_bt_device *dev_handle, int cmd, void *arg);

struct rt_bt_ops
{
    bt_control_cb control;
};

void rt_bt_event_notify(bt_notify_t      *param);
bt_connect_state_t rt_bt_get_connect_state(rt_bt_device_t *dev, bt_profile_t profile);
bt_role_t rt_bt_get_role(rt_bt_device_t *dev);
bt_acl_state_t rt_bt_get_acl_state(rt_bt_device_t *dev);
bt_state_t rt_bt_get_device_state(rt_bt_device_t *dev);
bt_media_state_t rt_bt_get_media_state(rt_bt_device_t *dev);
bt_call_info_t *rt_bt_get_call_info(rt_bt_device_t *dev);
bt_call_state_t rt_bt_get_call_state(rt_bt_device_t *dev);
rt_err_t rt_bt_register(struct rt_bt_device *dev_handle, const char *name);
bt_call_state_t rt_bt_get_call_state_by_idx(rt_bt_device_t *dev, uint8_t idx);
uint8_t rt_bt_get_hfp_sco_link(rt_bt_device_t *dev);

bt_connect_state_t rt_bt_get_connect_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx, bt_profile_t profile);
bt_acl_state_t rt_bt_get_acl_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx);
bt_media_state_t rt_bt_get_media_state_by_conn_idx(rt_bt_device_t *dev, uint8_t idx);


#endif
