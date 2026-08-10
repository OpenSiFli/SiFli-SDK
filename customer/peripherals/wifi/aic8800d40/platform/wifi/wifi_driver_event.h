/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WIFI_DRIVER_EVENT_H__
#define __WIFI_DRIVER_EVENT_H__
/*!
 * @file     wifi_driver_event.h
 * @brief    APIs&data structure for WiFi driver event.
 */

/*!
 * \addtogroup network
 * @{
 *     @defgroup WiFiDriver
 *     @brief Declare WiFi driver event data structure type.
 *     @{
 */

#ifndef ETH_ALEN
#define ETH_ALEN 6 /*!< Macro defines the length of the MAC address string */
#endif


/*!
 * @brief Declare WiFi device status.
 */
enum wifi_dev_driver_status {
	WIFI_DEVICE_DRIVER_UNKNOWN = 0,  /*!< Default status(0) */
	WIFI_DEVICE_DRIVER_LOADED,       /*!< WiFi ready. Network device recognition and successful loading */
	WIFI_DEVICE_DRIVER_UNLOAD,       /*!< Network device exit */
};

/*!
 * @brief Declare the data structure of the WiFi device event.
 */
struct wifi_dev_event {
	enum wifi_dev_driver_status drv_status;   /*!< The status of the WiFi device */
	unsigned char local_mac_addr[ETH_ALEN];   /*!< Network device local MAC address */
};

/*!
 * @brief Declare WiFi driver P2P event type.
 */
enum wifi_p2p_event_type {
	WIFI_P2P_EVENT_UNKNOWN = 0,                        /*!< Default type(0) */
	WIFI_P2P_EVENT_GOT_PRO_DISC_REQ_AFTER_GONEGO_OK,   /*!< Received a mobile terminal Provisioning Discovery request */
	WIFI_P2P_EVENT_ON_ASSOC_REQ,                       /*!< Receive connection request event */
	WIFI_P2P_EVENT_ON_DISASSOC,                        /*!< Disconnection to P2P device */
};

/*!
 * @brief Declare the data structure of the WiFi driver p2p event.
 */
struct wifi_p2p_event {
	enum wifi_p2p_event_type event_type;       /*!< The type of the WiFi p2p event */
	unsigned char peer_dev_mac_addr[ETH_ALEN]; /*!< The peer device's MAC address */
	unsigned int peer_dev_port;                /*!< TCP port number at which the peer WFD device listens for RTSP messages */
};

/*!
 * @brief Declare WiFi driver AP event type.
 */
enum wifi_ap_event_type {
	WIFI_AP_EVENT_UNKNOWN = 0,                /*!< Default type(0) */
	WIFI_AP_EVENT_ON_DISASSOC,                /*!< Disconnect to a station */
	WIFI_AP_EVENT_ON_ASSOC,                   /*!< Connect to a station */
};

/*!
 * @brief Declare the data structure of the WiFi driver AP event.
 */
struct wifi_ap_event {
	enum wifi_ap_event_type event_type;       /*!< The type of the WiFi AP event */
	unsigned char peer_dev_mac_addr[ETH_ALEN];/*!< peer device MAC address */
};

/*!
 * @brief Declare WiFi driver Station event type.
 */
enum wifi_sta_event_type {
	WIFI_STA_EVENT_UNKNOWN = 0,                /*!< Default type(0) */
	WIFI_STA_EVENT_ON_DISASSOC,                /*!< Disconnect to a AP */
	WIFI_STA_EVENT_ON_ASSOC_FAIL,              /*!< Connect to a AP timeout*/
	WIFI_STA_EVENT_ON_ASSOC,                   /*!< Connect to a AP */
};

/*!
 * @brief Declare the data structure of the WiFi driver Station event.
 */
struct wifi_sta_event {
	enum wifi_sta_event_type event_type;       /*!< The type of the WiFi Station event */
	unsigned char peer_dev_mac_addr[ETH_ALEN]; /*!< peer device MAC address */
};

/*!
 * @brief Declare WiFi driver event type.
 */
enum wifi_drv_event_type {
	WIFI_DRV_EVENT_DEFAULT = 0,               /*!< Default type(0) */
	WIFI_DRV_EVENT_NET_DEVICE,                /*!< Network device event */
	WIFI_DRV_EVENT_AP,                        /*!< WiFi driver AP event */
	WIFI_DRV_EVENT_P2P,                       /*!< WiFi driver P2P event */
	WIFI_DRV_EVENT_STA,                       /*!< WiFi driver Station event */
};

union wifi_drv_event_node {
	struct wifi_dev_event     dev_event;
	struct wifi_p2p_event     p2p_event;
	struct wifi_ap_event      ap_event;
	struct wifi_sta_event     sta_event;
};

/*!
 * @brief Declare the data structure of the WiFi driver event.
 */
typedef struct
{
	enum wifi_drv_event_type type;            /*!< wifi drv event type */
	union wifi_drv_event_node node;           /*!< wifi drv event information */
} wifi_drv_event;

/**
 * @brief Declare WiFi driver event callback function.
 * @param drv_event[out]  - Network event information, NetDevice event,P2P event, AP event.
 * @return success or not.
 * @retval 0  : success.
 * @retval -1 : fail,should be programming error, must fix.
 * @note
 *	\li The SDK should distinguish between different types of WiFi driver events and take different treatments.
 */
typedef int (*wifi_drv_event_cbk)(const wifi_drv_event* drv_event);

/**
 * @brief Setting WiFi driver event cbk.
 * @param cbk[in] WiFi driver event cbk.
 * @return success or not.
 * @retval 0  : success.
 * @retval -1 : fail.
 * @note
 * \li The initial network function is called.
 */
int wifi_drv_event_set_cbk(wifi_drv_event_cbk cbk);

/**
 * @brief Reset WiFi driver event cbk.
 * @param void
 * @return void
 * @note
 * \li Exiting the network function is called.
 */
void wifi_drv_event_reset_cbk(void);


/**
 * @brief Initialize the WiFi driver event mechanism.
 * @note
 * \li This interface is called when the WiFi Driver is initialized, and one of the interfaces that are not called by the SDK.
 */
int wifi_drv_event_initialize(void);

/**
 * @brief Finalize the WiFi driver event mechanism.
 * @note
 * \li This interface is called when the WiFi Driver is finalized, and one of the interfaces that are not called by the SDK.
 */
int wifi_drv_event_finalize(void);

/**
 * @brief Report netdevice event.
 * @note
 * \li This interface is called by the WiFi driver when the driver generates the corresponding event.
 * \li One of the interfaces that are not called by the SDK.
 */
void wifi_drv_event_upper_netdev_event(struct wifi_dev_event* upper_event);
/**
 * @brief Report wifi P2P event.
 * @note
 * \li This interface is called by the WiFi driver when the driver generates the corresponding event.
 * \li One of the interfaces that are not called by the SDK.
 */
void wifi_drv_event_upper_p2p_event(struct wifi_p2p_event* upper_event);
/**
 * @brief Report wifi AP event.
 * @note
 * \li This interface is called by the WiFi driver when the driver generates the corresponding event.
 * \li One of the interfaces that are not called by the SDK.
 */
void wifi_drv_event_upper_ap_event(struct wifi_ap_event* upper_event);

/*!
 *    @} end of defgroup WiFiDriver
 * @} end of addtogroup network
 */

typedef void (*wifi_priv_event_cb_asr)(unsigned char *MAC_addr, unsigned char mode);

#endif