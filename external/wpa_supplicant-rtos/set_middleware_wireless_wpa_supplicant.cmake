include_guard(GLOBAL)


if (CONFIG_USE_middleware_wireless_wpa_supplicant_rtos)
# Add set(CONFIG_USE_middleware_wireless_wpa_supplicant_rtos true) in config.cmake to use this component

message("middleware_wireless_wpa_supplicant_rtos component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_freertos-kernel AND CONFIG_USE_utility_debug_console AND CONFIG_USE_component_osa_free_rtos AND CONFIG_USE_middleware_wifi_cli AND CONFIG_USE_middleware_wifi AND CONFIG_USE_middleware_lwip AND CONFIG_USE_middleware_mbedtls)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./port/mbedtls/supp_crypto.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/wpa_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/ieee802_11_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/hw_features_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/wpa_ctrl.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/drivers/driver_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/drivers/drivers.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/l2_packet/l2_packet_freertos.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/drivers/driver_freertos.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/base64.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/wpabuf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/bitfield.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/os_freertos.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/crc32.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/ip_addr.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/block_alloc.c
  ${CMAKE_CURRENT_LIST_DIR}/./freertos/src/wpa_cli.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/config.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/notify.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/bss.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/eap_register.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/op_classes.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/rrm.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/wmm_ac.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/config_none.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/sme.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/wpa_supplicant.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/events.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/bssid_ignore.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/wpas_glue.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/scan.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/robust_av.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/ctrl_iface.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/ctrl_iface_udp.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/mbo.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/wnm_sta.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/wpa_debug.c
  ${CMAKE_CURRENT_LIST_DIR}/./freertos/src/supp_main.c
  ${CMAKE_CURRENT_LIST_DIR}/./freertos/src/supp_api.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/ap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ap_config.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ap_drv_ops.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ap_list.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ap_mlme.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/authsrv.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/beacon.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/bss_load.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/dfs.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/drv_callbacks.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/eap_user_db.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/hostapd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/hw_features.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11_auth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11_he.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11_ht.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11_shared.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_11_vht.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ieee802_1x.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/neighbor_db.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/p2p_hostapd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/pmksa_cache_auth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/preauth_auth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/rrm.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/sta_info.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/tkip_countermeasures.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/utils.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wmm.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/ctrl_iface_ap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/mbo_ap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wpa_auth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wpa_auth_ie.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wpa_auth_ft.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wpa_auth_glue.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/eap_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_identity.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_methods.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eapol_auth/eapol_auth_sm.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-eax.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-encblock.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-ctr.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-cbc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-siv.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-unwrap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-wrap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/des-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/dh_groups.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/rc4.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/md4-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/md5.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/md5-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha1-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/fips_prf_internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha256-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha1.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha256.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/tls_none.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/ms_funcs.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha1-tlsprf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha1-prf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha256-prf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha1-pbkdf2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-omac1.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-internal-enc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-internal-dec.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/aes-internal.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp/wpa.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp/preauth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp/pmksa_cache.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp/wpa_ie.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/crypto_mbedtls2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/tls_mbedtls2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha256-kdf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha384-kdf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/sha512-kdf.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/sae.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dragonfly.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/p2p_supplicant.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/p2p_supplicant_sd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_utils.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_parse.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_build.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_go_neg.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_sd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_pd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_invitation.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_dev_disc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p/p2p_group.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/gas.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/gas_query.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/offchannel.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/wps_supplicant.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/uuid.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_wsc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/eap_wsc_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wps_hostapd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_attr_parse.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_attr_build.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_attr_process.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_dev_attr.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_enrollee.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps/wps_registrar.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eapol_supp/eapol_supp_sm.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_methods.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_tls.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_tls_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_peap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/eap_peap_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_ttls.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_mschapv2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/chap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/mschapv2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_fast.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_fast_pac.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_gtc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/eap_fast_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./hostapd/hlr_auc_gw.c
  ${CMAKE_CURRENT_LIST_DIR}/./hostapd/ctrl_iface.c
  ${CMAKE_CURRENT_LIST_DIR}/./hostapd/eap_register.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp/wpa_ft.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/wnm_ap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/eth_p_oui.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_wsc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_tls.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/radius/radius_server.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/radius/radius_das.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/radius/radius_client.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/radius/radius.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_sim_db.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common/eap_sim_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_ttls.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_tls_common.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_peap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_mschapv2.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_md5.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_gtc.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_fast.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_sim.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server/eap_server_aka.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/acs.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/eloop_freertos.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto/milenage.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_sim.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer/eap_aka.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/dpp_supplicant.c
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant/interworking.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_auth.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_backup.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_crypto.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_pkex.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_reconfig.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/dpp_tcp.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/common/gas_server.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils/json.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/tls/asn1.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/dpp_hostapd.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/gas_query_ap.c
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap/gas_serv.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./.
  ${CMAKE_CURRENT_LIST_DIR}/./src
  ${CMAKE_CURRENT_LIST_DIR}/./src/utils
  ${CMAKE_CURRENT_LIST_DIR}/./src/common
  ${CMAKE_CURRENT_LIST_DIR}/./src/crypto
  ${CMAKE_CURRENT_LIST_DIR}/./src/ap
  ${CMAKE_CURRENT_LIST_DIR}/./src/drivers
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_common
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_peer
  ${CMAKE_CURRENT_LIST_DIR}/./src/eap_server
  ${CMAKE_CURRENT_LIST_DIR}/./src/eapol_auth
  ${CMAKE_CURRENT_LIST_DIR}/./src/eapol_supp
  ${CMAKE_CURRENT_LIST_DIR}/./src/fst
  ${CMAKE_CURRENT_LIST_DIR}/./src/l2_packet
  ${CMAKE_CURRENT_LIST_DIR}/./src/p2p
  ${CMAKE_CURRENT_LIST_DIR}/./src/pae
  ${CMAKE_CURRENT_LIST_DIR}/./src/radius
  ${CMAKE_CURRENT_LIST_DIR}/./src/rsn_supp
  ${CMAKE_CURRENT_LIST_DIR}/./src/tls
  ${CMAKE_CURRENT_LIST_DIR}/./src/wps
  ${CMAKE_CURRENT_LIST_DIR}/./port/mbedtls
  ${CMAKE_CURRENT_LIST_DIR}/./hostapd
  ${CMAKE_CURRENT_LIST_DIR}/./wpa_supplicant
  ${CMAKE_CURRENT_LIST_DIR}/./freertos
  ${CMAKE_CURRENT_LIST_DIR}/./freertos/src
)

if(CONFIG_USE_COMPONENT_CONFIGURATION)
  message("===>Import configuration from ${CMAKE_CURRENT_LIST_FILE}")

  target_compile_definitions(${MCUX_SDK_PROJECT_NAME} PUBLIC
    -DUSE_RTOS=1
    -DSDK_OS_FREE_RTOS
    -DFSL_OSA_TASK_ENABLE=1
    -DPRINTF_ADVANCED_ENABLE=1
    -DMBEDTLS_CONFIG_FILE="wpa_supp_mbedtls_config.h"
  )

  if(CONFIG_TOOLCHAIN STREQUAL iar)
    target_compile_options(${MCUX_SDK_PROJECT_NAME} PUBLIC
      --dlib_config full
    )
  endif()
  if(CONFIG_TOOLCHAIN STREQUAL armgcc)
    target_compile_options(${MCUX_SDK_PROJECT_NAME} PUBLIC
      -fomit-frame-pointer
      -Wno-unused-function
    )
  endif()
  if(CONFIG_TOOLCHAIN STREQUAL mcux)
    target_compile_options(${MCUX_SDK_PROJECT_NAME} PUBLIC
      -fomit-frame-pointer
    )
  endif()

endif()

else()

message(SEND_ERROR "middleware_wireless_wpa_supplicant_rtos dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()

