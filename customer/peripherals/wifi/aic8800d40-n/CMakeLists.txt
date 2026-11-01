get_filename_component(AIC_PACKAGE_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PACKAGE_NAME softutil-${AIC_PACKAGE_DIR_NAME})
set(PRIV_PACKAGE_NAME ${PACKAGE_NAME}_priv)
set(OBJ_PACKAGE_NAME ${PACKAGE_NAME}_obj)
set(PRIV_OBJ_PACKAGE_NAME ${PRIV_PACKAGE_NAME}_obj)

set(PACKAGE_PATH ${XROOT}/softutil/${AIC_PACKAGE_DIR_NAME})
set(PACKAGE_DRIVER_PATH   ${PACKAGE_PATH}/host/driver)
set(PACKAGE_PLATFORM_PATH ${PACKAGE_PATH}/platform)
set(PACKAGE_WPA_PATH      ${PACKAGE_PATH}/host/thirdparty/wpa_supplicant)
set(PACKAGE_MBEDTLS_PATH  ${PACKAGE_PATH}/host/thirdparty/mbedtls_v3_6_2)

# 平台 RTOS 端口文件 (platform/rtos/ 下): 默认 FreeRTOS 参考实现,
# 平台用 -DAIC_RTOS_PORT_SRC=rtos_threadx.c 之类覆盖 (如 ASR/064 ThreadX)。
if(NOT DEFINED AIC_RTOS_PORT_SRC)
    set(AIC_RTOS_PORT_SRC rtos_freertos.c)
endif()

# ============================================================================
# host 构建模式 (与 Makefile AIC_BUILD_HOST 对齐)
#   AIC_USE_PREBUILT_HOST=ON  (host/ 不存在时默认): 引用 lib/ 预编译库,
#       只编译 platform 源码 (AIC_BUILD_HOST=0 语义), 头文件用 lib/include + port/inc
#   AIC_USE_PREBUILT_HOST=OFF (host/ 存在时默认): 全源码编译 host+platform
#       (AIC_BUILD_HOST=1 语义, 开发调试用)
# host 库由 make 体系 (lib-sync/release) 产出到 lib/, CMake 不再直接编译 host 源码
# ============================================================================
set(AIC_USE_PREBUILT_HOST "" CACHE STRING "ON=引用 lib 预编译库; OFF=全源码编译 (空=自动检测 host/)")
if(NOT AIC_USE_PREBUILT_HOST)
    if(EXISTS ${PACKAGE_PATH}/host)
        set(AIC_USE_PREBUILT_HOST OFF)
    else()
        set(AIC_USE_PREBUILT_HOST ON)
    endif()
endif()

set(PACKAGE_DFLAGS)
set(PACKAGE_SRC_OPEN_FILES)
set(PACKAGE_SRC_PRIV_FILES)

# ============================================================================
# mbedTLS crypto 层选择 (与 Makefile 同名变量对齐)
#   USE_BUILTIN_MBEDTLS=1 (默认): 内置 mbedtls crypto (引用模式链接 libaic_mbedtls.a)
#   USE_BUILTIN_MBEDTLS=0: 平台自实现 aic_mbedtls_* 符号 —— 按 Makefile 口径自动编入
#     转发示例 platform/crypto/aic_mbedtls_port_example.c (AIC_MBEDTLS_FWD_EXAMPLE=0 可关)。
# 此前 USE_BUILTIN_MBEDTLS 只在引用模式块内声明, 全源码模式取不到值 → 示例源从不编入。
# ============================================================================
set(USE_BUILTIN_MBEDTLS 1 CACHE STRING "1=builtin mbedtls crypto; 0=platform provides aic_mbedtls_* symbols")
set(AIC_MBEDTLS_FWD_EXAMPLE 1 CACHE STRING "with USE_BUILTIN_MBEDTLS=0: 1=compile platform/crypto/aic_mbedtls_port_example.c; 0=platform provides its own")

set(PACKAGE_INC_PATHS			${XROOT}/os/threadx/inc
								${XROOT}/hal/SDIO/inc
								${XROOT}/hal/MMU/inc
								${XROOT}/hop/telephony/atparser/inc
								${XROOT}/hop/ssipc_mat/inc
								${XROOT}/hop/telephony/atcmdsrv
								${XROOT}/hop/telephony/atparser
								${XROOT}/../PS/ci/inc
								${XROOT}/pcac/atcmdsrv/inc
								${XROOT}/pcac/duster/inc            ## 1.7.6  SDK
								#${XROOT}/pcac/fota/inc				## 1.7.15 SDK
								${XROOT}/pcac/mbedTLS/include 
								${XROOT}/pcac/mbedTLS/asros 
								${XROOT}/pcac/lwipv4v6/src/include 
								${XROOT}/pcac/lwipv4v6/src/include/lwip
								${XROOT}/pcac/lwipv4v6/src/include/arch
								${XROOT}/pcac/lwipv4v6/src/include/ipv4
								${XROOT}/pcac/lwipv4v6/src/include/ipv6
if(AIC_USE_PREBUILT_HOST)
								${PACKAGE_PATH}/lib/include/drv_api/inc
								${PACKAGE_PATH}/lib/include/drv_wifi
								${PACKAGE_PATH}/lib/include/drv_wlan
								${PACKAGE_PATH}/lib/include/utils/inc
								${PACKAGE_PATH}/port/inc
else()
								${PACKAGE_DRIVER_PATH}/drv_api/inc
								${PACKAGE_DRIVER_PATH}/drv_fhost
								${PACKAGE_DRIVER_PATH}/drv_macif
								${PACKAGE_DRIVER_PATH}/drv_wifi
								${PACKAGE_DRIVER_PATH}/drv_wlan
								${PACKAGE_DRIVER_PATH}/drv_bt
								${PACKAGE_PATH}/port/inc
								${PACKAGE_DRIVER_PATH}/api
								${PACKAGE_DRIVER_PATH}/layer_common/inc
endif()
								${PACKAGE_PLATFORM_PATH}/fmac
								${PACKAGE_PLATFORM_PATH}/fmac/fw
								${PACKAGE_PLATFORM_PATH}/bt
								${PACKAGE_PLATFORM_PATH}/hal
								${PACKAGE_PLATFORM_PATH}/netif
								${PACKAGE_PLATFORM_PATH}/rtos
								${PACKAGE_PLATFORM_PATH}/wifi
								${PACKAGE_PLATFORM_PATH}/uapapi/inc
if(NOT AIC_USE_PREBUILT_HOST)
								${PACKAGE_WPA_PATH}/src
								${PACKAGE_WPA_PATH}/src/ap
								${PACKAGE_WPA_PATH}/src/common
								${PACKAGE_WPA_PATH}/src/crypto
								${PACKAGE_WPA_PATH}/src/drivers
								${PACKAGE_WPA_PATH}/src/eap_common
								${PACKAGE_WPA_PATH}/src/eap_peer
								${PACKAGE_WPA_PATH}/src/eap_server
								${PACKAGE_WPA_PATH}/src/eapol_auth
								${PACKAGE_WPA_PATH}/src/eapol_supp
								${PACKAGE_WPA_PATH}/src/l2_packet
								${PACKAGE_WPA_PATH}/src/rsn_supp
								${PACKAGE_WPA_PATH}/src/utils
								${PACKAGE_WPA_PATH}/src/tls
								${PACKAGE_WPA_PATH}/src/wps
								${PACKAGE_WPA_PATH}/wpa_supplicant
endif()
								${XROOT}/pcac/duster/inc
								${XROOT}/pcac/tr069/src
								${XROOT}/pcac/mrvxml/inc
								)

# --- host 驱动源码 (仅 AIC_USE_PREBUILT_HOST=OFF 全源码模式编译) ---
if(NOT AIC_USE_PREBUILT_HOST)
list(APPEND PACKAGE_SRC_PRIV_FILES   ${PACKAGE_DRIVER_PATH}/drv_fhost/fhost.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_cntrl.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_config.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_rx.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_tx.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_wpa.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_tcp_ack.c
								${PACKAGE_DRIVER_PATH}/drv_fhost/fhost_wpa_config.c
								${PACKAGE_DRIVER_PATH}/drv_macif/cli_cmd.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_cmds.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_main.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_msg.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_msg_rx.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_msg_tx.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_platform.c
								${PACKAGE_DRIVER_PATH}/drv_macif/rwnx_utils.c
								${PACKAGE_DRIVER_PATH}/drv_wifi/wifi_if.c
								${PACKAGE_DRIVER_PATH}/drv_wlan/wlan_if.c
								# drv_api/src/bt_api.c 跟随 host BT 驱动开关, 见下方 AIC_HOST_BT_DRIVER 块
								${PACKAGE_DRIVER_PATH}/drv_api/src/cli_api.c
								${PACKAGE_DRIVER_PATH}/drv_api/src/netif_api.c
								${PACKAGE_DRIVER_PATH}/drv_api/src/sdio_api.c
								${PACKAGE_DRIVER_PATH}/drv_api/src/sys_api.c
								${PACKAGE_DRIVER_PATH}/drv_api/src/wifi_api.c
								# 清单对齐 Makefile drv_api 通配: app_api.c 为 iperf 迁移后的空占位,
								# 无符号但必须枚举保持清单一致
								${PACKAGE_DRIVER_PATH}/drv_api/src/app_api.c
								${PACKAGE_DRIVER_PATH}/layer_common/src/hal_co.c
								${PACKAGE_DRIVER_PATH}/layer_common/src/netif_co.c
								${PACKAGE_DRIVER_PATH}/layer_common/src/sdio_co.c
								# 清单对齐 Makefile layer_common 通配: USB CO 层 (Plan B), 主体被
								# CONFIG_USB_SUPPORT 包裹, SDIO 构建下编译为空对象
								${PACKAGE_DRIVER_PATH}/layer_common/src/usb_co.c
								# 必编: 运行态型号/索引状态 (sys_chip_id/sub_id/mcu_id/is_chip_id_h/fw_match、
								# sys_mac_vif_index、sys_sta_index) —— 缺此项时整镜像链接报一堆 sys_* undefined
								# (回流源: ASR/CPSDK 集成树客户链接报错, 见 aidoc/AC5-DS5构建注意.md)。
								${PACKAGE_DRIVER_PATH}/layer_common/src/sys_state.c
								${PACKAGE_DRIVER_PATH}/layer_common/src/aic_sock.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_dlist.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_list.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_math.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_pool.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_ring.c
								${PACKAGE_DRIVER_PATH}/utils/src/co_version.c
								# 必编: aic_chip_desc_get/chipid_get/cache_set (host 各层取型号差异数据)
								${PACKAGE_DRIVER_PATH}/utils/src/aic_chip_desc.c
								)

# mbedTLS port 层 (TLS 客户端 / 证书 / 硬件熵源) —— 与 Makefile AIC_BUILD_MBEDTLS_PORT 对齐:
#   1 (默认) = 编入基准 port 层 (host/Makefile 侧 lib-sync 亦以默认 1 编入同一批 port/src/*.c);
#   0       = 平台自实现 (需提供 aic_mbedtls_client_* / 证书数组等符号, 契约见
#             host/thirdparty/mbedtls_v3_6_2/port/include/mbedtls_port_if.h)。
# 此前 CMake 清单从未列这批源: 静态库不解析符号, 引用这批 API 的平台只在整镜像链接
# 阶段报 undefined (与 e8a21cc/7f2935f 两笔 CMake 清单回流同源)。
set(AIC_BUILD_MBEDTLS_PORT 1 CACHE STRING "1=compile mbedTLS port layer; 0=platform provides it")
if(AIC_BUILD_MBEDTLS_PORT)
list(APPEND PACKAGE_SRC_PRIV_FILES
								${PACKAGE_MBEDTLS_PATH}/port/src/tls_client.c
								${PACKAGE_MBEDTLS_PATH}/port/src/tls_certificate.c
								${PACKAGE_MBEDTLS_PATH}/port/src/tls_hardware.c
								)
# 头路径只加给私有(host)目标: PACKAGE_INC_PATHS 以 BEFORE 注入, 若并入会把基准 mbedtls
# 头提到平台 SDK mbedTLS 之前, 影响平台侧 include 选择
set(PACKAGE_MBEDTLS_PORT_INC
								${PACKAGE_MBEDTLS_PATH}/include
								${PACKAGE_MBEDTLS_PATH}/library
								${PACKAGE_MBEDTLS_PATH}/port/include
								)
endif()

# host BT 驱动裁剪: 单开关 AIC_HOST_BT_DRIVER (host_config.mk, 默认 1), 与 AIC_FEATURE_BT
# 无关 —— AIC_FEATURE_BT 管 platform 层 BT 固件注册与 HCI 传输 (见下方平台 BT 块)。
# host 源码 (rwnx_main.c / rwnx_msg_*.c / wlan_if.c / lmac_msg.h) 以 AIC_HOST_BT_DRIVER
# 守卫 BT 路径, 该宏必须注入, 否则 host 库 BT 代码被静默裁掉 (此前 CMake 从未注入)。
set(AIC_HOST_BT_DRIVER 1 CACHE STRING "1=compile host BT driver (drv_bt/aic_bt_driver.c + drv_api/bt_api.c); 0=WiFi-only host lib")
if(AIC_HOST_BT_DRIVER)
list(APPEND PACKAGE_SRC_PRIV_FILES
${PACKAGE_DRIVER_PATH}/drv_bt/aic_bt_driver.c
								${PACKAGE_DRIVER_PATH}/drv_api/src/bt_api.c
								)
# 注: 平台侧无该宏引用 (已核), 故走 PACKAGE_DFLAGS, 与 Make 侧 HOST_DFLAGS 语义等价
list(APPEND PACKAGE_DFLAGS AIC_HOST_BT_DRIVER)
endif()
endif()  # NOT AIC_USE_PREBUILT_HOST (host 驱动源码)

# AIC_HCI_OVER_*: BT HCI 传输宏, 与 AIC_FEATURE_BT 解耦 — 按 AIC_HCI_TRANSPORT 恒注入
# (对齐 aic_cfg.mk L142-154)。驱动 platform 层 BT 固件注册: aic_fw_port.c DC_H 选
# hbt 变体 / userconfig.c btport=MB。
# 本段必须在 host-only 块【之外】: 与 AIC_FEATURE_BT 解耦后, 引用模式下平台层同样
# 需要该宏 (曾遗留嵌在块内 → 引用模式收不到, 同一发布两套构建选不同固件)。
# BT 传输源文件 (aic_hci.c/aic_bt_*.c) 由 AIC_FEATURE_BT 裁剪, 见下方平台 BT 块。
set(AIC_HCI_TRANSPORT "SDIO" CACHE STRING "BT HCI transport: SDIO, UART or USB")
if(AIC_HCI_TRANSPORT STREQUAL "UART")
    list(APPEND PACKAGE_DFLAGS AIC_HCI_OVER_UART)
elseif(AIC_HCI_TRANSPORT STREQUAL "USB")
    list(APPEND PACKAGE_DFLAGS AIC_HCI_OVER_USB)
else()
    list(APPEND PACKAGE_DFLAGS AIC_HCI_OVER_SDIO)
endif()

list(APPEND PACKAGE_SRC_OPEN_FILES   ${PACKAGE_PLATFORM_PATH}/fmac/aic_fw_port.c
								${PACKAGE_PLATFORM_PATH}/fmac/userconfig.c
								# 基准 HAL/日志参考实现 (aicwf_ATRESP / aicwf_log_printf / 电源 GPIO 等)。
								# Make 路径不自动编入: 平台须自行 `SRC_PLATFORM += 自己的 plat_port.c`
								# (BUILD.md「plat_port.c — HAL/硬件抽象」、release-adaptation §7.4)。
								# CMake 路径恒编基准版 —— 平台若已自带实现, 请在集成 CMake 里移除本行,
								# 否则重复符号; 未自带则此处即基准参考实现 (AT 回包走 DBG_CMN_INF)。
								${PACKAGE_PLATFORM_PATH}/hal/plat_port.c
								${PACKAGE_PLATFORM_PATH}/hal/sdio_port.c
								# 注: 本清单按 SDIO 总线固定 (DFLAGS 恒注入 CONFIG_SDIO_SUPPORT), 不含 USB
								# 端口源 platform/hal/usb_port.c (Make 路径在 AIC_BUS_TYPE=USB 时才编);
								# USB 总线平台需自行在本清单调整。
								${PACKAGE_PLATFORM_PATH}/rtos/${AIC_RTOS_PORT_SRC}
								${PACKAGE_PLATFORM_PATH}/platform_init.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_ap.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_sta.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_wps.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_p2p.c
								${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_cmd.c
								# 平台启动/demo 框架 wifi_port_plt.c 不在此列: 见下方 AIC_FEATURE_PLT 门控
								# (对齐 Makefile/aic_cfg.mk 的 AIC_FEATURE_PLT ?= 0)
								# 必编: 网络接口契约实现 (aicwf_net_if_* / aicwf_net_init / tx|rx_eth_data_process
								# / DHCP 系列)。缺此项时静态库仍能归档(不解析符号), 但整镜像链接会报一堆
								# undefined symbol —— 回流源: ASR/CPSDK 集成 (见 aidoc/AC5-DS5构建注意.md)。
								${PACKAGE_PLATFORM_PATH}/netif/netif_port.c
								# 必编: aic_chip_desc_lookup() —— host drv_wifi/wifi_if.c 的 aicwf_get_chipid()
								# 依赖它做运行时型号查表 (port/inc/aic_chip_desc.h)。
								${PACKAGE_PLATFORM_PATH}/fmac/aic_chip_desc_port.c
								# 平台专有源(如 UART/USB 端口、iperf 工具、平台自实现 mbedtls、平台 UAP/duster 桥接)
								# 默认不入清单: 由各平台按需在此追加, 或在自己的集成 CMake 里 target_sources。
								)

# 平台启动/demo 框架 (AT/CLI demo 入口 + 随 WiFi bring-up 的 nimble 拉起, 见
# platform/wifi/wifi_port_plt.c)。Make 路径由 aic_cfg.mk 的 AIC_FEATURE_PLT 门控,
# 默认 0 不编; 此处对齐同默认 —— 平台需要该参考框架时 -DAIC_FEATURE_PLT=1。
set(AIC_FEATURE_PLT 0 CACHE STRING "1=build platform/wifi/wifi_port_plt.c (demo/bring-up); 0=platform provides its own")
if(AIC_FEATURE_PLT)
list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/wifi/wifi_port_plt.c)
endif()

# BT 平台层 (HCI 拆包 aic_hci.c + 传输后端) —— 由 AIC_FEATURE_BT 裁剪 (对齐 aic_cfg.mk,
# 默认 0)。AIC_FEATURE_BT 同时被 platform/wifi/wifi_port_cmd.c、hal/usb_port.c 的 BT
# 分支守卫, 故宏与源必须成对注入; 此前两者都嵌在 host-only 块内 → 引用模式整体缺失。
# (CONFIG_BT_SUPPORT 已由 port/inc/aic_cfg.h 无条件定义, 此处保留显式注入为等价冗余)
set(AIC_FEATURE_BT 0 CACHE STRING "1=enable BT (platform BT layer + AIC_FEATURE_BT macro); 0=WiFi-only")
if(AIC_FEATURE_BT)
list(APPEND PACKAGE_DFLAGS AIC_FEATURE_BT CONFIG_BT_SUPPORT)
list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/bt/aic_hci.c)
if(AIC_HCI_TRANSPORT STREQUAL "UART")
    list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/bt/aic_bt_uart.c)
elseif(AIC_HCI_TRANSPORT STREQUAL "USB")
    list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/bt/aic_btusb.c)
else()
    list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/bt/aic_bt_sdio.c)
endif()
endif()

# mbedTLS crypto 层: 平台自实现 (USE_BUILTIN_MBEDTLS=0) 时按 Makefile 口径自动编入
# 转发示例 (AIC_MBEDTLS_FWD_EXAMPLE=0 可关)。此前 CMake 从不编入该示例源。
if(NOT USE_BUILTIN_MBEDTLS AND AIC_MBEDTLS_FWD_EXAMPLE)
list(APPEND PACKAGE_SRC_OPEN_FILES ${PACKAGE_PLATFORM_PATH}/crypto/aic_mbedtls_port_example.c)
endif()

# --- WPA Supplicant 源码 (host 部分, 仅全源码模式编译) ---
if(NOT AIC_USE_PREBUILT_HOST)
list(APPEND PACKAGE_SRC_PRIV_FILES   ${PACKAGE_WPA_PATH}/wpa_supplicant/ap.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/blacklist.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/bss.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/config_wpa.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/config_none.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/ctrl_iface.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/ctrl_iface_udp.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/eap_register.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/events.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/main_rwnx.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/notify.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/op_classes.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/scan.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/sme.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/sta_rrm.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/wmm_ac.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/wpas_glue.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/wpa_supplicant.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/wps_supplicant.c
								# 必编: CONFIG_OFFCHANNEL 默认开启 (port/inc/aic_cfg.h), events.c/
								# wpa_supplicant.c 依赖 offchannel_remain_on_channel_cb/offchannel_deinit 等
								${PACKAGE_WPA_PATH}/wpa_supplicant/offchannel.c
								${PACKAGE_WPA_PATH}/src/ap/ap_config.c
								${PACKAGE_WPA_PATH}/src/ap/acs.c
								${PACKAGE_WPA_PATH}/src/ap/ap_drv_ops.c
								${PACKAGE_WPA_PATH}/src/ap/ap_list.c
								${PACKAGE_WPA_PATH}/src/ap/ap_mlme.c
								${PACKAGE_WPA_PATH}/src/ap/authsrv.c
								${PACKAGE_WPA_PATH}/src/ap/beacon.c
								${PACKAGE_WPA_PATH}/src/ap/bss_load.c
								${PACKAGE_WPA_PATH}/src/ap/ctrl_iface_ap.c
								${PACKAGE_WPA_PATH}/src/ap/dfs.c
								${PACKAGE_WPA_PATH}/src/ap/drv_callbacks.c
								${PACKAGE_WPA_PATH}/src/ap/eap_user_db.c
								${PACKAGE_WPA_PATH}/src/ap/hostapd.c
								${PACKAGE_WPA_PATH}/src/ap/hw_features.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11_auth.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11_he.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11_ht.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11_shared.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_11_vht.c
								${PACKAGE_WPA_PATH}/src/ap/ieee802_1x.c
								${PACKAGE_WPA_PATH}/src/ap/neighbor_db.c
								${PACKAGE_WPA_PATH}/src/ap/pmksa_cache_auth.c
								${PACKAGE_WPA_PATH}/src/ap/rrm.c
								${PACKAGE_WPA_PATH}/src/ap/sta_info.c
								${PACKAGE_WPA_PATH}/src/ap/tkip_countermeasures.c
								${PACKAGE_WPA_PATH}/src/ap/utils.c
								${PACKAGE_WPA_PATH}/src/ap/wmm.c
								${PACKAGE_WPA_PATH}/src/ap/wnm_ap.c
								${PACKAGE_WPA_PATH}/src/ap/wpa_auth.c
								${PACKAGE_WPA_PATH}/src/ap/wpa_auth_glue.c
								${PACKAGE_WPA_PATH}/src/ap/wpa_auth_ie.c
								${PACKAGE_WPA_PATH}/src/ap/wps_hostapd.c
								${PACKAGE_WPA_PATH}/src/common/sae.c
								${PACKAGE_WPA_PATH}/src/common/dragonfly.c
								${PACKAGE_WPA_PATH}/src/common/hw_features_common.c
								${PACKAGE_WPA_PATH}/src/common/ieee802_11_common.c
								${PACKAGE_WPA_PATH}/src/common/wpa_common.c
								${PACKAGE_WPA_PATH}/src/common/gas.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-internal-dec.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-internal-enc.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-internal.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-cbc.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-omac1.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-unwrap.c
								${PACKAGE_WPA_PATH}/src/crypto/aes-wrap.c
								${PACKAGE_WPA_PATH}/src/crypto/crypto_mbedtls.c
								${PACKAGE_WPA_PATH}/src/crypto/crypto_internal-modexp.c
								${PACKAGE_WPA_PATH}/src/crypto/dh_groups.c
								${PACKAGE_WPA_PATH}/src/crypto/dh_group5.c
								${PACKAGE_WPA_PATH}/src/crypto/md5-internal.c
								${PACKAGE_WPA_PATH}/src/crypto/md5_wpa.c
								${PACKAGE_WPA_PATH}/src/crypto/random.c
								${PACKAGE_WPA_PATH}/src/crypto/rc4.c
								${PACKAGE_WPA_PATH}/src/crypto/sha1-internal.c
								${PACKAGE_WPA_PATH}/src/crypto/sha1-pbkdf2.c
								${PACKAGE_WPA_PATH}/src/crypto/sha1-prf.c
								${PACKAGE_WPA_PATH}/src/crypto/sha1.c
								${PACKAGE_WPA_PATH}/src/crypto/sha256-internal.c
								${PACKAGE_WPA_PATH}/src/crypto/sha256-prf.c
								${PACKAGE_WPA_PATH}/src/crypto/sha256.c
								${PACKAGE_WPA_PATH}/src/crypto/tls_none.c
								${PACKAGE_WPA_PATH}/src/drivers/drivers.c
								${PACKAGE_WPA_PATH}/src/drivers/driver_common.c
								${PACKAGE_WPA_PATH}/src/drivers/driver_rwnx.c
								${PACKAGE_WPA_PATH}/src/eap_common/chap.c
								${PACKAGE_WPA_PATH}/src/eap_common/eap_common.c
								${PACKAGE_WPA_PATH}/src/eap_common/eap_wsc_common.c
								${PACKAGE_WPA_PATH}/src/eap_peer/eap.c
								${PACKAGE_WPA_PATH}/src/eap_peer/eap_methods.c
								${PACKAGE_WPA_PATH}/src/eap_peer/eap_wsc.c
								${PACKAGE_WPA_PATH}/src/eap_server/eap_server.c
								${PACKAGE_WPA_PATH}/src/eap_server/eap_server_identity.c
								${PACKAGE_WPA_PATH}/src/eap_server/eap_server_methods.c
								${PACKAGE_WPA_PATH}/src/eap_server/eap_server_wsc.c
								${PACKAGE_WPA_PATH}/src/eapol_auth/eapol_auth_sm.c
								${PACKAGE_WPA_PATH}/src/eapol_supp/eapol_supp_sm.c
								${PACKAGE_WPA_PATH}/src/l2_packet/l2_packet_rwnx.c
								${PACKAGE_WPA_PATH}/src/rsn_supp/preauth.c
								${PACKAGE_WPA_PATH}/src/rsn_supp/wpa.c
								${PACKAGE_WPA_PATH}/src/rsn_supp/wpa_ie.c
								${PACKAGE_WPA_PATH}/src/rsn_supp/pmksa_cache.c
								${PACKAGE_WPA_PATH}/src/tls/bignum.c
								${PACKAGE_WPA_PATH}/src/utils/base64.c
								${PACKAGE_WPA_PATH}/src/utils/bitfield.c
								${PACKAGE_WPA_PATH}/src/utils/common.c
								${PACKAGE_WPA_PATH}/src/utils/eloop.c
								${PACKAGE_WPA_PATH}/src/utils/ip_addr.c
								${PACKAGE_WPA_PATH}/src/utils/os_rwnx.c
								${PACKAGE_WPA_PATH}/src/utils/uuid.c
								${PACKAGE_WPA_PATH}/src/utils/wpabuf.c
								${PACKAGE_WPA_PATH}/src/utils/wpa_debug.c
								${PACKAGE_WPA_PATH}/src/wps/wps.c
								${PACKAGE_WPA_PATH}/src/wps/wps_attr_build.c
								${PACKAGE_WPA_PATH}/src/wps/wps_attr_parse.c
								${PACKAGE_WPA_PATH}/src/wps/wps_attr_process.c
								${PACKAGE_WPA_PATH}/src/wps/wps_common.c
								${PACKAGE_WPA_PATH}/src/wps/wps_dev_attr.c
								${PACKAGE_WPA_PATH}/src/wps/wps_enrollee.c
								${PACKAGE_WPA_PATH}/src/wps/wps_registrar.c
								)

# P2P 开关: 与 Makefile/aic_cfg.mk 的 AIC_FEATURE_P2P ?= 1 对齐 (默认开)。
# 宏必须【无条件】注入 (含引用模式): platform 层 netif_port.c / wifi_port.c /
# wifi_port_cmd.c 用 CONFIG_P2P_SUPPORT 守卫 P2P 分支, 而预编译 host 库内 P2P 代码
# 按 Make 默认是编入的 —— 此前宏只加在全源码块内, 引用模式下两侧能力分叉。
# 注意: set 必须先于下方所有 AIC_FEATURE_P2P 引用 —— CMake 顺序执行, 未定义变量
# 在 if() 中恒为 false (曾因定义晚于引用, 全新 configure 首次求值时 P2P 源被静默
# 丢弃而宏照常注入, 同一目录二次 configure 因 CMakeCache 已存 =1 而恢复)。
set(AIC_FEATURE_P2P 1 CACHE STRING "1=enable P2P (macro always; wpa p2p sources in full-source mode); 0=disable")
if(AIC_FEATURE_P2P)
	list(APPEND PACKAGE_DFLAGS AIC_FEATURE_P2P CONFIG_P2P_SUPPORT)
endif()

if(NOT AIC_USE_PREBUILT_HOST AND AIC_FEATURE_P2P)
# 注: P2P 宏注入见上方开关块 —— 引用模式下 platform 层同样需要该宏,
# 留在本块内会导致「预编译 host 库含 P2P 代码、platform 层 P2P 分支被裁」的能力分叉
list(APPEND PACKAGE_SRC_PRIV_FILES
								${PACKAGE_WPA_PATH}/wpa_supplicant/p2p_supplicant.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/p2p_supplicant_sd.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_utils.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_parse.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_build.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_go_neg.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_sd.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_pd.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_invitation.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_dev_disc.c
								${PACKAGE_WPA_PATH}/src/p2p/p2p_group.c
								${PACKAGE_WPA_PATH}/src/ap/p2p_hostapd.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/gas_query.c
								${PACKAGE_WPA_PATH}/wpa_supplicant/wifi_display.c
								)
# 注: offchannel.c 已在上方无条件段列入 (对齐 Makefile: offchannel 必须无条件编译,
# CONFIG_OFFCHANNEL 恒定义), 此处不重复列出
endif()
endif()  # NOT AIC_USE_PREBUILT_HOST (WPA 源码)

# local group build flags for source files
# contained in this group directory
set(AIC_CHIP_MODEL "D80" CACHE STRING "Chip model: D80, D80N, D80N_U02C, DW, DC, DL, DC_H, DW_H, DL_H")
if(AIC_CHIP_MODEL STREQUAL "D80N")
    list(APPEND PACKAGE_DFLAGS CONFIG_AIC8800D80N USE_5G)
elseif(AIC_CHIP_MODEL STREQUAL "D80N_U02C")
    list(APPEND PACKAGE_DFLAGS CONFIG_AIC8800D80N CONFIG_AIC8800D80N_U02C USE_5G)
else()
    list(APPEND PACKAGE_DFLAGS #CONFIG_AIC8800DW
                                CONFIG_AIC8800D80
                                USE_5G)
endif()

list(APPEND PACKAGE_DFLAGS
							CONFIG_SDIO_SUPPORT
#### firmware bin file
							CONFIG_DOWNLOAD_FW
							CONFIG_LOAD_FW=1
#### wifi setting
							CONFIG_RWNX_FULLMAC
#### hardware settings
							CONFIG_PMIC_SETTING
							CONFIG_LOAD_USERCONFIG
							CONFIG_DPD
							CONFIG_FORCE_DPD_CALIB
							CONFIG_SDIO_BUS_PWRCTRL
							CONFIG_SDIO_BUS_PWRCTRL_DYN
#### software settings
							PLATFORM_ASR_THREADX
							CONFIG_LWIP
							CONFIG_REORD_FORWARD_LIST
							CONFIG_RX_NOCOPY
							CONFIG_MBEDTLS
							SDIO_FC_NARB
							#CONFIG_DRIVER_EDCA
							CONFIG_SOCK_SENDTO
							CONFIG_RTOS_TRACE
							#CONFIG_FILTER_TCP_ACK
							CONFIG_SOCK_RTOS_AL
							CONFIG_MBEDTLS_V3
#### debug log
							AIC_LOG_DEBUG_ON
							CONFIG_DRIVER_ORM
							WPA_CTRL_IFACE_PORT=9877
							)

# 组合对象库源: 引用模式 PRIV 为空, 只含 platform 开源文件;
# 全源码模式 PRIV (host) + OPEN (platform) 一起编译
set(_AIC_OBJ_SRC ${PACKAGE_SRC_OPEN_FILES})
if(PACKAGE_SRC_PRIV_FILES)
    list(INSERT _AIC_OBJ_SRC 0 ${PACKAGE_SRC_PRIV_FILES})
endif()
add_library(${OBJ_PACKAGE_NAME} OBJECT ${_AIC_OBJ_SRC})
add_library(${PACKAGE_NAME} $<TARGET_OBJECTS:${OBJ_PACKAGE_NAME}>)
# host 源码 (fhost_*.c) 需要驱动内部 utils/inc (co_endian.h 等): 基准 Makefile 流程
# 由 host/Makefile 的 -I 提供, CMake 流程此前遗漏该目录 → #5 cannot open co_endian.h。
target_include_directories(${OBJ_PACKAGE_NAME} PRIVATE ${PACKAGE_DRIVER_PATH}/utils/inc)
# 引用模式 (AIC_BUILD_HOST=0 语义): 链接 host 预编译库 (lib/ 由 make 体系产出)
if(AIC_USE_PREBUILT_HOST)
    # 单库统一: 所有型号共用 libaic8800_driver.a (host 无型号宏, 运行时 desc 分发)
    target_link_libraries(${PACKAGE_NAME} PUBLIC
        ${PACKAGE_PATH}/lib/libaic8800_driver.a)
    # mbedTLS crypto 库: USE_BUILTIN_MBEDTLS=0 时由平台自实现 aic_mbedtls_*
    # 符号 (转发示例 platform/crypto/aic_mbedtls_port_example.c; 变量声明见文件头), 不链接内置库
    if(USE_BUILTIN_MBEDTLS)
        target_link_libraries(${PACKAGE_NAME} PUBLIC
            ${PACKAGE_PATH}/lib/libaic_mbedtls.a)
    endif()
endif()
# NimBLE 开关 (默认 OFF): option 须在此声明 (早于 PACKAGE_DFLAGS 消费点),
# 使 AIC_FEATURE_NIMBLE=ON 时 platform 层获得 -DAIC_FEATURE_NIMBLE,
# 激活 wifi_port_plt.c 的 nimble_stack_start() 门控
option(AIC_FEATURE_NIMBLE "Build the NimBLE host stack (STACK/nimble)" OFF)
if(AIC_FEATURE_NIMBLE)
    list(APPEND PACKAGE_DFLAGS AIC_FEATURE_NIMBLE)
endif()
if(PACKAGE_DFLAGS)
	target_compile_definitions(${OBJ_PACKAGE_NAME} PRIVATE ${PACKAGE_DFLAGS})
endif()
target_compile_options(${OBJ_PACKAGE_NAME} PRIVATE --diag_suppress=warning)
target_include_directories(${OBJ_PACKAGE_NAME} BEFORE PRIVATE ${PACKAGE_INC_PATHS})

creat_prebuild_file_pack(${PACKAGE_NAME} ${OBJ_PACKAGE_NAME})

# 私有包 (host 闭源对象库) 仅在全源码模式创建; 引用模式下 host 以 lib/*.a 预编译库形式提供
if(NOT AIC_USE_PREBUILT_HOST)
	add_library(${PRIV_OBJ_PACKAGE_NAME} OBJECT ${PACKAGE_SRC_PRIV_FILES})
	add_library(${PRIV_PACKAGE_NAME} $<TARGET_OBJECTS:${PRIV_OBJ_PACKAGE_NAME}>)
	if(PACKAGE_DFLAGS)
		target_compile_definitions(${PRIV_OBJ_PACKAGE_NAME} PRIVATE ${PACKAGE_DFLAGS})
	endif()
	target_compile_options(${PRIV_OBJ_PACKAGE_NAME} PRIVATE --diag_suppress=warning)
	target_include_directories(${PRIV_OBJ_PACKAGE_NAME} BEFORE PRIVATE ${PACKAGE_INC_PATHS})
	# mbedTLS port 层头 (仅 AIC_BUILD_MBEDTLS_PORT=1 且全源码模式时存在)
	if(PACKAGE_MBEDTLS_PORT_INC)
		target_include_directories(${PRIV_OBJ_PACKAGE_NAME} PRIVATE ${PACKAGE_MBEDTLS_PORT_INC})
	endif()

	creat_prebuild_file_pack(${PRIV_PACKAGE_NAME} ${PRIV_OBJ_PACKAGE_NAME})
endif()

# ============================================================================
# release: 生成自包含发布包 (与 Makefile release 目标等价)
# 输出到与驱动源码同级的 <驱动目录名>_release/ (与 Makefile RELEASE_PATH 一致)
# ============================================================================
get_filename_component(DRIVER_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
get_filename_component(DRIVER_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(RELEASE_DIR ${DRIVER_PARENT_DIR}/${DRIVER_DIR_NAME}_release)

# 引用模式 (AIC_USE_PREBUILT_HOST=ON): host 库不在 ${PACKAGE_NAME} 内, release 包
# 需额外携带 host 预编译库 (lib/ 由 make 体系产出); 全源码模式 host 对象已并入
# ${PACKAGE_NAME}, 无需再拷
set(_REL_HOST_LIBS "")
if(AIC_USE_PREBUILT_HOST)
    set(_REL_HOST_LIBS
        COMMAND ${CMAKE_COMMAND} -E copy
                ${PACKAGE_PATH}/lib/libaic8800_driver.a ${RELEASE_DIR}/lib/
    )
    if(USE_BUILTIN_MBEDTLS)
        list(APPEND _REL_HOST_LIBS
            COMMAND ${CMAKE_COMMAND} -E copy
                    ${PACKAGE_PATH}/lib/libaic_mbedtls.a ${RELEASE_DIR}/lib/
        )
    endif()
endif()

# NimBLE 协议栈源码: 仅 AIC_FEATURE_NIMBLE=ON 时随 release 包携带 (默认 OFF 不打包)
set(_REL_NIMBLE "")
if(AIC_FEATURE_NIMBLE)
    set(_REL_NIMBLE
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${PACKAGE_PATH}/STACK/nimble ${RELEASE_DIR}/STACK/nimble
    )
endif()

add_custom_target(release
	# 创建目录
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/lib
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/lib/include
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/include
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/platform
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/port
	COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}/docs
	# 拷贝驱动库
	COMMAND ${CMAKE_COMMAND} -E copy
		$<TARGET_FILE:${PACKAGE_NAME}> ${RELEASE_DIR}/lib/
	# 引用模式: 额外携带 host 预编译库 (libaic8800_driver.a + mbedtls)
	${_REL_HOST_LIBS}
	# NimBLE 协议栈源码 (仅 AIC_FEATURE_NIMBLE=ON 时非空)
	${_REL_NIMBLE}
	# 拷贝头文件 (include/ 兼容 + lib/include/ 自包含, 与 Makefile release 一致)
	COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/host/scripts/copy_release_headers.py
		${PACKAGE_DRIVER_PATH}
		${PACKAGE_PATH}/port
		${RELEASE_DIR}/include
	COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/host/scripts/copy_release_headers.py
		${PACKAGE_DRIVER_PATH}
		${PACKAGE_PATH}/port
		${RELEASE_DIR}/lib/include
	# 拷贝平台源码
	COMMAND ${CMAKE_COMMAND} -E copy_directory
		${PACKAGE_PLATFORM_PATH} ${RELEASE_DIR}/platform
	# 拷贝 port 头文件
	COMMAND ${CMAKE_COMMAND} -E copy_directory
		${PACKAGE_PATH}/port/inc ${RELEASE_DIR}/port/inc
	# 拷贝构建配置
	COMMAND ${CMAKE_COMMAND} -E copy
		${PACKAGE_PATH}/Makefile ${PACKAGE_PATH}/aic_cfg.mk
		${PACKAGE_PATH}/platform_config.mk ${PACKAGE_PATH}/BUILD.md
		${PACKAGE_PATH}/CMakeLists.txt
		${RELEASE_DIR}/
	# 拷贝文档 (排除 aic8800-porting skill 目录)
	COMMAND ${CMAKE_COMMAND} -E copy_directory
		${PACKAGE_PATH}/docs ${RELEASE_DIR}/docs
	COMMAND ${CMAKE_COMMAND} -E remove_directory
		${RELEASE_DIR}/docs/aic8800-porting
	COMMENT "=== Generating release package at ${RELEASE_DIR} ==="
	VERBATIM
)

# ============================================================================
# NimBLE host 协议栈 (STACK/nimble) — BT HCI over SDIO/UART/USB
# 单独 target 编译: tinycrypt 的 utils.c 与 wpa ap/utils.c 同名, 不能混编
# 启用: -DAIC_FEATURE_NIMBLE=ON, 同时需要 AIC_FEATURE_BT=ON
# option(AIC_FEATURE_NIMBLE) 已在 PACKAGE_DFLAGS 消费点前声明 (默认 OFF)
# ============================================================================
if(AIC_FEATURE_NIMBLE)
    add_subdirectory(${PACKAGE_PATH}/STACK/nimble ${CMAKE_CURRENT_BINARY_DIR}/nimble)
endif()
