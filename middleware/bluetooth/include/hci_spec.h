/**
  ******************************************************************************
  * @file   hci_spec.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _HCI_SPEC_H_
#define _HCI_SPEC_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef  CFG_BT_VER_20
#define CFG_BT_VER_12
#endif

#ifdef  CFG_BT_VER_21
#define CFG_BT_VER_12
#define CFG_BT_VER_20
#define CFG_BT_VER_21
#endif


#define HCI_VAR_ARG_POOL_SIZE   248

#define HCI_REVISION                   ((U16)0x00)  /* HCI revision in hardware */
#define HCI_OGF_BIT_OFFSET             ((U8)10)     /* num of bit shiftp_srv */

#define HCI_LINK                       ((U16)0x0400)
#define HCI_POLICY                     ((U16)0x0800)
#define HCI_HOST_BB                    ((U16)0x0C00)
#define HCI_INFO                       ((U16)0x1000)
#define HCI_STS                        ((U16)0x1400)
#define HCI_TEST                       ((U16)0x1800)

#define HCI_MAX_OGF                    ((U16)0x1C00)

#define HCI_MANUFACTURER_EXTENSION     ((U16)0xFC00)
#define HCI_OPP_CLTODE_GROUP_MASK      ((U16)0xFC00)
#define HCI_OPP_CLTODE_MASK            ((U16)0x03FF)
#define HCI_LINK_RET                   HCI_LINK
#define HCI_POLICY_RET                 HCI_POLICY
#define HCI_HOST_BB_RET                HCI_HOST_BB
#define HCI_INFO_RET                   HCI_INFO
#define HCI_STS_RET                    HCI_STS
#define HCI_TEST_RET                   HCI_TEST
#define HCI_NOP                        (U16)0x0000
#define HCI_INQUIRY                    ((U16)HCI_LINK | 0x0001)
#define HCI_INQUIRY_ESC                ((U16)HCI_LINK | 0x0002)
#define HCI_PERIODIC_INQUIRY_MODE      ((U16)HCI_LINK | 0x0003)
#define HCI_EXIT_PERIODIC_INQUIRY_MODE ((U16)HCI_LINK | 0x0004)
#define HCI_CREATE_CONN                ((U16)HCI_LINK | 0x0005)
#define HCI_DISC                       ((U16)HCI_LINK | 0x0006)
#define HCI_ADD_SCO_CONN               ((U16)HCI_LINK | 0x0007)
#define HCI_CREATE_CONN_ESC            ((U16)HCI_LINK | 0x0008)
#define HCI_ACPT_CONN_REQ              ((U16)HCI_LINK | 0x0009)
#define HCI_REJ_CONN_REQ               ((U16)HCI_LINK | 0x000A)
#define HCI_LINK_KEY_REQ_REPLY         ((U16)HCI_LINK | 0x000B)
#define HCI_LINK_KEY_REQ_NEG_REPLY     ((U16)HCI_LINK | 0x000C)
#define HCI_PIN_CODE_REQ_REPLY         ((U16)HCI_LINK | 0x000D)
#define HCI_PIN_CODE_REQ_NEG_REPLY     ((U16)HCI_LINK | 0x000E)
#define HCI_CHANGE_CONN_PKT_TYPE       ((U16)HCI_LINK | 0x000F)
#define HCI_AUTH_REQ                   ((U16)HCI_LINK | 0x0011)
#define HCI_SET_CONN_ENCRYPTION        ((U16)HCI_LINK | 0x0013)
#define HCI_CHANGE_CONN_LINK_KEY       ((U16)HCI_LINK | 0x0015)
#define HCI_MASTER_LINK_KEY            ((U16)HCI_LINK | 0x0017)
#define HCI_RMT_NAME_REQ               ((U16)HCI_LINK | 0x0019)
#define HCI_RMT_NAME_REQ_ESC           ((U16)HCI_LINK | 0x001A)
#define HCI_RD_RMT_SUPP_FEATR          ((U16)HCI_LINK | 0x001B)
#define HCI_RD_RMT_EXT_FEATR           ((U16)HCI_LINK | 0x001C)
#define HCI_RD_RMT_VER_INFO            ((U16)HCI_LINK | 0x001D)
#define HCI_RD_CLOCK_OFFSET            ((U16)HCI_LINK | 0x001F)

#ifdef CFG_BT_VER_21
#define HCI_MAX_LINK_OCF_V1_1  ((U16)0x0035)
#else
#define HCI_MAX_LINK_OCF_V1_1  ((U16)0x0020)
#endif

/*1.2 featr */
#define HCI_RD_LMP_HANDLE                 ((U16)HCI_LINK | 0x0020)
#define HCI_EXCHANGE_FIXED_INFO           ((U16)HCI_LINK | 0x0021)
#define HCI_EXCHANGE_ALIAS_INFO           ((U16)HCI_LINK | 0x0022)
#define HCI_PRIVATE_PAIRING_REQ_REPLY     ((U16)HCI_LINK | 0x0023)
#define HCI_PRIVATE_PAIRING_REQ_NEG_REPLY ((U16)HCI_LINK | 0x0024)
#define HCI_GENERATED_ALIAS               ((U16)HCI_LINK | 0x0025)
#define HCI_ALIAS_ADDR_REQ_REPLY          ((U16)HCI_LINK | 0x0026)
#define HCI_ALIAS_ADDR_REQ_NEG_REPLY      ((U16)HCI_LINK | 0x0027)
#define HCI_SETUP_SYNCHRONOUS_CONN        ((U16)HCI_LINK | 0x0028)
#define HCI_ACPT_SYNCHRONOUS_CONN_REQ     ((U16)HCI_LINK | 0x0029)
#define HCI_REJ_SYNCHRONOUS_CONN_REQ      ((U16)HCI_LINK | 0x002A)

#ifdef CFG_BT_VER_21
#define HCI_IO_CAPABILITY_REQ_REPLY     ((U16)HCI_LINK | 0x002B)
#define HCI_USR_CNFIRM_REQ_REPLY        ((U16)HCI_LINK | 0x002C)
#define HCI_USR_CNFIRAM_REQ_NEG_REPLY   ((U16)HCI_LINK | 0x002D)
#define HCI_USR_PASSKEY_REQ_REPLY       ((U16)HCI_LINK | 0x002E)
#define HCI_USR_PASSKEY_NEG_REQ_REPLY   ((U16)HCI_LINK | 0x002F)
#define HCI_RMT_OOB_DATA_REQ_REPLY      ((U16)HCI_LINK | 0x0030)
#define HCI_RMT_OOB_DATA_REQ_NEG_REPLY  ((U16)HCI_LINK | 0x0033)
#define HCI_IO_CAPABILITY_REQ_NEG_REPLY ((U16)HCI_LINK | 0x0034)
#endif
#define HCI_ENH_SETUP_SYNCHRONOUS_CONN    ((U16)HCI_LINK | 0x003D)
#define HCI_ENH_ACPT_SYNCHRONOUS_CONN_REQ ((U16)HCI_LINK | 0x003E)

#ifdef CFG_BT_VER_21
#define HCI_MAX_LINK_OCF ((U16)0x003F)
#else
#define HCI_MAX_LINK_OCF ((U16)0x002B)
#endif

#define HCI_HOLD_MODE                        ((U16)HCI_POLICY | 0x0001)
#define HCI_SNIFF_MODE                       ((U16)HCI_POLICY | 0x0003)
#define HCI_EXIT_SNIFF_MODE                  ((U16)HCI_POLICY | 0x0004)
#define HCI_PARK_MODE                        ((U16)HCI_POLICY | 0x0005)
#define HCI_EXIT_PARK_MODE                   ((U16)HCI_POLICY | 0x0006)
#define HCI_QOS_SETUP                        ((U16)HCI_POLICY | 0x0007)
#define HCI_ROLE_DISCOV                      ((U16)HCI_POLICY | 0x0009)
#define HCI_SWITCH_ROLE                      ((U16)HCI_POLICY | 0x000B)
#define HCI_RD_LINK_POLICY_SETTINGS          ((U16)HCI_POLICY | 0x000C)
#define HCI_WR_LINK_POLICY_SETTINGS          ((U16)HCI_POLICY | 0x000D)
#define HCI_MAX_POLICY_OCF_V1_1              ((U16)0x000E)
#define HCI_RD_DEFAULT_LINK_POLICY_SETTINGS  ((U16)HCI_POLICY | 0x000E)
#define HCI_WR_DEFAULT_LINK_POLICY_SETTINGS  ((U16)HCI_POLICY | 0x000F)
#define HCI_FLOW_SPEC                        ((U16)HCI_POLICY | 0x0010)
#define HCI_SNIFF_SUB_RATE                   ((U16)HCI_POLICY | 0x0011)
#define HCI_MAX_POLICY_OCF                   ((U16)0x0013)
#define HCI_SET_EV_MASK                      ((U16)HCI_HOST_BB | 0x0001)
#define HCI_RESET                            ((U16)HCI_HOST_BB | 0x0003)
#define HCI_SET_EV_FILTER                    ((U16)HCI_HOST_BB | 0x0005)
#define HCI_FLUSH                            ((U16)HCI_HOST_BB | 0x0008)
#define HCI_RD_PIN_TYPE                      ((U16)HCI_HOST_BB | 0x0009)
#define HCI_WR_PIN_TYPE                      ((U16)HCI_HOST_BB | 0x000A)
#define HCI_CREATE_NEW_UNIT_KEY              ((U16)HCI_HOST_BB | 0x000B)
#define HCI_RD_STORED_LINK_KEY               ((U16)HCI_HOST_BB | 0x000D)
#define HCI_WR_STORED_LINK_KEY               ((U16)HCI_HOST_BB | 0x0011)
#define HCI_DEL_STORED_LINK_KEY              ((U16)HCI_HOST_BB | 0x0012)
#define HCI_CHANGE_LOCAL_NAME                ((U16)HCI_HOST_BB | 0x0013)
#define HCI_RD_LOCAL_NAME                    ((U16)HCI_HOST_BB | 0x0014)
#define HCI_RD_CONN_ACPT_TIMEOUT             ((U16)HCI_HOST_BB | 0x0015)
#define HCI_WR_CONN_ACPT_TIMEOUT             ((U16)HCI_HOST_BB | 0x0016)
#define HCI_RD_PAGE_TIMEOUT                  ((U16)HCI_HOST_BB | 0x0017)
#define HCI_WR_PAGE_TIMEOUT                  ((U16)HCI_HOST_BB | 0x0018)
#define HCI_RD_SCAN_ENB                      ((U16)HCI_HOST_BB | 0x0019)
#define HCI_WR_SCAN_ENB                      ((U16)HCI_HOST_BB | 0x001A)
#define HCI_RD_PAGESCAN_ACTIVITY             ((U16)HCI_HOST_BB | 0x001B)
#define HCI_WR_PAGESCAN_ACTIVITY             ((U16)HCI_HOST_BB | 0x001C)
#define HCI_RD_INQUIRYSCAN_ACTIVITY          ((U16)HCI_HOST_BB | 0x001D)
#define HCI_WR_INQUIRYSCAN_ACTIVITY          ((U16)HCI_HOST_BB | 0x001E)
#define HCI_RD_AUTH_ENB                      ((U16)HCI_HOST_BB | 0x001F)
#define HCI_WR_AUTH_ENB                      ((U16)HCI_HOST_BB | 0x0020)
#define HCI_RD_ENC_MODE                      ((U16)HCI_HOST_BB | 0x0021)
#define HCI_WR_ENC_MODE                      ((U16)HCI_HOST_BB | 0x0022)
#define HCI_RD_DEV_CLS                       ((U16)HCI_HOST_BB | 0x0023)
#define HCI_WR_DEV_CLS                       ((U16)HCI_HOST_BB | 0x0024)
#define HCI_RD_VOICE_SETTING                 ((U16)HCI_HOST_BB | 0x0025)
#define HCI_WR_VOICE_SETTING                 ((U16)HCI_HOST_BB | 0x0026)
#define HCI_RD_AUTO_FLUSH_TIMEOUT            ((U16)HCI_HOST_BB | 0x0027)
#define HCI_WR_AUTO_FLUSH_TIMEOUT            ((U16)HCI_HOST_BB | 0x0028)
#define HCI_RD_NUM_BCAST_RETXS               ((U16)HCI_HOST_BB | 0x0029)
#define HCI_WR_NUM_BCAST_RETXS               ((U16)HCI_HOST_BB | 0x002A)
#define HCI_RD_HOLD_MODE_ACTIVITY            ((U16)HCI_HOST_BB | 0x002B)
#define HCI_WR_HOLD_MODE_ACTIVITY            ((U16)HCI_HOST_BB | 0x002C)
#define HCI_RD_TX_POWER_LEVEL                ((U16)HCI_HOST_BB | 0x002D)
#define HCI_RD_SCO_FLOW_CON_ENB              ((U16)HCI_HOST_BB | 0x002E)
#define HCI_WR_SCO_FLOW_CON_ENB              ((U16)HCI_HOST_BB | 0x002F)
#define HCI_SET_HCTOHOST_FLOW_CTRL           ((U16)HCI_HOST_BB | 0x0031)
#define HCI_HOST_BUFF_SIZE                   ((U16)HCI_HOST_BB | 0x0033)
#define HCI_HOST_NUM_COMPD_PKTS              ((U16)HCI_HOST_BB | 0x0035)
#define HCI_RD_LINK_SUPERV_TIMEOUT           ((U16)HCI_HOST_BB | 0x0036)
#define HCI_WR_LINK_SUPERV_TIMEOUT           ((U16)HCI_HOST_BB | 0x0037)
#define HCI_RD_NUM_SUPP_IAC                  ((U16)HCI_HOST_BB | 0x0038)
#define HCI_RD_CUR_IAC_LAP                   ((U16)HCI_HOST_BB | 0x0039)
#define HCI_WR_CUR_IAC_LAP                   ((U16)HCI_HOST_BB | 0x003A)
#define HCI_RD_PAGESCAN_PERIOD_MODE          ((U16)HCI_HOST_BB | 0x003B)
#define HCI_WR_PAGESCAN_PERIOD_MODE          ((U16)HCI_HOST_BB | 0x003C)
#define HCI_RD_PAGESCAN_MODE                 ((U16)HCI_HOST_BB | 0x003D)
#define HCI_WR_PAGESCAN_MODE                 ((U16)HCI_HOST_BB | 0x003E)
#define HCI_MAX_HOST_BB_OCF_V1_1             ((U16)0x003f)
#define HCI_SET_AFH_CHNL_CLS                 ((U16)HCI_HOST_BB | 0x003F)
#define HCI_RD_INQUIRY_SCAN_TYPE             ((U16)HCI_HOST_BB | 0x0042)
#define HCI_WR_INQUIRY_SCAN_TYPE             ((U16)HCI_HOST_BB | 0x0043)
#define HCI_RD_INQUIRY_MODE                  ((U16)HCI_HOST_BB | 0x0044)
#define HCI_WR_INQUIRY_MODE                  ((U16)HCI_HOST_BB | 0x0045)
#define HCI_RD_PAGE_SCAN_TYPE                ((U16)HCI_HOST_BB | 0x0046)
#define HCI_WR_PAGE_SCAN_TYPE                ((U16)HCI_HOST_BB | 0x0047)
#define HCI_RD_AFH_CHNL_CLS_M                ((U16)HCI_HOST_BB | 0x0048)
#define HCI_WR_AFH_CHNL_CLS_M                ((U16)HCI_HOST_BB | 0x0049)
#define HCI_RD_ANON_MODE                     ((U16)HCI_HOST_BB | 0x004A)
#define HCI_WR_ANON_MODE                     ((U16)HCI_HOST_BB | 0x004B)
#define HCI_RD_ALIAS_AUTH_ENB                ((U16)HCI_HOST_BB | 0x004C)
#define HCI_WR_ALIAS_AUTH_ENB                ((U16)HCI_HOST_BB | 0x004D)
#define HCI_RD_ANON_ADDR_CHANGE_PARAMS       ((U16)HCI_HOST_BB | 0x004E)
#define HCI_WR_ANON_ADDR_CHANGE_PARAMS       ((U16)HCI_HOST_BB | 0x004F)
#ifdef CFG_BT_VER_21
#define HCI_RD_EXT_INQUIRE_RESP              ((U16)HCI_HOST_BB | 0x0051)
#define HCI_WR_EXT_INQUIRE_RESP              ((U16)HCI_HOST_BB | 0x0052)
#define HCI_RD_SMP_PAIR_MODE                 ((U16)HCI_HOST_BB | 0x0055)
#define HCI_WR_SMP_PAIR_MODE                 ((U16)HCI_HOST_BB | 0x0056)
#define HCI_RD_LOCAL_OOB_DATA                ((U16)HCI_HOST_BB | 0x0057)
#define HCI_KEYPRESS_NOTIFI                  ((U16)HCI_HOST_BB | 0x0060)
#endif
#define HCI_SET_EV_MASK2                     ((U16)HCI_HOST_BB | 0x0063)
#define HCI_WR_LE_HOST_SUPPORT               ((U16)HCI_HOST_BB | 0x006D)


#define HCI_RESET_FIXED_ADDR_ATTMPTS_COUNTER ((U16)HCI_HOST_BB | 0x0050)

#ifdef CFG_BT_VER_21
#define HCI_MAX_HOST_BB_OCF            ((U16)0x0061)
#else
#define HCI_MAX_HOST_BB_OCF            ((U16)0x0051)
#endif
#define HCI_RD_LOCAL_VER_INFO          ((U16)HCI_INFO | 0x0001)
#define HCI_RD_LOCAL_SUPP_CMDS         ((U16)HCI_INFO | 0x0002)
#define HCI_RD_LOCAL_SUPP_FEATR        ((U16)HCI_INFO | 0x0003)
#define HCI_RD_LOCAL_EXT_FEATR         ((U16)HCI_INFO | 0x0004)
#define HCI_RD_BUFF_SIZE               ((U16)HCI_INFO | 0x0005)
#define HCI_RD_COUNTRY_CODE            ((U16)HCI_INFO | 0x0007)
#define HCI_RD_BD_ADDR                 ((U16)HCI_INFO | 0x0009)
#define HCI_MAX_INFO_OCF_V1_1          ((U16)0x000A)
#define HCI_MAX_INFO_OCF               ((U16)0x000A)
#define HCI_RD_FAILED_CONTACT_COUNT    ((U16)HCI_STS | 0x0001)
#define HCI_RESET_FAILED_CONTACT_COUNT ((U16)HCI_STS | 0x0002)
#define HCI_GET_LINK_QA                ((U16)HCI_STS | 0x0003)
#define HCI_RD_RSSI                    ((U16)HCI_STS | 0x0005)
#define HCI_MAX_STS_OCF_V1_1           ((U16)0x0006)
#define HCI_RD_AFH_CHNL_MAP            ((U16)HCI_STS | 0x0006)
#define HCI_RD_CLOCK                   ((U16)HCI_STS | 0x0007)
#define HCI_MAX_STS_OCF                ((U16)0x0008)
#define HCI_RD_LOOPBACK_MODE           ((U16)HCI_TEST | 0x0001)
#define HCI_WR_LOOPBACK_MODE           ((U16)HCI_TEST | 0x0002)
#define HCI_ENB_DUT_MODE               ((U16)HCI_TEST | 0x0003)
#define HCI_MAX_TEST_OCF_V1_1          ((U16)0x0004)
#define HCI_MAX_TEST_OCF               ((U16)0x0004)
/* evs from host ctrller to host */
#define HCI_DBG_STATS                  ((U16)0x0001)
#define HCI_DBG_STATS_ACL              ((U16)0x0002)
#define HCI_DBG_STATS_SCO              ((U16)0x0003)
#define HCI_DBG_MEM                    ((U16)0x0004)
#define HCI_DBG_RAND                   ((U16)0x0005)
#define HCI_DBG_KEY                    ((U16)0x0006)
#define HCI_DBG_SRES                   ((U16)0x0007)
#define HCI_DBG_LMP_TEST_ENB           ((U16)0x0008)
#define HCI_DBG_LMP_TEST_CTRL          ((U16)0x0009)
#define HCI_DBG_STATS_PARK             ((U16)0x000a)
#define HCI_DBG_STATS_ESCO             ((U16)0x000b)
#define HCI_DBG_SCATTER_REQ            ((U16)0x000c)
#define HCI_DBG_UNSCATTER_REQ          ((U16)0x000d)
#define HCI_DBG_SET_SUBRATE            ((U16)0x000e)
#define HCI_DBG_PP_REQ                 ((U16)0x000f)

#define HCI_DBG_MASTER_JITTER            ((U16)0x000a)
#define HCI_DBG_GENERAL                  ((U16)0x0013)
#define HCI_DBG_TX_LMP                   ((U16)0x0014)
#define HCI_DBG_STOP_LM                  ((U16)0x0015)
#define HCI_DBG_START_LM                 ((U16)0x0016)
#define HCI_DBG_PS_WR                    ((U16)0x0020)
#define HCI_DBG_SET_BTCLOCK              ((U16)0x0040)
#define HCI_DBG_BLOCK_RX                 ((U16)0x0041)
#define HCI_DBG_SET_ABSENCE              ((U16)0x0042)
#define HCI_DBG_CLEAR_ABSENCE            ((U16)0x0043)
#define HCI_DBG_ESCO_PING                ((U16)0x0044)
#define HCI_EV_INQUIRY_COMP              ((U8)0x01)
#define HCI_EV_INQUIRY_RES               ((U8)0x02)
#define HCI_EV_CONN_COMP                 ((U8)0x03)
#define HCI_EV_CONN_REQ                  ((U8)0x04)
#define HCI_EV_DISC_COMP                 ((U8)0x05)
#define HCI_EV_AUTH_COMP                 ((U8)0x06)
#define HCI_EV_RMT_NAME_REQ_COMP         ((U8)0x07)
#define HCI_EV_ENCRYPTION_CHANGE         ((U8)0x08)
#define HCI_EV_CHANGE_CONN_LINK_KEY_COMP ((U8)0x09)
#define HCI_EV_MASTER_LINK_KEY_COMP      ((U8)0x0A)
#define HCI_EV_RD_REM_SUPP_FEATR_COMP    ((U8)0x0B)
#define HCI_EV_RD_RMT_VER_INFO_COMP      ((U8)0x0C)
#define HCI_EV_QOS_SETUP_COMP            ((U8)0x0D)
#define HCI_EV_CMD_COMP                  ((U8)0x0E)
#define HCI_EV_CMD_STS                   ((U8)0x0F)
#define HCI_EV_HARDWARE_ERR              ((U8)0x10)
#define HCI_EV_FLUSH_OCCURRED            ((U8)0x11)
#define HCI_EV_ROLE_CHANGE               ((U8)0x12)
#define HCI_EV_NUM_COMPD_PKTS            ((U8)0x13)
#define HCI_EV_MODE_CHANGE               ((U8)0x14)
#define HCI_EV_RETURN_LINK_KEYS          ((U8)0x15)
#define HCI_EV_PIN_CODE_REQ              ((U8)0x16)
#define HCI_EV_LINK_KEY_REQ              ((U8)0x17)
#define HCI_EV_LINK_KEY_NOTI             ((U8)0x18)
#define HCI_EV_LOOPBACK_CMD              ((U8)0x19)
#define HCI_EV_DATA_BUFF_OVERFLOW        ((U8)0x1A)
#define HCI_EV_MAX_SLOTS_CHANGE          ((U8)0x1B)
#define HCI_EV_RD_CLOCK_OFFSET_COMP      ((U8)0x1C)
#define HCI_EV_CONN_PKT_TYPE_CHANGED     ((U8)0x1D)
#define HCI_EV_QOS_VIOLATION             ((U8)0x1E)
#define HCI_EV_PAGE_SCAN_MODE_CHANGE     ((U8)0x1F)
#define HCI_EV_PAGE_SCAN_REP_MODE_CHANGE ((U8)0x20)
/*1.2 evs */
#define HCI_EV_FLOW_SPEC_COMP            ((U8)0x21)
#define HCI_EV_INQUIRY_RES_WITH_RSSI     ((U8)0x22)
#define HCI_EV_RD_REM_EXT_FEATR_COMP     ((U8)0x23)
#define HCI_EV_FIXED_ADDR                ((U8)0x24)
#define HCI_EV_ALIAS_ADDR                ((U8)0x25)
#define HCI_EV_GENERATE_ALIAS_REQ        ((U8)0x26)
#define HCI_EV_ACT_ADDR                  ((U8)0x27)
#define HCI_EV_ALLOW_PRIVATE_PAIRING     ((U8)0x28)
#define HCI_EV_ALIAS_ADDR_REQ            ((U8)0x29)
#define HCI_EV_ALIAS_NOT_RECOGNISED      ((U8)0x2A)
#define HCI_EV_FIXED_ADDR_ATTMPT         ((U8)0x2B)
#define HCI_EV_SYNC_CONN_COMP            ((U8)0x2C)
#define HCI_EV_SYNC_CONN_CHANGED         ((U8)0x2D)
#define HCI_EV_SNIFF_SUB_RATE            ((U8)0x2E)

#ifdef  CFG_BT_VER_21
#define HCI_EV_SUBNIFF_RATING_EVENT      ((U8)0x2E)
#define HCI_EV_EXT_INQUIRY_RES           ((U8)0x2F)
#define HCI_EV_ENCRY_KEY_REF_COMPLETE    ((U8)0x30)
#define HCI_EV_IO_CAP_REQ                ((U8)0x31)
#define HCI_EV_IO_CAP_RESP               ((U8)0x32)
#define HCI_EV_USER_CONF_REQ             ((U8)0x33)
#define HCI_EV_USER_PASSKEY_REQ          ((U8)0x34)
#define HCI_EV_RMT_OOB_DATA_REQ          ((U8)0x35)
#define HCI_EV_SMP_PAIR_COMPLETE         ((U8)0x36)
//#define NOT_DEFINED
#define HCI_EV_LINK_SUPER_TIMEOUT_CHANGED   ((U8)0x38)
#define HCI_EV_EHA_FLUSH_COMPLETE           ((U8)0x39)
//#define  NOT_DEFINED                      ((U8)0x3A)
#define HCI_EV_USER_PASSKEY_NOTIFI          ((U8)0x3B)
#define HCI_EV_KEYPRESS_NOTIFI              ((U8)0x3C)
#define HCI_EV_RMT_HOST_SUPP_FEATUR_NOTIFI  ((U8)0x3D)
#endif


#ifdef  CFG_BT_VER_21
#define HCI_MAX_EV_OPP_CLTODE               ((U8)0x3E)
#else
#define HCI_MAX_EV_OPP_CLTODE               ((U8)0x2F)
#endif


/* Note:
   The HCI_EV_MANUFACTURER_EXTENSION definition below, uses U8 instead of
   U8, as we do not wish the auto hci generating code
   to take this num as the last elem in the ev messages
   hence creating 0xFF locations when only 0x21 will be used */

#define HCI_EV_MANUFACTURER_EXTENSION       ((U8)0xFF)
#define HCI_CREATE_CONN_ESC_RET             ((U16)HCI_LINK_RET | 0x0008)
#define HCI_LINK_KEY_REQ_REPLY_RET          ((U16)HCI_LINK_RET | 0x000B)
#define HCI_LINK_KEY_REQ_NEG_REPLY_RET      ((U16)HCI_LINK_RET | 0x000C)
#define HCI_PIN_CODE_REQ_REPLY_RET          ((U16)HCI_LINK_RET | 0x000D)
#define HCI_PIN_CODE_REQ_NEG_REPLY_RET      ((U16)HCI_LINK_RET | 0x000E)
#define HCI_RMT_NAME_REQ_ESC_RET            ((U16)HCI_LINK_RET | 0x001A)
#define HCI_MAX_LINK_RET_OCF                ((U16)0x000F)
#define HCI_ROLE_DISCOV_RET                 ((U16)HCI_POLICY_RET | 0x0009)
#define HCI_RD_LINK_POLICY_SETTINGS_RET     ((U16)HCI_POLICY_RET | 0x000C)
#define HCI_WR_LINK_POLICY_SETTINGS_RET     ((U16)HCI_POLICY_RET | 0x000D)
#define HCI_MAX_POLICY_RET_OCF              ((U16)0x000E)
#define HCI_FLUSH_RET                       ((U16)HCI_HOST_BB_RET | 0x0008)
#define HCI_RD_PIN_TYPE_RET                 ((U16)HCI_HOST_BB_RET | 0x0009)
#define HCI_RD_STORED_LINK_KEY_RET          ((U16)HCI_HOST_BB_RET | 0x000D)
#define HCI_WR_STORED_LINK_KEY_RET          ((U16)HCI_HOST_BB_RET | 0x0011)
#define HCI_DEL_STORED_LINK_KEY_RET         ((U16)HCI_HOST_BB_RET | 0x0012)
#define HCI_RD_LOCAL_NAME_RET               ((U16)HCI_HOST_BB_RET | 0x0014)
#define HCI_RD_CONN_ACPT_TIMEOUT_RET        ((U16)HCI_HOST_BB_RET | 0x0015)
#define HCI_RD_PAGE_TIMEOUT_RET             ((U16)HCI_HOST_BB_RET | 0x0017)
#define HCI_RD_SCAN_ENB_RET                 ((U16)HCI_HOST_BB_RET | 0x0019)
#define HCI_RD_PAGESCAN_ACTIVITY_RET        ((U16)HCI_HOST_BB_RET | 0x001B)
#define HCI_RD_INQUIRYSCAN_ACTIVITY_RET     ((U16)HCI_HOST_BB_RET | 0x001D)
#define HCI_RD_AUTH_ENB_RET                 ((U16)HCI_HOST_BB_RET | 0x001F)
#define HCI_RD_ENC_MODE_RET                 ((U16)HCI_HOST_BB_RET | 0x0021)
#define HCI_RD_DEV_CLS_RET                  ((U16)HCI_HOST_BB_RET | 0x0023)
#define HCI_RD_VOICE_SETTING_RET            ((U16)HCI_HOST_BB_RET | 0x0025)
#define HCI_RD_AUTO_FLUSH_TIMEOUT_RET       ((U16)HCI_HOST_BB_RET | 0x0027)
#define HCI_WR_AUTO_FLUSH_TIMEOUT_RET       ((U16)HCI_HOST_BB_RET | 0x0028)
#define HCI_RD_NUM_BCAST_RETXS_RET          ((U16)HCI_HOST_BB_RET | 0x0029)
#define HCI_RD_HOLD_MODE_ACTIVITY_RET       ((U16)HCI_HOST_BB_RET | 0x002B)
#define HCI_RD_TX_POWER_LEVEL_RET           ((U16)HCI_HOST_BB_RET | 0x002D)
#define HCI_RD_SCO_FLOW_CON_ENB_RET         ((U16)HCI_HOST_BB_RET | 0x002E)
#define HCI_RD_LINK_SUPERV_TIMEOUT_RET      ((U16)HCI_HOST_BB_RET | 0x0036)
#define HCI_WR_LINK_SUPERV_TIMEOUT_RET      ((U16)HCI_HOST_BB_RET | 0x0037)
#define HCI_RD_NUM_SUPP_IAC_RET             ((U16)HCI_HOST_BB_RET | 0x0038)
#define HCI_RD_CUR_IAC_LAP_RET              ((U16)HCI_HOST_BB_RET | 0x0039)
#define HCI_RD_PAGESCAN_PERIOD_MODE_RET     ((U16)HCI_HOST_BB_RET | 0x003B)
#define HCI_RD_PAGESCAN_MODE_RET            ((U16)HCI_HOST_BB_RET | 0x003D)
#define HCI_RD_AFH_CHNL_CLS_M_RET           ((U16)HCI_HOST_BB_RET | 0x0048)
#define HCI_MAX_HOST_BB_RET_OCF             ((U16)0x0049)
#define HCI_RD_LOCAL_VER_INFO_RET           ((U16)HCI_INFO_RET | 0x0001)
#define HCI_RD_LOCAL_SUPP_CMDS_RET          ((U16)HCI_INFO_RET | 0x0002)
#define HCI_RD_LOCAL_SUPP_FEATR_RET         ((U16)HCI_INFO_RET | 0x0003)
#define HCI_RD_LOCAL_EXT_FEATR_RET          ((U16)HCI_INFO_RET | 0x0004)
#define HCI_RD_BUFF_SIZE_RET                ((U16)HCI_INFO_RET | 0x0005)
#define HCI_RD_COUNTRY_CODE_RET             ((U16)HCI_INFO_RET | 0x0007)
#define HCI_RD_BD_ADDR_RET                  ((U16)HCI_INFO_RET | 0x0009)
#define HCI_MAX_INFO_RET_OCF                ((U16)0x000A)
#define HCI_RD_FAILED_CONTACT_COUNT_RET     ((U16)HCI_STS_RET | 0x0001)
#define HCI_RESET_FAILED_CONTACT_COUNT_RET  ((U16)HCI_STS_RET | 0x0002)
#define HCI_GET_LINK_QA_RET                 ((U16)HCI_STS_RET | 0x0003)
#define HCI_RD_RSSI_RET                     ((U16)HCI_STS_RET | 0x0005)
#define HCI_RD_AFH_CHNL_MAP_RET             ((U16)HCI_STS_RET | 0x0006)
#define HCI_RD_CLOCK_RET                    ((U16)HCI_STS_RET | 0x0007)
#define HCI_MAX_STS_RET_OCF                 ((U16)0x0008)
#define HCI_RD_LOOPBACK_MODE_RET            ((U16)HCI_TEST_RET | 0x0001)
#define HCI_MAX_TEST_RET_OCF                ((U16)0x0004)
#define HCI_INQUIRY_PARAM_LEN               ((U8)5)
#define HCI_INQUIRY_ESC_PARAM_LEN           ((U8)0)
#define HCI_PERIODIC_INQUIRY_MODE_PARAM_LEN         ((U8)9)
#define HCI_EXIT_PERIODIC_INQUIRY_MODE_PARAM_LEN    ((U8)0)
#define HCI_CREATE_CONN_PARAM_LEN                   ((U8)13)
#define HCI_DISC_PARAM_LEN                          ((U8)3)
#define HCI_ADD_SCO_CONN_PARAM_LEN                  ((U8)4)
#define HCI_CREATE_CONN_ESC_PARAM_LEN               ((U8)6)
#define HCI_ACPT_CONN_REQ_PARAM_LEN                 ((U8)7)
#define HCI_REJ_CONN_REQ_PARAM_LEN                  ((U8)7)
#define HCI_LINK_KEY_REQ_REPLY_PARAM_LEN            ((U8)22)
#define HCI_LINK_KEY_REQ_NEG_REPLY_PARAM_LEN        ((U8)6)
#define HCI_PIN_CODE_REQ_REPLY_PARAM_LEN            ((U8)23)
#define HCI_PIN_CODE_REQ_NEG_REPLY_PARAM_LEN        ((U8)6)
#define HCI_CHANGE_CONN_PKT_TYPE_PARAM_LEN          ((U8)4)
#define HCI_AUTH_REQ_PARAM_LEN                      ((U8)2)
#define HCI_SET_CONN_ENCRYPTION_PARAM_LEN           ((U8)3)
#define HCI_CHANGE_CONN_LINK_KEY_PARAM_LEN          ((U8)2)
#define HCI_MASTER_LINK_KEY_PARAM_LEN               ((U8)1)
#define HCI_RMT_NAME_REQ_PARAM_LEN                  ((U8)10)
#define HCI_RMT_NAME_REQ_ESC_PARAM_LEN              ((U8)6)
#define HCI_RD_RMT_SUPP_FEATR_PARAM_LEN             ((U8)2)
#define HCI_RD_RMT_EXT_FEATR_PARAM_LEN              ((U8)3)
#define HCI_RD_RMT_VER_INFO_PARAM_LEN               ((U8)2)
#define HCI_RD_CLOCK_OFFSET_PARAM_LEN               ((U8)2)
#define HCI_RD_LMP_HANDLE_PARAM_LEN                 ((U8)2)
#define HCI_EXCHANGE_FIXED_INFO_PARAM_LEN           ((U8)2)
#define HCI_EXCHANGE_ALIAS_INFO_PARAM_LEN           ((U8)8)
#define HCI_PRIVATE_PAIRING_REQ_REPLY_PARAM_LEN     ((U8)2)
#define HCI_PRIVATE_PAIRING_REQ_NEG_REPLY_PARAM_LEN ((U8)2)
#define HCI_GENERATED_ALIAS_PARAM_LEN               ((U8)8)
#define HCI_ALIAS_ADDR_REQ_REPLY_PARAM_LEN          ((U8)12)
#define HCI_ALIAS_ADDR_REQ_NEG_REPLY_PARAM_LEN      ((U8)6)
#define HCI_SETUP_SYNCHRONOUS_CONN_PARAM_LEN        ((U8)17)
#define HCI_ACPT_SYNCHRONOUS_CONN_REQ_PARAM_LEN     ((U8)21)
#define HCI_REJ_SYNCHRONOUS_CONN_REQ_PARAM_LEN      ((U8)7)
#ifdef CFG_BT_VER_21
#define HCI_IO_CAPACITY_REQ_REPLY_PARAM_LEN         ((U8)9)
#define HCI_USER_CONF_REQUEST_REPLY_PARAM_LEN       ((U8)6)
#define HCI_USER_CONF_REQUEST_NEG_REPLY_PARAM_LEN   ((U8)6)
#define HCI_USER_PASSKEY_REPLY_PARAM_LEN            ((U8)10)
#define HCI_USER_PASSKEY_NEG_REPLY_PARAM_LEN        ((U8)6)

#define HCI_RMT_OOB_DATA_REPLEY_PARAM_LEN           ((U8)38)
#define HCI_RMT_OOB_DATA_NEG_REPLEY_PARAM_LEN       ((U8)6)
#define HCI_IO_CAPACITY_REQ_NEG_REPLY_PARAM_LEN     ((U8)7)
#endif
#define HCI_ENH_SETUP_SYNCHRONOUS_CONN_PARAM_LEN     ((U8)59)
#define HCI_ENH_ACPT_SYNCHRONOUS_CONN_REQ_PARAM_LEN  ((U8)63)

#define HCI_HOLD_MODE_PARAM_LEN                        ((U8)6)
#define HCI_SNIFF_MODE_PARAM_LEN                       ((U8)10)
#define HCI_EXIT_SNIFF_MODE_PARAM_LEN                  ((U8)2)
#define HCI_PARK_MODE_PARAM_LEN                        ((U8)6)
#define HCI_EXIT_PARK_MODE_PARAM_LEN                   ((U8)2)
#define HCI_QOS_SETUP_PARAM_LEN                        ((U8)20)
#define HCI_ROLE_DISCOV_PARAM_LEN                      ((U8)2)
#define HCI_SWITCH_ROLE_PARAM_LEN                      ((U8)7)
#define HCI_RD_LINK_POLICY_SETTINGS_PARAM_LEN          ((U8)2)
#define HCI_WR_LINK_POLICY_SETTINGS_PARAM_LEN          ((U8)4)
#define HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_PARAM_LEN  ((U8)0)
#define HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_PARAM_LEN  ((U8)2)
#define HCI_FLOW_SPEC_PARAM_LEN                        ((U8)21)

#define HCI_SET_EV_MASK_PARAM_LEN                      ((U8)8)
#define HCI_RESET_PARAM_LEN                            ((U8)0)
#define HCI_SET_EV_FILTER_PARAM_LEN                    ((U8)8) /* variable */
#define HCI_FLUSH_PARAM_LEN                            ((U8)2)
#define HCI_RD_PIN_TYPE_PARAM_LEN                      ((U8)0)
#define HCI_WR_PIN_TYPE_PARAM_LEN                      ((U8)1)
#define HCI_CREATE_NEW_UNIT_KEY_PARAM_LEN              ((U8)0)
#define HCI_RD_STORED_LINK_KEY_PARAM_LEN               ((U8)7)
#define HCI_WR_STORED_LINK_KEY_PARAM_LEN               ((U8)23) /* variable */
#define HCI_DEL_STORED_LINK_KEY_PARAM_LEN              ((U8)7)
#define HCI_CHANGE_LOCAL_NAME_PARAM_LEN                ((U8)248)
#define HCI_RD_LOCAL_NAME_PARAM_LEN                    ((U8)0)
#define HCI_RD_CONN_ACPT_TIMEOUT_PARAM_LEN             ((U8)0)
#define HCI_WR_CONN_ACPT_TIMEOUT_PARAM_LEN             ((U8)2)
#define HCI_RD_PAGE_TIMEOUT_PARAM_LEN                  ((U8)0)
#define HCI_WR_PAGE_TIMEOUT_PARAM_LEN                  ((U8)2)
#define HCI_RD_SCAN_ENB_PARAM_LEN                      ((U8)0)
#define HCI_WR_SCAN_ENB_PARAM_LEN                      ((U8)1)
#define HCI_RD_PAGESCAN_ACTIVITY_PARAM_LEN             ((U8)0)
#define HCI_WR_PAGESCAN_ACTIVITY_PARAM_LEN             ((U8)4)
#define HCI_RD_INQUIRYSCAN_ACTIVITY_PARAM_LEN          ((U8)0)
#define HCI_WR_INQUIRYSCAN_ACTIVITY_PARAM_LEN          ((U8)4)
#define HCI_RD_AUTH_ENB_PARAM_LEN                      ((U8)0)
#define HCI_WR_AUTH_ENB_PARAM_LEN                      ((U8)1)
#define HCI_RD_ENC_MODE_PARAM_LEN                      ((U8)0)
#define HCI_WR_ENC_MODE_PARAM_LEN                      ((U8)1)
#define HCI_RD_DEV_CLS_PARAM_LEN                       ((U8)0)
#define HCI_WR_DEV_CLS_PARAM_LEN                       ((U8)3)
#define HCI_RD_VOICE_SETTING_PARAM_LEN                 ((U8)0)
#define HCI_WR_VOICE_SETTING_PARAM_LEN                 ((U8)2)
#define HCI_RD_AUTO_FLUSH_TIMEOUT_PARAM_LEN            ((U8)2)
#define HCI_WR_AUTO_FLUSH_TIMEOUT_PARAM_LEN            ((U8)4)
#define HCI_RD_NUM_BCAST_RETXS_PARAM_LEN               ((U8)0)
#define HCI_WR_NUM_BCAST_RETXS_PARAM_LEN               ((U8)1)
#define HCI_RD_HOLD_MODE_ACTIVITY_PARAM_LEN            ((U8)0)
#define HCI_WR_HOLD_MODE_ACTIVITY_PARAM_LEN            ((U8)1)
#define HCI_RD_TX_POWER_LEVEL_PARAM_LEN                ((U8)3)
#define HCI_RD_SCO_FLOW_CON_ENB_PARAM_LEN              ((U8)0)
#define HCI_WR_SCO_FLOW_CON_ENB_PARAM_LEN              ((U8)1)
#define HCI_SET_HCTOHOST_FLOW_CTRL_PARAM_LEN           ((U8)1)
#define HCI_HOST_BUFF_SIZE_PARAM_LEN                   ((U8)7)
#define HCI_HOST_NUM_COMPD_PKTS_PARAM_LEN              ((U8)5) /* variable */
#define HCI_RD_LINK_SUPERV_TIMEOUT_PARAM_LEN           ((U8)2)
#define HCI_WR_LINK_SUPERV_TIMEOUT_PARAM_LEN           ((U8)4)
#define HCI_RD_NUM_SUPP_IAC_PARAM_LEN                  ((U8)0)
#define HCI_RD_CUR_IAC_LAP_PARAM_LEN                   ((U8)0)
#define HCI_WR_CUR_IAC_LAP_PARAM_LEN                   ((U8)4) /* variable */
#define HCI_RD_PAGESCAN_PERIOD_MODE_PARAM_LEN          ((U8)0)
#define HCI_WR_PAGESCAN_PERIOD_MODE_PARAM_LEN          ((U8)1)
#define HCI_RD_PAGESCAN_MODE_PARAM_LEN                 ((U8)0)
#define HCI_WR_PAGESCAN_MODE_PARAM_LEN                 ((U8)1)
#define HCI_SET_AFH_CHNL_CLS_PARAM_LEN                 ((U8)10)
#define HCI_RD_INQUIRY_SCAN_TYPE_PARAM_LEN             ((U8)0)
#define HCI_WR_INQUIRY_SCAN_TYPE_PARAM_LEN             ((U8)1)
#define HCI_RD_INQUIRY_MODE_PARAM_LEN                  ((U8)0)
#define HCI_WR_INQUIRY_MODE_PARAM_LEN                  ((U8)1)
#define HCI_RD_PAGE_SCAN_TYPE_PARAM_LEN                ((U8)0)
#define HCI_WR_PAGE_SCAN_TYPE_PARAM_LEN                ((U8)1)
#define HCI_RD_AFH_CHNL_CLS_M_PARAM_LEN                ((U8)0)
#define HCI_WR_AFH_CHNL_CLS_M_PARAM_LEN                ((U8)1)
#define HCI_RD_ANON_MODE_PARAM_LEN                     ((U8)0)
#define HCI_WR_ANON_MODE_PARAM_LEN                     ((U8)1)
#define HCI_RD_ALIAS_AUTH_ENB_PARAM_LEN                ((U8)0)
#define HCI_WR_ALIAS_AUTH_ENB_PARAM_LEN                ((U8)1)
#define HCI_RD_ANON_ADDR_CHANGE_PARAMS_PARAM_LEN       ((U8)0)
#define HCI_WR_ANON_ADDR_CHANGE_PARAMS_PARAM_LEN       ((U8)6)
#define HCI_RESET_FIXED_ADDR_ATTMPTS_COUNTER_PARAM_LEN ((U8)1)

#define HCI_RD_LOCAL_VER_INFO_PARAM_LEN          ((U8)0)
#define HCI_RD_LOCAL_SUPP_CMDS_PARAM_LEN         ((U8)0)
#define HCI_RD_LOCAL_SUPP_FEATR_PARAM_LEN        ((U8)0)
#define HCI_RD_LOCAL_EXT_FEATR_PARAM_LEN         ((U8)1)
#define HCI_RD_BUFF_SIZE_PARAM_LEN               ((U8)0)
#define HCI_RD_COUNTRY_CODE_PARAM_LEN            ((U8)0)
#define HCI_RD_BD_ADDR_PARAM_LEN                 ((U8)0)

#define HCI_RD_FAILED_CONTACT_COUNT_PARAM_LEN    ((U8)2)
#define HCI_RESET_FAILED_CONTACT_COUNT_PARAM_LEN ((U8)2)
#define HCI_GET_LINK_QA_PARAM_LEN                ((U8)2)
#define HCI_RD_RSSI_PARAM_LEN                    ((U8)2)
#define HCI_RD_AFH_CHNL_MAP_PARAM_LEN            ((U8)2)
#define HCI_RD_CLOCK_PARAM_LEN                   ((U8)3)

#define HCI_RD_LOOPBACK_MODE_PARAM_LEN           ((U8)0)
#define HCI_WR_LOOPBACK_MODE_PARAM_LEN           ((U8)1)
#define HCI_ENB_DUT_MODE_PARAM_LEN               ((U8)0)

#define HCI_DBG_REQ_PARAM_LEN                       ((U8)2)
#define HCI_EV_INQUIRY_COMP_PARAM_LEN               ((U8)1)
#define HCI_EV_INQUIRY_RES_PARAM_LEN                ((U8)15) /* variable */
#define HCI_EV_CONN_COMP_PARAM_LEN                  ((U8)11)
#define HCI_EV_CONN_REQ_PARAM_LEN                   ((U8)10)
#define HCI_EV_DISC_COMP_PARAM_LEN                  ((U8)4)
#define HCI_EV_AUTH_COMP_PARAM_LEN                  ((U8)3)
#define HCI_EV_RMT_NAME_REQ_COMP_MAX_LEN            ((U8)255)
#define HCI_EV_RMT_NAME_REQ_COMP_BASIC_LEN          ((U8)7)
#define HCI_EV_ENCRYPTION_CHANGE_PARAM_LEN          ((U8)4)
#define HCI_EV_CHANGE_CONN_LINK_KEY_COMP_PARAM_LEN  ((U8)3)
#define HCI_EV_MASTER_LINK_KEY_COMP_PARAM_LEN       ((U8)4)
#define HCI_EV_RD_REM_SUPP_FEATR_COMP_PARAM_LEN     ((U8)11)
#define HCI_EV_RD_RMT_VER_INFO_COMP_PARAM_LEN       ((U8)8)
#define HCI_EV_QOS_SETUP_COMP_PARAM_LEN             ((U8)21)
#define HCI_EV_CMD_COMP_PARAM_LEN                   ((U8)3) /* variable see below */
#define HCI_EV_CMD_STS_PARAM_LEN                    ((U8)4)
#define HCI_EV_HARDWARE_ERR_PARAM_LEN               ((U8)1)
#define HCI_EV_FLUSH_OCCURRED_PARAM_LEN             ((U8)2)
#define HCI_EV_ROLE_CHANGE_PARAM_LEN                ((U8)8)
#define HCI_EV_NUM_COMPD_PKTS_PARAM_LEN             ((U8)5) /* variable */
#define HCI_EV_MODE_CHANGE_PARAM_LEN                ((U8)6)
#define HCI_EV_RETURN_LINK_KEYS_PARAM_LEN           ((U8)23)/* variable */
#define HCI_EV_PIN_CODE_REQ_PARAM_LEN               ((U8)6)
#define HCI_EV_LINK_KEY_REQ_PARAM_LEN               ((U8)6)
#define HCI_EV_LINK_KEY_NOTI_PARAM_LEN              ((U8)23)
#define HCI_EV_LOOPBACK_CMD_PARAM_LEN               ((U8)0) /* variable */
#define HCI_EV_DATA_BUFF_OVERFLOW_PARAM_LEN         ((U8)1)
#define HCI_EV_MAX_SLOTS_CHANGE_PARAM_LEN           ((U8)3)
#define HCI_EV_RD_CLOCK_OFFSET_COMP_PARAM_LEN       ((U8)5)
#define HCI_EV_CONN_PKT_TYPE_CHANGED_PARAM_LEN      ((U8)5)
#define HCI_EV_QOS_VIOLATION_PARAM_LEN              ((U8)2)
#define HCI_EV_PAGE_SCAN_MODE_CHANGE_PARAM_LEN      ((U8)7)
#define HCI_EV_PAGE_SCAN_REP_MODE_CHANGE_PARAM_LEN  ((U8)7)
#define HCI_EV_FLOW_SPEC_COMP_PARAM_LEN             ((U8)22)
#define HCI_EV_INQUIRY_RES_WITH_RSSI_PARAM_LEN      ((U8)15) /* variable */
#define HCI_EV_RD_REM_EXT_FEATR_COMP_PARAM_LEN      ((U8)13)
#define HCI_EV_FIXED_ADDR_PARAM_LEN                 ((U8)9)
#define HCI_EV_ALIAS_ADDR_PARAM_LEN                 ((U8)8)
#define HCI_EV_GENERATE_ALIAS_REQ_PARAM_LEN         ((U8)2)
#define HCI_EV_ACT_ADDR_PARAM_LEN                   ((U8)12)
#define HCI_EV_ALLOW_PRIVATE_PAIRING_PARAM_LEN      ((U8)2)
#define HCI_EV_ALIAS_ADDR_REQ_PARAM_LEN             ((U8)6)
#define HCI_EV_ALIAS_NOT_RECOGNISED_PARAM_LEN       ((U8)7)
#define HCI_EV_FIXED_ADDR_ATTMPT_PARAM_LEN          ((U8)4)
#define HCI_EV_SYNC_CONN_COMP_PARAM_LEN             ((U8)17)
#define HCI_EV_SYNC_CONN_CHANGED_PARAM_LEN          ((U8)9)
#define HCI_EV_SNIFF_SUB_RATE_PARAM_LEN             ((U8)11)

#define HCI_EV_DBG_PARAM_LEN                        ((U8)20)

/* HCI_CMD_COMP, arg len definitions (full len)
   3 from cmd comp + return pars.
   return pars means st + other pars.
   when an arg len is dependant on the num of elems in the array
   the defined len contains the const par lens only. the full
   array len must be calculated */

#define HCI_INQUIRY_ESC_ARG_LEN                   ((U8)4)
#define HCI_PERIODIC_INQ_MODE_ARG_LEN             ((U8)4)
#define HCI_EXIT_PERIODIC_INQ_MODE_ARG_LEN        ((U8)4)
#define HCI_CREATE_CONN_ESC_ARG_LEN               ((U8)10)
#define HCI_LINK_KEY_REQ_REPLY_ARG_LEN            ((U8)10)
#define HCI_LINK_KEY_REQ_NEG_REPLY_ARG_LEN        ((U8)10)
#define HCI_PIN_CODE_REQ_REPLY_ARG_LEN            ((U8)10)
#define HCI_PIN_CODE_REQ_NEG_REPLY_ARG_LEN        ((U8)10)
#define HCI_RMT_NAME_REQ_ESC_ARG_LEN              ((U8)10)
#define HCI_RD_LMP_HANDLE_ARG_LEN                 ((U8)11)
#define HCI_PRIVATE_PAIRING_REQ_REPLY_ARG_LEN     ((U8)6)
#define HCI_PRIVATE_PAIRING_REQ_NEG_REPLY_ARG_LEN ((U8)6)
#define HCI_GENERATED_ALIAS_ARG_LEN               ((U8)6)
#define HCI_ALIAS_ADDR_REQ_REPLY_ARG_LEN          ((U8)10)
#define HCI_ALIAS_ADDR_REQ_NEG_REPLY_ARG_LEN      ((U8)10)

#define HCI_ROLE_DISCOV_ARG_LEN                   ((U8)7)
#define HCI_RD_LINK_POLICY_SETTINGS_ARG_LEN       ((U8)8)
#define HCI_WR_LINK_POLICY_SETTINGS_ARG_LEN       ((U8)6)
#define HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_ARG_LEN ((U8)6)
#define HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_ARG_LEN ((U8)4)

#define HCI_SET_EV_MASK_ARG_LEN                      ((U8)4)
#define HCI_RESET_ARG_LEN                            ((U8)4)
#define HCI_SET_EV_FILTER_ARG_LEN                    ((U8)4)
#define HCI_FLUSH_ARG_LEN                            ((U8)6)
#define HCI_RD_PIN_TYPE_ARG_LEN                      ((U8)5)
#define HCI_WR_PIN_TYPE_ARG_LEN                      ((U8)4)
#define HCI_CREATE_NEW_UNIT_KEY_ARG_LEN              ((U8)4)
#define HCI_RD_STORED_LINK_KEY_ARG_LEN               ((U8)8)
#define HCI_WR_STORED_LINK_KEY_ARG_LEN               ((U8)5)
#define HCI_DEL_STORED_LINK_KEY_ARG_LEN              ((U8)6)
#define HCI_CHANGE_LOCAL_NAME_ARG_LEN                ((U8)4)
#define HCI_RD_LOCAL_NAME_ARG_LEN                    ((U8)252)
#define HCI_RD_CONN_ACPT_TIMEOUT_ARG_LEN             ((U8)6)
#define HCI_WR_CONN_ACPT_TIMEOUT_ARG_LEN             ((U8)4)
#define HCI_RD_PAGE_TIMEOUT_ARG_LEN                  ((U8)6)
#define HCI_WR_PAGE_TIMEOUT_ARG_LEN                  ((U8)4)
#define HCI_RD_SCAN_ENB_ARG_LEN                      ((U8)5)
#define HCI_WR_SCAN_ENB_ARG_LEN                      ((U8)4)
#define HCI_RD_PAGESCAN_ACTIVITY_ARG_LEN             ((U8)8)
#define HCI_WR_PAGESCAN_ACTIVITY_ARG_LEN             ((U8)4)
#define HCI_RD_INQUIRYSCAN_ACTIVITY_ARG_LEN          ((U8)8)
#define HCI_WR_INQUIRYSCAN_ACTIVITY_ARG_LEN          ((U8)4)
#define HCI_RD_AUTH_ENB_ARG_LEN                      ((U8)5)
#define HCI_WR_AUTH_ENB_ARG_LEN                      ((U8)4)
#define HCI_RD_ENC_MODE_ARG_LEN                      ((U8)5)
#define HCI_WR_ENC_MODE_ARG_LEN                      ((U8)4)
#define HCI_RD_DEV_CLS_ARG_LEN                       ((U8)7)
#define HCI_WR_DEV_CLS_ARG_LEN                       ((U8)4)
#define HCI_RD_VOICE_SETTING_ARG_LEN                 ((U8)6)
#define HCI_WR_VOICE_SETTING_ARG_LEN                 ((U8)4)
#define HCI_RD_AUTO_FLUSH_TIMEOUT_ARG_LEN            ((U8)8)
#define HCI_WR_AUTO_FLUSH_TIMEOUT_ARG_LEN            ((U8)6)
#define HCI_RD_NUM_BCASTXS_ARG_LEN                   ((U8)5)
#define HCI_WR_NUM_BCASTXS_ARG_LEN                   ((U8)4)
#define HCI_RD_HOLD_MODE_ACTIVITY_ARG_LEN            ((U8)5)
#define HCI_WR_HOLD_MODE_ACTIVITY_ARG_LEN            ((U8)4)
#define HCI_RD_TX_POWER_LEVEL_ARG_LEN                ((U8)7)
#define HCI_RD_SCO_FLOW_CON_ENB_ARG_LEN              ((U8)5)
#define HCI_WR_SCO_FLOW_CON_ENB_ARG_LEN              ((U8)4)
#define HCI_SET_HC_TO_H_FLOW_CTRL_ARG_LEN            ((U8)4)
#define HCI_HOST_BUFF_SIZE_ARG_LEN                   ((U8)4)
#define HCI_HOST_NUM_COMPD_PKTS_ARG_LEN              ((U8)4)
#define HCI_RD_LINK_SUPERV_TIMEOUT_ARG_LEN           ((U8)8)
#define HCI_WR_LINK_SUPERV_TIMEOUT_ARG_LEN           ((U8)6)
#define HCI_RD_NUM_SUPP_IAC_ARG_LEN                  ((U8)5)
#define HCI_RD_CUR_IAC_LAP_ARG_LEN                   ((U8)5) /* variable */
#define HCI_WR_CUR_IAC_LAP_ARG_LEN                   ((U8)4)
#define HCI_RD_PAGESCAN_PERIOD_MODE_ARG_LEN          ((U8)5)
#define HCI_WR_PAGESCAN_PERIOD_MODE_ARG_LEN          ((U8)4)
#define HCI_RD_PAGESCAN_MODE_ARG_LEN                 ((U8)5)
#define HCI_WR_PAGESCAN_MODE_ARG_LEN                 ((U8)4)
#define HCI_SET_AFH_CHNL_CLS_ARG_LEN                 ((U8)4)
#define HCI_RD_INQUIRY_SCAN_TYPE_ARG_LEN             ((U8)5)
#define HCI_WR_INQUIRY_SCAN_TYPE_ARG_LEN             ((U8)4)
#define HCI_RD_INQUIRY_MODE_ARG_LEN                  ((U8)5)
#define HCI_WR_INQUIRY_MODE_ARG_LEN                  ((U8)4)
#define HCI_RD_PAGE_SCAN_TYPE_ARG_LEN                ((U8)5)
#define HCI_WR_PAGE_SCAN_TYPE_ARG_LEN                ((U8)4)
#define HCI_RD_AFH_CHNL_CLS_M_ARG_LEN                ((U8)5)
#define HCI_WR_AFH_CHNL_CLS_M_ARG_LEN                ((U8)4)
#define HCI_RD_ANON_MODE_ARG_LEN                     ((U8)5)
#define HCI_WR_ANON_MODE_ARG_LEN                     ((U8)4)
#define HCI_RD_ALIAS_AUTH_ENB_ARG_LEN                ((U8)5)
#define HCI_WR_ALIAS_AUTH_ENB_ARG_LEN                ((U8)4)
#define HCI_RD_ANON_ADDR_CHANGE_PARAMS_ARG_LEN       ((U8)10)
#define HCI_WR_ANON_ADDR_CHANGE_PARAMS_ARG_LEN       ((U8)4)
#define HCI_RESET_FIXED_ADDR_ATTMPTS_COUNTER_ARG_LEN ((U8)4)

#define HCI_RD_LOCAL_VER_INFO_ARG_LEN          ((U8)12)
#define HCI_RD_LOCAL_CMDS_LEN                  ((U8)68)
#define HCI_RD_LOCAL_FEATR_LEN                 ((U8)12)
#define HCI_RD_LOCAL_EXT_FEATR_LEN             ((U8)14)
#define HCI_RD_BUFF_SIZE_ARG_LEN               ((U8)11)
#define HCI_RD_COUNTRY_CODE_ARG_LEN            ((U8)5)
#define HCI_RD_BD_ADDR_ARG_LEN                 ((U8)10)
#define HCI_RD_FAILED_CONTACT_COUNT_ARG_LEN    ((U8)8)
#define HCI_RESET_FAILED_CONTACT_COUNT_ARG_LEN ((U8)6)
#define HCI_GET_LINK_QA_ARG_LEN                ((U8)7)
#define HCI_RD_RSSI_ARG_LEN                    ((U8)7)
#define HCI_RD_AFH_CHNL_MAP_ARG_LEN            ((U8)17)
#define HCI_RD_CLOCK_ARG_LEN                   ((U8)12)

#define HCI_RD_LOOPBACK_MODE_ARG_LEN           ((U8)5)
#define HCI_WR_LOOPBACK_MODE_ARG_LEN           ((U8)4)
#define HCI_ENB_DUT_ARG_LEN                    ((U8)4)
#define HCI_MNFR_EXTENSION_ARG_LEN             ((U8)4)
#define HCI_SNIFF_SUB_RATE_ARG_LEN             ((U8)6)

/* some aliasses to simplify auto - generated code */
#define HCI_PERIODIC_INQUIRY_MODE_ARG_LEN \
    HCI_PERIODIC_INQ_MODE_ARG_LEN
#define HCI_EXIT_PERIODIC_INQUIRY_MODE_ARG_LEN \
    HCI_EXIT_PERIODIC_INQ_MODE_ARG_LEN
#define HCI_RD_NUM_BCAST_RETXS_ARG_LEN \
    HCI_RD_NUM_BCASTXS_ARG_LEN
#define HCI_WR_NUM_BCAST_RETXS_ARG_LEN \
    HCI_WR_NUM_BCASTXS_ARG_LEN
#define HCI_SET_HCTOHOST_FLOW_CTRL_ARG_LEN \
    HCI_SET_HC_TO_H_FLOW_CTRL_ARG_LEN
#define HCI_RD_LOCAL_SUPP_FEATR_ARG_LEN \
    HCI_RD_LOCAL_FEATR_LEN
#define HCI_RD_LOCAL_SUPP_CMDS_ARG_LEN \
    HCI_RD_LOCAL_CMDS_LEN
#define HCI_RD_LOCAL_EXT_FEATR_ARG_LEN \
    HCI_RD_LOCAL_EXT_FEATR_LEN
#define HCI_ENB_DUT_MODE_ARG_LEN \
    HCI_ENB_DUT_ARG_LEN

#define HCI_BROADCAST_FLAG_ACT      ((U16)0x4000)
#define HCI_BROADCAST_FLAG_PICONET  ((U16)0x8000)
#define HCI_BROADCAST_FLAG_MASK     ((U16)0xC000)
/* this would be a val of 1 over HCI */
#define HCI_NBC_DEFAULT             (2)

#define HCI_PKT_BOUNDARY_FLAG_RSV         ((U16)0)
#define HCI_PKT_BOUNDARY_FLAG_CONT        ((U16)0x1000)
#define HCI_PKT_BOUNDARY_FLAG_FIRST       ((U16)0x2000)
#define HCI_PKT_BOUNDARY_MASK             ((U16)0x3000)
#define HCI_SUCC                          ((U8)0x00)
#define HCI_CMD_CURLY_PENDING             ((U8)0x00)
#define HCI_ERR_ILLEGAL_CMD               ((U8)0x01)
#define HCI_ERR_NO_CONN                   ((U8)0x02)
#define HCI_ERR_HARDWARE_FAIL             ((U8)0x03)
#define HCI_ERR_PAGE_TIMEOUT              ((U8)0x04)
#define HCI_ERR_AUTH_FAIL                 ((U8)0x05)
#define HCI_ERR_KEY_MISSING               ((U8)0x06)
#define HCI_ERR_MEM_FULL                  ((U8)0x07)
#define HCI_ERR_CONN_TIMEOUT              ((U8)0x08)
#define HCI_ERR_MAX_NR_OF_CONNS           ((U8)0x09)
#define HCI_ERR_MAX_NR_OF_SCO             ((U8)0x0A)
#define HCI_ERR_MAX_NR_OF_ACL             ((U8)0x0B)
#define HCI_ERR_CMD_DISALLOWED            ((U8)0x0C)
#define HCI_ERR_REJ_BY_RMT_NO_RES         ((U8)0x0D) /* no resrc */
#define HCI_ERR_REJ_BY_RMT_SEC            ((U8)0x0E) /* secu violation */
#define HCI_ERR_REJ_BY_RMT_PERS           ((U8)0x0F) /* personal dev */
#define HCI_ERR_HOST_TIMEOUT              ((U8)0x10)
#define HCI_ERR_UNSUPP_FEATURE            ((U8)0x11) /* or incorrect param val */
#define HCI_ERR_ILLEGAL_FMT               ((U8)0x12)
#define HCI_ERR_OETC_USER                 ((U8)0x13) /* other end terminated */
#define HCI_ERR_OETC_LOW_RESRC            ((U8)0x14) /* other end terminated */
#define HCI_ERR_OETC_POWERING_OFF         ((U8)0x15) /* other end terminated */
#define HCI_ERR_CONN_TERM_LOCAL_HOST      ((U8)0x16) /* local host terminated */
#define HCI_ERR_AUTH_REPEATED             ((U8)0x17)
#define HCI_ERR_PAIRING_NOT_ALLOWED       ((U8)0x18)
#define HCI_ERR_UNKNOWN_LMP_PDU           ((U8)0x19)
#define HCI_ERR_UNSUPP_REM_FEATURE        ((U8)0x1A)
#define HCI_ERR_SCO_OFFSET_REJED          ((U8)0x1B)
#define HCI_ERR_SCO_INTVL_REJED           ((U8)0x1C)
#define HCI_ERR_SCO_AIR_MODE_REJED        ((U8)0x1D)
#define HCI_ERR_INVLD_LMP_PARS            ((U8)0x1E)
#define HCI_ERR_UNSPECIFIED               ((U8)0x1F)
#define HCI_ERR_UNSUPP_LMP_PARAM          ((U8)0x20)
#define HCI_ERR_ROLE_CHANGE_NOT_ALLOWED   ((U8)0x21)
#define HCI_ERR_LMP_RSP_TIMEOUT           ((U8)0x22)
#define HCI_ERR_LMP_TRANSACTION_COLLISION ((U8)0x23)
#define HCI_ERR_LMP_PDU_NOT_ALLOWED       ((U8)0x24)
#define HCI_ERR_ENC_MODE_NOT_ACPTABLE     ((U8)0x25)
#define HCI_ERR_UNIT_KEY_USED             ((U8)0x26)
#define HCI_ERR_QOS_NOT_SUPP              ((U8)0x27)
#define HCI_ERR_INSTANT_PASSED            ((U8)0x28)
#define HCI_ERR_PAIR_UNIT_KEY_NO_SUPPORT  ((U8)0x29)
#define HCI_ERR_DIFFERENT_TRANSACTION_COLLISION  ((U8)0x2A)
#define HCI_ERR_SGAP_INSUFFICIENT_RESRCS         ((U8)0x2B)
#define HCI_ERR_QOS_UNACPTABLE_PAR               ((U8)0x2C)
#define HCI_ERR_QOS_REJED                        ((U8)0x2D)
#define HCI_ERR_CHNL_CLS_NO_SUPPORT              ((U8)0x2E)
#define HCI_ERR_INSUFFICIENT_SECU                ((U8)0x2F)
#define HCI_ERR_PARAM_OUT_OF_MAND_RANGE          ((U8)0x30)
#define HCI_ERR_SGAP_NO_LONGER_REQD              ((U8)0x31)
#define HCI_ERR_ROLE_SWITCH_PENDING              ((U8)0x32)
#define HCI_ERR_SGAP_PARAM_CHANGE_PENDING        ((U8)0x33)
#define HCI_ERR_RESVD_SLOT_VIOLATION             ((U8)0x34)
#define HCI_ERR_ROLE_SWITCH_FAILED               ((U8)0x35)


#define HCI_DEFAULT ((U8)0) /* 0 in a msg field means default */
#define HCI_MASTER ((U8)0)
#define HCI_SLAVE ((U8)1)

#define HCI_MASTER_SLAVE_UNKNOWN  ((U8)2)

#define HCI_LINK_KEY_SEMIPERM  ((U8)0)
#define HCI_LINK_KEY_TMP       ((U8)1)

#define HCI_LINK_ENC_OFF       ((U8)0)
#define HCI_LINK_ENC_ON        ((U8)1)

#define HCI_QOS_NO_TRAFFIC     ((U8)0)
#define HCI_QOS_BEST_EFFORT    ((U8)1)
#define HCI_QOS_GUARANTEED     ((U8)2)

/* HCI pkt types */
#define HCI_PKT_DM1       ((U16)0x0008) /* SCO and ACL */
#define HCI_PKT_DH1       ((U16)0x0010) /* ACL only */
#define HCI_PKT_HV1       ((U16)0x0020) /* SCO only */
#define HCI_PKT_HV2       ((U16)0x0040) /* SCO only */
#define HCI_PKT_HV3       ((U16)0x0080) /* SCO only */
#define HCI_PKT_DV        ((U16)0x0100) /* SCO only */
#define HCI_PKT_AUX1      ((U16)0x0200) /* ACL only NOT L2C */
#define HCI_PKT_DM3       ((U16)0x0400) /* ACL only */
#define HCI_PKT_DH3       ((U16)0x0800) /* ACL only */
#define HCI_PKT_DM5       ((U16)0x4000) /* ACL only */
#define HCI_PKT_DH5       ((U16)0x8000) /* ACL only */

#define HCI_PKT_HV123           ((U16)0x00E0) /* all SCO only */
#define HCI_PKT_HV123_INVERSE   ((U16)0xFF1F) /* all SCO only */

#define HCI_PKT_2DH1      ((U16)0x0002) /* ACL only */
#define HCI_PKT_3DH1      ((U16)0x0004) /* ACL only */
#define HCI_PKT_2DH3      ((U16)0x0100) /* ACL only */
#define HCI_PKT_3DH3      ((U16)0x0200) /* ACL only */
#define HCI_PKT_2DH5      ((U16)0x1000) /* ACL only */
#define HCI_PKT_3DH5      ((U16)0x2000) /* ACL only */

#define HCI_PKT_2MBP_LIST ((U16)HCI_PKT_2DH1|HCI_PKT_2DH3|HCI_PKT_2DH5)
#define HCI_PKT_3MBP_LIST ((U16)HCI_PKT_3DH1|HCI_PKT_3DH3|HCI_PKT_3DH5)
#define HCI_PKT_MR_1_SLOT_LIST ((U16)HCI_PKT_2DH1|HCI_PKT_3DH1)
#define HCI_PKT_MR_3_SLOT_LIST ((U16)HCI_PKT_2DH3|HCI_PKT_3DH3)
#define HCI_PKT_MR_5_SLOT_LIST ((U16)HCI_PKT_2DH5|HCI_PKT_3DH5)
#define HCI_PKT_MR_LIST ((U16)HCI_PKT_2MBP_LIST|HCI_PKT_3MBP_LIST)
#define HCI_PKT_3_SLOT_LIST ((U16)HCI_PKT_DM3|HCI_PKT_DH3)
#define HCI_PKT_5_SLOT_LIST ((U16)HCI_PKT_DM5|HCI_PKT_DH5)

/* bandwidth defaults */
#define HCI_ESCO_BDW_DONT_CARE      ((U32)0xFFFFFFFF)

/* max latency default */
#define HCI_ESCO_MAX_LATENCY_DEFAULT ((U16)0xFFFF)

/* pkt type defaults */
#define HCI_ESCO_DEFAULT_PKT_TYPE   ((U16)0xFFFF)

#define HCI_EV_MASK_MSB                     ((U32)0x00000000)
#define HCI_EV_MASK_NONE_HI                 ((U32)0x00000000)
#define HCI_EV_MASK_NONE_LO                 ((U32)0x00000000)
#define HCI_EV_MASK_INQUIRY_COMP_HI         ((U32)0x00000000)
#define HCI_EV_MASK_INQUIRY_COMP_LO         ((U32)0x00000001)
#define HCI_EV_MASK_INQUIRY_RES_HI          ((U32)0x00000000)
#define HCI_EV_MASK_INQUIRY_RES_LO          ((U32)0x00000002)
#define HCI_EV_MASK_CONN_COMP_HI            ((U32)0x00000000)
#define HCI_EV_MASK_CONN_COMP_LO            ((U32)0x00000004)
#define HCI_EV_MASK_CONN_REQ_HI             ((U32)0x00000000)
#define HCI_EV_MASK_CONN_REQ_LO             ((U32)0x00000008)
#define HCI_EV_MASK_DISC_COMP_HI            ((U32)0x00000000)
#define HCI_EV_MASK_DISC_COMP_LO            ((U32)0x00000010)
#define HCI_EV_MASK_AUTH_COMP_HI            ((U32)0x00000000)
#define HCI_EV_MASK_AUTH_COMP_LO            ((U32)0x00000020)
#define HCI_EV_MASK_REM_NAME_REQ_COMP_HI    ((U32)0x00000000)
#define HCI_EV_MASK_REM_NAME_REQ_COMP_LO    ((U32)0x00000040)
#define HCI_EV_MASK_CHG_CONN_ENC_EN_HI      ((U32)0x00000000)
#define HCI_EV_MASK_CHG_CONN_ENC_EN_LO      ((U32)0x00000080)
#define HCI_EV_MASK_CHG_CONN_LINK_KEY_HI    ((U32)0x00000000)
#define HCI_EV_MASK_CHG_CONN_LINK_KEY_LO    ((U32)0x00000100)
#define HCI_EV_MASK_MASTER_LINK_KEY_HI      ((U32)0x00000000)
#define HCI_EV_MASK_MASTER_LINK_KEY_LO      ((U32)0x00000200)
#define HCI_EV_MASK_RD_REM_SUPP_FEAT_HI     ((U32)0x00000000)
#define HCI_EV_MASK_RD_REM_SUPP_FEAT_LO     ((U32)0x00000400)
#define HCI_EV_MASK_RD_REM_EXT_FEAT_HI      ((U32)0x00000004)
#define HCI_EV_MASK_RD_REM_EXT_FEAT_LO      ((U32)0x00000000)
#define HCI_EV_MASK_RD_REM_VER_INFO_HI      ((U32)0x00000000)
#define HCI_EV_MASK_RD_REM_VER_INFO_LO      ((U32)0x00000800)
#define HCI_EV_MASK_QOS_SETUP_COMP_HI       ((U32)0x00000000)
#define HCI_EV_MASK_QOS_SETUP_COMP_LO       ((U32)0x00001000)
#define HCI_EV_MASK_CMD_COMP_HI             ((U32)0x00000000)
#define HCI_EV_MASK_CMD_COMP_LO             ((U32)0x00002000)
#define HCI_EV_MASK_CMD_STS_HI              ((U32)0x00000000)
#define HCI_EV_MASK_CMD_STS_LO              ((U32)0x00004000)
#define HCI_EV_MASK_HARDWARE_ERR_HI         ((U32)0x00000000)
#define HCI_EV_MASK_HARDWARE_ERR_LO         ((U32)0x00008000)
#define HCI_EV_MASK_FLUSH_OCCURRED_EV_HI    ((U32)0x00000000)
#define HCI_EV_MASK_FLUSH_OCCURRED_EV_LO    ((U32)0x00010000)
#define HCI_EV_MASK_ROLE_CHANGE_HI          ((U32)0x00000000)
#define HCI_EV_MASK_ROLE_CHANGE_LO          ((U32)0x00020000)
#define HCI_EV_MASK_NUM_HCI_DATA_PKTS_HI    ((U32)0x00000000)
#define HCI_EV_MASK_NUM_HCI_DATA_PKTS_LO    ((U32)0x00040000)
#define HCI_EV_MASK_MODE_CHANGE_HI          ((U32)0x00000000)
#define HCI_EV_MASK_MODE_CHANGE_LO          ((U32)0x00080000)
#define HCI_EV_MASK_RETURN_LINK_KEYS_HI     ((U32)0x00000000)
#define HCI_EV_MASK_RETURN_LINK_KEYS_LO     ((U32)0x00100000)
#define HCI_EV_MASK_PIN_CODE_REQ_HI         ((U32)0x00000000)
#define HCI_EV_MASK_PIN_CODE_REQ_LO         ((U32)0x00200000)
#define HCI_EV_MASK_LINK_KEY_REQ_HI         ((U32)0x00000000)
#define HCI_EV_MASK_LINK_KEY_REQ_LO         ((U32)0x00400000)
#define HCI_EV_MASK_LINK_KEY_NOTIFY_HI      ((U32)0x00000000)
#define HCI_EV_MASK_LINK_KEY_NOTIFY_LO      ((U32)0x00800000)
#define HCI_EV_MASK_LOOPBACK_CMD_HI         ((U32)0x00000000)
#define HCI_EV_MASK_LOOPBACK_CMD_LO         ((U32)0x01000000)
#define HCI_EV_MASK_DATA_BUFF_OVERFLOW_HI   ((U32)0x00000000)
#define HCI_EV_MASK_DATA_BUFF_OVERFLOW_LO   ((U32)0x02000000)
#define HCI_EV_MASK_MAX_SLOTS_CHANGE_HI     ((U32)0x00000000)
#define HCI_EV_MASK_MAX_SLOTS_CHANGE_LO     ((U32)0x04000000)
#define HCI_EV_MASK_RD_CLOCK_OFFSET_HI      ((U32)0x00000000)
#define HCI_EV_MASK_RD_CLOCK_OFFSET_LO      ((U32)0x08000000)
#define HCI_EV_MASK_CONN_PKT_TYPE_HI        ((U32)0x00000000)
#define HCI_EV_MASK_CONN_PKT_TYPE_LO        ((U32)0x10000000)
#define HCI_EV_MASK_QOS_VIOLATION_HI        ((U32)0x00000000)
#define HCI_EV_MASK_QOS_VIOLATION_LO        ((U32)0x20000000)
#define HCI_EV_MASK_PAGE_SCAN_MODE_HI       ((U32)0x00000000)
#define HCI_EV_MASK_PAGE_SCAN_MODE_LO       ((U32)0x40000000)
#define HCI_EV_MASK_PAGE_SCAN_REP_MODE_HI   ((U32)0x00000000)
#define HCI_EV_MASK_PAGE_SCAN_REP_MODE_LO   ((U32)0x80000000)
#define HCI_EV_MASK_FLOW_SPEC_COMP_HI        ((U32)0x00000001)
#define HCI_EV_MASK_FLOW_SPEC_COMP_LO        ((U32)0x00000000)
#define HCI_EV_MASK_INQUIRY_RES_RSSI_HI      ((U32)0x00000002)
#define HCI_EV_MASK_INQUIRY_RES_RSSI_LO      ((U32)0x00000000)
#define HCI_EV_MASK_RR_EXT_FEATR_HI          ((U32)0x00000004)
#define HCI_EV_MASK_RR_EXT_FEATR_LO          ((U32)0x00000000)
#define HCI_EV_MASK_FIXED_ADDR_HI            ((U32)0x00000008)
#define HCI_EV_MASK_FIXED_ADDR_LO            ((U32)0x00000000)
#define HCI_EV_MASK_ALIAS_ADDR_HI            ((U32)0x00000010)
#define HCI_EV_MASK_ALIAS_ADDR_LO            ((U32)0x00000000)
#define HCI_EV_MASK_GENERATE_ALIAS_REQ_HI    ((U32)0x00000020)
#define HCI_EV_MASK_GENERATE_ALIAS_REQ_LO    ((U32)0x00000000)
#define HCI_EV_MASK_ACT_ADDR_HI              ((U32)0x00000040)
#define HCI_EV_MASK_ACT_ADDR_LO              ((U32)0x00000000)
#define HCI_EV_MASK_ALLOW_PRIVATE_PAIRING_HI ((U32)0x00000080)
#define HCI_EV_MASK_ALLOW_PRIVATE_PAIRING_LO ((U32)0x00000000)
#define HCI_EV_MASK_ALIAS_ADDR_REQ_HI        ((U32)0x00000100)
#define HCI_EV_MASK_ALIAS_ADDR_REQ_LO        ((U32)0x00000000)
#define HCI_EV_MASK_ALIAS_NOT_RECOGNISED_HI  ((U32)0x00000200)
#define HCI_EV_MASK_ALIAS_NOT_RECOGNISED_LO  ((U32)0x00000000)
#define HCI_EV_MASK_FIXED_ADDR_ATTMPT_HI     ((U32)0x00000400)
#define HCI_EV_MASK_FIXED_ADDR_ATTMPT_LO     ((U32)0x00000000)
#define HCI_EV_MASK_SYNC_CONN_COMP_HI        ((U32)0x00000800)
#define HCI_EV_MASK_SYNC_CONN_COMP_LO        ((U32)0x00000000)
#define HCI_EV_MASK_SYNC_CONN_CHANGED_HI     ((U32)0x00001000)
#define HCI_EV_MASK_SYNC_CONN_CHANGED_LO     ((U32)0x00000000)
#define HCI_EV_MASK_SNIFF_SUB_RATE_HI        ((U32)0x00002000)
#define HCI_EV_MASK_SNIFF_SUB_RATE_LO        ((U32)0x00000000)

#define HCI_EV_MASK_DEFAULT_HI               ((U32)0x00003FFF)
#define HCI_EV_MASK_DEFAULT_LO               ((U32)0xFFFFFFFF)

/* auto acpt vals */
#define HCI_AUTO_ACPT_OFF                 ((U8)0x01)
#define HCI_AUTO_ACPT_ON                  ((U8)0x02)
#define HCI_AUTO_ACPT_WITH_ROLE_SWITCH    ((U8)0x03)

/* inquiry vals :   HCI_INQUIRY, HCI_PERIODIC_INQUIRY_MODE,HCWI_WR_INQUIRYSCAN_ACTIVITY */
#define HCI_INQUIRY_LEN_MIN              ((U8)0x01)
#define HCI_INQUIRY_LEN_MAX              ((U8)0x30)
#define HCI_INQUIRY_RSPS_MIN             ((U8)0x01)
#define HCI_INQUIRY_RSPS_MAX             ((U8)0xFF)
#define HCI_INQUIRY_MAX_PERIOD_MIN       ((U16)0x0003)
#define HCI_INQUIRY_MAX_PERIOD_MAX       ((U16)0xFFFF)
#define HCI_INQUIRY_MIN_PERIOD_MIN       ((U16)0x0002)
#define HCI_INQUIRY_MIN_PERIOD_MAX       ((U16)0xFFFE)
#define HCI_INQUIRYSCAN_INTVL_MIN        ((U16)0x0012)
#define HCI_INQUIRYSCAN_INTVL_DEFAULT    ((U16)0x0800)
#define HCI_INQUIRYSCAN_INTVL_MAX        ((U16)0x1000)
#define HCI_INQUIRYSCAN_WINDOW_MIN       ((U16)0x0012)
#define HCI_INQUIRYSCAN_WINDOW_DEFAULT   ((U16)0x0012)
#define HCI_INQUIRYSCAN_WINDOW_MAX       ((U16)0x1000)

#ifndef HCI_MAX_INQUIRY_RESS
#define HCI_MAX_INQUIRY_RESS 1 /* max rsps in inquiry res msg */
#endif

/* scan enb vals  */
#define HCI_SCAN_ENB_OFF             ((U8)0x00) /* default */
#define HCI_SCAN_ENB_INQ             ((U8)0x01)
#define HCI_SCAN_ENB_PAGE            ((U8)0x02)
#define HCI_SCAN_ENB_INQ_AND_PAGE    ((U8)0x03)

/* PURPOSE page and conn acpt timer defaults */
#define HCI_DEFAULT_PAGE_TIMEOUT        0x2000
#define HCI_DEFAULT_CONN_ACPT_TIMEOUT   0x1FA0

/* HCI CREATE CONN boundary conditions */
#define HCI_DO_NOT_ALLOW_ROLE_SWITCH    ((U8)0x00)
#define HCI_ALLOW_ROLE_SWITCH           ((U8)0x01)

/* HCI ACPT CONN REQ role vals */
#define HCI_ROLE_BECOME_MASTER          ((U8)0x00)
#define HCI_ROLE_STAY_SLAVE             ((U8)0x01)

/* auth enb vals */
#define HCI_AUTH_ENB_OFF                ((U8)0x00)
#define HCI_AUTH_ENB_ON                 ((U8)0x01) /* for all conns */

/* encryption mode vals */
#define HCI_ENC_OFF                     ((U8)0x00)
#define HCI_ENC_ON                      ((U8)0x01)

#define HCI_ENC_MODE_OFF                ((U8)0x00)
#define HCI_ENC_MODE_PT_TO_PT           ((U8)0x01)
#define HCI_ENC_MODE_PT_TO_PT_AND_BCAST ((U8)0x02)/* ? */

#ifdef CFG_BT_VER_21
/* hci  even mask vals */
#define HCI_EV_MASK_LOW                     ((U32)0xFFFB9FFF)
#define HCI_EV_MASK_HIGH                     ((U32)0x1DBFF807)
#endif


/* voice setting mask vals */
#define HCI_VOICE_INPUT_MASK            ((U16)0x0300)
#define HCI_VOICE_INPUT_LIN             ((U16)0x0000)
#define HCI_VOICE_INPUT_MU_LAW          ((U16)0x0100)
#define HCI_VOICE_INPUT_A_LAW           ((U16)0x0200)
#define HCI_VOICE_FMT_MASK              ((U16)0x00C0)
#define HCI_VOICE_FMT_1SCOMP            ((U16)0x0000)
#define HCI_VOICE_FMT_2SCOMP            ((U16)0x0040)
#define HCI_VOICE_FMT_SMAG              ((U16)0x0080)
#define HCI_VOICE_SAMP_SIZE_MASK        ((U16)0x0020)
#define HCI_VOICE_SAMP_SIZE_8BIT        ((U16)0x0000)
#define HCI_VOICE_SAMP_SIZE_16BIT       ((U16)0x0020)
#define HCI_VOICE_LINEAR_PGAP_MASK      ((U16)0x001C)
#define HCI_VOICE_AIR_CODING_MASK       ((U16)0x0003)
#define HCI_VOICE_AIR_CODING_CVSD       ((U16)0x0000)
#define HCI_VOICE_AIR_CODING_MU_LAW     ((U16)0x0001)
#define HCI_VOICE_AIR_CODING_A_LAW      ((U16)0x0002)
#define HCI_VOICE_TRANSPARENT_DATA      ((U16)0x0003)

#define HCI_VOICE_SETTINGS_DEFAULT      ((U16)0x0060)
#define HCI_VOICE_SETTING_MAX_VAL       ((U16)0x03FF) /* 10 bts2g_bits meaningful */

/*  wr automatic flush timeout vals  */
#define HCI_MAX_FLUSH_TIMEOUT           ((U16)0x07FF)

/* hold mode activity vals */
#define HCI_HOLD_CURR_PWR_ST            ((U8)0x00)
#define HCI_HOLD_SUSPEND_PAGE_SCAN      ((U8)0x01)
#define HCI_HOLD_SUSPEND_INQ_SCAN       ((U8)0x02)
#define HCI_HOLD_SUSPEND_PER_INQ        ((U8)0x04)

#define HCI_HOLD_MIN_PERIOD             ((U16)0x01)

/* sniff mode activity vals */
#define HCI_SNIFF_MAX_INTVL_MIN           ((U16)0x01)
#define HCI_SNIFF_MIN_INTVL_MIN           ((U16)0x01)
#define HCI_SNIFF_ATTMPT_MIN              ((U16)0x01)
#define HCI_SNIFF_ATTMPT_MAX              ((U16)0x7FFF)
#define HCI_SNIFF_TIMEOUT_MIN             ((U16)0x00)
#define HCI_SNIFF_TIMEOUT_MAX             ((U16)0x7FFF)
#define HCI_SNIFF_SUB_RATE_LATENCY_MAX    ((U16)0x9FFF)
#define BTS2S_HCI_SNIFF_SUB_RATEIMEOUT_MAX ((U16)0x9FFF)

/* host ctrller chnl clsification mode vals */
#define HCI_CHNL_CLS_MODE_DISB  ((U8)0x00)
#define HCI_CHNL_CLS_MODE_ENB   ((U8)0x01)

/*  HCI version infmtion vals */
#define HCI_VER_1_0               ((U8)0x00)
#define HCI_VER_1_1               ((U8)0x01)
#define HCI_VER_1_2               ((U8)0x02)

/* HCI country code vals */
#define HCI_CO_CODE_NA_AND_EUR     ((U8)0x00)
#define HCI_CO_CODE_FRANCE         ((U8)0x01)
#define HCI_CO_CODE_SPAIN          ((U8)0x02)
#define HCI_CO_CODE_JAPAN          ((U8)0x03)

/* HCI cur dev mode vals */
#define HCI_BT_MODE_ACT            ((U8)0x00)
#define HCI_BT_MODE_HOLD           ((U8)0x01)
#define HCI_BT_MODE_SNIFF          ((U8)0x02)
#define HCI_BT_MODE_PARK           ((U8)0x03)
#define HCI_BT_MODE_SCATTERMODE    ((U8)0x04)

/* HCI test vals */
#define HCI_GEN_SELF_TEST          ((U8)0x00)
#define HCI_LOOPBACK_OFF           ((U8)0x00)
#define HCI_LOCAL_LOOPBACK         ((U8)0x01)
#define HCI_RMT_LOOPBACK           ((U8)0x02)
#define HCI_LOOPBACK_MODE_MAX      ((U8)0x03)

/* HCI link type vals */
#define HCI_LINK_TYPE_SCO          ((U8)0x00)
#define HCI_LINK_TYPE_ACL          ((U8)0x01)
#define HCI_LINK_TYPE_ESCO         ((U8)0x02)
#define HCI_LINK_TYPE_DONT_CARE    ((U8)0x03)

/* HCI page scan repetition mode vals */
#define HCI_PAGE_SCAN_REP_MODE_R0     ((U8)0x00)
#define HCI_PAGE_SCAN_REP_MODE_R1     ((U8)0x01)
#define HCI_PAGE_SCAN_REP_MODE_R2     ((U8)0x02)

/* HCI page scan mode vals */
#define HCI_PAGE_SCAN_MODE_MANDATORY      ((U8)0x00)
#define HCI_PAGE_SCAN_MODE_OPTIONAL_1     ((U8)0x01)
#define HCI_PAGE_SCAN_MODE_OPTIONAL_2     ((U8)0x02)
#define HCI_PAGE_SCAN_MODE_OPTIONAL_3     ((U8)0x03)
#define HCI_PAGE_SCAN_MODE_DEFAULT        HCI_PAGE_SCAN_MODE_MANDATORY

/* HCI page scan intvl : HCI_WR_PAGESCAN_ACTIVITY */
#define HCI_PAGESCAN_INTVL_MIN       ((U16)0x12)
#define HCI_PAGESCAN_INTVL_DEFAULT   ((U16)0x800)
#define HCI_PAGESCAN_INTVL_MAX       ((U16)0x1000)

/* HCI page scan window : HCI_WR_PAGESCAN_ACTIVITY */
#define HCI_PAGESCAN_WINDOW_MIN       ((U16)0x12)
#define HCI_PAGESCAN_WINDOW_DEFAULT   ((U16)0x12)
#define HCI_PAGESCAN_WINDOW_MAX       ((U16)0x1000)

/* HCI page scan PERIOD : HCI_WR_PAGESCAN_PERIOD_MODE */
#define HCI_PAGESCAN_PERIOD_MODE_P0         ((U8)0x00)
#define HCI_PAGESCAN_PERIOD_MODE_P1         ((U8)0x01)
#define HCI_PAGESCAN_PERIOD_MODE_P2         ((U8)0x02)
#define HCI_PAGESCAN_PERIOD_MODE_DEFAULT    HCI_PAGESCAN_PERIOD_MODE_P0

/* HCI page and inquiry scan type : HCI_WR_PAGE_SCAN_TYPE
   HCI_WR_INQUIRY_SCAN_TYPE */
#define HCI_SCAN_TYPE_LEGACY           ((U8)0x00)
#define HCI_SCAN_TYPE_INTERLACED       ((U8)0x01)

/* HCI inquiry mode : HCI_WR_INQUIRY_MODE */
#define HCI_INQUIRY_MODE_STANDARD      ((U8)0x00)
#define HCI_INQUIRY_MODE_WITH_RSSI     ((U8)0x01)
#define HCI_INQUIRY_MODE_WITH_EXT     ((U8)0x02)

/* HCI clock offset vals */
#define HCI_CLOCK_OFFSET_MASK     ((U16)0x7fff)
#define HCI_CLOCK_OFFSET_VLD_MASK ((U16)0x8000)
#define HCI_CLOCK_OFFSET_INVLD    ((U16)0x0000)
#define HCI_CLOCK_OFFSET_VLD      ((U16)0x8000)

/* HCI link policy settings */
#define DISB_ALL_LM_MODES         ((U16)0x0000)
#define ENB_MS_SWITCH             ((U16)0x0001)
#define ENB_HOLD                  ((U16)0x0002)
#define ENB_SNIFF                 ((U16)0x0004)
#define ENB_PARK                  ((U16)0x0008)
#define ENB_SCATTER_MODE          ((U16)0x0010)

/* HCI filter types */
#define CLEAR_ALL_FILTERS         ((U8)0x00)
#define INQUIRY_RES_FILTER        ((U8)0x01)
#define CONN_FILTER               ((U8)0x02)

/* HCI filter condition types */
#define NEW_DEV_RESPONDED         ((U8)0x00)
#define ALL_CONN                  ((U8)0x00)
#define RSP_DEV_CLS               ((U8)0x01)
#define ADDRED_DEV_RESPONDED      ((U8)0x02)

/* HCI pin types */
#define HCI_VARIABLE_PIN    ((U8)0x00)
#define HCI_FIXED_PIN       ((U8)0x01)

/* HCI pin code len */
#define HCI_MIN_PIN_LEN  ((U8)0x01)
#define HCI_MAX_PIN_LEN  ((U8)0x10)

/* size of link keys */

/* HCI rd stored link key rd all flag types */
#define RETURN_BDADDR   ((U8)0x00)
#define RETURN_ALL      ((U8)0x01)

/* HCI del stored link key rd all flag types */
#define DEL_BDADDR   ((U8)0x00)
#define DEL_ALL      ((U8)0x01)

/* HCI IAC LAP boundary vals and other vals */
#define HCI_NUM_CUR_IAC_MIN   ((U8)0x01)
#define HCI_NUM_CUR_IAC_MAX   ((U8)0x40)
#define HCI_IAC_LAP_MIN       ((U32)0x9E8B00)
#define HCI_IAC_LAP_MAX       ((U32)0x9E8B3F)

#define HCI_INQ_CODE_GIAC  ((U32)0x9e8b33)
//#define HCI_INQ_CODE_GIAC  ((U32)0x9E8B00)

/* HCI conn acpt timeout vals */
#define HCI_CONN_ACPT_TIMEOUT_MIN   ((U16)0x01)
#define HCI_CONN_ACPT_TIMEOUT_MAX   ((U16)0x0B540)

/* HCI hdl range */
#define HCI_HANDLE_MAX   ((U16)0x0EFF)
#define HCI_HANDLE_INVLD ((U16)0xffff)

/* HCI link supvisn timeout vals */
#define HCI_LINK_SUPVISN_INFINITY   ((U16)0x0000)
#define HCI_LINK_SUPVISN_MIN        ((U16)0x0001)
#define HCI_LINK_SUPVISN_DEFAULT    ((U16)0x7D00)
#define HCI_LINK_SUPVISN_MAX        ((U16)0xFFFF)

/* HCI transmit power type vals */
#define HCI_RD_CUR_TX_POWER       ((U8)0x00)
#define HCI_RD_MAX_TX_POWER           ((U8)0x01)

/* HCI SCO flow ctrl type vals */
#define HCI_SCO_FLOW_CTRL_DISBD   ((U8)0x00)
#define HCI_SCO_FLOW_CTRL_ENBD    ((U8)0x01)

/* HCI HC to H flow ctrl type vals */
#define HCI_HCITOH_FLOW_CTRL_DISBD            ((U8)0x00)
#define HCI_HCITOH_FLOW_CTRL_ENBD_ACL_ONLY    ((U8)0x01)
#define HCI_HCITOH_FLOW_CTRL_ENBD_SCO_ONLY    ((U8)0x02)
#define HCI_HCITOH_FLOW_CTRL_ENBD_ACL_AND_SCO ((U8)0x03)

/* HCI wr stored link key, HCI return link keys ev,
   maximum num of keys that can be present
   in a wr stored link key. max cmd par size= 255 bytes.
   num keys par = 1 byte
   (BD_ADDR + key) = (6 + 16) = X
   max keys = (255 - 1) / X = 10 */
//#define HCI_STORED_LINK_KEY_MAX             ((U8)0xB)
#define HCI_STORED_LINK_KEY_MAX             ((U8)0x1)
#define HCI_RETURN_LINK_KEY_VAR_ARG_SIZE    22

/* HCI link key noti ev key type */
#define HCI_COMBINATION_KEY                 ((U8)0)
#define HCI_LOCAL_UNIT_KEY                  ((U8)1)
#define HCI_RMT_UNIT_KEY                    ((U8)2)
#define HCI_KEY_TYPE_UNKNOWN                ((U8)0xFF)

/* supp featr definitions */
/* byte 0 featr */
#define HCI_FEATURE_3_SLOT_PKTS             ((U8)0x01)
#define HCI_FEATURE_5_SLOT_PKTS             ((U8)0x02)
#define HCI_FEATURE_ENCRYPTION              ((U8)0x04)
#define HCI_FEATURE_SLOT_OFFSET             ((U8)0x08)
#define HCI_FEATURE_TIMING_ACCURACY         ((U8)0x10)
#define HCI_FEATURE_SWITCH                  ((U8)0x20)
#define HCI_FEATURE_HOLD_MODE               ((U8)0x40)
#define HCI_FEATURE_SNIFF_MODE              ((U8)0x80)
/* byte 1 featr */
#define HCI_FEATURE_PARK_MODE               ((U8)0x01)
#define HCI_FEATURE_RSSI                    ((U8)0x02)
#define HCI_FEATURE_CQD_DATA_RATE           ((U8)0x04)
#define HCI_FEATURE_SCO_LINK                ((U8)0x08)
#define HCI_FEATURE_HV2_PKTS                ((U8)0x10)
#define HCI_FEATURE_HV3_PKTS                ((U8)0x20)
#define HCI_FEATURE_U_LAW_LOG               ((U8)0x40)
#define HCI_FEATURE_A_LAW_LOG               ((U8)0x80)

/* byte 2 */
#define EXT_FEAT_CVSD_SYNCHRONOUS_DATA        0x0001
#define EXT_FEAT_PAGING_PAR_NEGOTIATION       0x0002
#define EXT_FEAT_POWER_CTRL                   0x0004
#define EXT_FEAT_TRANSPARENT_SYNCHRONOUS_DATA 0x0008
#define EXT_FEAT_FLOW_CTRL_LAG_LSB            0x0010
#define EXT_FEAT_FLOW_CTRL_LAG_MB             0x0020
#define EXT_FEAT_FLOW_CTRL_LAG_MSB            0x0040
#define EXT_FEAT_BROADCAST_ENCRYPTION         0x0080

/* byte 3 */
/* rsv 0x0100 */
#define EXT_FEAT_EDR_ACL_2_MBPS_MODE          0x0200
#define EXT_FEAT_EDR_ACL_3_MBPS_MODE          0x0400
#define EXT_FEAT_ENHANCED_INQUIRY_SCAN        0x0800
#define EXT_FEAT_INTERLACED_INQUIRY_SCAN      0x1000
#define EXT_FEAT_INTERLACED_PAGE_SCAN         0x2000
#define EXT_FEAT_RSSI_WITH_INQUIRY_RESS       0x4000
#define EXT_FEAT_EV3_PKTS                     0x8000

/* byte 4 */
#define EXT_FEAT_EV4_PKTS                     0x0001
#define EXT_FEAT_EV5_PKTS                     0x0002
/* rsv                                        0x0004 */
#define EXT_FEAT_AFH_CAPABLE_SLAVE            0x0008
#define EXT_FEAT_AFH_CLSIFICATION_SLAVE       0x0010
/* rsv                                        0x0020 */
/* rsv                                        0x0040 */
#define EXT_FEAT_3_SLOT_EDR_ACL_PKTS          0x0080

/* byte 5 */
#define EXT_FEAT_5_SLOT_EDR_ACL_PKTS          0x0100
/* rsv                                        0x0200 */
/* rsv                                        0x0400 */
#define EXT_FEAT_AFH_CAPABLE_MASTER           0x0800
#define EXT_FEAT_AFH_CLSIFICATION_MASTER      0x1000
#define EXT_FEAT_EDR_ESCO_2_MBPS_MODE         0x2000
#define EXT_FEAT_EDR_ESCO_3_MBPS_MODE         0x4000
#define EXT_FEAT_3_SLOT_ENHANCED_DATA_RATE    0x8000

/* byte 6 */
/* rsv                                     0x0001 */
/* rsv                                     0x0002 */
/* rsv                                     0x0004 */
#define EXT_FEAT_SSP_SUPP                  0x0008
/* rsv                                     0x0010 */
/* rsv                                     0x0020 */
/* rsv                                     0x0040 */
/* rsv                                     0x0080 */

/* byte 7 */
/* rsv                                     0x0100 */
/* rsv                                     0x0200 */
/* rsv                                     0x0400 */
/* rsv                                     0x0800 */
/* rsv                                     0x1000 */
/* rsv                                     0x2000 */
#define EXT_FEAT_SUPP_BIT                  0x8000

#define EXT_SIMPLE_PAIRING_MODE_ENABLED    0x01

/* LM SCO hdl type. curly not available through the HCI ui */
#define HCI_LM_SCO_HANDLE_INVLD       ((U8)0x00)

#define HCI_ARC_CONVERT(input_struct)        (sizeof(input_struct) *sizeof(U8)/ sizeof(U16))

#define HCI_NAME_LEN                         248
#define HCI_LOCAL_NAME_BYTES_PER_PTR         HCI_VAR_ARG_POOL_SIZE
#define HCI_LOCAL_NAME_BYTE_PKT_PTRS         ((HCI_NAME_LEN + HCI_LOCAL_NAME_BYTES_PER_PTR - 1) / HCI_LOCAL_NAME_BYTES_PER_PTR)

#define HCI_BYTE_SIZE_OFHANDLE_COMP          4
#define HCI_EV_HANDLE_COMPS_PER_PTR          (HCI_VAR_ARG_POOL_SIZE / HCI_ARC_CONVERT(BTS2S_HANDLE_COMP))
#define HCI_EV_NUM_HANDLE_COMP_PKT_PTRS      (((254 / HCI_BYTE_SIZE_OFHANDLE_COMP) + HCI_EV_HANDLE_COMPS_PER_PTR - 1) / HCI_EV_HANDLE_COMPS_PER_PTR)

#define HCI_HOST_NUM_COMPD_PKTS_PER_PTR      HCI_EV_HANDLE_COMPS_PER_PTR
#define HCI_HOST_NUM_COMPD_PKT_PTRS          HCI_EV_NUM_HANDLE_COMP_PKT_PTRS

#define HCI_BYTE_SIZE_OFHCI_INQ_RES          14

#define HCI_MAX_INQ_RES_PER_PTR              (HCI_VAR_ARG_POOL_SIZE / HCI_ARC_CONVERT(BTS2S_HCI_INQ_RES))
#define HCI_MAX_INQ_RES_PTRS                 (((254 / HCI_BYTE_SIZE_OFHCI_INQ_RES) + HCI_MAX_INQ_RES_PER_PTR - 1) / HCI_MAX_INQ_RES_PER_PTR)


#define HCI_BYTE_SIZE_OF_HCI_IAC_LAP         3
#define HCI_IAC_LAP_PER_PTR                  (HCI_VAR_ARG_POOL_SIZE / HCI_ARC_CONVERT(U24))
#define HCI_IAC_LAP_PTRS                     (((254 / HCI_BYTE_SIZE_OF_HCI_IAC_LAP) + HCI_IAC_LAP_PER_PTR - 1) / HCI_IAC_LAP_PER_PTR)

#define HCI_EV_PKT_LEN                       255
#define HCI_LOOPBACK_BYTES_PER_PTR           HCI_VAR_ARG_POOL_SIZE
#define HCI_LOOPBACK_BYTE_PKT_PTRS           ((HCI_EV_PKT_LEN + HCI_LOOPBACK_BYTES_PER_PTR - 1) / HCI_LOOPBACK_BYTES_PER_PTR)

#define HCI_RD_SUPP_CMDS_EV_LEN              64
#define HCI_RD_SUPP_CMDS_PER_PTR             HCI_VAR_ARG_POOL_SIZE
#define HCI_RD_SUPP_CMDS_PKT_PTRS            ((HCI_RD_SUPP_CMDS_EV_LEN + HCI_RD_SUPP_CMDS_PER_PTR - 1) / HCI_RD_SUPP_CMDS_PER_PTR)

#define HCI_LOCAL_CLOCK      0
#define HCI_PICONET_CLOCK    1

typedef struct
{
    U16 op_code; /* op code of cmd */
    U8  len;     /* par total len  */
} BTS2S_HCI_CMD_COMMON;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_WITH_CH_COMMON;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_WITH_BD_COMMON;

typedef struct
{
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_BD_ADDR_GEN_RET;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_WITH_BD_ADDR_RET;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_WITH_HANDLE_RET;

typedef struct
{
    U16 op_code;/* op code of cmd */
    U8  len;    /* par total len */
    U8  data;   /* dummy field used to determine addr of par data in an arbitrary HCI cmd */
} BTS2S_HCI_CMD_GEN;

typedef struct
{
    U8 ev_code; /* ev code */
    U8 len;     /* par total len */
} BTS2S_HCI_EV_COMMON;

typedef struct
{
    U8  data[EXTERN_INQUIRY_RSP_LEN]; /* upper addr part 24..31 */
} BTS2S_HCI_EV_EXT_INQ_RES;

typedef struct
{
    U16 hdl_plus_flag;  /* hdl is 12 lsb, flag 4 msb */
    U16 len;            /* data total len */
} BTS2S_HCI_ACL_DATA;

typedef struct
{
    U16 hdl; /* hdl is 12 lsb */
    U8  len; /* data total len */
} BTS2S_HCI_SCO_DATA;

typedef struct
{
    U16  hdl;
    U16  num_compd;

} BTS2S_HANDLE_COMP;

typedef struct
{
    BTS2S_BD_ADDR  bd; /* bluetooth dev addr */
    U8 link_key[LINK_KEY_SIZE]; /* link key */
} LINK_KEYBD_ADDR;

typedef struct
{
    BTS2S_HCI_CMD_COMMON  common;
    U24 lap; /* lower 3 bytes used only */
    U8  inquiry_len;
    U8  rsp_num;
} BTS2S_HCI_INQUIRY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_INQUIRY_ESC;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 max_period_len; /* non zero, max_period_len > min_period_len */
    U16 min_period_len; /* non zero, min_period_len > inquiry_len */
    U24 lap; /* lower 3 bytes used only */
    U8  inquiry_len;
    U8  rsp_num;
} BTS2S_HCI_PERIODIC_INQUIRY_MODE;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_EXIT_PERIODIC_INQUIRY_MODE;


/* create conn cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U16 pkt_type;
    U8 page_scan_rep_mode;
    U8 page_scan_mode;
    U16 clock_offset;
    U8 allow_role_switch;
} BTS2S_HCI_CREATE_CONN;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    U8  reason;
} BTS2S_HCI_DISC;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U16 pkt_type;
} BTS2S_HCI_ADD_SCO_CONN;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_CREATE_CONN_ESC;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_CREATE_CONN_ESC_RET;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 role;
} BTS2S_HCI_ACPT_CONN_REQ;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 reason;
} BTS2S_HCI_REJ_CONN_REQ;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 key_val[LINK_KEY_SIZE];
} BTS2S_HCI_LINK_KEY_REQ_REPLY;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_LINK_KEY_REQ_REPLY_RET;

typedef struct
{
    BTS2S_HCI_CMD_COMMON  common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_LINK_KEY_REQ_NEG_REPLY;

typedef BTS2S_HCI_LINK_KEY_REQ_REPLY_RET BTS2S_HCI_LINK_KEY_REQ_NEG_REPLY_RET;

typedef struct
{
    BTS2S_HCI_CMD_COMMON  common;
    BTS2S_BD_ADDR bd;
    U8 pin_code_len; /* in bytes, 1 to 16 decimal */
    U8 pin[HCI_MAX_PIN_LEN];      /* 8 16bit words. */
} BTS2S_HCI_PIN_CODE_REQ_REPLY;

typedef BTS2S_HCI_LINK_KEY_REQ_REPLY_RET BTS2S_HCI_PIN_CODE_REQ_REPLY_RET;

/* PIN code req negative reply cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_PIN_CODE_REQ_NEG_REPLY;

typedef BTS2S_HCI_LINK_KEY_REQ_REPLY_RET BTS2S_HCI_PIN_CODE_REQ_NEG_REPLY_RET;

/* change conn pkt type cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    U16 pkt_type;
} BTS2S_HCI_CHANGE_CONN_PKT_TYPE;

/* auth reqed cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
} BTS2S_HCI_AUTH_REQ;

/* set conn encryption cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    U8 enc_enb; /* 0 = off, 1 = on */
} BTS2S_HCI_SET_CONN_ENCRYPTION;

/* change conn link key cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_CHANGE_CONN_LINK_KEY;

/* master link key cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 link_key_type; /* 0 = regular link key, 1 = tmp link key */
}  BTS2S_HCI_MASTER_LINK_KEY;

/* rmt name req cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON   common;
    BTS2S_BD_ADDR          bd;

    /* these fields are filled in by the dev manager */
    U8 page_scan_rep_mode;
    U8 page_scan_mode;
    U16 clock_offset;
} BTS2S_HCI_RMT_NAME_REQ;

/* rmt name req esc cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RMT_NAME_REQ_ESC;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RMT_NAME_REQ_ESC_RET;

/* rd rmt supp featr cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_RD_RMT_SUPP_FEATR;


/* rd rmt extended featr cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8  page_num;
} BTS2S_HCI_RD_RMT_EXT_FEATR;

/* rd rmt version information cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
}  BTS2S_HCI_RD_RMT_VER_INFO;

/* rd clock offset cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_RD_CLOCK_OFFSET;

/* rd SCO LMP hdl cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    /* note - there is NO  BD_ADDR for this message
       because the hdl passed in is actually a SCO
       hdl which an app will obtain by monitoring
       SCO indication */
} BTS2S_HCI_RD_LMP_HANDLE;

/* cmd comp arg */
typedef struct
{
    U16 hdl;
    U8 lmp_hdl;
    U32 rsv;
} BTS2S_HCI_RD_LMP_HANDLE_RET;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
} BTS2S_HCI_EXCHANGE_FIXED_INFO;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR local_alias_addr;
} BTS2S_HCI_EXCHANGE_ALIAS_INFO;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
} BTS2S_HCI_PRIVATE_PAIRING_REQ_REPLY;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_PRIVATE_PAIRING_REQ_REPLY_RET;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
} BTS2S_HCI_PRIVATE_PAIRING_REQ_NEG_REPLY;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_PRIVATE_PAIRING_REQ_NEG_REPLY_RET;


/* anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd_alias;
} BTS2S_HCI_GENERATED_ALIAS;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_GENERATED_ALIAS_RET;


/* anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    BTS2S_BD_ADDR bd_alias;
} BTS2S_HCI_ALIAS_ADDR_REQ_REPLY;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_ALIAS_ADDR_REQ_REPLY_RET;


/* anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_ALIAS_ADDR_REQ_NEG_REPLY;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_ALIAS_ADDR_REQ_NEG_REPLY_RET;


/* rd setup synchronous conn cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U32 tx_bdw;
    U32 rx_bdw;
    U16 max_latency;
    U16 voice_setting;
    U8  retx_effort;
    U16 pkt_type;
} BTS2S_HCI_SETUP_SYNCHRONOUS_CONN;


typedef struct
{
    U32      tx_bw;                // Transmit Bandwidth (in B/sec)
    U32      rx_bw;                // Receive Bandwidth (in B/sec)
    U8       tx_cod_fmt[5];        // Transmit Coding Format
    U8       rx_cod_fmt[5];        // Receive Coding Format
    U16      tx_cod_fr_sz;         // Transmit Codec Frame Size (in B)
    U16      rx_cod_fr_sz;         // Receive Codec Frame Size (in B)
    U32      in_bw;                // Input Bandwidth (in B/sec)
    U32      out_bw;               // Output Bandwidth (in B/sec)
    U8       in_cod_fmt[5];        // Input Coding Format
    U8       out_cod_fmt[5];       // Output Coding Format
    U16      in_cod_data_sz;       // Input Coded Data Size (in bits)
    U16      out_cod_data_sz;      // Output Coded Data Size (in bits)
    U8       in_data_fmt;          // Input PCM Data Format
    U8       out_data_fmt;         // Output PCM Data Format
    U8       in_msb_pos;           // Input PCM Sample Payload MSB Position (in bits)
    U8       out_msb_pos;          // Output PCM Sample Payload MSB Position (in bits)
    U8       in_data_path;         // Input Data Path
    U8       out_data_path;        // Output Data Path
    U8       in_tr_unit_sz;        // Input Transport Unit Size (in bits)
    U8       out_tr_unit_sz;       // Output Transport Unit Size (in bits)
    U16      max_lat;              // Max Latency (in ms)
    U16      packet_type;          // Packet Type
    U8       retx_eff;             // Retransmission Effort
} BTS2S_HCI_ENH_SYNCHRONOUS_CONN_PARA;


/*Enhanced Setup Synchronous Connection command */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16      conhdl;               // Connection Handle
    BTS2S_BD_ADDR bd;
    BTS2S_HCI_ENH_SYNCHRONOUS_CONN_PARA para;

} BTS2S_HCI_ENH_SETUP_SYNCHRONOUS_CONN;

/* acpt synchronous conn cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U32 tx_bdw;
    U32 rx_bdw;
    U16 max_latency;
    U16 voice_setting;
    U8 retx_effort;
    U16 pkt_type;
} BTS2S_HCI_ACPT_SYNCHRONOUS_CONN_REQ;





typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR     bd_addr;       // BD address
    BTS2S_HCI_ENH_SYNCHRONOUS_CONN_PARA para;

} BTS2S_HCI_ENH_ACPT_SYNCHRONOUS_CONN_REQ;



/* rej synchronous conn cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 reason;
} BTS2S_HCI_REJ_SYNCHRONOUS_CONN_REQ;

/* hold mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
    U16 max_intvl;
    U16 min_intvl;
} BTS2S_HCI_HOLD_MODE;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_USR_CONF_REQ_REPLY;

typedef BTS2S_HCI_USR_CONF_REQ_REPLY BTS2S_HCI_USR_CONF_REQ_NEG_REPLY;

#ifdef CFG_BT_VER_21
/* IO Capability Request Reply conn cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 io_capability;
    U8 oob_data_present;
    U8 auth_req;
} BTS2S_HCI_IO_CAPABILITY_REQ_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U32 passky_value;
} BTS2S_HCI_USR_PASSKEY_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_USR_PASSKEY_NEG_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 hash_c[16]; /* Simple Pairing Hash C */
    U8 rand_r[16]; /* Simple Pairing Randomizer R */
} BTS2S_HCI_RMT_OOB_DATA_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RMT_OOB_DATA_NEG_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 reason;
} BTS2S_HCI_IO_CAPABILITY_REQ_NEG_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 notifi_type;
} BTS2S_HCI_KEYPRESS_NOTIFICATION;

#endif
/* sniff mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U16 max_intvl;
    U16 min_intvl;
    U16 attmpt;
    U16 timeout;
} BTS2S_HCI_SNIFF_MODE;


/* exit sniff mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EXIT_SNIFF_MODE;




/* qoS setup cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8 flag; /* rsv */
    U8 svc_type;
    U32 token_rate; /* in bytes per second */
    U32 peak_bandwidth; /* peak bandwidth in bytes per sec */
    U32 latency; /* in microseconds */
    U32 delay_variation; /* in microseconds */
} BTS2S_HCI_QOS_SETUP;


/* role discov cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_ROLE_DISCOV;

typedef struct
{
    U16 hdl;
    U8 cur_role;
} BTS2S_HCI_ROLE_DISCOV_RET;


/* switch role cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 role;
} BTS2S_HCI_SWITCH_ROLE;


/* rd link policy settings cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RD_LINK_POLICY_SETTINGS;

typedef struct
{
    U16 hdl;
    U16 link_policy_setting;
} BTS2S_HCI_RD_LINK_POLICY_SETTINGS_RET;


/* wr link policy settings cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
    U16  link_policy_setting;
} BTS2S_HCI_WR_LINK_POLICY_SETTINGS;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_WR_LINK_POLICY_SETTINGS_RET;

/* rd default link policy settings cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS;

typedef struct
{
    U16  default_lps;
} BTS2S_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_RET;


/* wr default link policy settings cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16  default_lps;
} BTS2S_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS;

/* flow specification */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8 flag;
    U8 flow_direction; /* 0=out (to air), 1=in (from air) */
    U8 svc_type;
    U32 token_rate;
    U32 token_bucket_size;
    U32 peak_bandwidth;
    U32 access_latency;
} BTS2S_HCI_FLOW_SPEC;

#ifdef CFG_BT_VER_21
/* sub sniff sub rate */

/* this one lives in the LMP_T structure as well, so split out here */

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U16 max_latency;
    U16 min_remote_timeout;
    U16 min_local_timeout;
} BTS2S_HCI_SNIFF_SUBRATING;
#endif

/* sniff sub rate */
/* this one lives in the LMP_T structure as well, so split out here */
#ifdef CFG_BT_VER_21
typedef struct
{
    U16 connection_hdl;
    U16 max_rmt_latency;
    U16 max_local_latency;
    U16 min_rmt_timeout;
    U16 min_local_timeout;
} BTS2S_SNIFF_SUB_RATE;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_SNIFF_SUB_RATE sr;
} BTS2S_HCI_SNIFF_SUB_RATE;


typedef struct
{
    U16 hdl;
} BTS2S_HCI_SNIFF_SUB_RATE_RET;

#endif

/* set ev mask cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U32 ev_mask[2]; /* ouch, 8 bytes */
} BTS2S_HCI_SET_EV_MASK;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RESET;


/* set ev filter cmd
   NOTE: variable len, spec st that unused fields shouldn't be
   present, this defn doesn't comply! */
typedef struct
{
    U24  dev_cls;
    U24  mask;
} BTS2S_CLS_MASK;

typedef struct
{
    BTS2S_CLS_MASK cls_mask;
    U8 auto_acpt;
} BTS2S_CLS_MASK_AUTO;

typedef struct
{
    BTS2S_BD_ADDR bd;
    U8  auto_acpt;
} BTS2S_ADDR_AUTO;

typedef union
{
    BTS2S_CLS_MASK cls_mask; /* type 1 condtype 1 */
    BTS2S_BD_ADDR bd;        /* type 1 condtype 2 */
    U8 auto_acpt;            /* type 2 condtype 0 */
    BTS2S_CLS_MASK_AUTO cma; /* type 2 condtype 1 */
    BTS2S_ADDR_AUTO addr_auto; /*type 2 condtype 2 */
} CONDITION;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 filter_type;
    U8 filter_condition_type;
    CONDITION condition;
} BTS2S_HCI_SET_EV_FILTER;


/* flush cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_FLUSH;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_FLUSH_RET;


/* rd pin type cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PIN_TYPE;

typedef struct
{
    U8 pin_type;
} BTS2S_HCI_RD_PIN_TYPE_RET;


/* wr pin type cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 pin_type;
} BTS2S_HCI_WR_PIN_TYPE;


/* create new unit key cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_CREATE_NEW_UNIT_KEY;


/* rd stored link key cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 rd_all;
} BTS2S_HCI_RD_STORED_LINK_KEY;

typedef struct
{
    U16 max_key_num;
    U16 key_num_rd;
} BTS2S_HCI_RD_STORED_LINK_KEY_RET;


/* wr stored link key cmd / ev
   NOTE: each pointed at block contain a single LINK_KEY_BDADDR_T */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 key_num; /* num of link keys to wr */

    /* store single link_key_bd per ptr */
    LINK_KEYBD_ADDR  link_key_bd[HCI_STORED_LINK_KEY_MAX];
} BTS2S_HCI_WR_STORED_LINK_KEY;

typedef struct
{
    U8 key_num_written;
} BTS2S_HCI_WR_STORED_LINK_KEY_RET;


/* del stored link key cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 flag;
} BTS2S_HCI_DEL_STORED_LINK_KEY;

typedef struct
{
    U16 key_num_deld;
} BTS2S_HCI_DEL_STORED_LINK_KEY_RET;

/* change local name cmd
   NOTE: the name is null terminated (unless it is exactly 248 bytes!)
   and if less than 247 bytes cmd is smaller than this struct,
   the size of the struct is therefore variable.
   HCI_LOCAL_NAME_BYTES_PER_PTR - bytes in a pointed at block.
   HCI_LOCAL_NAME_BYTE_PKT_PTRS - num of pointer blocks */

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    /* the pointers below, point to a byte array of size HCI_LOCAL_NAME_BYTES_PER_PTR */
    U8 *name_part[HCI_LOCAL_NAME_BYTE_PKT_PTRS];
} BTS2S_HCI_CHANGE_LOCAL_NAME;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    /* the pointers below, point to a byte array of size HCI_LOCAL_NAME_BYTES_PER_PTR */
    U8 name_part[HCI_NAME_LEN];
} BTS2S_HCI_CHANGE_LOCAL_NAME_ext;


/* rd local name cmd NOTE: the name is null terminated (unless it is exactly 248 bytes!)
   and if less than 247 bytes cmd is smaller than this struct,
   the size of the struct is therefore variable */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOCAL_NAME;

typedef struct
{
    /* the pointers below, point to a byte array of size HCI_LOCAL_NAME_BYTES_PER_PTR */
    U8 *name_part[HCI_LOCAL_NAME_BYTE_PKT_PTRS];
} BTS2S_HCI_RD_LOCAL_NAME_RET;


typedef struct
{
    /* the pointers below, point to a byte array of size HCI_LOCAL_NAME_BYTES_PER_PTR */
    U8 name_part[HCI_NAME_LEN];
} BTS2S_HCI_RD_LOCAL_NAME_EXT;



/* rd conn acpt timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_CONN_ACPT_TIMEOUT;

typedef struct
{
    U16 conn_acc_timeout;
} BTS2S_HCI_RD_CONN_ACPT_TIMEOUT_RET;


/* wr conn acpt timeout cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 conn_acc_timeout;
} BTS2S_HCI_WR_CONN_ACPT_TIMEOUT;


/* rd page timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PAGE_TIMEOUT;

typedef struct
{
    U16 page_timeout;
} BTS2S_HCI_RD_PAGE_TIMEOUT_RET;


/* wr page timeout cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 page_timeout;
} BTS2S_HCI_WR_PAGE_TIMEOUT;


/* rd scan enb cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_SCAN_ENB;

typedef struct
{
    U8 scan_enb;
} BTS2S_HCI_RD_SCAN_ENB_RET;

/* wr scan enb cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 scan_enb; /* TODO: vals */
} BTS2S_HCI_WR_SCAN_ENB;


/* rd page_scan activity cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PAGESCAN_ACTIVITY;

typedef struct
{
    U16 pagescan_intvl;
    U16 pagescan_window;
} BTS2S_HCI_RD_PAGESCAN_ACTIVITY_RET;


/* wr page_scan activity cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 pagescan_intvl;
    U16 pagescan_window;
} BTS2S_HCI_WR_PAGESCAN_ACTIVITY;


/* rd inquiry_scan activity cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_INQUIRYSCAN_ACTIVITY;

typedef struct
{
    U16 inqscan_intvl;
    U16 inqscan_window;
} BTS2S_HCI_RD_INQUIRYSCAN_ACTIVITY_RET;


/* wr inquiry_scan activity cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 inqscan_intvl;
    U16 inqscan_window;
} BTS2S_HCI_WR_INQUIRYSCAN_ACTIVITY;


/* rd auth enb cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_AUTH_ENB;

typedef struct
{
    U8 auth_enb;
} BTS2S_HCI_RD_AUTH_ENB_RET;


/* wr auth enb cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 auth_enb;
} BTS2S_HCI_WR_AUTH_ENB;


/* rd encryption mode cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_ENC_MODE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_ENC_MODE_RET;


/* wr encryption mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode;
} BTS2S_HCI_WR_ENC_MODE;


/* rd cls of dev cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_DEV_CLS;

typedef struct
{
    U24 dev_cls; /* lower 3 bytes only used */
} BTS2S_HCI_RD_DEV_CLS_RET;


/* wr cls of dev cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U24 dev_cls; /* lower 3 bytes only used */
} BTS2S_HCI_WR_DEV_CLS;


/* rd voice setting cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_VOICE_SETTING;

typedef struct
{
    U16 voice_setting;
} BTS2S_HCI_RD_VOICE_SETTING_RET;


/* wr voice setting cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 voice_setting;
} BTS2S_HCI_WR_VOICE_SETTING;


/* rd automatic flush timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RD_AUTO_FLUSH_TIMEOUT;

typedef struct
{
    U16 hdl;
    U16 timeout; /* N x 0.625msec */
} BTS2S_HCI_RD_AUTO_FLUSH_TIMEOUT_RET;


/* wr automatic flush timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
    U16 timeout; /* N x 0.625msec */
} BTS2S_HCI_WR_AUTO_FLUSH_TIMEOUT;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_WR_AUTO_FLUSH_TIMEOUT_RET;


/* rd num broadcast retransmissions cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_NUM_BCAST_RETXS;

typedef struct
{
    U8 num;
} BTS2S_HCI_RD_NUM_BCAST_RETXS_RET;


/* wr num broadcast retransmissions cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 num;
} BTS2S_HCI_WR_NUM_BCAST_RETXS;


/* rd hold mode activity cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_HOLD_MODE_ACTIVITY;

typedef struct
{
    U8 activity;
} BTS2S_HCI_RD_HOLD_MODE_ACTIVITY_RET;


/* wr hold mode activity cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 activity; /* TODO: define activities */
} BTS2S_HCI_WR_HOLD_MODE_ACTIVITY;


/* rd transmit power level cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
    U8 type; /*TODO: 0=cur 1=max */
} BTS2S_HCI_RD_TX_POWER_LEVEL;

typedef struct
{
    U16 hdl;
    U8 pwr_level;
} BTS2S_HCI_RD_TX_POWER_LEVEL_RET;

/* rd SCO flow ctrl enb cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_SCO_FLOW_CON_ENB;

typedef struct
{
    U8 sco_flow_ctrl_enb;
} BTS2S_HCI_RD_SCO_FLOW_CON_ENB_RET;

/* wr SCO flow ctrl enb cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 sco_flow_ctrl_enb;
} BTS2S_HCI_WR_SCO_FLOW_CON_ENB;

/* set host contoller to host flow ctrl cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 enb; /* 0=off, 1=on */
} BTS2S_HCI_SET_HCTOHOST_FLOW_CTRL;


/* host buff size cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 acl_pkt_len;
    U8  sco_pkt_len;
    U16 acl_total_pkt;
    U16 sco_total_pkt;
} BTS2S_HCI_HOST_BUFF_SIZE;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * host num of compd pkt cmd
 *
 * NOTE: variable struct. num_hdls denotes how many conn
 * hdls follow and then the same num of counts.
 *
 * max size of HCI payload = 255, variable size = 255 - (U8)= 254
 *
 * HCI_BYTE_SIZE_OFHANDLE_COMP - byte size of BTS2S_HANDLE_COMP
 * when transmitted across the HCI ui.
 * HCI_HOST_NUM_COMPD_PKTS_PER_PTR - num of BTS2S_HANDLE_COMP's
 * in a pointed at block.
 * HCI_HOST_NUM_COMPD_PKT_PTRS - num of pointer blocks.
 *---------------------------------------------------------------------------- */

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 num_hdls;

    /* BTS2S_HANDLE_COMP (*num_comp_pkt_ptr[4])[16]
       the pointers below, point to
       BTS2S_HANDLE_COMP[HCI_HOST_NUM_COMPD_PKTS_PER_PTR],
       ie a pointer to the beginning of 16 BTS2S_HANDLE_COMP */
    BTS2S_HANDLE_COMP *num_comp_pkt_ptr[HCI_HOST_NUM_COMPD_PKT_PTRS];

} BTS2S_HCI_HOST_NUM_COMPD_PKTS;


/* rd link surpervison timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RD_LINK_SUPERV_TIMEOUT;

typedef struct
{
    U16 hdl;
    U16 timeout;
} BTS2S_HCI_RD_LINK_SUPERV_TIMEOUT_RET;


/* wr link surpvison timeout cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U16 timeout;
} BTS2S_HCI_WR_LINK_SUPERV_TIMEOUT;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_WR_LINK_SUPERV_TIMEOUT_RET;


/* rd num of supp IAC cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_NUM_SUPP_IAC;

typedef struct
{
    U8 num;
} BTS2S_HCI_RD_NUM_SUPP_IAC_RET;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * rd cur IAC LAP cmd / ev
 *
 * NOTE: variable struct.
 * we have rd num_cur_iac.
 *
 * max size of HCI payload = 255, variable size = 255 - (U8)= 254
 *
 * HCI_IAC_LAP_PER_PTR - num of U24's
 * in a pointed at block.
 * HCI_IAC_LAP_PTRS - num of pointer blocks.
 *---------------------------------------------------------------------------- */

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_CUR_IAC_LAP;

typedef struct
{
    U8 num_cur_iac;
    /* array of pointers to an array of U24 */
    U24 *iac_lap[HCI_IAC_LAP_PTRS];
} BTS2S_HCI_RD_CUR_IAC_LAP_RET;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * wr cur IAC LAP cmd
 *
 * NOTE: variable struct.
 * we have written num_cur_iac.
 *
 * max size of HCI payload = 255, variable size = 255 - (U8)= 254
 *
 * HCI_IAC_LAP_PER_PTR - num of U24's
 * in a pointed at block.
 * HCI_IAC_LAP_PTRS - num of pointer blocks.
 *---------------------------------------------------------------------------- */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 num_cur_iac;
    /* array of pointers to an array of U24 */
    U24 *iac_lap[HCI_IAC_LAP_PTRS];
} BTS2S_HCI_WR_CUR_IAC_LAP;

/* rd page scan period mode cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PAGESCAN_PERIOD_MODE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_PAGESCAN_PERIOD_MODE_RET;


/* wr page scan period mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode; /* TODO: 0=P0 1=P1 2=P2 */
} BTS2S_HCI_WR_PAGESCAN_PERIOD_MODE;


/* rd page scan mode cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PAGESCAN_MODE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_PAGESCAN_MODE_RET;


/* wr page scan mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode; /* TODO: 0=mandatory 1=optional1 2=opt2 3=opt3 */
} BTS2S_HCI_WR_PAGESCAN_MODE;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * set AFH chnl clsification cmd
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 map[10];
} BTS2S_HCI_SET_AFH_CHNL_CLS;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * rd inquiry scan type cmd / ev
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_INQUIRY_SCAN_TYPE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_INQUIRY_SCAN_TYPE_RET;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * wr inquiry scan type cmd
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode; /*0=legacy 1=interlaced */
} BTS2S_HCI_WR_INQUIRY_SCAN_TYPE;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * rd inquiry mode cmd / ev
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_INQUIRY_MODE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_INQUIRY_MODE_RET;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * wr inquiry mode cmd
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode; /*0=standard 1=with rssi */
} BTS2S_HCI_WR_INQUIRY_MODE;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * rd page scan type cmd / ev
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_PAGE_SCAN_TYPE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_PAGE_SCAN_TYPE_RET;


/* wr page scan type cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode; /* 0=legacy 1=interlaced */
} BTS2S_HCI_WR_PAGE_SCAN_TYPE;

/* rd AFH chnl classification mode cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_AFH_CHNL_CLS_M;

typedef struct
{
    U8 cls_mode;
} BTS2S_HCI_RD_AFH_CHNL_CLS_M_RET;


/* wr AFH chnl clsification mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 cls_mode;
} BTS2S_HCI_WR_AFH_CHNL_CLS_M;

#ifdef CFG_BT_VER_21
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_EXT_INQ_RESP;


typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 fec_req;
    BTS2S_HCI_EV_EXT_INQ_RES ext_inq_resp;
} BTS2S_HCI_WR_EXT_INQ_RESP;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
} BTS2S_HCI_REFESH_ENCRY_KEY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_SMP_PAIR_MODE;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 smp_pair_mode;
} BTS2S_HCI_WR_SMP_PAIR_MODE;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOCAL_OOB_DATA;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_INQ_RESP_TRANST_POWE_LEVEL;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 tx_power;
} BTS2S_HCI_WR_INQ_RESP_TRANST_POWE_LEVEL;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 notifi_type;
} BTS2S_HCI_KEYPRES_NOTIFICATION;


typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_DEFAULT_ERRON_DATA_REPORT;


typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 error_data_report;
} BTS2S_HCI_WR_DEFAULT_ERRON_DATA_REPORT;

typedef struct
{
    BTS2S_HCI_CMD_COMMON  common;
    U16 con_hdl;
    U8 packet_type;
} BTS2S_HCI_ENHANCED_FLUSH;
#endif

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 le_support_host;
    U8 unused;
} BTS2S_HCI_WR_LE_SUPPORT;



#ifdef CFG_BT_VER_21
typedef struct
{
    U8 fec_req_flag;
    U8 ext_inquiry_resp[240];
} BTS2S_HCI_RD_EXT_INQUIRE_RESP_M_RET;

typedef struct
{
    U8 st;
} BTS2S_HCI_WR_EXT_INQUIRE_RESP_M_RET;

typedef struct
{
    U8 hash_c[16];
    U8 rand_r[16];
} BTS2S_HCI_RD_LOCAL_OOB_DATA_RESP_M_RET;

typedef struct
{
    U8 smp_pari_mode;

} BTS2S_HCI_RD_SMP_PAIR_MODE_RESP_M_RET;

/*typedef struct
{
    U8 st;
} BTS2S_HCI_WR_SMP_PAIR_MODE_RESP_M_RET; */

#endif

/* rd anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON  common;
} BTS2S_HCI_RD_ANON_MODE;

typedef struct
{
    U8 mode;
} BTS2S_HCI_RD_ANON_MODE_RET;


/* wr anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode;
} BTS2S_HCI_WR_ANON_MODE;


/* rd anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_ALIAS_AUTH_ENB;

typedef struct
{
    U8 enbd;
} BTS2S_HCI_RD_ALIAS_AUTH_ENB_RET;


/* wr anon mode */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 enb;
} BTS2S_HCI_WR_ALIAS_AUTH_ENB;


/* rd anon addr change pars */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_ANON_ADDR_CHANGE_PARAMS;

typedef struct
{
    U32 addr_update_time;
    U16 addr_inquiry_period;
} BTS2S_HCI_RD_ANON_ADDR_CHANGE_PARAMS_RET;

/* wr anon addr change pars*/
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U32 addr_update_time;
    U16 addr_inquiry_period;
} BTS2S_HCI_WR_ANON_ADDR_CHANGE_PARAMS;


/* reset fixed addr attmpts counter */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 max_fails;
} BTS2S_HCI_RESET_FIXED_ADDR_ATTMPTS_COUNTER;

/* rd local version infmtion cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOCAL_VER_INFO;

typedef struct
{
    U8  hci_version;
    U16 hci_revision;
    U8 lmp_version; /* defined in LMP */
    U16 manuf_name; /* defined in LMP */
    U16 lmp_subversion;
} BTS2S_HCI_RD_LOCAL_VER_INFO_RET;


/* rd local supp commnads cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOCAL_SUPP_CMDS;

typedef struct
{
    U8  supp_cmds[HCI_RD_SUPP_CMDS_EV_LEN];
} BTS2S_HCI_RD_LOCAL_SUPP_CMDS_RET;


/* rd local supp featr cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOCAL_SUPP_FEATR;

typedef struct
{
    U8 lmp_supp_featr[8]; /* TODO: bit masks! */
} BTS2S_HCI_RD_LOCAL_SUPP_FEATR_RET;


/* rd local extended featr cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 page_num;
} BTS2S_HCI_RD_LOCAL_EXT_FEATR;

typedef struct
{
    U8 page_num;
    U8 max_page_num;
    U8 lmp_ext_featr[8]; /* TODO: bit masks! */
} BTS2S_HCI_RD_LOCAL_EXT_FEATR_RET;


/* rd buff size */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_BUFF_SIZE;

typedef struct
{
    U16 bts2s_acl_data_pkt_len;
    U8 bts2s_sco_data_pkt_len;
    U16 bts2s_total_acl_data_pkt;
    U16 bts2s_total_sco_data_pkt;
} BTS2S_HCI_RD_BUFF_SIZE_RET;


/* rd country code */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_COUNTRY_CODE;

typedef struct
{
    U8 country_code;
} BTS2S_HCI_RD_COUNTRY_CODE_RET;


/* rd BD_ADDR */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} HCI_RDBD_ADDR;

typedef struct
{
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RD_BD_ADDR_RET;

/* rd failed contact counter cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_RD_FAILED_CONTACT_COUNT;

typedef struct
{
    U16 hdl;
    U16 failed_contact_count;
} BTS2S_HCI_RD_FAILED_CONTACT_COUNT_RET;


/* reset failed contact counter cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_RESET_FAILED_CONTACT_COUNT;

typedef struct
{
    U16 hdl;
} BTS2S_HCI_RESET_FAILED_CONTACT_COUNT_RET;

/* get link qa cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_GET_LINK_QA;

typedef struct
{
    U16 hdl;
    U8 link_qa;
} BTS2S_HCI_GET_LINK_QA_RET;

/* rd RSSI cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR  bd;
} BTS2S_HCI_RD_RSSI;

typedef struct
{
    U16 hdl;
    S8 rssi;
} BTS2S_HCI_RD_RSSI_RET;


/* rd AFH chnl map */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_RD_AFH_CHNL_MAP;

typedef struct
{
    U16 hdl;
    U8 mode;
    U8 map[10];
} BTS2S_HCI_RD_AFH_CHNL_MAP_RET;

/* rd BT clock */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8 which_clock;
} BTS2S_HCI_RD_CLOCK;

typedef struct
{
    U16 hdl;
    U32 clock;
    U16 accuracy;
} BTS2S_HCI_RD_CLOCK_RET;

/* rd loopback mode cmd / ev */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_RD_LOOPBACK_MODE;

typedef struct
{
    U8 mode; /* TODO: define loopback modes */
} BTS2S_HCI_RD_LOOPBACK_MODE_RET;


/* wr loopback mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 mode;
} BTS2S_HCI_WR_LOOPBACK_MODE;


/* enb dev under test mode cmd */
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
} BTS2S_HCI_ENB_DUT_MODE;


/* manufacturer - specific cmds */
typedef struct
{
    U8  cmd;
    U16 cmd_size;
    U16 hdl;
    U16 num0;
    U16 num1;
    U16 num2;
    U16 num3;
    U16 num4;
    U16 num5;
    U16 num6;
    U16 num7;
    U16 num8;
} BTS2S_HCI_DBG_REQ;


/* 1. to tunnel additional prots over HCI. these are carried in chip - friendly fmt.
   2. for dbg cmds  */
typedef union
{
    U8 *bcf;                    /* carries tunnelled messages */
    BTS2S_HCI_DBG_REQ dbg_req;  /* carries dbg reqs */
} MNFR_EXTN_PAYLOAD;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * allow manufacturer's extension cmds to go over HCI.  these are
 * used for:
 *
 * 1. to tunnel additional prots over HCI.
 * 2. for dbg cmds.
 *
 * the payload_desp indicates the chnl ID and contains two
 * bts2g_bits which ctrl fragmentation and reassembly of HCI cmds
 * into prot msgs.  bts2g_bits in the payload desp are:
 *
 * 7       indicates 'fragment end'
 * 6       indicates 'fragment start'
 * [5:0]   chnl ID
 *
 * chnl ids in the range 0 to 15 are equivalent to the BCSP prot
 * ids, although some are not used.  chnl ID CHNL_ID_DBG indicates
 * that the cmd is a dbg cmd.
 *
 *---------------------------------------------------------------------------- */

#define FRAGMENT_END     (0x80)
#define FRAGMENT_START   (0x40)
#define CHNL_ID_DBG      (20)
#define CHNL_ID_MASK     (0x3F)

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    U8 payload_desp;
    MNFR_EXTN_PAYLOAD payload;
} BTS2S_HCI_MNFR_EXTENSION;


typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_COMMON_EV;

/* dbg ev */
typedef struct
{
    U8 dbg_ev;
    U16 num0;
    U16 num1;
    U16 num2;
    U16 num3;
    U16 num4;
    U16 num5;
    U16 num6;
    U16 num7;
    U16 num8;
} BTS2S_HCI_EV_DBG;


/* allow manufacturer's extension evs to go over HCI.  these are used for:
   1. to tunnel additional prots over HCI.
   2. for rsps to dbg cmds */
typedef union
{
    U8 *bcf;                 /* carries tunnelled msgs */
    BTS2S_HCI_EV_DBG dbg_ev; /* carries dbg evs */
} EV_MNFR_EXTN_PAYLOAD;


/*---------------------------------------------------------------------------- *
 * PURPOSE
 * allow manufacturer's extension eves to go over HCI.  these are
 * used for:
 *
 * 1. to tunnel additional prots over HCI.
 * 2. for rsps to dbg cmds.
 *
 * the payload_desp indicates the chnl ID and contains two
 * bts2g_bits which ctrl fragmentation and reassembly of HCI cmds
 * into prot msgs.  bts2g_bits in the payload desp are:
 *
 * 7       indicates 'fragment end'
 * 6       indicates 'fragment start'
 * [5:0]   chnl ID
 *
 * chnl ids in the range 0 to 15 are equivalent to the BCSP prot
 * ids, although some are not used.  chnl ID CHNL_ID_DBG indicates
 * that the cmd is a dbg cmd.
 *
 *---------------------------------------------------------------------------- */

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    /* coding of payload_desp is the same as BTS2S_HCI_MNFR_EXTENSION */
    U8 payload_desp;
    EV_MNFR_EXTN_PAYLOAD payload;
} BTS2S_HCI_EV_MNFR_EXTENSION;


/* inquiry comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
} BTS2S_HCI_EV_INQUIRY_COMP;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * inquiry res ev
 *
 * NOTE: variable struct.
 * we have rsp_num ress.
 *
 * max size of HCI payload = 255, variable size = 255 - (U8)= 254
 *
 * HCI_MAX_INQ_RES_PER_PTR - num of BTS2S_HCI_INQ_RES's
 * in a pointed at block.
 * HCI_MAX_INQ_RES_PTRS - num of pointer blocks.
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */

typedef struct
{
    BTS2S_BD_ADDR bd;
    U8 page_scan_rep_mode;
    U8 page_scan_period_mode;
    U8 page_scan_mode;
    U24 dev_cls; /* lower 3 bytes only used */
    U16 clock_offset;
} BTS2S_HCI_INQ_RES;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 rsp_num;
    BTS2S_HCI_INQ_RES *res[HCI_MAX_INQ_RES_PTRS];
} BTS2S_HCI_EV_INQUIRY_RES;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 rsp_num;
    BTS2S_HCI_INQ_RES res;
} BTS2S_HCI_EV_INQUIRY_RES_ext;



/*---------------------------------------------------------------------------- *
 * PURPOSE
 * inquiry res with RSSI ev
 *
 * NOTE: variable struct.
 * we have rsp_num ress.
 *
 * max size of HCI payload = 255, variable size = 255 - (U8)= 254
 *
 * HCI_MAX_INQ_RES_PER_PTR - num of BTS2S_HCI_INQ_RES's
 * in a pointed at block.
 * HCI_MAX_INQ_RES_PTRS - num of pointer blocks.
 *
 * ----------------------------------------------------------------------------  --  --  --  --  --  --  - */

typedef struct
{
    BTS2S_BD_ADDR bd;
    U8 page_scan_rep_mode;
    U8 page_scan_period_mode;
    U24 dev_cls; /* lower 3 bytes only used */
    U16 clock_offset;
    S8 rssi;
} BTS2S_HCI_INQ_RES_WITH_RSSI;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 rsp_num;
    BTS2S_HCI_INQ_RES_WITH_RSSI *res[HCI_MAX_INQ_RES_PTRS];
} BTS2S_HCI_EV_INQUIRY_RES_WITH_RSSI;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 rsp_num; // only support rsp_num is 1
    BTS2S_HCI_INQ_RES_WITH_RSSI res;
} BTS2S_HCI_EV_INQUIRY_RES_WITH_RSSI_ext;


/* conn comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8 link_type;
    U8 enc_mode;
} BTS2S_HCI_EV_CONN_COMP;

/* conn req ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U24 dev_cls; /* lower 3 bytes only used */
    U8 link_type;
} BTS2S_HCI_EV_CONN_REQ;


/* disconn comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;      /* cast to err if err */
    U16 hdl;
    U8 reason;  /* reason for disconn 0x08, 0x13 - 0x16 only */
} BTS2S_HCI_EV_DISC_COMP;


/* auth comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
} BTS2S_HCI_EV_AUTH_COMP;


/* rmt name req comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    BTS2S_BD_ADDR bd;
    U8 *name_part[HCI_LOCAL_NAME_BYTE_PKT_PTRS];
} BTS2S_HCI_EV_RMT_NAME_REQ_COMP;


typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    BTS2S_BD_ADDR bd;
    U8 name_part[HCI_NAME_LEN];
} BTS2S_HCI_EV_RMT_NAME_REQ_COMP_ext;


/* encryption change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 enc_enb;
} BTS2S_HCI_EV_ENCRYPTION_CHANGE;

/* change conn link key comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
} BTS2S_HCI_EV_CHANGE_CONN_LINK_KEY_COMP;


/* master link key comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 key_flag;
} BTS2S_HCI_EV_MASTER_LINK_KEY_COMP;


/* rd rmt supp featr comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U16 lmp_supp_featr[4];
} BTS2S_HCI_EV_RD_REM_SUPP_FEATR_COMP;


/* rd remote version information comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 lmp_version; /* defined in LMP */
    U16 manuf_name; /* defined in LMP */
    U16 lmp_subversion;
} BTS2S_HCI_EV_RD_RMT_VER_INFO_COMP;

/* qoS setup comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 flag; /* rsv */
    U8 svc_type;
    U32 token_rate;
    U32 peak_bandwidth;
    U32 latency;
    U32 delay_variation;
} BTS2S_HCI_EV_QOS_SETUP_COMP;

/* cmd st ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U8 hci_cmd_pkt_num;
    U16 op_code; /* op code of cmd that caused this ev */
} BTS2S_HCI_EV_CMD_STS;

/* hardware err ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 hw_err;
} BTS2S_HCI_EV_HARDWARE_ERR;

/* flush occurred ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U16 hdl;
} BTS2S_HCI_EV_FLUSH_OCCURRED;

/* role change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    BTS2S_BD_ADDR bd;
    U8 new_role;
} BTS2S_HCI_EV_ROLE_CHANGE;

/*---------------------------------------------------------------------------- *
 * PURPOSE
 * num of compd pkt ev
 *
 * NOTE: another variable struct. num_hdls denotes how many conn
 * hdls follow and then the same num of counts.
 *
 * max size of HCI payload = 256, variable size = 256 - (U8)= 255
 *
 * HCI_BYTE_SIZE_OFHANDLE_COMP - byte size of BTS2S_HANDLE_COMP
 * when transmitted across the HCI ui.
 * HCI_EV_NUM_COMPD_PKTS_PER_PTR - num of BTS2S_HANDLE_COMP's
 * in a pointed at block.
 * HCI_EV_NUM_COMPD_PKT_PTRS - num of pointer blocks.
 *---------------------------------------------------------------------------- */

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 num_hdls;

    /* BTS2S_HANDLE_COMP (*num_comp_pkt_ptr[4])[16]
       the pointers below, point to
       BTS2S_HANDLE_COMP[HCI_HOST_NUM_COMPD_PKTS_PER_PTR],
       ie a pointer to the beginning of 16 BTS2S_HANDLE_COMP */
    BTS2S_HANDLE_COMP *num_comp_pkt_ptr;
} BTS2S_HCI_EV_NUM_COMPD_PKTS;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 num_hdls;

    /* BTS2S_HANDLE_COMP (*num_comp_pkt_ptr[4])[16]
       the pointers below, point to
       BTS2S_HANDLE_COMP[HCI_HOST_NUM_COMPD_PKTS_PER_PTR],
       ie a pointer to the beginning of 16 BTS2S_HANDLE_COMP */
    BTS2S_HANDLE_COMP num_comp_pkt_ptr;
} BTS2S_HCI_EV_NUM_COMPD_PKTS_ext;


/* mode change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 curr_mode;
    U16 intvl;
} BTS2S_HCI_EV_MODE_CHANGE;

/* NOTE: each pointed at block contain a single LINK_KEY_BDADDR_T */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 key_num;
    LINK_KEYBD_ADDR link_key_bd[HCI_STORED_LINK_KEY_MAX];
} BTS2S_HCI_EV_RETURN_LINK_KEYS;

/* PIN code req ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_PIN_CODE_REQ;

/* link key req ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_LINK_KEY_REQ;

/* link key noti ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 link_key[LINK_KEY_SIZE];
    U8 key_type;
} BTS2S_HCI_EV_LINK_KEY_NOTI;


/* NOTE: cmd_pkt is variable len, it contains the full cmd
   pkt sent, including the header */
typedef struct
{
    U8 bytes[HCI_LOOPBACK_BYTES_PER_PTR];
} BTS2S_HCI_EV_LOOPBACK_BYTE_STRUCT;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_HCI_EV_LOOPBACK_BYTE_STRUCT *loopback_part_ptr[HCI_LOOPBACK_BYTE_PKT_PTRS];
} BTS2S_HCI_EV_LOOPBACK_CMD;

/* data buff overflow ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 link_type;
} BTS2S_HCI_EV_DATA_BUFF_OVERFLOW;


/* max slots change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8  st;
    U16 hdl;
    U8 lmp_max_slot;
} BTS2S_HCI_EV_MAX_SLOTS_CHANGE;

/* rd clock offset comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U16 clock_offset;
} BTS2S_HCI_EV_RD_CLOCK_OFFSET_COMP;


/* conn pkt type changed ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U16 pkt_type;
} BTS2S_HCI_EV_CONN_PKT_TYPE_CHANGED;


/* qoS violation ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U16 hdl;
} BTS2S_HCI_EV_QOS_VIOLATION;


/* page scan mode change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 page_scan_mode;
} BTS2S_HCI_EV_PAGE_SCAN_MODE_CHANGE;


/* page scan repitition mode change ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 page_scan_rep_mode;
} BTS2S_HCI_EV_PAGE_SCAN_REP_MODE_CHANGE;


/* flow specification changed ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8  st;
    U16 hdl;
    U8 flag;
    U8 flow_direction; /*0=out (to air), 1=in (from air) */
    U8 svc_type;
    U32 token_rate;
    U32 token_bucket_size;
    U32 peak_bandwidth;
    U32 access_latency;
} BTS2S_HCI_EV_FLOW_SPEC_COMP;

/* rd rmt extended featr comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    U8 page_num;
    U8 max_page_num;
    U16 lmp_ext_featr[4];
} BTS2S_HCI_EV_RD_REM_EXT_FEATR_COMP;

/* FIXED_ADDR */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_FIXED_ADDR;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* ALIAS_ADDR
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    U16 hdl;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_ALIAS_ADDR;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* GENERATE_ALIAS_REQ
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U16 hdl;
} BTS2S_HCI_EV_GENERATE_ALIAS_REQ;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* ACT_ADDR
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR       bd_fixed;
    BTS2S_BD_ADDR       bd;
} BTS2S_HCI_EV_ACT_ADDR;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* ALLOW_PRIVATE_PAIRING
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U16 hdl;
} BTS2S_HCI_EV_ALLOW_PRIVATE_PAIRING;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* ALIAS_ADDR_REQ
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_ALIAS_ADDR_REQ;


/*----------------------------------------------------------------------------  --  --  --  --  --  --  - *
* PURPOSE
* ALIAS_NOT_RECOGNISED
*
* ----------------------------------------------------------------------------  --  --  --  --  --  --  - */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st; /* cast to err if err */
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_ALIAS_NOT_RECOGNISED;


/* FIXED_ADDR_ATTMPT */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U32 rsv;
} BTS2S_HCI_EV_FIXED_ADDR_ATTMPT;


/* synchronous conn comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;
    U16 hdl;
    BTS2S_BD_ADDR bd;
    U8 link_type;
    U8 tx_intvl;
    U8 wesco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
    U8 air_mode;
} BTS2S_HCI_EV_SYNC_CONN_COMP;


/* synchronous conn changed ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;
    U16 hdl;
    U8 tx_intvl;
    U8 wesco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
} BTS2S_HCI_EV_SYNC_CONN_CHANGED;

/* sniff sub rate comp ev */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;
    U16 hdl;
    U16 transmit_latency;
    U16 recv_latency;
    U16 rmt_timeout;
    U16 local_timeout;
} BTS2S_HCI_EV_SNIFF_SUB_RATE;

/* Encryption Key Refresh Complete */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;
    U16 hdl;

} BTS2S_HCI_EV_ENCRY_KEY_REF_COMPLETE;


#ifdef CFG_BT_VER_21
/* HCI_EV_EXT_INQUIRY_RES */

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 num_res;
    BTS2S_BD_ADDR bd;
    U8 page_scan_rep_mode;
    U8 page_scan_mode;
    U24 class_device;
    U16 clock_offset;
    U8 rssi;
    U8 device_name_len;
    U8 device_name[50];

    U8 device_service_len;
    U8 device_service[100];
} BTS2S_HCI_EV_EXT_INQ_RESPONSE;

typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 num_res;
    BTS2S_BD_ADDR bd;
    U8 page_scan_rep_mode;
    U8 page_scan_mode;
    U24 class_device;
    U16 clock_offset;
    U8 rssi;
    U8 eir_data[240];
} BTS2S_HCI_EV_EXT_INQ_RESPONSE_ext;


/* IO Capability Request Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_IO_CAP_REQ;

/* IO Capability Response Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 io_cap;
    U8 oob_data_present;
    U8 auth_req;
} BTS2S_HCI_EV_CAP_RESP_EVENT;

/* User Confirmation Request Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U32 num_val;
} BTS2S_HCI_EV_USER_CONF_REQ;

/* User Passkey Request Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_USR_PASSKY_REQ;


/* Remote OOB Data Request Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_RMT_OOB_DATA_REQ;


/* Simple Pairing Complete Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 st;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_EV_SSP_COMPLETE;

/* Link Supervision Timeout Changed Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 con_hdl;
    U16 link_super_timeout;
} BTS2S_HCI_EV_LINK_SUPER_TIMEOUT_CHANGED;

/* Enhanced Flush Complete Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 con_hdl;
} BTS2S_HCI_EV_EHA_FLUSH_COMPLETE;

/* User Passkey Notification Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U32 passkey;
} BTS2S_HCI_EV_USR_PASSKY_NOTIFI;

/* User Passkey Notification Event */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    BTS2S_BD_ADDR bd;
    U32 passkey;
} BTS2S_HCI_PASSKY_NOTIFI;

/* User keymissing Notification Event */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_KEYMISSING_NOTIFI;


/*  Keypress Notification Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 notifi_type;
} BTS2S_HCI_EV_KEY_NOTIFI;

/* Remote Host Supported Features Notification Event */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    BTS2S_BD_ADDR bd;
    U8 host_supp_feat;

} BTS2S_HCI_EV_RMT_HOST_SUPPORT_FEAUT_NOTIFI;
#endif


/* data pkt indicate ev MSG_BLK_T defined in msg_blk.h in common */

/* ACL */
typedef struct
{
    U8  total_acl ;
    U16 hdl_plus_flag;
    void *data_ptr; /* BTS2S_BUFF */
} BTS2S_HCI_ACL_DATA_PKT_IND;

/* SCO */
typedef struct
{
    BTS2S_HCI_SCO_DATA *header_ptr;
    void *data_ptr; /* can be either MSG_BLK_T or BTS2S_BUFF */
} BTS2S_HCI_SCO_DATA_PKT_IND;

/* UNION OF CMD COMP ARGS */

typedef union
{
    BTS2S_HCI_WITH_BD_ADDR_RET evt_with_bd;
    BTS2S_HCI_WITH_HANDLE_RET evt_with_hdl;
    BTS2S_HCI_BD_ADDR_GEN_RET gen_BD_ADDR;

    BTS2S_HCI_CREATE_CONN_ESC_RET create_conn_esc_arg;
    BTS2S_HCI_LINK_KEY_REQ_REPLY_RET link_key_req_reply_arg;
    BTS2S_HCI_LINK_KEY_REQ_NEG_REPLY_RET link_key_reqneg_reply_arg;
    BTS2S_HCI_PIN_CODE_REQ_REPLY_RET pin_code_req_reply_arg;
    BTS2S_HCI_PIN_CODE_REQ_NEG_REPLY_RET pin_code_reqneg_reply_arg;
    BTS2S_HCI_RMT_NAME_REQ_ESC_RET rmt_name_req_esc_arg;
    BTS2S_HCI_RD_LMP_HANDLE_RET rd_lmp_hdl_arg;
    BTS2S_HCI_PRIVATE_PAIRING_REQ_REPLY_RET private_pairing_req_reply;
    BTS2S_HCI_PRIVATE_PAIRING_REQ_NEG_REPLY_RET private_pairing_req_neg_reply;
    BTS2S_HCI_GENERATED_ALIAS_RET generated_alias;
    BTS2S_HCI_ALIAS_ADDR_REQ_REPLY_RET alias_addr_req_reply;
    BTS2S_HCI_ALIAS_ADDR_REQ_NEG_REPLY_RET alias_addr_req_neg_reply;

    BTS2S_HCI_ROLE_DISCOV_RET role_discov_arg;
    BTS2S_HCI_RD_LINK_POLICY_SETTINGS_RET rd_link_policy_setting_arg;
    BTS2S_HCI_WR_LINK_POLICY_SETTINGS_RET wr_link_policy_setting_arg;
    BTS2S_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_RET rd_default_lps_arg;
    BTS2S_HCI_FLUSH_RET flush_arg;
    BTS2S_HCI_RD_PIN_TYPE_RET rd_pin_type_arg;
    BTS2S_HCI_RD_STORED_LINK_KEY_RET rd_stored_link_key_arg;
    BTS2S_HCI_WR_STORED_LINK_KEY_RET wr_stored_link_key_arg;
    BTS2S_HCI_DEL_STORED_LINK_KEY_RET del_stored_link_key_arg;
    BTS2S_HCI_RD_LOCAL_NAME_RET rd_local_name_arg;
    BTS2S_HCI_RD_CONN_ACPT_TIMEOUT_RET rd_conn_acc_timeout_arg;
    BTS2S_HCI_RD_PAGE_TIMEOUT_RET rd_page_timeout_arg;
    BTS2S_HCI_RD_SCAN_ENB_RET rd_scan_enb_arg;
    BTS2S_HCI_RD_PAGESCAN_ACTIVITY_RET rd_pagescan_activity_arg;
    BTS2S_HCI_RD_INQUIRYSCAN_ACTIVITY_RET rd_inqscan_activity_arg;
    BTS2S_HCI_RD_AUTH_ENB_RET rd_auth_enb_arg;
    BTS2S_HCI_RD_ENC_MODE_RET rd_enc_mode_arg;
    BTS2S_HCI_RD_DEV_CLS_RET rd_cls_dev_arg;
    BTS2S_HCI_RD_VOICE_SETTING_RET rd_voice_setting_arg;
    BTS2S_HCI_RD_AUTO_FLUSH_TIMEOUT_RET rd_auto_flush_timeout_arg;
    BTS2S_HCI_WR_AUTO_FLUSH_TIMEOUT_RET wr_auto_flush_timeout_arg;
    BTS2S_HCI_RD_NUM_BCAST_RETXS_RET rd_num_bcast_retxs_arg;
    BTS2S_HCI_RD_HOLD_MODE_ACTIVITY_RET rd_hold_mode_activity_arg;
    BTS2S_HCI_RD_TX_POWER_LEVEL_RET rd_tx_power_level_arg;
    BTS2S_HCI_RD_SCO_FLOW_CON_ENB_RET rd_sco_flow_con_enb_arg;
    BTS2S_HCI_RD_LINK_SUPERV_TIMEOUT_RET rd_link_superv_timeout_arg;
    BTS2S_HCI_WR_LINK_SUPERV_TIMEOUT_RET wr_link_superv_timeout_arg;
    BTS2S_HCI_RD_NUM_SUPP_IAC_RET rd_num_supp_iac_arg;
    BTS2S_HCI_RD_CUR_IAC_LAP_RET rd_cur_iac_lap_arg;
    BTS2S_HCI_RD_PAGESCAN_PERIOD_MODE_RET rd_pagescan_period_mode_arg;
    BTS2S_HCI_RD_PAGESCAN_MODE_RET rd_pagescan_mode_arg;
    BTS2S_HCI_RD_AFH_CHNL_CLS_M_RET rd_afh_chnl_cls_m_arg;
    BTS2S_HCI_RD_INQUIRY_SCAN_TYPE_RET rd_inquiry_scan_type_arg;
    BTS2S_HCI_RD_INQUIRY_MODE_RET rd_inquiry_mode_arg;
    BTS2S_HCI_RD_PAGE_SCAN_TYPE_RET rd_page_scan_type_arg;

    BTS2S_HCI_RD_LOCAL_VER_INFO_RET rd_local_ver_info_arg;
    BTS2S_HCI_RD_LOCAL_SUPP_CMDS_RET rd_local_supp_cmds_arg;
    BTS2S_HCI_RD_LOCAL_SUPP_FEATR_RET rd_local_supp_featr_arg;
    BTS2S_HCI_RD_LOCAL_EXT_FEATR_RET rd_local_ext_featr_arg;
    BTS2S_HCI_RD_BUFF_SIZE_RET rd_buff_size_arg;
    BTS2S_HCI_RD_COUNTRY_CODE_RET rd_country_code_arg;
    BTS2S_HCI_RD_BD_ADDR_RET rd_bd_arg;
    BTS2S_HCI_RD_FAILED_CONTACT_COUNT_RET rd_failed_contact_count_arg;
    BTS2S_HCI_RESET_FAILED_CONTACT_COUNT_RET reset_failed_contact_count_arg;
    BTS2S_HCI_GET_LINK_QA_RET get_link_qa_arg;
    BTS2S_HCI_RD_RSSI_RET rd_rssi_arg;
    BTS2S_HCI_RD_LOOPBACK_MODE_RET rd_loopback_mode_arg;
    BTS2S_HCI_RD_AFH_CHNL_MAP_RET rd_afh_chnl_map_arg;
    BTS2S_HCI_RD_CLOCK_RET rd_clock_arg;
#ifdef CFG_BT_VER_21
    BTS2S_HCI_SNIFF_SUB_RATE_RET BTS2S_SNIFF_sub_rate_arg;
    BTS2S_HCI_SNIFF_SUB_RATE_RET sniff_sub_rate_arg;
    BTS2S_HCI_RD_EXT_INQUIRE_RESP_M_RET rd_ext_inquire_rest_arg;
    BTS2S_HCI_RD_LOCAL_OOB_DATA_RESP_M_RET rd_local_oob_res_arg;
    BTS2S_HCI_RD_SMP_PAIR_MODE_RESP_M_RET rd_smp_pair_mode_res_arg;
#endif
} HCI_CMD_COMP_ARGS;


/* cmd comp ev use specific return structures for particular cmds
   the st arg, should in all rights be in the arg_ptr
   list but, as every cmd comp ev returns a st we have
   moved it into the body of the structure to prev us having malloc
   cmds which only return a st */
typedef struct
{
    BTS2S_HCI_EV_COMMON ev;
    U8 hci_cmd_pkt_num;
    U16 op_code;                /* op code of cmd that caused this ev */
    U8 st;
    HCI_CMD_COMP_ARGS arg_ptr; /* pointer to arg */
} BTS2S_HCI_EV_CMD_COMP;

/* UNION OF MSGITIVES */

typedef union hci_umsg_tag
{
    U16 op_code; /* op code of cmd */
    U8 ev_code;  /* op code of ev */

    BTS2S_HCI_CMD_COMMON  hci_cmd;
    BTS2S_HCI_CMD_GEN  hci_gen;
    BTS2S_HCI_EV_COMMON hci_ev;

    BTS2S_HCI_WITH_CH_COMMON  hci_cmd_with_hdl;
    BTS2S_HCI_WITH_BD_COMMON  hci_cmd_with_bd;
    BTS2S_HCI_EV_COMMON_EV hci_ev_with_hdl;

    BTS2S_HCI_ACL_DATA bts2s_hci_acl_data;
    BTS2S_HCI_SCO_DATA hci_sco_data;

    BTS2S_HCI_INQUIRY hci_inquiry;
    BTS2S_HCI_INQUIRY_ESC hci_inquiry_esc;
    BTS2S_HCI_PERIODIC_INQUIRY_MODE hci_periodic_inquiry_mode;
    BTS2S_HCI_EXIT_PERIODIC_INQUIRY_MODE hci_exit_periodic_inquiry_mode;
    BTS2S_HCI_CREATE_CONN hci_create_conn;
    BTS2S_HCI_DISC hci_disc;
    BTS2S_HCI_ADD_SCO_CONN hci_add_sco_conn;
    BTS2S_HCI_CREATE_CONN_ESC hci_create_conn_esc;
    BTS2S_HCI_ACPT_CONN_REQ hci_acpt_conn;
    BTS2S_HCI_REJ_CONN_REQ hci_rej_conn;
    BTS2S_HCI_LINK_KEY_REQ_REPLY hci_link_key_req_reply;
    BTS2S_HCI_LINK_KEY_REQ_NEG_REPLY hci_link_key_reqneg_reply;
    BTS2S_HCI_PIN_CODE_REQ_REPLY hci_pin_code_req_reply;
    BTS2S_HCI_PIN_CODE_REQ_NEG_REPLY hci_pin_code_reqneg_reply;
    BTS2S_HCI_CHANGE_CONN_PKT_TYPE hci_change_conn_pkt_type;
    BTS2S_HCI_AUTH_REQ hci_auth_req;
    BTS2S_HCI_SET_CONN_ENCRYPTION hci_set_conn_encryption;
    BTS2S_HCI_CHANGE_CONN_LINK_KEY hci_change_conn_link_key;
    BTS2S_HCI_MASTER_LINK_KEY hci_master_link_key;
    BTS2S_HCI_RMT_NAME_REQ hci_rmt_name_req;
    BTS2S_HCI_RMT_NAME_REQ_ESC hci_rmt_name_req_esc;
    BTS2S_HCI_RD_RMT_SUPP_FEATR hci_rd_rem_supp_featr;
    BTS2S_HCI_RD_RMT_EXT_FEATR hci_rd_rem_ext_featr;
    BTS2S_HCI_RD_RMT_VER_INFO hci_rd_rem_ver_info;
    BTS2S_HCI_RD_CLOCK_OFFSET hci_rd_clock_offset;
    BTS2S_HCI_RD_LMP_HANDLE hci_rd_lmp_hdl;
    BTS2S_HCI_EXCHANGE_FIXED_INFO hci_exchange_fixed_info;
    BTS2S_HCI_EXCHANGE_ALIAS_INFO hci_exchange_alias_info;
    BTS2S_HCI_PRIVATE_PAIRING_REQ_REPLY hci_private_pairing_req_reply;
    BTS2S_HCI_PRIVATE_PAIRING_REQ_NEG_REPLY hci_private_pairing_req_neg_reply;
    BTS2S_HCI_GENERATED_ALIAS hci_generated_alias;
    BTS2S_HCI_ALIAS_ADDR_REQ_REPLY hci_alias_addr_req_reply;
    BTS2S_HCI_ALIAS_ADDR_REQ_NEG_REPLY hci_alias_addr_req_neg_reply;
    BTS2S_HCI_SETUP_SYNCHRONOUS_CONN hci_setup_sync_conn;
    BTS2S_HCI_ACPT_SYNCHRONOUS_CONN_REQ hci_acpt_sync_conn_req;
    BTS2S_HCI_REJ_SYNCHRONOUS_CONN_REQ hci_rej_sync_conn_req;

#ifdef CFG_BT_VER_21
    BTS2S_HCI_EV_IO_CAP_REQ hci_io_capability_rep;
    BTS2S_HCI_EV_KEY_NOTIFI hci_ev_key_notifi;
#endif
    BTS2S_HCI_HOLD_MODE hci_hold_mode;
    BTS2S_HCI_SNIFF_MODE hciSNIFF_mode;
    BTS2S_HCI_EXIT_SNIFF_MODE hci_exitSNIFF_mode;
    BTS2S_HCI_QOS_SETUP hci_qos_setup;
    BTS2S_HCI_ROLE_DISCOV hci_role_discov;
    BTS2S_HCI_SWITCH_ROLE hci_switch_role;
    BTS2S_HCI_RD_LINK_POLICY_SETTINGS hci_rd_link_policy_setting;
    BTS2S_HCI_WR_LINK_POLICY_SETTINGS hci_wr_link_policy_setting;
    BTS2S_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS hci_rd_default_lps;
    BTS2S_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS hci_wr_default_lps;
    BTS2S_HCI_FLOW_SPEC hci_flow_spec;
#ifdef CFG_BT_VER_21
    BTS2S_HCI_SNIFF_SUB_RATE hciSNIFF_sub_rate;
#endif

    BTS2S_HCI_SET_EV_MASK hci_set_ev_mask;
    BTS2S_HCI_RESET hci_reset;
    BTS2S_HCI_SET_EV_FILTER hci_set_ev_filter;
    BTS2S_HCI_FLUSH hci_flush;
    BTS2S_HCI_RD_PIN_TYPE hci_rd_pin_type;
    BTS2S_HCI_WR_PIN_TYPE hci_wr_pin_type;
    BTS2S_HCI_CREATE_NEW_UNIT_KEY hci_create_new_unit_key;
    BTS2S_HCI_RD_STORED_LINK_KEY hci_rd_stored_link_key;
    BTS2S_HCI_WR_STORED_LINK_KEY hci_wr_stored_link_key;
    BTS2S_HCI_DEL_STORED_LINK_KEY hci_del_stored_link_key;
    BTS2S_HCI_CHANGE_LOCAL_NAME_ext hci_change_local_name;
    BTS2S_HCI_RD_LOCAL_NAME_EXT hci_rd_local_name;
    BTS2S_HCI_RD_CONN_ACPT_TIMEOUT hci_rd_conn_acc_timeout;
    BTS2S_HCI_WR_CONN_ACPT_TIMEOUT hci_wr_conn_acc_timeout;
    BTS2S_HCI_RD_PAGE_TIMEOUT hci_rd_page_timeout;
    BTS2S_HCI_WR_PAGE_TIMEOUT hci_wr_page_timeout;
    BTS2S_HCI_RD_SCAN_ENB hci_scan_enb;
    BTS2S_HCI_WR_SCAN_ENB hci_wr_scan_enb;
    BTS2S_HCI_RD_PAGESCAN_ACTIVITY hci_rd_pagescan_activity;
    BTS2S_HCI_WR_PAGESCAN_ACTIVITY hci_wr_pagescan_activity;
    BTS2S_HCI_RD_INQUIRYSCAN_ACTIVITY hci_rd_inqscan_activity;
    BTS2S_HCI_WR_INQUIRYSCAN_ACTIVITY hci_wr_inqscan_activity;
    BTS2S_HCI_RD_AUTH_ENB hci_rd_auth_enb;
    BTS2S_HCI_WR_AUTH_ENB hci_wr_auth_enb;
    BTS2S_HCI_RD_ENC_MODE hci_rd_enc_mode;
    BTS2S_HCI_WR_ENC_MODE hci_wr_enc_mode;
    BTS2S_HCI_RD_DEV_CLS hci_rd_dev_cls;
    BTS2S_HCI_WR_DEV_CLS hci_wr_dev_cls;
    BTS2S_HCI_RD_VOICE_SETTING hci_rd_voice_setting;
    BTS2S_HCI_WR_VOICE_SETTING hci_wr_voice_setting;
    BTS2S_HCI_RD_AUTO_FLUSH_TIMEOUT hci_rd_auto_flush_timeout;
    BTS2S_HCI_WR_AUTO_FLUSH_TIMEOUT hci_wr_auto_flush_timeout;
    BTS2S_HCI_RD_NUM_BCAST_RETXS hci_rd_num_bcast_retxs;
    BTS2S_HCI_WR_NUM_BCAST_RETXS hci_wr_num_bcast_retxs;
    BTS2S_HCI_RD_HOLD_MODE_ACTIVITY hci_rd_hold_mode_activity;
    BTS2S_HCI_WR_HOLD_MODE_ACTIVITY hci_wr_hold_mode_activity;
    BTS2S_HCI_RD_TX_POWER_LEVEL hci_rd_tx_power_level;
    BTS2S_HCI_RD_SCO_FLOW_CON_ENB hci_rd_sco_flow_con_enb;
    BTS2S_HCI_WR_SCO_FLOW_CON_ENB hci_wr_sco_flow_con_enb;
    BTS2S_HCI_SET_HCTOHOST_FLOW_CTRL hci_set_hctohost_flow_ctrl;
    BTS2S_HCI_HOST_BUFF_SIZE hci_host_buff_size;
    BTS2S_HCI_HOST_NUM_COMPD_PKTS hci_host_num_coml_pkt;
    BTS2S_HCI_RD_LINK_SUPERV_TIMEOUT hci_rd_link_superv_timeout;
    BTS2S_HCI_WR_LINK_SUPERV_TIMEOUT hci_wr_link_superv_timeout;
    BTS2S_HCI_RD_NUM_SUPP_IAC hci_rd_num_supp_iac;
    BTS2S_HCI_RD_CUR_IAC_LAP hci_rd_curr_iac_lap;
    BTS2S_HCI_WR_CUR_IAC_LAP hci_wr_curr_iac_lap;
    BTS2S_HCI_RD_PAGESCAN_PERIOD_MODE hci_rd_pagescan_period_mode;
    BTS2S_HCI_WR_PAGESCAN_PERIOD_MODE hci_wr_pagescan_period_mode;
    BTS2S_HCI_RD_PAGESCAN_MODE hci_rd_pagescan_mode;
    BTS2S_HCI_WR_PAGESCAN_MODE hci_wr_pagescan_mode;
    BTS2S_HCI_SET_AFH_CHNL_CLS hci_set_afh_chnl_cls;
    BTS2S_HCI_RD_INQUIRY_SCAN_TYPE hci_rd_inquiry_scan_type;
    BTS2S_HCI_WR_INQUIRY_SCAN_TYPE hci_wr_inquiry_scan_type;
    BTS2S_HCI_RD_INQUIRY_MODE hci_rd_inquiry_mode;
    BTS2S_HCI_WR_INQUIRY_MODE hci_wr_inquiry_mode;
    BTS2S_HCI_RD_PAGE_SCAN_TYPE hci_rd_page_scan_type;
    BTS2S_HCI_WR_PAGE_SCAN_TYPE hci_wr_page_scan_type;
    BTS2S_HCI_RD_AFH_CHNL_CLS_M hci_rd_afh_chnl_cls_m;
    BTS2S_HCI_WR_AFH_CHNL_CLS_M hci_wr_afh_chnl_cls_m;
    BTS2S_HCI_RD_ANON_MODE hci_rd_anon_mode;
    BTS2S_HCI_WR_ANON_MODE hci_wr_anon_mode;
    BTS2S_HCI_RD_ALIAS_AUTH_ENB hci_rd_alias_auth_enb;
    BTS2S_HCI_WR_ALIAS_AUTH_ENB hci_wr_alias_auth_enb;
    BTS2S_HCI_RD_ANON_ADDR_CHANGE_PARAMS hci_rd_anon_addr_change_params;
    BTS2S_HCI_WR_ANON_ADDR_CHANGE_PARAMS hci_wr_anon_addr_change_params;
    BTS2S_HCI_RESET_FIXED_ADDR_ATTMPTS_COUNTER hci_reset_fixed_addr_attmpts_counter;

    BTS2S_HCI_RD_LOCAL_VER_INFO hci_rd_local_ver_info;
    BTS2S_HCI_RD_LOCAL_SUPP_CMDS hci_rd_local_supp_cmds;
    BTS2S_HCI_RD_LOCAL_SUPP_FEATR hci_rd_local_supp_featr;
    BTS2S_HCI_RD_LOCAL_EXT_FEATR hci_rd_local_ext_featr;
    BTS2S_HCI_RD_BUFF_SIZE hci_rd_buff_size;
    BTS2S_HCI_RD_COUNTRY_CODE hci_rd_country_code;
    HCI_RDBD_ADDR hci_rd_bd;

    BTS2S_HCI_RD_FAILED_CONTACT_COUNT hci_rd_failed_contact_count;
    BTS2S_HCI_RESET_FAILED_CONTACT_COUNT hci_reset_failed_contact_count;
    BTS2S_HCI_GET_LINK_QA hci_get_link_qa;
    BTS2S_HCI_RD_RSSI hci_rd_rssi;
    BTS2S_HCI_RD_AFH_CHNL_MAP hci_rd_afh_chnl_map;
    BTS2S_HCI_RD_CLOCK hci_rd_clock;

    BTS2S_HCI_RD_LOOPBACK_MODE hci_rd_loopback_mode;
    BTS2S_HCI_WR_LOOPBACK_MODE hci_wr_loopback_mode;
    BTS2S_HCI_ENB_DUT_MODE hci_enb_dut_mode;

    BTS2S_HCI_MNFR_EXTENSION hci_mnfr_extension;

    BTS2S_HCI_EV_INQUIRY_COMP hci_inquiry_comp_ev;
    BTS2S_HCI_EV_INQUIRY_RES_ext hci_inquiry_res_ev;
    BTS2S_HCI_EV_CONN_COMP hci_conn_comp_ev;
    BTS2S_HCI_EV_CONN_REQ hci_conn_req_ev;
    BTS2S_HCI_EV_DISC_COMP hci_disc_comp_ev;
    BTS2S_HCI_EV_AUTH_COMP hci_auth_comp_ev;
    BTS2S_HCI_EV_RMT_NAME_REQ_COMP_ext hci_rmt_name_req_comp;
    BTS2S_HCI_EV_ENCRYPTION_CHANGE hci_enc_change_ev;
    BTS2S_HCI_EV_CHANGE_CONN_LINK_KEY_COMP hci_change_conn_linkkey_coml_ev;
    BTS2S_HCI_EV_MASTER_LINK_KEY_COMP hci_master_linkkey_compl_ev;
    BTS2S_HCI_EV_RD_REM_SUPP_FEATR_COMP hci_rd_rem_supp_featr_ev;
    BTS2S_HCI_EV_RD_RMT_VER_INFO_COMP hci_rd_rem_ver_info_compl_ev;
    BTS2S_HCI_EV_QOS_SETUP_COMP hci_qos_setup_compl_ev;
    BTS2S_HCI_EV_CMD_COMP hci_cmd_comp_ev;
    BTS2S_HCI_EV_CMD_STS hci_cmd_st_ev;
    BTS2S_HCI_EV_HARDWARE_ERR hci_hardware_err_ev;
    BTS2S_HCI_EV_FLUSH_OCCURRED hci_flush_occurred_ev;
    BTS2S_HCI_EV_ROLE_CHANGE hci_role_change_ev;
    BTS2S_HCI_EV_NUM_COMPD_PKTS_ext hci_num_compl_pkt_ev;
    BTS2S_HCI_EV_MODE_CHANGE hci_mode_change_ev;
#ifdef CFG_OPEN_FULL_HCI_FUNC
    BTS2S_HCI_EV_RETURN_LINK_KEYS hci_ret_link_keys_ev;
#endif
    BTS2S_HCI_EV_PIN_CODE_REQ hci_pin_code_req_ev;
    BTS2S_HCI_EV_LINK_KEY_REQ hci_link_key_req_ev;
    BTS2S_HCI_EV_LINK_KEY_NOTI hci_link_key_notif_ev;
    BTS2S_HCI_EV_LOOPBACK_CMD hci_loopback_cmd_ev;
    BTS2S_HCI_EV_DATA_BUFF_OVERFLOW hci_data_buff_overflow_ev;
    BTS2S_HCI_EV_MAX_SLOTS_CHANGE hci_max_slots_change_ev;
    BTS2S_HCI_EV_RD_CLOCK_OFFSET_COMP hci_rd_clock_offset_compl_ev;
    BTS2S_HCI_EV_CONN_PKT_TYPE_CHANGED hci_conn_pkt_type_chge_ev;
    BTS2S_HCI_EV_QOS_VIOLATION hci_qos_violation_ev;
    BTS2S_HCI_EV_PAGE_SCAN_MODE_CHANGE hci_pagescan_mode_chge_ev;
    BTS2S_HCI_EV_PAGE_SCAN_REP_MODE_CHANGE hci_pagescan_repmode_chge_ev;
    BTS2S_HCI_EV_FLOW_SPEC_COMP hci_flow_spec_comp_ev;
    BTS2S_HCI_EV_INQUIRY_RES_WITH_RSSI_ext hci_inquiry_res_with_rssi_ev;
    BTS2S_HCI_EV_RD_REM_EXT_FEATR_COMP hci_rd_rem_ext_featr_ev;
    BTS2S_HCI_EV_FIXED_ADDR hci_fixed_addr;
    BTS2S_HCI_EV_ALIAS_ADDR hci_alias_addr;
    BTS2S_HCI_EV_GENERATE_ALIAS_REQ hci_generate_alias_req;
    BTS2S_HCI_EV_ACT_ADDR hci_act_addr;
    BTS2S_HCI_EV_ALLOW_PRIVATE_PAIRING hci_allow_private_pairing;
    BTS2S_HCI_EV_ALIAS_ADDR_REQ hci_alias_addr_req;
    BTS2S_HCI_EV_ALIAS_NOT_RECOGNISED hci_alias_not_recognized;
    BTS2S_HCI_EV_FIXED_ADDR_ATTMPT hci_fixed_addr_attmpt;
    BTS2S_HCI_EV_SYNC_CONN_COMP hci_sync_conn_comp_ev;
    BTS2S_HCI_EV_SYNC_CONN_CHANGED hci_sync_conn_changed_ev;

    BTS2S_HCI_EV_MNFR_EXTENSION hci_mnfr_extension_ev;
#ifdef CFG_BT_VER_21
    BTS2S_HCI_EV_EXT_INQ_RESPONSE_ext hci_ev_ext_inquiry_res;
    BTS2S_HCI_RD_LOCAL_OOB_DATA_RESP_M_RET hci_ev_rd_local_oob_data_res;
#endif
} HCI_UMSG;

typedef struct BTS2S_HCI_QUE_TAG
{
    struct BTS2S_HCI_QUE_TAG *next_item;
    HCI_UMSG *msg;
    U16 size;
} BTS2S_HCI_QUE;

#define DEFAULT_ACL_PKT_TYPE    (HCI_PKT_DM1 | HCI_PKT_DH1 | HCI_PKT_DM3 | HCI_PKT_DH3 | HCI_PKT_DM5 | HCI_PKT_DH5)
#define DEFAULT_SCO_PKT_TYPE    (HCI_PKT_HV3)

#define DM_MSG_BASE            0x0000
#define DM_ACL_MSG             0x2000
#define DM_SCO_MSG             0x2400
#define DM_PRIV_MSG            0x2800 /* cmds specific to this dev manager */
#define DM_SM_MSG              0x2C00
#define DM_EN_MSG              0x3000
#define DM_SYNC_MSG            0x7C00

#define DM_LC_MSG              HCI_LINK
#define DM_LP_MSG              HCI_POLICY
#define DM_BB_MSG              HCI_HOST_BB
#define DM_INF_MSG             HCI_INFO
#define DM_STS_MSG             HCI_STS
#define DM_TEST_MSG            HCI_TEST

#define DM_1P2_MSG             0x7800
#define DM_OGF_MASK            0xFC00
#define DM_OPP_CLTODE_MASK     0x03FF


/* first add your msg here... */
typedef enum BTS2E_DM_MSG_TAG
{
    /* registration msgs (applicable to AM ui only, SCO and ACL registration msgs are supplied by the SCO and ACL uis) */
    ENUM_DM_AM_REG_REQ = DM_MSG_BASE,
    ENUM_DM_AM_REG_CFM,

    /* ACL conn ui msgs - for L2C use only */
    ENUM_SEP_DM_ACL_FIRST = DM_ACL_MSG, /* not a msg */
    ENUM_DM_ACL_REG_REQ,
    ENUM_DM_ACL_REG_CFM,
    ENUM_DM_ACL_CONN_REQ,
    ENUM_DM_ACL_CONN_CFM,
    ENUM_DM_ACL_CONN_IND,
    ENUM_DM_ACL_DISC_REQ,
    ENUM_DM_ACL_DISC_IND,
    ENUM_DM_ACL_BT_CLOSE_REQ,
    ENUM_DM_ACL_BT_OPEN_REQ,
    ENUM_DM_ACL_BUFF_SIZE_IND,
    ENUM_DM_ACL_DATA_SENT_IND,
    ENUM_DM_CONNLESS_CH_REG_REQ,

    //extend message
    ENUM_DM_ACL_CANCEL_CONN_REQ,
    ENUM_SEP_DM_ACL_LAST, /* not a msg */

    /* SCO conn ui msgs */
    ENUM_SEP_DM_SCO_FIRST = DM_SCO_MSG, /* not a msg */
    ENUM_OBSOLETE_DM_SCO_INCOMING_REG_REQ,
    ENUM_OBSOLETE_DM_SCO_INCOMING_UNREG_REQ,
    ENUM_OBSOLETE_DM_SCO_CONN_REQ,
    ENUM_OBSOLETE_DM_SCO_CONN_CFM,
    ENUM_OBSOLETE_DM_SCO_CONN_IND,
    ENUM_OBSOLETE_DM_SCO_CONN_RES,
    ENUM_OBSOLETE_DM_SCO_DISC_REQ,
    ENUM_OBSOLETE_DM_SCO_DISC_IND,
    ENUM_OBSOLETE_DM_SCO_BUFF_SIZE_IND,
    ENUM_OBSOLETE_DM_SCO_DATA_SENT_IND,
    ENUM_SEP_DM_SCO_LAST, /* not a msg */

    /* secu mgm msgs */
    ENUM_SEP_DM_SM_DOWN_FIRST = DM_SM_MSG, /* not a msg */
    ENUM_DM_SM_SET_DEFAULT_SECU_REQ,
    ENUM_DM_SM_REG_REQ,
    ENUM_DM_SM_UNREG_REQ,
    ENUM_DM_SM_REG_OUTGOING_REQ,
    ENUM_DM_SM_UNREG_OUTGOING_REQ,
    ENUM_DM_SM_ACCESS_REQ,
    ENUM_DM_SM_SET_SEC_MODE_REQ,
    ENUM_DM_SM_ADD_DEV_REQ,
    ENUM_DM_SM_REMOVE_DEV_REQ,
    ENUM_DM_SM_LINK_KEY_REQ_RES,
    ENUM_DM_SM_PIN_REQ_RES,
    ENUM_DM_SM_AUTHORISE_RES,
    ENUM_DM_SM_AUTH_REQ,
    ENUM_DM_SM_ENCRYPT_REQ,
    ENUM_DM_SM_L2C_REG_REQ,
    ENUM_SEP_DM_SM_DOWN_LAST, /* not a msg */

    ENUM_SEP_DM_SM_UP_FIRST, /* not a msg */
    ENUM_DM_SM_ACCESS_CFM,
    ENUM_DM_SM_SET_SEC_MODE_CFM,
    ENUM_DM_SM_ADD_DEV_CFM,
    ENUM_DM_SM_REMOVE_DEV_CFM,
    ENUM_DM_SM_LINK_KEY_REQ_IND,
    ENUM_DM_SM_PIN_REQ_IND,
    ENUM_DM_SM_LINK_KEY_IND,
    ENUM_DM_SM_AUTHORISE_IND,
    ENUM_DM_SM_AUTH_CFM,
    ENUM_DM_SM_ENCRYPT_CFM,
    ENUM_DM_SM_ENCRYPTION_CHANGE,
    ENUM_DM_SM_L2C_CLRX_ENB_IND,
#ifdef CFG_BT_VER_21
    ENUM_DM_HCI_IO_CAPABILITY_REQ_IND,
    ENUM_DM_HCI_IO_CAPABILITY_RSP_IND,
#endif
    ENUM_DM_SM_AUTH_IND,
    ENUM_SEP_DM_SM_UP_LAST,

    /* dev manager private msgs, not HCI cmds */
    ENUM_SEP_DM_PRIV_DOWN_FIRST = DM_PRIV_MSG, /* not a msg */
    ENUM_DM_WR_CACHED_PAGE_MODE_REQ,
    ENUM_DM_WR_CACHED_CLOCK_OFFSET_REQ,
    ENUM_DM_CLEAR_PARAM_CACHE_REQ,
    ENUM_DM_SET_DEFAULT_LINK_POLICY,
    ENUM_DM_ACL_OPEN_REQ,
    ENUM_DM_ACL_CLOSE_REQ,
    ENUM_SEP_DM_PRIV_DOWN_LAST, /* not a msg */

    ENUM_DM_WR_CACHED_PAGE_MODE_CFM,
    ENUM_DM_WR_CACHED_CLOCK_OFFSET_CFM,
    ENUM_DM_CLEAR_PARAM_CACHE_CFM,
    ENUM_DM_HC_TO_HOST_FLOW_CTRL,
    ENUM_DM_ACL_OPEN_CFM,
    ENUM_DM_ACL_OPENED_IND,
    ENUM_DM_ACL_CLOSED_IND,
    ENUM_DM_ACL_OPEN_IND,
    ENUM_SEP_DM_PRIV_UP_LAST, /*not a msg */

    /* HCI API msgs */

    /* link ctrl */

    /* dw_msg msgs */
    ENUM_SEP_DM_LC_DOWN_FIRST = HCI_LINK, /* not a msg */

    ENUM_DM_HCI_INQUIRY = HCI_INQUIRY,
    ENUM_DM_HCI_INQUIRY_ESC = HCI_INQUIRY_ESC,
    ENUM_DM_HCI_PERIODIC_INQUIRY = HCI_PERIODIC_INQUIRY_MODE,
    ENUM_DM_HCI_EXIT_PERIODIC_INQUIRY = HCI_EXIT_PERIODIC_INQUIRY_MODE,
    ENUM_DM_HCI_CHANGE_PKT_TYPE = HCI_CHANGE_CONN_PKT_TYPE,
    ENUM_DM_HCI_CHANGE_LINK_KEY = HCI_CHANGE_CONN_LINK_KEY,
    ENUM_DM_HCI_MASTER_LINK_KEY = HCI_MASTER_LINK_KEY,
    ENUM_DM_HCI_RMT_NAME_REQ = HCI_RMT_NAME_REQ,
    ENUM_DM_HCI_RD_RMT_FEATR = HCI_RD_RMT_SUPP_FEATR,
    ENUM_DM_HCI_RD_RMT_VERSION = HCI_RD_RMT_VER_INFO,
    ENUM_DM_HCI_RD_CLOCK_OFFSET = HCI_RD_CLOCK_OFFSET,
#ifdef CFG_BT_VER_21
    ENUM_DM_HCI_IO_CAPABILITY_REQ_REPLY = HCI_IO_CAPABILITY_REQ_REPLY,
    ENUM_DM_HCI_USER_CONF_REQUEST_REPLY = HCI_USR_CNFIRM_REQ_REPLY,
    ENUM_DM_HCI_USER_CONF_REQUEST_NEG_REPLY = HCI_USR_CNFIRAM_REQ_NEG_REPLY,
    ENUM_DM_HCI_USER_UPASSKEY_REPLY = HCI_USR_PASSKEY_REQ_REPLY,
    ENUM_DM_HCI_USER_UPASSKEY_NEG_REPLY = HCI_USR_PASSKEY_NEG_REQ_REPLY,
    ENUM_DM_HCI_RMT_OOB_DATA_REPLY = HCI_RMT_OOB_DATA_REQ_REPLY,
    ENUM_DM_HCI_RMT_OOB_DATA_NEG_REPLY = HCI_RMT_OOB_DATA_REQ_NEG_REPLY,
    ENUM_DM_HCI_IO_CAPABILITY_REQ_NEG_RSP = HCI_IO_CAPABILITY_REQ_NEG_REPLY,
    ENUM_DM_HCI_SEND_KEYPRESS_NOTIFI  = HCI_KEYPRESS_NOTIFI,
#endif

    ENUM_SEP_DM_LC_DOWN_LAST = HCI_LINK | HCI_MAX_LINK_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_INQUIRY_ESC_COMP,
    ENUM_DM_HCI_PERIODIC_INQUIRY_COMP,
    ENUM_DM_HCI_EXIT_PERIODIC_INQUIRY_COMP,
    ENUM_DM_HCI_INQUIRY_RES,

#ifdef CFG_BT_VER_21
    ENUM_DM_HCI_EXT_INQUIRY_RES,
    ENUM_DM_HCI_USER_CONF_REQ,
    ENUM_DM_HCI_USER_PASSKY_REQ,
    ENUM_DM_HCI_PASSKEY_NOTIFI,
    ENUM_DM_HCI_RMT_OOB_DATA_REQ,
    ENUM_DM_HCI_SSP_COMP_REQ,
    ENUM_DM_HCI_KEYPRESS_NOTIFI,
#endif


    ENUM_DM_HCI_INQUIRY_COMP,
    ENUM_DM_HCI_LINK_KEY_CHANGE_COMP,
    ENUM_DM_HCI_RETURN_LINK_KEYS,
    ENUM_DM_HCI_MASTER_LINK_KEY_COMP,
    ENUM_DM_HCI_KEYMISSING_NOTIFI,
    ENUM_DM_HCI_RMT_NAME_COMP,
    ENUM_DM_HCI_RD_RMT_FEATR_COMP,
    ENUM_DM_HCI_RD_RMT_VERSION_COMP,
    ENUM_DM_HCI_RD_CLOCK_OFFSET_COMP,
    ENUM_SEP_DM_LC_UP_LAST, /* not a msg */

    /* link policy */
    /* dw_msg msgs */
    ENUM_SEP_DM_LP_DOWN_FIRST = HCI_POLICY, /* not a msg */

    ENUM_DM_HCI_HOLD_MODE = HCI_HOLD_MODE,
    ENUM_DM_HCI_SNIFF_MODE = HCI_SNIFF_MODE,
    ENUM_DM_HCI_EXIT_SNIFF_MODE = HCI_EXIT_SNIFF_MODE,
    ENUM_DM_HCI_PARK_MODE = HCI_PARK_MODE,
    ENUM_DM_HCI_EXIT_PARK_MODE = HCI_EXIT_PARK_MODE,
    ENUM_DM_HCI_QOS_SETUP_REQ = HCI_QOS_SETUP,
    ENUM_DM_HCI_ROLE_DISCOV = HCI_ROLE_DISCOV,
    ENUM_DM_HCI_SWITCH_ROLE = HCI_SWITCH_ROLE,
    ENUM_DM_HCI_RD_LP_SETTINGS = HCI_RD_LINK_POLICY_SETTINGS,
    ENUM_DM_HCI_WR_LP_SETTINGS = HCI_WR_LINK_POLICY_SETTINGS,

    ENUM_SEP_DM_LP_DOWN_LAST = HCI_POLICY | HCI_MAX_POLICY_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_MODE_CHANGE_EV,
    ENUM_DM_HCI_QOS_SETUP_CFM,
    ENUM_DM_HCI_QOS_VIOLATION_IND,
    ENUM_DM_HCI_ROLE_DISCOV_COMP,
    ENUM_DM_HCI_SWITCH_ROLE_COMP,
    ENUM_DM_HCI_RD_LP_SETTINGS_COMP,
    ENUM_DM_HCI_WR_LP_SETTINGS_COMP,
#ifdef  CFG_BT_VER_21
    ENUM_DM_HCI_SUBNIFF_MODE_CHANGE_EV = HCI_EV_SUBNIFF_RATING_EVENT,/* 0x2E */
#endif

    ENUM_SEP_DM_LP_UP_LAST, /* not a msg */

    /* host ctrller and baseband */
    /* dw_msg msgs */
    ENUM_SEP_DM_BB_DOWN_FIRST = HCI_HOST_BB, /* not a msg */

    ENUM_DM_HCI_SET_EV_MASK = HCI_SET_EV_MASK,
    ENUM_DM_HCI_RESET = HCI_RESET,
    ENUM_DM_HCI_SET_EV_FILTER = HCI_SET_EV_FILTER,
    ENUM_DM_HCI_FLUSH = HCI_FLUSH,
    ENUM_DM_HCI_RD_PIN_TYPE = HCI_RD_PIN_TYPE,
    ENUM_DM_HCI_WR_PIN_TYPE = HCI_WR_PIN_TYPE,
    ENUM_DM_HCI_CREATE_NEW_UNIT_KEY = HCI_CREATE_NEW_UNIT_KEY,
    ENUM_DM_HCI_RD_STORED_LINK_KEY = HCI_RD_STORED_LINK_KEY,
    ENUM_DM_HCI_WR_STORED_LINK_KEY = HCI_WR_STORED_LINK_KEY,
    ENUM_DM_HCI_DEL_STORED_LINK_KEY = HCI_DEL_STORED_LINK_KEY,
    ENUM_DM_HCI_CHANGE_LOCAL_NAME = HCI_CHANGE_LOCAL_NAME,
    ENUM_DM_HCI_RD_LOCAL_NAME = HCI_RD_LOCAL_NAME,
    ENUM_DM_HCI_RD_CONN_ACPT_TO = HCI_RD_CONN_ACPT_TIMEOUT,
    ENUM_DM_HCI_WR_CONN_ACPT_TO = HCI_WR_CONN_ACPT_TIMEOUT,
    ENUM_DM_HCI_RD_PAGE_TO = HCI_RD_PAGE_TIMEOUT,
    ENUM_DM_HCI_WR_PAGE_TO = HCI_WR_PAGE_TIMEOUT,
    ENUM_DM_HCI_RD_SCAN_ENB = HCI_RD_SCAN_ENB,
    ENUM_DM_HCI_WR_SCAN_ENB = HCI_WR_SCAN_ENB,
    ENUM_DM_HCI_RD_PAGESCAN_ACTIVITY = HCI_RD_PAGESCAN_ACTIVITY,
    ENUM_DM_HCI_WR_PAGESCAN_ACTIVITY = HCI_WR_PAGESCAN_ACTIVITY,
    ENUM_DM_HCI_RD_INQUIRYSCAN_ACTIVITY = HCI_RD_INQUIRYSCAN_ACTIVITY,
    ENUM_DM_HCI_WR_INQUIRYSCAN_ACTIVITY = HCI_WR_INQUIRYSCAN_ACTIVITY,
    ENUM_DM_HCI_RD_AUTH_ENB = HCI_RD_AUTH_ENB,
    ENUM_DM_HCI_RD_ENCRYPTION_MODE = HCI_RD_ENC_MODE,
    ENUM_DM_HCI_RD_DEV_CLS = HCI_RD_DEV_CLS,
    ENUM_DM_HCI_WR_DEV_CLS = HCI_WR_DEV_CLS,
    ENUM_DM_HCI_RD_VOICE_SETTING = HCI_RD_VOICE_SETTING,
    ENUM_DM_HCI_WR_VOICE_SETTING = HCI_WR_VOICE_SETTING,
    ENUM_DM_HCI_RD_AUTO_FLUSH_TIMEOUT = HCI_RD_AUTO_FLUSH_TIMEOUT,
    ENUM_DM_HCI_WR_AUTO_FLUSH_TIMEOUT = HCI_WR_AUTO_FLUSH_TIMEOUT,
    ENUM_DM_HCI_RD_NUM_BCAST_TXS = HCI_RD_NUM_BCAST_RETXS,
    ENUM_DM_HCI_WR_NUM_BCAST_TXS = HCI_WR_NUM_BCAST_RETXS,
    ENUM_DM_HCI_RD_HOLD_MODE_ACTIVITY = HCI_RD_HOLD_MODE_ACTIVITY,
    ENUM_DM_HCI_WR_HOLD_MODE_ACTIVITY = HCI_WR_HOLD_MODE_ACTIVITY,
    ENUM_DM_HCI_RD_TX_POWER_LEVEL = HCI_RD_TX_POWER_LEVEL,
    ENUM_DM_HCI_RD_SCO_FLOW_CTRL_ENB = HCI_RD_SCO_FLOW_CON_ENB,
    ENUM_DM_HCI_WR_SCO_FLOW_CTRL_ENB = HCI_WR_SCO_FLOW_CON_ENB,
    ENUM_DM_HCI_SET_HC_TO_HOST_FLOW = HCI_SET_HCTOHOST_FLOW_CTRL,
    ENUM_HCI_HOST_NUM_COMP_PKT = HCI_HOST_NUM_COMPD_PKTS,
    ENUM_DM_HCI_RD_LINK_SUPERV_TIMEOUT = HCI_RD_LINK_SUPERV_TIMEOUT,
    ENUM_DM_HCI_WR_LINK_SUPERV_TIMEOUT = HCI_WR_LINK_SUPERV_TIMEOUT,
    ENUM_DM_HCI_RD_NUM_SUPP_IAC = HCI_RD_NUM_SUPP_IAC,
    ENUM_DM_HCI_RD_CUR_IAC_LAP = HCI_RD_CUR_IAC_LAP,
    ENUM_DM_HCI_WR_CUR_IAC_LAP = HCI_WR_CUR_IAC_LAP,
    ENUM_DM_HCI_RD_PAGESCAN_PERIOD_MODE = HCI_RD_PAGESCAN_PERIOD_MODE,
    ENUM_DM_HCI_WR_PAGESCAN_PERIOD_MODE = HCI_WR_PAGESCAN_PERIOD_MODE,
    ENUM_DM_HCI_RD_PAGESCAN_MODE = HCI_RD_PAGESCAN_MODE,
    ENUM_DM_HCI_WR_PAGESCAN_MODE = HCI_WR_PAGESCAN_MODE,

    ENUM_SEP_DM_BB_DOWN_LAST = HCI_HOST_BB | HCI_MAX_HOST_BB_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_SET_EV_MASK_COMP,
    ENUM_DM_HCI_RESET_COMP,
    ENUM_DM_HCI_SET_EV_FILTER_COMP,
    ENUM_DM_HCI_FLUSH_COMP,
    ENUM_DM_HCI_RD_PIN_TYPE_COMP,
    ENUM_DM_HCI_WR_PIN_TYPE_COMP,
    ENUM_DM_HCI_CREATE_NEW_UNIT_KEY_COMP,
    ENUM_DM_HCI_RD_STORED_LINK_KEY_COMP,
    ENUM_DM_HCI_WR_STORED_LINK_KEY_COMP,
    ENUM_DM_HCI_DEL_STORED_LINK_KEY_COMP,
    ENUM_DM_HCI_CHANGE_LOCAL_NAME_COMP,
    ENUM_DM_HCI_RD_LOCAL_NAME_COMP,
    ENUM_DM_HCI_RD_CONN_ACPT_TO_COMP,
    ENUM_DM_HCI_WR_CONN_ACPT_TO_COMP,
    ENUM_DM_HCI_RD_PAGE_TO_COMP,
    ENUM_DM_HCI_WR_PAGE_TO_COMP,
    ENUM_DM_HCI_RD_SCAN_ENB_COMP,
    ENUM_DM_HCI_WR_SCAN_ENB_COMP,
    ENUM_DM_HCI_RD_PAGESCAN_ACTIVITY_COMP,
    ENUM_DM_HCI_WR_PAGESCAN_ACTIVITY_COMP,
    ENUM_DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP,
    ENUM_DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP,
    ENUM_DM_HCI_RD_AUTH_ENB_COMP,
    ENUM_DM_HCI_RD_ENCRYPTION_MODE_COMP,
    ENUM_DM_HCI_RD_DEV_CLS_COMP,
    ENUM_DM_HCI_WR_DEV_CLS_COMP,
    ENUM_DM_HCI_RD_VOICE_SETTING_COMP,
    ENUM_DM_HCI_WR_VOICE_SETTING_COMP,
    ENUM_DM_HCI_RD_AUTO_FLUSH_TIMEOUT_COMP,
    ENUM_DM_HCI_WR_AUTO_FLUSH_TIMEOUT_COMP,
    ENUM_DM_HCI_RD_NUM_BCAST_TXS_COMP,
    ENUM_DM_HCI_WR_NUM_BCAST_TXS_COMP,
    ENUM_DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP,
    ENUM_DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP,
    ENUM_DM_HCI_RD_TX_POWER_LEVEL_COMP,
    ENUM_DM_HCI_SET_HC_TO_HOST_FLOW_COMP,
    ENUM_HCI_HOST_NUM_COMP_PKT_COMP,
    ENUM_DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP,
    ENUM_DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP,
    ENUM_DM_HCI_RD_NUM_SUPP_IAC_COMP,
    ENUM_DM_HCI_RD_CUR_IAC_LAP_COMP,
    ENUM_DM_HCI_WR_CUR_IAC_LAP_COMP,
    ENUM_DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP,
    ENUM_DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP,
    ENUM_DM_HCI_RD_PAGESCAN_MODE_COMP,
    ENUM_DM_HCI_WR_PAGESCAN_MODE_COMP,
    ENUM_DM_HCI_HARDWARE_ERR,
    ENUM_DM_HCI_DATA_BUFF_OVERFLOW,
    ENUM_DM_HCI_MAX_SLOTS_CHANGE,
    ENUM_SEP_DM_BB_UP_LAST, /* not a msg */

    /* informational pars */
    /* dw_msg msgs */
    ENUM_SEP_DM_INF_DOWN_FIRST = HCI_INFO, /* not a msg */

    ENUM_DM_HCI_RD_LOCAL_VERSION = HCI_RD_LOCAL_VER_INFO,
    ENUM_DM_HCI_RD_LOCAL_FEATR = HCI_RD_LOCAL_SUPP_FEATR,
    ENUM_DM_HCI_RD_COUNTRY_CODE = HCI_RD_COUNTRY_CODE,
    ENUM_DM_HCI_RD_BD_ADDR = HCI_RD_BD_ADDR,

    ENUM_SEP_DM_INF_DOWN_LAST = HCI_INFO | HCI_MAX_INFO_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_RD_LOCAL_VERSION_COMP,
    ENUM_DM_HCI_RD_LOCAL_FEATR_COMP,
    ENUM_DM_HCI_RD_COUNTRY_CODE_COMP,
    ENUM_DM_HCI_RD_BD_ADDR_COMP,
    ENUM_SEP_DM_INF_UP_LAST, /* not a msg */

    /* st pars */
    /* dw_msg msgs */
    ENUM_SEP_DM_STS_DOWN_FIRST = HCI_STS, /* not a msg */

    ENUM_DM_HCI_FAILED_CONTACT_COUNTER = HCI_RD_FAILED_CONTACT_COUNT,
    ENUM_DM_HCI_RESET_CONTACT_COUNTER = HCI_RESET_FAILED_CONTACT_COUNT,
    ENUM_DM_HCI_GET_LINK_QA = HCI_GET_LINK_QA,
    ENUM_DM_HCI_RD_RSSI = HCI_RD_RSSI,

    ENUM_SEP_DM_STS_DOWN_LAST = HCI_STS | HCI_MAX_STS_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_FAILED_CONTACT_COUNTER_COMP,
    ENUM_DM_HCI_RESET_CONTACT_COUNTER_COMP,
    ENUM_DM_HCI_GET_LINK_QA_COMP,
    ENUM_DM_HCI_RD_RSSI_COMP,
    ENUM_SEP_DM_STS_UP_LAST, /* not a msg */

    /* testing msgs */
    /* dw_msg msgs */
    ENUM_SEP_DM_TEST_DOWN_FIRST = HCI_TEST, /* not a msg */

    ENUM_DM_HCI_RD_LOOPBACK_MODE = HCI_RD_LOOPBACK_MODE,
    ENUM_DM_HCI_WR_LOOPBACK_MODE = HCI_WR_LOOPBACK_MODE,
    ENUM_DM_HCI_ENB_DEV_UT_MODE = HCI_ENB_DUT_MODE,

    ENUM_SEP_DM_TEST_DOWN_LAST = HCI_TEST | HCI_MAX_TEST_OCF_V1_1, /* not a msg */

    /* umsg msgs */
    ENUM_DM_HCI_RD_LOOPBACK_MODE_COMP,
    ENUM_DM_HCI_WR_LOOPBACK_MODE_COMP,
    ENUM_DM_HCI_LOOPBACK_EV,
    ENUM_DM_HCI_ENB_DEV_UT_MODE_COMP,
    ENUM_SEP_DM_TEST_UP_LAST, /* not a msg */

    /* add everything above here */
    ENUM_SEP_DM_LAST_MSG,

    ENUM_DM_HCI_CREATE_CONN_ESC = 0x7000,
    ENUM_DM_HCI_RMT_NAME_REQ_ESC,
    ENUM_DM_HCI_RD_RMT_EXT_FEATR,
    ENUM_DM_HCI_RD_LMP_HANDLE,

    ENUM_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS,
    ENUM_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS,
    ENUM_DM_HCI_FLOW_SPEC,

    ENUM_DM_HCI_SET_AFH_CHNL_CLS,
    ENUM_DM_HCI_RD_INQUIRY_SCAN_TYPE,
    ENUM_DM_HCI_WR_INQUIRY_SCAN_TYPE,
    ENUM_DM_HCI_RD_INQUIRY_MODE,
    ENUM_DM_HCI_WR_INQUIRY_MODE,
    ENUM_DM_HCI_RD_PAGE_SCAN_TYPE,
    ENUM_DM_HCI_WR_PAGE_SCAN_TYPE,
    ENUM_DM_HCI_RD_AFH_CHNL_CLS_M,
    ENUM_DM_HCI_WR_AFH_CHNL_CLS_M,

    ENUM_DM_HCI_RD_LOCAL_EXT_FEATR,
    ENUM_DM_HCI_RD_AFH_CHNL_MAP,
    ENUM_DM_HCI_RD_CLOCK,

    ENUM_OBSOLETE_DM_HCI_SETUP_SYNCHRONOUS_CONN,
    ENUM_OBSOLETE_DM_HCI_ACPT_SYNCHRONOUS_CONN_REQ,
    ENUM_OBSOLETE_DM_HCI_REJ_SYNCHRONOUS_CONN,

    ENUM_DM_HCI_CREATE_CONN_ESC_COMP = DM_1P2_MSG,
    ENUM_DM_HCI_RMT_NAME_REQ_ESC_COMP,
    ENUM_DM_HCI_RD_RMT_EXT_FEATR_COMP,
    ENUM_DM_HCI_RD_LMP_HANDLE_COMP,
    ENUM_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_COMP,
    ENUM_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_COMP,
    ENUM_DM_HCI_FLOW_SPEC_COMP,


    ENUM_DM_HCI_SET_AFH_CHNL_CLS_COMP,
    ENUM_DM_HCI_RD_INQUIRY_SCAN_TYPE_COMP,
    ENUM_DM_HCI_WR_INQUIRY_SCAN_TYPE_COMP,
    ENUM_DM_HCI_RD_INQUIRY_MODE_COMP,
    ENUM_DM_HCI_WR_INQUIRY_MODE_COMP,
    ENUM_DM_HCI_RD_PAGE_SCAN_TYPE_COMP,
    ENUM_DM_HCI_WR_PAGE_SCAN_TYPE_COMP,
    ENUM_DM_HCI_RD_AFH_CHNL_CLS_M_COMP,
    ENUM_DM_HCI_WR_AFH_CHNL_CLS_M_COMP,

    ENUM_DM_HCI_RD_LOCAL_EXT_FEATR_COMP,
    ENUM_DM_HCI_RD_AFH_CHNL_MAP_COMP,
    ENUM_DM_HCI_RD_CLOCK_COMP,

    ENUM_OBSOLETE_DM_HCI_SETUP_SYNCHRONOUS_CONN_COMP,
    ENUM_OBSOLETE_DM_HCI_ACPT_SYNCHRONOUS_CONN_REQ_COMP,
    ENUM_OBSOLETE_DM_HCI_REJ_SYNCHRONOUS_CONN_COMP,
    ENUM_DM_HCI_INQUIRY_RES_WITH_RSSI,


#ifndef CFG_NOT_STK_EN
    /* dev manager bts2s_enhancement */
    ENUM_DM_EN_ENB_ENHANCEMENTS_REQ = DM_EN_MSG,
    ENUM_DM_EN_ENB_ENHANCEMENTS_CFM,
    ENUM_DM_EN_RD_ENHANCEMENTS_REQ,
    ENUM_DM_EN_RD_ENHANCEMENTS_CFM,
    ENUM_DM_EN_ACL_OPENED_IND,
    ENUM_DM_EN_ACL_CLOSED_IND,
#endif

    /* link policy power st msgs */
    ENUM_DM_LP_WR_POWERSTS_REQ = 0x7900,
    ENUM_DM_LP_WR_POWERSTS_CFM,

    /* synchronous conn ui msgs */
    ENUM_SEP_DM_SYNC_FIRST = DM_SYNC_MSG, /* not a msg */
    ENUM_DM_SYNC_REG_REQ,
    ENUM_DM_SYNC_UNREG_REQ,
    ENUM_DM_SYNC_CONN_REQ,
    ENUM_DM_SYNC_CONN_CFM,
    ENUM_DM_SYNC_CONN_COMP_IND,
    ENUM_DM_SYNC_CONN_IND,
    ENUM_DM_SYNC_CONN_RES,
    ENUM_DM_SYNC_DISC_REQ,
    ENUM_DM_SYNC_DISC_IND,
    ENUM_DM_SYNC_DISC_CFM,
    ENUM_DM_SYNC_REG_CFM,
    ENUM_DM_SYNC_UNREG_CFM,
    ENUM_DM_SYNC_RENEGOTIATE_REQ,
    ENUM_DM_SYNC_RENEGOTIATE_IND,
    ENUM_DM_SYNC_RENEGOTIATE_CFM,
    ENUM_SEP_DM_SYNC_LAST /* not a msg */
} BTS2E_DM_MSG;

/* used for BT2.1 */
typedef enum BTS2E_DM_IO_CAPABILITY_TAG
{
    /* IO_Capability: Value Parameter Description */
    DISPLAY_ONLY = 0X00,
    DISPLAY_YES_NO = 0X01,
    KEYBOARD_ONLY = 0X02,
    NO_INPUT_NO_OUTPUT = 0X03,
    IO_CAPABILITY_RESERVED  = 0X04 /* 0X04 - 0XFF */
} BTS2E_IO_CAPABILITY;

typedef enum BTS2E_DM_KEYPRESS_NOTIFI_TAG
{
    /* Value Parameter Description */
    KEYPRESS_NOTIFI_STARTED = 0X00,   /* Passkey entry started */
    KEYPRESS_NOTIFI_ENTERED = 0X01,   /* 1 Passkey digit entered */
    KEYPRESS_NOTIFI_ERASED = 0X02,    /* Passkey digit erased */
    KEYPRESS_NOTIFI_CLEARED = 0X03,   /* Passkey cleared */
    KEYPRESS_NOTIFI_COMPLETED = 0X04, /* 4 Passkey entry completed */
    KEYPRESS_NOTIFI_RESERVED = 0X05   /* 5-255 Reserved for future use */
} BTS2E_KEYPRESS_NOTIFI;

/* HCI Authentication_Requirements */
typedef enum BTS2E_DM_IO_AUTH_REQUIRE_TAG
{
    MITM_PROTECT_SINGLE_PROFILE_NOT_REQUIRED = 0x00,/*Numeric comparison with automatic accept allowed*/
    MITM_PROTECT_SINGLE_PROFILE_REQUIRED = 0x01,    /*Use IO Capabilities to determine authentication procedure*/
    MITM_PROTECT_ALL_PROFILE_NOT_REQUIRED = 0x02,   /*Numeric comparison with automatic accept allowed*/
    MITM_PROTECT_ALL_PROFILE_REQUIRED = 0x03,       /*Use IO Capabilities to determine authentication procedure*/
    MITM_PROTECT_RESERVED = 0x04
} BTS2E_DM_IO_AUTH_REQUIRE;
#define MITM_NOT_REQUIRED_NO_BONDING        0x00
#define MITM_REQUIRED_NO_BONDING            0x01
#define MITM_NOT_REQUIRED_DEDICATED_BONDING 0x02
#define MITM_REQUIRED_DEDICATED_BONDING     0x03
#define MITM_NOT_REQUIRED_GENERAL_BONDING   0x04
#define MITM_REQUIRED_GENERAL_BONDING       0x05



typedef enum BTS2E_DM_OOB_DATA_PRESENT_TAG
{
    /* Value Parameter Description */
    OOB_AUTH_DATA_NOT_PRESENT = 0X00,               /* 0x00 OOB authentication data not present */
    OOB_AUTH_DATA_FROM_RMT_DEVICE_PRESENT = 0X01,   /* 0x01 OOB authentication data from remote device present */
    OOB_DATA_PRESENT_RESERVED = 0X02                /* 0x02-0xFF Reserved for future use */
} BTS2E_DM_OOB_DATA_PRESENT;

/* and then reflect it here */
#define HCI_GAP_MGM_REG_REQ  ((U16)ENUM_DM_AM_REG_REQ)
#define HCI_GAP_MGM_REG_CFM  ((U16)ENUM_DM_AM_REG_CFM)

/* ACL conn ui msgs */
#define DM_ACL_REG_REQ              ((U16)(ENUM_DM_ACL_REG_REQ))
#define DM_ACL_REG_CFM              ((U16)(ENUM_DM_ACL_REG_CFM))
#define DM_ACL_CONN_REQ             ((U16)(ENUM_DM_ACL_CONN_REQ))
#define DM_ACL_CONN_CFM             ((U16)(ENUM_DM_ACL_CONN_CFM))
#define DM_ACL_CONN_IND             ((U16)(ENUM_DM_ACL_CONN_IND))
#define DM_ACL_DISC_REQ             ((U16)(ENUM_DM_ACL_DISC_REQ))
#define DM_ACL_BT_CLOSE_REQ         ((U16)(ENUM_DM_ACL_BT_CLOSE_REQ))
#define DM_ACL_BT_OPEN_REQ          ((U16)(ENUM_DM_ACL_BT_OPEN_REQ))
#define DM_ACL_BT_CANCEL_CONN_REQ   ((U16)(ENUM_DM_ACL_CANCEL_CONN_REQ))


#define DM_ACL_DISC_IND             ((U16)(ENUM_DM_ACL_DISC_IND))
#define DM_ACL_BUFF_SIZE_IND        ((U16)(ENUM_DM_ACL_BUFF_SIZE_IND))
#define DM_ACL_DATA_SENT_IND        ((U16)(ENUM_DM_ACL_DATA_SENT_IND))
#define DM_CONNLESS_CH_REG_REQ      ((U16)(ENUM_DM_CONNLESS_CH_REG_REQ))

/* synchronous conn ui msgs */
#define DM_SYNC_REG_REQ             ((U16)(ENUM_DM_SYNC_REG_REQ))
#define DM_SYNC_REG_CFM             ((U16)(ENUM_DM_SYNC_REG_CFM))
#define DM_SYNC_UNREG_REQ           ((U16)(ENUM_DM_SYNC_UNREG_REQ))
#define DM_SYNC_UNREG_CFM           ((U16)(ENUM_DM_SYNC_UNREG_CFM))
#define DM_SYNC_CONN_REQ            ((U16)(ENUM_DM_SYNC_CONN_REQ))
#define DM_SYNC_CONN_CFM            ((U16)(ENUM_DM_SYNC_CONN_CFM))
#define DM_SYNC_CONN_COMP_IND       ((U16)(ENUM_DM_SYNC_CONN_COMP_IND))
#define DM_SYNC_CONN_IND            ((U16)(ENUM_DM_SYNC_CONN_IND))
#define DM_SYNC_CONN_RES            ((U16)(ENUM_DM_SYNC_CONN_RES))
#define DM_SYNC_RENEGOTIATE_REQ     ((U16)(ENUM_DM_SYNC_RENEGOTIATE_REQ))
#define DM_SYNC_RENEGOTIATE_IND     ((U16)(ENUM_DM_SYNC_RENEGOTIATE_IND))
#define DM_SYNC_RENEGOTIATE_CFM     ((U16)(ENUM_DM_SYNC_RENEGOTIATE_CFM))
#define DM_SYNC_DISC_REQ            ((U16)(ENUM_DM_SYNC_DISC_REQ))
#define DM_SYNC_DISC_IND            ((U16)(ENUM_DM_SYNC_DISC_IND))
#define DM_SYNC_DISC_CFM            ((U16)(ENUM_DM_SYNC_DISC_CFM))

/* secu mgm msgs */

#define DM_SM_SET_DEFAULT_SECU_REQ  ((U16)(ENUM_DM_SM_SET_DEFAULT_SECU_REQ))
#define DM_SM_REG_REQ               ((U16)(ENUM_DM_SM_REG_REQ))
#define DM_SM_UNREG_REQ             ((U16)(ENUM_DM_SM_UNREG_REQ))
#define DM_SM_REG_OUTGOING_REQ      ((U16)(ENUM_DM_SM_REG_OUTGOING_REQ))
#define DM_SM_UNREG_OUTGOING_REQ    ((U16)(ENUM_DM_SM_UNREG_OUTGOING_REQ))
#define DM_SM_ACCESS_REQ            ((U16)(ENUM_DM_SM_ACCESS_REQ))
#define DM_SM_ACCESS_CFM            ((U16)(ENUM_DM_SM_ACCESS_CFM))
#define DM_SM_SET_SEC_MODE_REQ      ((U16)(ENUM_DM_SM_SET_SEC_MODE_REQ))
#define DM_SM_SET_SEC_MODE_CFM      ((U16)(ENUM_DM_SM_SET_SEC_MODE_CFM))
#define DM_SM_ADD_DEV_REQ           ((U16)(ENUM_DM_SM_ADD_DEV_REQ))
#define DM_SM_ADD_DEV_CFM           ((U16)(ENUM_DM_SM_ADD_DEV_CFM))
#define DM_SM_REMOVE_DEV_REQ        ((U16)(ENUM_DM_SM_REMOVE_DEV_REQ))
#define DM_SM_REMOVE_DEV_CFM        ((U16)(ENUM_DM_SM_REMOVE_DEV_CFM))
#define DM_SM_LINK_KEY_REQ_IND      ((U16)(ENUM_DM_SM_LINK_KEY_REQ_IND))
#define DM_SM_LINK_KEY_REQ_RES      ((U16)(ENUM_DM_SM_LINK_KEY_REQ_RES))
#define DM_SM_PIN_REQ_IND           ((U16)(ENUM_DM_SM_PIN_REQ_IND))
#define DM_SM_PIN_REQ_RES           ((U16)(ENUM_DM_SM_PIN_REQ_RES))
#define DM_SM_LINK_KEY_IND          ((U16)(ENUM_DM_SM_LINK_KEY_IND))
#define DM_SM_AUTHORISE_IND         ((U16)(ENUM_DM_SM_AUTHORISE_IND))
#define DM_SM_AUTHORISE_RES         ((U16)(ENUM_DM_SM_AUTHORISE_RES))
#define DM_SM_AUTH_REQ              ((U16)(ENUM_DM_SM_AUTH_REQ))
#define DM_SM_AUTH_CFM              ((U16)(ENUM_DM_SM_AUTH_CFM))
#define DM_SM_AUTH_IND              ((U16)(ENUM_DM_SM_AUTH_IND))
#define DM_SM_ENCRYPT_REQ           ((U16)(ENUM_DM_SM_ENCRYPT_REQ))
#define DM_SM_ENCRYPT_CFM           ((U16)(ENUM_DM_SM_ENCRYPT_CFM))
#define DM_SM_ENCRYPTION_CHANGE     ((U16)(ENUM_DM_SM_ENCRYPTION_CHANGE))
#define DM_SM_L2C_REG_REQ           ((U16)(ENUM_DM_SM_L2C_REG_REQ))
#define DM_SM_L2C_CLRX_ENB_IND      ((U16)(ENUM_DM_SM_L2C_CLRX_ENB_IND))

#define DM_SM_BASE                      (ENUM_SEP_DM_SM_DOWN_FIRST + 1)
#define DM_SM_DOWN_MAX                  (ENUM_SEP_DM_SM_DOWN_LAST - 1)
#define DM_SM_MAX                       (ENUM_SEP_DM_SM_UP_LAST - 1)

/* DM private msgs */

#define DM_WR_CACHED_PAGE_MODE_REQ      ((U16)(ENUM_DM_WR_CACHED_PAGE_MODE_REQ))
#define DM_WR_CACHED_PAGE_MODE_CFM      ((U16)(ENUM_DM_WR_CACHED_PAGE_MODE_CFM))
#define DM_WR_CACHED_CLOCK_OFFSET_REQ   ((U16)(ENUM_DM_WR_CACHED_CLOCK_OFFSET_REQ))
#define DM_WR_CACHED_CLOCK_OFFSET_CFM   ((U16)(ENUM_DM_WR_CACHED_CLOCK_OFFSET_CFM))
#define DM_CLEAR_PARAM_CACHE_REQ        ((U16)(ENUM_DM_CLEAR_PARAM_CACHE_REQ))
#define DM_CLEAR_PARAM_CACHE_CFM        ((U16)(ENUM_DM_CLEAR_PARAM_CACHE_CFM))
#define DM_HC_TO_HOST_FLOW_CTRL         ((U16)(ENUM_DM_HC_TO_HOST_FLOW_CTRL))
#define DM_ACL_OPEN_REQ                 ((U16)(ENUM_DM_ACL_OPEN_REQ))
#define DM_ACL_OPEN_CFM                 ((U16)(ENUM_DM_ACL_OPEN_CFM))
#define DM_ACL_CLOSE_REQ                ((U16)(ENUM_DM_ACL_CLOSE_REQ))
#define DM_ACL_OPENED_IND               ((U16)(ENUM_DM_ACL_OPENED_IND))
#define DM_ACL_CLOSED_IND               ((U16)(ENUM_DM_ACL_CLOSED_IND))
#define DM_ACL_OPEN_IND                 ((U16)ENUM_DM_ACL_OPEN_IND)
#define DM_SET_DEFAULT_LINK_POLICY      ((U16)(ENUM_DM_SET_DEFAULT_LINK_POLICY))

#define DM_LP_WR_POWERSTS_REQ           ((U16)(ENUM_DM_LP_WR_POWERSTS_REQ))
#define DM_LP_WR_POWERSTS_CFM           ((U16)(ENUM_DM_LP_WR_POWERSTS_CFM))

#ifndef CFG_NOT_STK_EN
#define DM_EN_ENB_ENHANCEMENTS_REQ      ((U16)(ENUM_DM_EN_ENB_ENHANCEMENTS_REQ))
#define DM_EN_ENB_ENHANCEMENTS_CFM      ((U16)(ENUM_DM_EN_ENB_ENHANCEMENTS_CFM))
#define DM_EN_RD_ENHANCEMENTS_REQ       ((U16)(ENUM_DM_EN_RD_ENHANCEMENTS_REQ))
#define DM_EN_RD_ENHANCEMENTS_CFM       ((U16)(ENUM_DM_EN_RD_ENHANCEMENTS_CFM))
#define DM_EN_ACL_OPENED_IND            ((U16)(ENUM_DM_EN_ACL_OPENED_IND))
#define DM_EN_ACL_CLOSED_IND            ((U16)(ENUM_DM_EN_ACL_CLOSED_IND))
#endif

#define DM_PRIV_BASE                    (ENUM_SEP_DM_PRIV_DOWN_FIRST + 1)
#define DM_PRIV_DOWN_MAX                (ENUM_SEP_DM_PRIV_DOWN_LAST - 1)
#define DM_PRIV_MAX                     (ENUM_SEP_DM_PRIV_UP_LAST - 1)

/* SCO conn ui (none - no special SCO msgs defined.just use conn ui msgs) */

#define DM_SCO_BASE             DM_CONN_BASE
#define DM_SCO_MAX              DM_CONN_MAX

/* HCI API msgs */

/* link ctrl */

/* dw_msg msgs */
#define DM_HCI_INQUIRY                  ((U16)ENUM_DM_HCI_INQUIRY)
#define DM_HCI_INQUIRY_ESC              ((U16)ENUM_DM_HCI_INQUIRY_ESC)
#define DM_HCI_PERIODIC_INQUIRY         ((U16)ENUM_DM_HCI_PERIODIC_INQUIRY)
#define DM_HCI_EXIT_PERIODIC_INQUIRY    ((U16)ENUM_DM_HCI_EXIT_PERIODIC_INQUIRY)
#define DM_HCI_CHANGE_PKT_TYPE          ((U16)ENUM_DM_HCI_CHANGE_PKT_TYPE)
#define DM_HCI_CHANGE_LINK_KEY          ((U16)ENUM_DM_HCI_CHANGE_LINK_KEY)
#define DM_HCI_MASTER_LINK_KEY          ((U16)ENUM_DM_HCI_MASTER_LINK_KEY)
#define DM_HCI_RMT_NAME_REQ             ((U16)ENUM_DM_HCI_RMT_NAME_REQ)
#define DM_HCI_RD_RMT_FEATR             ((U16)ENUM_DM_HCI_RD_RMT_FEATR)
#define DM_HCI_RD_RMT_VERSION           ((U16)ENUM_DM_HCI_RD_RMT_VERSION)
#define DM_HCI_RD_CLOCK_OFFSET          ((U16)ENUM_DM_HCI_RD_CLOCK_OFFSET)
#ifdef CFG_BT_VER_21
#define DM_HCI_IO_CAPABILITY_REQ_REPLY       ((U16)ENUM_DM_HCI_IO_CAPABILITY_REQ_REPLY)
#define DM_HCI_IO_CAPABILITY_REQ_NEG_RSP     ((U16)ENUM_DM_HCI_IO_CAPABILITY_REQ_NEG_RSP)
#define DM_HCI_USER_CONF_REQUEST_REPLY       ((U16)ENUM_DM_HCI_USER_CONF_REQUEST_REPLY)
#define DM_HCI_USER_CONF_REQUEST_NEG_REPLY   ((U16)ENUM_DM_HCI_USER_CONF_REQUEST_NEG_REPLY)
#define DM_HCI_USER_PASSKEY_REPLY            ((U16)ENUM_DM_HCI_USER_UPASSKEY_REPLY)
#define DM_HCI_USER_PASSKEY_NEG_REPLY        ((U16)ENUM_DM_HCI_USER_UPASSKEY_NEG_REPLY)
#define DM_HCI_RMT_OOB_DATA_REPLY            ((U16)ENUM_DM_HCI_RMT_OOB_DATA_REPLY)
#define DM_HCI_RMT_OOB_DATA_NEG_REPLY        ((U16)ENUM_DM_HCI_RMT_OOB_DATA_NEG_REPLY)
#define DM_HCI_SEND_KEYPRESS_NOTIFI          ((U16)ENUM_DM_HCI_SEND_KEYPRESS_NOTIFI)

#endif

/* umsg msgs */
#define DM_HCI_INQUIRY_ESC_COMP             ((U16)ENUM_DM_HCI_INQUIRY_ESC_COMP)
#define DM_HCI_PERIODIC_INQUIRY_COMP        ((U16)ENUM_DM_HCI_PERIODIC_INQUIRY_COMP)
#define DM_HCI_EXIT_PERIODIC_INQUIRY_COMP   ((U16)ENUM_DM_HCI_EXIT_PERIODIC_INQUIRY_COMP)
#define DM_HCI_INQUIRY_RES                  ((U16)ENUM_DM_HCI_INQUIRY_RES)
#define DM_HCI_INQUIRY_COMP                 ((U16)ENUM_DM_HCI_INQUIRY_COMP)
#define DM_HCI_LINK_KEY_CHANGE_COMP         ((U16)ENUM_DM_HCI_LINK_KEY_CHANGE_COMP)
#define DM_HCI_RETURN_LINK_KEYS             ((U16)ENUM_DM_HCI_RETURN_LINK_KEYS)
#define DM_HCI_MASTER_LINK_KEY_COMP         ((U16)ENUM_DM_HCI_MASTER_LINK_KEY_COMP)
#define DM_HCI_KEYMISSING_NOTIFI            ((U16)ENUM_DM_HCI_KEYMISSING_NOTIFI)
#define DM_HCI_RMT_NAME_COMP                ((U16)ENUM_DM_HCI_RMT_NAME_COMP)
#define DM_HCI_RD_RMT_FEATR_COMP            ((U16)ENUM_DM_HCI_RD_RMT_FEATR_COMP)
#define DM_HCI_RD_RMT_VERSION_COMP          ((U16)ENUM_DM_HCI_RD_RMT_VERSION_COMP)
#define DM_HCI_RD_CLOCK_OFFSET_COMP         ((U16)ENUM_DM_HCI_RD_CLOCK_OFFSET_COMP)

#ifdef CFG_BT_VER_21
#define DM_HCI_EXT_INQUIRY_RES              ((U16)ENUM_DM_HCI_EXT_INQUIRY_RES)
#define DM_HCI_IO_CAPABILITY_REQ_IND        ((U16)ENUM_DM_HCI_IO_CAPABILITY_REQ_IND)
#define DM_HCI_IO_CAPABILITY_RSP_IND        ((U16)ENUM_DM_HCI_IO_CAPABILITY_RSP_IND)
#define DM_HCI_USER_CONF_REQ                ((U16)ENUM_DM_HCI_USER_CONF_REQ)
/* #define DM_HCI_USER_CONF_NEG_REQ      ((U16)ENUM_DM_HCI_USER_CONF_NEG_REQ) */
#define DM_HCI_USER_PASSKY_REQ              ((U16)ENUM_DM_HCI_USER_PASSKY_REQ)
#define DM_HCI_PASSKEY_NOTIFI               ((U16)ENUM_DM_HCI_PASSKEY_NOTIFI)
#define DM_HCI_RMT_OOB_DATA_REQ             ((U16)ENUM_DM_HCI_RMT_OOB_DATA_REQ)
#define DM_HCI_SSP_COMP_IND                 ((U16)ENUM_DM_HCI_SSP_COMP_REQ)
#define DM_HCI_KEYPRESS_NOTIFI_IND          ((U16)ENUM_DM_HCI_KEYPRESS_NOTIFI)
#endif


#define DM_LC_BASE                          (ENUM_SEP_DM_LC_DOWN_FIRST + 1)
#define DM_LC_DOWN_MAX                      (ENUM_SEP_DM_LC_DOWN_LAST - 1)
#define DM_LC_MAX                           (ENUM_SEP_DM_LC_UP_LAST - 1)

/* link policy */
/* dw_msg msgs */
#define DM_HCI_HOLD_MODE              ((U16)ENUM_DM_HCI_HOLD_MODE)
#define DM_HCI_SNIFF_MODE             ((U16)ENUM_DM_HCI_SNIFF_MODE)
#define DM_HCI_EXIT_SNIFF_MODE        ((U16)ENUM_DM_HCI_EXIT_SNIFF_MODE)
#define DM_HCI_PARK_MODE              ((U16)ENUM_DM_HCI_PARK_MODE)
#define DM_HCI_EXIT_PARK_MODE         ((U16)ENUM_DM_HCI_EXIT_PARK_MODE)
#define DM_HCI_QOS_SETUP_REQ          ((U16)ENUM_DM_HCI_QOS_SETUP_REQ)
#define DM_HCI_ROLE_DISCOV            ((U16)ENUM_DM_HCI_ROLE_DISCOV)
#define DM_HCI_SWITCH_ROLE            ((U16)ENUM_DM_HCI_SWITCH_ROLE)
#define DM_HCI_RD_LP_SETTINGS         ((U16)ENUM_DM_HCI_RD_LP_SETTINGS)
#define DM_HCI_WR_LP_SETTINGS         ((U16)ENUM_DM_HCI_WR_LP_SETTINGS)

/* umsg msgs */
#define DM_HCI_MODE_CHANGE_EV           ((U16)ENUM_DM_HCI_MODE_CHANGE_EV)
#define DM_HCI_QOS_SETUP_CFM            ((U16)ENUM_DM_HCI_QOS_SETUP_CFM)
#define DM_HCI_QOS_VIOLATION_IND        ((U16)ENUM_DM_HCI_QOS_VIOLATION_IND)
#define DM_HCI_ROLE_DISCOV_COMP         ((U16)ENUM_DM_HCI_ROLE_DISCOV_COMP)
#define DM_HCI_SWITCH_ROLE_COMP         ((U16)ENUM_DM_HCI_SWITCH_ROLE_COMP)
#define DM_HCI_RD_LP_SETTINGS_COMP      ((U16)ENUM_DM_HCI_RD_LP_SETTINGS_COMP)
#define DM_HCI_WR_LP_SETTINGS_COMP      ((U16)ENUM_DM_HCI_WR_LP_SETTINGS_COMP)

#ifdef CFG_BT_VER_21
#define DM_HCI_SUBNIFF_RATING_MODE_CHANGE_EV    ((U16)ENUM_DM_HCI_SUBNIFF_MODE_CHANGE_EV)
#endif

#define DM_LP_BASE                      (ENUM_SEP_DM_LP_DOWN_FIRST + 1)
#define DM_LP_DOWN_MAX                  (ENUM_SEP_DM_LP_DOWN_LAST - 1)
#define DM_LP_MAX                       (ENUM_SEP_DM_LP_DOWN_LAST - 1)

#define DM_HCI_BB_RD_SPEC         (0X00)
#define DM_HCI_BB_RD_ALL          (0X01)
#define DM_HCI_BB_WR_SPEC         (0X00)
#define DM_HCI_BB_WR_ALL          (0X01)
/* host ctrller and baseband */
/* dw_msg msgs */
#define DM_HCI_SET_EV_MASK                  ((U16)ENUM_DM_HCI_SET_EV_MASK)
#define DM_HCI_RESET                        ((U16)ENUM_DM_HCI_RESET)
#define DM_HCI_SET_EV_FILTER                ((U16)ENUM_DM_HCI_SET_EV_FILTER)
#define DM_HCI_FLUSH                        ((U16)ENUM_DM_HCI_FLUSH)
#define DM_HCI_RD_PIN_TYPE                  ((U16)ENUM_DM_HCI_RD_PIN_TYPE)
#define DM_HCI_WR_PIN_TYPE                  ((U16)ENUM_DM_HCI_WR_PIN_TYPE)
#define DM_HCI_CREATE_NEW_UNIT_KEY          ((U16)ENUM_DM_HCI_CREATE_NEW_UNIT_KEY)
#define DM_HCI_RD_STORED_LINK_KEY           ((U16)ENUM_DM_HCI_RD_STORED_LINK_KEY)
#define DM_HCI_WR_STORED_LINK_KEY           ((U16)ENUM_DM_HCI_WR_STORED_LINK_KEY)
#define DM_HCI_DEL_STORED_LINK_KEY          ((U16)ENUM_DM_HCI_DEL_STORED_LINK_KEY)
#define DM_HCI_CHANGE_LOCAL_NAME            ((U16)ENUM_DM_HCI_CHANGE_LOCAL_NAME)
#define DM_HCI_RD_LOCAL_NAME                ((U16)ENUM_DM_HCI_RD_LOCAL_NAME)
#define DM_HCI_RD_CONN_ACPT_TO              ((U16)ENUM_DM_HCI_RD_CONN_ACPT_TO)
#define DM_HCI_WR_CONN_ACPT_TO              ((U16)ENUM_DM_HCI_WR_CONN_ACPT_TO)
#define DM_HCI_RD_PAGE_TO                   ((U16)ENUM_DM_HCI_RD_PAGE_TO)
#define DM_HCI_WR_PAGE_TO                   ((U16)ENUM_DM_HCI_WR_PAGE_TO)
#define DM_HCI_RD_SCAN_ENB                  ((U16)ENUM_DM_HCI_RD_SCAN_ENB)
#define DM_HCI_WR_SCAN_ENB                  ((U16)ENUM_DM_HCI_WR_SCAN_ENB)
#define DM_HCI_RD_PAGESCAN_ACTIVITY         ((U16)ENUM_DM_HCI_RD_PAGESCAN_ACTIVITY)
#define DM_HCI_WR_PAGESCAN_ACTIVITY         ((U16)ENUM_DM_HCI_WR_PAGESCAN_ACTIVITY)
#define DM_HCI_RD_INQUIRYSCAN_ACTIVITY      ((U16)ENUM_DM_HCI_RD_INQUIRYSCAN_ACTIVITY)
#define DM_HCI_WR_INQUIRYSCAN_ACTIVITY      ((U16)ENUM_DM_HCI_WR_INQUIRYSCAN_ACTIVITY)
#define DM_HCI_RD_AUTH_ENB                  ((U16)ENUM_DM_HCI_RD_AUTH_ENB)
#define DM_HCI_RD_ENCRYPTION_MODE           ((U16)ENUM_DM_HCI_RD_ENCRYPTION_MODE)
#define DM_HCI_RD_DEV_CLS                   ((U16)ENUM_DM_HCI_RD_DEV_CLS)
#define DM_HCI_WR_DEV_CLS                   ((U16)ENUM_DM_HCI_WR_DEV_CLS)
#define DM_HCI_RD_VOICE_SETTING             ((U16)ENUM_DM_HCI_RD_VOICE_SETTING)
#define DM_HCI_WR_VOICE_SETTING             ((U16)ENUM_DM_HCI_WR_VOICE_SETTING)
#define DM_HCI_RD_AUTO_FLUSH_TIMEOUT        ((U16)ENUM_DM_HCI_RD_AUTO_FLUSH_TIMEOUT)
#define DM_HCI_WR_AUTO_FLUSH_TIMEOUT        ((U16)ENUM_DM_HCI_WR_AUTO_FLUSH_TIMEOUT)
#define DM_HCI_RD_NUM_BCAST_TXS             ((U16)ENUM_DM_HCI_RD_NUM_BCAST_TXS)
#define DM_HCI_WR_NUM_BCAST_TXS             ((U16)ENUM_DM_HCI_WR_NUM_BCAST_TXS)
#define DM_HCI_RD_HOLD_MODE_ACTIVITY        ((U16)ENUM_DM_HCI_RD_HOLD_MODE_ACTIVITY)
#define DM_HCI_WR_HOLD_MODE_ACTIVITY        ((U16)ENUM_DM_HCI_WR_HOLD_MODE_ACTIVITY)
#define DM_HCI_RD_TX_POWER_LEVEL            ((U16)ENUM_DM_HCI_RD_TX_POWER_LEVEL)
#define DM_HCI_RD_SCO_FLOW_CTRL_ENB         ((U16)ENUM_DM_HCI_RD_SCO_FLOW_CTRL_ENB)
#define DM_HCI_WR_SCO_FLOW_CTRL_ENB         ((U16)ENUM_DM_HCI_WR_SCO_FLOW_CTRL_ENB)
#define DM_HCI_SET_HC_TO_HOST_FLOW          ((U16)ENUM_DM_HCI_SET_HC_TO_HOST_FLOW)
#define HCI_HOST_NUM_COMP_PKT               ((U16)ENUM_HCI_HOST_NUM_COMP_PKT)
#define DM_HCI_RD_LINK_SUPERV_TIMEOUT       ((U16)ENUM_DM_HCI_RD_LINK_SUPERV_TIMEOUT)
#define DM_HCI_WR_LINK_SUPERV_TIMEOUT       ((U16)ENUM_DM_HCI_WR_LINK_SUPERV_TIMEOUT)
#define DM_HCI_RD_NUM_SUPP_IAC              ((U16)ENUM_DM_HCI_RD_NUM_SUPP_IAC)
#define DM_HCI_RD_CUR_IAC_LAP               ((U16)ENUM_DM_HCI_RD_CUR_IAC_LAP)
#define DM_HCI_WR_CUR_IAC_LAP               ((U16)ENUM_DM_HCI_WR_CUR_IAC_LAP)
#define DM_HCI_RD_PAGESCAN_PERIOD_MODE      ((U16)ENUM_DM_HCI_RD_PAGESCAN_PERIOD_MODE)
#define DM_HCI_WR_PAGESCAN_PERIOD_MODE      ((U16)ENUM_DM_HCI_WR_PAGESCAN_PERIOD_MODE)
#define DM_HCI_RD_PAGESCAN_MODE             ((U16)ENUM_DM_HCI_RD_PAGESCAN_MODE)
#define DM_HCI_WR_PAGESCAN_MODE             ((U16)ENUM_DM_HCI_WR_PAGESCAN_MODE)

/* umsg msgs */
#define DM_HCI_SET_EV_MASK_COMP             ((U16)ENUM_DM_HCI_SET_EV_MASK_COMP)
#define DM_HCI_RESET_COMP                   ((U16)ENUM_DM_HCI_RESET_COMP)
#define DM_HCI_SET_EV_FILTER_COMP           ((U16)ENUM_DM_HCI_SET_EV_FILTER_COMP)
#define DM_HCI_FLUSH_COMP                   ((U16)ENUM_DM_HCI_FLUSH_COMP)
#define DM_HCI_RD_PIN_TYPE_COMP             ((U16)ENUM_DM_HCI_RD_PIN_TYPE_COMP)
#define DM_HCI_WR_PIN_TYPE_COMP             ((U16)ENUM_DM_HCI_WR_PIN_TYPE_COMP)
#define DM_HCI_CREATE_NEW_UNIT_KEY_COMP     ((U16)ENUM_DM_HCI_CREATE_NEW_UNIT_KEY_COMP)
#define DM_HCI_RD_STORED_LINK_KEY_COMP      ((U16)ENUM_DM_HCI_RD_STORED_LINK_KEY_COMP)
#define DM_HCI_WR_STORED_LINK_KEY_COMP      ((U16)ENUM_DM_HCI_WR_STORED_LINK_KEY_COMP)
#define DM_HCI_DEL_STORED_LINK_KEY_COMP     ((U16)ENUM_DM_HCI_DEL_STORED_LINK_KEY_COMP)
#define DM_HCI_CHANGE_LOCAL_NAME_COMP       ((U16)ENUM_DM_HCI_CHANGE_LOCAL_NAME_COMP)
#define DM_HCI_RD_LOCAL_NAME_COMP           ((U16)ENUM_DM_HCI_RD_LOCAL_NAME_COMP)
#define DM_HCI_RD_CONN_ACPT_TO_COMP         ((U16)ENUM_DM_HCI_RD_CONN_ACPT_TO_COMP)
#define DM_HCI_WR_CONN_ACPT_TO_COMP         ((U16)ENUM_DM_HCI_WR_CONN_ACPT_TO_COMP)
#define DM_HCI_RD_PAGE_TO_COMP              ((U16)ENUM_DM_HCI_RD_PAGE_TO_COMP)
#define DM_HCI_WR_PAGE_TO_COMP              ((U16)ENUM_DM_HCI_WR_PAGE_TO_COMP)
#define DM_HCI_RD_SCAN_ENB_COMP             ((U16)ENUM_DM_HCI_RD_SCAN_ENB_COMP)
#define DM_HCI_WR_SCAN_ENB_COMP             ((U16)ENUM_DM_HCI_WR_SCAN_ENB_COMP)
#define DM_HCI_RD_PAGESCAN_ACTIVITY_COMP    ((U16)ENUM_DM_HCI_RD_PAGESCAN_ACTIVITY_COMP)
#define DM_HCI_WR_PAGESCAN_ACTIVITY_COMP    ((U16)ENUM_DM_HCI_WR_PAGESCAN_ACTIVITY_COMP)
#define DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP ((U16)ENUM_DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP)
#define DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP ((U16)ENUM_DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP)
#define DM_HCI_RD_AUTH_ENB_COMP             ((U16)ENUM_DM_HCI_RD_AUTH_ENB_COMP)
#define DM_HCI_RD_ENCRYPTION_MODE_COMP      ((U16)ENUM_DM_HCI_RD_ENCRYPTION_MODE_COMP)
#define DM_HCI_RD_DEV_CLS_COMP              ((U16)ENUM_DM_HCI_RD_DEV_CLS_COMP)
#define DM_HCI_WR_DEV_CLS_COMP              ((U16)ENUM_DM_HCI_WR_DEV_CLS_COMP)
#define DM_HCI_RD_VOICE_SETTING_COMP        ((U16)ENUM_DM_HCI_RD_VOICE_SETTING_COMP)
#define DM_HCI_WR_VOICE_SETTING_COMP        ((U16)ENUM_DM_HCI_WR_VOICE_SETTING_COMP)
#define DM_HCI_RD_AUTO_FLUSH_TIMEOUT_COMP   ((U16)ENUM_DM_HCI_RD_AUTO_FLUSH_TIMEOUT_COMP)
#define DM_HCI_WR_AUTO_FLUSH_TIMEOUT_COMP   ((U16)ENUM_DM_HCI_WR_AUTO_FLUSH_TIMEOUT_COMP)
#define DM_HCI_RD_NUM_BCAST_TXS_COMP        ((U16)ENUM_DM_HCI_RD_NUM_BCAST_TXS_COMP)
#define DM_HCI_WR_NUM_BCAST_TXS_COMP        ((U16)ENUM_DM_HCI_WR_NUM_BCAST_TXS_COMP)
#define DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP   ((U16)ENUM_DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP)
#define DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP   ((U16)ENUM_DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP)
#define DM_HCI_RD_TX_POWER_LEVEL_COMP       ((U16)ENUM_DM_HCI_RD_TX_POWER_LEVEL_COMP)
#define DM_HCI_SET_HC_TO_HOST_FLOW_COMP     ((U16)ENUM_DM_HCI_SET_HC_TO_HOST_FLOW_COMP)
#define HCI_HOST_NUM_COMP_PKT_COMP          ((U16)ENUM_HCI_HOST_NUM_COMP_PKT_COMP)
#define DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP  ((U16)ENUM_DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP)
#define DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP  ((U16)ENUM_DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP)
#define DM_HCI_RD_NUM_SUPP_IAC_COMP         ((U16)ENUM_DM_HCI_RD_NUM_SUPP_IAC_COMP)
#define DM_HCI_RD_CUR_IAC_LAP_COMP          ((U16)ENUM_DM_HCI_RD_CUR_IAC_LAP_COMP)
#define DM_HCI_WR_CUR_IAC_LAP_COMP          ((U16)ENUM_DM_HCI_WR_CUR_IAC_LAP_COMP)
#define DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP ((U16)ENUM_DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP)
#define DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP ((U16)ENUM_DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP)
#define DM_HCI_RD_PAGESCAN_MODE_COMP        ((U16)ENUM_DM_HCI_RD_PAGESCAN_MODE_COMP)
#define DM_HCI_WR_PAGESCAN_MODE_COMP        ((U16)ENUM_DM_HCI_WR_PAGESCAN_MODE_COMP)
#define DM_HCI_HARDWARE_ERR                 ((U16)ENUM_DM_HCI_HARDWARE_ERR)
#define DM_HCI_DATA_BUFF_OVERFLOW           ((U16)ENUM_DM_HCI_DATA_BUFF_OVERFLOW)
#define DM_HCI_MAX_SLOTS_CHANGE             ((U16)ENUM_DM_HCI_MAX_SLOTS_CHANGE)

#define DM_BB_BASE                          (ENUM_SEP_DM_BB_DOWN_FIRST + 1)
#define DM_BB_DOWN_MAX                      (ENUM_SEP_DM_BB_DOWN_LAST - 1)
#define DM_BB_MAX                           (ENUM_SEP_DM_BB_UP_LAST - 1)

/* infmtional pars */
/* dw_msg msgs */
#define DM_HCI_RD_LOCAL_VERSION               ((U16)ENUM_DM_HCI_RD_LOCAL_VERSION)
#define DM_HCI_RD_LOCAL_FEATR                 ((U16)ENUM_DM_HCI_RD_LOCAL_FEATR)
#define DM_HCI_RD_COUNTRY_CODE                ((U16)ENUM_DM_HCI_RD_COUNTRY_CODE)
#define DM_HCI_RD_BD_ADDR                     ((U16)ENUM_DM_HCI_RD_BD_ADDR)

/* umsg msgs */
#define DM_HCI_RD_LOCAL_VERSION_COMP          ((U16)ENUM_DM_HCI_RD_LOCAL_VERSION_COMP)
#define DM_HCI_RD_LOCAL_FEATR_COMP            ((U16)ENUM_DM_HCI_RD_LOCAL_FEATR_COMP)
#define DM_HCI_RD_COUNTRY_CODE_COMP           ((U16)ENUM_DM_HCI_RD_COUNTRY_CODE_COMP)
#define DM_HCI_RD_BD_ADDR_COMP                ((U16)ENUM_DM_HCI_RD_BD_ADDR_COMP)

#define DM_IP_BASE                            (ENUM_SEP_DM_INF_DOWN_FIRST + 1)
#define DM_IP_DOWN_BASE                       DM_IP_BASE
#define DM_IP_DOWN_MAX                        (ENUM_SEP_DM_INF_DOWN_LAST - 1)
#define DM_IP_UP_BASE                         (ENUM_SEP_DM_INF_DOWN_LAST + 1)
#define DM_IP_MAX                             (ENUM_SEP_DM_INF_UP_LAST - 1)
#define DM_IP_UP_MAX                          DM_IP_MAX

/* st pars */
/* dw_msg msg */
#define DM_HCI_FAILED_CONTACT_COUNTER           ((U16)ENUM_DM_HCI_FAILED_CONTACT_COUNTER)
#define DM_HCI_RESET_CONTACT_COUNTER            ((U16)ENUM_DM_HCI_RESET_CONTACT_COUNTER)
#define DM_HCI_GET_LINK_QA                      ((U16)ENUM_DM_HCI_GET_LINK_QA)
#define DM_HCI_RD_RSSI                          ((U16)ENUM_DM_HCI_RD_RSSI)

/* umsg msgs */
#define DM_HCI_FAILED_CONTACT_COUNTER_COMP  ((U16)ENUM_DM_HCI_FAILED_CONTACT_COUNTER_COMP)
#define DM_HCI_RESET_CONTACT_COUNTER_COMP   ((U16)ENUM_DM_HCI_RESET_CONTACT_COUNTER_COMP)
#define DM_HCI_GET_LINK_QA_COMP             ((U16)ENUM_DM_HCI_GET_LINK_QA_COMP)
#define DM_HCI_RD_RSSI_COMP                 ((U16)ENUM_DM_HCI_RD_RSSI_COMP)

#define DM_STS_BASE                         (ENUM_SEP_DM_STS_DOWN_FIRST + 1)
#define DM_STS_DOWN_MAX                     (ENUM_SEP_DM_STS_DOWN_LAST - 1)
#define DM_STS_MAX                          (ENUM_SEP_DM_STS_UP_LAST - 1)

/* testing msgs */
/* dw_msg msgs */
#define DM_HCI_RD_LOOPBACK_MODE             ((U16)ENUM_DM_HCI_RD_LOOPBACK_MODE)
#define DM_HCI_WR_LOOPBACK_MODE             ((U16)ENUM_DM_HCI_WR_LOOPBACK_MODE)
#define DM_HCI_ENB_DEV_UT_MODE              ((U16)ENUM_DM_HCI_ENB_DEV_UT_MODE)

/* umsg msgs */
#define DM_HCI_RD_LOOPBACK_MODE_COMP        ((U16)ENUM_DM_HCI_RD_LOOPBACK_MODE_COMP)
#define DM_HCI_WR_LOOPBACK_MODE_COMP        ((U16)ENUM_DM_HCI_WR_LOOPBACK_MODE_COMP)
#define DM_HCI_LOOPBACK_EV                  ((U16)ENUM_DM_HCI_LOOPBACK_EV)
#define DM_HCI_ENB_DEV_UT_MODE_COMP         ((U16)ENUM_DM_HCI_ENB_DEV_UT_MODE_COMP)

#define DM_TEST_BASE                        (ENUM_SEP_DM_TEST_DOWN_FIRST + 1)
#define DM_TEST_DOWN_MAX                    (ENUM_SEP_DM_TEST_DOWN_LAST - 1)
#define DM_TEST_MAX                         (ENUM_SEP_DM_TEST_UP_LAST - 1)

/* end of #define of msgs */

/* msg type for dm */

/* REG MSGITIVES */

/* note1:A val of 0 in the 'st' field of returned msgs indicates
   succ. all other vals indicate fail.
   the err codes are defined in the bluetooth HCI specification */

/* PURPOSE standard cmd comp */

typedef struct
{
    U16 type; /* msg ID */
    U16 phdl; /* dest phdl */
    U8 st;    /* succ or fail - see note1 */
} BTS2S_DM_HCI_STANDARD_CMD_COMP;

/* PURPOSE reg req to AM ui, reging a dest phdl for umsg app msgs */
typedef struct
{
    U16 type; /* always HCI_GAP_MGM_REG_REQ */
    U16 phdl; /* prot hdl */
} BTS2S_DM_AM_REG_REQ;

/* PURPOSE cfm that the registration req has been recvd */
typedef struct
{
    U16 type; /* always HCI_GAP_MGM_REG_CFM */
    U16 phdl; /* dest phdl */
} BTS2S_DM_AM_REG_CFM;

/* ACL CONN UI MSGITIVES these msgs are all sent to the DM_ACLQUE input que, and
 are intended for use only by L2C */

/* reg req to ACL ui, reging a dest phdl for all umsg ACL conn - related msgs (expected to be L2C) */

typedef struct
{
    U16 type; /* always DM_ACL_REG_REQ */
    U16 phdl; /* prot hdl */
} BTS2S_DM_ACL_REG_REQ;

/* PURPOSE cfm that the registration req has been recv */
typedef struct
{
    U16 type; /* always DM_ACL_REG_CFM */
    U16 phdl; /* dest phdl */
} BTS2S_DM_ACL_REG_CFM;

/* PURPOSE req from L2C to create ACL conn ro rmt bluetooth dev */

typedef struct
{
    U16 type; /* always DM_ACL_CONN_REQ */
    BTS2S_BD_ADDR bd; /* bluetooth dev addr */
    U16 pkt_type; /* set to zero for default */
} BTS2S_DM_ACL_CONN_REQ;

/* PURPOSE ACL conn cfm to L2C for conn to rmt bluetooth dev */

typedef struct
{
    U16 type; /* always DM_ACL_CONN_CFM */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail - see note1 */
    U16 hdl; /* conn hdl */
    BTS2S_BD_ADDR bd; /* bluetooth dev addr */
    U8 enc_mode; /* encryption mode (point to point, etc) */
} BTS2S_DM_ACL_CONN_CFM;

/* ACL conn indi to L2C from rmt bluetooth dev */

typedef struct
{
    U16 type; /* always DM_ACL_CONN_IND */
    U16 phdl; /* dest phdl */
    U8  st; /* succ or fail - see note1 */
    U16 hdl; /* conn hdl */
    BTS2S_BD_ADDR bd; /* bluetooth dev addr */
    U8 enc_mode; /* encryption mode (point to point, etc) */
} BTS2S_DM_ACL_CONN_IND;


/* disc req from L2C for a particular ACL conn hdl */

typedef struct
{
    U16 type; /* always DM_ACL_DISC_REQ */
    U16 hdl; /* HCI conn hdl */
    U8 reason; /* reason for disconn */
} BTS2S_DM_ACL_DISC_REQ;

typedef struct
{
    U16 type; /* always DM_ACL_CLOSE_REQ */
    U16 hdl; /* HCI conn hdl */
    U8 reason; /* reason for disconn */
} BTS2S_DM_ACL_BT_CLOSE_REQ;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
} BTS2S_DM_ACL_CANCEL_CONN_REQ;

typedef struct
{
    U16 type; /* always DM_ACL_OPEN_REQ */
} BTS2S_DM_ACL_BT_OPEN_REQ;



/* disc indicate to L2C for a particular ACL conn hd */

typedef struct
{
    U16 type; /* always DM_ACL_DISC_IND */
    U16 phdl; /* dest phdl */
    BTS2S_BD_ADDR cur_bd;
    U8 st; /* succ or fail */
    U16 hdl; /* HCI conn hdl */
    U8 reason; /* reason for disconn */
} BTS2S_DM_ACL_DISC_IND;

/* an indi of host ctrller buff sizes this is sent once only on start - up to each ACL ui */

typedef struct
{
    U16 type; /* always DM_ACL_BUFF_SIZE_IND */
    U16 phdl; /* dest phdl */
    U16 size; /* max size of data pkt */
    U16 max_pkt_num; /* max num pkt on host */
} BTS2S_DM_ACL_BUFF_SIZE_IND;

/* data sent indi for each ACL ui sent upon receiving hci_num_compd_pkt ev see defn of BTS2S_HCI_EV_NUM_COMPD_PKTS for details */

typedef struct
{
    U16 type;       /* always DM_ACL_DATA_SENT_IND */
    U16 phdl;       /* dest phdl */
    U8 num_hdls;    /* num of hdls */
    /* hdl plus num of pkt */
#if 0
    BTS2S_HANDLE_COMP *hdl_comp_ptr[HCI_EV_NUM_HANDLE_COMP_PKT_PTRS];
#else
    /* optimization for android */
    BTS2S_HANDLE_COMP *hdl_comp_ptr;
#endif
} BTS2S_DM_ACL_DATA_SENT_IND;

typedef struct
{
    U16 type;       /* always DM_ACL_DATA_SENT_IND */
    U16 phdl;       /* dest phdl */
    U8 num_hdls;    /* num of hdls */
    /* hdl plus num of pkt */
#if 0
    BTS2S_HANDLE_COMP *hdl_comp_ptr[HCI_EV_NUM_HANDLE_COMP_PKT_PTRS];
#else
    /* optimization for android */
    BTS2S_HANDLE_COMP hdl_comp_ptr;
#endif
} BTS2S_DM_ACL_DATA_SENT_IND_ext;


/* reg connectless chnl ID */
typedef struct
{
    U16 type; /* always DM_CONNLESS_CH_REG_REQ */
    U16 hdl;  /* HCI conn hdl */
} BTS2S_DM_CONNLESS_CH_REG_REQ;

/* these msgs are all sent to the BTS2T_HCI_CMD input que */

typedef struct
{
    U16 type; /* always DM_SYNC_REG_REQ */
    U16 phdl; /* prot hdl */
    U32 pv_cbarg;
} BTS2S_DM_SYNC_REG_REQ;

typedef struct
{
    U16 type; /* always DM_SYNC_REG_CFM */
    U16 phdl; /* prot hdl */
    U32 pv_cbarg;
} BTS2S_DM_SYNC_REG_CFM;

typedef struct
{
    U16 type; /* always DM_SYNC_UNREG_REQ */
    U16 phdl; /* clt phdl */
    U32 pv_cbarg;
} BTS2S_DM_SYNC_UNREG_REQ;

typedef struct
{
    U16 type; /* always DM_SYNC_UNREG_CFM */
    U16 phdl; /* clt phdl */
    U32 pv_cbarg;
} BTS2S_DM_SYNC_UNREG_CFM;

typedef struct
{
    U16 type; /* always DM_SYNC_CONN_REQ */
    U16 phdl; /* clt phdl */
    U32 pv_cbarg;
    BTS2S_BD_ADDR bd; /* bluetooth dev addr */
    U32 tx_bdw;
    U32 rx_bdw;
    U16 max_latency;
    U16 voice_setting;
    U8 retx_effort;
    U16 pkt_type; /* set to zero for default */
} BTS2S_DM_SYNC_CONN_REQ;

typedef struct
{
    U16  type;          /* always DM_SYNC_CONN_CFM */
    U16  phdl;          /* clt phdl */
    U32  pv_cbarg;
    U8   st;            /* succ or fail - see note1 */
    U16  hdl;           /* conn hdl */
    U8   lm_sco_hdl;    /* LM SCO hdl, or HCI_LM_SCO_HANDLE_INVLD if unknown */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 link_type;
    U8 tx_intvl;
    U8 wesco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
    U8 air_mode;
} BTS2S_DM_SYNC_CONN_CFM;


typedef struct
{
    U16 type;           /* always DM_SYNC_CONN_COMP_IND */
    U16 phdl;           /* clt phdl */
    U32 pv_cbarg;
    U8 st;              /* succ or fail */
    U16 hdl;            /* conn hdl */
    U8 lm_sco_hdl;      /* LM SCO hdl, or HCI_LM_SCO_HANDLE_INVLD if unknown */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 link_type;
    U8 tx_intvl;
    U8 wesco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
    U8 air_mode;
} BTS2S_DM_SYNC_CONN_COMP_IND;

typedef struct
{
    U16 type;           /* always DM_SYNC_CONN_IND */
    U16 phdl;           /* clt phdl */
    U32 pv_cbarg;
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
} BTS2S_DM_SYNC_CONN_IND;

typedef struct
{
    U16 type;           /* always DM_SYNC_CONN_RES */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    BOOL acpt;          /* TRUE to acpt the conn */
    U32 tx_bdw;
    U32 rx_bdw;
    U16 max_latency;
    U16 voice_setting;
    U8 retx_effort;
    U16 pkt_type;       /* set to zero for default */
    U8 reason;          /* reason for rej */
} BTS2S_DM_SYNC_CONN_RES;

typedef struct
{
    U16 type;           /* always DM_SYNC_RENEGOTIATE_REQ */
    U16 hdl;
    U8 retx_effort;
    U16 pkt_type;
    U16 sco_hdl;
    U16 audio_qa;
    U32 tx_bandwidth;
    U32 rx_bandwidth;
    U16 max_latency;
    U16 voice_setting;
    U8  re_tx_effort;
} BTS2S_DM_SYNC_RENEGOTIATE_REQ;

typedef struct
{
    U16 type;           /* always DM_SYNC_RENEGOTIATE_IND */
    U16 phdl;           /* dest phdl */
    U16 hdl;            /* sco hdl */
    U32 pv_cbarg;
    U8 st;              /* Status: Connection successfully reconfigured. */
    U8 tx_intvl;
    U8 we_sco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
} BTS2S_DM_SYNC_RENEGOTIATE_IND;

typedef struct
{
    U16 type;           /* always DM_SYNC_RENEGOTIATE_CFM */
    U16 phdl;           /* dest phdl */
    U16 hdl;            /* sco hdl */
    U32 pv_cbarg;
    U8 st;              /* Status: Connection successfully reconfigured. */
    U8 tx_intvl;
    U8 we_sco;
    U16 rx_pkt_len;
    U16 tx_pkt_len;
} BTS2S_DM_SYNC_RENEGOTIATE_CFM;

typedef struct
{
    U16 type;           /* always DM_SYNC_DISC_REQ */
    U16 hdl;            /* HCI conn hdl */
    U8 reason;          /* reason for disconn */
} BTS2S_DM_SYNC_DISC_REQ;

typedef struct
{
    U16 type;           /* always DM_SYNC_DISC_IND */
    U16 phdl;           /* dest phdl */
    U32 pv_cbarg;
    U8 st;              /* succ or fail - see note1 */
    U16 hdl;            /* HCI conn hdl */
    U8 reason;          /* reason for disconn */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
} BTS2S_DM_SYNC_DISC_IND;

typedef struct
{
    U16 type;           /* always DM_SYNC_DISC_CFM */
    U16 phdl;           /* dest phdl */
    U32 pv_cbarg;
    U8 st;              /* succ or fail */
    U16 hdl;            /* HCI conn hdl */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
} BTS2S_DM_SYNC_DISC_CFM;



/* SECU MGM UI dw_msg msgs are sent to the BTS2T_HCI_CMD, umsg msgs
   are sent either to the reged AM que (see HCI_GAP_MGM_REG_REQ), or to
   the que of the prot layer reqing access (for DM_SM_ACCESS_REQ)
   there is a private L2C / SM ui for ctrlling secu of recvd
   connless pkt */

#ifndef SEC_MODE0_OFF
#define SEC_MODE0_OFF                   ((U8)0x00)
#define SEC_MODE1_NON_SECURE            ((U8)0x01)
#define SEC_MODE2_SVC                   ((U8)0x02)
#define SEC_MODE3_LINK                  ((U8)0x03)
#endif


#ifdef CFG_BT_VER_21
#define SEC_MODE4_SVC                   ((U8)0x04)
#endif

/* SEC_MODE2a_DEV is a custom secu mode that auths on a per
   dev basis using the secu manager's dev db.
   on starting a new svc (incoming and outgoing), the dev will be
   authd unless it is marked as 'trusted' in the dev db
   (see DM_SM_ADD_DEV_REQ) */
#define SEC_MODE2a_DEV                  ((U8)0x80)
#define SS_NULL                         ((U16)0x0000)
#define SSI_AUTHOR                      ((U16)0x0001)
#define SSI_AUTHEN                      ((U16)0x0002)
#define SSI_ENCRPT                      ((U16)0x0004)
#define SSO_AUTHOR                      ((U16)0x0008)
#define SSO_AUTHEN                      ((U16)0x0010)
#define SSO_ENCRPT                      ((U16)0x0020)
#define SECL_IN_CONNLESS                ((U16)0x0040)

/****************** *default secu settings ******************
          Default Security Settings here

        SS_NULL       No security setting
        SSI_AUTHOR    Authorization when incoming
        SSI_AUTHEN    Authentication when incoming
        SSI_ENCRPT    Encryption when incoming
        SSO_AUTHOR    Authorization when outgoing
        SSO_AUTHEN    Authentication when outgoing
        SSO_ENCRPT    Encryption when outgoing
 **********************************************************************/

#define SSI_FAX_CLT                (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_HFP_HF                 (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_HFP_AG                 (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_OPP_SRV                (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_FTP_SRV                (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_SPP                    (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_FAX_GW                 (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_AV                     (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_AVRCP                  (SS_NULL)
#define SSI_HID_HOST               (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_BIP_SRV                (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_BPP_SRV                (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_DUN_GW                 (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_PAN                    (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_RFCOMM                 (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_HCRP                   (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_SYNC_SRV               (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_SIM                    (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSI_PBAP_SRV               (SSI_AUTHEN | SSI_ENCRPT | SSI_AUTHOR)
#define SSO_HFP_HF                 (SS_NULL)
#define SSO_HFP_AG                 (SSI_AUTHEN)
#define SSO_FTP_CLT                (SS_NULL)
#define SSO_OPP_CLT                (SS_NULL)
#define SSO_SPP                    (SS_NULL)
#define SSO_FAXC                   (SS_NULL)
#define SSO_AV                     (SS_NULL)
#define SSO_AVRCP                  (SS_NULL)
#define SSO_HID_HOST               (SS_NULL)
#define SSO_BIP_CLT                (SS_NULL)
#define SS0_BPP_CLT                (SS_NULL)
#define SSO_DUNC                   (SS_NULL)
#define SSO_PAN                    (SS_NULL)
#define SSO_SYNC_SRV               (SS_NULL)
#define SSO_SIM                    (SS_NULL)
#define SSO_PBAP_CLT               (SS_NULL)

/* the default secu level, applied in secu modes 2 and 3 when no
  specific secu level has been reged for a svc. this is set to the
  val defined in the bluetooth secu architecture white paper, but it may
  be overridden at runtime using the DM_SET_DEFAULT_SECU_REQ msg */
#define DEFAULT_SEC_MODE    SEC_MODE1_NON_SECURE
#define DEFAULT_ENC_MODE3   HCI_ENC_MODE_OFF

/* standard secu prot ids provided by stk. higher multiplexing prots must define their own ids, from the range
   SEC_PROT_USER and above */

#define SEC_PROT_L2C             ((U32)0x0000)
#define SEC_PROT_RFC             ((U32)0x0001)
/* higher layers base their prot ids from here */
#define SEC_PROT_USER            ((U32)0x8000)

/* sets the default secu level to be applied in secu modes 2 and 3 in cases where a svc has not been reged with the secu
   manager. the "default" default secu level is defined by
   SECL_DEFAULT, and using this msg overrides SECL_DEFAULT */
typedef struct
{
    U16 type;               /* always DM_SM_REG_REQ */
    U16 bts2s_secl_default;  /* new default secu level */
} BTS2S_DM_SM_SET_DEFAULT_SECU_REQ;

/* regs the secu requirements for access to a svc or a multiplexing prot layer when the secu manager is in secu
   mode 2 or 3. the reged secu level is applied to all incoming
   conns on the specified 'chnl', and optionally to all outgoing
   conns on that chnl if 'outgoing_ok' is TRUE.

   it does not always make sense to apply secu to outgoing conns,
   for example a local RFC srv chnl is only vld for incoming
   conns. an L2C PSM, however, may be vld in both directions.
   therefore use the 'outgoing_ok' flag to specify whether secu
   should be applied to outgoing conns as well as incoming. for
   outgoing conns where the chnl may be different for each rmt
   dev, secu requirements may be reged using the
   DM_SM_REG_OUTGOING_REQ msg.
   NOTE
   the secu manager enforces a default secu level in modes 2
   and 3, given by the SECL_DEFAULT definition. any access req for
   which there is no reged secu level will be subject to
   the default secu. therefore it is recommended that certain
   svcs and prots are always reged with no secu, in
   particular SDP and RFC.

   some secu examples are presented:

   eg 1 - A TCS terminal regs with 'prot_id' = SEC_PROT_L2C,
   'chnl' = 7 (cordless telephony PSM), and 'outgoing_ok' = TRUE. then
   L2C will req access from the secu manager for all conns,
   incoming and outgoing, on PSM 7.

   eg 2 - A headset prof might reg with 'prot_id' =
   SEC_PROT_RFC, 'chnl' = the RFC srv chnl num of the
   headset svc, and 'outgoing_ok' = FALSE. then RFC will req
   access from the secu manager for all incoming conns on that
   srv chnl. access reqs for outgoing conns will be
   subjected to the default secu level.
   eg 3 - as for (eg 2), but whenever the prof makes an outgoing
   conn to an audio gateway it first performs svc discov to
   obtain the srv chnl num on the AG dev, then regs using
   the DM_SM_REG_OUTGOING_REQ msg with 'prot_id' =
   SEC_PROT_RFC, 'chnl' = rmt srv chnl, 'bd' =
   dev addr of the AG. when the outgoing conn is started,
   RFC will req access from the secu manager which will apply
   the secu level reged for the outgoing conn. incoming
   conns proceed as in (eg 2).

   eg 4 - an app can specify the secu requirements for access
   to RFC by sending DM_SM_REG_REQ with 'prot_id' =
   SEC_PROT_L2C, 'chnl' = 3, and 'outgoing_ok' = TRUE. then L2C
   will req access from the secu manager for all new RFC
   conns */
typedef struct
{
    U16 type; /* always DM_SM_REG_REQ */

    /* prot at which secu is to be applied */
    U32 prot_id;

    /* chnl for that prot (e.g. RFC srv chnl num) */
    U32 chnl;

    /* TRUE if this registration applies to outgoing conns in addition to incoming conns */
    BOOL outgoing_ok;

    /* level of secu to be applied */
    U16 secu_level;

    /* the L2C PSM num is required if connless secu is used. if there is a conflict, where multiple svcs based on the same PSM have
       different connless secu requirements, then connless reception will be disbd for that PSM.
       if connless secu does not matter, set 'psm' to zero */
    U16 psm;
} BTS2S_DM_SM_REG_REQ;

/* Regs the secu requirements for outgoing conns to the specified prot and chnl on the specified rmt dev. this is
   typically used to ctrl secu when w4conn to a rmt RFC srv chnl.
   see the desp of DM_SM_REG_REQ (above) for more detail */
typedef struct
{
    U16 type; /* always DM_SM_REG_OUTGOING_REQ */

    /* the addr of the dev to which the given 'chnl' applies. for example, RFC srv chnls may be different on each dev */
    BTS2S_BD_ADDR bd;

    /* prot at which secu is to be applied */
    U32 prot_id;

    /* rmt chnl for that prot (e.g. RFC srv chnl num) */
    U32 rmt_chnl;

    /* level of secu to be applied - outgoing and connless only */
    U16 outgoing_secu_level;

    /* The L2C PSM num is required if connless secu is used. if there is a conflict, where multiple svcs based on the same PSM have
       different connless secu requirements, then connless reception will be disbd for that PSM.
       if connless secu does not matter, set 'psm' to zero */
    U16 psm;
} BTS2S_DM_SM_REG_OUTGOING_REQ;

/* unregs the secu requirements for a svc prely reged with DM_SM_REG_REQ. 'prot_id' and 'chnl' are the
   same as in the reg req */
typedef struct
{
    U16 type; /* always DM_SM_UNREG_REQ */

    /* prot and chnl nums prely reged */
    U32 prot_id;
    U32 chnl;
} BTS2S_DM_SM_UNREG_REQ;

/* unregs the secu requirements for a svc prely reged with DM_SM_REG_OUTGOING_REQ. 'bd', 'prot_id'
   and 'rmt_chnl' are the same as in the reg req */
typedef struct
{
    U16 type; /* always DM_SM_UNREG_OUTGOING_REQ */

    /* prot and chnl nums prely reged */
    BTS2S_BD_ADDR bd;
    U32 prot_id;
    U32 rmt_chnl;
} BTS2S_DM_SM_UNREG_OUTGOING_REQ;

/* req from a prot layer (e.g. L2C, RFC, or a higher layer) to
   determine if the rmt dev 'bd' is to be granted access to that
   prot layer's chnl.
   the secu manager will srch its svc and dev dbs, may
   perform some secu procedures, and may req authorisation from
   the app. the res is returned in a BTS2S_DM_SM_ACCESS_CFM
   msg */
typedef struct
{
    U16 type; /* always DM_SM_ACCESS_REQ */
    U16 phdl; /* phdl for the rsp */

    /* BD addr of the rmt dev */
    BTS2S_BD_ADDR bd;

    /* prot and chnl for the prot layer reqing access */
    U32 prot_id;
    U32 chnl;

    /* TRUE for incoming conn, FALSE for outgoing conn */
    BOOL incoming;

    /* an opaque val supplied by the reqing entity, and returned in the access confirm msg. it may be used to match a confirm to the
       correct req */
    U32 pv_context;
    U32 context;
} BTS2S_DM_SM_ACCESS_REQ;

/* rsp to an access req from a prot layer. this is sent to
   the phdl that was sent in the DM_SM_ACCESS_REQ msg */
typedef struct
{
    U16 type; /* always DM_SM_ACCESS_CFM */
    U16 phdl; /* dest phdl */

    /* the pars from the access req */
    BTS2S_BD_ADDR bd;
    U32 prot_id;
    U32 chnl;
    BOOL  incoming;

    /* TRUE for access granted, FALSE for access denied */
    BOOL granted;

    /* context val supplied in the req */
    U32 pv_context;
    U32 context;
} BTS2S_DM_SM_ACCESS_CFM;

/* req to place the secu manager into the specified secu mode.
   the res is returned in a DM_SM_SET_SEC_MODE_CFM msg. it is
   recommended that the secu mode is changed only when the system is
   not in the process of creating or acpting any new conns.
   mode 0 is for use by apps which perform their own secu
   procedures. the svc db is unused in this mode, whereas the
   dev db is used only for link keys. unknown link keys res in
   a link key req being forwarded to the app. this is the
   default mode at startup.

   in mode 1, the secu manager automatically acpts all access
   reqs without initiating auth, encryption, or
   authorisation procedures. any attmpt by an app to perform
   auth or encryption procedures will be rejed. the secu
   manager will respond to peer - invoked secu procedures as for mode 2.
   in mode 2, access reqs are subject to the secu requirements
   reged in the svc and dev dbs. the secu manager
   will respond to link key reqs using the dev db - if a link
   key is unknown, the req is forwarded to the app. PIN code
   reqs are forwarded to the app. the app may invoke
   secu procedures in this mode.
   in mode 3, the host ctrller is cfg to perform auth
   and possibly encryption at LMP link setup. the secu manager still
   uses the svc db to ctrl authorisation and encryption.
   the encryption level for mode 3 is specified in 'bts2s_mode3_enc'. if the host
   ctrller will not enter auth mode, then the req fails
   (DM_SM_SET_SEC_MODE_CFM has a negative rsp). if the host ctrller
   does enter auth mode, then the req succeeds. note that the
   reqed encryption level may not be supp - in this case the
   req still succeeds and the actual encryption level is returned in
   DM_SM_SET_SEC_MODE_CFM */
typedef struct
{
    U16 type; /* always DM_SM_SET_SEC_MODE_REQ */

    /* which secu mode to use */
    U8  mode;

    /* encryption level for mode 3. one of:
       HCI_ENC_MODE_OFF
       HCI_ENC_MODE_PT_TO_PT
       HCI_ENC_MODE_PT_TO_PT_AND_BCAST */
    U8             bts2s_mode3_enc;
} BTS2S_DM_SM_SET_SEC_MODE_REQ;

/* rsp to a secu mode change req. A mode change may fail if
   an unknown mode was reqed, or if the host ctrller refused to
   enb / disb auth at LMP link setup */

typedef struct
{
    U16 type; /* always DM_SM_SET_SEC_MODE_CFM */
    U16 phdl; /* dest phdl */
    U8  mode;
    U8  bts2s_mode3_enc;

    /* TRUE if the reqed secu mode has been enbd */
    BOOL succ;
} BTS2S_DM_SM_SET_SEC_MODE_CFM;

/* req to add a dev to the secu manager's dev db, or to modify the record for an existing entry. the oper will be
   confirmed in a DM_SM_ADD_DEV_CFM msg */
typedef struct
{
    U16 type; /* always DM_SM_ADD_DEV_REQ */

    /* the addr of the rmt dev */
    BTS2S_BD_ADDR bd;

    /* the trust level of the rmt dev, TRUE=trusted, FALSE=untrusted.
       trusted devs are automatically granted access by the secu manager.
       untrusted devs res in a DM_SM_AUTHORISE_IND for access to prots
       that require authorisation.
       in SEC_MODE2a_DEV, a dev with 'trusted' set to TRUE will not be
       authd when a new svc is started.
       the trust level is changed to 'trusted' if 'update_trust_level' is TRUE */
    BOOL update_trust_level;
    BOOL trusted;

    /* the link key for the rmt dev. the secu manager expects to have
       a cp of the link key for each dev. the trust level may be changed
       for an existing dev record by setting 'update_link_key' to FALSE, in
       which case the secu manager will not overwr its cp of the link
       key */
    BOOL update_link_key;
    U8 link_key[LINK_KEY_SIZE];
} BTS2S_DM_SM_ADD_DEV_REQ;

/* cfm of adding a dev to the dev db */
typedef struct
{
    U16 type; /* always DM_SM_ADD_DEV_CFM */
    U16 phdl; /* dest phdl */

    /* the addr of the rmt dev */
    BTS2S_BD_ADDR bd;

    /* TRUE if added, FALSE if not added */
    BOOL succ;
} BTS2S_DM_SM_ADD_DEV_CFM;

/* req to remove a dev from the secu manager's dev db. the oper will be confirmed in a DM_SM_REMOVE_DEV_CFM msg */
typedef struct
{
    U16 type; /* always DM_SM_REMOVE_DEV_REQ */

    /* the addr of the rmt dev to remove */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_SM_REMOVE_DEV_REQ;

/* cfm of removing a dev from the dev db */
typedef struct
{
    U16 type; /* always DM_SM_REMOVE_DEV_CFM */
    U16 phdl; /* dest phdl */
    /* the addr of the rmt dev that was removed */
    BTS2S_BD_ADDR bd;
    /* TRUE if the dev was removed, FALSE if it did not exist */
    BOOL succ;
} BTS2S_DM_SM_REMOVE_DEV_CFM;

/* A req from the secu manager for a link key required for auth with the specified rmt dev. the app must
   respond with a DM_SM_LINK_KEY_REQ_RES msg containing the link
   key, or with 'vld' set to FALSE to rej the req.
   the secu manager will only issue a link key req if it does not
   alrdy have a vld link key for that dev in the dev db */

typedef struct
{
    U16 type; /* always DM_SM_LINK_KEY_REQ_IND */
    U16 phdl; /* dest phdl */

    /* the addr of the rmt dev for which a link key is required */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_SM_LINK_KEY_REQ_IND;

/* A rsp to the secu manager containing the link key for rmt
  dev 'bd'. if no link key is known, then set 'vld' to FALSE to
  rej the req */
typedef struct
{
    U16 type; /* always DM_SM_LINK_KEY_REQ_RES */

    /* the addr of the rmt dev for which a link key was reqed */
    BTS2S_BD_ADDR bd;

    /* the link key - set 'vld' to FALSE to rej */
    U8 key[LINK_KEY_SIZE]; /* the link key val */
    BOOL vld; /* TRUE if link key vld, FALSE to rej */
} BTS2S_DM_SM_LINK_KEY_REQ_RES;

/* A req from the secu manager for a PIN code required for  pairing with the specified rmt dev. the app must respond
   with a DM_SM_PIN_REQ_RES msg containing the PIN code, or
   with 'pin_len' set to 0 to rej the req */

typedef struct
{
    U16 type; /* always DM_SM_PIN_REQ_IND */
    U16 phdl; /* dest phdl */

    /* the addr of the rmt dev for which a PIN is required */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_SM_PIN_REQ_IND;

/* A rsp to the secu manager containing the PIN for rmt dev
  'bd'. if no PIN is known, then set 'pin_len' to zero to rej
  the req.
  if pairing succeeds as a res of the PIN, then the secu manager
  will issue a DM_SM_LINK_KEY_IND msg, containing the new link key */
typedef struct
{
    U16 type; /* always DM_SM_PIN_REQ_RES */

    /* the addr of the rmt dev for which a PIN was reqed */
    BTS2S_BD_ADDR bd;

    /* the PIN code - set 'pin_len' to zero to rej */
    U8 pin_len;
    U8 pin[HCI_MAX_PIN_LEN];
} BTS2S_DM_SM_PIN_REQ_RES;

/* A noti from the secu manager that a new link key is now in force for the rmt dev 'bd'. this can be as a res of
   pairing, or as a res of one of the devs reqing a change of
   link key with the change_conn_link_key HCI cmd.
   the secu manager keeps a cp of the new link key, and passes it up
   to the app for non - volatile storage */
typedef struct
{
    U16 type; /* always DM_SM_LINK_KEY_IND */
    U16 phdl; /* dest phdl */

    /* the addr of the rmt dev with which pairing compd */
    BTS2S_BD_ADDR bd;

    /* the link key type and val. the type can be:
       HCI_COMBINATION_KEY
       HCI_LOCAL_UNIT_KEY
       HCI_RMT_UNIT_KEY */
    U8 key_type;
    U8 key[LINK_KEY_SIZE];
} BTS2S_DM_SM_LINK_KEY_IND;

/* A req for authorisation from the secu manager to the app. this is only performed when an untrusted or unknown dev
   is attmpting to access a svc that requires authorisation in
   secu mode 2.
   the app must make a decision and then respond with a
   DM_SM_AUTHORISE_RES msg containing the res. devs can be
   marked as trusted using the DM_SM_ADD_DEV_REQ msg */
typedef struct
{
    U16 type; /* always DM_SM_AUTHORISE_IND */
    U16 phdl; /* dest phdl */

    /* dev addr, and the prot + chnl of the svc being accessed */
    BTS2S_BD_ADDR bd;
    U32 prot_id;
    U32 chnl;

    /* TRUE for incoming conn, FALSE for outgoing conn */
    BOOL incoming;
} BTS2S_DM_SM_AUTHORISE_IND;

/* authorisation rsp from the app to the secu manager */
typedef struct
{
    U16 type; /* always DM_SM_AUTHORISE_RES */

    /* the pars from the authorization req */
    BTS2S_BD_ADDR bd;
    U32 prot_id;
    U32 chnl;
    BOOL incoming;

    /* TRUE if authorisation granted, FALSE to rej */
    BOOL authorised;
} BTS2S_DM_SM_AUTHORISE_RES;

/* apps may req auth of a link at any time, using
   the DM_SM_AUTH_REQ msg. if the link has alrdy been
   authd, the secu manager responds immediately, otherwise it
   performs the HCI auth procedure, which may involve pairing.
   this req will be rejed in secu mode 1 */
typedef struct
{
    U16 type; /* always DM_SM_AUTH_REQ */

    /* addr of dev to auth */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_SM_AUTH_REQ;

/* res of an auth req */
typedef struct
{
    U16 type; /* always DM_SM_AUTH_CFM */
    U16 phdl; /* dest phdl */

    /* addr of dev being authd */
    BTS2S_BD_ADDR bd;

    /* TRUE if succful */
    BOOL succ;
    /* failed reason if FALSE */
    U8 res;
} BTS2S_DM_SM_AUTH_CFM;

/* apps may req encryption act or de - act of a
   link at any time, using the DM_SM_ENCRYPT_REQ msg. if the link
   encryption is alrdy in the reqed st, the secu manager
   responds immediately, otherwise it performs the relevant HCI procedure.
   the link must have been authd before encryption is allowed.
   this req will be rejed in secu mode 1 */
typedef struct
{
    U16 type; /* always DM_SM_ENCRYPT_REQ */

    /* link to encrypt */
    BTS2S_BD_ADDR bd;

    /* TRUE to enb encryption, FALSE to stop encryption */
    BOOL encrypt;
} BTS2S_DM_SM_ENCRYPT_REQ;

/* res of a locally - initiated encryption req (DM_SM_ENCRYPT_REQ) */
typedef struct
{
    U16 type; /* always DM_SM_ENCRYPT_CFM */
    U16 phdl; /* dest phdl */

    /* link being encrypted */
    BTS2S_BD_ADDR bd;

    /* TRUE if the cmd was succful, FALSE if it failed. the actual encryption st of the link is in 'encrypted' */
    BOOL succ;

    /* TRUE if encrypted, FALSE if not encrypted */
    BOOL encrypted;
} BTS2S_DM_SM_ENCRYPT_CFM;

/* noti of a possible change in the encryption st of a link due to a peer - initiated encryption procedure */
typedef struct
{
    U16 type; /* always DM_SM_ENCRYPTION_CHANGE */
    U16 phdl; /* dest phdl */

    /* link being encrypted */
    BTS2S_BD_ADDR bd;

    /* TRUE if encrypted, FALSE if not encrypted */
    BOOL encrypted;
} BTS2S_DM_SM_ENCRYPTION_CHANGE;

/* internal to stk
   L2C needs to reg with the secu manager in order to recv
   notis of which Psms should allow connless reception */
typedef struct
{
    U16 type; /* always DM_SM_L2C_REG_REQ */

    /* phdl to recv connless secu infmtion */
    U16 phdl;
} BTS2S_DM_SM_L2C_REG_REQ;

/* internal to stk
   the secu manager informs L2C of which Psms should allow
   connless reception. in case of conflict with connless
   settings defined by the L2CA_ENB_CONNLESS_REQ and
   L2CA_DISB_CONNLESS_REQ msgs, any global enb
   takes priority over secu manager enb, and secu
   manager enb takes priority over individual L2C
   enb */
#define MAX_CLRX_PSMS 10
typedef struct
{
    U16 type; /* always DM_SM_L2C_CLRX_ENB_IND */

    /* phdl to recv connless secu infmtion */
    U16 phdl;

    /* array of Psms for which connless reception is allowed. all other Psms should disallow it */
    U16 psm_array[MAX_CLRX_PSMS];
} BTS2S_DM_SM_L2C_CLRX_ENB_IND;


/* CONN PAR CACHE UI */

/* cache the page mode pars for a given bd */
typedef struct
{
    U16 type;           /* always DM_WR_CACHED_PAGE_MODE_REQ */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 page_scan_mode;
    U8 page_scan_rep_mode;
} BTS2S_DM_WR_CACHED_PAGE_MODE_REQ;

/* always DM_WR_CACHED_PAGE_MODE_CFM */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_WR_CACHED_PAGE_MODE_CFM;

/* cache the clock offset par for a given bd */
typedef struct
{
    U16 type;           /* always DM_WR_CACHED_CLOCK_OFFSET_REQ */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U16 clock_offset;   /* the clock offset */
} BTS2S_DM_WR_CACHED_CLOCK_OFFSET_REQ;

/* always DM_WR_CACHED_CLOCK_OFFSET_CFM */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_WR_CACHED_CLOCK_OFFSET_CFM;

/* clear all cached pars for a given bd */
typedef struct
{
    U16 type;           /* always DM_CLEAR_PARAM_CACHE_REQ */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
} BTS2S_DM_CLEAR_PARAM_CACHE_REQ;

/* always DM_CLEAR_PARAM_CACHE_CFM */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_CLEAR_PARAM_CACHE_CFM;

/* letting higher layers know about the flow ctrl st */
typedef struct
{
    U16 type; /* always DM_REG_CFM */
    U16 phdl; /* dest phdl */
    BOOL act; /* TRUE = ON */

} BTS2S_DM_HC_TO_HOST_FLOW_CTRL;

/* req from app task for the DM to create an ACL to the
   specified rmt dev. this might be used for pairing, for example.
   the DM allows both L2C and the app to req ACls. L2C
   uses the DM_ACLQUE ui msgs (DM_ACL_CONN_REQ / CFM),
   and the app uses the BTS2T_HCI_CMD msgs
   (DM_ACL_OPEN_REQ / CFM). the DM keeps track of who has an interest in
   each ACL, and will only disc the ACL when all interested parties
   have reld the ACL (by DM_ACL_DISC_REQ and DM_ACL_CLOSE_REQ).

   DM_ACL_OPEN_REQ regs the app's interest in the ACL. DM will
   create the link if it does not alrdy exist. DM will always respond
   with DM_ACL_OPEN_CFM, indicating succ or fail. if succful, the
   app may subsequently call DM_ACL_CLOSE_REQ to relinquish its
   interest in the ACL - DM will then rel the link if L2C is not
   using it.
   the DM keeps the app informed of the st of all ACls via the
   DM_ACL_OPENED_IND and DM_ACL_CLOSED_IND msgs. note that there is
   no specific rsp to DM_ACL_CLOSE_REQ, as the DM_ACL_CLOSED_IND is
   only issued when all users of the ACL have reld it */
typedef struct
{
    U16 type;           /* always DM_ACL_OPEN_REQ */
    BTS2S_BD_ADDR bd;   /* rmt dev addr */
} BTS2S_DM_ACL_OPEN_REQ;

/* cfm of an app req to create an ACL. see text for
   DM_ACL_OPEN_REQ for more details */
typedef struct
{
    U16 type;           /* always DM_ACL_OPEN_CFM */
    U16 phdl;           /* dest phdl */
    U16 acl_hdl;        /*Rember the acl handle number*/
    BTS2S_BD_ADDR bd;   /* rmt dev addr */
    BOOL succ; /* TRUE if open, FALSE if not */
} BTS2S_DM_ACL_OPEN_CFM;

/* req from the app to close the ACL to the specified dev.
   an app may send this req only if it prely reqed
   ACL creation via the DM_ACL_OPEN_REQ msg.
   there is no specific rsp to this msg - DM_ACL_CLOSED_IND is
   issued when the ACL is closed, but the DM may keep the ACL open if L2C
   is still using it */
typedef struct
{
    U16 type; /* always DM_ACL_CLOSE_REQ */
    BTS2S_BD_ADDR bd;   /* rmt dev addr */
} BTS2S_DM_ACL_CLOSE_REQ;

/* let higher layers know about ACL comings and goings. this msg
   is issued to the app ui whenever an ACL is succfully
   estbed (incoming or outgoing) */
typedef struct
{
    U16 type;           /* always DM_ACL_OPENED_IND */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* peer dev addr */
    /* flag indicating peer - initiated ACL (TRUE) or locally - initiated (FALSE) */
    BOOL incoming;
    /* cls of dev for peer, vld for incoming conns only */
    U24 dev_cls;
} BTS2S_DM_ACL_OPENED_IND;

/* letting higher layers know about ACL comings and goings. this msg
   is issued to the app ui whenever a succfully
   estbed ACL is disced */
typedef struct
{
    U16 type;           /* always DM_ACL_CLOSED_IND */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* peer dev addr */
} BTS2S_DM_ACL_CLOSED_IND;

/* inform the application when remote side want to create an ACL link */
typedef struct
{
    U16 type;           /* always DM_ACL_OPENED_IND */
    BTS2S_BD_ADDR bd;   /* peer dev addr */
    U24 dev_cls;        /* device class */
} BTS2S_DM_ACL_OPEN_IND;

/* define the default link policy to be applied to all new conns
   this does not affect existing conns */
typedef struct
{
    U16 type;           /* always DM_SET_DEFAULT_LINK_POLICY */

    /* link manager policy settings for incoming and outgoing conns */
    U16 link_policy_setting_in;
    U16 link_policy_setting_out;
} BTS2S_DM_SET_DEFAULT_LINK_POLICY;

/* HCI API MSGITIVES */

/* inquiry cmd */

typedef BTS2S_HCI_INQUIRY DMHCI_INQUIRY;

/* inquiry cmd */

typedef BTS2S_HCI_INQUIRY_ESC DMHCI_INQUIRY_ESC;

#ifdef CFG_BT_VER_21
typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 io_capability;
    U8 oob_data_present;
    U8 auth_req;
} BTS2S_DM_HCI_IO_CAPABILITY_REQ_REPLY;
#endif

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 reason;
} BTS2S_DM_HCI_IO_CAPABILITY_REQ_NEG_REPLY;

typedef struct
{
    BTS2S_HCI_CMD_COMMON common;
    BTS2S_BD_ADDR bd;
    U8 confirm;
} BTS2S_DM_HCI_USER_CFM_REQ_RSP;


/* noti of inquiry esc cmd comp */
/* always BTS2S_DM_HCI_INQUIRY_ESC_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_INQUIRY_ESC_COMP;


/* enter periodic inquiry mode */

typedef BTS2S_HCI_PERIODIC_INQUIRY_MODE BTS2S_DM_HCI_PERIODIC_INQUIRY;

/* noti of entered periodic inquiry mode */
/* always BTS2S_DM_HCI_PERIODIC_INQUIRY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_PERIODIC_INQUIRY_COMP;

/* exit periodic inquiry mode */

typedef BTS2S_HCI_EXIT_PERIODIC_INQUIRY_MODE BTS2S_DM_HCI_EXIT_PERIODIC_INQUIRY;

/* noti of exited periodic inquiry mode */
/* always BTS2S_DM_HCI_EXIT_PERIODIC_INQUIRY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_EXIT_PERIODIC_INQUIRY_COMP;

/* noti of inquiry ress see BTS2S_HCI_EV_INQUIRY_RES */

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    U8 rsp_num;

    BTS2S_HCI_INQ_RES *res[HCI_MAX_INQ_RES_PTRS];

} BTS2S_DM_HCI_INQUIRY_RES;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    U8 rsp_num;

    BTS2S_HCI_INQ_RES res;

} BTS2S_DM_HCI_INQUIRY_RES_ext;


/* noti of inquiry ress with rssi infmtion see BTS2S_HCI_EV_INQUIRY_RES_WITH_RSSI */

#define DM_HCI_INQUIRY_RES_WITH_RSSI ((U16)(ENUM_DM_HCI_INQUIRY_RES_WITH_RSSI))

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    U8 rsp_num;

    BTS2S_HCI_INQ_RES_WITH_RSSI *res[HCI_MAX_INQ_RES_PTRS];

} BTS2S_DM_HCI_INQUIRY_RES_WITH_RSSI;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    U8 rsp_num; // only support rsp_num is 1

    BTS2S_HCI_INQ_RES_WITH_RSSI res;

} BTS2S_DM_HCI_INQUIRY_RES_WITH_RSSI_ext;


#ifdef CFG_BT_VER_21

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    BTS2S_HCI_EV_EXT_INQ_RESPONSE resp;
} BTS2S_DM_EXT_HCI_INQUIRY_RES;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_IO_CAP_REQ;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    U8 io_capability;
    U8 oob_data_present;
    U8 auth_req;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_IO_CAP_RSP;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */
    BTS2S_BD_ADDR bd;
    U32 num_val;
} BTS2S_HCI_USER_CONF_REQ;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */

    BTS2S_BD_ADDR bd;

} BTS2S_HCI_USER_PASSKY_REQ;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */

    BTS2S_BD_ADDR bd;

} BTS2S_HCI_RMT_OOB_DATA_REQ;

/* Simple Pairing Complete req */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */

    U8 st;
    BTS2S_BD_ADDR bd;
} BTS2S_HCI_SSP_PAIR_COMPLETE;

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_RES */
    U16 phdl; /* dest phdl */

    BTS2S_BD_ADDR bd;
    U8 notifi_type;

} BTS2S_HCI_KEYPRESS_NOTIFI;

#endif

/* noti of inquiry comp */

typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_INQUIRY_COMP */
    U16 phdl; /* dest phdl */
    U8  st;   /* cast to err if err */

} BTS2S_DM_HCI_INQUIRY_COMP;

/* req change of pkt type on a conn */
typedef BTS2S_HCI_CHANGE_CONN_PKT_TYPE BTS2S_DM_HCI_CHANGE_PKT_TYPE;



/* change link key for a dev */

typedef BTS2S_HCI_CHANGE_CONN_LINK_KEY BTS2S_DM_HCI_CHANGE_LINK_KEY;

/* noti of change of link key for a dev */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_LINK_KEY_CHANGE_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */

} BTS2S_DM_HCI_LINK_KEY_CHANGE_COMP;

/* noti of link keys for devs see BTS2S_HCI_EV_RETURN_LINK_KEYS for details */
typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RETURN_LINK_KEYS */
    U16 phdl;           /* dest phdl */
    U8 key_num;         /* num of link keys */
    LINK_KEYBD_ADDR link_key_bd[HCI_STORED_LINK_KEY_MAX];

} BTS2S_DM_HCI_RETURN_LINK_KEYS;

/*  use the master dev link keys */

typedef BTS2S_HCI_MASTER_LINK_KEY DMHCI_MASTER_LINK_KEY;

/* noti of use of the master dev link keys */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_MASTER_LINK_KEY_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    U8 key_flag;        /* regular / tmporary link key */

} BTS2S_DM_HCI_MASTER_LINK_KEY_COMP;

/* rmt name req */

typedef BTS2S_HCI_RMT_NAME_REQ BTS2S_DM_HCI_RMT_NAME_REQ;

/*  PURPOSE noti of rmt name */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RMT_NAME_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    /* user friendly name */
    U8 *name_part[HCI_LOCAL_NAME_BYTE_PKT_PTRS];

} BTS2S_DM_HCI_RMT_NAME_COMP;

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RMT_NAME_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    /* user friendly name */
    U8 name_part[HCI_NAME_LEN];

} BTS2S_DM_HCI_RMT_NAME_COMP_ext;


/* rd rmt featr */

typedef BTS2S_HCI_RD_RMT_SUPP_FEATR BTS2S_DM_HCI_RD_RMT_FEATR;

/* noti of rmt featr */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RD_RMT_FEATR_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U16 featr[4];       /* LMP featr */

} BTS2S_DM_HCI_RD_RMT_FEATR_COMP;

/* rd rmt version */

typedef BTS2S_HCI_RD_RMT_VER_INFO BTS2S_DM_HCI_RD_RMT_VERSION;

/* noti of rmt version infmtion */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RD_RMT_VERSION_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 LMP_version;     /* version of LMP */
    U16 manufacturer_name; /* name of manufacturer */
    U16 LMP_subversion; /*subversion of LMP */

} BTS2S_DM_HCI_RD_RMT_VERSION_COMP;

/* rd clock offset */

typedef BTS2S_HCI_RD_CLOCK_OFFSET DMHCI_RD_CLOCK_OFFSET;

/* rd clock offset */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RD_CLOCK_OFFSET_COMP */
    U16 phdl;           /* dest phdl */
    U8 st;              /*succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U16 clock_offset;   /* dev's clock offset */

} BTS2S_DM_HCI_RD_CLOCK_OFFSET_COMP;

/* put local or rmt dev into hold mode use the rmt bluetooth dev addr to map onto
   conn hdl */

typedef BTS2S_HCI_HOLD_MODE DMHCI_HOLD_MODE;

/* put local dev (for a particular conn) into BTS2S_SNIFF mode
   note, SCO conns cannot be put into BTS2S_SNIFF */

typedef BTS2S_HCI_SNIFF_MODE DMHCI_SNIFF_MODE;

/* end local dev (for a particular conn) in BTS2S_SNIFF mode
   note, SCO conns cannot be put into BTS2S_SNIFF */

typedef BTS2S_HCI_EXIT_SNIFF_MODE DMHCI_EXIT_SNIFF_MODE;


/* mode change ev */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_MODE_CHANGE_EV */
    U16 phdl;           /* dest phdl */
    U8 st;              /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 mode;            /* cur mode */
    U16 len;            /* len of mode (or similar) */

} BTS2S_DM_HCI_MODE_CHANGE_EV;

/* mode change ev */
#ifdef CFG_BT_VER_21
/* more information please referr to */
/* Volume Part_E[7.7.37 Sniff Subrating Event] */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_MODE_CHANGE_EV */
    U16 phdl; /* dest phdl */
    U8 status;
    U16 connection_hdl;
    U16 maximum_transmit_latency;
    U16 maximum_receive_latency;
    U16 minimum_remote_timeout;
    U16 minimum_local_timeout;

} BTS2S_DM_HCI_SUBNIFF_MODE_CHANGE_EV;
#endif


/* qa of svc setup from L2C */

typedef BTS2S_HCI_QOS_SETUP BTS2S_DM_HCI_QOS_SETUP_REQ;

/* completion of qa of svc setup */
typedef struct
{
    U16 type;           /* always DM_HCI_QOS_SETUP_CFM */
    U16 phdl;           /* dest phdl */
    U8 st;              /* cast to err if err */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */
    U8 flag;            /* rsv */
    U8 svc_type;
    U32 token_rate;
    U32 peak_bandwidth;
    U32 latency;
    U32 delay_variation;
} BTS2S_DM_HCI_QOS_SETUP_CFM;

/* qa of svc violation */
typedef struct
{
    U16 type; /* always DM_HCI_QOS_VIOLATION_IND */
    U16 phdl; /* dest phdl */
} BTS2S_DM_HCI_QOS_VIOLATION_IND;

/* discover role use the bd to identify the (ACL) link at this ui */

typedef BTS2S_HCI_ROLE_DISCOV DMHCI_ROLE_DISCOV;

/* discover role comp */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_ROLE_DISCOV_COMP */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* BD_ADDR of link */
    U8 st;              /* succ or fail - see note1 */
    U8 role;            /* master or slave */

} BTS2S_DM_HCI_ROLE_DISCOV_COMP;

/* switch role use the bd to identify the (ACL) link at this ui */

typedef BTS2S_HCI_SWITCH_ROLE DMHCI_SWITCH_ROLE;

/* switch role comp the cur role is contained in 'role', even in case of fail */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_SWITCH_ROLE_COMP */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* BD_ADDR of link */
    U8 st;              /* succ or fail - see note1 */
    U8 role;            /* master or slave */

} BTS2S_DM_HCI_SWITCH_ROLE_COMP;

/* rd link policy settings from LM use the bd to identify the (ACL) link at this ui */

typedef BTS2S_HCI_RD_LINK_POLICY_SETTINGS BTS2S_DM_HCI_RD_LP_SETTING;

/* rd link policy settings comp from LM */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_RD_LP_SETTINGS_COMP */
    U16 phdl;           /* dest phdl */
    U8  st;             /* st */
    BTS2S_BD_ADDR bd;   /* bd of link */
    U16 link_policy;    /* link policy settings */

} BTS2S_DM_HCI_RD_LP_SETTINGS_COMP;

typedef struct
{

    BTS2S_BD_ADDR bd;
    U8 link_type;
} BTS2S_HCI_EV_SYNC_LINK;


/*
    wr link policy settings.
    use the bd to identify the ACL link at this ui.

    there are two settings - the link manager settings, which define the
    conn modes that are allowed (hold / BTS2S_SNIFF / BTS2S_PARK); and the dev
    manager settings, which define how the dev manager tries to ctrl
    the conn mode. it is the responsibility of the caller to specify
    compatible LM and DM link policies.

    the DM policies are as follows:
    DM_LINK_POLICY_DEFAULT - DM does not manage the link in any way.
    it is left to the higher layers to ctrl the BTS2S_PARK and BTS2S_SNIFF
    modes. this is the default mode if no DM policy is specified.
    DM_LINK_POLICY_KEEP_ACT - DM tries to keep the link act as
    much as possible. if the link is placed into BTS2S_PARK mode or BTS2S_SNIFF
    mode, DM will try to return the link to act mode as soon as
    possible.
    DM_LINK_POLICY_KEEP_SNIFF - DM tries to keep the link in BTS2S_SNIFF
    mode. if the link gets BTS2S_PARked, DM will try to unBTS2S_PARK it and then
    enter BTS2S_SNIFF mode. this mode is intended for use in very low
    data - rate apps. the BTS2S_SNIFF settings are specified in the
    'bts2s_sniff_setting' structure.
    DM_LINK_POLICY_KEEP_PARKED - DM tries to keep the link in BTS2S_PARK
    mode as much as possible, while it is "idle" and there are no
    SCO conns with the rmt dev. the link is deemed to be
    "idle" if there has been no ACL data in either direction during
    the last 'park_idle_time' seconds. once the idle criteria are
     met, DM will try to BTS2S_PARK the conn. if the conn is
    BTS2S_PARked at any time and the idle criteria are not met, DM will
    try to unBTS2S_PARK the conn. if ACL data is seen while the
    conn is BTS2S_PARked, DM will unBTS2S_PARK the conn. if a
    locally - initiated SCO conn is reqed while the ACL
    conn is BTS2S_PARked, DM will unBTS2S_PARK the conn before
    attmpting to estb the SCO conn. the BTS2S_PARK settings
    are specified in the 'bts2s_park_setting' structure.
     - DM_LINK_POLICY_TRANSPARENT - this policy is intended for use by
    apps that don't really care what mode the link is in,
    and do not want to actly ctrl the mode. the DM will allow
    the rmt dev to place the link in any mode. however, if
    dw_msg ACL traffic or an outgoing SCO conn req occurs
    while the link is BTS2S_PARked, the DM will unBTS2S_PARK the link to allow
    data transfer.
    - DM_LINK_POLICY_TWO_STAGE_SNIFF - DM toggles between two BTS2S_SNIFF
    intvls depending on whether there has been ACL data for a
    specified period of time.
    'min_intvl' is the BTS2S_SNIFF intvl that is used when there
    has been an ACL pkt within the timeout period.
    'max_intvl' is the BTS2S_SNIFF intvl that is used when there
    has been no ACL activity for the timeout period.
    'attmpt' and 'timeout' are the same as a normal BTS2S_SNIFF req.
    'link_policy_setting' is overloaded for this DM policy to
    represent the timeout in milliseconds.  note that the
    link_policy_setting therefore are not passed to HCI for this
    special msg.
    the timeout is not reset on receiving a new set of pars,
    so if alrdy using max_intvl, changing this val will cause
    the DM to enter the new mode without returning to min_intvl.
    - DM_LINK_POLICY_NO_CHANGE - not really a mode, this simply means
    that the cur DM link policy is not changed. this is used to
    change the LM link policy settings without affecting the DM link
    policy settings.

    the DM will acpt mode change reqs from the app (i.e.
    DM_HCI_PARK_MODE, DM_HCI_EXIT_SNIFF_MODE etc) only when the ACL conn is
     in the DM_LINK_POLICY_DEFAULT mode. when it is in any of the other
    modes, any mode change reqs will just be answered with a mode change
    ev, DM_HCI_MODE_CHANGE_EV, indicating the cur mode.

    in all modes, the DM will keep the app informed of any changes
    of mode by issuing DM_HCI_MODE_CHANGE_EV evs */


#define DM_LINK_POLICY_DEFAULT         ((U8)0x00)
#define DM_LINK_POLICY_KEEP_ACT        ((U8)0x01)
#define DM_LINK_POLICY_KEEP_SNIFF      ((U8)0x02)
#define DM_LINK_POLICY_TRANSPARENT     ((U8)0x04)
#define DM_LINK_POLICY_NO_CHANGE       ((U8)0x05)
#define DM_LINK_POLICY_TWO_STAGE_SNIFF ((U8)0x06)
#define DM_LINK_POLICY_KEEP_SNIFF_INTERVAL ((U8)0x07)

#define HCI_LINK_POLICY_NO_CHANGE      ((U16)0xffff)

/* the settings to be used in DM mode DM_LINK_POLICY_KEEP_SNIFF */
#ifndef BTS2S_SNIFF_SETTINGS_DEF
#define BTS2S_SNIFF_SETTINGS_DEF
typedef struct
{
    U16 max_intvl;  /* max BTS2S_SNIFF intvl */
    U16 min_intvl;  /* min BTS2S_SNIFF intvl */
    U16 attmpt;     /* sniff attmpt */
    U16 timeout;    /* sniff timeout */
} BTS2S_SNIFF_SETTINGS;
#endif


typedef union
{
    BTS2S_SNIFF_SETTINGS bts2s_sniff_setting;
} U_SNIFF_PARK;

typedef struct
{
    U16 type;           /* always DM_ACL_SET_LINK_POLICY */
    BTS2S_BD_ADDR bd;   /* bluetooth dev addr */

    /* link manager link policy settings - i.e. hold / BTS2S_SNIFF / BTS2S_PARK allowed, sent
       to host ctrller via HCI_wr_link_policy_setting cmd.
       can be set to HCI_LINK_POLICY_NO_CHANGE to not send the HCI cmd */
    U16  link_policy_setting;
    U16 sniff_idle_time;
    /* dev manager link policy */
    U8 dm_policy;

    U_SNIFF_PARK u;


} BTS2S_DM_HCI_WR_LP_SETTINGS;


/* defines the powerst(s) to be applied to the specified ACL. it
   takes the form of a list of succive power modes which the ACL moves
   through over periods of increasing inactivity.  if at anytime there
   is activity on the ACL, ctrl will reset to the first powerst.
   each powerst is defined using the BTS2S_LP_POWERST structure.
   the 'mode' entry is used to define the bluetooth power mode.
   the 'min_intvl' etc fields define the pars of the bluetooth
   power mode - see the HCI spec.
   the 'duration' is the len of time, in seconds, that the ACL will
   remain in this st.  when this period expires, ctrl moves
   to the next st.  the last powerst therefore MUST have an
   infinite duration (zero) */

#define LP_POWERMODE_ACT     ((U8)0x00)
#define LP_POWERMODE_SNIFF   ((U8)0x01)
/* passive mode is a "don't care" setting where the local dev will not
   attmpt to alter the power mode */
#define LP_POWERMODE_PASSIVE ((U8)0xff)

typedef struct
{
    U8 mode;
    U16 min_intvl;
    U16 max_intvl;
    U16 attmpt;
    U16 timeout;
    U16 duration; /* time to spend in this mode */
} BTS2S_LP_POWERST;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;       /* bluetooth dev addr */
    U16 num_st;             /* num of st in list */
    BTS2S_LP_POWERST *st;   /* table of power st */
} BTS2S_DM_LP_WR_POWERSTS_REQ;

/* generated in rsp to DM_LP_WR_POWERSTS_REQ */
#define LP_POWERSTS_SUCC             ((U8)0) /* power_st set succfully */
#define LP_POWERSTS_UNSUPP_MODE      ((U8)1) /* unrecognised LP_POWERMODE */
#define LP_POWERSTS_UNKNOWN_DEV      ((U8)2) /* unknown bluetooth dev */
#define LP_POWERSTS_BAD_TERMINATION  ((U8)3) /* last st duration not zero */
#define LP_POWERSTS_ERR              ((U8)4) /* err described above */
typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U8 res;
} BTS2S_DM_LP_WR_POWERSTS_CFM;

/* wr link policy settings comp from LM */

typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_WR_LP_SETTINGS_COMP */
    U16 phdl;           /* dest phdl */
    U8  st;             /* st */
    BTS2S_BD_ADDR bd;   /* bd of link */

} BTS2S_DM_HCI_WR_LP_SETTINGS_COMP;


#ifndef CFG_NOT_STK_EN
/* enbd / disbs the enhanced featr of stk.
   stk will respond with DM_EN_ENB_ENHANCEMENTS_CFM.
   NB: it may be dangerous to enb / disb particular bts2s_enhancement
   during particular opers.  for this reason, it is recommended
   that all modifications are carried out during initialisation */
#define ENHANCEMENT_ACL_INDI        ((U32)0x00000001)

typedef struct
{
    U16 type;
    U32 bts2s_enhancement;
} BTS2S_DM_EN_ENB_ENHANCEMENTS_REQ;

/* rsp to a DM_EN_ENB_ENHANCEMENTS_REQ.
  'enbd_enhancement' contains the curly enbd bts2s_enhancement.
  (stk will only allow supp bts2s_enhancement to be enbd!) */
typedef struct
{
    U16 type;
    U32 enbd_enhancement;
} BTS2S_DM_EN_ENB_ENHANCEMENTS_CFM;

/* rd the curly enbd bts2s_enhancement. stk will respond with DM_EN_RD_ENHANCEMENTS_CFM */
typedef struct
{
    U16       type;
} BTS2S_DM_EN_RD_ENHANCEMENTS_REQ;

/* generated in rsp to a DM_EN_RD_ENHANCEMENTS_REQ. 'enbd_enhancement' indicates which bts2s_enhancement are enbd.
  'supp_enhancement' indicates the bts2s_enhancement supp */
typedef struct
{
    U16 type;
    U32 enbd_enhancement;
    U32 supp_enhancement;
} BTS2S_DM_EN_RD_ENHANCEMENTS_CFM;


/* an enhanced version of DM_ACL_OPENED_IND (see original).
   this replaces the old ev and contains the HCI st and is
   sent on conn fail as well as succ.
   enbd by ENHANCEMENT_ACL_INDI */
typedef struct
{
    U16 type;           /* always DM_ACL_OPENED_IND */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* peer dev addr */

    /* flag indicating peer - initiated ACL (TRUE) or locally - initiated (FALSE) */
    BOOL incoming;
    /* cls of dev for peer, vld for incoming conns only */
    U24  dev_cls;

    U8 st; /* HCI st */
} BTS2S_DM_EN_ACL_OPENED_IND;

/* an enhanced version of DM_ACL_CLOSED_IND (see original).
   this replaces the original ev and contains the HCI reason for
   the ACL being closed.
   enbd by ENHANCEMENT_ACL_INDI */
typedef struct
{
    U16 type;           /* always DM_ACL_CLOSED_IND */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /* peer dev addr */
    U8 reason;          /* HCI reason code */
} BTS2S_DM_EN_ACL_CLOSED_IND;

#endif

/* these msgs will map directly onto HCI msgs */
typedef BTS2S_HCI_SET_EV_MASK DMHCI_SET_EV_MASK;

/* noti of set ev mask */
typedef struct
{
    U16 type;           /* always BTS2S_DM_HCI_SET_EV_MASK_COMP */
    U16 phdl;           /* dest phdl */
    U8  st;             /* st */
} BTS2S_DM_HCI_SET_EV_MASK_COMP;

typedef BTS2S_HCI_RESET DMHCI_RESET;
/* noti of reset cmd */

/* always BTS2S_DM_HCI_RESET_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_RESET_COMP;

typedef BTS2S_HCI_SET_EV_FILTER DMHCI_SET_EV_FILTER;

/* req for flush */
typedef BTS2S_HCI_FLUSH DMHCI_FLUSH;

/*always DM_HCI_SET_EV_FILTER_COMP_T */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_SET_EV_FILTER_COMP;

/* noti of flush comp */
typedef struct
{
    U16 type;       /* always BTS2S_DM_HCI_FLUSH_COMP */
    U16 phdl;       /* dest phdl */
    U8 st;          /* st */
    BTS2S_BD_ADDR bd; /* bd of link */

} BTS2S_DM_HCI_FLUSH_COMP;

/* noti of set ev filter cmd */

typedef BTS2S_HCI_RD_PIN_TYPE DMHCI_RD_PIN_TYPE;

/* rd pin type comp ev */
typedef struct
{
    U16 type;       /* always BTS2S_DM_HCI_RD_PIN_TYPE_COMP */
    U16 phdl;       /* dest phdl */
    U8 st;          /* st */
    U8 pin_type;    /* the pin type */

} BTS2S_DM_HCI_RD_PIN_TYPE_COMP;

typedef BTS2S_HCI_WR_PIN_TYPE DMHCI_WR_PIN_TYPE;

/* wr pin type comp ev */

/* always BTS2S_DM_HCI_WR_PIN_TYPE_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_PIN_TYPE_COMP;

typedef BTS2S_HCI_CREATE_NEW_UNIT_KEY DMHCI_CREATE_NEW_UNIT_KEY;

/* create new unit key comp ev */

/* always BTS2S_DM_HCI_CREATE_NEW_UNIT_KEY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_CREATE_NEW_UNIT_KEY_COMP;

typedef BTS2S_HCI_RD_STORED_LINK_KEY DMHCI_RD_STORED_LINK_KEY;

/* rd stored link key comp ev */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_RD_STORED_LINK_KEY_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 max_key_num;
    U16 key_num_rd;

} BTS2S_DM_HCI_RD_STORED_LINK_KEY_COMP;

typedef BTS2S_HCI_WR_STORED_LINK_KEY DMHCI_WR_STORED_LINK_KEY;

/* wr stored link key comp ev */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_WR_STORED_LINK_KEY_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U8 key_num_written;

} BTS2S_DM_HCI_WR_STORED_LINK_KEY_COMP;

typedef BTS2S_HCI_DEL_STORED_LINK_KEY DMHCI_DEL_STORED_LINK_KEY;

/* del stored link keys comp ev */
typedef struct
{
    U16 type;   /* always DM_HCI_DEL_STORED_LINK_KEY_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 key_num_deld;

} BTS2S_DM_HCI_DEL_STORED_LINK_KEY_COMP;

typedef BTS2S_HCI_CHANGE_LOCAL_NAME_ext DMHCI_CHANGE_LOCAL_NAME;

/* change local name comp ev */

/* always DM_HCI_CHANGE_LOCAL_NAME_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_CHANGE_LOCAL_NAME_COMP;

typedef BTS2S_HCI_RD_CONN_ACPT_TIMEOUT BTS2S_DM_HCI_RD_CONN_ACPT_TO;

/* rd local name */

typedef BTS2S_HCI_RD_LOCAL_NAME DMHCI_RD_LOCAL_NAME;

typedef struct
{
    U16 type;   /* always DM_HCI_RD_LOCAL_NAME_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    /* see BTS2S_HCI_RD_LOCAL_NAME_RET */
    U8 *name_part[HCI_LOCAL_NAME_BYTE_PKT_PTRS];
} BTS2S_DM_HCI_RD_LOCAL_NAME_COMP;

typedef struct
{
    U16 type;   /* always DM_HCI_RD_LOCAL_NAME_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    /* see BTS2S_HCI_RD_LOCAL_NAME_RET */
    U8 name_part[HCI_NAME_LEN];
} BTS2S_DM_HCI_RD_LOCAL_NAME_COMP_ext;


typedef BTS2S_HCI_WR_CONN_ACPT_TIMEOUT BTS2S_DM_HCI_WR_CONN_ACPT_TO;

/* rd conn acpt timeout comp ev */
typedef struct
{
    U16 type;   /* always DM_HCI_RD_CONN_ACPT_TO_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 conn_acc_timeout; /* the timeout val */

} BTS2S_DM_HCI_RD_CONN_ACPT_TO_COMP;

/* wr conn acpt timeout comp ev */

/* always DM_HCI_WR_CONN_ACPT_TO_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_CONN_ACPT_TO_COMP;

typedef BTS2S_HCI_RD_PAGE_TIMEOUT BTS2S_DM_HCI_RD_PAGE_TO;

/* rd page timeout comp ev */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_RD_PAGE_TO_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 page_timeout; /* the timeout val */

} BTS2S_DM_HCI_RD_PAGE_TO_COMP;

typedef BTS2S_HCI_WR_PAGE_TIMEOUT BTS2S_DM_HCI_WR_PAGE_TO;

/* wr page timeout comp ev */

/* always DM_HCI_WR_PAGE_TO_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_PAGE_TO_COMP;

typedef BTS2S_HCI_RD_SCAN_ENB DMHCI_RD_SCAN_ENB;

/* rd scan enb comp ev */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_RD_SCAN_ENB_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U8 scan_enb; /* return par */

} BTS2S_DM_HCI_RD_SCAN_ENB_COMP;

typedef BTS2S_HCI_WR_SCAN_ENB DMHCI_WR_SCAN_ENB;

/* wr scan enb comp ev */

/* always BTS2S_DM_HCI_RD_SCAN_ENB_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_SCAN_ENB_COMP;

typedef BTS2S_HCI_RD_PAGESCAN_ACTIVITY DMHCI_RD_PAGESCAN_ACTIVITY;

/* rd pagescan activity comp */
typedef struct
{
    U16 type;       /* always BTS2S_DM_HCI_RD_PAGESCAN_ACTIVITY_COMP */
    U16 phdl;       /* dest phdl */
    U8  st;         /* st */
    U16 pagescan_intvl;
    U16 pagescan_window;

} BTS2S_DM_HCI_RD_PAGESCAN_ACTIVITY_COMP;

typedef BTS2S_HCI_WR_PAGESCAN_ACTIVITY DMHCI_WR_PAGESCAN_ACTIVITY;

/* wr pagescan activity comp */

/* always BTS2S_DM_HCI_WR_PAGESCAN_ACTIVITY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_PAGESCAN_ACTIVITY_COMP;

typedef BTS2S_HCI_RD_INQUIRYSCAN_ACTIVITY DMHCI_RD_INQUIRYSCAN_ACTIVITY;

/* rd inquiryscan activity comp */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 inqscan_intvl;
    U16 inqscan_window;
} BTS2S_DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP;

typedef BTS2S_HCI_WR_INQUIRYSCAN_ACTIVITY DMHCI_WR_INQUIRYSCAN_ACTIVITY;

/* wr inquiryscan activity comp */

/* always BTS2S_DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP;

typedef BTS2S_HCI_RD_AUTH_ENB DMHCI_RD_AUTH_ENB;

/* rd auth enb comp */
typedef struct
{
    U16 type;   /* always DM_HCI_RD_AUTH_ENB_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U8 auth_enb;
} BTS2S_DM_HCI_RD_AUTH_ENB_COMP;

typedef BTS2S_HCI_RD_ENC_MODE BTS2S_DM_HCI_RD_ENCRYPTION_MODE;

/* rd encryption mode comp */
typedef struct
{
    U16 type;   /* always DM_HCI_RD_ENCRYPTION_MODE_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U8 mode;
} BTS2S_DM_HCI_RD_ENCRYPTION_MODE_COMP;

typedef BTS2S_HCI_RD_DEV_CLS DMHCI_RD_DEV_CLS;

/* rd cls of dev comp */
typedef struct
{
    U16 type;   /* always BTS2S_DM_HCI_RD_DEV_CLS_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U32 dev_cls;
} BTS2S_DM_HCI_RD_DEV_CLS_COMP;

typedef BTS2S_HCI_WR_DEV_CLS DMHCI_WR_DEV_CLS;

/* wr cls of dev comp */

/* always BTS2S_DM_HCI_WR_DEV_CLS_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_DEV_CLS_COMP;

typedef BTS2S_HCI_RD_VOICE_SETTING DMHCI_RD_VOICE_SETTING;

/* rd voice setting comp */
typedef struct
{
    U16 type;   /* always DM_HCI_RD_VOICE_SETTING_COMP */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    U16 voice_setting;
} BTS2S_DM_HCI_RD_VOICE_SETTING_COMP;

typedef BTS2S_HCI_WR_VOICE_SETTING DMHCI_WR_VOICE_SETTING;

/* wr voice setting comp */

/* always DM_HCI_WR_VOICE_SETTING_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_VOICE_SETTING_COMP;

typedef BTS2S_HCI_RD_NUM_BCAST_RETXS BTS2S_DM_HCI_RD_NUM_BCAST_TXS;

/* req for rding the automatic flush timeout */
typedef BTS2S_HCI_RD_AUTO_FLUSH_TIMEOUT DMHCI_RD_AUTO_FLUSH_TIMEOUT;

/* rd auto flush timeout comp */
typedef struct
{
    U16 type;           /* always DM_HCI_RD_AUTO_FLUSH_COMP */
    U16 phdl;           /* dest phdl */
    BTS2S_BD_ADDR bd;   /*bd of link (allways ACL) */
    U8            st;   /*st */
    U16           flush_timeout; /*11 bts2g_bits of this num *0.625ms */

} BTS2S_DM_HCI_RD_AUTO_FLUSH_COMP;

/* this definition is provided for naming consistency */
typedef BTS2S_DM_HCI_RD_AUTO_FLUSH_COMP BTS2S_DM_HCI_RD_AUTO_FLUSH_TIMEOUT_COMP;

/* req for writing the automatic flush timeout */
typedef BTS2S_HCI_WR_AUTO_FLUSH_TIMEOUT DMHCI_WR_AUTO_FLUSH_TIMEOUT;

/* wr auto flush timeout comp */
typedef struct
{
    U16 type;   /* always DM_WR_AUTO_FLUSH_TIMEOUT */
    U16 phdl;   /* dest phdl */
    U8 st;      /* st */
    BTS2S_BD_ADDR bd; /* bd of link (allways ACL) */
} BTS2S_DM_HCI_WR_AUTO_FLUSH_COMP;

/* this definition is provided for naming consistency */
typedef BTS2S_DM_HCI_WR_AUTO_FLUSH_COMP BTS2S_DM_HCI_WR_AUTO_FLUSH_TIMEOUT_COMP;

/* req for rding SCO flow ctrl settings */
typedef BTS2S_HCI_RD_SCO_FLOW_CON_ENB BTS2S_DM_HCI_RD_SCO_FLOW_CTRL_ENB;

/* wr auto flush timeout comp */
typedef struct
{
    U16 type;       /* always BTS2S_DM_HCI_RD_NUM_BCAST_TXS_COMP */
    U16 phdl;       /* dest phdl */
    U8 st;          /* st */
    U8 flow_ctrl_enbd;
} BTS2S_DM_HCI_RD_SCO_FLOW_CTRL_COMP;

/* req for writing SCO flow ctrl settings */
typedef BTS2S_HCI_WR_SCO_FLOW_CON_ENB BTS2S_DM_HCI_WR_SCO_FLOW_CTRL_ENB;

/* wr SCO flow ctrl comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_WR_SCO_FLOW_CTRL_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
} BTS2S_DM_HCI_WR_SCO_FLOW_CTRL_COMP;

/* rd num of broadcast transmissions comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_NUM_BCAST_TXS_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 num;
} BTS2S_DM_HCI_RD_NUM_BCAST_TXS_COMP;

typedef BTS2S_HCI_WR_NUM_BCAST_RETXS BTS2S_DM_HCI_WR_NUM_BCAST_TXS;

/* wr num of broadcast transmissions comp */

/* always BTS2S_DM_HCI_WR_NUM_BCAST_TXS_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_NUM_BCAST_TXS_COMP;

typedef BTS2S_HCI_RD_HOLD_MODE_ACTIVITY DMHCI_RD_HOLD_MODE_ACTIVITY;

/* wr num of broadcast transmissions comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 activity;
} BTS2S_DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP;

typedef BTS2S_HCI_WR_HOLD_MODE_ACTIVITY DMHCI_WR_HOLD_MODE_ACTIVITY;

/* wr hold mode activity comp */

/* always BTS2S_DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP;

/* rd transmit power level - use the bd to reference the link */
typedef BTS2S_HCI_RD_TX_POWER_LEVEL DMHCI_RD_TX_POWER_LEVEL;

/* rd transmit power level comp - use the bd to reference the link */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_TX_POWER_LEVEL_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    BTS2S_BD_ADDR bd; /* bd of the link */
    U8 pwr_level;
} BTS2S_DM_HCI_RD_TX_POWER_LEVEL_COMP;

typedef BTS2S_HCI_SET_HCTOHOST_FLOW_CTRL BTS2S_DM_HCI_SET_HC_TO_HOST_FLOW;

/* set hc to host flow comp */

/* DM_HCI_SET_HC_TO_HOST_FLOW_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_SET_FC_HC_TO_HOST_COMP;
/* required by parsemsg so that msg name can be matched to msg structure! */
typedef BTS2S_DM_HCI_SET_FC_HC_TO_HOST_COMP BTS2S_DM_HCI_SET_HC_TO_HOST_FLOW_COMP;

typedef BTS2S_HCI_HOST_NUM_COMPD_PKTS DMHCI_HOST_NUM_COMPD_PKTS;

/* num compd pkt comp
   only used if there is an err */

/* always HCI_HOST_NUM_COMP_PKT_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_HCI_HOST_NUM_COMP_PKT_COMP;

/* rd link supvisn timeout */

typedef BTS2S_HCI_RD_LINK_SUPERV_TIMEOUT DMHCI_RD_LINK_SUPERV_TIMEOUT;

/* rd link supvisn timeout comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    BTS2S_BD_ADDR bd;
    U16 timeout;
} BTS2S_DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP;

/* wr link supvisn timeout */

typedef BTS2S_HCI_WR_LINK_SUPERV_TIMEOUT DMHCI_WR_LINK_SUPERV_TIMEOUT;

/* wr link supvisn timeout comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP;

typedef BTS2S_HCI_RD_NUM_SUPP_IAC DMHCI_RD_NUM_SUPP_IAC;

/* rd num suppport iac comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_NUM_SUPP_IAC_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 num;
} BTS2S_DM_HCI_RD_NUM_SUPP_IAC_COMP;

typedef BTS2S_HCI_RD_CUR_IAC_LAP DMHCI_RD_CUR_IAC_LAP;

/* rd cur iac lap comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_CUR_IAC_LAP_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 num_cur_iac;

    /* array of HCI_IAC_LAP_PTRS pointers to HCI_IAC_LAP_PER_PTR iac */
    U24 *iac_lap[HCI_IAC_LAP_PTRS];
} BTS2S_DM_HCI_RD_CUR_IAC_LAP_COMP;

typedef BTS2S_HCI_WR_CUR_IAC_LAP DMHCI_WR_CUR_IAC_LAP;

/* wr cur iac lap comp */

/*always BTS2S_DM_HCI_WR_CUR_IAC_LAP_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_CUR_IAC_LAP_COMP;

typedef BTS2S_HCI_RD_PAGESCAN_PERIOD_MODE DMHCI_RD_PAGESCAN_PERIOD_MODE;

/* rd pagescan period mode comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 mode;
} BTS2S_DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP;

typedef BTS2S_HCI_WR_PAGESCAN_PERIOD_MODE DMHCI_WR_PAGESCAN_PERIOD_MODE;

/* wr pagescan period mode comp */

/* always BTS2S_DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP;

typedef BTS2S_HCI_RD_PAGESCAN_MODE DMHCI_RD_PAGESCAN_MODE;

/* rd pagescan mode comp */
typedef struct
{
    U16 type; /* always BTS2S_DM_HCI_RD_PAGESCAN_MODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 mode;
} BTS2S_DM_HCI_RD_PAGESCAN_MODE_COMP;

typedef BTS2S_HCI_WR_PAGESCAN_MODE DMHCI_WR_PAGESCAN_MODE;

/* wr pagescan mode comp */

/* always BTS2S_DM_HCI_WR_PAGESCAN_MODE_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_PAGESCAN_MODE_COMP;

/* noti of hardware err */
typedef struct
{
    U16 type; /* always DM_HCI_HARDWARE_ERR */
    U16 phdl; /* dest phdl */
    U8 hw_err;
} BTS2S_DM_HCI_HARDWARE_ERR;

/* noti of data buff overflow */
typedef struct
{
    U16 type; /* always DM_HCI_DATA_BUFF_OVERFLOW */
    U16 phdl; /* dest phdl */
} BTS2S_DM_HCI_DATA_BUFF_OVERFLOW;

/* max slots change ev */
typedef struct
{
    U16 type; /* always DM_HCI_MAX_SLOTS_CHANGE */
    U16 phdl; /* dest phdl */
    U8 lmp_max_slot;
} BTS2S_DM_HCI_MAX_SLOTS_CHANGE;

typedef BTS2S_HCI_RD_LOCAL_VER_INFO BTS2S_DM_HCI_RD_LOCAL_VERSION;

/* rd local version comp */
typedef struct
{
    U16 type; /* always DM_HCI_RD_LOCAL_VERSION_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 hci_version;
    U16 hci_revision;
    U8 lmp_version; /* defined in LMP */
    U16 manuf_name; /* defined in LMP */
    U16 lmp_subversion;
} BTS2S_DM_HCI_RD_LOCAL_VERSION_COMP;

typedef BTS2S_HCI_RD_LOCAL_SUPP_FEATR BTS2S_DM_HCI_RD_LOCAL_FEATR;

/* rd local featr comp */
typedef struct
{
    U16 type; /* always DM_HCI_RD_LOCAL_FEATR_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 lmp_supp_featr[8]; /* bit masks */
} BTS2S_DM_HCI_RD_LOCAL_FEATR_COMP;

typedef BTS2S_HCI_RD_COUNTRY_CODE DMHCI_RD_COUNTRY_CODE;

/* rd country code comp */
typedef struct
{
    U16 type; /* always DM_HCI_RD_COUNTRY_CODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* st */
    U8 country_code;
} BTS2S_DM_HCI_RD_COUNTRY_CODE_COMP;

typedef HCI_RDBD_ADDR DM_HCI_RDBD_ADDR;

/* rd bd return msg */
typedef struct
{
    U16 type;
    U16 phdl;   /* dest phdl */
    U8 st;      /* succ or fail */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_HCI_RD_BD_ADDR_COMP;

/* rd failed contact counter */
typedef BTS2S_HCI_RD_FAILED_CONTACT_COUNT BTS2S_DM_HCI_FAILED_CONTACT_COUNTER;

/* rd failed contact counter comp */
typedef struct
{
    U16 type; /* always DM_HCI_FAILED_CONTACT_COUNTER_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    U16 failed_contact_count;
} BTS2S_DM_HCI_FAILED_CONTACT_COUNTER_COMP;

/* reset contact counter req */
typedef BTS2S_HCI_RESET_FAILED_CONTACT_COUNT BTS2S_DM_HCI_RESET_CONTACT_COUNTER;

/* reset contact counter comp */
typedef struct
{
    U16 type; /* always DM_HCI_RESET_CONTACT_COUNTER_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_HCI_RESET_CONTACT_COUNTER_COMP;

/* get link qa */
typedef BTS2S_HCI_GET_LINK_QA DMHCI_GET_LINK_QA;

/* get link qa comp */
typedef struct
{
    U16 type; /* always DM_HCI_GET_LINK_QA_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    U8 link_qa;
} BTS2S_DM_HCI_GET_LINK_QA_COMP;

/* rd RSSI */
typedef BTS2S_HCI_RD_RSSI DMHCI_RD_RSSI;

/* rd RSSI comp */
typedef struct
{
    U16 type; /* always DM_HCI_RD_RSSI_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    S8 rssi;
} BTS2S_DM_HCI_RD_RSSI_COMP;

/* these msgs will map directly onto HCI msgs */
typedef BTS2S_HCI_RD_LOOPBACK_MODE DMHCI_RD_LOOPBACK_MODE;

/* rd loopback mode comp */
typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_LOOPBACK_MODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 mode;
} BTS2S_DM_HCI_RD_LOOPBACK_MODE_COMP;

typedef BTS2S_HCI_WR_LOOPBACK_MODE DMHCI_WR_LOOPBACK_MODE;

/* wr loopback mode comp */

/* BTS2S_DM_HCI_WR_LOOPBACK_MODE_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_WR_LOOPBACK_MODE_COMP;

/* loopback cmds ev */
typedef struct
{
    U16 type; /* BTS2S_DM_HCI_LOOPBACK_EV */
    U16 phdl; /* dest phdl */
    BTS2S_HCI_EV_LOOPBACK_BYTE_STRUCT *loopback_part_ptr[HCI_LOOPBACK_BYTE_PKT_PTRS];
} BTS2S_DM_HCI_LOOPBACK_EV;

typedef BTS2S_HCI_ENB_DUT_MODE BTS2S_DM_HCI_ENB_DEV_UT_MODE;

/* enb dev under test comp */

/* DM_HCI_ENB_DEV_UT_MODE_COMP */
typedef BTS2S_DM_HCI_STANDARD_CMD_COMP BTS2S_DM_HCI_ENB_DEV_UT_MODE_COMP;

/* common elems of an "umsg" msg */
typedef struct
{
    U16 type;
    U16 phdl;
} BTS2S_DM_UP_MSG_COMMON;

/* UNION OF       MSGITIVES */

typedef union
{
    U16 type;
    BTS2S_DM_UP_MSG_COMMON dm_umsg_common;
    BTS2S_DM_AM_REG_REQ dm_am_reg_req;
    BTS2S_DM_AM_REG_CFM dm_am_reg_cfm;
    BTS2S_DM_HC_TO_HOST_FLOW_CTRL dm_hc_to_host_fc;
    BTS2S_DM_ACL_REG_REQ dm_acl_reg_req;
    BTS2S_DM_ACL_REG_CFM dm_acl_reg_cfm;
    BTS2S_DM_ACL_CONN_REQ dm_acl_conn_req;
    BTS2S_DM_ACL_CONN_CFM dm_acl_conn_cfm;
    BTS2S_DM_ACL_CONN_IND dm_acl_conn_ind;
    BTS2S_DM_ACL_DISC_REQ dm_acl_disc_req;
    BTS2S_DM_ACL_CANCEL_CONN_REQ dm_acl_cancel_conn_req;
    BTS2S_DM_ACL_DISC_IND dm_acl_disc_ind;
    BTS2S_DM_ACL_BUFF_SIZE_IND dm_acl_buff_size_ind;
    BTS2S_DM_ACL_DATA_SENT_IND_ext dm_acl_data_sent_ind;
    BTS2S_DM_CONNLESS_CH_REG_REQ dm_connless_ch_reg_req;
    BTS2S_DM_HCI_RMT_NAME_COMP_ext dm_rmt_name_comp;

    BTS2S_DM_HCI_INQUIRY_RES_ext dm_inquiry_res;
    BTS2S_DM_HCI_INQUIRY_RES_WITH_RSSI dm_inquiry_res_with_rssi;
    BTS2S_DM_HCI_INQUIRY_COMP dm_inquiry_comp;
    BTS2S_DM_HCI_INQUIRY_ESC_COMP dm_inquiry_esc_comp;
    BTS2S_DM_HCI_PERIODIC_INQUIRY_COMP dm_periodic_inquiry_comp;
    BTS2S_DM_HCI_EXIT_PERIODIC_INQUIRY_COMP dm_exit_periodic_inquiry_comp;
    BTS2S_DM_HCI_CHANGE_LINK_KEY dm_change_link_key;
    BTS2S_DM_HCI_LINK_KEY_CHANGE_COMP dm_link_key_change_comp;
    BTS2S_DM_HCI_RETURN_LINK_KEYS dm_return_link_keys;
    BTS2S_DM_HCI_MASTER_LINK_KEY_COMP dm_master_link_key_comp;
    BTS2S_DM_HCI_RMT_NAME_REQ dm_rmt_name_req;
    BTS2S_DM_HCI_RD_RMT_FEATR dm_rd_rmt_featr;
    BTS2S_DM_HCI_RD_RMT_FEATR_COMP dm_rd_rmt_featr_comp;
    BTS2S_DM_HCI_RD_RMT_VERSION dm_rd_rmt_version;
    BTS2S_DM_HCI_RD_RMT_VERSION_COMP dm_rd_rmt_version_comp;
    DMHCI_RD_CLOCK_OFFSET dm_rd_clock_offset;
    BTS2S_DM_HCI_RD_CLOCK_OFFSET_COMP dm_rd_clock_offset_comp;

    BTS2S_DM_HCI_WR_LP_SETTINGS dm_wr_lp_settings;
    BTS2S_DM_HCI_WR_LP_SETTINGS_COMP dm_wr_lp_settings_comp;
    BTS2S_DM_HCI_RD_LP_SETTING dm_rd_lp_settings;
    BTS2S_DM_HCI_RD_LP_SETTINGS_COMP dm_rd_lp_settings_comp;
    DMHCI_ROLE_DISCOV  dm_role_discov;
    BTS2S_DM_HCI_ROLE_DISCOV_COMP dm_role_discov_comp;
    BTS2S_DM_HCI_SWITCH_ROLE_COMP dm_switch_role_comp;
    BTS2S_DM_HCI_MODE_CHANGE_EV dm_mode_change_ev;
    BTS2S_DM_HCI_SET_EV_MASK_COMP dm_set_ev_mask_comp;
    BTS2S_DM_HCI_RESET_COMP dm_reset_comp;
    BTS2S_DM_HCI_SET_EV_FILTER_COMP dm_set_ev_filter_comp;
    DMHCI_FLUSH  dm_flush;
    BTS2S_DM_HCI_FLUSH_COMP dm_flush_comp;
    BTS2S_DM_HCI_RD_PIN_TYPE_COMP dm_rd_pin_type_comp;
    BTS2S_DM_HCI_WR_PIN_TYPE_COMP dm_wr_pin_type_comp;
    BTS2S_DM_HCI_CREATE_NEW_UNIT_KEY_COMP dm_create_new_unit_key_comp;
    BTS2S_DM_HCI_RD_STORED_LINK_KEY_COMP dm_rd_stored_link_key_comp;
    BTS2S_DM_HCI_WR_STORED_LINK_KEY_COMP dm_wr_stored_link_key_comp;
    BTS2S_DM_HCI_DEL_STORED_LINK_KEY_COMP dm_del_stored_link_key_comp;
    BTS2S_DM_HCI_CHANGE_LOCAL_NAME_COMP dm_change_local_name_comp;
    BTS2S_DM_HCI_RD_LOCAL_NAME_COMP_ext dm_rd_local_name_comp;
    BTS2S_DM_HCI_RD_CONN_ACPT_TO_COMP dm_rd_conn_U8o_comp;
    BTS2S_DM_HCI_WR_CONN_ACPT_TO_COMP dm_wr_conn_U8o_comp;
    BTS2S_DM_HCI_RD_PAGE_TO_COMP dm_rd_page_to_comp;
    BTS2S_DM_HCI_WR_PAGE_TO_COMP dm_wr_page_to_comp;
    BTS2S_DM_HCI_RD_SCAN_ENB_COMP dm_rd_scan_enb_comp;
    BTS2S_DM_HCI_WR_SCAN_ENB_COMP dm_wr_scan_enb_comp;
    BTS2S_DM_HCI_RD_PAGESCAN_ACTIVITY_COMP dm_rd_pagescan_activity_comp;
    BTS2S_DM_HCI_WR_PAGESCAN_ACTIVITY_COMP dm_wr_pagescan_activity_comp;
    BTS2S_DM_HCI_RD_INQUIRYSCAN_ACTIVITY_COMP dm_rd_inquiryscan_activity_comp;
    BTS2S_DM_HCI_WR_INQUIRYSCAN_ACTIVITY_COMP dm_wr_inquiryscan_activity_comp;
    BTS2S_DM_HCI_RD_AUTH_ENB_COMP dm_rd_auth_enb_comp;
    BTS2S_DM_HCI_RD_ENCRYPTION_MODE_COMP dm_rd_encryption_mode_comp;
    BTS2S_DM_HCI_RD_DEV_CLS_COMP dm_rd_dev_cls_comp;
    BTS2S_DM_HCI_WR_DEV_CLS_COMP dm_wr_dev_cls_comp;
    BTS2S_DM_HCI_RD_VOICE_SETTING_COMP dm_rd_voice_setting_comp;
    BTS2S_DM_HCI_WR_VOICE_SETTING_COMP dm_wr_voice_setting_comp;
    DMHCI_RD_AUTO_FLUSH_TIMEOUT dm_rd_auto_flush_timeout;
    BTS2S_DM_HCI_RD_AUTO_FLUSH_COMP dm_rd_auto_flush_timeout_comp;
    DMHCI_WR_AUTO_FLUSH_TIMEOUT dm_wr_auto_flush_timeout;
    BTS2S_DM_HCI_WR_AUTO_FLUSH_COMP dm_wr_auto_flush_timeout_comp;
    BTS2S_DM_HCI_RD_SCO_FLOW_CTRL_COMP dm_rd_sco_flow_ctrl_comp;
    BTS2S_DM_HCI_WR_SCO_FLOW_CTRL_COMP dm_wr_sco_flow_ctrl_comp;
    BTS2S_DM_HCI_RD_NUM_BCAST_TXS_COMP dm_rd_num_bcast_txs_comp;
    BTS2S_DM_HCI_WR_NUM_BCAST_TXS_COMP dm_wr_num_bcast_txs_comp;
    BTS2S_DM_HCI_RD_HOLD_MODE_ACTIVITY_COMP dm_rd_hold_mode_activity_comp;
    BTS2S_DM_HCI_WR_HOLD_MODE_ACTIVITY_COMP dm_wr_hold_mode_activity_comp;
    DMHCI_RD_TX_POWER_LEVEL dm_rd_tx_power_level;
    BTS2S_DM_HCI_RD_TX_POWER_LEVEL_COMP dm_rd_tx_power_level_comp;
    DMHCI_HOST_NUM_COMPD_PKTS dm_host_num_compd_pkt;
    BTS2S_HCI_HOST_NUM_COMP_PKT_COMP dm_host_num_compd_pkt_comp;
    DMHCI_RD_LINK_SUPERV_TIMEOUT dm_rd_link_superv_timeout;
    BTS2S_DM_HCI_RD_LINK_SUPERV_TIMEOUT_COMP dm_rd_link_superv_timeout_comp;
    DMHCI_WR_LINK_SUPERV_TIMEOUT dm_wr_link_superv_timeout;
    BTS2S_DM_HCI_WR_LINK_SUPERV_TIMEOUT_COMP dm_wr_link_superv_timeout_comp;
    BTS2S_DM_HCI_RD_NUM_SUPP_IAC_COMP dm_rd_num_supp_iac_comp;
    BTS2S_DM_HCI_RD_CUR_IAC_LAP_COMP dm_rd_cur_iac_lap_comp;
    BTS2S_DM_HCI_WR_CUR_IAC_LAP_COMP dm_wr_cur_iac_lap_comp;
    BTS2S_DM_HCI_RD_PAGESCAN_PERIOD_MODE_COMP dm_rd_pagescan_period_mode;
    BTS2S_DM_HCI_WR_PAGESCAN_PERIOD_MODE_COMP dm_wr_pagescan_period_mode;
    BTS2S_DM_HCI_RD_PAGESCAN_MODE_COMP dm_rd_pagescan_mode;
    BTS2S_DM_HCI_WR_PAGESCAN_MODE_COMP dm_wr_pagescan_mode;
    BTS2S_DM_HCI_HARDWARE_ERR  dm_hardware_err;
    BTS2S_DM_HCI_DATA_BUFF_OVERFLOW  dm_data_buff_overflow;
    BTS2S_DM_HCI_MAX_SLOTS_CHANGE dm_max_slots_change;

    BTS2S_DM_HCI_RD_LOCAL_VERSION_COMP dm_rd_local_version_comp;
    BTS2S_DM_HCI_RD_LOCAL_FEATR_COMP dm_rd_local_featr_comp;
    BTS2S_DM_HCI_RD_COUNTRY_CODE_COMP dm_rd_country_code_comp;
    BTS2S_DM_HCI_RD_BD_ADDR_COMP dm_rd_bd_comp;

    BTS2S_DM_HCI_QOS_SETUP_REQ dm_qos_setup;
    BTS2S_DM_HCI_QOS_SETUP_CFM dm_qos_setup_comp;
    BTS2S_DM_HCI_QOS_VIOLATION_IND dm_qos_violation_indi;
    BTS2S_DM_HCI_FAILED_CONTACT_COUNTER dm_failed_contact_counter;
    BTS2S_DM_HCI_FAILED_CONTACT_COUNTER_COMP dm_failed_contact_counter_comp;
    BTS2S_DM_HCI_RESET_CONTACT_COUNTER dm_reset_contact_counter;
    BTS2S_DM_HCI_RESET_CONTACT_COUNTER_COMP dm_reset_contact_counter_compete;
    DMHCI_GET_LINK_QA dm_get_link_qa;
    BTS2S_DM_HCI_GET_LINK_QA_COMP dm_get_link_qa_comp;
    DMHCI_RD_RSSI dm_rd_rssi;
    BTS2S_DM_HCI_RD_RSSI_COMP dm_rd_rssi_comp;


    BTS2S_DM_HCI_RD_LOOPBACK_MODE_COMP dm_rd_loopback_mode_comp;
    BTS2S_DM_HCI_WR_LOOPBACK_MODE_COMP dm_wr_loopback_mode_comp;
    BTS2S_DM_HCI_ENB_DEV_UT_MODE_COMP dm_enb_dev_ut_mode_comp;
    BTS2S_DM_HCI_LOOPBACK_EV dm_loopback_ev;
    BTS2S_DM_HCI_SET_FC_HC_TO_HOST_COMP dm_flow_ctrl_hc_to_host_comp;

    BTS2S_DM_WR_CACHED_PAGE_MODE_REQ dm_wr_cached_page_mode_req;
    BTS2S_DM_WR_CACHED_PAGE_MODE_CFM dm_wr_cached_page_mode_cfm;
    BTS2S_DM_WR_CACHED_CLOCK_OFFSET_REQ dm_wr_cached_clock_offset_req;
    BTS2S_DM_WR_CACHED_CLOCK_OFFSET_CFM dm_wr_cached_clock_offset_cfm;
    BTS2S_DM_CLEAR_PARAM_CACHE_REQ dm_clear_param_cache_req;
    BTS2S_DM_CLEAR_PARAM_CACHE_CFM dm_clear_param_cache_cfm;
    BTS2S_DM_SET_DEFAULT_LINK_POLICY dm_set_default_link_policy;
    BTS2S_DM_ACL_OPEN_REQ dm_acl_open_req;
    BTS2S_DM_ACL_OPEN_CFM dm_acl_open_cfm;
    BTS2S_DM_ACL_CLOSE_REQ dm_acl_close_req;
    BTS2S_DM_ACL_OPENED_IND dm_acl_opened_ind;
    BTS2S_DM_ACL_CLOSED_IND dm_acl_closed_ind;

    BTS2S_DM_SM_SET_DEFAULT_SECU_REQ dm_sm_set_default_secu_req;
    BTS2S_DM_SM_REG_REQ dm_sm_reg_req;
    BTS2S_DM_SM_UNREG_REQ dm_sm_unreg_req;
    BTS2S_DM_SM_ACCESS_REQ dm_sm_access_req;
    BTS2S_DM_SM_ACCESS_CFM dm_sm_access_cfm;
    BTS2S_DM_SM_SET_SEC_MODE_REQ dm_sm_set_sec_mode_req;
    BTS2S_DM_SM_SET_SEC_MODE_CFM dm_sm_set_sec_mode_cfm;
    BTS2S_DM_SM_ADD_DEV_REQ dm_sm_add_dev_req;
    BTS2S_DM_SM_ADD_DEV_CFM dm_sm_add_dev_cfm;
    BTS2S_DM_SM_REMOVE_DEV_REQ dm_sm_remove_dev_req;
    BTS2S_DM_SM_REMOVE_DEV_CFM dm_sm_remove_dev_cfm;
    BTS2S_DM_SM_LINK_KEY_REQ_IND dm_sm_link_key_req_ind;
    BTS2S_DM_SM_LINK_KEY_REQ_RES dm_sm_link_key_req_res;
    BTS2S_DM_SM_PIN_REQ_IND dm_sm_pin_req_ind;
    BTS2S_DM_SM_PIN_REQ_RES dm_sm_pin_req_res;
    BTS2S_DM_SM_LINK_KEY_IND dm_sm_link_key_ind;
    BTS2S_DM_SM_AUTHORISE_IND dm_sm_authorise_ind;
    BTS2S_DM_SM_AUTHORISE_RES dm_sm_authorise_res;
    BTS2S_DM_SM_AUTH_REQ dm_sm_auth_req;
    BTS2S_DM_SM_AUTH_CFM dm_sm_auth_cfm;
    BTS2S_DM_SM_ENCRYPT_REQ dm_sm_encrypt_req;
    BTS2S_DM_SM_ENCRYPT_CFM dm_sm_encrypt_cfm;
    BTS2S_DM_SM_ENCRYPTION_CHANGE dm_sm_encryption_change;
    BTS2S_DM_SM_L2C_REG_REQ dm_sm_l2c_reg_req;
    BTS2S_DM_SM_L2C_CLRX_ENB_IND dm_sm_l2c_clrx_enb_ind;

    BTS2S_DM_SYNC_REG_REQ dm_sync_reg_req;
    BTS2S_DM_SYNC_REG_CFM dm_sync_reg_cfm;
    BTS2S_DM_SYNC_UNREG_REQ dm_sync_unreg_req;
    BTS2S_DM_SYNC_UNREG_CFM dm_sync_unreg_cfm;
    BTS2S_DM_SYNC_CONN_REQ dm_sync_conn_req;
    BTS2S_DM_SYNC_CONN_CFM dm_sync_conn_cfm;
    BTS2S_DM_SYNC_CONN_COMP_IND dm_sync_conn_comp_ind;
    BTS2S_DM_SYNC_CONN_IND dm_sync_conn_ind;
    BTS2S_DM_SYNC_CONN_RES dm_sync_conn_res;
    BTS2S_DM_SYNC_RENEGOTIATE_REQ dm_sync_renegotiate_req;
    BTS2S_DM_SYNC_RENEGOTIATE_IND dm_sync_renegotiate_ind;
    BTS2S_DM_SYNC_RENEGOTIATE_CFM dm_sync_renegotiate_cfm;
    BTS2S_DM_SYNC_DISC_REQ dm_sync_disc_req;
    BTS2S_DM_SYNC_DISC_IND dm_sync_disc_ind;
    BTS2S_DM_SYNC_DISC_CFM dm_sync_disc_cfm;
#ifdef CFG_OPEN_FULL_HCI_FUNC
    BTS2S_HCI_RD_STORED_LINK_KEY  dm_rd_lnk;
    BTS2S_HCI_WR_STORED_LINK_KEY  dm_wr_lnk;
    BTS2S_HCI_DEL_STORED_LINK_KEY  dm_del_lnk;
#endif
} BTS2U_HCI_MSG;

typedef BTS2S_HCI_CREATE_CONN_ESC DMHCI_CREATE_CONN_ESC;
#define DM_HCI_CREATE_CONN_ESC ((U16)(ENUM_DM_HCI_CREATE_CONN_ESC))
#define DM_HCI_CREATE_CONN_ESC_COMP ((U16)(ENUM_DM_HCI_CREATE_CONN_ESC_COMP))

typedef struct
{
    U16 type; /* DM_HCI_CREATE_CONN_COMP_T */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
} BTS2S_DM_HCI_CREATE_CONN_ESC_COMP;

typedef BTS2S_HCI_RMT_NAME_REQ_ESC DMHCI_RMT_NAME_REQ_ESC;
#define DM_HCI_RMT_NAME_REQ_ESC ((U16)(ENUM_DM_HCI_RMT_NAME_REQ_ESC))
#define DM_HCI_RMT_NAME_REQ_ESC_COMP ((U16)(ENUM_DM_HCI_RMT_NAME_REQ_ESC_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RMT_NAME_REQ_ESC_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;

} BTS2S_DM_HCI_RMT_NAME_REQ_ESC_COMP;


typedef BTS2S_HCI_RD_RMT_EXT_FEATR DMHCI_RD_RMT_EXT_FEATR;
#define DM_HCI_RD_RMT_EXT_FEATR ((U16)(ENUM_DM_HCI_RD_RMT_EXT_FEATR))
#define DM_HCI_RD_RMT_EXT_FEATR_COMP ((U16)(ENUM_DM_HCI_RD_RMT_EXT_FEATR_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_RMT_EXT_FEATR_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    U8 page_num;
    U8 max_page_num;
    U16 lmp_ext_featr[4];
} BTS2S_DM_HCI_RD_RMT_EXT_FEATR_COMP;


typedef BTS2S_HCI_RD_LMP_HANDLE DMHCI_RD_LMP_HANDLE;
#define DM_HCI_RD_LMP_HANDLE ((U16)(ENUM_DM_HCI_RD_LMP_HANDLE))
#define DM_HCI_RD_LMP_HANDLE_COMP ((U16)(ENUM_DM_HCI_RD_LMP_HANDLE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_LMP_HANDLE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 lmp_hdl;
    U32 rsv;
} BTS2S_DM_HCI_RD_LMP_HANDLE_COMP;


typedef BTS2S_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS DMHCI_RD_DEFAULT_LINK_POLICY_SETTINGS;
#define DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS ((U16)(ENUM_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS))
#define DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_COMP ((U16)(ENUM_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_COMP))

typedef struct
{
    U16  type; /* BTS2S_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_COMP */
    U16  phdl; /* dest phdl */
    U8   st; /* succ or fail */
    U16  default_lps;
} BTS2S_DM_HCI_RD_DEFAULT_LINK_POLICY_SETTINGS_COMP;


typedef BTS2S_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS DMHCI_WR_DEFAULT_LINK_POLICY_SETTINGS;
#define DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS ((U16)(ENUM_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS))
#define DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_COMP ((U16)(ENUM_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
} BTS2S_DM_HCI_WR_DEFAULT_LINK_POLICY_SETTINGS_COMP;


typedef BTS2S_HCI_FLOW_SPEC DMHCI_FLOW_SPEC;
#define DM_HCI_FLOW_SPEC ((U16)(ENUM_DM_HCI_FLOW_SPEC))
#define DM_HCI_FLOW_SPEC_COMP ((U16)(ENUM_DM_HCI_FLOW_SPEC_COMP))

#ifdef  CFG_BT_VER_21
typedef BTS2S_HCI_SNIFF_SUBRATING DMHCI_SNIFF_SUBRATING;
#define DM_HCI_SNIFF_SUBRATING ((U16)(ENUM_DM_HCI_SNIFF_SUBRATING))
#endif

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_FLOW_SPEC_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail - see note1 */
    BTS2S_BD_ADDR bd;
    U8 flag;
    U8 flow_direction; /* 0=out (to air), 1=in (from air) */
    U8 svc_type;
    U32 token_rate;
    U32 token_bucket_size;
    U32 peak_bandwidth;
    U32 access_latency;
} BTS2S_DM_HCI_FLOW_SPEC_COMP;


typedef BTS2S_HCI_SET_AFH_CHNL_CLS DMHCI_SET_AFH_CHNL_CLS;
#define DM_HCI_SET_AFH_CHNL_CLS ((U16)(ENUM_DM_HCI_SET_AFH_CHNL_CLS))
#define DM_HCI_SET_AFH_CHNL_CLS_COMP ((U16)(ENUM_DM_HCI_SET_AFH_CHNL_CLS_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_SET_AFH_CHNL_CLS_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
} BTS2S_DM_HCI_SET_AFH_CHNL_CLS_COMP;


typedef BTS2S_HCI_RD_INQUIRY_SCAN_TYPE DMHCI_RD_INQUIRY_SCAN_TYPE;
#define DM_HCI_RD_INQUIRY_SCAN_TYPE ((U16)(ENUM_DM_HCI_RD_INQUIRY_SCAN_TYPE))
#define DM_HCI_RD_INQUIRY_SCAN_TYPE_COMP ((U16)(ENUM_DM_HCI_RD_INQUIRY_SCAN_TYPE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_INQUIRY_SCAN_TYPE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 mode;
} BTS2S_DM_HCI_RD_INQUIRY_SCAN_TYPE_COMP;


typedef BTS2S_HCI_WR_INQUIRY_SCAN_TYPE DMHCI_WR_INQUIRY_SCAN_TYPE;
#define DM_HCI_WR_INQUIRY_SCAN_TYPE ((U16)(ENUM_DM_HCI_WR_INQUIRY_SCAN_TYPE))
#define DM_HCI_WR_INQUIRY_SCAN_TYPE_COMP ((U16)(ENUM_DM_HCI_WR_INQUIRY_SCAN_TYPE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_WR_INQUIRY_SCAN_TYPE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
} BTS2S_DM_HCI_WR_INQUIRY_SCAN_TYPE_COMP;


typedef BTS2S_HCI_RD_INQUIRY_MODE DMHCI_RD_INQUIRY_MODE;
#define DM_HCI_RD_INQUIRY_MODE ((U16)(ENUM_DM_HCI_RD_INQUIRY_MODE))
#define DM_HCI_RD_INQUIRY_MODE_COMP ((U16)(ENUM_DM_HCI_RD_INQUIRY_MODE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_INQUIRY_MODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 mode;
} BTS2S_DM_HCI_RD_INQUIRY_MODE_COMP;


typedef BTS2S_HCI_WR_INQUIRY_MODE DMHCI_WR_INQUIRY_MODE;
#define DM_HCI_WR_INQUIRY_MODE ((U16)(ENUM_DM_HCI_WR_INQUIRY_MODE))
#define DM_HCI_WR_INQUIRY_MODE_COMP ((U16)(ENUM_DM_HCI_WR_INQUIRY_MODE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_WR_INQUIRY_MODE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
} BTS2S_DM_HCI_WR_INQUIRY_MODE_COMP;


typedef BTS2S_HCI_RD_PAGE_SCAN_TYPE DMHCI_RD_PAGE_SCAN_TYPE;
#define DM_HCI_RD_PAGE_SCAN_TYPE ((U16)(ENUM_DM_HCI_RD_PAGE_SCAN_TYPE))
#define DM_HCI_RD_PAGE_SCAN_TYPE_COMP ((U16)(ENUM_DM_HCI_RD_PAGE_SCAN_TYPE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_PAGE_SCAN_TYPE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 mode;
} BTS2S_DM_HCI_RD_PAGE_SCAN_TYPE_COMP;


typedef BTS2S_HCI_WR_PAGE_SCAN_TYPE DMHCI_WR_PAGE_SCAN_TYPE;
#define DM_HCI_WR_PAGE_SCAN_TYPE ((U16)(ENUM_DM_HCI_WR_PAGE_SCAN_TYPE))
#define DM_HCI_WR_PAGE_SCAN_TYPE_COMP ((U16)(ENUM_DM_HCI_WR_PAGE_SCAN_TYPE_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_WR_PAGE_SCAN_TYPE_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */

} BTS2S_DM_HCI_WR_PAGE_SCAN_TYPE_COMP;


typedef BTS2S_HCI_RD_AFH_CHNL_CLS_M DMHCI_RD_AFH_CHNL_CLS_M;
#define DM_HCI_RD_AFH_CHNL_CLS_M ((U16)(ENUM_DM_HCI_RD_AFH_CHNL_CLS_M))
#define DM_HCI_RD_AFH_CHNL_CLS_M_COMP ((U16)(ENUM_DM_HCI_RD_AFH_CHNL_CLS_M_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_AFH_CHNL_CLS_M_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 cls_mode;

} BTS2S_DM_HCI_RD_AFH_CHNL_CLS_M_COMP;


typedef BTS2S_HCI_WR_AFH_CHNL_CLS_M DMHCI_WR_AFH_CHNL_CLS_M;
#define DM_HCI_WR_AFH_CHNL_CLS_M ((U16)(ENUM_DM_HCI_WR_AFH_CHNL_CLS_M))
#define DM_HCI_WR_AFH_CHNL_CLS_M_COMP ((U16)(ENUM_DM_HCI_WR_AFH_CHNL_CLS_M_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_WR_AFH_CHNL_CLS_M_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
} BTS2S_DM_HCI_WR_AFH_CHNL_CLS_M_COMP;


typedef BTS2S_HCI_RD_LOCAL_EXT_FEATR DMHCI_RD_LOCAL_EXT_FEATR;
#define DM_HCI_RD_LOCAL_EXT_FEATR ((U16)(ENUM_DM_HCI_RD_LOCAL_EXT_FEATR))
#define DM_HCI_RD_LOCAL_EXT_FEATR_COMP ((U16)(ENUM_DM_HCI_RD_LOCAL_EXT_FEATR_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_LOCAL_EXT_FEATR_COMP */
    U16 phdl; /* dest phdl */
    U8 st; /* succ or fail */
    U8 page_num;
    U8 max_page_num;
    U8 lmp_ext_featr[8]; /* TODO: bit masks! */
} BTS2S_DM_HCI_RD_LOCAL_EXT_FEATR_COMP;


typedef BTS2S_HCI_RD_AFH_CHNL_MAP DMHCI_RD_AFH_CHNL_MAP;
#define DM_HCI_RD_AFH_CHNL_MAP ((U16)(ENUM_DM_HCI_RD_AFH_CHNL_MAP))
#define DM_HCI_RD_AFH_CHNL_MAP_COMP ((U16)(ENUM_DM_HCI_RD_AFH_CHNL_MAP_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_AFH_CHNL_MAP_COMP */
    U16 phdl; /* dest phdl */
    U8  st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    U8 mode;
    U8 map[10];
} BTS2S_DM_HCI_RD_AFH_CHNL_MAP_COMP;


typedef BTS2S_HCI_RD_CLOCK DMHCI_RD_CLOCK;
#define DM_HCI_RD_CLOCK ((U16)(ENUM_DM_HCI_RD_CLOCK))
#define DM_HCI_RD_CLOCK_COMP ((U16)(ENUM_DM_HCI_RD_CLOCK_COMP))

typedef struct
{
    U16 type; /* BTS2S_DM_HCI_RD_CLOCK_COMP */
    U16 phdl; /* dest phdl */
    U8  st; /* succ or fail */
    BTS2S_BD_ADDR bd;
    U32 clock;
    U16 accuracy;
} BTS2S_DM_HCI_RD_CLOCK_COMP;

/* the secu manager ques up reqs relating to each ACL, and processes them one at a time */
typedef struct BTS2S_DM_SM_REQ_tag
{
    /* A cp of the req msg */
    union
    {
        U16 type;
        BTS2S_DM_SM_ACCESS_REQ access_req;
        BTS2S_DM_SM_AUTH_REQ auth_req;
        BTS2S_DM_SM_ENCRYPT_REQ enc_req;
    } u;

    /* secu requirements needed for an access req */
    BOOL auth;
    BOOL authorise;
    BOOL encrypt;

    /* pointer to next qued req */
    struct BTS2S_DM_SM_REQ_tag *next;
} BTS2S_DM_SM_REQ;

/* fix here, should be add in config.h  */
#if defined(CFG_BT_VER_21) || defined(CFG_BT_VER_20)
#define HOST_ACL_DATA_PKT_LEN     ((U16)1021)
#else
#define HOST_ACL_DATA_PKT_LEN     ((U16)330)
#endif

#if defined(CFG_BT_VER_21)
#define HCI_EV_MASH_LOW_21          0xFFFF9FFF
#define HCI_EV_MASH_HIGHT_21        0x3DBFF807

#define HCI_EV_MASH_LOW             0xFFFFFFFF
#define HCI_EV_MASH_HIGHT           0x1DBFDFFF

#define INQUERY_EXT_REQ             0x01
#define NOT_INQUERY_EXT_REQ         0x00
#endif

#define HCI_EV_MASK2_LOW           0x01000000
#define HCI_EV_MASK2_HIGH          0x0

#define HCI_EV_MASK_LOW_20          0xFFFFFFFF
#define HCI_EV_MASK_HIGHT_20        0x00001FFF

#define HOST_SCO_DATA_PKT_LEN       ((U8)128)
#define HOST_TOTAL_NUM_ACL_DATA_PKTS ((U16)100)
#define HOST_TOTAL_NUM_SCO_DATA_PKTS ((U16)16)

/* defines for the thresholds that when hit a msg is sent about processed data */
/* note that these should be approx 50% of HOST_TOTAL_NUM_XXX_DATA_PKTS */
#if defined(CFG_BT_VER_211) || defined(CFG_BT_VER_20)
#define HOST_TOTAL_NUM_ACL_DATA_PKTS_BEFORE_NOTI ((U16)25)
#else
#define HOST_TOTAL_NUM_ACL_DATA_PKTS_BEFORE_NOTI ((U16)8)
#endif

#define HOST_TOTAL_NUM_SCO_DATA_PKTS_BEFORE_NOTI ((U16)8)

//#define POWERUP_TIMEOUT ((U32)(1*SECOND)) /* change time from 5s to 1s*/
#define POWERUP_TIMEOUT ((U32)(500*MILLISECOND)) /* change time from 5s to 1s*/

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
