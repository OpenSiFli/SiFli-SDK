/**
  ******************************************************************************
  * @file   bts2_bt.h
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

#ifndef _BTS2_BT_H_
#define _BTS2_BT_H_

#include "bts2_type.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Transfer layer channel type define */
#if defined(CFG_TL_USB)

#define USB_CHNL_HCI                (0)
#define USB_CHNL_ACL                (1)
#define USB_CHNL_SCO                (2)

#elif defined(CFG_TL_BCSP)

#define BCSP_UNRELIABLE_CHNL        (0)
#define BCSP_RELIABLE_CHNL          (1)
#define BCSP_CHNL_BCCMD             (2)
#define BCSP_CHNL_HQ                (3)
#define BCSP_CHNL_DM                (4)
#define BCSP_CHNL_HCI               (5)
#define BCSP_CHNL_ACL               (6)
#define BCSP_CHNL_SCO               (7)
#define BCSP_CHNL_L2C               (8)
#define BCSP_CHNL_RFC               (9)
#define BCSP_CHNL_SDP               (10)
#define BCSP_CHNL_DFU               (12)

#elif defined(CFG_TL_UART)

#define UART_CHNL_HCI               (0)
#define UART_CHNL_ACL               (1)
#define UART_CHNL_SCO               (2)

#endif

/* Inquiry filter flag */
#define BTS2_INQ_FILTER_CLEAR                (U8)(0x00)
#define BTS2_INQ_FILTER_CLASS_OF_DEV         (U8)(0x01)
#define BTS2_INQ_FILTER_DEV_ADDR             (U8)(0x02)
#define BTS2_INQ_FILTER_PAGE_SCAN_MODE       (U8)(0x04)
#define BTS2_INQ_FILTER_PAGE_SCAN_REP_MODE   (U8)(0x08)
#define BTS2_INQ_FILTER_NOT_CLEAR            (U8)(0x80)

#define BT_HCI_UNLIMITED_INQ_RSPS           0x00
#define BT_HCI_INQUIRY_SCAN_INTVL_DEFAULT   0x0800
#define BT_HCI_INQUIRY_SCAN_WINDOW_DEFAULT  0x0012
#define BT_HCI_PAGE_SCAN_INTVL_DEFAULT      0x0800
#define BT_HCI_PAGE_SCAN_WINDOW_DEFAULT     0x0012
#define BT_HCI_SCAN_ENB_NO_SCAN             0x00
#define BT_HCI_SCAN_ENB_INQUIRY_ONLY        0x01
#define BT_HCI_SCAN_ENB_PAGE_ONLY           0x02
#define BT_HCI_SCAN_ENB_PAGE_AND_INQUIRY    0x03

#define BT_HCI_MIN_PIN_CODE_LEN             ((U8)0x01)
#define BT_HCI_MAX_PIN_CODE_LEN             ((U8)0x10)

#define BT_MASTER_ROLE                      ((U8)0)
#define BT_SLAVE_ROLE                       ((U8)1)
#define BT_ROLE_UNDEFINED                   ((U8)2)

/* Page scan interval: 0x12 - 0x1000 */
#define PAGE_SCAN_INTVL                     0x800
/* Page scan window: window 0x12 - 0x1000 */
#define PAGE_SCAN_WINDOW                    0x12

/* Page scan default interval: 0x1000 (N * 0.625ms) */
#define INQ_SCAN_DFT_INTVL                      0x1000
/* Page scan default window: 0x12 (N * 0.625ms) */
#define INQ_SCAN_DFT_WINDOW                     0x12

/* Time = PAGE_TIMEOUT * 0.000625 */
#define PAGE_TIMEOUT                        (24000)

#define LINK_KEY_SIZE                       0x10

#define EXTERN_INQUIRY_RSP_LEN              0xF0

#ifndef MAX_FRIENDLY_NAME_LEN
//#define MAX_FRIENDLY_NAME_LEN               (247)
#define MAX_FRIENDLY_NAME_LEN               (60)
#endif

#define DB_FILE_PATH    64

typedef U16 BTS2_UUID;
typedef U8 BTS2S_UUID_128[16];
typedef U8 BTS2_LINKKEY[LINK_KEY_SIZE];
typedef S8 BTS2S_DEV_NAME[MAX_FRIENDLY_NAME_LEN + 1];
typedef U8 BTS2S_DB_FILE_PATH[DB_FILE_PATH];

/* Major Service Classes */
#define BT_SRVCLS_LDM                       0x002000
#define BT_SRVCLS_POSITION                  0x010000
#define BT_SRVCLS_NETWORK                   0x020000
#define BT_SRVCLS_RENDER                    0x040000
#define BT_SRVCLS_CAPTURE                   0x080000
#define BT_SRVCLS_OBJECT                    0x100000
#define BT_SRVCLS_AUDIO                     0x200000
#define BT_SRVCLS_TELEPHONE                 0x400000
#define BT_SRVCLS_INFOR                     0x800000
#define BT_SRVCLS_MASK(a)                   (((U32)(a) >> 13) & 0x7FF)

/* Major Device Classes */
#define BT_DEVCLS_MISC                      0x000000
#define BT_DEVCLS_COMPUTER                  0x000100
#define BT_DEVCLS_PHONE                     0x000200
#define BT_DEVCLS_LAP                       0x000300
#define BT_DEVCLS_AUDIO                     0x000400
#define BT_DEVCLS_PERIPHERAL                0x000500
#define BT_DEVCLS_IMAGE                     0x000600
#define BT_DEVCLS_WEARABLE                  0x000700
#define BT_DEVCLS_TOY                       0x000800
#define BT_DEVCLS_HEALTH                    0x000900
#define BT_DEVCLS_UNCLASSIFIED              0x001F00
#define BT_DEVCLS_UTEST                     0x009F00
#define BT_DEVCLS_MASK(a)                   (((U32)(a) >> 8) & 0x1F)
#define BT_MINDEVCLS_MASK(a)                (((U32)(a) >> 2) & 0x3F)

/* the minor device class field - computer major class */
#define BT_COMPCLS_UNCLASSIFIED             (BT_DEVCLS_COMPUTER | 0x000000)
#define BT_COMPCLS_DESKTOP                  (BT_DEVCLS_COMPUTER | 0x000004)
#define BT_COMPCLS_SERVER                   (BT_DEVCLS_COMPUTER | 0x000008)
#define BT_COMPCLS_LAPTOP                   (BT_DEVCLS_COMPUTER | 0x00000C)
#define BT_COMPCLS_HANDHELD                 (BT_DEVCLS_COMPUTER | 0x000010)
#define BT_COMPCLS_PALMSIZED                (BT_DEVCLS_COMPUTER | 0x000014)
#define BT_COMPCLS_WEARABLE                 (BT_DEVCLS_COMPUTER | 0x000018)

/* the minor device class field - phone major class */
#define BT_PHONECLS_UNCLASSIFIED            (BT_DEVCLS_PHONE | 0x000000)
#define BT_PHONECLS_CELLULAR                (BT_DEVCLS_PHONE | 0x000004)
#define BT_PHONECLS_CORDLESS                (BT_DEVCLS_PHONE | 0x000008)
#define BT_PHONECLS_SMARTPHONE              (BT_DEVCLS_PHONE | 0x00000C)
#define BT_PHONECLS_WIREDMODEM              (BT_DEVCLS_PHONE | 0x000010)
#define BT_PHONECLS_COMMONISDNACCESS        (BT_DEVCLS_PHONE | 0x000014)
#define BT_PHONECLS_SIMCARDREADER           (BT_DEVCLS_PHONE | 0x000018)

/* the minor device class field - LAN/Network access point major class */
#define BT_LAP_FULLY                        (BT_DEVCLS_LAP | 0x000000)
#define BT_LAP_17                           (BT_DEVCLS_LAP | 0x000020)
#define BT_LAP_33                           (BT_DEVCLS_LAP | 0x000040)
#define BT_LAP_50                           (BT_DEVCLS_LAP | 0x000060)
#define BT_LAP_67                           (BT_DEVCLS_LAP | 0x000080)
#define BT_LAP_83                           (BT_DEVCLS_LAP | 0x0000A0)
#define BT_LAP_99                           (BT_DEVCLS_LAP | 0x0000C0)
#define BT_LAP_NOSRV                        (BT_DEVCLS_LAP | 0x0000E0)

/* the minor device class field - Audio/Video major class */
#define BT_AV_UNCLASSIFIED                  (BT_DEVCLS_AUDIO | 0x000000)
#define BT_AV_HEADSET                       (BT_DEVCLS_AUDIO | 0x000004)
#define BT_AV_HANDSFREE                     (BT_DEVCLS_AUDIO | 0x000008)
#define BT_AV_HEADANDHAND                   (BT_DEVCLS_AUDIO | 0x00000C)
#define BT_AV_MICROPHONE                    (BT_DEVCLS_AUDIO | 0x000010)
#define BT_AV_LOUDSPEAKER                   (BT_DEVCLS_AUDIO | 0x000014)
#define BT_AV_HEADPHONES                    (BT_DEVCLS_AUDIO | 0x000018)
#define BT_AV_PORTABLEAUDIO                 (BT_DEVCLS_AUDIO | 0x00001C)
#define BT_AV_CARAUDIO                      (BT_DEVCLS_AUDIO | 0x000020)
#define BT_AV_SETTOPBOX                     (BT_DEVCLS_AUDIO | 0x000024)
#define BT_AV_HIFIAUDIO                     (BT_DEVCLS_AUDIO | 0x000028)
#define BT_AV_VCR                           (BT_DEVCLS_AUDIO | 0x00002C)
#define BT_AV_VIDEOCAMERA                   (BT_DEVCLS_AUDIO | 0x000030)
#define BT_AV_CAMCORDER                     (BT_DEVCLS_AUDIO | 0x000034)
#define BT_AV_VIDEOMONITOR                  (BT_DEVCLS_AUDIO | 0x000038)
#define BT_AV_VIDEODISPANDLOUDSPK           (BT_DEVCLS_AUDIO | 0x00003C)
#define BT_AV_VIDEOCONFERENCE               (BT_DEVCLS_AUDIO | 0x000040)
#define BT_AV_GAMEORTOY                     (BT_DEVCLS_AUDIO | 0x000048)

/* the minor device class field - peripheral major class */
#define BT_PERIPHERAL_UNCLASSIFIED          (BT_DEVCLS_PERIPHERAL | 0x000000)
#define BT_PERIPHERAL_JOYSTICK              (BT_DEVCLS_PERIPHERAL | 0x000004)
#define BT_PERIPHERAL_GAMEPAD               (BT_DEVCLS_PERIPHERAL | 0x000008)
#define BT_PERIPHERAL_REMCONTROL            (BT_DEVCLS_PERIPHERAL | 0x00000C)
#define BT_PERIPHERAL_SENSE                 (BT_DEVCLS_PERIPHERAL | 0x000010)
#define BT_PERIPHERAL_TABLET                (BT_DEVCLS_PERIPHERAL | 0x000014)
#define BT_PERIPHERAL_SIMCARDREADER         (BT_DEVCLS_PERIPHERAL | 0x000018)
#define BT_PERIPHERAL_KEYBOARD              (BT_DEVCLS_PERIPHERAL | 0x000040)
#define BT_PERIPHERAL_POINT                 (BT_DEVCLS_PERIPHERAL | 0x000080)
#define BT_PERIPHERAL_KEYORPOINT            (BT_DEVCLS_PERIPHERAL | 0x0000C0)

/* the minor device class field - imaging major class */
#define BT_IMAGE_DISPLAY                    (BT_DEVCLS_IMAGE | 0x000010)
#define BT_IMAGE_CAMERA                     (BT_DEVCLS_IMAGE | 0x000020)
#define BT_IMAGE_SCANNER                    (BT_DEVCLS_IMAGE | 0x000040)
#define BT_IMAGE_PRINTER                    (BT_DEVCLS_IMAGE | 0x000080)

/* the minor device class field - wearable major class */
#define BT_WERABLE_WATCH                    (BT_DEVCLS_WEARABLE | 0x000004)
#define BT_WERABLE_PAGER                    (BT_DEVCLS_WEARABLE | 0x000008)
#define BT_WERABLE_JACKET                   (BT_DEVCLS_WEARABLE | 0x00000C)
#define BT_WERABLE_HELMET                   (BT_DEVCLS_WEARABLE | 0x000010)
#define BT_WERABLE_GLASSES                  (BT_DEVCLS_WEARABLE | 0x000014)

/* Minor Device Class field - Toy Major Class */
#define BT_TOY_ROBOT                        (BT_DEVCLS_TOY | 0x000004)
#define BT_TOY_VEHICLE                      (BT_DEVCLS_TOY | 0x000008)
#define BT_TOY_DOLL                         (BT_DEVCLS_TOY | 0x00000C)
#define BT_TOY_CONTROLLER                   (BT_DEVCLS_TOY | 0x000010)
#define BT_TOY_GAME                         (BT_DEVCLS_TOY | 0x000014)

/* Minor Device Class field - Health */
#define BT_HEALTH_UNDEFINED                 (BT_DEVCLS_HEALTH | 0x000000)
#define BT_HEALTH_BLOOD                     (BT_DEVCLS_HEALTH | 0x000004)
#define BT_HEALTH_THERMOMETER               (BT_DEVCLS_HEALTH | 0x000008)
#define BT_HEALTH_WEIGHING                  (BT_DEVCLS_HEALTH | 0x00000C)
#define BT_HEALTH_GLUCOSE                   (BT_DEVCLS_HEALTH | 0x000010)
#define BT_HEALTH_PULSE                     (BT_DEVCLS_HEALTH | 0x000014)
#define BT_HEALTH_HEART                     (BT_DEVCLS_HEALTH | 0x000018)
#define BT_HEALTH_DISPLAY                   (BT_DEVCLS_HEALTH | 0x00001C)

/* Service class UUID */
#define BT_UUID_SERVER_SERVICE          0x1000  /* ServiceDiscoveryServerServiceClassID */
#define BT_UUID_BROWSE_GROUP            0x1001  /* BrowseGroupDescriptorServiceClassID */
#define BT_UUID_PUBLIC_BROWSE_GROUP     0x1002  /* PublicBrowseGroup */
#define BT_UUID_SERIAL_PORT             0x1101  /* SerialPort */
#define BT_UUID_LAN_ACCESS              0x1102  /* LANAccessUsingPPP */
#define BT_UUID_DIALUP_NET              0x1103  /* DialupNetworking */
#define BT_UUID_IRMC_SYNC               0x1104  /* IrMCSync */
#define BT_UUID_OBEX_OBJ_PUSH           0x1105  /* OBEXObjectPush */
#define BT_UUID_OBEX_FILE_TRANS         0x1106  /* OBEXFileTransfer */
#define BT_UUID_IRMC_SYNC_CMD           0x1107  /* IrMCSyncCommand */
#define BT_UUID_HEADSET                 0x1108  /* Headset */
#define BT_UUID_CORDLESS_TELE           0x1109  /* CordlessTelephony */
#define BT_UUID_AUDIO_SOURCE            0x110A  /* AudioSource  */
#define BT_UUID_AUDIO_SINK              0x110B  /* AudioSink */
#define BT_UUID_AVRCP_TG                0x110C  /* A/V_RemoteControlTarget */
#define BT_UUID_ADV_AUDIO_DISTRIB       0x110D  /* AdvancedAudioDistribution */
#define BT_UUID_AVRCP_CT                0x110E  /* A/V_RemoteControl */
#define BT_UUID_VIDEO_CONFERENCE        0x110F  /* A/V_RemoteControlController */
#define BT_UUID_INTERCOM                0x1110  /* Intercom */
#define BT_UUID_FAX                     0x1111  /* Fax */
#define BT_UUID_HEADSET_AG              0x1112  /* HeadsetAudioGateway */
#define BT_UUID_WAP                     0x1113  /* WAP */
#define BT_UUID_WAP_CLIENT              0x1114  /* WAP_CLIENT */
#define BT_UUID_PAN_PANU                0x1115  /* PANU */
#define BT_UUID_PAN_NAP                 0x1116  /* NAP */
#define BT_UUID_PAN_GN                  0x1117  /* GN */
#define BT_UUID_DIRECT_PRINT            0x1118  /* DirectPrinting */
#define BT_UUID_REF_PRINT               0x1119  /* ReferencePrinting */
#define BT_UUID_IMAGING                 0x111A  /* Imaging */
#define BT_UUID_IMAG_RESPONDER          0x111B  /* ImagingResponder */
#define BT_UUID_IMAG_AUTO_ARCH          0x111C  /* ImagingAutomaticArchive  */
#define BT_UUID_IMAG_REF_OBJ            0x111D  /* ImagingReferencedObjects */
#define BT_UUID_HF                      0x111E  /* Handsfree  */
#define BT_UUID_HF_AG                   0x111F  /* HandsfreeAudioGateway */
#define BT_UUID_DIRECT_PRINT_REF_OBJ    0x1120  /* DirectPrintingReferenceObjectsService  */
#define BT_UUID_REFLECTED_UI            0x1121  /* ReflectedUI */
#define BT_UUID_BASIC_PRINT             0x1122  /* BasicPrinting */
#define BT_UUID_PRINT_ST                0x1123  /* PrintingStatus */
#define BT_UUID_HID                     0x1124  /* HumanInterfaceDeviceService */
#define BT_UUID_HCRP                    0x1125  /* HardcopyCableReplacement */
#define BT_UUID_HCR_PRINT               0x1126  /* HCR_Print */
#define BT_UUID_HCR_SCAN                0x1127  /* HCR_Scan */
#define BT_UUID_COMMON_ISDN_ACCESS      0x1128  /* Common_ISDN_Access */
#define BT_UUID_VIDEO_CONFERENCE_GW     0x1129  /* VideoConferencingGW  */
#define BT_UUID_UDI_MT                  0x112A  /* UDI_MT */
#define BT_UUID_UDI_TA                  0x112B  /* UDI_TA */
#define BT_UUID_AUDIO_VIDEO             0x112C  /* Audio/Video */
#define BT_UUID_SIM_ACCESS              0x112D  /* SIM_Access  */
#define BT_UUID_PB_ACCESS_PCE           0x112E  /* Phonebook Access - PCE */
#define BT_UUID_PB_ACCESS_PSE           0x112F  /* Phonebook Access - PSE */
#define BT_UUID_PB_ACCESS               0x1130  /* Phonebook Access  */
#define BT_UUID_PNP_INFO                0x1200  /* PnPInformation */
#define BT_UUID_GENERIC_NET             0x1201  /* GenericNetworking */
#define BT_UUID_GENERIC_FILE_TRANS      0x1202  /* GenericFileTransfer */
#define BT_UUID_GENERIC_AUDIO           0x1203  /* GenericAudio */
#define BT_UUID_GENERIC_TELE            0x1204  /* GenericTelephony */
#define BT_UUID_UPNP_SVC                0x1205  /* UPNP_Service */
#define BT_UUID_UPNP_IP_SVC             0x1206  /* UPNP_IP_Service */
#define BT_UUID_ESDP_UPNP_IP_PAN        0x1300  /* ESDP_UPNP_IP_PAN */
#define BT_UUID_ESDP_UPNP_IP_LAP        0x1301  /* ESDP_UPNP_IP_LAP */
#define BT_UUID_ESDP_UPNP_L2C           0x1302  /* ESDP_UPNP_L2CAP */
#define BT_UUID_VIDEO_SRC               0x1303  /* VideoSource */
#define BT_UUID_VIDEO_SNK               0x1304  /* VideoSink */
#define BT_UUID_VIDEO_DISTRIBUTION      0x1305  /* VideoDistribution */
#define BT_UUID_HDP                     0x1400  /* HDP */
#define BT_UUID_HDP_SRC                 0x1401  /* HDP Source */
#define BT_UUID_HDP_SNK                 0x1402  /* HDP Sink */

/* Protocol UUID */
#define BT_PROT_SDP                     0x0001
#define BT_PROT_UDP                     0x0002
#define BT_PROT_RFCOMM                  0x0003
#define BT_PROT_TCP                     0x0004
#define BT_PROT_TCSBIN                  0x0005
#define BT_PROT_TCS_AT                  0x0006
#define BT_PROT_TCSBIN_LESS             0x0007
#define BT_PROT_OBEX                    0x0008
#define BT_PROT_IP                      0x0009
#define BT_PROT_FTP                     0x000A
#define BT_PROT_HTTP                    0x000C
#define BT_PROT_WSP                     0x000E
#define BT_PROT_BNEP                    0x000F
#define BT_PROT_HIDP                    0x0011
#define BT_PROT_HCR_CONTROL             0x0012
#define BT_PROT_HIDP_INTERRUPT          0x0013
#define BT_PROT_HCR_DATA                0x0014
#define BT_PROT_HCR_NOTIFY              0x0016
#define BT_PROT_AVCTP                   0x0017
#define BT_PROT_AVDTP                   0x0019
#define BT_PROT_CMTP                    0x001B
#define BT_PROT_UDI_C_PLANE             0x001D
#define BT_PROT_MCAP_CTRL_CHNL          0x001E
#define BT_PROT_L2CAP                   0x0100

/* Protocol and Service Multiplexer */
#define BT_PSM_SDP                      0x0001
#define BT_PSM_RFCOMM                   0x0003
#define BT_PSM_TSC_BIN                  0x0005
#define BT_PSM_TSC_BIN_CORDLESS         0x0007
#define BT_PSM_BNEP                     0x000F
#define BT_PSM_HID_CTRL                 0x0011
#define BT_PSM_HID_INTR                 0x0013
#define BT_PSM_UPNP                     0x0015
#define BT_PSM_AVCTP                    0x0017
#define BT_PSM_AVDTP                    0x0019
#define BT_PSM_AVCTP_BROWSING           0x001B
#define BT_PSM_UDI_C_PLANE              0x001D
#define BT_PSM_HCRP_CNTL                0x2001
#define BT_PSM_HCRP_DATA                0x2003

/* Audio */
#define BT_INPUT_CODING_LINEAR                  0x0000
#define BT_INPUT_CODING_U_LAW                   0x0100
#define BT_INPUT_CODING_A_LAW                   0x0200

#define BT_INPUT_DATA_FMT_1COMPLEMENT           0x0000
#define BT_INPUT_DATA_FMT_2COMPLEMENT           0x0040
#define BT_INPUT_DATA_FMT_SIGNMAGNITUDE         0x0080

#define BT_INPUT_SAMPLE_SIZE_8BIT               0x0000
#define BT_INPUT_SAMPLE_SIZE_16BIT              0x0020

#define BT_AIR_CODING_CVSD                      0x0000
#define BT_AIR_CODING_U_LAW                     0x0001
#define BT_AIR_CODING_A_LAW                     0x0002
#define BT_AIR_CODING_TRANSPARENT_DATA          0x0003

#define BT_VOICE_SETTING (BT_INPUT_CODING_LINEAR | BT_INPUT_DATA_FMT_2COMPLEMENT | BT_INPUT_SAMPLE_SIZE_16BIT | BT_AIR_CODING_CVSD)
#define BT_VOICE_SETTING_TRANS (BT_INPUT_CODING_LINEAR | BT_INPUT_DATA_FMT_2COMPLEMENT | BT_INPUT_SAMPLE_SIZE_16BIT | BT_AIR_CODING_TRANSPARENT_DATA)


/* this is the ESCO pkt type definitions */
#define BT_ESCO_PKT_HV1   ((U16)0x0001) /* eSCO only */
#define BT_ESCO_PKT_HV2   ((U16)0x0002) /* eSCO only */
#define BT_ESCO_PKT_HV3   ((U16)0x0004) /* eSCO only */
#define BT_ESCO_PKT_EV3   ((U16)0x0008) /* eSCO only */
#define BT_ESCO_PKT_EV4   ((U16)0x0010) /* eSCO only */
#define BT_ESCO_PKT_EV5   ((U16)0x0020) /* eSCO only */

/* medium rate eSCO */
#define BT_ESCO_PKT_2EV3  ((U16)0x0040) /* eSCO only */
#define BT_ESCO_PKT_3EV3  ((U16)0x0080) /* eSCO only */
#define BT_ESCO_PKT_2EV5  ((U16)0x0100) /* eSCO only */
#define BT_ESCO_PKT_3EV5  ((U16)0x0200) /* eSCO only */

#define BT_SYNC_S1_PACKET_TYPE              0x003F  // 0xFFCF (BT_ESCO_PKT_EV3)
#define BT_SYNC_S1_TX_BANDWIDTH             (8000)
#define BT_SYNC_S1_RX_BANDWIDTH             (8000)
#define BT_SYNC_S1_VOICE_SETTING            (BT_VOICE_SETTING)
#define BT_SYNC_S1_MAX_LATENCY              (0x0007)
#define BT_SYNC_S1_RE_TX_EFFORT             (0x01)

#define BT_SYNC_S4_PACKET_TYPE              0x003F  // 0xFFBF (BT_ESCO_PKT_2EV3)
#define BT_SYNC_S4_TX_BANDWIDTH             (8000)
#define BT_SYNC_S4_RX_BANDWIDTH             (8000)
#define BT_SYNC_S4_VOICE_SETTING            (BT_VOICE_SETTING)
// Change to 0x13 to avoid nego uncetain value in CVSD. But for bqb, this value may change to 0x0C
#define BT_SYNC_S4_MAX_LATENCY              (0x013)
#define BT_SYNC_S4_RE_TX_EFFORT             (0x02)

#define BT_SYNC_T1_PACKET_TYPE              0x003F  // 0xFFCF (BT_ESCO_PKT_EV3)
#define BT_SYNC_T1_TX_BANDWIDTH             (8000)
#define BT_SYNC_T1_RX_BANDWIDTH             (8000)
#define BT_SYNC_T1_VOICE_SETTING            (BT_VOICE_SETTING_TRANS)
#define BT_SYNC_T1_MAX_LATENCY              (8)
#define BT_SYNC_T1_RE_TX_EFFORT             (0x02)

#define BT_SYNC_T2_PACKET_TYPE              (BT_ESCO_PKT_EV3 | BT_ESCO_PKT_3EV3)  // 0xFFBF (BT_ESCO_PKT_EV3 | BT_ESCO_PKT_3EV3)  // 0xFFBF (BT_ESCO_PKT_2EV3)
#define BT_SYNC_T2_TX_BANDWIDTH             (8000)
#define BT_SYNC_T2_RX_BANDWIDTH             (8000)
#define BT_SYNC_T2_VOICE_SETTING            (BT_VOICE_SETTING_TRANS)
#define BT_SYNC_T2_MAX_LATENCY              (13)
#define BT_SYNC_T2_RE_TX_EFFORT             (0x02)

/* eSCO retransmission effort */
#define BT_ESCO_NO_RETX            ((U8)0x00)
#define BT_ESCO_POWER_SAVING_RETX  ((U8)0x01)
#define BT_ESCO_LINK_QA_RETX       ((U8)0x02)
#define BT_ESCO_DONT_CARE_RETX     ((U8)0xFF)

#define BT_ALL_MR_ESCO    ((U16)BT_ESCO_PKT_2EV3 | BT_ESCO_PKT_3EV3 | BT_ESCO_PKT_2EV5 | BT_ESCO_PKT_3EV5)

#define BT_SCO_DFLT_ACPT_TX_BANDWIDTH        (0xFFFFFFFF)
#define BT_SCO_DFLT_ACPT_RX_BANDWIDTH        (0xFFFFFFFF)
#define BT_SCO_DFLT_ACPT_MAX_LATENCY         (0xFFFF)
#define BT_SCO_DFLT_ACPT_VOICE_SETTINGS      (BT_VOICE_SETTING)
#define BT_SCO_DFLT_ACPT_RE_TX_EFFORT        (0xFF)
#define BT_SCO_DFLT_ACPT_AUDIO_QA            (0xFC3F)

#define BT_ESCO_DFLT_CONN_TX_BANDWIDTH       (8000)
#define BT_ESCO_DFLT_CONN_RX_BANDWIDTH       (8000)
#define BT_ESCO_DFLT_CONN_MAX_LATENCY        (0x000A)
#define BT_ESCO_DFLT_CONN_VOICE_SETTINGS     (BT_VOICE_SETTING)
#define BT_ESCO_DFLT_CONN_RE_TX_EFFORT       (BT_ESCO_POWER_SAVING_RETX)
#define BT_ESCO_DFLT_CONN_AUDIO_QA           (0xFC3F)

#define BT_ESCO_DFLT_V20_S3_TX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V20_S3_RX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V20_S3_MAX_LATENCY                   (0x000A)
#define BT_ESCO_DFLT_V20_S3_VOICE_SETTINGS                (BT_VOICE_SETTING)
#define BT_ESCO_DFLT_V20_S3_RE_TX_EFFORT                  (BT_ESCO_POWER_SAVING_RETX)
#define BT_ESCO_DFLT_V20_S3_AUDIO_QA                      (BT_ESCO_PKT_3EV3 | BT_ESCO_PKT_2EV5 | BT_ESCO_PKT_3EV5)

#define BT_ESCO_DFLT_V20_S2_TX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V20_S2_RX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V20_S2_MAX_LATENCY                   (0x0007)
#define BT_ESCO_DFLT_V20_S2_VOICE_SETTINGS                (BT_VOICE_SETTING)
#define BT_ESCO_DFLT_V20_S2_RE_TX_EFFORT                  (BT_ESCO_POWER_SAVING_RETX)
#define BT_ESCO_DFLT_V20_S2_AUDIO_QA                      (BT_ESCO_PKT_3EV3 | BT_ESCO_PKT_2EV5 | BT_ESCO_PKT_3EV5)

#define BT_ESCO_DFLT_V12_S1_TX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V12_S1_RX_BANDWIDTH                  (8000)
#define BT_ESCO_DFLT_V12_S1_MAX_LATENCY                   (0x0007)
#define BT_ESCO_DFLT_V12_S1_VOICE_SETTINGS                (BT_VOICE_SETTING)
#define BT_ESCO_DFLT_V12_S1_RE_TX_EFFORT                  (BT_ESCO_POWER_SAVING_RETX)
#define BT_ESCO_DFLT_V12_S1_AUDIO_QA                      (BT_ESCO_PKT_EV3 | BT_ALL_MR_ESCO)

#define BT_SCO_DFLT_V11_TX_BANDWIDTH                      (8000)
#define BT_SCO_DFLT_V11_RX_BANDWIDTH                      (8000)
#define BT_SCO_DFLT_V11_MAX_LATENCY                       (0x0005)
#define BT_SCO_DFLT_V11_VOICE_SETTINGS                    (BT_VOICE_SETTING)
#define BT_SCO_DFLT_V11_RE_TX_EFFORT                      (BT_ESCO_POWER_SAVING_RETX)
#define BT_SCO_DFLT_V11_AUDIO_QA                          (BT_ESCO_PKT_HV3 | BT_ESCO_PKT_HV2 | BT_ESCO_PKT_HV1)


/* HCI result code from specification */
#define BT_HCI_ACPT                                          0x00
#define BT_HCI_UNKNOWN_HCI_CMD                               0x01
#define BT_HCI_UNKNOWN_CONN_ID                               0x02
#define BT_HCI_HARDWARE_FAIL                                 0x03
#define BT_HCI_PAGE_TIMEOUT                                  0x04
#define BT_HCI_AUTH_FAIL                                     0x05
#define BT_HCI_PIN_MISSING                                   0x06
#define BT_HCI_MEM_CAPACITY_EXCEEDED                         0x07
#define BT_HCI_CONN_TIMEOUT                                  0x08
#define BT_HCI_CONN_LIMIT_EXCEEDED                           0x09
#define BT_HCI_SYNCHRONOUS_CONN_LIMIT_TO_A_DEV_EXCEEDED      0x0A
#define BT_HCI_ACL_CONN_ALRDY_EXITS                          0x0B
#define BT_HCI_CMD_DISALLOWED                                0x0C
#define BT_HCI_CONN_REJED_DUE_TO_LIMITED_RESRCS              0x0D
#define BT_HCI_CONN_REJED_DUE_TO_SECU_REASONS                0x0E
#define BT_HCI_CONN_REJED_DUE_TO_UNACPTABLE_BD_ADDR          0x0F
#define BT_HCI_CONN_ACPT_TIMEOUT_EXCEEDED                    0x10
#define BT_HCI_UNSUPP_FEATURE_OR_PAR_VAL                     0x11
#define BT_HCI_INVLD_HCI_CMD_PARS                            0x12
#define BT_HCI_RMT_USER_TERMINATED_CONN                      0x13
#define BT_HCI_RMT_DEV_TERMINATED_CONN_DUE_TO_LOW_RESRCS     0x14
#define BT_HCI_RMT_DEV_TERMINATED_CONN_DUE_TO_POWER_OFF      0x15
#define BT_HCI_CONN_TERMINATED_BY_LOCAL_HOST                 0x16
#define BT_HCI_REPEATED_ATTMPTS                              0x17
#define BT_HCI_PARING_NOT_ALLOWED                            0x18
#define BT_HCI_UNKNOW_LMP_PDU                                0x19
#define BT_HCI_UNSUPP_RMT_FEATURE                            0x1A
#define BT_HCI_SCO_OFFSET_REJED                              0x1B
#define BT_HCI_SCO_INTVL_REJED                               0x1C
#define BT_HCI_SCO_AIR_MODE_REJED                            0x1D
#define BT_HCI_INVLD_LMP_PARS                                0x1E
#define BT_HCI_UNSPECIFIED_ERR                               0x1F
#define BT_HCI_UNSUPP_LMP_PAR_VAL                            0x20
#define BT_HCI_ROLE_CHANGE_NOT_ALLOWED                       0x21
#define BT_HCI_LMP_RSP_TIMEOUT                               0x22
#define BT_HCI_LMP_ERR_TRANSACTION_COLLISION                 0x23
#define BT_HCI_LMP_PDU_NOT_ALLOWED                           0x24
#define BT_HCI_ENCRYPTION_MODE_NOT_ACPTABLE                  0x25
#define BT_HCI_LINK_KEY_CAN_NOT_BE_CHANGED                   0x26
#define BT_HCI_REQED_qoS_NOT_SUPP                            0x27
#define BT_HCI_INSTANT_PASSED                                0x28
#define BT_HCI_PAIRING_WITH_UNIT_KEY_NOT_SUPP                0x29
#define BT_HCI_DIFFERENT_TRANSACTION_COLLISION               0x2A
#define BT_HCI_QOS_UNACPTABLE_PAR                            0x2C
#define BT_HCI_QOS_REJED                                     0x2D
#define BT_HCI_CHNL_CLSIFICATION_NOT_SUPP                    0x2E
#define BT_HCI_INSUFFICIENT_SECU                             0x2F
#define BT_HCI_PAR_OUT_OF_MANDATORY_RANGE                    0x30
#define BT_HCI_ROLE_SWITCH_PENDING                           0x32
#define BT_HCI_RSV_SLOT_VIOLATION                            0x34
#define BT_HCI_ROLE_SWITCH_FAILED                            0x35

/* Company name string */
#define COMPANY_ID_0            "Ericsson Technology Licensing"
#define COMPANY_ID_1            "Nokia Mobile Phones"
#define COMPANY_ID_2            "Intel Corp"
#define COMPANY_ID_3            "IBM Corp"
#define COMPANY_ID_4            "Toshiba Corp"
#define COMPANY_ID_5            "3Com"
#define COMPANY_ID_6            "Microsoft"
#define COMPANY_ID_7            "Lucent"
#define COMPANY_ID_8            "Motorola"
#define COMPANY_ID_9            "Infineon Technologies AG"
#define COMPANY_ID_10           "Cambridge Silicon Radio"
#define COMPANY_ID_11           "Silicon Wave"
#define COMPANY_ID_12           "Digianswer A/S"
#define COMPANY_ID_13           "Texas Instruments Inc"
#define COMPANY_ID_14           "Parthus Technologies Inc"
#define COMPANY_ID_15           "Broadcom Corporation"
#define COMPANY_ID_16           "Mitel Semiconductor"
#define COMPANY_ID_17           "Widcomm, Inc"
#define COMPANY_ID_18           "Zeevo, Inc"
#define COMPANY_ID_19           "Atmel Corporation"
#define COMPANY_ID_20           "Mitsubishi Electric Corporation"
#define COMPANY_ID_21           "RTX Telecom A/S"
#define COMPANY_ID_22           "KC Technology Inc"
#define COMPANY_ID_23           "Newlogic"
#define COMPANY_ID_24           "Transilica, Inc"
#define COMPANY_ID_25           "Rohde & Schwarz GmbH & Co. KG"
#define COMPANY_ID_26           "TTPCom Limited"
#define COMPANY_ID_27           "Signia Technologies, Inc"
#define COMPANY_ID_28           "Conexant Systems Inc"
#define COMPANY_ID_29           "Qualcomm"
#define COMPANY_ID_30           "Inventel"
#define COMPANY_ID_31           "AVM Berlin"
#define COMPANY_ID_32           "BandSpeed, Inc"
#define COMPANY_ID_33           "Mansella Ltd"
#define COMPANY_ID_34           "NEC Corporation"
#define COMPANY_ID_35           "WavePlus Technology Co., Ltd"
#define COMPANY_ID_36           "Alcatel"
#define COMPANY_ID_37           "Philips Semiconductors"
#define COMPANY_ID_38           "C Technologies"
#define COMPANY_ID_39           "Open Interface"
#define COMPANY_ID_40           "R F Micro Devices"
#define COMPANY_ID_41           "Hitachi Ltd"
#define COMPANY_ID_42           "Symbol Technologies, Inc"
#define COMPANY_ID_43           "Tenovis"
#define COMPANY_ID_44           "Macronix International Co. Ltd"
#define COMPANY_ID_45           "GCT Semiconductor"
#define COMPANY_ID_46           "Norwood Systems"
#define COMPANY_ID_47           "MewTel Technology Inc"
#define COMPANY_ID_48           "ST Microelectronics"
#define COMPANY_ID_49           "Synopsys"
#define COMPANY_ID_50           "Red-M (Communications) Ltd"
#define COMPANY_ID_51           "Commil Ltd"
#define COMPANY_ID_52           "Computer Access Technology Corporation (CATC)"
#define COMPANY_ID_53           "Eclipse (HQ Espana) S.L."
#define COMPANY_ID_54           "Renesas Technology Corp"
#define COMPANY_ID_55           "Mobilian Corporation"
#define COMPANY_ID_56           "Terax"
#define COMPANY_ID_57           "Integrated System Solution Corp"
#define COMPANY_ID_58           "Matsushita Electric Industrial Co., Ltd"
#define COMPANY_ID_59           "Gennum Corporation"
#define COMPANY_ID_60           "Research In Motion"
#define COMPANY_ID_61           "IPextreme, Inc"
#define COMPANY_ID_62           "Systems and Chips, Inc"
#define COMPANY_ID_63           "Bluetooth SIG, Inc"
#define COMPANY_ID_64           "Seiko Epson Corporation"
#define COMPANY_ID_65           "Integrated Silicon Solution Taiwan, Inc"
#define COMPANY_ID_66           "CONWISE Technology Corporation Ltd"
#define COMPANY_ID_67           "PARROT SA"
#define COMPANY_ID_68           "Socket Mobile"
#define COMPANY_ID_69           "Atheros Communications, Inc"
#define COMPANY_ID_70           "MediaTek, Inc"
#define COMPANY_ID_71           "Bluegiga (tentative)"
#define COMPANY_ID_72           "Marvell Technology Group Ltd"
#define COMPANY_ID_73           "3DSP Corporation"
#define COMPANY_ID_74           "Accel Semiconductor Ltd"
#define COMPANY_ID_75           "Continental Automotive Systems"
#define COMPANY_ID_76           "Apple, Inc"
#define COMPANY_ID_77           "Staccato Communications, Inc"
#define COMPANY_ID_78           "Avago Technologies"
#define COMPANY_ID_79           "APT Ltd"
#define COMPANY_ID_80           "SiRF Technology, Inc"
#define COMPANY_ID_81           "Tzero Technologies, Inc"
#define COMPANY_ID_82           "J&M Corporation"
#define COMPANY_ID_83           "Free2move AB"
#define COMPANY_ID_84           "3DiJoy Corporation"
#define COMPANY_ID_85           "Plantronics, Inc"
#define COMPANY_ID_86           "Sony Ericsson Mobile Communications"
#define COMPANY_ID_87           "Harman International Industries, Inc"
#define COMPANY_ID_OTHER        "Unknown"

/* The Link Manager Version parameter */
#define LMP_VER_1_1             0x01
#define LMP_VER_1_2             0x02
#define LMP_VER_2_0             0x03
#define LMP_VER_2_1             0x04
#define LMP_VER_3_0             0x05

#define LMP_ID_0                "Bluetooth LMP 1.0"
#define LMP_ID_1                "Bluetooth LMP 1.1"
#define LMP_ID_2                "Bluetooth LMP 1.2"
#define LMP_ID_3                "Bluetooth LMP 2.0"
#define LMP_ID_4                "Bluetooth LMP 2.1"
#define LMP_ID_5                "Bluetooth LMP 3.0"
#define LMP_ID_OTHER            "Reserved"

/* Host Controller Interface Version */
#define HCI_Version_ID_0        "Bluetooth HCI Specification 1.0B"
#define HCI_Version_ID_1        "Bluetooth HCI Specification 1.1"
#define HCI_Version_ID_2        "Bluetooth HCI Specification 1.2"
#define HCI_Version_ID_3        "Bluetooth HCI Specification 2.0"
#define HCI_Version_ID_4        "Bluetooth HCI Specification 2.1"
#define HCI_Version_ID_5        "Bluetooth HCI Specification 3.0"
#define HCI_Version_ID_OTHER    "Unknown"

#define ELEM_LANGUAGE_MAX_NUM                                3

typedef enum
{
    BTS2_SUCC = 0,
    BTS2_FAILED,       /*  1 */
    BTS2_TIMEOUT,
    BTS2_BOND_FAIL,
    BTS2_CONN_FAIL,
    BTS2_SCO_FAIL,     /*  5 */
    BTS2_LINK_LOST,
    BTS2_LOCAL_DISC,
    BTS2_RMT_DISC,
    BTS2_REJ,
    BTS2_PSM_REJ,
    BTS2_SECU_FAIL,    /* 10 */
    BTS2_SDP_CLT_FAIL,
    BTS2_SDP_SRV_FAIL,
    BTS2_CMD_ERR,
    BTS2_DATA_WR_FAIL,
    BTS2_HW_ERR,       /* 15 */
    BTS2_UNSUPP_FEAT,
    BTS2_EXCEED_MAX_CONN_NUM,
    BTS2_IN_W4CONN_ST,
    BTS2_LOCAL_DISC_FAIL,
    BTS2_SCO_DISC_FAIL,
    BTS2_INQURI_INTERUPT,
    BTS2_INQURI_REPEAT_ERR,
    BTS2_INQURI_DEV_BUSY,
    BTS2_DEV_BUSY,
    BTS2_NO_PROFILE_LINK, //ADD for
} BTS2E_RESULT_CODE;

typedef struct
{
    U24 lap;            /* Lower Address Part 00..23 */
    U8  uap;            /* Upper Address Part 24..31 */
    U16 nap;            /* Non-significant    32..47 */
} BTS2S_BD_ADDR;

typedef struct
{
    U8  callStatus;//:BT_NO_CALL/BT_CALL_ACTIVE
    U8  callSetupStatus;//:BT_NO_CALL/BT_CALL_SETUP_INCOME_CALL/BT_CALL_SETUP_OUT_ALERT_CALL
    U8  callHeldStatus;//:BT_NO_CALL/BT_CALL_BOTH_ACTIVE_HOLD_CALL/BT_CALL_HOLD_ONLY/BT_CALL_NO_HOLD
} bts2_hfp_hf_cind;


typedef struct
{
    U8  filter;         /* Inquiry filter flag */
    U32 dev_mask_cls;   /* Device class mask, A device with a specific Class of Device responded to the Inquiry process */
} BTS2S_CPL_FILTER;

typedef struct
{
    BOOL info_vld;
    U16  clock_offset;
    U8   scan_rep_mode;
    U8   scan_mode;
    U8   rssi;
} BTS2S_CONN_INFO;

typedef struct
{
    BTS2S_BD_ADDR  bd;
    BTS2S_DEV_NAME dev_disp_name;
    U32           dev_cls;
    U32           known_svcs11_00_31;
    U32           known_svcs11_32_63;
    U32           known_svcs12_00_31;
    U32           known_svcs13_00_31;
    BOOL          authorised;
} BTS2S_DEV_PROP;

typedef struct
{
    U16 national_language_id;
    U16 character_encoding;
    U16 attrute_id;
} BTS2S_LANGUAGE_ELEM;

typedef enum
{
    on,
    off
} BTS2E_AUDIO_ST;

#if 0
typedef enum
{
    P0  = 0,
    P1,
    P2
} BTS2E_PAGE_SCAN;
#endif

typedef struct
{
    U8  svc_type;     /* best effort, etc */
    U32 token_rate;   /* token rate */
    U32 token_bucket; /* token bucket */
    U32 peak_bw;      /* peak bandwidth */
    U32 latency;      /* latency */
    U32 delay_var;    /* delay variation */
} BTS2S_QOS_FLOW;

typedef struct BTS2S_ETHER_ADDR
{
    U16 w[3]; /* big endian */
} BTS2S_ETHER_ADDR;
#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
