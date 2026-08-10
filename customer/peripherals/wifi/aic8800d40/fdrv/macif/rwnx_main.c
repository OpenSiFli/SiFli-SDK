/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * @file rwnx_main.c
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */
#include "lmac_mac.h"
#include "reg_access.h"
#include "rwnx_main.h"
#include "rwnx_defs.h"
#include "rwnx_msg_tx.h"
#include "rwnx_platform.h"
#include "fhost_config.h"
#include "sys_al.h"
#include "sdio_co.h"
#include "wlan_if.h"
#include "aic_plat_log.h"
#include "aicwf_compat_8800dc.h"
#include <string.h>
#include "aic_fw.h"

#define ASSOC_REQ 0x00
#define ASSOC_RSP 0x10
#define PROBE_REQ 0x40
#define PROBE_RSP 0x50
#define ACTION 0xD0
#define AUTH 0xB0
#define DEAUTH 0xC0
#define QOS 0x88

#define ACTION_MAC_HDR_LEN 24
#define QOS_MAC_HDR_LEN 26

u8 chip_id = 0;
u8 chip_sub_id = 0;
u8 chip_mcu_id = 0;
u8 chip_fw_match = 0;
int adap_test = 0;

struct rwnx_hw *g_rwnx_hw = NULL;

void rwnx_frame_parser(char* tag, char* data, unsigned long len){
	char print_data[100];
	int print_index = 0;

	memset(print_data, 0, 100);

	if(data[0] == ASSOC_REQ){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "ASSOC_REQ \r\n");
	}else if(data[0] == ASSOC_RSP){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "ASSOC_RSP \r\n");
	}else if(data[0] == PROBE_REQ){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "PROBE_REQ \r\n");
	}else if(data[0] == PROBE_RSP){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "PROBE_RSP \r\n");
	}else if(data[0] == ACTION){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "ACTION ");
		print_index = strlen(print_data);
		if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x00){
			sprintf(&print_data[print_index], "%s", "GO_NEG_REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x01){
			sprintf(&print_data[print_index], "%s", "GO_NEG_RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x02){
			sprintf(&print_data[print_index], "%s", "GO_NEG_CFM \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x03){
			sprintf(&print_data[print_index], "%s", "P2P_INV_REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x04){
			sprintf(&print_data[print_index], "%s", "P2P_INV_RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x05){
			sprintf(&print_data[print_index], "%s", "DD_REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x06){
			sprintf(&print_data[print_index], "%s", "DD_RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x07){
			sprintf(&print_data[print_index], "%s", "PD_REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x08){
			sprintf(&print_data[print_index], "%s", "PD_RSP \r\n");
		}else{
			sprintf(&print_data[print_index], "%s", "UNKNOW \r\n");
		}

	}else if(data[0] == AUTH){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "AUTH \r\n");
	}else if(data[0] == DEAUTH){
		sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "DEAUTH \r\n");
	}else if(data[0] == QOS){
		if(data[QOS_MAC_HDR_LEN + 6] == 0x88 && data[QOS_MAC_HDR_LEN + 7] == 0x8E){
			sprintf(&print_data[print_index], "%s %s %s", __func__, tag, "QOS_802.1X ");
			print_index = strlen(print_data);
			if(data[QOS_MAC_HDR_LEN + 9] == 0x03){
				sprintf(&print_data[print_index], "%s", "EAPOL \r\n");
			}else if(data[QOS_MAC_HDR_LEN + 9] == 0x00){
				sprintf(&print_data[print_index], "%s", "EAP_PACKAGE ");
				print_index = strlen(print_data);
				if(data[QOS_MAC_HDR_LEN + 12] == 0x01){
					sprintf(&print_data[print_index], "%s", "EAP_REQ \r\n");
				}else if(data[QOS_MAC_HDR_LEN + 12] == 0x02){
					sprintf(&print_data[print_index], "%s", "EAP_RSP \r\n");
				}else if(data[QOS_MAC_HDR_LEN + 12] == 0x04){
					sprintf(&print_data[print_index], "%s", "EAP_FAIL \r\n");
				}else{
					sprintf(&print_data[print_index], "%s", "UNKNOW \r\n");
				}
			}else if(data[QOS_MAC_HDR_LEN + 9] == 0x01){
				sprintf(&print_data[print_index], "%s","EAP_START \r\n");
			}
		}
	}

	if(print_index > 0){
		aic_dbg("%s", print_data);
	}

#if 0
	if(data[0] == ASSOC_REQ){
		AIC_LOG_PRINTF("%s %s ASSOC_REQ \r\n", __func__, tag);
	}else if(data[0] == ASSOC_RSP){
		AIC_LOG_PRINTF("%s %s ASSOC_RSP \r\n", __func__, tag);
	}else if(data[0] == PROBE_REQ){
		AIC_LOG_PRINTF("%s %s PROBE_REQ \r\n", __func__, tag);
	}else if(data[0] == PROBE_RSP){
		AIC_LOG_PRINTF("%s %s PROBE_RSP \r\n", __func__, tag);
	}else if(data[0] == ACTION){
		AIC_LOG_PRINTF("%s %s ACTION ", __func__, tag);
		if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x00){
			AIC_LOG_PRINTF("GO NEG REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x01){
			AIC_LOG_PRINTF("GO NEG RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x02){
			AIC_LOG_PRINTF("GO NEG CFM \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x03){
			AIC_LOG_PRINTF("P2P INV REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x04){
			AIC_LOG_PRINTF("P2P INV RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x05){
			AIC_LOG_PRINTF("DD REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x06){
			AIC_LOG_PRINTF("DD RSP \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x07){
			AIC_LOG_PRINTF("PD REQ \r\n");
		}else if(data[ACTION_MAC_HDR_LEN] == 0x04 && data[ACTION_MAC_HDR_LEN + 6] == 0x08){
			AIC_LOG_PRINTF("PD RSP \r\n");
		}else{
			AIC_LOG_PRINTF("\r\n");
		}

	}else if(data[0] == AUTH){
		AIC_LOG_PRINTF("%s %s AUTH \r\n", __func__, tag);
	}else if(data[0] == DEAUTH){
		AIC_LOG_PRINTF("%s %s DEAUTH \r\n", __func__, tag);
	}else if(data[0] == QOS){
		if(data[QOS_MAC_HDR_LEN + 6] == 0x88 && data[QOS_MAC_HDR_LEN + 7] == 0x8E){
			AIC_LOG_PRINTF("%s %s QOS 802.1X ", __func__, tag);
			if(data[QOS_MAC_HDR_LEN + 9] == 0x03){
				AIC_LOG_PRINTF("EAPOL");
			}else if(data[QOS_MAC_HDR_LEN + 9] == 0x00){
				AIC_LOG_PRINTF("EAP PACKAGE ");
				if(data[QOS_MAC_HDR_LEN + 12] == 0x01){
					AIC_LOG_PRINTF("EAP REQ\r\n");
				}else if(data[QOS_MAC_HDR_LEN + 12] == 0x02){
					AIC_LOG_PRINTF("EAP RSP\r\n");
				}else if(data[QOS_MAC_HDR_LEN + 12] == 0x04){
					AIC_LOG_PRINTF("EAP FAIL\r\n");
				}else{
					AIC_LOG_PRINTF("\r\n");
				}
			}else if(data[QOS_MAC_HDR_LEN + 9] == 0x01){
				aic_dbg("EAP START \r\n");

			}
		}
	}
#endif
}

void rwnx_data_dump(char* tag, void* data, unsigned long len){
    unsigned long i = 0;
    unsigned char* data_ = (unsigned char* )data;

    aic_dbg("%s %s len:(%lu)\r\n", __func__, tag, len);

    for (i = 0; i < len; i += 16){
        aic_dbg("%02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            data_[0 + i],
            data_[1 + i],
            data_[2 + i],
            data_[3 + i],
            data_[4 + i],
            data_[5 + i],
            data_[6 + i],
            data_[7 + i],
            data_[8 + i],
            data_[9 + i],
            data_[10 + i],
            data_[11 + i],
            data_[12 + i],
            data_[13 + i],
            data_[14 + i],
            data_[15 + i]);
    }

    #if 1
    for(i = 0; i < len; i++){
        if(data_[i] == 0x63 &&
            data_[i-1] == 0x53 &&
            data_[i-2] == 0x82 &&
            data_[i-3] == 0x63){
            if(data_[i+3] == 0x01){
                aic_dbg("%s DHCP DISCOVER\r\n", tag);
                break;
            }else if(data_[i+3] == 0x02){
                aic_dbg("%s DHCP OFFER\r\n", tag);
                break;
            }else if(data_[i+3] == 0x03){
                aic_dbg("%s DHCP REQUEST\r\n", tag);
                break;
            }else if(data_[i+3] == 0x05){
                aic_dbg("%s DHCP ACK\r\n", tag);
                break;
            }
        }
    }
    #endif

}

#define  LINE       //"\n"
void rwnx_buffer_dump(char* tag, u8 *data, u32_l len)
{
    int i, diff=0;
    AIC_LOG_PRINTF("dump:%s, len(%d)", tag, len);
    for(i=0; i<len; i+=16) {
        if((i+16)<=len) {
            AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x" LINE,
                data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                data[i+8], data[i+9], data[i+10], data[i+11], data[i+12], data[i+13], data[i+14], data[i+15]);
        } else {
            diff = len - i;
            if(diff==15)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9], data[i+10], data[i+11], data[i+12], data[i+13], data[i+14]);
            if(diff==14)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9], data[i+10], data[i+11], data[i+12], data[i+13]);
            if(diff==13)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9], data[i+10], data[i+11], data[i+12]);
            if(diff==12)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9], data[i+10], data[i+11]);
            if(diff==11)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9], data[i+10]);
            if(diff==10)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8], data[i+9]);
            if(diff==9)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x  %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7],
                    data[i+8]);
            if(diff==8)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],  data[i+4],  data[i+5],  data[i+6],  data[i+7]);
            if(diff==7)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],
                    data[i+4], data[i+5], data[i+6]);
            if(diff==6)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],
                    data[i+4], data[i+5]);
            if(diff==5)
                AIC_LOG_PRINTF("%02x %02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3],
                    data[i+4]);
            if(diff==4)
                AIC_LOG_PRINTF("%02x %02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2],  data[i+3]);
            if(diff==3)
                AIC_LOG_PRINTF("%02x %02x %02x" LINE,
                    data[i],   data[i+1], data[i+2]);
            if(diff==2)
                AIC_LOG_PRINTF("%02x %02x" LINE,
                    data[i],   data[i+1]);
            if(diff==1)
                AIC_LOG_PRINTF("%02x" LINE,
                    data[i]);
        }
    }
}


u32 adaptivity_patch_tbl[][2] = {
	{0x0004, 0x0000320A}, //linkloss_thd
    {0x0094, 0x00000000}, //ac_param_conf
	{0x00F8, 0x00010138}, //tx_adaptivity_en
};
u32 patch_tbl[][2] =
{
#if 0 //!defined(CONFIG_LINK_DET_5G)
    {0x0104, 0x00000000}, //link_det_5g
#endif
};

static void patch_config(struct rwnx_hw *rwnx_hw)
{
    #ifdef CONFIG_ROM_PATCH_EN
    const u32 rd_patch_addr = 0x10180;
    #else
    const u32 rd_patch_addr = RAM_FMAC_FW_ADDR + 0x0180;
    #endif
    u32 config_base;
    uint32_t start_addr = 0x1e6000;
    u32 patch_addr = start_addr;
    u32 patch_num = sizeof(patch_tbl)/4;
    struct dbg_mem_read_cfm rd_patch_addr_cfm;
	u32 patch_addr_reg = 0x1e5318;
	u32 patch_num_reg = 0x1e531c;
    int ret = 0;
    u16 cnt = 0;
    int tmp_cnt = 0;
	int adap_patch_num = 0;

    if (rwnx_hw->mode == WIFI_MODE_RFTEST) {
        patch_addr_reg = 0x1e5304;
		patch_num_reg = 0x1e5308;
    }

    AIC_LOG_PRINTF("Read FW mem: %08x\n", rd_patch_addr);
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, rd_patch_addr, &rd_patch_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("patch rd fail\n");
    }
    AIC_LOG_PRINTF("%x=%x\n", rd_patch_addr_cfm.memaddr, rd_patch_addr_cfm.memdata);

    config_base = rd_patch_addr_cfm.memdata;

    ret = rwnx_send_dbg_mem_write_req(rwnx_hw, patch_addr_reg, patch_addr);
    if (ret) {
        AIC_LOG_PRINTF("%x write fail\n", patch_addr_reg);
    }

	if(adap_test){
		AIC_LOG_PRINTF("%s for adaptivity test \r\n", __func__);
		adap_patch_num = sizeof(adaptivity_patch_tbl)/4;
		ret = rwnx_send_dbg_mem_write_req(rwnx_hw, patch_num_reg, patch_num + adap_patch_num);
	}else{
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, patch_num_reg, patch_num);
	}
    if (ret) {
        AIC_LOG_PRINTF("%x write fail\n", patch_num_reg);
    }

    for (cnt = 0; cnt < patch_num/2; cnt+=1) {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt, patch_tbl[cnt][0]+config_base);
        if (ret) {
            AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt);
        }
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt+4, patch_tbl[cnt][1]);
        if (ret) {
            AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt+4);
        }
    }

    if(adap_test){
		for(cnt = 0; cnt < adap_patch_num/2; cnt+=1)
		{
			if((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*(cnt+tmp_cnt), adaptivity_patch_tbl[cnt][0]+config_base))) {
				AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt);
			}

			if((ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*(cnt+tmp_cnt)+4, adaptivity_patch_tbl[cnt][1]))) {
				AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt+4);
			}
		}
	}
}

#define AIC_PATCH_MAGIG_NUM     0x48435450 // "PTCH"
#define AIC_PATCH_MAGIG_NUM_2   0x50544348 // "HCTP"
#define AIC_PATCH_BLOCK_MAX     4

typedef struct {
    uint32_t magic_num;
    uint32_t pair_start;
    uint32_t magic_num_2;
    uint32_t pair_count;
    uint32_t block_dst[AIC_PATCH_BLOCK_MAX];
    uint32_t block_src[AIC_PATCH_BLOCK_MAX];
    uint32_t block_size[AIC_PATCH_BLOCK_MAX]; // word count
} aic_patch_t;

#define AIC_PATCH_OFST(mem) ((size_t) &((aic_patch_t *)0)->mem)
#define AIC_PATCH_ADDR(mem) ((u32)(aic_patch_str_base + AIC_PATCH_OFST(mem)))

u32 adaptivity_patch_tbl_8800d80[][2] = {
	{0x000C, 0x0000320A}, //linkloss_thd
	{0x009C, 0x00000000}, //ac_param_conf
	{0x0154, 0x00010000}, //tx_adaptivity_en
};

u32 patch_tbl_8800d80[][2] = {
    //{0x00b0, 0x9D0a0100}, //debug_mask
	#ifdef USE_5G
	{0x00b4, 0xf3010001},
	#else
	{0x00b4, 0xf3010000},
	#endif
};

static int aicwifi_patch_config_8800d80(struct rwnx_hw *rwnx_hw)
{
	const u32 rd_patch_addr = (((rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) &&
        chip_mcu_id && (rwnx_hw->mode != WIFI_MODE_RFTEST)) ? RAM_FMAC_WD4M_FW_ADDR : RAM_FMAC_FW_ADDR) + 0x0198;
	u32 aic_patch_addr;
	u32 config_base, aic_patch_str_base;
	uint32_t start_addr = 0x0016F800;
	u32 patch_addr = start_addr;
	u32 patch_cnt = sizeof(patch_tbl_8800d80)/sizeof(u32)/2;
	struct dbg_mem_read_cfm rd_patch_addr_cfm;
	int ret = 0;
	int cnt = 0;
	//adap test
	int adap_patch_cnt = 0;

	if (adap_test) {
        AIC_LOG_PRINTF("%s for adaptivity test \r\n", __func__);
		adap_patch_cnt = sizeof(adaptivity_patch_tbl_8800d80)/sizeof(u32)/2;
	}

	aic_patch_addr = rd_patch_addr + 8;

	ret = rwnx_send_dbg_mem_read_req(rwnx_hw, rd_patch_addr, &rd_patch_addr_cfm);
	if (ret) {
		AIC_LOG_PRINTF("patch rd fail\n");
		return ret;
	}

	config_base = rd_patch_addr_cfm.memdata;

	ret = rwnx_send_dbg_mem_read_req(rwnx_hw, aic_patch_addr, &rd_patch_addr_cfm);
	if (ret) {
		AIC_LOG_PRINTF("patch str rd fail\n");
		return ret;
	}
	aic_patch_str_base = rd_patch_addr_cfm.memdata;

	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(magic_num), AIC_PATCH_MAGIG_NUM);
	if (ret) {
		AIC_LOG_PRINTF("0x%x write fail\n", AIC_PATCH_ADDR(magic_num));
		return ret;
	}

	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(magic_num_2), AIC_PATCH_MAGIG_NUM_2);
	if (ret) {
		AIC_LOG_PRINTF("0x%x write fail\n", AIC_PATCH_ADDR(magic_num_2));
		return ret;
	}

	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(pair_start), patch_addr);
	if (ret) {
		AIC_LOG_PRINTF("0x%x write fail\n", AIC_PATCH_ADDR(pair_start));
		return ret;
	}

	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(pair_count), patch_cnt + adap_patch_cnt);
	if (ret) {
		AIC_LOG_PRINTF("0x%x write fail\n", AIC_PATCH_ADDR(pair_count));
		return ret;
	}

	for (cnt = 0; cnt < patch_cnt; cnt++) {
		ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt, patch_tbl_8800d80[cnt][0]+config_base);
		if (ret) {
			AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt);
			return ret;
		}
		ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt+4, patch_tbl_8800d80[cnt][1]);
		if (ret) {
			AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt+4);
			return ret;
		}
	}

	if (adap_test){
		int tmp_cnt = patch_cnt + adap_patch_cnt;
		for (cnt = patch_cnt; cnt < tmp_cnt; cnt++) {
			int tbl_idx = cnt - patch_cnt;
			ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt, adaptivity_patch_tbl_8800d80[tbl_idx][0]+config_base);
			if(ret) {
				AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt);
				return ret;
			}
			ret = rwnx_send_dbg_mem_write_req(rwnx_hw, start_addr+8*cnt+4, adaptivity_patch_tbl_8800d80[tbl_idx][1]);
			if(ret) {
				AIC_LOG_PRINTF("%x write fail\n", start_addr+8*cnt+4);
				return ret;
			}
		}
	}

	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(block_size[0]), 0);
	if (ret) {
		AIC_LOG_PRINTF("block_size[0x%x] write fail: %d\n", AIC_PATCH_ADDR(block_size[0]), ret);
		return ret;
	}
	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(block_size[1]), 0);
	if (ret) {
		AIC_LOG_PRINTF("block_size[0x%x] write fail: %d\n", AIC_PATCH_ADDR(block_size[1]), ret);
		return ret;
	}
	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(block_size[2]), 0);
	if (ret) {
		AIC_LOG_PRINTF("block_size[0x%x] write fail: %d\n", AIC_PATCH_ADDR(block_size[2]), ret);
		return ret;
	}
	ret = rwnx_send_dbg_mem_write_req(rwnx_hw, AIC_PATCH_ADDR(block_size[3]), 0);
	if (ret) {
		AIC_LOG_PRINTF("block_size[0x%x] write fail: %d\n", AIC_PATCH_ADDR(block_size[3]), ret);
		return ret;
	}

	return 0;
}

u32 syscfg_tbl[][2] = {
	{0x40500014, 0x00000101}, // 1)
	{0x40500018, 0x00000109}, // 2)
	{0x40500004, 0x00000010}, // 3) the order should not be changed

	// def CONFIG_PMIC_SETTING
	// U02 bootrom only
	{0x40040000, 0x00001AC8}, // 1) fix panic
	{0x40040084, 0x00011580},
	{0x40040080, 0x00000001},
	{0x40100058, 0x00000000},

	{0x50000000, 0x03220204}, // 2) pmic interface init
	{0x50019150, 0x00000002}, // 3) for 26m xtal, set div1
	{0x50017008, 0x00000000}, // 4) stop wdg
};

u32 syscfg_tbl_masked[][3] = {
	{0x40506024, 0x000000FF, 0x000000DF}, // for clk gate lp_level
};

u32 rf_tbl_masked[][3] = {
	{0x40344058, 0x00800000, 0x00000000},// pll trx
};

u32 aicbsp_syscfg_tbl_8800d80[][2] = {
};

u32 syscfg_tbl_masked_8800d80[][3] = {
//	{0x40506024, 0x000000FF, 0x000000DF}, // for clk gate lp_level
};

u32 rf_tbl_masked_8800d80[][3] = {
//	{0x40344058, 0x00800000, 0x00000000},// pll trx
};

/**
 * @brief Check fw state, Called before system_config
 *
 * @param rwnx_hw
 * @return int 0:fw_down 1:fw_up
 */
int fw_state_check(struct rwnx_hw *rwnx_hw)
{
    int ret, fw_state;
    const u32 mem_addr = 0x40500004;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("%x rd fail: %d\n", mem_addr, ret);
        return -1;
    }
    fw_state = (int)(rd_mem_addr_cfm.memdata >> 4) & 0x01;
    AIC_LOG_PRINTF("fw_state:%x\n", fw_state);
    return fw_state;
}

static void system_config(struct rwnx_hw *rwnx_hw)
{
    int syscfg_num;
    int ret, cnt;
    const u32 mem_addr = 0x40500000;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("%x rd fail: %d\n", mem_addr, ret);
        return;
    }
    chip_id = (u8)(rd_mem_addr_cfm.memdata >> 16);
    AIC_LOG_PRINTF("%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, 0x00000004, &rd_mem_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("[0x00000004] rd fail: %d\n", ret);
        return;
    }
    chip_sub_id = (u8)(rd_mem_addr_cfm.memdata >> 4);
    //AIC_LOG_PRINTF("%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    AIC_LOG_PRINTF("chip_id=%x, chip_sub_id=%x\n", chip_id, chip_sub_id);

    syscfg_num = sizeof(syscfg_tbl) / sizeof(u32) / 2;
    for (cnt = 0; cnt < syscfg_num; cnt++) {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, syscfg_tbl[cnt][0], syscfg_tbl[cnt][1]);
        if (ret) {
            AIC_LOG_PRINTF("%x write fail: %d\n", syscfg_tbl[cnt][0], ret);
            return;
        }
    }

    syscfg_num = sizeof(syscfg_tbl_masked) / sizeof(u32) / 3;
    for (cnt = 0; cnt < syscfg_num; cnt++) {
        ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
            syscfg_tbl_masked[cnt][0], syscfg_tbl_masked[cnt][1], syscfg_tbl_masked[cnt][2]);
        if (ret) {
            AIC_LOG_PRINTF("%x mask write fail: %d\n", syscfg_tbl_masked[cnt][0], ret);
            return;
        }
    }

    ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
                rf_tbl_masked[0][0], rf_tbl_masked[0][1], rf_tbl_masked[0][2]);
    if (ret) {
        AIC_LOG_PRINTF("rf config %x write fail: %d\n", rf_tbl_masked[0][0], ret);
    }
}

static int system_config_8800d80(struct rwnx_hw *rwnx_hw)
{
    int syscfg_num;
    int ret, cnt;
    const u32 mem_addr = 0x40500000;
    struct dbg_mem_read_cfm rd_mem_addr_cfm;
    AIC_LOG_PRINTF("%s rd 0x%x\n", __func__, mem_addr);
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("%x rd fail: %d\n", mem_addr, ret);
        return -1;
    }
    chip_id = (u8)(rd_mem_addr_cfm.memdata >> 16);
    AIC_LOG_PRINTF("%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    if (((rd_mem_addr_cfm.memdata >> 25) & 0x01UL) == 0x00UL) {
        chip_mcu_id = 1;
    }
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, 0x00000020, &rd_mem_addr_cfm);
    if (ret) {
        AIC_LOG_PRINTF("[0x00000020] rd fail: %d\n", ret);
        return -2;
    }
    chip_sub_id = (u8)(rd_mem_addr_cfm.memdata);
    AIC_LOG_PRINTF("flag:%x=%x\n", rd_mem_addr_cfm.memaddr, rd_mem_addr_cfm.memdata);
    AIC_LOG_PRINTF("chip_id=%x, chip_sub_id=%x\n", chip_id, chip_sub_id);

    syscfg_num = sizeof(aicbsp_syscfg_tbl_8800d80) / sizeof(u32) / 2;
    for (cnt = 0; cnt < syscfg_num; cnt++) {
        ret = rwnx_send_dbg_mem_write_req(rwnx_hw, aicbsp_syscfg_tbl_8800d80[cnt][0], aicbsp_syscfg_tbl_8800d80[cnt][1]);
        if (ret) {
            AIC_LOG_PRINTF("%x write fail: %d\n", aicbsp_syscfg_tbl_8800d80[cnt][0], ret);
            return -3;
        }
    }
    return 0;
}

static void sys_config_8800d80(struct rwnx_hw *rwnx_hw)
{
	int ret, cnt;
	int syscfg_num = sizeof(syscfg_tbl_masked_8800d80) / sizeof(u32) / 3;
	for (cnt = 0; cnt < syscfg_num; cnt++) {
		ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
			syscfg_tbl_masked_8800d80[cnt][0], syscfg_tbl_masked_8800d80[cnt][1], syscfg_tbl_masked_8800d80[cnt][2]);
		if (ret) {
			AIC_LOG_PRINTF("%x mask write fail: %d\n", syscfg_tbl_masked_8800d80[cnt][0], ret);
			return;
		}
	}

	ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
				rf_tbl_masked_8800d80[0][0], rf_tbl_masked_8800d80[0][1], rf_tbl_masked_8800d80[0][2]);
	if (ret) {
		AIC_LOG_PRINTF("rf config %x write fail: %d\n", rf_tbl_masked_8800d80[0][0], ret);
		return;
	}
}

#if 0
static void rf_config(struct rwnx_hw *rwnx_hw)
{
    int ret;
    ret = rwnx_send_dbg_mem_mask_write_req(rwnx_hw,
                rf_tbl_masked[0][0], rf_tbl_masked[0][1], rf_tbl_masked[0][2]);
    if (ret) {
        AIC_LOG_PRINTF("rf config %x write fail: %d\n", rf_tbl_masked[0][0], ret);
    }
}
#endif

static int start_from_bootrom(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    /* memory access */
#ifdef CONFIG_ROM_PATCH_EN
    const u32 rd_addr = ROM_FMAC_FW_ADDR;
    const u32 fw_addr = ROM_FMAC_FW_ADDR;
#else
    u32 rd_addr = RAM_FMAC_FW_ADDR;
    u32 fw_addr = RAM_FMAC_FW_ADDR;
#endif
    if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) &&
        chip_mcu_id && (rwnx_hw->mode != WIFI_MODE_RFTEST)) {
        rd_addr = fw_addr = RAM_FMAC_WD4M_FW_ADDR;
    }
    struct dbg_mem_read_cfm rd_cfm;
    AIC_LOG_PRINTF("Read FW mem: %08x\n", rd_addr);
    ret = rwnx_send_dbg_mem_read_req(rwnx_hw, rd_addr, &rd_cfm);
    if (ret) {
        return -1;
    }
    AIC_LOG_PRINTF("cfm: [%08x] = %08x\n", rd_cfm.memaddr, rd_cfm.memdata);

    /* fw start */
    AIC_LOG_PRINTF("Start app: %08x\n", fw_addr);
    ret = rwnx_send_dbg_start_app_req(rwnx_hw, fw_addr, HOST_START_APP_AUTO);
    if (ret) {
        return -1;
    }
    return 0;
}

static rtos_task_handle apm_staloss_task_hdl = NULL;
static rtos_queue apm_staloss_queue = NULL;

void rwnx_apm_staloss_task(void *param)
{
    struct rwnx_hw* rwnx_hw = (struct rwnx_hw *)param;
    struct mm_apm_staloss_ind ind;
    int ret;

    while (1) {
        ret = rtos_queue_read(apm_staloss_queue, &ind, -1, false);
        if (ret == 0) {
            #if 0
            ret = rwnx_send_me_sta_del(rwnx_hw, ind.sta_idx, false);
            if (ret < 0) {
                aic_dbg("me_sta_del msg send fail, ret=%d\n", ret);
            }
            #else
            ret = wlan_ap_disassociate_sta((struct mac_addr *)&ind.mac_addr);
            if (ret < 0) {
                aic_dbg("wlan_ap_disassociate_sta fail, ret=%d\n", ret);
            }
            #endif
        }
    }
}

int rwnx_apm_staloss_init(struct rwnx_hw *rwnx_hw)
{
    int ret;

    if (apm_staloss_task_hdl) {
        aic_dbg("Err: apm_staloss_task exist\n");
        return -1;
    }

    ret = rtos_queue_create(sizeof(struct mm_apm_staloss_ind), 5, &apm_staloss_queue, "apm_staloss_queue");
    if (ret) {
        aic_dbg("Err: apm_staloss_queue create fail, ret=%d\n", ret);
        return -2;
    }

    ret = rtos_task_create(rwnx_apm_staloss_task, "apm_staloss_task", RWNX_APM_STALOSS_TASK,
                    rwnx_apm_staloss_stack_size, (void *)rwnx_hw, rwnx_apm_staloss_priority, &apm_staloss_task_hdl);
    if (ret) {
        aic_dbg("Err: apm_staloss_task create fail, ret=%d\n", ret);
        return -3;
    }

    return 0;
}

int rwnx_apm_staloss_deinit(void)
{
    uint32_t msg;

    if (apm_staloss_task_hdl) {
        rtos_task_delete(apm_staloss_task_hdl);
        apm_staloss_task_hdl = NULL;
    }

    // flush tx queue
    if (rtos_queue_cnt(apm_staloss_queue) > 0) {
        AIC_LOG_PRINTF("apm_staloss_queue cnt:%d\n", rtos_queue_cnt(apm_staloss_queue));
    }
    while (!rtos_queue_is_empty(apm_staloss_queue)) {
        rtos_queue_read(apm_staloss_queue, &msg, 30, false);
        AIC_LOG_PRINTF("apm_staloss_queue msg:%X\n", msg);
    }
    rtos_queue_delete(apm_staloss_queue);
    apm_staloss_queue = NULL;

    return 0;
}

int rwnx_apm_staloss_notify(struct mm_apm_staloss_ind *ind)
{
    int ret;

    ret = rtos_queue_write(apm_staloss_queue, ind, -1, false);

    return ret;
}

static int start_from_bootrom_8800dc(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    u32 rd_addr;
    u32 fw_addr;
    u32 boot_type;
    struct dbg_mem_read_cfm rd_cfm;

    /* memory access */
    if(rwnx_hw->mode == WIFI_MODE_RFTEST){
        rd_addr = RAM_LMAC_FW_ADDR;
        fw_addr = RAM_LMAC_FW_ADDR;
    }
    else{
        rd_addr = RAM_FMAC_FW_ADDR;
        fw_addr = RAM_FMAC_FW_ADDR;
    }

    AIC_LOG_PRINTF("Read FW mem: %08x\n", rd_addr);
    if ((ret = rwnx_send_dbg_mem_read_req(rwnx_hw, rd_addr, &rd_cfm))) {
        return -1;
    }
    AIC_LOG_PRINTF("cfm: [%08x] = %08x\n", rd_cfm.memaddr, rd_cfm.memdata);

    if(rwnx_hw->mode != WIFI_MODE_RFTEST){
        boot_type = HOST_START_APP_DUMMY;
    } else {
        boot_type = HOST_START_APP_AUTO;
        /* for rftest, sdio rx switch to func1 before start_app_req */
        func_flag_rx = false;
    }
    /* fw start */
    AIC_LOG_PRINTF("Start app: %08x, %d\n", fw_addr, boot_type);
    if ((ret = rwnx_send_dbg_start_app_req(rwnx_hw, fw_addr, boot_type))) {
        return -1;
    }
    return 0;
}

#ifdef CONFIG_BT_SUPPORT
extern int aicbt_init(struct rwnx_hw *rwnx_hw);
#endif

int rwnx_fdrv_init(struct rwnx_hw *rwnx_hw)
{
    int ret = 0, idx;
    uint8_t mac_addr_efuse[ETH_ALEN];
    struct mm_set_rf_calib_cfm rf_calib_cfm;
    struct mm_set_stack_start_cfm set_start_cfm;
    struct me_config_req me_config;
    struct mm_start_req start;
    struct mac_addr base_mac_addr;

    AIC_LOG_PRINTF("rwnx_fdrv_init enter\n");

    ret = rwnx_apm_staloss_init(rwnx_hw);
    if (ret) {
        goto err_platon;
    }

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        if (fw_state_check(rwnx_hw) == 0x1) {
            AIC_LOG_PRINTF("fw already loaded\n");
        }
    }

    // system config
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
        system_config(rwnx_hw);
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW){
        ret = system_config_8800dc(rwnx_hw);
        aicwf_patch_wifi_setting_8800dc(rwnx_hw);
    }else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        ret = system_config_8800d80(rwnx_hw);

	if(ret){
		AIC_LOG_PRINTF("system config fail\n");
		goto err_platon;
	}

    #ifdef CONFIG_BT_SUPPORT
    AIC_LOG_PRINTF("aicbt_init enter\n");
    aicbt_init(rwnx_hw);
    #endif

    // platform on(load fw)
    ret = rwnx_platform_on(rwnx_hw);
    if (ret) {
        goto err_platon;
    }

    #ifdef ADAP_TEST
    adap_test = 1;
    #endif

    // patch config
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        patch_config(rwnx_hw);
    }else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        aicwf_patch_config_8800dc(rwnx_hw);
    } else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        ret = aicwifi_patch_config_8800d80(rwnx_hw);
        if (ret) {
            AIC_LOG_PRINTF("AIC aicwifi_patch_config_8800d80 fail\n");
        }
        //sys_config_8800d80(rwnx_hw);
    }

    // start from bootrom
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801 || rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        ret = start_from_bootrom(rwnx_hw);
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
        ret = start_from_bootrom_8800dc(rwnx_hw);
    if (ret) {
        goto err_lmac_reqs;
    }

    // release sdio function2
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        if ((rwnx_hw->mode != WIFI_MODE_RFTEST)) {
            func_flag_tx = false;
            sdio_release_func2();
        }
    }

    #ifdef USE_5G
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, CO_BIT(5), 0, &set_start_cfm);
    }
    #else
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, 0, 0, &set_start_cfm);
    }
    #endif /* USE_5G */
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, 0, 0, &set_start_cfm);
        set_start_cfm.is_5g_support = false;
        fhost_chan.chan5G_cnt = 0;
    } else {
        ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, CO_BIT(5), 0, &set_start_cfm);
    }

    // ic rf init
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801) {
        ret = rwnx_send_txpwr_idx_req(rwnx_hw);
        if (ret) {
            goto err_platon;
        }
        ret = rwnx_send_txpwr_ofst_req(rwnx_hw);
        if (ret) {
            goto err_platon;
        }
        if (rwnx_hw->mode != WIFI_MODE_RFTEST) {
            ret = rwnx_send_rf_calib_req(rwnx_hw, &rf_calib_cfm);
            if (ret) {
                goto err_platon;
            }
        }
    } else if(rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
                rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
        ret = aicwf_set_rf_config_8800dc(rwnx_hw, &rf_calib_cfm);
        if (ret) {
            goto err_platon;
        }
    } else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
        ret = aicwf_set_rf_config_8800d80(rwnx_hw, &rf_calib_cfm);
        if (ret) {
            goto err_platon;
        }
    }

    // read & set base mac address
    #ifdef CONFIG_USE_LOCAL_MAC_ADDR
    rwnx_read_local_mac();
    #else
    rwnx_read_efuse_mac(rwnx_hw);
    #endif

    /* Reset FW */
    ret = rwnx_send_reset(rwnx_hw);
    if (ret) {
        goto err_lmac_reqs;
    }
    ret = rwnx_send_version_req(rwnx_hw, &rwnx_hw->version_cfm);
    if (ret) {
        goto err_lmac_reqs;
    }
    /* Set parameters to firmware */
    fhost_config_prepare(&me_config, &start, &base_mac_addr, true);

    for (idx = 0; idx < NX_VIRT_DEV_MAX; idx++) {
        fhost_cntrl_vif_init(idx, &base_mac_addr);
    }

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801 ||
        ((rwnx_hw->chipid == PRODUCT_ID_AIC8800DC||
        rwnx_hw->chipid == PRODUCT_ID_AIC8800DW ||
        rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) && rwnx_hw->mode != WIFI_MODE_RFTEST)){
        rwnx_send_me_config_req(rwnx_hw, &me_config);
        rwnx_send_me_chan_config_req(rwnx_hw, &fhost_chan);
    }

    if (rwnx_hw->mode != WIFI_MODE_RFTEST) {
        rwnx_send_start(rwnx_hw, &start);
    }

    #ifdef CONFIG_OOB
    aicwf_sdio_oob_enable();
    #endif

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    aicwf_sdio_bus_pwrctl_timer_task_init();
    #endif

    AIC_LOG_PRINTF("rwnx_fdrv_init exit\n");
    return ret;

err_lmac_reqs:
    AIC_LOG_PRINTF("err_lmac_reqs\n");
    return ret;

err_platon:
    AIC_LOG_PRINTF("err_platon\n");
    return ret;
}

int rwnx_read_efuse_mac(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;
    uint8_t mac_addr_efuse[ETH_ALEN];
    uint8_t *mac_addr_ptr = NULL;

    ret = rwnx_send_get_macaddr_req(rwnx_hw, (struct mm_get_mac_addr_cfm *)mac_addr_efuse);
    if (ret)
	{
		AIC_LOG_PRINTF("get mac req fail:%d\n",ret);
        return -1;
    }

    AIC_LOG_PRINTF("get efuse mac %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr_efuse[0],mac_addr_efuse[1],
           mac_addr_efuse[2],mac_addr_efuse[3],mac_addr_efuse[4],mac_addr_efuse[5]);

    if (mac_addr_efuse[0] | mac_addr_efuse[1] | mac_addr_efuse[2] |
        mac_addr_efuse[3] | mac_addr_efuse[4] | mac_addr_efuse[5])
    {
        mac_addr_ptr = mac_addr_efuse;
    }
    set_mac_address(mac_addr_ptr);
    return 0;
}

int rwnx_read_local_mac(void)
{
    int ret = 0;
    uint8_t mac_addr_local[ETH_ALEN] = {0x00,};
    uint8_t *mac_addr_ptr = NULL;

    //ret = get_local_mac_addr(mac_addr_local);
    if (ret) {
        AIC_LOG_PRINTF("get mac req fail:%d\n",ret);
        return -1;
    }

    AIC_LOG_PRINTF("get local mac %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr_local[0],mac_addr_local[1],
           mac_addr_local[2],mac_addr_local[3],mac_addr_local[4],mac_addr_local[5]);

    if (mac_addr_local[0] | mac_addr_local[1] | mac_addr_local[2] |
        mac_addr_local[3] | mac_addr_local[4] | mac_addr_local[5]) {
        mac_addr_ptr = mac_addr_local;
    }
    set_mac_address(mac_addr_ptr);
    return 0;
}

// reset fdrv env
int rwnx_fdrv_deinit(struct rwnx_hw *rwnx_hw)
{
#ifndef	CONFIG_DRIVER_ORM
	rwnx_apm_staloss_deinit();
#endif

	rwnx_platform_off(rwnx_hw);
	rwnx_platform_deinit(rwnx_hw);

	return 0;
}

uint32_t rwnx_fmac_remote_sta_max_get(struct rwnx_hw *rwnx_hw)
{
    uint32_t rem_sta_max = 0;
    if (rwnx_hw) {
        if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) && chip_mcu_id) {
            rem_sta_max = NX_REMOTE_STA_MAX_WD4M;
        } else {
            rem_sta_max = NX_REMOTE_STA_MAX;
        }
    }
    return rem_sta_max;
}
