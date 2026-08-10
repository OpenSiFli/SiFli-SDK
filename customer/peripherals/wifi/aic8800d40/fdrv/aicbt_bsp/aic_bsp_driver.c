/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "aic_fw.h"
#include "aic_bsp_driver.h"
#include "rwnx_msg_tx.h"
#include "rwnx_utils.h"
#include "rtos_port.h"
#include <string.h>
#include "sys_al.h"

u8 is_chip_id_h = 0;

static int rwnx_load_firmware(uint32_t **fw_buf, enum aic_fw name)
{
    int size = 0;

    *fw_buf = aic_fw_ptr_get(name);
    size = aic_fw_size_get(name);

    return size;
}

int rwnx_plat_bin_fw_upload_android(struct rwnx_hw *rwnx_hw, uint32_t fw_addr,
							   const enum aic_fw name)
{
	unsigned int i = 0;
	int size;
	uint32_t *dst = NULL;
	int err = 0;

	aic_dbg("%s\n",__func__);

	/* load aic firmware */
	size = rwnx_load_firmware(&dst, name);
	if (size <= 0) {
		aic_dbg("wrong size of firmware file\n");
		dst = NULL;
		return -1;
	}

	/* Copy the file on the Embedded side */
	if (size > 1024) {// > 1KB data
		for (i = 0; i < (size - 1024); i += 1024) {//each time write 1KB
			//aic_dbg("wr blk 0: %p -> %x\r\n", dst + i / 4, fw_addr + i);
			err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, 1024, dst + i / 4);
			if (err) {
				aic_dbg("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
				break;
			}
		}
	}

	if (!err && (i < size)) {// <1KB data
		//aic_dbg("wr blk 1: %p -> %x\r\n", dst + i / 4, fw_addr + i);
		err = rwnx_send_dbg_mem_block_write_req(rwnx_hw, fw_addr + i, size - i, dst + i / 4);
		if (err) {
			aic_dbg("bin upload fail: %x, err:%d\r\n", fw_addr + i, err);
		}
	}

	if (dst) {
		dst = NULL;
	}

	return err;
}

int aicbt_patch_table_free(struct aicbt_patch_table **head)
{
	struct aicbt_patch_table *p = *head, *n = NULL;
	while (p) {
		n = p->next;
		rtos_free(p->name);
		rtos_free(p->data);
		rtos_free(p);
		p = n;
	}
	*head = NULL;
	return 0;
}

struct aicbt_patch_table *aicbt_patch_table_alloc(enum aic_fw name)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	uint8_t *rawdata = NULL, *p;
	int size;
	struct aicbt_patch_table *head = NULL, *new = NULL, *cur = NULL;

	/* load aic firmware */
	size = rwnx_load_firmware((uint32_t **)&rawdata, name);
	if (size <= 0) {
		aic_dbg("wrong size of firmware file\n");
		goto err;
	}

	p = rawdata;
	if (memcmp(p, AICBT_PT_TAG, sizeof(AICBT_PT_TAG) < 16 ? sizeof(AICBT_PT_TAG) : 16)) {
		aic_dbg("TAG err\n");
		goto err;
	}
	p += 16;

	while (p - rawdata < size) {
		new = (struct aicbt_patch_table *)rtos_malloc(sizeof(struct aicbt_patch_table));
		memset(new, 0, sizeof(struct aicbt_patch_table));
		if (head == NULL) {
			head = new;
			cur  = new;
		} else {
			cur->next = new;
			cur = cur->next;
		}

		cur->name = (char *)rtos_malloc(sizeof(char) * 16);
		memset(cur->name, 0, sizeof(char) * 16);
		memcpy(cur->name, p, 16);
		p += 16;

		cur->type = *(uint32_t *)p;
		p += 4;

		cur->len = *(uint32_t *)p;
		p += 4;

		if((cur->type )  >= 1000 ) {//Temp Workaround
			cur->len = 0;
		}else{
			if(cur->len > 0){
				cur->data = (uint32_t *)rtos_malloc(sizeof(uint8_t) * cur->len * 8);
				memset(cur->data, 0, sizeof(uint8_t) * cur->len * 8);
				memcpy(cur->data, p, cur->len * 8);
				p += cur->len * 8;
			}
		}
	}
	return head;

err:
	aicbt_patch_table_free(&head);
	return NULL;
}
#if 0
int aicbt_patch_info_unpack(struct aicbt_patch_info_t *patch_info, struct aicbt_patch_table *head_t)
{
    if (AICBT_PT_INF == head_t->type) {
        patch_info->info_len = head_t->len;
        if(patch_info->info_len == 0)
            return 0;
        memcpy(&patch_info->adid_addrinf, head_t->data, patch_info->info_len);
    }
    return 0;
}
#endif
int aicbt_patch_info_unpack(struct aicbt_patch_info_t *patch_info, struct aicbt_patch_table *head_t)
{
    uint8_t *patch_info_array = (uint8_t*)patch_info;
    int base_len = 0;
    int memcpy_len = 0;
    
    if (AICBT_PT_INF == head_t->type) {
        base_len = ((offsetof(struct aicbt_patch_info_t,  ext_patch_nb_addr) - offsetof(struct aicbt_patch_info_t,  adid_addrinf) )/sizeof(uint32_t))/2;
        aic_dbg("%s head_t->len:%d base_len:%d \r\n", __func__, head_t->len, base_len);

        if (head_t->len > base_len){
            patch_info->info_len = base_len;
            memcpy_len = patch_info->info_len + 1;//include ext patch nb     
        } else{
            patch_info->info_len = head_t->len;
            memcpy_len = patch_info->info_len;
        }
        aic_dbg("%s memcpy_len:%d \r\n", __func__, memcpy_len);   

        if (patch_info->info_len == 0)
            return 0;
       
        memcpy(((patch_info_array) + sizeof(patch_info->info_len)), 
            head_t->data, 
            memcpy_len * sizeof(uint32_t) * 2);
        aic_dbg("%s adid_addrinf:%x addr_adid:%x \r\n", __func__, 
            ((struct aicbt_patch_info_t *)patch_info_array)->adid_addrinf,
            ((struct aicbt_patch_info_t *)patch_info_array)->addr_adid);

        if (patch_info->ext_patch_nb > 0){
            int index = 0;
            patch_info->ext_patch_param = (uint32_t *)(head_t->data + ((memcpy_len) * 2));
            
            for(index = 0; index < patch_info->ext_patch_nb; index++){
                aic_dbg("%s id:%x addr:%x \r\n", __func__, 
                    *(patch_info->ext_patch_param + (index * 2)),
                    *(patch_info->ext_patch_param + (index * 2) + 1));
            }
        }

    }
    return 0;
}

int aicbt_ext_patch_data_load(struct rwnx_hw *rwnx_hw, struct aicbt_patch_info_t *patch_info)
{
    int ret = 0;
    uint32_t ext_patch_nb = patch_info->ext_patch_nb;
    char ext_patch_file_name[50];
    int index = 0;
    uint32_t id = 0;
    uint32_t addr = 0;

    
    if (ext_patch_nb > 0){
        if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) {
			aic_dbg("[0x40506004]: 0x04318000\n");
			ret = rwnx_send_dbg_mem_write_req(rwnx_hw, 0x40506004, 0x04318000);
			aic_dbg("[0x40506004]: 0x04338000\n");
			ret = rwnx_send_dbg_mem_write_req(rwnx_hw, 0x40506004, 0x04338000);
        }
        for (index = 0; index < patch_info->ext_patch_nb; index++){
            id = *(patch_info->ext_patch_param + (index * 2));
            addr = *(patch_info->ext_patch_param + (index * 2) + 1); 
            //memset(ext_patch_file_name, 0, sizeof(ext_patch_file_name));
            //sprintf(ext_patch_file_name,"%s%d.bin",
                //aicbsp_firmware_list[aicbsp_info.cpmode].bt_ext_patch,
                //id);
            aic_dbg("%s ext_patch_id:%x ext_patch_addr:%x \r\n",
                __func__, id, addr);
            enum aic_fw patch_ext_name;
            switch(rwnx_hw->chipid){
                case PRODUCT_ID_AIC8800DC:
                    if(is_chip_id_h){

                    }else{
                        patch_ext_name = FW_PATCH_8800DC_U02_EXT;

                    }break;
                case PRODUCT_ID_AIC8800D80:
                    patch_ext_name = FW_PATCH_8800D80_U02_EXT;
                    break;
                default:
                    break;
            }

            if (rwnx_plat_bin_fw_upload_android(rwnx_hw, addr, patch_ext_name)) {
                ret = -1;
                break;
            }
        }
    }
    return ret;
}

int aicbt_patch_trap_data_load(struct rwnx_hw *rwnx_hw, struct aicbt_patch_table *head)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	struct aicbt_patch_info_t patch_info = {
		.info_len          = 0,
		.adid_addrinf      = 0,
		.addr_adid         = 0,
		.patch_addrinf     = 0,
		.addr_patch        = 0,
		.reset_addr        = 0,
        .reset_val         = 0,
        .adid_flag_addr    = 0,
        .adid_flag         = 0,
        .ext_patch_nb_addr = 0,
		.ext_patch_nb      = 0,
		.ext_patch_param   = 0,
	};
    if(head == NULL){
        return -1;
    }

    enum aic_fw fw_adid = FW_UNKNOWN;
    enum aic_fw fw_patch = FW_UNKNOWN;
	aic_dbg("%s, chipid = %x\n",__func__,rwnx_hw->chipid);

	if(rwnx_hw->chipid == PRODUCT_ID_AIC8801){
		patch_info.addr_adid  = FW_RAM_ADID_BASE_ADDR;
		patch_info.addr_patch = FW_RAM_PATCH_BASE_ADDR;
	}
	else if(rwnx_hw->chipid == PRODUCT_ID_AIC8800DC){
        aicbt_patch_info_unpack(&patch_info, head);
        if(patch_info.reset_addr == 0) {
            patch_info.reset_addr        = FW_RESET_START_ADDR;
            patch_info.reset_val         = FW_RESET_START_VAL;
            patch_info.adid_flag_addr    = FW_ADID_FLAG_ADDR;
            patch_info.adid_flag         = FW_ADID_FLAG_VAL;
            if (rwnx_send_dbg_mem_write_req(rwnx_hw, patch_info.reset_addr, patch_info.reset_val))
                return -1;
            if (rwnx_send_dbg_mem_write_req(rwnx_hw, patch_info.adid_flag_addr, patch_info.adid_flag))
                return -1;
        }
	}
	else{
		aicbt_patch_info_unpack(&patch_info,head);
        if(patch_info.info_len == 0) {
            aic_dbg("%s, aicbt_patch_info_unpack fail\n", __func__);
            return -1;
        }
		aic_dbg("%s, chipid = %x\n",__func__,rwnx_hw->chipid);
	}

    
    enum aic_fw  adid_name;
    enum aic_fw  patch_name;
    switch(rwnx_hw->chipid){
        case PRODUCT_ID_AIC8800DC:
            if(is_chip_id_h){
                adid_name = FW_ADID_8800DC_U02H;
                patch_name = FW_PATCH_8800DC_U02H;
            }else{
                adid_name = FW_ADID_8800DC_U02;
                patch_name = FW_PATCH_8800DC_U02;
            }break;
        case PRODUCT_ID_AIC8800D80:
            adid_name = FW_ADID_8800D80_U02;
            patch_name = FW_PATCH_8800D80_U02;
            break;
        default:
            break;
    }

	if (rwnx_plat_bin_fw_upload_android(rwnx_hw, patch_info.addr_adid, adid_name))
		return -1;
	if (rwnx_plat_bin_fw_upload_android(rwnx_hw, patch_info.addr_patch, patch_name))
		return -1;
	if (aicbt_ext_patch_data_load(rwnx_hw, &patch_info))
		return -1;
	return 0;

}

static struct aicbt_info_t aicbt_info = {
	.btmode        = AICBT_BTMODE_DEFAULT,
	.btport        = AICBT_BTPORT_DEFAULT,
	.uart_baud     = AICBT_UART_BAUD_DEFAULT,
	.uart_flowctrl = AICBT_UART_FC_DEFAULT,
	.lpm_enable = AICBT_LPM_ENABLE_DEFAULT,
	.txpwr_lvl     = AICBT_TXPWR_LVL_DEFAULT,
};

struct aicbt_info_t aicbt_info_8800dc = {
	.btmode 	   = AICBT_BTMODE_BT_WIFI_COMBO,
	.btport 	   = AICBT_BTPORT_DEFAULT,
	.uart_baud	   = AICBT_UART_BAUD_DEFAULT,
	.uart_flowctrl = AICBT_UART_FC_DEFAULT,
	.lpm_enable    = AICBT_LPM_ENABLE_DEFAULT,
	.txpwr_lvl	   = AICBT_TXPWR_LVL_8800dc,
};

struct aicbt_info_t aicbt_info_880080 = {
	.btmode 	   = AICBT_BTMODE_BT_ONLY,
	.btport 	   = AICBT_BTPORT_DEFAULT,
	.uart_baud	   = AICBT_UART_BAUD_1_5M,
	.uart_flowctrl = AICBT_UART_FC_DEFAULT,
	.lpm_enable    = AICBT_LPM_ENABLE_DEFAULT,
	.txpwr_lvl	   = AICBT_TXPWR_LVL_8800dc,
};

int aicbt_patch_table_load(struct rwnx_hw *rwnx_hw, struct aicbt_patch_table *head)
{
	struct aicbt_patch_table *p;
	int ret = 0, i;
	uint32_t *data = NULL;
    if(head == NULL){
        return -1;
    }

    for (p = head; p != NULL; p = p->next) {
    	data = p->data;
    	if (AICBT_PT_BTMODE == p->type) {
    		*(data + 1)  = aicbsp_info.hwinfo < 0;
    		*(data + 3)  = aicbsp_info.hwinfo;
    		*(data + 5)  = (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC?aicbsp_info.cpmode:0);//0;//aicbsp_info.cpmode;

			*(data + 7)  = aicbt_info_880080.btmode;
    		*(data + 9)  = aicbt_info_880080.btport;
    		*(data + 11) = aicbt_info_880080.uart_baud;
    		*(data + 13) = aicbt_info_880080.uart_flowctrl;
    		*(data + 15) = aicbt_info_880080.lpm_enable;
    		*(data + 17) = aicbt_info_880080.txpwr_lvl;

            aic_dbg("%s bt btmode[%d]:%d \r\n", __func__, rwnx_hw->chipid, aicbt_info_880080.btmode);
    		aic_dbg("%s bt uart_baud[%d]:%d \r\n", __func__, rwnx_hw->chipid, aicbt_info_880080.uart_baud);
    		aic_dbg("%s bt uart_flowctrl[%d]:%d \r\n", __func__, rwnx_hw->chipid, aicbt_info_880080.uart_flowctrl);
    		aic_dbg("%s bt lpm_enable[%d]:%d \r\n", __func__, rwnx_hw->chipid, aicbt_info_880080.lpm_enable);
    		aic_dbg("%s bt tx_pwr[%d]:%d \r\n", __func__, rwnx_hw->chipid, aicbt_info_880080.txpwr_lvl);
    	}

    	if (AICBT_PT_VER == p->type) {
    		aic_dbg("aicbsp: bt patch version: %s\n", (char *)p->data);
    		continue;
    	}

    	for (i = 0; i < p->len; i++) {
    		ret = rwnx_send_dbg_mem_write_req(rwnx_hw, *data, *(data + 1));
    		if (ret != 0)
    			return ret;
    		data += 2;
    	}
    	if (p->type == AICBT_PT_PWRON)
    		rtos_udelay(500);
    }


	///aicbt_patch_table_free(&head);
	return 0;
}
#if 0
int aicbt_patch_table_load(struct rwnx_hw *rwnx_hw, struct aicbt_patch_table *head)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	struct aicbt_patch_table *p;
	int ret = 0, i;
	uint32_t *data = NULL;
    if(head == NULL){
        return -1;
    }
	if(rwnx_hw->chipid == PRODUCT_ID_AIC8801){
		for (p = head; p != NULL; p = p->next) {
			data = p->data;
			if (AICBT_PT_BTMODE == p->type) {
				*(data + 1)  = aicbsp_info.hwinfo < 0;
				*(data + 3)  = aicbsp_info.hwinfo;
				*(data + 5)  = aicbsp_info.cpmode;

				*(data + 7)  = aicbt_info.btmode;
				*(data + 9)  = aicbt_info.btport;
				*(data + 11) = aicbt_info.uart_baud;
				*(data + 13) = aicbt_info.uart_flowctrl;
				*(data + 15) = aicbt_info.lpm_enable;
				*(data + 17) = aicbt_info.txpwr_lvl;

				aic_dbg("%s bt uart_baud:%d \r\n", __func__, aicbt_info.uart_baud);
				aic_dbg("%s bt uart_flowctrl:%d \r\n", __func__, aicbt_info.uart_flowctrl);
				aic_dbg("%s bt lpm_enable:%d \r\n", __func__, aicbt_info.lpm_enable);
				aic_dbg("%s bt tx_pwr:%d \r\n", __func__, aicbt_info.txpwr_lvl);
			}

			if (AICBT_PT_VER == p->type) {
				aic_dbg("aicbsp: bt patch version: %s\n", (char *)p->data);
				continue;
			}

			for (i = 0; i < p->len; i++) {
				ret = rwnx_send_dbg_mem_write_req(rwnx_hw, *data, *(data + 1));
				if (ret != 0)
					return ret;
				data += 2;
			}
			if (p->type == AICBT_PT_PWRON)
				rtos_msleep(1);
		}
	}
	else if(rwnx_hw->chipid == PRODUCT_ID_AIC8800DC){
	}
	///aicbt_patch_table_free(&head);
	return 0;
}
#endif
int aicbt_init(struct rwnx_hw *rwnx_hw)
{
    enum aic_fw patch_name;
    RWNX_DBG(RWNX_FN_ENTRY_STR);
	u32 mem_addr;
	struct dbg_mem_read_cfm rd_mem_addr_cfm;
	u32 btenable = 0;
    int ret = 0;

	mem_addr = 0x40500000;
    if (rwnx_send_dbg_mem_read_req(rwnx_hw, mem_addr, &rd_mem_addr_cfm))
        return -1;

    aicbsp_info.chip_rev = (u8)((rd_mem_addr_cfm.memdata >> 16) & 0x3F);
    is_chip_id_h = (u8)(((rd_mem_addr_cfm.memdata >> 16) & 0xC0) == 0xC0);

    aic_dbg( "chipid %d,is_chip_id_h %x ,chip_rev %x\n",rwnx_hw->chipid,is_chip_id_h,aicbsp_info.chip_rev);

    switch(rwnx_hw->chipid){
        case PRODUCT_ID_AIC8800DC:
            btenable = 1;
            if(is_chip_id_h)
                patch_name = FW_PATCH_TABLE_8800DC_U02H;
            else
                patch_name = FW_PATCH_TABLE_8800DC_U02;
            break;
        case PRODUCT_ID_AIC8800DW:
            btenable = 0;
            break;
        case PRODUCT_ID_AIC8800D80:
            btenable = 1;
            patch_name = FW_PATCH_TABLE_8800D80_U02;
            break;
        default:
            break;
    }
    if(btenable == 0){
        aic_dbg("btenable 0\n");
        return -1;
    }
    struct aicbt_patch_table *head = aicbt_patch_table_alloc(patch_name);
	if (head == NULL){
        aic_dbg("aicbt_patch_table_alloc fail\n");
        return -1;
    }

    if (aicbt_patch_trap_data_load(rwnx_hw, head)) {
		aic_dbg("aicbt_patch_trap_data_load fail\n");
        ret = -1;
		goto err;
	}

	if (aicbt_patch_table_load(rwnx_hw, head)) {
		 aic_dbg("aicbt_patch_table_load fail\n");
        ret = -1;
		goto err;
	}

err:
	aicbt_patch_table_free(&head);
	return ret;
}

