/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "sys_al.h"
#include "wifi_al.h"
#include "fmac_al.h"
#include "wifi_def.h"
#include "rtos_port.h"
#include "aic_plat_log.h"


#ifdef CONFIG_LOAD_FW_FROM_FILESYSTEM
#include <unistd.h>
#include <fcntl.h>

int rwnx_read_fw_bin(char *fw_name, u32 **fw_buf)
{
	char path[128];
	void *buffer;
	int fd, n;
	off_t length;

	*fw_buf = NULL;
	if (!fw_name)
		return 0;

	snprintf(path, 128, "%s/%s", AICBSP_FW_PATH, fw_name);

    fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        AIC_LOG_PRINTF("Can not open the file %s\n", path);
    } else {
		length = lseek(fd, 0, SEEK_END);
		lseek(fd, 0, SEEK_SET);
		buffer = rtos_malloc(length);
		if (buffer) {
			n = read(fd, buffer, length);
			if (n != length) {
				AIC_LOG_PRINTF("Can not read fw %s\n", path);
				rtos_free(buffer);
				close(fd);
		        return 0;
			}
			*fw_buf = buffer;
			return length;
		}
		close(fd);
	}

	return 0;
}

int rwnx_load_firmware(uint16_t chipid, int mode, u32 **fw_buf)
{
    int size = 0;
	char *fw_name = NULL;

	if (chipid == PRODUCT_ID_AIC8801) {
        if (mode == WIFI_MODE_RFTEST) {
            fw_name = "fmacfw_rf.bin";
        } else {
            fw_name = "fmacfw.bin";
        }
    } else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        if (aic_get_chip_sub_id() == 0) {
            if (mode == WIFI_MODE_RFTEST) {
                if (aic_get_aic8800dc_rf_flag() == 0) {
                    fw_name = "lmacfw_rf_8800dc.bin";
                } else if (aic_get_aic8800dc_rf_flag() == 1) {
                    fw_name = "fmacfw_rf_patch_8800dc.bin";
                }
            } else {
                fw_name = "fmacfw_patch_8800dc.bin";
            }
        } else if (aic_get_chip_sub_id() == 1) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800dc.bin";
            } else {
                fw_name = "fmacfw_patch_8800dc_u02.bin";
            }
        }
    } else if (chipid == PRODUCT_ID_AIC8800D80) {
        if (aic_get_chip_id() == CHIP_REV_U01) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800d80.bin";
            } else {
                fw_name = "fmacfw_8800d80.bin";
            }
        } else if (aic_get_chip_id() == CHIP_REV_U02 || aic_get_chip_id() == CHIP_REV_U03) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800d80_u02.bin";
            } else {
                fw_name = "fmacfw_8800d80_u02.bin";
            }
        }
    }

	return rwnx_read_fw_bin(fw_name, fw_buf);
}

int rwnx_load_patch_tbl(uint16_t chipid, u32 **fw_buf)
{
    int size = 0;
	char *fw_name = NULL;

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        if (aic_get_chip_sub_id() == 1) {
            fw_name = "fmacfw_patch_tbl_8800dc_u02.bin";
        }
    }

	return rwnx_read_fw_bin(fw_name, fw_buf);
}

void rwnx_restore_firmware(u32 **fw_buf)
{
	rtos_free(*fw_buf);
    *fw_buf = NULL;
}

#elif defined(CONFIG_LOAD_FW_FROM_FLASH)
#include "ReliableData.h"
#include "MRD.h"
#include "FlashPartition.h"

extern int RDWIFI_ReadFmSize(int type, int *length, unsigned int *fmaddr, int *headerlen);
extern void Rdisk_FlashRead(unsigned int flashaddress, unsigned char *buffer, unsigned int size);

extern unsigned int LzmaUncompressLen(const unsigned char *src);
int LzmaUncompress(unsigned char *dest, size_t  *destLen, const unsigned char *src, size_t  *srcLen);

#define	WIFI_LZMA_LEN			(128*1024)
static void * wifibin = NULL;
static uint8_t rdwifi_fw_type = 0;

static unsigned char isLzmaCompressed(const unsigned char *src)
{
    unsigned int len = 0;
    int ii = 0;
    unsigned char *temp = NULL;
    unsigned char header_fixed[5] = {0x5d, 0x00, 0x00, 0x10, 0x00};

    temp = (unsigned char *)src;
    for (ii = 0; ii < sizeof(header_fixed); ii++)
    {
        if (*(temp+ii) != header_fixed[ii])
            break;
    }
    if ( ii == sizeof(header_fixed))
        return 1;
    else
        return 0;
}

uint8_t get_wifi_fw_type(void)
{
    return rdwifi_fw_type;
}

unsigned char RDWIFI_Fw_type(void)
{
	FlashLayoutConfInfo *pFlashLayout = GetFlashLayoutConfig();
	int   headersize = (sizeof(RD_BUFFER_HEADER)+sizeof(RD_ENTRY));
	UINT8  *WiFiArea = (UINT8 *)rtos_malloc(headersize);
	UINT8 *pRAMbuf = NULL;
	RD_ENTRY entry = {0};
	RD_BUFFER_HEADER header = {0};

	Rdisk_FlashRead(pFlashLayout->WIFIFirmAddress, WiFiArea, headersize);

	pRAMbuf = WiFiArea + sizeof(RD_BUFFER_HEADER);

	memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));

	if(entry.entryType == MRD_WIFI_NORMAL_FM)
	{
		rdwifi_fw_type |= 0x01;			//nomal fw in wifi area
	}
	else if(entry.entryType == MRD_WIFI_CALI_FM)
	{
		rdwifi_fw_type |= 0x02;			//rf test fw in wifi area
	}

	if(pFlashLayout->WIFICalFirmAddress != 0xFFFFFFFF)
	{
		rdwifi_fw_type |= 0x04;			//rf rest fw in cal area
	}

	Rdisk_FlashRead(pFlashLayout->MRDStartAddress, WiFiArea, headersize);

	pRAMbuf = WiFiArea + sizeof(RD_BUFFER_HEADER);

	memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));

	if(entry.entryType == MRD_WIFI_NORMAL_FM)
	{
		rdwifi_fw_type |= 0x08;		  //patch and cali bin for rf rest
	}


	rtos_free(WiFiArea);

	AIC_LOG_PRINTF("%s:%d",__func__,rdwifi_fw_type);

	return rdwifi_fw_type;
}


static int RDWIFI_ReadFm(char *fw_name, u32 **fw_buf,unsigned char type)
{
	RD_ENTRY entry = {0};
	RD_BUFFER_HEADER header = {0};
	UINT8 *pRAMbuf = NULL;
	int   ret = -1;
	int len,hlen;
	unsigned int fwaddr,offset = 0;
	void *buffer;

	int   headersize = (sizeof(RD_BUFFER_HEADER)+sizeof(RD_ENTRY));
	UINT8  *WiFiArea = (UINT8 *)rtos_malloc(headersize);

	FlashLayoutConfInfo *pFlashLayout = GetFlashLayoutConfig();

	if(NULL == WiFiArea)
	{
		AIC_LOG_PRINTF("malloc wifiarea error");
		return -1;
	}

	fwaddr = pFlashLayout->WIFIFirmAddress;

	if(pFlashLayout->WIFICalFirmAddress != 0xFFFFFFFF)
	{
		if((type == 2)&&(!(rdwifi_fw_type&0x02)))
			fwaddr = pFlashLayout->WIFICalFirmAddress;
	}

	if((rdwifi_fw_type&0x02)&&(rdwifi_fw_type&0x08)&&(type == 1))
	{
		fwaddr = pFlashLayout->MRDStartAddress;
	}

	AIC_LOG_PRINTF("Read WiFi Firmware--->:%s",fw_name);

	hlen = sizeof(RD_BUFFER_HEADER);
	do
	{

		Rdisk_FlashRead(fwaddr, WiFiArea, headersize);
		pRAMbuf = WiFiArea + hlen;

		{
			UINT32 entry_size = 0;
			UINT8  pad_size = 0;
			UINT32 file_size = 0;

			memcpy(&entry, (RD_ENTRY *)pRAMbuf, sizeof(RD_ENTRY));
			entry_size = (entry.fileAndPadSize & 0x00FFFFFF);
			pad_size = (UINT8)(((entry.fileAndPadSize & 0xFF000000) >> 24));
			AIC_LOG_PRINTF("name:%s:%d,%d",entry.file_name,entry_size,pad_size);

			//check type
			if(type == 1 && entry.entryType == MRD_WIFI_NORMAL_FM ||
				type == 2 && entry.entryType == MRD_WIFI_CALI_FM)
			{
				file_size = entry_size - pad_size;
				if(file_size == 0)
				{
					ret = -2;
					goto end;
				}
			}
			else
			{
				ret = -2;
				goto end;
			}

			//find file name
			if(!strcmp(fw_name,entry.file_name))
			{

				buffer = rtos_malloc(file_size);
				if(buffer != NULL)
				{
					Rdisk_FlashRead(fwaddr+headersize-4,buffer,file_size);
					if(isLzmaCompressed(buffer))
					{
						unsigned int wificombosize = LzmaUncompressLen(buffer);
						void *wifibin = rtos_malloc(wificombosize);
						unsigned int combolzmalen = WIFI_LZMA_LEN;
						ret = LzmaUncompress(wifibin, &wificombosize, buffer, &combolzmalen);
						if(!ret)
						{
							*fw_buf = wifibin;
							ret = wificombosize;
							rtos_free(buffer);

							AIC_LOG_PRINTF("decomp WiFi Firmware--->:%d %d\n",wificombosize,**fw_buf);
						}
						else
						{
							ret = -3;
							rtos_free(buffer);
							rtos_free(wifibin);
						}
					}
					else
					{
						*fw_buf = buffer;
						ret = file_size;
					}

				}
				else
				{
					ret = -4;
				}
				break;
			}
			else
			{
				fwaddr += (entry_size+headersize-4);
				hlen = 0;
				headersize = sizeof(RD_ENTRY);
			}

		}
	}while(1);
end:
	rtos_free(WiFiArea);

	return ret;
}


static int rwnx_read_fw_bin(char *fw_name, u32 **fw_buf,unsigned char mode)
{
	int ret;

	ret = RDWIFI_ReadFm(fw_name, fw_buf,mode);

	if(ret <= 0)
	{
        aic_set_chip_fw_match(ret);
		return 0;
	}
	else
	{
        aic_set_chip_fw_match(1);
		return ret;
	}
}

int rwnx_load_firmware(uint16_t chipid, int mode, u32 **fw_buf)
{
	unsigned char local_mode;
	char *fw_name = NULL;

	if (chipid == PRODUCT_ID_AIC8801) {
        if (mode == WIFI_MODE_RFTEST) {
            fw_name = "fmacfw_rf.bin";
        } else {
            fw_name = "fmacfw.bin";
        }
    } else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        if (aic_get_chip_sub_id() == 0) {
            if (mode == WIFI_MODE_RFTEST) {
                if (aic_get_aic8800dc_rf_flag() == 0) {
                    fw_name = "lmacfw_rf_8800dc.bin";
                } else if (aic_get_aic8800dc_rf_flag() == 1) {
                    fw_name = "fmacfw_rf_patch_8800dc.bin";
                }
            } else {
                fw_name = "fmacfw_patch_8800dc.bin";
            }
        } else if (aic_get_chip_sub_id() == 1) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800dc.bin";
            } else {
                fw_name = "fmacfw_patch_8800dc_u02.bin";
            }
        } else if (aic_get_chip_sub_id() == 2) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800dc.bin";
            } else {
                fw_name = "fmacfw_patch_8800dc_h_u02.bin";
            }
        }
    } else if (chipid == PRODUCT_ID_AIC8800D80) {
        if (aic_get_chip_id() == CHIP_REV_U01) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800d80.bin";
            } else {
                fw_name = "fmacfw_8800d80.bin";
            }
        } else if (aic_get_chip_id() == CHIP_REV_U02 || aic_get_chip_id() == CHIP_REV_U03) {
            if (mode == WIFI_MODE_RFTEST) {
                fw_name = "lmacfw_rf_8800d80_u02.bin";
            } else {
                fw_name = "fmacfw_8800d80_u02.bin";
            }
        }
    }

	if (mode == WIFI_MODE_RFTEST)
		local_mode = 2;
	else
		local_mode = 1;

	return rwnx_read_fw_bin(fw_name, fw_buf, local_mode);
}

int rwnx_load_patch_tbl(uint16_t chipid, u32 **fw_buf)
{
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1) {
            return rwnx_read_fw_bin("fmacfw_patch_tbl_8800dc_u02.bin", fw_buf, 1);
        } else if (aic_get_chip_sub_id() == 2) {
            return rwnx_read_fw_bin("fmacfw_patch_tbl_8800dc_h_u02.bin", fw_buf, 1);
        }
    }

	return 0;
}

int rwnx_load_calib(uint16_t chipid, u32 **fw_buf)
{
    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        if (aic_get_chip_sub_id() == 1) {
            return rwnx_read_fw_bin("fmacfw_calib_8800dc_u02.bin", fw_buf, 1);
        } else if (aic_get_chip_sub_id() == 2) {
            return rwnx_read_fw_bin("fmacfw_calib_8800dc_h_u02.bin", fw_buf, 1);
        }
    }

	return 0;
}

void rwnx_restore_firmware(u32 **fw_buf)
{
	rtos_free(*fw_buf);
    *fw_buf = NULL;
}
#else /* read from array */

#include "rwnx_platform.h"

int rwnx_load_firmware(uint16_t chipid, int mode, u32 **fw_buf)
{
    int size = 0;

    if (chipid == PRODUCT_ID_AIC8801) {
        #if defined(CONFIG_AIC8801_SUPPORT)
        if (mode == WIFI_MODE_RFTEST) {
            *fw_buf = (u32 *)aic8800d_rf_fw_ptr_get();
            size = aic8800d_rf_fw_size_get();
        } else {
            *fw_buf = (u32 *)aic8800d_fw_ptr_get();
            size = aic8800d_fw_size_get();
        }
        #endif /* CONFIG_AIC8801_SUPPORT */
    } else if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        #if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
        if (aic_get_chip_sub_id() == 0) {
            if (mode == WIFI_MODE_RFTEST) {
                #if defined(CONFIG_WIFI_MODE_RFTEST)
                if (aic_get_aic8800dc_rf_flag() == 0) {
                    *fw_buf = (u32 *)fmac_aic8800dc_rf_lmacfw_ptr_get();          //lmacfw_rf_8800dc.bin
                    size = fmac_aic8800dc_rf_lmacfw_size_get();
                } else if (aic_get_aic8800dc_rf_flag() == 1) {
                    *fw_buf = (u32 *)fmac_aic8800dc_rf_fmacfw_ptr_get();          //fmacfw_rf_patch_8800dc.bin
                    size = fmac_aic8800dc_rf_fmacfw_size_get();
                }
                #endif
            } else {
                *fw_buf = (u32 *)fmac_aic8800dc_u01_fw_ptr_get();                 //fmacfw_patch_8800dc.bin
                size = fmac_aic8800dc_u01_fw_size_get();
            }
        } else if (aic_get_chip_sub_id() == 1) {
        	if (aic8800dc_calib_flag == 0) {
                if (mode == WIFI_MODE_RFTEST) {
					#if defined(CONFIG_WIFI_MODE_RFTEST)
                    AIC_LOG_PRINTF("aic load lmacfw_rf_8800dc.bin\n\r");
	                *fw_buf = (u32 *)fmac_aic8800dc_rf_lmacfw_ptr_get();          //lmacfw_rf_8800dc.bin
	                size = fmac_aic8800dc_rf_lmacfw_size_get();
                    #endif
	            } else {
					AIC_LOG_PRINTF("aic load fmacfw_patch_8800dc_u02.bin\n\r");
	                *fw_buf = (u32 *)fmac_aic8800dc_u02_fw_ptr_get();             //fmacfw_patch_8800dc_u02.bin
	                size = fmac_aic8800dc_u02_fw_size_get();
	            }
        	} else if (aic8800dc_calib_flag == 1) {
        		AIC_LOG_PRINTF("aic load fmacfw_calib_8800dc_u02.bin\n\r");
        		*fw_buf = (u32 *)fmac_aic8800dc_u02_calib_fw_ptr_get();           //fmacfw_calib_8800dc_u02.bin
				size = fmac_aic8800dc_u02_calib_fw_size_get();
        	}
        } else if (aic_get_chip_sub_id() == 2) {
        	if (aic8800dc_calib_flag == 0) {
	            if (mode == WIFI_MODE_RFTEST) {
					#if defined(CONFIG_WIFI_MODE_RFTEST)
                    AIC_LOG_PRINTF("aic load lmacfw_rf_8800dc.bin\n\r");
	                *fw_buf = (u32 *)fmac_aic8800dc_rf_lmacfw_ptr_get();          //lmacfw_rf_8800dc.bin
	                size = fmac_aic8800dc_rf_lmacfw_size_get();
                    #endif
	            } else {
					AIC_LOG_PRINTF("aic load fmacfw_patch_8800dc_h_u02.bin\n\r");
	                *fw_buf = (u32 *)fmac_aic8800dc_h_u02_fw_ptr_get();           //fmacfw_patch_8800dc_h_u02.bin
	                size = fmac_aic8800dc_h_u02_fw_size_get();
	            }
	       	} else if (aic8800dc_calib_flag == 1) {
	       		AIC_LOG_PRINTF("aic load fmacfw_calib_8800dc_h_u02.bin\n\r");
        		*fw_buf = (u32 *)fmac_aic8800dc_h_u02_calib_fw_ptr_get();         //fmacfw_calib_8800dc_h_u02.bin
				size = fmac_aic8800dc_h_u02_calib_fw_size_get();
        	}
        }
        #endif /* CONFIG_AIC8800DC_SUPPORT or CONFIG_AIC8800DW_SUPPORT */
    } else if (chipid == PRODUCT_ID_AIC8800D80) {
        #if 1//defined(CONFIG_AIC8800D80_SUPPORT)
        if (mode == WIFI_MODE_RFTEST) {
            #if defined(CONFIG_WIFI_MODE_RFTEST)
            *fw_buf = (u32 *)fmac_aic8800d80_u02_rf_fw_ptr_get();
            size = fmac_aic8800d80_u02_rf_fw_size_get();
            #endif
        } else {
            if (aic_get_chip_mcu_id()) {
                AIC_LOG_PRINTF("load 8800m40 m2d fw\n\r");
                *fw_buf = (u32 *)fmac_aic8800m40_fw_ptr_get();
                size = fmac_aic8800m40_fw_size_get();
            } else {
                if (aic_is_chip_id_h()) {
                    *fw_buf = (u32 *)fmac_aic8800d80_h_u02_fw_ptr_get();
                    size = fmac_aic8800d80_h_u02_fw_size_get();
                } else {
                    if (aic_get_chip_id() == CHIP_REV_U01) {
                        *fw_buf = (u32 *)fmac_aic8800d80_fw_ptr_get();
                        size = fmac_aic8800d80_fw_size_get();
                    } else if (aic_get_chip_id() == CHIP_REV_U02 || aic_get_chip_id() == CHIP_REV_U03) {
                        *fw_buf = (u32 *)fmac_aic8800d80_u02_fw_ptr_get();
                        size = fmac_aic8800d80_u02_fw_size_get();
                    }
                }
            }
        }
        #endif /* CONFIG_AIC8800D80_SUPPORT */
    }
    return size;
}

int rwnx_load_patch_tbl(uint16_t chipid, u32 **fw_buf)
{
    int size = 0;

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW) {
        #if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
        if (aic_get_chip_sub_id() == 1) {
			AIC_LOG_PRINTF("aic load fmacfw_patch_tbl_8800dc_u02.bin\n\r");
            *fw_buf = (u32 *)fmac_aic8800dc_u02_patch_tbl_ptr_get();         //fmacfw_patch_tbl_8800dc_u02.bin
            size = fmac_aic8800dc_u02_patch_tbl_size_get();
        } else if (aic_get_chip_sub_id() == 2) {
			AIC_LOG_PRINTF("aic load fmacfw_patch_tbl_8800dc_h_u02.bin\n\r");
            *fw_buf = (u32 *)fmac_aic8800dc_h_u02_patch_tbl_ptr_get();       //fmacfw_patch_tbl_8800dc_h_u02.bin
            size = fmac_aic8800dc_h_u02_patch_tbl_size_get();
        }
        #endif /* CONFIG_AIC8800DC_SUPPORT or CONFIG_AIC8800DW_SUPPORT */
    }
    return size;
}

int rwnx_load_calib(uint16_t chipid, u32 **fw_buf)
{
    int size = 0;

    if (chipid == PRODUCT_ID_AIC8800DC || chipid == PRODUCT_ID_AIC8800DW)
    {
        #if defined(CONFIG_AIC8800DC_SUPPORT) || defined(CONFIG_AIC8800DW_SUPPORT)
        if (aic_get_chip_sub_id() == 1) {
            AIC_LOG_PRINTF("aic load fmacfw_calib_8800dc_u02.bin\n\r");
            *fw_buf = (u32 *)fmac_aic8800dc_u02_calib_fw_ptr_get();
            size = fmac_aic8800dc_u02_calib_fw_size_get();
        } else if (aic_get_chip_sub_id() == 2) {
            AIC_LOG_PRINTF("aic load fmacfw_calib_8800dc_h_u02.bin\n\r");
            *fw_buf = (u32 *)fmac_aic8800dc_h_u02_calib_fw_ptr_get();
            size = fmac_aic8800dc_h_u02_calib_fw_size_get();
        }
        #endif
    }

    return size;
}

void rwnx_restore_firmware(u32 **fw_buf)
{
    *fw_buf = NULL;
}
#endif

