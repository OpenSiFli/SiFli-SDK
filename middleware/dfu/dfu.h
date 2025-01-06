/**
  ******************************************************************************
  * @file   dfu.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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

#ifndef DFU_H
#define DFU_H

#include <rtconfig.h>


#ifndef RT_USED
    #if defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
        #define RT_USED __root
    #elif defined (_MSC_VER) || defined (__TI_COMPILER_VERSION__)
        #define RT_USED
    #else
        #define RT_USED __attribute__((used))
    #endif
#endif


#define HW_EFUSE

#define DFU_UID_SIZE            16
#define DFU_KEY_SIZE            32
#define DFU_SIG_KEY_SIZE        294     // RSA-2048 pub key in DER format
#define DFU_SIG_HASH_SIZE       8
#define DFU_IV_LEN              16
#define DFU_SIG_SIZE            256
#define DFU_SECURE_SIZE         1

#define DFU_IMG_HASH_SIZE       32
#define DFU_PKT_HASH_SIZE       32

#define DFU_MAX_BLK_SIZE        (2048+4)

#define CORE_ALL                0xff
#define CORE_LCPU               0
#define CORE_BL                 1         //reuse for bootloader in flash for pro
#define CORE_HCPU               2
#define CORE_BOOT               3
#define CORE_MAX                (CORE_BOOT+1)


#define DFU_FLASH_SEC_CONFIG    0         //for secure configurations
#define DFU_FLASH_FACTORY_CAL   1         //for factory calibration data
// DFU images is using FLASH 2-9
#define DFU_FLASH_IMG_LCPU      2
#define DFU_FLASH_IMG_BL        3         //reuse for bootloader in flash for pro
#define DFU_FLASH_IMG_HCPU      4
#define DFU_FLASH_IMG_BOOT      5
#define DFU_FLASH_IMG_LCPU2     6
#define DFU_FLASH_IMG_BCPU2     7
#define DFU_FLASH_IMG_HCPU2     8
#define DFU_FLASH_IMG_BOOT2     9

#define DFU_FLASH_HCPU_EXT1     10
#define DFU_FLASH_CONFIG        DFU_FLASH_HCPU_EXT1
#define DFU_FLASH_HCPU_EXT2     11
#define DFU_FLASH_LCPU_EXT1     12
#define DFU_FLASH_LCPU_EXT2     13
#define DFU_FLASH_RESERVED      14
#define DFU_FLASH_SINGLE        15
#define DFU_FLASH_PARTITION     16

#ifdef BSP_USING_DFU_COMPRESS
    #define DFU_FLASH_COMPRESS_CONFIG 17
    #define DFU_FLASH_COMPRESS        18
    #define DFU_FLASH_COMPRESS_IMG_HCPU 19

    #define DFU_FLASH_IMG_COMPRESS_IDX(flash_id) (((flash_id & 0xC0)>>6) + DFU_FLASH_IMG_LCPU)
    #define DFU_FLASH_IMG_COMPRESS_FLASH(flash_id) (flash_id & 0x3F)
#endif

#define DFU_FLASH_IMAGE        20
#define DFU_FLASH_FONT         21

#ifdef BSP_USING_DFU_COMPRESS

    #define DFU_FLASH_IMAGE_COMPRESS      22
    #define DFU_FLASH_FONT_COMPRESS       23


#endif


#define DFU_MAX_VERSION_LEN 18
#define DFU_PART_VERSION_LEN 8

// nand only ota
#if defined(BACKUP_FTAB_START_ADDR) && defined(FLASH_TABLE_START_ADDR)
    #define DFU_FTAB_ADDR FLASH_TABLE_START_ADDR
    #define DFU_BACKUP_FTAB_ADDR BACKUP_FTAB_START_ADDR
#else
    #define DFU_FTAB_ADDR 0x62000000
    #define DFU_BACKUP_FTAB_ADDR 0x62020000
#endif
#define DFU_INFO_OFFSET 0x3000 // 12K

#ifndef DFU_RAM_RUN_STATE_START_ADDR
    #define DFU_RAM_RUN_STATE_START_ADDR 0xFFFFFFFF
#endif
#define DFU_RAM_RUN_ADDR DFU_RAM_RUN_STATE_START_ADDR


#define DFU_FLASH_IMG_IDX(flash_id)  ((flash_id)-(DFU_FLASH_IMG_LCPU))


#define DFU_SING_IMG_START      (FLASH_TABLE_END_ADDR+1)

#define DFU_VERSION_LEN 20

#define DFU_SECURE_PATTERN 0xA5

struct dfu_hdr
{
    /** command, refer to #DFU_IMG_HDR_ENC  */
    uint8_t command;
    /** for command=DFU_CONFIG_ENC: keyid, refer to #DFU_CONFIG_UID
        otherwise: flashid, refer to #DFU_FLASH_SEC_CONFIG */
    uint8_t id;
};

struct dfu_hdr_body
{
    /** command, refer to #DFU_IMG_HDR_ENC  */
    uint8_t command;
    /** for command=DFU_CONFIG_ENC: keyid, refer to #DFU_CONFIG_UID
        otherwise: flashid, refer to #DFU_FLASH_SEC_CONFIG */
    uint8_t id;
    uint16_t reserved;
    uint32_t offset;
};



#ifdef BSP_USING_DFU_COMPRESS
struct dfu_compress_hdr
{
    uint8_t command; /** <command, refer to #DFU_IMG_HDR_COMPRESS */
};


typedef struct
{
    uint32_t packet_len; /**< Compressed packet len */
} dfu_compress_packet_header_t;

#endif

/* Header packet encrypted
---------------------------------------------------------------------------------
|DFU_CHANNEL(1B)|DFU_IMG_HDR(1B)|Core_ID(1B)|flags(1B)|Encrypted image_header   |
---------------------------------------------------------------------------------
*/
#define DFU_FLAG_ENC                1           // Save encrypted on flash, signature is for encrypted content
#define DFU_FLAG_AUTO               2
#define DFU_FLAG_SINGLE             4
#define DFU_IMGHDR_KEY_OFFSET       8
#define DFU_FLAG_COMPRESS           16
struct image_header_enc     // Total length 512, but only 296 bytes are useful in transfer
{
    // In encrypted
    uint32_t    length;
    uint16_t    blksize;
    uint16_t    flags;
    uint8_t     key[DFU_KEY_SIZE];
    uint8_t     sig[DFU_SIG_SIZE];
    uint8_t     ver[DFU_VERSION_LEN];
    uint8_t     reserved[256 - DFU_KEY_SIZE - 8 - DFU_VERSION_LEN]; // Align to 256 bytes for flash erase.
};


struct image_header
{
    uint32_t    length;
    uint16_t    blksize;
    uint16_t    flags;
    uint8_t     sig[DFU_SIG_SIZE];
    uint32_t    target_base;
};

/* Image BODY packet encrypted
--------------------------------------------------------------------------------------------------------------
|DFU_CHANNEL(1B)|DFU_IMG_BODY(1B)|Core_ID(1B)|flags(1B)|hash(32B)|offset(4B)|image_body    |
--------------------------------------------------------------------------------------------------------------
*/
struct image_body_hdr
{
    // struct dfu_hdr hdr;

    // Not encrypted
    uint8_t     hash[DFU_PKT_HASH_SIZE];
    uint32_t    offset;

    // Encrypted
    // uint8_t    image_body[];
};

/* Configuration packet encrypted
--------------------------------------------------------------------------------------------------------------
|DFU_CHANNEL(1B)|DFU_IMG_BODY(1B)|ID(1B)|flags(1B)|hash(32B)|configuration    |
--------------------------------------------------------------------------------------------------------------
*/
struct image_cfg_hdr
{
    // struct dfu_hdr hdr;

    // Not encrypted
    uint8_t     hash[DFU_PKT_HASH_SIZE];

    // Encrypted
    // uint8_t    configuration[];
};

#ifdef BSP_USING_DFU_COMPRESS

struct image_header_compress
{
    union
    {
        struct image_header_enc enc_img;
        struct image_header none_enc_img;
    } img;
    uint32_t    current_img_len;
    uint16_t current_packet_count;
    uint8_t    state;
    uint8_t    compress_img_id;
};

struct image_header_compress_info
{
    uint8_t     sig[DFU_SIG_SIZE];
    uint32_t    total_len;
    uint32_t    pksize;
};

struct image_header_compress_resume
{
    uint8_t     sig[DFU_SIG_SIZE];
    uint8_t     ver[DFU_VERSION_LEN];
    uint32_t    total_len;
    uint32_t    total_dl_len;
    uint32_t    offset;
    uint16_t    img_id;
    uint16_t    pksize;
};


/* Image BODY packet encrypted
--------------------------------------------------------------------------------------------------------------
|DFU_CHANNEL(1B)|DFU_IMG_BODY(1B)|Core_ID(1B)|flags(1B)|hash(32B)|offset(4B)|image_body    |
--------------------------------------------------------------------------------------------------------------
*/
struct image_body_compress_hdr
{

    uint8_t     compress;
    uint32_t    offset;
};

struct dfu_end_hdr
{
    uint8_t     sig[DFU_SIG_SIZE];
};


#endif
struct flash_table
{
    uint32_t base;
    uint32_t size;
    uint32_t xip_base;
    uint32_t flags;
};

/*Reuse on flash_table entry, must keep same size as flash_table*/
struct flash_config
{
    uint32_t page_size;
    uint32_t block_size;
    uint32_t reserved0;
    uint32_t reserved1;
};

typedef struct
{
    uint32_t magic;
    uint8_t current_hcpu;
} flash_target_t;

#define DFU_IMG_HDR_ENC     1
#define DFU_IMG_BODY_ENC    2
#define DFU_CONFIG_ENC      3
#define DFU_END             4
#define DFU_IMG_HDR         5
#define DFU_IMG_BODY        6
#define DFU_CONFIG          7
#define DFU_END_NO_ENC      8


#ifdef BSP_USING_DFU_COMPRESS
    #define DFU_IMG_HDR_COMPRESS  1
    #define DFU_COMPRESS_RESUNME  2
    #define DFU_IMG_BODY_COMPRESS 3
    #define DFU_IMG_END           4
    #define DFU_DOWNLOAD_END      5

    #define DFU_COMPRESS_IDLE   0
    #define DFU_COMPRESS_START  1
    #define DFU_COMPRESS_READY  2
#endif

#define DFU_CONFIG_UID          1
#define DFU_CONFIG_ROOT         2
#define DFU_CONFIG_SIG_HASH     3
#define DFU_CONFIG_SECURE_ENABLED   4
#define DFU_CONFIG_FLASH_TABLE  8
#define DFU_CONFIG_SIG          9
#define DFU_CONFIG_BOOT_PATCH_SIG    10

#define DFU_STATE_BIN_NOT_EXISTED   0
#define DFU_STATE_BIN_READY         1
#define DFU_STATE_BIN_DOWNLOADING   2
#define DFU_STATE_BIN_DOWNLOADED    3
#define DFU_STATE_BIN_INSTALLING    4
#define DFU_STATE_BIN_INSTALLED     5


#define EFUSE_UID           DFU_CONFIG_UID
#define EFUSE_ID_ROOT       DFU_CONFIG_ROOT
#define EFUSE_ID_SIG_HASH   DFU_CONFIG_SIG_HASH
#define EFUSE_ID_SECURE_ENABLED DFU_CONFIG_SECURE_ENABLED

#define DFU_SUCCESS             0
#define DFU_FAIL                -1

#define SECFG_FTAB_OFFSET       (4)
#define SECFG_SIGKEY_OFFSET     (SECFG_FTAB_OFFSET+DFU_FLASH_PARTITION*sizeof(struct flash_table))
#define SECFG_IMG_OFFSET        (4096)
#define SECFG_RUNIMG_OFFSET     (SECFG_IMG_OFFSET+sizeof(struct image_header_enc)*(DFU_FLASH_PARTITION-2))

#define SEC_CONFIG_MAGIC    0x53454346
#define SEC_HALF_MAGIC      0x4546
#define FLASH_UNINIT_32     0xffffffff
#define FLASH_UNINIT_8      0xff


struct sec_configuration
{
    uint32_t magic;
    struct flash_table  ftab[DFU_FLASH_PARTITION];
    uint8_t             sig_pub_key[DFU_SIG_KEY_SIZE];
    uint8_t reserved[4096 - (SECFG_SIGKEY_OFFSET + DFU_SIG_KEY_SIZE) ];
    // Align to sector boundary (4096)
    /** index using flashid-2,  */
    struct image_header_enc imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_PARTITION)];
    /** index by coreid, such as #CORE_LCPU */
    struct image_header_enc *running_imgs[CORE_MAX];
};

typedef struct
{
    uint32_t magic;
    uint8_t running_target;
} dfu_nand_info;

typedef enum
{
    DFU_RAM_STATE_NONE,
    DFU_RAM_STATE_UPDATING,
    DFU_RAM_STATE_UPDATED,
    DFU_RAM_STATE_UPDATE_FAIL,
} dfu_ram_state_t;

typedef struct
{
    uint32_t magic;
    uint32_t ota_addr;
    uint8_t dfu_state;
} dfu_ram_info;


#ifdef BSP_USING_DFU_COMPRESS

struct dfu_compress_flash_table
{
    uint32_t base;
    uint32_t size;
};

#define DFU_COMPRESS_MAX_IMG 3

struct dfu_compress_configuration
{
    uint32_t magic;
    uint32_t img_count;
    struct dfu_compress_flash_table ctab[DFU_COMPRESS_MAX_IMG];
    struct image_header_compress imgs[DFU_COMPRESS_MAX_IMG];
    uint8_t sig_pub_key[DFU_SIG_KEY_SIZE];
    uint8_t reserved[4096 - (8 + sizeof(struct dfu_compress_flash_table) * DFU_COMPRESS_MAX_IMG + sizeof(struct image_header_compress) * DFU_COMPRESS_MAX_IMG + DFU_SIG_KEY_SIZE) ];
};
#endif

#ifndef SF32LB55X
    //TODO: LCPU RAM not available yet
    extern struct sec_configuration sec_config_cache;
    extern struct sec_configuration backup_sec_config;
    extern dfu_nand_info dfu_info;
    #define FLASH_SEC_CACHE ((uint8_t *)&sec_config_cache)
    #define FLASH_SEC_CACHE_SIZE  (sizeof(sec_config_cache))
#else
    #define FLASH_SEC_CACHE (LCPU_RAM_DATA_START_ADDR)
#endif

#ifdef BSP_USING_DFU_COMPRESS
    #define FLASH_DFU_COMPRESS_START       DFU_RES_FLASH_CODE_START_ADDR
#endif

#ifndef HW_EFUSE
#define EFUSE_OFFSET 16384
#define SECFG_UID_OFFSET    (EFUSE_OFFSET)
#define SECFG_ROOTKEY_OFFSET    (SECFG_UID_OFFSET+DFU_UID_SIZE)
#define SECFG_SIG_HASH_OFFSET   (SECFG_ROOTKEY_OFFSET+DFU_KEY_SIZE)
struct sec_efuse
{
    uint8_t             uid[DFU_UID_SIZE];
    uint8_t             root_key[DFU_KEY_SIZE];
    uint8_t             sig_pub_hash[DFU_SIG_HASH_SIZE];
};
#else
#endif


extern struct sec_configuration *g_sec_config;

#ifdef BSP_USING_DFU_COMPRESS
    extern volatile struct dfu_compress_configuration *g_dfu_compress_config;
#endif


void sec_flash_init(void);
int  sec_flash_read(int flashid, uint32_t offset, uint8_t *data, uint32_t size);
void sec_flash_write(int flashid, uint32_t offset, uint8_t *data, uint32_t size);
void sec_flash_erase(int flashid, uint32_t offset, uint32_t size);

void sec_flash_update(int flashid, uint32_t offset, uint8_t *data, uint32_t size);
void sec_flash_write_target(int flashid, uint32_t offset, uint8_t *data, uint32_t size);
void sec_flash_erase_target(int flashid, uint32_t offset, uint32_t size);
void sec_flash_key_update(void);

void init_dfu_efuse_read_hook();
int  sifli_hw_efuse_read_init(void);
int  sifli_hw_efuse_read(uint8_t id, uint8_t *data, int size);
int  sifli_hw_efuse_read_all(void);
int  sifli_hw_efuse_read_bank(uint32_t i);
int  sifli_hw_efuse_write(uint8_t id, uint8_t *data, int size);
int  sifli_hw_dec(uint8_t *key, uint8_t *in_data, uint8_t *out_data, int size, uint32_t init_offset);
int  sifli_hw_dec_key(uint8_t *in_data, uint8_t *out_data, int size);
void sifli_hw_enc(uint8_t *in_data, uint8_t *out_data, int size);
void sifli_hw_enc_with_key(uint8_t *key, uint8_t *in_data, uint8_t *out_data, int size, uint32_t init_offset);
void sifli_hw_init_xip_key(uint8_t *enc_img_key);


void dfu_flash_init(void);
int  dfu_receive_pkt(int len, uint8_t *data);
void dfu_boot_img(int flashid);
void dfu_boot_img_in_flash(int flashid);
uint8_t *dfu_get_counter(uint32_t offset);

struct image_header_enc *dfu_get_img_info(uint8_t img_idx);
int dfu_receive_resume(uint8_t flashid, uint8_t *data, int size);
int dfu_bin_verify(uint8_t *dfu_key);

typedef int (* boot_init_hook)();
typedef int (*flash_read_func)(uint32_t addr, const int8_t *buf, uint32_t size);
typedef int (*flash_write_func)(uint32_t addr, const int8_t *buf, uint32_t size);
typedef int (*flash_erase_func)(uint32_t addr, uint32_t size);

typedef int (*dfu_efuse_read_hook_t)(uint8_t id, uint8_t *data, int size);

extern flash_read_func g_flash_read;
extern flash_write_func g_flash_write;
extern flash_erase_func g_flash_erase;
extern dfu_efuse_read_hook_t g_dfu_efuse_read_hook;


//#define FLASH_SIMULATE
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
