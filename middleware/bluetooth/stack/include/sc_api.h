/**
  ******************************************************************************
  * @file   sc_api.h
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

#ifndef _SC_API_H_
#define _SC_API_H_

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    BTS2MD_SC_REG_REQ = BTS2MD_START,
    BTS2MD_SC_PAIR_REQ,
    BTS2MD_SC_UNPAIR_REQ,
    BTS2MD_SC_PASSKEY_RES,
    BTS2MD_SC_SET_SECU_LEVEL_REQ,
    BTS2MD_SC_ENCRYPTION_REQ,
    BTS2MD_SC_AUTHORISE_RES,
    BTS2MD_SC_DEREG_REQ,
    BTS2MD_SC_ENB_REQ,
    BTS2MD_SC_SET_TRUST_LEVEL_REQ,
    BTS2MD_SC_INTERNAL_TIMER,
    BTS2MD_SC_RD_DEV_RECORD_REQ,
    BTS2MD_SC_RD_PAIRED_DEV_RECORD_REQ,
    BTS2MU_SC_IO_CAPABILITY_RSP,
    BTS2MU_SC_USER_CFM_REQ_RSP,
    BTS2MD_SC_UPDATE_DEV_REQ,
    BTS2MD_SC_RD_PAIRED_DEV_KEY_REQ,

    BTS2MU_SC_PASSKEY_IND = BTS2MU_START,
    BTS2MU_SC_PAIR_CFM,
    BTS2MU_SC_UNPAIR_CFM,
    BTS2MU_SC_SET_SECU_LEVEL_CFM,
    BTS2MU_SC_ENCRYPTION_CFM,
    BTS2MU_SC_AUTHORISE_IND,
    BTS2MU_SC_PAIR_IND,
    BTS2MU_SC_RD_DEV_RECORD_IND,
    BTS2MU_SC_RD_DEV_RECORD_CFM,
    BTS2MU_SC_RD_PAIRED_DEV_RECORD_CFM,
    BTS2MU_SC_UPDATE_TRUST_LEVEL_IND,
    BTS2MU_SC_REMOTE_IO_CAPABILITY_IND,
    BTS2MU_SC_IO_CAPABILITY_REQ_IND,
    BTS2MU_SC_REQ_USER_CONFIRM_IND,
    BTS2MU_SC_REQ_USER_PASSKEY_IND,
    BTS2MU_SC_USER_PASSKEY_NOTIFICATION_IND,
    BTS2MU_SC_RMT_OOB_DATA_CFM,
    BTS2MU_SC_RD_PAIRED_DEV_KEY_CFM,
    BTS2MU_SC_PAIRED_DEV_KEY_DELETE_CFM,
};

#define SC_RECV_MSG_NUM      (BTS2MD_SC_UPDATE_DEV_REQ - BTS2MD_START + 1)

#define SC_SEND_MSG_NUM      (BTS2MU_SC_RMT_OOB_DATA_CFM-BTS2MU_START + 1)

/* IO Capability */
typedef enum
{
    IO_CAPABILITY_DISPLAY_ONLY,         /* Display Only */
    IO_CAPABILITY_DISPLAY_YES_NO,       /* Display Yes/No */
    IO_CAPABILITY_KEYBOARD_ONLY,        /* Keyboard Only */
    IO_CAPABILITY_NO_INPUT_NO_OUTPUT,   /* No Input/Output */
    IO_CAPABILITY_REJECT_REQ            /* Use this to reject the IO capability request */
} BTS2E_SC_IO_CAPABILITY;

/* Authentication Requirements */
typedef enum
{
    SC_NO_MITM_NO_BONDING,      /* MITM Protection Not Required –No Bonding */
    SC_MITM_REQUIRE_NO_BONDING, /* MITM Protection Required – No Bonding */
    SC_NO_MITM_DEDICATED_BONDING_REQUIRE, /* MITM Protection Not Required – Dedicated Bonding */
    SC_MITM_AND_DEDICATED_BONDING_REQUIRE, /* MITM Protection Required – Dedicated Bonding */
    SC_NO_MITM_GENERAL_BONDING_REQUIRE, /* MITM Protection Not Required – General Bonding */
    SC_MITM_AND_GENERAL_BONDING_REQUIRE, /* MITM Protection Required – General Bonding */
    SC_UNKNOWN_AUTH_REQUIREMENTS /* Unrecognized authentication requirements */
} BTS2E_AUTH_REQUIREMENTS;

typedef struct
{
    U16 type;             /* message identity */
    U16 tid;              /* task ID */
    U32 max_num_of_bytes; /* maximum allowed bytes */
} BTS2S_SC_RD_DEV_RECORD_REQ;

typedef struct
{
    U16 type; /* message identity */
    U16 tid;  /* task ID */
} BTS2S_SC_RD_PAIRED_DEV_RECORD_REQ;

typedef struct
{
    U16           type;       /* message identity */
    U32           dev_num;    /* device records number */
    BTS2S_DEV_PROP *dev_prop; /* device property */
} BTS2S_SC_RD_DEV_RECORD_IND;

typedef struct
{
    U16           type;           /* message identity */
    U32           dev_num;        /* device records number */
    U32           total_dev_num;  /* total device records number  */
    BTS2S_DEV_PROP *dev_prop;     /* device property */
} BTS2S_SC_RD_DEV_RECORD_CFM;

typedef struct
{
    U16          type;           /* message identity */
    U32          total_dev_num;  /* total device records number */
    BTS2S_BD_ADDR bd[1];         /* pointer to the Bluetooth address, the actual size is total_dev_num */
} BTS2S_SC_RD_PAIRED_DEV_RECORD_CFM;

typedef struct
{
    U16            type;               /* message identity */
    U16            tid;                /* task ID */
    BTS2S_BD_ADDR  bd;                 /* Bluetooth address */
    BTS2S_DEV_NAME rmt_name;           /* remote side name */
    U32           dev_cls;            /* device class */
    U32           known_svcs11_00_31; /*  */
    U32           known_svcs11_32_63; /*  */
    U32           known_svcs12_00_31; /*  */
    U32           known_svcs13_00_31; /*  */
    BOOL          authorised;         /*  */
} BTS2S_SC_UPDATE_DEV_REQ;

typedef struct
{
    U16 type; /* message type BTS2MU_SC_IO_CAPABILITY_RSP */
    BTS2S_BD_ADDR bd;
    BTS2E_SC_IO_CAPABILITY io_capability;
    U8 mitm;
    U8 bonding;
} BTS2S_SC_IO_CAPABILITY_RSP;

typedef struct
{
    U16 type; /* message type BTS2MU_SC_USER_CFM_REQ_RSP */
    BTS2S_BD_ADDR bd;
    U8 confirm;
} BTS2S_SC_USER_CFM_REQ_RSP;

typedef struct
{
    U16       type;      /* message identity */
    BTS2_UUID prof_uuid; /* profile UUID */
    U32      chnl;      /*  */
    U32      prot_id;   /*  */
    U16      sec_level; /*  */
} BTS2S_SC_REG_REQ;

typedef struct
{
    U16 type;    /* message identity */
    U32 prot_id; /*  */
    U32 chnl;    /*  */
} BTS2S_SC_UNREG_REQ;

typedef struct
{
    U16             type;      /* message identity */
    U16             tid;       /* task ID */
    BTS2S_BD_ADDR   bd;        /* Bluetooth address */
    BTS2S_CONN_INFO conn_info; /* connection information */
} BTS2S_SC_PAIR_REQ;

typedef struct
{
    U16           type;                /* message identity */
    BTS2S_BD_ADDR bd;                  /* peer Bluetooth address */
    U32           cod;                 /* class of device */
    BOOL          added_to_sc_db_list; /* whether or not add in SC manager */
    U8            res;                 /* result */
} BTS2S_SC_PAIR_IND;

typedef struct
{
    U16           type;                /* message identity */
    BOOL          added_to_sc_db_list; /* whether or not add in SC manager */
    BTS2S_BD_ADDR bd;                  /* peer Bluetooth address */
    U32           cod;                 /* class of device */
    U8            res;                 /* pair result */
} BTS2S_SC_PAIR_CFM;

typedef struct
{
    U16          type; /* message identity */
    U16          tid;  /* task ID */
    BTS2S_BD_ADDR bd;   /* unpair Bluetooth address */
} BTS2S_SC_UNPAIR_REQ;

typedef struct
{
    U16           type; /* message identity */
    U8            res;  /* opetation result */
    BTS2S_BD_ADDR bd;   /* Bluetooth address */
} BTS2S_SC_UNPAIR_CFM;

typedef struct
{
    U16            type;     /* message identity */
    U32            cod;      /* class of device */
    BTS2S_BD_ADDR  bd;       /* Bluetooth address */
    BTS2S_DEV_NAME dev_name; /* devcie name */
} BTS2S_SC_PASSKEY_IND;

typedef struct
{
    U16           type; /* message identity */
    BOOL          acpt; /* accept or reject */
    BTS2S_BD_ADDR bd;   /* Bluetooth address  */
    U8            pass_key_len; /* actual pass key length */
    U8            pass_key[HCI_MAX_PIN_LEN]; /* pass key string */
    BOOL          add_dev; /* if TRUE the device is added to the security manager's device database */
    BOOL          authorised; /* TRUE if authorization is automatic granted, FALSE if not. */
} BTS2S_SC_PASSKEY_RSP;

typedef struct
{
    U16 type;           /* message identity */
    U16 tid;            /* task ID */
    U8  bts2s_secu_mode; /* security mode from 1 to 4 */
} BTS2S_SC_SET_SECU_LEVEL_REQ;

typedef struct
{
    U16           type;    /* message identity */
    U16           tid;     /* task ID */
    BOOL          enc_enb; /* encryption mode, 1 enable, 0 disable */
    BTS2S_BD_ADDR bd;      /* Bluetooth address */
} BTS2S_SC_ENCRYPTION_REQ;

typedef struct
{
    U16           type;     /* message identity */
    U8            res;      /* result */
    BOOL          enc_enbd; /* encryption mode, 1 enable, 0 disable */
    BTS2S_BD_ADDR bd;       /* Bluetooth address */
} BTS2S_SC_ENCRYPTION_CFM;

typedef struct
{
    U16 type; /* message identity */
    U8   res; /* result */
} BTS2S_SC_SET_SECU_LEVEL_CFM;

typedef struct
{
    U16            type;     /* message identity */
    BTS2S_BD_ADDR  bd;       /* peer Bluetooth address */
    BTS2S_DEV_NAME dev_name; /* device name */
    U16            svc_id;   /* service id */
} BTS2S_SC_AUTHORISE_IND;

typedef struct
{
    U16           type;       /* message identity */
    BOOL          authorised; /* whether authorize or not */
    BTS2S_BD_ADDR bd;         /* Bluetooth address */
} BTS2S_SC_AUTHORISE_RSP;

typedef struct
{
    U16           type;       /* message identity */
    BTS2S_BD_ADDR bd;         /* Bluetooth address */
    BOOL          authorised; /* TRUE (trusted) or FALSE (untrusted) */
} BTS2S_SC_SET_TRUST_LEVEL_REQ;

typedef struct
{
    U16           type;       /* message identity */
    BTS2S_BD_ADDR bd;         /* Bluetooth address */
    BOOL          authorised; /* authorise or not */
} BTS2S_SC_UPDATE_TRUST_LEVEL_IND;

/* used for 2.1 spec */
typedef struct
{
    U16 type;         /* message identity BTS2MU_SC_IO_CAPABILITY_REQ_IND */
    BTS2S_BD_ADDR bd; /* Bluetooth address */
} BTS2S_SC_IO_CAPABILITY_REQ_IND;

typedef struct
{
    U16 type;         /* message identity BTS2MU_SC_IO_CAPABILITY_REQ_IND */
    BTS2S_BD_ADDR bd; /* Bluetooth address */
    BTS2E_SC_IO_CAPABILITY io_capability; /* IO capability of the remote device */
    U8 oob_data_present; /* If there is any OOB data present on the remote device */
    BTS2E_AUTH_REQUIREMENTS auth_requirements; /* Remote authentication requirements */
} BTS2S_SC_REMOTE_IO_CAPABILITY_IND;

typedef struct
{
    U16 type;         /* message identity */
    BTS2S_BD_ADDR bd; /* Bluetooth address */
} BTS2S_SC_RMT_OOB_DATA_CFM;

typedef struct
{
    U16 type;         /* message identity */
    U8 st;
    BTS2S_BD_ADDR bd; /* Bluetooth address */
} BTS2S_SC_SSP_COMP_CFM;

typedef struct
{
    U16           type;    /* message identity */
    BTS2S_BD_ADDR bd;      /* Bluetooth address */
    U32           num_val;
} BTS2S_SC_USER_CONF_CFM;

typedef struct
{
    U16           type; /* message identity */
    BTS2S_BD_ADDR bd;   /* Bluetooth address */
    U32 passkey;
} BTS2S_SC_PASSKEY_NOTIFI;

typedef struct
{
    U16           type; /* message identity */
    BTS2S_BD_ADDR bd;   /* Bluetooth address */
} BTS2S_SC_USER_PASSKY_CFM;


typedef struct
{
    U16 type; /* message identity */
    U16 tid; /* task ID */
    BTS2S_BD_ADDR bd;  /* bd address */
} BTS2S_SC_RD_PAIRED_DEV_KEY_REQ;

typedef struct
{
    U16 type; /* message identity */
    BTS2E_RESULT_CODE res;  /* result */
    BTS2S_BD_ADDR bd;  /* task ID */
    BTS2_LINKKEY link_key; /* link key */
} BTS2S_SC_RD_PAIRED_DEV_KEY_CFM;

typedef struct
{
    U16 type; /* message identity */
    BTS2S_BD_ADDR bd;
    U8 res;
} BTS2S_SC_PAIRED_DEV_DELETE_KEY_CFM;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to start a pair process with the remote device.
 *
 * INPUT:
 *      U16 tid: task ID.
 *      BTS2S_BD_ADDR *bd: destination Bluetooth device address.
 *      BTS2S_CONN_INFO *conn_info: connection information.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_PASSKEY_IND with structure BTS2S_SC_PASSKEY_IND will be
 *      received as a hint to reply the pass key.
 *      Message BTS2MU_SC_PAIR_CFM with structure BTS2S_SC_PAIR_CFM will be received
 *      to inform the result.
 *
 *----------------------------------------------------------------------------*/
void sc_pair_req(U16 tid, BTS2S_BD_ADDR *bd, BTS2S_CONN_INFO *conn_info);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Remove an existing pair relationship between the remote device.
 *
 * INPUT:
 *      U16 tid: task ID.
 *      BTS2S_BD_ADDR *bd: Bluetooth device address.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      If parameter bd is NULL, it will delete all records.
 *      Message BTS2MU_SC_UNPAIR_CFM  with structure BTS2S_SC_UNPAIR_CFM will be
 *      received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void sc_unpair_req(U16 tid, BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reply a pin code request indication from the remote device.
 *
 * INPUT:
 *      BOOL acpt: TRUE to acpt the passkey req, FALSE to reject.
 *      BTS2S_BD_ADDR *bd: address of device for which a passkey is requested.
 *      U8 passkey_len: the length of the passkey. the maximum number is 16.
 *      U8 passkey[BTS2_PASSKEY_MAX_LEN]: the passkey.
 *      BOOL add_dev: if TRUE the device is added to the security manager's device database.
 *      BOOL authorised: TRUE if authorization is automatic granted, FALSE if not.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_PASSKEY_IND with structure BTS2S_SC_PASSKEY_IND will be
 *      received as a hint to call this function.
 *
 *----------------------------------------------------------------------------*/
void sc_passkey_rsp(BOOL acpt,
                    BTS2S_BD_ADDR *bd,
                    U8 passkey_len,
                    U8 passkey[HCI_MAX_PIN_LEN],
                    BOOL add_dev,
                    BOOL authorised);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Change the current security level from 1 - 4.
 *
 * INPUT:
 *      U16 tid: task ID.
 *      U8 mode: security mode 1 - 4.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void sc_set_secu_level_req(U16 tid, U8 mode);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Change the encryption mode for a specified device connection.
 *
 * INPUT:
 *      U16 tid: task ID.
 *      BTS2S_BD_ADDR *bd: Bluetooth device address.
 *      BOOL mode: encryption mode, 1 enable, 0 disable.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_ENCRYPTION_CFM with structure BTS2S_SC_ENCRYPTION_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void sc_encryption_req(U16 tid, BTS2S_BD_ADDR *bd, BOOL mode);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Response to authorize request indication.
 *
 * INPUT:
 *      BOOL authorised: whether authorize or not.
 *      BTS2S_BD_ADDR *bd: remote address.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_AUTHORISE_IND with structure BTS2S_SC_AUTHORISE_IND
 *      will be received as a hint to call this function.
 *
 *----------------------------------------------------------------------------*/
void sc_authorise_rsp(BOOL authorised, BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Read sc database.
 *
 * INPUT:
 *      U16 tid: task ID.
 *      U32 byte_max_num: Max number of reading recorder, it should be number * BTS2S_DEV_PROP.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_RD_DEV_RECORD_CFM with structure BTS2S_SC_RD_DEV_RECORD_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void sc_rd_dev_record_req(U16 tid, U32 byte_max_num);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Read the device record stored in the local device.
 *
 * INPUT:
 *      U16 tid: task ID where SC must return indication and confirm.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_SC_RD_PAIRED_DEV_RECORD_CFM with structure BTS2S_SC_RD_PAIRED_DEV_RECORD_CFM
 *      will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void sc_rd_paired_dev_record_req(U16 tid);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is called in response to BTS2MU_SC_IO_CAPABILITY_REQ_IND.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: remote side Bluetooth address.
 *      BTS2S_SC_IO_CAPABILITY io_capability: Application IO Capability.
 *      U8 mitm: MITM Protection required or not.
 *      U8 bonding: need bonding or not.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void sc_io_capability_rsp(BTS2S_BD_ADDR *bd,
                          BTS2E_SC_IO_CAPABILITY io_capability,
                          U8 mitm,
                          U8 bonding);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Reply to a User Confirmation Request and indicates that the user selected
 *      "yes" or "no".
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: remote side Bluetooth address.
 *      U8 confirm: TRUE or FALSE.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void sc_user_cfm_rsp(BTS2S_BD_ADDR *bd, U8 confirm);

void sc_ble_bt_link_key_ind(uint8_t *p_bd, uint8_t key_type, uint8_t key[LINK_KEY_SIZE]);

void sc_clean_all_link_key(void);

void sc_rd_paired_dev_link_key_req(U16 tid, BTS2S_BD_ADDR *bd);


#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
