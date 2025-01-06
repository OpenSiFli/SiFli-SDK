/**
  ******************************************************************************
  * @file   bts2_app_pbap_c.c
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

#include "bts2_app_inc.h"

#ifdef CFG_PBAP_CLT
#include "bts2_app_cardparser.h"
#include <assert.h>

#define LOG_TAG         "btapp_pbapc"
//#define DBG_LVL          LOG_LVL_INFO
#include "log.h"


#define USER_ID             "BTS2_TEST"
#define TEMP_PHONEBOOK_NAME "temp_phone_book.vcf"

typedef enum
{
    BT_PBAPC_IDLE_ST,
    BT_PBAPC_CONNING_ST,
    BT_PBAPC_CONNED_ST,
    BT_PBAPC_DISC_ST
} BT_PBAPC_ST;

typedef struct
{
    BT_PBAPC_ST pbap_clt_st;
    BT_PBAP_VCARD *pbab_vcard;
    BOOL is_valid_vcard;
    U32 elem_index;
    U16 mfs;
    BTS2S_BD_ADDR rmt_bd;
    U8 rmt_supp_repos;
    BT_PBAP_CLT_RUN_CMD curr_cmd;

    BTS2E_PBAP_PHONE_REPOSITORY curr_repos;
    BTS2E_PBAP_PHONE_BOOK curr_phonebook;

    BTS2E_PBAP_PHONE_REPOSITORY target_repos;
    BTS2E_PBAP_PHONE_BOOK target_phonebook;

    FILE *cur_file_hdl;
    U8 curr_path[PBAP_CLT_PATH_LEN];
    U32 entry_handle;
} bts2s_pbap_clt_inst_data;

typedef struct TUserData
{
    int indent;
    int inBegin, inEnd, startData;
    char *cardType;
} TUserData;

static bts2s_pbap_clt_inst_data *local_inst = NULL;
TUserData userData = {0, 0, 0, 0, NULL};
CARD_Parser vp = NULL;
int count = 0;
int pring_count = 1;
int type = BT_PBAP_CLT_VCARD_IDLE;

static const char *pbap_find_char(const char *data, int len, char ch)
{
    const char *pend = data + len;
    while (data < pend)
    {
        if (*data == ch)
        {
            return data;
        }
        data += 1;
    }
    return NULL;
}

static const char *pbap_match_blank(const char *data)
{
    if (*data == ' ' || *data == '\t' || *data == '\r' || *data == '\n')
    {
        return data;
    }
    else
    {
        return NULL;
    }
}

static const char *pbap_skip_blanks(const char *data, int len)
{
    const char *pend = data + len;
    while (data < pend)
    {
        if (pbap_match_blank(data))
        {
            data += 1;
        }
        else
        {
            return data;
        }
    }
    return NULL;
}

static char pbap_to_lower_case(char ch)
{
    if (ch >= 'A' && ch <= 'Z')
    {
        return ch - 'A' + 'a';
    }
    else
    {
        return ch;
    }
}

static const char *pbap_match_string(const char *data, int len, const char *substr)
{
    int substr_len = strlen(substr);
    if (substr_len <= len)
    {
        int i = 0;
        for (; i < substr_len; i += 1)
        {
            if (pbap_to_lower_case(data[i]) != substr[i])
            {
                return NULL;
            }
        }
        return data;
    }
    else
    {
        return NULL;
    }
}

static const char *pbap_search_substr(const char *data, int len, const char *substr)
{
    const char *found = NULL;
    int i = 0;

    for (; i < len; i += 1)
    {
        if ((found = pbap_match_string(data + i, len - i, substr)))
        {
            return found;
        }
    }

    return NULL;
}

static struct pbap_key_value_t pbap_find_key_value(const char *data, int len, const char *keystr)
{
    struct pbap_key_value_t kv = {0};
    const char *pend = data + len;
    bool key_matched = false;
    const char *value_start = 0;
    const char *value_end = 0;

continue_parse_value:

    data = pbap_skip_blanks(data, pend - data);
    if (!data)
    {
        return kv;
    }

    key_matched = (NULL != pbap_match_string(data, pend - data, keystr));

    data = pbap_find_char(data, pend - data, '=');
    if (!data)
    {
        return kv;
    }

    data = pbap_find_char(data, pend - data, '"');
    if (!data)
    {
        return kv;
    }

    data += 1;
    value_start = data;

    data = pbap_find_char(data, pend - data, '"');
    if (!data)
    {
        return kv;
    }

    value_end = data;
    data += 1;

    if (key_matched)
    {
        kv.value_start = value_start;
        kv.value_end = value_end;
        return kv;
    }

    goto continue_parse_value;

}

static void bt_pbap_clt_dump_vcard(void)
{
    LOG_D("*********************************************\n");
    LOG_D("pbap_vcard[%d]:\n", pring_count++);
    LOG_D("[name]  ");
    char *name = bmalloc(local_inst->pbab_vcard->v_name.length + 1);
    if (name)
    {
        memcpy(name, local_inst->pbab_vcard->v_name.name, local_inst->pbab_vcard->v_name.length);
        name[local_inst->pbab_vcard->v_name.length] = 0;
        LOG_D("%s", name);
        LOG_D("\n");
        bfree(name);
    }
    else
        LOG_D("out of memory");

    vcard_tel_list *tmp = local_inst->pbab_vcard->v_tel;

    while (tmp)
    {
        LOG_D("[tel]  ");
        char *name_tel = bmalloc(tmp->length + 1);
        if (name_tel)
        {
            memcpy(name_tel, tmp->tel, tmp->length);
            name_tel[tmp->length] = 0;
            tmp = tmp->next_struct;
            LOG_D("%s", name_tel);
            LOG_D("\n");
            bfree(name_tel);
        }
        else
            LOG_D("out of memory");
    }
    LOG_D("*********************************************\n");
}


static void bt_pbap_check_vcard_valid(U32 index)
{
    if ((local_inst->elem_index & index) != index)
    {
        local_inst->is_valid_vcard = FALSE;
    }
    else
    {
        local_inst->is_valid_vcard = TRUE;
    }
}

void PropHandler(void *userData, const CARD_Char *propName, const CARD_Char **params)
{
    TUserData *ud = (TUserData *) userData;
    char *pname = NULL;
    const CARD_Char **p = NULL;

    assert(ud != NULL);

    ud->inBegin = FALSE;
    ud->inEnd = FALSE;
    ud->startData = TRUE;

    if (bstricmp((char *)propName, "BEGIN") == 0)
    {
        /* begin: vcard/vcal/whatever */
        ud->inBegin = TRUE;
        ud->indent++;

        local_inst->pbab_vcard = (BT_PBAP_VCARD *)bmalloc(sizeof(BT_PBAP_VCARD));
        if (!local_inst->pbab_vcard)
        {
            USER_TRACE(">> Can't create phone-book buffer!!!!!!!!!\n");
            local_inst->is_valid_vcard = FALSE;
        }
        else
        {
            local_inst->pbab_vcard->v_tel = (vcard_tel_list *)bmalloc(sizeof(vcard_tel_list));
            BT_OOM_ASSERT(local_inst->pbab_vcard->v_tel);
            if (local_inst->pbab_vcard->v_tel)
            {
                local_inst->pbab_vcard->v_tel->next_struct = NULL;
                local_inst->pbab_vcard->v_tel->tel = NULL;
                local_inst->pbab_vcard->v_tel->num = 0;
                local_inst->is_valid_vcard = TRUE;
            }
        }
    }
    else if (bstricmp((char *)propName, "END") == 0)
    {
        /* end: vcard/vcal/whatever */
        bfree(ud->cardType);
        ud->cardType = NULL;
        ud->inEnd = TRUE;
        ud->indent--;
    }
    else
    {
        // pname = bmalloc(strlen(propName) + 1);
        // memcpy(pname, propName, strlen(propName) + 1);
        /*pname = strdup(propName);
        strupr(pname); */
        // printf("%s:\n", pname);

        if (bstricmp((char *)propName, "FN") == 0)
        {
            type = BT_PBAP_CLT_FN;
            local_inst->elem_index |= BT_PBAP_ELEM_FN;
        }
        else if (bstricmp((char *)propName, "TEL") == 0)
        {
            type = BT_PBAP_CLT_TEL;
            local_inst->elem_index |= BT_PBAP_ELEM_TEL;
        }
        else if (bstricmp((char *)propName, "PHOTO") == 0)
        {
            type = BT_PBAP_CLT_PHOTO;
        }
    }
}

void DataHandler(void *userData, const CARD_Char *data, int len)
{
    TUserData *ud = (TUserData *) userData;
    assert(ud != NULL);

    if (ud->inBegin)
    {
        /* accumulate begin data */
        if (len > 0)
        {
            ud->cardType = realloc(ud->cardType, len + 1);
            memcpy(ud->cardType, data, len);
            ud->cardType[len] = 0;
        }
    }
    else if (ud->inEnd)
    {
        if (len > 0)
        {
            ud->cardType = realloc(ud->cardType, len + 1);
            memcpy(ud->cardType, data, len);
            ud->cardType[len] = 0;
        }
        else if (ud->cardType)
        {
            bfree(ud->cardType);
            ud->cardType = NULL;

            bt_pbap_check_vcard_valid(BT_PBAP_ELEM_FN | BT_PBAP_ELEM_TEL);

            if (local_inst->is_valid_vcard)
            {
                bt_pbap_clt_dump_vcard();
                //todo 上报solution
            }
            else
            {
                LOG_E("error,no valid vcard!!!!!!\n");
            }

            bfree(local_inst->pbab_vcard->v_name.name);
            local_inst->pbab_vcard->v_name.name = NULL;

            vcard_tel_list *ptr = local_inst->pbab_vcard->v_tel;
            vcard_tel_list *last_ptr = NULL;
            while (ptr != NULL)
            {
                /* kill it and move on */
                last_ptr = ptr;
                ptr = ptr->next_struct;
                if (last_ptr->tel)
                {
                    bfree(last_ptr->tel);
                    last_ptr->tel = NULL;
                }
                bfree(last_ptr);      /* kill the block */
                last_ptr = NULL;
            }
            bfree(local_inst->pbab_vcard);
            local_inst->pbab_vcard = NULL;
            type = BT_PBAP_CLT_VCARD_IDLE;
            local_inst->elem_index = BT_PBAP_ELEM_VCARD_IDLE;
        }
    }
    else
    {
        if (ud->startData)
        {
            ud->startData = FALSE;
        }

        if (len == 0)
        {
            // printf("}\n");
        }
        else
        {
            /* output printable data */

            if (type  == BT_PBAP_CLT_TEL)
            {
                // printf("tel:");
                if (local_inst->is_valid_vcard)
                {
                    if (local_inst->pbab_vcard->v_tel->num == 0)
                    {
                        local_inst->pbab_vcard->v_tel->tel = bmalloc(len);
                        BT_OOM_ASSERT(local_inst->pbab_vcard->v_tel->tel);
                        if (local_inst->pbab_vcard->v_tel->tel)
                        {
                            local_inst->pbab_vcard->v_tel->length = len;
                            memcpy(local_inst->pbab_vcard->v_tel->tel, data, len);
                            local_inst->pbab_vcard->v_tel->num++;
                            local_inst->pbab_vcard->v_tel->next_struct = NULL;
                        }
                    }
                    else
                    {
                        vcard_tel_list *tel_tmp = (vcard_tel_list *)bmalloc(sizeof(vcard_tel_list));
                        BT_OOM_ASSERT(tel_tmp);
                        if (tel_tmp)
                        {
                            vcard_tel_list *tmp = local_inst->pbab_vcard->v_tel;
                            tel_tmp->length = len;
                            tel_tmp->tel = bmalloc(len);
                            BT_OOM_ASSERT(tel_tmp->tel);
                            if (tel_tmp->tel)
                            {
                                memcpy(tel_tmp->tel, data, len);
                                tel_tmp->next_struct = NULL;
                                while (tmp->next_struct)
                                {
                                    tmp = tmp->next_struct;
                                }
                                tmp->next_struct = tel_tmp;
                                local_inst->pbab_vcard->v_tel->num++;
                            }
                            else
                                bfree(tel_tmp);
                        }
                    }
                }
            }
            else if (type == BT_PBAP_CLT_FN)
            {
                if (local_inst->is_valid_vcard)
                {
                    local_inst->pbab_vcard->v_name.length = len;
                    local_inst->pbab_vcard->v_name.name = bmalloc(len);
                    BT_OOM_ASSERT(local_inst->pbab_vcard->v_name.name);
                    if (local_inst->pbab_vcard->v_name.name)
                        memcpy(local_inst->pbab_vcard->v_name.name, data, len);
                }
            }
        }
    }
}

void bt_parser_vcard_property(U8 *buf, U32 len, void *param)
{
    int parseErr = FALSE;
    TUserData userData = {0, 0, 0, 0, NULL};
    CARD_Parser vp = NULL;
    U32 rc = 0;

    /* allocate parser */
    vp = CARD_ParserCreate(NULL);

    /* initialize */
    CARD_SetUserData(vp, &userData);
    CARD_SetPropHandler(vp, PropHandler);
    CARD_SetDataHandler(vp, DataHandler);

    rc = CARD_Parse(vp, (const char *)buf, len, FALSE);
    if (rc != 0)
    {
        CARD_Parse(vp, NULL, 0, TRUE);
        USER_TRACE("parsing vcard complete\n");
    }
    else
    {
        USER_TRACE("Error parsing vcard\n");
    }

    /* free parser */
    CARD_ParserFree(vp);
    /* free up any remaining user data buffers */
    bfree(userData.cardType);
}

static int bt_pbapc_parse_vcard_list(const char *data, U16 dataLen)
{
    // U8 res;
    const char *pend = data + dataLen;
    const char *curr_start = NULL;
    const char *curr_end = NULL;
    const char *vcard_handle_start = NULL;
    const char *vcard_handle_end = NULL;
    const char *vcard_name_start = NULL;
    const char *vcard_name_end = NULL;
    int card_count = 0;
    struct pbap_key_value_t kv = {0};
    bt_notify_pbap_vcard_listing_item_t listing_item = {0};

    if (!data || !dataLen)
    {
        USER_TRACE("vcard_list err invalid body content");
        return 0;
    }

    data = pbap_search_substr(data, dataLen, "<vcard-listing");
    if (!data)
    {
        USER_TRACE("vcard_list cannot find vcard-listing tag");
        return 0;
    }

    data += strlen("<vcard-listing");

continue_parse_card:

    data = pbap_find_char(data, pend - data, '<');
    if (!data)
    {
        return card_count;
    }

    data += 1;

    data = pbap_skip_blanks(data, pend - data);
    if (!data)
    {
        return card_count;
    }

    data = pbap_match_string(data, pend - data, "card");
    if (!data)
    {
        return card_count;
    }

    curr_start = data + strlen("card");

    data = pbap_find_char(data, pend - data, '/');
    if (!data)
    {
        return card_count;
    }

    curr_end = data;
    data += 1;

    data  = pbap_skip_blanks(data, pend - data);
    if (!data)
    {
        return card_count;
    }

    if (*data != '>')
    {
        return card_count;
    }

    data += 1;

    kv = pbap_find_key_value(curr_start, curr_end - curr_start, "handle");
    if (!kv.value_start)
    {
        return card_count;
    }

    vcard_handle_start = kv.value_start;
    vcard_handle_end = kv.value_end;

    kv = pbap_find_key_value(curr_start, curr_end - curr_start, "name");
    if (kv.value_start)
    {
        vcard_name_start = kv.value_start;
        vcard_name_end = kv.value_end;
    }
    else
    {
        vcard_name_start = NULL;
        vcard_name_end = NULL;
    }

    listing_item.vcard_handle_len = vcard_handle_end - vcard_handle_start;

    if (listing_item.vcard_handle_len <= PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE)
    {
        bmemcpy(listing_item.vcard_handle, vcard_handle_start, listing_item.vcard_handle_len);
    }
    else
    {
        listing_item.vcard_handle_len = PBAP_MAX_VCARD_ENTRY_HANDLE_SIZE;
        bmemcpy(listing_item.vcard_handle, vcard_handle_start, listing_item.vcard_handle_len);
    }
    listing_item.vcard_handle[listing_item.vcard_handle_len] = 0; // zero terminated

    listing_item.name_len = vcard_name_end - vcard_name_start;
    if (listing_item.name_len <= PBAP_MAX_VCARD_CONTACT_NAME_SIZE)
    {
        memcpy(listing_item.vcard_name, vcard_name_start, listing_item.name_len);
    }
    else
    {
        listing_item.name_len = PBAP_MAX_VCARD_CONTACT_NAME_SIZE;
        memcpy(listing_item.vcard_name, vcard_name_start, listing_item.name_len);
    }

    listing_item.vcard_name[listing_item.name_len] = 0; // zero terminated
    bt_interface_bt_event_notify(BT_NOTIFY_PBAP, BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND,
                                 &listing_item, sizeof(bt_notify_pbap_vcard_listing_item_t));

    card_count += 1;
    USER_TRACE("vcard_list hdl:%s name:%s card_count %d ", listing_item.vcard_handle, listing_item.vcard_name, card_count);
    goto continue_parse_card;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:Setup PhoneBook Access Client
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_pbap_clt_init(bts2_app_stru *bts2_app_data)
{
    local_inst = (bts2s_pbap_clt_inst_data *)bmalloc(sizeof(bts2s_pbap_clt_inst_data));
    // Must allocate successful
    BT_ASSERT(local_inst);
    local_inst->pbap_clt_st = BT_PBAPC_IDLE_ST;
    local_inst->is_valid_vcard = FALSE;
    local_inst->elem_index = BT_PBAP_ELEM_VCARD_IDLE;
    local_inst->pbab_vcard = NULL;
    local_inst->mfs = pbap_clt_get_max_mtu();
    local_inst->rmt_supp_repos = 0;
    local_inst->curr_cmd = BT_PBAP_CLT_IDLE;

    local_inst->curr_repos = PBAP_LOCAL;
    local_inst->curr_phonebook = PBAP_PB;

    local_inst->target_repos = PBAP_UNKNOWN_REPO;
    local_inst->target_phonebook = PBAP_UNKNOWN_PHONEBOOK;

    local_inst->cur_file_hdl = NULL;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Free global instance.
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_pbap_clt_free_inst()
{
    if (local_inst != NULL)
        bfree(local_inst);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_pbap_clt_rel(bts2_app_stru *bts2_app_data, void *msg_data)
{
    U16 type = *(U16 *)msg_data;

    switch (type)
    {
    case BTS2MU_PBAP_CLT_PULL_PB_BEGIN_IND:
    {
        BTS2S_PBAP_CLT_PULL_PB_BEGIN_IND *msg = msg_data;

        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
        break;
    }

    case BTS2MU_PBAP_CLT_PULL_PB_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_PB_NEXT_IND *msg = msg_data;
        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
    }
    break;

    case BTS2MU_PBAP_CLT_PULL_VCARD_BEGIN_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND *msg = msg_data;
        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
        break;
    }

    case BTS2MU_PBAP_CLT_PULL_VCARD_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND *msg = msg_data;
        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
        break;
    }

    case BTS2MU_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND *msg = msg_data;
        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
        break;
    }

    case BTS2MU_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_NEXT_IND *msg = msg_data;
        if (msg->data != NULL)
        {
            bfree(msg->data);
        }
        break;
    }
    default:
        break;
    }
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
static U8 bt_pbapc_get_curr_path(U8 *curr_path)
{
    strcpy((char *)curr_path, "test\\pbap");
    if (local_inst->curr_repos == PBAP_LOCAL)
    {
        strcat((char *)curr_path, "\\telecom");
    }
    else if (local_inst->curr_repos == PBAP_SIM1)
    {
        strcat((char *)curr_path, "\\SIM1\\telecom");
    }
    else
    {
        return FALSE;
    }

    if (local_inst->curr_phonebook == PBAP_PB)
    {
        strcat((char *)curr_path, "\\pb\\");
    }
    else if (local_inst->curr_phonebook == PBAP_ICH)
    {
        strcat((char *)curr_path, "\\ich\\");
    }
    else if (local_inst->curr_phonebook == PBAP_OCH)
    {
        strcat((char *)curr_path, "\\och\\");
    }
    else if (local_inst->curr_phonebook == PBAP_MCH)
    {
        strcat((char *)curr_path, "\\mch\\");
    }
    else if (local_inst->curr_phonebook == PBAP_CCH)
    {
        strcat((char *)curr_path, "\\cch\\");
    }
    else
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * @brief Create PBAP connection to a Phone Book Server (PSE) server on a remote device.
 * If the server requires authentication, a PBAP_SUBEVENT_AUTHENTICATION_REQUEST is emitted, which
 * can be answered with pbap_authentication_password(..).
 * The status of PBAP connection establishment is reported via PBAP_SUBEVENT_CONNECTION_OPENED event,
 * i.e. on success status field is set to ERROR_CODE_SUCCESS.
 *
 * @param addr
 * @return status ERROR_CODE_SUCCESS on success, otherwise BTSTACK_MEMORY_ALLOC_FAILED if PBAP or GOEP connection already exists.
 */

bt_err_t bt_pbap_client_connect(BTS2S_BD_ADDR *bd, BOOL auth_flag)
{
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->pbap_clt_st == BT_PBAPC_IDLE_ST)
    {
#if 1
        pbap_clt_conn_req(pbap_clt_get_max_mtu(),
                          auth_flag,
                          bd,
                          NULL);
#else
        //和小米1140不行；1170获取第二个包失败
        pbap_clt_conn_req(1180,
                          FALSE,
                          bd,
                          NULL);

#endif
        ret = BT_EOK;
        USER_TRACE("-- start to connect with auth...\n");
    }
    else
    {
        USER_TRACE("-- State wrong\n");
    }
    return ret;
}

bt_err_t bt_pbap_client_disconnect(BTS2S_BD_ADDR *bd)
{
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->pbap_clt_st == BT_PBAPC_CONNED_ST)
    {
        pbap_clt_disc_req();
        ret = BT_EOK;
        USER_TRACE(">> Request to disconnect\n");
    }
    else
    {
        USER_TRACE(">> State error\n");
    }
    return ret;
}

// #define FILTER_TEST PBAP_FILTER_VERSION | PBAP_FILTER_FN | PBAP_FILTER_N | pbap_filter_tel

bt_err_t bt_pbap_client_pull_pb(BTS2E_PBAP_PHONE_REPOSITORY repos, U8 phone_book, U8 max_size)
{
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->curr_cmd == BT_PBAP_CLT_IDLE)
    {
        pbap_clt_pull_pb_req((U8)local_inst->curr_repos,
                             (U8)local_inst->curr_phonebook,
                             0,
                             0x00,
                             PBAP_FORMAT_21,
                             max_size, //0x00
                             0x00);
        USER_TRACE(">> Download phone book\n");
        /* allocate parser */
        vp = CARD_ParserCreate(NULL);

        /* initialize */
        CARD_SetUserData(vp, &userData);
        CARD_SetPropHandler(vp, PropHandler);
        CARD_SetDataHandler(vp, DataHandler);
        ret = BT_EOK;
    }
    else
    {
        USER_TRACE(">> Operate illegal\n");
    }
    return ret;
}

bt_err_t bt_pbap_client_set_pb(BTS2E_PBAP_PHONE_REPOSITORY repos, U8 phone_book)
{
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->pbap_clt_st != BT_PBAPC_CONNED_ST)
    {
        USER_TRACE("-- there isn't an OBEX Session\n");
        return ret;
    }

    if (local_inst->curr_cmd == BT_PBAP_CLT_IDLE)
    {
        if (local_inst->rmt_supp_repos & repos)
        {
            local_inst->curr_cmd = BT_PBAP_CLT_SETPHONEBOOK;
            local_inst->target_repos = repos;
            local_inst->target_phonebook = phone_book;
            pbap_clt_set_pb_req((U8)repos, phone_book);
            ret = BT_EOK;
            USER_TRACE(">> pbap set phone book\n");
        }
        else
        {
            USER_TRACE("-- Server don't support this repository\n");
        }
    }
    else
    {
        USER_TRACE("-- Another operation is running\n");
    }
    return ret;
}

bt_err_t bt_pbap_client_pull_vcard(U8 *p, U8 len)
{
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->pbap_clt_st != BT_PBAPC_CONNED_ST)
    {
        USER_TRACE("-- there isn't an OBEX Session\n");
        return ret;
    }

    if (local_inst->curr_cmd == BT_PBAP_CLT_IDLE)
    {
        U8 name[30];
        U32 name_len;
        U8 vCardExt[] = {0, '.', 0, 'v', 0, 'c', 0, 'f', 0, 0};

        local_inst->entry_handle = atoi((const char *)p);
        name_len = bstr2u((char *)name, (const char *)p);
        bmemcpy(name + name_len - 2, vCardExt, sizeof(vCardExt));
        pbap_clt_pull_vcard_req(name,
                                0x84,
                                0,
                                PBAP_FORMAT_21);
        local_inst->curr_cmd = BT_PBAP_CLT_PULLVCARD;
        ret = BT_EOK;
    }
    else
    {
        USER_TRACE("-- Another operation is running\n");
    }
    return ret;
}

bt_err_t bt_pbap_client_pull_vcard_list(bts2_app_stru *bts2_app_data)
{
    USER_TRACE(">> pull vcard list\n");
    bt_err_t ret = BT_ERROR_STATE;
    if (local_inst->pbap_clt_st != BT_PBAPC_CONNED_ST)
    {
        USER_TRACE("-- there isn't an OBEX Session\n");
        return ret;
    }
#if 1
    char *str = "10086";

    pbap_clt_pull_vcard_list_req(PBAP_ORDER_IDX,
                                 PBAP_UNKNOWN_PHONEBOOK,
                                 PBAP_SEARCH_NUMBER,
                                 (U8 *)str,
                                 strlen(str),
                                 0xFFFF, 0x00);
    ret = BT_EOK;
#else
    pbap_clt_pull_vcard_list_req(PBAP_ORDER_ALPHA,
                                 PBAP_UNKNOWN_PHONEBOOK,
                                 PBAP_SEARCH_ATTR_DEFAULT,
                                 NULL,
                                 0,
                                 0x00, 0x00);
#endif
    return ret;
}

bt_err_t bt_pbap_client_get_name_by_number(char *phone_number, U16 phone_len)
{
    bt_err_t ret = BT_ERROR_STATE;

    if (local_inst->pbap_clt_st != BT_PBAPC_CONNED_ST)
    {
        USER_TRACE("-- there isn't an OBEX Session\n");
        return ret;
    }

    pbap_clt_pull_vcard_list_req(PBAP_ORDER_IDX,
                                 PBAP_UNKNOWN_PHONEBOOK,
                                 PBAP_SEARCH_NUMBER,
                                 (U8 *)phone_number,
                                 phone_len,
                                 0xFFFF, 0x00);
    ret = BT_EOK;
    return ret;
}

bt_err_t bt_pbap_client_auth(U8 *password, U8 len)
{
    bt_err_t ret = BT_EOK;
    pbap_clt_auth_rsp((const U8 *)password, len, USER_ID);
    return ret;
}

/**
 * The vCard-listing object is an XML object and is encoded in UTF-8
 * by default. The default character set may be omitted.
 *
 * The vCard-Listing object is defined according to the following DTD:
 *
 * <!DTD for the PBAP vCard-Listing Object-->
 *
 * <!ELEMENT vcard-listing ( card ) * >
 * <!ATTLIST vcard-listing version CDATA #FIXED "1.0">
 *
 * <!ELEMENT card EMPTY>
 * <!ATTLIST card
 *  handle CDATA #REQUIRED
 *  name CDATA #IMPLIED >
 *
 * <?xml version="1.0"?>
 * <!DOCTYPE vcard-listing SYSTEM "vcard-listing.dtd">
 *
 * <vCard-listing version="1.0">
 *  <card handle = "0.vcf" name = "Miyajima;?Andy"/>
 *  <card handle = "1.vcf" name = "Poujade;Guillaume" />
 *  <card handle = "2.vcf" name = "Hung;Scott" />
 * </vCard-listing>
 *
 * The name property of the vCard-listing DTD has the
 * same definition as the name property of a vCard -
 * i.e., the N property. Therefore, the format to be
 * used is the property structure "LastName;FirstName;MiddleName;Prefix;Suffix"
 *
 */

void bt_pbapc_hdl_vcardlist(BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND *msg)
{
    if (msg->data != NULL)
    {
        // USER_TRACE("vcard_list msg type: 0x%2x  pbook_size:0x%2x totalLength:0x%2x",msg->type,msg->pbook_size,msg->totalLength);
        // USER_TRACE("vcard_list msg data_len: 0x%2x  data:%s more_data:0x%2x",msg->dataLen,msg->data,msg->more_data);
        bt_pbapc_parse_vcard_list((const char *)msg->data, msg->dataLen);
        bfree(msg->data);
    }

    if (msg->more_data)
    {
        pbap_clt_pull_vcard_list_next_req();
        INFO_TRACE(">> pull next packet\n");
    }
}

void bt_pbapc_hdl_phone_book_entry(BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND *msg)
{

    const char *data = (char *)msg->data;
    U16 len = msg->dataLen;
    const char *pend = data + len;
    const char *curr_start = NULL;
    const char *curr_end = NULL;

    if (!data || !len)
    {
        // TRACE(3, "%s invalid body content %p len %d", __func__, data, len);
        return;
    }

    data = pbap_search_substr(data, pend - data, "begin:vcard");
    if (!data)
    {
        // TRACE(1, "%s cannot find vcard begin", __func__);
        return;
    }

    data += strlen("begin:vcard");
    curr_start = data;

    data = pbap_search_substr(data, pend - data, "end:vcard");
    if (!data)
    {
        // TRACE(1, "%s cannot find vard end", __func__);
        return;
    }

    curr_end = data;
    data += strlen("end:vcard");

    return;
}

void bt_pbap_clt_hdl_msg(bts2_app_stru *bts2_app_data)
{
    U16 msg_type;

    msg_type = *(U16 *)bts2_app_data->recv_msg;
    switch (msg_type)
    {
    case BTS2MU_PBAP_CLT_CONN_CFM:
    {
        BTS2S_PBAP_CLT_CONN_CFM *msg;
        msg = (BTS2S_PBAP_CLT_CONN_CFM *)bts2_app_data->recv_msg;
        if (msg->res == PBAPC_SUCCESS)
        {
            local_inst->pbap_clt_st = BT_PBAPC_CONNED_ST;
            local_inst->mfs = msg->mfs;
            local_inst->rmt_supp_repos = msg->supp_repos;
            bt_pbap_client_set_pb(PBAP_LOCAL, PBAP_PB);

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_PBAP;
            profile_state.res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_PBAP, BT_NOTIFY_PBAP_PROFILE_CONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));
            USER_TRACE(">> Pbap client connect success\n");
        }
        else
        {
            local_inst->pbap_clt_st = BT_PBAPC_IDLE_ST;

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_PBAP;
            profile_state.res = msg->res;
            bt_interface_bt_event_notify(BT_NOTIFY_PBAP, BT_NOTIFY_PBAP_PROFILE_DISCONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));
            INFO_TRACE(">> Connect fail, result = %d\n", msg->res);
        }
        break;
    }
    case BTS2MU_PBAP_CLT_DISC_IND:
    {
        BTS2S_PBAP_CLT_DISC_IND *msg;

        msg = (BTS2S_PBAP_CLT_DISC_IND *)bts2_app_data->recv_msg;

        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_PBAP;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_PBAP, BT_NOTIFY_PBAP_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));

        local_inst->pbap_clt_st = BT_PBAPC_IDLE_ST;
        USER_TRACE(">> Connection disconnect indcation\n");
        break;
    }
    case BTS2MU_PBAP_CLT_AUTH_IND:
    {
        INFO_TRACE(">> BTS2MU_PBAP_CLT_AUTH_IND\n");
        // bt_pbap_client_auth(bts2_app_data);
        break;
    }
    case BTS2MU_PBAP_CLT_SET_PB_CFM:
    {
        BTS2S_PBAP_CLT_SET_PB_CFM *msg;

        msg = (BTS2S_PBAP_CLT_SET_PB_CFM *)bts2_app_data->recv_msg;
        local_inst->curr_cmd = BT_PBAP_CLT_IDLE;
        if (msg->res == PBAPC_SUCCESS)
        {
            local_inst->curr_repos = local_inst->target_repos;
            local_inst->curr_phonebook = local_inst->target_phonebook;
            USER_TRACE(">> pbap set phonebook success\n");
        }
        else
        {
            USER_TRACE(">> pbap set phonebook failed\n");
        }
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_PB_BEGIN_IND:
    {
        BTS2S_PBAP_CLT_PULL_PB_BEGIN_IND *msg;

        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_PB_BEGIN_IND\n");
        msg = (BTS2S_PBAP_CLT_PULL_PB_BEGIN_IND *)bts2_app_data->recv_msg;
        if (msg->data)
        {
            CARD_Parse(vp, (const char *)msg->data + msg->dataOffset, msg->dataLen, FALSE);
            bfree(msg->data);
        }

        if (msg->more_data)
        {
            pbap_clt_pull_pb_next_req();
            INFO_TRACE(">> pull next packet\n");
        }
        else
        {
            INFO_TRACE(">> no more packet data\n");
        }
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_PB_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_PB_NEXT_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_PB_NEXT_IND *)bts2_app_data->recv_msg;
        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_PB_NEXT_IND\n");
        if (msg->data)
        {
            if (msg->more_data)
            {
                CARD_Parse(vp, (const char *)msg->data + msg->dataOffset, msg->dataLen, FALSE);
            }
            else
            {
                CARD_Parse(vp, (const char *)msg->data + msg->dataOffset, msg->dataLen, TRUE);
            }
        }
        if (msg->more_data)
        {
            pbap_clt_pull_pb_next_req();
        }
        bfree(msg->data);
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_PB_COMPLETE_IND:
    {
        BTS2S_PBAP_CLT_PULL_CMPT_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_CMPT_IND *)bts2_app_data->recv_msg;
        local_inst->curr_cmd = BT_PBAP_CLT_IDLE;
        if (msg->res == PBAPC_SUCCESS)
        {
            USER_TRACE(">> Pbap get phonebook complete\n");
        }
        else
        {
            USER_TRACE(">> Pbap get phonebook failed\n");
        }
        count = 0;
        pring_count = 1;
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_BEGIN_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_VCARD_BEGIN_IND *)bts2_app_data->recv_msg;
        bt_pbapc_hdl_phone_book_entry(msg);
        if (msg->more_data)
        {
            pbap_clt_pull_vcard_next_req();
            USER_TRACE(">> Pull phone book entry\n");
        }
        bfree(msg->data);
        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_VCARD_BEGIN_IND\n");
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_NEXT_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_VCARD_NEXT_IND *)bts2_app_data->recv_msg;
        if (local_inst->cur_file_hdl != NULL && msg->data != NULL)
        {
            fwrite(msg->data + msg->dataOffset, 1, msg->dataLen, local_inst->cur_file_hdl);
        }
        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_VCARD_NEXT_IND\n");
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_COMPLETE_IND:
    {
        BTS2S_PBAP_CLT_PULL_CMPT_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_CMPT_IND *)bts2_app_data->recv_msg;
        local_inst->curr_cmd = BT_PBAP_CLT_IDLE;
        if (msg->res == PBAPC_SUCCESS)
        {
            USER_TRACE(">> Get vCard success\n");
        }
        else
        {

            USER_TRACE(">> Get vCard fail\n");
        }
        if (local_inst->cur_file_hdl != NULL)
        {
            fclose(local_inst->cur_file_hdl);
            local_inst->cur_file_hdl = NULL;
        }
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND:
    {
        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND \n");
        bt_pbapc_hdl_vcardlist((BTS2S_PBAP_CLT_PULL_VCARD_LIST_BEGIN_IND *)bts2_app_data->recv_msg);
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND:
    {
        BTS2S_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND *msg;
        msg = (BTS2S_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND *)bts2_app_data->recv_msg;

        if (msg->data != NULL)
        {
            // USER_TRACE("vcard_list msg type: 0x%2x  pbook_size:0x%2x totalLength:0x%2x",msg->type,msg->pbook_size,msg->totalLength);
            // USER_TRACE("vcard_list msg data_len: 0x%2x  data:%s more_data:0x%2x",msg->dataLen,msg->data,msg->more_data);
            bt_pbapc_parse_vcard_list((const char *)msg->data, msg->dataLen);
            bfree(msg->data);
        }

        if (msg->more_data)
        {
            pbap_clt_pull_vcard_list_next_req();
            INFO_TRACE(">> pull next packet\n");
        }
        INFO_TRACE(">> BTS2MU_PBAP_CLT_PULL_VCARD_LIST_NEXT_IND \n");
        break;
    }
    case BTS2MU_PBAP_CLT_PULL_VCARD_LIST_COMPLETE_IND:
    {
        BTS2S_PBAP_CLT_PULL_CMPT_IND *msg;

        msg = (BTS2S_PBAP_CLT_PULL_CMPT_IND *)bts2_app_data->recv_msg;
        local_inst->curr_cmd = BT_PBAP_CLT_IDLE;
        if (msg->res == PBAPC_SUCCESS)
        {
            USER_TRACE(">> Get vCard list success\n");
        }
        else
        {
            USER_TRACE(">> Get vCard list fail, reslut=%d\n", msg->res);
        }
        bt_interface_bt_event_notify(BT_NOTIFY_PBAP, BT_NOTIFY_PBAP_VCARD_LIST_CMPL,
                                     &msg->res, sizeof(uint8_t));
        break;
    }
    case BTS2MU_PBAP_CLT_ABORT_CFM:
    {
        BTS2S_PBAP_CLT_ABORT_CFM *msg;

        msg = (BTS2S_PBAP_CLT_ABORT_CFM *)bts2_app_data->recv_msg;
        local_inst->curr_cmd = BT_PBAP_CLT_IDLE;
        INFO_TRACE(">> BTS2MU_PBAP_CLT_ABORT_CFM = %d\n", msg->res);
        break;
    }
    default:
    {
        INFO_TRACE(">> Unhandle message %d\n", msg_type);
        break;
    }
    }
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
