/*
 * HLR/AuC testing gateway for hostapd EAP-SIM/AKA database/authenticator
 * Copyright (c) 2005-2007, 2012-2017, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 *
 * This is an example implementation of the EAP-SIM/AKA database/authentication
 * gateway interface to HLR/AuC. It is expected to be replaced with an
 * implementation of SS7 gateway to GSM/UMTS authentication center (HLR/AuC) or
 * a local implementation of SIM triplet and AKA authentication data generator.
 *
 * hostapd will send SIM/AKA authentication queries over a UNIX domain socket
 * to and external program, e.g., this hlr_auc_gw. This interface uses simple
 * text-based format:
 *
 * EAP-SIM / GSM triplet query/response:
 * SIM-REQ-AUTH <IMSI> <max_chal>
 * SIM-RESP-AUTH <IMSI> Kc1:SRES1:RAND1 Kc2:SRES2:RAND2 [Kc3:SRES3:RAND3]
 * SIM-RESP-AUTH <IMSI> FAILURE
 * GSM-AUTH-REQ <IMSI> RAND1:RAND2[:RAND3]
 * GSM-AUTH-RESP <IMSI> Kc1:SRES1:Kc2:SRES2[:Kc3:SRES3]
 * GSM-AUTH-RESP <IMSI> FAILURE
 *
 * EAP-AKA / UMTS query/response:
 * AKA-REQ-AUTH <IMSI>
 * AKA-RESP-AUTH <IMSI> <RAND> <AUTN> <IK> <CK> <RES>
 * AKA-RESP-AUTH <IMSI> FAILURE
 *
 * EAP-AKA / UMTS AUTS (re-synchronization):
 * AKA-AUTS <IMSI> <AUTS> <RAND>
 *
 * IMSI and max_chal are sent as an ASCII string,
 * Kc/SRES/RAND/AUTN/IK/CK/RES/AUTS as hex strings.
 *
 * An example implementation here reads GSM authentication triplets from a
 * text file in IMSI:Kc:SRES:RAND format, IMSI in ASCII, other fields as hex
 * strings. This is used to simulate an HLR/AuC. As such, it is not very useful
 * for real life authentication, but it is useful both as an example
 * implementation and for EAP-SIM/AKA/AKA' testing.
 *
 * For a stronger example design, Milenage and GSM-Milenage algorithms can be
 * used to dynamically generate authenticatipn information for EAP-AKA/AKA' and
 * EAP-SIM, respectively, if Ki is known.
 *
 * SQN generation follows the not time-based Profile 2 described in
 * 3GPP TS 33.102 Annex C.3.2. The length of IND is 5 bits by default, but this
 * can be changed with a command line options if needed.
 */

#include "includes.h"
#include "rtthread.h"

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
#include <sys/un.h>
#endif
#ifdef CONFIG_SQLITE
#include <sqlite3.h>
#endif /* CONFIG_SQLITE */

#include "utils/common.h"
#include "crypto/milenage.h"
#include "crypto/random.h"
#include "utils/common.h"
#include "eap_server/eap_sim_db.h"

#include <string.h>
#include <cli.h>
#include <cli_utils.h>

static const char *default_socket_path = "/tmp/hlr_auc_gw.sock";
static const char *socket_path;
static int serv_sock       = -1;
#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
static char *milenage_file = NULL;
#endif
static int update_milenage = 0;
static int sqn_changes     = 0;
static int ind_len         = 5;
static int stdout_debug    = 1;

/* GSM triplets */
struct gsm_triplet
{
    struct gsm_triplet *next;
    char imsi[20];
    u8 kc[8];
    u8 sres[4];
    u8 _rand[16];
};

static struct gsm_triplet *gsm_db = NULL, *gsm_db_pos = NULL;

/* OPc and AMF parameters for Milenage (Example algorithms for AKA). */
struct milenage_parameters
{
    struct milenage_parameters *next;
    char imsi[20];
    u8 ki[16];
    u8 opc[16];
    u8 amf[2];
    u8 sqn[6];
    int set;
    size_t res_len;
};

static struct milenage_parameters *milenage_db = NULL;

#define EAP_SIM_MAX_CHAL 3

#define EAP_AKA_RAND_LEN    16
#define EAP_AKA_AUTN_LEN    16
#define EAP_AKA_AUTS_LEN    14
#define EAP_AKA_RES_MIN_LEN 4
#define EAP_AKA_RES_MAX_LEN 16
#define EAP_AKA_IK_LEN      16
#define EAP_AKA_CK_LEN      16

#ifdef CONFIG_SQLITE

static sqlite3 *sqlite_db = NULL;
static struct milenage_parameters db_tmp_milenage;

static int db_table_exists(sqlite3 *db, const char *name)
{
    char cmd[128];
    os_snprintf(cmd, sizeof(cmd), "SELECT 1 FROM %s;", name);
    return sqlite3_exec(db, cmd, NULL, NULL, NULL) == SQLITE_OK;
}

static int db_table_create_milenage(sqlite3 *db)
{
    char *err = NULL;
    const char *sql =
        "CREATE TABLE milenage("
        "  imsi INTEGER PRIMARY KEY NOT NULL,"
        "  ki CHAR(32) NOT NULL,"
        "  opc CHAR(32) NOT NULL,"
        "  amf CHAR(4) NOT NULL,"
        "  sqn CHAR(12) NOT NULL,"
        "  res_len INTEGER"
        ");";

    PRINTF("Adding database table for milenage information\r\n");
    if (sqlite3_exec(db, sql, NULL, NULL, &err) != SQLITE_OK)
    {
        PRINTF("SQLite error: %s\r\n", err);
        sqlite3_free(err);
        return -1;
    }

    return 0;
}

static sqlite3 *db_open(const char *db_file)
{
    sqlite3 *db;

    if (sqlite3_open(db_file, &db))
    {
        PRINTF("Failed to open database %s: %s\r\n", db_file, sqlite3_errmsg(db));
        sqlite3_close(db);
        return NULL;
    }

    if (!db_table_exists(db, "milenage") && db_table_create_milenage(db) < 0)
    {
        sqlite3_close(db);
        return NULL;
    }

    return db;
}

static int get_milenage_cb(void *ctx, int argc, char *argv[], char *col[])
{
    struct milenage_parameters *m = ctx;
    int i;

    m->set = 1;

    for (i = 0; i < argc; i++)
    {
        if (os_strcmp(col[i], "ki") == 0 && argv[i] && hexstr2bin(argv[i], m->ki, sizeof(m->ki)))
        {
            PRINTF("Invalid ki value in database\r\n");
            return -1;
        }

        if (os_strcmp(col[i], "opc") == 0 && argv[i] && hexstr2bin(argv[i], m->opc, sizeof(m->opc)))
        {
            PRINTF("Invalid opcvalue in database\r\n");
            return -1;
        }

        if (os_strcmp(col[i], "amf") == 0 && argv[i] && hexstr2bin(argv[i], m->amf, sizeof(m->amf)))
        {
            PRINTF("Invalid amf value in database\r\n");
            return -1;
        }

        if (os_strcmp(col[i], "sqn") == 0 && argv[i] && hexstr2bin(argv[i], m->sqn, sizeof(m->sqn)))
        {
            PRINTF("Invalid sqn value in database\r\n");
            return -1;
        }

        if (os_strcmp(col[i], "res_len") == 0 && argv[i])
        {
            m->res_len = atoi(argv[i]);
        }
    }

    return 0;
}

static struct milenage_parameters *db_get_milenage(const char *imsi_txt)
{
    char cmd[128];
    unsigned long long imsi;

    os_memset(&db_tmp_milenage, 0, sizeof(db_tmp_milenage));
    imsi = atoll(imsi_txt);
    os_snprintf(db_tmp_milenage.imsi, sizeof(db_tmp_milenage.imsi), "%llu", imsi);
    os_snprintf(cmd, sizeof(cmd), "SELECT * FROM milenage WHERE imsi=%llu;", imsi);
    if (sqlite3_exec(sqlite_db, cmd, get_milenage_cb, &db_tmp_milenage, NULL) != SQLITE_OK)
        return NULL;

    if (!db_tmp_milenage.set)
        return NULL;
    return &db_tmp_milenage;
}

static int db_update_milenage_sqn(struct milenage_parameters *m)
{
    char cmd[128], val[13], *pos;

    if (sqlite_db == NULL)
        return 0;

    pos = val;
    pos += wpa_snprintf_hex(pos, sizeof(val), m->sqn, 6);
    *pos = '\0';
    os_snprintf(cmd, sizeof(cmd), "UPDATE milenage SET sqn='%s' WHERE imsi=%s;", val, m->imsi);
    if (sqlite3_exec(sqlite_db, cmd, NULL, NULL, NULL) != SQLITE_OK)
    {
        PRINTF("Failed to update SQN in database for IMSI %s\r\n", m->imsi);
        return -1;
    }
    return 0;
}

#endif /* CONFIG_SQLITE */

static int open_socket(const char *path)
{
#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    struct sockaddr_un addr;
    int s;

    s = socket(PF_UNIX, SOCK_DGRAM, 0);
    if (s < 0)
    {
        perror("socket(PF_UNIX)");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    os_strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("hlr-auc-gw: bind(PF_UNIX)");
        lwip_close(s);
        return -1;
    }
    return s;
#else

    return 0;
#endif
}

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)

static int read_gsm_triplets2(const char *fname)
{
    FILE *f;
    char buf[200], *pos, *pos2;
    struct gsm_triplet *g = NULL;
    int line, ret = 0;

    if (fname == NULL)
        return -1;

    f = fopen(fname, "r");
    if (f == NULL)
    {
        PRINTF("Could not open GSM triplet data file '%s'\n", fname);
        return -1;
    }

    line = 0;
    while (fgets(buf, sizeof(buf), f))
    {
        line++;

        /* Parse IMSI:Kc:SRES:RAND */
        buf[sizeof(buf) - 1] = '\0';
        if (buf[0] == '#')
            continue;
        pos = buf;
        while (*pos != '\0' && *pos != '\n')
            pos++;
        if (*pos == '\n')
            *pos = '\0';
        pos = buf;
        if (*pos == '\0')
            continue;

        g = os_zalloc(sizeof(*g));
        if (g == NULL)
        {
            ret = -1;
            break;
        }

        /* IMSI */
        pos2 = NULL;
        pos  = str_token(buf, ":", &pos2);
        if (!pos || os_strlen(pos) >= sizeof(g->imsi))
        {
            PRINTF("%s:%d - Invalid IMSI\n", fname, line);
            ret = -1;
            break;
        }
        os_strlcpy(g->imsi, pos, sizeof(g->imsi));

        /* Kc */
        pos = str_token(buf, ":", &pos2);
        if (!pos || os_strlen(pos) != 16 || hexstr2bin(pos, g->kc, 8))
        {
            PRINTF("%s:%d - Invalid Kc\n", fname, line);
            ret = -1;
            break;
        }

        /* SRES */
        pos = str_token(buf, ":", &pos2);
        if (!pos || os_strlen(pos) != 8 || hexstr2bin(pos, g->sres, 4))
        {
            PRINTF("%s:%d - Invalid SRES\n", fname, line);
            ret = -1;
            break;
        }

        /* RAND */
        pos = str_token(buf, ":", &pos2);
        if (!pos || os_strlen(pos) != 32 || hexstr2bin(pos, g->_rand, 16))
        {
            PRINTF("%s:%d - Invalid RAND\n", fname, line);
            ret = -1;
            break;
        }

        g->next = gsm_db;
        gsm_db  = g;
        g       = NULL;
    }
    os_free(g);

    fclose(f);

    return ret;
}
#endif

static struct gsm_triplet *get_gsm_triplet(const char *imsi)
{
    struct gsm_triplet *g = gsm_db_pos;

    while (g)
    {
        if (strcmp(g->imsi, imsi) == 0)
        {
            gsm_db_pos = g->next;
            return g;
        }
        g = g->next;
    }

    g = gsm_db;
    while (g && g != gsm_db_pos)
    {
        if (strcmp(g->imsi, imsi) == 0)
        {
            gsm_db_pos = g->next;
            return g;
        }
        g = g->next;
    }

    return NULL;
}

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)

static int read_milenage2(const char *fname)
{
    FILE *f;
    char buf[200], *pos, *pos2;
    struct milenage_parameters *m = NULL;
    int line, ret = 0;

    if (fname == NULL)
        return -1;

    f = fopen(fname, "r");
    if (f == NULL)
    {
        PRINTF("Could not open Milenage data file '%s'\n", fname);
        return -1;
    }

    line = 0;
    while (fgets(buf, sizeof(buf), f))
    {
        line++;

        /* Parse IMSI Ki OPc AMF SQN [RES_len] */
        buf[sizeof(buf) - 1] = '\0';
        if (buf[0] == '#')
            continue;
        pos = buf;
        while (*pos != '\0' && *pos != '\n')
            pos++;
        if (*pos == '\n')
            *pos = '\0';
        pos = buf;
        if (*pos == '\0')
            continue;

        m = os_zalloc(sizeof(*m));
        if (m == NULL)
        {
            ret = -1;
            break;
        }

        /* IMSI */
        pos2 = NULL;
        pos  = str_token(buf, " ", &pos2);
        if (!pos || os_strlen(pos) >= sizeof(m->imsi))
        {
            PRINTF("%s:%d - Invalid IMSI\n", fname, line);
            ret = -1;
            break;
        }
        os_strlcpy(m->imsi, pos, sizeof(m->imsi));

        /* Ki */
        pos = str_token(buf, " ", &pos2);
        if (!pos || os_strlen(pos) != 32 || hexstr2bin(pos, m->ki, 16))
        {
            PRINTF("%s:%d - Invalid Ki\n", fname, line);
            ret = -1;
            break;
        }

        /* OPc */
        pos = str_token(buf, " ", &pos2);
        if (!pos || os_strlen(pos) != 32 || hexstr2bin(pos, m->opc, 16))
        {
            PRINTF("%s:%d - Invalid OPc\n", fname, line);
            ret = -1;
            break;
        }

        /* AMF */
        pos = str_token(buf, " ", &pos2);
        if (!pos || os_strlen(pos) != 4 || hexstr2bin(pos, m->amf, 2))
        {
            PRINTF("%s:%d - Invalid AMF\n", fname, line);
            ret = -1;
            break;
        }

        /* SQN */
        pos = str_token(buf, " ", &pos2);
        if (!pos || os_strlen(pos) != 12 || hexstr2bin(pos, m->sqn, 6))
        {
            PRINTF("%s:%d - Invalid SEQ\n", fname, line);
            ret = -1;
            break;
        }

        pos = str_token(buf, " ", &pos2);
        if (pos)
        {
            m->res_len = atoi(pos);
            if (m->res_len && (m->res_len < EAP_AKA_RES_MIN_LEN || m->res_len > EAP_AKA_RES_MAX_LEN))
            {
                PRINTF("%s:%d - Invalid RES_len\n", fname, line);
                ret = -1;
                break;
            }
        }

        m->next     = milenage_db;
        milenage_db = m;
        m           = NULL;
    }
    os_free(m);

    fclose(f);

    return ret;
}

static void update_milenage_file(const char *fname)
{
    FILE *f, *f2;
    char name[500], buf[500], *pos;
    char *end = buf + sizeof(buf);
    struct milenage_parameters *m;
    size_t imsi_len;

    f = fopen(fname, "r");
    if (f == NULL)
    {
        PRINTF("Could not open Milenage data file '%s'\n", fname);
        return;
    }

    snprintf(name, sizeof(name), "%s.new", fname);
    f2 = fopen(name, "w");
    if (f2 == NULL)
    {
        PRINTF("Could not write Milenage data file '%s'\n", name);
        fclose(f);
        return;
    }

    while (fgets(buf, sizeof(buf), f))
    {
        /* IMSI Ki OPc AMF SQN */
        buf[sizeof(buf) - 1] = '\0';

        pos = strchr(buf, ' ');
        if (buf[0] == '#' || pos == NULL || pos - buf >= 20)
            goto no_update;

        imsi_len = pos - buf;

        for (m = milenage_db; m; m = m->next)
        {
            if (strncmp(buf, m->imsi, imsi_len) == 0 && m->imsi[imsi_len] == '\0')
                break;
        }

        if (!m)
            goto no_update;

        pos = buf;
        pos += snprintf(pos, end - pos, "%s ", m->imsi);
        pos += wpa_snprintf_hex(pos, end - pos, m->ki, 16);
        *pos++ = ' ';
        pos += wpa_snprintf_hex(pos, end - pos, m->opc, 16);
        *pos++ = ' ';
        pos += wpa_snprintf_hex(pos, end - pos, m->amf, 2);
        *pos++ = ' ';
        pos += wpa_snprintf_hex(pos, end - pos, m->sqn, 6);
        *pos++ = '\n';

    no_update:
        fprintf(f2, "%s", buf);
    }

    fclose(f2);
    fclose(f);

    snprintf(name, sizeof(name), "%s.bak", fname);
    if (rename(fname, name) < 0)
    {
        perror("rename");
        return;
    }

    snprintf(name, sizeof(name), "%s.new", fname);
    if (rename(name, fname) < 0)
    {
        perror("rename");
        return;
    }
}
#endif

static struct milenage_parameters *get_milenage(const char *imsi)
{
    struct milenage_parameters *m = milenage_db;

    while (m)
    {
        if (strcmp(m->imsi, imsi) == 0)
            break;
        m = m->next;
    }

#ifdef CONFIG_SQLITE
    if (!m)
        m = db_get_milenage(imsi);
#endif /* CONFIG_SQLITE */

    return m;
}

static int sim_req_auth(char *imsi, char *resp, size_t resp_len)
{
    int count, max_chal, ret;
    char *pos;
    char *rpos, *rend;
    struct milenage_parameters *m;
    struct gsm_triplet *g;

    resp[0] = '\0';

    pos = strchr(imsi, ' ');
    if (pos)
    {
        *pos++   = '\0';
        max_chal = atoi(pos);
        if (max_chal < 1 || max_chal > EAP_SIM_MAX_CHAL)
            max_chal = EAP_SIM_MAX_CHAL;
    }
    else
        max_chal = EAP_SIM_MAX_CHAL;

    rend = resp + resp_len;
    rpos = resp;
    ret  = snprintf(rpos, rend - rpos, "SIM-RESP-AUTH %s", imsi);
    if (ret < 0 || ret >= rend - rpos)
        return -1;
    rpos += ret;

    m = get_milenage(imsi);
    if (m)
    {
        u8 _rand[16], sres[4], kc[8];
        for (count = 0; count < max_chal; count++)
        {
            if (random_get_bytes(_rand, 16) < 0)
                return -1;
            gsm_milenage(m->opc, m->ki, _rand, sres, kc);
            *rpos++ = ' ';
            rpos += wpa_snprintf_hex(rpos, rend - rpos, kc, 8);
            *rpos++ = ':';
            rpos += wpa_snprintf_hex(rpos, rend - rpos, sres, 4);
            *rpos++ = ':';
            rpos += wpa_snprintf_hex(rpos, rend - rpos, _rand, 16);
        }
        *rpos = '\0';
        return 0;
    }

    count = 0;
    while (count < max_chal && (g = get_gsm_triplet(imsi)))
    {
        if (strcmp(g->imsi, imsi) != 0)
            continue;

        if (rpos < rend)
            *rpos++ = ' ';
        rpos += wpa_snprintf_hex(rpos, rend - rpos, g->kc, 8);
        if (rpos < rend)
            *rpos++ = ':';
        rpos += wpa_snprintf_hex(rpos, rend - rpos, g->sres, 4);
        if (rpos < rend)
            *rpos++ = ':';
        rpos += wpa_snprintf_hex(rpos, rend - rpos, g->_rand, 16);
        count++;
    }

    if (count == 0)
    {
        PRINTF("No GSM triplets found for %s\r\n", imsi);
        ret = snprintf(rpos, rend - rpos, " FAILURE");
        if (ret < 0 || ret >= rend - rpos)
            return -1;
        rpos += ret;
    }

    return 0;
}

static int gsm_auth_req(char *imsi, char *resp, size_t resp_len)
{
    int count, ret;
    char *pos, *rpos, *rend;
    struct milenage_parameters *m;

    resp[0] = '\0';

    pos = os_strchr(imsi, ' ');
    if (!pos)
        return -1;
    *pos++ = '\0';

    rend = resp + resp_len;
    rpos = resp;
    ret  = os_snprintf(rpos, rend - rpos, "GSM-AUTH-RESP %s", imsi);
    if (os_snprintf_error(rend - rpos, ret))
        return -1;
    rpos += ret;

    m = get_milenage(imsi);
    if (m)
    {
        u8 _rand[16], sres[4], kc[8];
        for (count = 0; count < EAP_SIM_MAX_CHAL; count++)
        {
            if (hexstr2bin(pos, _rand, 16) != 0)
                return -1;
            gsm_milenage(m->opc, m->ki, _rand, sres, kc);
            *rpos++ = count == 0 ? ' ' : ':';
            rpos += wpa_snprintf_hex(rpos, rend - rpos, kc, 8);
            *rpos++ = ':';
            rpos += wpa_snprintf_hex(rpos, rend - rpos, sres, 4);
            pos += 16 * 2;
            if (*pos != ':')
                break;
            pos++;
        }
        *rpos = '\0';
        return 0;
    }

    PRINTF("No GSM triplets found for %s\r\n", imsi);
    ret = os_snprintf(rpos, rend - rpos, " FAILURE");
    if (os_snprintf_error(rend - rpos, ret))
        return -1;
    rpos += ret;

    return 0;
}

static void inc_sqn(u8 *sqn)
{
    u64 val, seq, ind;

    /*
     * SQN = SEQ | IND = SEQ1 | SEQ2 | IND
     *
     * The mechanism used here is not time-based, so SEQ2 is void and
     * SQN = SEQ1 | IND. The length of IND is ind_len bits and the length
     * of SEQ1 is 48 - ind_len bits.
     */

    /* Increment both SEQ and IND by one */
    val = ((u64)WPA_GET_BE32(sqn) << 16) | ((u64)WPA_GET_BE16(sqn + 4));
    seq = (val >> ind_len) + 1;
    ind = (val + 1) & ((1 << ind_len) - 1);
    val = (seq << ind_len) | ind;
    WPA_PUT_BE32(sqn, val >> 16);
    WPA_PUT_BE16(sqn + 4, val & 0xffff);
}

static int aka_req_auth(char *imsi, char *resp, size_t resp_len)
{
    /* AKA-RESP-AUTH <IMSI> <RAND> <AUTN> <IK> <CK> <RES> */
    char *pos, *end;
    u8 _rand[EAP_AKA_RAND_LEN];
    u8 autn[EAP_AKA_AUTN_LEN];
    u8 ik[EAP_AKA_IK_LEN];
    u8 ck[EAP_AKA_CK_LEN];
    u8 res[EAP_AKA_RES_MAX_LEN];
    size_t res_len;
    int ret;
    struct milenage_parameters *m;
    int failed = 0;

    m = get_milenage(imsi);
    if (m)
    {
        if (random_get_bytes(_rand, EAP_AKA_RAND_LEN) < 0)
            return -1;
        res_len = EAP_AKA_RES_MAX_LEN;
        inc_sqn(m->sqn);
#ifdef CONFIG_SQLITE
        db_update_milenage_sqn(m);
#endif /* CONFIG_SQLITE */
        sqn_changes = 1;
        if (stdout_debug)
        {
            PRINTF("AKA: Milenage with SQN=%02x%02x%02x%02x%02x%02x\r\n", m->sqn[0], m->sqn[1], m->sqn[2], m->sqn[3],
                   m->sqn[4], m->sqn[5]);
        }
        milenage_generate(m->opc, m->amf, m->ki, m->sqn, _rand, autn, ik, ck, res, &res_len);
        if (m->res_len >= EAP_AKA_RES_MIN_LEN && m->res_len <= EAP_AKA_RES_MAX_LEN && m->res_len < res_len)
            res_len = m->res_len;
    }
    else
    {
        PRINTF("Unknown IMSI: %s\r\n", imsi);
#ifdef AKA_USE_FIXED_TEST_VALUES
        PRINTF("Using fixed test values for AKA\r\n");
        memset(_rand, '0', EAP_AKA_RAND_LEN);
        memset(autn, '1', EAP_AKA_AUTN_LEN);
        memset(ik, '3', EAP_AKA_IK_LEN);
        memset(ck, '4', EAP_AKA_CK_LEN);
        memset(res, '2', EAP_AKA_RES_MAX_LEN);
        res_len = EAP_AKA_RES_MAX_LEN;
#else  /* AKA_USE_FIXED_TEST_VALUES */
        failed = 1;
#endif /* AKA_USE_FIXED_TEST_VALUES */
    }

    pos = resp;
    end = resp + resp_len;
    ret = snprintf(pos, end - pos, "AKA-RESP-AUTH %s ", imsi);
    if (ret < 0 || ret >= end - pos)
        return -1;
    pos += ret;
    if (failed)
    {
        ret = snprintf(pos, end - pos, "FAILURE");
        if (ret < 0 || ret >= end - pos)
            return -1;
        pos += ret;
        return 0;
    }
    pos += wpa_snprintf_hex(pos, end - pos, _rand, EAP_AKA_RAND_LEN);
    *pos++ = ' ';
    pos += wpa_snprintf_hex(pos, end - pos, autn, EAP_AKA_AUTN_LEN);
    *pos++ = ' ';
    pos += wpa_snprintf_hex(pos, end - pos, ik, EAP_AKA_IK_LEN);
    *pos++ = ' ';
    pos += wpa_snprintf_hex(pos, end - pos, ck, EAP_AKA_CK_LEN);
    *pos++ = ' ';
    pos += wpa_snprintf_hex(pos, end - pos, res, res_len);

    return 0;
}

static int aka_auts(char *imsi, char *resp, size_t resp_len)
{
    char *auts, *__rand;
    u8 _auts[EAP_AKA_AUTS_LEN], _rand[EAP_AKA_RAND_LEN], sqn[6];
    struct milenage_parameters *m;

    resp[0] = '\0';

    /* AKA-AUTS <IMSI> <AUTS> <RAND> */

    auts = strchr(imsi, ' ');
    if (auts == NULL)
        return -1;
    *auts++ = '\0';

    __rand = strchr(auts, ' ');
    if (__rand == NULL)
        return -1;
    *__rand++ = '\0';

    if (stdout_debug)
    {
        PRINTF("AKA-AUTS: IMSI=%s AUTS=%s RAND=%s\r\n", imsi, auts, __rand);
    }
    if (hexstr2bin(auts, _auts, EAP_AKA_AUTS_LEN) || hexstr2bin(__rand, _rand, EAP_AKA_RAND_LEN))
    {
        PRINTF("Could not parse AUTS/RAND\r\n");
        return -1;
    }

    m = get_milenage(imsi);
    if (m == NULL)
    {
        PRINTF("Unknown IMSI: %s\n", imsi);
        return -1;
    }

    if (milenage_auts(m->opc, m->ki, _rand, _auts, sqn))
    {
        PRINTF("AKA-AUTS: Incorrect MAC-S\r\n");
    }
    else
    {
        memcpy(m->sqn, sqn, 6);
        if (stdout_debug)
        {
            PRINTF(
                "AKA-AUTS: Re-synchronized: "
                "SQN=%02x%02x%02x%02x%02x%02x\r\n",
                sqn[0], sqn[1], sqn[2], sqn[3], sqn[4], sqn[5]);
        }
#ifdef CONFIG_SQLITE
        db_update_milenage_sqn(m);
#endif /* CONFIG_SQLITE */
        sqn_changes = 1;
    }

    return 0;
}

static int process_cmd(char *cmd, char *resp, size_t resp_len)
{
    if (os_strncmp(cmd, "SIM-REQ-AUTH ", 13) == 0)
        return sim_req_auth(cmd + 13, resp, resp_len);

    if (os_strncmp(cmd, "GSM-AUTH-REQ ", 13) == 0)
        return gsm_auth_req(cmd + 13, resp, resp_len);

    if (os_strncmp(cmd, "AKA-REQ-AUTH ", 13) == 0)
        return aka_req_auth(cmd + 13, resp, resp_len);

    if (os_strncmp(cmd, "AKA-AUTS ", 9) == 0)
        return aka_auts(cmd + 9, resp, resp_len);

    PRINTF("Unknown request: %s\r\n", cmd);
    return -1;
}

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)

static int process(int s)
{
    char buf[1000], resp[1000];
    struct sockaddr_un from;
    socklen_t fromlen;
    ssize_t res;

    fromlen = sizeof(from);
    res     = recvfrom(s, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);
    if (res < 0)
    {
        perror("recvfrom");
        return -1;
    }

    if (res == 0)
        return 0;

    if ((size_t)res >= sizeof(buf))
        res = sizeof(buf) - 1;
    buf[res] = '\0';

    PRINTF("Received: %s\n", buf);

    if (process_cmd(buf, resp, sizeof(resp)) < 0)
    {
        PRINTF("Failed to process request\n");
        return -1;
    }

    if (resp[0] == '\0')
    {
        PRINTF("No response\n");
        return 0;
    }

    PRINTF("Send: %s\n", resp);

    if (sendto(s, resp, os_strlen(resp), 0, (struct sockaddr *)&from, fromlen) < 0)
        perror("send");

    return 0;
}

#endif

static void cleanup(void)
{
    struct gsm_triplet *g, *gprev;
    struct milenage_parameters *m, *prev;

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    if (update_milenage && milenage_file && sqn_changes)
        update_milenage_file(milenage_file);
#endif

    g = gsm_db;
    while (g)
    {
        gprev = g;
        g     = g->next;
        os_free(gprev);
    }

    m = milenage_db;
    while (m)
    {
        prev = m;
        m    = m->next;
        os_free(prev);
    }

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    if (serv_sock >= 0)
    lwip_close(serv_sock);
    if (socket_path)
        unlink(socket_path);
#endif

#ifdef CONFIG_SQLITE
    if (sqlite_db)
    {
        sqlite3_close(sqlite_db);
        sqlite_db = NULL;
    }
#endif /* CONFIG_SQLITE */
}

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)

static void handle_term(int sig)
{
    PRINTF("Signal %d - terminate\n", sig);
    exit(0);
}

#endif

int read_gsm_triplets(const char *imsi, const char *kc, const char *sres, const char *_rand)
{
    struct gsm_triplet *g = NULL;

    g = os_zalloc(sizeof(*g));
    if (g == NULL)
    {
        return -1;
    }

    /* IMSI */
    if (os_strlen(imsi) >= sizeof(g->imsi))
    {
        PRINTF("Invalid IMSI\r\n");
        return -1;
    }
    os_strlcpy(g->imsi, imsi, sizeof(g->imsi));

    /* Kc */
    if (os_strlen(kc) != 16 || hexstr2bin(kc, g->kc, 8))
    {
        PRINTF("Invalid Kc\r\n");
        return -1;
    }

    /* SRES */
    if (os_strlen(sres) != 8 || hexstr2bin(sres, g->sres, 4))
    {
        PRINTF("Invalid SRES\r\n");
        return -1;
    }

    /* RAND */
    if (os_strlen(_rand) != 32 || hexstr2bin(_rand, g->_rand, 16))
    {
        PRINTF("Invalid RAND\r\n");
        return -1;
    }

    g->next = gsm_db;
    gsm_db  = g;
    g       = NULL;

    return 0;
}

void test_gsm_triplets(int argc, char **argv)
{
    if (argc != 5)
    {
        return;
    }

    read_gsm_triplets(argv[1], argv[2], argv[3], argv[4]);
}

int read_milenage(const char *imsi, const char *ki, const char *opc, const char *amf, const char *sqn)
{
    struct milenage_parameters *m = NULL;

    m = os_zalloc(sizeof(*m));
    if (m == NULL)
    {
        return -1;
    }

    /* IMSI */
    if (os_strlen(imsi) >= sizeof(m->imsi))
    {
        PRINTF("Invalid IMSI\r\n");
        return -1;
    }
    os_strlcpy(m->imsi, imsi, sizeof(m->imsi));

    /* Ki */
    if (os_strlen(ki) != 32 || hexstr2bin(ki, m->ki, 16))
    {
        PRINTF("Invalid Ki\r\n");
        return -1;
    }

    /* OPc */
    if (os_strlen(opc) != 32 || hexstr2bin(opc, m->opc, 16))
    {
        PRINTF("Invalid OPc\r\n");
        return -1;
    }

    /* AMF */
    if (os_strlen(amf) != 4 || hexstr2bin(amf, m->amf, 2))
    {
        PRINTF("Invalid AMF\r\n");
        return -1;
    }

    /* SQN */
    if (os_strlen(sqn) != 12 || hexstr2bin(sqn, m->sqn, 6))
    {
        PRINTF("Invalid SEQ\r\n");
        return -1;
    }

#if 0
    pos = str_token(buf, " ", &pos2);
    if (pos)
    {
        m->res_len = atoi(pos);
        if (m->res_len && (m->res_len < EAP_AKA_RES_MIN_LEN || m->res_len > EAP_AKA_RES_MAX_LEN))
        {
            PRINTF("%s:%d - Invalid RES_len\n", fname, line);
            ret = -1;
            break;
        }
    }
#endif

    m->next     = milenage_db;
    milenage_db = m;
    m           = NULL;

    return 0;
}

void test_milenage(int argc, char **argv)
{
    if (argc != 6)
    {
        return;
    }

    read_milenage(argv[1], argv[2], argv[3], argv[4], argv[5]);
}

static struct cli_command hlr_cli[] = {
    {"wlan-read-gsm-triplets", "<imsi> <kc> <sres> <rand>", test_gsm_triplets},
    {"wlan-read-milenage", "<imsi> <ki> <opc> <amf> <sqn>", test_milenage},
};

int hlr_cli_init(void)
{
    unsigned int i;
    for (i = 0; i < sizeof(hlr_cli) / sizeof(struct cli_command); i++)
    {
        if (cli_register_command(&hlr_cli[i]) != 0)
        {
            return -WM_FAIL;
        }
    }
    return WM_SUCCESS;
}

int hlr_cli_deinit(void)
{
    unsigned int i;

    for (i = 0; i < sizeof(hlr_cli) / sizeof(struct cli_command); i++)
    {
        if (cli_unregister_command(&hlr_cli[i]) != 0)
        {
            return -WM_FAIL;
        }
    }
    return WM_SUCCESS;
}
#ifdef __ZEPHYR__
static void hlr_main_task(void *arg, void *arg1, void *arg2);
#else
static void hlr_main_task(osa_task_param_t arg);
#endif
#ifdef __ZEPHYR__
const int HLR_TASK_PRIO       = OS_PRIO_3;
#define CONFIG_HLR_THREAD_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(hlrCliTaskStack, CONFIG_HLR_THREAD_STACK_SIZE);
struct k_thread hlrCliTask;
k_tid_t hlr_cli_thread;

#define HLR_NUM_MESSAGES (20)
K_MSGQ_DEFINE(hlr_cli_event_queue, sizeof(void *), HLR_NUM_MESSAGES, 4);
K_EVENT_DEFINE(hlrCliTaskEvent);
#else

const int HLR_TASK_PRIO       = 1; //OS_PRIO_3;

#define CONFIG_HLR_THREAD_STACK_SIZE 2048

#define HLR_NUM_MESSAGES (20)

static sys_mbox_t hlr_cli_event_queue;

OSA_TASK_HANDLE_DEFINE(hlr_thread);
OSA_EVENT_HANDLE_DEFINE(hlr_event_Handle);

/* OSA_TASKS: name, priority, instances, stackSz, useFloat */
static OSA_TASK_DEFINE(hlr_main_task, PRIORITY_RTOS_TO_OSA(1), 1, CONFIG_HLR_THREAD_STACK_SIZE, 0);

#endif

typedef enum __hlr_event
{
    EVENT = 1U,
} hlr_event_t;

static void process_hlr_event()
{
    void *mem;
    char *buf = NULL, *resp;
    int res;
    
#ifdef __ZEPHYR__
    while (k_msgq_get(&hlr_cli_event_queue, &mem, K_NO_WAIT) == 0)
#else
    if (sys_mbox_valid(&hlr_cli_event_queue))
    {
        while (sys_mbox_tryfetch(&hlr_cli_event_queue, &mem) != SYS_MBOX_EMPTY)
#endif
        {
            if (mem != NULL)
            {
                buf = (char *)mem;
                res = os_strlen(buf);

                if (res == 0)
                  return;

                PRINTF("Received: %s\r\n", buf);

                resp = (char *)os_zalloc(1000);

                if (resp == NULL)
                {
                    return;
                }

                if (process_cmd(buf, resp, 1000) < 0)
                {
                    PRINTF("Failed to process request\r\n");
                    os_free(resp);
                    return;
                }

                if (resp[0] == '\0')
                {
                    PRINTF("No response\r\n");
                    os_free(resp);
                    return;
                }

                PRINTF("Send: %s\r\n", resp);

                if (send_eap_sim_db_event(resp) < 0)
                    perror("send");

                os_free(buf);
            }
        }
#ifndef __ZEPHYR__		
    }
#endif
}

static void notify_hlr_event(hlr_event_t event)
{
#ifdef __ZEPHYR__
    k_event_post(&hlrCliTaskEvent, (1U << event));
    k_yield();
    k_sleep(K_MSEC(10));
#else
    (void)OSA_EventSet((osa_event_handle_t)hlr_event_Handle, (1U << event));
    if (!__get_IPSR())
    {
        OSA_TaskYield();
        OSA_TimeDelay(10);
    }
#endif
}

int send_hlr_event(char *msg)
{
#ifdef __ZEPHYR__
    k_msgq_put(&hlr_cli_event_queue, (void *)(&msg), K_FOREVER);
#else
    sys_mbox_post(&hlr_cli_event_queue, (void *)msg);
#endif
    notify_hlr_event(EVENT);

    return 0;
}

#ifdef __ZEPHYR__
static void hlr_main_task(void *arg, void *arg1, void *arg2)
#else
static void hlr_main_task(osa_task_param_t arg)
#endif
{
#ifdef __ZEPHYR__
    uint32_t taskNotification = 0U;
#else
     osa_event_flags_t taskNotification;
#endif
#ifndef __ZEPHYR__
    if (sys_mbox_new(&hlr_cli_event_queue, HLR_NUM_MESSAGES) != ERR_OK)
    {
        wpa_printf(MSG_ERROR, "Failed to create msg queue");
        return;
    }
#endif

    for (;;)
    {
        taskNotification = 0U;
#ifdef __ZEPHYR__
        taskNotification = k_event_wait(&hlrCliTaskEvent, (1U << EVENT), 0, K_FOREVER);
        k_event_clear(&hlrCliTaskEvent, 0xFF);
#else

        (void)OSA_EventWait((osa_event_handle_t)hlr_event_Handle, (1U << EVENT), false, osaWaitForever_c,
                            &taskNotification);
#endif
        if (taskNotification == 0)
        {
            continue;
        }

        if (taskNotification & (1U << EVENT))
        {
            process_hlr_event();
        }
    }
}

static void usage(void)
{
    PRINTF(
        "HLR/AuC testing gateway for hostapd EAP-SIM/AKA "
        "database/authenticator\r\n"
        "Copyright (c) 2005-2017, Jouni Malinen <j@w1.fi>\r\n"
        "\r\n"
        "usage:\r\n"
        "hlr_auc_gw [-hu] [-s<socket path>] [-g<triplet file>] "
        "[-m<milenage file>] \\\r\n"
        "        [-D<DB file>] [-i<IND len in bits>] [command]\r\n"
        "\r\n"
        "options:\r\n"
        "  -h = show this usage help\r\n"
        "  -u = update SQN in Milenage file on exit\r\n"
        "  -s<socket path> = path for UNIX domain socket\n"
        "                    (default: %s)\r\n"
        "  -g<triplet file> = path for GSM authentication triplets\r\n"
        "  -m<milenage file> = path for Milenage keys\r\n"
        "  -D<DB file> = path to SQLite database\r\n"
        "  -i<IND len in bits> = IND length for SQN (default: 5)\r\n"
        "\r\n"
        "If the optional command argument, like "
        "\"AKA-REQ-AUTH <IMSI>\" is used, a single\r\n"
        "command is processed with response sent to stdout. Otherwise, "
        "hlr_auc_gw opens\r\n"
        "a control interface and processes commands sent through it "
        "(e.g., by EAP server\r\n"
        "in hostapd).\r\n",
        default_socket_path);
}

static u8 hlr_init_done = 0;

int hlr_main(int argc, char *argv[])
{
    int c;
#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    char *gsm_triplet_file = NULL;
    char *sqlite_db_file   = NULL;
#endif
    int ret                = 0;
    cli_optind             = 0;

    if (os_program_init())
        return -1;

    if (hlr_init_done == 1)
    {
        PRINTF("Already Listening for requests on %s\r\n", socket_path);
        return 0;
    }

    socket_path = default_socket_path;

    if (argc > 0)
    {
        for (;;)
        {
            c = cli_getopt(argc, argv, "D:g:hi:m:s:u");
            if (c < 0)
                break;
            switch (c)
            {
                case 'D':
#ifdef CONFIG_SQLITE
                    sqlite_db_file = cli_optarg;
                    break;
#else  /* CONFIG_SQLITE */
                    PRINTF("No SQLite support included in the build\r\n");
                    return -1;
#endif /* CONFIG_SQLITE */
                case 'g':
#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
                    gsm_triplet_file = cli_optarg;
#endif
                    break;
                case 'h':
                    usage();
                    return 0;
                case 'i':
                    ind_len = atoi(cli_optarg);
                    if (ind_len < 0 || ind_len > 32)
                    {
                        PRINTF("Invalid IND length\r\n");
                        return -1;
                    }
                    break;
                case 'm':
#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
                    milenage_file = cli_optarg;
#endif
                    break;
                case 's':
                    socket_path = cli_optarg;
                    break;
                case 'u':
                    update_milenage = 1;
                    break;
                default:
                    usage();
                    return -1;
            }
        }
    }

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    if (!gsm_triplet_file && !milenage_file && !sqlite_db_file)
    {
        usage();
        return -1;
    }
#else
    if (!gsm_db)
    {
        PRINTF("No GSM authentication triplets\r\n");
        return -1;
    }

    if (!milenage_db)
    {
        PRINTF("No Milenage keys\r\n");
        return -1;
    }
#endif

#ifdef CONFIG_SQLITE
    if (sqlite_db_file && (sqlite_db = db_open(sqlite_db_file)) == NULL)
        return -1;
#endif /* CONFIG_SQLITE */

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
    if (gsm_triplet_file && read_gsm_triplets2(gsm_triplet_file) < 0)
        return -1;

    if (milenage_file && read_milenage2(milenage_file) < 0)
        return -1;
#endif

    if (cli_optind == argc)
    {
        serv_sock = open_socket(socket_path);
        if (serv_sock < 0)
            return -1;

        PRINTF("Listening for requests on %s\r\n", socket_path);

#if !defined(CONFIG_FREERTOS) && !defined(__ZEPHYR__)
        atexit(cleanup);
        signal(SIGTERM, handle_term);
        signal(SIGINT, handle_term);

        for (;;)
            process(serv_sock);
#endif
        hlr_init_done = 1;
#ifdef __ZEPHYR__
		hlr_cli_thread = k_thread_create(&hlrCliTask, hlrCliTaskStack,
			K_THREAD_STACK_SIZEOF(hlrCliTaskStack), hlr_main_task, NULL, NULL, NULL,
			HLR_TASK_PRIO, 0, K_NO_WAIT);
		k_thread_name_set(hlr_cli_thread, "hlr");
#else
        int status;

        status = OSA_EventCreate((osa_event_handle_t)hlr_event_Handle, 1);
        if (status != KOSA_StatusSuccess)
        {
            return -WM_FAIL;
        }

        status = OSA_TaskCreate((osa_task_handle_t)hlr_thread, OSA_TASK(hlr_main_task), NULL);
        if (status != KOSA_StatusSuccess)
        {
            return -WM_FAIL;
        }
#endif
    }
    else
    {
        char buf[1000];
        socket_path  = NULL;
        stdout_debug = 0;
        if (process_cmd(argv[cli_optind], buf, sizeof(buf)) < 0)
        {
            PRINTF("FAIL\r\n");
            ret = -1;
        }
        else
        {
            PRINTF("%s\r\n", buf);
        }
        cleanup();
    }

#ifdef CONFIG_SQLITE
    if (sqlite_db)
    {
        sqlite3_close(sqlite_db);
        sqlite_db = NULL;
    }
#endif /* CONFIG_SQLITE */

    os_program_deinit();

    return ret;
}
