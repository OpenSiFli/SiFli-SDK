/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_SYS_CFG_FILE_H
#define __BF0_SYS_CFG_FILE_H

#ifdef __cplusplus
extern "C" {
#endif


/* For Factory configuration in OTP/Flash */

#define SYSCFG_FACTORY_HDR_SIZE         2           /*!< Configuration header size*/

#define CFG_IN_OTP_PAGE                 (1)         /*!< Configuration OTP sector index*/
#define CFG_USER_OTP_PAGE               (2)         /*!< Configuration USER sector index*/
#define CFG_CUST_OTP_PAGE               (3)         /*!< Configuration customer sector index*/


#define FACTORY_CFG_ID_INVALID          0       /*!< Invalid ID */
#define FACTORY_CFG_ID_MAC              1       /*!< BLE MAC address */
#define FACTORY_CFG_ID_SN               2       /*!< Serial Number */
#define FACTORY_CFG_ID_CRYSTAL          3       /*!< Crystal tuning information */
#define FACTORY_CFG_ID_ADC              4       /*!< ADC tuning information*/
#define FACTORY_CFG_ID_SDMADC           5       /*!< SDMADC tuning information*/
#define FACTORY_CFG_ID_VBUCK            6       /*!< VBUCK /LDO information*/
#define FACTORY_CFG_ID_SECCODE          7       /*!< Security Code or something like this*/
#define FACTORY_CFG_ID_LOCALNAME        8       /*!< BLE localname*/
#define FACTORY_CFG_ID_BATTERY          9       /*!< Battery verify value*/
#define FACTORY_CFG_ID_FWVERIFY         10      /*!< FW verify code generated based in uid*/
#define FACTORY_CFG_ID_ALIPAY_PK        11      /*!< for alipay product key  code*/
#define FACTORY_CFG_ID_ALIPAY_DN        12      /*!< for alipay device name code*/
#define FACTORY_CFG_ID_ALIPAY_DS        13      /*!< for alipay device secret code*/
#define FACTORY_CFG_ID_BTNAME           14      /*!< for alipay device secret code*/
#define FACTORY_CFG_ID_WECHAT           15      /*!< for Wechat flag*/
#define FACTORY_CFG_ID_THIRDFUNC        16      /*!< for customer speicial purpose*/
#define FACTORY_CFG_ID_CTEI             17      /*!< for customer speicial purpose*/
#define FACTORY_CFG_ID_BLECALI          19      /*!< for BLE RF CALI   T_BLE_CALI_PARA*/
#define FACTORY_CFG_ID_SIPMODE          20      /*!< Mpi1 sip mode , for sip a pin setting*/
#define FACTORY_CFG_ID_CHARGER          21      /*!< Charger parameters */
#define FACTORY_CFG_ID_KVDB_DISABLE     22      /*!< disable kvdb */
#define FACTORY_CFG_ID_TMXCAPINEFUSE    23      /*!< IF use tmxcap value in EFUSE, 0xa5a55a5a: use   other:no use*/
#define FACTORY_CFG_ID_SNSPECIAL        24      /*!< User specific format SN, may coexist with SN  */
#define FACTORY_CFG_ID_GOMORE           25      /*!< User motion algo (gomore) key  */
#define FACTORY_CFG_ID_USERFN           26      /*!< User special code */
#define FACTORY_CFG_ID_USERK1           27      /*!< User special code  */
#define FACTORY_CFG_ID_USERK2           28      /*!< User special code  */
#define FACTORY_CFG_ID_USERK3           29      /*!< User special code  */
#define FACTORY_CFG_ID_UNINIT           0xFF    /*!< Uninitialized ID */

#define CFG_USER_SIZE            (256)
#if defined(SF32LB52X) || defined(SF32LB57X)
#define CFG_SYS_SIZE            (32)
#else
#define CFG_SYS_SIZE            (64)
#endif // SF32LB52X || SF32LB57X



#ifndef SF32LB55X
typedef struct
{
    uint16_t vol10;      /*!< Reg value for low voltage. */
    uint16_t vol25;      /*!< Reg value for high voltage. */
    uint16_t low_mv;     /*!< voltage for low with mv . */
    uint16_t high_mv;    /*!< voltage for high with mv. */
#if defined(SF32LB52X) || defined(SF32LB57X)
    uint16_t vbat_reg;      /*!< Reg value for reference vbat */
    uint16_t vbat_mv;     /*!< voltage for vbat with mv. */
    uint8_t ldovref_flag;    /*!< ldo vref flag, if 1,  gpadc ldo vref has been calibrated. */
    uint8_t ldovref_sel;    /*!< ldo vref sleect. */
#endif
} FACTORY_CFG_ADC_T;

#else
typedef struct
{
    uint16_t vol10;
    uint16_t vol25;
} FACTORY_CFG_ADC_T;
#endif

typedef struct
{
    uint32_t vol_mv;
    uint32_t value;
} FACTORY_CFG_SDMADC_T;

typedef struct
{
#if !defined(SF32LB52X) && !defined(SF32LB57X)
    uint8_t vbuck1;
    uint8_t hp_ldo: 4;
    uint8_t lp_ldo: 4;
    uint8_t vret;
    uint8_t vbuck2;
#else
    uint8_t buck_vos_trim;
    uint8_t buck_vos_polar;
    uint8_t hpsys_ldo_vout;   // buck = 1.25v
    uint8_t lpsys_ldo_vout;
    uint8_t vret_trim;
    uint8_t ldo18_vref_sel;
    uint8_t vdd33_ldo2_vout;
    uint8_t vdd33_ldo3_vout;
    uint8_t aon_vos_trim;
    uint8_t aon_vos_polar;
    uint8_t buck_vos_trim2;
    uint8_t buck_vos_polar2;
    uint8_t hpsys_ldo_vout2;      // buck = 1.35v
    uint8_t lpsys_ldo_vout2;
#endif
} FACTORY_CFG_VBK_LDO_T;

typedef struct
{
    uint32_t cbank_sel;
} FACTORY_CFG_CRYSTAL_T;

typedef struct
{
    int32_t magic;
    uint32_t a;
    int32_t b;
} FACTORY_CFG_BATTERY_CALI_T;

typedef struct
{
    uint8_t mpi1_mode;
    uint8_t mpi2_mode;
} FACTORY_CFG_SIP_MOD_T;

typedef struct
{
    int32_t  maxPower;
    int32_t  meanPower;
    int32_t  rxSensitivity;
    int32_t  freqOffset;
    int32_t  spectrumPrecision;
    int32_t  testResult;
} T_BLE_CALI_CHAN_PARA;

typedef struct
{
    T_BLE_CALI_CHAN_PARA ch0;
    T_BLE_CALI_CHAN_PARA ch19;
    T_BLE_CALI_CHAN_PARA ch39;
    int32_t meanCurrent;
} T_BLE_CALI_PARA;

typedef struct
{
    uint8_t prog_v1p2;
    uint8_t cv_vctrl;
    uint8_t cc_mn;
    uint8_t cc_mp;
    uint8_t chg_step;
    uint8_t reserved[3];
} FACTORY_CFG_CHARGER_T;

int BSP_System_Config(void);

char *BSP_Get_UserOTP_Cache();
char *BSP_Get_CustOTP_Cache();
#if defined(SF32LB52X) || defined(SF32LB57X)
char *BSP_Get_SysCfg_Cache();
#endif
int BSP_CONFIG_get(int type, uint8_t *buf, int length);
HAL_StatusTypeDef BSP_CONFIG_set(int type, uint8_t *buf, int length);

uint32_t BSP_Get_Sip1_Mode();
uint32_t BSP_Get_Sip2_Mode();

uint32_t BSP_GetOtpBase(void);

#ifdef __cplusplus
}
#endif

#endif /* __BF0_SYS_CFG_FILE_H */