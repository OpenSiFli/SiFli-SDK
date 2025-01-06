#ifndef _REG_MWSGEN_H_
#define _REG_MWSGEN_H_

#include <stdint.h>
#include "_reg_mwsgen.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

#define REG_MWSGEN_COUNT 15

#define REG_MWSGEN_DECODING_MASK 0x0000003F

/**
 * @brief MWSGENCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            MWSRELOAD   0
 *     12          WLAN_SOURCE   0
 *     11        WLAN_RXRND_EN   0
 *     10        WLAN_TXRND_EN   0
 *     09           WLAN_RX_EN   0
 *     08           WLAN_TX_EN   0
 *     07         MWS_RXRND_EN   0
 *     06         MWS_TXRND_EN   0
 *     05            MWS_RX_EN   0
 *     04            MWS_TX_EN   0
 *     02               WCI_EN   0
 *     01              WLAN_EN   0
 *     00               MWS_EN   0
 * </pre>
 */
#define MWSGEN_MWSGENCNTL_ADDR   0x10009000
#define MWSGEN_MWSGENCNTL_OFFSET 0x00000000
#define MWSGEN_MWSGENCNTL_INDEX  0x00000000
#define MWSGEN_MWSGENCNTL_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgencntl_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
}

__INLINE void mwsgen_mwsgencntl_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, value);
}

// field definitions
#define MWSGEN_MWSRELOAD_BIT        ((uint32_t)0x80000000)
#define MWSGEN_MWSRELOAD_POS        31
#define MWSGEN_WLAN_SOURCE_BIT      ((uint32_t)0x00001000)
#define MWSGEN_WLAN_SOURCE_POS      12
#define MWSGEN_WLAN_RXRND_EN_BIT    ((uint32_t)0x00000800)
#define MWSGEN_WLAN_RXRND_EN_POS    11
#define MWSGEN_WLAN_TXRND_EN_BIT    ((uint32_t)0x00000400)
#define MWSGEN_WLAN_TXRND_EN_POS    10
#define MWSGEN_WLAN_RX_EN_BIT       ((uint32_t)0x00000200)
#define MWSGEN_WLAN_RX_EN_POS       9
#define MWSGEN_WLAN_TX_EN_BIT       ((uint32_t)0x00000100)
#define MWSGEN_WLAN_TX_EN_POS       8
#define MWSGEN_MWS_RXRND_EN_BIT     ((uint32_t)0x00000080)
#define MWSGEN_MWS_RXRND_EN_POS     7
#define MWSGEN_MWS_TXRND_EN_BIT     ((uint32_t)0x00000040)
#define MWSGEN_MWS_TXRND_EN_POS     6
#define MWSGEN_MWS_RX_EN_BIT        ((uint32_t)0x00000020)
#define MWSGEN_MWS_RX_EN_POS        5
#define MWSGEN_MWS_TX_EN_BIT        ((uint32_t)0x00000010)
#define MWSGEN_MWS_TX_EN_POS        4
#define MWSGEN_WCI_EN_BIT           ((uint32_t)0x00000004)
#define MWSGEN_WCI_EN_POS           2
#define MWSGEN_WLAN_EN_BIT          ((uint32_t)0x00000002)
#define MWSGEN_WLAN_EN_POS          1
#define MWSGEN_MWS_EN_BIT           ((uint32_t)0x00000001)
#define MWSGEN_MWS_EN_POS           0

#define MWSGEN_MWSRELOAD_RST        0x0
#define MWSGEN_WLAN_SOURCE_RST      0x0
#define MWSGEN_WLAN_RXRND_EN_RST    0x0
#define MWSGEN_WLAN_TXRND_EN_RST    0x0
#define MWSGEN_WLAN_RX_EN_RST       0x0
#define MWSGEN_WLAN_TX_EN_RST       0x0
#define MWSGEN_MWS_RXRND_EN_RST     0x0
#define MWSGEN_MWS_TXRND_EN_RST     0x0
#define MWSGEN_MWS_RX_EN_RST        0x0
#define MWSGEN_MWS_TX_EN_RST        0x0
#define MWSGEN_WCI_EN_RST           0x0
#define MWSGEN_WLAN_EN_RST          0x0
#define MWSGEN_MWS_EN_RST           0x0

__INLINE void mwsgen_mwsgencntl_pack(uint8_t mwsreload, uint8_t wlansource, uint8_t wlanrxrnden, uint8_t wlantxrnden, uint8_t wlanrxen, uint8_t wlantxen, uint8_t mwsrxrnden, uint8_t mwstxrnden, uint8_t mwsrxen, uint8_t mwstxen, uint8_t wcien, uint8_t wlanen, uint8_t mwsen)
{
    ASSERT_ERR((((uint32_t)mwsreload << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)wlansource << 12) & ~((uint32_t)0x00001000)) == 0);
    ASSERT_ERR((((uint32_t)wlanrxrnden << 11) & ~((uint32_t)0x00000800)) == 0);
    ASSERT_ERR((((uint32_t)wlantxrnden << 10) & ~((uint32_t)0x00000400)) == 0);
    ASSERT_ERR((((uint32_t)wlanrxen << 9) & ~((uint32_t)0x00000200)) == 0);
    ASSERT_ERR((((uint32_t)wlantxen << 8) & ~((uint32_t)0x00000100)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxrnden << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)mwstxrnden << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)mwsrxen << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)mwstxen << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)wcien << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)wlanen << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)mwsen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, ((uint32_t)mwsreload << 31) | ((uint32_t)wlansource << 12) | ((uint32_t)wlanrxrnden << 11) | ((uint32_t)wlantxrnden << 10) | ((uint32_t)wlanrxen << 9) | ((uint32_t)wlantxen << 8) | ((uint32_t)mwsrxrnden << 7) | ((uint32_t)mwstxrnden << 6) | ((uint32_t)mwsrxen << 5) | ((uint32_t)mwstxen << 4) | ((uint32_t)wcien << 2) | ((uint32_t)wlanen << 1) | ((uint32_t)mwsen << 0));
}

__INLINE void mwsgen_mwsgencntl_unpack(uint8_t *mwsreload, uint8_t *wlansource, uint8_t *wlanrxrnden, uint8_t *wlantxrnden, uint8_t *wlanrxen, uint8_t *wlantxen, uint8_t *mwsrxrnden, uint8_t *mwstxrnden, uint8_t *mwsrxen, uint8_t *mwstxen, uint8_t *wcien, uint8_t *wlanen, uint8_t *mwsen)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);

    *mwsreload = (localVal & ((uint32_t)0x80000000)) >> 31;
    *wlansource = (localVal & ((uint32_t)0x00001000)) >> 12;
    *wlanrxrnden = (localVal & ((uint32_t)0x00000800)) >> 11;
    *wlantxrnden = (localVal & ((uint32_t)0x00000400)) >> 10;
    *wlanrxen = (localVal & ((uint32_t)0x00000200)) >> 9;
    *wlantxen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *mwsrxrnden = (localVal & ((uint32_t)0x00000080)) >> 7;
    *mwstxrnden = (localVal & ((uint32_t)0x00000040)) >> 6;
    *mwsrxen = (localVal & ((uint32_t)0x00000020)) >> 5;
    *mwstxen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *wcien = (localVal & ((uint32_t)0x00000004)) >> 2;
    *wlanen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *mwsen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t mwsgen_mwsreload_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void mwsgen_mwsreload_setf(uint8_t mwsreload)
{
    ASSERT_ERR((((uint32_t)mwsreload << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)mwsreload << 31));
}

__INLINE uint8_t mwsgen_wlan_source_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void mwsgen_wlan_source_setf(uint8_t wlansource)
{
    ASSERT_ERR((((uint32_t)wlansource << 12) & ~((uint32_t)0x00001000)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)wlansource << 12));
}

__INLINE uint8_t mwsgen_wlan_rxrnd_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void mwsgen_wlan_rxrnd_en_setf(uint8_t wlanrxrnden)
{
    ASSERT_ERR((((uint32_t)wlanrxrnden << 11) & ~((uint32_t)0x00000800)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)wlanrxrnden << 11));
}

__INLINE uint8_t mwsgen_wlan_txrnd_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void mwsgen_wlan_txrnd_en_setf(uint8_t wlantxrnden)
{
    ASSERT_ERR((((uint32_t)wlantxrnden << 10) & ~((uint32_t)0x00000400)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)wlantxrnden << 10));
}

__INLINE uint8_t mwsgen_wlan_rx_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void mwsgen_wlan_rx_en_setf(uint8_t wlanrxen)
{
    ASSERT_ERR((((uint32_t)wlanrxen << 9) & ~((uint32_t)0x00000200)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)wlanrxen << 9));
}

__INLINE uint8_t mwsgen_wlan_tx_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void mwsgen_wlan_tx_en_setf(uint8_t wlantxen)
{
    ASSERT_ERR((((uint32_t)wlantxen << 8) & ~((uint32_t)0x00000100)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)wlantxen << 8));
}

__INLINE uint8_t mwsgen_mws_rxrnd_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void mwsgen_mws_rxrnd_en_setf(uint8_t mwsrxrnden)
{
    ASSERT_ERR((((uint32_t)mwsrxrnden << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)mwsrxrnden << 7));
}

__INLINE uint8_t mwsgen_mws_txrnd_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void mwsgen_mws_txrnd_en_setf(uint8_t mwstxrnden)
{
    ASSERT_ERR((((uint32_t)mwstxrnden << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)mwstxrnden << 6));
}

__INLINE uint8_t mwsgen_mws_rx_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void mwsgen_mws_rx_en_setf(uint8_t mwsrxen)
{
    ASSERT_ERR((((uint32_t)mwsrxen << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)mwsrxen << 5));
}

__INLINE uint8_t mwsgen_mws_tx_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void mwsgen_mws_tx_en_setf(uint8_t mwstxen)
{
    ASSERT_ERR((((uint32_t)mwstxen << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)mwstxen << 4));
}

__INLINE uint8_t mwsgen_wci_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void mwsgen_wci_en_setf(uint8_t wcien)
{
    ASSERT_ERR((((uint32_t)wcien << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)wcien << 2));
}

__INLINE uint8_t mwsgen_wlan_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void mwsgen_wlan_en_setf(uint8_t wlanen)
{
    ASSERT_ERR((((uint32_t)wlanen << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)wlanen << 1));
}

__INLINE uint8_t mwsgen_mws_en_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void mwsgen_mws_en_setf(uint8_t mwsen)
{
    ASSERT_ERR((((uint32_t)mwsen << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(MWSGEN_MWSGENCNTL_ADDR, (REG_PL_RD(MWSGEN_MWSGENCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)mwsen << 0));
}

/**
 * @brief MWSGEN_PERIOD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           MWS_PERIOD   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_PERIOD_ADDR   0x10009004
#define MWSGEN_MWSGEN_PERIOD_OFFSET 0x00000004
#define MWSGEN_MWSGEN_PERIOD_INDEX  0x00000001
#define MWSGEN_MWSGEN_PERIOD_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_period_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_PERIOD_ADDR);
}

__INLINE void mwsgen_mwsgen_period_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_PERIOD_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_PERIOD_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_MWS_PERIOD_LSB    0
#define MWSGEN_MWS_PERIOD_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_MWS_PERIOD_RST    0x0

__INLINE uint32_t mwsgen_mws_period_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_PERIOD_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_mws_period_setf(uint32_t mwsperiod)
{
    ASSERT_ERR((((uint32_t)mwsperiod << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_PERIOD_ADDR, (uint32_t)mwsperiod << 0);
}

/**
 * @brief MWSGEN_DUTY_CYCLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00       MWS_DUTY_CYCLE   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_DUTY_CYCLE_ADDR   0x10009008
#define MWSGEN_MWSGEN_DUTY_CYCLE_OFFSET 0x00000008
#define MWSGEN_MWSGEN_DUTY_CYCLE_INDEX  0x00000002
#define MWSGEN_MWSGEN_DUTY_CYCLE_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_duty_cycle_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_DUTY_CYCLE_ADDR);
}

__INLINE void mwsgen_mwsgen_duty_cycle_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_DUTY_CYCLE_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_DUTY_CYCLE_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_MWS_DUTY_CYCLE_LSB    0
#define MWSGEN_MWS_DUTY_CYCLE_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_MWS_DUTY_CYCLE_RST    0x0

__INLINE uint32_t mwsgen_mws_duty_cycle_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_DUTY_CYCLE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_mws_duty_cycle_setf(uint32_t mwsdutycycle)
{
    ASSERT_ERR((((uint32_t)mwsdutycycle << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_DUTY_CYCLE_ADDR, (uint32_t)mwsdutycycle << 0);
}

/**
 * @brief MWSGEN_DELAY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         MWS_RX_DELAY   0x0
 *  15:00         MWS_TX_DELAY   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_DELAY_ADDR   0x1000900C
#define MWSGEN_MWSGEN_DELAY_OFFSET 0x0000000C
#define MWSGEN_MWSGEN_DELAY_INDEX  0x00000003
#define MWSGEN_MWSGEN_DELAY_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_delay_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR);
}

__INLINE void mwsgen_mwsgen_delay_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_DELAY_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_RX_DELAY_MASK   ((uint32_t)0xFFFF0000)
#define MWSGEN_MWS_RX_DELAY_LSB    16
#define MWSGEN_MWS_RX_DELAY_WIDTH  ((uint32_t)0x00000010)
#define MWSGEN_MWS_TX_DELAY_MASK   ((uint32_t)0x0000FFFF)
#define MWSGEN_MWS_TX_DELAY_LSB    0
#define MWSGEN_MWS_TX_DELAY_WIDTH  ((uint32_t)0x00000010)

#define MWSGEN_MWS_RX_DELAY_RST    0x0
#define MWSGEN_MWS_TX_DELAY_RST    0x0

__INLINE void mwsgen_mwsgen_delay_pack(uint16_t mwsrxdelay, uint16_t mwstxdelay)
{
    ASSERT_ERR((((uint32_t)mwsrxdelay << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)mwstxdelay << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_DELAY_ADDR, ((uint32_t)mwsrxdelay << 16) | ((uint32_t)mwstxdelay << 0));
}

__INLINE void mwsgen_mwsgen_delay_unpack(uint16_t *mwsrxdelay, uint16_t *mwstxdelay)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR);

    *mwsrxdelay = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *mwstxdelay = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t mwsgen_mws_rx_delay_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void mwsgen_mws_rx_delay_setf(uint16_t mwsrxdelay)
{
    ASSERT_ERR((((uint32_t)mwsrxdelay << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_DELAY_ADDR, (REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)mwsrxdelay << 16));
}

__INLINE uint16_t mwsgen_mws_tx_delay_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void mwsgen_mws_tx_delay_setf(uint16_t mwstxdelay)
{
    ASSERT_ERR((((uint32_t)mwstxdelay << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_DELAY_ADDR, (REG_PL_RD(MWSGEN_MWSGEN_DELAY_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)mwstxdelay << 0));
}

/**
 * @brief MWSGEN_TX_ACTIVITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           MWS_TX_ACT   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_TX_ACTIVITY_ADDR   0x10009010
#define MWSGEN_MWSGEN_TX_ACTIVITY_OFFSET 0x00000010
#define MWSGEN_MWSGEN_TX_ACTIVITY_INDEX  0x00000004
#define MWSGEN_MWSGEN_TX_ACTIVITY_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_tx_activity_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_TX_ACTIVITY_ADDR);
}

__INLINE void mwsgen_mwsgen_tx_activity_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_TX_ACTIVITY_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_TX_ACT_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_MWS_TX_ACT_LSB    0
#define MWSGEN_MWS_TX_ACT_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_MWS_TX_ACT_RST    0x0

__INLINE uint32_t mwsgen_mws_tx_act_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_TX_ACTIVITY_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_mws_tx_act_setf(uint32_t mwstxact)
{
    ASSERT_ERR((((uint32_t)mwstxact << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_TX_ACTIVITY_ADDR, (uint32_t)mwstxact << 0);
}

/**
 * @brief MWSGEN_RX_ACTIVITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           MWS_RX_ACT   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_RX_ACTIVITY_ADDR   0x10009014
#define MWSGEN_MWSGEN_RX_ACTIVITY_OFFSET 0x00000014
#define MWSGEN_MWSGEN_RX_ACTIVITY_INDEX  0x00000005
#define MWSGEN_MWSGEN_RX_ACTIVITY_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_rx_activity_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_RX_ACTIVITY_ADDR);
}

__INLINE void mwsgen_mwsgen_rx_activity_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_RX_ACTIVITY_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_RX_ACT_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_MWS_RX_ACT_LSB    0
#define MWSGEN_MWS_RX_ACT_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_MWS_RX_ACT_RST    0x0

__INLINE uint32_t mwsgen_mws_rx_act_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_RX_ACTIVITY_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_mws_rx_act_setf(uint32_t mwsrxact)
{
    ASSERT_ERR((((uint32_t)mwsrxact << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_RX_ACTIVITY_ADDR, (uint32_t)mwsrxact << 0);
}

/**
 * @brief MWSGEN_MISC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  17:16          MWS_PATTERN   0x0
 *  12:08   MWS_INACTIVITY_DURATION   0x0
 *  04:00   MWS_SCAN_FREQUENCY   0x0
 * </pre>
 */
#define MWSGEN_MWSGEN_MISC_ADDR   0x10009018
#define MWSGEN_MWSGEN_MISC_OFFSET 0x00000018
#define MWSGEN_MWSGEN_MISC_INDEX  0x00000006
#define MWSGEN_MWSGEN_MISC_RESET  0x00000000

__INLINE uint32_t mwsgen_mwsgen_misc_get(void)
{
    return REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR);
}

__INLINE void mwsgen_mwsgen_misc_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_MWSGEN_MISC_ADDR, value);
}

// field definitions
#define MWSGEN_MWS_PATTERN_MASK               ((uint32_t)0x00030000)
#define MWSGEN_MWS_PATTERN_LSB                16
#define MWSGEN_MWS_PATTERN_WIDTH              ((uint32_t)0x00000002)
#define MWSGEN_MWS_INACTIVITY_DURATION_MASK   ((uint32_t)0x00001F00)
#define MWSGEN_MWS_INACTIVITY_DURATION_LSB    8
#define MWSGEN_MWS_INACTIVITY_DURATION_WIDTH  ((uint32_t)0x00000005)
#define MWSGEN_MWS_SCAN_FREQUENCY_MASK        ((uint32_t)0x0000001F)
#define MWSGEN_MWS_SCAN_FREQUENCY_LSB         0
#define MWSGEN_MWS_SCAN_FREQUENCY_WIDTH       ((uint32_t)0x00000005)

#define MWSGEN_MWS_PATTERN_RST                0x0
#define MWSGEN_MWS_INACTIVITY_DURATION_RST    0x0
#define MWSGEN_MWS_SCAN_FREQUENCY_RST         0x0

__INLINE void mwsgen_mwsgen_misc_pack(uint8_t mwspattern, uint8_t mwsinactivityduration, uint8_t mwsscanfrequency)
{
    ASSERT_ERR((((uint32_t)mwspattern << 16) & ~((uint32_t)0x00030000)) == 0);
    ASSERT_ERR((((uint32_t)mwsinactivityduration << 8) & ~((uint32_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint32_t)mwsscanfrequency << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_MISC_ADDR, ((uint32_t)mwspattern << 16) | ((uint32_t)mwsinactivityduration << 8) | ((uint32_t)mwsscanfrequency << 0));
}

__INLINE void mwsgen_mwsgen_misc_unpack(uint8_t *mwspattern, uint8_t *mwsinactivityduration, uint8_t *mwsscanfrequency)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR);

    *mwspattern = (localVal & ((uint32_t)0x00030000)) >> 16;
    *mwsinactivityduration = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *mwsscanfrequency = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t mwsgen_mws_pattern_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void mwsgen_mws_pattern_setf(uint8_t mwspattern)
{
    ASSERT_ERR((((uint32_t)mwspattern << 16) & ~((uint32_t)0x00030000)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_MISC_ADDR, (REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)mwspattern << 16));
}

__INLINE uint8_t mwsgen_mws_inactivity_duration_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void mwsgen_mws_inactivity_duration_setf(uint8_t mwsinactivityduration)
{
    ASSERT_ERR((((uint32_t)mwsinactivityduration << 8) & ~((uint32_t)0x00001F00)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_MISC_ADDR, (REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)mwsinactivityduration << 8));
}

__INLINE uint8_t mwsgen_mws_scan_frequency_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void mwsgen_mws_scan_frequency_setf(uint8_t mwsscanfrequency)
{
    ASSERT_ERR((((uint32_t)mwsscanfrequency << 0) & ~((uint32_t)0x0000001F)) == 0);
    REG_PL_WR(MWSGEN_MWSGEN_MISC_ADDR, (REG_PL_RD(MWSGEN_MWSGEN_MISC_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)mwsscanfrequency << 0));
}

/**
 * @brief WLANGEN_PERIOD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          WLAN_PERIOD   0x0
 * </pre>
 */
#define MWSGEN_WLANGEN_PERIOD_ADDR   0x1000901C
#define MWSGEN_WLANGEN_PERIOD_OFFSET 0x0000001C
#define MWSGEN_WLANGEN_PERIOD_INDEX  0x00000007
#define MWSGEN_WLANGEN_PERIOD_RESET  0x00000000

__INLINE uint32_t mwsgen_wlangen_period_get(void)
{
    return REG_PL_RD(MWSGEN_WLANGEN_PERIOD_ADDR);
}

__INLINE void mwsgen_wlangen_period_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WLANGEN_PERIOD_ADDR, value);
}

// field definitions
#define MWSGEN_WLAN_PERIOD_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_WLAN_PERIOD_LSB    0
#define MWSGEN_WLAN_PERIOD_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_WLAN_PERIOD_RST    0x0

__INLINE uint32_t mwsgen_wlan_period_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_PERIOD_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_wlan_period_setf(uint32_t wlanperiod)
{
    ASSERT_ERR((((uint32_t)wlanperiod << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_PERIOD_ADDR, (uint32_t)wlanperiod << 0);
}

/**
 * @brief WLANGEN_DUTY_CYCLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00      WLAN_DUTY_CYCLE   0x0
 * </pre>
 */
#define MWSGEN_WLANGEN_DUTY_CYCLE_ADDR   0x10009020
#define MWSGEN_WLANGEN_DUTY_CYCLE_OFFSET 0x00000020
#define MWSGEN_WLANGEN_DUTY_CYCLE_INDEX  0x00000008
#define MWSGEN_WLANGEN_DUTY_CYCLE_RESET  0x00000000

__INLINE uint32_t mwsgen_wlangen_duty_cycle_get(void)
{
    return REG_PL_RD(MWSGEN_WLANGEN_DUTY_CYCLE_ADDR);
}

__INLINE void mwsgen_wlangen_duty_cycle_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WLANGEN_DUTY_CYCLE_ADDR, value);
}

// field definitions
#define MWSGEN_WLAN_DUTY_CYCLE_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_WLAN_DUTY_CYCLE_LSB    0
#define MWSGEN_WLAN_DUTY_CYCLE_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_WLAN_DUTY_CYCLE_RST    0x0

__INLINE uint32_t mwsgen_wlan_duty_cycle_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_DUTY_CYCLE_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_wlan_duty_cycle_setf(uint32_t wlandutycycle)
{
    ASSERT_ERR((((uint32_t)wlandutycycle << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_DUTY_CYCLE_ADDR, (uint32_t)wlandutycycle << 0);
}

/**
 * @brief WLANGEN_DELAY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16        WLAN_RX_DELAY   0x0
 *  15:00        WLAN_TX_DELAY   0x0
 * </pre>
 */
#define MWSGEN_WLANGEN_DELAY_ADDR   0x10009024
#define MWSGEN_WLANGEN_DELAY_OFFSET 0x00000024
#define MWSGEN_WLANGEN_DELAY_INDEX  0x00000009
#define MWSGEN_WLANGEN_DELAY_RESET  0x00000000

__INLINE uint32_t mwsgen_wlangen_delay_get(void)
{
    return REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR);
}

__INLINE void mwsgen_wlangen_delay_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WLANGEN_DELAY_ADDR, value);
}

// field definitions
#define MWSGEN_WLAN_RX_DELAY_MASK   ((uint32_t)0xFFFF0000)
#define MWSGEN_WLAN_RX_DELAY_LSB    16
#define MWSGEN_WLAN_RX_DELAY_WIDTH  ((uint32_t)0x00000010)
#define MWSGEN_WLAN_TX_DELAY_MASK   ((uint32_t)0x0000FFFF)
#define MWSGEN_WLAN_TX_DELAY_LSB    0
#define MWSGEN_WLAN_TX_DELAY_WIDTH  ((uint32_t)0x00000010)

#define MWSGEN_WLAN_RX_DELAY_RST    0x0
#define MWSGEN_WLAN_TX_DELAY_RST    0x0

__INLINE void mwsgen_wlangen_delay_pack(uint16_t wlanrxdelay, uint16_t wlantxdelay)
{
    ASSERT_ERR((((uint32_t)wlanrxdelay << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    ASSERT_ERR((((uint32_t)wlantxdelay << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_DELAY_ADDR, ((uint32_t)wlanrxdelay << 16) | ((uint32_t)wlantxdelay << 0));
}

__INLINE void mwsgen_wlangen_delay_unpack(uint16_t *wlanrxdelay, uint16_t *wlantxdelay)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR);

    *wlanrxdelay = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *wlantxdelay = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t mwsgen_wlan_rx_delay_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void mwsgen_wlan_rx_delay_setf(uint16_t wlanrxdelay)
{
    ASSERT_ERR((((uint32_t)wlanrxdelay << 16) & ~((uint32_t)0xFFFF0000)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_DELAY_ADDR, (REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)wlanrxdelay << 16));
}

__INLINE uint16_t mwsgen_wlan_tx_delay_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void mwsgen_wlan_tx_delay_setf(uint16_t wlantxdelay)
{
    ASSERT_ERR((((uint32_t)wlantxdelay << 0) & ~((uint32_t)0x0000FFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_DELAY_ADDR, (REG_PL_RD(MWSGEN_WLANGEN_DELAY_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)wlantxdelay << 0));
}

/**
 * @brief WLANGEN_TX_ACTIVITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          WLAN_TX_ACT   0x0
 * </pre>
 */
#define MWSGEN_WLANGEN_TX_ACTIVITY_ADDR   0x10009028
#define MWSGEN_WLANGEN_TX_ACTIVITY_OFFSET 0x00000028
#define MWSGEN_WLANGEN_TX_ACTIVITY_INDEX  0x0000000A
#define MWSGEN_WLANGEN_TX_ACTIVITY_RESET  0x00000000

__INLINE uint32_t mwsgen_wlangen_tx_activity_get(void)
{
    return REG_PL_RD(MWSGEN_WLANGEN_TX_ACTIVITY_ADDR);
}

__INLINE void mwsgen_wlangen_tx_activity_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WLANGEN_TX_ACTIVITY_ADDR, value);
}

// field definitions
#define MWSGEN_WLAN_TX_ACT_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_WLAN_TX_ACT_LSB    0
#define MWSGEN_WLAN_TX_ACT_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_WLAN_TX_ACT_RST    0x0

__INLINE uint32_t mwsgen_wlan_tx_act_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_TX_ACTIVITY_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_wlan_tx_act_setf(uint32_t wlantxact)
{
    ASSERT_ERR((((uint32_t)wlantxact << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_TX_ACTIVITY_ADDR, (uint32_t)wlantxact << 0);
}

/**
 * @brief WLANGEN_RX_ACTIVITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          WLAN_RX_ACT   0x0
 * </pre>
 */
#define MWSGEN_WLANGEN_RX_ACTIVITY_ADDR   0x1000902C
#define MWSGEN_WLANGEN_RX_ACTIVITY_OFFSET 0x0000002C
#define MWSGEN_WLANGEN_RX_ACTIVITY_INDEX  0x0000000B
#define MWSGEN_WLANGEN_RX_ACTIVITY_RESET  0x00000000

__INLINE uint32_t mwsgen_wlangen_rx_activity_get(void)
{
    return REG_PL_RD(MWSGEN_WLANGEN_RX_ACTIVITY_ADDR);
}

__INLINE void mwsgen_wlangen_rx_activity_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WLANGEN_RX_ACTIVITY_ADDR, value);
}

// field definitions
#define MWSGEN_WLAN_RX_ACT_MASK   ((uint32_t)0xFFFFFFFF)
#define MWSGEN_WLAN_RX_ACT_LSB    0
#define MWSGEN_WLAN_RX_ACT_WIDTH  ((uint32_t)0x00000020)

#define MWSGEN_WLAN_RX_ACT_RST    0x0

__INLINE uint32_t mwsgen_wlan_rx_act_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WLANGEN_RX_ACTIVITY_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void mwsgen_wlan_rx_act_setf(uint32_t wlanrxact)
{
    ASSERT_ERR((((uint32_t)wlanrxact << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(MWSGEN_WLANGEN_RX_ACTIVITY_ADDR, (uint32_t)wlanrxact << 0);
}

/**
 * @brief WCICNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:16            WCIIFSCNT   0x0
 *  15:03           WCI_INTDIV   0x0
 *  02:00         WCI_FRACTDIV   0x0
 * </pre>
 */
#define MWSGEN_WCICNTL0_ADDR   0x10009030
#define MWSGEN_WCICNTL0_OFFSET 0x00000030
#define MWSGEN_WCICNTL0_INDEX  0x0000000C
#define MWSGEN_WCICNTL0_RESET  0x00000000

__INLINE uint32_t mwsgen_wcicntl0_get(void)
{
    return REG_PL_RD(MWSGEN_WCICNTL0_ADDR);
}

__INLINE void mwsgen_wcicntl0_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WCICNTL0_ADDR, value);
}

// field definitions
#define MWSGEN_WCIIFSCNT_MASK      ((uint32_t)0x000F0000)
#define MWSGEN_WCIIFSCNT_LSB       16
#define MWSGEN_WCIIFSCNT_WIDTH     ((uint32_t)0x00000004)
#define MWSGEN_WCI_INTDIV_MASK     ((uint32_t)0x0000FFF8)
#define MWSGEN_WCI_INTDIV_LSB      3
#define MWSGEN_WCI_INTDIV_WIDTH    ((uint32_t)0x0000000D)
#define MWSGEN_WCI_FRACTDIV_MASK   ((uint32_t)0x00000007)
#define MWSGEN_WCI_FRACTDIV_LSB    0
#define MWSGEN_WCI_FRACTDIV_WIDTH  ((uint32_t)0x00000003)

#define MWSGEN_WCIIFSCNT_RST       0x0
#define MWSGEN_WCI_INTDIV_RST      0x0
#define MWSGEN_WCI_FRACTDIV_RST    0x0

__INLINE void mwsgen_wcicntl0_pack(uint8_t wciifscnt, uint16_t wciintdiv, uint8_t wcifractdiv)
{
    ASSERT_ERR((((uint32_t)wciifscnt << 16) & ~((uint32_t)0x000F0000)) == 0);
    ASSERT_ERR((((uint32_t)wciintdiv << 3) & ~((uint32_t)0x0000FFF8)) == 0);
    ASSERT_ERR((((uint32_t)wcifractdiv << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL0_ADDR, ((uint32_t)wciifscnt << 16) | ((uint32_t)wciintdiv << 3) | ((uint32_t)wcifractdiv << 0));
}

__INLINE void mwsgen_wcicntl0_unpack(uint8_t *wciifscnt, uint16_t *wciintdiv, uint8_t *wcifractdiv)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL0_ADDR);

    *wciifscnt = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *wciintdiv = (localVal & ((uint32_t)0x0000FFF8)) >> 3;
    *wcifractdiv = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t mwsgen_wciifscnt_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void mwsgen_wciifscnt_setf(uint8_t wciifscnt)
{
    ASSERT_ERR((((uint32_t)wciifscnt << 16) & ~((uint32_t)0x000F0000)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL0_ADDR, (REG_PL_RD(MWSGEN_WCICNTL0_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)wciifscnt << 16));
}

__INLINE uint16_t mwsgen_wci_intdiv_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x0000FFF8)) >> 3);
}

__INLINE void mwsgen_wci_intdiv_setf(uint16_t wciintdiv)
{
    ASSERT_ERR((((uint32_t)wciintdiv << 3) & ~((uint32_t)0x0000FFF8)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL0_ADDR, (REG_PL_RD(MWSGEN_WCICNTL0_ADDR) & ~((uint32_t)0x0000FFF8)) | ((uint32_t)wciintdiv << 3));
}

__INLINE uint8_t mwsgen_wci_fractdiv_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL0_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void mwsgen_wci_fractdiv_setf(uint8_t wcifractdiv)
{
    ASSERT_ERR((((uint32_t)wcifractdiv << 0) & ~((uint32_t)0x00000007)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL0_ADDR, (REG_PL_RD(MWSGEN_WCICNTL0_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)wcifractdiv << 0));
}

/**
 * @brief WCICNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            WCIIRQACK   0
 *     16     RESEND_REAL_TIME   0
 *     15           SENDWCIMSG   0
 *  10:08             TXIDTYPE   0x0
 *  07:00               TXDATA   0x0
 * </pre>
 */
#define MWSGEN_WCICNTL1_ADDR   0x10009034
#define MWSGEN_WCICNTL1_OFFSET 0x00000034
#define MWSGEN_WCICNTL1_INDEX  0x0000000D
#define MWSGEN_WCICNTL1_RESET  0x00000000

__INLINE uint32_t mwsgen_wcicntl1_get(void)
{
    return REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
}

__INLINE void mwsgen_wcicntl1_set(uint32_t value)
{
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, value);
}

// field definitions
#define MWSGEN_WCIIRQACK_BIT           ((uint32_t)0x80000000)
#define MWSGEN_WCIIRQACK_POS           31
#define MWSGEN_RESEND_REAL_TIME_BIT    ((uint32_t)0x00010000)
#define MWSGEN_RESEND_REAL_TIME_POS    16
#define MWSGEN_SENDWCIMSG_BIT          ((uint32_t)0x00008000)
#define MWSGEN_SENDWCIMSG_POS          15
#define MWSGEN_TXIDTYPE_MASK           ((uint32_t)0x00000700)
#define MWSGEN_TXIDTYPE_LSB            8
#define MWSGEN_TXIDTYPE_WIDTH          ((uint32_t)0x00000003)
#define MWSGEN_TXDATA_MASK             ((uint32_t)0x000000FF)
#define MWSGEN_TXDATA_LSB              0
#define MWSGEN_TXDATA_WIDTH            ((uint32_t)0x00000008)

#define MWSGEN_WCIIRQACK_RST           0x0
#define MWSGEN_RESEND_REAL_TIME_RST    0x0
#define MWSGEN_SENDWCIMSG_RST          0x0
#define MWSGEN_TXIDTYPE_RST            0x0
#define MWSGEN_TXDATA_RST              0x0

__INLINE void mwsgen_wcicntl1_pack(uint8_t wciirqack, uint8_t resendrealtime, uint8_t sendwcimsg, uint8_t txidtype, uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)wciirqack << 31) & ~((uint32_t)0x80000000)) == 0);
    ASSERT_ERR((((uint32_t)resendrealtime << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)sendwcimsg << 15) & ~((uint32_t)0x00008000)) == 0);
    ASSERT_ERR((((uint32_t)txidtype << 8) & ~((uint32_t)0x00000700)) == 0);
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, ((uint32_t)wciirqack << 31) | ((uint32_t)resendrealtime << 16) | ((uint32_t)sendwcimsg << 15) | ((uint32_t)txidtype << 8) | ((uint32_t)txdata << 0));
}

__INLINE void mwsgen_wcicntl1_unpack(uint8_t *wciirqack, uint8_t *resendrealtime, uint8_t *sendwcimsg, uint8_t *txidtype, uint8_t *txdata)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);

    *wciirqack = (localVal & ((uint32_t)0x80000000)) >> 31;
    *resendrealtime = (localVal & ((uint32_t)0x00010000)) >> 16;
    *sendwcimsg = (localVal & ((uint32_t)0x00008000)) >> 15;
    *txidtype = (localVal & ((uint32_t)0x00000700)) >> 8;
    *txdata = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mwsgen_wciirqack_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void mwsgen_wciirqack_setf(uint8_t wciirqack)
{
    ASSERT_ERR((((uint32_t)wciirqack << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, (REG_PL_RD(MWSGEN_WCICNTL1_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)wciirqack << 31));
}

__INLINE uint8_t mwsgen_resend_real_time_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void mwsgen_resend_real_time_setf(uint8_t resendrealtime)
{
    ASSERT_ERR((((uint32_t)resendrealtime << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, (REG_PL_RD(MWSGEN_WCICNTL1_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)resendrealtime << 16));
}

__INLINE uint8_t mwsgen_sendwcimsg_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void mwsgen_sendwcimsg_setf(uint8_t sendwcimsg)
{
    ASSERT_ERR((((uint32_t)sendwcimsg << 15) & ~((uint32_t)0x00008000)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, (REG_PL_RD(MWSGEN_WCICNTL1_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)sendwcimsg << 15));
}

__INLINE uint8_t mwsgen_txidtype_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void mwsgen_txidtype_setf(uint8_t txidtype)
{
    ASSERT_ERR((((uint32_t)txidtype << 8) & ~((uint32_t)0x00000700)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, (REG_PL_RD(MWSGEN_WCICNTL1_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)txidtype << 8));
}

__INLINE uint8_t mwsgen_txdata_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCICNTL1_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void mwsgen_txdata_setf(uint8_t txdata)
{
    ASSERT_ERR((((uint32_t)txdata << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_PL_WR(MWSGEN_WCICNTL1_ADDR, (REG_PL_RD(MWSGEN_WCICNTL1_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)txdata << 0));
}

/**
 * @brief WCISTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            WCIIRQSAT   0
 *  10:08             RXIDTYPE   0x0
 *  07:00           RXDATATYPE   0x0
 * </pre>
 */
#define MWSGEN_WCISTAT_ADDR   0x10009038
#define MWSGEN_WCISTAT_OFFSET 0x00000038
#define MWSGEN_WCISTAT_INDEX  0x0000000E
#define MWSGEN_WCISTAT_RESET  0x00000000

__INLINE uint32_t mwsgen_wcistat_get(void)
{
    return REG_PL_RD(MWSGEN_WCISTAT_ADDR);
}

// field definitions
#define MWSGEN_WCIIRQSAT_BIT     ((uint32_t)0x80000000)
#define MWSGEN_WCIIRQSAT_POS     31
#define MWSGEN_RXIDTYPE_MASK     ((uint32_t)0x00000700)
#define MWSGEN_RXIDTYPE_LSB      8
#define MWSGEN_RXIDTYPE_WIDTH    ((uint32_t)0x00000003)
#define MWSGEN_RXDATATYPE_MASK   ((uint32_t)0x000000FF)
#define MWSGEN_RXDATATYPE_LSB    0
#define MWSGEN_RXDATATYPE_WIDTH  ((uint32_t)0x00000008)

#define MWSGEN_WCIIRQSAT_RST     0x0
#define MWSGEN_RXIDTYPE_RST      0x0
#define MWSGEN_RXDATATYPE_RST    0x0

__INLINE void mwsgen_wcistat_unpack(uint8_t *wciirqsat, uint8_t *rxidtype, uint8_t *rxdatatype)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCISTAT_ADDR);

    *wciirqsat = (localVal & ((uint32_t)0x80000000)) >> 31;
    *rxidtype = (localVal & ((uint32_t)0x00000700)) >> 8;
    *rxdatatype = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t mwsgen_wciirqsat_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCISTAT_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t mwsgen_rxidtype_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCISTAT_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE uint8_t mwsgen_rxdatatype_getf(void)
{
    uint32_t localVal = REG_PL_RD(MWSGEN_WCISTAT_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}


#endif // _REG_MWSGEN_H_

