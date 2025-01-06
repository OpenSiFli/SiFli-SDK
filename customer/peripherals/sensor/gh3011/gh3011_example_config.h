/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_config.h
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */

#ifndef _GH3011_EXAMPLE_CONFIG_H_
#define _GH3011_EXAMPLE_CONFIG_H_


// application config

/* common */

/// gh30x communicate interface type: <1=> i2c, <2=> spi
#define __GH30X_COMMUNICATION_INTERFACE__   (GH30X_COMMUNICATION_INTERFACE_I2C)

/// gh30x default irq pluse width 20us, if need config irq pluse width, search this macro.
#define __GH30X_IRQ_PLUSE_WIDTH_CONFIG__    (1)

/// platform delay us config
#define __PLATFORM_DELAY_US_CONFIG__        (1)

/// gsensor sensitivity normalized
#define __GS_SENSITIVITY_CONFIG__           (HBD_GSENSOR_SENSITIVITY_512_COUNTS_PER_G)

/// retry max cnt
#define __RETRY_MAX_CNT_CONFIG__            (100)

/// reinit max cnt
#define __RESET_REINIT_CNT_CONFIG__         (5)

/// dbg data output config
#define __ALGO_CALC_WITH_DBG_DATA__         (1)

/// dbg buffer len, should define >= max fifo thr of all func enabled
#define __ALGO_CALC_DBG_BUFFER_LEN__        (105)

/// fifo int timeout check
#define __FIFO_INT_TIMEOUT_CHECK__          (0)


/* hb */

/// hb detect support
#define __HB_DET_SUPPORT__                  (1)

/// hb fifo thr cnt config
#define __HB_FIFO_THR_CNT_CONFIG__          (25)

/// hb wearing algo enable flag
#define __HBA_ENABLE_WEARING__              (1)

/// whether need gsensor motion before start adt detect
#define __HB_START_WITH_GSENSOR_MOTION__    (1)

/// hb adt confirm for wear detect algo
#define __HB_NEED_ADT_CONFIRM__             (1)

/// hb adt confirm for wear detect algo parameter
#define __HB_ADT_CONFIRM_GS_AMP__           (20) // mean amp (val/512*1g)
#define __HB_ADT_CONFIRM_CHECK_CNT__        (22)
#define __HB_ADT_CONFIRM_THR_CNT__          (20)


/* spo2 */

/// spo2 detect support
#define __SPO2_DET_SUPPORT__                (1)

/// spo2 fifo thr cnt config
#define __SPO2_FIFO_THR_CNT_CONFIG__        (100)


/* hrv */

/// hrv detect support
#define __HRV_DET_SUPPORT__                (0)

/// hrv fifo thr cnt config
#define __HRV_FIFO_THR_CNT_CONFIG__        (25)


/* hsm */

/// hsm detect support
#define __HSM_DET_SUPPORT__                (0)

/// hsm fifo thr cnt config
#define __HSM_FIFO_THR_CNT_CONFIG__        (25)


/* bpd */

/// bpd detect support
#define __BPD_DET_SUPPORT__                (0)

/// bpd fifo thr cnt config
#define __BPD_FIFO_THR_CNT_CONFIG__        (25)


/* pfa */

/// pfa detect support
#define __PFA_DET_SUPPORT__                (0)

/// pfa fifo thr cnt config
#define __PFA_FIFO_THR_CNT_CONFIG__        (25)


/* ble */

/// if need use GOODIX app debug
#define __USE_GOODIX_APP__                  (1)

/// mcu mode data to GOODIX app debug, must __USE_GOODIX_APP__ set 1
#define __GOODIX_APP_MCU_SUP__              (1)

/// if need mutli pkg, = (__NEW_DATA_MULTI_PKG_NUM__ + 1), so set 1 that send with two rawdata cmd
#define __NEW_DATA_MULTI_PKG_NUM__          (4) // mean 5

/// ble pkg size max suport, should < (mtu - 3)
#define __BLE_PKG_SIZE_MAX__                (180)

/// ble mcu mode pkg buffer len
#define __BLE_MCU_PKG_BUFFER_MAX_LEN__      (1500)


/* uart */

/// uart connect to goodix dongle or production test tools
#define __UART_WITH_GOODIX_TOOLS__          (0)


/* system test */

/// system test moudle support
#define __SYSTEM_TEST_SUPPORT__             (0)

/// system test moudle cnt check noise, must <= 100
#define __SYSTEM_TEST_DATA_CNT_CONFIG__     (100)

/* dbg log lvl */

/// log debug lvl: <0=> off , <1=> normal info ,  <2=> with data info
#define __EXAMPLE_DEBUG_LOG_LVL__           (1)

/// log support len
#define __EXAMPLE_LOG_DEBUG_SUP_LEN__       (128)


#endif /* _GH3011_EXAMPLE_CONFIG_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
