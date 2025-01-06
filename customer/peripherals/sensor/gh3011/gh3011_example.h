/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example.h
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */

#ifndef _GH3011_EXAMPLE_H_
#define _GH3011_EXAMPLE_H_


#include "gh3011_example_common.h"


/**
 * @brief start run mode enum
 */
typedef enum
{
    GH30X_RUN_MODE_HB = RUN_MODE_ADT_HB_DET,
    GH30X_RUN_MODE_HRV = RUN_MODE_HRV_DET,
    GH30X_RUN_MODE_HSM = RUN_MODE_HSM_DET,
    GH30X_RUN_MODE_BPD = RUN_MODE_BPD_DET,
    GH30X_RUN_MODE_PFA = RUN_MODE_PFA_DET,
    GH30X_RUN_MODE_SPO2 = RUN_MODE_SPO2_DET,
} EMGh30xRunMode;

/// hb algo scenario, call before start: <0=> default, <1=> routine, <2=> run, <3=> clibm,  <4=> bike, <5=> irregular
#define GH30X_HBA_SCENARIO_CONFIG(sce)         do { HBD_HbAlgoScenarioConfig(sce); } while(0)

/// gh30x module init
int gh30x_module_init(void);

/// gh30x module start, with adt
void gh30x_module_start(EMGh30xRunMode start_run_mode);

/// gh30x module start, without adt
void gh30x_module_start_without_adt(EMGh30xRunMode start_run_mode);

/// gh30x module stop
void gh30x_module_stop(void);

/// ble recv data handler
void ble_module_recv_data_via_gdcs(uint8_t *data, uint8_t length);

/// uart recv data handler
void uart_module_recv_data(uint8_t *data, uint8_t length);

/// gh30x module system test otp check
void gh30x_module_system_test_otp_check(void);

/// gh30x module system test os check
void gh30x_module_system_test_os_start(void);

#endif /* _GH3011_EXAMPLE_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
