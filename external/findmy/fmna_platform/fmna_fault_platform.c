/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

// #include "nrf_log_ctrl.h"

// #include "fmna_platform_includes.h"
// #include "fmna_constants.h"
// #include "fmna_crypto.h"

// /// Function for handling asserts in the SoftDevice.
// /// @param line_num Line number of the failing assert call.
// /// @param p_file_name File name of the failing assert call.
// void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
// {
//     app_error_handler(0xDEADBEEF, line_num, p_file_name);
// }

// /// Handle various SW faults.
// void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
//     // Disable IRQs as we are about to handle the fault and reset.
//     __disable_irq();

//     // Get any pending logs out.
//     NRF_LOG_FINAL_FLUSH();

//     assert_info_t * p_info;
//     error_info_t  * p_info_err;

//     fmna_log_mfi_token();

//     switch(id) {
// #if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
//         case NRF_FAULT_ID_SD_ASSERT:
//             NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED %08x", pc);
//             p_info = (assert_info_t *)info;
//             NRF_LOG_ERROR("SD ASSERTION FAILED at %s:%u", p_info->p_file_name, p_info->line_num);
//             break;
// #endif

//         case NRF_FAULT_ID_SDK_ASSERT:
//             p_info = (assert_info_t *)info;
//             NRF_LOG_ERROR("ASSERTION FAILED at %s:%u", p_info->p_file_name, p_info->line_num);
//             break;

//         case NRF_FAULT_ID_SDK_ERROR:
//             p_info_err = (error_info_t *)info;
//             NRF_LOG_ERROR("ERROR 0x%08x [%s] at %s:%u", p_info_err->err_code, nrf_strerror_get(p_info_err->err_code), p_info_err->p_file_name, p_info_err->line_num)
//             NRF_LOG_ERROR("PC 0x%08x", pc);
//             break;

//         case NRF_FAULT_ID_APP_MEMACC:
//             NRF_LOG_ERROR("NRF_FAULT_ID_APP_MEMACC -- INFO: 0x%08x", info);
//             break;

//         default:
//             NRF_LOG_ERROR("Unrecognized Error ID, 0x%08x", id);
//             break;
//     }

//     NRF_LOG_WARNING("System reset");
//     NVIC_SystemReset();
// }
