/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_LCPU_INIT_H__
#define __BF0_LCPU_INIT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @addtogroup LCPU_INIT LCPU power and ROM configuration
 * @ingroup CMSIS_Device
 * @brief Interface exported by drivers/cmsis/<chip>/bf0_lcpu_init.c
 *
 * This file is the header counterpart of bf0_lcpu_init.c, which exists once per chip
 * series (sf32lb52x / sf32lb55x / sf32lb56x / sf32lb57x / sf32lb58x) and is only
 * compiled into HCPU images (see the SConscript of each chip directory).
 *
 * API availability
 *
 * | symbol                  | 52x | 55x | 56x | 57x | 58x | note                        |
 * |-------------------------|-----|-----|-----|-----|-----|-----------------------------|
 * | lcpu_power_on           |  Y  |  Y  |  Y  |  Y  |  Y  | stub on LCPU (returns 0)    |
 * | lcpu_power_off          |  Y  |  Y  |  Y  |  Y  |  Y  | HCPU only                   |
 * | lcpu_disable_rf_cal     |  Y  |  Y  |  Y  |  Y  |  Y  | HCPU only                   |
 * | lcpu_rom_config         |  Y  |  Y  |  Y  |  Y  |  Y  | __WEAK, app may override    |
 * | lcpu_rom_config_default |  Y  |  -  |  -  |  Y  |  Y  | 58x needs LCPU_CONFIG_V2    |
 * | lcpu_img_install        |  Y  |  Y  |  Y  |  Y  |  Y  | __WEAK, may be macro-empty  |
 * | adc_resume              |  Y  |  Y  |  Y  |  Y  |  Y  | __WEAK, app may override    |
 * | lcpu_nvds_config        |  Y  |  -  |  -  |  -  |  -  | __WEAK, HCPU only           |
 *
 * Functions marked __WEAK are defined with a default (usually empty) body inside
 * bf0_lcpu_init.c and can be redefined by the application without touching the SDK,
 * the same way HAL callbacks are overridden. lcpu_img_install() is turned into an
 * empty macro by LCPU_RUN_ROM_ONLY (52x: SF32LB52X_REV_B also), in that case no
 * symbol is emitted at all.
 * @{
 ****************************************************************************************
 */

/**
 * @brief Power on the peer LCPU core and release it from reset
 *
 * Sequence: wake core, halt and reset LCPU, apply ROM configuration
 * (lcpu_rom_config), install LCPU image if needed (lcpu_img_install), set start
 * address, install BT patch and RF calibration (lcpu_ble_patch_install), release
 * reset. On 52x/57x the LPSYS HCLK is divided down to 24MHz or below before the
 * code is loaded, as required by the LCPU boot ROM.
 *
 * @retval 0 always returned currently, reserved for error reporting
 */
uint8_t lcpu_power_on(void);

#ifdef SOC_BF0_HCPU

/**
 * @brief Reset and halt the peer LCPU core
 *
 * @retval 0 always returned currently, reserved for error reporting
 */
uint8_t lcpu_power_off(void);

/**
 * @brief Skip or allow the RF calibration done while powering on LCPU
 *
 * The flag is only sampled inside lcpu_power_on(), it has no effect after the
 * LCPU has been released from reset.
 *
 * @param[in] is_disable 1: skip RF calibration, 0: run RF calibration
 */
void lcpu_disable_rf_cal(uint8_t is_disable);

/**
 * @brief Hook called by lcpu_power_on() to program the LCPU boot ROM parameters
 *
 * __WEAK, can be overridden to fully replace the ROM configuration. The default
 * body is chip dependent: 52x and 57x call lcpu_rom_config_default(); 58x calls it
 * only when LCPU_CONFIG_V2 is defined; 55x writes HAL_LCPU_CONFIG_CHIP_REV only;
 * 56x is empty.
 */
void lcpu_rom_config(void);

#if defined(SF32LB52X) || defined(SF32LB57X) || \
    (defined(SF32LB58X) && defined(LCPU_CONFIG_V2))

/**
 * @brief Program the default LCPU boot ROM parameters
 *
 * Sets WDT status/time/clock, LXT enable and BT RC calibration flags, plus the
 * HCPU->LCPU mailbox queue address and bluetooth ROM config on 52x/58x. Called by
 * the default body of lcpu_rom_config(). Not weak: overriding lcpu_rom_config()
 * is the supported way to change the configuration.
 */
void lcpu_rom_config_default(void);

#endif /* SF32LB52X || SF32LB57X || (SF32LB58X && LCPU_CONFIG_V2) */

/*
 * lcpu_img_install() is not a callable API in every build: when LCPU_RUN_ROM_ONLY
 * is defined (any series), or when SF32LB52X_REV_B is defined on 52x, bf0_lcpu_init.c
 * turns the name into an empty function-like macro and no symbol is emitted, so a
 * call from another translation unit would fail at link time. Declare it only on the
 * configurations where the definition really exists, matching the #if that guards the
 * __WEAK definition in bf0_lcpu_init.c.
 */
#if !defined(LCPU_RUN_ROM_ONLY) && \
    !(defined(SF32LB52X) && defined(SF32LB52X_REV_B))

/**
 * @brief Hook called by lcpu_power_on() to copy the LCPU patch/image into its RAM
 *
 * __WEAK with an empty default body; the strong definition is provided by the image
 * project (middleware/bluetooth/patch/... or example/rom_bin/...). Intended to be
 * overridden, not called by application code.
 */
void lcpu_img_install(void);

#endif /* !LCPU_RUN_ROM_ONLY && !(SF32LB52X && SF32LB52X_REV_B) */

/**
 * @brief Hook called after the BT patch installation to restore the ADC
 *
 * __WEAK with an empty default body, meant to be overridden by boards that keep
 * analog blocks powered down across the LCPU boot.
 */
void adc_resume(void);

#if defined(SF32LB52X)

/**
 * @brief Hook called by lcpu_power_on() before the ROM configuration
 *
 * __WEAK with an empty default body. Only 52x provides this hook; it is invoked
 * between the LCPU reset and lcpu_rom_config().
 */
void lcpu_nvds_config(void);

#endif /* SF32LB52X */

#endif /* SOC_BF0_HCPU */

/// @} LCPU_INIT

#ifdef __cplusplus
}
#endif

#endif /* __BF0_LCPU_INIT_H__ */
