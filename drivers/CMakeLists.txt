#
# Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(Include) # dir name with lower case to get good score in zephyr github CI
zephyr_include_directories(cmsis/Include)

zephyr_include_directories(cmsis/sf32lb52x)
zephyr_include_directories_ifdef(CONFIG_SOC_SERIES_SF32LB52X cmsis/sf32lb52x)

zephyr_library()

zephyr_library_sources(hal/bf0_hal.c)
zephyr_library_sources(hal/bf0_hal_cortex.c)
zephyr_library_sources(hal/bf0_hal_efuse.c)
zephyr_library_sources(hal/bf0_hal_pmu.c)
zephyr_library_sources(hal/bf0_hal_rcc.c)
zephyr_library_sources(cmsis/sf32lb52x/Templates/system_bf0_ap.c)

zephyr_library_sources_ifdef(CONFIG_GPIO_SF32LB hal/bf0_hal_gpio.c)
zephyr_library_sources_ifdef(CONFIG_I2C_SF32LB hal/bf0_hal_i2c.c)
