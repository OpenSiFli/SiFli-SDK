#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0
#

function(sifli_flash_configure_version_header sdk_root output_dir)
  file(READ "${sdk_root}/.version" version_text)

  string(REGEX MATCH "MAJOR=([0-9]+)" _ "${version_text}")
  set(major "${CMAKE_MATCH_1}")
  string(REGEX MATCH "MINOR=([0-9]+)" _ "${version_text}")
  set(minor "${CMAKE_MATCH_1}")
  string(REGEX MATCH "REV=([0-9]+)" _ "${version_text}")
  set(rev "${CMAKE_MATCH_1}")

  math(EXPR flash_version "((${major}) << 24) + ((${minor}) << 16) + (${rev})")

  file(MAKE_DIRECTORY "${output_dir}")
  file(WRITE "${output_dir}/flashdrv_version.h"
    "#ifndef FLASHDRV_VERSION_H_\n"
    "#define FLASHDRV_VERSION_H_\n"
    "#define FLASH_JLINK_VERSION ${flash_version}\n"
    "#endif\n"
  )

  set(FLASH_VERSION_VALUE "${flash_version}" PARENT_SCOPE)
  set(FLASH_VERSION_HEADER "${output_dir}/flashdrv_version.h" PARENT_SCOPE)
endfunction()

function(sifli_flash_target_includes out_var soc app_src_dir)
  set(includes
    "${FLASH_ROOT}/src"
    "${app_src_dir}"
    "${FLASH_ROOT}/project/src"
    "${SDK_ROOT}/drivers/Include"
    "${SDK_ROOT}/drivers/hal"
    "${SDK_ROOT}/drivers/cmsis/Include"
    "${SDK_ROOT}/drivers/cmsis/${soc}"
    "${SDK_ROOT}/external/CMSIS/Include"
    "${SDK_ROOT}/customer/boards/include"
    "${SDK_ROOT}/customer/boards/include/config/${soc}"
    "${FLASH_GENERATED_DIR}"
  )
  set("${out_var}" "${includes}" PARENT_SCOPE)
endfunction()

function(sifli_flash_add_loader)
  set(options)
  set(one_value_args
    TARGET
    KIND
    SOC
    APP_SRC_DIR
    OUTPUT_SUBDIR
    OUTPUT_NAME
    SUFFIX
    LINKER
    ENTRY
    DEV_NAME
    DEV_BASE
    DEV_SIZE
    DEV_PAGE_SIZE
    DEV_SECTOR_SIZE
    DEV_ERASED_VALUE
    DEV_TIMEOUT_PROG
    DEV_TIMEOUT_ERASE
  )
  set(multi_value_args SOURCES DEFINES)
  cmake_parse_arguments(LOADER "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT LOADER_TARGET)
    message(FATAL_ERROR "sifli_flash_add_loader requires TARGET")
  endif()

  sifli_flash_target_includes(target_includes "${LOADER_SOC}" "${LOADER_APP_SRC_DIR}")

  set(target_defines ${LOADER_DEFINES})
  if(LOADER_KIND STREQUAL "cmsis")
    list(FIND target_defines KEIL has_keil_define)
    if(has_keil_define EQUAL -1)
      list(APPEND target_defines JLINK)
    endif()
  endif()
  list(JOIN target_defines ";" target_defines_joined)
  if(target_defines_joined MATCHES "(^|;)JLINK_(SDIO|SDEMMC)_[0-9]+($|;)")
    list(APPEND target_defines FLASH_DEVICE_USE_SDIO_OS)
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/maps")
  set(map_file "${CMAKE_BINARY_DIR}/maps/${LOADER_TARGET}.map")
  set(stage_dir "${FLASH_STAGE_DIR}/${LOADER_OUTPUT_SUBDIR}")
  set(stage_file "${stage_dir}/${LOADER_OUTPUT_NAME}${LOADER_SUFFIX}")
  set(raw_output_dir "${CMAKE_BINARY_DIR}/raw/${LOADER_TARGET}")

  add_executable("${LOADER_TARGET}"
    ${LOADER_SOURCES}
    "${FLASH_ROOT}/src/min_runtime.c"
  )

  target_include_directories("${LOADER_TARGET}" PRIVATE ${target_includes})

  target_compile_definitions("${LOADER_TARGET}" PRIVATE
    ${target_defines}
    FLASH_DEVICE_NAME=${LOADER_DEV_NAME}
    FLASH_DEVICE_BASE=${LOADER_DEV_BASE}
    FLASH_DEVICE_SIZE=${LOADER_DEV_SIZE}
    FLASH_DEVICE_PAGE_SIZE_VALUE=${LOADER_DEV_PAGE_SIZE}
    FLASH_DEVICE_SECTOR_SIZE_VALUE=${LOADER_DEV_SECTOR_SIZE}
    FLASH_DEVICE_ERASED_VALUE=${LOADER_DEV_ERASED_VALUE}
    FLASH_DEVICE_TIMEOUT_PROG=${LOADER_DEV_TIMEOUT_PROG}
    FLASH_DEVICE_TIMEOUT_ERASE=${LOADER_DEV_TIMEOUT_ERASE}
  )

  target_compile_options("${LOADER_TARGET}" PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -Os
    -ffreestanding
    -fno-builtin
    -fdata-sections
    -ffunction-sections
    -fno-common
    -fno-unwind-tables
    -fno-asynchronous-unwind-tables
    -fno-exceptions
    -include
    "${FLASH_ROOT}/src/bf0_hal_conf_hcpu.h"
    -g
  )

  if(LOADER_KIND STREQUAL "cmsis")
    target_compile_options("${LOADER_TARGET}" PRIVATE
      -fpic
      -msingle-pic-base
      -mpic-register=r9
    )
  endif()

  target_link_options("${LOADER_TARGET}" PRIVATE
    -mcpu=cortex-m33
    -mthumb
    -nostartfiles
    -nostdlib
    -nodefaultlibs
    "-T${LOADER_LINKER}"
    "-Wl,-e,${LOADER_ENTRY}"
    "-Wl,-Map,${map_file}"
    -Wl,--gc-sections
    -Wl,--build-id=none
    -Wl,--no-warn-rwx-segments
    -Wl,--wrap=memcpy
    -Wl,--wrap=memmove
    -Wl,--wrap=memset
    -Wl,--wrap=memcmp
    -Wl,--wrap=strlen
    -Wl,--wrap=strcmp
    -Wl,--wrap=strncmp
    -Wl,--wrap=strcpy
    -Wl,--wrap=atoi
  )

  target_link_libraries("${LOADER_TARGET}" PRIVATE gcc)
  set_target_properties("${LOADER_TARGET}" PROPERTIES
    OUTPUT_NAME "${LOADER_OUTPUT_NAME}"
    SUFFIX "${LOADER_SUFFIX}"
    LINKER_LANGUAGE C
    RUNTIME_OUTPUT_DIRECTORY "${raw_output_dir}"
  )

  add_custom_command(TARGET "${LOADER_TARGET}" POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${stage_dir}"
    COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:${LOADER_TARGET}>" "${stage_file}"
    VERBATIM
  )

  set_property(GLOBAL APPEND PROPERTY SIFLI_FLASH_ALL_TARGETS "${LOADER_TARGET}")
  if(LOADER_KIND STREQUAL "jlink")
    set_property(GLOBAL APPEND PROPERTY SIFLI_FLASH_JLINK_TARGETS "${LOADER_TARGET}")
  elseif(LOADER_KIND STREQUAL "cmsis")
    set_property(GLOBAL APPEND PROPERTY SIFLI_FLASH_CMSIS_TARGETS "${LOADER_TARGET}")
  else()
    message(FATAL_ERROR "Unknown loader kind: ${LOADER_KIND}")
  endif()
endfunction()

set(_GENERIC_FLASH_DEV "${FLASH_ROOT}/src/FlashDev.c")
set(_GENERIC_SDIO_DEV "${_GENERIC_FLASH_DEV}")
set(_USER_CFG "${FLASH_ROOT}/project/src/user_cfg.c")

set(_HAL_52_NOR
  "${SDK_ROOT}/drivers/cmsis/sf32lb52x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_ext_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hlp.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi_ex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
  "${SDK_ROOT}/drivers/hal/nand_table.c"
)

set(_HAL_52_NAND "${_HAL_52_NOR}" "${SDK_ROOT}/drivers/hal/sifli_bbm.c")

set(_HAL_52_SDIO
  "${SDK_ROOT}/drivers/cmsis/sf32lb52x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_efuse.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hlp.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/bf0_sys_cfg.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
)

set(_HAL_55_NOR
  "${SDK_ROOT}/drivers/cmsis/sf32lb55x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_qspi.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_qspi_ex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
  "${SDK_ROOT}/drivers/hal/nand_table.c"
)

set(_HAL_55_SDIO
  "${SDK_ROOT}/drivers/cmsis/sf32lb55x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
)

set(_HAL_56_NOR
  "${SDK_ROOT}/drivers/cmsis/sf32lb56x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_ext_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hlp.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi_ex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
  "${SDK_ROOT}/drivers/hal/nand_table.c"
)

set(_HAL_56_NAND "${_HAL_56_NOR}" "${SDK_ROOT}/drivers/hal/sifli_bbm.c")

set(_HAL_56_SDIO
  "${SDK_ROOT}/drivers/cmsis/sf32lb56x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
)

set(_HAL_58_NOR
  "${SDK_ROOT}/drivers/cmsis/sf32lb58x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_ext_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_mpi_ex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
  "${SDK_ROOT}/drivers/hal/nand_table.c"
)

set(_HAL_58_NAND "${_HAL_58_NOR}" "${SDK_ROOT}/drivers/hal/sifli_bbm.c")

set(_HAL_58_SDIO
  "${SDK_ROOT}/drivers/cmsis/sf32lb58x/bf0_pin_const.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_cortex.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_dma.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_gpio.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_hpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_lpaon.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pinmux.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_pmu.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rcc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_rtc.c"
  "${SDK_ROOT}/drivers/hal/bf0_hal_uart.c"
  "${SDK_ROOT}/drivers/hal/flash_table.c"
)

set(_APP_55_NOR
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/src/FlashPrg.c"
  "${FLASH_ROOT}/project/src/main.c"
  "${FLASH_ROOT}/project/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_52_NOR
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb52x/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb52x/src/main.c"
  "${_USER_CFG}"
)

set(_APP_52_NAND
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb52x_nand/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb52x_nand/src/main.c"
  "${_USER_CFG}"
)

set(_APP_52_NAND_NOBBM
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb52x_nand_nobbm/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb52x_nand_nobbm/src/main.c"
  "${_USER_CFG}"
)

set(_APP_52_SDIO
  "${_GENERIC_SDIO_DEV}"
  "${FLASH_ROOT}/project/sf32lb52x_sdio/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb52x_sdio/src/main.c"
  "${FLASH_ROOT}/project/sf32lb52x_sdio/src/sd_emmc_ops.c"
  "${FLASH_ROOT}/project/sf32lb52x_sdio/src/sd_nand_drv.c"
  "${FLASH_ROOT}/project/sf32lb52x_sdio/src/sd_nand_ops.c"
  "${_USER_CFG}"
)

set(_APP_55_SDIO
  "${_GENERIC_SDIO_DEV}"
  "${FLASH_ROOT}/project/sf32lb55x_sdio/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb55x_sdio/src/main.c"
  "${FLASH_ROOT}/project/sf32lb55x_sdio/src/pmic_controller.c"
  "${FLASH_ROOT}/project/sf32lb55x_sdio/src/sdmmc_emmc.c"
  "${FLASH_ROOT}/project/sf32lb55x_sdio/src/sdmmc_tst_drv.c"
  "${_USER_CFG}"
)

set(_APP_56_NOR
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb56x/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb56x/src/main.c"
  "${FLASH_ROOT}/project/sf32lb56x/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_56_NAND
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb56x_nand/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb56x_nand/src/main.c"
  "${FLASH_ROOT}/project/sf32lb56x_nand/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_56_SDIO
  "${_GENERIC_SDIO_DEV}"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/main.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/pmic_controller.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/sdmmc1_drv.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/sdmmc1_emmc.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/sdmmc2_drv.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/sdmmc2_emmc_ops.c"
  "${FLASH_ROOT}/project/sf32lb56x_sdio/src/sdmmc2_nand_ops.c"
  "${_USER_CFG}"
)

set(_APP_58_NOR
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb58x/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb58x/src/main.c"
  "${FLASH_ROOT}/project/sf32lb58x/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_58_NAND
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb58x_nand/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb58x_nand/src/main.c"
  "${FLASH_ROOT}/project/sf32lb58x_nand/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_58_NAND_NOBBM
  "${_GENERIC_FLASH_DEV}"
  "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src/main.c"
  "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src/pmic_controller.c"
  "${_USER_CFG}"
)

set(_APP_58_SDIO
  "${_GENERIC_SDIO_DEV}"
  "${FLASH_ROOT}/project/sf32lb58x_sdio/src/FlashPrg.c"
  "${FLASH_ROOT}/project/sf32lb58x_sdio/src/main.c"
  "${FLASH_ROOT}/project/sf32lb58x_sdio/src/pmic_controller.c"
  "${FLASH_ROOT}/project/sf32lb58x_sdio/src/sdmmc_emmc.c"
  "${FLASH_ROOT}/project/sf32lb58x_sdio/src/sdmmc_tst_drv.c"
  "${_USER_CFG}"
)

function(sifli_flash_define_all_targets)
  set(jlink_linker "${FLASH_ROOT}/linker/jlink_open.ld")
  set(jlink_nand_linker "${FLASH_ROOT}/linker/jlink_open_nand.ld")
  set(cmsis_linker "${FLASH_ROOT}/linker/cmsis_flm.ld")

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x__flash1
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x/src"
    SOURCES ${_APP_52_NOR} ${_HAL_52_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x
    OUTPUT_NAME SF32LB52X_INT_FLASH1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X Internal Flash1\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x__flash2
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x/src"
    SOURCES ${_APP_52_NOR} ${_HAL_52_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x
    OUTPUT_NAME SF32LB52X_EXT_FLASH2
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Flash2\""
    DEV_BASE 0x12000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x__keil
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x/src"
    SOURCES ${_APP_52_NOR} ${_HAL_52_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG KEIL
    OUTPUT_SUBDIR keil_drv/sf32lb52x
    OUTPUT_NAME SF32LB52X
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB54X UNI_FLASH\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x60000000
    DEV_PAGE_SIZE 0x00002000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 3000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__nand2
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_NAND2
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__nand2_nobbm
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_ORG_NAND2
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__nand2_bigblk
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_BIGBLK_NAND2
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__nand2_bigblk_nobbm
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_ORG_BIGBLK_NAND2
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__cmsis_nand2
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_NAND2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__cmsis_nand2_nobbm
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_ORG_NAND2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__cmsis_nand2_bigblk
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_BIGBLK_NAND2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand__cmsis_nand2_bigblk_nobbm
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand/src"
    SOURCES ${_APP_52_NAND} ${_HAL_52_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_nand
    OUTPUT_NAME SF32LB52X_EXT_ORG_BIGBLK_NAND2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand_nobbm__flash2
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand_nobbm/src"
    SOURCES ${_APP_52_NAND_NOBBM} ${_HAL_52_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_nand_nobbm
    OUTPUT_NAME SF32LB52X_EXT_ORG_NAND2
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00020000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_nand_nobbm__cmsis_flash2
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_nand_nobbm/src"
    SOURCES ${_APP_52_NAND_NOBBM} ${_HAL_52_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_nand_nobbm
    OUTPUT_NAME SF32LB52X_EXT_ORG_NAND2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB52X External Nand2\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00020000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_sdio__sdio1
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_sdio/src"
    SOURCES ${_APP_52_SDIO} ${_HAL_52_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_SDIO_1 HAL_TICK_PER_SECOND=1000 _USE_PRODUCTLINE CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_sd
    OUTPUT_NAME SF32LB52X_EXT_SD1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB54X External SDIO1\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_sdio__emmc1
    KIND jlink
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_sdio/src"
    SOURCES ${_APP_52_SDIO} ${_HAL_52_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK JLINK_SDEMMC_1 HAL_TICK_PER_SECOND=1000 _USE_PRODUCTLINE CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb52x_sd
    OUTPUT_NAME SF32LB52X_EXT_EMMC1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB54X External SDIO1\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_sdio__cmsis_sdio1
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_sdio/src"
    SOURCES ${_APP_52_SDIO} ${_HAL_52_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_SDIO_1 HAL_TICK_PER_SECOND=1000 _USE_PRODUCTLINE CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_sd
    OUTPUT_NAME SF32LB52X_EXT_SD1
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB54X External SDIO1\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb52x_sdio__cmsis_emmc1
    KIND cmsis
    SOC sf32lb52x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb52x_sdio/src"
    SOURCES ${_APP_52_SDIO} ${_HAL_52_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB52X ARM_MATH_LOOPUNROLL JLINK_SDEMMC_1 HAL_TICK_PER_SECOND=1000 _USE_PRODUCTLINE CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb52x_sd
    OUTPUT_NAME SF32LB52X_EXT_EMMC1
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB54X External SDIO1\""
    DEV_BASE 0x62000000
    DEV_SIZE 0x3E000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x__flash1
    KIND jlink
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/src"
    SOURCES ${_APP_55_NOR} ${_HAL_55_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb55x
    OUTPUT_NAME SF32LB55X_INT_FLASH1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB55X Internal Flash1\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x00800000
    DEV_PAGE_SIZE 0x00002000
    DEV_SECTOR_SIZE 0x00002000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x__flash2
    KIND jlink
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/src"
    SOURCES ${_APP_55_NOR} ${_HAL_55_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb55x
    OUTPUT_NAME SF32LB55X_EXT_FLASH2
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB55X External Flash2\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x__flash3
    KIND jlink
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/src"
    SOURCES ${_APP_55_NOR} ${_HAL_55_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb55x
    OUTPUT_NAME SF32LB55X_EXT_FLASH3
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB55X External Flash3\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x08000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x__flash4
    KIND jlink
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/src"
    SOURCES ${_APP_55_NOR} ${_HAL_55_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_4 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb55x
    OUTPUT_NAME SF32LB55X_EXT_FLASH4
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB55X External Flash4\""
    DEV_BASE 0x12000000
    DEV_SIZE 0x00400000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x__keil
    KIND cmsis
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/src"
    SOURCES ${_APP_55_NOR} ${_HAL_55_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG KEIL
    OUTPUT_SUBDIR keil_drv/sf32lb55x
    OUTPUT_NAME SF32LB55X
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB55X UNI_FLASH\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x60000000
    DEV_PAGE_SIZE 0x00002000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 3000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x_sdio__sdio1
    KIND jlink
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb55x_sdio/src"
    SOURCES ${_APP_55_SDIO} ${_HAL_55_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK JLINK_SDIO_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb55x_sd
    OUTPUT_NAME SF32LB55X_EXT_SD1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB55X External SDIO1\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00020000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb55x_sdio__cmsis_sdio1
    KIND cmsis
    SOC sf32lb55x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb55x_sdio/src"
    SOURCES ${_APP_55_SDIO} ${_HAL_55_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB55X ARM_MATH_LOOPUNROLL JLINK_SDIO_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb55x_sd
    OUTPUT_NAME SF32LB55X_EXT_SD1
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB55X External SDIO1\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00020000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x__flash1
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x/src"
    SOURCES ${_APP_56_NOR} ${_HAL_56_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x
    OUTPUT_NAME SF32LB56X_INT_FLASH1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X Internal Flash1\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x00800000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x__flash2
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x/src"
    SOURCES ${_APP_56_NOR} ${_HAL_56_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x
    OUTPUT_NAME SF32LB56X_EXT_FLASH2
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External Flash2\""
    DEV_BASE 0x10400000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x__flash3
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x/src"
    SOURCES ${_APP_56_NOR} ${_HAL_56_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x
    OUTPUT_NAME SF32LB56X_EXT_FLASH3
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External Flash3\""
    DEV_BASE 0x14000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x__flash5
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x/src"
    SOURCES ${_APP_56_NOR} ${_HAL_56_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL _USE_PRODUCTLINE JLINK JLINK_FLASH_5 HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x
    OUTPUT_NAME SF32LB56X_FLASH5
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External Flash5\""
    DEV_BASE 0x1C000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x__keil
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x/src"
    SOURCES ${_APP_56_NOR} ${_HAL_56_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG KEIL
    OUTPUT_SUBDIR keil_drv/sf32lb56x
    OUTPUT_NAME SF32LB56X
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X UNI_FLASH\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x60000000
    DEV_PAGE_SIZE 0x00002000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 3000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__nand3
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__nand3_bigblk
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_BIGBLK_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__nand3_nobbm
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_ORG_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__nand3_nobbm_bigblk
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_ORG_BIGBLK_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__cmsis_nand3
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__cmsis_nand3_bigblk
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 BSP_USING_BBM PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_BIGBLK_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__cmsis_nand3_nobbm
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=131072 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_ORG_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_nand__cmsis_nand3_nobbm_bigblk
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_nand/src"
    SOURCES ${_APP_56_NAND} ${_HAL_56_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 PAGE_SIZE=262144 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_nand
    OUTPUT_NAME SF32LB56X_EXT_ORG_BIGBLK_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x20000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00040000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__sdio1
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_SDIO_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_SD1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External SDIO1\""
    DEV_BASE 0xA0000000
    DEV_SIZE 0x40000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__sdio2
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_SDIO_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_SD2
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External SDIO2\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x3C000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__emmc2
    KIND jlink
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK JLINK_SDEMMC_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_EMMC2
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB56X External SDIO2\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x3C000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__cmsis_sdio1
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_SDIO_1 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_SD1
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External SDIO1\""
    DEV_BASE 0xA0000000
    DEV_SIZE 0x40000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__cmsis_sdio2
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_SDIO_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_SD2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External SDIO2\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x3C000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb56x_sdio__cmsis_emmc2
    KIND cmsis
    SOC sf32lb56x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb56x_sdio/src"
    SOURCES ${_APP_56_SDIO} ${_HAL_56_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB56X ARM_MATH_LOOPUNROLL JLINK_SDEMMC_2 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb56x_sd
    OUTPUT_NAME SF32LB56X_EXT_EMMC2
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB56X External SDIO2\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x3C000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x__flash1
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x/src"
    SOURCES ${_APP_58_NOR} ${_HAL_58_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_1 HAL_TICK_PER_SECOND=1000 BSP_ENABLE_QSPI1 BSP_QSPI1_MEM_SIZE=4 BSP_QSPI1_MODE=0 BSP_QSPI1_USING_DMA CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x
    OUTPUT_NAME SF32LB58X_INT_FLASH1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X Internal Flash1\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x00800000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x__flash3
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x/src"
    SOURCES ${_APP_58_NOR} ${_HAL_58_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x
    OUTPUT_NAME SF32LB58X_EXT_FLASH3
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External Flash3\""
    DEV_BASE 0x14000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x__flash4
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x/src"
    SOURCES ${_APP_58_NOR} ${_HAL_58_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_4 CFG_BOOTLOADER HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x
    OUTPUT_NAME SF32LB58X_EXT_FLASH4
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External Flash4\""
    DEV_BASE 0x18000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x__flash5
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x/src"
    SOURCES ${_APP_58_NOR} ${_HAL_58_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_5 HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x
    OUTPUT_NAME SF32LB58X_FLASH5
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External Flash5\""
    DEV_BASE 0x1C000000
    DEV_SIZE 0x02000000
    DEV_PAGE_SIZE 0x00001000
    DEV_SECTOR_SIZE 0x00001000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x__keil
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x/src"
    SOURCES ${_APP_58_NOR} ${_HAL_58_NOR}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL CFG_BOOTLOADER KEIL HAL_TICK_PER_SECOND=1000 CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb58x
    OUTPUT_NAME SF32LB58X
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X UNI_FLASH\""
    DEV_BASE 0x10000000
    DEV_SIZE 0x60000000
    DEV_PAGE_SIZE 0x00002000
    DEV_SECTOR_SIZE 0x00010000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 3000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand__flash3
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand/src"
    SOURCES ${_APP_58_NAND} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x_nand
    OUTPUT_NAME SF32LB58X_EXT_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x04000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand__flash4
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand/src"
    SOURCES ${_APP_58_NAND} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_4 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x_nand
    OUTPUT_NAME SF32LB58X_EXT_NAND4
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External NAND4\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand__cmsis_flash3
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand/src"
    SOURCES ${_APP_58_NAND} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb58x_nand
    OUTPUT_NAME SF32LB58X_EXT_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X External NAND3\""
    DEV_BASE 0x64000000
    DEV_SIZE 0x04000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand__cmsis_flash4
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand/src"
    SOURCES ${_APP_58_NAND} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK_FLASH_4 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb58x_nand
    OUTPUT_NAME SF32LB58X_EXT_NAND4
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X External NAND4\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand_nobbm__flash3
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src"
    SOURCES ${_APP_58_NAND_NOBBM} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x_nand_nobbm
    OUTPUT_NAME SF32LB58X_EXT_ORG_NAND3
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External NAND3\""
    DEV_BASE 0x14000000
    DEV_SIZE 0x04000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand_nobbm__flash4
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src"
    SOURCES ${_APP_58_NAND_NOBBM} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_FLASH_4 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR jlink_drv/sf32lb58x_nand_nobbm
    OUTPUT_NAME SF32LB58X_EXT_ORG_NAND4
    SUFFIX .elf
    LINKER "${jlink_nand_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External NAND4\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand_nobbm__cmsis_flash3
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src"
    SOURCES ${_APP_58_NAND_NOBBM} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK_FLASH_3 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb58x_nand_nobbm
    OUTPUT_NAME SF32LB58X_EXT_ORG_NAND3
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X External NAND3\""
    DEV_BASE 0x14000000
    DEV_SIZE 0x04000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_nand_nobbm__cmsis_flash4
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_nand_nobbm/src"
    SOURCES ${_APP_58_NAND_NOBBM} ${_HAL_58_NAND}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK_FLASH_4 _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000 HAL_USE_NAND CFG_FACTORY_DEBUG
    OUTPUT_SUBDIR keil_drv/sf32lb58x_nand_nobbm
    OUTPUT_NAME SF32LB58X_EXT_ORG_NAND4
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X External NAND4\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00010000
    DEV_SECTOR_SIZE 0x00020000
    DEV_ERASED_VALUE 0xFF
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_sdio__sdio1
    KIND jlink
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_sdio/src"
    SOURCES ${_APP_58_SDIO} ${_HAL_58_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK JLINK_SDIO_1 CFG_FACTORY_DEBUG _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000
    OUTPUT_SUBDIR jlink_drv/sf32lb58x_sd
    OUTPUT_NAME SF32LB58X_EXT_SD1
    SUFFIX .elf
    LINKER "${jlink_linker}"
    ENTRY Init
    DEV_NAME "\"SF32LB58X External SDIO1\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  sifli_flash_add_loader(
    TARGET flash__sf32lb58x_sdio__cmsis_sdio1
    KIND cmsis
    SOC sf32lb58x
    APP_SRC_DIR "${FLASH_ROOT}/project/sf32lb58x_sdio/src"
    SOURCES ${_APP_58_SDIO} ${_HAL_58_SDIO}
    DEFINES SOC_BF0_HCPU USE_HAL_DRIVER SF32LB58X ARM_MATH_LOOPUNROLL JLINK_SDIO_1 CFG_FACTORY_DEBUG _USE_PRODUCTLINE HAL_TICK_PER_SECOND=1000
    OUTPUT_SUBDIR keil_drv/sf32lb58x_sd
    OUTPUT_NAME SF32LB58X_EXT_SD1
    SUFFIX .FLM
    LINKER "${cmsis_linker}"
    ENTRY 0
    DEV_NAME "\"SF32LB58X External SDIO1\""
    DEV_BASE 0x68000000
    DEV_SIZE 0x38000000
    DEV_PAGE_SIZE 0x00004000
    DEV_SECTOR_SIZE 0x00004000
    DEV_ERASED_VALUE 0x37
    DEV_TIMEOUT_PROG 6000
    DEV_TIMEOUT_ERASE 6000
  )

  get_property(all_targets GLOBAL PROPERTY SIFLI_FLASH_ALL_TARGETS)
  get_property(jlink_targets GLOBAL PROPERTY SIFLI_FLASH_JLINK_TARGETS)
  get_property(cmsis_targets GLOBAL PROPERTY SIFLI_FLASH_CMSIS_TARGETS)

  add_custom_target(flash_all DEPENDS ${all_targets})
  add_custom_target(flash_jlink_all DEPENDS ${jlink_targets})
  add_custom_target(flash_cmsis_all DEPENDS ${cmsis_targets})
endfunction()
