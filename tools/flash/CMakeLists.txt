cmake_minimum_required(VERSION 3.20)

include(cmake/toolchain-arm-none-eabi-gcc.cmake)

project(sifli_flash_loaders C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

get_filename_component(FLASH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
get_filename_component(SDK_ROOT "${FLASH_ROOT}/../.." ABSOLUTE)

set(FLASH_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
set(FLASH_STAGE_DIR "${CMAKE_BINARY_DIR}/stage")

include("${FLASH_ROOT}/cmake/flash_targets.cmake")

sifli_flash_configure_version_header("${SDK_ROOT}" "${FLASH_GENERATED_DIR}")
sifli_flash_define_all_targets()
