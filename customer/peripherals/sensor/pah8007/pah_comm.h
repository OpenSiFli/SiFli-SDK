
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

//============================================================================
// Functions
//============================================================================
#ifdef __cplusplus
extern  "C" {
#endif
#ifndef __FILENAME__
#   define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#endif

//============================================================================
// Option
//============================================================================
#define PAH_8007_ENABLE_DEBUG_PRINT
#define PAH_8007_ENABLE_LOG_PRINT

#ifdef PAH_8007_ENABLE_LOG_PRINT
#define LOG_PRINT(...) printf(__VA_ARGS__)
#else
#define LOG_PRINT(...) do {} while (0)
#endif

#ifdef PAH_8007_ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do {} while (0)
#endif

/*
void pah_print(
    int32_t                         fileline,
    const char                      *funcname,
    const char                      *filename,
    const char                      *format, ...);
*/

void pah_delay_ms(uint32_t ms);

uint64_t pah_get_tick_count(void);

void pah_comm_i2c_write(uint8_t addr, const uint8_t *data);
void pah_comm_i2c_read(uint8_t addr, uint8_t *data, size_t read_size);

#ifdef __cplusplus
}
#endif
