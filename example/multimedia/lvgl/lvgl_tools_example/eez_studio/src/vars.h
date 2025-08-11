#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations



// Flow global variables

enum FlowGlobalVariables
{
    FLOW_GLOBAL_VARIABLE_NONE
};

// Native global variables

extern int32_t get_var_flow_count();
extern void set_var_flow_count(int32_t value);
extern int32_t get_var_native_count();
extern void set_var_native_count(int32_t value);


void vars_init();
#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/