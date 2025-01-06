/**
 ****************************************************************************************
 *
 * @file ke_lp.h
 *
 * @brief Contains SIFLI kernel low power definition.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */


#ifndef _KE_LP_H_
#define _KE_LP_H_


/*
* INCLUDE FILES
****************************************************************************************
*/

/*
* ENUMERATION
****************************************************************************************
*/

/*
* FUNCTION DECLARATIONS
****************************************************************************************
*/

typedef int (*ke_sleep_os_hook_handler)(void);

typedef void (*sifli_standby_recovery_handler)(void);

extern ke_sleep_os_hook_handler _ke_sleep_os_hook_handler;

extern sifli_standby_recovery_handler _sifli_standby_recovery_handler;

#define ke_sleep_os_hook() _ke_sleep_os_hook_handler()

#define sifli_standby_recovery() _sifli_standby_recovery_handler()

int _ke_sleep_os_hook(void);

bool ke_MAC_sleep_check(void);

void ke_MAC_sleep_wakeup(bool wakeup);

void _sifli_standby_recovery(void);


#endif

