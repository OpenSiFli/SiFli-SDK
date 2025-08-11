/*
 *  Copyright 2008-2022 NXP
 *
 *  Licensed under the LA_OPT_NXP_Software_License.txt (the "Agreement")
 *
 */

/*! \file wlan_mcu_mem_access_cli.h
 * \brief MCU Mem Read/Write.
 */

#ifdef CONFIG_MCU_MEM_ACCESS

    /** Register MCU mem read/write CLI commands
    * This function registers MCU mem read/write CLI commands
    *
    * \note This function gets called by \ref wlan_cli_init().
    *
    * \return WLAN_ERROR_NONE if the CLI commands were registered or
    * \return WLAN_ERROR_ACTION if they were not registered (for example
    *   if this function was called while the CLI commands were already
    *   registered).
    */
    int mcu_mem_access_init(void);

#endif /*CONFIG_MCU_MEM_ACCESS */
