/*
 * Copyright 2008-2022, 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*!\file cli.h
 *\brief This file provides CLI interfaces to register commands in CLI mode.
 *
 * Modules that wish to register the commands should initialize the struct cli_command
 * structure and pass it to cli_register_command(). These commands will then
 * be available on the CLI.
 */

#ifndef __CLI_H__
#define __CLI_H__
#include <wmtypes.h>

#ifdef RW610
    #define COEX_APP_SUPPORT
#endif

#ifndef RW610
    #define CONFIG_APP_FRM_CLI_HISTORY 1
#endif

/** Structure for registering CLI commands */
struct cli_command
{
    /** The name of the CLI command */
    const char *name;
    /** The help text associated with the command */
    const char *help;
    /** The function that should be invoked for this command. */
    void (*function)(int argc, char **argv);
};

#ifdef COEX_APP_SUPPORT
    const struct cli_command *lookup_command(char *name, int len);
#endif

/** Register a CLI command
 *
 * This function registers a command with the command-line interface.
 *
 * \param[in] command The structure to register one CLI command
 * \return 0 on success
 * \return 1 on failure
 */
int cli_register_command(const struct cli_command *command);

/** Unregister a CLI command
 *
 * This function unregisters a command from the command-line interface.
 *
 * \param[in] command The structure to unregister one CLI command
 * \return 0 on success
 * \return 1 on failure
 */
int cli_unregister_command(const struct cli_command *command);

/** Initialize the CLI module
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise.
 */
int cli_init(void);

/** DeInitialize the CLI module
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise.
 */
int cli_deinit(void);

/** Stop the CLI thread and carry out the cleanup
 *
 * \return WM_SUCCESS on success
 * \return error code otherwise.
 *
 */
int cli_stop(void);

/** Register a batch of CLI commands
 *
 * Often, a module will want to register several commands.
 *
 * \param[in] commands Pointer to an array of commands.
 * \param[in] num_commands Number of commands in the array.
 * \return 0 on success
 * \return 1 on failure
 */
int cli_register_commands(const struct cli_command *commands, int num_commands);

/** Unregister a batch of CLI commands
 *
 * \param[in] commands Pointer to an array of commands.
 * \param[in] num_commands Number of commands in the array.
 * \return 0 on success
 * \return 1 on failure
 */
int cli_unregister_commands(const struct cli_command *commands, int num_commands);

/** Get a command buffer
 *
 * If an external input task wants to use the CLI, it can use
 * cli_get_cmd_buffer() to get a command buffer that it can then
 * submit to the CLI later using cli_submit_cmd_buffer().
 *
 * \param buff Pointer to a char * to place the buffer pointer in.
 * \return WM_SUCCESS on success
 * \return error code otherwise.
 */
int cli_get_cmd_buffer(char **buff);

/** Submit a command buffer to the CLI
 *
 * Sends the command buffer to the CLI for processing.
 *
 * \param buff Pointer to a char * buffer.
 * \return WM_SUCCESS on success
 * \return error code otherwise.
 */
int cli_submit_cmd_buffer(char **buff);

/**
 * @internal
 *
 */
typedef int (*cli_name_val_get)(const char *name, char *value, int max_len);

/**
 * @internal
 *
 */
typedef int (*cli_name_val_set)(const char *name, const char *value);
#if CONFIG_APP_FRM_CLI_HISTORY
    /**
    * @internal
    *
    * Hook registration function for cli history functionality
    */
    int cli_add_history_hook(cli_name_val_get get_cb, cli_name_val_set set_cb);
#endif /* CONFIG_APP_FRM_CLI_HISTORY */


/**
 * @internal
 *
 * CLI help command to print all registered CLIs
 */
void help_command(int argc, char **argv);

#if CONFIG_UART_INTERRUPT
    #if CONFIG_HOST_SLEEP
        /** Reinit USART
        *
        * \return kStatus_Success, others fail.
        */
        int cli_uart_reinit();

        /** Deinit USART
        *
        * \return kStatus_Success, others fail.
        */
        int cli_uart_deinit();

        /** Notify uart_task
        */
        void cli_uart_notify();
    #endif
#endif
#endif /* __CLI_H__ */
