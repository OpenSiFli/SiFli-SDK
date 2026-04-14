# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
from click.core import Context


class FatalError(RuntimeError):
    """
    Wrapper class for runtime errors that aren't caused by bugs in sdk.py or the build process.
    """

    exit_code = 10

    def __init__(self, message: str, ctx: Context=None, exit_code: int=None):
        super(RuntimeError, self).__init__(message)
        if exit_code is not None:
            self.exit_code = exit_code
        # if context is defined, check for the cleanup tasks
        if ctx is not None and 'cleanup' in ctx.meta:
            # cleans up the environment before failure
            ctx.meta['cleanup']()


class UsageError(FatalError):
    exit_code = 2


class EnvironmentError(FatalError):
    exit_code = 3


class CommandExecutionError(FatalError):
    exit_code = 4


class AuthError(FatalError):
    exit_code = 5


class NoSerialPortFoundError(FatalError):
    pass
