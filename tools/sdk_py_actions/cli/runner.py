# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import os
import subprocess
from typing import Mapping
from typing import Optional
from typing import Sequence

from sdk_py_actions.errors import CommandExecutionError
from sdk_py_actions.errors import EnvironmentError as SdkEnvironmentError


class CommandRunner:
    def __init__(self, logger) -> None:
        self._logger = logger

    def run(
        self,
        args: Sequence[str],
        cwd: Optional[str] = None,
        env: Optional[Mapping[str, str]] = None,
        check: bool = True,
        capture_output: bool = False,
        text: bool = True,
        timeout: Optional[float] = None,
        retries: int = 0,
    ) -> subprocess.CompletedProcess:
        run_env = dict(os.environ)
        if env is not None:
            run_env.update(dict(env))

        cmd_text = " ".join(str(arg) for arg in args)

        last_exception: Optional[Exception] = None
        for _ in range(retries + 1):
            try:
                return subprocess.run(
                    list(args),
                    cwd=cwd,
                    env=run_env,
                    check=check,
                    capture_output=capture_output,
                    text=text,
                    timeout=timeout,
                )
            except FileNotFoundError as exc:
                raise SdkEnvironmentError(f'Executable not found: {args[0]}') from exc
            except subprocess.TimeoutExpired as exc:
                last_exception = exc
            except subprocess.CalledProcessError as exc:
                if not check:
                    raise
                last_exception = exc

        if isinstance(last_exception, subprocess.TimeoutExpired):
            raise CommandExecutionError(
                f'Command timed out after {last_exception.timeout}s: {cmd_text}'
            ) from last_exception

        if isinstance(last_exception, subprocess.CalledProcessError):
            raise CommandExecutionError(
                f'Command failed with exit code {last_exception.returncode}: {cmd_text}'
            ) from last_exception

        raise CommandExecutionError(f'Command failed unexpectedly: {cmd_text}')
