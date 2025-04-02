# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2025 SiFli
# SPDX-License-Identifier: Apache-2.0
import argparse
import os
from subprocess import run
from subprocess import SubprocessError
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


class Config:
    """
    Config serves as global hodler for variables used across modules
    It holds also arguments from command line
    """
    def __init__(self) -> None:
        self.SIFLI_SDK_PATH = os.environ['SIFLI_SDK_PATH']
        self.SIFLI_SDK_PATH_OLD = os.environ['SIFLI_SDK_PATH_OLD']
        self.SIFLI_SDK_VERSION = os.environ['SIFLI_SDK_VERSION']
        self.SIFLI_SDK_PYTHON_ENV_PATH = os.environ['SIFLI_SDK_PYTHON_ENV_PATH']
        self.SIFLI_SDK_TOOLS_PY = os.path.join(self.SIFLI_SDK_PATH, 'tools', 'sifli_sdk_tools.py')
        self.SIFLI_SDK_PY = os.path.join(self.SIFLI_SDK_PATH, 'tools', 'sdk.py')
        self.ARGS: Optional[argparse.Namespace] = None


# Global variable instance
conf = Config()


def run_cmd(cmd: List[str], env: Optional[Dict[str, Any]]=None) -> str:
    new_env = os.environ.copy()
    if env is not None:
        new_env.update(env)

    cmd_str = '"{}"'.format(' '.join(cmd))
    try:
        p = run(cmd, env=new_env, text=True, capture_output=True)
    except (OSError, SubprocessError) as e:
        raise RuntimeError(f'Command {cmd_str} failed: {e}')

    stdout: str = p.stdout.strip()
    stderr: str = p.stderr.strip()
    if p.returncode:
        raise RuntimeError(f'Command {cmd_str} failed with error code {p.returncode}\n{stdout}\n{stderr}')

    return stdout
