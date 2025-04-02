#!/usr/bin/env python
# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2025 SiFli
# SPDX-License-Identifier: Apache-2.0
"""
Ensure that the Python version used to initiate this script is appropriate for
running the SiFli-SDK shell activation. The primary goal is to perform the minimum
necessary checks to identify the virtual environment with the default user Python
and then launch activate.py using the SiFli-SDK Python virtual environment.
"""
import os
import sys
from subprocess import run
from subprocess import SubprocessError


def die(msg: str) -> None:
    sys.exit(f'\nERROR: {msg}')


sifli_sdk_tools_path = os.path.realpath(os.path.dirname(__file__))
sdk_path = os.path.dirname(sifli_sdk_tools_path)
sys.path.insert(0, sifli_sdk_tools_path)

try:
    # The sifli_sdk_tools module checks for Python version compatibility.
    import sifli_sdk_tools
except ImportError as e:
    die(f'Unable to import the sifli_sdk_tools module: {e}')

# Get SiFli-SDK venv python path
sifli_sdk_tools.g.sifli_sdk_path = sdk_path
sifli_sdk_tools.g.sifli_sdk_tools_path = os.environ.get('SIFLI_SDK_TOOLS_PATH') or os.path.expanduser(sifli_sdk_tools.SIFLI_SDK_TOOLS_PATH_DEFAULT)
sdk_python_env_path, sdk_python_export_path, virtualenv_python, sdk_version = sifli_sdk_tools.get_python_env_path()

os.environ['SIFLI_SDK_PATH_OLD'] = os.environ.get('SIFLI_SDK_PATH', '')
os.environ['SIFLI_SDK_PATH'] = sdk_path
os.environ['SIFLI_SDK_PYTHON_ENV_PATH'] = sdk_python_env_path
os.environ['SIFLI_SDK_VERSION'] = sdk_version

if not os.path.exists(virtualenv_python):
    die((f'SiFli-SDK Python virtual environment "{virtualenv_python}" '
         f'not found. Please run the install script to set it up before '
         f'proceeding.'))

try:
    run([virtualenv_python, os.path.join(sdk_path, 'tools', 'export_utils', 'activate_venv.py')] + sys.argv[1:], check=True, env=os.environ.copy())
except (OSError, SubprocessError) as e:
    die('\n'.join(['Activation script failed', str(e),
                   'To view detailed debug information, set SIFLI_SDK_EXPORT_DEBUG=1 and run the export script again.']))
