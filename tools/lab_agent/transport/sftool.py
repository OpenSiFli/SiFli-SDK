# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""UART download transport, wrapping the ``sftool`` command-line tool.

The probe path (:mod:`lab_agent.transport.jlink`) is the default because it needs
no special board state. This path exists for boards wired without a debug probe,
and for reproducing exactly what ``sdk.py flash`` does.

The image list and the chip/memory identifiers all come from
``build_<board>/sftool_param.json``, which the *build* already produced -- see
``tools/build/sdk_resource.py``. Reading it here rather than recomputing keeps
one source of truth for what gets programmed where.

The download port is the console port: the ROM bootloader answers on the same
UART the console uses. The console must therefore be closed while flashing, and
:meth:`SftoolTransport.flash` refuses to guess if it is not.
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
from typing import List
from typing import Optional

from sdk_test.errors import FirmwareError
from sdk_test.firmware import portable
from sdk_test.firmware import read_sftool_param

from ..errors import TransportError

#: Default download baud. The SDK's flash_ext.py uses 1 Mbaud.
DEFAULT_BAUD = 1000000


def normalize_uart_port(port: str) -> str:
    """Accept ``13`` or ``com13`` for ``COM13`` on Windows, as flash_ext does."""
    if platform.system() != "Windows":
        return port
    upper = port.upper()
    if not upper.startswith("COM") and port.isdigit():
        return f"COM{port}"
    if port != upper and upper.startswith("COM"):
        return upper
    return port


def build_flash_arguments(build_dir: str, config: dict) -> List[str]:
    """Turn ``sftool_param.json`` into ``sftool write_flash`` file arguments.

    Relative image paths are resolved against the directory being flashed, which
    is what the generator means by them -- the build directory when a bench is
    flashed in place, the unpacked package when one was uploaded.
    """
    files = (config.get("write_flash") or {}).get("files") or []
    if not isinstance(files, list):
        raise FirmwareError("sftool_param.json: write_flash.files must be a list")

    arguments: List[str] = []
    for entry in files:
        if not isinstance(entry, dict):
            continue
        path = entry.get("path")
        if not path:
            continue
        path = portable(str(path))
        if not os.path.isabs(path):
            path = os.path.join(build_dir, path)
        if not os.path.isfile(path):
            raise FirmwareError(f"Flash image not found: {path}")
        address = entry.get("address")
        arguments.append(f"{path}@{address}" if address else path)

    if not arguments:
        raise FirmwareError("sftool_param.json lists no flashable images")
    return arguments


class SftoolTransport:
    """Flashes a board through its UART download mode."""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        sftool: Optional[str] = None,
        timeout: float = 300.0,
    ) -> None:
        self.port = normalize_uart_port(port)
        self.baud = baud
        self.timeout = timeout
        self._sftool = sftool or shutil.which("sftool")

    @property
    def sftool(self) -> str:
        if not self._sftool:
            raise TransportError(
                "Cannot find the `sftool` command. It is installed with the SDK "
                "python environment; run export.ps1/export.sh first."
            )
        return self._sftool

    def flash(self, build_dir: str) -> str:
        """Program every image named in the build's ``sftool_param.json``."""
        config = read_sftool_param(build_dir)
        chip = config.get("chip")
        memory = str(config.get("memory", "NOR")).lower()
        if not chip:
            raise TransportError("sftool_param.json has no `chip` field")

        command = [
            self.sftool,
            "-p",
            self.port,
            "-b",
            str(self.baud),
            "-c",
            str(chip),
            "-m",
            memory,
            "write_flash",
        ] + build_flash_arguments(build_dir, config)

        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.timeout,
                cwd=build_dir,
                check=False,
            )
        except FileNotFoundError as exc:
            raise TransportError(f"sftool not found: {self._sftool}") from exc
        except subprocess.TimeoutExpired as exc:
            raise TransportError(
                f"sftool timed out after {self.timeout:.0f}s on {self.port}"
            ) from exc

        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise TransportError(
                f"sftool failed (exit {completed.returncode}):\n{output.strip()}"
            )
        return output
