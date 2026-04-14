# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import os
import platform
from typing import List
from typing import Optional

import click

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import EnvironmentError as SdkEnvironmentError
from sdk_py_actions.errors import UsageError

EXTENSION_ID = "flash"
EXTENSION_VERSION = "2.1.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None


def _resolve_build_dir(sdk_ctx: SdkContext, device: Optional[str]) -> str:
    project_dir = sdk_ctx.project_dir

    if device:
        device = device.strip()
        if not device:
            raise UsageError("--device cannot be empty")

        if os.path.isabs(device) and os.path.isdir(device):
            return os.path.realpath(device)

        # backward compatibility: allow either full build dir name or board name
        direct_path = os.path.realpath(os.path.join(project_dir, device))
        if os.path.isdir(direct_path):
            return direct_path

        prefixed_path = os.path.realpath(os.path.join(project_dir, f"build_{device}"))
        if os.path.isdir(prefixed_path):
            return prefixed_path

        raise SdkEnvironmentError(
            f"Cannot find build directory for device '{device}'. "
            "Please pass a valid build directory path/name or build board name."
        )

    candidate = os.path.realpath(sdk_ctx.build_dir)
    if os.path.isdir(candidate):
        return candidate

    raise SdkEnvironmentError(
        f'Configured build directory does not exist: "{candidate}". '
        "Please run build first, use sdk.py set-target to update .project.toml board, "
        "or pass -B/--build-dir or --device/-d explicitly."
    )


def find_sftool_param(build_dir: str) -> str:
    json_path = os.path.join(build_dir, "sftool_param.json")
    if not os.path.exists(json_path):
        raise SdkEnvironmentError(f"sftool_param.json not found in {build_dir}. Please build the project first.")
    return json_path


def find_download_script(build_dir: str) -> str:
    script_name = "download.bat" if platform.system() == "Windows" else "download.sh"
    script_path = os.path.join(build_dir, script_name)
    if not os.path.exists(script_path):
        raise SdkEnvironmentError(f"Download script not found: {script_path}. Please build the project first.")
    return script_path


def _normalize_uart_port(port: str) -> str:
    if platform.system() != "Windows":
        return port

    port_upper = port.upper()
    if not port_upper.startswith("COM") and port.isdigit():
        return f"COM{port}"
    if port != port_upper and port_upper.startswith("COM"):
        return port_upper
    return port


def _collect_flash_files(build_dir: str, files: List[dict]) -> List[str]:
    flash_args: List[str] = []

    for file_info in files:
        file_path = file_info.get("path", "")
        address = file_info.get("address", None)
        if not file_path:
            continue

        if not os.path.isabs(file_path):
            file_path = os.path.join(build_dir, file_path)

        if not os.path.exists(file_path):
            raise SdkEnvironmentError(f"Flash image file not found: {file_path}")

        flash_args.append(f"{file_path}@{address}" if address else file_path)

    if not flash_args:
        raise SdkEnvironmentError("No valid files to flash found in sftool_param.json")

    return flash_args


def _flash_uart(sdk_ctx: SdkContext, build_dir: str, port: str, baud: int) -> None:
    json_path = find_sftool_param(build_dir)

    with open(json_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    chip = config.get("chip", "")
    memory = str(config.get("memory", "NOR")).lower()
    write_flash = config.get("write_flash", {})
    files = write_flash.get("files", [])

    if not isinstance(files, list):
        raise SdkEnvironmentError("Invalid sftool_param.json: write_flash.files must be a list")

    flash_files = _collect_flash_files(build_dir, files)
    normalized_port = _normalize_uart_port(port)

    cmd = ["sftool", "-p", normalized_port, "-b", str(baud), "-c", chip, "-m", memory, "write_flash"] + flash_files
    print(f"Using build directory: {build_dir}")
    print(f"Flashing via UART on port {normalized_port} at {baud} baud...")
    sdk_ctx.runner.run(cmd, cwd=build_dir)
    print("Flash completed successfully!")


def _flash_jlink(sdk_ctx: SdkContext, build_dir: str) -> None:
    script_path = find_download_script(build_dir)

    print(f"Using build directory: {build_dir}")
    print(f"Flashing via JLink using script: {script_path}")

    if platform.system() == "Windows":
        sdk_ctx.runner.run(["cmd", "/c", script_path], cwd=build_dir)
    else:
        sdk_ctx.runner.run(["bash", script_path], cwd=build_dir)

    print("Flash completed successfully!")


def flash_callback(
    sdk_ctx: SdkContext,
    protocol: str = "uart",
    port: Optional[str] = None,
    baud: int = 1000000,
    device: Optional[str] = None,
) -> None:
    selected_protocol = protocol.lower()
    if selected_protocol == "uart":
        if not port:
            raise UsageError("--port/-p is required when protocol is uart")
        build_dir = _resolve_build_dir(sdk_ctx, device=device)
        _flash_uart(sdk_ctx, build_dir=build_dir, port=port, baud=baud)
        return

    if selected_protocol == "jlink":
        build_dir = _resolve_build_dir(sdk_ctx, device=device)
        _flash_jlink(sdk_ctx, build_dir=build_dir)
        return

    raise UsageError(f"Unsupported protocol: {protocol}")


def register(registry: CommandRegistry) -> None:
    registry.command(
        path="flash",
        callback=flash_callback,
        help="Flash firmware to device.",
        options=[
            {
                "names": ["-u", "--protocol"],
                "help": "Flash protocol. Default is uart.",
                "type": click.Choice(["uart", "jlink"], case_sensitive=False),
                "default": "uart",
            },
            {
                "names": ["-p", "--port"],
                "help": "Serial port for UART flash (e.g. COM3, /dev/ttyUSB0). Required when protocol is uart.",
                "type": str,
                "default": None,
            },
            {
                "names": ["-b", "--baud"],
                "help": "Baud rate for UART flash.",
                "type": int,
                "default": 1000000,
            },
            {
                "names": ["-d", "--device"],
                "help": "Optional board/device name (or build directory name/path). If omitted, use resolved --build-dir.",
                "type": str,
                "default": None,
            },
        ],
    )
