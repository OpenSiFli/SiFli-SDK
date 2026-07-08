# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import platform
from pathlib import Path
from typing import Optional

import click

from sdk_py_actions import crash_dump
from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry


EXTENSION_ID = "crash-dump"
EXTENSION_VERSION = "1.0.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None


def _jlink_default() -> str:
    return "JLink.exe" if platform.system() == "Windows" else "JLinkExe"


def _number(value: Optional[str]) -> Optional[int]:
    return int(value, 0) if value is not None else None


def capture_live_callback(
    sdk_ctx: SdkContext,
    transport: str,
    chip: str,
    output_dir: str,
    probe: Optional[str] = None,
    probe_rs: str = "probe-rs",
    jlink: Optional[str] = None,
    script_path: Optional[str] = None,
    elf_path: Optional[str] = None,
    log_path: Optional[str] = None,
    psram_size: Optional[str] = None,
    chip_model: Optional[str] = None,
    include_psram: bool = False,
    jlink_ip: Optional[str] = None,
    core: Optional[str] = None,
    json_output: bool = False,
) -> None:
    result = crash_dump.capture_live(
        sdk_ctx,
        transport,
        chip,
        Path(output_dir),
        probe=probe,
        probe_rs=probe_rs,
        jlink=jlink or _jlink_default(),
        script_path=Path(script_path) if script_path else None,
        elf_path=Path(elf_path) if elf_path else None,
        log_path=Path(log_path) if log_path else None,
        legacy_layout=True,
        psram_size=psram_size,
        chip_model=chip_model,
        include_psram=include_psram,
        jlink_ip=jlink_ip,
        core=core,
    )
    click.echo(
        json.dumps(result, sort_keys=True)
        if json_output
        else str(Path(output_dir).resolve())
    )


def export_coredump_callback(
    sdk_ctx: SdkContext,
    output_dir: str,
    input_path: Optional[str] = None,
    transport: str = "uart",
    chip: Optional[str] = None,
    memory: str = "nor",
    port: Optional[str] = None,
    baud: int = 1_000_000,
    address: Optional[str] = None,
    size: Optional[str] = None,
    ptab_path: Optional[str] = None,
    kind: str = "full",
    sftool: str = "sftool",
    jlink: Optional[str] = None,
    elf_path: Optional[str] = None,
    log_path: Optional[str] = None,
    script_path: Optional[str] = None,
    psram_size: Optional[str] = None,
    chip_model: Optional[str] = None,
    include_psram: bool = False,
    json_output: bool = False,
) -> None:
    result = crash_dump.export_coredump(
        sdk_ctx,
        Path(output_dir),
        input_path=Path(input_path) if input_path else None,
        transport=transport,
        chip=chip,
        memory=memory,
        port=port,
        baud=baud,
        address=_number(address),
        size=_number(size),
        ptab_path=Path(ptab_path) if ptab_path else None,
        kind=kind,
        sftool=sftool,
        jlink=jlink or _jlink_default(),
        elf_path=Path(elf_path) if elf_path else None,
        log_path=Path(log_path) if log_path else None,
        script_path=Path(script_path) if script_path else None,
        legacy_layout=True,
        psram_size=psram_size,
        chip_model=chip_model,
        include_psram=include_psram,
    )
    click.echo(
        json.dumps(result, sort_keys=True)
        if json_output
        else str(Path(output_dir).resolve())
    )


def readcore_callback(
    sdk_ctx: SdkContext,
    package_dir: str,
    elf_path: Optional[str] = None,
    output_path: Optional[str] = None,
) -> None:
    del sdk_ctx
    result = crash_dump.write_core_elf(
        Path(package_dir),
        Path(elf_path) if elf_path else None,
        Path(output_path) if output_path else None,
    )
    click.echo(str(result.resolve()))


def analyze_callback(
    sdk_ctx: SdkContext,
    package_dir: Optional[str] = None,
    core_path: Optional[str] = None,
    elf_path: Optional[str] = None,
    output_path: Optional[str] = None,
    json_output: bool = False,
    engine: str = "auto",
) -> None:
    del sdk_ctx
    destination = Path(output_path) if output_path else None
    if core_path:
        if not elf_path:
            raise click.UsageError("--elf is required with --core")
        result = crash_dump.analyze_core_elf(
            Path(core_path),
            Path(elf_path),
            destination,
        )
        if result is None:
            raise click.ClickException("arm-none-eabi-gdb was not found")
    elif package_dir:
        result = crash_dump.analyze_package(
            Path(package_dir),
            Path(elf_path) if elf_path else None,
            destination,
            engine=engine,
        )
    else:
        raise click.UsageError("--package or --core is required")
    default_output = (
        Path(package_dir) / "analysis.json"
        if package_dir
        else Path(core_path).resolve().parent / "analysis.json"
    )
    click.echo(
        json.dumps(result, sort_keys=True)
        if json_output
        else str((destination or default_output).resolve())
    )


def register(registry: CommandRegistry) -> None:
    registry.group(path="crash-dump", help="Capture and analyze SiFli crash dumps.")
    registry.command(
        path="crash-dump/capture-live",
        callback=capture_live_callback,
        help="Capture live memory through J-Link or SiFli UART DEBUG IP.",
        options=[
            {
                "names": ["--transport"],
                "type": click.Choice(["jlink", "uart"]),
                "default": "jlink",
            },
            {"names": ["--chip"], "required": True},
            {
                "names": ["--output", "output_dir"],
                "type": click.Path(),
                "required": True,
            },
            {"names": ["--probe"], "default": None},
            {"names": ["--probe-rs", "probe_rs"], "default": "probe-rs"},
            {"names": ["--jlink"], "default": None},
            {"names": ["--jlink-ip", "jlink_ip"], "default": None},
            {
                "names": ["--core"],
                "type": click.Choice(["hcpu", "lcpu"]),
                "default": None,
            },
            {
                "names": ["--script", "script_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--elf", "elf_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--log", "log_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {"names": ["--psram-size", "psram_size"], "default": None},
            {"names": ["--chip-model", "chip_model"], "default": None},
            {"names": ["--include-psram"], "is_flag": True, "default": False},
            {
                "names": ["--json", "json_output"],
                "is_flag": True,
                "default": False,
            },
        ],
    )
    registry.command(
        path="crash-dump/export-coredump",
        callback=export_coredump_callback,
        help="Read a persisted coredump partition or package an existing dump.",
        options=[
            {
                "names": ["--output", "output_dir"],
                "type": click.Path(),
                "required": True,
            },
            {
                "names": ["--input", "input_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--transport"],
                "type": click.Choice(["jlink", "uart"]),
                "default": "uart",
            },
            {"names": ["--chip"], "default": None},
            {"names": ["--memory"], "default": "nor"},
            {"names": ["--port"], "default": None},
            {"names": ["--baud"], "type": int, "default": 1_000_000},
            {"names": ["--address"], "default": None},
            {"names": ["--size"], "default": None},
            {
                "names": ["--ptab", "ptab_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--kind"],
                "type": click.Choice(["full", "minimum"]),
                "default": "full",
            },
            {"names": ["--sftool"], "default": "sftool"},
            {"names": ["--jlink"], "default": None},
            {
                "names": ["--elf", "elf_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--log", "log_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--script", "script_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {"names": ["--psram-size", "psram_size"], "default": None},
            {"names": ["--chip-model", "chip_model"], "default": None},
            {"names": ["--include-psram"], "is_flag": True, "default": False},
            {
                "names": ["--json", "json_output"],
                "is_flag": True,
                "default": False,
            },
        ],
    )
    registry.command(
        path="crash-dump/readcore",
        callback=readcore_callback,
        help="Convert a crash package to a GDB-compatible core ELF.",
        options=[
            {
                "names": ["--package", "package_dir"],
                "type": click.Path(exists=True),
                "required": True,
            },
            {
                "names": ["--elf", "elf_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--output", "output_path"],
                "type": click.Path(),
                "default": None,
            },
        ],
    )
    registry.command(
        path="crash-dump/analyze",
        callback=analyze_callback,
        help="Analyze a core ELF and its matching firmware ELF.",
        options=[
            {
                "names": ["--package", "package_dir"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--core", "core_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--elf", "elf_path"],
                "type": click.Path(exists=True),
                "default": None,
            },
            {
                "names": ["--output", "output_path"],
                "type": click.Path(),
                "default": None,
            },
            {
                "names": ["--json", "json_output"],
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["--engine"],
                "type": click.Choice(["auto", "gdb", "python"]),
                "default": "python",
                "help": "Analysis engine: python (pyelftools only), gdb (GDB only), auto (try GDB, fall back to Python).",
            },
        ],
    )
