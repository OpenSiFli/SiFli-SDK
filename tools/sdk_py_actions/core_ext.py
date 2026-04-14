# SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import glob
import multiprocessing
import os
import shutil
import sys
import tomllib
from typing import Dict
from typing import Optional
from typing import Tuple

import tomli_w

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import UsageError

EXTENSION_ID = "core"
EXTENSION_VERSION = "2.1.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None


def _read_project_config(project_dir: str) -> Dict[str, Optional[str]]:
    config_path = os.path.join(project_dir, ".project.toml")
    config = {
        "board": None,
        "board_search_path": None,
    }

    if not os.path.exists(config_path):
        return config

    with open(config_path, "rb") as f:
        data = tomllib.load(f)

    config["board"] = data.get("board")
    config["board_search_path"] = data.get("board_search_path")
    return config


def _write_project_config(project_dir: str, board: Optional[str], board_search_path: Optional[str]) -> None:
    config_path = os.path.join(project_dir, ".project.toml")

    config: Dict[str, object] = {}
    if os.path.exists(config_path):
        with open(config_path, "rb") as f:
            config = tomllib.load(f)

    if board is not None:
        config["board"] = board
    if board_search_path is not None:
        config["board_search_path"] = board_search_path

    with open(config_path, "wb") as f:
        tomli_w.dump(config, f)


def _resolve_board_options(
    project_dir: str,
    board: Optional[str],
    board_search_path: Optional[str],
    require_board: bool,
) -> Tuple[Optional[str], Optional[str]]:
    resolved_board = board
    resolved_search_path = board_search_path

    if resolved_board is None or resolved_search_path is None:
        config = _read_project_config(project_dir)
        if resolved_board is None:
            resolved_board = config.get("board")
        if resolved_search_path is None:
            resolved_search_path = config.get("board_search_path")

    if require_board and not resolved_board:
        raise UsageError('Board name is required. Use "--board" or run "sdk.py set-target <board>".')

    return resolved_board, resolved_search_path


def set_target_callback(
    sdk_ctx: SdkContext,
    board: str,
    board_search_path: Optional[str] = None,
) -> None:
    if not board:
        raise UsageError('Board name is required. Usage: sdk.py set-target <board> [--board-search-path <path>]')

    _write_project_config(sdk_ctx.project_dir, board=board, board_search_path=board_search_path)
    print(f"Target board set to: {board}")
    if board_search_path:
        print(f"Board search path set to: {board_search_path}")


def menuconfig_callback(
    sdk_ctx: SdkContext,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
) -> None:
    board, board_search_path = _resolve_board_options(
        sdk_ctx.project_dir,
        board=board,
        board_search_path=board_search_path,
        require_board=False,
    )

    menuconfig_path = os.path.join(os.environ["SIFLI_SDK_PATH"], "tools", "kconfig", "menuconfig.py")
    cmd = [sys.executable, menuconfig_path]
    if board:
        cmd.extend(["--board", board])
    if board_search_path:
        cmd.extend(["--board_search_path", board_search_path])

    sdk_ctx.runner.run(cmd, cwd=sdk_ctx.project_dir)


def _resolve_jobs(jobs: Optional[int]) -> int:
    if jobs not in (None, 0):
        return jobs

    try:
        return multiprocessing.cpu_count()
    except NotImplementedError:
        return 1


def _build_command(
    board: str,
    board_search_path: Optional[str],
    jobs: Optional[int],
    target: Optional[str],
) -> list[str]:
    cmd = ["scons", f"--board={board}"]
    if board_search_path:
        cmd.append(f"--board_search_path={board_search_path}")
    if target:
        cmd.append(target)
    cmd.append(f"-j{_resolve_jobs(jobs)}")
    return cmd


def build_callback(
    sdk_ctx: SdkContext,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
    jobs: Optional[int] = None,
) -> None:
    board, board_search_path = _resolve_board_options(
        sdk_ctx.project_dir,
        board=board,
        board_search_path=board_search_path,
        require_board=True,
    )

    if not board:
        raise UsageError('Board name is required. Use "--board" or run "sdk.py set-target <board>".')

    sdk_ctx.runner.run(_build_command(board, board_search_path, jobs, target=None), cwd=sdk_ctx.project_dir)


def build_target_callback(
    sdk_ctx: SdkContext,
    target: str,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
    jobs: Optional[int] = None,
) -> None:
    if not target:
        raise UsageError("Build target is required")

    board, board_search_path = _resolve_board_options(
        sdk_ctx.project_dir,
        board=board,
        board_search_path=board_search_path,
        require_board=True,
    )

    if not board:
        raise UsageError('Board name is required. Use "--board" or run "sdk.py set-target <board>".')

    sdk_ctx.runner.run(_build_command(board, board_search_path, jobs, target=target), cwd=sdk_ctx.project_dir)


def export_codebase_callback(
    sdk_ctx: SdkContext,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
) -> None:
    board, board_search_path = _resolve_board_options(
        sdk_ctx.project_dir,
        board=board,
        board_search_path=board_search_path,
        require_board=True,
    )

    if not board:
        raise UsageError('Board name is required. Use "--board" or run "sdk.py set-target <board>".')

    cmd = ["scons", f"--board={board}"]
    if board_search_path:
        cmd.append(f"--board_search_path={board_search_path}")
    cmd.extend(["--target=json", "-s"])

    sdk_ctx.runner.run(cmd, cwd=sdk_ctx.project_dir)


def fullclean_callback(sdk_ctx: SdkContext) -> None:
    config = _read_project_config(sdk_ctx.project_dir)
    board = config.get("board")
    if not board:
        raise UsageError('No board configured. Please run "sdk.py set-target <board>" first.')

    patterns = [
        os.path.join(sdk_ctx.project_dir, f"build_{board}"),
        os.path.join(sdk_ctx.project_dir, f"build_{board}_*"),
    ]

    removed_dirs = []
    project_root = os.path.realpath(sdk_ctx.project_dir)
    for pattern in patterns:
        for path in glob.glob(pattern):
            if not os.path.isdir(path):
                continue

            abs_path = os.path.realpath(path)
            if os.path.commonpath([project_root, abs_path]) != project_root:
                continue
            if abs_path == project_root:
                continue

            shutil.rmtree(abs_path)
            removed_dirs.append(abs_path)
            print(f"Removed {abs_path}")

    if not removed_dirs:
        print(f'No build directories matching "build_{board}_*" found in {sdk_ctx.project_dir}')


def register(registry: CommandRegistry) -> None:
    board_options = [
        {
            "names": ["--board"],
            "help": "Board name.",
            "envvar": "MENUCONFIG_BOARD",
            "default": None,
        },
        {
            "names": ["--board-search-path", "--board_search_path"],
            "help": "Board search path.",
            "envvar": "MENUCONFIG_BOARD_SEARCH_PATH",
            "default": None,
        },
    ]

    jobs_option = {
        "names": ["-j", "--jobs"],
        "help": "Number of parallel jobs for scons. If omitted, CPU count is used.",
        "type": int,
        "default": None,
    }

    registry.command(
        path="set-target",
        callback=set_target_callback,
        help="Set target board configuration and save to .project.toml file.",
        arguments=[{"names": ["board"], "required": True}],
        options=[
            {
                "names": ["--board-search-path", "--board_search_path"],
                "help": "Board search path.",
                "default": None,
            }
        ],
    )

    registry.command(
        path="menuconfig",
        callback=menuconfig_callback,
        help='Run "menuconfig" project configuration tool.',
        options=board_options,
    )

    registry.command(
        path="build",
        callback=build_callback,
        help="Build project with scons.",
        options=board_options + [jobs_option],
    )

    registry.command(
        path="build-target",
        callback=build_target_callback,
        help="Build a specific scons target.",
        arguments=[{"names": ["target"], "required": True}],
        options=board_options + [jobs_option],
    )

    registry.command(
        path="export-codebase",
        callback=export_codebase_callback,
        help="Export the source and header file index for the configured board.",
        options=board_options,
    )

    registry.command(
        path="fullclean",
        callback=fullclean_callback,
        help="Remove build directories for the configured board.",
    )
