# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import os
import sys

import rt_pkg
import rt_pkg.rt_package_list
import rt_pkg.rt_package_printenv
import rt_pkg.rt_package_update
import rt_pkg.rt_package_upgrade
import rt_pkg.rt_package_utils

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import CommandExecutionError

EXTENSION_ID = "rt-pkg"
EXTENSION_VERSION = "2.0.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None


def setup_rt_pkg_environment() -> None:
    if "SIFLI_SDK" not in os.environ and "SIFLI_SDK_PATH" in os.environ:
        os.environ["SIFLI_SDK"] = os.environ["SIFLI_SDK_PATH"]

    if "RTT_CC" not in os.environ:
        os.environ["RTT_CC"] = "gcc"

    rt_pkg_path = os.path.join(os.environ.get("SIFLI_SDK_PATH", ""), "tools", "sdk_py_actions", "rt_pkg")
    if rt_pkg_path and rt_pkg_path not in sys.path:
        sys.path.insert(0, rt_pkg_path)


def _run_rt_pkg(action_name: str, callback) -> None:
    setup_rt_pkg_environment()
    try:
        callback()
    except SystemExit as exc:
        raise CommandExecutionError(f"rt-pkg {action_name} failed with exit code {exc.code}") from exc
    except Exception as exc:
        raise CommandExecutionError(f"rt-pkg {action_name} failed: {exc}") from exc


def update_callback(sdk_ctx: SdkContext, force: bool = False) -> None:
    _run_rt_pkg("update", lambda: rt_pkg.rt_package_update.package_update(force_update=force))


def list_callback(sdk_ctx: SdkContext) -> None:
    _run_rt_pkg("list", rt_pkg.rt_package_list.list_packages)


def wizard_callback(sdk_ctx: SdkContext) -> None:
    def _run_wizard() -> None:
        import rt_pkg.rt_package_wizard

        rt_pkg.rt_package_wizard.package_wizard()

    _run_rt_pkg("wizard", _run_wizard)


def upgrade_callback(sdk_ctx: SdkContext, force: bool = False, script: bool = False) -> None:
    _run_rt_pkg(
        "upgrade",
        lambda: rt_pkg.rt_package_upgrade.package_upgrade(force_upgrade=force, upgrade_script=script),
    )


def upgrade_modules_callback(sdk_ctx: SdkContext) -> None:
    _run_rt_pkg("upgrade-modules", rt_pkg.rt_package_upgrade.package_upgrade_modules)


def printenv_callback(sdk_ctx: SdkContext) -> None:
    _run_rt_pkg("printenv", rt_pkg.rt_package_printenv.package_print_env)


def register(registry: CommandRegistry) -> None:
    registry.group(path="rt-pkg", help="RT-Thread package management.")

    registry.command(
        path="rt-pkg/update",
        callback=update_callback,
        help="Update packages by menuconfig settings.",
        options=[
            {
                "names": ["--force"],
                "help": "Force update and clean packages.",
                "is_flag": True,
                "default": False,
            }
        ],
    )

    registry.command(path="rt-pkg/list", callback=list_callback, help="List target packages.")
    registry.command(path="rt-pkg/wizard", callback=wizard_callback, help="Create a new package with wizard.")

    registry.command(
        path="rt-pkg/upgrade",
        callback=upgrade_callback,
        help="Upgrade local packages index from git repository.",
        options=[
            {
                "names": ["--force"],
                "help": "Force upgrade local packages index.",
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["--script"],
                "help": "Also upgrade Env script from git repository.",
                "is_flag": True,
                "default": False,
            },
        ],
    )

    registry.command(
        path="rt-pkg/upgrade-modules",
        callback=upgrade_modules_callback,
        help="Upgrade python modules (e.g. requests).",
    )

    registry.command(
        path="rt-pkg/printenv",
        callback=printenv_callback,
        help="Print environment variables to check.",
    )
