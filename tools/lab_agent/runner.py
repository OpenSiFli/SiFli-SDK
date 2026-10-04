# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Lab Agent entry point.

Run it on the machine the boards are plugged into::

    python tools/lab_agent/runner.py --config tools/lab_agent/examples/runner.local.yaml

``--check`` and ``--list`` exist so that a mis-wired lab is discovered while
someone is looking at it, rather than from a failed nightly run.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from typing import List
from typing import Optional


def _bootstrap_import_path() -> None:
    """Make ``import lab_agent`` and ``import sdk_test`` work when run as a file.

    ``python tools/lab_agent/runner.py`` puts ``tools/lab_agent`` on ``sys.path``,
    not ``tools/``, so the package's own siblings would be unimportable. Adding
    ``tools/`` fixes both at once.
    """
    tools_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)


_bootstrap_import_path()

from lab_agent.config import find_jlink  # noqa: E402
from lab_agent.config import load_runner_config  # noqa: E402
from lab_agent.errors import LabAgentError  # noqa: E402
from lab_agent.server import LabAgentServer  # noqa: E402
from lab_agent.testbed import LabManager  # noqa: E402
from lab_agent.testbed import TestBedRuntime  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lab_agent",
        description="SiFli SDK Lab Agent: owns the boards, hands out testbeds.",
    )
    parser.add_argument(
        "-c",
        "--config",
        required=True,
        help="Runner config listing this machine's testbeds.",
    )
    parser.add_argument("--host", default=None, help="Override the listen address.")
    parser.add_argument("--port", type=int, default=None, help="Override the listen port.")
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print the configured testbeds and exit.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate the configuration (including the J-Link probe) and exit.",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Log every request."
    )
    return parser


def _configure_logging(verbose: bool) -> logging.Logger:
    logger = logging.getLogger("lab_agent")
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)-7s %(message)s", "%H:%M:%S")
        )
        logger.addHandler(handler)
    return logger


def _print_testbeds(manager: LabManager) -> None:
    testbeds = manager.list()
    if not testbeds:
        print("No testbeds configured.")
        return

    print(f"{len(testbeds)} testbed(s):")
    for testbed in testbeds:
        console = testbed["console"]
        console_text = (
            f"{console['port']} @ {console['baud']}" if console else "no console"
        )
        fixtures = ", ".join(testbed["fixtures"]) or "none"
        print(f"  {testbed['id']}")
        print(f"    platform   : {testbed['platform']}")
        print(f"    console    : {console_text}")
        print(f"    fixtures   : {fixtures}")
        if testbed["busy"]:
            print(f"    BUSY       : held by {testbed['holder']}")


def _run_checks(manager: LabManager, logger: logging.Logger) -> List[str]:
    """Return a list of human-readable problems. Empty means the lab looks sane."""
    problems: List[str] = []

    for warning in manager.config.check():
        problems.append(f"[warn] {warning}")

    if not manager.ids():
        problems.append("[warn] no testbeds configured; the agent will serve nothing")

    # A lab of UART-downloaded, manually-reset boards needs no probe at all, so
    # neither the locator nor the per-testbed probe build may run for it -- an
    # unconditional `Cannot find the J-Link commander` would be the only thing
    # `--check` ever reported, and the useful warnings underneath it would never
    # be read. A testbed whose flash or reset still goes through the probe is
    # checked exactly as before, so a genuine misconfiguration is not hidden.
    runtimes = [manager.get(testbed_id) for testbed_id in manager.ids()]
    wants_probe: List[TestBedRuntime] = []
    for runtime in runtimes:
        try:
            if runtime.needs_jlink():
                wants_probe.append(runtime)
        except LabAgentError as exc:
            problems.append(f"[warn] {runtime.testbed.id}: {exc}")

    if wants_probe:
        try:
            find_jlink(manager.config.settings.jlink_exe)
        except LabAgentError as exc:
            problems.append(f"[warn] {exc}")

    try:
        from serial import __version__ as pyserial_version  # noqa: F401

        logger.debug("pyserial %s available", pyserial_version)
    except ImportError:
        problems.append(
            "[warn] pyserial is not installed; no console can be opened "
            "(pip install pyserial)"
        )

    for runtime in wants_probe:
        try:
            runtime.jlink()
        except LabAgentError as exc:
            problems.append(f"[warn] {runtime.testbed.id}: {exc}")

    return problems


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logger = _configure_logging(args.verbose)

    try:
        config = load_runner_config(args.config)
    except LabAgentError as exc:
        logger.error("%s", exc)
        return 2

    if args.host:
        config.settings.host = args.host
    if args.port:
        config.settings.port = args.port

    sdk_root = os.environ.get("SIFLI_SDK_PATH")
    manager = LabManager(config, sdk_root=sdk_root)

    if args.list:
        _print_testbeds(manager)
        return 0

    if args.check:
        problems = _run_checks(manager, logger)
        for problem in problems:
            print(problem)
        if not problems:
            print("Configuration OK.")
        return 0

    server = LabAgentServer(manager, logger=logger)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Interrupted; releasing testbeds")
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
