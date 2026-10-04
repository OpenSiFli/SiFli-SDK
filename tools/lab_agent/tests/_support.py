# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Shared helpers and fakes for the Lab Agent self-tests.

These run without hardware. The console and probe are faked at the transport
boundary -- the two injection seams on :class:`~lab_agent.testbed.TestBedRuntime`
-- so everything above them (buffer, cursor arithmetic, allocation, the HTTP
surface) is the real implementation under test.
"""

from __future__ import annotations

import os
import sys
import tempfile
import textwrap
import time
import unittest
from typing import List
from typing import Optional

#: ``tools/`` -- holds both ``lab_agent`` and ``sdk_test``.
TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)


class FakeSerialTransport:
    """A console that replays a scripted transcript.

    Output is dripped a character at a time by default, which is the harshest
    realistic chunking: the console has no line discipline
    (``FINSH_ECHO_DISABLE_DEFAULT=1``) and ``RT_CONSOLEBUF_SIZE`` is 128, so
    lines genuinely arrive split.
    """

    def __init__(self, script: str = "", chunk: int = 1) -> None:
        self.script = script
        self.chunk = max(1, chunk)
        self.written: List[str] = []
        self.opened = 0
        self.closed = 0
        self._cursor = 0
        self.fail_reads = False

    def open(self) -> None:
        self.opened += 1

    def close(self) -> None:
        self.closed += 1

    def read(self, timeout: float = 0.05) -> str:
        if self.fail_reads:
            raise OSError("simulated serial failure")
        if self._cursor >= len(self.script):
            # Behave like a real port: block for the timeout, return nothing.
            time.sleep(min(timeout, 0.005))
            return ""
        piece = self.script[self._cursor : self._cursor + self.chunk]
        self._cursor += len(piece)
        return piece

    def write(self, text: str) -> None:
        self.written.append(text)

    @property
    def sent(self) -> str:
        return "".join(self.written)


class FakeJLink:
    """Stands in for a debug probe. Records what it was asked to do."""

    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.resets: List[Optional[str]] = []
        self.flashed: List[str] = []

    def reset(self, extra_script: Optional[str] = None) -> str:
        if self.fail:
            raise RuntimeError("simulated probe failure")
        self.resets.append(extra_script)
        return "reset ok"

    def flash(self, build_dir: str) -> str:
        if self.fail:
            raise RuntimeError("simulated probe failure")
        self.flashed.append(build_dir)
        return "flash ok"

    def run_script_file(self, path: str) -> str:
        return "script ok"


#: A minimal but complete runner config, with one 52x-flavoured testbed.
CONFIG_YAML = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0
  console_poll: 0.01

testbed:
  - id: tb_01
    platform: sf32lb52-lcd_n16r8
    console:
      port: COM13
    connections:
      - fixture: spi_nor_flash:spi1
      - fixture: i2c_sensor
    jlink:
      serial: "59605049"
    flash:
      method: jlink

  - id: tb_02
    platform: eh-lb563
    console:
      port: COM30
    connections:
      - fixture: spi_nor_flash:spi1
"""


class LabTestCase(unittest.TestCase):
    """Base class: temp tree, config helpers, and fake factories."""

    def setUp(self) -> None:
        self._tmpdir = tempfile.TemporaryDirectory(prefix="lab_agent_")
        self.addCleanup(self._tmpdir.cleanup)
        self.root = self._tmpdir.name

    def write(self, relative_path: str, content: str) -> str:
        path = os.path.join(self.root, relative_path.replace("/", os.sep))
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(textwrap.dedent(content).lstrip("\n"))
        return path

    def mkdir(self, relative_path: str) -> str:
        path = os.path.join(self.root, relative_path.replace("/", os.sep))
        os.makedirs(path, exist_ok=True)
        return path

    def make_config(self, yaml_text: str = CONFIG_YAML):
        from lab_agent.config import parse_runner_config

        import yaml

        return parse_runner_config(yaml.safe_load(textwrap.dedent(yaml_text)))

    def make_manager(self, yaml_text: str = CONFIG_YAML, script: str = "", chunk: int = 1):
        """A LabManager whose consoles and probes are fakes."""
        from lab_agent.testbed import LabManager

        transports = {}

        def console_factory(testbed):
            transport = FakeSerialTransport(script=script, chunk=chunk)
            transports[testbed.id] = transport
            return transport

        jlinks = {}

        def jlink_factory(testbed, settings):
            probe = FakeJLink()
            jlinks[testbed.id] = probe
            return probe

        manager = LabManager(
            self.make_config(yaml_text),
            sdk_root=self.root,
            console_factory=console_factory,
            jlink_factory=jlink_factory,
        )
        manager.transports = transports  # type: ignore[attr-defined]
        manager.probes = jlinks  # type: ignore[attr-defined]
        return manager
