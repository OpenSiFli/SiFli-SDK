# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""pytest plugin backing the ``pytest`` harness.

A test directory that declares ``harness: pytest`` brings its own ``test_*.py``
and drives the device through the fixtures defined here. The vocabulary is
deliberately the one pytest-embedded uses (``dut``, ``console``), so that anyone
who has written embedded pytest before recognises it.

The runner loads this plugin automatically; a suite does not need a conftest.py.

The DUT is *already allocated* by the time these fixtures run -- the runner
claimed a testbed, flashed it, reset it and opened its console. The fixtures
therefore attach to that existing console (coordinates arrive in the
environment) rather than asking the Lab Agent for hardware of their own. That is
what keeps a pytest test on the same terms as a ``console`` test: one testbed,
one holder, no second allocation to deadlock against.
"""

from __future__ import annotations

import json
import os
import re
import time
from typing import List
from typing import Optional

import pytest

from .agent_client import LabAgentClient
from .agent_client import RemoteConsole
from .instruments import INSTRUMENTS
from .instruments import InstrumentConsole
from .instruments.registry import RESERVED_NAMES
from .harness.pytest import ENV_AGENT_URL
from .harness.pytest import ENV_BUILD_DIR
from .harness.pytest import ENV_CASE_ID
from .harness.pytest import ENV_FIXTURES
from .harness.pytest import ENV_PLATFORM
from .harness.pytest import ENV_TESTBED_ID
from .harness.unity import DEFAULT_CMD
from .harness.unity import UnityResult
from .harness.unity import is_complete
from .harness.unity import parse_unity_output

#: Default budget for a single `expect`, in seconds.
DEFAULT_EXPECT_TIMEOUT = 10.0


class Dut:
    """The device under test, as seen by a pytest suite.

    Everything the suite needs is a method here rather than a raw console handle,
    so a failure message can include what the device actually said -- which is
    the difference between a five-minute fix and an afternoon.
    """

    def __init__(
        self,
        console: RemoteConsole,
        client: Optional[LabAgentClient] = None,
        testbed_id: Optional[str] = None,
        platform: Optional[str] = None,
        build_dir: Optional[str] = None,
        case_id: Optional[str] = None,
    ) -> None:
        self.console = console
        self._client = client
        self.testbed_id = testbed_id
        self.platform = platform
        self.build_dir = build_dir
        self.case_id = case_id
        self._sent: List[str] = []
        #: Everything read so far, and how much of it earlier expectations have
        #: already accounted for. One buffer for the whole session, walked by a
        #: cursor that only moves forward -- see `expect` for why.
        self._buffer = ""
        self._cursor = 0

    # -- io ------------------------------------------------------------------

    def write(self, data: str) -> None:
        """Send ``data`` verbatim."""
        self.console.write(data)

    def send_line(self, line: str) -> None:
        """Send ``line`` with a CRLF, as the device's shell expects."""
        self._sent.append(line)
        self.console.send_line(line)

    def read(self, timeout: float = 1.0) -> str:
        """Return whatever the device said in the next ``timeout`` seconds.

        Kept in the buffer as well, so an ``expect`` later in the suite can
        still see it.
        """
        return self._pull(timeout)

    def _pull(self, timeout: float) -> str:
        """Read once, and keep what came back.

        Every read goes through here, because anything that bypassed the buffer
        would be invisible to the next ``expect`` -- which is the bug this
        class used to have.
        """
        chunk = self.console.read(timeout)
        if chunk:
            self._buffer += chunk
        return chunk

    def drain(self) -> None:
        """Discard anything buffered, e.g. before provoking new output.

        Both halves of it: what the console still holds, and what has been read
        here but not yet accounted for by an expectation.
        """
        self.console.skip_to_end()
        self._buffer = ""
        self._cursor = 0

    # -- assertions ----------------------------------------------------------

    def expect(self, pattern: str, timeout: float = DEFAULT_EXPECT_TIMEOUT) -> str:
        """Wait for ``pattern`` (a regex) and return the text that matched.

        Matching runs over everything read so far, from a cursor that only moves
        forward -- the same model the console harness uses
        (:class:`sdk_test.harness.console.Matcher`), and both halves of it are
        load-bearing:

        * a single read can carry this expectation *and* the next one's text.
          Searching a fresh buffer per call throws the second half away, and the
          next call then waits out its whole timeout for bytes that already
          arrived -- a failure that looks like a silent device.
        * "the device said X" means *from here on*, not "X is somewhere in the
          session". Re-reading from zero would let an earlier test's output
          satisfy this one, which is a pass that proves nothing.

        Chunking is handled here rather than by the caller: the console splits
        long lines (``RT_CONSOLEBUF_SIZE`` is 128 bytes), so a match is searched
        across chunk boundaries rather than line by line.
        """
        try:
            compiled = re.compile(pattern)
        except re.error as exc:
            raise AssertionError(f"invalid regex {pattern!r}: {exc}") from exc

        arriving_from = len(self._buffer)
        deadline = time.monotonic() + timeout
        while True:
            match = compiled.search(self._buffer, self._cursor)
            if match:
                self._cursor = match.end()
                return match.group(0)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise AssertionError(
                    f"expected /{pattern}/ within {timeout:.0f}s.\n"
                    f"--- device said ---\n"
                    f"{self._buffer[arriving_from:] or '<nothing>'}\n"
                    "-------------------"
                )
            self._pull(min(0.5, remaining))

    def expect_unity_test_output(
        self,
        timeout: float = 30.0,
        command: Optional[str] = None,
    ) -> UnityResult:
        """Ask the device to run Unity and return the parsed result.

        Output buffered before the command is discarded first, so a previous
        run's summary cannot be mistaken for this one's.
        """
        self.drain()
        self.send_line(command or DEFAULT_CMD)

        text = ""
        deadline = time.monotonic() + timeout
        while not is_complete(text):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            chunk = self.console.read(min(0.5, remaining))
            if chunk:
                text += chunk

        return parse_unity_output(text)

    # -- control -------------------------------------------------------------

    def reset(self) -> None:
        """Reset the board out-of-band through the Lab Agent.

        The console stays open across the reset, so the boot log that follows is
        captured rather than raced.
        """
        if self._client is None or not self.testbed_id:
            raise RuntimeError(
                "reset() needs a Lab Agent connection; this DUT was built without one"
            )
        self.drain()
        self._client.reset(self.testbed_id)

    def flush_console(self) -> str:
        """Force a console read and return an empty string (readability alias)."""
        self._pull(0.0)
        return ""


def _require(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        pytest.skip(
            f"{name} is not set: this suite must be run through `sdk.py test run` "
            "(or another framework entry point), which allocates a testbed and "
            "opens its console before starting pytest."
        )
    return value


@pytest.fixture(scope="session")
def dut() -> Dut:
    """The allocated device under test, one for the whole suite.

    Session-scoped so that the console -- and the forward-only cursor over what
    it has said -- survives from one test to the next. A fresh Dut per test
    would put every test back at the start of the session's output, where a
    ``expect`` can be satisfied by something the device said before that test
    began: a pass that proves nothing about the test that just ran.
    """
    agent_url = _require(ENV_AGENT_URL)
    testbed_id = _require(ENV_TESTBED_ID)

    client = LabAgentClient(agent_url)
    return Dut(
        console=client.console_for(testbed_id),
        client=client,
        testbed_id=testbed_id,
        platform=os.environ.get(ENV_PLATFORM),
        build_dir=os.environ.get(ENV_BUILD_DIR),
        case_id=os.environ.get(ENV_CASE_ID),
    )


@pytest.fixture(scope="session")
def console(dut: Dut) -> RemoteConsole:
    """The raw console, for suites that want to do their own buffering.

    Session-scoped to match ``dut``: a narrower fixture may not depend on a
    wider one, and it would be the wrong shape anyway -- two tests sharing one
    device must share one view of what it has said.
    """
    return dut.console


def _declared_fixtures() -> List[str]:
    """The fixture specs this test case declared, from the runner's environment.

    Encoded as a JSON array: a fixture parameter may contain a comma, which any
    delimiter-based format would split. A malformed value is an error rather
    than an empty list -- silently reading it as "no fixtures" would turn a
    misconfigured run into a suite of skips that look like a missing
    declaration.
    """
    raw = os.environ.get(ENV_FIXTURES, "")
    if not raw:
        return []
    try:
        decoded = json.loads(raw)
    except ValueError as exc:
        raise RuntimeError(
            f"{ENV_FIXTURES} is not valid JSON ({raw!r}); the runner writes this "
            "variable, so a hand-rolled value is a mistake"
        ) from exc
    if not isinstance(decoded, list):
        raise RuntimeError(
            f"{ENV_FIXTURES} must be a JSON array of fixture specs, got {raw!r}"
        )
    return [str(entry) for entry in decoded]


def _make_instrument_fixture(name: str, factory):
    """Build one session fixture that opens ``name`` and yields ``factory``.

    The declared spec, not the bare name, is the stream: `bt_peer:peer1` pins
    one of several same-named instruments, and the agent resolves an exact spec
    before a name. The parameter is also the capability the firmware must
    advertise, so a mis-flashed lab fails at construction rather than at the
    first command.
    """

    def _fixture(dut: Dut):
        declared = [
            spec
            for spec in _declared_fixtures()
            if spec.split(":", 1)[0] == name
        ]
        if not declared:
            pytest.skip(
                f"this suite needs the {name} instrument, but the test case did "
                f'not declare it: add "fixture: {name}:<capability>" under '
                "harness_config."
            )
        if len(declared) > 1:
            pytest.fail(
                f"the test case declares {name} more than once: {declared}. "
                "A fixture resolves to one instrument; pin one spec or split the case."
            )

        # Taken only so the instrument belongs to the same session as the DUT;
        # its coordinates are not used, and nothing is built or flashed for it.
        _ = dut

        client = LabAgentClient(_require(ENV_AGENT_URL))
        testbed_id = _require(ENV_TESTBED_ID)
        spec = declared[0]
        _, _, capability = spec.partition(":")

        instrument_console = client.console_for(testbed_id, stream=spec)
        instrument_console.open()
        # Deliberately not closing the stream in a teardown: the runner reads
        # every open instrument's retained transcript right before it releases
        # the testbed and writes it next to `device.log` as `<fixture>.log`.
        yield factory(InstrumentConsole(instrument_console), capability=capability)

    _fixture.__name__ = name
    _fixture.__doc__ = (
        f"The {name!r} instrument declared by "
        f"``harness_config.fixture: {name}:<capability>``."
    )
    return pytest.fixture(scope="session", name=name)(_fixture)


#: Reject a registration that would shadow the framework or cannot be a fixture
#: name. This is checked at import time, where the mistake is cheap to see.
#: Names already bound in this module: assigning a fixture over one of them
#: would silently replace an import the plugin depends on.
_MODULE_NAMES = frozenset(globals())

for _name in INSTRUMENTS:
    if (
        _name in RESERVED_NAMES
        or _name in _MODULE_NAMES
        or not _name.isidentifier()
    ):
        raise RuntimeError(
            f"instrument fixture {_name!r} cannot be registered: it must be a "
            "Python identifier and must not shadow a framework fixture, a "
            "pytest built-in, or a name this plugin imports"
        )

for _name, _factory in INSTRUMENTS.items():
    globals()[_name] = _make_instrument_fixture(_name, _factory)
