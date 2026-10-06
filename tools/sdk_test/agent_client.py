# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""HTTP client for a Lab Agent.

Written against ``urllib`` from the standard library: the SDK already carries
enough dependencies, and this client is a few dozen lines of JSON over HTTP.

The one class here that is more than a thin wrapper is :class:`RemoteConsole`.
It presents a lab-agent console in the same shape the harnesses expect of a
local one (:meth:`read` / :meth:`write`), so the code that decides whether a test
passed never needs to know whether the device is across a wire or across the
room. It owns the cursor, which is what makes the reads stateless on the wire.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from .errors import LabAgentError
from .errors import MatchError
from .version import PROTOCOL

#: Generous default: flashing over a slow link is not a hung agent.
DEFAULT_TIMEOUT = 60.0


class LabAgentClient:
    """Talks to one Lab Agent runner."""

    def __init__(self, base_url: str, timeout: float = DEFAULT_TIMEOUT) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    # -- plumbing ------------------------------------------------------------

    def _request(
        self, method: str, path: str, body: Optional[dict] = None, timeout: Optional[float] = None
    ) -> Dict[str, Any]:
        data = None
        if body is not None:
            data = json.dumps(body).encode("utf-8")

        request = urllib.request.Request(self.base_url + path, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")

        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as error:
            # The agent puts a human-readable reason in the body; surfacing it is
            # the whole point of the error contract.
            detail = ""
            try:
                detail = json.loads(error.read() or b"{}").get("error", "")
            except (ValueError, OSError):
                detail = error.reason or ""
            raise LabAgentError(
                f"{method} {path} failed ({error.code}): {detail or error.reason}"
            ) from error
        except urllib.error.URLError as error:
            raise LabAgentError(
                f"Cannot reach the Lab Agent at {self.base_url}: {error.reason}. "
                "Start it with `python tools/lab_agent/runner.py --config <file>`."
            ) from error
        except OSError as error:
            raise LabAgentError(f"{method} {path} failed: {error}") from error

        if not raw:
            return {}
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as error:
            raise LabAgentError(
                f"The Lab Agent returned a non-JSON response to {method} {path}"
            ) from error

        if not isinstance(payload, dict):
            # Valid JSON, wrong shape -- an older or foreign server, most likely.
            # Rejecting here means a confusing AttributeError cannot surface five
            # frames deeper in a caller that assumed a dict.
            raise LabAgentError(
                f"The Lab Agent returned {type(payload).__name__} instead of an "
                f"object for {method} {path}"
            )
        return payload

    # -- control plane -------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        return self._request("GET", "/health")

    def testbeds(self) -> List[Dict[str, Any]]:
        return self._request("GET", "/testbeds").get("testbeds", [])

    def allocate(
        self,
        platform: Optional[str] = None,
        fixtures: Optional[List[str]] = None,
        testbed_id: Optional[str] = None,
        holder: Optional[str] = None,
    ) -> Dict[str, Any]:
        allocation = self._request(
            "POST",
            "/allocate",
            {
                "platform": platform,
                "fixtures": list(fixtures or []),
                "testbed_id": testbed_id,
                "holder": holder,
            },
        )
        check_protocol(allocation)
        return allocation

    def release(self, testbed_id: str, holder: Optional[str] = None) -> None:
        self._request("POST", "/release", {"testbed_id": testbed_id, "holder": holder})

    # -- testbed operations --------------------------------------------------

    def prepare(
        self, testbed_id: str, build_dir: Optional[str] = None, reset: bool = True
    ) -> Dict[str, Any]:
        return self._request(
            "POST",
            f"/testbed/{_quote(testbed_id)}/prepare",
            {"build_dir": build_dir, "reset": reset},
        )

    def reset(self, testbed_id: str) -> Dict[str, Any]:
        return self._request("POST", f"/testbed/{_quote(testbed_id)}/reset")

    def flash(self, testbed_id: str, build_dir: str) -> Dict[str, Any]:
        return self._request(
            "POST", f"/testbed/{_quote(testbed_id)}/flash", {"build_dir": build_dir}
        )

    # -- console -------------------------------------------------------------

    def console_open(self, testbed_id: str) -> None:
        self._request("POST", f"/testbed/{_quote(testbed_id)}/console/open")

    def console_close(self, testbed_id: str) -> None:
        self._request("POST", f"/testbed/{_quote(testbed_id)}/console/close")

    def console_read(
        self, testbed_id: str, cursor: int, timeout: float
    ) -> Tuple[str, int]:
        query = urllib.parse.urlencode({"cursor": int(cursor), "timeout": float(timeout)})
        payload = self._request(
            "GET",
            f"/testbed/{_quote(testbed_id)}/console/read?{query}",
            # The agent blocks for `timeout` internally, so the socket deadline
            # must be longer than that or every quiet read looks like a failure.
            timeout=max(self.timeout, float(timeout) + 10.0),
        )
        return str(payload.get("data", "")), int(payload.get("cursor", cursor))

    def console_write(self, testbed_id: str, data: str) -> None:
        self._request(
            "POST", f"/testbed/{_quote(testbed_id)}/console/write", {"data": data}
        )

    # -- convenience ---------------------------------------------------------

    def console_for(self, testbed_id: str) -> "RemoteConsole":
        return RemoteConsole(self, testbed_id)


def _quote(value: str) -> str:
    return urllib.parse.quote(str(value), safe="")


class RemoteConsole:
    """A console on a lab agent, shaped like a local one.

    The cursor lives here rather than on the wire, so every read is a pure
    request/response. A dropped connection therefore costs nothing but the bytes
    of one read -- the next call resumes exactly where the last one stopped.
    """

    def __init__(self, client: LabAgentClient, testbed_id: str) -> None:
        self._client = client
        self.testbed_id = testbed_id
        self._cursor = 0

    @property
    def cursor(self) -> int:
        """Absolute offset of the oldest byte not yet returned to a caller."""
        return self._cursor

    def read(self, timeout: float = 1.0) -> str:
        """Return text received after the last read; ``""`` if nothing arrived."""
        data, self._cursor = self._client.console_read(
            self.testbed_id, self._cursor, timeout
        )
        return data

    def read_from(self, cursor: int, timeout: float = 1.0) -> Tuple[str, int]:
        """Return text after an explicit cursor without disturbing :meth:`read`."""
        return self._client.console_read(self.testbed_id, cursor, timeout)

    def write(self, data: str) -> None:
        self._client.console_write(self.testbed_id, data)

    def send_line(self, line: str) -> None:
        self.write(line + "\r\n")

    def drain(self, timeout: float = 0.0) -> str:
        """Read and discard whatever is already buffered.

        Used before sending a command so that output from a previous run cannot
        be mistaken for the answer to this one.
        """
        return self.read(timeout)

    def skip_to_end(self) -> int:
        """Advance the cursor past everything buffered, and return it."""
        while True:
            data, cursor = self._client.console_read(self.testbed_id, self._cursor, 0.0)
            if cursor <= self._cursor:
                # Either the stream is drained, or the agent is not honouring
                # the cursor it was given. Stop either way: blindly looping
                # would spin forever and exhaust sockets against a server that
                # keeps returning the same offset.
                return self._cursor
            self._cursor = cursor


def require_testbed(allocation: Dict[str, Any]) -> str:
    """Pull the testbed id out of an allocation response."""
    testbed_id = allocation.get("testbed_id")
    if not testbed_id:
        raise MatchError(f"Lab Agent returned no testbed id: {allocation!r}")
    return str(testbed_id)


def check_protocol(allocation: Dict[str, Any]) -> None:
    """Refuse an agent that speaks a different version of this interface.

    The agent is a long-running process, so it is routinely older or newer than
    the checkout driving it, and being different is not by itself a problem --
    see :mod:`sdk_test.version`. Speaking *differently* is, because the
    alternative to refusing here is a field the other side cannot read showing
    up as a wrong result three calls later.
    """
    spoken = allocation.get("protocol")
    if not isinstance(spoken, int) or isinstance(spoken, bool):
        raise LabAgentError(
            "The Lab Agent did not say which protocol it speaks, so it is from "
            f"before this checkout was written (which speaks {PROTOCOL}). "
            "Restart it from the same revision: python tools/lab_agent/runner.py "
            "--config <file>."
        )
    if spoken != PROTOCOL:
        raise LabAgentError(
            f"The Lab Agent speaks protocol {spoken}, this checkout speaks "
            f"{PROTOCOL}. The two cannot be mixed; restart the agent from the "
            "same revision."
        )
