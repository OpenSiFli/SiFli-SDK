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

import base64
import binascii
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

#: Uploading a package is the one call whose duration follows the payload, so it
#: gets a floor of its own rather than the request default.
UPLOAD_TIMEOUT = 300.0

#: Opening a CDC port can outlast the default: the OS may wait for the device's
#: SET_LINE_CODING handler, and some examples take about a minute to answer.
#: Cutting that short aborts the request just as the agent is opening the port.
SERIAL_OPEN_TIMEOUT = 150.0

#: ``code`` carried by a failure that never reached the agent. The agent's own
#: error bodies name their cause; a local socket failure has no body, and
#: callers still need to tell "the bench is unreachable" from "the answer was
#: unusable" -- both arrive with ``status=None``.
TRANSPORT_ERROR_CODE = "transport"


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

        return self._send(request, method, path, timeout)

    def _send(
        self, request: urllib.request.Request, method: str, path: str, timeout: Optional[float]
    ) -> Dict[str, Any]:
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as error:
            # The agent puts a human-readable reason in the body, and on the
            # routes that need it a machine-readable `code` beside it. Both are
            # read here: the status alone cannot tell "no such route" from "no
            # such device", and a second fetch of the body is impossible.
            detail = ""
            code: Optional[str] = None
            details: Any = None
            try:
                body = json.loads(error.read() or b"{}")
            except (ValueError, OSError):
                body = {}
            if isinstance(body, dict):
                reported = body.get("error")
                if reported:
                    detail = str(reported)
                reported_code = body.get("code")
                if isinstance(reported_code, str) and reported_code:
                    code = reported_code
                details = body.get("details")
            if not detail:
                detail = error.reason or ""
            message = f"{method} {path} failed ({error.code}): {detail}"
            if details:
                # A 408 from `usb/wait` carries the devices that could have
                # matched. It has no field on the exception -- a caller keys on
                # the type, and a suite reads this message -- so it is rendered
                # here rather than dropped.
                message += " " + json.dumps(details, sort_keys=True)
            raise LabAgentError(
                message, status=error.code, code=code
            ) from error
        except urllib.error.URLError as error:
            raise LabAgentError(
                f"Cannot reach the Lab Agent at {self.base_url}: {error.reason}. "
                "Start it with `python tools/lab_agent/runner.py --config <file>`.",
                code=TRANSPORT_ERROR_CODE,
            ) from error
        except OSError as error:
            raise LabAgentError(
                f"{method} {path} failed: {error}", code=TRANSPORT_ERROR_CODE
            ) from error

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
        return payload

    # -- control plane -------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        return self._request("GET", "/health")

    def testbeds(self) -> List[Dict[str, Any]]:
        """What this agent can host, as its own binding describes it.

        An answer without a ``testbeds`` list is refused rather than read as an
        empty lab: "there is no hardware here" and "this server did not answer
        the question" lead an operator to opposite conclusions, and only the
        first is about the bench.
        """
        payload = self._request("GET", "/testbeds")
        if "testbeds" not in payload:
            raise LabAgentError(
                "The Lab Agent's GET /testbeds response has no \"testbeds\" "
                "list; it is incomplete, or this URL is not a Lab Agent serving "
                "the version of the interface this checkout speaks"
            )
        entries = payload["testbeds"]
        if not isinstance(entries, list):
            raise LabAgentError(
                "The Lab Agent returned a "
                f"{type(entries).__name__} for GET /testbeds \"testbeds\"; "
                "expected a list"
            )
        return entries

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

    def upload_firmware(self, testbed_id: str, payload: bytes) -> Dict[str, Any]:
        """Send a firmware package. The body is a tar, not JSON.

        The agent unpacks it and answers with the token that names it, which is
        what the later ``prepare`` must quote. Nothing is flashed by this call:
        a package that is still travelling is not a package to program a board
        from.
        """
        request = urllib.request.Request(
            f"{self.base_url}/testbed/{_quote(testbed_id)}/firmware",
            data=payload,
            method="POST",
        )
        request.add_header("Content-Type", "application/octet-stream")
        return self._send(
            request,
            "POST",
            f"/testbed/{testbed_id}/firmware",
            timeout=max(self.timeout, UPLOAD_TIMEOUT),
        )

    def prepare(
        self, testbed_id: str, firmware: str, reset: bool = True
    ) -> Dict[str, Any]:
        return self._request(
            "POST",
            f"/testbed/{_quote(testbed_id)}/prepare",
            {"firmware": firmware, "reset": reset},
        )

    def reset(self, testbed_id: str) -> Dict[str, Any]:
        return self._request("POST", f"/testbed/{_quote(testbed_id)}/reset")

    def flash(self, testbed_id: str, firmware: str) -> Dict[str, Any]:
        return self._request(
            "POST", f"/testbed/{_quote(testbed_id)}/flash", {"firmware": firmware}
        )

    def read_memory(self, testbed_id: str, address: int, length: int) -> bytes:
        """Read ``length`` bytes of target memory through the lab's probe.

        The lab owns the probe, so the suite never names one; it asks for an
        address and a size, exactly as it would name a console.
        """
        query = urllib.parse.urlencode(
            {"address": hex(int(address)), "length": int(length)}
        )
        payload = self._request(
            "GET", f"/testbed/{_quote(testbed_id)}/memory?{query}"
        )
        encoded = payload.get("data")
        if not isinstance(encoded, str):
            raise LabAgentError(
                f"the Lab Agent returned no memory data for {testbed_id} "
                f"at 0x{int(address):08x}"
            )
        try:
            data = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise LabAgentError(
                f"the Lab Agent returned undecodable memory data: {exc}"
            ) from exc
        if len(data) != int(length):
            raise LabAgentError(
                f"the Lab Agent returned {len(data)} of {int(length)} memory "
                f"bytes at 0x{int(address):08x}"
            )
        return data

    # -- console -------------------------------------------------------------

    # `stream` names which console on the testbed: "dut" (the default, so every
    # existing caller is unchanged) or the name of an instrument fixture the
    # allocation declared. An instrument is not a testbed -- it has no build,
    # no flash and no reset -- so it gets a stream, not an allocation of its own.

    @staticmethod
    def _stream_query(stream: str) -> str:
        return "" if stream == "dut" else "?" + urllib.parse.urlencode({"stream": stream})

    def console_open(self, testbed_id: str, stream: str = "dut") -> None:
        self._request(
            "POST",
            f"/testbed/{_quote(testbed_id)}/console/open{self._stream_query(stream)}",
        )

    def console_close(self, testbed_id: str, stream: str = "dut") -> None:
        self._request(
            "POST",
            f"/testbed/{_quote(testbed_id)}/console/close{self._stream_query(stream)}",
        )

    def console_read(
        self, testbed_id: str, cursor: int, timeout: float, stream: str = "dut"
    ) -> Tuple[str, int]:
        fields = {"cursor": int(cursor), "timeout": float(timeout)}
        if stream != "dut":
            fields["stream"] = stream
        query = urllib.parse.urlencode(fields)
        payload = self._request(
            "GET",
            f"/testbed/{_quote(testbed_id)}/console/read?{query}",
            # The agent blocks for `timeout` internally, so the socket deadline
            # must be longer than that or every quiet read looks like a failure.
            timeout=max(self.timeout, float(timeout) + 10.0),
        )
        return str(payload.get("data", "")), int(payload.get("cursor", cursor))

    def console_write(self, testbed_id: str, data: str, stream: str = "dut") -> None:
        self._request(
            "POST",
            f"/testbed/{_quote(testbed_id)}/console/write{self._stream_query(stream)}",
            {"data": data},
        )

    # -- usb host ------------------------------------------------------------
    #
    # The USB evidence a device example needs is collected on the machine the
    # device port is plugged into, which is the agent's machine. These are the
    # raw routes; `sdk_test.usb_host` shapes them for a suite.

    @staticmethod
    def _usb_path(testbed_id: str, suffix: str) -> str:
        return f"/testbed/{_quote(testbed_id)}/usb/{suffix}"

    def usb_capabilities(self, testbed_id: str) -> Dict[str, Any]:
        return self._request("GET", self._usb_path(testbed_id, "capabilities"))

    def usb_devices(self, testbed_id: str) -> Dict[str, Any]:
        return self._request("GET", self._usb_path(testbed_id, "devices"))

    def usb_wait(
        self,
        testbed_id: str,
        match: Dict[str, Any],
        since: Optional[List[str]] = None,
        timeout: float = 15.0,
        interval: float = 0.25,
    ) -> Dict[str, Any]:
        # The agent blocks for `timeout` internally, so the socket deadline
        # must outlast it or a quiet wait looks like a transport failure.
        return self._request(
            "POST",
            self._usb_path(testbed_id, "wait"),
            {
                "match": dict(match),
                "since": list(since or []),
                "timeout": float(timeout),
                "interval": float(interval),
            },
            timeout=max(self.timeout, float(timeout) + 10.0),
        )

    def usb_serial_open(
        self,
        testbed_id: str,
        device_key: str,
        com_port: str,
        baud: int = 115200,
        dtr: bool = False,
        rts: bool = False,
    ) -> Dict[str, Any]:
        return self._request(
            "POST",
            self._usb_path(testbed_id, "serial/open"),
            {
                "device_key": device_key,
                "com_port": com_port,
                "baud": int(baud),
                "dtr": bool(dtr),
                "rts": bool(rts),
            },
            timeout=max(self.timeout, SERIAL_OPEN_TIMEOUT),
        )

    def usb_serial_read(
        self, testbed_id: str, token: str, cursor: int, timeout: float = 1.0
    ) -> Tuple[bytes, int, bool]:
        """Return ``(data, cursor, closed)``; ``data`` is already decoded."""
        query = urllib.parse.urlencode(
            {"cursor": int(cursor), "timeout": float(timeout)}
        )
        payload = self._request(
            "GET",
            self._usb_path(testbed_id, f"serial/{_quote(token)}/read") + "?" + query,
            timeout=max(self.timeout, float(timeout) + 10.0),
        )
        encoded = payload.get("data", "")
        if not isinstance(encoded, str):
            raise LabAgentError(
                "the Lab Agent returned serial data that is not base64 text"
            )
        try:
            data = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise LabAgentError(
                f"the Lab Agent returned undecodable serial data: {exc}"
            ) from exc
        return data, int(payload.get("cursor", cursor)), bool(payload.get("closed"))

    def usb_serial_write(self, testbed_id: str, token: str, data: bytes) -> int:
        payload = self._request(
            "POST",
            self._usb_path(testbed_id, f"serial/{_quote(token)}/write"),
            {"data_base64": base64.b64encode(data).decode("ascii")},
        )
        return int(payload.get("written") or 0)

    def usb_serial_lines(
        self, testbed_id: str, token: str, dtr: bool, rts: bool
    ) -> Dict[str, Any]:
        return self._request(
            "POST",
            self._usb_path(testbed_id, f"serial/{_quote(token)}/lines"),
            {"dtr": bool(dtr), "rts": bool(rts)},
        )

    def usb_serial_close(self, testbed_id: str, token: str) -> None:
        self._request(
            "POST", self._usb_path(testbed_id, f"serial/{_quote(token)}/close")
        )

    def usb_volumes(self, testbed_id: str) -> Dict[str, Any]:
        return self._request("GET", self._usb_path(testbed_id, "volumes"))

    def usb_file(
        self,
        testbed_id: str,
        device_key: str,
        path: str,
        encoding: str = "text",
    ) -> Dict[str, Any]:
        query = urllib.parse.urlencode(
            {"device_key": device_key, "path": path, "encoding": encoding}
        )
        return self._request(
            "GET", self._usb_path(testbed_id, "files") + "?" + query
        )

    def usb_audio(self, testbed_id: str, device_key: str) -> Dict[str, Any]:
        query = urllib.parse.urlencode({"device_key": device_key})
        return self._request(
            "GET", self._usb_path(testbed_id, "audio") + "?" + query
        )

    def usb_hid(self, testbed_id: str, device_key: str) -> Dict[str, Any]:
        query = urllib.parse.urlencode({"device_key": device_key})
        return self._request(
            "GET", self._usb_path(testbed_id, "hid") + "?" + query
        )

    # -- convenience ---------------------------------------------------------

    def console_for(self, testbed_id: str, stream: str = "dut") -> "RemoteConsole":
        return RemoteConsole(self, testbed_id, stream)


def _quote(value: str) -> str:
    return urllib.parse.quote(str(value), safe="")


class RemoteConsole:
    """A console on a lab agent, shaped like a local one.

    The cursor lives here rather than on the wire, so every read is a pure
    request/response. A dropped connection therefore costs nothing but the bytes
    of one read -- the next call resumes exactly where the last one stopped.
    """

    def __init__(
        self, client: LabAgentClient, testbed_id: str, stream: str = "dut"
    ) -> None:
        self._client = client
        self.testbed_id = testbed_id
        #: "dut" for the board under test, or the name of an instrument fixture
        #: the allocation declared. The cursor semantics are identical; only
        #: which physical port the agent reads differs.
        self.stream = stream
        self._cursor = 0

    @property
    def cursor(self) -> int:
        """Absolute offset of the oldest byte not yet returned to a caller."""
        return self._cursor

    def read(self, timeout: float = 1.0) -> str:
        """Return text received after the last read; ``""`` if nothing arrived."""
        data, self._cursor = self._client.console_read(
            self.testbed_id, self._cursor, timeout, stream=self.stream
        )
        return data

    def read_from(self, cursor: int, timeout: float = 1.0) -> Tuple[str, int]:
        """Return text after an explicit cursor without disturbing :meth:`read`."""
        return self._client.console_read(
            self.testbed_id, cursor, timeout, stream=self.stream
        )

    def write(self, data: str) -> None:
        self._client.console_write(self.testbed_id, data, stream=self.stream)

    def open(self) -> None:
        """Open the stream on the agent. The DUT's is opened by the runner."""
        self._client.console_open(self.testbed_id, stream=self.stream)

    def close(self) -> None:
        self._client.console_close(self.testbed_id, stream=self.stream)
        # The agent drops the stream on close; a later open() creates a new one
        # whose cursor starts at 0. Keeping the old cursor here would make the
        # first read after reopening skip everything the new stream already
        # held, which reads as a device that went quiet.
        self._cursor = 0

    def for_stream(self, stream: str) -> "RemoteConsole":
        """Another console on the same testbed, with its own cursor.

        Used to read an instrument's retained transcript without disturbing the
        cursor of the console a harness is still reading through.
        """
        return RemoteConsole(self._client, self.testbed_id, stream)

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
            data, cursor = self._client.console_read(
                self.testbed_id, self._cursor, 0.0, stream=self.stream
            )
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
