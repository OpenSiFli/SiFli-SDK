# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The Lab Agent's HTTP control plane.

Deliberately built on the standard library's ``http.server`` rather than a web
framework: the agent runs on a lab PC next to the hardware, and adding an ASGI
stack there would be a deployment problem for no benefit. The SDK side talks to
it with :mod:`urllib` for the same reason.

Console output is *not* streamed. The client asks for everything after a cursor
and the server blocks up to a timeout for something to arrive. Streaming over
HTTP would mean chunked responses, reconnection logic and a second source of
truth about how far the client has read; a cursor is a single integer and
survives a dropped connection.

Routes::

    GET  /health
    GET  /testbeds
    POST /allocate                     {platform, fixtures, testbed_id, holder}
    POST /release                      {testbed_id, holder}
    POST /testbed/{id}/firmware        body = a firmware package (tar)
    POST /testbed/{id}/prepare         {firmware, reset}
    POST /testbed/{id}/reset
    POST /testbed/{id}/flash           {firmware}
    GET  /testbed/{id}/memory?address=0x...&length=N
    POST /testbed/{id}/console/open           [?stream=dut|ble_peer]
    POST /testbed/{id}/console/close          [?stream=dut|ble_peer]
    GET  /testbed/{id}/console/read?cursor=N&timeout=SECONDS[&stream=...]
    POST /testbed/{id}/console/write   {data}   [?stream=dut|ble_peer]
    GET  /testbed/{id}/usb/capabilities
    GET  /testbed/{id}/usb/devices
    POST /testbed/{id}/usb/wait                {match, since, timeout, interval}
    POST /testbed/{id}/usb/serial/open         {device_key, com_port, baud, dtr, rts}
    GET  /testbed/{id}/usb/serial/{token}/read?cursor=N&timeout=S
    POST /testbed/{id}/usb/serial/{token}/write  {data_base64}
    POST /testbed/{id}/usb/serial/{token}/lines  {dtr, rts}
    POST /testbed/{id}/usb/serial/{token}/close
    GET  /testbed/{id}/usb/volumes
    GET  /testbed/{id}/usb/files?device_key=...&path=...&encoding=text|base64
    GET  /testbed/{id}/usb/audio?device_key=...
    GET  /testbed/{id}/usb/hid?device_key=...
"""

from __future__ import annotations

import base64
import hashlib
import json
import logging
import re
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from typing import Any
from typing import Dict
from typing import Optional
from typing import Tuple
from urllib.parse import parse_qs
from urllib.parse import urlparse

from sdk_test.errors import BindingError
from sdk_test.errors import FirmwareError
from sdk_test.firmware import MAX_TOTAL_BYTES

from .errors import AllocationError
from .errors import ConfigError
from .errors import LabAgentError
from .errors import TestbedError
from .errors import TransportError
from .testbed import LabManager
from .testbed import allocate_request_from_mapping
from .usb_host.errors import UsbHostBadRequest
from .usb_host.errors import UsbHostError
from .usb_host.errors import UsbHostUnsupported
from .usb_host.serial_sessions import MAX_WRITE_BYTES

#: Default value for an omitted ``timeout`` on a console read, in seconds.
DEFAULT_READ_TIMEOUT = 1.0

#: Upper bound on a single console read, so a client cannot pin a thread forever.
MAX_READ_TIMEOUT = 60.0

#: Upper bound on one memory read. The response is JSON (base64), so this also
#: bounds what a caller can make the agent encode and send; a test that needs
#: more can issue several reads.
MAX_MEMORY_READ = 4 * 1024 * 1024

_ERROR_STATUS = {
    AllocationError: 409,
    TestbedError: 409,
    TransportError: 502,
    ConfigError: 500,
    # A package the agent cannot accept is the sender's problem, not the lab's:
    # it was built and packed elsewhere. 400 rather than 5xx so a caller goes
    # looking at its own build step instead of at the bench.
    FirmwareError: 400,
    # A fixture spec the caller wrote cannot be parsed. It reaches this layer on
    # several paths -- an allocation's `fixtures`, a console stream name -- and
    # is always the sender's mistake, so it must not surface as a 500.
    BindingError: 400,
    # Least specific, so it must be checked last: a plain LabAgentError is a
    # malformed request (bad JSON, a body that is not an object), which is the
    # client's fault, not the agent's. Reporting it as 500 would make a caller's
    # bad request look like a crashed lab and bury it in a stack trace.
    LabAgentError: 400,
}


def _status_for(error: Exception) -> int:
    # USB errors carry their own status: the capability's contract has more
    # statuses (408, 413, 501, 503) than the base error families do, and a
    # route should not have to translate them a second time.
    status = getattr(error, "status", None)
    if isinstance(status, int):
        return status
    for error_type, status in _ERROR_STATUS.items():
        if isinstance(error, error_type):
            return status
    return 500


#: A serial token is a capability for a COM port, and an HTTP path is logged.
#: Redacting it keeps the log useful for routing without leaving a usable
#: secret in a log file the lab may ship with a run report.
_SERIAL_TOKEN_PATH = re.compile(r"(/usb/serial/)([^/?#\s]+)")


def _redact_tokens(text: str) -> str:
    def label(match: "re.Match[str]") -> str:
        digest = hashlib.sha256(match.group(2).encode("utf-8")).hexdigest()[:12]
        return f"{match.group(1)}<token:{digest}>"

    return _SERIAL_TOKEN_PATH.sub(label, text)


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "SiFliLabAgent/0.1"

    # -- plumbing ------------------------------------------------------------

    @property
    def manager(self) -> LabManager:
        return self.server.manager  # type: ignore[attr-defined]

    @property
    def logger(self) -> logging.Logger:
        return self.server.logger  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        # Default behaviour writes straight to stderr on every request, which
        # buries the agent's own logging. Route it through the logger instead,
        # with USB serial tokens redacted.
        self.logger.debug(
            "%s - %s", self.address_string(), _redact_tokens(fmt % args)
        )

    def _read_body(self) -> Dict[str, Any]:
        length = self.headers.get("Content-Length")
        if not length:
            return {}
        try:
            raw = self.rfile.read(int(length))
        except (ValueError, OSError) as exc:
            raise LabAgentError(f"Cannot read request body: {exc}") from exc
        if not raw:
            return {}
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise LabAgentError(f"Request body is not valid JSON: {exc}") from exc
        if not isinstance(data, dict):
            raise LabAgentError("Request body must be a JSON object")
        return data

    def _read_body_bytes(self) -> bytes:
        """Read a raw body -- a firmware package, not JSON.

        The length is taken from the header and enforced before anything is
        read, so a caller cannot make the agent buffer an unbounded amount of
        whatever it feels like sending.
        """
        length = self.headers.get("Content-Length")
        if not length:
            raise LabAgentError('A firmware upload needs a "Content-Length" header')
        try:
            size = int(length)
        except ValueError as exc:
            raise LabAgentError(f'Bad "Content-Length": {length!r}') from exc
        if size <= 0:
            raise LabAgentError("A firmware upload cannot be empty")
        if size > MAX_TOTAL_BYTES:
            raise LabAgentError(
                f"Firmware upload is {size} bytes; at most {MAX_TOTAL_BYTES} "
                "are accepted"
            )
        try:
            return self.rfile.read(size)
        except OSError as exc:
            raise LabAgentError(f"Cannot read the firmware upload: {exc}") from exc

    def _respond(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _fail(self, status: int, message: str) -> None:
        self._respond(status, {"error": message})

    # -- verbs ---------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 - name fixed by BaseHTTPRequestHandler
        self._handle("GET")

    def do_POST(self) -> None:  # noqa: N802 - name fixed by BaseHTTPRequestHandler
        self._handle("POST")

    def _handle(self, method: str) -> None:
        parsed = urlparse(self.path)
        segments = [segment for segment in parsed.path.split("/") if segment]
        # Blank values are kept so `?stream=` is a visible mistake (a 400 below)
        # rather than silently falling back to the DUT's console.
        query = parse_qs(parsed.query, keep_blank_values=True)

        try:
            status, payload = self._dispatch(method, segments, query)
        except Exception as exc:  # noqa: BLE001 - every failure becomes a response
            status = _status_for(exc)
            payload = {"error": str(exc)}
            if isinstance(exc, UsbHostError):
                payload["code"] = exc.code
                payload["details"] = exc.details
            log_path = _redact_tokens(self.path)
            if status >= 500 and not isinstance(exc, UsbHostUnsupported):
                self.logger.exception("Request failed: %s %s", method, log_path)
            else:
                self.logger.info(
                    "%s %s -> %s: %s", method, log_path, status, exc
                )

        try:
            self._respond(status, payload)
        except (BrokenPipeError, ConnectionResetError):
            # The client went away mid-response; nothing useful to do.
            self.logger.debug("Client disconnected before the response was sent")

    def _dispatch(
        self, method: str, segments: list, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        if not segments:
            return 404, {"error": "Not found", "routes": __doc__}

        head = segments[0]

        if head == "health":
            if method != "GET":
                return 405, {"error": "GET only"}
            return 200, {
                "status": "ok",
                "runner": self.manager.config.settings.name,
                "testbeds": len(self.manager.ids()),
                # What this agent speaks, and what it is built from. A caller
                # compares the first and records the second -- see
                # sdk_test.version for why only one of them is a gate.
                "protocol": self.manager.protocol,
                "revision": self.manager.revision,
            }

        if head == "testbeds":
            if method != "GET":
                return 405, {"error": "GET only"}
            return 200, {"testbeds": self.manager.list()}

        if head == "allocate":
            if method != "POST":
                return 405, {"error": "POST only"}
            request = allocate_request_from_mapping(self._read_body())
            runtime = self.manager.allocate(request)
            self.logger.info(
                "Allocated %s to %s", runtime.testbed.id, runtime.holder
            )
            return 200, {
                "testbed_id": runtime.testbed.id,
                "platform": runtime.testbed.platform,
                "holder": runtime.holder,
                "console": runtime.describe()["console"],
                # Carried on the allocation, not on the later /reset call,
                # because the caller has to know before it flashes: a manual
                # reset only works for a harness that never writes to the
                # console, and that is worth refusing before a flash, not after.
                "reset": runtime.describe()["reset"],
                # Repeating /health's two answers here because this is the one
                # call every hardware run makes, and the revision has to reach
                # that run's report.
                "protocol": self.manager.protocol,
                "revision": self.manager.revision,
            }

        if head == "release":
            if method != "POST":
                return 405, {"error": "POST only"}
            body = self._read_body()
            testbed_id = body.get("testbed_id")
            if not testbed_id:
                return 400, {"error": '"testbed_id" is required'}
            self.manager.release(str(testbed_id), body.get("holder"))
            self.logger.info("Released %s", testbed_id)
            return 200, {"released": True, "testbed_id": str(testbed_id)}

        if head == "testbed" and len(segments) >= 3:
            return self._dispatch_testbed(method, segments[1], segments[2:], query)

        return 404, {"error": f"No such route: /{'/'.join(segments)}"}

    def _dispatch_testbed(
        self, method: str, testbed_id: str, action: list, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        try:
            runtime = self.manager.get(testbed_id)
        except TestbedError as exc:
            return 404, {"error": str(exc)}

        name = action[0]

        if name == "firmware":
            if method != "POST":
                return 405, {"error": "POST only"}
            staged = runtime.firmware.stage(self._read_body_bytes())
            self.logger.info(
                "Staged firmware %s (%s bytes) for %s",
                staged["token"][:12],
                staged["bytes"],
                testbed_id,
            )
            return 200, staged

        if name == "prepare":
            if method != "POST":
                return 405, {"error": "POST only"}
            body = self._read_body()
            runtime.prepare(
                build_dir=runtime.firmware.path(str(body.get("firmware") or "")),
                reset=bool(body.get("reset", True)),
            )
            return 200, {"prepared": True, "testbed_id": testbed_id}

        if name == "reset":
            if method != "POST":
                return 405, {"error": "POST only"}
            mode = runtime.reset_mode()
            runtime.reset()
            # `reset` says whether *this call* reset the board. Only `jlink`
            # does: under `manual` nothing was done and the operator is still
            # expected to press the button, and under `auto` the boot is the
            # download's rather than ours. Either way, claiming `true` would be
            # the kind of confident-wrong answer this codebase refuses
            # elsewhere (see RelayNotAvailable).
            return 200, {
                "testbed_id": testbed_id,
                "mode": mode,
                "reset": mode == "jlink",
            }

        if name == "flash":
            if method != "POST":
                return 405, {"error": "POST only"}
            body = self._read_body()
            token = str(body.get("firmware") or "")
            if not token:
                return 400, {"error": '"firmware" is required'}
            runtime.flash(runtime.firmware.path(token))
            return 200, {"flashed": True, "testbed_id": testbed_id}

        if name == "memory":
            if method != "GET":
                return 405, {"error": "GET only"}
            address = self._required_query_int(query, "address")
            length = self._required_query_int(query, "length")
            if address < 0:
                return 400, {"error": '"address" must not be negative'}
            if length <= 0 or length > MAX_MEMORY_READ:
                return 400, {
                    "error": f'"length" must be between 1 and {MAX_MEMORY_READ}'
                }
            data = runtime.read_memory(address, length)
            return 200, {
                "testbed_id": testbed_id,
                "address": address,
                "length": length,
                # Base64 rather than a JSON array of numbers: the payload is
                # binary and can be a framebuffer, and a host-side suite wants
                # bytes, not a list to reassemble.
                "data": base64.b64encode(data).decode("ascii"),
            }

        if name == "usb" and action[1:]:
            return self._dispatch_usb(method, runtime, action[1:], query)

        if name == "console" and action[1:]:
            return self._dispatch_console(method, runtime, testbed_id, action[1], query)

        return 404, {"error": f"No such testbed action: {name}"}

    @staticmethod
    def _usb_error(
        status: int, message: str, code: str, details: Any = None
    ) -> Tuple[int, Any]:
        """The USB error body, including the fields the client switches on."""
        return status, {
            "error": message,
            "code": code,
            "details": dict(details or {}),
        }

    def _dispatch_usb(
        self, method: str, runtime, action: list, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        """Host-side USB observation for one testbed.

        Every route except ``capabilities`` asks the runtime for a gated
        service; the gate is what keeps a run from reading (or opening) hardware
        it never declared. ``capabilities`` is deliberately ungated, because its
        answer is how a client learns the bench has no USB backend at all.
        """
        sub = action[0]

        if sub == "capabilities":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            return 200, runtime.usb_host_capabilities()

        if sub == "serial":
            if action[1:] == ["open"]:
                # The runtime performs the gate and the open under its teardown
                # lock; doing it here as two calls would leave a window where a
                # release slips in between them.
                return self._dispatch_usb_serial(
                    method, runtime, action[1:], query
                )
            if len(action) == 3:
                # A token-addressed route is answered from the retained session
                # registry without the allocation gate. The contract promises
                # that a token from a released run is ``token_not_found`` (404),
                # and the gate would answer ``not_allocated`` (409) first. The
                # registry is emptied by teardown, so the token is the only
                # thing that could still address anything, and it is gone.
                # ``usb_host_service`` takes the teardown lock, so this either
                # runs before close_all or sees its empty result.
                return self._dispatch_usb_serial(
                    method, runtime.usb_host_service(), action[1:], query
                )
            return self._usb_error(404, "No such usb serial action", "not_found")

        service = runtime.usb_host_for_request()

        if sub == "devices":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            return 200, service.devices_payload()

        if sub == "wait":
            if method != "POST":
                return self._usb_error(405, "POST only", "method_not_allowed")
            body = self._read_body()
            device, elapsed = service.wait(
                body.get("match"),
                since=body.get("since"),
                timeout=body.get("timeout", 15.0),
                interval=body.get("interval", 0.25),
            )
            return 200, {"device": device.to_dict(), "elapsed": elapsed}

        if sub == "volumes":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            return 200, service.volumes_payload()

        if sub == "files":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            device_key = self._required_usb_query(query, "device_key")
            path = self._required_usb_query(query, "path")
            encoding = (query.get("encoding") or ["text"])[0]
            return 200, service.read_file(device_key, path, encoding)

        if sub == "audio":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            return 200, service.audio_payload(
                self._required_usb_query(query, "device_key")
            )

        if sub == "hid":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            return 200, service.hid_payload(
                self._required_usb_query(query, "device_key")
            )

        return self._usb_error(404, f"No such usb action: {sub}", "not_found")

    def _dispatch_usb_serial(
        self, method: str, target, action: list, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        # ``target`` is the runtime for ``open`` (the gate and the open must be
        # one locked operation) and the service for every token-addressed verb.
        if action == ["open"]:
            if method != "POST":
                return self._usb_error(405, "POST only", "method_not_allowed")
            body = self._read_body()
            device_key = body.get("device_key")
            if not device_key:
                return self._usb_error(
                    400, '"device_key" is required', "bad_request"
                )
            baud = body.get("baud")
            if baud is not None:
                try:
                    baud = int(baud)
                except (TypeError, ValueError) as exc:
                    raise UsbHostBadRequest('"baud" must be an integer') from exc
            dtr = body.get("dtr", False)
            rts = body.get("rts", False)
            if not isinstance(dtr, bool) or not isinstance(rts, bool):
                return self._usb_error(
                    400,
                    '"dtr" and "rts" must be true or false',
                    "bad_request",
                )
            return 200, target.usb_host_open_serial(
                str(device_key),
                None if body.get("com_port") is None else str(body["com_port"]),
                baud=baud,
                dtr=dtr,
                rts=rts,
            )

        if len(action) != 2:
            return self._usb_error(404, "No such usb serial action", "not_found")
        token, operation = action

        if operation == "read":
            if method != "GET":
                return self._usb_error(405, "GET only", "method_not_allowed")
            cursor = self._usb_query_int(query, "cursor", 0)
            timeout = self._usb_query_float(
                query, "timeout", DEFAULT_READ_TIMEOUT
            )
            if timeout < 0:
                return self._usb_error(
                    400, '"timeout" must not be negative', "bad_request"
                )
            return 200, target.read_serial(
                token, cursor, min(timeout, MAX_READ_TIMEOUT)
            )

        if operation == "write":
            if method != "POST":
                return self._usb_error(405, "POST only", "method_not_allowed")
            body = self._read_body()
            raw = body.get("data_base64")
            if raw is None:
                return self._usb_error(
                    400, '"data_base64" is required', "bad_request"
                )
            try:
                data = base64.b64decode(str(raw), validate=True)
            except (ValueError, TypeError) as exc:
                raise UsbHostBadRequest('"data_base64" is not valid base64') from exc
            if len(data) > MAX_WRITE_BYTES:
                return self._usb_error(
                    400,
                    f"at most {MAX_WRITE_BYTES} bytes may be written at once",
                    "bad_request",
                )
            return 200, target.write_serial(token, data)

        if operation == "lines":
            if method != "POST":
                return self._usb_error(405, "POST only", "method_not_allowed")
            body = self._read_body()
            if "dtr" not in body and "rts" not in body:
                return self._usb_error(
                    400, '"dtr" or "rts" is required', "bad_request"
                )
            dtr = body.get("dtr")
            rts = body.get("rts")
            if (dtr is not None and not isinstance(dtr, bool)) or (
                rts is not None and not isinstance(rts, bool)
            ):
                return self._usb_error(
                    400,
                    '"dtr" and "rts" must be true or false',
                    "bad_request",
                )
            return 200, target.set_lines(token, dtr, rts)

        if operation == "close":
            if method != "POST":
                return self._usb_error(405, "POST only", "method_not_allowed")
            return 200, target.close_serial(token)

        return self._usb_error(
            404, f"No such usb serial action: {operation}", "not_found"
        )

    def _required_query(self, query: Dict[str, list], key: str) -> str:
        values = query.get(key)
        if not values or not values[0].strip():
            raise LabAgentError(f'Query parameter "{key}" is required')
        return values[0]

    def _required_usb_query(self, query: Dict[str, list], key: str) -> str:
        try:
            return self._required_query(query, key)
        except LabAgentError as exc:
            raise UsbHostBadRequest(str(exc)) from exc

    def _usb_query_int(self, query: Dict[str, list], key: str, default: int) -> int:
        try:
            return self._query_int(query, key, default)
        except LabAgentError as exc:
            raise UsbHostBadRequest(str(exc)) from exc

    def _usb_query_float(
        self, query: Dict[str, list], key: str, default: float
    ) -> float:
        try:
            return self._query_float(query, key, default)
        except LabAgentError as exc:
            raise UsbHostBadRequest(str(exc)) from exc

    def _dispatch_console(
        self, method: str, runtime, testbed_id: str, action: str, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        # One console per DUT, plus one per instrument fixture the allocation
        # asked for. `stream` names which; the default keeps every existing
        # caller and URL unchanged.
        stream = (query.get("stream") or ["dut"])[0]
        if not stream:
            return 400, {"error": '"stream" must not be empty'}
        instrument = stream != "dut"

        if action == "open":
            if method != "POST":
                return 405, {"error": "POST only"}
            if instrument:
                runtime.fixture_console_open(stream)
            else:
                runtime.console_open()
            return 200, {"console_open": True, "testbed_id": testbed_id, "stream": stream}

        if action == "close":
            if method != "POST":
                return 405, {"error": "POST only"}
            if instrument:
                runtime.fixture_console_close(stream)
            else:
                runtime.console_close()
            return 200, {"console_open": False, "testbed_id": testbed_id, "stream": stream}

        if action == "read":
            if method != "GET":
                return 405, {"error": "GET only"}
            cursor = self._query_int(query, "cursor", 0)
            timeout = self._query_float(query, "timeout", DEFAULT_READ_TIMEOUT)
            if timeout < 0:
                return 400, {"error": '"timeout" must not be negative'}
            timeout = min(timeout, MAX_READ_TIMEOUT)
            if instrument:
                data, new_cursor = runtime.fixture_console_read(stream, cursor, timeout)
            else:
                data, new_cursor = runtime.console_read(cursor, timeout)
            return 200, {"data": data, "cursor": new_cursor}

        if action == "write":
            if method != "POST":
                return 405, {"error": "POST only"}
            data = self._read_body().get("data")
            if data is None:
                return 400, {"error": '"data" is required'}
            if instrument:
                runtime.fixture_console_write(stream, str(data))
            else:
                runtime.console_write(str(data))
            return 200, {"written": len(str(data))}

        return 404, {"error": f"No such console action: {action}"}

    def _query_int(self, query: Dict[str, list], key: str, default: int) -> int:
        values = query.get(key)
        if not values:
            return default
        try:
            return int(values[0])
        except (TypeError, ValueError) as exc:
            raise LabAgentError(f'Query parameter "{key}" must be an integer') from exc

    def _required_query_int(self, query: Dict[str, list], key: str) -> int:
        """A query parameter that must be present, read as decimal or 0x-hex.

        Addresses are written in hex by every tool that shows one, so accepting
        ``0x2005a7c0`` is not a convenience -- it is the spelling a caller will
        reach for. Decimal is parsed as base 10 explicitly rather than with
        ``int(x, 0)``, which rejects a legal value such as ``length=0008``.
        """
        values = query.get(key)
        if not values or not values[0].strip():
            raise LabAgentError(f'Query parameter "{key}" is required')
        raw = values[0].strip()
        try:
            if raw.lower().lstrip("+-").startswith("0x"):
                return int(raw, 16)
            return int(raw, 10)
        except (TypeError, ValueError) as exc:
            raise LabAgentError(
                f'Query parameter "{key}" must be an integer (decimal or 0x-hex)'
            ) from exc

    def _query_float(self, query: Dict[str, list], key: str, default: float) -> float:
        values = query.get(key)
        if not values:
            return default
        try:
            return float(values[0])
        except (TypeError, ValueError) as exc:
            raise LabAgentError(f'Query parameter "{key}" must be a number') from exc


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, manager, logger):
        self.manager = manager
        self.logger = logger
        super().__init__(address, handler)


class LabAgentServer:
    """Runs the control plane until stopped."""

    def __init__(
        self,
        manager: LabManager,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.manager = manager
        self.logger = logger or logging.getLogger("lab_agent")
        settings = manager.config.settings
        self._server = _Server(
            (settings.host, settings.port), _Handler, manager, self.logger
        )
        self._thread: Optional[threading.Thread] = None

    @property
    def address(self) -> Tuple[str, int]:
        return self._server.server_address[0], self._server.server_address[1]

    @property
    def url(self) -> str:
        host, port = self.address
        return f"http://{host}:{port}"

    def serve_forever(self) -> None:
        """Block, serving requests until :meth:`stop` is called."""
        self.logger.info(
            "Lab Agent serving %d testbed(s) on %s",
            len(self.manager.ids()),
            self.url,
        )
        try:
            self._server.serve_forever()
        finally:
            self.manager.shutdown()

    def start_background(self) -> None:
        """Serve on a daemon thread (used by tests and embedding)."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="lab-agent", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
