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
    POST /testbed/{id}/prepare         {build_dir, reset}
    POST /testbed/{id}/reset
    POST /testbed/{id}/flash           {build_dir}
    POST /testbed/{id}/console/open
    POST /testbed/{id}/console/close
    GET  /testbed/{id}/console/read?cursor=N&timeout=SECONDS
    POST /testbed/{id}/console/write   {data}
"""

from __future__ import annotations

import json
import logging
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from typing import Any
from typing import Dict
from typing import Optional
from typing import Tuple
from urllib.parse import parse_qs
from urllib.parse import urlparse

from .errors import AllocationError
from .errors import ConfigError
from .errors import LabAgentError
from .errors import TestbedError
from .errors import TransportError
from .testbed import LabManager
from .testbed import allocate_request_from_mapping

#: Default value for an omitted ``timeout`` on a console read, in seconds.
DEFAULT_READ_TIMEOUT = 1.0

#: Upper bound on a single console read, so a client cannot pin a thread forever.
MAX_READ_TIMEOUT = 60.0

_ERROR_STATUS = {
    AllocationError: 409,
    TestbedError: 409,
    TransportError: 502,
    ConfigError: 500,
    # Least specific, so it must be checked last: a plain LabAgentError is a
    # malformed request (bad JSON, a body that is not an object), which is the
    # client's fault, not the agent's. Reporting it as 500 would make a caller's
    # bad request look like a crashed lab and bury it in a stack trace.
    LabAgentError: 400,
}


def _status_for(error: Exception) -> int:
    for error_type, status in _ERROR_STATUS.items():
        if isinstance(error, error_type):
            return status
    return 500


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
        # buries the agent's own logging. Route it through the logger instead.
        self.logger.debug("%s - %s", self.address_string(), fmt % args)

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
        query = parse_qs(parsed.query)

        try:
            status, payload = self._dispatch(method, segments, query)
        except Exception as exc:  # noqa: BLE001 - every failure becomes a response
            status = _status_for(exc)
            payload = {"error": str(exc)}
            if status >= 500:
                self.logger.exception("Request failed: %s %s", method, self.path)
            else:
                self.logger.info("%s %s -> %s: %s", method, self.path, status, exc)

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

        if name == "prepare":
            if method != "POST":
                return 405, {"error": "POST only"}
            body = self._read_body()
            runtime.prepare(
                build_dir=body.get("build_dir"),
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
            build_dir = self._read_body().get("build_dir")
            if not build_dir:
                return 400, {"error": '"build_dir" is required'}
            runtime.flash(str(build_dir))
            return 200, {"flashed": True, "testbed_id": testbed_id}

        if name == "console" and action[1:]:
            return self._dispatch_console(method, runtime, testbed_id, action[1], query)

        return 404, {"error": f"No such testbed action: {name}"}

    def _dispatch_console(
        self, method: str, runtime, testbed_id: str, action: str, query: Dict[str, list]
    ) -> Tuple[int, Any]:
        if action == "open":
            if method != "POST":
                return 405, {"error": "POST only"}
            runtime.console_open()
            return 200, {"console_open": True, "testbed_id": testbed_id}

        if action == "close":
            if method != "POST":
                return 405, {"error": "POST only"}
            runtime.console_close()
            return 200, {"console_open": False, "testbed_id": testbed_id}

        if action == "read":
            if method != "GET":
                return 405, {"error": "GET only"}
            cursor = self._query_int(query, "cursor", 0)
            timeout = self._query_float(query, "timeout", DEFAULT_READ_TIMEOUT)
            if timeout < 0:
                return 400, {"error": '"timeout" must not be negative'}
            timeout = min(timeout, MAX_READ_TIMEOUT)
            data, new_cursor = runtime.console_read(cursor, timeout)
            return 200, {"data": data, "cursor": new_cursor}

        if action == "write":
            if method != "POST":
                return 405, {"error": "POST only"}
            data = self._read_body().get("data")
            if data is None:
                return 400, {"error": '"data" is required'}
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
