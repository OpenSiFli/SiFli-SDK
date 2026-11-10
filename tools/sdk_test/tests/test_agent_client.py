# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.agent_client`` and the backend selection rules.

The HTTP client is exercised against a stub server rather than a real Lab Agent
here; the cross-process contract is covered by ``test_end_to_end.py``. What this
file pins is the client's own behaviour: how it shapes requests, how it turns
the agent's error bodies into messages, and -- most importantly -- that the
console cursor it owns keeps reads lossless across many calls.
"""

from __future__ import annotations

import base64
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.agent_client import LabAgentClient
from sdk_test.agent_client import RemoteConsole
from sdk_test.agent_client import require_testbed
from sdk_test.backends import create_backend
from sdk_test.backends import is_host_platform
from sdk_test.backends import select_backend_name
from sdk_test.backends.pc_sim import PcSimBackend
from sdk_test.backends.qemu import QemuBackend
from sdk_test.errors import BackendError
from sdk_test.errors import LabAgentError
from sdk_test.version import PROTOCOL


class _StubHandler(BaseHTTPRequestHandler):
    """Records requests and replies from a scripted table."""

    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    def _record(self, body=None):
        self.server.requests.append(
            {"method": self.command, "path": self.path, "body": body}
        )

    def _reply(self):
        rule = self.server.routes.get(self.path.split("?")[0])
        if rule is None:
            payload = json.dumps({"error": "no such route"}).encode()
            self.send_response(404)
        else:
            status, body = rule
            payload = json.dumps(body).encode()
            self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        self._record()
        self._reply()

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        self.server.raw.append(raw)
        try:
            body = json.loads(raw) if raw else None
        except ValueError:
            # A firmware upload is a tar, not JSON. Reading it as one would be
            # the bug this stub exists to let a test catch.
            body = None
        self._record(body)
        self._reply()


class ClientTestCase(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _StubHandler)
        self.server.requests = []
        self.server.raw = []
        self.server.routes = {}
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        host, port = self.server.server_address
        self.client = LabAgentClient(f"http://{host}:{port}")

    def route(self, path: str, body, status: int = 200):
        self.server.routes[path] = (status, body)


class TestRequestShaping(ClientTestCase):
    def test_allocate_sends_fixtures_not_ports(self):
        self.route("/allocate", {"testbed_id": "tb_01", "protocol": PROTOCOL})
        self.client.allocate(
            platform="board_a", fixtures=["flash"], holder="job-1"
        )

        body = self.server.requests[-1]["body"]
        self.assertEqual(body["platform"], "board_a")
        self.assertEqual(body["fixtures"], ["flash"])
        self.assertEqual(body["holder"], "job-1")
        # The design's central rule: no physical detail leaves this layer.
        self.assertNotIn("port", body)

    def test_console_read_encodes_cursor_and_timeout_as_query_parameters(self):
        self.route("/testbed/tb_01/console/read", {"data": "hi", "cursor": 2})
        data, cursor = self.client.console_read("tb_01", 0, 1.5)

        path = self.server.requests[-1]["path"]
        self.assertIn("cursor=0", path)
        self.assertIn("timeout=1.5", path)
        self.assertEqual((data, cursor), ("hi", 2))

    def test_console_write_sends_the_data_verbatim(self):
        self.route("/testbed/tb_01/console/write", {"written": 5})
        self.client.console_write("tb_01", "unity_run\r\n")
        self.assertEqual(self.server.requests[-1]["body"]["data"], "unity_run\r\n")

    def test_prepare_names_the_package_and_can_suppress_the_reset(self):
        # The board backend flashes with the reset suppressed so the console can
        # be opened first; the request must actually say so.
        self.route("/testbed/tb_01/prepare", {"prepared": True})
        self.client.prepare("tb_01", firmware="abc123", reset=False)
        body = self.server.requests[-1]["body"]
        self.assertEqual(body["firmware"], "abc123")
        self.assertFalse(body["reset"])
        # A path from this machine would mean the two sides share a filesystem,
        # which is exactly what sending a package removes the need for.
        self.assertNotIn("build_dir", body)

    def test_a_firmware_upload_is_a_bare_tar_body(self):
        self.route("/testbed/tb_01/firmware", {"token": "abc123", "platform": "p"})
        self.client.upload_firmware("tb_01", b"a tar")
        requests = self.server.requests[-1]
        self.assertEqual(requests["method"], "POST")
        self.assertEqual(self.server.raw[-1], b"a tar")


class TestMemoryRequests(ClientTestCase):
    """The probe read's request shape and its failure modes.

    A wrong address or a short dump must not reach a suite as data: this layer
    is the last place that can tell "the agent's answer is unusable" apart from
    "the board's memory says this".
    """

    def test_a_read_asks_for_a_hex_address_and_returns_the_bytes(self):
        payload = bytes(range(8))
        self.route(
            "/testbed/tb_01/memory",
            {
                "address": 0x2005A7C0,
                "length": 8,
                "data": base64.b64encode(payload).decode("ascii"),
            },
        )
        self.assertEqual(
            self.client.read_memory("tb_01", 0x2005A7C0, 8), payload
        )
        path = self.server.requests[-1]["path"]
        self.assertIn("address=0x2005a7c0", path)
        self.assertIn("length=8", path)

    def test_a_missing_data_field_is_an_error(self):
        self.route("/testbed/tb_01/memory", {"address": 0, "length": 4})
        with self.assertRaises(LabAgentError) as caught:
            self.client.read_memory("tb_01", 0, 4)
        self.assertIn("no memory data", str(caught.exception))

    def test_undecodable_data_is_an_error(self):
        self.route("/testbed/tb_01/memory", {"data": "not base64!!"})
        with self.assertRaises(LabAgentError) as caught:
            self.client.read_memory("tb_01", 0, 4)
        self.assertIn("undecodable", str(caught.exception))

    def test_a_short_dump_is_an_error(self):
        self.route(
            "/testbed/tb_01/memory",
            {"data": base64.b64encode(b"abcd").decode("ascii")},
        )
        with self.assertRaises(LabAgentError) as caught:
            self.client.read_memory("tb_01", 0, 8)
        self.assertIn("4 of 8", str(caught.exception))


class TestInstrumentStreams(ClientTestCase):
    """``stream`` selects an instrument console; the default stays the DUT.

    Instrument hardware is a second stream on the same testbed, so every
    existing call -- and every URL a running agent already understands -- must
    keep meaning the DUT when the argument is not given.
    """

    def test_the_default_console_urls_carry_no_stream(self):
        self.route("/testbed/tb_01/console/open", {"console_open": True})
        self.client.console_open("tb_01")
        self.assertEqual(
            self.server.requests[-1]["path"], "/testbed/tb_01/console/open"
        )

        self.route("/testbed/tb_01/console/write", {"written": 1})
        self.client.console_write("tb_01", "x")
        self.assertEqual(
            self.server.requests[-1]["path"], "/testbed/tb_01/console/write"
        )

    def test_an_instrument_stream_is_named_on_every_console_call(self):
        self.route("/testbed/tb_01/console/open", {"console_open": True})
        self.client.console_open("tb_01", stream="ble_peer")
        self.assertEqual(
            self.server.requests[-1]["path"],
            "/testbed/tb_01/console/open?stream=ble_peer",
        )

        self.route("/testbed/tb_01/console/read", {"data": "", "cursor": 0})
        self.client.console_read("tb_01", 0, 0.1, stream="ble_peer")
        self.assertIn("stream=ble_peer", self.server.requests[-1]["path"])

        self.route("/testbed/tb_01/console/write", {"written": 1})
        self.client.console_write("tb_01", "x", stream="ble_peer")
        self.assertIn("stream=ble_peer", self.server.requests[-1]["path"])

        self.route("/testbed/tb_01/console/close", {"console_open": False})
        self.client.console_close("tb_01", stream="ble_peer")
        self.assertIn("stream=ble_peer", self.server.requests[-1]["path"])

    def test_a_remote_console_keeps_its_stream_and_cursor(self):
        self.route("/testbed/tb_01/console/read", {"data": "ab", "cursor": 2})
        console = self.client.console_for("tb_01", stream="ble_peer")
        self.assertEqual(console.read(0.1), "ab")
        self.assertIn("stream=ble_peer", self.server.requests[-1]["path"])
        self.assertEqual(console.cursor, 2)

    def test_opening_and_closing_a_stream_reach_the_agent(self):
        self.route("/testbed/tb_01/console/open", {"console_open": True})
        self.route("/testbed/tb_01/console/close", {"console_open": False})
        console = self.client.console_for("tb_01", stream="ble_peer")
        console.open()
        self.assertEqual(
            self.server.requests[-1]["path"],
            "/testbed/tb_01/console/open?stream=ble_peer",
        )
        console.close()
        self.assertEqual(
            self.server.requests[-1]["path"],
            "/testbed/tb_01/console/close?stream=ble_peer",
        )


class TestInstrumentTranscript(ClientTestCase):
    """A transcript that could not be collected is not "not opened"."""

    class _Console:
        def __init__(self, error):
            self.error = error

        def for_stream(self, stream):
            return self

        def read_from(self, cursor, timeout):
            raise self.error

    def session(self, error):
        from sdk_test.backends.board import BoardSession

        return BoardSession(
            client=self.client,
            testbed_id="tb_01",
            console=self._Console(error),
            platform="board_a",
            holder=None,
        )

    def test_a_console_the_agent_says_is_not_open_is_none(self):
        self.assertIsNone(
            self.session(LabAgentError("console is not open", status=409))
            .instrument_transcript("ble_peer")
        )

    def test_any_other_failure_is_raised_for_the_caller_to_record(self):
        with self.assertRaises(LabAgentError) as caught:
            self.session(LabAgentError("agent went away", status=500)).instrument_transcript(
                "ble_peer"
            )
        self.assertEqual(caught.exception.status, 500)

    def test_an_unreachable_agent_is_raised_too(self):
        with self.assertRaises(LabAgentError):
            self.session(LabAgentError("cannot reach")).instrument_transcript("ble_peer")


class TestProtocolGate(ClientTestCase):
    """An agent that speaks a different version of this interface is refused.

    Both directions matter: a *newer* agent sends fields this checkout cannot
    read, and an agent from before the version was introduced sends none at all.
    Either would otherwise surface as a wrong result several calls later.
    """

    def test_an_agent_from_before_the_protocol_existed_is_refused(self):
        self.route("/allocate", {"testbed_id": "tb_01"})
        with self.assertRaises(LabAgentError) as caught:
            self.client.allocate(platform="board_a")
        self.assertIn("did not say which protocol", str(caught.exception))

    def test_a_different_protocol_is_refused_and_both_are_named(self):
        self.route("/allocate", {"testbed_id": "tb_01", "protocol": PROTOCOL + 1})
        with self.assertRaises(LabAgentError) as caught:
            self.client.allocate(platform="board_a")
        message = str(caught.exception)
        self.assertIn(str(PROTOCOL + 1), message)
        self.assertIn(str(PROTOCOL), message)

    def test_a_bool_is_not_a_protocol_number(self):
        # `True == 1` in Python, so a `protocol: true` would pass a naive check
        # against protocol 1 -- a version gate that opens for the wrong reason.
        self.route("/allocate", {"testbed_id": "tb_01", "protocol": True})
        with self.assertRaises(LabAgentError):
            self.client.allocate(platform="board_a")

    def test_the_matching_protocol_is_accepted(self):
        self.route("/allocate", {"testbed_id": "tb_01", "protocol": PROTOCOL})
        self.assertEqual(
            self.client.allocate(platform="board_a")["testbed_id"], "tb_01"
        )


class TestErrorHandling(ClientTestCase):
    def test_the_agents_error_message_is_surfaced(self):
        self.route("/allocate", {"error": "testbed tb_01 is busy"}, status=409)
        with self.assertRaises(LabAgentError) as caught:
            self.client.allocate(testbed_id="tb_01")
        message = str(caught.exception)
        self.assertIn("busy", message)
        self.assertIn("409", message)

    def test_an_http_error_carries_its_status(self):
        # Callers that must tell "the console is not open" (409) from "the
        # answer never arrived" need the number, not just the text.
        self.route(
            "/testbed/tb_01/console/read",
            {"error": "console is not open"},
            status=409,
        )
        with self.assertRaises(LabAgentError) as caught:
            self.client.console_read("tb_01", 0, 0.1, stream="ble_peer")
        self.assertEqual(caught.exception.status, 409)

    def test_an_unreachable_agent_says_how_to_start_it(self):
        client = LabAgentClient("http://127.0.0.1:1")
        with self.assertRaises(LabAgentError) as caught:
            client.health()
        self.assertIn("Lab Agent", str(caught.exception))
        self.assertIn("runner.py", str(caught.exception))
        self.assertIsNone(caught.exception.status)

    def test_a_json_response_of_the_wrong_shape_is_rejected(self):
        # Valid JSON, wrong shape -- an older or foreign agent. Rejecting it here
        # keeps the failure at the boundary instead of surfacing as an
        # AttributeError somewhere in a caller that assumed a dict.
        self.server.routes["/health"] = (200, "not a dict")
        with self.assertRaises(LabAgentError) as caught:
            self.client.health()
        self.assertIn("instead of an object", str(caught.exception))

    def test_require_testbed_rejects_an_allocation_without_one(self):
        with self.assertRaises(Exception) as caught:
            require_testbed({})
        self.assertIn("no testbed id", str(caught.exception))

    def test_require_testbed_extracts_the_id(self):
        self.assertEqual(require_testbed({"testbed_id": "tb_9"}), "tb_9")


class TestRemoteConsoleCursor(ClientTestCase):
    def test_reads_advance_the_cursor_and_never_repeat_text(self):
        # A device that emits a chunk per read: the console must return each
        # chunk exactly once, in order, with no gap and no overlap.
        chunks = ["one", "two", "three", "four"]
        state = {"index": 0}

        def handler(self):
            index = state["index"]
            if index < len(chunks):
                state["index"] += 1
                body = {"data": chunks[index], "cursor": sum(len(c) for c in chunks[: index + 1])}
            else:
                body = {"data": "", "cursor": sum(len(c) for c in chunks)}
            payload = json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        class ReadHandler(_StubHandler):
            do_GET = handler

        server = ThreadingHTTPServer(("127.0.0.1", 0), ReadHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)

        host, port = server.server_address
        console = RemoteConsole(LabAgentClient(f"http://{host}:{port}"), "tb_01")

        received = ""
        for _ in range(len(chunks) + 2):
            received += console.read(0.05)

        self.assertEqual(received, "".join(chunks))
        self.assertEqual(console.cursor, len("".join(chunks)))

    def test_send_line_appends_a_crlf(self):
        self.route("/testbed/tb_01/console/write", {"written": 1})
        RemoteConsole(self.client, "tb_01").send_line("unity_run")
        self.assertEqual(self.server.requests[-1]["body"]["data"], "unity_run\r\n")

    def test_skip_to_end_advances_past_buffered_output(self):
        self.route("/testbed/tb_01/console/read", {"data": "stale", "cursor": 5})
        console = RemoteConsole(self.client, "tb_01")
        self.assertEqual(console.skip_to_end(), 5)
        self.assertEqual(console.cursor, 5)

    def test_drain_discards_without_advancing_past_the_timeout(self):
        self.route("/testbed/tb_01/console/read", {"data": "", "cursor": 3})
        console = RemoteConsole(self.client, "tb_01")
        self.assertEqual(console.drain(), "")

    def test_reopening_a_stream_starts_reading_from_zero_again(self):
        # The agent drops the stream on close, so the next open is a fresh one
        # whose cursor starts at 0. Keeping the old cursor would skip everything
        # the new stream already held.
        self.route("/testbed/tb_01/console/read", {"data": "ab", "cursor": 2})
        console = RemoteConsole(self.client, "tb_01")
        self.assertEqual(console.read(0.1), "ab")
        self.assertEqual(console.cursor, 2)

        self.route("/testbed/tb_01/console/close", {"console_open": False})
        console.close()
        self.assertEqual(console.cursor, 0)

        self.route("/testbed/tb_01/console/open", {"console_open": True})
        console.open()
        console.read(0.1)
        self.assertIn("cursor=0", self.server.requests[-1]["path"])


class TestCanonicalBoardName(SdkTestCase):
    """The build directory is named after the canonical board, not the platform.

    ``--board=sf32lb52-lcd_n16r8`` compiles into ``build_sf32lb52-lcd_n16r8_hcpu``.
    Mirrored from building.py's GetBoardName; getting it wrong is silent, because
    the build succeeds and only the later lookup of its artifacts fails.
    """

    def test_a_platform_without_a_core_suffix_means_hcpu(self):
        from sdk_test.runner import canonical_board_name

        self.assertEqual(
            canonical_board_name("sf32lb52-lcd_n16r8"), "sf32lb52-lcd_n16r8_hcpu"
        )

    def test_an_explicit_core_suffix_is_preserved(self):
        from sdk_test.runner import canonical_board_name

        self.assertEqual(canonical_board_name("eh-lb563_lcpu"), "eh-lb563_lcpu")
        self.assertEqual(canonical_board_name("x_acpu"), "x_acpu")

    def test_the_host_carriers_are_canonicalised_too(self):
        from sdk_test.runner import canonical_board_name

        # `scons --board=pc` builds into build_pc_hcpu, which is what the
        # simulator backend has to look in.
        self.assertEqual(canonical_board_name("pc"), "pc_hcpu")
        self.assertEqual(canonical_board_name("qemu_sf32lb52x"), "qemu_sf32lb52x_hcpu")


class TestBackendSelection(SdkTestCase):
    def test_a_real_board_uses_the_lab_agent_backend(self):
        self.assertEqual(select_backend_name("sf32lb52-lcd_n16r8"), "board")
        self.assertEqual(select_backend_name("eh-lb563"), "board")
        self.assertFalse(is_host_platform("eh-lb563"))

    def test_the_simulator_is_a_host_platform(self):
        self.assertEqual(select_backend_name("pc_hcpu"), "pc_sim")
        self.assertEqual(select_backend_name("pc"), "pc_sim")
        self.assertTrue(is_host_platform("pc_hcpu"))

    def test_qemu_is_a_host_platform(self):
        self.assertEqual(select_backend_name("qemu_sf32lb52x"), "qemu")
        self.assertTrue(is_host_platform("qemu_sf32lb52x"))

    def test_an_empty_platform_is_refused(self):
        with self.assertRaises(BackendError) as caught:
            select_backend_name("")
        self.assertIn("--platform", str(caught.exception))

    def test_a_board_needs_a_client_and_a_host_backend_does_not(self):
        with self.assertRaises(BackendError) as caught:
            create_backend("board")
        self.assertIn("Lab Agent client", str(caught.exception))
        # Host backends must be constructible with nothing at all: that is what
        # "needs no Lab Agent" means in practice.
        self.assertIsInstance(create_backend("pc_sim"), PcSimBackend)
        self.assertIsInstance(create_backend("qemu"), QemuBackend)

    def test_an_unknown_backend_lists_the_known_ones(self):
        with self.assertRaises(BackendError) as caught:
            create_backend("telepathy")
        self.assertIn("board", str(caught.exception))

    def test_the_pc_simulator_is_still_a_stub(self):
        with self.assertRaises(BackendError) as caught:
            create_backend("pc_sim").open(_instance(), "pc_hcpu")
        message = str(caught.exception)
        self.assertIn("not implemented", message)
        self.assertIn("milestone", message)

    def test_the_qemu_backend_is_implemented(self):
        # It got past the stub: the first thing it asks for is a place to find
        # the ELF, which only a built project can supply.
        with self.assertRaises(BackendError) as caught:
            create_backend("qemu").open(_instance(), "qemu_sf32lb52x")
        self.assertIn("build directory", str(caught.exception))
        self.assertNotIn("not implemented", str(caught.exception))


def _instance():
    from sdk_test.model import TestInstance

    return TestInstance(
        id="t", testsuite_root="/s", testcase_dir="/s/d", tests_key="t"
    )


if __name__ == "__main__":
    unittest.main()
