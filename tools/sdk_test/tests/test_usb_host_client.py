# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the runner-side ``usb_host`` client and its pytest fixture.

The agent is replaced by a scripted HTTP stub, as in ``test_agent_client``:
what is pinned here is the client's own contract -- how it decodes the device
JSON, how it maps the agent's error bodies onto the exception types a suite
catches, and that ``read_exact`` reassembles a frame split across reads.

The last class runs a real pytest process: the fixture must turn a missing
declaration into an error, never a skip. A skip exits 0 and the framework
records the exit code as the verdict, so a skip would be a false pass.
"""

from __future__ import annotations

import base64
import json
import os
import subprocess
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer

from _support import SdkTestCase
from _support import TOOLS_DIR

from sdk_test.agent_client import LabAgentClient
from sdk_test.errors import LabAgentError
from sdk_test.harness.pytest import ENV_AGENT_URL
from sdk_test.harness.pytest import ENV_FIXTURES
from sdk_test.harness.pytest import ENV_TESTBED_ID
from sdk_test.instruments import registry
from sdk_test.usb_host import UsbConflict
from sdk_test.usb_host import UsbDenied
from sdk_test.usb_host import UsbHost
from sdk_test.usb_host import UsbHostError
from sdk_test.usb_host import UsbHostTimeout
from sdk_test.usb_host import UsbHostUnavailable
from sdk_test.usb_host import UsbNotFound
from sdk_test.usb_host import UsbSerial
from sdk_test.usb_host import UsbVolume
from sdk_test.usb_host import VolumeInfo

try:
    import pytest as _pytest  # noqa: F401

    HAVE_PYTEST = True
except ImportError:  # pragma: no cover - depends on the machine
    HAVE_PYTEST = False


#: The example from the design document, trimmed to what a test reads.
DEVICE = {
    "key": "USB\\VID_38F4&PID_FFFF\\2022123456",
    "instance_id": "USB\\VID_38F4&PID_FFFF\\2022123456",
    "vid": 14580,
    "pid": 65535,
    "serial": "2022123456",
    "product": "USB Serial Device",
    "manufacturer": None,
    "device_class": "cdc",
    "classes": ["cdc"],
    "interfaces": [
        {"number": 0, "class": 2, "subclass": 2, "protocol": 1, "name": "cdc_acm"},
        {"number": 1, "class": 10, "subclass": 0, "protocol": 0, "name": "cdc_data"},
    ],
    "com_ports": [{"port": "COM12", "driver": "usbser"}],
    "volumes": [],
    "hid": [],
    "audio": None,
    "speed": "full",
    "bus_port": 1,
    "present": True,
}


class _StubHandler(BaseHTTPRequestHandler):
    """Repies from a scripted table, popping a queue so reads can vary."""

    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    def _record(self, body=None):
        self.server.requests.append(
            {"method": self.command, "path": self.path, "body": body}
        )

    def _reply(self):
        key = self.path.split("?")[0]
        rules = self.server.routes.get(key)
        if not rules:
            status, body = 404, {"error": "no such route"}
        elif len(rules) == 1:
            status, body = rules[0]
        else:
            status, body = rules.pop(0)
        payload = json.dumps(body).encode("utf-8")
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
        try:
            body = json.loads(raw) if raw else None
        except ValueError:
            body = None
        self._record(body)
        self._reply()


class ClientTestCase(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _StubHandler)
        self.server.requests = []
        self.server.routes = {}
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        host, port = self.server.server_address
        self.client = LabAgentClient(f"http://{host}:{port}")
        self.usb = UsbHost(self.client, "tb_01")

    def route(self, path: str, body, status: int = 200):
        self.server.routes.setdefault(path, []).append((status, body))

    def route_once(self, path: str, body, status: int = 200):
        self.server.routes[path] = [(status, body)]

    @staticmethod
    def encoded(data: bytes) -> str:
        return base64.b64encode(data).decode("ascii")


class TestDeviceParsing(ClientTestCase):
    def test_the_device_json_becomes_typed_objects(self):
        self.route(
            "/testbed/tb_01/usb/devices",
            {"captured_at": "t", "devices": [DEVICE]},
        )

        devices = self.usb.devices()

        self.assertEqual(len(devices), 1)
        device = devices[0]
        self.assertEqual(device.key, DEVICE["key"])
        self.assertEqual((device.vid, device.pid), (14580, 65535))
        self.assertEqual(device.serial, "2022123456")
        self.assertEqual(device.classes, ("cdc",))
        self.assertEqual([i.interface_class for i in device.interfaces], [2, 10])
        self.assertEqual(device.com_ports[0].port, "COM12")
        self.assertTrue(device.present)

    def test_a_device_without_a_serial_keeps_none(self):
        payload = dict(DEVICE, serial=None, product=None)
        self.route("/testbed/tb_01/usb/devices", {"devices": [payload]})
        device = self.usb.devices()[0]
        self.assertIsNone(device.serial)
        self.assertIsNone(device.product)

    def test_a_response_without_a_device_list_is_an_error(self):
        self.route("/testbed/tb_01/usb/devices", {"captured_at": "t"})
        with self.assertRaises(UsbHostError):
            self.usb.devices()


class TestCapabilities(ClientTestCase):
    def test_a_supported_backend_lists_its_features(self):
        self.route(
            "/testbed/tb_01/usb/capabilities",
            {
                "supported": True,
                "platform": "win32",
                "features": ["enumerate", "serial"],
                "host": {"name": "local", "machine": "bench"},
                "reason": None,
            },
        )
        capabilities = self.usb.capabilities()
        self.assertTrue(capabilities.supported)
        self.assertEqual(capabilities.features, ("enumerate", "serial"))
        self.usb.require("enumerate")

    def test_unsupported_raises_rather_than_returning_a_flag(self):
        self.route(
            "/testbed/tb_01/usb/capabilities",
            {"supported": False, "reason": "no SetupAPI on this platform"},
        )
        with self.assertRaises(UsbHostUnavailable) as caught:
            self.usb.require_supported()
        self.assertIn("SetupAPI", str(caught.exception))

    def test_an_agent_older_than_the_routes_is_unavailable_not_notfound(self):
        # 404 with no `code` is the old-agent case; reading it as "no such
        # device" would send the reader looking at the board.
        self.route(
            "/testbed/tb_01/usb/capabilities",
            {"error": "no such route"},
            status=404,
        )
        with self.assertRaises(UsbHostUnavailable):
            self.usb.capabilities()

    def test_a_missing_feature_names_what_the_agent_offers(self):
        self.route(
            "/testbed/tb_01/usb/capabilities",
            {"supported": True, "features": ["enumerate"]},
        )
        with self.assertRaises(UsbHostUnavailable) as caught:
            self.usb.require("serial")
        self.assertIn("serial", str(caught.exception))
        self.assertIn("enumerate", str(caught.exception))


class TestWait(ClientTestCase):
    def test_a_matching_device_is_returned(self):
        self.route(
            "/testbed/tb_01/usb/wait",
            {"device": DEVICE, "elapsed": 1.2},
        )
        device = self.usb.wait_for(serial="2022123456", timeout=5.0)
        self.assertEqual(device.key, DEVICE["key"])

        body = self.server.requests[-1]["body"]
        self.assertEqual(body["match"], {"serial": "2022123456"})
        self.assertEqual(body["since"], [])
        self.assertEqual(body["timeout"], 5.0)
        self.assertEqual(body["interval"], 0.25)

    def test_a_timeout_is_the_typed_exception_and_keeps_the_candidates(self):
        self.route(
            "/testbed/tb_01/usb/wait",
            {
                "error": "no device matched within 15s",
                "code": "timeout",
                "details": {"candidates": [DEVICE["key"]]},
            },
            status=408,
        )
        with self.assertRaises(UsbHostTimeout) as caught:
            self.usb.wait_for(serial="nope", timeout=15.0)
        self.assertIsInstance(caught.exception, TimeoutError)
        # The candidates explain a near miss, so they must survive the mapping.
        self.assertIn("candidates", str(caught.exception))
        self.assertIn("2022123456", str(caught.exception))

    def test_criteria_can_be_a_mapping(self):
        # `class` is a reserved word, so it cannot be a keyword argument; the
        # mapping form is the only way to spell that criterion.
        self.route("/testbed/tb_01/usb/wait", {"device": DEVICE})
        self.usb.wait_for(match={"class": 10}, timeout=1.0)
        self.assertEqual(self.server.requests[-1]["body"]["match"], {"class": 10})

    def test_a_mapping_and_keywords_are_merged(self):
        self.route("/testbed/tb_01/usb/wait", {"device": DEVICE})
        self.usb.wait_for(match={"vid": 14580}, serial="2022123456", timeout=1.0)
        self.assertEqual(
            self.server.requests[-1]["body"]["match"],
            {"vid": 14580, "serial": "2022123456"},
        )

    def test_a_key_given_twice_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            self.usb.wait_for(match={"serial": "a"}, serial="b")
        self.assertIn("serial", str(caught.exception))
        # Refused before any request: one of the two would silently win.
        self.assertEqual(self.server.requests, [])

    def test_since_is_forwarded_unchanged(self):
        self.route("/testbed/tb_01/usb/wait", {"device": DEVICE})
        self.usb.wait_for(since=["USB\\A", "USB\\B"], serial="x", timeout=1.0)
        self.assertEqual(
            self.server.requests[-1]["body"]["since"], ["USB\\A", "USB\\B"]
        )


class TestSerial(ClientTestCase):
    def open(self, com_port="COM12"):
        self.route(
            "/testbed/tb_01/usb/serial/open",
            {
                "token": "tok-1",
                "port": com_port,
                "device_key": DEVICE["key"],
                "baud": 115200,
                "dtr": False,
                "rts": False,
            },
        )
        device = self._device()
        return self.usb.serial(device)

    def _device(self):
        from sdk_test.usb_host import UsbDevice

        return UsbDevice.from_json(DEVICE)

    def test_opening_names_the_device_and_the_port(self):
        serial = self.open()
        body = self.server.requests[-1]["body"]
        self.assertEqual(body["device_key"], DEVICE["key"])
        self.assertEqual(body["com_port"], "COM12")
        self.assertEqual(body["baud"], 115200)
        self.assertEqual(serial.port, "COM12")
        self.assertEqual(serial.token, "tok-1")

    def test_reading_decodes_base64_and_advances_the_cursor(self):
        serial = self.open()
        payload = b"\x01\x02hello"
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": self.encoded(payload), "cursor": len(payload), "closed": False},
        )

        self.assertEqual(serial.read(0.1), payload)
        self.assertEqual(serial.cursor, len(payload))
        # The next read resumes where this one stopped, not at zero.
        serial.read(0.1)
        self.assertIn(f"cursor={len(payload)}", self.server.requests[-1]["path"])

    def test_read_exact_reassembles_bytes_split_across_reads(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": self.encoded(b"abc"), "cursor": 3, "closed": False},
        )
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": self.encoded(b"defg"), "cursor": 7, "closed": False},
        )

        self.assertEqual(serial.read_exact(7, timeout=2.0), b"abcdefg")

    def test_read_exact_keeps_bytes_a_read_overshot(self):
        # One wire read can hand over more than the caller asked for. The
        # agent's cursor has moved past the surplus, so returning it would both
        # break "exactly size" and lose it for the next assertion.
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {
                "data": self.encoded(b"0123456789"),
                "cursor": 10,
                "closed": False,
            },
        )

        self.assertEqual(serial.read_exact(3), b"012")
        self.assertEqual(serial.read(0.1), b"3456789")
        self.assertEqual(self.read_requests(), 1)

    def test_two_read_exacts_share_one_wire_read(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {
                "data": self.encoded(b"0123456789"),
                "cursor": 10,
                "closed": False,
            },
        )

        self.assertEqual(serial.read_exact(4), b"0123")
        self.assertEqual(serial.read_exact(4), b"4567")
        self.assertEqual(serial.read(0.1), b"89")
        self.assertEqual(self.read_requests(), 1)

    def test_pending_bytes_survive_a_closed_port(self):
        # The same wire read can report the port closed and still carry bytes
        # the caller has not consumed. Those bytes are deliverable; the closed
        # flag must not discard them.
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {
                "data": self.encoded(b"0123456789"),
                "cursor": 10,
                "closed": True,
            },
        )

        self.assertEqual(serial.read_exact(3), b"012")
        self.assertEqual(serial.read_exact(2), b"34")
        self.assertEqual(self.read_requests(), 1)

    def test_a_closed_port_with_no_pending_still_raises(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": self.encoded(b"ab"), "cursor": 2, "closed": True},
        )
        with self.assertRaises(UsbConflict):
            serial.read_exact(3)

    def read_requests(self):
        return sum(
            1
            for request in self.server.requests
            if request["path"].split("?")[0]
            == "/testbed/tb_01/usb/serial/tok-1/read"
        )

    def test_read_exact_times_out_rather_than_returning_a_short_frame(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": self.encoded(b"abc"), "cursor": 3, "closed": False},
        )
        # Then the port goes quiet: the stub repeats this answer for every
        # later read, which is exactly a frame that never completes.
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"data": "", "cursor": 3, "closed": False},
        )

        with self.assertRaises(UsbHostTimeout) as caught:
            serial.read_exact(7, timeout=0.2)
        self.assertIn("3 of 7", str(caught.exception))

    def test_write_sends_base64_bytes(self):
        serial = self.open()
        self.route("/testbed/tb_01/usb/serial/tok-1/write", {"written": 7})
        self.assertEqual(serial.write("1234567"), 7)
        body = self.server.requests[-1]["body"]
        self.assertEqual(body["data_base64"], self.encoded(b"1234567"))

    def test_line_changes_send_and_read_back_both_signals(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/lines",
            {"dtr": True, "rts": False},
        )
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/lines",
            {"dtr": True, "rts": True},
        )

        serial.set_dtr(True)
        self.assertEqual(
            self.server.requests[-1]["body"], {"dtr": True, "rts": False}
        )
        self.assertTrue(serial.dtr)
        self.assertFalse(serial.rts)

        serial.set_rts(True)
        # The update carries the other line's current value, not a default.
        self.assertEqual(
            self.server.requests[-1]["body"], {"dtr": True, "rts": True}
        )
        self.assertTrue(serial.rts)

    def test_a_busy_port_is_a_conflict(self):
        self.route(
            "/testbed/tb_01/usb/serial/open",
            {"error": "COM12 is in use", "code": "port_busy"},
            status=409,
        )
        with self.assertRaises(UsbConflict) as caught:
            self.usb.serial(self._device())
        self.assertIn("in use", str(caught.exception))

    def test_an_excluded_port_is_denied(self):
        self.route(
            "/testbed/tb_01/usb/serial/open",
            {"error": "COM12 is the DUT console", "code": "forbidden_port"},
            status=409,
        )
        with self.assertRaises(UsbDenied):
            self.usb.serial(self._device())
        self.assertIsInstance(UsbDenied("x"), PermissionError)

    def test_a_device_without_a_com_port_is_not_found(self):
        from sdk_test.usb_host import UsbDevice

        payload = dict(DEVICE, com_ports=[])
        with self.assertRaises(UsbNotFound):
            self.usb.serial(UsbDevice.from_json(payload))

    def test_an_unknown_token_is_not_found(self):
        serial = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/read",
            {"error": "no such token", "code": "token_not_found"},
            status=404,
        )
        with self.assertRaises(UsbNotFound):
            serial.read(0.1)

    def test_closing_an_already_closed_session_is_not_an_error(self):
        # The second close is a distinct view of the same token, so it really
        # asks the agent; the local `_closed` flag must not hide that path.
        serial = self.open()
        self.route("/testbed/tb_01/usb/serial/tok-1/close", {"closed": True})
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/close",
            {"error": "no such token", "code": "token_not_found"},
            status=404,
        )

        serial.close()
        again = UsbSerial(
            client=self.client,
            testbed_id="tb_01",
            token="tok-1",
            port="COM12",
            device_key=DEVICE["key"],
            baud=115200,
            dtr=False,
            rts=False,
        )
        again.close()

        closes = [
            request
            for request in self.server.requests
            if request["path"] == "/testbed/tb_01/usb/serial/tok-1/close"
        ]
        self.assertEqual(len(closes), 2)

    def test_host_close_attempts_every_session_and_raises_the_first_error(self):
        first = self.open()
        second = self.open()
        self.route(
            "/testbed/tb_01/usb/serial/tok-1/close",
            {"error": "the port went away", "code": "port_closed"},
            status=409,
        )
        self.route("/testbed/tb_01/usb/serial/tok-1/close", {"closed": True})

        with self.assertRaises(UsbHostError):
            self.usb.close()

        closes = [
            request
            for request in self.server.requests
            if request["path"] == "/testbed/tb_01/usb/serial/tok-1/close"
        ]
        # Both sessions were attempted: leaking the second would hold COM12
        # against the next case.
        self.assertEqual(len(closes), 2)
        self.assertEqual(first.token, second.token)


class TestVolume(ClientTestCase):
    def test_a_matching_device_yields_its_own_volume(self):
        # Two volumes are mounted, and the other device's is listed first. A
        # missing filter would hand back drive F: and every later read would
        # look at the wrong filesystem.
        self.route(
            "/testbed/tb_01/usb/volumes",
            {
                "volumes": [
                    {
                        "device_key": "USB\\VID_0000&PID_0000\\OTHER",
                        "drive": "F:",
                        "label": "Other",
                        "filesystem": "FAT",
                        "size": 4096,
                        "removable": True,
                    },
                    {
                        "device_key": DEVICE["key"],
                        "drive": "E:",
                        "label": "SiFli MSC",
                        "filesystem": "FAT",
                        "size": 8192,
                        "removable": True,
                    },
                ]
            },
        )
        from sdk_test.usb_host import UsbDevice

        volume = self.usb.volume(UsbDevice.from_json(DEVICE), timeout=1.0)
        self.assertEqual((volume.drive, volume.label), ("E:", "SiFli MSC"))

    def test_a_volume_that_never_appears_times_out(self):
        self.route("/testbed/tb_01/usb/volumes", {"volumes": []})
        from sdk_test.usb_host import UsbDevice

        with self.assertRaises(UsbHostTimeout):
            self.usb.volume(UsbDevice.from_json(DEVICE), timeout=0.2)

    def volume(self):
        info = VolumeInfo(
            drive="E:", label="SiFli MSC", filesystem="FAT",
            size=8192, removable=True, device_key=DEVICE["key"],
        )
        return UsbVolume(self.client, "tb_01", info)

    def test_a_denied_path_is_a_permission_error(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"error": "relative paths only", "code": "path_denied"},
            status=400,
        )
        with self.assertRaises(UsbDenied):
            self.volume().read_text("../secret.txt")
        self.assertIsInstance(UsbDenied("x"), PermissionError)

    def test_a_missing_file_is_not_found_and_exists_returns_false(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"error": "no such file", "code": "file_not_found"},
            status=404,
        )
        volume = self.volume()
        with self.assertRaises(UsbNotFound):
            volume.read_text("ABSENT.TXT")
        self.assertFalse(volume.exists("ABSENT.TXT"))

    def test_the_port_is_requested_with_the_drive_and_path(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": self.encoded(b"SiFli"), "encoding": "base64", "size": 5},
        )
        self.assertEqual(self.volume().read_bytes("README.TXT"), b"SiFli")
        path = self.server.requests[-1]["path"]
        self.assertIn("device_key=", path)
        self.assertIn("path=README.TXT", path)
        self.assertIn("encoding=base64", path)

    def test_text_read_returns_the_agents_text(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": "hello\r\n", "encoding": "text"},
        )
        self.assertEqual(self.volume().read_text("README.TXT"), "hello\r\n")
        self.assertIn("encoding=text", self.server.requests[-1]["path"])

    def test_a_base64_answer_is_decoded_with_the_callers_charset(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": self.encoded("hé".encode("utf-8")), "encoding": "base64"},
        )
        self.assertEqual(self.volume().read_text("README.TXT", encoding="utf-8"), "hé")

    def test_non_ascii_base64_is_the_agents_fault(self):
        # `b64decode` rejects a non-ASCII string with ValueError before it
        # decodes anything; a suite catching only UsbHostError must still see
        # an agent failure rather than a bare ValueError.
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": "héllo", "encoding": "base64"},
        )
        with self.assertRaises(UsbHostError):
            self.volume().read_text("README.TXT")

    def test_bad_base64_is_the_agents_fault(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": "!!!not base64!!!", "encoding": "base64"},
        )
        with self.assertRaises(UsbHostError):
            self.volume().read_text("README.TXT")

    def test_a_charset_error_stays_a_local_decode_error(self):
        # The bytes are valid base64; only the caller's charset rejects them.
        # Wrapping that as an agent failure would blame the wrong side.
        self.route(
            "/testbed/tb_01/usb/files",
            {"data": self.encoded(b"\xff"), "encoding": "base64"},
        )
        with self.assertRaises(UnicodeDecodeError):
            self.volume().read_text("README.TXT", encoding="utf-8", errors="strict")


class TestAudioAndHid(ClientTestCase):
    def test_audio_endpoints_are_split_by_direction(self):
        self.route(
            "/testbed/tb_01/usb/audio",
            {
                "device_key": DEVICE["key"],
                "inputs": [{"interface": 2, "endpoint": "0x85", "direction": "in"}],
                "outputs": [{"interface": 3, "endpoint": "0x03", "direction": "out"}],
            },
        )
        from sdk_test.usb_host import UsbDevice

        endpoints = self.usb.audio(UsbDevice.from_json(DEVICE))
        self.assertEqual([e.endpoint for e in endpoints.inputs], ["0x85"])
        self.assertEqual([e.endpoint for e in endpoints.outputs], ["0x03"])

    def test_hid_interfaces_carry_usage(self):
        self.route(
            "/testbed/tb_01/usb/hid",
            {
                "device_key": DEVICE["key"],
                "interfaces": [
                    {
                        "interface": 3,
                        "usage_page": 1,
                        "usage": 6,
                        "protocol": 1,
                        "report_length": 8,
                        "product": "SiFli",
                    }
                ],
            },
        )
        from sdk_test.usb_host import UsbDevice

        interfaces = self.usb.hid(UsbDevice.from_json(DEVICE))
        self.assertEqual((interfaces[0].usage_page, interfaces[0].usage), (1, 6))
        self.assertEqual(interfaces[0].report_length, 8)


class TestErrorBody(ClientTestCase):
    def test_the_code_and_details_survive_at_the_client_layer(self):
        self.route(
            "/testbed/tb_01/usb/files",
            {
                "error": "path escapes the volume",
                "code": "path_denied",
                "details": {"path": "../x"},
            },
            status=400,
        )
        with self.assertRaises(LabAgentError) as caught:
            self.client.usb_file("tb_01", DEVICE["key"], "../x")
        self.assertEqual(caught.exception.status, 400)
        self.assertEqual(caught.exception.code, "path_denied")
        self.assertIn("../x", str(caught.exception))

    def test_an_error_body_without_a_code_leaves_it_none(self):
        self.route("/testbed/tb_01/usb/devices", {"error": "old agent"}, status=404)
        with self.assertRaises(LabAgentError) as caught:
            self.client.usb_devices("tb_01")
        self.assertIsNone(caught.exception.code)


class TestErrorMapping(ClientTestCase):
    """Every mapping branch the suite's exception handlers depend on."""

    def test_a_501_unsupported_backend_is_unavailable(self):
        self.route(
            "/testbed/tb_01/usb/capabilities",
            {"error": "no USB backend", "code": "usb_host_unsupported"},
            status=501,
        )
        with self.assertRaises(UsbHostUnavailable):
            self.usb.capabilities()

    def test_a_503_hid_unavailable_is_unavailable(self):
        self.route(
            "/testbed/tb_01/usb/hid",
            {"error": "HID driver not bound", "code": "hid_unavailable"},
            status=503,
        )
        from sdk_test.usb_host import UsbDevice

        with self.assertRaises(UsbHostUnavailable):
            self.usb.hid(UsbDevice.from_json(DEVICE))

    def test_a_404_device_not_found_is_not_found(self):
        self.route(
            "/testbed/tb_01/usb/audio",
            {"error": "no such device", "code": "device_not_found"},
            status=404,
        )
        from sdk_test.usb_host import UsbDevice

        with self.assertRaises(UsbNotFound):
            self.usb.audio(UsbDevice.from_json(DEVICE))

    def test_a_port_the_os_refuses_is_a_retryable_conflict(self):
        # The agent reports a failed SetCommState as 409 `port_denied`. The OS
        # can refuse a port that is still settling after a re-enumeration, so
        # this is retryable: the suite sees UsbConflict, not UsbDenied.
        self.route(
            "/testbed/tb_01/usb/serial/open",
            {
                "error": "cannot open COM12",
                "code": "port_denied",
                "details": {"port": "COM12"},
            },
            status=409,
        )
        from sdk_test.usb_host import UsbDevice

        with self.assertRaises(UsbConflict):
            self.usb.serial(UsbDevice.from_json(DEVICE))

    def test_an_unreachable_agent_is_unavailable(self):
        # A transport failure never reaches the error-body mapping: it must
        # still be `UsbHostUnavailable`, not the base class.
        host = UsbHost(LabAgentClient("http://127.0.0.1:1"), "tb_01")
        with self.assertRaises(UsbHostUnavailable):
            host.devices()


class TestFixtureNameIsReserved(SdkTestCase):
    def test_an_instrument_may_not_take_the_usb_host_name(self):
        self.assertIn("usb_host", registry.RESERVED_NAMES)
        with self.assertRaises(ValueError):
            registry.register_instrument("usb_host", object)


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestDutClient(SdkTestCase):
    def test_the_client_is_read_only(self):
        from sdk_test.pytest_plugin import Dut

        dut = Dut(console=None)
        self.assertIsNone(dut.client)
        with self.assertRaises(AttributeError):
            dut.client = object()


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestUsbHostFixture(SdkTestCase):
    """The fixture's failure mode, driven through a real pytest process."""

    def run_suite(
        self,
        body: str,
        declared: str = "[]",
        agent_url: str = "http://127.0.0.1:1",
    ):
        self.write("suite/test_usb.py", body)
        env = dict(os.environ)
        env["PYTHONPATH"] = TOOLS_DIR + (
            os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
        )
        env[ENV_AGENT_URL] = agent_url
        env[ENV_TESTBED_ID] = "tb_01"
        env[ENV_FIXTURES] = declared
        env["PYTHONIOENCODING"] = "utf-8"
        env["PYTHONUTF8"] = "1"
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                os.path.join(self.root, "suite"),
                "-p",
                "sdk_test.pytest_plugin",
                "-p",
                "no:cacheprovider",
                "-q",
            ],
            cwd=self.root,
            env=env,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=120,
            check=False,
        )
        return completed.returncode, (completed.stdout or "") + (completed.stderr or "")

    def test_an_undeclared_fixture_is_an_error_not_a_skip(self):
        # The fixture body runs before the test, and the suite below would fail
        # if it ran at all. The verdict must be non-zero: a skip exits 0 and the
        # framework records that as a pass.
        code, output = self.run_suite(
            "def test_usb(usb_host):\n    raise AssertionError('unreachable')\n"
        )
        self.assertNotEqual(code, 0, output)
        self.assertNotIn("skipped", output.lower())
        self.assertIn("usb_host", output)
        self.assertIn("did not declare", output)

    def test_a_supported_agent_yields_the_client(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), _StubHandler)
        server.requests = []
        server.routes = {
            "/testbed/tb_01/usb/capabilities": [
                (200, {"supported": True, "platform": "win32", "features": []})
            ]
        }
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        host, port = server.server_address

        code, output = self.run_suite(
            "def test_usb(usb_host):\n"
            "    assert usb_host.capabilities().supported\n",
            declared='["usb_host"]',
            agent_url=f"http://{host}:{port}",
        )
        self.assertEqual(code, 0, output)
        self.assertIn("1 passed", output)

    def test_a_parameterised_fixture_is_an_error(self):
        # `usb_host` is a connection, not an instrument with capabilities; the
        # agent would treat the suffix as a bus check and never match it.
        code, output = self.run_suite(
            "def test_usb(usb_host):\n    pass\n", declared='["usb_host:win32"]'
        )
        self.assertNotEqual(code, 0, output)
        self.assertIn("takes no capability", output)


if __name__ == "__main__":
    unittest.main()
