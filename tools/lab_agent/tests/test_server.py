# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the Lab Agent's HTTP control plane.

The server runs for real on an ephemeral port and is driven over HTTP, with only
the console and probe faked. That way routing, JSON encoding, status codes and
threading are the production code paths, not a mock of them.
"""

from __future__ import annotations

import json
import unittest
import urllib.error
import urllib.request

from _support import CONFIG_YAML
from _support import LabTestCase

from lab_agent.server import LabAgentServer


class ServerTestCase(LabTestCase):
    """Starts a real agent on a local port for each test."""

    #: Transcript the fake console replays.
    script = ""
    #: Transcript the fake instrument console replays, if the config has one.
    fixture_script = ""
    #: Runner config the agent is built from; override for a different lab.
    config_yaml = CONFIG_YAML

    def setUp(self):
        super().setUp()
        self.manager = self.make_manager(
            self.config_yaml,
            script=self.script,
            chunk=1,
            fixture_script=self.fixture_script,
        )
        self.manager.config.settings.host = "127.0.0.1"
        self.manager.config.settings.port = 0  # let the OS choose
        self.server = LabAgentServer(self.manager)
        self.server.start_background()
        self.addCleanup(self.server.stop)
        self.base = self.server.url

    def request(self, method, path, body=None):
        """Return ``(status, payload)``; HTTP errors are decoded, not raised."""
        data = None
        if body is not None:
            data = json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base + path, data=data, method=method
        )
        if data is not None:
            request.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read() or b"{}")

    def upload(self, testbed_id: str, payload: bytes):
        """POST a firmware package. The body is a tar, not JSON."""
        request = urllib.request.Request(
            f"{self.base}/testbed/{testbed_id}/firmware", data=payload, method="POST"
        )
        request.add_header("Content-Type", "application/octet-stream")
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read() or b"{}")

    def stage(self, testbed_id: str, platform: str = "sf32lb52-lcd_n16r8") -> str:
        """Upload a package and return the token that names it."""
        status, payload = self.upload(testbed_id, self.a_firmware_package(platform))
        self.assertEqual(status, 200, payload)
        return payload["token"]


class TestHealth(ServerTestCase):
    def test_health_names_the_runner_and_its_testbed_count(self):
        status, payload = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["runner"], "test-runner")
        self.assertEqual(payload["testbeds"], 2)

    def test_health_says_what_it_speaks_and_what_it_is_built_from(self):
        # A caller compares the first and records the second: only a protocol
        # mismatch is a refusal, because the agent is a long-running process and
        # is routinely older than the checkout driving it.
        from sdk_test.version import PROTOCOL

        status, payload = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(payload["protocol"], PROTOCOL)
        self.assertTrue(payload["revision"])


class TestTestbeds(ServerTestCase):
    def test_the_listing_exposes_fixtures_not_just_ids(self):
        status, payload = self.request("GET", "/testbeds")
        self.assertEqual(status, 200)
        by_id = {item["id"]: item for item in payload["testbeds"]}
        self.assertEqual(by_id["tb_01"]["platform"], "sf32lb52-lcd_n16r8")
        self.assertEqual(by_id["tb_01"]["fixtures"], ["spi_nor_flash:spi1", "i2c_sensor"])
        self.assertFalse(by_id["tb_01"]["busy"])

    def test_an_unknown_route_is_a_404(self):
        status, payload = self.request("GET", "/nope")
        self.assertEqual(status, 404)
        self.assertIn("error", payload)


class TestAllocation(ServerTestCase):
    def test_allocate_then_release_round_trips(self):
        status, payload = self.request(
            "POST", "/allocate", {"fixtures": ["i2c_sensor"], "holder": "job-1"}
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["testbed_id"], "tb_01")
        self.assertEqual(payload["holder"], "job-1")
        self.assertEqual(payload["console"]["port"], "COM13")

        status, payload = self.request(
            "POST", "/release", {"testbed_id": "tb_01", "holder": "job-1"}
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["released"])

        status, payload = self.request("GET", "/testbeds")
        self.assertFalse(payload["testbeds"][0]["busy"])

    def test_a_second_allocation_of_the_same_testbed_is_a_409(self):
        self.request("POST", "/allocate", {"testbed_id": "tb_01", "holder": "job-1"})
        status, payload = self.request(
            "POST", "/allocate", {"testbed_id": "tb_01", "holder": "job-2"}
        )
        self.assertEqual(status, 409)
        self.assertIn("job-1", payload["error"])

    def test_an_unsatisfiable_request_is_a_409_that_explains_itself(self):
        status, payload = self.request(
            "POST", "/allocate", {"fixtures": ["i2c_sensor", "no_such_fixture"]}
        )
        self.assertEqual(status, 409)
        self.assertIn("no_such_fixture", payload["error"])

    def test_release_without_a_testbed_id_is_a_400(self):
        status, payload = self.request("POST", "/release", {})
        self.assertEqual(status, 400)
        self.assertIn("testbed_id", payload["error"])

    def test_an_unknown_testbed_is_a_404(self):
        status, payload = self.request("POST", "/testbed/nope/reset")
        self.assertEqual(status, 404)
        self.assertIn("tb_01", payload["error"])


class TestConsoleOverHttp(ServerTestCase):
    script = "Hello world!\r\nmsh />"

    def test_open_read_write_close(self):
        status, payload = self.request("POST", "/testbed/tb_01/console/open")
        self.assertEqual(status, 200)
        self.assertTrue(payload["console_open"])

        # Reading from cursor 0 returns the transcript and the new cursor.
        status, payload = self.request(
            "GET", "/testbed/tb_01/console/read?cursor=0&timeout=2"
        )
        self.assertEqual(status, 200)
        self.assertIn("Hello world!", payload["data"])
        cursor = payload["cursor"]
        self.assertEqual(cursor, len("Hello world!\r\nmsh />"))

        # Reading again from the advanced cursor yields nothing new.
        status, payload = self.request(
            "GET", f"/testbed/tb_01/console/read?cursor={cursor}&timeout=0.2"
        )
        self.assertEqual(payload["data"], "")
        self.assertEqual(payload["cursor"], cursor)

        status, _ = self.request(
            "POST", "/testbed/tb_01/console/write", {"data": "unity_run lz4\r\n"}
        )
        self.assertEqual(status, 200)
        self.assertEqual(self.manager.transports["tb_01"].sent, "unity_run lz4\r\n")

        status, payload = self.request("POST", "/testbed/tb_01/console/close")
        self.assertFalse(payload["console_open"])
        self.assertEqual(self.manager.transports["tb_01"].closed, 1)

    def test_reading_without_opening_is_a_409(self):
        status, payload = self.request("GET", "/testbed/tb_01/console/read?cursor=0")
        self.assertEqual(status, 409)
        self.assertIn("not open", payload["error"])

    def test_a_non_integer_cursor_is_a_400(self):
        self.request("POST", "/testbed/tb_01/console/open")
        status, payload = self.request(
            "GET", "/testbed/tb_01/console/read?cursor=abc"
        )
        self.assertIn(status, (400, 500))
        self.assertIn("cursor", payload["error"])

    def test_a_negative_timeout_is_refused(self):
        self.request("POST", "/testbed/tb_01/console/open")
        status, payload = self.request(
            "GET", "/testbed/tb_01/console/read?cursor=0&timeout=-1"
        )
        self.assertEqual(status, 400)
        self.assertIn("timeout", payload["error"])

    def test_write_without_data_is_a_400(self):
        self.request("POST", "/testbed/tb_01/console/open")
        status, payload = self.request("POST", "/testbed/tb_01/console/write", {})
        self.assertEqual(status, 400)
        self.assertIn("data", payload["error"])


class TestInstrumentConsoleOverHttp(ServerTestCase):
    """The same console routes, addressed with ``stream=<fixture>``.

    An instrument is a second physical port on the testbed, so it is a second
    stream rather than a second testbed: one allocation, two consoles, and the
    default (``stream`` omitted) still means the DUT.
    """

    config_yaml = """
    runner:
      name: test-runner
      host: 127.0.0.1
      port: 0
      console_poll: 0.01

    testbed:
      - id: tb_peer
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: ble_peer
            console:
              port: COM99
              baud: 1000000
    """
    fixture_script = "msh />\r\n"

    def allocate_peer(self):
        status, payload = self.request(
            "POST", "/allocate", {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
        )
        self.assertEqual(status, 200)
        return payload["testbed_id"]

    def test_the_instrument_stream_is_independent_of_the_dut_stream(self):
        testbed = self.allocate_peer()
        # Open the DUT console too, so "the instrument write never reached it"
        # is checked against a real, open transport rather than a missing one.
        self.request("POST", f"/testbed/{testbed}/console/open")

        status, payload = self.request(
            "POST", f"/testbed/{testbed}/console/open?stream=ble_peer"
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["stream"], "ble_peer")

        status, payload = self.request(
            "GET", f"/testbed/{testbed}/console/read?stream=ble_peer&cursor=0&timeout=2"
        )
        self.assertEqual(status, 200)
        self.assertIn("msh", payload["data"])

        status, _ = self.request(
            "POST", f"/testbed/{testbed}/console/write?stream=ble_peer", {"data": "diss help\r\n"}
        )
        self.assertEqual(status, 200)
        self.assertIn("diss help", self.manager.fixture_transports["ble_peer"].sent)
        # The DUT console was never written to.
        self.assertEqual(self.manager.transports["tb_peer"].sent, "")

        status, payload = self.request(
            "POST", f"/testbed/{testbed}/console/close?stream=ble_peer"
        )
        self.assertEqual(status, 200)
        self.assertFalse(payload["console_open"])
        self.assertEqual(self.manager.fixture_transports["ble_peer"].closed, 1)

    def test_an_instrument_the_allocation_did_not_ask_for_is_refused(self):
        status, payload = self.request(
            "POST", "/allocate", {"testbed_id": "tb_peer"}
        )
        self.assertEqual(status, 200)
        testbed = payload["testbed_id"]

        status, payload = self.request(
            "POST", f"/testbed/{testbed}/console/open?stream=ble_peer"
        )
        self.assertEqual(status, 409)
        self.assertIn("not requested", payload["error"])

    def test_an_empty_stream_name_is_a_400(self):
        self.allocate_peer()
        status, payload = self.request("POST", "/testbed/tb_peer/console/open?stream=")
        self.assertEqual(status, 400)
        self.assertIn("stream", payload["error"])

    def test_a_malformed_stream_name_is_a_400(self):
        # `parse_fixture_ref` raises BindingError for a name with no fixture
        # part. It used to fall through to a 500 and a server stack trace.
        self.allocate_peer()
        status, payload = self.request(
            "POST", "/testbed/tb_peer/console/open?stream=%3Apeer1"
        )
        self.assertEqual(status, 400)
        self.assertIn("not a fixture name", payload["error"])

    def test_a_malformed_fixture_in_an_allocation_is_a_400(self):
        # The unpinned path parses the request's fixtures during selection; a
        # bad spec there is the caller's mistake, not a lab failure.
        status, payload = self.request(
            "POST",
            "/allocate",
            {"platform": "sf32lb52-lcd_n16r8", "fixtures": [":bogus"]},
        )
        self.assertEqual(status, 400)
        self.assertIn("bogus", payload["error"])

    def test_a_malformed_requested_fixture_reaches_open_as_a_400(self):
        status, _ = self.request(
            "POST", "/allocate", {"testbed_id": "tb_peer", "fixtures": [":bogus"]}
        )
        self.assertEqual(status, 200)
        status, payload = self.request(
            "POST", "/testbed/tb_peer/console/open?stream=ble_peer"
        )
        self.assertEqual(status, 400)
        self.assertIn("bogus", payload["error"])


class TestPreparationOverHttp(ServerTestCase):
    def test_reset_reaches_the_probe(self):
        status, _ = self.request("POST", "/testbed/tb_01/reset")
        self.assertEqual(status, 200)
        self.assertEqual(len(self.manager.probes["tb_01"].resets), 1)

    def test_flash_needs_a_firmware_token(self):
        status, payload = self.request("POST", "/testbed/tb_01/flash", {})
        self.assertEqual(status, 400)
        self.assertIn("firmware", payload["error"])

    def test_prepare_flashes_the_uploaded_package_and_resets(self):
        token = self.stage("tb_01")

        status, _ = self.request("POST", "/testbed/tb_01/prepare", {"firmware": token})
        self.assertEqual(status, 200)

        flashed = self.manager.probes["tb_01"].flashed
        self.assertEqual(len(flashed), 1)
        # Where it was unpacked, not where it was built -- the two machines need
        # share nothing, so the directory the agent flashes from is its own.
        self.assertTrue(flashed[0].endswith(token), flashed[0])
        self.assertEqual(len(self.manager.probes["tb_01"].resets), 1)


class TestFirmwareUpload(ServerTestCase):
    """A package arrives over HTTP and is flashed from where it landed."""

    def test_a_package_is_staged_under_its_own_digest(self):
        payload = self.a_firmware_package()
        status, body = self.upload("tb_01", payload)

        self.assertEqual(status, 200, body)
        self.assertEqual(body["platform"], "sf32lb52-lcd_n16r8")
        self.assertEqual(body["board"], "sf32lb52-lcd_n16r8_hcpu")
        self.assertEqual(body["bytes"], len(payload))
        # Derived here, not accepted from the caller: it names the package.
        from sdk_test.firmware import digest

        self.assertEqual(body["token"], digest(payload))

    def test_the_same_package_twice_is_staged_once(self):
        # Two test cases sharing one firmware, or a re-run of the same build.
        first = self.stage("tb_01")
        second = self.stage("tb_01")
        self.assertEqual(first, second)

    def test_a_flash_by_token_flashes_the_unpacked_package(self):
        token = self.stage("tb_01")

        status, _ = self.request("POST", "/testbed/tb_01/flash", {"firmware": token})
        self.assertEqual(status, 200)
        self.assertEqual(len(self.manager.probes["tb_01"].flashed), 1)

    def test_a_token_nothing_was_sent_under_is_refused(self):
        # The whole reason the upload and the flash are separate calls. If the
        # agent flashed "whatever arrived last", a run that skipped its upload
        # would test the previous run's image and report green.
        status, payload = self.request(
            "POST", "/testbed/tb_01/flash", {"firmware": "0" * 64}
        )
        self.assertEqual(status, 400)
        self.assertIn("has been uploaded", payload["error"])
        self.assertEqual(self.manager.probes.get("tb_01"), None)

    def test_prepare_by_unknown_token_is_refused_before_anything_moves(self):
        status, payload = self.request(
            "POST", "/testbed/tb_01/prepare", {"firmware": "0" * 64}
        )
        self.assertEqual(status, 400)
        self.assertIn("has been uploaded", payload["error"])
        self.assertEqual(self.manager.probes.get("tb_01"), None)

    def test_a_package_for_another_board_is_refused(self):
        # A wrong-platform image flashes perfectly well, which is why this is
        # checked here: nothing later in the run would notice.
        status, payload = self.upload(
            "tb_01", self.a_firmware_package(platform="eh-lb563")
        )
        self.assertEqual(status, 400)
        self.assertIn("eh-lb563", payload["error"])
        self.assertIn("sf32lb52-lcd_n16r8", payload["error"])

    def test_a_body_that_is_not_a_package_is_refused(self):
        status, payload = self.upload("tb_01", b"this is not a tar at all")
        self.assertEqual(status, 400)
        self.assertIn("not a readable tar", payload["error"])

    def test_an_empty_body_is_refused(self):
        status, payload = self.upload("tb_01", b"")
        self.assertEqual(status, 400)
        self.assertIn("cannot be empty", payload["error"])

    def test_releasing_the_testbed_takes_its_firmware_with_it(self):
        # "Uploaded" must never come to mean "uploaded by the run before this
        # one": the next caller uploads its own, and the disk does not grow.
        self.request("POST", "/allocate", {"testbed_id": "tb_01"})
        token = self.stage("tb_01")
        status, _ = self.request("POST", "/release", {"testbed_id": "tb_01"})
        self.assertEqual(status, 200)

        status, _ = self.request("POST", "/testbed/tb_01/flash", {"firmware": token})
        self.assertEqual(status, 400)


#: A lab with one board that has no probe: downloaded over UART, reset by hand.
MANUAL_CONFIG = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0
  console_poll: 0.01

testbed:
  - id: tb_manual
    platform: sf32lb52-lcd_a128r16
    console: {port: COM1}
    flash: {method: sftool}
    reset: {method: manual}
"""


class TestResetModeOverHttp(ServerTestCase):
    """The probed lab: unchanged behaviour, now with the mode spelled out.

    The mode travels on the **allocation**, not on the later ``/reset`` call:
    the caller has to refuse an unusable harness before it flashes, and by the
    time ``/reset`` is reached the download is already done.
    """

    def test_the_allocation_carries_the_reset_mode(self):
        status, payload = self.request("POST", "/allocate", {})
        self.assertEqual(status, 200)
        self.assertEqual(payload["reset"], "jlink")

    def test_the_listing_reports_the_mode(self):
        status, payload = self.request("GET", "/testbeds")
        self.assertEqual(status, 200)
        self.assertEqual(payload["testbeds"][0]["reset"], "jlink")

    def test_reset_reports_that_it_reset(self):
        status, payload = self.request("POST", "/testbed/tb_01/reset")
        self.assertEqual(status, 200)
        self.assertEqual(payload["mode"], "jlink")
        self.assertTrue(payload["reset"])


class TestManualResetOverHttp(ServerTestCase):
    config_yaml = MANUAL_CONFIG

    def test_the_allocation_carries_the_reset_mode(self):
        status, payload = self.request("POST", "/allocate", {})
        self.assertEqual(status, 200)
        self.assertEqual(payload["reset"], "manual")

    def test_the_listing_reports_the_mode(self):
        status, payload = self.request("GET", "/testbeds")
        self.assertEqual(status, 200)
        self.assertEqual(payload["testbeds"][0]["reset"], "manual")

    def test_reset_reports_that_it_did_not_reset_anything(self):
        # `reset: true` here would be a confident wrong answer: nothing was
        # reset, and whether the operator presses the button is unknowable
        # from this side.
        status, payload = self.request("POST", "/testbed/tb_manual/reset")
        self.assertEqual(status, 200)
        self.assertEqual(payload["mode"], "manual")
        self.assertFalse(payload["reset"])
        self.assertEqual(self.manager.probes, {})


#: The same probe-less board, but the download boots it, so nobody has to be
#: standing by. The mode is the difference; the wiring is identical.
AUTO_CONFIG = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0
  console_poll: 0.01

testbed:
  - id: tb_auto
    platform: sf32lb52-lcd_a128r16
    console: {port: COM1}
    flash: {method: sftool}
    reset: {method: auto}
"""


class TestAutoResetOverHttp(ServerTestCase):
    """``auto`` reaches the caller, and is not confused with a real reset."""

    config_yaml = AUTO_CONFIG

    def test_the_allocation_carries_the_reset_mode(self):
        status, payload = self.request("POST", "/allocate", {})
        self.assertEqual(status, 200)
        self.assertEqual(payload["reset"], "auto")

    def test_the_listing_reports_the_mode(self):
        status, payload = self.request("GET", "/testbeds")
        self.assertEqual(status, 200)
        self.assertEqual(payload["testbeds"][0]["reset"], "auto")

    def test_reset_reports_that_it_did_not_reset_anything(self):
        # The same confident wrong answer would be available here, and it is
        # the tempting one: `auto` is not `manual`, so the old test for it
        # (`mode != "manual"`) said the board had been reset when the call was
        # a deliberate no-op -- the boot was the download's, not ours.
        status, payload = self.request("POST", "/testbed/tb_auto/reset")
        self.assertEqual(status, 200)
        self.assertEqual(payload["mode"], "auto")
        self.assertFalse(payload["reset"])
        self.assertEqual(self.manager.probes, {})


class TestMalformedRequests(ServerTestCase):
    def test_a_body_that_is_not_json_is_reported(self):
        request = urllib.request.Request(
            self.base + "/allocate", data=b"not json", method="POST"
        )
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                status = response.status
                payload = json.loads(response.read())
        except urllib.error.HTTPError as error:
            status = error.code
            payload = json.loads(error.read())
        self.assertEqual(status, 400)
        self.assertIn("JSON", payload["error"])

    def test_a_json_array_body_is_refused(self):
        status, payload = self.request("POST", "/allocate", [])
        self.assertEqual(status, 400)
        self.assertIn("JSON object", payload["error"])

    def test_get_on_a_post_only_route_is_a_405(self):
        status, payload = self.request("GET", "/allocate")
        self.assertEqual(status, 405)


if __name__ == "__main__":
    unittest.main()
