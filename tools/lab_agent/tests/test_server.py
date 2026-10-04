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
    #: Runner config the agent is built from; override for a different lab.
    config_yaml = CONFIG_YAML

    def setUp(self):
        super().setUp()
        self.manager = self.make_manager(self.config_yaml, script=self.script, chunk=1)
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


class TestHealth(ServerTestCase):
    def test_health_names_the_runner_and_its_testbed_count(self):
        status, payload = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["runner"], "test-runner")
        self.assertEqual(payload["testbeds"], 2)


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


class TestPreparationOverHttp(ServerTestCase):
    def test_reset_reaches_the_probe(self):
        status, _ = self.request("POST", "/testbed/tb_01/reset")
        self.assertEqual(status, 200)
        self.assertEqual(len(self.manager.probes["tb_01"].resets), 1)

    def test_flash_needs_a_build_dir(self):
        status, payload = self.request("POST", "/testbed/tb_01/flash", {})
        self.assertEqual(status, 400)
        self.assertIn("build_dir", payload["error"])

    def test_prepare_flashes_and_resets(self):
        status, _ = self.request(
            "POST", "/testbed/tb_01/prepare", {"build_dir": "/tmp/build"}
        )
        self.assertEqual(status, 200)
        self.assertEqual(self.manager.probes["tb_01"].flashed, ["/tmp/build"])
        self.assertEqual(len(self.manager.probes["tb_01"].resets), 1)


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
