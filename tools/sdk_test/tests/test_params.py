# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the runner-side params and secrets chain.

Everything here runs without hardware and without a Lab Agent: keys are derived
from the testcase and the fixture contract, and values come from a file, the
environment or a command-line override. The cases that matter are the failure
ones -- a missing ``psk`` must fail *before* a build, a secret must not be
preferred from the wrong source, and a value must not survive into a log or a
temporary file.
"""

from __future__ import annotations

import json
import os
import time
import unittest
from unittest import mock

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import discovery
from sdk_test import fixture_types
from sdk_test import params
from sdk_test import report
from sdk_test.errors import DiscoveryError
from sdk_test.fixture_types import default_fixture_types
from sdk_test.fixtures import parse_fixture_ref
from sdk_test.harness import HarnessOutcome
from sdk_test.model import TestInstance
from sdk_test.runner import PlannedRun
from sdk_test.runner import RunOptions
from sdk_test.runner import TestRunner

PSK_ENV = "SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK"
FILE_SECRET = "file-secret-123"
ENV_SECRET = "env-secret-456"
CLI_SECRET = "cli-secret-789"


def make_instance(
    fixtures=(),
    declared_params=None,
    platform: str = "board_a",
    case_id: str = "suite/case",
) -> TestInstance:
    return TestInstance(
        id=case_id,
        testsuite_root=".",
        testcase_dir=".",
        tests_key="case",
        harness="pytest",
        fixtures=[parse_fixture_ref(item) for item in fixtures],
        params=dict(declared_params or {}),
        integration_platforms=[platform],
    )


def contracts():
    return default_fixture_types()


class TestParamsFileShape(SdkTestCase):
    def test_flattening_keeps_the_fixtures_prefix_and_leaves_params_keys_alone(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk:
                  value: secret-value
                  secret: true
            params:
              cloud.token: tok-abc
              sta.timeout_s: 30
            """,
        )
        values = params.load_params_file(path)
        self.assertEqual(sorted(values), [
            "cloud.token",
            "fixtures.wifi_ap:5g.psk",
            "fixtures.wifi_ap:5g.ssid",
            "sta.timeout_s",
        ])
        self.assertTrue(values["fixtures.wifi_ap:5g.psk"].secret)
        self.assertEqual(values["fixtures.wifi_ap:5g.psk"].value, "secret-value")
        self.assertFalse(values["cloud.token"].secret)
        self.assertEqual(values["cloud.token"].source, path)

    def test_nested_params_keys_flatten_without_adding_a_params_prefix(self):
        path = self.write(
            "params.yaml",
            """
            params:
              cloud:
                token: tok-abc
            """,
        )
        self.assertEqual(list(params.load_params_file(path)), ["cloud.token"])

    def test_two_spellings_of_one_key_are_rejected(self):
        path = self.write(
            "params.yaml",
            """
            params:
              cloud.token: one
              cloud:
                token: two
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.load_params_file(path)
        self.assertIn("defined more than once", str(caught.exception))

    def test_a_duplicate_key_in_one_mapping_is_rejected(self):
        # PyYAML keeps the last of two identical keys; the params file must
        # fail instead of silently preferring one value.
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: one
                psk: two
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.load_params_file(path)
        message = str(caught.exception)
        self.assertIn("duplicate key 'psk'", message)
        self.assertIn("line 5", message)

    def test_an_unknown_section_is_rejected(self):
        path = self.write(
            "params.yaml",
            """
            fixture:
              wifi_ap: {}
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.load_params_file(path)
        self.assertIn("unknown section", str(caught.exception))

    def test_an_empty_file_resolves_nothing(self):
        path = self.write("params.yaml", "")
        self.assertEqual(params.load_params_file(path), {})


class TestFlatteningCollisions(SdkTestCase):
    def test_a_colon_and_an_underscore_collide(self):
        with self.assertRaises(params.ParamsError) as caught:
            params.check_key_collisions(
                ["fixtures.wifi_ap:5g.psk", "fixtures.wifi_ap_5g.psk"]
            )
        self.assertIn("SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK", str(caught.exception))

    def test_distinct_keys_do_not_collide(self):
        params.check_key_collisions(
            ["fixtures.wifi_ap:5g.psk", "fixtures.wifi_ap:2g.psk"]
        )

    def test_a_collision_in_the_file_fails_resolution(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: A
                security: open
              "wifi_ap_5g":
                ssid: B
                security: open
            """,
        )
        with self.assertRaises(params.ParamsError):
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=path,
                environ={},
            )

    def test_the_environment_name_is_documented_in_the_design(self):
        self.assertEqual(
            params.flatten_env_name("fixtures.wifi_ap:5g.psk"),
            PSK_ENV,
        )


class TestPrecedence(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.path = self.write(
            "params.yaml",
            f"""
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: {FILE_SECRET}
            """,
        )
        self.instance = make_instance(["wifi_ap:5g"])

    def resolve(self, overrides=None, environ=None):
        return params.resolve_params(
            self.instance,
            contracts(),
            params_file=self.path,
            overrides=overrides,
            environ={} if environ is None else environ,
        )

    def test_the_file_supplies_the_value(self):
        resolved = self.resolve()
        self.assertEqual(
            resolved.fixtures()["wifi_ap:5g"]["psk"], FILE_SECRET
        )
        self.assertEqual(
            resolved.sources()["fixtures.wifi_ap:5g.psk"], self.path
        )

    def test_the_contract_default_fills_an_absent_option(self):
        resolved = self.resolve()
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["security"], "wpa2")
        self.assertIn(
            "default (contract wifi_ap)",
            resolved.sources()["fixtures.wifi_ap:5g.security"],
        )

    def test_the_environment_beats_the_file(self):
        resolved = self.resolve(environ={PSK_ENV: ENV_SECRET})
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], ENV_SECRET)
        self.assertIn(PSK_ENV, resolved.sources()["fixtures.wifi_ap:5g.psk"])

    def test_an_empty_environment_value_is_an_explicit_value(self):
        # Presence, not truthiness: an explicitly empty variable must not fall
        # through to the file.
        resolved = self.resolve(environ={PSK_ENV: ""})
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], "")
        self.assertIn(
            "environment", resolved.sources()["fixtures.wifi_ap:5g.psk"]
        )

    def test_the_command_line_beats_everything(self):
        resolved = self.resolve(
            overrides={"fixtures.wifi_ap:5g.psk": CLI_SECRET},
            environ={PSK_ENV: ENV_SECRET},
        )
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], CLI_SECRET)
        self.assertEqual(
            resolved.sources()["fixtures.wifi_ap:5g.psk"], "--param"
        )

    def test_a_declared_default_is_used_when_nothing_else_supplies_it(self):
        instance = make_instance(
            ["wifi_ap:5g"], declared_params={"upload.timeout_s": 30}
        )
        resolved = params.resolve_params(
            instance, contracts(), params_file=self.path, environ={}
        )
        self.assertEqual(resolved.params()["upload.timeout_s"], 30)
        self.assertEqual(
            resolved.sources()["upload.timeout_s"], "default (testcase)"
        )

    def test_a_declared_required_key_is_resolved_from_the_file(self):
        path = self.write(
            "more.yaml",
            """
            params:
              cloud.token: tok-abc
            """,
        )
        instance = make_instance(declared_params={"cloud.token": {"required": True}})
        resolved = params.resolve_params(
            instance, contracts(), params_file=path, environ={}
        )
        self.assertEqual(resolved.params()["cloud.token"], "tok-abc")

    def test_a_required_key_counts_as_present_when_the_environment_is_empty(self):
        path = self.write("more.yaml", "params:\n  other.key: x\n")
        instance = make_instance(declared_params={"cloud.token": {"required": True}})
        resolved = params.resolve_params(
            instance,
            contracts(),
            params_file=path,
            environ={"SIFLI_TEST_PARAM_CLOUD_TOKEN": ""},
        )
        self.assertEqual(resolved.params()["cloud.token"], "")
        self.assertIn("environment", resolved.sources()["cloud.token"])


class TestMissingValues(SdkTestCase):
    def test_a_missing_required_option_names_the_key_and_the_search_paths(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                security: wpa2
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=path,
                environ={},
            )
        message = str(caught.exception)
        self.assertIn("fixtures.wifi_ap:5g.ssid", message)
        self.assertIn("wifi_ap:5g", message)
        self.assertIn("searched for a params file", message)
        self.assertIn(PSK_ENV, message)

    def test_a_missing_spec_section_fails_and_names_the_spec(self):
        path = self.write("params.yaml", "params:\n  cloud.token: tok-abc\n")
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=path,
                environ={},
            )
        message = str(caught.exception)
        self.assertIn('fixture "wifi_ap:5g"', message)
        # The missing list, not the per-key override hints further down: both
        # name the keys, and only the former says the value is actually absent.
        self.assertIn("- fixtures.wifi_ap:5g.ssid  (fixture", message)
        # wpa2 is the contract default, so the psk becomes required too.
        self.assertIn("- fixtures.wifi_ap:5g.psk  (fixture", message)

    def test_no_params_file_is_reported_with_the_documented_locations(self):
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                environ={"USERPROFILE": self.root},
            )
        message = str(caught.exception)
        self.assertIn("SIFLI_TEST_PARAMS_FILE", message)
        self.assertIn(os.path.join(self.root, ".sifli", "test-params.yaml"), message)

    def test_an_explicitly_named_file_that_does_not_exist_is_an_error(self):
        missing = os.path.join(self.root, "nope.yaml")
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=missing,
                environ={},
            )
        self.assertIn(missing, str(caught.exception))

    def test_an_open_access_point_needs_no_psk(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: OpenAP
                security: open
            """,
        )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=path,
            environ={},
        )
        options = resolved.fixtures()["wifi_ap:5g"]
        self.assertEqual(options, {"ssid": "OpenAP", "security": "open"})
        self.assertNotIn("psk", options)

    def test_an_instance_needing_nothing_needs_no_file(self):
        resolved = params.resolve_params(
            make_instance(["bt_peer:ble"]),
            contracts(),
            environ={"USERPROFILE": self.root},
        )
        self.assertEqual(resolved.values, {})
        self.assertIsNone(resolved.params_file)

    def test_a_missing_conditional_dependency_is_reported_as_the_dependency(self):
        # `security` is required so the contract loads, but the params file does
        # not supply it. The condition on `psk` cannot be evaluated, so `psk`
        # must not be reported as the missing value.
        contract = fixture_types.parse_fixture_types(
            {
                "wifi_ap": {
                    "options": {
                        "ssid": {
                            "source": "params",
                            "key": "fixtures.{spec}.ssid",
                            "required": True,
                        },
                        "security": {
                            "source": "params",
                            "key": "fixtures.{spec}.security",
                            "required": True,
                        },
                        "psk": {
                            "source": "params",
                            "key": "fixtures.{spec}.psk",
                            "required_if": "security != open",
                            "secret": True,
                        },
                    }
                }
            }
        )
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]), contract, params_file=path, environ={}
            )
        message = str(caught.exception)
        missing_section = message.split("searched for a params file:")[0]
        self.assertIn("- fixtures.wifi_ap:5g.security", missing_section)
        self.assertIn("psk.required_if", missing_section)
        self.assertNotIn("- fixtures.wifi_ap:5g.psk", missing_section)


class TestKeyDerivation(SdkTestCase):
    def test_fixture_keys_come_from_the_requested_spec_exactly(self):
        instance = make_instance(["wifi_ap:5g"])
        keys = params.requirement_keys(instance, contracts())
        self.assertEqual(
            keys,
            [
                "fixtures.wifi_ap:5g.ssid",
                "fixtures.wifi_ap:5g.security",
                "fixtures.wifi_ap:5g.psk",
            ],
        )

    def test_a_type_without_a_contract_contributes_no_keys(self):
        instance = make_instance(["bt_peer:ble"])
        self.assertEqual(params.requirement_keys(instance, contracts()), [])

    def test_a_spec_that_prefixes_another_does_not_steal_its_options(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: Short
                security: open
              "wifi_ap:5g.x":
                ssid: Long
                security: open
            """,
        )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g", "wifi_ap:5g.x"]),
            contracts(),
            params_file=path,
            environ={},
        )
        self.assertEqual(
            sorted(resolved.fixtures()["wifi_ap:5g"]), ["security", "ssid"]
        )
        self.assertEqual(resolved.fixtures()["wifi_ap:5g.x"]["ssid"], "Long")

    def test_only_declared_params_are_delivered(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                security: open
            params:
              cloud.token: tok-abc
              unused.key: ignored
            """,
        )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=path,
            environ={},
        )
        self.assertEqual(resolved.params(), {})
        self.assertEqual(sorted(resolved.payload()), ["fixtures", "params"])
        self.assertEqual(sorted(resolved.payload()["fixtures"]), ["wifi_ap:5g"])

    def test_unused_keys_are_listed_and_not_a_failure(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                security: open
            params:
              unused.key: ignored
            """,
        )
        values = params.load_params_file(path)
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=path,
            environ={},
        )
        self.assertEqual(
            params.unused_keys(values, resolved.requested_keys), ["unused.key"]
        )


class TestSecrecy(SdkTestCase):
    def test_the_contract_decides_secrecy_not_the_file(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid:
                  value: LabAP
                  secret: true
                security: open
            params:
              cloud.token:
                value: tok-abcdef
                secret: true
            """,
        )
        resolved = params.resolve_params(
            make_instance(
                ["wifi_ap:5g"], declared_params={"cloud.token": {"required": True}}
            ),
            contracts(),
            params_file=path,
            environ={},
        )
        # The contract has no `secret` on ssid, so the file's marker is ignored.
        self.assertNotIn("LabAP", resolved.secrets)
        self.assertIn("tok-abcdef", resolved.secrets)

    def test_a_short_secret_warns_but_is_still_secret(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: abc
            """,
        )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=path,
            environ={},
        )
        self.assertIn("abc", resolved.secrets)
        self.assertEqual(len(resolved.warnings), 1)
        self.assertIn("shorter than", resolved.warnings[0])

    def test_longest_secret_replaced_first(self):
        text = params.redact_text("value=abcdefgh tail", ["abcd", "abcdefgh"])
        self.assertEqual(text, "value=*** tail")


class TestLoopbackGuard(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.path = self.write(
            "params.yaml",
            f"""
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: {FILE_SECRET}
            """,
        )

    def resolve(self, agent_url, environ=None):
        return params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=self.path,
            environ={} if environ is None else environ,
            agent_url=agent_url,
        )

    def test_a_remote_agent_with_secrets_is_refused(self):
        with self.assertRaises(params.ParamsError) as caught:
            self.resolve("http://10.0.0.5:8760")
        self.assertIn("SIFLI_TEST_ALLOW_REMOTE_SECRETS", str(caught.exception))

    def test_the_override_variable_allows_it(self):
        resolved = self.resolve(
            "http://10.0.0.5:8760",
            environ={params.ENV_ALLOW_REMOTE_SECRETS: "1"},
        )
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], FILE_SECRET)

    def test_loopback_spellings_are_allowed(self):
        for url in ("http://127.0.0.1:8760", "http://localhost:8760", "http://[::1]:8760"):
            with self.subTest(url=url):
                resolved = self.resolve(url)
                self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], FILE_SECRET)

    def test_a_host_backend_needs_no_guard(self):
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=self.path,
            environ={},
            agent_url="http://10.0.0.5:8760",
            uses_agent=False,
        )
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], FILE_SECRET)

    def test_the_guard_is_decided_by_uses_agent_not_by_the_url_alone(self):
        # The same remote URL and the same secret: refused when an agent is
        # used, allowed when the backend is local and there is nobody to send
        # the value to.
        with self.assertRaises(params.ParamsError):
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=self.path,
                environ={},
                agent_url="http://10.0.0.5:8760",
                uses_agent=True,
            )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=self.path,
            environ={},
            agent_url="http://10.0.0.5:8760",
            uses_agent=False,
        )
        self.assertEqual(resolved.fixtures()["wifi_ap:5g"]["psk"], FILE_SECRET)


class TestRedaction(SdkTestCase):
    def test_result_and_outcome_are_cleaned_before_they_are_written(self):
        secret = "psk-hunter2"
        result = report.TestResult(
            id="suite/case",
            status=report.FAILED,
            message=f"assertion saw {secret}",
            log=f"harness output {secret}",
            transcript=f"device said {secret}",
            build_output="",
            instrument_logs={"bt_peer:ble": f"peer saw {secret}"},
            capture_warnings=[f"capture failed on {secret}"],
            cases=[
                report.CaseResult(
                    name="case_one", status=report.FAILED, message=f"case {secret}"
                )
            ],
        )
        outcome = HarnessOutcome(
            passed=False,
            detail=f"failed with {secret}",
            log=f"raw harness log {secret}",
        )

        params.redact_outcome(outcome, [secret])
        params.redact_result(result, [secret])

        for text in (
            result.message,
            result.log,
            result.transcript,
            result.instrument_logs["bt_peer:ble"],
            result.capture_warnings[0],
            result.cases[0].message,
            outcome.log,
            outcome.detail,
        ):
            self.assertNotIn(secret, text)
            self.assertIn(params.REDACTED, text)

    def test_the_serialised_report_contains_no_secret(self):
        secret = "psk-hunter2"
        result = report.TestResult(
            id="suite/case",
            status=report.FAILED,
            message=f"failed {secret}",
            log=f"harness {secret}",
            transcript=f"device {secret}",
        )
        params.redact_result(result, [secret])
        summary = report.RunSummary(root=".", out_dir=".")
        summary.add(result)
        document = json.dumps(report.summary_to_dict(summary))
        self.assertNotIn(secret, document)
        self.assertIn(params.REDACTED, document)

    def test_redaction_is_skipped_when_there_is_nothing_to_hide(self):
        result = report.TestResult(id="x", status=report.PASSED, log="clean")
        params.redact_result(result, [])
        self.assertEqual(result.log, "clean")


class TestDataFile(SdkTestCase):
    def resolved(self) -> params.ResolvedParams:
        path = self.write(
            "params.yaml",
            f"""
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                security: wpa2
                psk: {FILE_SECRET}
            params:
              cloud.token: tok-abcdef
            """,
        )
        return params.resolve_params(
            make_instance(
                ["wifi_ap:5g"], declared_params={"cloud.token": {"required": True}}
            ),
            contracts(),
            params_file=path,
            environ={},
        )

    def test_the_file_carries_params_and_fixtures_by_spec(self):
        path = params.write_data_file(self.resolved(), directory=self.root)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        finally:
            params.remove_data_file(path)
        self.assertEqual(document["params"], {"cloud.token": "tok-abcdef"})
        self.assertEqual(
            document["fixtures"],
            {"wifi_ap:5g": {"ssid": "LabAP", "security": "wpa2", "psk": FILE_SECRET}},
        )

    def test_the_file_is_removed(self):
        path = params.write_data_file(self.resolved(), directory=self.root)
        self.assertTrue(os.path.isfile(path))
        self.assertTrue(params.remove_data_file(path))
        self.assertFalse(os.path.exists(path))
        # Removing it twice is not an error: cleanup runs on every failure path.
        self.assertTrue(params.remove_data_file(path))

    def test_only_stale_files_are_cleaned_up(self):
        now = time.time()
        stale = params.write_data_file(self.resolved(), directory=self.root)
        fresh = params.write_data_file(self.resolved(), directory=self.root)
        unrelated = self.write("keep-me.json", "{}")
        os.utime(stale, (now - params.STALE_DATA_FILE_SECONDS - 60,) * 2)
        os.utime(fresh, (now - 60,) * 2)

        removed = params.cleanup_stale_data_files(directory=self.root, now=now)

        self.assertEqual(removed, [stale])
        self.assertFalse(os.path.exists(stale))
        self.assertTrue(os.path.exists(fresh))
        self.assertTrue(os.path.exists(unrelated))

    def test_cleanup_ignores_a_directory_it_cannot_read(self):
        self.assertEqual(
            params.cleanup_stale_data_files(
                directory=os.path.join(self.root, "missing")
            ),
            [],
        )


class TestBuildEnvironment(SdkTestCase):
    def test_the_build_subprocess_does_not_inherit_param_overrides(self):
        seen = {}

        def build_fn(project_dir, platform, extra_args, jobs):
            seen["any"] = [
                key for key in os.environ if key.startswith(params.ENV_PARAM_PREFIX)
            ]
            return ""

        instance = make_instance(["bt_peer:ble"])
        runner = TestRunner(RunOptions(testsuite_root=self.root), build_fn=build_fn)
        planned = PlannedRun(
            instance=instance,
            platform="board_a",
            backend="board",
            project_dir=self.root,
            build_dir=self.root,
            board="board_a_hcpu",
        )
        with mock.patch.dict(os.environ, {PSK_ENV: FILE_SECRET}, clear=False):
            runner._do_build(planned)
            self.assertEqual(seen["any"], [])
            # Restored once the build returns, so the next step can still use it.
            self.assertEqual(os.environ[PSK_ENV], FILE_SECRET)


class TestDiscovery(SdkTestCase):
    def test_params_are_read_from_the_testcase(self):
        path = self.write(
            "case/testcase.yaml",
            """
            tests:
              upload:
                harness: pytest
                harness_config:
                  fixture: wifi_ap:5g
                params:
                  cloud.token: {required: true}
                  upload.timeout_s: 30
            """,
        )
        instances = discovery.parse_testcase_file(path, self.root)
        self.assertEqual(
            instances[0].params,
            {"cloud.token": {"required": True}, "upload.timeout_s": 30},
        )
        self.assertEqual(
            [fixture.spec for fixture in instances[0].fixtures], ["wifi_ap:5g"]
        )

    def test_a_declaration_may_mark_a_key_secret(self):
        path = self.write(
            "case/testcase.yaml",
            """
            tests:
              upload:
                params:
                  cloud.token: {required: true, secret: true}
            """,
        )
        instances = discovery.parse_testcase_file(path, self.root)
        self.assertEqual(
            instances[0].params, {"cloud.token": {"required": True, "secret": True}}
        )

    def test_a_non_scalar_non_required_declaration_is_rejected_by_the_schema(self):
        path = self.write(
            "case/testcase.yaml",
            """
            tests:
              upload:
                params:
                  cloud.token: [one, two]
            """,
        )
        with self.assertRaises(DiscoveryError):
            discovery.parse_testcase_file(path, self.root)

    def test_a_params_block_must_be_a_mapping(self):
        path = self.write(
            "case/testcase.yaml",
            """
            tests:
              upload:
                params: [cloud.token]
            """,
        )
        with self.assertRaises(DiscoveryError):
            discovery.parse_testcase_file(path, self.root)


class TestDeclarativeSecrets(SdkTestCase):
    """A testcase may mark a declared parameter secret.

    The file is not the only place a value can come from, so the marker has to
    travel with the *key*, not with the record: an override supplied through the
    environment or ``--param`` is as secret as one read from the file.
    """

    def instance(self) -> TestInstance:
        return make_instance(
            declared_params={"cloud.token": {"required": True, "secret": True}}
        )

    def resolve(self, *, path=None, overrides=None, environ=None, agent_url=None):
        return params.resolve_params(
            self.instance(),
            contracts(),
            params_file=path,
            overrides=overrides,
            environ={} if environ is None else environ,
            agent_url=agent_url,
        )

    def empty_file(self) -> str:
        return self.write("params.yaml", "params:\n  other.key: x\n")

    def test_a_secret_from_the_file(self):
        path = self.write("params.yaml", "params:\n  cloud.token: tok-from-file\n")
        self.assertEqual(self.resolve(path=path).secrets, ["tok-from-file"])

    def test_a_secret_from_the_environment(self):
        resolved = self.resolve(
            path=self.empty_file(),
            environ={"SIFLI_TEST_PARAM_CLOUD_TOKEN": "tok-from-env"},
        )
        self.assertEqual(resolved.secrets, ["tok-from-env"])

    def test_a_secret_from_the_command_line(self):
        resolved = self.resolve(
            path=self.empty_file(),
            overrides={"cloud.token": "tok-from-cli"},
        )
        self.assertEqual(resolved.secrets, ["tok-from-cli"])

    def test_a_remote_agent_is_refused_for_every_source(self):
        cases = {
            "file": {"path": self.write("f.yaml", "params:\n  cloud.token: tok-file\n")},
            "environment": {
                "path": self.empty_file(),
                "environ": {"SIFLI_TEST_PARAM_CLOUD_TOKEN": "tok-env"},
            },
            "command line": {
                "path": self.empty_file(),
                "overrides": {"cloud.token": "tok-cli"},
            },
        }
        for label, kwargs in cases.items():
            with self.subTest(source=label):
                with self.assertRaises(params.ParamsError) as caught:
                    self.resolve(agent_url="http://10.0.0.5:8760", **kwargs)
                self.assertIn("SIFLI_TEST_ALLOW_REMOTE_SECRETS", str(caught.exception))

    def test_the_value_is_redacted_whichever_source_supplied_it(self):
        for overrides, environ, expected in (
            ({"cloud.token": "tok-from-cli"}, {}, "tok-from-cli"),
            ({}, {"SIFLI_TEST_PARAM_CLOUD_TOKEN": "tok-from-env"}, "tok-from-env"),
        ):
            with self.subTest(value=expected):
                resolved = self.resolve(
                    path=self.empty_file(), overrides=overrides, environ=environ
                )
                self.assertEqual(
                    params.redact_text(f"log {expected} end", resolved.secrets),
                    "log *** end",
                )


class TestEnumValidation(SdkTestCase):
    def test_a_value_outside_the_contract_enum_fails(self):
        path = self.write(
            "params.yaml",
            f"""
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                security: wpa-2
                psk: {"a-real-psk"}
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contracts(),
                params_file=path,
                environ={},
            )
        message = str(caught.exception)
        self.assertIn("fixtures.wifi_ap:5g.security", message)
        self.assertIn("wpa2", message)

    def test_a_non_string_value_is_not_coerced_into_the_enum(self):
        # The contract's enum is a list of strings, so a YAML number that looks
        # the same is not a member. Coercing it would accept a value the
        # contract rejects.
        contract = fixture_types.parse_fixture_types(
            {
                "wifi_ap": {
                    "options": {
                        "ssid": {
                            "source": "params",
                            "key": "fixtures.{spec}.ssid",
                            "required": True,
                        },
                        "security": {
                            "source": "params",
                            "key": "fixtures.{spec}.security",
                            "enum": ["1"],
                            "secret": True,
                        },
                    }
                }
            }
        )
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                security: 1
            """,
        )
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(["wifi_ap:5g"]),
                contract,
                params_file=path,
                environ={},
            )
        message = str(caught.exception)
        self.assertIn("fixtures.wifi_ap:5g.security", message)
        self.assertIn("must be one of ['1']", message)
        # A secret value is never echoed.
        self.assertNotIn("got", message)


class TestJsonDelivery(SdkTestCase):
    def test_a_value_json_cannot_carry_fails_before_the_build(self):
        # PyYAML turns an unquoted ISO date into a `datetime.date`, which
        # `json.dump` cannot encode. Finding that out after a flash is a wasted
        # cycle, so resolution rejects it.
        path = self.write("params.yaml", "params:\n  release.date: 2024-01-01\n")
        with self.assertRaises(params.ParamsError) as caught:
            params.resolve_params(
                make_instance(declared_params={"release.date": {"required": True}}),
                contracts(),
                params_file=path,
                environ={},
            )
        message = str(caught.exception)
        self.assertIn("JSON", message)
        self.assertIn("release.date", message)


class TestPlanOrdering(SdkTestCase):
    """Resolution happens at plan time, so a build never starts for a bad config."""

    def instance(self, skip: bool) -> TestInstance:
        self.write("SConstruct", "")
        self.write("proj.conf", "")
        return TestInstance(
            id="suite/case",
            testsuite_root=self.root,
            testcase_dir=self.root,
            tests_key="case",
            harness="pytest",
            fixtures=[parse_fixture_ref("wifi_ap:5g")],
            params={"cloud.token": {"required": True}},
            skip=skip,
            integration_platforms=["board_a"],
        )

    def runner(self) -> TestRunner:
        return TestRunner(RunOptions(testsuite_root=self.root))

    def test_a_missing_key_is_a_plan_error_before_any_build(self):
        with mock.patch.dict(os.environ, {"USERPROFILE": self.root}, clear=False):
            os.environ.pop(params.ENV_PARAMS_FILE, None)
            with self.assertRaises(params.ParamsError):
                self.runner().plan_one(self.instance(skip=False))

    def test_a_skipped_case_needs_no_secret(self):
        with mock.patch.dict(os.environ, {"USERPROFILE": self.root}, clear=False):
            os.environ.pop(params.ENV_PARAMS_FILE, None)
            planned = self.runner().plan_one(self.instance(skip=True))
        self.assertEqual(planned.resolved.values, {})

    def test_a_short_secret_warning_is_echoed_before_the_build(self):
        path = self.write(
            "short.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: abc
            params:
              cloud.token: tok-abcdef
            """,
        )
        messages = []
        runner = TestRunner(
            RunOptions(testsuite_root=self.root, params_file=path),
            echo=messages.append,
        )
        runner.plan_one(self.instance(skip=False))
        matched = [line for line in messages if "shorter than" in line]
        self.assertEqual(len(matched), 1)
        self.assertIn("fixtures.wifi_ap:5g.psk", matched[0])
        # The warning names the key, never the value.
        self.assertNotIn("abc", matched[0])


class TestReportProvenance(SdkTestCase):
    def test_param_names_and_sources_reach_the_json_and_xml(self):
        import xml.etree.ElementTree as ElementTree

        result = report.TestResult(
            id="suite/case",
            status=report.PASSED,
            params=[
                {"name": "cloud.token", "source": "E:/lab/params.yaml"},
                {
                    "name": "fixtures.wifi_ap:5g.psk",
                    "source": "environment (SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK)",
                },
            ],
        )
        summary = report.RunSummary(root=".", out_dir=".")
        summary.add(result)
        document = report.summary_to_dict(summary)

        entries = document["testsuites"][0]["params"]
        # Names and sources only: a value must never be in the report.
        for entry in entries:
            self.assertEqual(sorted(entry), ["name", "source"])
            self.assertNotIn("value", entry)
        self.assertEqual(entries[0]["name"], "cloud.token")

        json_path = os.path.join(self.root, "sifli.json")
        report.write_json(document, json_path)
        with open(json_path, "r", encoding="utf-8") as handle:
            written = handle.read()
        self.assertIn("cloud.token", written)
        self.assertIn("E:/lab/params.yaml", written)

        xml_path = os.path.join(self.root, "sifli.xml")
        report.write_junit_from_dict(document, xml_path)
        tree = ElementTree.parse(xml_path)
        self.assertEqual(len(tree.getroot().findall("testcase")), 1)


class TestPluginSide(SdkTestCase):
    """The pytest-side accessors, exercised without starting pytest."""

    def setUp(self):
        super().setUp()
        try:
            from sdk_test import pytest_plugin  # noqa: F401
        except ImportError:  # pragma: no cover - pytest is part of the SDK env
            self.skipTest("pytest is not installed in this interpreter")

    def test_the_harness_passes_the_data_file_path(self):
        from sdk_test.harness import RunContext
        from sdk_test.harness.pytest import ENV_DATA_FILE
        from sdk_test.harness.pytest import plugin_env

        env = plugin_env(
            make_instance(), RunContext(data_file="C:/tmp/values.json")
        )
        self.assertEqual(env[ENV_DATA_FILE], "C:/tmp/values.json")

    def test_no_data_file_means_no_variable(self):
        from sdk_test.harness import RunContext
        from sdk_test.harness.pytest import ENV_DATA_FILE
        from sdk_test.harness.pytest import plugin_env

        self.assertNotIn(ENV_DATA_FILE, plugin_env(make_instance(), RunContext()))

    def write_data(self, document) -> str:
        path = self.write("data.json", json.dumps(document))
        return path

    def test_fixture_options_returns_the_resolved_options(self):
        from sdk_test import pytest_plugin

        path = self.write_data(
            {"params": {}, "fixtures": {"wifi_ap:5g": {"ssid": "LabAP", "psk": "x"}}}
        )
        with mock.patch.dict(
            os.environ,
            {pytest_plugin.ENV_DATA_FILE: path, pytest_plugin.ENV_FIXTURES: json.dumps(["wifi_ap:5g"])},
        ):
            dut = pytest_plugin.Dut(console=object())
            self.assertEqual(
                dut.fixture_options("wifi_ap:5g"), {"ssid": "LabAP", "psk": "x"}
            )

    def test_an_undeclared_spec_is_an_error(self):
        from sdk_test import pytest_plugin

        path = self.write_data({"params": {}, "fixtures": {}})
        with mock.patch.dict(
            os.environ,
            {pytest_plugin.ENV_DATA_FILE: path, pytest_plugin.ENV_FIXTURES: json.dumps(["wifi_ap:5g"])},
        ):
            dut = pytest_plugin.Dut(console=object())
            with self.assertRaises(RuntimeError) as caught:
                dut.fixture_options("wifi_ap:2g")
        self.assertIn("unknown fixture", str(caught.exception))

    def test_a_misspelt_option_is_a_key_error(self):
        from sdk_test import pytest_plugin

        path = self.write_data(
            {"params": {}, "fixtures": {"wifi_ap:5g": {"ssid": "LabAP"}}}
        )
        with mock.patch.dict(
            os.environ,
            {pytest_plugin.ENV_DATA_FILE: path, pytest_plugin.ENV_FIXTURES: json.dumps(["wifi_ap:5g"])},
        ):
            dut = pytest_plugin.Dut(console=object())
            options = dut.fixture_options("wifi_ap:5g")
            with self.assertRaises(KeyError):
                _ = options["pks"]

    def test_the_params_document_is_read_through_the_environment(self):
        from sdk_test import pytest_plugin

        path = self.write_data({"params": {"cloud.token": "tok-abc"}, "fixtures": {}})
        with mock.patch.dict(os.environ, {pytest_plugin.ENV_DATA_FILE: path}):
            self.assertEqual(
                pytest_plugin._resolved_data()["params"], {"cloud.token": "tok-abc"}
            )


class TestRunnerDelivery(SdkTestCase):
    """``run_one`` writes the values, hands over the path, and deletes the file."""

    class FakeSession:
        testbed_id = "tb_01"
        platform = "board_a"
        agent_revision = ""
        manual_reset = False
        dropped = 0
        transcript = "device said psk-hunter2"
        instrument_transcript = None
        console = object()

    class FakeBackend:
        name = "board"

        def open(self, instance, platform, **_kwargs):
            return TestRunnerDelivery.FakeSession()

        def close(self, _session):
            return None

    def planned(self, resolved) -> PlannedRun:
        return PlannedRun(
            instance=make_instance(["wifi_ap:5g"]),
            platform="board_a",
            backend="board",
            project_dir=self.root,
            build_dir=self.root,
            board="board_a_hcpu",
            out_dir=self.root,
            resolved=resolved,
        )

    def test_the_data_file_lives_for_the_harness_only_and_is_redacted(self):
        from sdk_test import runner as runner_module

        path = self.write(
            "params.yaml",
            f"""
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: psk-hunter2
            """,
        )
        resolved = params.resolve_params(
            make_instance(["wifi_ap:5g"]),
            contracts(),
            params_file=path,
            environ={},
        )
        seen = {}

        def fake_run(instance, console, timeout, context=None):
            seen["path"] = context.data_file
            seen["exists"] = os.path.isfile(context.data_file)
            with open(context.data_file, "r", encoding="utf-8") as handle:
                seen["document"] = json.load(handle)
            return HarnessOutcome(
                passed=False,
                detail="failed with psk-hunter2",
                log="harness log psk-hunter2",
            )

        runner = TestRunner(
            RunOptions(
                testsuite_root=self.root,
                build=False,
                agent_url="http://127.0.0.1:8760",
            )
        )
        planned = self.planned(resolved)
        with mock.patch.object(
            runner_module, "create_backend", return_value=self.FakeBackend()
        ), mock.patch.object(runner_module, "run_harness", side_effect=fake_run):
            result = runner.run_one(planned)

        self.assertTrue(seen["exists"])
        self.assertEqual(
            seen["document"]["fixtures"]["wifi_ap:5g"]["psk"], "psk-hunter2"
        )
        self.assertFalse(os.path.exists(seen["path"]))
        self.assertEqual(result.status, report.FAILED)
        self.assertNotIn("psk-hunter2", result.message)
        self.assertNotIn("psk-hunter2", result.log)
        self.assertNotIn("psk-hunter2", result.transcript)
        self.assertIn(params.REDACTED, result.message)

        # Provenance is recorded by name and source, and never by value.
        self.assertEqual(
            [entry["name"] for entry in result.params],
            [
                "fixtures.wifi_ap:5g.ssid",
                "fixtures.wifi_ap:5g.security",
                "fixtures.wifi_ap:5g.psk",
            ],
        )
        self.assertTrue(all(entry["source"] for entry in result.params))

        for name in ("device.log", "runner.log"):
            with open(os.path.join(self.root, name), "r", encoding="utf-8") as handle:
                self.assertNotIn("psk-hunter2", handle.read())

    def test_a_non_pytest_case_gets_no_data_file(self):
        from sdk_test import runner as runner_module

        resolved = params.ResolvedParams(specs=[])
        seen = {}

        def fake_run(instance, console, timeout, context=None):
            seen["path"] = context.data_file
            return HarnessOutcome(passed=True, detail="ok", log="")

        planned = self.planned(resolved)
        planned.instance.harness = "console"
        runner = TestRunner(
            RunOptions(testsuite_root=self.root, build=False)
        )
        with mock.patch.object(
            runner_module, "create_backend", return_value=self.FakeBackend()
        ), mock.patch.object(runner_module, "run_harness", side_effect=fake_run):
            runner.run_one(planned)
        self.assertIsNone(seen["path"])


if __name__ == "__main__":
    unittest.main()
