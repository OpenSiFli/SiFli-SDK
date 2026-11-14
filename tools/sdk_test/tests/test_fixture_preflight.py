# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the fixture commands and the pre-build fixture check.

Three commands answer "may I write this, and will the lab serve it":

* ``fixtures types`` reads the catalogue, offline by construction;
* ``fixtures list`` prints what one lab offers, from its binding or its agent;
* ``plan --check-fixtures`` compares a plan against the live inventory before
  anything is built.

The check must use the allocation's own matching rule. A capability subset is
enough for an ordinary type, but a parameterized type's spec is an identity:
``wifi_ap:5g`` is *not* served by a lab connection offering ``wifi_ap:5g,lab_a``,
because its values are looked up under the requested spec.
"""

from __future__ import annotations

import os
import unittest
from unittest import mock

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import cli
from sdk_test.errors import LabAgentError
from sdk_test.errors import MatchError
from sdk_test.errors import SdkTestError


class FakeAgent:
    """A Lab Agent's ``GET /testbeds``, without HTTP or a bench."""

    def __init__(self, testbeds=(), error=None):
        self._testbeds = list(testbeds)
        self._error = error
        self.calls = 0

    def testbeds(self):
        self.calls += 1
        if self._error is not None:
            raise self._error
        return self._testbeds


def agent_for(agent: FakeAgent):
    """A ``client_factory`` that hands out one fake agent."""
    return lambda _url: agent


def testbed(testbed_id="tb1", platform="board_a", fixtures=(), busy=False):
    return {
        "id": testbed_id,
        "platform": platform,
        "fixtures": list(fixtures),
        "console": {"port": "COM1", "baud": 1000000},
        "busy": busy,
    }


class TestFixtureTypesCommand(SdkTestCase):
    def test_the_catalogue_is_printed_with_descriptions_and_capabilities(self):
        lines = []
        self.assertEqual(cli.list_fixture_types(echo=lines.append), cli.EXIT_OK)
        text = "\n".join(lines)

        self.assertIn("fixture_types", lines[0])
        self.assertIn("bt_peer", text)
        self.assertIn("ble, classic", text)
        self.assertIn("A second board running the peer firmware", text)
        self.assertIn("spi_nor_flash", text)
        self.assertIn("spi_nand_flash", text)
        self.assertIn("i2c_sensor", text)
        self.assertIn("wifi_ap", text)

    def test_each_type_says_whether_its_spec_is_an_identity(self):
        lines = []
        cli.list_fixture_types(echo=lines.append)
        blocks = _blocks(lines)

        self.assertEqual(blocks["bt_peer"]["parameterized"], ["parameterized: no"])
        self.assertEqual(
            blocks["spi_nor_flash"]["parameterized"], ["parameterized: no"]
        )
        self.assertEqual(blocks["wifi_ap"]["parameterized"], ["parameterized: yes"])

    def test_options_are_printed_with_source_and_flags(self):
        lines = []
        cli.list_fixture_types(echo=lines.append)
        text = "\n".join(lines)
        options = _option_lines(lines, "wifi_ap")
        ssid = next(line for line in options if line.strip().startswith("ssid"))
        psk = next(line for line in options if line.strip().startswith("psk"))

        # Per option, so "required" on psk's required_if cannot satisfy ssid.
        self.assertIn("source=params", ssid)
        self.assertIn("required", ssid)
        self.assertNotIn("required_if", ssid)
        self.assertIn('required_if="security != open"', psk)
        self.assertIn("secret", psk)
        self.assertIn("enum=open|wpa2|wpa3", text)
        self.assertIn("default=wpa2", text)

    def test_a_type_without_options_says_so(self):
        lines = []
        cli.list_fixture_types(echo=lines.append)
        self.assertEqual(_blocks(lines)["i2c_sensor"]["options"], ["options:       none"])


def _option_lines(lines, type_name):
    """The indented lines under one type block of ``fixtures types``."""
    start = lines.index(type_name) + 1
    block = []
    for line in lines[start:]:
        if not line.startswith(" "):
            break
        block.append(line)
    return block


def _blocks(lines):
    """Split the command's output into ``{type name: {key: [lines]}}``."""
    blocks = {}
    current = None
    for line in lines[1:]:
        if not line.startswith(" "):
            current = blocks.setdefault(line, {})
            continue
        if current is None:
            continue
        name, _, value = line.strip().partition(":")
        current.setdefault(name, []).append(line.strip())
    return blocks


class TestFixtureListFromFile(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.path = self.write(
            "binding.yaml",
            """
            testbed:
              - id: tb-1
                platform: board_a
                console: {port: COM1, baud: 1000000}
                connections:
                  - fixture: bt_peer:ble
                  - fixture: bt_peer:ble,classic
              - id: tb-2
                platform: board_b
                connections:
                  - fixture: wifi_ap:5g
            """,
        )

    def run_list(self, **kwargs):
        lines = []
        code = cli.list_fixtures(binding_file=self.path, echo=lines.append, **kwargs)
        return code, "\n".join(lines)

    def test_every_testbed_and_its_offers_are_printed(self):
        code, text = self.run_list()

        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("2 testbed(s)", text)
        self.assertIn("tb-1", text)
        self.assertIn("platform=board_a", text)
        self.assertIn("console=COM1@1000000", text)
        self.assertIn("bt_peer:ble,classic", text)
        self.assertIn("tb-2", text)
        self.assertIn("wifi_ap:5g", text)

    def test_the_platform_filter_keeps_only_that_platform(self):
        _code, text = self.run_list(platform="board_b")
        self.assertIn("tb-2", text)
        self.assertNotIn("tb-1", text)
        self.assertIn("1 testbed(s)", text)

    def test_a_platform_with_no_testbed_says_so_without_failing(self):
        code, text = self.run_list(platform="board_c")
        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("No testbed on board_c", text)


class TestFixtureListFromAgent(SdkTestCase):
    def test_the_agent_is_asked_and_its_answer_printed(self):
        agent = FakeAgent(
            [
                testbed("tb-52", "board_a", ["bt_peer:ble,classic"]),
                testbed("tb-58", "board_b", ["wifi_ap:5g"], busy=True),
            ]
        )
        lines = []
        code = cli.list_fixtures(
            agent_url="http://lab:8760",
            echo=lines.append,
            client_factory=agent_for(agent),
        )
        text = "\n".join(lines)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(1, agent.calls)
        self.assertIn("http://lab:8760", text)
        self.assertIn("tb-52", text)
        self.assertIn("bt_peer:ble,classic", text)
        self.assertIn("tb-58", text)
        self.assertIn("[busy]", text)

    def test_an_unreachable_agent_is_an_error_not_an_empty_lab(self):
        agent = FakeAgent(error=LabAgentError("Cannot reach the Lab Agent at x"))
        with self.assertRaises(LabAgentError) as caught:
            cli.list_fixtures(
                agent_url="http://lab:8760",
                echo=lambda _m: None,
                client_factory=agent_for(agent),
            )
        self.assertIn("Cannot reach", str(caught.exception))

    def test_a_file_and_an_agent_url_are_mutually_exclusive(self):
        with self.assertRaises(SdkTestError) as caught:
            cli.list_fixtures(
                binding_file="somewhere.yaml",
                agent_url="http://lab:8760",
                echo=lambda _m: None,
            )
        # Not "the file does not exist": the conflict is what must be reported.
        self.assertIn("mutually exclusive", str(caught.exception))


class PlanFixtureCase(SdkTestCase):
    """Common scaffolding: one case, one platform, one project."""

    def setUp(self):
        super().setUp()
        # The developer's own params file and shell must not decide these
        # tests. An empty SIFLI_TEST_PARAMS_FILE is "not set" to the resolver,
        # and patch.dict puts the real value back afterwards.
        patched = mock.patch.dict(
            os.environ,
            {"USERPROFILE": self.root, "SIFLI_TEST_PARAMS_FILE": ""},
            clear=False,
        )
        patched.start()
        self.addCleanup(patched.stop)
        self.params_file = None

    def make_case(self, fixture, skip: bool = False, build_only: bool = False) -> str:
        if isinstance(fixture, str):
            spec = fixture
        else:
            spec = "[" + ", ".join(f'"{item}"' for item in fixture) + "]"
        flags = ""
        if skip:
            flags += "\n                skip: true"
        if build_only:
            flags += "\n                build_only: true"
        self.write(
            "testcase.yaml",
            f"""
            tests:
              - id: probe
                harness: console
                integration_platforms: [board_a]
                harness_config:
                  fixture: {spec}
                  record: ["x"]{flags}
            """,
        )
        self.write("project/SConstruct", "# project\n")
        self.write("project/proj.conf", "CONFIG_X=y\n")
        return self.root

    def plan(self, agent, check=True, platform="board_a"):
        lines = []
        code = cli.describe_plan(
            self.root,
            platform=platform,
            params_file=self.params_file,
            echo=lines.append,
            check_fixtures=check,
            client_factory=agent_for(agent),
        )
        return code, "\n".join(lines)

    def use_wifi_ap(self):
        self.params_file = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: a-real-psk
            """,
        )
        return self.make_case("wifi_ap:5g")


class TestPlanCheckFixtures(PlanFixtureCase):
    def test_a_fixture_the_platform_offers_passes(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent([testbed("tb1", "board_a", ["bt_peer:ble"])])

        code, text = self.plan(agent)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("would run", text)
        self.assertIn("probe", text)

    def test_a_capability_superset_still_satisfies_an_ordinary_type(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent([testbed("tb1", "board_a", ["bt_peer:ble,classic"])])
        code, _text = self.plan(agent)
        self.assertEqual(code, cli.EXIT_OK)

    def test_a_busy_testbed_still_counts_as_offering_the_fixture(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent([testbed("tb1", "board_a", ["bt_peer:ble"], busy=True)])
        code, _text = self.plan(agent)
        self.assertEqual(code, cli.EXIT_OK)

    def test_a_missing_fixture_fails_and_names_both_sides(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent([testbed("tb1", "board_a", ["bt_peer:classic"])])

        lines = []
        with self.assertRaises(MatchError):
            cli.describe_plan(
                self.root,
                platform="board_a",
                echo=lines.append,
                check_fixtures=True,
                client_factory=agent_for(agent),
            )

        text = "\n".join(lines)
        self.assertIn(
            "case probe: needs bt_peer:ble, no testbed on board_a offers it", text
        )
        self.assertIn("platform board_a offers:", text)
        self.assertIn("tb1: bt_peer:classic", text)
        # Nothing claims a run is about to start: the check is pre-build.
        self.assertNotIn("would run", text)

    def test_a_parameterized_spec_must_match_exactly(self):
        # The values are looked up under the requested spec, so a lab connection
        # with a longer spec cannot serve it -- the same rule allocation uses.
        self.use_wifi_ap()
        agent = FakeAgent([testbed("tb1", "board_a", ["wifi_ap:5g,lab_a"])])

        lines = []
        with self.assertRaises(MatchError):
            cli.describe_plan(
                self.root,
                platform="board_a",
                params_file=self.params_file,
                echo=lines.append,
                check_fixtures=True,
                client_factory=agent_for(agent),
            )
        self.assertIn("needs wifi_ap:5g", "\n".join(lines))

    def test_an_exactly_matching_parameterized_spec_passes(self):
        self.use_wifi_ap()
        agent = FakeAgent([testbed("tb1", "board_a", ["wifi_ap:5g"])])
        code, text = self.plan(agent, platform="board_a")
        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("would run", text)

    def test_a_fixture_on_another_platform_does_not_help(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent([testbed("tb1", "board_b", ["bt_peer:ble"])])

        lines = []
        with self.assertRaises(MatchError):
            cli.describe_plan(
                self.root,
                platform="board_a",
                echo=lines.append,
                check_fixtures=True,
                client_factory=agent_for(agent),
            )
        text = "\n".join(lines)
        self.assertIn("no testbed on board_a offers it", text)
        self.assertIn("(no testbed on this platform)", text)

    def test_fixtures_split_across_two_testbeds_fail(self):
        # A test's fixture list means "all of these", and allocation picks one
        # testbed. Two testbeds that each have half is a wiring problem the
        # plan must refuse, not a pass that costs a build.
        self.make_case(["bt_peer:ble", "spi_nor_flash"])
        agent = FakeAgent(
            [
                testbed("tb-peer", "board_a", ["bt_peer:ble"]),
                testbed("tb-flash", "board_a", ["spi_nor_flash"]),
            ]
        )

        lines = []
        with self.assertRaises(MatchError):
            cli.describe_plan(
                self.root,
                platform="board_a",
                echo=lines.append,
                check_fixtures=True,
                client_factory=agent_for(agent),
            )

        text = "\n".join(lines)
        self.assertIn(
            "case probe: no single testbed on board_a offers all of: "
            "bt_peer:ble, spi_nor_flash",
            text,
        )
        # Both sides again: the reader can see the two halves are present.
        self.assertIn("tb-peer: bt_peer:ble", text)
        self.assertIn("tb-flash: spi_nor_flash", text)

    def test_one_testbed_offering_everything_passes(self):
        self.make_case(["bt_peer:ble", "spi_nor_flash"])
        agent = FakeAgent(
            [testbed("tb-both", "board_a", ["bt_peer:ble", "spi_nor_flash"])]
        )
        code, text = self.plan(agent)
        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("would run", text)

    def test_a_skipped_case_is_not_checked(self):
        # `skip` says it will not run, so it will not ask for hardware either.
        # The same reason keeps it exempt from requiring a secret at plan time.
        self.make_case("bt_peer:ble", skip=True)
        agent = FakeAgent(error=AssertionError("the agent must not be asked"))

        code, _text = self.plan(agent)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(0, agent.calls)

    def test_a_build_only_case_is_not_checked(self):
        self.make_case("bt_peer:ble", build_only=True)
        agent = FakeAgent(error=AssertionError("the agent must not be asked"))

        code, _text = self.plan(agent)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(0, agent.calls)

    def test_an_unreachable_agent_is_a_connection_error_not_a_missing_fixture(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent(error=LabAgentError("Cannot reach the Lab Agent at x"))

        with self.assertRaises(LabAgentError) as caught:
            self.plan(agent)
        self.assertIn("Cannot reach", str(caught.exception))

    def test_a_plan_that_needs_no_fixture_does_not_ask_the_agent(self):
        self.make_case_plain()
        agent = FakeAgent(error=AssertionError("the agent must not be asked"))

        code, text = self.plan(agent)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(0, agent.calls)
        self.assertIn("would run", text)

    def test_the_flag_is_off_by_default_and_the_agent_is_never_contacted(self):
        self.make_case("bt_peer:ble")
        agent = FakeAgent(error=AssertionError("the agent must not be asked"))

        code, _text = self.plan(agent, check=False)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(0, agent.calls)

    def make_case_plain(self):
        self.write(
            "testcase.yaml",
            """
            tests:
              - id: probe
                harness: console
                integration_platforms: [board_a]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("project/SConstruct", "# project\n")
        self.write("project/proj.conf", "CONFIG_X=y\n")


class TestCallbackWiring(SdkTestCase):
    """The flag and the two sources must reach the library, not just exist."""

    def setUp(self):
        super().setUp()
        try:
            from sdk_py_actions import test_ext
        except ImportError as exc:  # pragma: no cover - environment problem
            self.skipTest(f"sdk_py_actions is not importable: {exc}")
        self.test_ext = test_ext

    def test_plan_passes_the_pre_check_flag_through(self):
        captured = {}

        def fake_describe_plan(*_args, **kwargs):
            captured.update(kwargs)
            return cli.EXIT_OK

        with mock.patch.object(
            self.test_ext.sdk_test_cli, "describe_plan", fake_describe_plan
        ):
            self.test_ext.test_plan_callback(
                None, testsuite_root=self.root, check_fixtures=True
            )

        self.assertTrue(captured["check_fixtures"])

    def test_a_failed_pre_check_exits_non_zero(self):
        # The library raises; only `_guard` decides what the process status is,
        # and a failed pre-check must never be mistaken for a clean plan.
        from sdk_py_actions.errors import FatalError

        with mock.patch.object(
            self.test_ext.sdk_test_cli,
            "check_fixture_offers",
            side_effect=MatchError("fixture pre-check: no host"),
        ):
            with self.assertRaises(FatalError) as caught:
                self.test_ext.test_plan_callback(
                    None,
                    testsuite_root=self.root,
                    platform="board_a",
                    check_fixtures=True,
                )
        self.assertEqual(caught.exception.exit_code, 10)

    def test_the_list_callback_refuses_two_sources(self):
        import click

        with self.assertRaises(click.BadParameter):
            self.test_ext.test_fixtures_list_callback(
                None, file="binding.yaml", agent_url="http://lab:8760"
            )


class TestFixtureCommandsAreRegistered(SdkTestCase):
    def test_the_commands_exist_and_plan_takes_the_pre_check_flag(self):
        try:
            from sdk_py_actions import test_ext
            from sdk_py_actions.cli.registry import CommandRegistry
        except ImportError as exc:  # pragma: no cover - environment problem
            self.skipTest(f"sdk_py_actions is not importable: {exc}")

        registry = CommandRegistry()
        test_ext.register(registry)
        # Built through the public builder and walked as click objects: the
        # registry keeps its command table private, and the question is what a
        # user can actually type.
        root = registry.build_click(lambda: 0, "sdk.py")
        test_group = root.commands["test"]

        self.assertIn("types", test_group.commands["fixtures"].commands)
        self.assertIn("list", test_group.commands["fixtures"].commands)
        plan_flags = {
            name
            for param in test_group.commands["plan"].params
            for name in getattr(param, "opts", ())
        }
        self.assertIn("--check-fixtures", plan_flags)


if __name__ == "__main__":
    unittest.main()
