# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import json
import os
from pathlib import Path
import unittest
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONF_PATH = REPOSITORY_ROOT / "docs/source/zh_CN/conf.py"
CHAT_SCRIPT_PATH = REPOSITORY_ROOT / "docs/source/zh_CN/static/js/algolia-agent-chat.js"


class RecordingSphinxApp:
    def __init__(self):
        self.js_files = []

    def add_js_file(self, filename, **kwargs):
        self.js_files.append((filename, kwargs))


def load_docsearch_config(chip_tag, environment):
    namespace = {
        "__file__": str(CONF_PATH),
        "__name__": "docsearch_test_conf",
        "tags": {chip_tag},
    }
    with mock.patch.dict(os.environ, environment, clear=True):
        exec(compile(CONF_PATH.read_bytes(), str(CONF_PATH), "exec"), namespace)

        app = RecordingSphinxApp()
        namespace["setup"](app)

    config_body = next(
        kwargs["body"]
        for filename, kwargs in app.js_files
        if filename is None
    )
    return json.loads(
        config_body.removeprefix("window.algoliaAgentChatConfig = ").removesuffix(";")
    )


class DocSearchAgentRoutingTests(unittest.TestCase):
    def test_57x_page_uses_57x_agent_and_all_same_version_chip_indices(self):
        config = load_docsearch_config(
            "SF32LB57X",
            {
                "SIFLI_DOC_VERSION": "latest",
                "ALGOLIA_DOCSEARCH_AGENT_IDS": json.dumps(
                    {
                        "sf32lb52x": "agent-52",
                        "sf32lb55x": "agent-55",
                        "sf32lb56x": "agent-56",
                        "sf32lb57x": "agent-57",
                        "sf32lb58x": "agent-58",
                    }
                ),
            },
        )

        self.assertEqual(config["agentId"], "agent-57")
        self.assertEqual(
            config["indices"],
            [
                "sdk_latest_sf32lb52x",
                "sdk_latest_sf32lb55x",
                "sdk_latest_sf32lb56x",
                "sdk_latest_sf32lb57x",
                "sdk_latest_sf32lb58x",
            ],
        )
        self.assertEqual(
            config["searchParameters"],
            {
                "sdk_latest_sf32lb52x": {},
                "sdk_latest_sf32lb55x": {},
                "sdk_latest_sf32lb56x": {},
                "sdk_latest_sf32lb57x": {},
                "sdk_latest_sf32lb58x": {},
            },
        )

    def test_chat_initializes_with_multiple_same_version_indices(self):
        chat_script = CHAT_SCRIPT_PATH.read_text()

        self.assertNotIn("indices.length !== 1", chat_script)
        self.assertIn("indices.length === 0", chat_script)

    def test_empty_agent_mapping_disables_ai_without_breaking_the_build(self):
        config = load_docsearch_config(
            "SF32LB57X",
            {"ALGOLIA_DOCSEARCH_AGENT_IDS": ""},
        )

        self.assertEqual(config["agentId"], "")


if __name__ == "__main__":
    unittest.main()
