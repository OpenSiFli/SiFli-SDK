# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import unittest
from unittest import mock


CONF_PATH = Path(__file__).resolve().parents[1] / "docs/source/zh_CN/conf.py"


class DocSearchConfigTests(unittest.TestCase):
    def test_search_uses_current_version_and_chip(self):
        for version in ("latest", "v2.5.1"):
            for chip in ("sf32lb52x", "sf32lb55x", "sf32lb56x", "sf32lb57x", "sf32lb58x"):
                with self.subTest(version=version, chip=chip):
                    namespace = {"__file__": str(CONF_PATH), "tags": {chip.upper()}}
                    environment = {
                        "SIFLI_DOC_VERSION": version,
                        "ALGOLIA_DOCSEARCH_APP_ID": "test-app",
                        "ALGOLIA_DOCSEARCH_SEARCH_API_KEY": "test-key",
                    }
                    with mock.patch.dict(os.environ, environment, clear=True):
                        exec(compile(CONF_PATH.read_bytes(), str(CONF_PATH), "exec"), namespace)
                    self.assertEqual(namespace["docsearch_index_name"], f"sdk_{version}_{chip}")
                    self.assertEqual(namespace["docsearch_app_id"], "test-app")
                    self.assertEqual(namespace["docsearch_api_key"], "test-key")
                    self.assertIn("sphinx_docsearch", namespace["extensions"])
                    self.assertEqual(namespace["templates_path"], ["_templates"])


if __name__ == "__main__":
    unittest.main()
