# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import logging
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict

from .runner import CommandRunner


@dataclass
class SdkConfig:
    strict_extensions: bool = False
    verbose: bool = False
    no_hints: bool = False
    group_options: Dict[str, Dict[str, Any]] = field(default_factory=dict)


@dataclass
class SdkContext:
    project_dir: str
    build_dir: str
    env: Dict[str, str]
    logger: logging.Logger
    runner: CommandRunner
    config: SdkConfig
