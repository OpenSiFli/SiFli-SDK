# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Which revision of this framework produced a result, and what it can say to whom.

Two copies of this repository take part in every hardware run: the checkout that
was typed into, and the one the Lab Agent was started from. The agent is a
long-running process, so the second can be days older than the first -- and a
green run says nothing about which code produced it.

The two sides therefore report their revision, and they are recorded rather than
enforced. A dirty checkout is the normal state of one being worked on, so
"revisions must match" would refuse to run exactly when somebody is mid-change,
which is when the framework is most useful. It would also be trivially bypassed:
a commit on one side and an unrelated one on the other is not the failure worth
catching.

What *is* enforced is :data:`PROTOCOL`, which moves only when the two sides stop
being able to talk to each other at all -- a field one side sends and the other
cannot read. That is a question a version number cannot answer and an integer
can, and it is the one mismatch that produces nonsense instead of an error.
"""

from __future__ import annotations

import os
import subprocess
from typing import Dict
from typing import Optional

#: The version of the SDK-side/agent interface.
#:
#: Bump this when a change makes one side unable to read the other: a new
#: required field, a removed route, a changed meaning. The two sides compare it
#: on every hardware run and refuse to proceed when they disagree, because the
#: alternative is a mismatch that shows up as a wrong result rather than an
#: error.
#:
#: 2 -- instrument consoles. ``?stream=<fixture>`` on the console routes means
#: "this other port", and an agent from before it ignores the parameter and
#: serves the DUT's console instead: a suite would then drive the DUT as if it
#: were the peer and record the wrong transcript. That is the silent-wrong-result
#: case this number exists for.
PROTOCOL = 2

#: What a checkout with no git, or no repository, reports.
UNKNOWN = "unknown"

#: Memoised by root: one run asks once per instance, and the answer cannot change
#: under a running process in any way worth re-reading for.
_CACHE: Dict[str, str] = {}


def package_root() -> str:
    """The checkout this module's own code lives in.

    Derived from this file rather than from an environment variable, because the
    question a revision answers is "which code is running". A lab machine with
    ``SIFLI_SDK_PATH`` pointing at one checkout and the agent started from
    another would otherwise report the revision of code that is not executing.
    """
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def revision(root: str) -> str:
    """``<short sha>`` for the checkout at ``root``, and ``-dirty`` if edited.

    ``-dirty`` means tracked files differ from ``HEAD``. Untracked files are
    deliberately not counted: a checkout of this repository collects them
    (build trees, downloads, reports), and a flag that is always on records
    nothing. What it is for is the case somebody has to explain later -- an
    agent started from a modified tree, still running days afterwards.

    ``unknown`` when git is missing or ``root`` is not a checkout. That is worth
    saying out loud rather than guessing: a lab machine may well be running a
    deployed copy with no repository behind it.
    """
    key = os.path.abspath(root)
    if key not in _CACHE:
        _CACHE[key] = _read(key)
    return _CACHE[key]


def _read(root: str) -> str:
    head = _git(root, "rev-parse", "--short", "HEAD")
    if head is None or head.returncode != 0:
        return UNKNOWN
    sha = head.stdout.strip()
    if not sha:
        return UNKNOWN
    edits = _git(root, "diff-index", "--quiet", "HEAD")
    dirty = edits is not None and edits.returncode != 0
    return f"{sha}-dirty" if dirty else sha


def _git(root: str, *args: str) -> Optional["subprocess.CompletedProcess[str]"]:
    try:
        return subprocess.run(
            ["git", "-C", root, *args],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        # git not installed, not a repository, or hung. All three mean the same
        # thing to a caller: this revision cannot be named.
        return None
