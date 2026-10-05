# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Generate a test project's Unity case registry.

A test project used to hand-write ``unity_suites.c``: a prototype for every case
plus the ``sifli_unity_cases[]`` table. Each case therefore named three things in
two files -- the definition, the prototype and the table row -- and the suite
name was written a fourth time in the project's ``testcase.yaml``.

This module removes the middle two. It reads the test sources the build is
*actually compiling* and writes both the registry and a manifest the host side
can check a ``testcase.yaml`` against.

It is called from a test project's ``src/SConscript`` through ``env.Command``,
so the input is the source list *after* ``SrcRemove``. A suite whose module the
carrier cannot build is simply not in that list, and disappears from the registry
with it. That is the point of doing it here rather than in the generated file:
the platform gating stays in the SConscript, where the decision is already made,
instead of being repeated as an ``#if``.

Naming, so that the generated names match what the projects used to write by
hand (see the ``case:``/``suite:`` hints below for the exceptions):

A hint is written on a line of its own inside a comment -- standalone
(``/* case: name */``) or as a line of a doc block (`` * case: name``).
The patterns are anchored to the whole line so that a sentence which happens to
contain "case:" is not mistaken for one.

Each case also carries the source it came from, under the same name the
manifest lists it by, so the device can print a real file on Unity's result
line instead of the suite standing in for one.

* **suite** -- a ``suite: name`` hint anywhere in the file if present, otherwise
  the file name with ``test_`` and ``.c`` stripped. Prefer naming a file
  ``test_<suite>.c`` so that no hint is needed.
* **case** -- a ``case: name`` hint in the comment block immediately above the
  definition if present, otherwise the function name with ``test_<suite>_``
  stripped, otherwise with ``test_`` stripped.

The generated file lands in a subdirectory of the variant directory, never
beside the objects: ``src = Glob('*.c')`` in the SConscript sees the variant
directory too, so a file written to ``build_<board>/src/`` would be picked up by
the next run's ``Glob`` as well as by the ``env.Command`` that produced it, and
the object would be linked twice.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

#: ``suite: name`` / ``case: name`` on a line of its own inside a comment. The
#: optional leading marker lets the same line serve as a standalone block
#: comment or as one line of a doc block; anchoring to both ends keeps prose out.
SUITE_HINT = re.compile(
    r'^\s*(?:/\*|//|\*)?\s*suite:\s*([A-Za-z0-9_.\-]+)\s*(?:\*/)?\s*$', re.M)
CASE_HINT = re.compile(
    r'^\s*(?:/\*|//|\*)?\s*case:\s*([A-Za-z0-9_.\-]+)\s*(?:\*/)?\s*$', re.M)

#: A case is a function taking no arguments and returning nothing. The pattern is
#: anchored and tolerant of the spacing styles in the tree.
DEFINITION = re.compile(r'^\s*void\s+(test_\w+)\s*\(\s*void\s*\)')

_CASE_PREFIX = 'test_'
_GENERATED_BY = 'tools/build/test_registry.py'


def suite_of(path: str, text: str) -> str:
    """The suite a test source belongs to."""
    hint = SUITE_HINT.search(text)
    if hint:
        return hint.group(1)

    name = os.path.basename(path)
    if name.endswith('.c'):
        name = name[:-2]
    if name.startswith(_CASE_PREFIX) and len(name) > len(_CASE_PREFIX):
        name = name[len(_CASE_PREFIX):]
    return name


def parse_cases(text: str, suite: str) -> List[Tuple[str, str]]:
    """Every case in ``text`` as ``(short_name, function_name)``, in file order."""
    lines = text.splitlines()
    cases: List[Tuple[str, str]] = []
    seen = set()

    for index, line in enumerate(lines):
        match = DEFINITION.match(line)
        if not match:
            continue
        function = match.group(1)
        if function in seen:
            # A prototype would have matched too; only the definition is of
            # interest and a file may legitimately declare one first.
            continue
        seen.add(function)
        cases.append((_hint_above(lines, index) or _derive_short(function, suite),
                      function))

    return cases


def _hint_above(lines: List[str], index: int) -> Optional[str]:
    """The nearest ``/* case: x */`` in the comment block above ``lines[index]``.

    Walking up stops at the first line that is not blank and not a comment, so a
    hint only ever applies to the function it sits above.
    """
    for back in range(index - 1, -1, -1):
        stripped = lines[back].strip()
        if not stripped:
            continue
        if not _is_comment(stripped):
            return None
        hint = CASE_HINT.search(lines[back])
        if hint:
            return hint.group(1)
    return None


def _is_comment(stripped: str) -> bool:
    return (stripped.startswith('/*') or stripped.startswith('//')
            or stripped.startswith('*') or stripped.endswith('*/'))


def _derive_short(function: str, suite: str) -> str:
    for prefix in (_CASE_PREFIX + suite + '_', _CASE_PREFIX):
        if function.startswith(prefix) and len(function) > len(prefix):
            return function[len(prefix):]
    return function


def collect(paths: List[str]) -> "Dict[str, List[Tuple[str, str, str]]]":
    """Map every suite found in ``paths`` to its cases.

    A case is ``(short_name, function_name, source_file)``. ``source_file`` is
    the basename, the same spelling the manifest lists its sources under, so
    the two never disagree about what a file is called.

    Suites come out in sorted order so the generated file does not depend on the
    order ``Glob`` happened to return. Case order within a suite follows the
    source file, which is the order the cases will run in.
    """
    suites: "Dict[str, List[Tuple[str, str, str]]]" = {}

    for path in paths:
        with open(path, 'r', encoding='utf-8') as handle:
            text = handle.read()

        suite = suite_of(path, text)
        cases = parse_cases(text, suite)
        if not cases:
            continue

        source = os.path.basename(path)
        bucket = suites.setdefault(suite, [])
        for short, function in cases:
            if any(short == existing for existing, _, _ in bucket):
                raise ValueError(
                    f'{path}: case "{short}" of suite "{suite}" is already '
                    f'defined; two cases of one suite must not share a name, '
                    f'because the name is what the host filters on'
                )
            bucket.append((short, function, source))

    return {suite: suites[suite] for suite in sorted(suites)}


def generate_c(suites: "Dict[str, List[Tuple[str, str, str]]]") -> str:
    """The registry translation unit."""
    flat = [(suite, short, function, source)
            for suite, cases in suites.items()
            for short, function, source in cases]

    out = [
        '/*',
        f' * Generated by {_GENERATED_BY} -- do not edit.',
        ' *',
        ' * Source: the test sources this project is actually compiling. A suite',
        ' * the carrier cannot build is absent from that list, and therefore',
        ' * absent here, without an #if to keep in step.',
        ' */',
        '',
        '#include <rtthread.h>',
        '',
        '#include "sifli_unity.h"',
        '',
    ]

    if not flat:
        out += [
            '/* No case was found. The project still links, and unity_list says so. */',
            '',
        ]
    else:
        for _, _, function, _ in flat:
            out.append(f'void {function}(void);')
        out.append('')

    suite_width = max((len(suite) for suite, _, _, _ in flat), default=0)
    case_width = max((len(short) for _, short, _, _ in flat), default=0)
    function_width = max((len(function) for _, _, function, _ in flat), default=0)

    out += [
        'const sifli_unity_case_t sifli_unity_cases[] =',
        '{',
    ]
    for suite, short, function, source in flat:
        out.append('    { %-*s %-*s %-*s %s },'
                   % (suite_width + 3, '"%s",' % suite,
                      case_width + 3, '"%s",' % short,
                      function_width + 1, function + ',',
                      '"%s"' % source))
    if flat:
        out.append('')
    out += [
        '    { RT_NULL, RT_NULL, RT_NULL, RT_NULL }',
        '};',
        '',
    ]

    return '\n'.join(out)


def generate_manifest(suites: "Dict[str, List[Tuple[str, str, str]]]",
                      paths: List[str],
                      meta: Optional[dict] = None) -> dict:
    """A description of what the firmware registered, for the host side.

    ``sdk_test`` reads this before it opens a serial port: a ``suite:`` in a
    ``testcase.yaml`` that is not in here is a suite name that drifted, and the
    difference between "the suite failed" and "the suite is not in this build"
    is worth knowing before the run rather than after it.
    """
    manifest = {
        'generated_by': _GENERATED_BY,
        'sources': [os.path.basename(path) for path in paths],
        'suites': {suite: [short for short, _, _ in cases]
                   for suite, cases in suites.items()},
    }
    manifest.update(meta or {})
    return manifest


def build(paths: List[str], meta: Optional[dict] = None) -> Tuple[str, dict]:
    """Both artefacts, from one pass over the sources."""
    suites = collect(paths)
    return generate_c(suites), generate_manifest(suites, paths, meta)


def manifest_path(c_source: str) -> str:
    """Where the manifest for ``c_source`` lives."""
    return os.path.splitext(c_source)[0] + '.json'


def action(**meta):
    """An SCons action writing both artefacts, for use with ``env.Command``.

    The board name is passed as meta by the SConscript, which is the only place
    that knows it.
    """
    def _run(target, source, env):
        paths = [str(node) for node in source]
        registry, manifest = build(paths, meta)
        c_source = str(target[0])

        with open(c_source, 'w', encoding='utf-8') as handle:
            handle.write(registry)
        with open(manifest_path(c_source), 'w', encoding='utf-8') as handle:
            json.dump(manifest, handle, indent=2)
            handle.write('\n')

        print('TEST-REGISTRY: %s: %d suite(s), %d case(s)'
              % (os.path.basename(c_source),
                 len(manifest['suites']),
                 sum(len(cases) for cases in manifest['suites'].values())))
        return None

    return _run


def main(argv: Optional[List[str]] = None) -> int:
    """Run the generator by hand, for tests and for inspecting a project."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('sources', nargs='+', help='test sources to read')
    parser.add_argument('--out', default=None,
                        help='where to write the .c (default: print it)')
    parser.add_argument('--board', default=None, help='recorded in the manifest')
    options = parser.parse_args(argv)

    registry, manifest = build(options.sources, {'board': options.board})

    if options.out:
        with open(options.out, 'w', encoding='utf-8') as handle:
            handle.write(registry)
        with open(manifest_path(options.out), 'w', encoding='utf-8') as handle:
            json.dump(manifest, handle, indent=2)
            handle.write('\n')
    else:
        sys.stdout.write(registry)

    return 0


if __name__ == '__main__':
    sys.exit(main())
