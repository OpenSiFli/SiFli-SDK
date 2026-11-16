#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Write tiny, clearly marked fixtures through the real artifact collection path."""

import argparse
from pathlib import Path

from dynamic_pipeline import DynamicPipelineGenerator
from resolve_build_job import resolve_build_job_by_label
from write_build_junit import write_report


def write_fixture(label, root=Path('.')):
    generator = DynamicPipelineGenerator(smoke_test=True)
    target = resolve_build_job_by_label(generator, label)
    if target is None:
        raise ValueError('unknown smoke target')
    failed = 'expected-failure' in target['PROJECT_PATH']
    status = 'failed' if failed else 'success'
    log = root / 'ci_build_logs' / f'{label}.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(f'CATALOG SMOKE TEST ONLY: {status}\nNo firmware was compiled.\n', encoding='utf-8')
    metadata = root / 'job_status' / f'{label}.env'
    metadata.parent.mkdir(parents=True, exist_ok=True)
    metadata.write_text(f'JOB_STATUS={status}\n', encoding='utf-8')
    write_report(status, metadata.with_suffix('.junit.xml'), label,
                 target['PROJECT_PATH'], target['BOARD'])
    if not failed:
        artifact = root / 'artifacts' / label / 'SMOKE_TEST_ONLY.txt'
        artifact.parent.mkdir(parents=True, exist_ok=True)
        artifact.write_text(f'TEST DATA ONLY. Not firmware.\nTarget: {label}\n', encoding='utf-8')
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job-label', required=True)
    args = parser.parse_args()
    return write_fixture(args.job_label)


if __name__ == '__main__':
    raise SystemExit(main())
