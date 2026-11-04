#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
import argparse
import shlex
import sys

from dynamic_pipeline import DynamicPipelineGenerator


def resolve_build_job(generator, batch_index, node_index):
    if batch_index < 0 or node_index < 1:
        raise ValueError('batch index must be non-negative and node index must be positive')

    jobs = generator.get_build_jobs()
    job_index = batch_index * generator.batch_size + node_index - 1
    if job_index >= len(jobs):
        return None

    job_name, project_path, board, is_common = jobs[job_index]
    scons_board = board.replace('_hcpu', '') if board and '_hcpu' in board else board
    return {
        'BUILD_BATCH_SKIP': '0',
        'PROJECT_PATH': project_path,
        'BOARD': board or '',
        'SCONS_BOARD': scons_board or '',
        'BUILD_ARGS': f'--board={scons_board}' if is_common and scons_board else '',
        'LOG_NAME': job_name[len('build_'):],
        'PROJECT_TYPE': 'common' if is_common else 'normal',
    }


def resolve_build_job_by_label(generator, label):
    for job_index, (job_name, *_rest) in enumerate(generator.get_build_jobs()):
        if job_name[len('build_'):] == label:
            return resolve_build_job(
                generator,
                job_index // generator.batch_size,
                job_index % generator.batch_size + 1,
            )
    return None


def main():
    parser = argparse.ArgumentParser(description='Resolve one named SDK build job')
    parser.add_argument('--config', default='build_config_gcc.yaml')
    parser.add_argument('--job-label', required=True)
    args = parser.parse_args()

    try:
        generator = DynamicPipelineGenerator(args.config)
        values = resolve_build_job_by_label(generator, args.job_label)
        if values is None:
            print(f'Unknown build job label: {args.job_label}', file=sys.stderr)
            return 1
        for key, value in values.items():
            print(f'export {key}={shlex.quote(value)}')
    except Exception as exc:
        print(f'Failed to resolve build job: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
