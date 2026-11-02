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


def main():
    parser = argparse.ArgumentParser(description='Resolve one indexed SDK build job')
    parser.add_argument('--config', default='build_config_gcc.yaml')
    parser.add_argument('--batch-index', type=int, required=True)
    parser.add_argument('--node-index', type=int, required=True)
    args = parser.parse_args()

    if args.batch_index < 0 or args.node_index < 1:
        parser.error('batch index must be non-negative and node index must be positive')

    try:
        generator = DynamicPipelineGenerator(args.config)
        values = resolve_build_job(generator, args.batch_index, args.node_index)
        if values is None:
            print('export BUILD_BATCH_SKIP=1')
        else:
            for key, value in values.items():
                print(f'export {key}={shlex.quote(value)}')
    except Exception as exc:
        print(f'Failed to resolve build job: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
