#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
import argparse
import math
import sys

import yaml


DEFAULT_BATCH_SIZE = 100
BATCH_PIPELINE = '.gitlab/ci/build_sdk_batch.yml'


class DynamicPipelineGenerator:
    def __init__(self, config_file='build_config_gcc.yaml'):
        self.config_file = config_file
        self.batch_size = DEFAULT_BATCH_SIZE
        self.config = self._load_config()

    def _load_config(self):
        try:
            with open(self.config_file, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except Exception as exc:
            raise ValueError(f'failed to load {self.config_file}: {exc}') from exc

        if not isinstance(config, dict):
            raise ValueError(f'{self.config_file} must contain a YAML mapping')
        return config

    @staticmethod
    def _generate_job_name(project_path, board=None):
        name = project_path.replace('/', '_').replace('-', '_').replace('.', '_')
        if board:
            name += f'_{board.replace("-", "_")}'
        return f'build_{name}'

    def get_build_jobs(self):
        jobs = []
        names = set()

        for project_group in self.config.get('projects', []):
            for project_path in project_group.get('path', []):
                jobs.append((self._generate_job_name(project_path), project_path, None, False))

        for common_project in self.config.get('common_projects', []):
            project_path = common_project['path']
            excluded = set(common_project.get('exclude_boards', []))
            for board in common_project.get('boards', []):
                if board not in excluded:
                    jobs.append((self._generate_job_name(project_path, board), project_path, board, True))

        for name, *_ in jobs:
            if name in names:
                raise ValueError(f'duplicate generated build job name: {name}')
            names.add(name)
        return jobs

    def generate_child_pipeline(self):
        jobs = self.get_build_jobs()
        if not jobs:
            raise ValueError('build configuration contains no build jobs')
        batch_count = math.ceil(len(jobs) / self.batch_size)
        pipeline = {
            'include': [{'local': BATCH_PIPELINE}],
            'stages': ['build', 'collect'],
            'variables': {
                'GIT_SUBMODULE_STRATEGY': 'recursive',
                'COMPILER': 'gcc',
                'PARALLEL_JOBS': '8',
            },
        }

        for batch_index in range(batch_count):
            suffix = f'{batch_index + 1:03d}'
            job_name = f'build_batch_{suffix}'
            pipeline[job_name] = {
                'extends': '.build_sdk_batch_template',
                'stage': 'build',
                'variables': {
                    'BATCH_INDEX': str(batch_index),
                    'BUILD_CONFIG': str(self.config_file),
                },
            }
            pipeline[f'collect_batch_{suffix}'] = {
                'extends': '.collect_sdk_batch_template',
                'stage': 'collect',
                'needs': [{'job': job_name, 'artifacts': True}],
            }

        print(
            f'Generated {len(jobs)} build jobs in {batch_count} parallel groups '
            f'(up to {self.batch_size} build slots per group)'
        )
        return pipeline

    def save_child_pipeline(self, output_file='child-pipeline.yml'):
        pipeline = self.generate_child_pipeline()
        with open(output_file, 'w', encoding='utf-8') as output:
            yaml.safe_dump(pipeline, output, sort_keys=False, allow_unicode=True)
        print(f'Child pipeline configuration saved: {output_file}')
        return output_file


def main():
    parser = argparse.ArgumentParser(description='Generate batched GitLab child pipelines')
    parser.add_argument('-c', '--config', default='build_config_gcc.yaml', help='build config YAML')
    parser.add_argument('-o', '--output', default='child-pipeline.yml', help='output pipeline YAML')
    parser.add_argument('--dry-run', action='store_true', help='print pipeline size and counts')
    args = parser.parse_args()

    try:
        generator = DynamicPipelineGenerator(args.config)
        pipeline = generator.generate_child_pipeline()
        if args.dry_run:
            rendered = yaml.safe_dump(pipeline, sort_keys=False, allow_unicode=True)
            print(f'Generated child pipeline YAML: {len(rendered.encode("utf-8"))} bytes')
        else:
            with open(args.output, 'w', encoding='utf-8') as output:
                yaml.safe_dump(pipeline, output, sort_keys=False, allow_unicode=True)
            print(f'Child pipeline configuration saved: {args.output}')
    except Exception as exc:
        print(f'Pipeline generation failed: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
